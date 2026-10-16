"""Build OdrForge terrain grids (.odrz) from open elevation data.

  python make_terrain.py --origin LAT,LON --bounds MINLAT,MINLON,MAXLAT,MAXLON \
      --dem dem.tif [--dem more.tif] [--lidar points.laz | --lidar points.npz --lidar-crs EPSG:3857] \
      --ground ground.odrz [--structures decks.odrz] [--cell 0.5] [--datum NAVD88]
      [--dsm surface.tif --deck-osm area.osm]   # bridge decks from a surface model where no lidar is at hand

Sources (any country): national lidar (USGS 3DEP, UK EA, NL AHN, FR IGN LiDAR HD, ...) and DEMs
(3DEP 1 m, Copernicus GLO-30 / FABDEM as global fallback).

* Grids are in the same local transverse Mercator frame OdrForge uses (origin LAT,LON), so pass the
  same --origin to osm2xodr.
* ground: lidar ground (ASPRS class 2) and road surface (11) where available, else the DEM.
* structures: lidar bridge deck (class 17) - used by osm2xodr for bridge=yes roads. Without lidar, a digital
  surface model (DSM: first-return heights, e.g. Hesse DOM1, UK EA DSM) gives the top deck: its cells more than
  2.5 m above the bare earth within 10 m of an OSM road bridge (highway + bridge=yes/viaduct, --deck-osm). Trees and
  buildings elsewhere are left out; a deck under another deck is not seen (osm2xodr: the lower road gives way).
* Lidar heights are aligned to the DEM's vertical datum by the median offset of ground points
  (e.g. ellipsoidal lidar vs NAVD88 DEM).
* Several --dem, finest first: a later DEM fills only the cells the earlier ones leave empty (a regional model
  ending at its border, e.g. South Tyrol's 2.5 m DTM at the Lombardy border, completed by GLO-30). The fill is
  shifted onto the earlier DEMs: by the difference at the nearest cell where both have data, fading over
  --seam metres to their median difference (another datum, a surface model's trees), so the border has no step.
Requires numpy, pyproj, rasterio (DEM), laspy[lazrs] for .las/.laz.
"""
import argparse
import struct
import sys

import numpy as np
from pyproj import Transformer


def write_odrz(path, origin_x, origin_y, cell, z, datum):
    """z: (ny, nx) single surface (v1) or (levels, ny, nx) stacked surfaces (v2)."""
    levels = 1 if z.ndim == 2 else z.shape[0]
    ny, nx = z.shape[-2:]
    with open(path, "wb") as f:
        f.write(b"ODRZGRD1" if levels == 1 else b"ODRZGRD2")
        f.write(struct.pack("<dddii", origin_x, origin_y, cell, nx, ny))
        if levels > 1:
            f.write(struct.pack("<i", levels))
        f.write(datum.encode("ascii")[:31].ljust(32, b"\0"))
        f.write(np.ascontiguousarray(z, dtype="<f4").tobytes())


def deck_levels(ix, iy, z, nx, ny, gap=2.0, max_levels=4):
    """Stacked bridge decks: per cell, split the deck points into levels separated by > gap metres
    and average each level. Returns (levels, ny, nx) with the lowest level first, NaN where absent."""
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    cell = (iy[ok] * nx + ix[ok]).astype(np.int64)
    zz = z[ok]
    order = np.lexsort((zz, cell))
    cell, zz = cell[order], zz[order]
    new_cell = np.r_[True, cell[1:] != cell[:-1]]
    new_level = new_cell | np.r_[True, np.diff(zz) > gap]
    # level index within the cell = number of level starts since the cell start
    lvl = np.cumsum(new_level) - 1
    lvl -= np.maximum.accumulate(np.where(new_cell, lvl, 0))
    keep = lvl < max_levels
    out = np.full((max_levels, ny * nx), np.nan)
    for l in range(max_levels):
        m = keep & (lvl == l)
        if not m.any():
            continue
        s = np.bincount(cell[m], weights=zz[m], minlength=nx * ny)
        n = np.bincount(cell[m], minlength=nx * ny)
        out[l, n > 0] = s[n > 0] / n[n > 0]
    used = [l for l in range(max_levels) if not np.all(np.isnan(out[l]))]
    out = out[used] if used else out[:1]
    return out.reshape(len(out), ny, nx)


def load_lidar(paths, crs):
    xs, ys, zs, cs = [], [], [], []
    for p in paths:
        if p.lower().endswith(".npz"):
            d = np.load(p)
            cls = d["cls"]
            keep = np.isin(cls, (2, 11, 17))
            xs.append(d["x"][keep]); ys.append(d["y"][keep]); zs.append(d["z"][keep]); cs.append(cls[keep])
        else:
            import laspy
            las = laspy.read(p)
            cls = np.asarray(las.classification)
            keep = np.isin(cls, (2, 11, 17))
            xs.append(np.asarray(las.x)[keep]); ys.append(np.asarray(las.y)[keep]); zs.append(np.asarray(las.z)[keep])
            cs.append(cls[keep])
            if crs is None and las.header.parse_crs() is not None:
                crs = las.header.parse_crs().to_string()
    if not xs:
        return None
    return np.concatenate(xs), np.concatenate(ys), np.concatenate(zs), np.concatenate(cs), crs


def shifted_fill(dem, fill, seam_cells):
    """`fill` moved onto `dem` for the cells `dem` leaves empty: the difference dem - fill at the nearest cell where
    both have data, fading over seam_cells to their median difference; `fill` as it is where they never overlap"""
    both = ~np.isnan(dem) & ~np.isnan(fill)
    if not both.any():
        return fill
    from scipy.ndimage import distance_transform_edt
    diff = np.where(both, dem - fill, 0.0)
    median = float(np.median(diff[both]))
    d, (ii, jj) = distance_transform_edt(~both, return_indices=True)
    w = np.clip(1.0 - d / max(seam_cells, 1e-9), 0.0, 1.0)
    print(f"DEM fill: {np.isnan(dem).mean() * 100:.1f} % of the grid, shifted {median:+.2f} m (median), "
          f"nearest-cell differences faded over {seam_cells:.0f} cells")
    return fill + w * diff[ii, jj] + (1.0 - w) * median


def bin_mean(ix, iy, z, nx, ny):
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    idx = iy[ok] * nx + ix[ok]
    s = np.bincount(idx, weights=z[ok], minlength=nx * ny)
    n = np.bincount(idx, minlength=nx * ny)
    out = np.full(nx * ny, np.nan, dtype=np.float64)
    m = n > 0
    out[m] = s[m] / n[m]
    return out.reshape(ny, nx), n.reshape(ny, nx)


def fill_small_holes(z, passes=2):
    """Fill isolated NaN cells from their 8 neighbours (lidar shadow of cars, poles)."""
    for _ in range(passes):
        nan = np.isnan(z)
        if not nan.any():
            break
        p = np.pad(z, 1, constant_values=np.nan)
        stack = np.stack([p[1 + dy:p.shape[0] - 1 + dy, 1 + dx:p.shape[1] - 1 + dx]
                          for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dx or dy])
        cnt = np.sum(~np.isnan(stack), axis=0)
        mean = np.nansum(stack, axis=0) / np.maximum(cnt, 1)
        fill = nan & (cnt >= 3)
        z[fill] = mean[fill]
    return z


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--origin", required=True)
    ap.add_argument("--bounds", required=True, help="minlat,minlon,maxlat,maxlon")
    ap.add_argument("--dem", action="append", default=[])
    ap.add_argument("--lidar", action="append", default=[])
    ap.add_argument("--lidar-crs", default=None)
    ap.add_argument("--ground", required=True)
    ap.add_argument("--structures")
    ap.add_argument("--cell", type=float, default=0.5)
    ap.add_argument("--margin", type=float, default=60.0)
    ap.add_argument("--seam", type=float, default=100.0, help="m over which a filling DEM fades onto the earlier ones")
    ap.add_argument("--datum", default="")
    ap.add_argument("--dsm", action="append", default=[], help="surface model raster(s): bridge decks (with --deck-osm)")
    ap.add_argument("--deck-osm", help="OSM file whose road bridges select the DSM deck cells")
    a = ap.parse_args()
    if a.dsm and not a.deck_osm:
        sys.exit("--dsm needs --deck-osm (the DSM also holds trees and buildings)")
    lat0, lon0 = map(float, a.origin.split(","))
    b = list(map(float, a.bounds.split(",")))
    local = f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
    to_local = Transformer.from_crs("EPSG:4326", local, always_xy=True)
    cx, cy = to_local.transform([b[1], b[3], b[1], b[3]], [b[0], b[0], b[2], b[2]])
    x0, x1 = min(cx) - a.margin, max(cx) + a.margin
    y0, y1 = min(cy) - a.margin, max(cy) + a.margin
    nx, ny = int(np.ceil((x1 - x0) / a.cell)) + 1, int(np.ceil((y1 - y0) / a.cell)) + 1
    print(f"grid {nx} x {ny} cells of {a.cell} m ({(x1 - x0):.0f} x {(y1 - y0):.0f} m)")

    # ---- DEM resampled into the local grid (bilinear)
    dem = np.full((ny, nx), np.nan)
    if a.dem:
        import rasterio
        from rasterio.transform import from_origin
        from rasterio.warp import Resampling, reproject
        dst_tr = from_origin(x0 - a.cell / 2, y1 + a.cell / 2, a.cell, a.cell)
        for path in a.dem:
            with rasterio.open(path) as src:
                tmp = np.full((ny, nx), np.nan, dtype=np.float32)
                reproject(rasterio.band(src, 1), tmp, dst_transform=dst_tr, dst_crs=local,
                          src_nodata=src.nodata, dst_nodata=np.nan, resampling=Resampling.bilinear)
                tmp = tmp[::-1]  # north-up raster -> row 0 = south
                dem = np.where(np.isnan(dem), shifted_fill(dem, tmp, a.seam / a.cell), dem)
        print(f"DEM coverage {np.mean(~np.isnan(dem)) * 100:.1f} %")

    ground, decks = dem.copy(), None
    # ---- lidar
    if a.lidar:
        L = load_lidar(a.lidar, a.lidar_crs)
        if L is not None:
            lx, ly, lz, lc, crs = L
            if crs is None:
                sys.exit("lidar CRS unknown: pass --lidar-crs")
            lx, ly = Transformer.from_crs(crs, local, always_xy=True).transform(lx, ly)
            ix = np.round((np.asarray(lx) - x0) / a.cell).astype(np.int64)
            iy = np.round((np.asarray(ly) - y0) / a.cell).astype(np.int64)
            g = np.isin(lc, (2, 11))
            # vertical datum: align lidar to the DEM by the median offset of ground points
            offset = 0.0
            if not np.all(np.isnan(dem)):
                ok = g & (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
                sel = np.flatnonzero(ok)
                if len(sel) > 200000:
                    sel = np.random.default_rng(0).choice(sel, 200000, replace=False)
                d = lz[sel] - dem[iy[sel], ix[sel]]
                d = d[~np.isnan(d)]
                if len(d):
                    offset = float(np.median(d))
            print(f"lidar: {g.sum():,} ground/road points, {(lc == 17).sum():,} bridge-deck points, "
                  f"vertical offset to DEM {offset:+.3f} m")
            lz = lz - offset
            # ground: the DEM is bare earth with bridges removed (interpolated underneath); lidar
            # ground refines it only where both agree. Lidar "ground" well above the DEM is a deck
            # the classifier missed (overpasses over trenched freeways) -> structure candidate.
            inb = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
            dem_at = np.full(len(lz), np.nan)
            dem_at[inb] = dem[iy[inb], ix[inb]]
            above = g & (lz - dem_at > 2.0)
            agree = g & ~(np.abs(lz - dem_at) > 0.5)  # agrees, or no DEM there
            lg, n = bin_mean(ix[agree], iy[agree], lz[agree], nx, ny)
            ground = np.where((n >= 2) & (np.isnan(ground) | ~np.isnan(lg)), np.where(np.isnan(lg), ground, lg), ground)
            print(f"lidar ground: {agree.sum():,} points refine the DEM, {above.sum():,} points are decks "
                  f"classified as ground (> 2 m above the bare earth)")
            dk = (lc == 17) | above
            if dk.any():
                decks = deck_levels(ix[dk], iy[dk], lz[dk], nx, ny)
                for l in range(decks.shape[0]):
                    decks[l] = fill_small_holes(decks[l], passes=2)
                print(f"bridge decks: {decks.shape[0]} stacked level(s)")
    # ---- bridge decks from a surface model, near OSM road bridges only
    if a.dsm:
        import xml.etree.ElementTree as ET
        import rasterio
        from rasterio.transform import from_origin
        from rasterio.warp import Resampling, reproject
        dst_tr = from_origin(x0 - a.cell / 2, y1 + a.cell / 2, a.cell, a.cell)
        dsm = np.full((ny, nx), np.nan)
        for path in a.dsm:
            with rasterio.open(path) as src:
                tmp = np.full((ny, nx), np.nan, dtype=np.float32)
                reproject(rasterio.band(src, 1), tmp, dst_transform=dst_tr, dst_crs=local,
                          src_nodata=src.nodata, dst_nodata=np.nan, resampling=Resampling.bilinear)
                dsm = np.where(np.isnan(dsm), tmp[::-1], dsm)
        nodes, bridges = {}, []
        for _, el in ET.iterparse(a.deck_osm):
            if el.tag == "node":
                nodes[el.get("id")] = (float(el.get("lon")), float(el.get("lat")))
            elif el.tag == "way":
                t = {g.get("k"): g.get("v") for g in el.findall("tag")}
                if "highway" in t and t.get("bridge") in ("yes", "viaduct"):
                    bridges.append([n.get("ref") for n in el.findall("nd")])
                el.clear()
        near = np.zeros((ny, nx), dtype=bool)
        buf = 10.0
        for refs in bridges:
            ll = [nodes[r] for r in refs if r in nodes]
            if len(ll) < 2:
                continue
            px, py = to_local.transform([p[0] for p in ll], [p[1] for p in ll])
            for (ax, ay), (bx, by) in zip(zip(px[:-1], py[:-1]), zip(px[1:], py[1:])):
                i0 = max(0, int((min(ax, bx) - buf - x0) / a.cell)); i1 = min(nx - 1, int((max(ax, bx) + buf - x0) / a.cell) + 1)
                j0 = max(0, int((min(ay, by) - buf - y0) / a.cell)); j1 = min(ny - 1, int((max(ay, by) + buf - y0) / a.cell) + 1)
                if i0 > i1 or j0 > j1:
                    continue
                gx = x0 + np.arange(i0, i1 + 1) * a.cell
                gy = y0 + np.arange(j0, j1 + 1) * a.cell
                X, Y = np.meshgrid(gx, gy)
                dx, dy = bx - ax, by - ay
                u = np.clip(((X - ax) * dx + (Y - ay) * dy) / max(dx * dx + dy * dy, 1e-9), 0.0, 1.0)
                near[j0:j1 + 1, i0:i1 + 1] |= np.hypot(X - ax - u * dx, Y - ay - u * dy) <= buf
        deck = np.where(near & (dsm - ground > 2.5), dsm, np.nan)
        decks = deck[None] if decks is None else np.concatenate([decks, deck[None]])
        print(f"surface model: {len(bridges)} OSM road bridges, {np.mean(~np.isnan(deck)) * 100:.2f} % deck cells")
    ground = fill_small_holes(ground, passes=3)
    print(f"ground coverage {np.mean(~np.isnan(ground)) * 100:.1f} %")
    write_odrz(a.ground, x0, y0, a.cell, ground.astype(np.float32), a.datum)
    print("wrote", a.ground)
    if a.structures:
        if decks is None:
            decks = np.full((ny, nx), np.nan)
        write_odrz(a.structures, x0, y0, a.cell, decks.astype(np.float32), a.datum)
        print(f"wrote {a.structures} ({np.mean(np.any(~np.isnan(decks.reshape(-1, ny, nx)), axis=0)) * 100:.2f} % deck cells)")


if __name__ == "__main__":
    main()
