"""Local server for OdrForge viewer pages with review flags (the user marks what is wrong, Claude checks and answers).

Serves a directory like `python -m http.server`, and keeps each page's review flags next to it:
  GET  /__odrforge_flags          {"ok": true}: tells the viewer to save here instead of only in the browser
  GET  /<page>.flags.json         the page's flags ({"flags": []} while there are none)
  POST /<page>.flags.json         merge the viewer's flags and return the merged file
Ownership of the fields: the viewer writes where a flag is and what it says (FLAG_FIELDS of viewer.html: id, x, y, road,
lane, laneType, junction, s, category, note, created, deleted); tools/inspect/flags.py writes status, reply and resolved.
A merge takes the viewer's fields from the request and keeps the others from the file, so a page left open never
overwrites an answer; flags are never removed, only marked deleted.
usage: python viz_server.py [--dir DIR] [--port 8766] [--bind 127.0.0.1]
"""
import argparse
import functools
import http.server
import json
import os
import threading

VIEWER_FIELDS = ("id", "x", "y", "road", "lane", "laneType", "junction", "s", "category", "note", "created", "deleted")
MAX_BODY = 2 << 20  # bytes
LOCK = threading.Lock()


def read_flags(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def write_flags(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        json.dump(data, f, indent=1, ensure_ascii=False)
    os.replace(tmp, path)


def merge(old, new):
    """the viewer's fields from `new`, everything else (status, reply, resolved) from `old`, by flag id"""
    have = {f["id"]: f for f in (old or {}).get("flags", []) if isinstance(f, dict) and "id" in f}
    for f in new.get("flags", []):
        if not isinstance(f, dict) or not isinstance(f.get("id"), int):
            continue
        cur = have.setdefault(f["id"], {"status": "open"})
        cur.update({k: f[k] for k in VIEWER_FIELDS if k in f})
    out = {k: v for k, v in new.items() if k != "flags"}
    out["flags"] = [have[i] for i in sorted(have)]
    return out


class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        path = self.path.split("?")[0]
        if path.endswith(".flags.json"):
            self.send_header("Cache-Control", "no-store")
        elif path.endswith(".html") or path.endswith("/"):
            # regenerated pages show at once: the browser revalidates (304 while unchanged) instead of reusing its copy
            self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def _json(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = self.path.split("?")[0]
        if url == "/__odrforge_flags":
            return self._json(200, {"ok": True})
        if url.endswith(".flags.json") and not os.path.exists(self.translate_path(url)):
            return self._json(200, {"flags": []})  # none yet (the viewer polls: no 404 noise)
        return super().do_GET()

    def do_POST(self):
        url = self.path.split("?")[0]
        if not url.endswith(".flags.json"):
            return self._json(404, {"error": "only <page>.flags.json accepts POST"})
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0 or n > MAX_BODY:
            return self._json(413, {"error": "empty or too large"})
        try:
            new = json.loads(self.rfile.read(n).decode("utf-8"))
            if not isinstance(new, dict) or not isinstance(new.get("flags"), list):
                raise ValueError("expected {\"flags\": [...]}")
        except ValueError as e:
            return self._json(400, {"error": str(e)})
        path = self.translate_path(url)  # inside the served directory
        with LOCK:
            merged = merge(read_flags(path), new)
            write_flags(path, merged)
        return self._json(200, merged)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", default=".")
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--bind", default="127.0.0.1")
    a = ap.parse_args()
    handler = functools.partial(Handler, directory=os.path.abspath(a.dir))
    srv = http.server.ThreadingHTTPServer((a.bind, a.port), handler)
    print(f"serving {os.path.abspath(a.dir)} at http://{a.bind}:{a.port}/ (review flags on)")
    srv.serve_forever()


if __name__ == "__main__":
    main()
