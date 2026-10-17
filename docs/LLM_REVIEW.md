# Optional LLM review of the generated OpenDRIVE

The deterministic checkers (tools/inspect: gate.py, invariants.py, laneflow.py, annex_f.py and the det_* detectors)
measure the outputs exactly, but they cannot say whether a network makes sense in the real world. The LLM review is a
separate, optional pass that does that judgement. A reviewer (a Claude Code session, or `claude -p` headless) looks at
each candidate spot through every real-world perspective: car driver, pedestrian, wheelchair user, cyclist, simulator
(CARLA, esmini), visual, and logical / programmatic. It flags what is wrong and marks legitimate real-world geometry
as not a defect. Its verdicts land next to each output as `<stem>.review.json`, in a schema close to the viewer's
`<stem>.flags.json`, so the odrforge-viewer can show them beside the user's own flags.

It never gates a change and never replaces a checker. A verdict is a judgement with its evidence and a confidence;
the exact checkers stay the definition of correct.

## Files

| file | what |
|---|---|
| `tools/inspect/llm_review.py` | `prepare`, `prompt`, `run`, `merge`, `check-prompt` |
| `tools/inspect/llm_review_prompt.py` | the versioned prompt: `PROMPT_VERSION`, target OpenDRIVE versions, the checkers it knows (with fingerprints), the verdict vocabulary, `text()` |
| `tests/llm_review_selftest.py` | deterministic tests: a planted defect for every own check, merge validation, staleness detection |
| `docs/research/ROAD_DESIGN_SENSIBILITY.md` | sourced real-world design ranges (lane widths, tapers, radii, crossings, ...) the prompt's judgement rests on |

## Inputs of a review (the bundle)

`python tools/inspect/llm_review.py prepare OUT.xodr [...] --out DIR` writes one bundle per output under `DIR/<stem>/`:

- `manifest.json`: the output path, the reviewed file's sha256 (with `--snapshot` the copy that was reviewed), header
  (revMinor, vendor), OSM source, clip box, basemap, checker results used, prompt version, chunks.
- `facts.json`: the review's own measured network facts over the whole file (not only the spots):
  - open lane ends (a lane of a moving type that stops wider than 0.3 m with no continuation), by how the road end is
    linked (none, road, junction, direct junction) and by flow (the lane runs into / out of the end);
  - dead ends against the OSM source: at the clip boundary, mapped by OSM (way ends with no other highway, noexit,
    turning circle), OSM continues (only as a `highway=track`: reported separately, minor), or no OSM node; other
    road ends close by;
  - crosswalk objects written twice (centres within 1 m);
  - gaps and kinks at every lane joint (road links, junction connections, lane sections), link symmetry;
  - lane drops / adds and their taper length against the MUTCD merging taper for the road's speed;
  - zero-width lanes, slivers, negative widths, implausible driving-lane widths;
  - crosswalk ends: reaching a walking lane directly, across lanes a pedestrian may cross (curb, parking, cycle lane),
    only across driving lanes, only an OSM footway the network does not model, or nothing;
  - per junction: the attached roads, connections, turns between arms that both have lanes for them but no connection
    (OSM turn restrictions applied), wrong-way connections;
  - the driving-lane graph (links, junction connections, lane changes): the share of lane length that can only end in
    an unexplained dead end, the share that cannot be reached, closed loops with no way out;
  - lane-centre turning radius of every connecting road (exact curvature, 1 m average);
  - OSM vehicle ways inside the area with no road.
- `candidates.json`: those facts plus every finding of the checkers for this file, normalised to one shape
  (`id, tool, kind, severity, x, y, road, lane, junction, s, osm_ways, metric`). Checker results come from the gate
  (`build/tmp/inspect/gate/<area>/`, used only when newer than the output; the gate converts the carla profile) or,
  with `--run-checkers`, from running the five detectors, invariants.py and laneflow.py on the file. The results used
  are copied into the bundle (`checkers/`) so a later gate run cannot change what was reviewed.
- Spots: candidates are clustered (30 m) and `--spots` of them (default 20) are chosen round-robin over the leading
  kind, so every kind of problem is looked at before any kind gets a second look; plus `--samples` (default 4)
  ordinary junctions with no candidate within 40 m, so unknown issues can surface.
- `chunk_NN.md` / `chunk_NN.json`: `--chunk` spots each (default 4): the file facts (with the strongest critical /
  major clusters no spot shows, and any checker that failed on the file), then per spot the roads in view (lanes,
  widths, flow and continuation at each end, elevation, lane-less helper roads labelled, OSM ways with the tags that
  decide lanes, direction, access, sidewalks and cycleways), the junctions in view, the measured facts in view and
  every candidate in view, strongest first (the first 30 numbered as in the image, the rest listed by id). Findings
  on roads without lanes stay listed but count as info when spots are chosen. Small enough for one LLM context.
- `img/spot_*.png`: plan views (matplotlib) in the file's metres: lanes coloured by type, lane borders, reference
  lines, driving-direction arrows, junction connection paths, open lane ends (red X), numbered candidates (red
  circles), OSM ways (dashed, with way ids; footways dotted), crosswalks (hatched), over the area's satellite / map
  basemap when one exists (`<area>_basemap` next to the output).
- `PROMPT.md`: the prompt this bundle was built for.

## How to run

```bash
# 1. bundles (a Python with matplotlib for the images; the osm2carla venv has none, so use Python 3.12 or --no-images)
python tools/inspect/llm_review.py prepare build/testareas/munich.xodr build/testareas/munich_standard.xodr \
    --out build/tmp/llm_review/bundles --snapshot build/tmp/llm_review/inputs --run-checkers
# 2a. review in a Claude Code session: print what is pending, then the session reads PROMPT.md, each chunk and its
#     images, and writes verdicts/chunk_NN.json per chunk
python tools/inspect/llm_review.py run build/tmp/llm_review/bundles --runner session
# 2b. or headless, one claude -p call per chunk (--dry-run prints the commands; --limit N chunks per bundle)
python tools/inspect/llm_review.py run build/tmp/llm_review/bundles --runner claude-cli --limit 1
# 3. verdicts -> <stem>.review.json next to each output (rejected verdicts are listed and make the exit code 1)
python tools/inspect/llm_review.py merge build/tmp/llm_review/bundles
# 3b. optionally also as open flags in the viewer's <stem>.flags.json (listed and answered with flags.py)
python tools/inspect/llm_review.py merge build/tmp/llm_review/bundles --as-flags
```

`--reuse` rebuilds existing bundles from the same reviewed file and checker results (after a prompt or tool change);
earlier verdicts are moved to `verdicts.prev.*` and never merged into new spots. With `--run-checkers`, a reused
bundle only re-runs the checkers whose result is missing. The checkers run with the gate's venv
(`--checker-python`); a checker that fails is named in the chunk facts, not silently skipped.

`--snapshot DIR` copies each output first (refusing a file written less than `--stable-seconds` ago, and checking
the copy's sha256 against the original), so a gate run that rewrites the outputs cannot change the file under review.
`merge` writes `matches_current_output: false` (and warns) when the output changed after the review.

A Claude Code session can review the bundles itself or hand one bundle to each of several subagents (the first
review did that: one subagent per output, each validating its own verdicts with
`merge <bundle> --dest <scratch>`). `run` and `merge` take a bundle directory or a directory of bundles.

Runners: `session` and `claude-cli` (the local Claude Code CLI; found on PATH or in the desktop app's
`%APPDATA%\Claude\claude-code\<version>\`); after each chunk it checks the verdict file as merge would and gives the
reviewer one call to fix what would be rejected. There is no runner that sends the network to any other service; adding
one (e.g. the Anthropic API) needs the user's approval first.

## Output: `<stem>.review.json`

```json
{"schema": 1, "source": "llm-review", "file": "munich.xodr", "sha256": "...", "matches_current_output": true,
 "prompt_version": "1.0.0", "odrforge_version": "0.6.0", "header": {"revMinor": "4", ...}, "reviewed": "...",
 "reviewers": ["..."], "chunks_reviewed": [...], "chunks_total": 6,
 "counts": {"verdict": {...}, "category": {...}, "severity": {...}, "kind": {...}},
 "flags": [{"id": 1, "status": "open", "x": 12.3, "y": -45.6, "road": "120", "lane": -1, "laneType": null,
            "junction": "5", "s": 33.2, "category": "connection", "note": "one line for the viewer list",
            "created": "...", "source": "llm-review", "verdict": "defect", "kind": "lane_ends_abruptly",
            "severity": "critical", "perspective": ["car", "simulator"], "osm_ways": ["4240770"],
            "evidence": ["measured fact with units", "..."], "reasoning": "real-world judgement",
            "spec": "1.8.0 11.5", "confidence": 0.85, "candidates": ["link_continuity:12"], "spot": "S03"}],
 "legitimate": [ ...the same shape, verdict "legitimate"... ],
 "rejected": ["chunk_02 verdict 3: x/y outside spot S07", "..."]}
```

`flags` hold verdicts `defect` and `uncertain`; `legitimate` holds what the reviewer cleared, with the reason, so it
stops coming back. The fields `id, x, y, road, lane, laneType, junction, s, category, note, created, status` are the
viewer's own; `category` is one of the viewer's categories (geometry, lanes, connection, junction, missing, signal,
elevation, other). Road ids change between builds: use `osm_ways` to find the place in another build.

With `--as-flags`, merge also appends every defect / uncertain verdict to the viewer's `<stem>.flags.json` as an
open flag: the viewer's fields (category, note "[LLM review defect, major, confidence 0.85] ... (OSM ways ...)", x,
y, road, lane, junction, s) plus source "llm-review", severity, perspective, evidence, reasoning, confidence,
osm_ways and the reviewed sha256. Their ids start at 1001, so a viewer page left open does not number a new
hand-made flag the same; the user's flags are never changed or removed (viz_server.py keeps fields it does not own),
and a verdict already added (same `llm_key`) is not added again. `flags.py list / set` then works on them.

A candidate may be given by its number in the spot's list (the numbered circles); merge resolves it to the id.
merge rejects a verdict that does not follow the vocabulary, has no evidence, names a road that is not in the file, a
candidate that does not exist, or a position outside the spot it was given: a reviewer cannot place a flag where it
did not look.

## Keeping the prompt current

The prompt is code (`llm_review_prompt.py`). `python tools/inspect/llm_review.py check-prompt [--xodr F ...]` exits 1
and says why when the prompt may be outdated:

- a checker in tools/inspect is new (not in `CHECKERS` or `NOT_CHECKERS`), changed (its fingerprint differs) or gone;
- the repository's VERSION is not the `ODRFORGE_VERSION` the prompt was revised for;
- an output's header revMinor is not a target, or its vendor version differs.

Then: read what changed, revise the prompt text where it no longer holds (a new checker's findings, a new lane type, a
new OpenDRIVE version's features, a changed rule), raise `PROMPT_VERSION`, set `REVISED` / `ODRFORGE_VERSION`, and
record the fingerprints (`check-prompt --print-fingerprint` prints the `CHECKERS` lines). Never update a fingerprint
without reading the change. Bundles and verdicts carry the prompt version, and merge rejects a verdict made with a
different prompt version than its bundle.

Run the selftest after any change to llm_review.py: `python tests/llm_review_selftest.py` (every planted defect must
be found at its place and not in the clean network; the limits in llm_review.py are never loosened to make a file look
clean).

## Limits

- The review judges spots, not the whole file: the facts and candidates cover every road, but a reviewer reads only
  the chosen spots (default 24 per file). Raise `--spots` for more coverage; the facts summary of each chunk gives the
  whole-file picture.
- Images are plan views. Elevation, bridges and stacked levels are only in the facts (and the checkers); a reviewer
  must not judge vertical clearance from a plan view.
- OSM is the reference, and OSM can be wrong or incomplete; a reviewer may call a faithful conversion of a bad OSM
  way legitimate for the converter while noting the OSM issue.
- Missing turns are measured against the arms of one junction; a movement served by a neighbouring junction of a
  split intersection can appear missing. U-turns are not counted.
- Checker results exist for the carla profile from the gate; the standard profile needs `--run-checkers` (slower).
  The checkers' own false positives reach the reviewer as candidates; the prompt asks the reviewer to clear them
  with the fact that does, and they are reported back to the checkers. The first review found four, fixed since
  (prompt 1.0.1, tests/review_checkers_selftest.py, which fails 14 of its 19 checks on the old checkers):
  det_junction_gore crashed on 1.8.0 outputs, compared reference lines instead of lane centres and read one lane
  per connecting road; det_osm_fidelity and det_lane_transitions read left-hand traffic as right-hand;
  det_link_continuity did not read direct junctions. On the 12 reviewed outputs: conn_discontinuity 12246 -> 0,
  lane_no_successor 2427 -> 13, osm_fidelity lane_count 693 -> 79, lane_transitions count_mismatch 1350 -> 68,
  open_lane_end_junction 1098 -> 764; conn_wiggle in London / Tokyo 22 / 8 -> 135 / 167 (hidden before by the
  right-hand headings).
- The console of this machine is cp932: print review files with `PYTHONIOENCODING=utf-8` (the files are UTF-8).
- Verdicts are LLM judgements: each carries a confidence and its evidence, and the deterministic checkers stay the
  definition of correct.
