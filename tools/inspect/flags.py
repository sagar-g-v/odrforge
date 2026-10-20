"""Review flags the user marked in the OdrForge viewer (see viz_server.py): list them, answer them.

  python flags.py list DIR_OR_FILE [...] [--all]        open flags (--all: every status), with where and what
  python flags.py set FILE ID --status fixed|question|wontfix|open [--reply TEXT]
A flag's location: x / y are OpenDRIVE inertial coordinates of the clicked point; road / lane / junction / s (along the
drawn reference line) are what the viewer found under it. Only status, reply and resolved are written here; the viewer
owns the rest (viz_server.py merges by id).
"""
import argparse
import datetime
import glob
import json
import os
import sys

STATUSES = ("open", "fixed", "question", "wontfix")


def files(args):
    out = []
    for a in args:
        out += sorted(glob.glob(os.path.join(a, "*.flags.json"))) if os.path.isdir(a) else [a]
    return out


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def cmd_list(a):
    n = 0
    for path in files(a.paths):
        data = load(path)
        for f in data.get("flags", []):
            if f.get("deleted") or (not a.all and f.get("status", "open") != "open"):
                continue
            n += 1
            where = f"road {f.get('road')} lane {f.get('lane')} s~{f.get('s')}" if f.get("road") else "no road"
            if f.get("junction") not in (None, "-1"):
                where += f" junction {f.get('junction')}"
            print(f"{os.path.basename(path)} #{f['id']} [{f.get('status', 'open')}] {f.get('category')}: {where} "
                  f"(x {f['x']:.2f}, y {f['y']:.2f}) - {f.get('note', '')}")
            if f.get("reply"):
                print(f"    reply: {f['reply']}")
    print(f"{n} flags")


def cmd_set(a):
    data = load(a.file)
    for f in data.get("flags", []):
        if f.get("id") == a.id:
            f["status"] = a.status
            if a.reply is not None:
                f["reply"] = a.reply
            f["resolved"] = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")
            tmp = a.file + ".tmp"
            with open(tmp, "w", encoding="utf-8", newline="\n") as fo:
                json.dump(data, fo, indent=1, ensure_ascii=False)
            os.replace(tmp, a.file)
            print(f"#{a.id} -> {a.status}")
            return 0
    print(f"no flag #{a.id} in {a.file}", file=sys.stderr)
    return 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("list")
    p.add_argument("paths", nargs="+")
    p.add_argument("--all", action="store_true")
    p = sub.add_parser("set")
    p.add_argument("file")
    p.add_argument("id", type=int)
    p.add_argument("--status", choices=STATUSES, required=True)
    p.add_argument("--reply")
    a = ap.parse_args()
    return cmd_list(a) if a.cmd == "list" else cmd_set(a)


if __name__ == "__main__":
    sys.exit(main() or 0)
