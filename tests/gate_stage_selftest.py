"""Self-test of gate.py's atomic publishing on temporary folders: while a run stages its outputs the published files stay
untouched, the checkers' paths point into the staging folder, and publish() moves every output of an area (carla file,
HTML view, standard file, conformed ground) into place.
usage: python gate_stage_selftest.py   (exit 0 = pass; ctest)"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools", "inspect"))
os.environ.pop("ODRFORGE_GATE_STAGE", None)
import gate as G  # noqa: E402
import terrain_contract as C  # noqa: E402

failures = []


def check(cond, what):
    if not cond:
        failures.append(what)
        print("FAIL:", what)


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "w", encoding="utf-8").write(text)


def read(path):
    return open(path, encoding="utf-8").read() if os.path.exists(path) else None


root = tempfile.mkdtemp()
pub, stage, other = os.path.join(root, "testareas"), os.path.join(root, "stage"), os.path.join(root, "elsewhere")
G.TOUT = C.TOUT = pub
G.FINAL["munich"] = dict(xodr=os.path.join(pub, "munich.xodr"), viz=os.path.join(pub, "munich.html"))
outputs = ("munich.xodr", "munich.html", "munich_standard.xodr", "munich_ground_conformed.odrz")
for name in outputs:
    write(os.path.join(pub, name), "published " + name)

os.environ[G.STAGE_ENV] = stage
G.use_stage(stage)
a = G.AREAS["munich"]
check(a["xodr"] == os.path.join(stage, "testareas", "munich.xodr"), "the carla output is staged")
check(a["args"][a["args"].index("--viz") + 1] == os.path.join(stage, "testareas", "munich.html"), "the HTML view is staged")
f = C.files("munich", pub)
check(f["standard"] == os.path.join(stage, "testareas", "munich_standard.xodr") and
      f["conformed"].startswith(os.path.join(stage, "testareas")), "the contract measures the staged standard / conformed files")
check(f["ground"] == os.path.join(pub, "munich_ground.odrz"), "the grids are read where they are (inputs are not staged)")
check(C.files("munich", other)["standard"] == os.path.join(other, "munich_standard.xodr"),
      "another folder (the contract self-test's copies) is not redirected")

for name in outputs:  # the run writes its outputs into the staging folder
    write(os.path.join(stage, "testareas", name), "new " + name)
check(all(read(os.path.join(pub, n)) == "published " + n for n in outputs), "before publishing the published files are untouched")

folders = G.publish(["munich"], stage)
check(all(read(os.path.join(pub, n)) == "new " + n for n in outputs), "publish moves all four outputs into place")
check(not any(os.path.exists(os.path.join(stage, "testareas", n)) for n in outputs), "nothing is left in the staging folder")
check(folders == {os.path.abspath(os.path.join(stage, "testareas")): os.path.abspath(pub)},
      "publish names the published folder for the build records")
check(G.publish(["munich"], stage) == {}, "publishing again moves nothing (a crashed area keeps its last outputs)")

print("gate stage self-test:", "FAILED" if failures else "ok")
sys.exit(1 if failures else 0)
