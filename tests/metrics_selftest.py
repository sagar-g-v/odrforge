"""Self-test of tools/inspect/metrics.py on a synthetic OpenDRIVE file: the snapshot measures what the file holds, the
diff reports exactly the metrics a planted change moves and nothing for an unchanged file.
usage: python metrics_selftest.py   (exit 0 = pass; ctest)"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools", "inspect"))
import metrics as M  # noqa: E402

failures = []


def check(cond, what):
    if not cond:
        failures.append(what)
        print("FAIL:", what)


def xodr(grade, lanes=1):
    lane_xml = "".join(
        f'<lane id="-{k}" type="driving" level="false"><link/><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane>'
        for k in range(1, lanes + 1))
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE><header revMajor="1" revMinor="8" name="t" version="1"/>
<road name="r" length="100" id="1" junction="-1"><link/>
<planView><geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry></planView>
<elevationProfile><elevation s="0" a="10" b="{grade}" c="0" d="0"/></elevationProfile>
<lateralProfile/>
<lanes><laneSection s="0"><center><lane id="0" type="none" level="false"/></center><right>{lane_xml}</right></laneSection></lanes>
</road></OpenDRIVE>
"""


def snap(text):
    d = tempfile.mkdtemp()
    open(os.path.join(d, "t.standard.xodr"), "w", encoding="utf-8").write(text)
    return M.snapshot(d, "test")


a = snap(xodr(0.10))
m = a["files"]["t.standard.xodr"]
check(m["roads"] == 1 and m["connecting_roads"] == 0 and m["lanes"] == 1 and m["lanes_driving"] == 1, "structure counted")
check(m["geom_line"] == 1 and m["elevation_records"] == 1 and m["length_plain_m"] == 100.0, "geometry and records")
check(abs(m["grade_max"] - 0.10) < 1e-3 and m["roads_over_20pct"] == 0, "grade measured (10 %)")
check(abs(m["z_max"] - 20.0) < 0.01 and abs(m["z_min"] - 10.0) < 0.01, "height range")

lines, n = M.diff(a, snap(xodr(0.10)))
check(n == 0, "an unchanged file: no metric changed")

b = snap(xodr(0.25, lanes=2))
lines, n = M.diff(a, b)
text = "\n".join(lines)
check("roads_over_20pct" in text and "grade_max" in text and "lanes_driving" in text, "a steeper, wider road is reported")
reported = {l.split()[0] for l in lines if l.startswith("   ")}
expected = {"file_bytes", "grade_max", "lanes", "lanes_driving", "lanes_without_link", "roads_over_20pct", "z_max"}
check(reported == expected and n == len(expected), f"exactly the moved metrics are reported: {sorted(reported)}")
check(" roads " not in text and "geom_line" not in text, "unmoved metrics are not reported")

b["files"]["t.standard.xodr"]["terrain:centre_p95"] = 0.1  # measured only after: absent before, not 0
lines, _ = M.diff(a, b)
check(any("terrain:centre_p95" in l and "absent" in l for l in lines), "a metric measured on one side only reads absent")

print("metrics self-test:", "FAILED" if failures else "ok")
sys.exit(1 if failures else 0)
