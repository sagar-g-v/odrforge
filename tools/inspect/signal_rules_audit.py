"""Audit traffic-light control against OpenDRIVE 1.8.0 and the ASAM Junction Guideline 1.0.0:
  - Junction Guideline §6.3.3: vehicles stop before the pedestrian crossing - no stop line (1100001) on a crosswalk;
  - 1.8.0 §14.1 "Signals shall be positioned in such a way that it is clear to which road or lane they belong": every
    approach of a signalised junction (a junction with <controller>) whose lanes drive into it carries a signal near
    its end (15 m);
  - 1.8.0 §12.1 Table 54: no controller in a direct or virtual junction (1.9 junctions.virtual.no_controllers);
  - reported: lights without a stop line (approaches too short to stop before their crosswalk keep <validity>).
usage: python signal_rules_audit.py file.xodr [...]
"""
import sys, xml.etree.ElementTree as ET
from collections import Counter
DRIVE = {"driving", "onRamp", "offRamp", "connectingRamp", "entry", "exit", "bidirectional"}
for path in sys.argv[1:]:
    root = ET.parse(path).getroot()
    sig_j = {j.get("id") for j in root.findall("junction") if j.find("controller") is not None}
    bad_j = [j.get("id") for j in root.findall("junction") if j.get("type") in ("direct", "virtual") and j.find("controller") is not None]
    unc, on_cw, n_stop, lights, lights_noref = [], 0, 0, 0, 0
    for r in root.findall("road"):
        if r.get("junction") != "-1": continue
        L = float(r.get("length")); lht = r.get("rule") == "LHT"
        secs = r.findall("lanes/laneSection"); sigs = r.findall("signals/signal")
        cws = [(float(o.get("s")), float(o.get("length", 0))) for o in r.findall("objects/object") if o.get("type") == "crosswalk"]
        for s in sigs:
            if s.get("type") == "1100001":
                n_stop += 1
                if any(abs(float(s.get("s")) - c) <= 0.5 * l for c, l in cws): on_cw += 1
            if s.get("dynamic") == "yes":
                lights += 1
                if s.find("reference") is None: lights_noref += 1
        for end, tag in (("end", "successor"), ("start", "predecessor")):
            lk = r.find(f"link/{tag}")
            if lk is None or lk.get("elementType") != "junction" or lk.get("elementId") not in sig_j: continue
            sec = secs[-1] if end == "end" else secs[0]
            side = ("right" if not lht else "left") if end == "end" else ("left" if not lht else "right")
            if not [l for l in sec.findall(f"{side}/lane") if l.get("type") in DRIVE]: continue
            if not [s for s in sigs if (float(s.get("s")) > L - 15 if end == "end" else float(s.get("s")) < 15)]:
                unc.append((r.get("id"), end))
    print(f"{path}: stop lines {n_stop} on crosswalk {on_cw}; lights {lights} without stop line {lights_noref}; "
          f"uncontrolled approaches {len(unc)} {unc[:5]}; controllers in direct/virtual {bad_j}")
