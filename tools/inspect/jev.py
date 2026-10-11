"""Calibrated second opinion from TypeSafe Jev (System One) for inspection verdicts.

Jev answers typed questions about a text state with probabilities instead of prose. The inspectors
use it to judge measured evidence (numbers, not images: Jev is text only), e.g. whether a flagged
instance is a real geometry defect or legitimate geometry. It never replaces the exact invariants.

    from jev import ask
    a = ask(state_text, {"real": {"type": "noul", "instructions": "...", "criteria": "..."}})
    a["real"]["noul"]  -> probability the answer is yes

Needs TYPESAFE_API_KEY in the environment (user scope on this machine). Only measurement text is
sent to api.typesafe.ai (approved by the user for this purpose).
"""
import json
import os
import urllib.request

URL = "https://api.typesafe.ai/v1/systemone"


def _key():
    k = os.environ.get("TYPESAFE_API_KEY")
    if not k and os.name == "nt":
        import winreg
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER, "Environment") as h:
            k = winreg.QueryValueEx(h, "TYPESAFE_API_KEY")[0]
    if not k:
        raise RuntimeError("TYPESAFE_API_KEY is not set")
    return k


def ask(state, questions, model="jev-latest", timeout=120):
    body = json.dumps({"model": model, "state": state, "questions": questions}).encode()
    req = urllib.request.Request(URL, data=body, method="POST",
                                 headers={"Authorization": "Bearer " + _key(), "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())["answers"]


if __name__ == "__main__":
    print(json.dumps(ask("A lane border jumps sideways by 2.4 m between two consecutive road sections; both "
                         "sections carry the same lanes and no taper is present.",
                         {"defect": {"type": "noul", "instructions": "Is this a real geometry defect in a lane-level road map?"}}),
                     indent=1))
