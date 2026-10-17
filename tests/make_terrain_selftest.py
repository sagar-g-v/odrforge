"""Self-test of tools/make_terrain.py's deck selection on synthetic surfaces: a smooth deck is a deck, the edge of a
building or structure and a rough tree crown are not (bridge and street decks both need a smooth surface).
usage: python make_terrain_selftest.py   (exit 0 = pass; ctest)"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import make_terrain as M  # noqa: E402

failures = []


def check(cond, what):
    if not cond:
        failures.append(what)
        print("FAIL:", what)


cell = 0.5
ny, nx = 60, 120
ground = np.full((ny, nx), 100.0)
dsm = ground.copy()
dsm[20:40, 10:50] = 108.0      # a smooth deck 8 m up, 20 m x 10 m
dsm[20:40, 70:110] = 112.0     # a building 12 m up ...
dsm[20:40, 70:90] = 108.0      # ... with a 4 m step in its roof: the edge of a structure (Frankfurt junction 64)
rng = np.random.default_rng(1)
crown = rng.normal(0.0, 1.5, (20, 10))
dsm[2:22, 55:65] = 106.0 + crown  # a tree crown: rough

deck = M.raised_smooth(dsm, ground, cell)
check(deck[30, 30], "the middle of a smooth deck is a deck")
check(deck[25:35, 20:40].all(), "the inner part of the smooth deck is a deck everywhere")
check(not deck[25:35, 88:92].any(), "the 4 m step in a roof (a structure edge) is not a deck")
check(not deck[8:16, 57:63].any(), "a rough tree crown is not a deck")
check(not deck[5, 5], "the bare ground is not a deck")
check(M.smooth(dsm, cell)[30, 30] and not M.smooth(dsm, cell)[30, 90], "smooth: flat deck yes, step edge no")

print("make_terrain self-test:", "FAILED" if failures else "ok")
sys.exit(1 if failures else 0)
