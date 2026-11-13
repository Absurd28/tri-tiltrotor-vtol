#!/usr/bin/env python3
"""Compare the mode A vs mode B probe runs from their monitor logs."""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).parent.parent


def summary(name):
    files = sorted((ROOT / "runs" / name).glob("monitor*.csv"))
    frames = [pd.read_csv(f, skiprows=1) for f in files if f.stat().st_size > 60]
    if not frames:
        return None
    df = pd.concat(frames)
    tail = df.tail(max(len(df) // 4, 20))
    return dict(episodes=len(df),
                mean_r_last_quarter=float(tail["r"].mean()),
                mean_len=float(tail["l"].mean()),
                best_r=float(df["r"].max()))


for name in ("probe_A", "probe_B"):
    s = summary(name)
    print(name, "->", s if s else "no data")
