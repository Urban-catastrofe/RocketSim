#!/usr/bin/env python3
"""Identify which force the sim is wrong about, from an `RLLEDGER=2` dump.

    RLLEDGER=2 cargo test -p rocketsim -- --nocapture --test-threads=4 \
        | grep '^LEDGER ' | grep -vE '^LEDGER (2v2|3v3)' > all.txt
    python scripts/ledger_fit.py all.txt

Every other pass in the harness reports how wrong the sim is. This one reports
*which force* it was wrong about, by pairing Rocket League's own per-tick
velocity change against the sim's, itemised by source.

Two analyses, in decreasing order of how much you should trust them.

**Localisation** (default) is the reliable one. It takes the residual with the
sim exactly as it stands, reports its median, and shows how concentrated it is.
In every regime measured so far the median is near zero and the worst 1% of
ticks carry ~90-99% of the mass, so the mean is meaningless and the interesting
question is only ever *which ticks*. The owning-cases list answers it.

**Regression** (`--fit`) is the seductive one, and it lies unless the population
is already isolated. Fitting a per-source scale factor over a mixed population
lets least squares spread the blame from a handful of catastrophic ticks across
every coefficient: pooled over all airborne ticks it returns 0.63 for the air
control torque, while the same fit over the single-axis `car_aerial_*`
recordings returns 0.9998 and drives the residual to 2e-5 rad/s. The second is
right. Use `--fit` only on a population where one force dominates, and read the
three guards it prints before believing any number:

  * `GRAVITY CONTROL` — gravity is an exactly-known constant, so its coefficient
    must come out 1.000. If it does not, the fit is misspecified and nothing
    else in the table means anything.
  * `explained` — if the residual does not collapse to near zero, some force is
    missing entirely, and the coefficients are absorbing it.
  * `indep` — the fraction of a regressor orthogonal to the others. Below ~0.05
    the coefficient is arbitrary rather than wrong (gravity and suspension are
    both near-vertical on a grounded car; boost and engine force are both along
    forward). Reported as UNIDENTIFIABLE rather than silently fitted.
"""
import sys
import io
import argparse
from collections import defaultdict, Counter

import numpy as np

# Velocity changes that are not forces: rounding, speed clamps, the flip spin
# cap, multiplicative damping, and impulses cached from another body's contact.
# Scaling these would be meaningless, so they come off the target at weight 1.
FIXED = {'Quantize', 'Clamp', 'FlipSpinCap', 'Damping', 'Bump', 'BallCarHit'}
# Combined entries superseded by their `~` decomposition; counting both
# double-counts.
SUPERSEDED = {'AirControl', 'WheelsFriction'}
# Discrete sub-frame events. Rocket League applies these at the instant of a
# press and splits them across the two straddling recorded frames in a
# proportion the recording never stores, so no per-tick target can match them.
# Such a tick constrains nothing and only injects bias.
SUBFRAME = {'Jump', 'double_jump', 'FlipVel', 'FlipTorque', 'FlipAirDamping',
            'FlipSpinCap', 'FlipZDamp', 'AutoFlipImpulse', 'AutoFlipTorque'}


def parse(path):
    rows = []
    for line in io.open(path, encoding='utf-8', errors='replace'):
        if not line.startswith('LEDGER '):
            continue
        tok = line.split()
        hdr, srcs = {}, {}
        for t in tok[4:]:
            if '=' not in t:
                continue
            k, v = t.split('=', 1)
            if '/' in v:
                lin, ang = v.split('/')
                srcs[k] = (np.array([float(x) for x in lin.split(',')]),
                           np.array([float(x) for x in ang.split(',')]))
            else:
                hdr[k] = v
        rows.append((tok[1], hdr, srcs))
    return rows


def vec(h, key):
    return np.array([float(x) for x in h[key].split(',')])


def localise(rows, label, channel):
    """Residual of the sim as it stands, and which ticks own it."""
    g, s = ('gdv', 'sdv') if channel == 0 else ('gdw', 'sdw')
    unit = 'UU/s' if channel == 0 else 'rad/s'
    if not rows:
        print(f"  {label:9} {g[1:]}: no ticks")
        return
    res = np.array([np.linalg.norm(vec(h, g) - vec(h, s)) for _, h, _ in rows])
    mass = res ** 2
    order = np.argsort(-mass)
    cum = np.cumsum(mass[order]) / max(mass.sum(), 1e-30)
    k1 = max(int(len(rows) * 0.01), 1)
    print(f"  {label:9} {g[1:]}: n={len(rows):6d}  median={np.median(res):.6f}  "
          f"p90={np.percentile(res, 90):.5f}  p99={np.percentile(res, 99):.5f}  "
          f"max={res.max():.4f} {unit}   worst 1% carry {100 * cum[k1 - 1]:.1f}%")
    owners = Counter(rows[i][0] for i in order[:max(len(rows) // 15, 20)])
    for case, n in owners.most_common(6):
        print(f"      {case:<44}{n}")


def fit(rows, channel, label):
    g = 'gdv' if channel == 0 else 'gdw'
    unit = 'UU/s' if channel == 0 else 'rad/s'
    names = sorted({n for _, _, s in rows for n in s
                    if n not in FIXED and n not in SUPERSEDED})
    if not names or not rows:
        print(f"  {label}: nothing to fit")
        return
    X, y = [], []
    for _, h, s in rows:
        target = vec(h, g).copy()
        for n, (lin, ang) in s.items():
            if n in FIXED:
                target -= lin if channel == 0 else ang
        cols = [(s[n][channel] if n in s else np.zeros(3)) for n in names]
        for a in range(3):
            X.append([c[a] for c in cols])
            y.append(target[a])
    X, y = np.asarray(X), np.asarray(y)
    keep = [i for i in range(X.shape[1]) if np.abs(X[:, i]).max() > 1e-9]
    if not keep:
        print(f"  {label}: no active regressors")
        return
    names = [names[i] for i in keep]
    X = X[:, keep]
    beta, _, _, sv = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ beta
    dof = max(len(y) - len(names), 1)
    se = np.sqrt(np.maximum(
        np.diag(float(resid @ resid) / dof * np.linalg.pinv(X.T @ X)), 0.0))
    rms0 = float(np.sqrt((y @ y) / len(y)))
    rms1 = float(np.sqrt((resid @ resid) / len(y)))
    explained = 100 * (1 - rms1 / rms0) if rms0 else 0.0
    cond = sv.max() / sv.min() if sv.min() > 0 else float('inf')

    print(f"\n  --- {label} ({'linear' if channel == 0 else 'angular'}) "
          f"n={len(y) // 3} cond={cond:,.0f} ---")
    print(f"      residual {rms0:.6f} -> {rms1:.6f} {unit} ({explained:.1f}% explained)")
    if explained < 95:
        print("      WARNING: residual did not collapse -- a force is missing "
              "entirely and the coefficients below are absorbing it.")
    for i, nm in enumerate(names):
        others = np.delete(X, i, axis=1)
        if others.shape[1]:
            proj, _, _, _ = np.linalg.lstsq(others, X[:, i], rcond=None)
            indep = float(np.linalg.norm(X[:, i] - others @ proj)
                          / max(np.linalg.norm(X[:, i]), 1e-30))
        else:
            indep = 1.0
        if indep < 0.05:
            tag = '  UNIDENTIFIABLE (collinear)'
        elif se[i] > 0 and abs(beta[i] - 1.0) > 3 * se[i]:
            tag = f'  <-- {100 * (beta[i] - 1):+.2f}%'
        else:
            tag = ''
        note = '   [GRAVITY CONTROL: must be 1.000]' if nm == 'Gravity' else ''
        print(f"      {nm:<22} beta={beta[i]: .5f} +/-{se[i]:.5f}  "
              f"indep={indep:5.3f}{tag}{note}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dump')
    ap.add_argument('--fit', action='store_true',
                    help='also run the per-source regression (see module docstring)')
    ap.add_argument('--case', help='restrict to cases containing this substring')
    ap.add_argument('--per-case', action='store_true',
                    help='fit each case separately -- the only way the regression '
                         'is reliable, because one force dominates per case')
    args = ap.parse_args()

    rows = [r for r in parse(args.dump) if int(r[1]['contact']) == 0]
    if args.case:
        rows = [r for r in rows if args.case in r[0]]
    total = len(rows)
    rows = [r for r in rows if not (SUBFRAME & set(r[2]))]
    print(f"{total} contact-free car-ticks, {len(rows)} after dropping "
          f"sub-frame event ticks\n")

    air = [r for r in rows if int(r[1]['wheels']) == 0]
    ground = [r for r in rows if int(r[1]['wheels']) == 1]
    print("LOCALISATION -- residual of the sim as it stands, and who owns it")
    for label, pop in (('AIRBORNE', air), ('GROUNDED', ground)):
        for ch in (0, 1):
            localise(pop, label, ch)

    if args.per_case:
        by = defaultdict(list)
        for r in rows:
            by[r[0]].append(r)
        for case in sorted(by):
            fit(by[case], 1, case)
    elif args.fit:
        for label, pop in (('AIRBORNE', air), ('GROUNDED', ground)):
            fit(pop, 0, label)
            fit(pop, 1, label)


if __name__ == '__main__':
    main()
