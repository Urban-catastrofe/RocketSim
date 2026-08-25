#!/usr/bin/env python3
"""Aggregate `RLRESID=7` car-ball hit episodes across the whole suite.

    RLRESID=7 cargo test -p rocketsim -- --nocapture --test-threads=1 \
        | grep '^EPGAP ' > once.txt
    RL_HIT_CADENCE=other RLRESID=7 cargo test -p rocketsim -- --nocapture \
        --test-threads=1 | grep '^EPGAP ' > other.txt
    python scripts/rl_episode_gap.py once.txt --vs other.txt

`RLRESID=7` had only ever been read one case at a time, which is why the
car-ball cadence question was settled on `car_ball_soft_touch` and then found
not to generalise. This aggregates the same measurement over every recording so
the suite decides instead of one clip.

What the two numbers mean, and why both are printed
---------------------------------------------------

Each episode is measured twice over the same free-running window:

  * **gap** -- the disagreement in *total* ball velocity change from the start
    of the window to the end. By then both engines have delivered whatever
    impulse they were going to, so the episode's internal cadence has divided
    out. A gap that survives here is a wrong impulse: wrong size, wrong
    direction, or one never fired at all.

  * **peak** -- the worst ball-velocity disagreement at any tick *inside* the
    window. This still contains the cadence phase, because a sim that delivers
    the whole impulse on frame 1 disagrees violently with a game that spread it
    over frames 1-3 even when the totals end up identical.

So `peak` large with `gap` small is the signature of a pure phase defect, and it
is the whole reason the per-tick pass reads car-ball contact so much worse than
the physics deserves. `car_ball_soft_touch` episode 111 is the clean example:
peak 152.2, gap 0.15.

**`phase_share` is the seductive number here.** It is reported as
`1 - sum(gap) / sum(peak)` per family -- mass-weighted, following the harness's
standing rule to rank by mass and never by a mean of per-episode ratios, which
tiny episodes dominate. Treat it as an upper bound on what a perfect cadence
model could recover, not as a measurement of phase:

  * `peak` is taken from a *free-running* window, so it also contains ordinary
    integration drift and any car-side error over those 4-7 ticks. That inflates
    it, and therefore inflates `phase_share`.
  * An episode whose error grows monotonically to the window end has
    `peak == gap` and contributes zero, which is correct.
  * It says nothing about whether the phase is *modelable*. The harness has
    already established that it is not, by any global cadence, delay, contact
    normal, gap, or closing-speed rule -- `EveryOtherTick` fixes soft touch and
    breaks slow push. A high `phase_share` means the impulse magnitudes are
    right, not that a fix is available.

The number to act on is **gap/|game| per family**: episode-total impulse error
as a fraction of the impulse being measured. That is the honest statement of
car-ball accuracy with phase removed, and it is what a cadence model cannot
improve.

`fire_mismatch` counts episodes where the sim fired a different number of extra
impulses than the game labelled `OnHitBall` frames. It is descriptive only --
the game's frame count is not a target, since RL emits `OnHitBall` on cached and
separating frames where no impulse is due.
"""
import argparse
import statistics
import sys
from collections import defaultdict

# Recording-family prefixes, longest first so `car_ball_` and `car_car_` are
# tested before the bare `car_` fallback.
FAMILIES = [
    ("car_ball_", "car_ball"),
    ("car_car_", "car_car"),
    ("mech_", "mech"),
    ("ball_", "ball"),
    ("car_", "car"),
    ("2v2", "match"),
    ("3v3", "match"),
]


def family_of(case):
    for prefix, name in FAMILIES:
        if case.startswith(prefix):
            return name
    return "other"


def parse(path):
    """Read EPGAP lines into dicts. Fields are read by name, so the Rust side
    can append new ones without breaking this."""
    episodes = []
    with open(path) as handle:
        for line in handle:
            if not line.startswith("EPGAP "):
                continue
            parts = line.split()
            row = {"case": parts[1]}
            for token in parts[2:]:
                if "=" not in token:
                    continue
                key, value = token.split("=", 1)
                if "," in value:
                    row[key] = [float(x) for x in value.split(",")]
                else:
                    try:
                        row[key] = float(value)
                    except ValueError:
                        row[key] = value
            row["family"] = family_of(row["case"])
            episodes.append(row)
    return episodes


def cadences(episodes):
    return sorted({e.get("cadence", "?") for e in episodes})


def summarise(episodes):
    """Mass-weighted per-family aggregates, plus an ALL row."""
    by_family = defaultdict(list)
    for e in episodes:
        by_family[e["family"]].append(e)
    by_family["ALL"] = list(episodes)

    rows = []
    for family, eps in by_family.items():
        gap = sum(e["gap"] for e in eps)
        peak = sum(e["peak_err"] for e in eps)
        gmag = sum(e["gmag"] for e in eps)
        mismatch = sum(
            1 for e in eps if e.get("sim_fires") != e.get("game_hit_frames")
        )
        rows.append(
            {
                "family": family,
                "n": len(eps),
                "gmag": gmag / len(eps),
                "gap": gap / len(eps),
                "rel": gap / gmag if gmag else float("nan"),
                "peak": peak / len(eps),
                "phase_share": 1.0 - gap / peak if peak else float("nan"),
                "median_gap": statistics.median(e["gap"] for e in eps),
                "fire_mismatch": mismatch / len(eps),
            }
        )
    rows.sort(key=lambda r: (r["family"] == "ALL", -r["n"]))
    return rows


def print_table(rows, title):
    print(f"\n{title}")
    print(
        f"{'family':<10}{'n':>6}{'|game_dv|':>11}{'gap':>10}{'gap/|game|':>12}"
        f"{'med gap':>10}{'peak':>10}{'phase_share':>13}{'fire_mism':>11}"
    )
    for r in rows:
        print(
            f"{r['family']:<10}{r['n']:>6}{r['gmag']:>11.1f}{r['gap']:>10.2f}"
            f"{r['rel'] * 100:>11.1f}%{r['median_gap']:>10.2f}{r['peak']:>10.1f}"
            f"{r['phase_share'] * 100:>12.1f}%{r['fire_mismatch'] * 100:>10.1f}%"
        )


def print_pairing(base_rows, other_rows, base_label, other_label):
    other_by_family = {r["family"]: r for r in other_rows}
    print(f"\nPAIRED: cadence={base_label} vs cadence={other_label}")
    print(
        f"{'family':<10}{'n':>6}{base_label + ' gap':>14}{other_label + ' gap':>14}"
        f"{'change':>10}{'verdict':>12}"
    )
    for base in base_rows:
        other = other_by_family.get(base["family"])
        if other is None:
            continue
        if base["n"] != other["n"]:
            note = f"n differs ({base['n']} vs {other['n']})"
        else:
            note = ""
        change = (
            (other["gap"] - base["gap"]) / base["gap"] * 100
            if base["gap"]
            else float("nan")
        )
        verdict = "better" if change < -1 else "worse" if change > 1 else "flat"
        print(
            f"{base['family']:<10}{base['n']:>6}{base['gap']:>14.2f}"
            f"{other['gap']:>14.2f}{change:>9.1f}%{verdict:>12}  {note}"
        )


def print_worst(episodes, count):
    ranked = sorted(episodes, key=lambda e: -e["gap"])[:count]
    print(f"\nWorst {len(ranked)} episodes by episode-total gap")
    for e in ranked:
        print(
            f"  {e['gap']:>9.1f}  {e['case']:<44} ep={e.get('ep')} "
            f"|game|={e['gmag']:.1f} |sim|={e['smag']:.1f} peak={e['peak_err']:.1f} "
            f"hit_frames={e.get('game_hit_frames'):.0f} fires={e.get('sim_fires'):.0f}"
        )


def print_split(episodes):
    """The phase-vs-magnitude split the whole exercise is for. Thresholds are
    named rather than tuned: an episode counts as phase-dominated when its
    episode-total gap is under a tenth of its intra-window peak, and as a real
    magnitude defect when the gap exceeds 5% of the impulse being measured."""
    phase = [e for e in episodes if e["peak_err"] and e["gap"] < 0.1 * e["peak_err"]]
    real = [e for e in episodes if e["gmag"] and e["gap"] > 0.05 * e["gmag"]]
    total_gap = sum(e["gap"] for e in episodes)
    print("\nSPLIT")
    print(
        f"  phase-dominated (gap < 10% of peak): {len(phase):>5} of {len(episodes)} "
        f"episodes, {sum(e['gap'] for e in phase) / total_gap * 100:.1f}% of gap mass"
    )
    print(
        f"  real impulse error (gap > 5% of |game_dv|): {len(real):>5} of "
        f"{len(episodes)} episodes, "
        f"{sum(e['gap'] for e in real) / total_gap * 100:.1f}% of gap mass"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="EPGAP log (the baseline run)")
    parser.add_argument("--vs", help="second EPGAP log to pair against")
    parser.add_argument(
        "--worst", type=int, default=15, help="how many worst episodes to list"
    )
    parser.add_argument(
        "--exclude",
        default="",
        help="comma-separated families to drop (e.g. 'match'). The ball's "
        "velocity channel in match replays cannot grade anything -- it holds "
        "78%% of the ball's raw error mass on records whose worst "
        "self-inconsistency is 631156 UU/s against a 6000 UU/s speed cap -- so "
        "'--exclude match' is the honest default for any ball number quoted "
        "from this tool.",
    )
    args = parser.parse_args()

    dropped = {f for f in args.exclude.split(",") if f}

    base = [e for e in parse(args.log) if e["family"] not in dropped]
    if not base:
        sys.exit(f"{args.log}: no EPGAP lines found")
    base_label = cadences(base)
    if len(base_label) > 1:
        sys.exit(f"{args.log}: mixes cadences {base_label}; one run per file")
    base_label = base_label[0]

    base_rows = summarise(base)
    print_table(base_rows, f"EPISODE-TOTAL IMPULSE ERROR (cadence={base_label})")
    print_split(base)
    print_worst(base, args.worst)

    if args.vs:
        other = [e for e in parse(args.vs) if e["family"] not in dropped]
        if not other:
            sys.exit(f"{args.vs}: no EPGAP lines found")
        other_label = cadences(other)
        if len(other_label) > 1:
            sys.exit(f"{args.vs}: mixes cadences {other_label}; one run per file")
        other_label = other_label[0]
        other_rows = summarise(other)
        print_table(other_rows, f"EPISODE-TOTAL IMPULSE ERROR (cadence={other_label})")
        print_pairing(base_rows, other_rows, base_label, other_label)


if __name__ == "__main__":
    main()
