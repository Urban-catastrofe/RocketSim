#!/usr/bin/env python3
"""Summarize RLRESID=12 continuous-vs-restored car-ball contact episodes."""

import argparse
import re
from dataclasses import dataclass
from pathlib import Path


ROW = re.compile(
    r"^EPPHASE (?P<name>\S+) ep=(?P<episode>\S+) "
    r"game=(?P<game>\[[^]]*]) "
    r"continuous_contacts=(?P<continuous>\[[^]]*]) "
    r"continuous_fires=(?P<fires>\[[^]]*]) "
    r"restored_contacts=(?P<restored>\[[^]]*]) "
    r"restored_fires=(?P<restored_fires>\[[^]]*]) "
    r"lost_contacts=(?P<lost>\[[^]]*]) endpoint_gap=(?P<gap>[0-9.]+)$"
)


def ticks(value: str) -> list[int]:
    return [int(tick) for tick in re.findall(r"\d+", value)]


@dataclass
class Episode:
    name: str
    episode: str
    game: list[int]
    continuous: list[int]
    lost: list[int]
    gap: float

    @property
    def cadence_candidate(self) -> bool:
        return not self.lost and len(self.continuous) >= 3 and self.gap > 10.0


def parse(path: Path) -> list[Episode]:
    episodes = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = ROW.match(line)
        if not match:
            continue
        episodes.append(
            Episode(
                name=match["name"],
                episode=match["episode"],
                game=ticks(match["game"]),
                continuous=ticks(match["continuous"]),
                lost=ticks(match["lost"]),
                gap=float(match["gap"]),
            )
        )
    return episodes


def summarize(episodes: list[Episode], worst: int) -> None:
    total_gap = sum(episode.gap for episode in episodes)
    lost = [episode for episode in episodes if episode.lost]
    cadence = [episode for episode in episodes if episode.cadence_candidate]

    def share(rows: list[Episode]) -> float:
        return 100.0 * sum(row.gap for row in rows) / total_gap if total_gap else 0.0

    print(f"episodes={len(episodes)} endpoint_gap_mass={total_gap:.1f}")
    print(
        f"continuous contact losses={len(lost)} gap_mass={sum(row.gap for row in lost):.1f} "
        f"share={share(lost):.1f}%"
    )
    print(
        f"cadence candidates={len(cadence)} gap_mass={sum(row.gap for row in cadence):.1f} "
        f"share={share(cadence):.1f}%"
    )

    print("\nWorst contact-loss episodes")
    for episode in sorted(lost, key=lambda row: row.gap, reverse=True)[:worst]:
        print(
            f"  {episode.name} {episode.episode}: gap={episode.gap:.1f} "
            f"lost={episode.lost} continuous={episode.continuous}"
        )

    print("\nCadence-positive controls")
    for episode in sorted(cadence, key=lambda row: row.gap, reverse=True)[:worst]:
        print(
            f"  {episode.name} {episode.episode}: gap={episode.gap:.1f} "
            f"continuous={episode.continuous}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path, help="cargo-test output containing EPPHASE rows")
    parser.add_argument("--worst", type=int, default=10)
    args = parser.parse_args()
    episodes = parse(args.log)
    if not episodes:
        raise SystemExit("no EPPHASE rows found")
    summarize(episodes, args.worst)


if __name__ == "__main__":
    main()
