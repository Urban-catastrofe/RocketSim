#!/usr/bin/env python3
"""Run scripted scenarios in Rocket League and rank RocketSim divergence."""

from __future__ import annotations

import argparse
import copy
import itertools
import json
import math
import re
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENARIOS = (ROOT / "../../tools/bakkesmod_physics_logger/all_scenarios.json").resolve()


def load_scenarios(path: Path) -> list[dict[str, Any]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    scenarios = data.get("scenarios") if isinstance(data, dict) else data
    if not isinstance(scenarios, list):
        raise ValueError(f"{path} must contain a scenario list or {{'scenarios': [...]}}")
    return scenarios


def select_scenarios(
    scenarios: Iterable[dict[str, Any]], names: list[str], filters: list[str], limit: int | None
) -> list[dict[str, Any]]:
    selected = []
    wanted = set(names)
    for scenario in scenarios:
        name = str(scenario.get("name", ""))
        if wanted and name not in wanted:
            continue
        if filters and not any(pattern in name for pattern in filters):
            continue
        selected.append(scenario)
        if limit is not None and len(selected) >= limit:
            break
    missing = wanted - {str(scenario.get("name", "")) for scenario in selected}
    if missing:
        raise ValueError(f"scenario name(s) not found: {', '.join(sorted(missing))}")
    return selected


def parse_number_range(spec: str) -> list[float]:
    parts = spec.split(":")
    if len(parts) not in (2, 3):
        raise ValueError("sweep range must be START:STOP[:STEP]")
    start, stop = float(parts[0]), float(parts[1])
    step = float(parts[2]) if len(parts) == 3 else 1.0
    if not all(math.isfinite(value) for value in (start, stop, step)) or step == 0:
        raise ValueError("sweep values must be finite and STEP must be non-zero")
    if (stop - start) * step < 0:
        raise ValueError("sweep STEP points away from STOP")
    values = []
    value = start
    comparison = (lambda current: current <= stop + abs(step) * 1e-7) if step > 0 else (
        lambda current: current >= stop - abs(step) * 1e-7
    )
    while comparison(value):
        values.append(value)
        if len(values) > 10_000:
            raise ValueError("sweep expands beyond 10,000 values")
        value += step
    return values


def parse_sweep(spec: str) -> tuple[list[str], list[float]]:
    if "=" not in spec:
        raise ValueError("sweep must be PATH=START:STOP[:STEP]")
    path, range_spec = spec.split("=", 1)
    parts = [part for part in path.split(".") if part]
    if not parts:
        raise ValueError("sweep path cannot be empty")
    return parts, parse_number_range(range_spec)


def set_path(value: Any, path: list[str], replacement: float) -> None:
    current = value
    for part in path[:-1]:
        current = current[int(part)] if isinstance(current, list) else current[part]
    leaf = path[-1]
    if isinstance(current, list):
        current[int(leaf)] = replacement
    else:
        current[leaf] = replacement


def expand_sweeps(
    scenarios: Iterable[dict[str, Any]], specs: list[str]
) -> list[dict[str, Any]]:
    if not specs:
        return [copy.deepcopy(scenario) for scenario in scenarios]
    sweeps = [parse_sweep(spec) for spec in specs]
    variants_per_scenario = math.prod(len(values) for _, values in sweeps)
    if variants_per_scenario > 10_000:
        raise ValueError("Cartesian sweep expands beyond 10,000 variants per scenario")
    expanded = []
    for scenario in scenarios:
        base_name = str(scenario.get("name", "scenario"))
        for values in itertools.product(*(values for _, values in sweeps)):
            variant = copy.deepcopy(scenario)
            suffix = []
            for (path, _), value in zip(sweeps, values, strict=True):
                set_path(variant, path, value)
                suffix.append(f"{'.'.join(path)}={value:g}")
            variant["name"] = f"{base_name}__{'__'.join(suffix)}"
            expanded.append(variant)
    return expanded


def request_for(scenario: dict[str, Any], contacts: bool = False) -> dict[str, Any]:
    request = copy.deepcopy(scenario)
    request["cmd"] = "run"
    request["contacts"] = contacts
    return request


class LiveClient:
    def __init__(self, host: str, port: int, timeout: float):
        self._socket = socket.create_connection((host, port), timeout=timeout)
        self._socket.settimeout(timeout)
        self._reader = self._socket.makefile("rb")

    def close(self) -> None:
        self._reader.close()
        self._socket.close()

    def __enter__(self) -> "LiveClient":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def send(self, request: dict[str, Any]) -> dict[str, Any]:
        payload = json.dumps(request, separators=(",", ":"), allow_nan=False).encode()
        self._socket.sendall(payload + b"\n")
        line = self._reader.readline()
        if not line:
            raise ConnectionError("Rocket League TCP bridge closed the connection")
        response = json.loads(line)
        if not response.get("ok"):
            raise RuntimeError(f"Rocket League rejected request: {response.get('err', response)}")
        return response

    def ping(self) -> None:
        response = self.send({"cmd": "ping"})
        if not response.get("pong"):
            raise RuntimeError(f"unexpected ping response: {response}")


def safe_name(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("._")
    return cleaned or "scenario"


def build_evaluator(cpp: bool) -> Path:
    command = ["cargo", "build", "-p", "rocketsim", "--example", "rl_live_diff"]
    if cpp:
        command.extend(["--features", "cpp-compare"])
    subprocess.run(command, cwd=ROOT, check=True)
    executable = ROOT / "target/debug/examples/rl_live_diff"
    return executable.with_suffix(".exe") if sys.platform == "win32" else executable


def evaluate_capture(executable: Path, capture: Path, worst: int, cpp: bool) -> dict[str, Any]:
    command = [str(executable), "--input", str(capture), "--worst", str(worst)]
    if cpp:
        command.append("--cpp")
    completed = subprocess.run(
        command,
        cwd=ROOT / "rocketsim",
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return json.loads(completed.stdout)


def save_capture(output_dir: Path, request: dict[str, Any], response: dict[str, Any]) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = safe_name(str(request.get("name", "scenario")))
    path = output_dir / f"{stem}.capture.json"
    suffix = 1
    while path.exists():
        path = output_dir / f"{stem}_{suffix}.capture.json"
        suffix += 1
    path.write_text(
        json.dumps({"request": request, "response": response}, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return path


def print_ranking(results: list[dict[str, Any]], top: int) -> None:
    rows = []
    for result in results:
        for engine, summaries in (
            ("rust", result.get("summary", [])),
            ("cpp", result.get("cpp_summary", [])),
        ):
            for summary in summaries:
                rows.append(
                    (
                        float(summary["p95"]), result["name"], engine, summary["entity"],
                        summary["field"], float(summary["mean"]), float(summary["max"]),
                        int(summary["max_tick"]),
                    )
                )
    rows.sort(reverse=True)
    print("\nHighest p95 divergence against Rocket League")
    print(f"{'scenario':38} {'engine':6} {'entity':8} {'field':10} {'mean':>11} {'p95':>11} {'max@tick':>18}")
    for p95, name, engine, entity, field, mean, maximum, tick in rows[:top]:
        print(f"{name[:38]:38} {engine:6} {entity:8} {field:10} {mean:11.4f} {p95:11.4f} {maximum:10.4f}@{tick:<6}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", type=Path, default=DEFAULT_SCENARIOS)
    parser.add_argument("--name", action="append", default=[], help="exact scenario name; repeatable")
    parser.add_argument("--filter", action="append", default=[], help="name substring; repeatable")
    parser.add_argument("--limit", type=int)
    parser.add_argument(
        "--sweep", action="append", default=[], metavar="PATH=START:STOP[:STEP]",
        help="Cartesian numeric JSON-path sweep, e.g. ball.p.2=90:100:1",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7777)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--contacts", action="store_true", help="capture Bullet manifolds")
    parser.add_argument("--cpp", action="store_true", help="also compare stock C++ RocketSim")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--capture", type=Path, action="append", default=[], help="evaluate existing capture")
    parser.add_argument("--capture-only", action="store_true")
    parser.add_argument("--worst", type=int, default=25)
    parser.add_argument("--top", type=int, default=30)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.limit is not None and args.limit <= 0:
        raise ValueError("--limit must be positive")
    if args.worst < 0 or args.top < 0:
        raise ValueError("--worst and --top must be non-negative")

    output_dir = args.output_dir or ROOT / "live_diff" / time.strftime("%Y%m%d-%H%M%S")
    capture_paths = list(args.capture)
    if not capture_paths:
        scenarios = select_scenarios(load_scenarios(args.scenarios), args.name, args.filter, args.limit)
        scenarios = expand_sweeps(scenarios, args.sweep)
        if not scenarios:
            raise ValueError("no scenarios selected")
        print(f"Capturing {len(scenarios)} scenario(s) from {args.host}:{args.port}")
        with LiveClient(args.host, args.port, args.timeout) as client:
            client.ping()
            for index, scenario in enumerate(scenarios, 1):
                request = request_for(scenario, args.contacts)
                print(f"[{index}/{len(scenarios)}] {request.get('name', 'scenario')}", flush=True)
                capture_paths.append(save_capture(output_dir, request, client.send(request)))

    if args.capture_only:
        if args.capture:
            print(f"Selected {len(capture_paths)} existing capture(s); evaluation skipped")
        else:
            print(f"Saved {len(capture_paths)} capture(s) under {output_dir}")
        return 0

    executable = build_evaluator(args.cpp)
    results = []
    for capture_path in capture_paths:
        result = evaluate_capture(executable, capture_path.resolve(), args.worst, args.cpp)
        diff_path = capture_path.with_suffix("").with_suffix(".diff.json")
        diff_path.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
        if not result.get("complete", True):
            print(
                f"warning: {result['name']} ended after {result['ticks']} of "
                f"{result['requested_ticks']} requested ticks",
                file=sys.stderr,
            )
        results.append(result)
    print_ranking(results, args.top)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ConnectionError, OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
