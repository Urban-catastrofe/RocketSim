#!/usr/bin/env bash
# Engine accuracy table: replays every .rlpr ground-truth recording through
# three engines and reports, per engine, the fraction of tick-samples within
# tolerance for velocity and position (one column each), like:
#
#   Engine                                   | Vel accuracy | Vel failing | Pos accuracy | Pos failing
#   Stock v3 ZealanL (b1c2ece, no additions) |     28.157%  |     550,063 |      ...
#   Stock v2 (base C++)                      |     88.667%  |      86,772 |      ...
#   Our v3 (tuned)                           |     94.420%  |      42,725 |      ...
#
# A tick-sample passes when its error is <= the tolerance for that field.
# Defaults: 0.5 UU position, 0.5 UU/s velocity (override via RL_POS_TOL /
# RL_VEL_TOL). Engines:
#   our v3    = this checkout (Rust, tuned)
#   stock v2  = C++ RocketSim via the cpp-compare feature (RLCPP=1 pass)
#   stock v3  = pristine v3-rust branch (b1c2ece) in .worktrees/stock-v3,
#               driven by this checkout's harness (overlaid at setup time)
#
# Logs land in /tmp/accuracy_table/ (override with ACC_TABLE_LOG_DIR).
# Re-use existing logs instead of re-running the surveys: ACC_TABLE_REUSE=1.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
POS_TOL="${RL_POS_TOL:-0.5}"
VEL_TOL="${RL_VEL_TOL:-0.5}"
LOG_DIR="${ACC_TABLE_LOG_DIR:-/tmp/accuracy_table}"
SV3="$ROOT/.worktrees/stock-v3"
STOCK_V3_REF="${STOCK_V3_REF:-b1c2ece}"
mkdir -p "$LOG_DIR"

export PATH="$HOME/.cargo/bin:$PATH"

setup_stock_v3() {
    if [ -d "$SV3" ]; then
        return
    fi
    echo ">> setting up stock-v3 worktree at $SV3 ($STOCK_V3_REF)"
    git -C "$ROOT" worktree add --detach "$SV3" "$STOCK_V3_REF"
    local meshes
    meshes="$(readlink -f "$ROOT/rocketsim/collision_meshes")"
    ln -sfn "$meshes" "$SV3/rocketsim/collision_meshes"
    cp "$ROOT/rocketsim/Cargo.toml" "$SV3/rocketsim/Cargo.toml"
    cp "$ROOT/rocketsim/build.rs" "$SV3/rocketsim/build.rs"
    rm -rf "$SV3/rocketsim/tests"
    mkdir -p "$SV3/rocketsim/tests/rl_comparison_test"
    cp -r "$ROOT/rocketsim/tests/mod.rs" "$SV3/rocketsim/tests/"
    cp -r "$ROOT/rocketsim/tests/rl_comparison_test/." "$SV3/rocketsim/tests/rl_comparison_test/"
    rm -rf "$SV3/rocketsim/tests/rl_comparison_test/test_recordings"
    ln -s "$ROOT/rocketsim/tests/rl_comparison_test/test_recordings" \
        "$SV3/rocketsim/tests/rl_comparison_test/test_recordings"
}

echo "== accuracy table: pos tol=$POS_TOL UU, vel tol=$VEL_TOL UU/s =="

if [ -z "${ACC_TABLE_REUSE:-}" ]; then
    setup_stock_v3

    echo ">> survey: our v3 + C++ reference -> $LOG_DIR/our_v3.log"
    (
        cd "$ROOT"
        RLGATE=off RL_POS_TOL="$POS_TOL" RL_VEL_TOL="$VEL_TOL" RLCPP=1 \
            cargo test -p rocketsim --features cpp-compare case_ -- \
            --nocapture --test-threads=1 >"$LOG_DIR/our_v3.log" 2>&1
    ) || {
        echo "!! our-v3 survey failed, see $LOG_DIR/our_v3.log" >&2
        exit 1
    }

    echo ">> survey: stock v3 -> $LOG_DIR/stock_v3.log"
    (
        cd "$SV3"
        # b1c2ece has a debug-only `shift left with overflow` panic in the
        # solver on 6-car recordings; wrapping (as in C++) is the intended
        # behavior, so turn overflow checks off for this engine only.
        RUSTFLAGS="-C overflow-checks=off" \
            RLGATE=off RL_POS_TOL="$POS_TOL" RL_VEL_TOL="$VEL_TOL" \
            cargo test -p rocketsim case_ -- \
            --nocapture --test-threads=1 >"$LOG_DIR/stock_v3.log" 2>&1
    ) || {
        echo "!! stock-v3 survey failed, see $LOG_DIR/stock_v3.log" >&2
        exit 1
    }
fi

awk '
    /==== RLPR / {
        # cargo prefixes rust-engine headers with "test case_x ... ", so the
        # header is not always at line start; take the token after "RLPR".
        name = ""
        for (i = 1; i <= NF; i++)
            if ($i == "RLPR") { name = $(i + 1); break }
        ticks = 0
        for (i = 1; i <= NF; i++)
            if ($i ~ /^ticks=[0-9]+$/) { split($i, a, "="); ticks = a[2] + 0 }
        next
    }
    /^\[/ {
        name = $1; gsub(/[\[\]]/, "", name)
        field = $3
        if (field != "pos" && field != "vel") next
        over = -1
        for (i = 1; i <= NF; i++)
            if ($i ~ /^over=/) {
                s = substr($i, 6)
                if (s == "") s = $(i + 1)
                sub(/%$/, "", s)
                over = s + 0
            }
        if (over < 0 || ticks <= 0) next
        eng = def
        if (name ~ /\|cpp$/) eng = "stock_v2"
        tot[eng, field] += ticks
        fail[eng, field] += int(over / 100 * ticks + 0.5)
        next
    }
    END {
        split("stock_v3 stock_v2 our_v3", engs, " ")
        label["stock_v3"] = "Stock v3 ZealanL (b1c2ece, no additions)"
        label["stock_v2"] = "Stock v2 (base C++)"
        label["our_v3"]   = "Our v3 (tuned)"
        printf "%-46s | %12s | %11s | %12s | %11s\n", \
            "Engine", "Vel accuracy", "Vel failing", "Pos accuracy", "Pos failing"
        printf "%-46s-+-%12s-+-%11s-+-%12s-+-%11s\n", \
            "----------------------------------------------", "------------", \
            "-----------", "------------", "-----------"
        for (e = 1; e <= 3; e++) {
            eng = engs[e]
            row = ""
            split("vel pos", flds, " ")
            for (f = 1; f <= 2; f++) {
                fld = flds[f]
                t = tot[eng, fld] + 0
                fl = fail[eng, fld] + 0
                acc = (t > 0) ? 100.0 * (1.0 - fl / t) : 0.0
                row = row sprintf(" | %11.3f%% | %11s", acc, commas(fl))
            }
            printf "%-46s%s\n", label[eng], row
        }
    }
    function commas(n,   s) {
        s = sprintf("%d", n)
        while (match(s, /^-?[0-9]+[0-9]{3}/))
            s = substr(s, 1, RLENGTH - 3) "," substr(s, RLENGTH - 2)
        return s
    }
' def=stock_v3 "$LOG_DIR/stock_v3.log" def=our_v3 "$LOG_DIR/our_v3.log"
