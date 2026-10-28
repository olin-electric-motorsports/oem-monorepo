#!/usr/bin/env bash
# Line and branch coverage of air.c and air_protocol.c from the four logic
# tests, for both contactor topologies. Fails below 100% unless a lower
# minimum is given. Needs gcc and gcov, as on the Linux CI runner.
# Usage: tests/coverage.sh [minimum_percent]
set -euo pipefail

# This firmware folder, the repo root, the minimum percent, and a scratch folder
APP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$APP/../../../../.." && pwd)"
MIN="${1:-100}"
TESTS=(air_test air_paths_test air_known_issues_test air_invariant_test)
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

status=0
# For each AIR wiring: build with coverage, run all four tests, then check gcov's totals
for negative in 1 0; do
    out="$WORK/negative_$negative"
    mkdir -p "$out"
    flags=(-std=c11 -Wall -Wextra -Werror -O0 --coverage -I"$ROOT" -I"$APP" "-DAIR_CONTROLLED_NEGATIVE=$negative")
    gcc "${flags[@]}" -c "$APP/air.c" -o "$out/air.o"
    gcc "${flags[@]}" -c "$APP/air_protocol.c" -o "$out/air_protocol.o"
    for test in "${TESTS[@]}"; do
        gcc "${flags[@]}" "$out/air.o" "$out/air_protocol.o" "$APP/tests/$test.c" -o "$out/$test"
        "$out/$test" > /dev/null
    done

    echo "AIR_CONTROLLED_NEGATIVE=$negative"
    (cd "$out" && gcov -b -o "$out" "$APP/air.c" "$APP/air_protocol.c") > "$out/gcov.txt"
    # Print line and branch percent for each file; fail if any is under the minimum
    if ! awk -v min="$MIN" '
        /^File / { name = $2; gsub(/\047/, "", name); sub(/.*\//, "", name) }
        /^(Lines executed|Taken at least once):/ {
            match($0, /[0-9.]+%/)
            percent = substr($0, RSTART, RLENGTH - 1)
            printf "  %-15s %-15s %7s%%\n", name, ($1 == "Lines" ? "lines" : "branches"), percent
            if (percent + 0 < min) low = 1
        }
        END { exit low }' "$out/gcov.txt"; then
        echo "  Below ${MIN}%. Lines never run (#####) and branches never taken:"
        grep -n -E '#####|never executed|taken 0%' "$out/air.c.gcov" "$out/air_protocol.c.gcov" || true
        status=1
    fi
done
exit "$status"
