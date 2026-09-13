#!/usr/bin/env bash
# =============================================================================
# Configure and build feature combinations for both boards.
#
# A row fails if a build that should work has errors or warnings, or if a
# configuration that must be rejected is accepted.
#
# Usage: scripts/build_matrix.sh [output dir, default build-matrix]
# =============================================================================

set -u
cd "$(dirname "$0")/.."

OUT=${1:-build-matrix}
mkdir -p "$OUT"
failures=0

# run <name> <preset> <ok|reject> [-DNAME=VALUE...]
run() {
    local name=$1 preset=$2 expect=$3
    shift 3
    local dir=$OUT/$name

    rm -rf "$dir"
    if cmake --preset "$preset" -B "$dir" "$@" >"$dir.configure.log" 2>&1; then
        if [ "$expect" = reject ]; then
            echo "FAIL  $name: configuration was accepted, expected an error"
            failures=$((failures + 1))
            return
        fi
    else
        if [ "$expect" = reject ]; then
            echo "ok    $name (rejected)"
        else
            echo "FAIL  $name: configure failed, see $dir.configure.log"
            failures=$((failures + 1))
        fi
        return
    fi

    if ! cmake --build "$dir" -j "$(nproc)" >"$dir.build.log" 2>&1; then
        echo "FAIL  $name: build failed, see $dir.build.log"
        failures=$((failures + 1))
    elif grep -q "warning:" "$dir.build.log"; then
        echo "FAIL  $name: warnings, see $dir.build.log"
        failures=$((failures + 1))
    else
        echo "ok    $name  $(grep -A1 'text' "$dir.build.log" | tail -1 | awk '{print "flash " $1 ", ram " $2 + $3}')"
    fi
}

run f103                    f103 ok
run f407                    f407 ok
run f103-blink              f103 ok -DAPP_BLINK_ONLY=ON
run f103-kalman             f103 ok -DATTITUDE_FILTER=kalman
run f103-minimal            f103 ok -DTELEMETRY=OFF -DWATCHDOG=OFF -DAT_CMD_PID_TOGGLE=OFF \
                                    -DFAULT_VERBOSE=OFF -DCONSOLE_ECHO=OFF
run f407-everything         f407 ok -DAT_CMD_HELP=ON -DAT_CMD_ALL_QUERY=ON -DLOGGING=ON
run f407-bt-only            f407 ok -DCONSOLE_USB=OFF
run f407-no-console         f407 ok -DCONSOLE_USB=OFF -DCONSOLE_BT=OFF -DAUTO_ENABLE=ON
run f103-bt                 f103 reject -DCONSOLE_BT=ON
run f407-no-console-manual  f407 reject -DCONSOLE_USB=OFF -DCONSOLE_BT=OFF
run bad-filter              f103 reject -DATTITUDE_FILTER=lqr

echo "$failures failure(s)"
[ "$failures" -eq 0 ]
