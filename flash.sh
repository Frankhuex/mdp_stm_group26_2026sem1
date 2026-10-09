#!/bin/sh
# Build and flash from any working directory: ./flash.sh [--build-only]
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
case "${1:-}" in
    '') mode=flash ;;
    --build-only) mode=build ;;
    -h|--help) echo 'Usage: ./flash.sh [--build-only]'; exit 0 ;;
    *) echo 'Usage: ./flash.sh [--build-only]' >&2; exit 2 ;;
esac
[ "$#" -le 1 ] || { echo 'Too many arguments.' >&2; exit 2; }
# Include Homebrew for launches outside an interactive shell.
PATH="${PATH}:/opt/homebrew/bin:/usr/local/bin"
export PATH
for tool in make; do
    command -v "$tool" >/dev/null 2>&1 || { echo "Missing tool: $tool" >&2; exit 1; }
done
toolchain=${TOOLCHAIN_DIR:-.tools/arm-gnu-toolchain-15.3.rel1-darwin-arm64-arm-none-eabi/bin}
if [ ! -x "$toolchain/arm-none-eabi-gcc" ]; then
    command -v arm-none-eabi-gcc >/dev/null 2>&1 || {
        echo 'Missing ARM compiler. Run ./scripts/setup-toolchain.sh first.' >&2
        exit 1
    }
fi
openocd=${OPENOCD:-openocd}
if [ "$mode" = flash ]; then
    command -v "$openocd" >/dev/null 2>&1 || { echo 'Missing OpenOCD. Run: brew install open-ocd' >&2; exit 1; }
fi
jobs=${JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || echo 4)}
case "$jobs" in ''|*[!0-9]*|0) echo 'JOBS must be a positive integer.' >&2; exit 2 ;; esac
mkdir -p build
stamp=$(date +%Y%m%d-%H%M%S)-$$
run_step() {
    label=$1
    log=$2
    shift 2
    echo "$label"
    if "$@" > "$log" 2>&1; then
        tail -n 12 "$log"
    else
        result=$?
        cat "$log" >&2
        echo "Failed. Log: $PWD/$log" >&2
        exit "$result"
    fi
}
run_step '[1] Building firmware...' "build/build-$stamp.log" make -s -j"$jobs" "TOOLCHAIN_DIR=$toolchain" build
if [ "$mode" = build ]; then
    echo "Build complete: $PWD/build/mdp21_8.elf"
    exit 0
fi
mkdir -p backups
backup="backups/flash-$stamp.bin"
run_step '[2] Backing up existing Flash...' "build/backup-$stamp.log" "$openocd" -f openocd.cfg -c init -c halt -c "dump_image $backup 0x08000000 0x80000" -c resume -c shutdown
[ "$(wc -c < "$backup" | tr -d ' ')" = 524288 ] || { echo 'Backup size mismatch; flashing stopped.' >&2; exit 1; }
run_step '[3] Programming, verifying and resetting...' "build/flash-$stamp.log" "$openocd" -f openocd.cfg -c 'program build/mdp21_8.elf verify reset exit'
echo "Flash complete. Device reset and running. Backup: $PWD/$backup"
