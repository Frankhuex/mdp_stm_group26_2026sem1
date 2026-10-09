#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
version=15.3.rel1
archive=arm-gnu-toolchain-${version}-darwin-arm64-arm-none-eabi.tar.xz
expected=376808a59ca209c1413236f1c6a509e33da4b29857ab28642b9927cf3048af55
[ "$(uname -s)" = Darwin ] && [ "$(uname -m)" = arm64 ] || { echo "This installer targets Apple Silicon macOS." >&2; exit 1; }
mkdir -p .tools
if [ ! -f ".tools/$archive" ]; then
    curl -fL --retry 2 "https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/packages/generic/gnu-toolchain/${version}/${archive}" -o ".tools/$archive.part"
    mv ".tools/$archive.part" ".tools/$archive"
fi
printf '%s  %s\n' "$expected" ".tools/$archive" | shasum -a 256 -c -
if [ ! -x ".tools/${archive%.tar.xz}/bin/arm-none-eabi-gcc" ]; then
    tar -xJf ".tools/$archive" -C .tools
fi
".tools/${archive%.tar.xz}/bin/arm-none-eabi-gcc" --version
command -v openocd >/dev/null 2>&1 || { echo "Install OpenOCD with: brew install open-ocd" >&2; exit 1; }
openocd --version
