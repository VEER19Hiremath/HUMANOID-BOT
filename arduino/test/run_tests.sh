#!/bin/bash
# Compile wheelodom.ino against the Arduino mock and run the host tests.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "${OUT}"' EXIT
{
  # Arduino auto-generates prototypes; the host compiler needs them.
  echo "void writeFR(int, bool); int aviByte(float); void writeAvi(int, float);"
  echo "struct Wheel; void setDir(Wheel&, bool); float measuredPps(Wheel&); long readCount(Wheel&); void synchronise(float&, float&);"
  sed -e 's/^void setup()/void setup_unused()/' -e 's/^void loop()/void loop_unused()/' \
    "${HERE}/../wheelodom.ino"
} > "${OUT}/fw_under_test.cpp"
g++ -std=c++17 -include "${HERE}/Arduino.h" -I "${OUT}" -o "${OUT}/t" \
  "${HERE}/test_firmware.cpp"
"${OUT}/t"
