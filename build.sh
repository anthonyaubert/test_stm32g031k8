#!/bin/bash
set -e

echo zig version $(zig version)
touch symbols.txt
zig build
/usr/local/opt/llvm/bin/llvm-objdump --source --disassemble-all --section-headers -t  zig-cache/bin/firmware.elf > main.asm
grep '^00000000.*:$' main.asm | sed 's/^00000000//' > symbols.txt
