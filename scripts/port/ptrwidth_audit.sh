#!/bin/sh
# ptrwidth_audit.sh - count the places the tree still assumes a 32-bit pointer.
#
# Compiles every PC-build TU with the host compiler as LP64 (no -m32),
# -fsyntax-only, with only the pointer-width diagnostics on, and prints a
# count per diagnostic and per file. Zero game code is changed; the number is
# the ratchet for the 64-bit lift: it must not go up, and a file is "done"
# when it reads zero. Run from the repository root. Any 64-bit gcc/clang works
# (aarch64 Linux gives the same pointer model as macOS arm64).
#
#   scripts/port/ptrwidth_audit.sh            # summary
#   scripts/port/ptrwidth_audit.sh -v         # plus every diagnostic line
#
# SDL-using TUs are excluded (they need SDL headers, not a pointer question).
set -e
CC=${CC:-gcc}
VERBOSE=${1:-}
OUT=${TMPDIR:-/tmp}/ptrwidth_audit.$$
: > "$OUT"
FLAGS="-fsyntax-only -std=gnu11 -fno-strict-aliasing -fwrapv -funsigned-char \
 -ffreestanding -fms-extensions -fcommon -Wno-all \
 -Wpointer-to-int-cast -Wint-to-pointer-cast -Wincompatible-pointer-types -Wint-conversion \
 -I. -Iinclude -Iinclude/ultra64 -Iinclude/PR -Isrc -Isrc/game -Isrc/inflate \
 -Iport/include -Isrc/libultra -Iport/generated \
 -DVERSION_US -DLANG_US -DREFRESH_NTSC -DLEFTOVERDEBUG -DLEFTOVERSPECTRUM -DBUGFIX_R0 \
 -DNONMATCHING -DAVOID_UB -D_LANGUAGE_C -DGE_HEADLESS=1 -DGE_VIDEO=0"
ls src/game/*.c src/*.c port/src/*.c \
 | grep -v -E 'src/(boot|tlb_manage|tlb_hardwire|rmon|usb|debug|crash|motor)\.c$' \
 | grep -v -E 'port/src/(audio_pc|input_pc|raphnet|video|sysstubs)\.c$' \
 | xargs -P"$(nproc 2>/dev/null || echo 4)" -I{} sh -c "$CC $FLAGS {} 2>&1 | sed 's|^|{}: |'" >> "$OUT" || true
echo "== by diagnostic"
grep -o '\[-W[a-z-]*\]' "$OUT" | sort | uniq -c | sort -rn
echo "== hard errors"
grep -c 'error:' "$OUT" || true
echo "== pointer<->int casts by file"
grep -E 'pointer-to-int-cast|int-to-pointer-cast' "$OUT" | cut -d: -f1 | sort | uniq -c | sort -rn
echo "== total pointer<->int: $(grep -c -E 'pointer-to-int-cast|int-to-pointer-cast' "$OUT")  in $(grep -E 'pointer-to-int-cast|int-to-pointer-cast' "$OUT" | cut -d: -f1 | sort -u | wc -l) files"
[ -n "$VERBOSE" ] && cat "$OUT"
rm -f "$OUT"
