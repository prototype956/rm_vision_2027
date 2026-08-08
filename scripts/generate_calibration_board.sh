#!/usr/bin/env bash

set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)
DEFAULT_OUTPUT="${PROJECT_ROOT}/docs/tool/assets/calibration_board_9x6_25mm_A4.pdf"
OUTPUT_PATH=${1:-"${DEFAULT_OUTPUT}"}

case "${OUTPUT_PATH}" in
  /*) ;;
  *) OUTPUT_PATH="${PROJECT_ROOT}/${OUTPUT_PATH}" ;;
esac

if ! command -v gs >/dev/null 2>&1; then
  echo "Ghostscript is required to generate the calibration board PDF." >&2
  exit 1
fi

OUTPUT_DIR=$(dirname -- "${OUTPUT_PATH}")
mkdir -p "${OUTPUT_DIR}"
TEMP_DIR=$(mktemp -d)
trap 'rm -rf "${TEMP_DIR}"' EXIT HUP INT TERM
POSTSCRIPT_PATH="${TEMP_DIR}/calibration_board.ps"
PDF_PATH="${TEMP_DIR}/calibration_board.pdf"

cat >"${POSTSCRIPT_PATH}" <<'POSTSCRIPT'
%!PS-Adobe-3.0
%%Title: 9x6 Inner-Corner Camera Calibration Board, 25 mm Squares
%%Creator: rm_vision_2027/scripts/generate_calibration_board.sh
%%Pages: 1
%%Orientation: Landscape
%%DocumentMedia: A4Landscape 841.889764 595.275591 0 () ()
%%BoundingBox: 0 0 842 596
%%HiResBoundingBox: 0 0 841.889764 595.275591
%%EndComments

<< /PageSize [841.889764 595.275591] >> setpagedevice

/mm { 72 25.4 div mul } bind def
/square_size 25 mm def
/board_x 23.5 mm def
/board_y 25 mm def

0 setgray
0 1 6 {
  /row exch def
  0 1 9 {
    /column exch def
    row column add 2 mod 0 eq {
      board_x column square_size mul add
      board_y row square_size mul add
      square_size square_size rectfill
    } if
  } for
} for

% A separate 100 mm verification line with 10 mm ticks.
0 setgray
0.30 mm setlinewidth
newpath
98.5 mm 14 mm moveto
198.5 mm 14 mm lineto
stroke

0 1 10 {
  /tick exch def
  newpath
  98.5 mm tick 10 mm mul add 12.5 mm moveto
  98.5 mm tick 10 mm mul add 15.5 mm lineto
  stroke
} for

/Helvetica findfont 7 scalefont setfont
56 mm 5.5 mm moveto
(100 mm CHECK LINE  |  PRINT AT 100% / ACTUAL SIZE  |  SQUARE 25 mm) show

showpage
%%EOF
POSTSCRIPT

gs -q -dSAFER -dBATCH -dNOPAUSE \
  -sDEVICE=pdfwrite \
  -dCompatibilityLevel=1.4 \
  -dPDFSETTINGS=/prepress \
  -dAutoRotatePages=/None \
  -sOutputFile="${PDF_PATH}" \
  "${POSTSCRIPT_PATH}"

mv "${PDF_PATH}" "${OUTPUT_PATH}"
echo "Generated calibration board: ${OUTPUT_PATH}"
