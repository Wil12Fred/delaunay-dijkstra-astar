#!/bin/sh
# Runs the real ./main in a scratch copy of Prueba/ (the program truncates
# aristas.txt, camino.txt and comprobar*.txt in its working directory).
#  1. OpenGL mode on the committed coord.txt: everything printed before the
#     window opens (points, triangles, edge weights, Dijkstra distance and
#     path) must match tests/expected/opengl_300_0_4.txt byte for byte. The
#     window is opened on a private xvfb-run display (never the caller's),
#     screenshotted, and must show the triangulation and the path.
#  2. Terminal (A*) mode: obstacles are random (srand(time)), so only
#     invariants are checked: the path ends at the goal and the map is 100 rows
#     of 100 cells once the colour codes are removed.
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
cp "$here"/main "$here"/coord.txt "$work"/
cd "$work"

# Always a private virtual display, never the caller's DISPLAY. Four seconds
# in, take a screenshot of it and stop the program (it must still be running).
command -v xvfb-run >/dev/null || { echo "FAIL: xvfb-run is required (package xvfb)"; exit 1; }
command -v import >/dev/null || { echo "FAIL: import is required (package imagemagick)"; exit 1; }
printf '300\n2\n0 4\n' > in.txt
env -u DISPLAY -u WAYLAND_DISPLAY xvfb-run -a -s "-screen 0 1024x1024x24" sh -c \
  './main < in.txt > opengl.txt & pid=$!; sleep 4; import -window root shot.png; kill $pid' \
  || { echo "FAIL: OpenGL mode was not running after 4 s"; exit 1; }
if ! cmp -s opengl.txt "$here/tests/expected/opengl_300_0_4.txt"; then
  echo "FAIL: OpenGL mode output differs from tests/expected/opengl_300_0_4.txt"
  diff "$here/tests/expected/opengl_300_0_4.txt" opengl.txt | head -20
  exit 1
fi
echo "ok: OpenGL mode prints the expected triangulation and Dijkstra path (length 736.935)"

# The window must show the triangulation (yellow) and the path (red). This runs
# in a fresh directory with no aristas.txt, which used to leave drawScene()
# looping on a stream that never opened, so the window stayed black.
im=magick; command -v magick >/dev/null || im=convert
frac() { $im shot.png -fill black +opaque "$1" -fill white -opaque "$1" -format '%[fx:mean]' info:; }
yellow=$(frac yellow); red=$(frac red)
awk -v y="$yellow" -v r="$red" 'BEGIN { exit !(y > 0.01 && r > 0.0001) }' \
  || { echo "FAIL: window shows yellow=$yellow red=$red of the screen, expected the triangulation and path"; exit 1; }
echo "ok: the window draws the triangulation (yellow $yellow) and the path (red $red)"

printf '300\n1\n0 0\n9 9\n1 1.4\n' | timeout 60 ./main > astar.txt
sed 's/\x1b\[[0-9;]*m//g' astar.txt > astar.plain
last=$(grep -E '^\([0-9]+,[0-9]+\)$' astar.plain | tail -1)
[ "$last" = "(9,9)" ] || { echo "FAIL: A* path does not end at the goal (got '$last')"; exit 1; }
rows=$(awk 'length($0) == 100' astar.plain | wc -l)
[ "$rows" -eq 100 ] || { echo "FAIL: A* map has $rows rows of 100 cells, expected 100"; exit 1; }
echo "ok: A* mode prints a path to the goal and a 100x100 map"
