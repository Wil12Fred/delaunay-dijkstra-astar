# delaunay-dijkstra-astar

> Delaunay road map, Dijkstra and A*

A university exercise in graph algorithms, from 2016 (updated 2018). Written on macOS with
Xcode, it now builds on Linux too. The program in `Prueba/` has two modes:

1. **Terminal (A\*)**: builds a random 100×100 grid with water, buildings and fire as
   obstacles. It finds a path between two cells with A\* and prints the path and the map in ANSI colors.
2. **OpenGL (Delaunay + Dijkstra)**: reads points from `coord.txt` and runs a Delaunay
   triangulation over them to build a "street" graph. It finds the shortest path between two
   nodes with Dijkstra, behind a small Strategy pattern. It then draws the triangulation in
   yellow and the path in red.

![Delaunay triangulation of 300 points with the Dijkstra path from node 0 to node 4](docs/screenshot.png)

## Build

Needs a C++11 compiler and freeglut on Linux (`sudo apt install freeglut3-dev`). On macOS the
system GLUT/OpenGL frameworks are used.

```sh
cd Prueba
make          # builds ./main and ./trans
```

The Xcode project `Prueba.xcodeproj` is kept for reference only. It points at a `main.cpp`
that is not in the repository, so use the Makefile.

## Run

Run from `Prueba/`, because the input files are read by relative path. **The program
overwrites `aristas.txt`, `camino.txt` and `comprobar2.txt` on every start.** To keep the
committed copies, run it in a scratch copy of the folder, or `git checkout -- Prueba/*.txt`
afterwards.

OpenGL mode (node count, mode `2`, then start and end node):

```sh
printf '300\n2\n0 4\n' | ./main
```

```
...
736.935                                    <- path length
4 20 219 222 107 171 180 135 280 14 69 252 202 211 0
CAMINO:
746 328
774 364
...
965 994
```

The arrow keys pan the view and F1/F2 zoom. Esc quits.

Terminal mode (node count is ignored, mode `1`, then start `x y` and goal `x y`):

```sh
printf '300\n1\n0 0\n9 9\n' | ./main
```

This prints the A\* path as `(row,col)` lines, then the map: `A` water, `B` building,
`F` fire, `T`/`*` path.

`trans` rebuilds `coord.txt` from the last N points of `coord3.txt`:

```sh
echo 300 | ./trans
```

## Layout

| File | Contents |
|---|---|
| `Prueba/main.cc` | entry point, Dijkstra (Strategy pattern), GLUT drawing |
| `Prueba/delaunay.h` | brute-force O(n⁴) Delaunay triangulation (lifting to a paraboloid) |
| `Prueba/grafo.h` | `Nodo`, `Arista`, `Grafo` → adjacency list |
| `Prueba/aest.h` | A\* on a grid with random obstacles |
| `Prueba/Trans.cpp` | point-file converter |
| `Prueba/*.txt` | input points (`coord*.txt`) and outputs from a previous run |
