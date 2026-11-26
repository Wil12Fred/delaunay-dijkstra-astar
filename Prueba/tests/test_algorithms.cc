// Unit tests for the three algorithms: Delaunay (delaunay.h), Dijkstra
// (the Strategy classes in main.cc) and A* (aest.h).
//
// main.cc is included with its main() renamed, so the classes are tested as
// they are, without moving them. Its globals open coord.txt and truncate
// aristas.txt / camino.txt / comprobar*.txt in the current directory: run this
// binary from a scratch directory (`make test` does).
#define main programa_main
#include "../main.cc"
#undef main

#include <cstdio>
#include <cstdlib>
#include <set>
#include <sstream>

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

// ---------------------------------------------------------------- Delaunay

static bool in_circumcircle(double ax, double ay, double bx, double by,
                            double cx, double cy, double px, double py) {
  double adx = ax - px, ady = ay - py, bdx = bx - px, bdy = by - py, cdx = cx - px, cdy = cy - py;
  double det = (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy)
             - (bdx * bdx + bdy * bdy) * (adx * cdy - cdx * ady)
             + (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady);
  double orient = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  return (orient > 0 ? det : -det) > 1e-6;
}

static int hull_size(const vector<double>& x, const vector<double>& y) {
  int n = x.size(), h = 0;
  for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) {   // i->j is a hull edge
    if (i == j) continue;
    bool all_left = true;
    for (int k = 0; k < n && all_left; k++)
      if (k != i && k != j && (x[j]-x[i])*(y[k]-y[i]) - (y[j]-y[i])*(x[k]-x[i]) < 0) all_left = false;
    if (all_left) h++;
  }
  return h;
}

// Four points in convex position, not cocircular: exactly two triangles, and
// the diagonal is the one that keeps the fourth point out of each circle.
static void test_delaunay_quad() {
  double xs[] = {0, 10, 0, 12}, ys[] = {0, 0, 10, 12};
  vector<double> x(xs, xs + 4), y(ys, ys + 4);
  vector<triple> t = delaunayTriangulation(x, y);
  CHECK(t.size() == 2);
  for (size_t i = 0; i < t.size(); i++) {
    std::set<int> v; v.insert(t[i].i); v.insert(t[i].j); v.insert(t[i].k);
    CHECK(v.count(1) && v.count(2));          // both triangles share the 1-2 diagonal
  }
}

// Random points in general position: every triangle has an empty
// circumcircle, and there are 2n - 2 - h of them (h = hull vertices).
static void test_delaunay_random() {
  srand(12345);
  for (int round = 0; round < 5; round++) {
    int n = 15 + 5 * round;
    vector<double> x(n), y(n);
    for (int i = 0; i < n; i++) { x[i] = rand() % 100000 / 100.0; y[i] = rand() % 100000 / 100.0; }
    vector<triple> t = delaunayTriangulation(x, y);
    CHECK((int)t.size() == 2 * n - 2 - hull_size(x, y));
    if ((int)t.size() != 2 * n - 2 - hull_size(x, y)) fprintf(stderr, "n=%d tri=%d hull=%d\n", n, (int)t.size(), hull_size(x, y));
    for (size_t a = 0; a < t.size(); a++)
      for (int p = 0; p < n; p++) {
        if (p == t[a].i || p == t[a].j || p == t[a].k) continue;
        CHECK(!in_circumcircle(x[t[a].i], y[t[a].i], x[t[a].j], y[t[a].j], x[t[a].k], y[t[a].k], x[p], y[p]));
      }
  }
}

// ---------------------------------------------------------------- Dijkstra

// A graph where the direct edge is longer than a detour, built the way
// main() builds it: Nodo -> Arista (Euclidean weight) -> Grafo.
static void test_dijkstra_takes_the_shorter_detour() {
  // 0 ---- 1 is long (10); 0 - 2 - 1 is 2*sqrt(26) ~ 10.198 ... make the detour shorter:
  // points: 0=(0,0) 1=(10,0) 2=(5,1) 3=(5,-20)
  double px[] = {0, 10, 5, 5}, py[] = {0, 0, 1, -20};
  int e[][2] = {{0, 3}, {3, 1}, {0, 2}, {2, 1}};
  vector<Arista*> calles;
  for (int i = 0; i < 4; i++) {
    for (int d = 0; d < 2; d++) {   // both directions, as main() does
      int u = e[i][d], v = e[i][1 - d];
      calles.push_back(new Arista("a", new Nodo("u", px[u], py[u], u), new Nodo("v", px[v], py[v], v)));
    }
  }
  Grafo g(calles);
  Dijkstra dj;
  Contexto ctx;
  ctx.set_algorithm(&dj);
  std::ostringstream out;
  std::streambuf* old = cout.rdbuf(out.rdbuf());
  Camino.clear();
  ctx.metodo(g.GetGrafo(), 0, 1);
  cout.rdbuf(old);
  double expected = 2 * sqrt(26.0);
  double printed = atof(out.str().c_str());
  CHECK(fabs(printed - expected) < 1e-3);
  CHECK(Camino.size() == 3 && Camino[0] == 1 && Camino[1] == 2 && Camino[2] == 0);
}

static void test_dijkstra_unreachable() {
  double px[] = {0, 1, 5, 6}, py[] = {0, 0, 0, 0};
  vector<Arista*> calles;
  calles.push_back(new Arista("a", new Nodo("u", px[0], py[0], 0), new Nodo("v", px[1], py[1], 1)));
  calles.push_back(new Arista("a", new Nodo("u", px[1], py[1], 1), new Nodo("v", px[0], py[0], 0)));
  calles.push_back(new Arista("a", new Nodo("u", px[2], py[2], 2), new Nodo("v", px[3], py[3], 3)));
  calles.push_back(new Arista("a", new Nodo("u", px[3], py[3], 3), new Nodo("v", px[2], py[2], 2)));
  Grafo g(calles);
  Dijkstra dj;
  std::ostringstream out;
  std::streambuf* old = cout.rdbuf(out.rdbuf());
  Camino.clear();
  dj.ACM1metodo(g.GetGrafo(), 0, 3);
  cout.rdbuf(old);
  CHECK(Camino.empty());
  CHECK(atof(out.str().c_str()) >= INF);
}

// ---------------------------------------------------------------- A*

struct AstarRun {
  vector<pair<int, int> > path;   // the "(r,c)" lines, in order
  vector<string> rows;            // the map rows with ANSI colour codes removed
};

static string strip_ansi(const string& s) {
  string r;
  for (size_t i = 0; i < s.size(); i++) {
    if (s[i] == '\033') { while (i < s.size() && s[i] != 'm') i++; continue; }
    r += s[i];
  }
  return r;
}

// Run Astar() on a 100x100 board with the given obstacle cells, feeding the
// two step costs through cin and capturing everything it writes to cout.
static AstarRun run_astar(pair<int, int> s, pair<int, int> f, const vector<pair<int, int> >& walls) {
  static nodito* mapa[alto][largo];
  for (int i = 0; i < alto; i++) for (int j = 0; j < largo; j++) mapa[i][j] = new nodito(i, j, "Free");
  vector<nodito*> obst;
  for (size_t k = 0; k < walls.size(); k++) {
    obst.push_back(new nodito(walls[k].first, walls[k].second, "Building"));
    delete mapa[walls[k].first][walls[k].second];
    mapa[walls[k].first][walls[k].second] = obst.back();
  }
  if (obst.empty()) obst.push_back(new nodito(-1, -1, "Free"));   // "no obstacles" sentinel
  std::istringstream in("1 1.4\n");
  std::ostringstream out;
  std::streambuf* oldin = cin.rdbuf(in.rdbuf());
  std::streambuf* oldout = cout.rdbuf(out.rdbuf());
  Astar(mapa, s, f, obst);
  cout.flush();
  cin.rdbuf(oldin);
  cout.rdbuf(oldout);
  AstarRun r;
  std::istringstream lines(out.str());
  string line;
  while (getline(lines, line)) {
    int a, b;
    if (sscanf(line.c_str(), "(%d,%d)", &a, &b) == 2) r.path.push_back(make_pair(a, b));
    else r.rows.push_back(strip_ansi(line));
  }
  return r;
}

static bool connected_8(const vector<pair<int, int> >& p) {
  for (size_t i = 1; i < p.size(); i++) {
    int dr = abs(p[i].first - p[i-1].first), dc = abs(p[i].second - p[i-1].second);
    if (dr > 1 || dc > 1 || dr + dc == 0) return false;
  }
  return true;
}

static void test_astar_open_board_goes_diagonal() {
  AstarRun r = run_astar(make_pair(0, 0), make_pair(5, 5), vector<pair<int, int> >());
  CHECK(!r.path.empty() && r.path.front() == make_pair(0, 0) && r.path.back() == make_pair(5, 5));
  CHECK(connected_8(r.path));
  CHECK(r.path.size() == 6);   // five diagonal steps
}

// A wall across column 5 from row 0 to row 8: the path has to go round it
// through row 9 or below, never through a wall cell.
static void test_astar_goes_around_a_wall() {
  vector<pair<int, int> > walls;
  for (int i = 0; i <= 8; i++) walls.push_back(make_pair(i, 5));
  AstarRun r = run_astar(make_pair(0, 0), make_pair(0, 9), walls);
  CHECK(!r.path.empty() && r.path.front() == make_pair(0, 0) && r.path.back() == make_pair(0, 9));
  CHECK(connected_8(r.path));
  std::set<pair<int, int> > w(walls.begin(), walls.end());
  bool below = false;
  for (size_t i = 0; i < r.path.size(); i++) {
    CHECK(!w.count(r.path[i]));
    if (r.path[i].first >= 9) below = true;
  }
  CHECK(below);
}

// The coloured map is part of the output: 100 rows of 100 cells, the path
// drawn as T (goal) and *, the obstacles as their letter. The map used to be
// written half with printf and half with cout after sync_with_stdio(false),
// so the cells and their colour codes came out in different places.
static void test_astar_map_is_written_in_order() {
  vector<pair<int, int> > walls;
  for (int i = 0; i <= 8; i++) walls.push_back(make_pair(i, 5));
  AstarRun r = run_astar(make_pair(0, 0), make_pair(0, 9), walls);
  int map_rows = 0;
  for (size_t i = 0; i < r.rows.size(); i++) if (r.rows[i].size() == largo) map_rows++;
  CHECK(map_rows == alto);
  if (map_rows != alto) return;
  size_t first = 0;
  while (r.rows[first].size() != largo) first++;
  CHECK(r.rows[first][5] == 'B');   // wall cell
  CHECK(r.rows[first][0] == '*');   // start, on the path
  CHECK(r.rows[first][9] == 'T');   // goal
  CHECK(r.rows[first + 50] == string(largo, ' '));   // far from everything
}

int main() {
  // Astar() calls sync_with_stdio(false); in libstdc++ the first such call
  // replaces cout's buffer, which would undo the redirection below. Make it here.
  ios_base::sync_with_stdio(false);
  test_delaunay_quad();
  test_delaunay_random();
  test_dijkstra_takes_the_shorter_detour();
  test_dijkstra_unreachable();
  test_astar_open_board_goes_diagonal();
  test_astar_goes_around_a_wall();
  test_astar_map_is_written_in_order();
  if (failures) { std::printf("%d check(s) failed\n", failures); return 1; }
  std::printf("all algorithm tests passed\n");
  return 0;
}
