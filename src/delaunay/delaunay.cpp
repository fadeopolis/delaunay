// Copyright (c) 2020, Fabian Gruber
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

///
/// Bowyer-Watson algorithm for Delaynay triangulation.
///

#include <delaunay/delaunay.hpp>

#include <cassert>   // for assert
#include <limits>    // for FLT_MAX
#include <cmath>     // for std::abs, std::isfinite, std::isnan
#include <optional>  // for std::optional
#include <set>       // for std::optional
#include <map>       // for std::map
#include <vector>    // for std::vector
#include <algorithm> // for std::sort

using namespace delaunay;

namespace {

static inline const double INFTY = std::numeric_limits<double>::infinity();

/// https://doubleing-point-gui.de/errors/comparison/
static bool nearly_equal(double a, double b, double epsilon) {
  /// TODO
  assert(!std::isnan(a));
  assert(!std::isnan(b));

  const double absA = std::abs(a);
  const double absB = std::abs(b);
  const double diff = std::abs(a - b);

  const double MIN_NORMAL = std::numeric_limits<double>::min(); // WTF C++
  const double MAX_VALUE  = std::numeric_limits<double>::max();

  if (a == b) {
    // shortcut, handles infinities
    return true;
  } else if ((a == 0) || (b == 0) || (absA + absB < MIN_NORMAL)) {
    // a or b is zero or both are extremely close to it
    // relative error is less meaningful here
    return diff < (epsilon * MIN_NORMAL);
  } else { // use relative error
    return diff / std::min<double>((absA + absB), MAX_VALUE) < epsilon;
  }
}

/// a 2D line represented by its line equation
/// ax + by = c
struct Line final {
  double a, b, c;
};

// Get the line equation (ax + by = c) from 2 points.
static Line line_equation_from_points(Point A, Point B) {
  assert(A != B);

  assert(std::isfinite(A.x()));
  assert(std::isfinite(A.y()));
  assert(std::isfinite(B.x()));
  assert(std::isfinite(B.y()));

  /// normalize
  // if (B < A) { std::swap(A, B); }

  const double a = B.y() - A.y();
  const double b = A.x() - B.x();

  /// normalize
  // if (a) { b = b / a; a = 1; }

  const double c = a * A.x() + b * A.y();

  assert(std::isfinite(a));
  assert(std::isfinite(b));
  assert(std::isfinite(c));
  assert(nearly_equal(a * B.x() + b * B.y(), c, 0.0001));

  return Line{a, b, c};
}

// Function which converts the input line to its perpendicular bisector.
// It also inputs the points whose mid-point lies on the bisector
static Line perpendicular_bisector_from_line(Point P, Point Q) {
  const Line PQ = line_equation_from_points(P, Q);

  assert(std::isfinite(PQ.a));
  assert(std::isfinite(PQ.b));
  assert(std::isfinite(PQ.c));

  const Point mid_point = { (P.x() + Q.x()) / 2, (P.y() + Q.y()) / 2 };

  assert(std::isfinite(mid_point.x()));
  assert(std::isfinite(mid_point.y()));

  // c = -bx + ay
  Line bisect;

  bisect.a = -PQ.b;
  bisect.b = +PQ.a;
  bisect.c = -PQ.b * mid_point.x() + PQ.a * mid_point.y();

  assert(std::isfinite(bisect.a));
  assert(std::isfinite(bisect.b));
  assert(std::isfinite(bisect.c));

  return bisect;
}

// Returns the intersection point of two lines
static std::optional<Point> line_line_intersect(Line l1, Line l2) {
  const double determinant = l1.a * l2.b - l2.a * l1.b;

  assert(std::isfinite(determinant));

  if (determinant == 0) {
    /// The lines are parallel.
    return std::nullopt;
  } else {
    const double x = (l2.b * l1.c - l1.b * l2.c) / determinant;
    const double y = (l1.a * l2.c - l2.a * l1.c) / determinant;

    assert(std::isfinite(x));
    assert(std::isfinite(y));

    return Point{x, y};
  }
}

static Point find_circumcenter(Point P, Point Q, Point R) {
  /// normal to the line PQ passing through (P-Q)/2
  const Line bisect_PQ = perpendicular_bisector_from_line(P, Q);
  const Line bisect_QR = perpendicular_bisector_from_line(Q, R);

  // The point of intersection of bisect_PQ and bisect_M gives the circumcenter
  std::optional<Point> circumcenter = line_line_intersect(bisect_PQ, bisect_QR);

  assert(circumcenter);
  assert(std::isfinite(circumcenter->x()));
  assert(std::isfinite(circumcenter->y()));

  const double eps = 0.0001;
  (void) eps;

  assert(nearly_equal(Point::dot(P - *circumcenter), Point::dot(Q - *circumcenter), eps));
  assert(nearly_equal(Point::dot(P - *circumcenter), Point::dot(R - *circumcenter), eps));
  assert(nearly_equal(Point::dot(Q - *circumcenter), Point::dot(R - *circumcenter), eps));

  return *circumcenter;
}

} // end anonymous namespace

delaunay::Triangle::Triangle(const Triangulate &data, Point_Idx A, Point_Idx B, Point_Idx C) {
  Point ptA = data.lookup_point(A);
  Point ptB = data.lookup_point(B);
  Point ptC = data.lookup_point(C);

  _corners[0] = A;
  _corners[1] = B;
  _corners[2] = C;

  const double winding = (ptB.x() - ptA.x()) * (ptC.y() - ptA.y()) - (ptC.x() - ptA.x()) * (ptB.y() - ptA.y());

  if (winding == 0) {
    /// special case: three points are colinear, i.e. don't actually form a valid triangle
    /// just make a circle that contains all three points on the line

    struct Segment {
      double len_squared;
      Point bgn, end;
    };

    std::array<Segment, 3> segments;

    segments[0] = { Point::dot(ptA - ptB), ptA, ptB };
    segments[1] = { Point::dot(ptA - ptC), ptA, ptC };
    segments[2] = { Point::dot(ptB - ptC), ptB, ptC };

    std::sort(segments.begin(), segments.end(), [](const Segment &a, const Segment &b) { return a.len_squared < b.len_squared; });

    /// longest span
    Segment seg = segments.back();

    _circumcircle_center        = (seg.bgn - seg.end) / 2;
    _circumcircle_square_radius = seg.len_squared / 2;
  } else {
    if (winding < 0) {
      /// triangle is winding clockwise.
      /// make counter-clockwise
      std::swap(ptA, ptC);
      std::swap(_corners[0], _corners[2]);
    }

    _circumcircle_center        = find_circumcenter(ptA, ptB, ptC);
    _circumcircle_square_radius = Point::dot(ptA - _circumcircle_center);
  }
}

delaunay::Triangulate::Triangulate(double min_x, double min_y, double max_x, double max_y)
: _spatial_index{min_x, min_y, max_x, max_y} {

  // add super-triangle to triangulation
  // must be large enough to completely contain all possible points
  const Point_Idx bottom_left  = _add_point({ min_x, min_y });
  const Point_Idx bottom_right = _add_point({ max_x, min_y });
  const Point_Idx top_left     = _add_point({ min_x, max_y });
  const Point_Idx top_right    = _add_point({ max_x, max_y });

  const Triangle A{*this, bottom_left, bottom_right, top_right};
  const Triangle B{*this, top_right, top_left, bottom_left};

  _spatial_index.insert(A);
  _spatial_index.insert(B);
}

delaunay::Triangulate::~Triangulate() {
}

std::pair<bool, Point_Idx> delaunay::Triangulate::step(Point point) {
  assert(point.x() >= min_x());
  assert(point.x() <= max_x());
  assert(point.y() >= min_y());
  assert(point.y() <= max_y());

  auto it = _point_2_idx.find(point);

  if (it != _point_2_idx.end()) {
    return std::make_pair(false, it->second);
  }

  const Point_Idx point_idx = _add_point(point);
  _point_2_idx.emplace_hint(it, point, point_idx);

  // std::cout << "BGN step\n";
  // foreach([](const Triangle &t) { std::cout << "  " << t << "\n"; });

  // std::cout << "WANT edges\n";
  // std::set<Triangle> want_tris;
  // std::map<Edge, unsigned> want_edges;
  // foreach([&](const Triangle &tri) {
  //   if (tri.circumcircle_contains(point)) {
  //     std::cout << "remove linear srch " << tri << "\n";
  //     std::cout << "              edge " << Edge{tri.corners()[0], tri.corners()[1]} << "\n";
  //     std::cout << "              edge " << Edge{tri.corners()[1], tri.corners()[2]} << "\n";
  //     std::cout << "              edge " << Edge{tri.corners()[2], tri.corners()[0]} << "\n";

  //     want_edges[Edge{tri.corners()[0], tri.corners()[1]}]++;
  //     want_edges[Edge{tri.corners()[1], tri.corners()[2]}]++;
  //     want_edges[Edge{tri.corners()[2], tri.corners()[0]}]++;
  //   } else {
  //     want_tris.insert(tri);
  //   }
  // });

  _removed_edges.clear();
  _spatial_index.remove_overlapping(point, _removed_edges);

  // std::set<Triangle> have_tris;
  // foreach([&](const Triangle &tri) {
  //     have_tris.insert(tri);
  // });

  // std::cout << "WANT\n";
  // for (auto p : want_tris) { std::cout << "  " << p << "\n"; }
  // for (auto p : want_edges) { std::cout << "  " << p.first << " : " << p.second << "\n"; }
  // std::cout << "HAVE\n";
  // for (auto p : _removed_edges) { std::cout << "  " << p.first << " : " << p.second << "\n"; }
  // for (auto p : have_tris) { std::cout << "  " << p << "\n"; }
  // assert(want_edges == _removed_edges);
  // assert(want_tris == have_tris);

  // std::cout << "MID step\n";
  // foreach([](const Triangle &t) { std::cout << "  " << t << "\n"; });

  // find and re-triangulate the polygonal hole created by deleting triangles
  for (auto it = _removed_edges.begin(), end = _removed_edges.end(); it != end;) {
    auto curr = it;
    ++it;
    const Edge     edge  = curr->first;
    const unsigned count = curr->second;

    assert(count > 0);

    if (count > 1) {
      /// edges that appear more than once are inside the polygonal hole
    } else {
      /// edges that appear once form the edge of the polygonal hole
      /// retriangulate: form a triangle from edge to inserted point
      _spatial_index.insert(Triangle{
        *this,
        point_idx,
        edge.begin(),
        edge.end(),
      });
    }
  }

  return std::make_pair(true, point_idx);
}

void delaunay::Triangulate::finish() {
  // for (auto it = _triangles.begin(), end = _triangles.end(); it != end;) {
  //   auto curr = it;
  //   ++it;
  //   const Triangle &triangle = *curr;

  //   bool outside = false;

  //   for (Point_Idx corner : triangle.corners()) {
  //     if (uint32_t(corner) < 4) { outside = true; break; }
  //   }

  //   if (outside) {
  //     _remove_triangle(curr);
  //   }
  // }
}

static double clamp(double d, double lo, double hi) {
  return std::max(lo, std::min(hi, d));
}

bool delaunay::Quad_Tree::Node::contains(Point p) const {
  if (p.x() < _min_x) { return false; }
  if (p.x() > _max_x) { return false; }
  if (p.y() < _min_y) { return false; }
  if (p.y() > _max_y) { return false; }
  return true;
}

bool delaunay::Quad_Tree::Node::contains(double min_x, double min_y, double max_x, double max_y) const {
  return contains({min_x, min_y}) && contains({max_x, max_y});
}

bool delaunay::Quad_Tree::Node::insert(const Triangle &tri, double min_x, double min_y, double max_x, double max_y) {
  if (!contains(min_x, min_y, max_x, max_y)) {
    /// does not fit in this, could never fit in children
    return false;
  }

  if (is_leaf()) {
    const double mid_x = (_min_x + _max_x) / 2;
    const double mid_y = (_min_y + _max_y) / 2;

    _children[0] = new Node{_min_x, _min_y, mid_x,  mid_y};
    _children[1] = new Node{mid_x,  _min_y, _max_x, mid_y};
    _children[2] = new Node{_min_x, mid_y,  mid_x,  _max_y};
    _children[3] = new Node{mid_x,  mid_y,  _max_x, _max_y};
  }

  /// first try if triangle fits in a child
  for (Node *child : _children) {
    if (child->insert(tri, min_x, min_y, max_x, max_y)) { return true; }
  }

  /// otherwise we already know it fits in self
  _tris.emplace_back(tri);
  return true;
}

/// like std::remove_if, but also call functor @p erase on every element that gets erased
template<class ForwardIt, class UnaryPredicate, class Erase>
ForwardIt my_remove_if(ForwardIt first, const ForwardIt last, UnaryPredicate &&p, Erase &&erase) {
  const auto base = first;

  /// c++20: use std::find_if
  for (; first != last;) {
    if (p(*first)) {
      break;
    } else {
      ++first;
    }
  }

  if (first != last) {
    assert(p(*first));

    erase(*first);

    for (ForwardIt it = first + 1; it != last; ++it) {
      if (p(*it)) {
        erase(*it);
      } else {
        *first++ = std::move(*it);
      }
    }
  }

  return first;
}

template<typename T, typename Pred, typename Erase>
void my_erase_if(std::vector<T> &vec, Pred &&pred, Erase &&erase) {
  auto it = my_remove_if(vec.begin(), vec.end(), std::forward<Pred>(pred), std::forward<Erase>(erase));

  vec.erase(it, vec.end());
}

bool delaunay::Quad_Tree::Node::remove_overlapping(const Point p, std::map<Edge, unsigned> &edges) {
  if (is_leaf()) { return true; }
  // if (!contains(p)) { return false; }

  bool children_have_nodes = false;

  /// first try if tri is in a child
  for (Node *child : _children) {
    children_have_nodes |= child->remove_overlapping(p, edges);
  }

  my_erase_if(
    _tris,
    // predicate
    [p](const Triangle &tri) { return tri.circumcircle_contains(p); },
    // called on erased triangles
    [&edges, p](const Triangle &tri) {
      assert(tri.circumcircle_contains(p));

      edges[Edge{tri.corners()[0], tri.corners()[1]}]++;
      edges[Edge{tri.corners()[1], tri.corners()[2]}]++;
      edges[Edge{tri.corners()[2], tri.corners()[0]}]++;
    }
  );

  if (!children_have_nodes && _tris.empty()) {
    for (Node *child : _children) { delete child; }
    return true;
  } else {
    return false;
  }
}

void delaunay::Quad_Tree::Node::foreach(const std::function<void(const Triangle&)> &fn) const {
  if (!is_leaf()) {
    for (const Triangle &t : _tris) { fn(t); }

    for (Node *child : _children) {
      child->foreach(fn);
    }
  }
}

void delaunay::Quad_Tree::Node::foreach_edge(const std::function<void(Edge)> &fn) const {
  if (!is_leaf()) {
    for (const Triangle &t : _tris) {
      fn(Edge{t.corners()[0], t.corners()[1]});
      fn(Edge{t.corners()[1], t.corners()[2]});
      fn(Edge{t.corners()[2], t.corners()[0]});
    }

    for (Node *child : _children) {
      child->foreach_edge(fn);
    }
  }
}

bool delaunay::Quad_Tree::Node::is_leaf() const {
  return _children[0] == nullptr;
}

static double next_power_of_two(double n) {
  if (n >= 0) {
    double pow2 = 1;
    while (n < pow2) {
      pow2 *= 2;
    }

    return pow2;
  } else {
    double pow2 = -1;
    while (n > pow2) {
      pow2 *= 2;
    }

    return pow2;
  }
}

delaunay::Quad_Tree::Quad_Tree(double min_x, double min_y, double max_x, double max_y)
// : _root{next_power_of_two(min_x), next_power_of_two(min_y), next_power_of_two(max_x), next_power_of_two(max_y)} {}
: _root{min_x, min_y, max_x, max_y} {}

void delaunay::Quad_Tree::insert(const Triangle &tri) {
  const Point  center = tri.circumcircle_center();
  const double radius = std::ceil(std::sqrt(tri.circumcircle_square_radius()));

  const double tri_min_x = clamp(center.x() - radius, min_x(), max_x());
  const double tri_min_y = clamp(center.y() - radius, min_x(), max_x());
  const double tri_max_x = clamp(center.x() + radius, min_y(), max_y());
  const double tri_max_y = clamp(center.y() + radius, min_y(), max_y());

  assert(_root.contains(tri_min_x, tri_min_y, tri_max_x, tri_max_y));

  const bool was_inserted = _root.insert(tri, tri_min_x, tri_min_y, tri_max_x, tri_max_y);
  (void) was_inserted;
  assert(was_inserted);
}

void delaunay::Quad_Tree::remove_overlapping(Point p, std::map<Edge, unsigned> &edges) {
  _root.remove_overlapping(p, edges);
}

void delaunay::Quad_Tree::foreach(const std::function<void(const Triangle&)> &fn) const {
  _root.foreach(fn);
}

void delaunay::Quad_Tree::foreach_edge(const std::function<void(Edge)> &fn) const {
  _root.foreach_edge(fn);
}
