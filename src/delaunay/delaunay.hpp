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

/// Compute Delaunay triangulation for a set of points

#include <algorithm>  // for std::find
#include <cassert>    // for assert
#include <cstdint>    // for uint16_t
#include <map>        // for std::map
#include <set>        // for std::set
#include <tuple>      // for std::tie
#include <utility>    // for std::swap
#include <vector>     // for std::vector
#include <functional> // for std::function

namespace delaunay {

struct Triangulate;

struct Point final {
  Point() = default;
  Point(double x, double y) : _x{x}, _y{y} {}

  double x() const { return _x; }
  double y() const { return _y; }

  bool operator==(Point that) const { return (_x == that._x) && (_y == that._y); }
  bool operator!=(Point that) const { return !(*this == that); }

  Point operator-(Point that) const {
    return Point{x() - that.x(), y() - that.y()};
  }

  Point operator/(double n) const {
    return Point{x()  / n, y()  / n};
  }

  /// interpret a and b as vectors and compute their dot product
  static double dot(Point a, Point b) {
    return (a.x() * b.x()) + (a.y() * b.y());
  }

  static double dot(Point p) {
    return Point::dot(p, p);
  }

  bool operator<(Point that) const {
    return std::tie(_x, _y) < std::tie(that._x, that._y);
  }
private:
  double _x, _y;
};

enum class Point_Idx : uint32_t {};

struct Edge final {
  Edge(Point_Idx begin, Point_Idx end) {
    if (end < begin) { std::swap(begin, end); }
    _points[0] = begin;
    _points[1] = end;
  }

  Point_Idx begin() const { return _points[0]; }
  Point_Idx end() const { return _points[1]; }

  const std::array<Point_Idx, 2> &points() const { return _points; }

  bool operator==(Edge that) const {
    return (begin() == that.begin()) && (end() == that.end());
  }

  bool operator<(Edge that) const {
    return _points < that._points;
  }
private:
  std::array<Point_Idx, 2> _points;
};

struct Triangle final {
  Triangle(const Triangulate &data, Point_Idx A, Point_Idx B, Point_Idx C);

  const std::array<Point_Idx, 3> &corners() const { return _corners; }

  bool circumcircle_contains(Point p) const {
    const double square_dist = Point::dot(p - circumcircle_center());
    return square_dist <= circumcircle_square_radius();
  }

  Point circumcircle_center() const { return _circumcircle_center; }
  double circumcircle_square_radius() const { return _circumcircle_square_radius; }

  bool operator==(const Triangle &that) const {
    return std::equal(_corners.begin(), _corners.end(), that._corners.begin());
  }

  bool operator<(const Triangle &that) const {
    return corners() < that.corners();
  }
private:
  std::array<Point_Idx, 3> _corners;

  Point _circumcircle_center;
  double _circumcircle_square_radius;
};

/// Very simple region-quad tree to speed up triangulation.
/// Used to find which triangle-circumcircles contain a point.
struct Quad_Tree {
  Quad_Tree(double min_x, double min_y, double max_x, double max_y);

  void insert(const Triangle &tri);

  void remove_overlapping(Point p, std::map<Edge, unsigned> &edges);

  void foreach(const std::function<void(const Triangle&)> &fn) const;
  void foreach_edge(const std::function<void(Edge)> &fn) const;

  const double min_x() const { return _root._min_x; }
  const double max_x() const { return _root._max_x; }
  const double min_y() const { return _root._min_y; }
  const double max_y() const { return _root._max_y; }
private:
  struct Node {
    Node(double min_x, double min_y, double max_x, double max_y)
    : _min_x{min_x}, _min_y{min_y}, _max_x{max_x}, _max_y{max_y} {
      _children.fill(nullptr);
    }

    /// check if node's bounding box contains @p p
    bool contains(Point p) const;
    /// true iff this node's bounding box 'fully' contains the given bounding box
    bool contains(double min_x, double min_y, double max_x, double max_y) const;

    bool insert(const Triangle &tri, double min_x, double min_y, double max_x, double max_y);
    bool remove_overlapping(const Point p, std::map<Edge, unsigned> &edges);

    void foreach(const std::function<void(const Triangle&)> &fn) const;
    void foreach_edge(const std::function<void(Edge)> &fn) const;

    bool is_leaf() const;

    double _min_x, _min_y, _max_x, _max_y;
    std::array<Node*, 4> _children;
    std::vector<Triangle> _tris;
  };

  Node _root;
};

struct Triangulate {
  Triangulate(double min_x, double min_y, double max_x, double max_y);

  ~Triangulate();

  /// @name delaunay triangulation
  /// @{

  std::pair<bool, Point_Idx> step(Point point);

  void finish();

  /// @}

  /// @name access intermediate state
  /// @{

  const std::vector<Point> &points() const { return _points; }

  // const std::set<Triangle> &triangulation() const { return _triangles; }

  template<typename Fn>
  void foreach(Fn &&fn) {
    _spatial_index.foreach(std::forward<Fn>(fn));
  }

  template<typename Fn>
  void foreach_edge(Fn &&fn) {
    _spatial_index.foreach([&fn](const Triangle &tri) {
      fn(Edge{tri.corners()[0], tri.corners()[1]});
      fn(Edge{tri.corners()[1], tri.corners()[2]});
      fn(Edge{tri.corners()[2], tri.corners()[0]});
    });
  }

  const std::map<Edge, unsigned> &removed_edges() const { return _removed_edges; }

  Point lookup_point(Point_Idx index) const {
    uint32_t i = uint32_t(index);

    assert(i < _points.size());
    return _points[i];
  }

  bool on_edge(Point p) const {
    if (p.x() <= min_x()) { return true; }
    if (p.x() >= max_x()) { return true; }

    if (p.y() <= min_y()) { return true; }
    if (p.y() >= max_y()) { return true; }

    return false;
  }

  bool on_edge(Point_Idx p) const {
    return on_edge(lookup_point(p));
  }

  const double min_x() const { return _spatial_index.min_x(); }
  const double max_x() const { return _spatial_index.max_x(); }
  const double min_y() const { return _spatial_index.min_y(); }
  const double max_y() const { return _spatial_index.max_y(); }

  /// @}
private:
  Point_Idx _add_point(Point p) {
    const Point_Idx idx{(uint32_t) _points.size()};
    _points.emplace_back(p);
    return idx;
  }

  Triangulate(const Triangulate&) = delete;
  Triangulate(Triangulate&&) = delete;

  std::map<Edge, unsigned> _removed_edges;

  Quad_Tree _spatial_index;

  std::vector<Point> _points;
  std::map<Point, Point_Idx> _point_2_idx;
};

} // end namespace s2c
