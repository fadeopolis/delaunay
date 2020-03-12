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

#include <delaunay/delaunay.hpp>

#include <SDL2/SDL.h>

#include <algorithm>
#include <cassert>
#include <charconv> // for std::from_chars
#include <cstdio>
#include <memory>  // for std::unique_ptr
#include <random>
#include <set>
#include <variant>

#define SDL_CHECK_CALL(CALL) do {\
  if ((CALL) != 0) {\
    fprintf(stderr, "error:%s:%u: " #CALL " failed: %s", __FILE__, __LINE__, SDL_GetError());\
    exit(1);\
  }\
} while (0)

namespace {

struct Resolution {
  int width, height;
};

struct Argument_List {
  Argument_List(const char *default_argv0, int argc, const char **argv) : _argv0{default_argv0}, _argc{argc}, _argv{argv} {
    if ((_argc > 0) && _argv[0][0]) {
      _argv0 = _argv[0];
      pop_front();
    }
  }

  const char *argv0() const { return _argv0; }

  bool empty() const { return _argc <= 0; }

  void pop_front() {
    if (empty()) { return; }

    _argc--;
    _argv++;
  }

  std::string_view front() const {
    assert(!empty());

    return _argv[0];
  }

  std::optional<std::string_view> one_after_front() const {
    if (_argc < 2) { return std::nullopt; }

    return _argv[1];
  }
private:
  const char *_argv0;
  int _argc;
  const char **_argv;
};

/// TODO: Use C++20 std::string_view::starts_with
static bool starts_with(std::string_view haystack, std::string_view needle) {
  const size_t len = std::min<size_t>(haystack.size(), needle.size());

  return haystack.compare(0, len, needle) == 0;
}

struct Parse_Error {
  explicit Parse_Error(std::string_view msg) : _message{msg} {}

  Parse_Error operator+(std::string_view txt) const {
    Parse_Error out{_message};
    out._message.append(txt);
    return out;
  }

  std::string_view message() const { return _message; }

  const char *c_str() const { return _message.c_str(); }
private:
  std::string _message;
};

template<typename T>
struct Parse_Result {
  static_assert(!std::is_same_v<std::decay_t<T>, Parse_Error>, "Parse_Result::value_type cannot be Parse_Error");

  Parse_Result(const T &value) : _data{std::in_place_type<T>, value} {}
  Parse_Result(std::nullopt_t) : _data{std::in_place_type<std::nullopt_t>, std::nullopt} {}
  Parse_Result(const Parse_Error &err) : _data{std::in_place_type<Parse_Error>, err} {}

  bool has_value() const {
    return std::holds_alternative<T>(_data);
  }

  const T &get_value() const {
    return std::get<T>(_data);
  }

  bool has_error() const {
    return std::holds_alternative<Parse_Error>(_data);
  }

  const Parse_Error &get_error() const {
    return std::get<Parse_Error>(_data);
  }
private:
  std::variant<T, std::nullopt_t, Parse_Error> _data;
};


Parse_Result<bool> parse_boolean_option(std::string_view short_name, std::string_view long_name, Argument_List &args) {
  if (args.empty()) {
    /// end of argument list
    return std::nullopt;
  }

  const std::string_view arg_str = args.front();

  if (starts_with(arg_str, short_name)) {
    const std::string_view arg_rest = arg_str.substr(short_name.size());

    if (arg_rest.empty()) {
      /// match '-o'
      args.pop_front();
      return true;
    } else {
      /// junk at the end of option
      return Parse_Error{"junk at the end of option '"} + short_name + "'";
    }
  }

  if (starts_with(arg_str, long_name)) {
    const std::string_view arg_rest = arg_str.substr(long_name.size());

    if (arg_rest.empty()) {
      /// match '--opt'
      args.pop_front();
      return true;
    } else {
      /// junk at the end of option
      return Parse_Error{"junk at the end of option '"} + long_name + "'";
    }
  }

  return std::nullopt;
}

Parse_Result<std::string_view> parse_option_with_argument(std::string_view short_name, std::string_view long_name, Argument_List &args) {
  if (args.empty()) {
    /// end of argument list
    return std::nullopt;
  }

  const std::string_view arg_str = args.front();

  if (starts_with(arg_str, short_name)) {
    const std::string_view arg_rest = arg_str.substr(short_name.size());

    if (arg_rest.empty()) {
      std::optional<std::string_view> arg_val = args.one_after_front();

      if (!arg_val) {
        return Parse_Error{"missing argument for option '"} + short_name + "'";
      }

      /// match '-o' 'arg'
      args.pop_front();
      args.pop_front();
      return *arg_val;
    } else {
      /// math '-oarg'
      args.pop_front();
      return arg_rest;
    }
  }

  if (starts_with(arg_str, long_name)) {
    const std::string_view arg_rest = arg_str.substr(long_name.size());

    if (arg_rest.empty()) {
      std::optional<std::string_view> arg_val = args.one_after_front();

      if (!arg_val) {
        return Parse_Error{"missing argument for option '"} + long_name + "'";
      }

      /// match '--opt' 'arg'
      args.pop_front();
      args.pop_front();
      return *arg_val;
    } else if (arg_rest.front() == '=') {
      /// match '--opt=arg'
      args.pop_front();
      return arg_rest.substr(1);
    } else {
      /// junk at the end of option
      return Parse_Error{"junk at the end of option '"} + long_name + "'";
    }
  }

  return std::nullopt;
}

void usage(const char *argv0, FILE *file = stdout) {
  if (!argv0 || !argv0[0]) { argv0 = "delaunay-viewer"; }

  fprintf(file, "usage: %s\n", argv0);
  fprintf(file, "options:\n");
  fprintf(file, "  -f --fullscreen                open fullscreen window (default: false)\n");
  fprintf(file, "  -r --resolution=WIDTHxHEIGHT   set window resolution\n");
  fprintf(file, "  -s --random_seed=SEED          set random seed for point generator\n");
}

void parse_boolean_option_or_bail(std::string_view short_name, std::string_view long_name, Argument_List &args, bool &dst) {
  Parse_Result<bool> ret = parse_boolean_option(short_name, long_name, args);

  if (ret.has_value()) {
    dst = ret.get_value();
  } else if (ret.has_error()) {
    fprintf(stderr, "error: %s\n", ret.get_error().c_str());
    exit(1);
  }
}

template<typename T>
void parse_option_with_argument_or_bail(std::string_view short_name, std::string_view long_name, Argument_List &args,
                                        std::optional<T> &dst, std::optional<T>(*parser)(std::string_view)) {
  Parse_Result<std::string_view> ret = parse_option_with_argument(short_name, long_name, args);

  if (ret.has_value()) {
    const std::string_view txt = ret.get_value();
    dst = parser(txt);
  } else if (ret.has_error()) {
    fprintf(stderr, "error: %s\n", ret.get_error().c_str());
    exit(1);
  }
}

std::optional<Resolution> parse_resolution(std::string_view txt) {
  const size_t x_pos = txt.find('x');

  if (x_pos >= txt.size()) {
    return std::nullopt;
  }

  const std::string_view txt_width  = txt.substr(0, x_pos);
  const std::string_view txt_heigth = txt.substr(x_pos + 1);

  int width;
  if (auto [ptr, ec] = std::from_chars(txt_width.data(), txt_width.data() + txt_width.size(), width); ec == std::errc()) {
    return std::nullopt;
  }

  int height;
  if (auto [ptr, ec] = std::from_chars(txt_heigth.data(), txt_heigth.data() + txt_heigth.size(), height); ec == std::errc()) {
    return std::nullopt;
  }

  if (width  <= 0) { return std::nullopt; }
  if (height <= 0) { return std::nullopt; }

  return Resolution{width, height};
}

std::optional<unsigned> parse_random_seed(std::string_view txt) {
  unsigned seed;
  if (auto [ptr, ec] = std::from_chars(txt.data(), txt.data() + txt.size(), seed); bool(ec) || (ptr != txt.end())) {
    return std::nullopt;
  }
  return seed;
}

static void render_point(SDL_Renderer *renderer, float x, float y, float point_size = 1) {
  if (point_size == 1) {
    SDL_CHECK_CALL(SDL_RenderDrawPointF(renderer, x, y));
  } else {
    SDL_FRect rect;
    /// top left corner
    const float half = point_size / 2;
    rect.x = x - half;
    rect.y = y - half;
    rect.w = point_size;
    rect.h = point_size;
    SDL_CHECK_CALL(SDL_RenderFillRectF(renderer, &rect));
  }
}

} // end anonymous namespace

int main(int argc, const char * argv[]) {
  std::optional<Resolution> desired_resolution;
  std::optional<unsigned> random_seed;
  bool fullscreen = false;

  {
    Argument_List args{"delaunay-viewer", argc, argv};

    while (!args.empty()) {
      bool help = false;
      parse_boolean_option_or_bail("-h", "--help", args, help);
      if (help) {
        usage(args.argv0());
        return 0;
      }

      parse_boolean_option_or_bail("-f", "--fullscreen", args, fullscreen);
      parse_option_with_argument_or_bail("-r", "--resolution", args, desired_resolution, &parse_resolution);
      parse_option_with_argument_or_bail("-s", "--random_seed", args, random_seed, &parse_random_seed);
    }
  }

  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    fprintf(stderr, "error: Could not initialize SDL.\n");
    fprintf(stderr, "  SDL error: %s\n", SDL_GetError());
    return 1;
  }

  const Resolution screen_resolution = [&]() {
    if (desired_resolution) {
      /// use user-provided WIDTHxHEIGHT
      return *desired_resolution;
    }

    /// detect screen WIDTHxHEIGHT

    const int num_displays = ({
      const int num = SDL_GetNumVideoDisplays();
      if (num < 1) {
        fprintf(stderr, "error: SDL could not detect any connected monitors.\n");
        fprintf(stderr, "  SDL error: %s\n", SDL_GetError());
          exit(1);
      }
      num;
    });

    for (int display_index = 0; display_index < num_displays; display_index++) {
      const int num_modes = SDL_GetNumDisplayModes(display_index);
      if (num_modes < 1) {
        /// FIXME: bail out with error message?
        continue;
      }

      for (int mode_index = 0; mode_index < num_modes; mode_index++) {
        SDL_DisplayMode mode;
        SDL_memset(&mode, 0, sizeof(mode));

        if (SDL_GetDisplayMode(display_index, mode_index, &mode) != 0) {
          fprintf(stderr, "error: SDL_GetDisplayMode failed: %s", SDL_GetError());
          exit(1);
        }

        if (!fullscreen) {
          mode.w /= 2;
          mode.h /= 2;
        }

        return Resolution{mode.w, mode.h};
      }
    }

    /// no valid mode found, fallback to something reasonable
    return Resolution{800, 600};
  }();

  const int sdl_window_flags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | (fullscreen ? SDL_WINDOW_FULLSCREEN : 0);

  /// enable anti-aliasing if possible
  SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "2");

  SDL_Window *const window = SDL_CreateWindow("Delaunay triangulation viewer",
                                              SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                              screen_resolution.width, screen_resolution.height,
                                              sdl_window_flags);
  if (!window) {
    fprintf(stderr, "error: Could not open a window.\n");
    fprintf(stderr, "  SDL error: %s\n", SDL_GetError());
    exit(1);
  }

  SDL_Renderer *const renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);
  if (!renderer) {
    fprintf(stderr, "error: Could not create SDL renderer.\n");
    fprintf(stderr, "  SDL error: %s\n", SDL_GetError());
    exit(1);
  }

  if (SDL_RenderSetLogicalSize(renderer, screen_resolution.width, screen_resolution.height) != 0) {
    fprintf(stderr, "error: Could not set logical render size.\n");
    fprintf(stderr, "  SDL error: %s\n", SDL_GetError());
    exit(1);
  }

  bool keep_running = true;

  const int x_offset = screen_resolution.width / 25;
  const int y_offset = screen_resolution.height / 25;

  const float min_x = x_offset;
  const float max_x = screen_resolution.width - x_offset;
  const float min_y = y_offset;
  const float max_y = screen_resolution.height - y_offset;

  {
    std::random_device rd;
    std::mt19937_64 gen;
  }

  // used to obtain a seed for the random number engine if user does not provide one
  std::mt19937 gen;

  if (random_seed) {
    gen.seed(*random_seed);
  } else {
    std::random_device rd;
    gen.seed(rd());
  }

  std::uniform_real_distribution<float> disX{min_x, max_x};
  std::uniform_real_distribution<float> disY{min_y, max_y};

  delaunay::Triangulate triangulate{min_x, min_y, max_x, max_y};

  std::set<delaunay::Point> points_todo;
  std::set<delaunay::Point> points_done;
  std::set<delaunay::Edge> triangulation;
  std::set<delaunay::Edge> triangulation_removed;
  std::set<delaunay::Edge> triangulation_polygon_hole;

  std::optional<delaunay::Point> point_highlight;

  enum State {
    ST_INITIAL,
    ST_SHOW_REMOVED,
    ST_SHOW_REMOVED_POLYGON,
    ST_SHOW_ADDED,
  } state = ST_INITIAL;

  auto update_model = [&]() {
    switch (state) {
      case ST_INITIAL: {
        /// add new point to triangulation
        const float x = disX(gen), y = disY(  gen);

        const delaunay::Point p = {x, y};

        triangulate.step(p);

        /// update data structures for viewing the triangulation

        points_todo.erase(p);
        points_done.insert(p);

        triangulation_removed.clear();
        triangulation_polygon_hole.clear();

        for (const auto &pair : triangulate.removed_edges()) {
          const delaunay::Edge edge = pair.first;

          const size_t num_erased = triangulation.erase(edge);
          assert(num_erased);
          triangulation_removed.insert(edge);
        }

        state = ST_SHOW_REMOVED;
        break;
      }
      case ST_SHOW_REMOVED:
        triangulation_removed.clear();
        triangulation_polygon_hole.clear();
        for (const auto &pair : triangulate.removed_edges()) {
          const delaunay::Edge edge  = pair.first;
          const unsigned       count = pair.second;

          if (count == 1) {
            triangulation_polygon_hole.insert(edge);
          }
        }

        state = ST_SHOW_REMOVED_POLYGON;
        break;
      case ST_SHOW_REMOVED_POLYGON:
        triangulation.clear();
        triangulate.foreach_edge([&](delaunay::Edge edge) {
          triangulation.insert(edge);
        });

        state = ST_SHOW_ADDED;
        break;
      case ST_SHOW_ADDED:
        triangulation_polygon_hole.clear();

        state = ST_INITIAL;
        break;
    }
  };

  triangulate.foreach_edge([&](delaunay::Edge edge) {
    triangulation.insert(edge);
  });
  for (delaunay::Point p : triangulate.points()) {
    points_todo.insert(p);
  }

  while (keep_running) {
    SDL_Event event;
    while(SDL_PollEvent(&event)) {
      switch(event.type) {
        case SDL_QUIT:
          keep_running = false;
          break;
        case SDL_KEYDOWN:
          switch (event.key.keysym.sym) {
            case SDLK_ESCAPE:
              keep_running = false;
              break;
            case SDLK_RETURN:
              update_model();
              break;
            case SDLK_0: case SDLK_1: case SDLK_2: case SDLK_3: case SDLK_4:
            case SDLK_5: case SDLK_6: case SDLK_7: case SDLK_8: case SDLK_9:
              {
                const int idx = event.key.keysym.sym - '0';
                if (idx < triangulate.points().size()) {
                  point_highlight = triangulate.points()[idx];
                }
              }
              break;
          }
          break;
      }
    }

    SDL_CHECK_CALL(SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE));
    SDL_CHECK_CALL(SDL_RenderClear(renderer));

    if (point_highlight) {
      SDL_CHECK_CALL(SDL_SetRenderDrawColor(renderer, 0, 255, 255, SDL_ALPHA_OPAQUE));
      render_point(renderer, point_highlight->x(), point_highlight->y(), 10);
    }

    // GL_CHECK_CALL(glLineWidth(1));
    for (const delaunay::Edge edge : triangulation) {
      bool outside = false;

      for (delaunay::Point_Idx point : edge.points()) {
        if (triangulate.on_edge(point)) { outside = true; break; }
      }

      const delaunay::Point bgn = triangulate.lookup_point(edge.begin());
      const delaunay::Point end = triangulate.lookup_point(edge.end());

      if (outside) {
        SDL_CHECK_CALL(SDL_SetRenderDrawColor(renderer, 0, 0, 255, SDL_ALPHA_OPAQUE));
      } else {
        SDL_CHECK_CALL(SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE));
      }

      SDL_CHECK_CALL(SDL_RenderDrawLine(renderer, bgn.x(), bgn.y(), end.x(), end.y()));
    }

    SDL_SetRenderDrawColor(renderer, 255, 255, 0, SDL_ALPHA_OPAQUE);
    // GL_CHECK_CALL(glLineWidth(3));
    for (const delaunay::Edge edge : triangulation_removed) {
      const delaunay::Point bgn = triangulate.lookup_point(edge.begin());
      const delaunay::Point end = triangulate.lookup_point(edge.end());

      SDL_CHECK_CALL(SDL_RenderDrawLine(renderer, bgn.x(), bgn.y(), end.x(), end.y()));
    }

    SDL_SetRenderDrawColor(renderer, 255, 0, 255, SDL_ALPHA_OPAQUE);
    // GL_CHECK_CALL(glLineWidth(3));
    for (const delaunay::Edge edge : triangulation_polygon_hole) {
      const delaunay::Point bgn = triangulate.lookup_point(edge.begin());
      const delaunay::Point end = triangulate.lookup_point(edge.end());

      SDL_CHECK_CALL(SDL_RenderDrawLine(renderer, bgn.x(), bgn.y(), end.x(), end.y()));
    }

    for (delaunay::Point p : points_todo) {
      SDL_CHECK_CALL(SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE));
      render_point(renderer, p.x(), p.y(), 5);
    }

    for (delaunay::Point p : points_done) {
      SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);
      render_point(renderer, p.x(), p.y(), 5);
    }

    // SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
    // SDL_RenderDrawLine(renderer, 320, 200, 300, 240);
    // SDL_RenderDrawLine(renderer, 300, 240, 340, 240);
    // SDL_RenderDrawLine(renderer, 340, 240, 320, 200);

    SDL_RenderPresent(renderer);
    SDL_Delay(1);
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;

}
