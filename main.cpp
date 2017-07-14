#include <cmath>
#include <iostream>
#include <vector>
#include <SDL2/SDL.h>


using namespace std;


// A classic resolution :)
const int WIDTH = 1024;
const int HEIGHT = 768;

// Timestep
const double dt = 0.5;
// Friction
const double friction = 0.95;


struct Point {
  double x;
  double y;
};
typedef Point Vector; // Makes verlet integration easier to follow


// Parameters for the Lennard-Jones potential
constexpr double epsilon4 = 2.0 * 4.0;
constexpr double sigma6 = pow(32.0, 6.0);

// Potential between a and b
double lj_potential(Point a, Point b) {
  double r2 = (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
  double r6 = pow(r2, 3.0);
  double sbyr6 = sigma6 / r6; // I wonder if reusing r2 here is faster? (avoids allocation?)
  return epsilon4 * (sbyr6*sbyr6 - sbyr6);
}

// Force on a because of potential between a and b
Vector lj_force(Point a, Point b) {
  double r2 = (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
  if (r2 <= 0)
    return Vector {0.0, 0.0};
  double r6 = pow(r2, 3.0);
  double f =  epsilon4 * 6.0 * sigma6 * (r6 - 2.0 * sigma6) / (r6 * r6 * sqrt(r2));
  Vector q {b.x - a.x, b.y - a.y};
  double qmag = sqrt(q.x*q.x + q.y*q.y);
  q.x /= qmag;
  q.y /= qmag;
  q.x *= f;
  q.y *= f;
  return q;
}


int main() {

  // ### Init SDL ### //
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window *window = SDL_CreateWindow("LJ-verlet"
                                        , SDL_WINDOWPOS_UNDEFINED
                                        , SDL_WINDOWPOS_UNDEFINED
                                        , WIDTH
                                        , HEIGHT
                                        , SDL_WINDOW_SHOWN
                                        );
  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);


  // ### System setup ### //
  vector<Point> points;
  points.push_back(Point{WIDTH/2, HEIGHT/2});
  // points.push_back(Point{WIDTH/2 + 30, HEIGHT/2 - 50});
  // points.push_back(Point{WIDTH/2 - 80, HEIGHT/2 + 20});

  vector<Vector> velocities;
  for (size_t i = 0; i < points.size(); ++i) {
    velocities.push_back(Vector{0.0, 0.0});
  }

  // ### Simulation Loop ### //
  SDL_Event event;
  bool run = true;
  int mouse_x = WIDTH/2;
  int mouse_y = HEIGHT/2;
  while (run) {
    // ### Physics simulation ### //
    // though the program is named verlet, sadly this is just an euler integration (for now anyway)
    // also, i probably forgot something. This is not a reference implementation! xD

    vector<Point> old_points = points;
    for (size_t i = 0; i < points.size(); ++i) {
      Point &p = points[i];

      // find acceleration
      Vector a {0.0, 0.0};
      for (size_t j = 0; j < old_points.size(); ++j) {
        if (i == j) continue; // Points do not interact with themselves
        Point &p2 = points[j];
        Vector f = lj_force(p, p2);
        // assume mass = 1
        a.x += f.x;
        a.y += f.y;
      }

      // update velocity
      Vector &v = velocities[i];
      v.x += a.x * dt;
      v.y += a.y * dt;

      // update position
      p.x += v.x * dt + a.x * dt * dt;
      p.y += v.y * dt + a.y * dt * dt;

      v.x *= friction;
      v.y *= friction;
    }

    // ### Drawing and events ### //
    SDL_SetRenderDrawColor(renderer, 0x6, 0x18, 0x20, 0xFF);
    SDL_RenderClear(renderer);

    for (auto &p: points) {
      SDL_SetRenderDrawColor(renderer, 0xAA, 0xAA, 0x3A, 0xFF);
      SDL_Rect c = {static_cast<int>(p.x) - 16,static_cast<int>(p.y) - 16, 32, 32};
      SDL_RenderFillRect(renderer, &c);
    }

    SDL_RenderPresent(renderer);

    while (SDL_PollEvent(&event) != 0) {
      if (event.type == SDL_QUIT) {
        run = false;
      } else if (event.type == SDL_MOUSEMOTION) {
        mouse_x = event.motion.x;
        mouse_y = event.motion.y;
      } else if (event.type == SDL_MOUSEBUTTONDOWN) {
        points.push_back(Point{static_cast<double>(mouse_x), static_cast<double>(mouse_y)});
        velocities.push_back(Vector{0.0, 0.0});
      }
    }
  }


  // ### Cleanup SDL ### //
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}
