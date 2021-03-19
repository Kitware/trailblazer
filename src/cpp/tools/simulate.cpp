/* Distributed under the Apache License, Version 2.0.
   See accompanying NOTICE file for details.*/

class PositionVector;
#include <libsumo/Simulation.h>
#include <libsumo/Vehicle.h>

#include <vital/math_constants.h>

#include <vital/range/iota.h>

#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <cmath>

namespace kv = kwiver::vital;
namespace kvr = kwiver::vital::range;

using ss = libsumo::Simulation;
using sv = libsumo::Vehicle;

// ----------------------------------------------------------------------------
struct State
{
  long long time;
  double x, y;
  double vx, vy;
};

using Track = std::map<int, State>;

// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  // Get arguments and load simulation
  auto args = std::vector<std::string>{};
  for (auto const n : kvr::iota(argc - 1))
  {
    args.push_back(argv[n + 1]);
  }
  ss::load(args);

  if (!ss::isLoaded())
  {
    std::cerr << "Failed to load simulation" << std::endl;
    return 1;
  }

  // Run simulation to end
  auto tracks = std::unordered_map<int, Track>{};
  while (ss::getMinExpectedNumber())
  {
    auto const f = ss::getCurrentTime(); // "frame number" (simulation step)
    auto const t = ss::getTime(); // real time (seconds) since simulation start
    auto const tn = static_cast<long long>(1e6 * t); // time in nanoseconds

    for (auto const& vid : sv::getIDList())
    {
      auto const& location = sv::getPosition(vid);
      auto const speed = sv::getSpeed(vid);
      auto const angle = sv::getAngle(vid) * kv::deg_to_rad;
      auto const velx = speed * std::cos(angle);
      auto const vely = speed * std::sin(angle);

      auto const tid = std::stoi(vid.substr(8));
      auto& track = tracks[tid];
      track.emplace(f, State{tn, location.x, location.y, velx, vely});
    }
    ss::step();
  }

  // Shut down simulation
  ss::close();

  // Write tracks
  for (auto const ti : tracks)
  {
    auto const tk = ti.second.size();
    for (auto const si : ti.second)
    {
      auto const& s = si.second;
      std::cout << ti.first << ' ' << tk << ' ' << si.first << ' '
                << s.x << ' ' << s.y << ' ' << s.vx << ' ' << s.vy << ' '
                << s.x << ' ' << s.y << ' '
                << s.x - 1.0 << ' ' << s.y - 1.0 << ' '
                << s.x + 1.0 << ' ' << s.y + 1.0 << ' '
                << "0 0 0 " << s.time << " 1\n";
    }
  }

  return 0;
}