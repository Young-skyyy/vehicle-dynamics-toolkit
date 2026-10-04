// Replay the exact dynamics core used by the ROS2 node.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include "vehicle_dynamics_node/dynamics.hpp"
int main(int argc, char** argv) {
  if (argc != 3) { std::cerr << "usage: dynamics_runner inputs.csv output.csv\n"; return 2; }
  try {
    std::ifstream input(argv[1]); std::ofstream output(argv[2]);
    if (!input || !output) throw std::runtime_error("cannot open CSV");
    std::string line, active_case;
    std::getline(input, line);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != "case,step,dt,initial_vx,throttle,brake,steer") throw std::runtime_error("invalid header");
    output << "case,step,t,vx,ax,vy,ay,yaw_rate,heading,position_x,position_y,gear,engine_rpm,engine_torque\n";
    output << std::setprecision(17);
    std::unique_ptr<vehicle_dynamics::Model> model;
    int expected_step = 1, count = 0;
    while (std::getline(input, line)) {
      if (line.empty()) continue;
      std::stringstream row(line); std::vector<std::string> columns; std::string token;
      while (std::getline(row, token, ',')) columns.push_back(token);
      if (columns.size() != 7) throw std::runtime_error("invalid row");
      const auto& name = columns[0]; const int step = std::stoi(columns[1]);
      if (name != active_case) {
        model = std::make_unique<vehicle_dynamics::Model>(vehicle_dynamics::Parameters{}, std::stod(columns[3]));
        active_case = name; expected_step = 1;
      }
      if (step != expected_step++) throw std::runtime_error("non-consecutive step");
      const auto& s = model->step(std::stod(columns[2]), std::stod(columns[4]), std::stod(columns[5]), std::stod(columns[6]));
      output << name << ',' << step << ',' << s.t << ',' << s.vx << ',' << s.ax << ','
        << s.vy << ',' << s.ay << ',' << s.yaw_rate << ',' << s.heading << ','
        << s.position_x << ',' << s.position_y << ',' << s.gear << ','
        << s.engine_rpm << ',' << s.engine_torque << '\n';
      ++count;
    }
    if (!count) throw std::runtime_error("empty replay");
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
