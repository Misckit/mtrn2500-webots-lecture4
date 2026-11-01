#pragma once

#include "EPuckGreenRobot.hpp"

#include <webots/GPS.hpp>
#include <webots/Display.hpp>

#include <utility>
const double WORLD_WIDTH{1};
const double WORLD_HEIGHT{1};

class EPuckGreenPlusRobot : public EPuckGreenRobot {
public:
  EPuckGreenPlusRobot();
  void run();
  
protected:
  void readGPS();
  void displayTrajectory() const;
  
private:
  webots::GPS* mGPS {};
  std::pair<double, double> mPosition{};
  
  webots::Display* mDisplay1{};
  int mDP1Width {};
  int mDP1Height {};  
};