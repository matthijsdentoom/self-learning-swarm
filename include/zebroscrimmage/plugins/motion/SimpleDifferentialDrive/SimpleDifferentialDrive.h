/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Matthijs den Toom <m.denToom@student.tudelft.nl>
 * @date 30 November 2018
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */
#ifndef INCLUDE_ZEBROSCRIMMAGE_PLUGINS_MOTION_SIMPLEDIFFERENTIALDRIVE_SIMPLEDIFFERENTIALDRIVE_H_
#define INCLUDE_ZEBROSCRIMMAGE_PLUGINS_MOTION_SIMPLEDIFFERENTIALDRIVE_SIMPLEDIFFERENTIALDRIVE_H_

#include <scrimmage/motion/MotionModel.h>

#include <map>
#include <string>

namespace scrimmage {
namespace motion {
class SimpleDifferentialDrive : public scrimmage::MotionModel {
 public:
    bool init(std::map<std::string, std::string> &info,
              std::map<std::string, std::string> &params) override;
    bool step(double time, double dt) override;
    void model(const vector_t &x , vector_t &dxdt , double t) override;

 protected:

    double width_;          // Width of the robot is taken as the wheel base.
    double wheel_radius_;
    bool enable_gravity_;
    double max_angular_speed_;   // Maximum angular speed of the wheels in rad/s (forward as well as backward)

    // Speed as a percentage of the maximum motor speed [-100 - 100].
    int input_left_sp_idx_ = 0;
    int input_right_sp_idx_ = 0;

 private:
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_ZEBROSCRIMMAGE_PLUGINS_MOTION_SIMPLEDIFFERENTIALDRIVE_SIMPLEDIFFERENTIALDRIVE_H_
