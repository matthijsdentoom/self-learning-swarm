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
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_ZEBROSCRIMMAGE_PLUGINS_AUTONOMY_ROBOTLEARNER_ROBOTLEARNER_H_
#define INCLUDE_ZEBROSCRIMMAGE_PLUGINS_AUTONOMY_ROBOTLEARNER_ROBOTLEARNER_H_

#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ScrimmageOpenAIAutonomy.h>

#include <string>
#include <map>

namespace scrimmage {
namespace autonomy {
class RobotLearner : public scrimmage::autonomy::ScrimmageOpenAIAutonomy {
 public:

    void init_helper(std::map<std::string, std::string> &params) override;
    bool step_helper() override;

    void set_environment() override;
    std::tuple<bool, double, pybind11::dict> calc_reward() override;


 protected:
    uint8_t output_ang_vel_left = 0;
    uint8_t output_ang_vel_right = 0;

    double max_angular_speed_;      // The maximal angular speed of one of the wheels.
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_ZEBROSCRIMMAGE_PLUGINS_AUTONOMY_ROBOTLEARNER_ROBOTLEARNER_H_
