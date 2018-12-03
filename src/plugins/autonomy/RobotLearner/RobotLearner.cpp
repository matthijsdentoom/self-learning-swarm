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

#include <zebroscrimmage/plugins/autonomy/RobotLearner/RobotLearner.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::RobotLearner,
                RobotLearner_plugin)

namespace scrimmage {
namespace autonomy {

    void RobotLearner::init_helper(std::map<std::string, std::string> &params)
    {
        using Dir = scrimmage::VariableIO::Direction;

        output_ang_vel_left = vars_.declare("desired_left_speed", Dir::Out);
        output_ang_vel_right = vars_.declare("desired_right_speed", Dir::Out);

        max_angular_speed_ = std::stod(params.at("max_angular_speed"));

        vars_.output(output_ang_vel_left, 0);
        vars_.output(output_ang_vel_right, 0);
    }

    bool RobotLearner::step_helper()
    {
        const double left_angular_vel = action.continuous[0];
        const double right_angular_vel = action.continuous[1];

        vars_.output(output_ang_vel_left, left_angular_vel);
        vars_.output(output_ang_vel_right, right_angular_vel);
        return true;
    }

    void RobotLearner::set_environment()
    {
        reward_range = std::make_pair(0, 1);
        action_space.continuous_extrema.push_back(std::make_pair(-max_angular_speed_, max_angular_speed_));
        action_space.continuous_extrema.push_back(std::make_pair(-max_angular_speed_, max_angular_speed_));
    }

    std::tuple<bool, double, pybind11::dict> RobotLearner::calc_reward()
    {
        const bool done = false;
        double reward = 0.0;

        // here we setup the debugging info.
        pybind11::dict info;
        info["x_within_radius"] = 0; // an example of adding debugging information
        return std::make_tuple(done, reward, info);
    }
} // namespace autonomy
} // namespace scrimmage
