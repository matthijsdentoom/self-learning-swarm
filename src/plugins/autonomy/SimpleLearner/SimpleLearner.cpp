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

#include <zebroscrimmage/plugins/autonomy/SimpleLearner/SimpleLearner.h>

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
                scrimmage::autonomy::SimpleLearner,
                SimpleLearner_plugin)

namespace scrimmage {
namespace autonomy {

    void SimpleLearner::init_helper(std::map<std::string, std::string> &params)
    {
        using Type = scrimmage::VariableIO::Type;
        using Dir = scrimmage::VariableIO::Direction;

        output_vel_x_idx_ = vars_.declare(Type::velocity_x, Dir::Out);
        const uint8_t output_vel_y_idx = vars_.declare(Type::velocity_y, Dir::Out);
        const uint8_t output_vel_z_idx = vars_.declare(Type::velocity_z, Dir::Out);

        vars_.output(output_vel_x_idx_, 0);
        vars_.output(output_vel_y_idx, 0);
        vars_.output(output_vel_z_idx, 0);

        radius_ = std::stod(params.at("radius"));
    }

    bool SimpleLearner::step_helper()
    {
        const double x_vel = action.discrete[0] ? 1 : -1;
        vars_.output(output_vel_x_idx_, x_vel);
    }

    void SimpleLearner::set_environment()
    {
        reward_range = std::make_pair(0, 1);
        action_space.discrete_count.push_back(2);
    }

    std::tuple<bool, double, pybind11::dict> SimpleLearner::calc_reward()
    {
        const bool done = false;
        const double x = state_->pos()(0);
        const bool within_radius = std::round(std::abs(x)) < radius_;
        double reward = within_radius ? 1 : 0;

        // here we setup the debugging info.
        pybind11::dict info;
        info["x_within_radius"] = within_radius; // an example of adding debugging information
        return std::make_tuple(done, reward, info);
    }
} // namespace autonomy
} // namespace scrimmage
