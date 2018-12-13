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

#include <zebroscrimmage/plugins/autonomy/ZebroStraight/ZebroStraight.h>

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
                scrimmage::autonomy::ZebroStraight,
                ZebroStraight_plugin)

namespace scrimmage {
namespace autonomy {

void ZebroStraight::init(std::map<std::string, std::string> &params) {
    initial_speed_ = sc::get<double>("initial_speed", params, initial_speed_);

    desired_left_speed_idx_ = vars_.declare("desired_left_speed", scrimmage::VariableIO::Direction::Out);
    desired_right_speed_idx_ = vars_.declare("desired_right_speed", scrimmage::VariableIO::Direction::Out);
}

bool ZebroStraight::step_autonomy(double t, double dt) {


    vars_.output(desired_left_speed_idx_, initial_speed_);
    vars_.output(desired_right_speed_idx_, initial_speed_);

    return true;
}
} // namespace autonomy
} // namespace scrimmage
