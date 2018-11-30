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

#include <zebroscrimmage/plugins/controller/SimpleDifferentialDriveController/SimpleDifferentialDriveController.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::SimpleDifferentialDriveController,
                SimpleDifferentialDriveController_plugin)

namespace scrimmage {
namespace controller {

void SimpleDifferentialDriveController::init(std::map<std::string, std::string> &params) {

    input_speed_l_idx_ = vars_.declare("desired_left_speed", VariableIO::Direction::In);
    input_speed_r_idx_ = vars_.declare("desired_right_speed", VariableIO::Direction::In);

    output_speed_l_idx_ = vars_.declare("left_speed", VariableIO::Direction::Out);
    output_speed_r_idx_ = vars_.declare("right_speed", VariableIO::Direction::Out);
}

bool SimpleDifferentialDriveController::step(double /* t */, double /* dt */) {
    vars_.output(output_speed_l_idx_, vars_.input(input_speed_l_idx_));
    vars_.output(output_speed_r_idx_, vars_.input(input_speed_r_idx_));

    return true;
}
} // namespace controller
} // namespace scrimmage
