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

#include <zebroscrimmage/plugins/motion/SimpleDifferentialDrive/SimpleDifferentialDrive.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>

#include <boost/algorithm/clamp.hpp>

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::MotionModel,
                scrimmage::motion::SimpleDifferentialDrive,
                SimpleDifferentialDrive_plugin)

using boost::algorithm::clamp;

namespace scrimmage {
namespace motion {

enum ModelParams {
    X = 0,
    Y,
    Z,
    Z_dot,
    THETA,
    MODEL_NUM_ITEMS
};

bool SimpleDifferentialDrive::init(std::map<std::string, std::string> &info,
                     std::map<std::string, std::string> &params) {
    x_.resize(MODEL_NUM_ITEMS);
    x_[X] = std::stod(info["x"]);
    x_[Y] = std::stod(info["y"]);
    x_[Z] = std::stod(info["z"]);
    x_[Z_dot] = 0;
    x_[THETA] = Angles::deg2rad(std::stod(info["heading"]));

    mass_ = get<double>("mass", params, 1.0);
    enable_gravity_ = get<bool>("enable_gravity", params, false);
    max_angular_speed_ = get<double>("max_angular_speed", params, 5.0);
    width_ = get<double>("width", params, 50.0);
    wheel_radius_ = get<double>("wheel_radius", params, 1.0);

    /////////
    state_->vel() << 0, 0, 0;
    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(0, 0, x_[THETA]);

    input_left_sp_idx_ = vars_.declare("left_speed", VariableIO::Direction::In);
    input_right_sp_idx_ = vars_.declare("right_speed", VariableIO::Direction::In);

    return true;
}

bool SimpleDifferentialDrive::step(double time, double dt) {

    double prev_x = x_[X];
    double prev_y = x_[Y];
    double prev_z = x_[Z];

    ode_step(dt);

    ext_force_ = Eigen::Vector3d::Zero();

    /////////////////////
    // Save state
    // Simple velocity
    state_->vel() << (x_[X] - prev_x) / dt, (x_[Y] - prev_y) / dt,
            (x_[Z] - prev_z) / dt;

    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(0, 0, x_[THETA]);
    return true;

}

void SimpleDifferentialDrive::model(const vector_t &x , vector_t &dxdt , double t) {

    /// 0 : x-position
    /// 1 : y-position
    /// 2 : theta

    // Formulas according to: http://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf

    // Angular speeds in rad/s
    const double left_angular_speed = clamp(vars_.input(input_left_sp_idx_), -max_angular_speed_, max_angular_speed_);
    const double right_angular_speed = clamp(vars_.input(input_right_sp_idx_), -max_angular_speed_, max_angular_speed_);

        // Calculate the speed of the middle of the car.
    const double left_speed = wheel_radius_ * left_angular_speed;       // m/s
    const double right_speed = wheel_radius_ * right_angular_speed;     // m/s
    const double avg_speed = (left_speed + right_speed) / 2;            // m/s

    // Calculate the turn speed of the robot.
    const double icc_w = (right_angular_speed - left_angular_speed) / width_;

    dxdt[X] = avg_speed * cos(x[THETA]);
    dxdt[Y] = avg_speed * sin(x[THETA]);
    dxdt[THETA] = icc_w;

    if (enable_gravity_) {
        dxdt[Z] = x[Z_dot];
        dxdt[Z_dot] = mass_ * -9.8;
    } else {
        dxdt[Z] = 0;
        dxdt[Z_dot] = 0;
    }

    // Saturate based on external force:
    if (std::abs(ext_force_(0)) > 0.1) {
        dxdt[X] = 0;
    }

    if (std::abs(ext_force_(1)) > 0.1) {
        dxdt[Y] = 0;
    }

    if (std::abs(ext_force_(2)) > 0.1) {
        dxdt[Z] = 0;
    }

}
} // namespace motion
} // namespace scrimmage
