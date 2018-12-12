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

#include <zebroscrimmage/plugins/sensor/OpenAINeighborSensor/OpenAINeighborSensor.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/proto/Shape.pb.h>         // scrimmage_proto::Shape
#include <scrimmage/proto/ProtoConversions.h> // scrimmage::set()

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/math/State.h>

#include <iostream>
#include <limits>
#include <scrimmage/math/Angles.h>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Sensor,
                scrimmage::sensor::OpenAINeighborSensor,
                OpenAINeighborSensor_plugin)

namespace scrimmage {
namespace sensor {


void OpenAINeighborSensor::init(std::map<std::string, std::string> &params)
{
    comm_range_ = sc::get<double>("comm_range", params, comm_range_);
    num_sensors_ = sc::get<int>("num_sensors", params, 8);      // Requires to be even.
    draw_sensor_ = sc::get<bool>("draw_sensor", params, false);
    rtree_ = parent_->rtree();
}

void OpenAINeighborSensor::set_observation_space()
{
    // Register sensor for the sensor skirt.
    for (int i = 0; i < num_sensors_; i++)
    {
        observation_space.continuous_extrema.emplace_back(std::make_pair(0, comm_range_));
    }

    // Register the sensor with the heading of the closes robot.
    observation_space.continuous_extrema.emplace_back(std::make_pair(-M_PI, M_PI));
}

void OpenAINeighborSensor::get_observation(double *data, uint32_t beg_idx, uint32_t end_idx)
{
    // Get the neighbours which are currently in comm range.
    std::vector<ID> rtree_neighbors;
    auto state = parent_->state();
    rtree_->neighbors_in_range(state->pos_const(), rtree_neighbors, comm_range_);

    double closest_distance = comm_range_ + 1;
    double heading_of_closest_robot = parent_->state()->quat().yaw();

    // Set all data to farrest possible distance (the comm ra   nge).
    for (uint i = beg_idx; i < end_idx - 1; i++)
    {
        data[i] = comm_range_;
    }

    // Find sector of each neighbor and update if shorter by.
    for (auto &rtree_neighbor : rtree_neighbors) {

        auto other_state = *(*parent_->contacts())[rtree_neighbor.id()].state();

        // Ignore own position / id
        if (other_state.pos() != state->pos()) {
            // Get distance to other.
            Eigen::Vector3d diff = other_state.pos() - state->pos();
            double dist = diff.norm();

            // Find sector of other.
            int sector = findSector(state, other_state);

            if (dist < data[sector])
            {
                data[sector] = dist;

                // Update the heading if the robot is closest.
                if (dist < closest_distance)
                {
                    closest_distance = dist;
                    heading_of_closest_robot = other_state.quat().yaw();
                }
            }
        }
    }

    // Set the relative heading of the closest robot.
    double own_heading = parent_->state()->quat().yaw();
    double relative_heading = heading_of_closest_robot - own_heading;
    if (relative_heading < -M_PI) relative_heading += 2 * M_PI;  // Ensure relative heading in [-pi, pi]
    data[end_idx - 1] = relative_heading;

    // Draw lines to to make the sensors.
    if (draw_sensor_)
    {
        for (int i = 0; i < num_sensors_; i++) {
            double sensor_angle = 2 * M_PI * (1.0 * i / num_sensors_) + M_PI / num_sensors_;

            if (i >= num_sensors_ / 2) {
                sensor_angle = -sensor_angle + M_PI;
            }

            auto own_position = parent_->state()->pos();
            double heading = parent_->state()->quat().yaw() + sensor_angle;

            double x_diff = data[i] * cos(heading);
            double y_diff = data[i] * sin(heading);

            auto line = std::make_shared<scrimmage_proto::Shape>();
            sc::set(line->mutable_color(), 255, 0, 0);
            line->set_opacity(0.75);
            sc::set(line->mutable_line()->mutable_start(), own_position);
            sc::set(line->mutable_line()->mutable_end(), own_position + Eigen::Vector3d(x_diff, y_diff, 0.42));
            draw_shape(line);
        }
    }
}

int OpenAINeighborSensor::findSector(scrimmage::StatePtr &own_state, State &other_state)
{
    int sector = 0;
    double fov_width = 4 * M_PI / num_sensors_;

    if (own_state->rel_pos_local_frame(other_state.pos())[1] < 0)
    {   // If the position of the other robot is to the left, put it on a sensor to the other side.
        sector += num_sensors_ / 2;
    }

    for (int i = 0; i < num_sensors_ / 2; i++)
    {
        if (own_state->InFieldOfView(other_state, fov_width /* fov width */, 90 /* fov height */))
        {   // The field of view works on 2 sides, but we already accounted this above.
            sector += i;
            break;
        }

        fov_width += 4 * M_PI / num_sensors_;
    }

    return sector;
}


} // namespace sensor
} // namespace scrimmage
