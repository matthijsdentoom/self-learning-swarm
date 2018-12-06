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


OpenAINeighborSensor::~OpenAINeighborSensor()
{
    delete nds_;
}

void OpenAINeighborSensor::init(std::map<std::string, std::string> &params)
{
    comm_range_ = sc::get<double>("comm_range", params, comm_range_);
    num_sensors_ = sc::get<int>("num_sensors", params, 8);      // Requires to be even.
    rtree_ = parent_->rtree();
    nds_ = new NeighbourDistanceSensor(num_sensors_, comm_range_, parent_);
}

void OpenAINeighborSensor::set_observation_space()
{
    ScrimmageOpenAISensor::set_observation_space();
    for (int i = 0; i < num_sensors_; i++)
    {
        observation_space.continuous_extrema.push_back(std::make_pair(0, comm_range_));
    }

}

void OpenAINeighborSensor::get_observation(double *data, uint32_t beg_idx, uint32_t end_idx)
{
    // Get the neighbours which are currently in comm range.
    std::vector<ID> rtree_neighbors;
    auto state = parent_->state();
    rtree_->neighbors_in_range(state->pos_const(), rtree_neighbors, comm_range_);

    auto skirt = nds_->create_skirt(rtree_neighbors);

    for (uint i = beg_idx; i < end_idx; i++)
    {
        data[i] = skirt[i].second;
    }
}


} // namespace sensor
} // namespace scrimmage
