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

#ifndef INCLUDE_ZEBROSCRIMMAGE_PLUGINS_SENSOR_OPENAINEIGHBORSENSOR_OPENAINEIGHBORSENSOR_H_
#define INCLUDE_ZEBROSCRIMMAGE_PLUGINS_SENSOR_OPENAINEIGHBORSENSOR_OPENAINEIGHBORSENSOR_H_

#include <scrimmage/plugins/sensor/ScrimmageOpenAISensor/ScrimmageOpenAISensor.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/fwd_decl.h>

#include <random>
#include <vector>
#include <map>
#include <string>

namespace scrimmage {
namespace sensor {
class OpenAINeighborSensor : public ScrimmageOpenAISensor {
 public:
    OpenAINeighborSensor() = default;
    void init(std::map<std::string, std::string> &params) override;
    void set_observation_space() override;
    void get_observation(double *data, uint32_t beg_idx, uint32_t end_idx) override;

    scrimmage::RTreePtr rtree_;       // Tree which maintains the contacts with all other robots in the field.

 private:
    double comm_range_{};     // Communication range of the robot.
    int num_sensors_{};       // Number of sensors the robot has.

    int findSector(scrimmage::StatePtr &own_state, State &other_state);

    bool draw_sensor_{};          // Indicates whether the sensor range needs to be drawn.
};
} // namespace sensor
} // namespace scrimmage
#endif // INCLUDE_ZEBROSCRIMMAGE_PLUGINS_SENSOR_OPENAINEIGHBORSENSOR_OPENAINEIGHBORSENSOR_H_
