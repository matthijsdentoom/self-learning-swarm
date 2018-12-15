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

#ifndef INCLUDE_ZEBROSCRIMMAGE_PLUGINS_METRICS_FLOCKINGMETRIC_FLOCKINGMETRIC_H_
#define INCLUDE_ZEBROSCRIMMAGE_PLUGINS_METRICS_FLOCKINGMETRIC_FLOCKINGMETRIC_H_

#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/parse/ParseUtils.h>
#include <zebroscrimmage/plugins/sensor/PositionSensor/PositionSensor.h>

#include <string>
#include <vector>

namespace sc = scrimmage;

namespace scrimmage {
namespace metrics {



class FlockingMetric : public scrimmage::Metrics {
public:
    FlockingMetric();
    std::string name() override { return std::string("FlockingMetric"); }

    void init(std::map<std::string, std::string> &params) override;

    bool step_metrics(double t, double dt) override;

    void calc_team_scores() override;

    void print_team_summaries() override;
protected:
    double norm2(double first, double second);
private:
    std::vector<PositionMessage> receivedMessages_;

    double max_tolerated_distance_; // Maximum distance which is taken into account.
    double gamma_;                  // Division factor between grouping and distance covered.
    double grouping_fitness_;       // Fitness based on the grouping of the robot.
    double distance_fitness_;       // Fitness based on the distance.
    double fitness_;                // Fitness found up till this point.
    double top_speed_;              // Top speed of the robot.
    int tick_counter_;
};

} // namespace metrics
} // namespace scrimmage
#endif // INCLUDE_ZEBROSCRIMMAGE_PLUGINS_METRICS_FLOCKINGMETRIC_FLOCKINGMETRIC_H_
