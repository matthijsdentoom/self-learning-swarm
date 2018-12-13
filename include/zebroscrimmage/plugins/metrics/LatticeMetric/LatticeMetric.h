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

#ifndef INCLUDE_ZEBROSCRIMMAGE_PLUGINS_METRICS_LATTICEMETRIC_LATTICEMETRIC_H_
#define INCLUDE_ZEBROSCRIMMAGE_PLUGINS_METRICS_LATTICEMETRIC_LATTICEMETRIC_H_

#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/parse/ParseUtils.h>

#include <map>
#include <string>
#include <zebroscrimmage/plugins/sensor/PositionSensor/PositionSensor.h>

namespace sc = scrimmage;

namespace scrimmage {
namespace metrics {

/**
 * This metric aims to access the deviation energy, as is described in the paper: Flocking for multi-agent dynamic
 * systems: algorithms and theory.
 */
class LatticeMetric : public scrimmage::Metrics {
 public:
    LatticeMetric();
    std::string name() override { return std::string("LatticeMetric"); }
    void init(std::map<std::string, std::string> &params) override;
    bool step_metrics(double t, double dt) override;
    void calc_team_scores() override;
    void print_team_summaries() override;
 protected:

    /**
     * This function calculates the distance between two robots.
     * @param robot1    - First robot.
     * @param robot2    - Second robots.
     * @return          - Eucledian distance between two robots.
     */
    double dist(PositionMessage *robot1, PositionMessage *robot2);
 private:
    std::vector<PositionMessage> receivedMessages_;
    std::vector<double> deviationEnergies_;

    double target_distance_;
    int tick_counter_;
};

} // namespace metrics
} // namespace scrimmage
#endif // INCLUDE_ZEBROSCRIMMAGE_PLUGINS_METRICS_LATTICEMETRIC_LATTICEMETRIC_H_
