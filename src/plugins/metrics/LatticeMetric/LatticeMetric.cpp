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

#include <zebroscrimmage/plugins/metrics/LatticeMetric/LatticeMetric.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/metrics/Metrics.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/msgs/Event.pb.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Metrics,
                scrimmage::metrics::LatticeMetric,
                LatticeMetric_plugin)

namespace scrimmage {
namespace metrics {

LatticeMetric::LatticeMetric() : target_distance_(5), tick_counter_(0)
{ }

void LatticeMetric::init(std::map<std::string, std::string> &params) {


    target_distance_ = sc::get<double>("target_distance", params, 5);
    tick_counter_ = 0;

    auto position_msg_cb = [&] (scrimmage::MessagePtr<PositionMessage> msg) {

        receivedMessages_.emplace_back(msg->data);
    };

    subscribe<PositionMessage>("GlobalNetwork", "Positions", position_msg_cb);
}

bool LatticeMetric::step_metrics(double t, double dt) {

    if (t > tick_counter_)
    {   // Only check every whole point.
        tick_counter_++;

        std::vector<double> distance_between_robots_in_range;

        // Get all robot pairs in communication range.
        for (auto robot1 = begin (receivedMessages_); robot1 != end (receivedMessages_); ++robot1)
        {
            for (auto robot2 = robot1; robot2 != end (receivedMessages_); ++robot2)
            {
                double distance = dist(robot1.base(), robot2.base());
                if ((robot1->id != robot2->id) && distance < target_distance_)
                {
                    distance_between_robots_in_range.emplace_back(distance);
                }
            }
        }

        // Calculate the deviation energy for the robots.
        double deviation_energy = 0.0;

        for (double distance_between_pair : distance_between_robots_in_range)
        {
            deviation_energy += pow(distance_between_pair - target_distance_, 2);
        }

        deviation_energy *= 1.0 / (distance_between_robots_in_range.size() + 1.0);

        deviationEnergies_.emplace_back(deviation_energy);
    }

    receivedMessages_.clear();

    return true;
}

void LatticeMetric::calc_team_scores()
{
    double min = std::numeric_limits<double>::max();
    double sum = 0.0;

    for (auto& n : deviationEnergies_)
    {
        sum += n;
        if (n < min) min = n;
    }

    headers_.emplace_back("min_dev");
    team_metrics_[1]["min_dev"] = min;

    headers_.emplace_back("sum_dev");
    team_metrics_[1]["sum_dev"] = sum;

    team_scores_[1] = -sum;
}

void LatticeMetric::print_team_summaries()
{
    cout << "Summed  deviation energy: " <<  team_metrics_[1]["sum_dev"] << endl;
    cout << "minimal deviation energy: " <<  team_metrics_[1]["min_dev"]<< endl;
}

double LatticeMetric::dist(PositionMessage *robot1, PositionMessage *robot2)
{
    return sqrt(
            pow((robot1->x_pos - robot2->x_pos), 2) +
            pow((robot1->y_pos - robot2->y_pos), 2)
            );
}
} // namespace metrics
} // namespace scrimmage
