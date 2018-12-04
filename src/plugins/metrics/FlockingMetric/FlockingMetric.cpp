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

#include <zebroscrimmage/plugins/metrics/FlockingMetric/FlockingMetric.h>

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
#include <math.h>       /* pow */

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Metrics,
                scrimmage::metrics::FlockingMetric,
                FlockingMetric_plugin)

namespace scrimmage {
namespace metrics {

FlockingMetric::FlockingMetric() = default;

void FlockingMetric::init(std::map<std::string, std::string> &params) {

    max_tolerated_distance_ = sc::get<double>("max_tolerated_distance", params, 20);
    gamma_ = sc::get<double>("gamma", params, 0.5);

    auto position_msg_cb = [&] (scrimmage::MessagePtr<PositionMessage> msg) {
//        cout << msg->data.id << " at " << msg->data.x_pos << ", " << msg->data.y_pos << endl;
        receivedMessages_.emplace_back(msg->data);
    };

    subscribe<PositionMessage>("GlobalNetwork", "Positions", position_msg_cb);
}

bool FlockingMetric::step_metrics(double t, double dt) {

    // Calculate average location
    double avg_x = 0;
    double avg_y = 0;

    for (auto msg : receivedMessages_)
    {
        avg_x += msg.x_pos;
        avg_y += msg.y_pos;
    }

    avg_x /= receivedMessages_.size();
    avg_y /= receivedMessages_.size();

    double summed_deviation = 0.0;

    // Calculate average deviation from mean
    for (auto msg : receivedMessages_)
    {
        summed_deviation += sqrt(pow(avg_x - msg.x_pos, 2) + pow(avg_y - msg.y_pos, 2));
    }

    summed_deviation /= max_tolerated_distance_;

    // Calculate deviation fitness
    grouping_fitness_ = 1 - summed_deviation / receivedMessages_.size();
    distance_fitness_ = norm2(avg_x, avg_y);

    fitness_ = gamma_ * grouping_fitness_ + (1 - gamma_) * distance_fitness_;

    receivedMessages_.clear();

    return true;
}

double FlockingMetric::norm2(double first, double second)
{
    return sqrt(pow(first, 2) + pow(second, 2));
}

void FlockingMetric::calc_team_scores() {

    headers_.emplace_back("distance");
    team_metrics_[1]["distance"] = distance_fitness_;

    headers_.emplace_back("grouping");
    team_metrics_[1]["grouping"] = grouping_fitness_;

    team_scores_[1] = fitness_;
}

void FlockingMetric::print_team_summaries() {

    cout << "distance fitness: " <<  distance_fitness_ << endl;
    cout << "grouping fitness: " <<  grouping_fitness_ << endl;
    cout << "Fitness: " << fitness_ << endl;

}
} // namespace metrics
} // namespace scrimmage
