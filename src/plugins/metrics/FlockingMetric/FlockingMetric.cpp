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
    auto position_msg_cb = [&] (scrimmage::MessagePtr<PositionMessage> msg) {
        cout << "received message" << endl;
        cout << msg->data.id << " at " << msg->data.x_pos << ", " << msg->data.y_pos << endl;
        receivedMessages_.insert(std::make_pair(msg->data.id, msg->data));
    };
    subscribe<PositionMessage>("GlobalNetwork", "Positions", position_msg_cb);
}

bool FlockingMetric::step_metrics(double t, double dt) {

    // Gather all received data and update it.

    return true;
}

void FlockingMetric::calc_team_scores() {

//    for (auto &kv : team_coll_scores_) {
//        int team_id = kv.first;
//        int &score = kv.second;
//        team_metrics_[team_id]["ground_coll"] = score.ground_collisions();
//        team_scores_[team_id] = score.score();
//    }

    // list the headers we want put in the csv file
    headers_.emplace_back("flocking_index");
}

void FlockingMetric::print_team_summaries() {
//    for (std::map<int, Score>::iterator it = team_coll_scores_.begin();
//         it != team_coll_scores_.end(); ++it) {
//
//        cout << "Score: " << it->second.score() << endl;
//        cout << "Ground Collisions: " << it->second.ground_collisions() << endl;
//        cout << sc::generate_chars("-", 70) << endl;
//    }
}
} // namespace metrics
} // namespace scrimmage
