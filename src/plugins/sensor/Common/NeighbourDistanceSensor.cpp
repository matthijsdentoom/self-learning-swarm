//
// Created by matthijs on 6-12-18.
//

#include <zebroscrimmage/plugins/sensor/Common/NeighbourDistanceSensor.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>

#include <scrimmage/common/Random.h>
#include <scrimmage/math/Quaternion.h>

#include <scrimmage/fwd_decl.h>

NeighbourDistanceSensor::NeighbourDistanceSensor(int num_sensors, double comm_range, scrimmage::EntityPtr entity) :
num_sensors_(num_sensors), comm_range_(comm_range), entity_(entity)
{ }

int NeighbourDistanceSensor::findSector(scrimmage::StatePtr &own_state, scrimmage::State &other_state)
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

std::vector<std::pair<double, double>> NeighbourDistanceSensor::create_skirt(std::vector<scrimmage::ID> rtree_neighbors)
{
    std::vector<std::pair<double, double>> skirt;

    auto state = entity_->state();

    // Set all data to farrest possible distance (the comm range).
    for (int i = 0; i < num_sensors_; i++)
    {
        double sensor_heading = i * 2 * M_PI;

        if (i >= num_sensors_ / 2) {
            sensor_heading -= M_PI;
            sensor_heading *= -1;
        }

        skirt.emplace_back(std::make_pair(sensor_heading, comm_range_));
    }

    // Find sector of each neighbor and update if shorter by.
    for (auto &rtree_neighbor : rtree_neighbors) {

        auto other_state = *(*entity_->contacts())[rtree_neighbor.id()].state();

        // Ignore own position / id
        if (other_state.pos() != state->pos()) {
            // Get distance to other.
            Eigen::Vector3d diff = other_state.pos() - state->pos();
            double dist = diff.norm();

            // Find sector of other.
            int sector = findSector(state, other_state);

            if (dist < skirt[sector].second)
            {
                skirt[sector].second = dist;
            }
        }
    }

    return skirt;
}
