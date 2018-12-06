//
// Created by matthijs on 6-12-18.
//

#ifndef ZEBROSCRIMMAGE_NEIGHBOURDISTANCESENSOR_H
#define ZEBROSCRIMMAGE_NEIGHBOURDISTANCESENSOR_H

#include <scrimmage/entity/Entity.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/fwd_decl.h>

#include <random>
#include <vector>
#include <map>
#include <string>


class NeighbourDistanceSensor
{
public:
    NeighbourDistanceSensor(int num_sensors, double comm_range, scrimmage::EntityPtr entity);

    virtual ~NeighbourDistanceSensor() = default;

    /**
     * This function creates a sensor skirt based on the robots which are in communication range.
     * @param rtree_neighbors   - Robots in range.
     * @return                  - Sensor skirt.
     */
    std::vector<std::pair<double, double>> create_skirt(std::vector<scrimmage::ID> rtree_neighbors);

private:


    int findSector(scrimmage::StatePtr &own_state, scrimmage::State &other_state);

    int num_sensors_;    // Number of sensors the robot has.
    double comm_range_;        // Communication range of the robot.

    scrimmage::EntityPtr entity_;    // The entity on which this class is being used.
};


#endif //ZEBROSCRIMMAGE_NEIGHBOURDISTANCESENSOR_H
