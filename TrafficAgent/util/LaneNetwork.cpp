/*
 * @copyright (c) 2022, FZI Forschungszentrum Informatik
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 *  Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "LaneNetwork.h"

#include "OSIUtil.h"

std::vector<FZIDriver::util::IntersectionLane> FZIDriver::util::Intersection::getAllConnectingLanes(unsigned int incomingLaneID) {
    std::vector<FZIDriver::util::IntersectionLane> outputLanes(this->intersectionLanes.size());
    auto it = std::copy_if(this->intersectionLanes.begin(), this->intersectionLanes.end(), outputLanes.begin(),
                            [incomingLaneID](const FZIDriver::util::IntersectionLane &obj) {
                                return (std::get<0>(obj.predecessorLane) == incomingLaneID);
                            });         
    outputLanes.resize(std::distance(outputLanes.begin(), it));
    return outputLanes;
}

FZIDriver::util::LaneNetwork FZIDriver::util::LaneNetwork::createLaneNetworkFromSensorView(osi3::SensorView sensorView) {
    std::map<unsigned int, FZIDriver::util::Lane> laneData;
    std::map<unsigned int, FZIDriver::util::Intersection> intersectionData;

    // iterate over all lanes in the sensor view
    for (osi3::Lane osiLane: sensorView.global_ground_truth().lane()) {
        // read the id of the lane
        int laneID = osiLane.id().value();


        // check if the lane is of driving type
        if (osiLane.classification().type() == osi3::Lane_Classification::TYPE_DRIVING) {
            // create lane object
            FZIDriver::util::Lane lane(laneID);
            
            // read the predecessor and successor id
            unsigned int predecessorID = osiLane.classification().lane_pairing().Get(0).antecessor_lane_id().value();
            unsigned int successorID = osiLane.classification().lane_pairing().Get(0).successor_lane_id().value();
            if (!osiLane.classification().centerline_is_driving_direction()) std::swap(predecessorID, successorID);

            osi3::Lane predecessorLane;
            if (FZIDriver::util::OSIUtil::getLaneByID(sensorView.global_ground_truth().lane(), predecessorID, predecessorLane)) {
                lane.predecessorLane = std::make_pair(predecessorID, FZIDriver::enums::getLaneTypeFromOSILaneType(predecessorLane.classification().type()));
            } else {
                lane.predecessorLane = std::make_pair(0, FZIDriver::enums::LaneType::undefined);
            }

            osi3::Lane successorLane;
            if (FZIDriver::util::OSIUtil::getLaneByID(sensorView.global_ground_truth().lane(), successorID, successorLane)) {
                lane.successorLane = std::make_pair(successorID, FZIDriver::enums::getLaneTypeFromOSILaneType(successorLane.classification().type()));
            } else {
                lane.successorLane = std::make_pair(0, FZIDriver::enums::LaneType::undefined);
            }
            
            // create a path
            lane.path = FZIDriver::util::Path::createPathFromOsiDrivingLane(osiLane);
            // store the lane
            laneData.insert(std::pair<unsigned int, FZIDriver::util::Lane>(laneID, lane));
        }

        // check if the lane is of type junction
        if (osiLane.classification().type() == osi3::Lane_Classification::TYPE_INTERSECTION) {
            FZIDriver::util::Intersection intersection(laneID);
            // iterate over all lane pairings
            for (auto lanePairing: osiLane.classification().lane_pairing()) {
                FZIDriver::util::IntersectionLane intersectionLane(laneID);

                // get the lane id of the predecessors and successors
                intersectionLane.predecessorLane = std::make_pair<unsigned int, FZIDriver::enums::LaneType>(lanePairing.antecessor_lane_id().value(), FZIDriver::enums::LaneType::driving);
                intersectionLane.successorLane = std::make_pair<unsigned int, FZIDriver::enums::LaneType>(lanePairing.successor_lane_id().value(), FZIDriver::enums::LaneType::driving);

                // get the predecessor lane
                osi3::Lane predecessorLane;
                FZIDriver::util::OSIUtil::getLaneByID(sensorView.global_ground_truth().lane(), std::get<0>(intersectionLane.predecessorLane), predecessorLane);
                // get the successor lane
                osi3::Lane successorLane;
                FZIDriver::util::OSIUtil::getLaneByID(sensorView.global_ground_truth().lane(), std::get<0>(intersectionLane.successorLane), successorLane);

                // create a path and determine turning direction
                std::tie(intersectionLane.path, intersectionLane.turningDirection) = FZIDriver::util::Path::createPathFromOsiIntersection(predecessorLane, successorLane);
                // store the information in the intersection
                intersection.intersectionLanes.push_back(intersectionLane);

            }
            intersectionData.insert(std::pair<unsigned int, FZIDriver::util::Intersection>(laneID, intersection));
        }
    }
    return FZIDriver::util::LaneNetwork(laneData, intersectionData);
}

#if TARGET_OPEN_PASS
/**
* @brief Create a lane network based on the Sensor View provided by OpenPASS.
* Note: This assumes that only one intersection is present in the ground truth.
*/
FZIDriver::util::LaneNetwork FZIDriver::util::LaneNetwork::createLaneNetworkFromOpenPassSensorView(osi3::SensorView sensorView) {
    // iterarte over all lanes and get the relevant information
    std::vector<unsigned int> drivingLaneIDs;
    std::vector<unsigned int> intersectionLaneIDs;
    // iterate over all lanes
    for(auto lane: sensorView.global_ground_truth().lane()) {
        if (lane.classification().type() == osi3::Lane::Classification::Type::Lane_Classification_Type_TYPE_DRIVING) {
             drivingLaneIDs.push_back(lane.id().value());
        }
        if (lane.classification().type() == osi3::Lane::Classification::Type::Lane_Classification_Type_TYPE_INTERSECTION) {
            intersectionLaneIDs.push_back(lane.id().value());
        }
    }

    // create a list of all lane pairings
    auto hash = [](const std::pair<unsigned int, unsigned int> &p){return p.first * 31 + p.second;};
    std::unordered_set<std::pair<unsigned int, unsigned int>, decltype(hash)> lanePairingsOnIntersection(0, hash);
    for (auto lane: sensorView.global_ground_truth().lane()) {
        if (lane.classification().type() == osi3::Lane::Classification::Type::Lane_Classification_Type_TYPE_INTERSECTION) {
            // read the predecessor and successor id
            unsigned int predecessorID, successorID;
            if (lane.classification().centerline_is_driving_direction()) {
                predecessorID = lane.classification().lane_pairing().Get(0).antecessor_lane_id().value();
                successorID = lane.classification().lane_pairing().Get(0).successor_lane_id().value();
            } else {
                predecessorID = lane.classification().lane_pairing().Get(0).successor_lane_id().value();
                successorID = lane.classification().lane_pairing().Get(0).antecessor_lane_id().value();
            }

            // find the predecessor
            osi3::Lane predecessorLane;
            FZIDriver::util::OSIUtil::getLaneByID(sensorView.global_ground_truth().lane(), predecessorID, predecessorLane);
            while (predecessorLane.classification().type() == osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION) {
                if (predecessorLane.classification().centerline_is_driving_direction()) {
                    predecessorID = predecessorLane.classification().lane_pairing().Get(0).antecessor_lane_id().value();
                } else {
                    predecessorID = predecessorLane.classification().lane_pairing().Get(0).successor_lane_id().value();
                }
                FZIDriver::util::OSIUtil::getLaneByID(sensorView.global_ground_truth().lane(), predecessorID, predecessorLane);
            }

            if (predecessorLane.classification().type() != osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING) {
                continue;
            }

            // find the successor
            osi3::Lane successorLane;
            FZIDriver::util::OSIUtil::getLaneByID(sensorView.global_ground_truth().lane(), successorID, successorLane);
            while (successorLane.classification().type() == osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION) {
                if (successorLane.classification().centerline_is_driving_direction()) {
                    successorID = successorLane.classification().lane_pairing().Get(0).successor_lane_id().value();
                } else {
                    successorID = successorLane.classification().lane_pairing().Get(0).antecessor_lane_id().value();
                }
                FZIDriver::util::OSIUtil::getLaneByID(sensorView.global_ground_truth().lane(), successorID, successorLane);
            }

            // insert the lane pairing
            lanePairingsOnIntersection.insert(std::make_pair(predecessorID, successorID));
        }
    }

    // create the lane objects
    std::map<unsigned int, FZIDriver::util::Lane> laneData;
    std::map<unsigned int, FZIDriver::util::Intersection> intersectionData;

    for (auto osiLane: sensorView.global_ground_truth().lane()) {
        if (osiLane.classification().type() == osi3::Lane::Classification::Type::Lane_Classification_Type_TYPE_DRIVING) {
            unsigned int laneID = osiLane.id().value();
            bool centerLineDrivingDirection = osiLane.classification().centerline_is_driving_direction();

            // create lane object
            FZIDriver::util::Lane lane(laneID);
            
            // read the predecessor and successor
            std::unordered_set<unsigned int> predecessorCandidates;
            std::unordered_set<unsigned int> successorCandidates;
            for (auto lanePairing: osiLane.classification().lane_pairing()) {
                unsigned int lanePairingPredecessor, lanePairingSuccessor;
                if (osiLane.classification().centerline_is_driving_direction()) {
                    lanePairingPredecessor = lanePairing.antecessor_lane_id().value();
                    lanePairingSuccessor = lanePairing.successor_lane_id().value();
                } else {
                    lanePairingPredecessor = lanePairing.successor_lane_id().value();
                    lanePairingSuccessor = lanePairing.antecessor_lane_id().value();
                }

                if (std::find(drivingLaneIDs.begin(), drivingLaneIDs.end(), lanePairingPredecessor) != drivingLaneIDs.end()) {
                    predecessorCandidates.insert(lanePairingPredecessor);
                } else if(std::find(intersectionLaneIDs.begin(), intersectionLaneIDs.end(), lanePairingPredecessor) != intersectionLaneIDs.end()) {
                    // ToDo: Deal with maps having multiple intersection
                    predecessorCandidates.insert(intersectionLaneIDs[0]);
                }

                if (std::find(drivingLaneIDs.begin(), drivingLaneIDs.end(), lanePairingSuccessor) != drivingLaneIDs.end()) {
                    successorCandidates.insert(lanePairingSuccessor);
                } else if(std::find(intersectionLaneIDs.begin(), intersectionLaneIDs.end(), lanePairingSuccessor) != intersectionLaneIDs.end()) {
                    // ToDo: Deal with maps having multiple intersection
                    successorCandidates.insert(intersectionLaneIDs[0]);
                }
            }

            if (predecessorCandidates.size() == 0) {
                lane.predecessorLane = std::make_pair(0, FZIDriver::enums::LaneType::undefined);
            } else {
                unsigned int predecessorLaneID = *(predecessorCandidates.begin());
                if (std::find(drivingLaneIDs.begin(), drivingLaneIDs.end(), predecessorLaneID) != drivingLaneIDs.end()) {
                    lane.predecessorLane = std::make_pair(predecessorLaneID, FZIDriver::enums::LaneType::driving);
                } else {
                    lane.predecessorLane = std::make_pair(intersectionLaneIDs[0], FZIDriver::enums::LaneType::intersection);
                }
            }

            if (successorCandidates.size() == 0) {
                lane.successorLane = std::make_pair(0, FZIDriver::enums::LaneType::undefined);
            } else {
                unsigned int successorLaneID = *(successorCandidates.begin());
                if (std::find(drivingLaneIDs.begin(), drivingLaneIDs.end(), successorLaneID) != drivingLaneIDs.end()) {
                    lane.successorLane = std::make_pair(successorLaneID, FZIDriver::enums::LaneType::driving);
                } else {
                    lane.successorLane = std::make_pair(intersectionLaneIDs[0], FZIDriver::enums::LaneType::intersection);
                }
            }

            // create a path
            lane.path = FZIDriver::util::Path::createPathFromOsiDrivingLane(osiLane);
            // store the lane
            laneData.insert(std::pair<unsigned int, FZIDriver::util::Lane>(laneID, lane));
        }
    }

    FZIDriver::util::Intersection intersection(intersectionLaneIDs[0]);
    for (auto lanePairing: lanePairingsOnIntersection) {
        //create an intersection-lane instance
        FZIDriver::util::IntersectionLane intersectionLane(intersectionLaneIDs[0]);
        // set the predecessor and successor
        intersectionLane.predecessorLane = std::make_pair(lanePairing.first, FZIDriver::enums::LaneType::driving);
        intersectionLane.successorLane = std::make_pair(lanePairing.second, FZIDriver::enums::LaneType::driving);

        // get both lanes
        osi3::Lane predecessorOSILane;
        FZIDriver::util::OSIUtil::getLaneByID(sensorView.global_ground_truth().lane(), lanePairing.first, predecessorOSILane);
        osi3::Lane successorOsiLane;
        FZIDriver::util::OSIUtil::getLaneByID(sensorView.global_ground_truth().lane(), lanePairing.second, successorOsiLane);
        // create the path
        std::tie(intersectionLane.path, intersectionLane.turningDirection) = 
                FZIDriver::util::Path::createPathFromOsiIntersection(predecessorOSILane, successorOsiLane);
        intersection.intersectionLanes.push_back(intersectionLane);
    }
    intersectionData.insert(std::pair<unsigned int, FZIDriver::util::Intersection>(intersectionLaneIDs[0], intersection));

    return FZIDriver::util::LaneNetwork(laneData, intersectionData);
}
#endif

FZIDriver::util::IntersectionLane FZIDriver::util::LaneNetwork::findIntermediateLaneByTurningDirection(unsigned int incomingLaneID, 
                                                                    FZIDriver::enums::TurningDirction turningDirection) {
    unsigned int intersectionID = std::get<0>(this->laneMap[incomingLaneID].successorLane);

    auto intermediateLane = std::find_if(this->intersectionMap[intersectionID].intersectionLanes.begin(),
                                        this->intersectionMap[intersectionID].intersectionLanes.end(),
                                        [incomingLaneID, turningDirection](const FZIDriver::util::IntersectionLane &obj){
                                            return ((std::get<0>(obj.predecessorLane) == incomingLaneID) && (obj.turningDirection == turningDirection));
                                        });
    return *intermediateLane;
} 

bool FZIDriver::util::LaneNetwork::findConnection(FZIDriver::util::Lane startLane, 
                                                  FZIDriver::util::Lane finalLane, 
                                                  std::deque<FZIDriver::util::Lane> &connectingLanes,
                                                  FZIDriver::enums::TurningDirction &turningDirection_out) {
    // stop criteria
    if (startLane.id == finalLane.id) {
        connectingLanes.push_front(finalLane);
        return true;
    }

    // check if the successor is an intersection or a lane
    if (std::get<1>(startLane.successorLane) == FZIDriver::enums::LaneType::driving) {
        auto nextLane = this->getLane(std::get<0>(startLane.successorLane));
        if (this->findConnection(nextLane, finalLane, connectingLanes, turningDirection_out)) {
            connectingLanes.push_front(startLane);
            return true;
        }
    } else if(std::get<1>(startLane.successorLane) == FZIDriver::enums::LaneType::intersection) {
        auto nextIntersection = this->getIntersection(std::get<0>(startLane.successorLane));
        for(auto nextLane: nextIntersection.getAllConnectingLanes(startLane.id)) {
            if(this->findConnection(nextLane, finalLane, connectingLanes, turningDirection_out)) {
                connectingLanes.push_front(startLane);
                turningDirection_out = nextLane.turningDirection;
                return true;
            }
        }
    }

    return false;
}

std::pair<FZIDriver::util::Lane, double> FZIDriver::util::LaneNetwork::findClosestLaneWithPosition(double posX, double posY) {
    
    if(this->globalPositionToPathMapping.find(std::pair<double, double>(posX, posY)) != this->globalPositionToPathMapping.end()) {
        return this->globalPositionToPathMapping[std::pair<double, double>(posX, posY)];
    } 
                
    FZIDriver::util::Lane currentLane;
    double posS_min = 0;
    double distance_min = std::numeric_limits<double>::max();

    // iterate over all lanes
    for (auto laneEntry: this->laneMap) {
        // calculate the distance to the path
        auto result = laneEntry.second.path.mapToPath(posX, posY);

        // check of the distance is smaller than the current minimal distance
        if (std::get<1>(result)  < distance_min) {
            posS_min = std::get<0>(result);
            distance_min = std::get<1>(result);
            currentLane = laneEntry.second;
        }
    }

    // iterate over all intersectoins
    for (auto intersectionEntry: this->intersectionMap) {
        // iterate over all intersections on the lane
        for (auto intersectionLaneEntry: intersectionEntry.second.intersectionLanes) {
            // calculate the distance to the path
            auto result = intersectionLaneEntry.path.mapToPath(posX, posY);

            // check of the distance is smaller than the current minimal distance
            if (std::get<1>(result)  < distance_min) {
                posS_min = std::get<0>(result);
                distance_min = std::get<1>(result);
                currentLane = intersectionLaneEntry;
            }
        }
    }

    auto result = std::pair<FZIDriver::util::Lane, double>(currentLane, posS_min);
    // store the closest path for the given coordinates
    this->globalPositionToPathMapping[std::pair<double, double>(posX, posY)] = result;

    return result;
}