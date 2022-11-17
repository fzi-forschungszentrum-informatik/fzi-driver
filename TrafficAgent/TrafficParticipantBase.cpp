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
# include "TrafficParticipantBase.h"

#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

FZIDriver::TrafficAgentBase::TrafficAgentBase(osi3::MovingObject movingObject, double vRef_in, double stepSize_in,
                                              std::shared_ptr<FZIDriver::util::LaneNetwork> laneNetwork,
                                              std::map<unsigned int, std::shared_ptr<TrafficAgentBase>> &trafficParticipantMap): 
                                              FZIDriver::TrafficAgentBase::TrafficAgentBase(stepSize_in) {
    // store the lane network
    this->laneNetwork = laneNetwork;
    // set the id
    this->agentID = movingObject.id().value();
    // set the current state
    FZIDriver::util::State state = FZIDriver::util::State(movingObject);
    // set the current lane id
    //this->initialLaneID = movingObject.assigned_lane_id().Get(0).value();
    this->initialLaneID = std::get<0>(this->laneNetwork->findClosestLaneWithPosition(state.posX, state.posY)).id;

    // set the traffic participants map
    this->trafficParticipantMap = trafficParticipantMap;
    // store the reference velocity 
    this->vRef = vRef_in;

    // get the indicator state
    auto indicatorState = movingObject.vehicle_classification().light_state().indicator_state();
    // set the turning direction based on the indicator state
    if (indicatorState == osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_OFF) {
        this->turningDirection = FZIDriver::enums::TurningDirction::strait;
    } else if (indicatorState == osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_LEFT) {
        this->turningDirection = FZIDriver::enums::TurningDirction::left;
    } else if (indicatorState == osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_RIGHT) {
        this->turningDirection = FZIDriver::enums::TurningDirction::right;
    } else {
        this->turningDirection = FZIDriver::enums::TurningDirction::undefined;
    }


    // find the road on the intersection the traffic participant is going to use
    auto incomingLane = this->laneNetwork->getLane(this->initialLaneID);
    while (std::get<1>(incomingLane.successorLane) == FZIDriver::enums::LaneType::driving) {
        incomingLane = this->laneNetwork->getLane(std::get<0>(incomingLane.successorLane));
    } 

    auto intermediateLane = this->laneNetwork->findIntermediateLaneByTurningDirection(incomingLane.id, this->turningDirection);
    auto finalLane = this->laneNetwork->getLane(std::get<0>(intermediateLane.successorLane));
    while (std::get<1>(finalLane.successorLane) == FZIDriver::enums::LaneType::driving) {
        finalLane = this->laneNetwork->getLane(std::get<0>(finalLane.successorLane));
    }

    std::deque<FZIDriver::util::Lane> intermediateLanes;
            this->laneNetwork->findConnection(this->laneNetwork->getLane(this->initialLaneID), 
                                              finalLane, 
                                              intermediateLanes, this->turningDirection);

    // create the centerline
    std::vector<FZIDriver::util::Vector2d<double>> centerline;
    for (auto lane: intermediateLanes) {
        for(unsigned int i = 0; i < lane.path.getXCoordinates().size(); i++) {
            centerline.push_back(FZIDriver::util::Vector2d<double>(lane.path.getXCoordinates().at(i),
                                                                   lane.path.getYCoordinates().at(i)));
        }
    }
    // create a data pair containing the path
    this->agentPath = std::make_shared<FZIDriver::util::Path>(FZIDriver::util::Path(centerline));
    // calculate the position on the path
    this->pathState = std::tuple<double, double>(this->mapToPath(state), state.velocity);

    // calculate the current lane id
    this->currentLaneID = this->calculateCurrentLaneID(state.posX, state.posY);

    // create relevant information for all targets and create a list of target ids
    for (auto &entry: this->trafficParticipantMap) {
        if (entry.first != this->agentID) {
            this->addTargetPath(entry.first, entry.second->getAgentPath());
            this->setRelationToTargetPath(entry.first);
            // calculate the priority
            this->setPriorityOver(entry.first);
        }
    }
}

void FZIDriver::TrafficAgentBase::updatePosition(osi3::MovingObject movingObject) {
    // set the current state
    FZIDriver::util::State state = FZIDriver::util::State(movingObject);
    // calculate the position on the path
    this->pathState = std::tuple<double, double>(this->mapToPath(state), state.velocity);
    this->setRelationToTargetPath(this->relevanTargets[0]);

    // calculate the current lane id
    this->currentLaneID = this->calculateCurrentLaneID(state.posX, state.posY);

    // calculate the priority
    this->setPriorityOver(this->relevanTargets[0]);
}

void FZIDriver::TrafficAgentBase::addTargetPath(int targetID, std::shared_ptr<FZIDriver::util::Path> targetPath) {
    (*this->targetPathMap)[targetID] = targetPath;
    (*this->intersectionCoordinateMap)[targetID] = FZIDriver::util::Path::calculatePathIntersection(*targetPath, *this->agentPath);
}

FZIDriver::PathRelation FZIDriver::TrafficAgentBase::calculatePathRelation(int targetID, std::tuple<double, double> pathState) {
    //calculate the global position
    auto globalPosition = this->agentPath->getPosition(std::get<0>(this->pathState));
    auto minDistance_hostPath = (*this->targetPathMap)[targetID]->mapToPath(std::get<0>(globalPosition), std::get<1>(globalPosition));

    double posS_host, distance, hostPath_velocity;
    FZIDriver::enums::TargetRelation targetRelation;
    if (std::get<1>(minDistance_hostPath) < 1e-3) {
        // the traffic participant drives on the same path as the host
        targetRelation = FZIDriver::enums::TargetRelation::IDENTICAL_PATH;
        // the reference velocity is the velocity of the target
        hostPath_velocity = std::get<1>(pathState);

        // ToDo: Make the actual (signed) distance along the path
        posS_host = std::get<0>(minDistance_hostPath);
        distance = std::get<1>(minDistance_hostPath);
    } else {
        // the host path velocity is approximately zero
        hostPath_velocity = 0;

        // calculate the intersection between the local path and the host path
        std::tuple<double, double, double> intersectionCoordinates = (*this->intersectionCoordinateMap)[targetID];
        posS_host = std::get<0>(intersectionCoordinates);  // the s coordinate of the intersection on the host path
        double targetS = std::get<1>(intersectionCoordinates);  // the s coordinate of the intersection on the traffic participants path
        double pathDistance = std::get<2>(intersectionCoordinates);  // the distance between the paths at the intersection point

        // calculate the distance to the intersection point
        distance = std::get<1>(pathState) - targetS;

        if (pathDistance < 0.5) {
            targetRelation = FZIDriver::enums::TargetRelation::CROSSING;
        } else {
            targetRelation = FZIDriver::enums::TargetRelation::NONE;
        }
    }

    return FZIDriver::PathRelation(posS_host, distance, hostPath_velocity, targetRelation);
}

void FZIDriver::TrafficAgentBase::setRelationToTargetPath(int targetID) {
    (*this->targetPathState)[targetID] = this->calculatePathRelation(targetID, this->pathState);
}

FZIDriver::PathRelation FZIDriver::TrafficAgentBase::getRelationToTargetPath(int targetID, int step) {
    return this->prediction_pathRelation[targetID][step];
}

unsigned int FZIDriver::TrafficAgentBase::calculateCurrentLaneID(double posX, double posY) {
    FZIDriver::util::Lane currentPath;
    std::tie(currentPath, std::ignore) = this->laneNetwork->findClosestLaneWithPosition(posX, posY);
    return currentPath.id;
}

void FZIDriver::TrafficAgentBase::setRelevantTargets(std::vector<unsigned int> consideredTargets) {
    std::vector<unsigned int> relevantTargets = std::vector<unsigned int>();

    std::map<int, FZIDriver::PathRelation> hostPathSateMap = std::map<int, FZIDriver::PathRelation>();
    //find the crossing targets and the targets on the same path
    std::vector<int> targets_onHostPath = std::vector<int>();
    std::map<int, std::vector<int>> targets_crossing = std::map<int, std::vector<int>>();
    for (unsigned int targetID: consideredTargets) {

        if (targetID == this->agentID) {
            // the target itself is not relevant
            continue;
        }

        // calculate the state of the target on the host path
        auto hostPathState_target = this->trafficParticipantMap[targetID]->calculatePathRelation(this->agentID, this->pathState);
        hostPathSateMap[targetID] = hostPathState_target;

        if (std::get<3>(hostPathState_target) == FZIDriver::enums::TargetRelation::IDENTICAL_PATH) {
            targets_onHostPath.push_back(targetID);
        } else {
            if (std::get<3>(hostPathState_target) == FZIDriver::enums::TargetRelation::CROSSING) {
                // store the crossing targets on their current lane
                if (targets_crossing.find(this->trafficParticipantMap[targetID]->getCurrentLaneID()) != targets_crossing.end()) {
                    targets_crossing.insert(std::pair<int, std::vector<int>>(this->trafficParticipantMap[targetID]->getCurrentLaneID(), std::vector<int>()));
                }
                targets_crossing[this->trafficParticipantMap[targetID]->getCurrentLaneID()].push_back(targetID);

            } else {
                // In this case the target does neither cross the host path nor is on the same path
                // therefore it can be ignored
                continue;
            }
        }
    }


    // find the closest target on the same path
    int relevantTarget_onPath = -1;
    double distanceToTarget_onPath = std::numeric_limits<double>::max();
    for (int targetID :targets_onHostPath) {
        // only targets in front of the host vehicle are relevant
        // Note: This only works for lanes where s is increasing in driving direction
        auto hostPath_state = hostPathSateMap[targetID];
        double posS = std::get<0>(this->pathState);
        if (std::get<0>(hostPath_state) > posS && abs(std::get<0>(hostPath_state) - posS) < distanceToTarget_onPath) {
            relevantTarget_onPath = targetID;
            distanceToTarget_onPath = abs(std::get<0>(hostPath_state) - posS);
        }
    }


    // find the closest target that crosses the path
    std::vector<unsigned int> relevantTarget_crossing = std::vector<unsigned int>();
    int closestCrossingTarget_id = -1;
    double closestCrossingTarget_distance = std::numeric_limits<double>::max();
    bool closestCrossingTarget_approaches = false;
    for (auto laneTargets : targets_crossing) {
        int laneID = laneTargets.first;
        std::vector<int> targetIDList = laneTargets.second;
        if (this->laneNetwork->getLaneNetworkElement(laneID).laneType == FZIDriver::enums::LaneType::intersection) {
            // add all targets on the intersection the relavant targets
            relevantTarget_crossing.insert(relevantTarget_crossing.end(), targetIDList.begin(), targetIDList.end());
            // iterate over all targets
            for (int targetID : targetIDList) {
                auto hostPath_state = hostPathSateMap[targetID];
                if (abs(std::get<1>(hostPath_state)) < closestCrossingTarget_distance) {
                    closestCrossingTarget_distance = abs(std::get<1>(hostPath_state));
                    closestCrossingTarget_id = targetID;
                    // check if the target is approaching
                    if (std::get<2>(hostPath_state) < 0) {
                        closestCrossingTarget_approaches = true;
                    }
                }

            }
        } else {
            int relevantTargetOnLane_id = -1;
            double relevantTargetOnLane_distance = std::numeric_limits<double>::max();
            bool relevantTargetLane_approaching = false;
            // find the target with the closest distance on the given lane
            for (int targetID : targetIDList) {
                auto hostPath_state = hostPathSateMap[targetID];
                if (abs(std::get<1>(hostPath_state)) < relevantTargetOnLane_distance) {
                    relevantTargetOnLane_id = targetID;
                    relevantTargetOnLane_distance = abs(std::get<1>(hostPath_state));
                    relevantTargetLane_approaching = std::get<1>(hostPath_state) < 0 ? true : false;
                }
            }
            // add the closest target on the lane to the relevant crossing targets
            relevantTarget_crossing.push_back(relevantTargetOnLane_id);
            // check if the target is closer than the previous closest target
            if (relevantTargetOnLane_distance < closestCrossingTarget_distance) {
                closestCrossingTarget_id = relevantTargetOnLane_id;
                closestCrossingTarget_distance = relevantTargetOnLane_distance;
                closestCrossingTarget_approaches = relevantTargetLane_approaching;
            }
        }
    }

    // use closest targets crossing and on the same path to determine the relevant target
    if (relevantTarget_crossing.size() > 0 && relevantTarget_onPath != -1) {
        if (!closestCrossingTarget_approaches) {
            // only the target on the same path is relevant
            relevantTargets.push_back(relevantTarget_onPath);
        } else {
            // all crossing targets are relevant
            relevantTargets.insert(relevantTargets.end(), relevantTarget_crossing.begin(), relevantTargets.end());
            // check if target on path is inbetween the closest crossing point and the vehicle
            auto hostPathState_onPath = hostPathSateMap[relevantTarget_onPath];
            auto hostPathState_crossing = hostPathSateMap[closestCrossingTarget_id];
            if (std::get<0>(hostPathState_onPath) < std::get<0>(hostPathState_crossing)) {
                relevantTargets.push_back(relevantTarget_onPath);
            }
        }
    } else {
        if (relevantTarget_crossing.size() > 0) {
            // add all crossing targets
            relevantTargets.insert(relevantTargets.end(), relevantTarget_crossing.begin(), relevantTarget_crossing.end());
        }
        if (relevantTarget_onPath != -1) {
            relevantTargets.push_back(relevantTarget_onPath);
        }
    }

    this->relevanTargets = relevantTargets;
}

std::tuple<double, double> FZIDriver::TrafficAgentBase::getGlobalPosition(int step) {
    return this->prediction_globalPosition[step];
}

double FZIDriver::TrafficAgentBase::mapToPath(FZIDriver::util::State state) {
    return std::get<0>(this->agentPath->mapToPath(state.posX, state.posY));
}

void FZIDriver::TrafficAgentBase::setPriorityOver(int targetID) {
    // get the orientation of the intersection at the entrance point of the host
    int hostEntranceLaneID = this->trafficParticipantMap[targetID]->getInitialLaneID();
    FZIDriver::util::Path hostEntrancePath = this->laneNetwork->getLane(hostEntranceLaneID).path;
    double intersectionEntranceOrientation_host = hostEntrancePath.getHeading(hostEntrancePath.getMaxS());
    // get the orienataion of the intersection at the entrance point of the target
    auto entrancePath_target = this->laneNetwork->getLane(this->getInitialLaneID()).path;
    double intersectionEntranceOrientation_target = entrancePath_target.getHeading(entrancePath_target.getMaxS());

    FZIDriver::enums::Priority priority;
    // check if the agents are comming from opposing directions or orthogonal roads
    if (std::abs(std::abs(intersectionEntranceOrientation_target - intersectionEntranceOrientation_host) - M_PI) < 0.1) {
        // the vehicles come from opposing roads
        FZIDriver::enums::TurningDirction hostTurningDirection = this->trafficParticipantMap[targetID]->getTurningDirection();
        if (hostTurningDirection == FZIDriver::enums::TurningDirction::left && this->turningDirection == FZIDriver::enums::TurningDirction::left) {
            priority = FZIDriver::enums::Priority::UNDEFINED;
        } else if (hostTurningDirection == FZIDriver::enums::TurningDirction::left) {
            priority = FZIDriver::enums::Priority::TRUE;
        } else {
            priority = FZIDriver::enums::Priority::FALSE;
        }
    } else {
        // the vehicles come from orthogonal roads
        if (intersectionEntranceOrientation_target - intersectionEntranceOrientation_host > 0) {
            priority = FZIDriver::enums::Priority::TRUE;
        } else {
            priority = FZIDriver::enums::Priority::FALSE;
        }
    }
    // store the priority information
    (*this->targetPriority)[targetID] = priority;
}

std::tuple<double, std::deque<std::shared_ptr<FZIDriver::maneuver::ManeuverNodeInterface>>> FZIDriver::TrafficAgentBase::determineManeuver(int step, std::tuple<double, double> state,
                                                                                                                                  std::vector<FZIDriver::TrafficAgentBase> &trafficParticipants,
                                                                                                                                  FZIDriver::maneuver::ManeuverNodeInterface &maneuverIn) {

    // check if the algorithm has reached the final step
    if (step == this->predictionHorizonLength) {
        return std::tuple<double, std::deque<std::shared_ptr<FZIDriver::maneuver::ManeuverNodeInterface>>>(this->costToGo(step, state, trafficParticipants), std::deque<std::shared_ptr<FZIDriver::maneuver::ManeuverNodeInterface>>());
    }

    // iterate over all maneuver options
    std::vector<double> performance;
    std::vector<std::deque<std::shared_ptr<FZIDriver::maneuver::ManeuverNodeInterface>>> maneuverOptions;
    for(auto nextManeuver: maneuverIn.getManeuverOptions()) {
        // calculate the cost-to-go
        double costToGo = this->costToGo(step, state, trafficParticipants);

        auto globalPosition = this->agentPath->getPosition(std::get<0>(state));
        // determine the current path
        FZIDriver::util::Lane currentLane;
        double currentLane_posS;
        std::tie(currentLane, currentLane_posS) = this->laneNetwork->findClosestLaneWithPosition(std::get<0>(globalPosition), 
                                                                                                 std::get<1>(globalPosition));

        // check if the current path leads to a junction ...
        // ... and the 
        double distanceToStopLine = std::numeric_limits<double>::max(); 
        if (std::get<1>(currentLane.successorLane) == FZIDriver::enums::LaneType::intersection) {
            distanceToStopLine = currentLane.path.getMaxS() - currentLane_posS;
        }


        // ToDo: Move to the cost to go function
        double currentAcceleration = maneuverIn.getAcceleration(std::get<1>(state), distanceToStopLine);
        double nextAcceleration = nextManeuver->getAcceleration(std::get<1>(state), distanceToStopLine);
        costToGo += this->weights.maneuverChangePenalty * sqrt(pow((currentAcceleration - nextAcceleration) / (this->stepSize * this->oversampling),2));

        // predict the state using the next maneuver option
        std::tuple<double, double> state_pred(std::get<0>(state) + this->stepSize * this->oversampling * std::get<1>(state),
                                              std::max(0.0, std::get<1>(state) + this->stepSize * this->oversampling * nextAcceleration));

        // check of the predicted position exceeds the lenth of the path
        if (std::get<0>(state_pred) > this->agentPath->getMaxS()) {
            return std::tuple<double, std::deque<std::shared_ptr<FZIDriver::maneuver::ManeuverNodeInterface>>>(0.0, std::deque<std::shared_ptr<FZIDriver::maneuver::ManeuverNodeInterface>>());
        }

        // perform the dynamic programming step
        auto result = this->determineManeuver(step + 1, state_pred, trafficParticipants, *nextManeuver);
        double performance_loc = std::get<0>(result);
        std::deque<std::shared_ptr<FZIDriver::maneuver::ManeuverNodeInterface>> maneuverOptions_loc = std::get<1>(result);
        maneuverOptions_loc.push_front(nextManeuver);

        // store the performance in vector
        performance.push_back(costToGo + performance_loc);
        // store the maneuver sequence
        maneuverOptions.push_back(maneuverOptions_loc);

    }

    // find the option with the minimal cost
    int minIdx = std::min_element(performance.begin(), performance.end()) - performance.begin();

    return std::tuple<double, std::deque<std::shared_ptr<FZIDriver::maneuver::ManeuverNodeInterface>>>(performance[minIdx], maneuverOptions[minIdx]);
}

double FZIDriver::TrafficAgentBase::costToGo(int step, std::tuple<double, double> state, std::vector<FZIDriver::TrafficAgentBase> &trafficParticipants) {
    double velocity_cost = sqrt(pow(std::get<1>(state) - this->vRef, 2));
    double distance_cost = -std::get<1>(state) * (this->stepSize * this->oversampling);

    std::tuple<double, double> globalPosition = this->agentPath->getPosition(std::get<0>(state));

    double cost = this->weights.refVelocityPenalty * velocity_cost + this->weights.distancePenalty * distance_cost;
    for (auto trafficParticipant: trafficParticipants) {

        FZIDriver::PathRelation participantState = trafficParticipant.getRelationToTargetPath(this->agentID, step);
        std::tuple<double, double> globalPosition_target = trafficParticipant.getGlobalPosition(step);

        double distX = abs(std::get<0>(globalPosition) - std::get<0>(globalPosition_target));
        double distY = abs(std::get<1>(globalPosition) - std::get<1>(globalPosition_target));
        double dist = sqrt(pow(distX,2) + pow(distY,2));

        // check warning and crash conditions
        if (distX < std::get<0>(this->weights.warningZone) && distY < std::get<1>(this->weights.warningZone)) {
            double direction = 0;
            if (std::get<3>(participantState) == FZIDriver::enums::TargetRelation::IDENTICAL_PATH) {
                direction = (std::get<0>(state) - std::get<0>(participantState)) / (abs(std::get<0>(state) - std::get<0>(participantState)) + 1e-4);
            }

            cost += this->weights.warningPenalty * (std::get<0>(this->weights.warningZone) / (dist + 10e-3));
            cost = (direction > 0) ? (cost + 10) : cost;

            if (distX < std::get<0>(this->weights.crashZone) && distY < std::get<0>(this->weights.crashZone)) {
                cost += this->weights.crashPenalty * (std::get<0>(this->weights.crashZone) / (dist + 10e-3));
            }
        }

        // check priority conditions
        double costPriority = 0;
        if ((*this->targetPriority)[trafficParticipant.agentID] == FZIDriver::enums::Priority::TRUE) {
            double participantVelocity = trafficParticipant.getPredictedVelocity(step);
            costPriority = (participantVelocity - std::get<1>(state)) / (abs(participantVelocity - std::get<1>(state)) + 1e-6);
        } 
        if ((*this->targetPriority)[trafficParticipant.agentID] == FZIDriver::enums::Priority::FALSE) {
            double participantVelocity = trafficParticipant.getPredictedVelocity(step);
            costPriority = (std::get<1>(state) - participantVelocity) / (abs(participantVelocity - std::get<1>(state)) + 1e-6);
        }
        cost += this->weights.priorityPenalty * costPriority;
    }

    return cost;
}

void FZIDriver::TrafficAgentBase::createInitialManeuverSequence() {
    this->prediction_maneuver = std::deque<std::shared_ptr<FZIDriver::maneuver::ManeuverNodeInterface>>();
    for (int i = 0; i < this->predictionHorizonLength; i++) {
        // Note: Initialize the maneuver strategy with the first entry in the graph
        // -> this souldn't have influence on the result over identifying the current maneuver
        this->prediction_maneuver.push_back(this->maneuverGraph.getManeuverNode(0));
    }
}

void FZIDriver::TrafficAgentBase::createPrediction() {
    // initialize the predictions
    this->prediction_pathState = std::deque<std::tuple<double, double>>();
    this->prediction_pathRelation = std::map<unsigned int, std::deque<FZIDriver::PathRelation>>();
    this->prediction_globalPosition = std::deque<std::tuple<double, double>>();
    // create vector to store prediction on the path
    this->prediction_pathState.push_back(this->pathState);
    // calculate the prediction on the horizon
    for (int i = 0; i < this->predictionHorizonLength; i++) {
        // define the s position based on the s coordinate
        double posS = std::get<0>(this->prediction_pathState[i]);
        double velocity = std::get<1>(this->prediction_pathState[i]);

        auto globalPosition = this->agentPath->getPosition(posS);
        // determine the current path
        FZIDriver::util::Lane currentLane;
        double currentLane_posS;
        std::tie(currentLane, currentLane_posS) = this->laneNetwork->findClosestLaneWithPosition(std::get<0>(globalPosition), std::get<1>(globalPosition));

        // check if the current path leads to a junction ...
        // ... and the 
        double distanceToStopLine = std::numeric_limits<double>::max(); 
        if (std::get<1>(currentLane.successorLane) == FZIDriver::enums::LaneType::intersection) {
            distanceToStopLine = currentLane.path.getMaxS() - currentLane_posS;
        }


        // update the estimation of the position
        double posS_estimate = posS + velocity * stepSize;
        // update the estimation of the velocity
        // ToDo: Implement distance to stop line if needed
        double velocity_estimate = velocity + stepSize * this->prediction_maneuver[i]->getAcceleration(velocity, distanceToStopLine);
        velocity_estimate = std::max(velocity_estimate, 0.0); // exclude negative velocities
        // store the prediction
        this->prediction_pathState.push_back(std::tuple<double, double>(posS_estimate, velocity_estimate));
    }

    
    for (int i = 0; i <= predictionHorizonLength; i++) {
        prediction_globalPosition.push_back(this->agentPath->getPosition(std::get<0>(this->prediction_pathState[i])));
        for(auto targetEntry: this->trafficParticipantMap) {
            if (targetEntry.first != this->agentID) {
                this->prediction_pathRelation[targetEntry.first].push_back(this->calculatePathRelation(targetEntry.first, this->prediction_pathState[i]));    
            }
        }
    }
    return;

}