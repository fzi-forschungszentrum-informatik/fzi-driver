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

#include <algorithm>
#include <fstream>

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include "TrafficAgent.h"
#include "util/MathUtil.h"

FZIDriver::TrafficAgent::TrafficAgent() {
    // set the maximum number of iterations
    this->maxIterations = 4;

    // set the agent to not initialized
    this->initialzed = false;
}

FZIDriver::TrafficAgent::~TrafficAgent() {
    // clean up the path map
    this->trafficParticipantMap.clear();
}

int FZIDriver::TrafficAgent::init() {
    // set the s position to be undefined
    this->pathState = std::tuple<double, double>(std::nan(""), std::nan(""));
    // define the agent to be initialized
    this->initialzed = true;

    return 0;
}

int FZIDriver::TrafficAgent::init(osi3::SensorView sensorView) {
    // call the basic init method
    this->init();

    // create the lane network from the sensor view
#if TARGET_OPEN_PASS
    this->laneNetwork = std::make_shared<FZIDriver::util::LaneNetwork>(FZIDriver::util::LaneNetwork::createLaneNetworkFromOpenPassSensorView(sensorView));
#else
    this->laneNetwork = std::make_shared<FZIDriver::util::LaneNetwork>(FZIDriver::util::LaneNetwork::createLaneNetworkFromSensorView(sensorView));
#endif

    return 0;
}

int FZIDriver::TrafficAgent::step(double stepSize, osi3::SensorView sensorView, osi3::TrafficCommand trafficCommand, osi3::TrafficUpdate &trafficUpdateOut) {
    // set the current step size
    this->stepSize = stepSize;

    // initalized the agent if necessary
    if (!this->initialzed) {
        this->init(sensorView);
    }

    // ToDo: Check wether this can be done in the init step
    // read the id of the traffic agent
    this->agentID = sensorView.host_vehicle_id().value();
    // add the agent itself to the traffic participant map
    auto pointer = std::dynamic_pointer_cast<FZIDriver::TrafficAgent>(shared_from_this());
    this->trafficParticipantMap[this->agentID] = pointer;

    // find the moving object message for the host vehicle
    osi3::MovingObject movingObject_host;
    FZIDriver::util::OSIUtil::getMovingObjectByID(sensorView, this->agentID, movingObject_host);

    // set the state of the agent if the s position is not defined yet
    if (std::isnan(std::get<0>(this->pathState))) {
        this->state = FZIDriver::util::State(sensorView.host_vehicle_data().location());
    }

    // check if a speed action is present in the traffic command
    for(osi3::TrafficAction trafficAction: trafficCommand.action()) {
        if (trafficAction.has_speed_action()) {
            this->vRef = trafficAction.speed_action().absolute_target_speed();
        }        
    }

    // read the path centerline from the traffic command
    // it is assumed that the traffic agent contains one path action
    bool pathCreated = this->createPathFromTrafficCommand(trafficCommand);
    if (pathCreated) {

        // map the current position to the path
        if (std::isnan(std::get<0>(this->pathState))) {
            double posS;
            std::tie(posS, std::ignore) = this->agentPath->mapToPath(this->state.posX, this->state.posY);
            this->pathState = std::tuple<double, double>(posS, this->state.velocity);
        }
        
        // set the current lane id
        auto state = this->agentPath->getPosition(std::get<0>(this->pathState));
        this->currentLaneID = this->calculateCurrentLaneID(std::get<0>(state), std::get<1>(state));

        // iterate over all traffic participants
        for (osi3::MovingObject movingObject: sensorView.global_ground_truth().moving_object()) {
            if (movingObject.id().value() != this->agentID) {
                int trafficParticipantID = movingObject.id().value();

                // check if the traffic participant is already in the map
                if (this->trafficParticipantMap.find(trafficParticipantID) == this->trafficParticipantMap.end()) {
                    auto tp = std::make_shared<FZIDriver::TrafficAgentBase>(FZIDriver::TrafficAgentBase(movingObject, this->vRef, this->stepSize,
                                                                                                        this->laneNetwork, this->trafficParticipantMap));
                    this->trafficParticipantMap[trafficParticipantID] = tp;
                
                    // store the target information 
                    this->addTargetPath(trafficParticipantID, tp->getAgentPath());
                    this->setRelationToTargetPath(trafficParticipantID);
                    // calculate the priority
                    this->setPriorityOver(trafficParticipantID);
                } else {
                    this->trafficParticipantMap.at(trafficParticipantID)->updatePosition(movingObject);
                }
            }

        }

        std::vector<unsigned int> targetIDs = std::vector<unsigned int>();
        for (auto entry: this->trafficParticipantMap) {
            if (entry.first != this->agentID) {
                targetIDs.push_back(entry.first);
            }
        }

        // set the relevant targets for the traffic agent and for all traffic participants
        this->setRelevantTargets(targetIDs);
        std::vector<unsigned int> consideresTargets = this->relevanTargets;
        consideresTargets.push_back(this->agentID);
        for (auto &trafficParticipant: this->trafficParticipantMap) {
            if (trafficParticipant.first != this->agentID) {
                trafficParticipant.second->setRelevantTargets(consideresTargets);
                break;
            }
        }

        // call the IBR algorithm to determine the switching strategy
        this->iterativeBestResponse();

        // determine the current path
        FZIDriver::util::Lane currentLane;
        double currentLane_posS;
        std::tie(currentLane, currentLane_posS) = this->laneNetwork->findClosestLaneWithPosition(this->state.posX, this->state.posY);

        // check if the current path leads to a junction ...
        // ... and the 
        double distanceToStopLine = std::numeric_limits<double>::max(); 
        if (std::get<1>(currentLane.successorLane) == FZIDriver::enums::LaneType::intersection) {
            distanceToStopLine = currentLane.path.getMaxS() - currentLane_posS;
        }

        // calculate the acceleration
        this->acceleration = this->prediction_maneuver.at(0)->getAcceleration(this->state.velocity, distanceToStopLine);
        // update the s position
        // ToDo: Find a more sophisticated way to deal with vehicle leaving the road network
        double posS_pred = std::min(std::get<0>(this->pathState) + this->stepSize * std::get<1>(this->pathState), this->agentPath->getMaxS() - 5.0);
        // update the velocity
        double velocity_pred = std::max(0.0, std::get<1>(this->pathState) + this->stepSize * acceleration);

        // update the state
        this->state.posX = this->agentPath->getPosX(posS_pred);
        this->state.posY = this->agentPath->getPosY(posS_pred);
        this->state.velocity = velocity_pred;
        this->state.yaw = this->agentPath->getHeading(posS_pred);
        // set the new state on the path
        this->pathState = std::tuple<double, double>(posS_pred, velocity_pred);
    } 

    // set the traffic update output
    this->createTrafficUpdate(trafficUpdateOut, sensorView.timestamp());
    return 0;
}

void FZIDriver::TrafficAgent::iterativeBestResponse() {
    // initialize a maneuver sequence for the traffic agent itself and all relevant targets
    this->createInitialManeuverSequence();
    this->createPrediction();
    for (unsigned int trafficParticipantID: this->relevanTargets) {
        this->trafficParticipantMap[trafficParticipantID]->createInitialManeuverSequence();
        this->trafficParticipantMap[trafficParticipantID]->createPrediction();
    }

    for (int iteration = 0; iteration < this->maxIterations; iteration++) {
        for (unsigned int trafficParticipantID: this->relevanTargets) {
            this->trafficParticipantMap[trafficParticipantID]->iterationStep();
        }

        this->iterationStep();
    }
}

bool FZIDriver::TrafficAgent::createPathFromTrafficCommand(osi3::TrafficCommand trafficCommand) {

    // iterate over all traffic commands
    for (osi3::TrafficAction action: trafficCommand.action()) {
        if (action.has_follow_path_action()) {
            unsigned int actionID = action.follow_path_action().action_header().action_id().value();

            // check if the agent path is already set
            if (this->agentPath) {
                // check if the current is identical to the currently stored id
                if (actionID == this->trafficCommandID) {
                    return true;
                }
            }

#if TARGET_OPEN_PASS
            std::vector<FZIDriver::util::Path> laneOnPath;
            unsigned int lastID = -1;
            for (osi3::StatePoint pathPoint: action.follow_path_action().path_point()) {
                FZIDriver::util::Lane current;
                std::tie(current, std::ignore) = this->laneNetwork->findClosestLaneWithPosition(pathPoint.position().x(), 
                                                                                                pathPoint.position().y());
                if (current.id != lastID) {
                    lastID = current.id;
                    laneOnPath.push_back(current.path);
                }
            }

            // create the centerline
            std::vector<FZIDriver::util::Vector2d<double>> centerLine;
            for (auto path: laneOnPath) {
                for(unsigned int i = 0; i < path.getXCoordinates().size(); i++) {
                    centerLine.push_back(FZIDriver::util::Vector2d<double>(path.getXCoordinates().at(i),
                                                                           path.getYCoordinates().at(i)));
                }
            }
#else
            // define centerline storage
            std::vector<FZIDriver::util::Vector2d<double>> centerLine;
            for (osi3::StatePoint pathPoint: action.follow_path_action().path_point()) {
                centerLine.push_back(FZIDriver::util::Vector2d<double>(pathPoint.position()));
            }
#endif
            // create a data pair containing the path
            this->agentPath = std::make_shared<FZIDriver::util::Path>(FZIDriver::util::Path(centerLine));
            this->trafficCommandID = actionID;
            // as the path has changed the the s position is no longer defined
            this->pathState = std::tuple<double, double>(std::nan(""), std::nan(""));

            // get the initial lane
            auto pos_init = this->agentPath.get()->getPosition(0);
            auto initialLane = std::get<0>(this->laneNetwork->findClosestLaneWithPosition(std::get<0>(pos_init), std::get<1>(pos_init)));
            this->initialLaneID = initialLane.id;
            // get the final lane
            auto pos_final = this->agentPath.get()->getPosition(this->agentPath.get()->getMaxS());
            auto finalLane = std::get<0>(this->laneNetwork->findClosestLaneWithPosition(std::get<0>(pos_final), std::get<1>(pos_final)));
            this->finalLaneID = finalLane.id;

            std::deque<FZIDriver::util::Lane> intermediateLanes;
            this->laneNetwork->findConnection(initialLane, finalLane, intermediateLanes, this->turningDirection);

            return true;
        }
        if(action.has_acquire_global_position_action()){
            unsigned int actionID = action.acquire_global_position_action().action_header().action_id().value();

            // check if the agent path is already set
            if (this->agentPath) {
                // check if the current is identical to the currently stored id
                if (actionID == this->trafficCommandID) {
                    return true;
                }
            }

            //then get the initiallane like the method fom follow traffic agent
            // get the initial lane
            // -> use the current position as the initial one
            auto initialLane = std::get<0>(this->laneNetwork->findClosestLaneWithPosition(this->state.posX, this->state.posY));
            this->initialLaneID = initialLane.id;
            // get or calculate the global position oder final position von the osi.trafficcommand.proto
            auto pos_x = action.acquire_global_position_action().position().x();
            auto pos_y = action.acquire_global_position_action().position().y();

            //then we have to find the finallane
            auto finalLane = std::get<0>(this->laneNetwork->findClosestLaneWithPosition(pos_x, pos_y));

            // find the intermediate lanes
            std::deque<FZIDriver::util::Lane> intermediateLanes;
            this->laneNetwork->findConnection(initialLane, finalLane, intermediateLanes, this->turningDirection);

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
            this->trafficCommandID = actionID;

            return true;
        }
    }

    // check if a path already exists
    if (this->agentPath) {
        // in this case a path already exists
        // -> therefore we can return true
        return true;
    } else {
        return false;
    }
}

void FZIDriver::TrafficAgent::createTrafficUpdate(osi3::TrafficUpdate &trafficUpdate, osi3::Timestamp simulationTime) {
    // set the timestamp
    trafficUpdate.mutable_timestamp()->CopyFrom(simulationTime);

    
    // read the corresponding moving object from the traffic update
    osi3::MovingObject *movingObject = nullptr;
    for(int i = 0; i < trafficUpdate.update().size(); i++) {
        if (trafficUpdate.update(i).id().value() == this->agentID) {
            movingObject = trafficUpdate.mutable_update(i);
        }
    } 
    // check if a moving object was found and create one if not
    if (movingObject == nullptr) {
        movingObject = trafficUpdate.add_update();
    } 

    // set the id if the agent
    movingObject->mutable_id()->set_value(this->agentID);

    movingObject->set_type(osi3::MovingObject_Type_TYPE_VEHICLE);
    // get the base moving of the update
    osi3::BaseMoving* baseMoving = movingObject->mutable_base();
    // set the type of the agent
    //baseMoving->
    // set the x and y postion of the vehicle
    baseMoving->mutable_position()->set_x(this->state.posX);
    baseMoving->mutable_position()->set_y(this->state.posY);
    baseMoving->mutable_position()->set_z(0.0);
    // set the velcoties in x and y direction
    double vx = this->state.velocity * cos(this->state.yaw);
    baseMoving->mutable_velocity()->set_x(vx);
    double vy = this->state.velocity * sin(this->state.yaw);
    baseMoving->mutable_velocity()->set_y(vy);
    baseMoving->mutable_velocity()->set_z(0.0);
    // set the acceleraion
    double ax = this->acceleration * cos(this->state.yaw);
    baseMoving->mutable_acceleration()->set_x(ax);
    double ay = this->acceleration * sin(this->state.yaw);
    baseMoving->mutable_acceleration()->set_y(ay);
    baseMoving->mutable_acceleration()->set_z(0.0);
    // set the orientation of the vehicle
    baseMoving->mutable_orientation()->set_roll(0.0);
    baseMoving->mutable_orientation()->set_pitch(0.0);
    baseMoving->mutable_orientation()->set_yaw(this->state.yaw);
}
