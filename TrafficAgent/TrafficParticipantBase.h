/**
 * @file TrafficParticipantBase.h
 * @author Markus Lemmer
 * @brief This class holds all relevant functionality to simulate a driver/traffic participant.
 * 
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
 * 
 */
#ifndef TRAFFIC_PARTICIPANT_BASE
#define TRAFFIC_PARTICIPANT_BASE

#include <deque>

#include "maneuvers/ManeuverGraphInterface.h"
#include "maneuvers/BasicManeuverGraph.h"
#include "maneuvers/IDMManeuberGraph.h"
#include "util/LaneNetwork.h"
#include "util/OSIUtil.h"
#include "util/Path.h"
#include "util/State.h"

namespace FZIDriver {

    typedef std::tuple<double, double, double, FZIDriver::enums::TargetRelation> PathRelation;
    
    /**
     * @brief Storage class for the weights and performance index parameters of a traffic participant. 
     * 
     */
    class Weights {
        public:
            double distancePenalty;
            double crashPenalty;
            double warningPenalty;
            double priorityPenalty;
            double refVelocityPenalty;
            double maneuverChangePenalty;

            // size of the warning zone
            std::tuple<double, double> warningZone;
            // size of the crash zone
            std::tuple<double, double> crashZone;

            Weights() {
                this->distancePenalty = 0.1;
                this->crashPenalty = 200;
                this->warningPenalty = 30;
                this->priorityPenalty = 40;
                this->refVelocityPenalty = 6;
                this->maneuverChangePenalty = 1;

                // set the size of the warning zone
                this->warningZone = std::tuple<double, double>(25, 25);
                // set the size of the crash zone
                this->crashZone = std::tuple<double, double>(7.5, 7.5);
            }
    };

    /**
     * @brief Traffic Participant Base Class.
     * This class can be used to represent driving traffic participants using game-theoretic planning based decision making.
     * 
     */
    class TrafficAgentBase : public std::enable_shared_from_this<TrafficAgentBase> {
        protected:
            //simulation step size
            double stepSize;
            // prediction horizon of the dynamic programming algorithm
            int predictionHorizonLength;
            // oversampling
            int oversampling;
            
            // reference velocity
            double vRef;
            // length of the vehicle
            // -> distance between the center points of the front and the rear axle
            double vehicleLength;
            // store the weights of the performance index
            Weights weights;

            // unique identifier of the agent in the simulation framework
            unsigned int agentID;

            // store all paths in the initial sensor view
            std::shared_ptr<FZIDriver::util::LaneNetwork> laneNetwork;
            // store the path of the agent
            std::shared_ptr<FZIDriver::util::Path> agentPath;
            // store the position and velocity on the path
            std::tuple<double, double> pathState;

            // store a maneuver graph
            FZIDriver::maneuver::ManeuverGraphInterface maneuverGraph;

            // store the id of the current lane
            int currentLaneID;
            // store the initial and the final lane information
            unsigned int initialLaneID;
            unsigned int finalLaneID;

            // store the turning direction
            FZIDriver::enums::TurningDirction turningDirection;

            // store information about the traffic participants
            std::map<unsigned int, std::shared_ptr<TrafficAgentBase>> trafficParticipantMap;
            // store the relevant targets
            std::vector<unsigned int> relevanTargets;

            // store intersection coordinates
            std::shared_ptr<std::map<unsigned int, std::shared_ptr<FZIDriver::util::Path>>> targetPathMap;
            std::shared_ptr<std::map<unsigned int, PathRelation>> targetPathState;
            std::shared_ptr<std::map<unsigned int, std::tuple<double, double, double>>> intersectionCoordinateMap;
            // store the priority of the targets
            // true if this has priority over the target with id
            std::shared_ptr<std::map<unsigned int, FZIDriver::enums::Priority>> targetPriority;

            std::deque<std::shared_ptr<FZIDriver::maneuver::ManeuverNodeInterface>> prediction_maneuver;
            std::deque<std::tuple<double, double>> prediction_pathState;
            std::deque<std::tuple<double, double>> prediction_globalPosition;
            std::map<unsigned int, std::deque<PathRelation>> prediction_pathRelation;

            /**
             * @brief Determines the lanes id based on the supplies global position
             * 
             * @param posX x-position in the global coordinate frame
             * @param posY y-position in the global coordinate fream
             * @return unsigned int id of the lane that is closest to the supplied global position
             */
            unsigned int calculateCurrentLaneID(double posX, double posY);
            /**
             * @brief Set the Priority over a given traffic participant.
             * The result is stored in the field "targetPriority".
             * @param targetID Unique ID of the targate. Note that the target must be part of trafficParticipantMap.
             */
            void setPriorityOver(int targetID);

            /**
             * @brief Determines the path relation to a specified target.
             * 
             * @param targetID Unique ID of the targate. Note that the target must be part of trafficParticipantMap.
             * @param pathState The current state of the TrafficParticipant on his path.
             * @return PathRelation The relation between to the specified targed in the provided state.
             */
            PathRelation calculatePathRelation(int targetID, std::tuple<double, double> pathState);
            /**
             * @brief Determine the relation to the path of a specific target
             * 
             * @param targetID The unique id of the target.
             */
            void setRelationToTargetPath(int targetID);
            /**
             * @brief Get the Relation to a Target Path at a prediction step.
             * 
             * @param targetID The unique id of the target.
             * @param step  The prediction step
             * @return PathRelation
             */
            PathRelation getRelationToTargetPath(int targetID, int step);

            /**
             * @brief Calculate the optimal maneuver sequence using dynamic programming
             * 
             * @param step The iteration step of the dynamic programming.
             * @param state The current state if the traffic participnat: <position of path, velocity> 
             * @param trafficParticipantMap A map of all traffic participants
             * @param maneuverIn The current maneuver selection.
             * @return std::tuple<double, std::deque<std::shared_ptr<FZIDriver::maneuver::ManeuverNodeInterface>>> 
             */
            std::tuple<double, std::deque<std::shared_ptr<FZIDriver::maneuver::ManeuverNodeInterface>>> determineManeuver(int step, std::tuple<double, double> state, 
                                                                                                                 std::vector<TrafficAgentBase> &trafficParticipantMap, 
                                                                                                                 FZIDriver::maneuver::ManeuverNodeInterface &maneuverIn);
            /**
             * @brief Calculates the cost to go.
             * 
             * @param step The current iteration step.
             * @param state The current state if the traffic participnat: <position of path, velocity>
             * @param trafficParticipants A list of all relevant traffic participants.
             * @return double 
             */
            double costToGo(int step, std::tuple<double, double> state, std::vector<TrafficAgentBase> &trafficParticipants);

        public:
            /**
             * @brief Construct a new Traffic Agent Base object
            * 
            */
            TrafficAgentBase(double stepSIze_in = 0.1) {
                // set the prediction hotizon
                this->predictionHorizonLength = 4;
                // set the oversampling factor
                this->oversampling = 8;
                // set the reference velocity
                this->vRef = 5;
                // set the simulation step size
                this->stepSize = stepSIze_in;

                // set the length of the vehicle
                this->vehicleLength = 1.5;
                // create a new weights object
                this->weights = Weights();

                // define the maneuver graph
                #if USE_IDM_MANEUVERS
                this->maneuverGraph = FZIDriver::maneuver::IDMManeuverGraph(this->vRef);
                #else
                this->maneuverGraph = FZIDriver::maneuver::BasicManeuverGraph();
                #endif

                // intialize the different maps that store the target data
                this->targetPathMap = std::make_shared<std::map<unsigned int, std::shared_ptr<FZIDriver::util::Path>>>(std::map<unsigned int, std::shared_ptr<FZIDriver::util::Path>>());
                this->targetPathState = std::make_shared<std::map<unsigned int, PathRelation>>(std::map<unsigned int, PathRelation>());
                this->intersectionCoordinateMap = std::make_shared<std::map<unsigned int, std::tuple<double, double, double>>>(std::map<unsigned int, std::tuple<double, double, double>>());
                this->targetPriority = std::make_shared<std::map<unsigned int, FZIDriver::enums::Priority>>(std::map<unsigned int, FZIDriver::enums::Priority>());
            };

            /**
             * @brief Construct a new Traffic Agent Base object
             * 
             * @param movingObject The moving object representation of the object in the SensorView
             * @param vRef_in The reference velocity of the vehicle.
             * @param laneNetwork A lane network for hte current simulation.
             * @param trafficParticipantMap A map of all traffic participants.
             */
            TrafficAgentBase(osi3::MovingObject movingObject, double vRef_in, double stepSize_in,
                             std::shared_ptr<FZIDriver::util::LaneNetwork> laneNetwork,
                             std::map<unsigned int, std::shared_ptr<TrafficAgentBase>> &trafficParticipantMap);
            /**
             * @brief Update the current position and all corresponding information.
             * 
             * @param movingObject MovingObject representation from the SensorView of the current object.
             */
            virtual void updatePosition(osi3::MovingObject movingObject);

            /**
             * @brief Get the Path State of the Traffic Participant
             * 
             * @return std::tuple<double, double> Current state on the path: <position on path, velocity>
             */
            std::tuple<double, double> getPathState() {return this->pathState;};

            /**
             * @brief Store the path of a target
             * 
             * @param targetID The unique ID of the target.
             * @param targetPath The path of the target.
             */
            void addTargetPath(int targetID, std::shared_ptr<FZIDriver::util::Path> targetPath);

            /**
             * @brief Get the unique id of the traffic participant
             * @return unsigned int 
             */
            unsigned int getAgentID() {return this->agentID;};
            /**
             * @brief Get the path of the traffic participant
             * @return std::shared_ptr<FZIDriver::util::Path> 
             */
            std::shared_ptr<FZIDriver::util::Path> getAgentPath() {return this->agentPath;};
            /**
             * @brief Get the id of the initial lane
             * @return int 
             */
            int getInitialLaneID() {return this->initialLaneID;}
            /**
             * @brief Get the id of the current lane
             * @return int 
             */
            int getCurrentLaneID()  {return this->currentLaneID;}
            /**
             * @brief Get the Turning Direction of the traffic participant
             * 
             * @return FZIDriver::enums::TurningDirction 
             */
            FZIDriver::enums::TurningDirction getTurningDirection() {return this->turningDirection;}

            /**
             * @brief Determine the relevant traffic participants from a list of traffic participants to consider
             * 
             * @param consideredTargets List of unique traffic participants to consider
             */
            void setRelevantTargets(std::vector<unsigned int> consideredTargets);

            /**
             * @brief Get the Predicted Velocity object in a specified prediction step
             * 
             * @param step The step in the prediction gorizon to use.
             * @return double 
             */
            double getPredictedVelocity(int step) {return std::get<1>(this->prediction_pathState[step]);};
            /**
             * @brief Get the Global Position of the object
             * 
             * @param step 
             * @return std::tuple<double, double> 
             */
            std::tuple<double, double> getGlobalPosition(int step);
            /**
             * @brief Find the point on the path with the closest distance to a given point in global coordinates
             * 
             * @param state State to map to the path. Only x and y are used.
             * @return double 
             */
            double mapToPath(FZIDriver::util::State state);

            /**
             * @brief Create a Initial Maneuver Sequence for the traffic participant.
             * 
             */
            void createInitialManeuverSequence();
            /**
             * @brief Create a Prediction of the traffic participant based on the current maneuver sequence
             * 
             */
            void createPrediction();
            /**
             * @brief Wrapper function for one prediction step
             * Step 1: Determine relevant targets.
             * Step 2: Finde the currently optimal maneuver sequence.
             * Step 3: Create a prediction based on the determined maneuver sequence.
             */
            void iterationStep() {
                // ToDo: Move this to a better position
                auto relevantTargetList = std::vector<FZIDriver::TrafficAgentBase>();
                for (int targetID : this->relevanTargets) {
                    relevantTargetList.push_back(*(this->trafficParticipantMap.find(targetID)->second));
                }
                auto result = this->determineManeuver(0, this->pathState, relevantTargetList, *this->prediction_maneuver[0]);
                this->prediction_maneuver = std::get<1>(result);
                this->createPrediction();
            }
    };
}

#endif
