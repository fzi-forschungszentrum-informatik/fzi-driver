/**
 * @file LaneNetwork.h
 * @author Markus Lemmer
 * @brief LaneNetwork to represent the lane topology of the dribing space.
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
#ifndef LANE_NETWORK_INTERFACE
#define LANE_NETWORK_INTERFACE

#include <deque>
#include <map>
#include <vector>

#include "osi3/osi_lane.pb.h"
#include "osi3/osi_sensorview.pb.h"

#include "Enums.h"
#include "Path.h"

namespace FZIDriver {
    namespace util {
        /**
         * @brief Base class of the different types of lanes.
         */
        struct LaneNetworkElement {
            unsigned int id;
            FZIDriver::enums::LaneType laneType;

            LaneNetworkElement(): id(0), laneType(FZIDriver::enums::LaneType::undefined) {};
            LaneNetworkElement(unsigned int id_in, FZIDriver::enums::LaneType laneType_in): id(id_in), laneType(laneType_in) {};
        };
        
        /**
         * @brief Representation of an OSI Lane.
         * 
         */
        struct Lane : LaneNetworkElement {
            public:
                    
                std::pair<unsigned int, FZIDriver::enums::LaneType> successorLane;
                std::pair<unsigned int, FZIDriver::enums::LaneType> predecessorLane;

                FZIDriver::util::Path path;

                Lane(): LaneNetworkElement(0, FZIDriver::enums::LaneType::driving) {};
                Lane(unsigned int laneID_in) : LaneNetworkElement(laneID_in, FZIDriver::enums::LaneType::driving) {};

            protected:
                Lane(unsigned int laneID_in, FZIDriver::enums::LaneType laneType_in) : LaneNetworkElement(laneID_in, laneType_in) {};
        };

        /**
         * @brief Extension of Lane to describe lanes on an intersection.
         * 
         */
        struct IntersectionLane: Lane {
            FZIDriver::enums::TurningDirction turningDirection;

            IntersectionLane() : Lane(0, FZIDriver::enums::LaneType::intersection_lane) {};
            IntersectionLane(unsigned int laneID_in) : Lane(laneID_in, FZIDriver::enums::LaneType::intersection_lane) {};
        };

        /**
         * @brief Representation of an OSI Intersection.
         * 
         */
        struct Intersection: LaneNetworkElement {
            std::vector<IntersectionLane> intersectionLanes; 

            Intersection(): LaneNetworkElement(0, FZIDriver::enums::LaneType::undefined) {};
            Intersection(unsigned int intersectionID_in) : LaneNetworkElement(intersectionID_in, FZIDriver::enums::LaneType::intersection) {};

            std::vector<IntersectionLane> getAllConnectingLanes(unsigned int incomingLaneID);
        };

        class LaneNetwork {
            private:
            std::map<unsigned int, Lane> laneMap;
            std::map<unsigned int, Intersection> intersectionMap;

            std::map<std::pair<double, double>, std::pair<FZIDriver::util::Lane, double>> globalPositionToPathMapping;

            public:

            /**
             * @brief Construct a new Lane Network object
             * 
             * @param laneMapIn A map holding the lanes.
             * @param intersectionMapIn A map holding all intersections.
             */
            LaneNetwork(std::map<unsigned int, Lane> laneMapIn, std::map<unsigned int, Intersection> intersectionMapIn) { 
                this->laneMap = laneMapIn;
                this->intersectionMap = intersectionMapIn;
            };

            /**
             * @brief Create a Lane Network from SensorView message.
             * The lane and intersrction information is extracted from the containes ground truth.
             * 
             * @param sensorView 
             * @return LaneNetwork 
             */
            static LaneNetwork createLaneNetworkFromSensorView(osi3::SensorView sensorView);
#if TARGET_OPEN_PASS
            static LaneNetwork createLaneNetworkFromOpenPassSensorView(osi3::SensorView sensorView);
#endif

            /**
             * @brief Get a specific LaneNetwork element by its id.
             * 
             * @param elementID The unique id of the network element.
             * @return LaneNetworkElement Eihter or a intersection with the specified id.
             */
            LaneNetworkElement getLaneNetworkElement(unsigned int elementID) {
                if(this->laneMap.find(elementID) != this->laneMap.end()) return this->laneMap[elementID];
                if(this->intersectionMap.find(elementID) != this->intersectionMap.end()) return this->intersectionMap[elementID];
                throw std::runtime_error("Could not find specified lane network element.");
            };
            /**
             * @brief Get a lane by its id
             * @param laneID The unqiue id of the lane.
             * @return Lane 
             */
            Lane getLane(unsigned int laneID) {return this->laneMap[laneID];};
            /**
             * @brief Get an intersection by its id
             * @param intersectionID The unique id of the intersection
             * @return Intersection 
             */
            Intersection getIntersection(unsigned int intersectionID) {return this->intersectionMap[intersectionID];};

            /**
             * @brief Find the intermediate lane on an intersection by the incoming lane and the turning direction.
             * @param incomingLaneID Unique id of the incoming lane-
             * @param turningDirection The turning direction
             * @return IntersectionLane 
             */
            IntersectionLane findIntermediateLaneByTurningDirection(unsigned int incomingLaneID, 
                                                                    FZIDriver::enums::TurningDirction turningDirection);
            /**
             * @brief Finds a connection between two lanes.
             * 
             * @param startLane The initial lane
             * @param finalLane The destination lane
             * @param connectingLanes A copy of a list of lanes. This will be used as output.
             * @param turningDirection_out Copy of a turningDirection container. Will be used as output if
             * connection contains an intersection.
             * @return true if a connection is found
             * @return false if no connection is found
             */
            bool findConnection(Lane startLane, Lane finalLane, std::deque<Lane> &connectingLanes, 
                                FZIDriver::enums::TurningDirction &turningDirection_out);

            /**
             * @brief Find the lane and the s position with the closest distance to a poin in the global coordinate frame.
             * 
             * @param posX The x position in the global coordinate system.
             * @param posY The y position in the global coordinate system.
             * @return std::pair<Lane, double> 
             */
            std::pair<Lane, double> findClosestLaneWithPosition(double posX, double posY);
        };
    };
};

#endif
