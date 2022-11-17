/**
 * @file LinearInterpolation.h
 * @author Markus Lemmer
 * @brief Stores various enumerations
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
#ifndef ENUMS
#define ENUMS

#include "osi3/osi_lane.pb.h"

namespace FZIDriver {
    namespace enums {
        /**
         * @brief Enumeration is used to indicate a turning direction.
         * This enumeration can either be used for lanes of for traffic participants themself.
         */
        enum class TurningDirction {undefined = 0, strait = 1, left = 2, right = 3};

        /**
         * @brief Enumeration to indicate the type of a lane.
         * 
         */
        enum class LaneType {undefined = 0, driving = 1, intersection = 2, intersection_lane = 3};

        /**
         * @brief Enumeration used to define the priority of a traffic participant over another.
         * In this case priority refers to the rightof way,
         */
        enum class Priority {FALSE = 0, TRUE = 1, UNDEFINED = 2};

        /**
         * @brief Enmeration to define the relaction between two traffic participants.
         * Two TrafficParticipants can either be on crossing lanes or on identical paths.
         * Otherwise the relation between the participants in NONE.
         */
        enum class TargetRelation {NONE = 0, CROSSING = 1, IDENTICAL_PATH = 2};

        /**
         * @brief Converts from OSI LaneType to enumeration objects-
         * @param type OSI LaneType object
         * @return LaneType 
         */
        static LaneType getLaneTypeFromOSILaneType(osi3::Lane_Classification_Type type) {
            if (type == osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING) {
                return LaneType::driving;
            } else if (type == osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION) {
                return LaneType::intersection;
            } else {
                return LaneType::undefined;
            }
        };
    }
}

#endif