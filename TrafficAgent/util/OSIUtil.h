/**
 * @file OSIUtil.h
 * @author Markus Lemmer
 * @brief Holds various helper functions for osi messages.
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
#ifndef OSI_UTIL
#define OSI_UTIL

#include <math.h>
#include <map>
#include <vector>

#include "osi3/osi_lane.pb.h"
#include "osi3/osi_object.pb.h"
#include "osi3/osi_sensorview.pb.h"

#include "Enums.h"
#include "Path.h"
#include "Vector2d.h"

namespace FZIDriver {
    namespace util {
        namespace OSIUtil {
            static bool getLaneByID(google::protobuf::RepeatedPtrField<osi3::Lane> laneList, unsigned int laneID, osi3::Lane &laneOut) {
                for (auto lane: laneList) {
                    if (lane.id().value() == laneID) {
                        laneOut = lane;
                        return true;
                    }
                }
                return false;
            }

            static bool getMovingObjectByID(osi3::SensorView sensorView, unsigned int agentID, osi3::MovingObject &movingObjectOut) {
                for(const osi3::MovingObject obj : sensorView.global_ground_truth().moving_object()) {
                    if (obj.id().value() == agentID) {
                    movingObjectOut = obj;
                    return true;
                    }
                }
                return false;
            }

            static double getAcceleration(osi3::MovingObject movingObject) {
                double yaw = movingObject.base().orientation().yaw();
                double acceleration = movingObject.base().acceleration().x() * std::cos(yaw) - movingObject.base().acceleration().y() * std::sin(yaw);
                return acceleration;
            }
        }
    }
}

#endif
