/**
 * @file TrafficAgent.h
 * @author Markus Lemmer
 * @brief This class holds an extension the TrafficParticipantBase class and should be used to simulate the ego vehicle
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
#ifndef TRAFICAGENT
#define TRAFICAGENT

#include <fstream>
#include <deque>
#include <math.h>
#include <map>
#include <vector>

#include "osi3/osi_common.pb.h"
#include "osi3/osi_lane.pb.h"
#include "osi3/osi_object.pb.h"
#include "osi3/osi_sensorview.pb.h"
#include "osi3/osi_trafficcommand.pb.h"
#include "osi3/osi_trafficupdate.pb.h"

#include "TrafficParticipantBase.h"

#include "util/OSIUtil.h"
#include "util/Path.h"
#include "util/State.h"
#include "util/Vector2d.h"

namespace FZIDriver {

class TrafficAgent: public TrafficAgentBase {

    private:
        // set to true if the road network has been initialized using a ground truth message
        bool initialzed;

        // set the maximum number of iterations
        int maxIterations;

        // current state of the vehicle
        FZIDriver::util::State state;
        // current acceleration
        double acceleration;
        // store the centerline of the agent and the path of the target
        unsigned int trafficCommandID;

        /**
         * @brief Create the path of the traffic participant from a TrafficCommand 
         * 
         * @param trafficCommand TrafficCommand containing either a PathAction or a GlobalPositionAction
         * @return true If a path was successfully created.
         * @return false If no path was creates.
         */
        bool createPathFromTrafficCommand(osi3::TrafficCommand trafficCommand);

        /**
         * @brief Create a TrafficUpdate from the current state
         * 
         * @param trafficUpdate Copy of a TrafficUpdate to be used as output.
         * @param simulationTime The current simulation time.
         */
        void createTrafficUpdate(osi3::TrafficUpdate &trafficUpdate, osi3::Timestamp simulationTime);


    public:
        TrafficAgent();
        ~TrafficAgent();

        /**
         * @brief Initialize the TrafficAgent
         * @return int retuns zero on success
         */
        int init();
        
        /**
         * @brief Initilaize TrafficAgent from SensorView
         * 
         * @param sensorView OSI SensorView Message containing a ground truth with lane information.
         * @return int returns zerso on success
         */
        int init(osi3::SensorView sensorView);

        /**
         * @brief Performs a single simulation step.
         * 
         * @param stepSize simulation steo size in seconds
         * @param sensorView OSI SensorView message containing the moving objects
         * @param trafficCommand OSI Traffic Command message containing either a PathAction or a GlobalPositionAction.
         * @param trafficUpdateOut Copy of a OSI TrafficUpdate message. This will be used as output.
         * @return int reuturnes zero on success
         */
        int step(double stepSize, osi3::SensorView sensorView, osi3::TrafficCommand trafficCommand, osi3::TrafficUpdate &trafficUpdateOut);

        void iterativeBestResponse();
};

}
#endif
