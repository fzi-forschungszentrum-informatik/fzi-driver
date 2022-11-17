/**
 * @file Path.h
 * @author Markus Lemmer
 * @brief Define a class that describes an arbitrary path.
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
#ifndef PATH
#define PATH

#include <vector>

#include "osi3/osi_lane.pb.h"

#include "Enums.h"
#include "Interpolation.h"
#include "Vector2d.h"

namespace FZIDriver {
    namespace util {
        class Path {
            private:
                // store the maximal s coordinate and the s coordinate grid
                double maxS;
                std::vector<double> sGrid;

                // store the coordinates of the path
                std::vector<double> xCoordinates;
                std::vector<double> yCoordinates;

                // store linear interpolations of the path
                std::shared_ptr<FZIDriver::util::Interpolation> xInterp;
                std::shared_ptr<FZIDriver::util::Interpolation> yInterp;

            public:
                Path() {};
                Path(std::vector<FZIDriver::util::Vector2d<double>> centerLine);
                ~Path() {};
                
                /**
                 * @brief Creates a path from an OSI lane.
                 * @param lane OSI Lane message
                 * @return Path 
                 */
                static Path createPathFromOsiDrivingLane(osi3::Lane lane);
                /**
                 * @brief Create a Path on an intersection based on the succesoor and predecessor lane.
                 * @param predecessorLane OSI lane message of the predeccessor approaching the intersection
                 * @param successorLane  OSI lane message of the successor leaving the intersection
                 * @return std::tuple<Path, FZIDriver::enums::TurningDirction> 
                 */
                static std::tuple<Path, FZIDriver::enums::TurningDirction> createPathFromOsiIntersection(osi3::Lane predecessorLane, osi3::Lane successorLane);

                /**
                 * @brief Get the max s position on the path.
                 * @return double 
                 */
                double getMaxS();
                /**
                 * @brief Return the x coordinates of the path.
                 * @return std::vector<double> 
                 */
                std::vector<double> getXCoordinates();
                /**
                 * @brief Return the y coordinates of the path.
                 * @return std::vector<double> 
                 */
                std::vector<double> getYCoordinates();

                /**
                 * @brief Get the gobal position of the path.
                 * @param posS the s coordinate on the path
                 * @return std::tuple<double, double> global position in the form <x Position, y Position>
                 */
                std::tuple<double, double> getPosition(double posS);
                /**
                 * @brief Get the gobal x position of the path.
                 * @param posS the s coordinate on the path
                 * @return std::tuple<double, double> 
                 */
                double getPosX(double posS);
                /**
                 * @brief Get the gobal x position of the path.
                 * @param posS the s coordinate on the path
                 * @return std::tuple<double, double> 
                 */
                double getPosY(double posS);
                /**
                 * @brief Get the heading of the path in the global coordinate frame.
                 * @param posS the s coordinate on the path
                 * @return std::tuple<double, double> 
                 */
                double getHeading(double posS);

                /**
                 * @brief Calculate the point with the closes distance on a path.
                 * @param posX x coordinate in the global coordinate frame
                 * @param posY y coordinate in the global coordinate frame
                 * @return std::tuple<double, double> position in path coordinates: <s, t>
                 */
                std::tuple<double, double> mapToPath(double posX, double posY);

                /**
                 * @brief Calculates the intersection of two paths
                 * @param path1 
                 * @param path2 
                 * @return std::tuple<double, double, double> intersection coordinates in the form: 
                 * <s Position on Path 1, s Position on Path 2, distance>
                 */
                static std::tuple<double, double, double> calculatePathIntersection(Path path1, Path path2);
        };
    }

}

#endif