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

#define _USE_MATH_DEFINES
#include <limits>
#include <math.h>

#include "LinearInterpolation.h"
#include "MathUtil.h"
#include "Path.h"

FZIDriver::util::Path::Path(std::vector<FZIDriver::util::Vector2d<double>> centerLine) {
    // calculate the grid of s coordinates for the path
    // ans start the grid with zero
    this->sGrid.push_back(0);
    for(int i = 1; i < centerLine.size(); i++) {
        sGrid.push_back(sGrid[i-1] + centerLine[i].distance(centerLine[i-1]));
    }
    this->maxS = *(--sGrid.end());

    // create vectors containing the x and y coordinates
    for (Vector2d<double> vec : centerLine) {
        this->xCoordinates.push_back(vec[0]);
        this->yCoordinates.push_back(vec[1]);
    }

    // calculate a linera interpolation
    this->xInterp = std::make_shared<FZIDriver::util::LinearInterpolation>(this->sGrid, this->xCoordinates);
    this->yInterp = std::make_shared<FZIDriver::util::LinearInterpolation>(this->sGrid, this->yCoordinates);
}

FZIDriver::util::Path FZIDriver::util::Path::createPathFromOsiDrivingLane(osi3::Lane lane) {
    // check the type of the lane
    if (lane.classification().type() == osi3::Lane_Classification::TYPE_DRIVING) {
        // read the centerline from the path
        std::vector<FZIDriver::util::Vector2d<double>> centerline;
        for (auto centerLinePoint: lane.classification().centerline()) {
            centerline.push_back(FZIDriver::util::Vector2d<double>(centerLinePoint));
        }

        // check the driving direction
        if (!lane.classification().centerline_is_driving_direction()) {
            std::reverse(centerline.begin(), centerline.end());
        }

        return FZIDriver::util::Path(centerline);
    } else {
        throw std::runtime_error("Can only process lanes of type Driving.");
    }
}

std::tuple<FZIDriver::util::Path, FZIDriver::enums::TurningDirction> FZIDriver::util::Path::createPathFromOsiIntersection(osi3::Lane predecessorLane, osi3::Lane successorLane) {
    // create a stroage for the centerline
    std::vector<FZIDriver::util::Vector2d<double>> centerline;

    unsigned int predecessorLaneID = predecessorLane.id().value();
    unsigned int successorLaneID = successorLane.id().value();

    Vector2d<double> predecessorLast, predecessorSecondToLast;
    // calculate the lane orientation of the predecessor lane
    // as the predecessor lane is the last lane processed in the for loop the last and second to last point can be read from the centerline vector
    if (predecessorLane.classification().centerline_is_driving_direction()) {
        predecessorLast = (*(--predecessorLane.classification().centerline().end()));
        predecessorSecondToLast = (*(--(--predecessorLane.classification().centerline().end())));
    } else {
        predecessorLast = predecessorLane.classification().centerline().Get(0);
        predecessorSecondToLast = predecessorLane.classification().centerline().Get(1);
    }
    double orientationStart = acos(((predecessorLast - predecessorSecondToLast)[0]) / ((predecessorLast - predecessorSecondToLast).norm()));
    
    // get the first element in the successor lane
    Vector2d<double> successorFirst;
    if (successorLane.classification().centerline_is_driving_direction()) {
        successorFirst = (*(successorLane.classification().centerline().begin()));
    } else {
        successorFirst = (*(--successorLane.classification().centerline().end()));
    }

    // calculate the x and y coordinate of the target point
    // in a coordinate system aligned with the end point of the start lane
    Vector2d<double> diff = successorFirst - predecessorLast;
    double xDistance = + cos(orientationStart) * diff[0] + sin(orientationStart) * diff[1];
    double yDistance = - sin(orientationStart) * diff[0] + cos(orientationStart) * diff[1];

    // store the turning direction
    FZIDriver::enums::TurningDirction turningDirection;

    if (abs(yDistance) > 6) {
        // In this case the agent either turns left or right

        // the agents turns
        if (yDistance > 0) {
            // the agent turns left
            turningDirection = FZIDriver::enums::TurningDirction::left;

            // define the mid point of the circle
            // in the coordinate system aligned to the end point of the start lane
            double ym = yDistance;

            double rStart = yDistance;
            double rStop = xDistance;

            // calculate the coefficients of the radial polynomial
            double a0 = rStart;
            double a1 = 2 * (rStop- rStart) / M_PI;

            // calculate the arc
            std::vector<double> phiGrid = FZIDriver::util::math::linspace(0, M_PI_2, 10);
            // remove the first and the last element as they are identical to the end / start of successor and predecessor
            // and would therefore be added twice otherwise
            phiGrid.erase(phiGrid.begin());
            phiGrid.pop_back();

            std::vector<Vector2d<double>> arc;
            for (double phi : phiGrid) {
                double r = a0 + a1 * phi;
                arc.push_back(Vector2d<double>(r * sin(phi), ym - r * cos(phi)));
            }

            // transform the arc back into the world frame
            for (Vector2d<double> vec : arc) {
                centerline.push_back(Vector2d<double>(
                    cos(orientationStart) * vec[0] - sin(orientationStart) * vec[1] + predecessorLast[0],
                    sin(orientationStart) * vec[0] + cos(orientationStart) * vec[1] + predecessorLast[1]
                ));
            }
        } else {
            // the agent turns right
            turningDirection = FZIDriver::enums::TurningDirction::right;

            // define the mid point of the circle
            // in the coordinate system aligned to the end point of the start lane
            double ym = yDistance;

            double rStart = -yDistance;
            double rStop = xDistance;

            // calculate the coefficients of the radial polynomial
            double a0 = rStart;
            double a1 = 2 * (rStop- rStart) / M_PI;

            // calculate the arc
            std::vector<double> phiGrid = FZIDriver::util::math::linspace(0, M_PI_2, 10);
            // remove the first and the last element as they are identical to the end / start of successor and predecessor
            // and would therefore be added twice otherwise
            phiGrid.erase(phiGrid.begin());
            phiGrid.pop_back();

            std::vector<Vector2d<double>> arc;
            for (double phi : phiGrid) {
                double r = a0 + a1 * phi;
                arc.push_back(Vector2d<double>(r * sin(phi), ym + r * cos(phi)));
            }

            // transform the arc back into the world frame
            for (Vector2d<double> vec : arc) {
                centerline.push_back(Vector2d<double>(
                    cos(orientationStart) * vec[0] - sin(orientationStart) * vec[1] + predecessorLast[0],
                    sin(orientationStart) * vec[0] + cos(orientationStart) * vec[1] + predecessorLast[1]
                ));
            }
        }
    } else {
        // the path crosses the junction strait
        turningDirection = FZIDriver::enums::TurningDirction::strait;

        centerline.push_back(predecessorLast);
        centerline.push_back(successorFirst);
    }


    return std::tuple<Path, FZIDriver::enums::TurningDirction>(FZIDriver::util::Path(centerline), turningDirection);
}

double FZIDriver::util::Path::getMaxS() {
    return this->maxS;
}

std::vector<double> FZIDriver::util::Path::getXCoordinates() {
    return this->xCoordinates;
}

std::vector<double> FZIDriver::util::Path::getYCoordinates() {
    return this->yCoordinates;
}

std::tuple<double, double> FZIDriver::util::Path::getPosition(double posS) {
    return std::tuple<double, double>(this->getPosX(posS), this->getPosY(posS));
}

double FZIDriver::util::Path::getPosX(double posS) {
    // calculate teh x position
    double posX = (*this->xInterp)(posS);
    return posX;
}

double FZIDriver::util::Path::getPosY(double posS) {
    // calculate the y position
    double posY = (*this->yInterp)(posS);
    return posY;
}

double FZIDriver::util::Path::getHeading(double posS) {
    // calculate the derivative in x direction
    double dXdS = this->xInterp->prime(posS);
    // calculate the derivative in y direction
    double dYdS = this->yInterp->prime(posS);

    double heading = atan2(dYdS, dXdS);
    return heading;
}

std::tuple<double, double> FZIDriver::util::Path::mapToPath(double posX, double posY) {
    // find the position in the s grid with the minimal value
    std::vector<double> distance;
    for (int i = 0; i < this->sGrid.size(); i++) {
        distance.push_back(abs(posX - this->xCoordinates[i]) + abs(posY - this->yCoordinates[i]));
    }
    int min_idx = std::min_element(distance.begin(), distance.end()) - distance.begin();
    double posS = this->sGrid[min_idx];

    double obj;
    for (int i = 0; i < 100; i++) {
        obj = pow(posX - (*this->xInterp)(posS), 2) + pow(posY - (*this->yInterp)(posS), 2);
        double grad = -2*((posX - (*this->xInterp)(posS)) * this->xInterp->prime(posS) + (posY - (*this->yInterp)(posS)) * this->yInterp->prime(posS));
        posS = posS - 0.9 * grad;

        // check the limits
        if (posS < 0) {
            posS = 0;
            return std::tuple<double, double>(posS, sqrt(pow(posX - (*this->xInterp)(posS), 2) + pow(posY - (*this->yInterp)(posS), 2)));
        }
        if (posS > this->maxS) {
            posS = maxS;
            return std::tuple<double, double>(posS, sqrt(pow(posX - (*this->xInterp)(posS), 2) + pow(posY - (*this->yInterp)(posS), 2)));
        }
    }

    return std::tuple<double, double>(posS, sqrt(obj));
}

std::tuple<double, double, double> FZIDriver::util::Path::calculatePathIntersection(FZIDriver::util::Path path1, FZIDriver::util::Path path2) {
    // find the minimal distance between supporting points
    int idxMin1, idxMin2;
    double minDistance = std::numeric_limits<double>::max();
    // iterate over the s grid of path 1
    for (int i = 0; i < path1.sGrid.size(); i++) {
        // iterate over the s grid of path 2
        for (int j = 0; j < path2.sGrid.size(); j++) {
            // calculate the distance
            double distance = abs(path1.xCoordinates[i] - path2.xCoordinates[j]) + abs(path1.yCoordinates[i] - path2.yCoordinates[j]);
            if (distance < minDistance) {
                idxMin1 = i;
                idxMin2 = j;
                minDistance = distance;
            }
        }
    }

    // define the minimal s values
    double sMin1 = path1.sGrid[idxMin1];
    double sMin2 = path2.sGrid[idxMin2];
    // perform a gradient search near the current solution
    double obj;
    for (int i = 0; i < 100; i++) {
        // calculate the objectove function
        obj = pow((*path1.xInterp)(sMin1) - (*path2.xInterp)(sMin2), 2) + pow((*path1.yInterp)(sMin1) - (*path2.yInterp)(sMin2), 2);
        // calculate the gradient in both parameters
        double grad1 = + 2*((*path1.xInterp)(sMin1) - (*path2.xInterp)(sMin2)) * path1.xInterp->prime(sMin1) 
                       + 2*((*path1.yInterp)(sMin1) - (*path2.yInterp)(sMin2)) * path1.yInterp->prime(sMin1);
        double grad2 = - 2*((*path1.xInterp)(sMin1) - (*path2.xInterp)(sMin2)) * path2.xInterp->prime(sMin2) 
                       - 2*((*path1.yInterp)(sMin1) - (*path2.yInterp)(sMin2)) * path2.yInterp->prime(sMin2);
        // calculate the update of the s coordinate
        sMin1 = sMin1 - 0.9 * grad1;
        sMin2 = sMin2 - 0.9 * grad2;
    }
    minDistance = sqrt(pow((*path1.xInterp)(sMin1) - (*path2.xInterp)(sMin2), 2) + pow((*path1.yInterp)(sMin1) - (*path2.yInterp)(sMin2), 2));

    return std::tuple<double, double, double>(sMin1, sMin2, minDistance);
}