#ifndef IDM_MANEUVER_GRAPH
#define IFM_MANEUVER_GRAPH

#include <functional>
#include <math.h>

#include "ManeuverGraphInterface.h"

namespace FZIDriver {
    namespace maneuver {

        class IDMManeuverNode : public ManeuverNodeInterface {
            private:
                std::function<double(double, double)> acceleration;

                
            public:
                IDMManeuverNode(std::function<double(double, double)> acceleration) {
                    // set the function that calculates the acceleration
                    this->acceleration = acceleration;
                };


                double getAcceleration(double velocity, double distToStopLine) {return this->acceleration(velocity, distToStopLine);};
        };

        class IDMManeuverGraph : public ManeuverGraphInterface {
            private:
                double maxAcceleration;
                double referenceVelocity;

            public:
                IDMManeuverGraph(double referenceVelocity_in = 5.0, double maxAcceleration_in = 2.5) {
                    // set the reference velocity
                    this->referenceVelocity = referenceVelocity_in;
                    this->maxAcceleration = maxAcceleration_in;

                    // create the maneuver options
                    std::shared_ptr<IDMManeuverNode> freeCruise = std::make_shared<IDMManeuverNode>(
                        IDMManeuverNode([referenceVelocity_in, maxAcceleration_in](double velocity, double distanceToStopLine){
                            return maxAcceleration_in*(1 - (velocity / referenceVelocity_in));
                        })
                    );
                    std::shared_ptr<IDMManeuverNode> speed = std::make_shared<IDMManeuverNode>(
                        IDMManeuverNode([referenceVelocity_in, maxAcceleration_in](double velocity, double distanceToStopLine){
                            // ToDo: Make the speed factor tunable...
                            return 1.25 * maxAcceleration_in*(1 - (velocity / referenceVelocity_in));
                        })
                    );
                    std::shared_ptr<IDMManeuverNode> yield = std::make_shared<IDMManeuverNode>(
                        IDMManeuverNode([referenceVelocity_in, maxAcceleration_in](double velocity, double distanceToStopLine){
                            double vel = velocity / referenceVelocity_in;
                            double dist = (0.5 * std::pow(velocity,2)) / (2 * std::sqrt(maxAcceleration_in));
                            return maxAcceleration_in * (1 - vel - dist);
                        })
                    );

                    // define the next maneuver options for the maneuver constant drive
                    freeCruise->addManeuverOption(freeCruise);
                    freeCruise->addManeuverOption(speed);
                    freeCruise->addManeuverOption(yield);
                    // define the next maneuver options of the maneuver accelerate
                    speed->addManeuverOption(speed);
                    speed->addManeuverOption(freeCruise);
                    // define the next maneuver options of the maneuver decelerate
                    yield->addManeuverOption(yield);
                    yield->addManeuverOption(freeCruise);

                    // store the maneuver options
                    this->maneuvers.push_back(freeCruise);
                    this->maneuvers.push_back(speed);
                    this->maneuvers.push_back(yield);
                }
        };
    }
}

#endif