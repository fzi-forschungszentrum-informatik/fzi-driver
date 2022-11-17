/**
 * @file BasicManeuverGraph.h
 * @author Markus Lemmer
 * @brief Basic implementation of a maneuver graph with 3 maneuvers using contsnt acceleratio values.
 * 
 * @copyright Copyright (c) 2022 FZI Forschungszentrum Informatik
 * 
 */

#ifndef BASIC_MANEUVER_GRAPH
#define BASIC_MANEUVER_GRAPH

#include <functional>
#include <memory>
#include <vector>

#include "ManeuverGraphInterface.h"

namespace FZIDriver {
    namespace maneuver {

        class BasicManeuverNode : public ManeuverNodeInterface {
            private:
                std::function<double()> acceleration;

                
            public:
                BasicManeuverNode(std::function<double()> acceleration) {
                    // set the function that calculates the acceleration
                    this->acceleration = acceleration;
                };

                double getAcceleration(double, double) {return this->acceleration();};
        };

        class BasicManeuverGraph: public ManeuverGraphInterface {
            public:
                BasicManeuverGraph() {
                    // create the maneuver options
                    std::shared_ptr<BasicManeuverNode> constantDrive = std::make_shared<BasicManeuverNode>(BasicManeuverNode([](){return 0.0;}));
                    std::shared_ptr<BasicManeuverNode> accelerate    = std::make_shared<BasicManeuverNode>(BasicManeuverNode([](){return 2.5;}));
                    std::shared_ptr<BasicManeuverNode> decelerate    = std::make_shared<BasicManeuverNode>(BasicManeuverNode([](){return -2.5;}));

                    // define the next maneuver options for the maneuver constant drive
                    constantDrive->addManeuverOption(constantDrive);
                    constantDrive->addManeuverOption(accelerate);
                    constantDrive->addManeuverOption(decelerate);
                    // define the next maneuver options of the maneuver accelerate
                    accelerate->addManeuverOption(accelerate);
                    accelerate->addManeuverOption(constantDrive);
                    // define the next maneuver options of the maneuver decelerate
                    decelerate->addManeuverOption(decelerate);
                    decelerate->addManeuverOption(constantDrive);

                    // store the maneuver options
                    this->maneuvers.push_back(constantDrive);
                    this->maneuvers.push_back(accelerate);
                    this->maneuvers.push_back(decelerate);
                }
        };
    }
}

#endif