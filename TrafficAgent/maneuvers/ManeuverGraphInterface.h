/**
 * @file ManeuverGraphInterface.h
 * @author Markus Lemmer
 * @brief Abstract class for the definition of a meneuver graph.
 * 
 * @copyright Copyright (c) 2022 FZI Forschungszentrum Informatik
 * 
 */

#ifndef MANEUVER_GRAPH_INTERFACE
#define MANEUVER_GRAPH_INTERFACE

#include <memory>
#include <vector>

namespace FZIDriver {
    namespace maneuver {
        /**
         * @brief Representation of a maneuver providing function to calculate acceleration.
         * 
         */
        class ManeuverNodeInterface {
            private:
                std::vector<std::shared_ptr<ManeuverNodeInterface>> nextManeuverOptions;

            public:
                virtual ~ManeuverNodeInterface(){};

                /**
                 * @brief Add a shared pointer to another maneuver node, if the maneuver is reachable from the current onr
                 * 
                 * @param maneuverOption Shared pointer to another maneuver node in the graph.
                 */
                void addManeuverOption(std::shared_ptr<ManeuverNodeInterface> maneuverOption) {this->nextManeuverOptions.push_back(maneuverOption);};

                /**
                 * @brief Get a list of all maneuber that can be reaches from the current one.
                 * 
                 * @return std::vector<std::shared_ptr<ManeuverNodeInterface>> 
                 */
                std::vector<std::shared_ptr<ManeuverNodeInterface>> getManeuverOptions() {return this->nextManeuverOptions;}

                /**
                 * @brief Calculate the acceleration of the maneuver.
                 * 
                 * @return double 
                 */
                virtual double getAcceleration(double, double) = 0;
        };

        /**
         * @brief Representation of a meneuver graph holding a list of all relevant maneuvers.
         * 
         */
        class ManeuverGraphInterface {
            protected:
                std::vector<std::shared_ptr<ManeuverNodeInterface>> maneuvers;
            
            public:
                /**
                 * @brief Get a maneuver node from the graph by its index.
                 * 
                 * @param i 
                 * @return std::shared_ptr<ManeuverNodeInterface> 
                 */
                std::shared_ptr<ManeuverNodeInterface> getManeuverNode(int i) {return this->maneuvers[i];};
        };
    };
};

#endif