//
// Created by h3ct0r on 22/09/20.
//

#ifndef ESPELEO_PLANNER_FRONTIER_INFORMATION_H
#define ESPELEO_PLANNER_FRONTIER_INFORMATION_H

#endif //ESPELEO_PLANNER_FRONTIER_INFORMATION_H

#include <chrono>

using namespace std::chrono;

struct FrontierInformation {
    double entropy = std::numeric_limits<double>::infinity();
    boost::tuple<double, double, double> center;
    std::chrono::milliseconds last_updated_ms = duration_cast< milliseconds >(
            system_clock::now().time_since_epoch()
    );

    FrontierInformation(boost::tuple<double, double, double> _center)
            : center(_center){

        // do something
    }

    bool isEntropyCalculated(){
        return entropy == std::numeric_limits<double>::infinity();
    }

};