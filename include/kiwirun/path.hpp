#ifndef PATH_HPP
#define PATH_HPP

#include "main.h"
#include "kiwirun/config.hpp"

namespace kiwi {

class Path {
    public:

        kiwi::Config& config;
        std::string fileID;

        std::vector<lemlib::Pose> pathRecordings;
        std::vector<std::vector<float>> subsysRecordings; //vector (timesteps) of vectors (states at timestep) of reference states
        std::vector<std::vector<float>> velRecordings; //vector (timesteps) of vectors (angular/linear vel at timestep) of velocities

        void fetchData();

        float findLateralError(float targetX, float targetY);
        float findLongitudinalError(float targetX, float targetY);
        int findClosestPoint(lemlib::Pose pose, int prevIndex);

        float toRPM(float linearVel);
        float toInches(float meterMeasurement);
        float toMeters(float inchMeasurement);

        void ramseteStep(int index);
        void updateSubsystems(int index);


        Path(kiwi::Config& config, std::string fileID);

        void follow();
};

}

#endif