#include "kiwirun/path.hpp"
#include <fstream>

namespace kiwi {

    // loads the motion path data from the specified JSON file
    Path::Path(kiwi::Config& config, std::string fileID) :
        config(config), fileID(fileID) {
        this->fetchData(); // automatically fetch path data when the object is created
    };

    // reads the json file and extracts robot motion and subsystem data
    void Path::fetchData() {
        nlohmann::json master;

        // try to open the file using the provided file path
        std::ifstream file(this->fileID);
        if (!file.is_open()) 
            throw std::runtime_error("unable to open file: " + this->fileID);

        // parse the file contents into a json object
        file >> master;
        file.close();

        // go through every point recorded in the file
        for (const auto& jsonPoint : master) {
            // create a new robot position (x, y, angle)
            lemlib::Pose point(0, 0, 0);
            point.x = jsonPoint["x pos"];
            point.y = jsonPoint["y pos"];
            point.theta = jsonPoint["theta"];
            this->pathRecordings.push_back(point); // store this position

            // store any additional subsystem values at this point
            std::vector<float> subsysInfoPoint;
            for (size_t k = 0; k < this->config.subsysStates.size(); k++) {
                subsysInfoPoint.push_back(jsonPoint[this->config.subsysNames[k]]);
            }
            this->subsysRecordings.push_back(subsysInfoPoint);

            // get the linear and angular velocities for this point
            std::vector<float> velRecordingsPoint;
            velRecordingsPoint.push_back(jsonPoint["linear vel"]);
            velRecordingsPoint.push_back(jsonPoint["angular vel"]);
            velRecordings.push_back(velRecordingsPoint); // store the velocities
        }
    };
}
