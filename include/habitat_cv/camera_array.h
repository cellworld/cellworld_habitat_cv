#pragma once
#include <habitat_cv/camera.h>
#include <habitat_cv/camera_configuration.h>
#include <thread>
namespace habitat_cv {
    struct Camera_array {
        Camera_array(const std::string &, Camera_configuration &);
        Images capture();
        void reset();
        std::string config_file_path;
        Camera_configuration &camera_configuration;
        ~Camera_array();
        std::vector<Camera *> cameras;
    };
}