#pragma once
#include <habitat_cv/camera_configuration.h>
#include <habitat_cv/camera_array.h>
#include <habitat_cv/image.h>
#include <habitat_cv/layouts.h>
#include <habitat_cv/background.h>
#include <habitat_cv/composite.h>
#include <habitat_cv/video.h>
#include <habitat_cv/detection.h>
#include <habitat_cv/frame_rate.h>
#include <agent_tracking/tracking_service.h>
#include <experiment/experiment_client.h>
#ifdef USE_SYNCHRONIZATION
#include <mcp2221.h>
#endif

#define NOLOCATION cell_world::Location(-1000,-1000)

namespace habitat_cv{
    struct Cv_server;

    struct Predator_data : json_cpp::Json_object{
        Json_object_members(
                Add_member(capture);
                Add_member(human_intervention);
                Add_member(best_camera);
                )
        bool capture;
        bool human_intervention;
        int best_camera;
    };

    struct Cv_server_experiment_client : experiment::Experiment_client{
        explicit Cv_server_experiment_client();
        void on_episode_started(const std::string &experiment_name) override;
        void on_experiment_resumed(const experiment::Resume_experiment_response &) override;
        void on_capture(int frame) override;
        void on_episode_finished() override;
        void on_experiment_started(const experiment::Start_experiment_response &) override;
        void on_human_intervention(bool) override;
        Cv_server *cv_server;
    };

    struct SyncLog : json_cpp::Json_object {
        Json_object_members(
                Add_member(time_stamps);
                Add_member(frames);
                Add_member(leds);
                Add_member(sync_signals);
        )
        cell_world::Json_float_vector time_stamps;
        cell_world::Json_int_vector frames;
        json_cpp::Json_vector<cell_world::Json_int_vector> leds;
        cell_world::Json_float_vector sync_signals;
        std::string file_path;
        void new_log(const std::string &);
        void sync_event(int frame, float time_stamp, const cell_world::Json_int_vector &leds);
        void sync_signal(float time_stamp);
        void close();
    };

    struct Cv_server {
        explicit Cv_server(Camera_configuration &camera_configuration,
                           const std::string &camera_configuration_file,
                           const std::string &background_path,
                           const std::string &video_path,
                           agent_tracking::Tracking_server &,
                           Cv_server_experiment_client &,
                           const cell_world::Location_list &,
                           const cell_world::Capture_parameters &,
                           bool unlimited = false);
        void tracking_process();
        bool new_episode(const std::string &subject, const std::string &experiment, int episode, const std::string &occlusions, const std::string &destination_folder);
        bool get_mouse_step(Binary_image &diff, const Image &masked, json_cpp::Json_vector<cell_world::Step> &steps, json_cpp::Json_vector<cell_world::Step> &old_steps, std::vector<Detection> &mouse_detections, const cell_world::Location &robot_location, float scale);
        bool get_robot_step(const Binary_image &image, cell_world::Step &step, float scale, cell_world::Location &mouse_head);
        bool end_episode();

        cell_world::Location robot_normalized_destination = NOLOCATION;
        cell_world::Location robot_destination = NOLOCATION;
        cell_world::Location gravity_adjustment = NOLOCATION;

        cv::Scalar robot_color {150, 0, 150};
        cv::Scalar mouse_color {120, 120, 0};
        cv::Scalar shaved_mouse_color {0, 0, 255};

        agent_tracking::Tracking_server &tracking_server;
        Cv_server_experiment_client &experiment_client;

        unsigned int mouse_threshold = 40;
        unsigned int robot_threshold = 237; //250;

        cell_world::Capture_parameters capture_parameters;

        cell_world::Space canonical_space;
        cell_world::World_implementation cv_implementation;
        cell_world::Space cv_space;
        bool human_intervention = false;
        cell_world::Timer ts;
        std::atomic<bool> tracking_running = false;
        std::atomic<int> puff_state = 0;
        cell_world::Cell_group occlusions;

        bool unlimited;
        bool waiting_for_prey = false;
        Camera_configuration camera_configuration;
        Camera_array cameras;

        Background background;
        Screen_layout screen_layout;
        Main_layout main_layout;
        Raw_layout raw_layout;
        Mouse_layout mouse_layout;
        bool reset_robot_connection = false;

        SyncLog sync_log;
        Video main_video;
        Video raw_video;
        Video zoom_video;

        Profile led_profile;
        Profile mouse_profile;
        std::string video_path;
        std::string background_path;

        float robot_height = 10.4;   // cm  // 5.0 (short) , 10.4 (tall), 12.0 (carapace)
        std::vector<cv::Point2f> zoom_rectangles;
        cv::Size zoom_size{150,150};
        unsigned int episode_count{};
        cell_world::Timer experiment_timer;
        unsigned int frame_number{};
        cell_world::Location_list sync_led_locations;
#ifdef USE_SYNCHRONIZATION
        mcp2221::Device *synchronization_device;
        bool synchronization_enabled;
#endif
    };
}