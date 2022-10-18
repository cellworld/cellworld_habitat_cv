#include <thread>
#include <filesystem>
#include <habitat_cv/cv_service.h>
#include <performance.h>


#define SAFETY_MARGIN 75
#define PUFF_DURATION 15

using namespace agent_tracking;
using namespace cell_world;
using namespace easy_tcp;
using namespace std;
using namespace tcp_messages;


#define ENTRANCE Location(0,.5)
#define  ENTRANCE_DISTANCE .05

namespace habitat_cv {

    bool Cv_server::new_episode(const string &subject, const string &experiment, int episode, const string &occlusions, const string &destination_folder) {
        cout << "new_episode" << endl;
        if (main_video.is_open()) end_episode();
        std::filesystem::create_directories(destination_folder);
        cout << "Video destination folder: " + destination_folder << endl;
        main_layout.new_episode(subject, experiment, episode, occlusions);
        main_video.new_video(destination_folder + "/main_" + experiment + ".mp4");
        raw_video.new_video(destination_folder + "/raw_" + experiment + ".mp4");
        for (int i=0; i<4; i++) {
            mouse_videos[i]->new_video(destination_folder + "/mouse" + to_string(i) + "_" + experiment + ".mp4");
        }
        ts.reset();
        waiting_for_prey = true;
        return true;
    }

    bool Cv_server::end_episode() {
        cout << "end_episode" << endl;
        main_video.close();
        for (auto &mouse_video:mouse_videos) {
            mouse_video->close();
        }
        raw_video.close();
        return true;
    }

    bool Cv_server::get_mouse_step(const Image &diff, Step &step, const Location &robot_location) {
        auto detections = Detection_list::get_detections(diff, 55, 2);
        auto mouse_candidates = detections.filter(mouse_profile);
        for (auto &mouse: mouse_candidates) {
            if (mouse.location.dist(robot_location) < SAFETY_MARGIN) continue;
            step.agent_name = "prey";
            step.location = mouse.location;
            return true;
        }
        return false;
    }

    bool Cv_server::get_robot_step(const Image &image, Step &step) {
        auto leds = Detection_list::get_detections(image, robot_threshold, 0).filter(led_profile);
        if (leds.size() != 3) return false;
        double d1 = leds[0].location.dist(leds[1].location);
        double d2 = leds[1].location.dist(leds[2].location);
        double d3 = leds[2].location.dist(leds[0].location);
        Location back;
        Location front;
        if (d1 < d2 && d1 < d3) {
            back = (leds[0].location + leds[1].location);
            back.x /= 2;
            back.y /= 2;
            front = leds[2].location;
        } else if (d2 < d3 && d2 < d1) {
            back = (leds[1].location + leds[2].location);
            back.x /= 2;
            back.y /= 2;
            front = leds[0].location;
        } else {
            back = (leds[2].location + leds[0].location);
            back.x /= 2;
            back.y /= 2;
            front = leds[1].location;
        }
        step.location.x = (front.x + back.x) / 2;
        step.location.y = (front.y + back.y) / 2;
        step.rotation = -to_degrees(atan2(front.y - back.y, front.x - back.x) - M_PI / 2);
        step.agent_name = "predator";
        return true;
    }

    float Cv_server::get_prey_robot_orientation(Image &prey_robot_cam) {
        auto detections = Detection_list::get_detections(prey_robot_cam, 50, 1);
        auto robot_center = detections.filter(mouse_profile);
        if (robot_center.empty()) return 0;
        auto robot_head = detections.filter(prey_robot_head_profile);
        if (robot_head.empty()) return 0;
        return robot_center[0].location.atan(robot_head[0].location);
    }

#define NOLOCATION Location(-1000,-1000)
    enum Screen_image {
        main,
        difference,
        led,
        raw,
        mouse,
        best_mouse,
        cam0,
        cam1,
        cam2,
        cam3
    };

    void Cv_server::tracking_process() {
        tracking_running = true;
        puff_state = false;
        Step mouse;
        mouse.location = NOLOCATION;
        Step robot;
        robot.location = NOLOCATION;
        int robot_counter = 0;
        ts.reset();
        int robot_best_cam = -1;
        int mouse_best_cam = 0;
        bool new_robot_data;
        string screen_text;
        Screen_image screen_image = Screen_image::main;
        double fps = Video::get_fps();
        double time_out = 1.0 / fps * .999;
//        cout << "time_out: " << time_out << endl;
        float camera_height = 205;
        float robot_height = 9;
        float height_ratio = robot_height / camera_height;
        Step canonical_step;
        vector<Location> camera_zero;
        {
            int i = 0;
            auto zero_point = cv::Point2f(Camera::frame_size.width / 2, Camera::frame_size.height / 2);
            auto images = cameras.capture();
            //images.emplace_back(images[2],"camera_3.png");
            auto composite_image = composite.get_composite(images);
            for (unsigned int c=0; c < cameras.cameras.size(); c++) {
                auto camera_zero_point = composite.get_warped_point(i++, zero_point);
                auto camera_zero_location = composite_image.get_location(camera_zero_point);
                camera_zero.push_back(camera_zero_location);
            }
            camera_zero.push_back(camera_zero[2]);
        }
        Timer frame_timer(time_out);
        Frame_rate fr;
        fr.filter = .9;
        bool show_occlusions = false;
        int input_counter=0;
        while (tracking_running) {
            //Timer capture_timer;
            PERF_START("WAIT");
//            while (!frame_timer.time_out());
            PERF_STOP("WAIT");
            frame_timer.reset();
            PERF_START("CAPTURE");
            auto images = cameras.capture();
            PERF_STOP("CAPTURE");
            PERF_START("COMPOSITE");
            composite.get_composite(images);
            PERF_STOP("COMPOSITE");
            PERF_START("COLOR CONVERSION");
            auto &composite_image_gray = composite.composite_detection;
            PERF_STOP("COLOR CONVERSION");
            PERF_START("ROBOT DETECTION");
            if (robot_best_cam == -1) {
                new_robot_data = get_robot_step(composite_image_gray, robot);
            } else {
                new_robot_data = get_robot_step(composite.get_camera(robot_best_cam), robot);
                if (!new_robot_data) {
                    new_robot_data = get_robot_step(composite_image_gray, robot);
                }
            }
            unsigned int frame_number = 0;
            if (main_video.is_open()) {
                frame_number = main_video.frame_count;
            }
            if (new_robot_data) {
                if ((robot.location.x < 500 || robot.location.x > 580) &&
                    (robot.location.y < 500 || robot.location.y > 580)) {
                    int cam_row = robot.location.y > 540 ? 0 : 1;
                    int cam_col = robot.location.x > 540 ? 1 : 0;
                    robot_best_cam = camera_configuration.order[cam_row][cam_col];
                }
                auto perspective_offset = robot.location - camera_zero[robot_best_cam];
                auto perspective_adjustment = perspective_offset * height_ratio;
                robot.location += (-perspective_adjustment);
                robot_counter = 30;
            } else {
                if (robot_counter) robot_counter--;
                else robot.location = NOLOCATION;
            }
            PERF_STOP("ROBOT DETECTION");
            PERF_START("MOUSE DETECTION");
            PERF_START("DIFF");
            auto diff = composite_image_gray.diff(background.composite);
            PERF_STOP("DIFF");
            auto send_prey_step = false;
            PERF_START("MOUSE_STEP");
            if (get_mouse_step(diff, mouse, robot.location)) {
                composite.start_zoom(mouse.location);
                PERF_START("MOUSE_DETECTED");
                int cam_row = mouse.location.y > 540 ? 0 : 1;
                int cam_col = mouse.location.x > 540 ? 1 : 0;
                mouse_best_cam = camera_configuration.order[cam_row][cam_col];
                canonical_step = mouse.convert(cv_space, canonical_space);
                if (waiting_for_prey && canonical_step.location.dist(ENTRANCE) > ENTRANCE_DISTANCE) {
                    waiting_for_prey = false;
                    experiment_client.prey_enter_arena();
                }
                send_prey_step = true;
                PERF_STOP("MOUSE_DETECTED");
            }
            PERF_STOP("MOUSE_STEP");
            PERF_STOP("MOUSE DETECTION");
            PERF_START("DETECTION_PROCESSING");
            auto &composite_image_rgb=composite.get_video();
            if (new_robot_data) {
                auto color_robot = cv::Scalar({255, 0, 255});
                if (puff_state) {
                    robot.data = "puff";
                    color_robot = cv::Scalar({0, 0, 255});
                    puff_state--;
                } else {
                    robot.data = "";
                }
                thread([this, frame_number](Step &robot, Timer &ts, Tracking_server &tracking_server) {
                    robot.time_stamp = ts.to_seconds();
                    robot.frame = frame_number;
                    tracking_server.send_step(robot.convert(cv_space, canonical_space));
                }, reference_wrapper(robot), reference_wrapper(ts), reference_wrapper(tracking_server)).detach();
                composite_image_rgb.circle(robot.location, 5, color_robot, true);
                auto robot_cell_id = composite.map.cells.find(robot.location);
                auto robot_cell_coordinates = composite.map.cells[robot_cell_id].coordinates;

                auto cell_polygon = composite.get_polygon(robot_cell_coordinates);
                composite_image_rgb.polygon(cell_polygon, color_robot);
                composite_image_rgb.arrow(robot.location, to_radians(robot.rotation), 50, color_robot);
            }
            PERF_STOP("DETECTION_PROCESSING");
            if (send_prey_step) {
                composite_image_rgb.circle(mouse.location, 5, {255, 0, 0}, true);
                auto mouse_cell_id = composite.map.cells.find(mouse.location);
                auto mouse_cell_coordinates = composite.map.cells[mouse_cell_id].coordinates;
                auto cell_polygon = composite.get_polygon(mouse_cell_coordinates);
                composite_image_rgb.polygon(cell_polygon, {255, 0, 0});
                thread([this, frame_number](Step &canonical_step, Timer &ts, Tracking_server &tracking_server) {
                    canonical_step.time_stamp = ts.to_seconds();
                    canonical_step.frame = frame_number;
                    canonical_step.rotation = 0;
                    tracking_server.send_step(canonical_step);
                }, reference_wrapper(canonical_step), reference_wrapper(ts), reference_wrapper(tracking_server)).detach();
            }
            PERF_START("MOUSE_CUT");
            auto main_frame = main_layout.get_frame(composite_image_rgb, frame_number);
            auto raw_frame = raw_layout.get_frame(images);
            Images &mouse_cut = composite.get_zoom();
            PERF_STOP("MOUSE_CUT");
            PERF_SCOPE("REST");
            auto mouse_frame = raw_layout.get_frame(mouse_cut);
            Image screen_frame;
            switch (screen_image) {
                case Screen_image::main :
                    if (show_occlusions) {
                        for (auto &occlusion: occlusions) {
                            composite_image_rgb.circle(occlusion.get().location, 20, {255, 0, 0}, true);
                        }
                    }
                    screen_frame = screen_layout.get_frame(composite_image_rgb, "main");
                    break;
                case Screen_image::difference :
                    screen_frame = screen_layout.get_frame(diff, "difference");
                    break;
                case Screen_image::led :
                    screen_frame = screen_layout.get_frame(Image(cv::Mat(composite_image_gray > robot_threshold),""), "LEDs");
                    break;
                case Screen_image::mouse :
                    screen_frame = screen_layout.get_frame(mouse_frame, "mouse");
                    break;
                case Screen_image::best_mouse :
                    screen_frame =  screen_layout.get_frame(Image(mouse_cut[mouse_best_cam].threshold(50).dilate(2).erode(2),""),"best mouse:" + to_string(mouse_best_cam) );
                    break;
                case Screen_image::raw :
                    screen_frame = screen_layout.get_frame(raw_frame, "raw");
                    break;
                case Screen_image::cam0 :
                    screen_frame = screen_layout.get_frame(images[0], "cam0");
                    break;
                case Screen_image::cam1 :
                    screen_frame = screen_layout.get_frame(images[1], "cam1");
                    break;
                case Screen_image::cam2 :
                    screen_frame = screen_layout.get_frame(images[2], "cam2");
                    break;
                case Screen_image::cam3 :
                    screen_frame = screen_layout.get_frame(images[3], "cam3");
                    break;
            }
            if (main_video.is_open()) screen_frame.circle({20, 20}, 10, {0, 0, 255}, true);
            cv::imshow("Agent Tracking", screen_frame);
            if (!input_counter) {
                input_counter = 10;
                auto key = cv::waitKey(1);
                switch (key) {
                    case 'C':
                        // start video recording
                        images.save(".");
                        break;
                    case 'V':
                        // start video recording
                        main_video.new_video("main.mp4");
                        raw_video.new_video("raw.mp4");
                        for (int i = 0; i < 4; i++) {
                            mouse_videos[i]->new_video("mouse" + to_string(i) + ".mp4");
                        }
                        break;
                    case 'B':
                        // end video recording
                        main_video.close();
                        for (auto &mouse_video: mouse_videos) {
                            mouse_video->close();
                        }
                        raw_video.close();
                        break;
                    case 'Q':
                        tracking_running = false;
                        break;
                    case 'M':
                        screen_image = main;
                        break;
                    case 'R':
                        cameras.reset();
                        break;
                    case '[':
                        robot_threshold++;
                        cout << "robot threshold set to " << robot_threshold << endl;
                        break;
                    case ']':
                        robot_threshold--;
                        cout << "robot threshold set to " << robot_threshold << endl;
                        break;
                    case 'U':
                        background.update(composite.composite_detection);
                        break;
                    case 'O':
                        show_occlusions = !show_occlusions;
                        break;
                    case '0':
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                    case '5':
                    case '6':
                    case '7':
                    case '8':
                    case '9':
                        screen_image = static_cast<Screen_image>(key-'0');
                        break;
                    case '\t':
                        if (screen_image == Screen_image::cam3)
                            screen_image = Screen_image::main;
                        else
                            screen_image = static_cast<Screen_image>(screen_image + 1);
                        cout << "change_screen_output to " << screen_image << endl;
                        break;
                    case ' ':
                        if (screen_image == Screen_image::main)
                            screen_image = Screen_image::cam3;
                        else
                            screen_image = static_cast<Screen_image>(screen_image - 1);
                        cout << "change_screen_output to " << screen_image << endl;
                        break;
                }
            } else {
                input_counter--;
            }
            fr.new_frame();
//            cout << fr.filtered_fps<< " fps  "<< fr.average_fps << " fps                \r";
            if (!main_video.is_open()) mouse.location = NOLOCATION;
            if (mouse.location == NOLOCATION) continue; // starts recording when mouse crosses the door
//            thread t([this, main_frame, mouse_cut, raw_frame]() {
            main_video.add_frame(main_frame);
            raw_video.add_frame(raw_frame);
            for (int i=0;i<4;i++) {
                mouse_videos[i]->add_frame(mouse_cut[i]);
            }
//            });
//            t.detach();
            // write videos
        }

    }
    Cv_server::Cv_server(const std::string &camera_configuration_file,
                         const std::string &background_path,
                         agent_tracking::Tracking_server &tracking_server,
                         Cv_server_experiment_client &experiment_client):
            tracking_server(tracking_server),
            experiment_client(experiment_client),
            canonical_space(World_implementation::get_from_parameters_name("hexagonal","canonical").space),
            cv_space(World_implementation::get_from_parameters_name("hexagonal","cv").space),
            camera_configuration(Resources::from("camera_configuration").key("default").get_resource<Camera_configuration>()),
            //cameras(camera_configuration_file, camera_configuration.order.count()),
            cameras(camera_configuration_file, 4),
            composite(camera_configuration),
            main_video(main_layout.size(), Image::rgb),
            raw_video(raw_layout.size(), Image::gray),
            led_profile(Resources::from("profile").key("led").get_resource<Profile>()),
            mouse_profile(Resources::from("profile").key("mouse").get_resource<Profile>()),
            prey_robot_head_profile(Resources::from("profile").key("prey_robot_head").get_resource<Profile>())
    {
        experiment_client.cv_server = this;
        for (int i = 0; i < 4; i++) {
            mouse_videos.push_back(new Video(cv::Size(150,150), Image::gray));
        }
        background.set_path(background_path);
        if (!background.load()) {
            auto images = cameras.capture();
            images.push_back(images[2]);
            composite.get_composite(images);
            background.update(composite.composite_detection);
        }

    }

    void Cv_server_experiment_client::on_episode_started(const string &experiment_name) {
        auto experiment = this->get_experiment(experiment_name);
        std::stringstream ss;
        ss << "/research/videos/" << experiment_name << "/episode_" << std::setw(3) << std::setfill('0') << experiment.episode_count;
        std::string destination_folder = ss.str();
        cv_server->new_episode(experiment.subject_name, experiment.experiment_name, experiment.episode_count, experiment.world_info.occlusions, destination_folder);
    }

    void Cv_server_experiment_client::on_episode_finished() {
        cv_server->end_episode();
    }

    Cv_server_experiment_client::Cv_server_experiment_client() {}

    void Cv_server_experiment_client::on_capture(int) {
        cv_server->puff_state =  PUFF_DURATION;
    }

    void Cv_server_experiment_client::on_experiment_started(const experiment::Start_experiment_response &experiment) {
        cv_server->occlusions = World::get_from_parameters_name(experiment.world.world_configuration,"cv",experiment.world.occlusions).create_cell_group().occluded_cells();
    }

}