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

namespace habitat_cv {

    string get_experiment_prefix(const string &experiment_name){
        return experiment_name.substr(0,experiment_name.find('_'));
    }

    void Cv_server_experiment_client::on_episode_finished() {
        cv_server->end_episode();
    }

    Cv_server_experiment_client::Cv_server_experiment_client() {

    }

    void Cv_server_experiment_client::on_capture(int) {
        cv_server->puff_state =  PUFF_DURATION;
    }

    void Cv_server_experiment_client::on_human_intervention(bool active) {
        cv_server->human_intervention = active;
    }

    void Cv_server_experiment_client::on_experiment_resumed(const experiment::Resume_experiment_response &resume_experiment_response) {
        cv_server->episode_count = resume_experiment_response.episode_count;
        cv_server->experiment_timer.check_point = resume_experiment_response.start_date;
        cv_server->end_episode();
    }

    void Cv_server_experiment_client::on_experiment_started(const experiment::Start_experiment_response &start_experiment_response) {
//        cv_server->occlusions = World::get_from_parameters_name(experiment.world.world_configuration,"cv",experiment.world.occlusions).create_cell_group().occluded_cells();
        cv_server->experiment_timer.check_point = start_experiment_response.start_date;
        cv_server->episode_count = 0;
    }

    void Cv_server_experiment_client::on_episode_started(const string &experiment_name) {
        auto experiment = this->get_experiment(experiment_name);
        std::stringstream ss;
        ss << get_experiment_prefix(experiment_name) << '/' << experiment_name << "/episode_" << std::setw(3) << std::setfill('0') << experiment.episode_count;
        std::string destination_folder = ss.str();
        cv_server->episode_count++;
        cv_server->new_episode(experiment.subject_name, experiment.experiment_name, experiment.episode_count, experiment.world_info.occlusions, destination_folder);
    }

    mutex video_mutex;

    bool Cv_server::new_episode(const string &subject,
                                const string &experiment,
                                int episode,
                                const string &occlusions,
                                const string &folder) {
        cout << "new_episode" << endl;
        ts.reset();
        auto destination_folder = video_path + folder;
        if (main_video.is_open()) end_episode();
        std::filesystem::create_directories(destination_folder);
        cout << "Video destination folder: " + destination_folder << endl;
        main_layout.new_episode(subject, experiment, episode, occlusions);
        while(!video_mutex.try_lock()){
            cout << "failed to lock  Cv_server::new_episode" << endl;
            this_thread::sleep_for(10ms);
        }
        main_video = Video(main_layout.size(), Image::rgb);
        frame_number = 0;
        raw_video = Video(raw_layout.size(), Image::gray);
        zoom_video = Video(cv::Size(300,300), Image::gray);
        if (!main_video.new_video(destination_folder + "/main_" + experiment )) cout << "error creating video: " << destination_folder + "/main_" + experiment << endl;
        if (!raw_video.new_video(destination_folder + "/raw_" + experiment )) cout << "error creating video: " << destination_folder + "/raw_" + experiment << endl;;
        if (!zoom_video.new_video(destination_folder + "/mouse_" + experiment )) cout << "error creating video: " << destination_folder + "/mouse_" + experiment << endl;;;

#ifdef USE_SYNCHRONIZATION
        if (synchronization_enabled){
            synchronization_device->set_pin(mcp2221::GPIO::pin0, true);
        }
#endif

        sync_log.new_log(destination_folder + "/sync_" + experiment + ".json");
        video_mutex.unlock();
        waiting_for_prey = true;
        return true;
    }


    bool Cv_server::end_episode() {
        cout << "end_episode" << endl;
        waiting_for_prey = false;
#ifdef USE_SYNCHRONIZATION
        if (synchronization_enabled){
            synchronization_device->set_pin(mcp2221::GPIO::pin0, false);
        }
#endif

        thread( [this]() {
                    while(!video_mutex.try_lock()){
                        this_thread::sleep_for(10ms);
                    }
                    try {
                        main_video.close();
                        raw_video.close();
                        zoom_video.close();
                        sync_log.close();
                    } catch (...) {
                        cout << "failed closing videos Cv_server::end_episode" << endl;
                    };
                    video_mutex.unlock();
                }
        ).detach();
        return true;
    }

    bool Cv_server::get_mouse_step(const Binary_image &diff, Step &step, const Location &robot_location, float scale) {
        //PERF_START("MOUSE_DETECTIONS");
        auto detections = Detection_list::get_detections(diff).scale(scale);
        //PERF_STOP("MOUSE_DETECTIONS");
        //PERF_START("MOUSE_FILTER");
        auto mouse_candidates = detections.filter(mouse_profile);
        //PERF_STOP("MOUSE_FILTER");
        //PERF_SCOPE("MOUSE_REST");
        for (auto &mouse: mouse_candidates) {
            if (mouse.location.dist(robot_location) < SAFETY_MARGIN) continue;
            step.agent_name = "prey";
            step.location = mouse.location;
            return true;
        }
        return false;
    }

    bool Cv_server::get_robot_step(const Binary_image &image, Step &step, float scale) {
        auto leds = Detection_list::get_detections(image).scale(scale).filter(led_profile);
        if (leds.size() != 3) {
            return false;
        }
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

 enum Screen_image {
        main,
        difference,
        raw,
        zoom,
        cam0,
        cam1,
        cam2,
        cam3,
        sync_led,
        led,

    };

    void Cv_server::tracking_process() {
        bool show_magnets = false;
        Location magnet_distance(0, 12.5);
        Profile sync_led;
        sync_led.area_upper_bound=1000;
        sync_led.area_lower_bound=15;

        unsigned int parallel_threads = 2;
        vector<Composite> composites;
        vector<thread> composite_threads;
        Json_int_vector leds(4);
        vector<cv::Rect> sync_leds_rects;
        float sunc_rect_rect_size = 40;
        for (auto &ll:sync_led_locations) {
            sync_leds_rects.emplace_back(ll.x-sunc_rect_rect_size/2, ll.y-sunc_rect_rect_size/2, sunc_rect_rect_size, sunc_rect_rect_size);
        }
        for (unsigned int pt=0; pt < parallel_threads; pt++){
            composites.emplace_back(camera_configuration);
            composite_threads.emplace_back();
        }
        for (auto &composite:composites) {
            Image bg;
            auto images = cameras.capture();
            if (file_exists(background_path + "composite.png")) { // if there is a background image saved
                bg = Image::read(background_path, "composite.png");
            } else {
                composite.start_composite(images);
                bg = composite.get_detection();
                create_folder(background_path);
                bg.save(background_path, "composite.png");
            }
            composite.set_background(bg);
            composite.set_cameras_center(images);
            zoom_rectangles = composite.zoom_rectangles;
            zoom_size = composite.zoom_size;
        }
        json_cpp::Json_date::set_local_time_zone_offset();
        tracking_running = true;
        bool show_markers = false;
        puff_state = false;
        Step mouse;
        mouse.location = NOLOCATION;
        Step robot;
        robot.location = NOLOCATION;
        int robot_counter = 0;
        ts.reset();
        int robot_camera = -1;
        bool robot_detected;
        bool mouse_detected;
        string screen_text;
        Screen_image screen_image = Screen_image::main;
        double fps = Video::get_fps();
        double time_out = 1.0 / fps * .999;
        Step canonical_step;
        Timer frame_timer(time_out);
        Frame_rate fr;
        fr.filtered_fps = fps;
        fr.filter = .1;
        bool show_occlusions = false;
        int input_counter=0;
        unsigned int current_composite = 0;
        Location_list occlusions_locations;
        Location entrance_location = cv_space.transform( ENTRANCE, canonical_space);
        entrance_location.x += composites[0].padding;
        float entrance_distance = cv_implementation.cell_transformation.size / 2;
        bool show_robot_destination = false;
        unsigned int prey_entered_arena_indicator = 0;
        vector<int> frozen_camera_counters(4, 0);
        bool change_threshold = false;
        bool record_perf_data = false;
        mutex message_mtx;
        Json_float_vector latency;
        Json_float_vector throughput;
        while (tracking_running) {
            ////PERF_START("WAIT");
            while ((!unlimited || main_video.is_open()) && !frame_timer.time_out()) this_thread::sleep_for(100us);
            ////PERF_STOP("WAIT");
            frame_timer.reset();
            ////PERF_START("CAPTURE");
            auto images = cameras.capture();
            auto time_stamp = ts.to_seconds();
            ////PERF_STOP("CAPTURE");
            ////PERF_START("COMPOSITE");
            current_composite = (current_composite + 1) % 2;
            auto &composite = composites[current_composite];
            Timer latency_timer;
            composite.start_composite(images);
            ////PERF_STOP("COMPOSITE");
            ////PERF_START("COLOR CONVERSION");
            ////PERF_STOP("COLOR CONVERSION");
            ////PERF_START("ROBOT DETECTION");
            if (robot_camera == -1 || !composite.is_transitioning(robot.location)) {
                robot_detected = get_robot_step(composite.get_detection_threshold(robot_threshold), robot, composite.detection_scale);
                if (robot_detected) robot_camera = composite.get_best_camera(robot.location);
            } else {
                robot_detected = get_robot_step(composite.get_detection_threshold(robot_threshold, robot_camera), robot, composite.detection_scale);
            }
            if (robot_detected) {
                auto perspective_adjustment = composite.get_perspective_correction(robot.location, robot_height, robot_camera);
                robot.location += (-perspective_adjustment);
                robot_counter = 30;
            } else {
                if (robot_counter) robot_counter--;
                else robot.location = NOLOCATION;
            }
            //PERF_STOP("ROBOT DETECTION");
            //PERF_START("MOUSE DETECTION");
            //PERF_START("MOUSE_STEP");
            mouse_detected = get_mouse_step(composite.get_subtracted_threshold(mouse_threshold), mouse, robot.location, composite.detection_scale);
            if (mouse_detected) {
                //PERF_START("MOUSE_DETECTED");
                if (waiting_for_prey && mouse.location.dist(entrance_location) > entrance_distance) {
                    waiting_for_prey = false;
                    experiment_client.prey_enter_arena();
                    prey_entered_arena_indicator = 100;
                }
                //PERF_STOP("MOUSE_DETECTED");
            }
            //PERF_STOP("MOUSE_STEP");
            //PERF_STOP("MOUSE DETECTION");
            //PERF_START("ZOOM");
            if (mouse.location != NOLOCATION) {
                composite.start_zoom(mouse.location);
            } else {
                composite.get_zoom().setTo(0);
            }
            //PERF_STOP("ZOOM");
            //PERF_START("DETECTION_PROCESSING");
            //PERF_START("SCREEN");
            composite.get_video().circle(entrance_location, entrance_distance, {120, 120, 0}, false);

            //SYNC LED DETECTION
            Images sync_led_images;
            for (int l=0; l<4; l++){
                auto &li = sync_led_images.emplace_back(images[l](sync_leds_rects[l]), "");
                auto ld = li.threshold(220);
                auto lv = !(Detection_list::get_detections(ld).filter(sync_led).empty());
                leds[l] = lv;
            }

            //PERF_START("SCREEN_ROBOT");
            if (robot_detected) {
                auto color_robot = robot_color;
                if (puff_state) {
                    robot.data = "puff";
                    color_robot = cv::Scalar({0, 0, 255});
                    puff_state--;
                } else {
                    robot.data = "";
                }
                composite.get_video().circle(robot.location, 5, color_robot, true);
                composite.get_video().arrow(robot.location, to_radians(robot.rotation), 50, color_robot, 3);
                // TODO: check gabbie added also make it 2 cell size
                composite.get_video().circle(robot.location, cv_implementation.cell_transformation.size * capture_parameters.distance,color_robot, false);

                if ( show_robot_destination && robot_destination != NOLOCATION) {
                    auto robot_normalized_destination_cv = cv_space.transform(robot_normalized_destination, canonical_space);
                    auto robot_destination_cv = cv_space.transform(robot_destination, canonical_space);
                    auto gravity_adjustment_cv = cv_space.transform(gravity_adjustment, canonical_space);

                    composite.get_video().arrow(robot.location, robot_destination_cv, {0,255,0}, 1);
                    composite.get_video().arrow(robot.location, robot_normalized_destination_cv, {0,0,255}, 1);

                    composite.get_video().arrow(robot_normalized_destination_cv, robot_normalized_destination_cv + gravity_adjustment_cv, {255,0,0}, 1);
                }
            }
            //PERF_STOP("SCREEN_ROBOT");
            //PERF_START("SCREEN_MOUSE");
            if (mouse_detected) {
                composite.get_video().circle(mouse.location, 5, mouse_color, true);
            }
            //PERF_STOP("SCREEN_MOUSE");
            //PERF_START("SCREEN_MARKERS");
            if (show_markers) {
                for (unsigned int c = 0; c < 4; c++){
                    auto &ci = composite.get_raw(c);
                    auto ms = ci.get_markers();
                    for (auto &m : ms){
                        auto mwp = composite.get_warped_point(c, m.centroid);
                        auto ml = composite.get_video().get_location(mwp);
                        composite.get_video().circle(ml, 25, {0,0,255}, false);
                    }
                }
            }
            //PERF_STOP("SCREEN_MARKERS");
            //PERF_STOP("SCREEN");
            //PERF_START("MESSAGE");
            if (record_perf_data) {
                latency.push_back(latency_timer.to_seconds()*1000);
                throughput.push_back(fr.current_fps);
            }

            if (robot_detected || mouse_detected) {
                thread([this, &composite, &message_mtx](
                        bool robot_detected,
                        bool mouse_detected,
                        Step robot,
                        Step mouse,
                        float time_stamp,
                        unsigned int frame_number,
                        Tracking_server &tracking_server ) {
                    message_mtx.lock();
                    if (robot_detected) {
                        Predator_data predator_data;
                        predator_data.capture = puff_state ==  PUFF_DURATION;
                        predator_data.best_camera = composite.get_best_camera(robot.location);
                        predator_data.human_intervention = human_intervention;
                        robot.time_stamp = time_stamp;
                        robot.frame = frame_number;
                        robot.data = predator_data.to_json();
                        robot.location.x -= composite.padding;
                        auto step = robot.convert(cv_space, canonical_space);
                        tracking_server.send_step(step);
                    }
                    if (mouse_detected) {
                       mouse.time_stamp = time_stamp;
                       mouse.frame = frame_number;
                       mouse.rotation = 0;
                       mouse.location.x -= composite.padding;
                       tracking_server.send_step(mouse.convert(cv_space, canonical_space));
                    }
                    message_mtx.unlock();
                },
                       robot_detected,
                       mouse_detected,
                       robot,
                       mouse,
                       time_stamp,
                       frame_number,
                       reference_wrapper(tracking_server) ).detach();
            }
            //PERF_STOP("MESSAGE");
            //PERF_STOP("DETECTION_PROCESSING");
            //PERF_START("DISPLAY");
            Image screen_frame;
            switch (screen_image) {
                case Screen_image::main :
                    if (show_magnets){
                        for (const Cell &cell: composite.world.cells) {
                            composite.get_video().circle(cell.location + magnet_distance, 2, {0, 0, 255}, true);
                            composite.get_video().circle(cell.location - magnet_distance, 2, {0, 0, 255}, true);
                        }
                    }
                    if (show_occlusions) {
                        for (const Cell &occlusion: occlusions) {
                            composite.get_video().circle(composite.world.cells[occlusion.id].location, 20, {255, 0, 0}, true);
                        }
                    }
                    {
                        if (episode_count > 0) {
                            auto minutes = (int(experiment_timer.to_seconds()) / 60) % 60;
                            auto seconds = int(experiment_timer.to_seconds()) % 60;
                            auto time = (minutes < 10 ? "0" + to_string(minutes) : to_string(minutes)) + ":" +
                                        (seconds < 10 ? "0" + to_string(seconds) : to_string(seconds));
                            screen_frame = screen_layout.get_frame(composite.get_video(),
                                                                   "episode: " + to_string(episode_count - 1) + " time: " +
                                                                   time, fr.filtered_fps);
                        } else {
                            screen_frame = screen_layout.get_frame(composite.get_video(),
                                                                   "main", fr.filtered_fps);
                        }
                    }break;
                case Screen_image::difference :
                    screen_frame = screen_layout.get_frame(composite.get_subtracted_small(), "difference", fr.filtered_fps);
                    break;
                case Screen_image::sync_led :
                    screen_frame = screen_layout.get_frame(raw_layout.get_frame(sync_led_images), "sync led", fr.filtered_fps);
                    break;
                case Screen_image::led :
                    screen_frame = screen_layout.get_frame(Image(composite.get_detection_threshold(robot_threshold),""), "LEDs", fr.filtered_fps);
                    break;
                case Screen_image::zoom :
                    screen_frame = screen_layout.get_frame(composite.get_zoom(), "zoom", fr.filtered_fps);
                    break;
                case Screen_image::raw :
                    screen_frame = screen_layout.get_frame(composite.get_raw_composite(), "raw", fr.filtered_fps);
                    break;
                case Screen_image::cam0 :
                    screen_frame = screen_layout.get_frame(composite.get_detection_small(0), "cam0", fr.filtered_fps);
                    break;
                case Screen_image::cam1 :
                    screen_frame = screen_layout.get_frame(composite.get_detection_small(1), "cam1", fr.filtered_fps);
                    break;
                case Screen_image::cam2 :
                    screen_frame = screen_layout.get_frame(composite.get_detection_small(2), "cam2", fr.filtered_fps);
                    break;
                case Screen_image::cam3 :
                    screen_frame = screen_layout.get_frame(composite.get_detection_small(3), "cam3", fr.filtered_fps);
                    break;
            }
            //PERF_START("SHOW");
            if (main_video.is_open()) {
                screen_frame.circle({15, 15}, 10, {0, 0, 255}, true);
                if (mouse.location == NOLOCATION) {
                    screen_frame.circle({15, 15}, 5, {0, 0, 0}, true);
                }
            }
            for (int l=0;l<4;l++){
                if (leds[l]) {
                    screen_frame.circle({55 + (float)l * 15, 15}, 5, {0, 255, 0}, true);
                }
            }

            if (human_intervention) screen_frame.circle({35, 15}, 10, {255, 0, 0}, true);
            if (prey_entered_arena_indicator){
                prey_entered_arena_indicator--;
                screen_frame.circle({55, 15}, 10, {0, 255, 0}, true);
            }
            cv::imshow("Agent Tracking", screen_frame);
            //PERF_STOP("SHOW");
            //PERF_STOP("DISPLAY");
            if (!input_counter) {
                input_counter = 10;
                auto key = cv::waitKey(1);
                 switch (key) {
                    case 'I':
                        // start video recording
                        if (record_perf_data){
                            latency.save("latency_data.json");
                            latency.clear();
                            throughput.save("throughput_data.json");
                            throughput.clear();
                        }
                        record_perf_data = !reset_robot_connection;
                        break;
                    case 'K':
                        reset_robot_connection = true;
                        break;
                    case 'D':
                        show_robot_destination=!show_robot_destination;
                        break;
                    case 'C':
                        images.save(".");
                        break;
                    case 'T':
                        {
                            change_threshold = !change_threshold;
                            if (change_threshold) {
                                screen_image = Screen_image::led;
                                cout << "Threshold change enabled" << endl;
                            } else {
                                screen_image = Screen_image::main;
                                cout << "Threshold change disabled" << endl;
                            }
                        }
                        break;
                    case '!':
                    {
                        key = cv::waitKey();
                        int  door_number = key - 176;
                        if (door_number<=3) {
                            auto m = Message("open_door", door_number);
                            experiment_client.experiment_broadcast(m);
                        } else {
                            cout << "door number " << door_number << " not found." << endl;
                        }
                        break;
                    }
                    case '@':
                    {
                        key = cv::waitKey();
                        int  door_number = key - 176;
                        if (door_number<=3) {
                            auto m = Message("close_door", door_number);
                            experiment_client.experiment_broadcast(m);
                        } else {
                            cout << "door number " << door_number << " not found." << endl;
                        }
                        break;
                    }
                    case 'V':
                        // start video recording
                        if (main_video.is_open() && mouse.location==NOLOCATION){
                            mouse.location = Location(0, 0);
                        } else {
                            main_layout.new_episode("","",0,"");
                            main_video.new_video("main");
                            raw_video.new_video("raw");
                            zoom_video.new_video("mouse");
                            frame_number = 0;
                        }
                        break;
                    case 'B':
                        // end video recording
                        main_video.close();
                        raw_video.close();
                        zoom_video.close();
                        break;
                    case 'Q':
                        tracking_running = false;
                        break;
                    case 'M':
                        //show_markers = !show_markers;
                        break;
                    case 'R':
                        cameras.reset();
                        break;
                    case ']':
                        if (!change_threshold){
                            cout << "Threshold change disabled" << endl;
                        } else {
                            if (robot_threshold < 254) robot_threshold++;
                            cout << "Threshold set to " << robot_threshold << endl;
                        }
                        break;
                    case '[':
                        if (!change_threshold){
                            cout << "Threshold change disabled" << endl;
                        } else {
                            if (robot_threshold > 0) robot_threshold--;
                            cout << "Threshold set to " << robot_threshold << endl;
                        }
                        break;
                    case 'U':
//                        cout << "Are you sure? confirm by pressing the key again. Any other key to cancel." << endl;
//                        key = cv::waitKey(2000);
//                        if ( key == 'U') {
//                            cout << "Updating background" << endl;
                            for (auto &comp: composites)
                                comp.set_background(composite.get_detection());
                            composite.get_detection().save(background_path, "composite.png");
//                        } else {
//                            cout << "Canceled" << endl;
//                        }
                        break;
                    case 'L':
                        show_magnets = !show_magnets;
                        break;
                    case 'O':
                        show_occlusions = !show_occlusions;
                        break;
                    case '0' ... '8':
                        screen_image = static_cast<Screen_image>(key-'0');
                        break;
                    case '\t':
                        if (screen_image == Screen_image::led)
                            screen_image = Screen_image::main;
                        else
                            screen_image = static_cast<Screen_image>(screen_image + 1);
                        //cout << "change_screen_output to " << screen_image << endl;
                        break;
                    case ' ':
                        if (screen_image == Screen_image::main)
                            screen_image = Screen_image::led;
                        else
                            screen_image = static_cast<Screen_image>(screen_image - 1);
                        //cout << "change_screen_output to " << screen_image << endl;
                        break;
                    case 'S':
                        //"%Y-%m-%d %T"
                        auto file_name = json_cpp::Json_date::now().to_string("%Y%m%d_%H%M%S");
                        cv::imwrite(file_name + ".png", screen_frame);
                        cv::imwrite(file_name + "_composite.png", composite.get_composite());
                        cv::imwrite(file_name + "_darken.png", (composite.get_composite().to_gray() - 140)*(255/(255-140)));
                        cv::imwrite(file_name + "_robot.png", composite.get_composite().to_gray().threshold(robot_threshold));

                        cv::imwrite(file_name + "_raw.png", composite.get_raw_composite());
                        cv::imwrite(file_name + "_subtracted.png", composite.get_subtracted());
                        cv::imwrite(file_name + "_threshold.png", composite.subtracted_threshold);
                        auto t2 = screen_frame.threshold(60).negative();
                        cv::imwrite(file_name + "_t1.png", t2);
                        t2 = screen_frame.threshold(70).negative();
                        cv::imwrite(file_name + "_t2.png", t2);
                        t2 = screen_frame.threshold(80).negative();
                        cv::imwrite(file_name + "_t3.png", t2);
                        t2 = screen_frame.threshold(90).negative();
                        cv::imwrite(file_name + "_t4.png", t2);
                        t2 = screen_frame.threshold(100).negative();
                        cv::imwrite(file_name + "_t5.png", t2);
                        t2 = screen_frame.threshold(110).negative();
                        cv::imwrite(file_name + "_t6.png", t2);
                        t2 = screen_frame.threshold(120).negative();
                        cv::imwrite(file_name + "_t7.png", t2);
                        t2 = screen_frame.threshold(130).negative();
                        cv::imwrite(file_name + "_t8.png", t2);
                        t2 = screen_frame.threshold(140).negative();
                        cv::imwrite(file_name + "_t9.png", t2);
                        cv::imwrite(file_name + "_zoom.png", composite.get_zoom());

                        auto o = Image(composite.get_video().size,Image::Type::rgb);
                        for (const Cell &cell: composite.world.cells) {
                            o.circle(cell.location + magnet_distance, 2, {0, 0, 255}, true);
                            o.circle(cell.location - magnet_distance, 2, {0, 0, 255}, true);
                        }
                        cv::imwrite(file_name + "_predicted.png", o);
                        break;
                }
            } else {
                input_counter--;
            }
            fr.new_frame();
            //PERF_START("VIDEO");
            if (main_video.is_open() && mouse.location != NOLOCATION) { // starts recording when mouse crosses the door
                while(!video_mutex.try_lock()) this_thread::sleep_for(10us);
                if (main_video.is_open()) {
                    sync_log.sync_event(frame_number,time_stamp, leds);
                    thread([this, &composite](unsigned int frame_number) {
                        try {
                            auto main_frame = main_layout.get_frame(composite.get_video(), frame_number);
                            main_video.add_frame(main_frame);
                            raw_video.add_frame(composite.get_raw_composite());
                            zoom_video.add_frame(composite.get_zoom());
                        } catch (...) {
                            cout << "failed add frames Cv_server::cv_process" << endl;
                        }
                        video_mutex.unlock();
                    }, frame_number).detach();
                    frame_number++;
                }
            } else {
                mouse.location = NOLOCATION;
            }
            //PERF_STOP("VIDEO");
        }
        for (auto &t:composite_threads) if (t.joinable()) t.join();
    }

    Cv_server::Cv_server(Camera_configuration &camera_configuration,
                         const std::string &camera_configuration_file,
                         const std::string &background_path,
                         const std::string &video_path,
                         agent_tracking::Tracking_server &tracking_server,
                         Cv_server_experiment_client &experiment_client,
                         const Location_list &sync_led_locations,
                         const cell_world::Capture_parameters &capture_parameters,
                         bool unlimited):
            tracking_server(tracking_server),
            experiment_client(experiment_client),
            capture_parameters(capture_parameters),
            canonical_space(World_implementation::get_from_parameters_name("hexagonal","canonical").space),
            cv_implementation(World_implementation::get_from_parameters_name("hexagonal","cv")),
            cv_space(cv_implementation.space),
            unlimited(unlimited),
            camera_configuration(camera_configuration),
            cameras(camera_configuration_file, camera_configuration),
            main_video(main_layout.size(), Image::rgb),
            raw_video(raw_layout.size(), Image::gray),
            zoom_video(cv::Size(300,300), Image::gray),
            led_profile(Resources::from("profile").key("led").get_resource<Profile>()),
            mouse_profile(Resources::from("profile").key("mouse").get_resource<Profile>()),
            video_path(video_path),
            background_path(background_path),
            sync_led_locations(sync_led_locations)
    {
        experiment_client.cv_server = this;
#ifdef USE_SYNCHRONIZATION

        auto device_info = mcp2221::Device::get_devices();
        if (device_info.empty()){
            synchronization_enabled = false;
            cerr << "MCP2221 device not found" << endl;
        } else {
            cout << "MCP2221 device found" << endl;
            auto d = device_info[0];
            synchronization_device = new mcp2221::Device(d.path);
            synchronization_device->set_direction(mcp2221::pin0, mcp2221::OUTPUT);
            synchronization_device->set_direction(mcp2221::pin1, mcp2221::INTERRUPT);
            synchronization_device->open();
            synchronization_device->set_interrupt(mcp2221::RISING, [this](auto){
                cout << "sync signal received:" << ts.to_seconds() << endl;
                if (main_video.is_open()) {
                    sync_log.sync_signal(ts.to_seconds());
                }
                }, NULL);
            synchronization_enabled = true;
        }
        cout << "MCP2221 Syncronization enabled" << endl;
#endif
        unsigned int i=0;
        Camera::set_frame_size(camera_configuration.width,camera_configuration.height);
        for (auto &ca: cameras.cameras) {
            if (i < camera_configuration.offsets.size()) {
                ca->set_offset(camera_configuration.offsets[i].x, camera_configuration.offsets[i].y);
            }
            i++;
        }
    }

    void SyncLog::new_log(const string &pfile_path) {
        file_path = pfile_path;
        leds.clear();
        frames.clear();
        time_stamps.clear();
    }

    void SyncLog::sync_event(int frame, float time_stamp, const Json_int_vector &pleds) {
        if (!leds.empty()) {
            auto &b = leds.back();
            for (unsigned int i = 0; i < pleds.size(); i++) {
                if (b[i] != pleds[i]) {
                    leds.push_back(pleds);
                    frames.push_back(frame);
                    time_stamps.push_back(time_stamp);
                    return;
                }
            }
        }
        else
        {
            leds.push_back(pleds);
            frames.push_back(frame);
            time_stamps.push_back(time_stamp);
        }
    }

    void SyncLog::close() {
        this->save(file_path);
    }

    void SyncLog::sync_signal(float time_stamp) {
        sync_signals.push_back(time_stamp);
    }
}