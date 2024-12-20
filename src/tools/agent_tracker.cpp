#include <mutex>
#include <habitat_cv/cv_service.h>
#include <easy_tcp.h>
#include <experiment/experiment_client.h>
#include <sstream>
#include <iomanip>
#include <controller.h>
#include <robot_lib.h>
#include <experiment/experiment_service.h>
#include <params_cpp.h>

using namespace controller;
using namespace cell_world;
using namespace std;
using namespace json_cpp;
using namespace agent_tracking;
using namespace habitat_cv;
using namespace experiment;

struct Agent_tracker_configuration : Json_object {
    Json_object_members(
            Add_member(logs_folder);
            Add_member(videos_folder);
            Add_member(backgrounds_folder);
            Add_member(config_folder);
    )
    string logs_folder;
    string videos_folder;
    string backgrounds_folder;
    string config_folder;
};

struct Thresholds : Json_object {
    Json_object_members(
            Add_member(mouse_threshold);
            Add_member(robot_threshold);
    )
    int mouse_threshold = 90;
    int robot_threshold = 250;
};

int main(int argc, char **argv){

    params_cpp::Parser p(argc,argv);

    Agent_tracker_configuration config;
    config.load("../config/agent_tracker_config.json");

    controller::Agent_operational_limits limits;

    bool fake_robot = p.contains(params_cpp::Key("-fr"));

    auto hab_config = p.get(params_cpp::Key("-hc", "--habitat_configuration"), "");

    auto robot_operational_limits_file = p.get(params_cpp::Key("-rol", "--robot_operational_limits"), "robot_operational_limits.json");

    limits.load("../config/" + robot_operational_limits_file); // robot, ghost

    Thresholds thresholds;
    auto thresholds_file = p.get(params_cpp::Key("-tf", "--thresholds_file"), "");
    if (!thresholds_file.empty()) {
        thresholds.load("../config/" + thresholds_file);
    }

    auto  robot_pid_values_file = p.get(params_cpp::Key("-rpv", "--robot_pid_values_file"), "pid.json");


    auto robot_ip = p.get(params_cpp::Key("-ri", "--robot_ip"), "192.168.137.155");
    auto robot_port = stoi(p.get(params_cpp::Key("-rp", "--robot_port"), "4500"));
    auto occlusions_str = p.get(params_cpp::Key("-w", "--world"),"21_05");
    auto manually_armed = p.contains(params_cpp::Key("-m", "--manually_armed"));
    auto configuration = Resources::from("world_configuration").key("hexagonal").get_resource<World_configuration>();
    auto implementation = Resources::from("world_implementation").key("hexagonal").key("canonical").get_resource<World_implementation>(); // mice, vr, canonical
    auto occlusions = Resources::from("cell_group").key("hexagonal").key(occlusions_str).key("occlusions").get_resource<Cell_group_builder>();
    auto capture_parameters = Resources::from("capture_parameters").key(p.get(params_cpp::Key("-cp", "--capture-parameters"), "default")).get_resource<Capture_parameters>();
    auto peeking_parameters = Resources::from("peeking_parameters").key(p.get(params_cpp::Key("-pp", "--peeking-parameters"), "default")).get_resource<Peeking_parameters>();
    auto raw_video_frame_rate = stoi(p.get(params_cpp::Key("-rv", "--raw_video"),"90"));
    auto world = World(configuration, implementation, occlusions);
    auto cells = world.create_cell_group();
    Map map(cells);
    Location_visibility visibility(cells, configuration.cell_shape, implementation.cell_transformation);
    Capture capture(capture_parameters, world);
    Peeking peeking(peeking_parameters, world);

    experiment::Experiment_server experiment_server;
    Experiment_service::set_logs_folder(config.logs_folder);
    experiment_server.start(Experiment_service::get_port()); // added by gabbie // 4540

    Tracking_server tracking_server;
    string cam_config = p.get(params_cpp::Key("-pc","--pixci_config"), hab_config);
    string cam_file = config.config_folder + "EPIX_" + cam_config + ".fmt";
    string bg_path = config.backgrounds_folder + cam_config + "/";


    auto &experiment_client = experiment_server.create_local_client<Cv_server_experiment_client>();
    experiment_client.subscribe();
    auto homography_file = "homography_" + p.get(params_cpp::Key("-h","--homography"), hab_config);
    auto camera_configuration = json_cpp::Json_from_file<Camera_configuration>(config.config_folder + homography_file + ".json");

    auto sync_led_locations_file = "sync_led_locations_" + p.get(params_cpp::Key("-sll","--sync_led_locations"), hab_config);
#ifdef USE_SYNCHRONIZATION
    auto sync_led_locations = json_cpp::Json_from_file<Location_list>(config.config_folder + sync_led_locations_file + ".json");
#else
    auto sync_led_locations = Location_list();
#endif
    Cv_server cv_server(camera_configuration, cam_file, bg_path, config.videos_folder, tracking_server, experiment_client, sync_led_locations, capture_parameters, p.contains(params_cpp::Key("-u")), raw_video_frame_rate);
    cv_server.mouse_threshold = thresholds.mouse_threshold;
    cv_server.robot_threshold = thresholds.robot_threshold;
    auto &experiment_tracking_client = tracking_server.create_local_client<Experiment_tracking_client>();
    experiment_tracking_client.subscribe();

    thread *fake_robot_thread = NULL;
    atomic<bool> fake_robot_running = false;
    auto fake_robot_location = Location(.5,.5); // Container of Locations
    if (fake_robot) {
        fake_robot_running = true;
        fake_robot_thread = new thread([&fake_robot_location, &tracking_server, &fake_robot_running](){
            int frame = 0;
            Timer d;
            auto step = Step();
            // Send out "predator_step" while we are running fake_robot
            while (fake_robot_running) {
                Timer t(.05);
                while (!t.time_out());
                step.agent_name = "predator";
                step.frame = frame ++;
                step.time_stamp = d.to_seconds();
                step.rotation = to_degrees(step.location.atan(fake_robot_location));
                step.location = fake_robot_location;
                tracking_server.send_step(step);
            }
        });
    }

    experiment_server.set_tracking_client(experiment_tracking_client);

    cv_server.occlusions = world.create_cell_group().occluded_cells();

    World_info wi;
    wi.world_configuration = "hexagonal";
    wi.world_implementation = "mice";
    wi.occlusions = "00_00";

    auto &controller_tracking_client = tracking_server.create_local_client<Controller_server::Controller_tracking_client>(
            visibility,
            float(360), //180 degrees each side -- sounds good?
            capture,
            peeking,
            "predator",
            "prey");
//    controller_tracking_client.subscribe();

    auto &controller_experiment_client = experiment_server.create_local_client<Controller_server::Controller_experiment_client>();
    controller_experiment_client.subscribe();


    robot::Robot_agent robot(limits, cv_server.reset_robot_connection);

    if (!p.contains(params_cpp::Key("-n"))) {
        if (!robot.connect(robot_ip, robot_port)){
            cout << "Failed to connect to robot" << endl;
            //exit(1);
        }
    }

    Controller_service::set_logs_folder("controller/");
    Controller_server controller_server("../config/" + robot_pid_values_file, robot, controller_tracking_client, controller_experiment_client,
                                        cv_server.robot_destination,
                                        cv_server.robot_normalized_destination,
                                        cv_server.gravity_adjustment,
                                        manually_armed);

    if (!controller_server.start(Controller_service::get_port())) {
        cout << "failed to start controller" << endl;
        exit(1);
    }

//     initial corrector
    tracking_server.start(Tracking_service::get_port());
    cv_server.tracking_process();
    tracking_server.stop();
    experiment_client.disconnect();
    if (fake_robot_running) {
        fake_robot_running = false;
        if (fake_robot_thread->joinable()) fake_robot_thread->join();
        delete (fake_robot_thread);
    }
    exit(0);
}