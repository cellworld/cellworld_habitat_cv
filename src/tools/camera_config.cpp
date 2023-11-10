#include <iostream>
#include <opencv2/opencv.hpp>
#include <habitat_cv.h>
#include <habitat_cv/layouts.h>
#include <params_cpp.h>
#include "opencv2/video.hpp"
#include <cell_world.h>
#include <filesystem>

using namespace habitat_cv;
using namespace std;
using namespace cv;
using namespace cell_world;
using namespace json_cpp;


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

Point2f centroid;
float view_ratio = 1.0;

static void on_mouse(int event, int x, int y, int, void *){
    if (event!=EVENT_LBUTTONDOWN) return;

    centroid.x = float(x) / view_ratio;
    centroid.y = float(y) / view_ratio;
}

void show_small(const string &window_name, Image im){
    auto small_size = Size(im.size[1] * view_ratio, im.size[0] * view_ratio);
    Image small_image(small_size, im.type);
    resize(im, small_image,small_size, INTER_LINEAR);
    imshow(window_name, small_image);
}

void set_camera_config(Camera_configuration &cc, int camera_number, Image lim, Location_list &sync_led_locations){
    namedWindow("Homography", 0);
    moveWindow("Homography", 0,0);
    setMouseCallback("Homography", on_mouse, 0);
    auto im = lim;
    int camera_position = -1;
    cout << endl;
    cout << "0 | 1 " << endl;
    cout << "2 | 3 " << endl;
    auto p = 0;
    auto cp = -1;
    for (auto &row:cc.order){
        for (auto &cell:row){
            if ((int)cell==camera_number) cp = p;
            p++;
        }
    }
    cout << "current position " << cp << ": (0/1/2/3 or space to skip)" << endl;
    while (camera_position <0 || camera_position >3) {
        auto nim = im.to_rgb();
        show_small("Homography", im);
        auto key = waitKey(1);
        switch (key) {
            case '0':
                camera_position = 0;
                break;
            case '1':
                camera_position = 1;
                break;
            case '2':
                camera_position = 2;
                break;
            case '3':
                camera_position = 3;
                break;
            case ' ':
                camera_position = cp;
                break;
            case '+':
                view_ratio += .1;
                cout << "view_ratio: " << view_ratio << endl;
                destroyWindow("Homography");
                namedWindow("Homography", 0);
                moveWindow("Homography", 0,-100);
                setMouseCallback("Homography", on_mouse, 0);
                break;
            case '-':
                view_ratio -= .1;
                cout << "view_ratio: " << view_ratio << endl;
                destroyWindow("Homography");
                namedWindow("Homography", 0);
                moveWindow("Homography", 0,-100);
                setMouseCallback("Homography", on_mouse, 0);
                break;


        }
    }
    for (auto &row:cc.order){
        for (auto &cell:row){
            if ((int)cell==camera_number) cell = 4;
        }
    }
    cc.order[camera_position / 2][camera_position % 2] = camera_number;
    cout << "Configuring Homography" << endl;
    bool is_correct = false;
    while(!is_correct) {
        for (auto &c: cc.centroids[camera_number]) {
            cout << "Coordinates x: " << c.cell_coordinates.x << ", y:" << c.cell_coordinates.y << endl;
            centroid.x = c.centroid.x;
            centroid.y = c.centroid.y;
            bool done = false;
            while (!done) {
                auto key = waitKey(1);
                auto nim = im.to_rgb();
                auto mc = nim.get_location(centroid);
                nim.circle(mc, 10, Scalar(255, 0, 0));
                show_small("Homography", nim);
                switch(key){
                    case 'a':
                        centroid.x-=1;
                        break;
                    case 's':
                        centroid.y+=1;
                        break;
                    case 'd':
                        centroid.x+=1;
                        break;
                    case 'w':
                        centroid.y-=1;
                        break;
                    case '+':
                        view_ratio += .1;
                        cout << "view_ratio: " << view_ratio << endl;
                        destroyWindow("Homography");
                        namedWindow("Homography", 0);
                        moveWindow("Homography", 0,-100);
                        setMouseCallback("Homography", on_mouse, 0);
                        break;
                    case '-':
                        view_ratio -= .1;
                        cout << "view_ratio: " << view_ratio << endl;
                        destroyWindow("Homography");
                        namedWindow("Homography", 0);
                        moveWindow("Homography", 0,-100);
                        setMouseCallback("Homography", on_mouse, 0);
                        break;
                    case ' ':
                        done = true;
                }
            }
            c.centroid.x = centroid.x;
            c.centroid.y = centroid.y;
        }

        auto nim = im.to_rgb();

#ifdef USE_SYNCHRONIZATION
        cout << "Configuring Sync LED location" << endl;
        centroid.x = sync_led_locations[camera_number].x;
        centroid.y = sync_led_locations[camera_number].y;
        bool done = false;
        while (!done) {
            auto key = waitKey(1);
            auto nim = im.to_rgb();
            rectangle(nim, Point(centroid.x-20, centroid.y-20), Point(centroid.x+20, centroid.y+20), Scalar(0,0,255));
            show_small("Homography", nim);
            switch(key){
                case 'a':
                    centroid.x-=1;
                    break;
                case 's':
                    centroid.y+=1;
                    break;
                case 'd':
                    centroid.x+=1;
                    break;
                case 'w':
                    centroid.y-=1;
                    break;
                case '+':
                    view_ratio += .1;
                    cout << "view_ratio: " << view_ratio << endl;
                    destroyWindow("Homography");
                    namedWindow("Homography", 0);
                    moveWindow("Homography", 0,-100);
                    setMouseCallback("Homography", on_mouse, 0);
                    break;
                case '-':
                    view_ratio -= .1;
                    cout << "view_ratio: " << view_ratio << endl;
                    destroyWindow("Homography");
                    namedWindow("Homography", 0);
                    moveWindow("Homography", 0,-100);
                    setMouseCallback("Homography", on_mouse, 0);
                    break;
                case ' ':
                    done = true;
            }
        }
        sync_led_locations[camera_number].x = centroid.x;
        sync_led_locations[camera_number].y = centroid.y;

        rectangle(nim, Point(sync_led_locations[camera_number].x-20, sync_led_locations[camera_number].y-20), Point(sync_led_locations[camera_number].x+20, sync_led_locations[camera_number].y+20), Scalar(0,0,255));

#endif
        for (auto &c: cc.centroids[camera_number]) {
            auto mc = nim.get_location({c.centroid.x,c.centroid.y});
            nim.circle(mc, 10, Scalar(255, 0, 0));
        }
        bool answer = false;
        cout << "is this correct? (y/n)" << endl;
        while (!answer){
            show_small("Homography", nim);
            auto key = waitKey(1);
            switch (key) {
                case 'y':
                case 'Y':
                    answer = true;
                    is_correct = true;
                    break;
                case 'n':
                case 'N':
                    answer = true;
                    is_correct = false;
                    break;
            }
        }
    }
    destroyWindow("Homography");

}


void show_positions(Camera_configuration &camera_configuration){
    auto pos = 0;
    for (auto &row:camera_configuration.order){
        bool first = true;
        for (auto &cell:row) {
            if (!first) cout << " | ";
            first = false;
            cout << pos ++ << ":" << cell;
        }
        cout << endl;
    }
}

//void set_crop(Camera_configuration &cc, int camera_number, Image &im){
//    namedWindow("Crop", 0);
//    moveWindow("Crop", 0,0);
//    bool full_view = true;
//    while (cc.offsets.size() <= (unsigned int)camera_number) cc.offsets.emplace_back(0,0);
//    Point offset(cc.offsets[camera_number].x, cc.offsets[camera_number].y);
//    Point size(cc.width,cc.height);
//    bool done = false;
//    while (!done) {
//        auto key = waitKey(1);
//        if (full_view) {
//            auto nim = im.to_rgb();
//            rectangle(nim, offset, offset + size, Scalar(0,0,255), 3);
//            show_small("Crop", nim);
//        } else {
//            auto nim = im.to_rgb();
//            show_small("Crop", nim.crop({(float)offset.x, (float)offset.y}, cc.height, cc.width));
//        }
//
//        switch(key){
//            case '\t':
//                full_view = !full_view;
//                break;
//            case 'a':
//                if (offset.x > 0)
//                offset.x -= 1;
//                break;
//            case 'A':
//                if (offset.x > 10)
//                    offset.x -= 10;
//                break;
//
//            case 's':
//                if (offset.y < im.size[0] - cc.height - 1)
//                    offset.y += 1;
//
//                break;
//            case 'S':
//                if (offset.y + 10 < im.size[0] - cc.height - 1)
//                    offset.y += 10;
//                else
//                    offset.y = im.size[0] - cc.height - 1 ;
//                break;
//
//            case 'd':
//                if (offset.x < im.size[1] - cc.width - 1)
//                offset.x += 1;
//                break;
//            case 'D':
//                if (offset.x + 10 < im.size[1] - cc.width - 1)
//                    offset.x += 10;
//                else
//                    offset.x = im.size[1] - cc.width - 1;
//                break;
//
//            case 'w':
//                if (offset.y > 0)
//                offset.y -= 1;
//                break;
//            case 'W':
//                if (offset.y > 10)
//                    offset.y -= 10;
//                else
//                    offset.y -= 0;
//                break;
//
//            case '+':
//                view_ratio += .1;
//                cout << "view_ratio: " << view_ratio << endl;
//                destroyWindow("Crop");
//                namedWindow("Crop", 0);
//                moveWindow("Crop", 0,-100);
//                setMouseCallback("Crop", on_mouse, 0);
//                break;
//            case '-':
//                view_ratio -= .1;
//                cout << "view_ratio: " << view_ratio << endl;
//                destroyWindow("Crop");
//                namedWindow("Crop", 0);
//                moveWindow("Crop", 0,-100);
//                setMouseCallback("Crop", on_mouse, 0);
//                break;
//            case ' ':
//                done = true;
//        }
//    }
//    cc.offsets[camera_number].x = offset.x;
//    cc.offsets[camera_number].y = offset.y;
//    destroyWindow("Crop");
//
//}

int main(int argc, char **argv) {
    params_cpp::Parser p(argc,argv);

    Agent_tracker_configuration config;
    config.load("../config/agent_tracker_config.json");

    string cam_config = p.get(params_cpp::Key("-pc","--pixci_config"), "Default");
    string cam_file = "/usr/local/xcap/settings/xcvidset.fmt";
    if (cam_config!="Default"){
        cam_file = config.config_folder + "EPIX_" + cam_config + ".fmt";
    }
    auto homography_file = "homography_" + p.get(params_cpp::Key("-h","--homography"), "hab2");
    auto homography_file_path = config.config_folder + homography_file + ".json";
    auto camera_configuration = json_cpp::Json_from_file<Camera_configuration>(homography_file_path);

    auto sync_led_locations_file = "sync_led_locations_" + p.get(params_cpp::Key("-sll","--sync_led_locations"), "hab2");
    auto sync_led_locations_path = config.config_folder + sync_led_locations_file + ".json";
    auto sync_led_locations = json_cpp::Json_from_file<Location_list>(config.config_folder + sync_led_locations_file + ".json");

    Composite composite(camera_configuration);
    auto ca = new Camera_array(cam_file, camera_configuration);
    Raw_layout layout;
    auto ims = ca->capture();
    for (int i=0;i<4;i++) {
        auto frame_size = ims[i].size();
        cout << frame_size << endl;
    }
    show_positions(camera_configuration);
    namedWindow("cameras",0);
    moveWindow("cameras", 0, 0);
    bool raw_view = true;
    while(true){
        ims = ca->capture();
        if (raw_view){
            cv::imshow("cameras", layout.get_frame(ims));
        } else {
            composite.start_composite(ims);
            auto comp = composite.get_composite().clone();
            cv::imshow("cameras", comp);
        }
        cout << "frame_rates: ";
        for (int i=0;i<4;i++) {
            cout << ca->cameras[i]->frame_rate.filtered_fps << " ";
        }
        cout << "\r";
        auto key = cv::waitKey(1);
        switch (key){
            case '\t':
                raw_view = !raw_view;
                break;
            case 'b':
                cout << "saving images" << endl;
                for (int i=0;i<4;i++) {
                    cv::imwrite("cam_" + std::to_string(i) + ".png", ims[i]);
                }
                break;
            case 'h':
                for (int i=0;i<4;i++) {
                    set_camera_config(camera_configuration,i,ims[i], sync_led_locations);
                }
                composite = Composite(camera_configuration);
                show_positions(camera_configuration);
                break;
            case 'c': {
                string cs;
                Camera::frame_size = Camera::original_frame_size;
                auto full_config = camera_configuration;
                for (auto &o:full_config.offsets) o = Coordinates(0,0);
                if (camera_configuration.width == -1) camera_configuration.width = ims[0].size[0];
                if (camera_configuration.height == -1) camera_configuration.height = ims[0].size[1];

                cout << endl << "current crop size: (" << camera_configuration.width << " x " << camera_configuration.height << ")" << endl;
                cout << "new width (" << camera_configuration.width << "): ";
                cin >> cs;
                if (!cs.empty()) camera_configuration.width = stoi(cs);
                cout << "new height (" << camera_configuration.height << "): ";
                cin >> cs;
                if (!cs.empty()) camera_configuration.height = stoi(cs);
                cout << endl;
                cout << "Crop offsets: " << endl;
                for (unsigned int i = 0; i < camera_configuration.order.count(); i++) {
                    if (camera_configuration.offsets.size()<i+1){
                        camera_configuration.offsets.push_back(Coordinates());
                    }
                }
                for (unsigned int i = 0; i < camera_configuration.offsets.size(); i++){
                    auto &o = camera_configuration.offsets[i];
                    cout <<  "Camera " << i << " current offset ( x=" << o.x << ", y=" << o.y << "): " << endl;
                    cout << "x: " << endl;
                    cin >> cs;
                    if (!cs.empty()) o.x = stoi(cs);
                    cout << "y: " << endl;
                    cin >> cs;
                    if (!cs.empty()) o.y = stoi(cs);
                }

                break;
            }
            case 'q':
                exit(0);
            case 's':
                cout << endl << "config name: " << endl;
                string config_name;
                cin >> config_name;
                filesystem::copy(cam_file,config.config_folder + "EPIX_" + config_name + ".fmt", filesystem::copy_options::update_existing);
                camera_configuration.save(config.config_folder + "homography_" + config_name + ".json");
                create_folder("../backgrounds/" + config_name);
                for (int i = 0; i < 4; i++) {
                    cv::imwrite(config.backgrounds_folder + config_name + "/background_" + std::to_string(i) + ".png", ims[i]);
                }
#ifdef USE_SYNCHRONIZATION
                sync_led_locations.save(config.config_folder + "sync_led_locations_" + config_name + ".json");
#endif
                break;
        }
    }
}