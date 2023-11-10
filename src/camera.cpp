#include<habitat_cv/camera.h>
#include <xcliball.h>

using namespace std;

namespace habitat_cv{

    cv::Size Camera::frame_size;
    cv::Size Camera::original_frame_size;


    void capture_process (Camera *camera){
        camera->running = true;
        long prev = -1;
        int size = Camera::frame_size.height * Camera::frame_size.width;
        pxd_goLivePair(camera->grabber_bit_map,1,2);
        while (camera->running){
            int destination = (camera->current_frame + 1) % (int)camera->buffer.size();
            while(pxd_capturedBuffer(camera->grabber_bit_map)==prev && camera->running );
            prev = pxd_capturedBuffer(camera->grabber_bit_map);
            camera->frame_rate.new_frame();
            if (pxd_readuchar(camera->grabber_bit_map, prev, 0, 0, -1, -1, camera->buffer[destination].data, size,"Grey") > 0) {
                camera->buffer[destination].time_stamp.reset();
                camera->current_frame = destination;
            }
        }
    }

    Camera::Camera(int camera_number, int buffer_size) : grabber_bit_map(1 << camera_number){
        for (int i=0; i<buffer_size; i++) {
            buffer.emplace_back(frame_size.height, frame_size.width, Image::Type::gray);
        }
        current_frame = -1;
        last_read_frame = - 1;
        capture_thread = std::thread(capture_process,this);
        while (current_frame == -1);
    }

    void Camera::init(const std::string &config_file) {
        pxd_PIXCIopen("-DM 0xF", "", config_file.c_str());
    }



    Image &Camera::get_current_frame() {
        while (last_read_frame == current_frame) this_thread::sleep_for(1ms); //waits until the next frame is ready
        last_read_frame = current_frame;
        return buffer[current_frame];
    }

    Camera::~Camera() {
        running = false;
        if (capture_thread.joinable()) capture_thread.join();
    }

    void Camera::close() {
        pxd_PIXCIclose();
    }

    Camera::Camera(int grabber_bit_map):Camera(grabber_bit_map, 5) {
    }

    void Camera::set_offset(int ox, int oy) {
        offset_x = ox;
        offset_y = oy;
    }

    void Camera::set_frame_size(int width, int height) {
        if (width==-1) width = pxd_imageXdim();
        if (height==-1) height = pxd_imageYdim();
        frame_size = {width, height};
    }
}