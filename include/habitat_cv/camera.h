#pragma once
#include <habitat_cv/image.h>
#include <habitat_cv/frame_rate.h>
#include <thread>
namespace habitat_cv {
    struct Camera {
        Camera(int, int);
        Camera(int);
        Image &get_current_frame();
        int grabber_bit_map;
        Images buffer;
        std::atomic<int> current_frame;
        int last_read_frame;
        static void init(const std::string &config_file);
        static void close();
        static void set_frame_size(int width, int height);
        static cv::Size frame_size;
        static cv::Size original_frame_size;
        std::atomic<bool> running;
        std::thread capture_thread;
        ~Camera();
        Frame_rate frame_rate;
        void set_offset(int, int);
        std::atomic<int> offset_x = 0;
        std::atomic<int> offset_y = 0;

    };
}