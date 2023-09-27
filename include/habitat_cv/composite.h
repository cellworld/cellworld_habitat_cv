#pragma once
#include <habitat_cv/image.h>
#include <cell_world.h>
#include <habitat_cv/camera_configuration.h>
#include <thread>
#ifdef USE_CUDA
#include <thread>
#define cudaStreamNonBlocking 0x01
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#endif

namespace habitat_cv{
    struct Composite {
        Composite(Camera_configuration camera_configuration);
        void set_background(const Image &);
        cell_world::Polygon &get_polygon(const cell_world::Coordinates &);
        cell_world::Coordinates get_coordinates(const cell_world::Location &);
        cv::Point2f get_raw_point(unsigned int camera_index, const cv::Point2f &);
        cv::Point2f get_warped_point(unsigned int camera_index, const cv::Point2f &);
        static cv::Rect_<int> get_zoom_rect(cv::Size rs, cv::Size zs, cv::Point2i point, cv::Point2i &);
        void set_zoom_size(const cv::Size &);
        Image &get_zoom();
        void start_zoom(const cell_world::Location &);
        void start_composite (const Images &);
        Image &get_video();
        Image &get_subtracted();
        Image &get_subtracted_small();
        Image &get_detection();
        Image &get_detection_small();
        Image &get_detection_small(unsigned int);
        Binary_image &get_detection_threshold(unsigned char, int camera=-1);
        Binary_image &get_subtracted_threshold(unsigned char);
        Image &get_raw(unsigned int);
        Image &get_raw_composite();
        Image &get_composite();
        void set_cameras_center(const Images&);
        unsigned int get_best_camera(const cell_world::Location &);
        bool is_transitioning(const cell_world::Location &);
        cell_world::Location get_perspective_correction(const cell_world::Location &location, float height, int camera = -1);

        float camera_height = 205;  // 200 (hab0) cm
        int transition_size = 40;
        int padding = 50;

        Image zoom;
        Image background; // background image

        Image composite_subtracted; // composite image with background subtracted
        Image composite_subtracted_small; // composite image with background subtracted

        std::vector<cv::Point2f> zoom_rectangles;
        cv::Size zoom_size{150,150};

        Binary_image mask_detection;
        Binary_image mask_video;

        //threads
        std::thread zoom_thread;

        cell_world::World world;
        cell_world::Map map;
        cell_world::Polygon_list cells;
        cell_world::Location_list cameras_center;
        float detection_scale = 2;

        Binary_image detection_threshold;
        Binary_image subtracted_threshold;
        std::vector<Binary_image> detection_camera_threshold;
        Images freezing_control;
        cv::Rect freezing_control_rect;
        std::vector<bool> frozen_camera;
    private:
        Camera_configuration configuration;
        Images raw; // raw images from the cameras
        Images warped; // warped images

        cv::Size composite_size; //composite size
        cv::Size crop_size; //composite size
        Image composite; // composite without mask

        Image composite_video; //composite with video mask
        float video_padding = .05;

        Image composite_detection; // composite with detection mask
        cv::Size detection_small_size;
        Image composite_detection_small; // composite resized and with detection mask

        Images detection; // warped images with detection mask
        Images detection_small; // warped and resized images with detection mask

        Images raw_small;

        Image composite_raw; // composite without mask

        std::vector<cv::Mat> homographies;
        std::vector<cv::Mat> inverted_homographies;
        std::vector<cv::Rect> crop_rectangles;

#ifdef USE_CUDA
        cv::cuda::Stream gpu_raw_stream;
        cv::cuda::Stream gpu_video_stream;
        cv::cuda::Stream gpu_detection_stream;
        cv::cuda::Stream gpu_zoom_stream;
        cv::cuda::Stream gpu_subtracted_stream;
        std::vector<cv::cuda::Stream> gpu_detection_streams;

        std::vector<cv::cuda::GpuMat> gpu_raw;
        std::vector<cv::cuda::GpuMat> gpu_warped;
        std::vector<cv::cuda::GpuMat> gpu_detection;
        std::vector<cv::cuda::GpuMat> gpu_detection_small;

        cv::cuda::GpuMat gpu_zoom;
        cv::cuda::GpuMat gpu_background;
        cv::cuda::GpuMat gpu_composite;
        cv::cuda::GpuMat gpu_composite_subtracted;
        cv::cuda::GpuMat gpu_composite_subtracted_small;
        cv::cuda::GpuMat gpu_composite_detection;
        cv::cuda::GpuMat gpu_composite_detection_small;
        cv::cuda::GpuMat gpu_composite_video;
        cv::cuda::GpuMat gpu_composite_video_rgb;
        cv::cuda::GpuMat gpu_composite_raw;

        cv::cuda::GpuMat gpu_mask_detection;
        cv::cuda::GpuMat gpu_mask_video;
#endif
    };
}