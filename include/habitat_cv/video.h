#pragma once

#ifdef USE_CUDA_2
#include <opencv2/cudacodec.hpp>
#else
#include "opencv2/video.hpp"
#endif

#include <habitat_cv/image.h>
#include <thread>

namespace habitat_cv {
    struct Video {
        Video(const cv::Size &size, Image::Type type);
        ~Video();
        bool new_video(const std::string &file_name, int fps = 90);
        bool close();
        bool add_frame(const Image &);
        bool is_open() const;
        std::string file_name;
        int frame_count;
        cv::Size size;
        Image::Type type;
        cell_world::Timer queue_check;
        int fourcc;
        std::string extension;
#ifdef USE_CUDA_2
        cv::Ptr<cv::cudacodec::VideoWriter> writer;
#else
        cv::VideoWriter writer;
#endif
        std::thread *writer_thread = nullptr;
        std::queue<Image> pending_frames;
        std::atomic<bool> *running = nullptr;
        void split_video(const std::vector<cv::Point2f> &, const cv::Size &);
    };

}