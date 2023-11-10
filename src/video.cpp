#include <habitat_cv/video.h>
#include <mutex>


using namespace std;

namespace habitat_cv {

    unsigned int fps = 90;



    Video::Video(const cv::Size &size, Image::Type type):
    frame_count(-1),
    size(size),
    type(type),
    queue_check(5)
    {
        if (type == Image::rgb) {
            fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
            extension  = ".mp4";
        } else {
            fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            extension  = ".avi";
        }
// WORKING BUT VERY BIG VIDEOS
//        if (type == Image::rgb) {
//            fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
//            extension  = ".mp4";
//        } else {
//            fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
//            extension  = ".avi";
//        }
    }


    bool Video::add_frame(const Image &frame) {
        if (!running) return false;
        if (queue_check.time_out()){
            queue_check.reset();
            cout << file_name << " queue size -> " << pending_frames.size() << endl;
        }
        // **** will lower framerate if queue gets too large to prevent system crash ****
        while(pending_frames.size()>100) this_thread::sleep_for(10us);

        if (frame.type != this->type || frame.size() != this->size) {
            Image temp;
            cv::resize(frame, temp, this->size);
            pending_frames.emplace(temp);
        } else {
            pending_frames.emplace(frame);
        }
        return true;
    }

    Video::~Video() {
        close();
    }

    bool Video::new_video(const std::string &new_file_name) {
        if (running) return false;
        close();
        running = new atomic_bool(true);
        file_name = new_file_name;

#ifdef USE_CUDA_2
        writer = cv::cudacodec::createVideoWriter(file_name + extension, size, fps);
        if (writer) {
            writer_thread = new thread([this]() {
                frame_count = 0;
                while (*running || !pending_frames.empty()) {
                    cv::cuda::GpuMat cuda_frame;
                    while (*running && pending_frames.empty());
                    if (!pending_frames.empty()) {
                        auto frame = pending_frames.front();
                        pending_frames.pop();
                        if (frame.type != this->type || frame.size() != this->size) continue;
                        cuda_frame.upload(frame);
                        writer->write(cuda_frame);
                        frame_count++;
                    }
                }
                pending_frames = std::queue<Image>();
                writer.release();
            });
            return true;
        }
#else
        if (writer.open(file_name + extension, fourcc, fps, size, type == Image::rgb)) {
            writer_thread = new thread([this]() {
                frame_count = 0;
                while (*running || !pending_frames.empty()) {
                    while (*running && pending_frames.empty());
                    if (!pending_frames.empty()) {
                        auto frame = pending_frames.front();
                        pending_frames.pop();
                        writer.write(frame);
                        frame_count++;
                    }
                }
                pending_frames = std::queue<Image>();
                writer.release();
            });
            return true;
        }
#endif
        return false;
    }

    bool Video::close() {
        if (running) {
            *running = false;
        }
        if (writer_thread && writer_thread->joinable()) writer_thread->join();
        delete running;
        delete writer_thread;
        running = nullptr;
        writer_thread = nullptr;
        frame_count = 0;
        return true;
    }

    bool Video::is_open() const {
        return running && * running;
    }

    void Video::set_fps(unsigned int new_fps) {
        fps = new_fps;
    }

    unsigned int Video::get_fps() {
        return fps;
    }

    void Video::split_video(const vector <cv::Point2f> &tls, const cv::Size &crop_size) {
        vector<Video *> cropped_videos;
        vector<cv::Rect_<int>> crop_rects;
        for (unsigned int parts_counter = 0; parts_counter < tls.size(); parts_counter++) {
            auto cropped_video = cropped_videos.emplace_back(new Video(crop_size, type));
            cropped_video->new_video(file_name + "_" + to_string(parts_counter));
            crop_rects.emplace_back(tls[parts_counter], crop_size);
        }

        cv::VideoCapture capture(file_name + extension);
        while (true) {
            cv::Mat frame;
            // Capture frame-by-frame
            capture >> frame;
            if (frame.empty())
                break;
            Image im_frame(frame,"");
            if (im_frame.type!=type){
                if (type == Image::gray) im_frame = im_frame.to_gray();
                if (type == Image::rgb) im_frame = im_frame.to_rgb();
            }
            for (unsigned int parts_counter = 0; parts_counter < tls.size(); parts_counter++) {
                Image cropped_frame(crop_size, type);
                im_frame(crop_rects[parts_counter]).copyTo(cropped_frame);
                cropped_videos[parts_counter]->add_frame(cropped_frame);
            }
        }
        capture.release();
        for (unsigned int parts_counter = 0; parts_counter < tls.size(); parts_counter++) {
            cropped_videos[parts_counter]->close();
            delete cropped_videos[parts_counter];
        }
    }
}
