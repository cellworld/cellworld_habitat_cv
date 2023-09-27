#include <string>
#include <habitat_cv/layouts.h>
#include <opencv2/opencv.hpp>

using namespace habitat_cv;
using namespace std;
namespace habitat_cv {
    Main_layout::Main_layout() :
            Layout(1125, 1180, Image::Type::rgb),
            composite({0.0, 45.0}, {1180.0, 1030.0}, Image::Type::rgb),
            date_time({590, 45}, Image::Type::rgb, {255, 255, 255}, {30, 30, 30}, .8, 0, 1),
            occlusions({590, 45}, Image::Type::rgb, {255, 255, 255}, {30, 30, 30}, .8, 2, 1),
            subject({590, 45}, Image::Type::rgb, {255, 255, 255}, {30, 30, 30}, .8, 0, 1),
            experiment({900, 45}, Image::Type::rgb, {255, 255, 255}, {30, 30, 30}, .8, 0, 1),
            episode({590, 45}, Image::Type::rgb, {255, 255, 255}, {30, 30, 30}, .8, 2, 1),
            frame({280, 45}, Image::Type::rgb, {255, 255, 255}, {30, 30, 30}, .8, 2, 1) {
        add_place_holder(composite, {0, 0});
        add_place_holder(date_time, {0, 1080});
        add_place_holder(subject, {0, 1035});
        add_place_holder(experiment, {0, 990});
        add_place_holder(occlusions, {590, 1080});
        add_place_holder(episode, {590, 1035});
        add_place_holder(frame, {900, 990});
    }

    string get_time(double time_stamp) {
        ostringstream oss;
        unsigned int hour = (unsigned int) time_stamp / 60 / 60;
        unsigned int minute = (unsigned int) (time_stamp / 60) % 60;
        unsigned int second = (unsigned int) (time_stamp) % 60;
        unsigned int millisecond = (unsigned int) (time_stamp * 1000) % 1000;
        oss << setfill('0') << setw(2) << hour << ":" << setw(2) << minute << ":" << setw(2) << second << "." << setw(3)
            << millisecond;
        return oss.str();
    }

    using namespace std::chrono_literals;
    using namespace std::chrono;

    habitat_cv::Image Main_layout::get_frame(const Image &image, unsigned int frame_count) {
        composite = image;
        frame = "Frame: " + to_string(frame_count) + "  ";
        date_time = "  " + json_cpp::Json_date::now().to_string();
        return get_image();
    }

    void Main_layout::new_episode(std::string subject_name, std::string experiment_identifier, int episode_number,
                                  std::string occlusions_name) {
        subject = "  Subject: " + subject_name;
        experiment = "  Experiment: " + experiment_identifier;
        episode = "Episode: " + to_string(episode_number) + "  ";
        occlusions = "World: " + occlusions_name + "  ";
    }

    Raw_layout::Raw_layout() :
            Layout(1080, 1080, Image::Type::gray),
            panel0({540, 540}, Image::Type::gray),
            panel1({540, 540}, Image::Type::gray),
            panel2({540, 540}, Image::Type::gray),
            panel3({540, 540}, Image::Type::gray) {
        add_place_holder(panel0, {0, 0});
        add_place_holder(panel1, {540, 0});
        add_place_holder(panel2, {0, 540});
        add_place_holder(panel3, {540, 540});
    }

    habitat_cv::Image Raw_layout::get_frame(const Images &images) {
        panel0 = images[0];
        panel1 = images[1];
        panel2 = images[2];
        panel3 = images[3];
        return get_image();
    }

    Screen_layout::Screen_layout() :
            Layout(860, 880, Image::rgb),
            screen({880, 800}, Image::rgb),
            screen_text({880, 30}, Image::rgb, {255, 255, 255}, {30, 30, 30}, 1, 1, 1),
            fps_text({880, 30}, Image::rgb, {255, 255, 255}, {30, 30, 30}, 1, 2, 1) {
        add_place_holder(fps_text, {0, 0});
        add_place_holder(screen, {0, 30});
        add_place_holder(screen_text, {0, 830});
    }

    habitat_cv::Image Screen_layout::get_frame(const Image &image, const string &text, float fps) {
        screen = image.to_rgb();
        screen_text = text;
        fps_text = "fps: " + to_string(int(fps));
        return get_image();
    }

    Mouse_layout::Mouse_layout():
    Layout(300,300, Image::gray),
    mouse({300,300}, Image::gray){

    }

    habitat_cv::Image Mouse_layout::get_frame(const Image &mouse_image) {
        mouse = mouse_image.to_gray();
        return get_image();
    }
}