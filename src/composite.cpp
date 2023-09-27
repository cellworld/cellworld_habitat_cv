#include <habitat_cv/composite.h>
#include <cell_world.h>
#include <utility>
#include <performance.h>

using namespace json_cpp;
using namespace std;
using namespace cell_world;

namespace habitat_cv {
    Composite::Composite(Camera_configuration camera_configuration) :
    configuration(std::move(camera_configuration)){
        auto wc = Resources::from("world_configuration")
                .key("hexagonal")
                .get_resource<World_configuration>();
        auto wi = Resources::from("world_implementation")
                .key("hexagonal")
                .key("cv")
                .get_resource<World_implementation>();

        for (auto &c:wi.cell_locations) {
            c.x += padding;
        }
        wi.space.center.x += padding;
        cells = Polygon_list(wi.cell_locations, wc.cell_shape, Transformation(wi.cell_transformation.size , wi.cell_transformation.rotation + wi.space.transformation.rotation));
        world = World(wc, wi);
        map = Map(world.create_cell_group());
        composite_size = cv::Size(wi.space.transformation.size + 2 * padding, wi.space.transformation.size);
        crop_size =cv::Size (composite_size.width / configuration.order.cols(), composite_size.height / configuration.order.rows());

        composite_video = Image(composite_size.height, composite_size.width, Image::Type::rgb);
        composite_raw = Image(composite_size.height, composite_size.width, Image::Type::gray);
        composite = Image(composite_size.height, composite_size.width, Image::Type::gray);
        background = Image(composite_size.height, composite_size.width, Image::Type::gray);
        composite_subtracted = Image(composite_size.height, composite_size.width, Image::Type::gray);
        composite_detection = Image(composite_size.height, composite_size.width, Image::Type::gray);
        detection_small_size = cv::Size((float)composite_size.width / detection_scale, (float)composite_size.height / detection_scale);
        composite_detection_small = Image(detection_small_size, Image::gray);
        composite_subtracted_small = Image(detection_small_size, Image::gray);
        zoom = Image(zoom_size.height * configuration.order.cols(), zoom_size.width * configuration.order.rows(),Image::Type::gray);

        // generate masks
        Image detection_mask_image(composite_size.height, composite_size.width, Image::Type::gray);
        detection_mask_image.clear();
        // we added 50 px padding for detection, if this fails or have unintended consequences they should be removed
        Polygon detection_polygon(world.space.center, world.space.shape, {world.space.transformation.size + 50,world.space.transformation.rotation});
        detection_mask_image.polygon(detection_polygon,{255},true);
        mask_detection = detection_mask_image.threshold(0);

        Image video_mask_image(composite_size.height, composite_size.width, Image::Type::gray);
        auto t = world.space.transformation;
        t.size *= (1 + video_padding);
        Polygon video_polygon(world.space.center, world.space.shape, t);
        video_mask_image.polygon(video_polygon,{255},true);
        mask_video = video_mask_image.threshold(0);
        freezing_control_rect = cv::Rect(500,500,5,5);
        auto new_size = composite_size;
        for (unsigned int c = 0; c < configuration.order.count(); c++) {
            warped.emplace_back(composite_size, Image::Type::gray);
            detection.emplace_back(new_size, Image::Type::gray);
            detection_small.emplace_back(detection_small_size, Image::Type::gray);
            raw_small.emplace_back(crop_size, Image::Type::gray);
            freezing_control.emplace_back(cv::Size(5,5), Image::Type::gray);
            frozen_camera.push_back(false);
        }

#ifdef USE_CUDA
        gpu_composite.upload(composite);
        gpu_composite_raw.upload(composite_raw);
        gpu_background.upload(background);
        gpu_composite_subtracted.upload(composite_subtracted);
        gpu_composite_video.upload(composite_video);
        gpu_composite_detection.upload(composite_detection);
        gpu_composite_detection_small.upload(composite_detection_small);
        gpu_mask_detection.upload(mask_detection);
        gpu_mask_video.upload(mask_video);
        gpu_zoom.upload(zoom);
        for (unsigned int c = 0; c < configuration.order.count(); c++) {
            gpu_zoom_stream = cv::cuda::Stream(cudaStreamNonBlocking);
            gpu_detection_streams.emplace_back(cudaStreamNonBlocking);
            gpu_raw.emplace_back(raw_small[c]);
            gpu_warped.emplace_back().upload(warped[c]);
            gpu_detection.emplace_back().upload(detection[c]);
            gpu_detection_small.emplace_back().upload(detection_small[c]);
        }
#endif

        //creates the crop rectangles for each camera
        for (unsigned int c = 0; c < configuration.order.count(); c++) {
            auto camera_coordinates = configuration.order.get_camera_coordinates(c);
            cv::Point crop_location (camera_coordinates.x * crop_size.width,
                                     camera_coordinates.y * crop_size.height);
            crop_rectangles.emplace_back(crop_location, crop_size);
            cv::Point2f zoom_location (camera_coordinates.x * zoom_size.width,
                                       camera_coordinates.y * zoom_size.height);
            zoom_rectangles.emplace_back(zoom_location);
            vector<cv::Point2f> src_cp;
            vector<cv::Point2f> dst_cp;
            for (auto &a:configuration.centroids[c]) {
                src_cp.emplace_back(a.centroid.x,a.centroid.y );
                dst_cp.emplace_back(composite_video.get_point(map.cells[map.find(a.cell_coordinates)].location));
            }
            inverted_homographies.emplace_back(homographies.emplace_back(findHomography(src_cp, dst_cp)).inv());
        }
    }

    cell_world::Coordinates Composite::get_coordinates(const cell_world::Location & point) {
        auto cell_id = map.cells.find(point);
        return map.cells[cell_id].coordinates;
    }

    void Composite::set_cameras_center(const habitat_cv::Images &images) {
        for (unsigned int c = 0; c < images.size(); c++) {
            cv::Point2f zero_point(configuration.width / 2 - configuration.offsets[c].x, configuration.height / 2 - configuration.offsets[c].y);
            auto camera_zero_point = get_warped_point(c, zero_point);
            auto camera_zero_location = warped[c].get_location(camera_zero_point);
            cameras_center.push_back(camera_zero_location);
        }
    }

    void Composite::start_composite(const Images &images) {
        composite_video = Image();
        subtracted_threshold = Binary_image();
        detection_threshold = Binary_image();
        detection_camera_threshold = vector<Binary_image>(4);
        raw = images;

        if (cameras_center.empty()) set_cameras_center(images);
#ifdef USE_CUDA
        for (unsigned int c = 0; c < configuration.order.count(); c++){
            cv::Mat control;
            cv::absdiff(raw[c](freezing_control_rect), freezing_control[c], control);
            frozen_camera[c] = cv::sum(control)[0] == 0;
            raw[c](freezing_control_rect).copyTo(freezing_control[c]);
            gpu_raw[c].upload(raw[c], gpu_detection_streams[c]);
            cv::cuda::warpPerspective(gpu_raw[c], gpu_warped[c], homographies[c], composite_size, cv::INTER_LINEAR,
                                      cv::BORDER_CONSTANT, cv::Scalar(), gpu_detection_streams[c]);
            cv::cuda::bitwise_and(gpu_warped[c], gpu_mask_detection, gpu_detection[c], cv::noArray(),gpu_detection_streams[c]);
            cv::cuda::resize(gpu_detection[c], gpu_detection_small[c], detection_small_size, 0, 0, cv::INTER_LINEAR,gpu_detection_streams[c]);
            gpu_detection[c](crop_rectangles[c]).copyTo(gpu_composite_detection(crop_rectangles[c]), gpu_detection_streams[c]);
        }
        thread( [this]() {
            for (unsigned int c = 0; c < configuration.order.count(); c++) {
                gpu_detection_streams[c].waitForCompletion();
            }
            cv::cuda::resize(gpu_composite_detection, gpu_composite_detection_small, detection_small_size, 0, 0,
                             cv::INTER_LINEAR, gpu_detection_stream);
            gpu_composite_detection_small.download(composite_detection_small, gpu_detection_stream);

            cv::cuda::absdiff(gpu_composite_detection, gpu_background, gpu_composite_subtracted, gpu_subtracted_stream);
            cv::cuda::resize(gpu_composite_subtracted, gpu_composite_subtracted_small, detection_small_size, 0, 0,
                             cv::INTER_LINEAR, gpu_subtracted_stream);
            gpu_composite_subtracted_small.download(composite_subtracted_small, gpu_subtracted_stream);

            for (unsigned int c = 0; c < configuration.order.count(); c++) {
                gpu_warped[c](crop_rectangles[c]).copyTo(gpu_composite(crop_rectangles[c]), gpu_video_stream);
            }
            cv::cuda::bitwise_and(gpu_composite, gpu_mask_video, gpu_composite_video, cv::noArray(), gpu_video_stream);
            gpu_composite_video.download(composite, gpu_video_stream);

            for (unsigned int c = 0; c < configuration.order.count(); c++) {
                cv::cuda::resize(gpu_raw[c], gpu_composite_raw(crop_rectangles[c]), crop_rectangles[c].size(), 0, 0,
                                 cv::INTER_LINEAR, gpu_raw_stream);
            }
            gpu_composite_raw.download(composite_raw, gpu_raw_stream);
        }).detach();
#else
        for (unsigned int c = 0; c < configuration.order.count(); c++){
            cv::warpPerspective(raw[c], warped[c], homographies[c], composite_size);
            warped[c](crop_rectangles[c]).copyTo(composite(crop_rectangles[c]));
            cv::resize(raw[c], composite_raw(crop_rectangles[c]), crop_rectangles[c].size(), 0, 0,
                             cv::INTER_LINEAR);
        }
        composite = composite.mask(mask_video);
        composite_detection =  composite.mask(mask_detection);
        cv::resize(composite_detection,composite_detection_small,detection_small_size);
        cv::absdiff(composite_detection, background, composite_subtracted);
        cv::resize(composite_subtracted,composite_subtracted_small,detection_small_size);
#endif
    }

    cv::Point2f Composite::get_raw_point(unsigned int camera_index, const cv::Point2f &composite_point){
        cv::Matx33f warp = inverted_homographies[camera_index];
        cv::Point3f raw_point = warp * composite_point;
        return {raw_point.x / raw_point.z, raw_point.y / raw_point.z};
    }

    cell_world::Polygon &Composite::get_polygon(const Coordinates &coordinates) {
        return cells[map.find(coordinates)];
    }

    cv::Point2f Composite::get_warped_point(unsigned int camera_index, const cv::Point2f &camera_point) {
        cv::Matx33f warp = homographies[camera_index];
        cv::Point2f homogeneous_point(camera_point.x,camera_point.y);
        cv::Point3f warped = warp * homogeneous_point;
        return {warped.x, warped.y};
    }

    Image &Composite::get_zoom() {
#ifdef USE_CUDA
        gpu_zoom_stream.waitForCompletion();
#else
        if (zoom_thread.joinable()) zoom_thread.join();
#endif
        return zoom;
    }

    void Composite::set_zoom_size(const cv::Size &s) {
        zoom_size = s;
    }

    cv::Rect_<int> Composite::get_zoom_rect(cv::Size rs, cv::Size zs, cv::Point2i point, cv::Point2i &os) {

        cv::Point2i offset(zs.width / 2,zs.height / 2);

        auto tl = point - offset;
        auto br = point + offset;
        cv::Size rect_size = zs;

        //cut on the left
        if (tl.x<0) {
            rect_size.width += tl.x;
            os.x = -tl.x;
        }
        //cut on the right
        if (br.x >= rs.width) {
            rect_size.width -= br.x - rs.width;
        }

        //cut on the top
        if (tl.y<0) {
            rect_size.height += tl.y;
            os.y = -tl.y;
        }

        //cut on the bottom
        if (br.y >= rs.height) {
            rect_size.height -= br.y - rs.height;
        }

        return cv::Rect_<int>(tl + os, rect_size);
    }

    Image &Composite::get_video() {
#ifdef USE_CUDA
        gpu_video_stream.waitForCompletion();
#endif
        if (composite_video.empty()) composite_video = composite.to_rgb();
        return composite_video;
    }

    void Composite::start_zoom(const Location &location) {
#ifdef USE_CUDA
        auto mouse_point = composite.get_point(location);
        gpu_zoom.setTo(0, gpu_zoom_stream);
        for (unsigned int camera_index=0;camera_index<raw.size();camera_index++) {
            auto raw_point = get_raw_point(camera_index, mouse_point);
            cv::Point2i offset(0, 0);
            cv::Rect_<int> source = get_zoom_rect(raw[camera_index].size(), zoom_size, raw_point, offset);
            offset.x += zoom_rectangles[camera_index].x;
            offset.y += zoom_rectangles[camera_index].y;
            cv::Rect_<int> destination(offset, source.size());
            if (source.size().height<0  || source.width<0 ) continue;
            gpu_raw[camera_index](source).copyTo(gpu_zoom(destination), gpu_zoom_stream);
        }
        gpu_zoom.download(zoom, gpu_zoom_stream);
#else
        zoom_thread = thread ([this](Location location) {
            auto mouse_point = composite.get_point(location);
            zoom.setTo(0);
            for (unsigned int camera_index = 0; camera_index < raw.size(); camera_index++) {
                auto raw_point = get_raw_point(camera_index, mouse_point);
                cv::Point2i offset(0, 0);
                cv::Rect_<int> source = get_zoom_rect(composite_size, zoom_size, raw_point, offset);
                offset.x = (float)zoom_rectangles[camera_index].x - offset.x;
                offset.y = (float)zoom_rectangles[camera_index].y - offset.y;
                cv::Rect_<int> destination(offset, source.size());
                if (destination.height > 0 && destination.width > 0 && destination.x >= 0 && destination.y >= 0)
                    raw[camera_index](source).copyTo(zoom(destination));
            }
        }, location);
        zoom_thread.join();
#endif
    }

    void Composite::set_background(const Image &bg) {
        background = bg.to_gray();
#ifdef USE_CUDA
        gpu_background.upload(background);
#endif
    }

    Image &Composite::get_detection_small() {
#ifdef USE_CUDA
    gpu_detection_stream.waitForCompletion();
#endif
        return composite_detection_small;
    }

    Image &Composite::get_raw(unsigned int camera_index) {
        return raw[camera_index];
    }

    Image &Composite::get_composite() {
#ifdef USE_CUDA
        gpu_composite.download(composite);
#endif
        return composite;
    }

    Image &Composite::get_detection() {
#ifdef USE_CUDA
        gpu_composite_detection.download(composite_detection);
#endif
        return composite_detection;
    }

    Image &Composite::get_subtracted() {
#ifdef USE_CUDA
        gpu_composite_subtracted.download(composite_subtracted);
#endif
        return composite_subtracted;
    }

    Image &Composite::get_subtracted_small() {
#ifdef USE_CUDA
        gpu_subtracted_stream.waitForCompletion();
#endif
        return composite_subtracted_small;
    }

    Image &Composite::get_detection_small(unsigned int c) {
#ifdef USE_CUDA
        gpu_detection_small[c].download(detection_small[c]);
#endif
        return detection_small[c];
    }

    bool Composite::is_transitioning(const cell_world::Location &l) {
        auto p = composite.get_point(l);
        for (auto &r: crop_rectangles) {
            if(r.x && abs(p.x-r.x) < (float)transition_size) return true;
            if(r.y && abs(p.y-r.y) < (float)transition_size) return true;
        }
        return false;
    }

    unsigned int Composite::get_best_camera(const cell_world::Location &l) {
        auto p = composite.get_point(l);
        for (unsigned int c = 0;c < raw.size();c++)  {
            if (crop_rectangles[c].contains(p)) return c;
        }
        return -1;
    }

    Image &Composite::get_raw_composite() {
#ifdef USE_CUDA
        gpu_raw_stream.waitForCompletion();
#endif
        return composite_raw;
    }

    cell_world::Location Composite::get_perspective_correction(const Location &location, float height, int camera) {
        if (camera == -1) camera = get_best_camera(location);
        return ((location - cameras_center[camera]) * (height / (camera_height - height)));
    }

        Binary_image &Composite::get_subtracted_threshold(unsigned char t) {
        if (subtracted_threshold.empty()) {
            subtracted_threshold = Binary_image(get_subtracted_small()>t);
        }
        return subtracted_threshold;
    }

    Binary_image &Composite::get_detection_threshold(unsigned char t, int c) {
        if (c==-1){
            if (detection_threshold.empty()) {
                detection_threshold = Binary_image(get_detection_small()>t);
            }
            return detection_threshold;

        }
        if (detection_camera_threshold[c].empty()) {
            detection_camera_threshold[c] = Binary_image(get_detection_small(c) > t);
        }
        return detection_camera_threshold[c];
        }
}
