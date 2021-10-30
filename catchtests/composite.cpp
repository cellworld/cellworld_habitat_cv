#include <catch.h>
#include <habitat_cv/composite.h>
#include <habitat_cv/util.h>
#include <iostream>

using namespace std;
using namespace habitat_cv;
using namespace cell_world;

TEST_CASE("Composite"){
    auto camera_configuration = Resources::from("camera_configuration").key("default").get_resource<Camera_configuration>();
    Composite composite( camera_configuration);
    auto images = read_images("../../images/",{"camera_0.png","camera_1.png","camera_2.png","camera_3.png"});
    cv::Mat comp = composite.get_composite(images, true);
    srand (time(NULL));
    auto cells = composite.world.create_cell_group();
    for (auto i=0;i<30;i++) {
        auto c = cells.random_cell();
        auto coord = composite.get_coordinates(c.location);
        composite.draw_circle(c.location, 3);
        composite.draw_cell(coord);
    }
    cv::imwrite("composite.png",comp);
    cv::imwrite("rgb.png",composite.get_rgb());
//    cv::imshow("test", rgb);
//    cv::waitKey();
}


TEST_CASE("point-coordinates association") {
    auto camera_configuration = Resources::from("camera_configuration").key("default").get_resource<Camera_configuration>();
    Composite composite( camera_configuration);
    auto images = read_images("../../images/",{"camera_0.png","camera_1.png","camera_2.png","camera_3.png"});
    cv::Mat comp = composite.get_composite(images);
    srand (time(NULL));
    for (int i=0;i<30; i++) {
        auto point = Location{(double)(rand() % 1080), (double)(rand() % 1080)};
        auto coord = composite.get_coordinates(point);
        composite.draw_circle(point, 3);
        composite.draw_cell(coord);
    }
    cv::imwrite("centroid_check.png",composite.get_rgb());
//    cv::imshow("test", rgb);
//    cv::waitKey();
}


TEST_CASE("arrows") {
    auto camera_configuration = Resources::from("camera_configuration").key("default").get_resource<Camera_configuration>();
    Composite composite( camera_configuration);
    auto images = read_images("../../images/",{"camera_0.png","camera_1.png","camera_2.png","camera_3.png"});
    cv::Mat comp = composite.get_composite(images);
    srand (time(NULL));
    for (int i=0;i<30; i++) {
        auto point = Location{(double)(rand() % 1080), (double)(rand() % 1080)};
        auto coord = composite.get_coordinates(point);
        composite.draw_circle(point, 3);
        composite.draw_cell(coord);
        double theta = (double)i / 30 * 2 * M_PI;
        composite.draw_arrow(point, theta);
    }
    cv::imwrite("arrow_check.png",composite.get_rgb());
//    cv::imshow("test", rgb);
//    cv::waitKey();
}
