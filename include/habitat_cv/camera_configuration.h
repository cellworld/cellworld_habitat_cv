#pragma once
#include <cell_world.h>
#include <json_cpp.h>

namespace habitat_cv {
    struct Centroid : json_cpp::Json_object{
        Json_object_members(
                Add_member(centroid);
                Add_member(cell_coordinates);
                Ignore_member("trust");
        );
        cell_world::Location centroid;
        cell_world::Coordinates cell_coordinates;
    };

    using Camera_centroids = json_cpp::Json_vector<Centroid>;

    struct Camera_order: json_cpp::Json_vector <json_cpp::Json_vector<unsigned int>> {
        unsigned int rows() const;
        unsigned int cols() const;
        unsigned int count() const ;
        cell_world::Coordinates get_camera_coordinates(unsigned int) const;
    };

    struct Camera_configuration : json_cpp::Json_object  {
        Camera_order order;
        json_cpp::Json_vector<Camera_centroids> centroids;
        Json_object_members(
                Add_member(order);
                Add_member(centroids);
        );
    };
}