#include <cell_world/map.h>
#include <cell_world.h>
#include <params_cpp.h>

using namespace params_cpp;
using namespace std;
using namespace cell_world;

bool must_occlude(Coordinates coord, Coordinates_list &cnn_pat, Map &map1){
    Coordinates_list blocked_sides;
    for (auto cn: cnn_pat){
        auto c = coord + cn;
        // check if new coordinate is in world
        if (map1.find(c) == Not_found){
            blocked_sides.push_back(cn);
            continue;
        }

        // check if new coordinate is occluded
        if (map1[c].occluded) {
            blocked_sides.push_back(cn);
        }
    }
    // 0 or 1 side blocked is ok
    if (blocked_sides.size()<2) return false;
    // 3,4,5,6 sides blocked must be occluded
    if (blocked_sides.size()>2) return true;

    // if 2 sides are blocked, we need to check if they are adjacent


    // if one of the left/right sides is blocked, and the other is adjacent
    // the sum on x must be 3 or -3
    auto sum_sides_x=blocked_sides[0].x + blocked_sides[1].x;
    if (sum_sides_x == 3 || sum_sides_x == -3) return false; // they are adjacent

    // if none of the two sides are on the left/right side (y!=0)
    // y must be the same on both
    if (blocked_sides[0].y && blocked_sides[0].y == blocked_sides[1].y) return false; // they are adjacent

    //if the two sides are not adjacent must be occluded
    return true;
}
//OMG!

int main (int argc, char **argv){
    Parser p(argc,argv);
    auto occlusions = p.get(Key("-o","--occlusions"),"");
    auto configuration = p.get(Key("-c","--configuration"),"hexagonal");
    auto folder = Resources::cache_folder();
    auto output_file = p.get(Key("-of","--output_file"),folder + "/cell_group/" + configuration + "." + occlusions + ".occlusions.robot");

    // load world and create new_world to modify
    auto world1 = World::get_from_parameters_name(configuration, "canonical", occlusions);
    auto world2 = world1;

    // modify world by occluding cells
    Cell_group cg1 = world1.create_cell_group();
    Cell_group cg2 = world2.create_cell_group();
    Map map1 (cg1);
    Map map2 (cg2);
    auto free_cells = map2.cells.free_cells();
    // occlude cells that are not navigable
    for(const Cell &cell : free_cells){
        bool occluded_bool = must_occlude(cell.coordinates, world1.connection_pattern, map1);
        if (occluded_bool){
            int index = map2.find(cell.coordinates);
            world2.cells[index].occluded = true;
        }
    }
    cg2.occluded_cells().save(output_file);
    cout << "robot occlusions saved to " << output_file << endl;
    return 0;
}

