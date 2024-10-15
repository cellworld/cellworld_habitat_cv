#include <cell_world.h>
#include <params_cpp.h>
#include <params_cpp.h>

using namespace params_cpp;
using namespace cell_world;
using namespace std;
using namespace json_cpp;


int main (int argc, char **argv){
    Parser p(argc,argv);
    auto occlusions = p.get(Key("-o","--occlusions"),"21_05");
    auto configuration = p.get(Key("-c","--configuration"),"hexagonal");
    auto folder = Resources::cache_folder();
    auto output_file = folder + "/cell_group/" + p.get(Key("-of","--output_file"), "");
    auto threshold = stof(p.get(Key("-d","--distance"),"0.5"));

    World world = World::get_from_parameters_name(configuration,"canonical", occlusions);

    auto cells = world.create_cell_group();
    Cell_group pd = world.create_cell_group(Resources::from("cell_group").key("hexagonal").key(occlusions).key("predator_destinations").get_resource<Cell_group_builder>());
    Graph g = world.create_graph();
    Graph visibility = world.create_graph(Resources::from("graph").key("hexagonal").key(occlusions).key("cell_visibility").get_resource<Graph_builder>());
    Cell_group spawn_locations;
    Map map(cells);
    auto &start_location = map[Coordinates(-20,0)];
    auto &start_location_visibility = visibility[start_location];
    for (const Cell &cell:pd) {
        if (cell.location.dist(start_location.location) > threshold &&
            !start_location_visibility.contains(cell) && g[cell].size() == world.connection_pattern.size()) {
            spawn_locations.add(cell);
        }
    }
    spawn_locations.save(output_file);  // MAKE SURE WORLD ARE IS NAMED PROPERLY
}