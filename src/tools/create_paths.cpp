#include <cell_world.h>
#include <params_cpp.h>

using namespace params_cpp;
using namespace cell_world;
using namespace std;
using namespace json_cpp;

int main (int argc, char **argv){
    Parser p(argc,argv);
    auto occlusions = p.get(Key("-o","--occlusions"),"21_05");
    auto configuration = p.get(Key("-c","--configuration"),"hexagonal");
//    auto output_file = p.get(Key("-of","--output_file"),".cellworld_cache/paths/" + configuration +"." + occlusions + ".astar");
    World world = World::get_from_parameters_name(configuration,"canonical", occlusions);
//    Graph graph = world.create_graph();
//   Paths paths = Paths::get_astar(graph);
//    paths.save(output_file);
    auto paths = Paths(world.create_paths(Resources::from("paths").key("hexagonal").key(occlusions).key("astar").get_resource<Path_builder>()));
    auto cells = world.create_cell_group();
    auto src = cells.find(Coordinates(-8,0));
    auto dst = cells.find(Coordinates(6,-2));

    auto move = paths.get_move(cells[src], cells[dst]);
    cout << cells[src] << endl;
    cout << cells[dst] << endl;
    cout << paths.get_path(cells[src], cells[dst]) <<  endl;
    cout << move << endl;
}

