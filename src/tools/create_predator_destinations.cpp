#include <cell_world.h>
#include <params_cpp.h>
#include <iostream>



using namespace params_cpp;
using namespace cell_world;
using namespace std;
using namespace json_cpp;


int main (int argc, char **argv){
    Parser p(argc,argv);
    auto occlusions = p.get(Key("-o","--occlusions"),"");
    auto folder = Resources::cache_folder();
    auto configuration = p.get(Key("-c","--configuration"),"hexagonal");
    auto output_file = folder + "/cell_group/" + p.get(Key("-of","--output_file"), "");

    World world = World::get_from_parameters_name(configuration,"canonical", occlusions);
    Graph graph = world.create_graph();
    Cell_group cells = world.create_cell_group().free_cells();
    Graph g = world.create_graph();
    Cell_group pd;
    for (const Cell &cell:cells) {
        if (graph.is_connected(cell,world.cells[0]) && g[cell].size() == world.connection_pattern.size()){
            pd.add(cell);
        }
    }
    pd.save(output_file);
}
