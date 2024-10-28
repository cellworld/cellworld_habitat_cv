#include <cell_world.h>
#include <params_cpp.h>

using namespace params_cpp;
using namespace cell_world;
using namespace std;
using namespace json_cpp;

int main (int argc, char **argv){
    Parser p(argc,argv);
    auto occlusions = p.get(Key("-o","--occlusions"),"");
    auto configuration = p.get(Key("-c","--configuration"),"hexagonal");
    auto folder = Resources::cache_folder();
    auto output_file = p.get(Key("-of","--output_file"),folder + "/graph/" + configuration + "." + occlusions + ".cell_visibility");
    World world = World::get_from_parameters_name(configuration,"canonical", occlusions);
    Cell_group cells = world.create_cell_group();
    auto cell_shape = world.get_configuration().cell_shape;
    auto cell_transformation = world.get_implementation().cell_transformation;
    auto visibility = Coordinates_visibility::create_graph(cells,cell_shape,cell_transformation);
    visibility.save(output_file);
    cout << "visibility saved to " << output_file << endl;
}

