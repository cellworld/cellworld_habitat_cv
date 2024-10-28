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
    auto output_file = p.get(Key("-of","--output_file"),folder + "/paths/" + configuration + "." + occlusions + ".astar");
    World world = World::get_from_parameters_name(configuration,"canonical", occlusions);
    Graph graph = world.create_graph();
    cout << "building paths... " << endl;
    Paths paths = Paths::get_astar(graph);
    paths.save(output_file);
    cout << "paths saved to " << output_file << endl;
}

