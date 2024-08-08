#include "h_pdb.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace planopt_heuristics {
PDBHeuristic::PDBHeuristic(const options::Options &options)
    : Heuristic(options),
      pdb(create_tnf_task(task_proxy), options.get_list<int>("pattern")) {
}

int PDBHeuristic::compute_heuristic(const GlobalState &global_state) {
    TNFState state = global_state.unpack().get_values();

    int h = pdb.lookup_distance(state);
    if (h == numeric_limits<int>::max()) {
        return DEAD_END;
    } else {
        return h;
    }
}

static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    Heuristic::add_options_to_parser(parser);
    parser.add_list_option<int>("pattern");
    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<PDBHeuristic>(opts);
}

static Plugin<Evaluator> _plugin("planopt_pdb", _parse);

}
