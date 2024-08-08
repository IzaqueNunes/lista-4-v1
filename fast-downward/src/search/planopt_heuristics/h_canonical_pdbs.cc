#include "h_canonical_pdbs.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace planopt_heuristics {
CanonicalPDBsHeuristic::CanonicalPDBsHeuristic(const options::Options &options)
    : Heuristic(options),
      pdbs(create_tnf_task(task_proxy), options.get_list<vector<int>>("patterns")) {
}

int CanonicalPDBsHeuristic::compute_heuristic(const GlobalState &global_state) {
    TNFState state = global_state.unpack().get_values();

    int h = pdbs.compute_heuristic(state);
    if (h == numeric_limits<int>::max()) {
        return DEAD_END;
    } else {
        return h;
    }
}

static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    Heuristic::add_options_to_parser(parser);
    parser.add_list_option<vector<int>>("patterns");
    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<CanonicalPDBsHeuristic>(opts);
}

static Plugin<Evaluator> _plugin("planopt_cpdbs", _parse);

}
