#ifndef PLANOPT_HEURISTICS_PROJECTION_H
#define PLANOPT_HEURISTICS_PROJECTION_H

#include "tnf_task.h"

#include <vector>

namespace planopt_heuristics {

using Pattern = std::vector<int>;

class Projection {
    Pattern pattern;

    /*
      Multipliers for perfect hashing. In the slides, these are called N_i.
    */
    std::vector<int> perfect_hash_multipliers;

    TNFTask projected_task;

public:
    Projection(const TNFTask &task, const Pattern &pattern);

    TNFState project_state(const TNFState &state) const;
    int rank_state(const TNFState &state) const;
    TNFState unrank_state(int index) const;

    const TNFTask &get_projected_task() { return projected_task; }

};
}

#endif
