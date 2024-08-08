#ifndef PLANOPT_HEURISTICS_PDB_H
#define PLANOPT_HEURISTICS_PDB_H

#include "projection.h"

#include <vector>

namespace planopt_heuristics {

class PatternDatabase {
    Projection projection;
    std::vector<int> distances;
public:
    PatternDatabase(const TNFTask &task, const Pattern &pattern);

    int lookup_distance(const TNFState &original_state) const;
};
}

#endif
