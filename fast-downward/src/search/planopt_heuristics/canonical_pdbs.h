#ifndef PLANOPT_HEURISTICS_CANONICAL_PDBS_H
#define PLANOPT_HEURISTICS_CANONICAL_PDBS_H

#include "pdb.h"

#include <vector>

namespace planopt_heuristics {

class CanonicalPatternDatabases {
    std::vector<PatternDatabase> pdbs;
    std::vector<std::vector<int>> maximal_additive_sets;
public:
    CanonicalPatternDatabases(const TNFTask &task, const std::vector<Pattern> &patterns);

    int compute_heuristic(const TNFState &original_state);
};
}

#endif
