#ifndef PLANOPT_HEURISTICS_H_CANONICAL_PDBS_H
#define PLANOPT_HEURISTICS_H_CANONICAL_PDBS_H

#include "canonical_pdbs.h"

#include "../heuristic.h"

namespace planopt_heuristics {
class CanonicalPDBsHeuristic : public Heuristic {
    CanonicalPatternDatabases pdbs;
protected:
    virtual int compute_heuristic(const GlobalState &state) override;
public:
    explicit CanonicalPDBsHeuristic(const options::Options &options);
};
}
#endif
