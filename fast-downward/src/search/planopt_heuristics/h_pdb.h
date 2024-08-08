#ifndef PLANOPT_HEURISTICS_H_PDB_H
#define PLANOPT_HEURISTICS_H_PDB_H

#include "pdb.h"

#include "../heuristic.h"

namespace planopt_heuristics {
class PDBHeuristic : public Heuristic {
    PatternDatabase pdb;
protected:
    virtual int compute_heuristic(const GlobalState &state) override;
public:
    explicit PDBHeuristic(const options::Options &options);
};
}
#endif
