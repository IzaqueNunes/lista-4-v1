#ifndef PLANOPT_HEURISTICS_TNF_TASK_H
#define PLANOPT_HEURISTICS_TNF_TASK_H

#include "../task_proxy.h"

#include <string>
#include <vector>

namespace planopt_heuristics {
/*
  States in the TNF task are represented as a vector of values: The vector has
  one entry per variable, and position i has the value of variable i.
*/
using TNFState = std::vector<int>;

struct TNFOperatorEntry {
    int variable_id;
    int precondition_value;
    int effect_value;

    TNFOperatorEntry() = default;
    TNFOperatorEntry(int variable_id, int precondition_value, int effect_value)
        : variable_id(variable_id),
          precondition_value(precondition_value),
          effect_value(effect_value) {
    }
};

/*
  An Operator in TNF mentions the same variables in its precondition and its
  effect. Instead of storing a precondition and an effect, we can thus store it
  as a list of triples (v, p, e)  meaning that variable v is changed from
  value p to value e. Formally, the tuples represents the precondition (v = p)
  and the effect (v := e).
*/
struct TNFOperator {
    std::vector<TNFOperatorEntry> entries;
    int cost;
    std::string name;

    TNFOperator() = default;
    TNFOperator(std::vector<TNFOperatorEntry> entries, int cost, std::string name)
        : entries(entries), cost(cost), name(name) {
    }
};

struct TNFTask {
    /*
      The task has variables 0, ..., n and variable i has
      k_i = variable_domains[i] values: 0, ..., (k_i - 1).
    */
    std::vector<int> variable_domains;

    TNFState initial_state;

    // In TNF there is only one goal state.
    TNFState goal_state;

    // All operators are in TNF (see documentation above).
    std::vector<TNFOperator> operators;

    int get_num_states() const {
        int result = 1;
        for (int d : variable_domains) {
            result *= d;
        }
        return result;
    }
};

extern TNFTask create_tnf_task(const TaskProxy &sas_task);

}

#endif
