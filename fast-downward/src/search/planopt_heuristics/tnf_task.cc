#include "tnf_task.h"

#include "../task_utils/task_properties.h"


using namespace std;

namespace planopt_heuristics {
static int get_unknown_value(const VariableProxy &variable) {
    return variable.get_domain_size();
}

static TNFOperator create_tnf_operator(
    const OperatorProxy &op, const VariablesProxy &variables,
    vector<bool> &unknown_fact_needed) {
    int num_vars = variables.size();

    vector<int> precondition_values(num_vars, -1);
    for (FactProxy precondition : op.get_preconditions()) {
        const FactPair fact = precondition.get_pair();
        precondition_values[fact.var] = fact.value;
    }

    vector<int> effect_values(num_vars, -1);
    for (EffectProxy effect : op.get_effects()) {
        const FactPair fact = effect.get_fact().get_pair();
        effect_values[fact.var] = fact.value;
    }

    vector<TNFOperatorEntry> entries;
    for (int var_id = 0; var_id < num_vars; ++var_id) {
        int pre_value = precondition_values[var_id];
        int post_value = effect_values[var_id];
        if (pre_value == -1 && post_value == -1) {
            continue;
        } else if (pre_value == -1) {
            unknown_fact_needed[var_id] = true;
            pre_value = get_unknown_value(variables[var_id]);
        } else if (post_value == -1) {
            post_value = pre_value;
        }
        entries.emplace_back(var_id, pre_value, post_value);
    }
    return TNFOperator(entries, op.get_cost(), op.get_name());
}

TNFTask create_tnf_task(const TaskProxy &sas_task) {
    task_properties::verify_no_axioms(sas_task);
    task_properties::verify_no_conditional_effects(sas_task);
    OperatorsProxy sas_operators = sas_task.get_operators();
    VariablesProxy sas_variables = sas_task.get_variables();
    int num_sas_variables = sas_variables.size();

    TNFTask tnf_task;

    /*
      We add an "unknown" fact for variables occuring in effects, but
      not in preconditions, and variables missing from the goal
      description.
    */
    vector<bool> unknown_fact_needed(num_sas_variables, false);

    /*
      Compute TNF versions of normal operators and record which
      variables need "unknown" fact.
    */
    tnf_task.operators.reserve(sas_operators.size());
    for (OperatorProxy op : sas_operators) {
        TNFOperator tnf_op = create_tnf_operator(op, sas_variables, unknown_fact_needed);
        tnf_task.operators.push_back(tnf_op);
    }

    /*
      Transform initial state.
    */
    tnf_task.initial_state.reserve(num_sas_variables);
    for (FactProxy fact : sas_task.get_initial_state()) {
        tnf_task.initial_state.push_back(fact.get_value());
    }

    /*
      Transform goal state.
    */
    tnf_task.goal_state.reserve(num_sas_variables);
    for (VariableProxy var : sas_variables) {
        tnf_task.goal_state.emplace_back(get_unknown_value(var));
    }
    for (FactProxy goal : sas_task.get_goals()) {
        const FactPair fact = goal.get_pair();
        tnf_task.goal_state[fact.var] = fact.value;
    }
    for (size_t var_id = 0; var_id < tnf_task.goal_state.size(); ++var_id) {
        int goal_value = tnf_task.goal_state[var_id];
        if (goal_value == get_unknown_value(sas_variables[var_id])) {
            // Variables missing in goal description need an "unknown" fact.
            unknown_fact_needed[var_id] = true;
        }
    }

    /*
      Create "forget" operators for the variables with "unknown" facts.
    */
    for (VariableProxy var : sas_variables) {
        if (unknown_fact_needed[var.get_id()]) {
            int var_id = var.get_id();
            int post_value = get_unknown_value(var);
            for (int value = 0; value < var.get_domain_size(); ++value) {
                TNFOperatorEntry entry(var_id, value, post_value);
                string name = "forget_" + var.get_name() + "_" + var.get_fact(value).get_name();
                TNFOperator forget({entry}, 0, name);
                tnf_task.operators.push_back(forget);
            }
        }
    }

    /*
      Create variables.
    */
    tnf_task.variable_domains.reserve(num_sas_variables);
    for (VariableProxy var : sas_variables) {
        tnf_task.variable_domains.push_back(var.get_domain_size());
        if (unknown_fact_needed[var.get_id()]) {
            ++tnf_task.variable_domains.back();
        }
    }

    return tnf_task;
}
}
