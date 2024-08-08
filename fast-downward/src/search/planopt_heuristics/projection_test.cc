#include "projection_test.h"

#include "projection.h"
#include "tnf_task.h"

#include "../utils/logging.h"

#include <unordered_map>
#include <vector>

using namespace std;

namespace planopt_heuristics {
void verify_tasks_match(TNFTask task, TNFTask expected) {
    if (task.variable_domains != expected.variable_domains) {
        cerr << "Expected " << expected.variable_domains.size()
             << " variables with domain sizes " << expected.variable_domains
             << endl
             << "but got " << task.variable_domains.size()
             << " variables with domain sizes " << task.variable_domains
             << endl;
    } else {
        cout << "Variable domains are as expected." << endl;
    }
    if (task.initial_state != expected.initial_state) {
        cerr << "Expected initial state " << expected.initial_state
             << endl
             << "but got " << task.initial_state
             << endl;
    } else {
        cout << "Initial state is as expected." << endl;
    }
    if (task.goal_state != expected.goal_state) {
        cerr << "Expected goal state " << expected.goal_state
             << endl
             << "but got " << task.goal_state
             << endl;
    } else {
        cout << "Goal state is as expected." << endl;
    }

    unordered_map<string, TNFOperator> expected_operators;
    for (const TNFOperator &op : expected.operators) {
        expected_operators[op.name] = op;
    }
    unordered_map<string, TNFOperator> task_operators;
    for (const TNFOperator &op : task.operators) {
        if (task_operators.find(op.name) != task_operators.end()) {
            cerr << "duplicate definition of operator " << op.name << endl;
        }
        task_operators[op.name] = op;
    }
    for (const auto &map_entry : expected_operators) {
        string name = map_entry.first;
        if (task_operators.find(name) == task_operators.end()) {
            cerr << "Did not find a definition of operator " << name << endl;
        } else {
            const TNFOperator &expected_op = map_entry.second;
            const TNFOperator &task_op = task_operators[name];
            if (task_op.cost != expected_op.cost) {
                cerr << "Operator " << name << " should have cost "
                     << expected_op.cost << " but has cost " << task_op.cost << endl;
            }
            unordered_map<int, TNFOperatorEntry> expected_entries;
            for (TNFOperatorEntry entry : expected_op.entries) {
                expected_entries[entry.variable_id] = entry;
            }
            unordered_map<int, TNFOperatorEntry> task_op_entries;
            for (TNFOperatorEntry entry : task_op.entries) {
                if (task_op_entries.find(entry.variable_id) != task_op_entries.end()) {
                    cerr << "Operator " << task_op.name << " has two entries for "
                         << "variable " << entry.variable_id << endl;
                }
                task_op_entries[entry.variable_id] = entry;
            }

            for (const auto &item : task_op_entries) {
                int var_id = item.first;
                TNFOperatorEntry task_op_entry = item.second;
                if (expected_entries.find(var_id) == expected_entries.end()) {
                    cerr << "Operator " << name << " changes variable "
                         << var_id << " from " << task_op_entry.precondition_value
                         << " to " << task_op_entry.effect_value
                         << " but should not change the variable at all." << endl;
                }
            }
            for (const auto &item : expected_entries) {
                int var_id = item.first;
                TNFOperatorEntry expected_entry = item.second;
                if (task_op_entries.find(var_id) == task_op_entries.end()) {
                    cerr << "Operator " << name << " should change variable "
                         << var_id << " from " << expected_entry.precondition_value
                         << " to " << expected_entry.effect_value
                         << " but doesn't change it at all." << endl;
                } else {
                    TNFOperatorEntry task_op_entry = task_op_entries[item.first];
                    if (expected_entry.precondition_value != task_op_entry.precondition_value ||
                        expected_entry.effect_value != task_op_entry.effect_value) {
                        cerr << "Operator " << name << " should change variable "
                             << var_id << " from " << expected_entry.precondition_value
                             << " to " << expected_entry.effect_value
                             << " but changes it from "
                             << task_op_entry.precondition_value
                             << " to " << task_op_entry.effect_value << "." << endl;
                    }
                }
            }
        }
    }
    for (const auto &map_entry : task_operators) {
        string name = map_entry.first;
        if (expected_operators.find(name) == expected_operators.end()) {
            cerr << "Operator " << name << " should not exist in the projection"<< endl;
        }
    }
}

void test_projections() {
    TNFTask task;
    int var_package = 0;
    int var_truck_a = 1;
    int var_truck_b = 2;
    int val_left = 0;
    int val_right = 1;
    int val_in_truck_a = 2;
    int val_in_truck_b = 3;
    int val_truck_unknown = 2;
    task.variable_domains = {4, 3, 3};
    task.initial_state = {val_left, val_right, val_right};
    task.goal_state = {val_right, val_truck_unknown, val_truck_unknown};
    task.operators = {
        TNFOperator({{var_truck_a, val_left, val_right}}, 1, "drive_truck_a_left_right"),
        TNFOperator({{var_truck_a, val_right, val_left}}, 1, "drive_truck_a_right_left"),
        TNFOperator({{var_truck_b, val_left, val_right}}, 1, "drive_truck_b_left_right"),
        TNFOperator({{var_truck_b, val_right, val_left}}, 1, "drive_truck_b_right_left"),
        TNFOperator({{var_truck_a, val_left, val_left},
                     {var_package, val_left, val_in_truck_a}}, 1, "load_truck_a_left"),
        TNFOperator({{var_truck_a, val_right, val_right},
                     {var_package, val_right, val_in_truck_a}}, 1, "load_truck_a_right"),
        TNFOperator({{var_truck_b, val_left, val_left},
                     {var_package, val_left, val_in_truck_b}}, 1, "load_truck_b_left"),
        TNFOperator({{var_truck_b, val_right, val_right},
                     {var_package, val_right, val_in_truck_b}}, 1, "load_truck_b_right"),
        TNFOperator({{var_truck_a, val_left, val_left},
                     {var_package, val_in_truck_a, val_left}}, 1, "unload_truck_a_left"),
        TNFOperator({{var_truck_a, val_right, val_right},
                     {var_package, val_in_truck_a, val_right}}, 1, "unload_truck_a_right"),
        TNFOperator({{var_truck_b, val_left, val_left},
                     {var_package, val_in_truck_b, val_left}}, 1, "unload_truck_b_left"),
        TNFOperator({{var_truck_b, val_right, val_right},
                     {var_package, val_in_truck_b, val_right}}, 1, "unload_truck_b_right"),
        TNFOperator({{var_truck_a, val_left, val_truck_unknown}}, 0, "forget_truck_a_left"),
        TNFOperator({{var_truck_a, val_right, val_truck_unknown}}, 0, "forget_truck_a_right"),
        TNFOperator({{var_truck_b, val_left, val_truck_unknown}}, 0, "forget_truck_b_left"),
        TNFOperator({{var_truck_b, val_right, val_truck_unknown}}, 0, "forget_truck_b_right"),
    };

    Projection p1(task, {var_package});
    TNFTask expected1;
    int var_p1_package = 0;
    expected1.variable_domains = {4};
    expected1.initial_state = {val_left};
    expected1.goal_state = {val_right};
    expected1.operators = {
        TNFOperator({{var_p1_package, val_left, val_in_truck_a}}, 1, "load_truck_a_left"),
        TNFOperator({{var_p1_package, val_right, val_in_truck_a}}, 1, "load_truck_a_right"),
        TNFOperator({{var_p1_package, val_left, val_in_truck_b}}, 1, "load_truck_b_left"),
        TNFOperator({{var_p1_package, val_right, val_in_truck_b}}, 1, "load_truck_b_right"),
        TNFOperator({{var_p1_package, val_in_truck_a, val_left}}, 1, "unload_truck_a_left"),
        TNFOperator({{var_p1_package, val_in_truck_a, val_right}}, 1, "unload_truck_a_right"),
        TNFOperator({{var_p1_package, val_in_truck_b, val_left}}, 1, "unload_truck_b_left"),
        TNFOperator({{var_p1_package, val_in_truck_b, val_right}}, 1, "unload_truck_b_right"),
    };
    cout << "Verifying projection to package:" << endl;
    verify_tasks_match(p1.get_projected_task(), expected1);
    cout << endl;

    Projection p2(task, {var_truck_a, var_package});
    TNFTask expected2;
    int var_p2_truck_a = 0;
    int var_p2_package = 1;
    expected2.variable_domains = {3, 4};
    expected2.initial_state = {val_right, val_left};
    expected2.goal_state = {val_truck_unknown, val_right};
    expected2.operators = {
        TNFOperator({{var_p2_truck_a, val_left, val_right}}, 1, "drive_truck_a_left_right"),
        TNFOperator({{var_p2_truck_a, val_right, val_left}}, 1, "drive_truck_a_right_left"),
        TNFOperator({{var_p2_truck_a, val_left, val_left},
                     {var_p2_package, val_left, val_in_truck_a}}, 1, "load_truck_a_left"),
        TNFOperator({{var_p2_truck_a, val_right, val_right},
                     {var_p2_package, val_right, val_in_truck_a}}, 1, "load_truck_a_right"),
        TNFOperator({{var_p2_package, val_left, val_in_truck_b}}, 1, "load_truck_b_left"),
        TNFOperator({{var_p2_package, val_right, val_in_truck_b}}, 1, "load_truck_b_right"),
        TNFOperator({{var_p2_truck_a, val_left, val_left},
                     {var_p2_package, val_in_truck_a, val_left}}, 1, "unload_truck_a_left"),
        TNFOperator({{var_p2_truck_a, val_right, val_right},
                     {var_p2_package, val_in_truck_a, val_right}}, 1, "unload_truck_a_right"),
        TNFOperator({{var_p2_package, val_in_truck_b, val_left}}, 1, "unload_truck_b_left"),
        TNFOperator({{var_p2_package, val_in_truck_b, val_right}}, 1, "unload_truck_b_right"),
        TNFOperator({{var_p2_truck_a, val_left, val_truck_unknown}}, 0, "forget_truck_a_left"),
        TNFOperator({{var_p2_truck_a, val_right, val_truck_unknown}}, 0, "forget_truck_a_right"),
    };
    cout << "verifying projection to truck A and package:" << endl;
    verify_tasks_match(p2.get_projected_task(), expected2);
}
}
