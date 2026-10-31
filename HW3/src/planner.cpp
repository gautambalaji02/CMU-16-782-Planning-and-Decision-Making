#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <cstring>
#include <chrono>
#include <limits>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6
#ifndef ENVS_DIR
#define ENVS_DIR "../envs"
#endif

constexpr int INF = std::numeric_limits<int>::max() / 2;
// 1 = Dijkstra (h = 0), 2 = A* goal count, 3 = A* empty-delete-list
int planner_mode = 2;

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(const string &predicate, const list<string> &arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth; // fixed
        for (const string &l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition &gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth; // fixed
        for (const string &l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream &operator<<(ostream &os, const GroundedCondition &pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition &rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        temp += this->predicate;
        temp += "(";
        for (const string &l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition &lhs, const GroundedCondition &rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition &gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(const string &pred, const list<string> &args, const bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (const string &ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream &operator<<(ostream &os, const Condition &cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition &rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (const string &l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition &lhs, const Condition &rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition &cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(const string &name, const list<string> &args,
           const unordered_set<Condition, ConditionHasher, ConditionComparator> &preconditions,
           const unordered_set<Condition, ConditionHasher, ConditionComparator> &effects)
    {
        this->name = name;
        for (const string &l : args)
        {
            this->args.push_back(l);
        }
        for (const Condition &pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (const Condition &pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action &rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream &operator<<(ostream &os, const Action &ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (const Condition &precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (const Condition &effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->get_name();
        temp += "(";
        for (const string &l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action &lhs, const Action &rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action &ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(const GroundedCondition &gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(const GroundedCondition &gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(const GroundedCondition &gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(const GroundedCondition &gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(const string &symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(const list<string> &symbols)
    {
        for (const string &l : symbols)
            this->symbols.insert(l);
    }
    void add_action(const Action &action)
    {
        this->actions.insert(action);
    }

    Action get_action(const string &name) const
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }

    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    friend ostream &operator<<(ostream &os, const Env &w)
    {
        os << "***** Environment *****" << endl
           << endl;
        os << "Symbols: ";
        for (const string &s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (const GroundedCondition &s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (const GroundedCondition &g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (const Action &g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }

    // add getters
    auto get_initial_conditions() const
    {
        return this->initial_conditions;
    }
    auto get_goal_conditions() const
    {
        return this->goal_conditions;
    }
    auto get_actions() const
    {
        return this->actions;
    }
};

class GroundedAction
{
    string name;
    list<string> arg_values;

public:
    GroundedAction(const string &name, const list<string> &arg_values)
    {
        this->name = name;
        for (const string &ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction &rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream &operator<<(ostream &os, const GroundedAction &gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->name;
        temp += "(";
        for (const string &l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env *create_env(char *filename)
{
    ifstream input_file(filename);
    Env *env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line.empty())
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str())); // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

// For convenience
using CondSet = unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>;

// Creating an extended grounded actions struct
struct GroundedActionExt{
    
    GroundedAction grounded_action;
    CondSet preconditions;
    CondSet add_list;
    CondSet delete_list;

    public:

        GroundedActionExt(const GroundedAction &ga,
                  const CondSet &pre,
                  const CondSet &add,
                  const CondSet &del)
                    : grounded_action(ga), preconditions(pre), add_list(add), delete_list(del) {}
};


// Function to generate permutations of all symbols
void permute_symbols(vector<list<string>> &symbols_permute_list, const unordered_set<string> &symbols, int num_inputs, list<string> permute_list = list<string>()){

    // If the number of permutations has been reached, append to the list
    if (num_inputs == 0){
        symbols_permute_list.push_back(permute_list);
        return;
    }
    
    // Iterate through each symbol
    for(const string &symb : symbols){

        // If symb is already in the list, then don't append
        if (find(permute_list.begin(), permute_list.end(), symb) != permute_list.end()){
            continue;
        } 
        permute_list.push_back(symb);

        // Call recursion to put the next element in
        permute_symbols(symbols_permute_list, symbols, num_inputs - 1, permute_list);
        // Remove the last element to append the next element
        permute_list.pop_back();
    }
}

// Function to convert a condition to the grounded condition
GroundedCondition convert_to_groundedcondition(const Condition &condition, const unordered_map<string, string> &map){

    // Create the arg_list
    list<string> ground_args_list;

    // Check if the condition argument exists within the map, else add the argument
    for (const string &arg : condition.get_args()){
        if (map.count(arg)){
            ground_args_list.push_back(map.at(arg));
        } else {
            ground_args_list.push_back(arg);
        }
    }

    // Return as a Grounded Condition 
    return GroundedCondition(condition.get_predicate(), ground_args_list, condition.get_truth());
    
}

// Function to check if the given action is applicable at the current state
bool isActionValid(const CondSet &state, const GroundedActionExt &action){


    // Iterate through each predicate in the preconditions
    for (const GroundedCondition &pred : action.preconditions){

        // Checking if the precondtion exists in the state
        if (pred.get_truth()) {
            if (!state.count(pred)) return false;          
        } else {
            GroundedCondition pos(pred.get_predicate(), pred.get_arg_values(), true);
            if (state.count(pos)) return false;            
        }
    }

    return true;
}

// Apply a given action to a state
CondSet applyAction(CondSet state, const GroundedActionExt &action){

    // Iterate through all conditions in the delete list and remove from state
    for (const GroundedCondition &del : action.delete_list){
        state.erase(del);
    }

    // Iterate through all conditions in add list and add them to the state
    for (const GroundedCondition &add : action.add_list){
        state.insert(add);
    }

    return state;
}

// State comparators and hashers
struct StateComparator
{
    bool operator()(const CondSet &lhs, const CondSet &rhs) const
    {
        return lhs == rhs;
    }
};

struct StateHasher
{
    size_t operator()(const CondSet &cond) const
    {
        size_t total = 0;
        for (const GroundedCondition &literal: cond){
            total += GroundedConditionHasher{}(literal);
        }
        return total;
    }
};

// Node parsed within A*
struct Node {

    CondSet state;
    int g;
    int h;
    int parent;
    int action;
    bool closed;

    public:

        Node(const CondSet &state,  int g, int h, int parent, int action, bool closed = false)
            : state(state), g(g), h(h), parent(parent), action(action), closed(closed) {}
};

// Function to check if the current state has the goal conditions
bool isGoal(const CondSet &state, const CondSet &goal) {
    for (const GroundedCondition &g : goal) {
        if (!state.count(g)) return false;
    }
    return true;
}

// Function to return the heuristic: Number of states in current not present in goal
int astar_heuristic(const CondSet &state, const CondSet &goal){

    // Counter to add if state doesn't exist
    int h = 0;

    // Iterating through each goal condition
    for (const GroundedCondition &cond : goal){
        if(!state.count(cond)){
            h++;
        }
    }

    return h;
}

int empty_delete_heuristic(const CondSet &state, const CondSet &goal, const vector<GroundedActionExt> &action_list){
    
    // Set the facts as they are in the state
    CondSet facts = state;
    // layers is the heuristic
    int layers = 0;

    while (!isGoal(facts, goal)){
        // As facts get added
        CondSet new_facts = facts;
        for (const GroundedActionExt &action : action_list){
            if (isActionValid(facts, action)){
                for (const GroundedCondition &add : action.add_list){
                    new_facts.insert(add);
                }
            }
        }

        if (new_facts == facts){
            return INF;
        }
        facts = new_facts;
        layers++;
    }

    return layers;
}

// Function to get choice of heuristic
int compute_h(const CondSet &state, const CondSet &goal, const vector<GroundedActionExt> &action_list) {
    switch (planner_mode) {
        case 1: return 0;
        case 2: return astar_heuristic(state, goal);   // goal count
        case 3: return empty_delete_heuristic(state, goal, action_list);                        // TODO: empty-delete-list
    }
    return 0;
}

list<GroundedAction> planner(Env *env)
{
    //////////////////////////////////////////
    ///// TODO: INSERT YOUR PLANNER HERE /////
    //////////////////////////////////////////

    // Starting a timer
    auto t_start = chrono::steady_clock::now();

    // Phase 1: Ground all actions before searching for a plan

    // Collect all symbols, actions, initial and goal conditions from the environment
    unordered_set<string> symbols = env->get_symbols();
    unordered_set<Action, ActionHasher, ActionComparator> actions = env->get_actions();
    CondSet start_state = env->get_initial_conditions();
    CondSet  goal_state = env->get_goal_conditions();

    // Create a set of Grounded Actions ext
    vector<GroundedActionExt> grounded_actions_set;

    // Build all possible grounded actions
    for (const Action &action : actions){
        
        // Taking all the attributes of the given action
        list<string> act_args = action.get_args();
        unordered_set<Condition, ConditionHasher, ConditionComparator> act_preconds = action.get_preconditions();
        unordered_set<Condition, ConditionHasher, ConditionComparator> act_effects = action.get_effects();

        // permutation of the symbols
        vector<list<string>> symbols_permute_list;
        permute_symbols(symbols_permute_list, symbols, act_args.size());
        
        // Create a map each for the parameters for all symbol permutations
        for (list<string> symb_list : symbols_permute_list){

            // Obtain the grounded conditions for each action based on its param map
            CondSet act_ground_preconds, act_ground_addeffects, act_ground_deleteeffects;

            // Map per symbol permutation
            unordered_map<string, string> param_list;
            auto it = symb_list.begin();
            for (const string &arg : act_args){
                param_list.emplace(arg, *it);
                ++it;   
            }
            
            // Get the grounded preconditions for each param map
            for (const Condition &precond : act_preconds){
                act_ground_preconds.insert(convert_to_groundedcondition(precond, param_list));
            }

            // Get the grounded effects for each param map (add, delete)
            for (const Condition &effect : act_effects){

                GroundedCondition g = convert_to_groundedcondition(effect, param_list);

                if (g.get_truth()){
                    act_ground_addeffects.insert(g);
                } else {
                    act_ground_deleteeffects.insert(GroundedCondition(g.get_predicate(), g.get_arg_values(), true));
                }
            }

            // Create the Grounded Action
            GroundedAction grounded_act = GroundedAction(action.get_name(), symb_list);

            // Add it to a Grounded Action Ext vector
            grounded_actions_set.push_back(GroundedActionExt(grounded_act, act_ground_preconds, 
                                                    act_ground_addeffects, act_ground_deleteeffects));
        }
    }

    // Phase 2 : Create the A* planner

    // Create a list of nodes for the A* planner
    vector<Node> all_nodes;
    // To record the states with a number
    unordered_map<CondSet, int, StateHasher, StateComparator> state_to_index;
    // The plan given by A*
    list<GroundedAction> plan;
    // To check how many nodes were expanded (closed)
    int expanded = 0;

    // Add the start state
    all_nodes.push_back(Node(start_state, 0, compute_h(start_state, goal_state, grounded_actions_set), -1, -1, false));
    state_to_index[start_state] = 0;

    // Creating the open queue
    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> open;
    open.push({all_nodes[0].h + all_nodes[0].g, 0});

    while (!open.empty()){
        int priority = open.top().first;
        int curr = open.top().second;
        open.pop();

        if (all_nodes[curr].closed){
            continue;
        }
        all_nodes[curr].closed = true;
        expanded++;

        if (isGoal(all_nodes[curr].state, goal_state)){
            int i = curr;
            while (all_nodes[i].parent != -1){

                int a = all_nodes[i].action;
                plan.push_front(grounded_actions_set[a].grounded_action);
                i = all_nodes[i].parent;

            }
            // right before you return the plan:
            auto t_end = chrono::steady_clock::now();
            double seconds = chrono::duration<double>(t_end - t_start).count();
            // Print time taken by the planner
            cout << "The total time taken by the planner is : " << seconds << endl;
            cout << "The number of nodes expanded by the planner is : " << expanded << endl;
            return plan;
        }

        for (size_t i = 0; i < grounded_actions_set.size(); i++){
            
            if (!isActionValid(all_nodes[curr].state, grounded_actions_set[i])){
                continue;
            }

            CondSet child = applyAction(all_nodes[curr].state, grounded_actions_set[i]);
            int new_g = all_nodes[curr].g + 1;

            auto it = state_to_index.find(child);

            // If it is found
            if (it != state_to_index.end()) {

                // Get the node's index
                int k = it->second;

                // If it isnt a better choice
                if (all_nodes[k].closed || new_g >= all_nodes[k].g){
                    continue;
                }   

                // Otherwise
                all_nodes[k].g = new_g;
                all_nodes[k].parent = curr;
                all_nodes[k].action = i;
                open.push({new_g + all_nodes[k].h, k});
            } else {
                // Create a node and record it
                int k = all_nodes.size();
                all_nodes.push_back(Node(child, new_g, compute_h(child, goal_state, grounded_actions_set), curr, i, false));
                state_to_index[child] = k;
                open.push({new_g + all_nodes[k].h, k});
            }
        }
    }
    // right before you return the plan:
    auto t_end = chrono::steady_clock::now();
    double seconds = chrono::duration<double>(t_end - t_start).count();
    // Print time taken by the planner
    cout << "The total time taken by the planner is : " << seconds << endl;
    cout << "The number of nodes expanded by the planner is : " << expanded << endl;
    cout << "No plan found" << endl;
    return plan;
}

int main(int argc, char *argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    // char *env_file = static_cast<char *>("example.txt");
    const char *env_file = argv[1];
    if (argc > 1) {
        env_file = argv[1];
    }

    if (argc > 2) {
        planner_mode = std::stoi(argv[2]);
    }

    if (planner_mode < 1 || planner_mode > 3) {
        throw runtime_error("Planner mode must be 1, 2, or 3");
    }
    std::string envsDirPath = ENVS_DIR;
    char *filename = new char[envsDirPath.length() + strlen(env_file) + 2];
    strcpy(filename, envsDirPath.c_str());
    strcat(filename, "/");
    strcat(filename, env_file);

    cout << "Environment: " << filename << endl;
    Env *env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (const GroundedAction &gac : actions)
    {
        cout << gac << endl;
    }

    delete env;
    return 0;
}