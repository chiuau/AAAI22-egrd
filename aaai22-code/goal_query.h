#ifndef FOCTL_GOAL_QUERY_H
#define FOCTL_GOAL_QUERY_H

#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <unordered_map>
#include <memory>
#include <locale>

#include "shared.h"
#include "name_id_map.h"
#include "foctl.h"


// -----------------------------------------------------------------------------------------
// GOAL_QUERY_TYPE
// -----------------------------------------------------------------------------------------

enum class GOAL_QUERY_TYPE {
  GOAL_QUERY,
  TRUE_GOAL_QUERY,
  FALSE_GOAL_QUERY,
  CONSTANT_GOAL_QUERY,
  VARIABLE_GOAL_QUERY,
  NOT_GOAL_QUERY,
  AND_GOAL_QUERY,
  OR_GOAL_QUERY,
  IMPLY_GOAL_QUERY,
  IFF_GOAL_QUERY,
  EQUAL_GOAL_QUERY,
  FORALL_GOAL_QUERY,
  EXIST_GOAL_QUERY,
  EXCLUDE_GOAL_QUERY,
  EDGE_GOAL_QUERY,
  NIL_GOAL_QUERY,
  END_GOAL_QUERY,
};


// -----------------------------------------------------------------------------------------
// Translate Info
// -----------------------------------------------------------------------------------------

struct TranslateInfo {
  const std::set<std::string>& strongly_recognizable_set;
  NameIdMap& constantIdMap;
  NameIdMap& variableIdMap;
  const std::vector<int>& variableId2CommonParentNodeId;
  const std::unordered_map<int,std::vector<int>>& commonParentNodeId2VariableIds;
  // const std::vector<std::vector<int>>& dependedVariablesOfEdge;  // *** should be removed ***
  std::vector<std::vector<bool>> unequal_variables_matrix;
  std::vector<std::set<int>> unequal_constants;

  std::vector<int> addedRecognizableVariableIds;
  std::vector<std::string> addedRecognizableVariableNames;

  TranslateInfo(const std::set<std::string>& strongly_recognizable_set,
                NameIdMap& constantIdMap,
                NameIdMap& variableIdMap,
                const std::vector<int>& variableId2CommonParentNodeId,
                const std::unordered_map<int,std::vector<int>>& commonParentNodeId2VariableIds,
                const std::vector<std::vector<bool>>& unequal_variables_matrix,
                const std::vector<std::set<int>>& unequal_constants) :
      strongly_recognizable_set{strongly_recognizable_set}, constantIdMap{constantIdMap}, variableIdMap{variableIdMap},
      variableId2CommonParentNodeId{variableId2CommonParentNodeId}, commonParentNodeId2VariableIds{commonParentNodeId2VariableIds},
      unequal_variables_matrix{unequal_variables_matrix}, unequal_constants{unequal_constants}
  {
    // do nothing
  }

//  TranslateInfo(const std::set<std::string>& strongly_recognizable_set,
//                NameIdMap& constantIdMap,
//                NameIdMap& variableIdMap,
//                const std::vector<int>& variableId2CommonParentNodeId,
//                const std::unordered_map<int,std::vector<int>>& commonParentNodeId2VariableIds,
//                const std::vector<std::vector<int>>& dependedVariablesOfEdge,
//                const std::vector<std::vector<bool>>& unequal_variables_matrix,
//                const std::vector<std::set<int>>& unequal_constants) :
//      strongly_recognizable_set{strongly_recognizable_set}, constantIdMap{constantIdMap}, variableIdMap{variableIdMap},
//      variableId2CommonParentNodeId{variableId2CommonParentNodeId}, commonParentNodeId2VariableIds{commonParentNodeId2VariableIds}, dependedVariablesOfEdge{dependedVariablesOfEdge},
//      unequal_variables_matrix{unequal_variables_matrix}, unequal_constants{unequal_constants}
//  {
//    // do nothing
//  }

  bool isStronglyRecognizable(const std::string& variable_name) {
    return strongly_recognizable_set.find(variable_name) != strongly_recognizable_set.end();
  }

};

// -----------------------------------------------------------------------------------------
// Helper function
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
std::unique_ptr<ForAllStatement<T,V>> makeExclusiveStatement(NameIdMap& variableIdMap, NameIdMap& constantIdMap, const std::vector<std::string> excluded_variable_names = {}, const std::vector<std::string> excluded_constant_names = {}) {
  int new_variable_id = variableIdMap.makeNewName();
  std::string new_variable_name = variableIdMap.getNameById(new_variable_id);

  std::vector<std::pair<int,std::string>> excluded_variable_idnames;
  for(auto& vname : excluded_variable_names) {
    int vid = variableIdMap.getIdByName(vname);
    if (vid < 0) throw std::runtime_error("Error in makeExclusiveStatement(): Unknown variable name " + vname);
    excluded_variable_idnames.emplace_back(vid, vname);
  }

  std::vector<std::pair<int,std::string>> excluded_constant_idnames;
  for(auto& cname : excluded_constant_names) {
    int cid = constantIdMap.getIdByName(cname);
    if (cid < 0) cid = constantIdMap.assignIdToName(cname);
    excluded_constant_idnames.emplace_back(cid, cname);
  }

  std::unique_ptr<Variable<T,V>> var_ptr_1{new Variable<T,V>(0, new_variable_name, new_variable_id)};
  std::unique_ptr<Variable<T,V>> var_ptr_2{new Variable<T,V>(0, new_variable_name, new_variable_id)};
  std::unique_ptr<NotStatement<T,V>> not_ptr{new NotStatement<T,V>(0, std::move(var_ptr_2))};

  //  std::unique_ptr<ForAllStatement<T,V>> forall_ptr{ new ForAllStatement<T,V>(0, std::move(var_ptr_1), excluded_variable_idnames, excluded_constant_idnames, std::move(not_ptr)) };
  //  return forall_ptr;

  return std::make_unique<ForAllStatement<T,V>>(0, std::move(var_ptr_1), excluded_variable_idnames, excluded_constant_idnames, std::move(not_ptr));
}

template<typename T, typename V>
Statement<T,V>* appendRecognizableVariables(TranslateInfo& translate_info, std::unique_ptr<Statement<T,V>> statement, std::vector<std::string> vnames, std::vector<int> vids) {
  if (vnames.empty()) return statement.release();

  std::string variable_name = vnames.back();
  vnames.pop_back();
  int variable_id = vids.back();
  vids.pop_back();

  std::vector<std::pair<int,std::string>> excluded_variable_idnames;
  std::vector<std::pair<int,std::string>> excluded_constant_idnames;

  // **** wrong ****
//  for(auto vid : translate_info.dependedVariablesOfEdge[variable_id]) {
//    if (translate_info.unequal_variables_matrix[variable_id][vid]) {
//      excluded_variable_idnames.emplace_back(vid, translate_info.variableIdMap.getNameById(vid));
//    }
//  }
//  for(int cid : translate_info.unequal_constants[variable_id]) {
//    excluded_constant_idnames.emplace_back(cid, translate_info.constantIdMap.getNameById(cid));
//  }
  // ***********

  for(int vid=0; vid < translate_info.variableIdMap.size(); vid++) {
    if (std::ranges::find(translate_info.addedRecognizableVariableIds, vid) == translate_info.addedRecognizableVariableIds.end()) {
      if (translate_info.unequal_variables_matrix[variable_id][vid]) {
        excluded_variable_idnames.emplace_back(vid, translate_info.variableIdMap.getNameById(vid));
      }
    }
  }
  for(int cid : translate_info.unequal_constants[variable_id]) {
    excluded_constant_idnames.emplace_back(cid, translate_info.constantIdMap.getNameById(cid));
  }

  translate_info.addedRecognizableVariableIds.push_back(variable_id);
  translate_info.addedRecognizableVariableNames.push_back(variable_name);

  std::unique_ptr<Variable<T,V>> var_ptr{new Variable<T,V>(0, variable_name, variable_id)};
  std::unique_ptr<Statement<T,V>> statement2{new ExistStatement<T,V>(0, std::move(var_ptr), excluded_variable_idnames, excluded_constant_idnames, std::move(statement))};

  return appendRecognizableVariables(translate_info, std::move(statement2), vnames, vids);
}

// -----------------------------------------------------------------------------------------
// GqsGraph
// -----------------------------------------------------------------------------------------

class GqsGraph {
  int n;
  std::vector<bool> isState;
  std::vector<bool> isEdge;
  std::vector<bool> isEdgeCond;
  std::vector<int>  edgeCondIdOfEdge;

  std::vector<int> state_next_edge;
  std::vector<std::vector<int>> edge_next_states;

public:

  GqsGraph(int n) :
      n{n}, isState(n, true), isEdge(n, false), isEdgeCond(n, false), edgeCondIdOfEdge(n, -1), state_next_edge(n, -1), edge_next_states(n)
  {
    // do nothing
  }

  void setNotState(int id) { isState[id] = false; }

  void setEdge(int id) { isEdge[id] = true; }

  void setEdgeCond(int cond_id, int edge_id) { isEdgeCond[cond_id] = true; edgeCondIdOfEdge[edge_id] = cond_id; }

  void setStateNextEdge(int state_id, int edge_id) {
    state_next_edge[state_id] = edge_id;
  }

  void setEdgeNextState(int edge_id, int state_id) {
    edge_next_states[edge_id].push_back(state_id);
  }

  std::vector<int> getStates() const {
    std::vector<int> result;
    for(int id=0; id<n; id++) {
      if (isState[id]) result.push_back(id);
    }
    return result;
  }

  std::vector<int> getEdges() const {
    std::vector<int> result;
    for(int id=0; id<n; id++) {
      if (isEdge[id]) result.push_back(id);
    }
    return result;
  }

  int getEdgeCondIdOfEdge(int edge_id) const { return edgeCondIdOfEdge[edge_id]; }

  const int getNextEdgeIdOfState(int state_id) const { return state_next_edge[state_id]; }

  const std::vector<int>& getNextStateIdsOfEdge(int edge_id) const { return edge_next_states[edge_id]; }


  void debug() {
    for(int i=0; i<n; i++) {
      if (isState[i]) __pp__("Node ", i, " is state.");
      if (edgeCondIdOfEdge[i] >= 0) {
        if (isEdge[i]) __pp__("Node ", i, " is Edge with Cond ", edgeCondIdOfEdge[i]);
      } else {
        if (isEdge[i]) __pp__("Node ", i, " is Edge with no Cond");
      }
      if (isEdgeCond[i]) __pp__("Node ", i, " is Edge Cond");
      if (state_next_edge[i] >= 0) __pp__("Node ", i, "'s next edge is Edge ", state_next_edge[i]);
      if (!edge_next_states[i].empty()) __pp__("Edge ", i, "'s next state is ", edge_next_states[i]);
    }
    __pp__();
  }

};


// -----------------------------------------------------------------------------------------
// GOAL_QUERY
// -----------------------------------------------------------------------------------------


template<typename T, typename V>
class GoalQuery {
public:

  struct ToStringInfo {
    const std::set<std::string>& strongly_recognizable_set;
    const std::set<std::string>& weakly_recognizable_set;
    std::set<std::string> visited;

    ToStringInfo(const std::set<std::string>& strongly_recognizable_set,
                 const std::set<std::string>& weakly_recognizable_set) :
       strongly_recognizable_set{strongly_recognizable_set}, weakly_recognizable_set{weakly_recognizable_set}
    {
      // do nothing
    }

    bool isStronglyRecognizable(const std::string& vname) const {
      return std::ranges::find(strongly_recognizable_set, vname) != strongly_recognizable_set.end();
    }

    bool isWeaklyRecognizable(const std::string& vname) const {
      return std::ranges::find(weakly_recognizable_set, vname) != weakly_recognizable_set.end();
    }

    bool hasVisited(const std::string& vname) const {
      return std::ranges::find(visited, vname) != visited.end();
    }

    void addToVisited(const std::string& vname) {
      visited.insert(vname);
    }
  };

protected:
  int id;
  std::string spec_db_key;

public:

  GoalQuery() {}

  GoalQuery(const GoalQuery& other) : id{other.id}, spec_db_key{other.spec_db_key} {}

  std::unique_ptr<GoalQuery<T,V>> clone() const { return std::unique_ptr<GoalQuery<T,V>>(doClone()); }

  int getId() const { return id; }

  virtual GOAL_QUERY_TYPE getType() const { return GOAL_QUERY_TYPE::GOAL_QUERY; }

  void setEvalSpecDbKey(const std::string& spec_db_key) { GoalQuery::spec_db_key = spec_db_key; }

  virtual void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) = 0;
  virtual void assignConstantId(NameIdMap& constantIdMap) = 0;
  virtual void assignVariableId(NameIdMap& variableIdMap) = 0;
  virtual void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const = 0;
//  virtual void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const = 0;
  virtual void buildGqsGraph(GqsGraph& gqs_graph) const = 0;

  virtual std::unique_ptr<Statement<T,V>> translate(TranslateInfo& translate_info) const {
    auto statement_ptr = doTranslate(translate_info);
    if (!spec_db_key.empty()) statement_ptr->setEvalSpecByDbKey(spec_db_key);
    return std::unique_ptr<Statement<T,V>>(statement_ptr);
  }

  virtual Statement<T,V>* doTranslate(TranslateInfo& translate_info) const = 0;

  virtual std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const { return ""; }
  // virtual std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const { return std::to_string(id) + ":"; }


private:
  virtual GoalQuery<T,V>* doClone() const = 0;

};


// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class TrueGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

public:

  TrueGoalQuery() {}
  TrueGoalQuery(const TrueGoalQuery& other) : GoalQuery<T,V>{other} {}

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::TRUE_GOAL_QUERY; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
  }

  void assignConstantId(NameIdMap& constantIdMap) final { }
  void assignVariableId(NameIdMap& variableIdMap) final { }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {}
//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {}

  void buildGqsGraph(GqsGraph& gqs_graph) const final {}


  Statement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    return new TrueStatement<T,V>(0);
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    return GoalQuery<T,V>::to_string(context) + "True";
  }

private:

  TrueGoalQuery<T,V>* doClone() const final { return new TrueGoalQuery<T,V>(*this); }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class FalseGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

public:

  FalseGoalQuery() {}
  FalseGoalQuery(const FalseGoalQuery& other) : GoalQuery<T,V>{other} {}

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::FALSE_GOAL_QUERY; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
  }

  void assignConstantId(NameIdMap& constantIdMap) final { }
  void assignVariableId(NameIdMap& variableIdMap) final { }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {}
//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {}

  void buildGqsGraph(GqsGraph& gqs_graph) const final {}


  Statement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    return new FalseStatement<T,V>(0);
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    return GoalQuery<T,V>::to_string(context) + "False";
  }

private:

  FalseGoalQuery<T,V>* doClone() const final { return new FalseGoalQuery<T,V>(*this); }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class ConstantGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  std::string constant_name;
  int constant_id;

public:

  ConstantGoalQuery(std::string constant_name) :
      constant_name{constant_name}
  {
    // do nothing
  }

  ConstantGoalQuery(const ConstantGoalQuery& other) :
      GoalQuery<T,V>{other}, constant_name{other.constant_name}, constant_id{other.constant_id}
  {
    // do nothing
  }


  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::CONSTANT_GOAL_QUERY; }

  std::string getConstantName() const { return constant_name; }

  int getConstantId() const { return constant_id; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
  }

  void assignConstantId(NameIdMap& constantIdMap) final { constant_id = constantIdMap.assignIdToName(constant_name); }
  void assignVariableId(NameIdMap& variableIdMap) final { }
  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {}
//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {}

  void buildGqsGraph(GqsGraph& gqs_graph) const final {}

  Constant<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    return new Constant<T,V>(0, constant_name, translate_info.constantIdMap.getIdByName(constant_name));
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    return GoalQuery<T,V>::to_string(context) + constant_name;
  }

private:

  ConstantGoalQuery<T,V>* doClone() const final { return new ConstantGoalQuery<T,V>(*this); }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class VariableGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  std::string variable_name;
  int variable_id;

public:

  VariableGoalQuery(std::string variable_name) :
      variable_name{variable_name}
  {
    // do nothing
  }

  VariableGoalQuery(const VariableGoalQuery& other) :
      GoalQuery<T,V>{other}, variable_name{other.variable_name}, variable_id{other.variable_id}
  {
    // do nothing
  }

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::VARIABLE_GOAL_QUERY; }

  std::string getVariableName() const { return variable_name; }

  int getVariableId() const { return variable_id; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
  }

  void assignConstantId(NameIdMap& constantIdMap) final { }
  void assignVariableId(NameIdMap& variableIdMap) final { variable_id = variableIdMap.assignIdToName(variable_name); }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {
    std::vector<int>& last_prefix = variableId2Prefix[variable_id];
    if (last_prefix.empty()) {
      last_prefix = current_prefix;
    } else {
      std::vector<int> common_prefix;
      for(int i=0; i<std::min(last_prefix.size(), current_prefix.size()); i++) {
        if (last_prefix[i] == current_prefix[i]) {
          common_prefix.push_back(current_prefix[i]);
        } else {
          break;
        }
      }
      last_prefix = common_prefix;
    }
  };

//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {}

  void buildGqsGraph(GqsGraph& gqs_graph) const final {}

  Variable<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    return new Variable<T,V>(0, variable_name, translate_info.variableIdMap.getIdByName(variable_name));
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    std::string s = GoalQuery<T,V>::to_string(context) + variable_name;
    if (context.isWeaklyRecognizable(variable_name) && !context.hasVisited(variable_name)) {
      s += "*";
      context.addToVisited(variable_name);
    }
    return s;
  }

private:

  VariableGoalQuery<T,V>* doClone() const final { return new VariableGoalQuery<T,V>(*this); }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
std::string calcExcludedCVDescription(const std::string& prefix,
                                      typename GoalQuery<T,V>::ToStringInfo& context,
                                      const std::vector<std::unique_ptr<VariableGoalQuery<FPST,FPSV>>>& excluded_variables,
                                      const std::vector<std::unique_ptr<ConstantGoalQuery<FPST,FPSV>>>& excluded_constants)
{
  if (excluded_variables.empty() && excluded_constants.empty()) return "";

  std::string s = prefix + "{";

  for(bool isFirst=true; auto& ev : excluded_variables) {
    if (!isFirst) s += ",";
    s += ev->to_string(context);
    isFirst=false;
  }
  if (!excluded_variables.empty() && !excluded_constants.empty()) s += ",";
  for(bool isFirst=true; auto& ec : excluded_constants) {
    if (!isFirst) s += ",";
    s += ec->to_string(context);
    isFirst=false;
  }
  s += "}";
  return s;
}

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class NotGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr;

public:

  NotGoalQuery(std::unique_ptr<GoalQuery<T,V>> goal_query_ptr) : goal_query_ptr{std::move(goal_query_ptr)} {
    // do nothing
  }

  NotGoalQuery(const NotGoalQuery& other) :
      GoalQuery<T,V>{other}, goal_query_ptr{other.goal_query_ptr->clone()}
  {
    // do nothing
  }

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::NOT_GOAL_QUERY; }
  const std::unique_ptr<GoalQuery<T,V>>& getGoalQueryPtr1() const { return goal_query_ptr; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
    goal_query_ptr->setNodeIdByDFS(goal_query_nodes);
  }

  void assignConstantId(NameIdMap& constantIdMap) final { goal_query_ptr->assignConstantId(constantIdMap); }
  void assignVariableId(NameIdMap& variableIdMap) final { goal_query_ptr->assignVariableId(variableIdMap); }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {
    goal_query_ptr->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
  }

//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {
//    goal_query_ptr->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//  }

  void buildGqsGraph(GqsGraph& gqs_graph) const final {
    goal_query_ptr->buildGqsGraph(gqs_graph);
    gqs_graph.setNotState(goal_query_ptr->getId());
  }

  NotStatement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    return new NotStatement<T,V>(0, std::move(goal_query_ptr->translate(translate_info)));
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    return GoalQuery<T,V>::to_string(context) + "(not " + goal_query_ptr->to_string(context) + ")";
  }

private:

  NotGoalQuery<T,V>* doClone() const final { return new NotGoalQuery<T,V>(*this); }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class EdgeGoalQuery;

template<typename T, typename V>
class AndGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_1;
  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_2;
  bool isBetweenStateAndEdge;

public:

  AndGoalQuery(std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_1, std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_2, bool isBetweenStateAndEdge = false) :
     goal_query_ptr_1{std::move(goal_query_ptr_1)}, goal_query_ptr_2{std::move(goal_query_ptr_2)}, isBetweenStateAndEdge{isBetweenStateAndEdge}
  {
    // do nothing
  }

  AndGoalQuery(const AndGoalQuery& other) :
      GoalQuery<T,V>{other}, goal_query_ptr_1{other.goal_query_ptr_1->clone()}, goal_query_ptr_2{other.goal_query_ptr_2->clone()}, isBetweenStateAndEdge{other.isBetweenStateAndEdge}
  {
    // do nothing
  }

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::AND_GOAL_QUERY; }
  const std::unique_ptr<GoalQuery<T,V>>& getGoalQueryPtr1() const { return goal_query_ptr_1; }
  const std::unique_ptr<GoalQuery<T,V>>& getGoalQueryPtr2() const { return goal_query_ptr_2; }

  bool getIsBetweenStateAndEdge() const { return isBetweenStateAndEdge; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
    goal_query_ptr_1->setNodeIdByDFS(goal_query_nodes);
    goal_query_ptr_2->setNodeIdByDFS(goal_query_nodes);
  }

  void assignConstantId(NameIdMap& constantIdMap) final { goal_query_ptr_1->assignConstantId(constantIdMap); goal_query_ptr_2->assignConstantId(constantIdMap); }
  void assignVariableId(NameIdMap& variableIdMap) final { goal_query_ptr_1->assignVariableId(variableIdMap); goal_query_ptr_2->assignVariableId(variableIdMap); }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {
    goal_query_ptr_1->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
    goal_query_ptr_2->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
  }

//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {
//    goal_query_ptr_1->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//    goal_query_ptr_2->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//  }

  void buildGqsGraph(GqsGraph& gqs_graph) const final {
    if (isBetweenStateAndEdge) {
      goal_query_ptr_1->buildGqsGraph(gqs_graph);
      goal_query_ptr_2->buildGqsGraph(gqs_graph);
      gqs_graph.setNotState(goal_query_ptr_2->getId());
      gqs_graph.setEdge(goal_query_ptr_2->getId());
      gqs_graph.setNotState(id);
      gqs_graph.setStateNextEdge(goal_query_ptr_1->getId(), goal_query_ptr_2->getId());
    } else {
      goal_query_ptr_1->buildGqsGraph(gqs_graph);
      goal_query_ptr_2->buildGqsGraph(gqs_graph);
      gqs_graph.setNotState(goal_query_ptr_1->getId());
      gqs_graph.setNotState(goal_query_ptr_2->getId());
    }
  }

  Statement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    if (goal_query_ptr_1->getType() == GOAL_QUERY_TYPE::EDGE_GOAL_QUERY && (dynamic_cast<EdgeGoalQuery<T,V>*>(goal_query_ptr_1.get()))->isPrunable()) {
      if (goal_query_ptr_2->getType() == GOAL_QUERY_TYPE::EDGE_GOAL_QUERY && (dynamic_cast<EdgeGoalQuery<T,V>*>(goal_query_ptr_2.get()))->isPrunable()) {
        return new TrueStatement<T,V>(0);
      } else {
        return goal_query_ptr_2->doTranslate(translate_info);
      }
    } else {
      if (goal_query_ptr_2->getType() == GOAL_QUERY_TYPE::EDGE_GOAL_QUERY && (dynamic_cast<EdgeGoalQuery<T,V>*>(goal_query_ptr_2.get()))->isPrunable()) {
        return goal_query_ptr_1->doTranslate(translate_info);
      } else {
        return new AndStatement<T,V>(0, std::move(goal_query_ptr_1->translate(translate_info)), std::move(goal_query_ptr_2->translate(translate_info)));
      }
    }
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    if (goal_query_ptr_1->getType() != GOAL_QUERY_TYPE::EDGE_GOAL_QUERY && goal_query_ptr_2->getType() == GOAL_QUERY_TYPE::EDGE_GOAL_QUERY) {
      return goal_query_ptr_1->to_string(context) + " " + goal_query_ptr_2->to_string(context);
    } else if (goal_query_ptr_1->getType() == GOAL_QUERY_TYPE::EDGE_GOAL_QUERY && goal_query_ptr_2->getType() != GOAL_QUERY_TYPE::EDGE_GOAL_QUERY) {
      return goal_query_ptr_1->to_string(context) + " " + goal_query_ptr_2->to_string(context);
    } else {
      return GoalQuery<T,V>::to_string(context) + "(" + goal_query_ptr_1->to_string(context) + " and " + goal_query_ptr_2->to_string(context) + ")";
    }
  }

private:

  AndGoalQuery<T,V>* doClone() const final { return new AndGoalQuery<T,V>(*this); }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class OrGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_1;
  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_2;
  bool isBetweenTraces;

public:

  OrGoalQuery(std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_1, std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_2, bool isBetweenTraces = false) :
      goal_query_ptr_1{std::move(goal_query_ptr_1)}, goal_query_ptr_2{std::move(goal_query_ptr_2)}, isBetweenTraces{isBetweenTraces}
  {
    // do nothing
  }

  OrGoalQuery(const OrGoalQuery& other) :
      GoalQuery<T,V>{other}, goal_query_ptr_1{other.goal_query_ptr_1->clone()}, goal_query_ptr_2{other.goal_query_ptr_2->clone()}, isBetweenTraces{other.isBetweenTraces}
  {
    // do nothing
  }

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::OR_GOAL_QUERY; }
  const std::unique_ptr<GoalQuery<T,V>>& getGoalQueryPtr1() const { return goal_query_ptr_1; }
  const std::unique_ptr<GoalQuery<T,V>>& getGoalQueryPtr2() const { return goal_query_ptr_2; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
    goal_query_ptr_1->setNodeIdByDFS(goal_query_nodes);
    goal_query_ptr_2->setNodeIdByDFS(goal_query_nodes);
  }

  void assignConstantId(NameIdMap& constantIdMap) final { goal_query_ptr_1->assignConstantId(constantIdMap); goal_query_ptr_2->assignConstantId(constantIdMap); }
  void assignVariableId(NameIdMap& variableIdMap) final { goal_query_ptr_1->assignVariableId(variableIdMap); goal_query_ptr_2->assignVariableId(variableIdMap); }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {
    goal_query_ptr_1->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
    goal_query_ptr_2->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
  }

//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {
//    goal_query_ptr_1->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//    goal_query_ptr_2->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//  }

  void buildGqsGraph(GqsGraph& gqs_graph) const final {
    if (isBetweenTraces) {
      gqs_graph.setNotState(id);
      goal_query_ptr_1->buildGqsGraph(gqs_graph);
      goal_query_ptr_2->buildGqsGraph(gqs_graph);
    } else {
      goal_query_ptr_1->buildGqsGraph(gqs_graph);
      goal_query_ptr_2->buildGqsGraph(gqs_graph);
      gqs_graph.setNotState(goal_query_ptr_1->getId());
      gqs_graph.setNotState(goal_query_ptr_2->getId());
    }
  }

  OrStatement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    return new OrStatement<T,V>(0, std::move(goal_query_ptr_1->translate(translate_info)), std::move(goal_query_ptr_2->translate(translate_info)));
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    if (isBetweenTraces) {
      std::string s;
      if (goal_query_ptr_1->getType() == GOAL_QUERY_TYPE::OR_GOAL_QUERY) {
        auto or_ptr = dynamic_cast<OrGoalQuery<T,V>*>(goal_query_ptr_1.get());
        if (or_ptr->isBetweenTraces) {
          s += goal_query_ptr_1->to_string(context);
        } else {
          s += "[ " + goal_query_ptr_1->to_string(context) + " ]";
        }
      } else {
        s += "[ "+ goal_query_ptr_1->to_string(context) + " ]";
      }
      if (goal_query_ptr_2->getType() == GOAL_QUERY_TYPE::OR_GOAL_QUERY) {
        auto or_ptr = dynamic_cast<OrGoalQuery<T,V>*>(goal_query_ptr_2.get());
        if (or_ptr->isBetweenTraces) {
          s += " " + goal_query_ptr_2->to_string(context);
        } else {
          s += " [ "+ goal_query_ptr_2->to_string(context) + " ]";
        }
      } else {
        s += " [ "+ goal_query_ptr_2->to_string(context) + " ]";
      }
      return s;
    } else {
      return GoalQuery<T,V>::to_string(context) + "(" + goal_query_ptr_1->to_string(context) + " or " + goal_query_ptr_2->to_string(context) + ")";
    }
  }

private:

  OrGoalQuery<T,V>* doClone() const final { return new OrGoalQuery<T,V>(*this); }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class ImplyGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_1;
  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_2;

public:

  ImplyGoalQuery(std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_1, std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_2) :
      goal_query_ptr_1{std::move(goal_query_ptr_1)}, goal_query_ptr_2{std::move(goal_query_ptr_2)}
  {
    // do nothing
  }

  ImplyGoalQuery(const ImplyGoalQuery& other) :
      GoalQuery<T,V>{other}, goal_query_ptr_1{other.goal_query_ptr_1->clone()}, goal_query_ptr_2{other.goal_query_ptr_2->clone()}
  {
    // do nothing
  }

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::IMPLY_GOAL_QUERY; }
  const std::unique_ptr<GoalQuery<T,V>>& getGoalQueryPtr1() const { return goal_query_ptr_1; }
  const std::unique_ptr<GoalQuery<T,V>>& getGoalQueryPtr2() const { return goal_query_ptr_2; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
    goal_query_ptr_1->setNodeIdByDFS(goal_query_nodes);
    goal_query_ptr_2->setNodeIdByDFS(goal_query_nodes);
  }

  void assignConstantId(NameIdMap& constantIdMap) final { goal_query_ptr_1->assignConstantId(constantIdMap); goal_query_ptr_2->assignConstantId(constantIdMap); }
  void assignVariableId(NameIdMap& variableIdMap) final { goal_query_ptr_1->assignVariableId(variableIdMap); goal_query_ptr_2->assignVariableId(variableIdMap); }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {
    goal_query_ptr_1->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
    goal_query_ptr_2->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
  }

//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {
//    goal_query_ptr_1->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//    goal_query_ptr_2->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//  }

  void buildGqsGraph(GqsGraph& gqs_graph) const final {
    goal_query_ptr_1->buildGqsGraph(gqs_graph);
    goal_query_ptr_2->buildGqsGraph(gqs_graph);
    gqs_graph.setNotState(goal_query_ptr_1->getId());
    gqs_graph.setNotState(goal_query_ptr_2->getId());
  }

  ImplyStatement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    return new ImplyStatement<T,V>(0, std::move(goal_query_ptr_1->translate(translate_info)), std::move(goal_query_ptr_2->translate(translate_info)));
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    return GoalQuery<T,V>::to_string(context) + "(" + goal_query_ptr_1->to_string(context) + " => " + goal_query_ptr_2->to_string(context) + ")";
  }

private:

  ImplyGoalQuery<T,V>* doClone() const final { return new ImplyGoalQuery<T,V>(*this); }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class IffGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_1;
  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_2;

public:

  IffGoalQuery(std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_1, std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_2) :
     goal_query_ptr_1{std::move(goal_query_ptr_1)}, goal_query_ptr_2{std::move(goal_query_ptr_2)}
  {
    // do nothing
  }

  IffGoalQuery(const IffGoalQuery& other) :
      GoalQuery<T,V>{other}, goal_query_ptr_1{other.goal_query_ptr_1->clone()}, goal_query_ptr_2{other.goal_query_ptr_2->clone()}
  {
    // do nothing
  }

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::IFF_GOAL_QUERY; }
  const std::unique_ptr<GoalQuery<T,V>>& getGoalQueryPtr1() const { return goal_query_ptr_1; }
  const std::unique_ptr<GoalQuery<T,V>>& getGoalQueryPtr2() const { return goal_query_ptr_2; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
    goal_query_ptr_1->setNodeIdByDFS(goal_query_nodes);
    goal_query_ptr_2->setNodeIdByDFS(goal_query_nodes);
  }

  void assignConstantId(NameIdMap& constantIdMap) final { goal_query_ptr_1->assignConstantId(constantIdMap); goal_query_ptr_2->assignConstantId(constantIdMap); }
  void assignVariableId(NameIdMap& variableIdMap) final { goal_query_ptr_1->assignVariableId(variableIdMap); goal_query_ptr_2->assignVariableId(variableIdMap); }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {
    goal_query_ptr_1->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
    goal_query_ptr_2->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
  }

//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {
//    goal_query_ptr_1->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//    goal_query_ptr_2->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//  }

  void buildGqsGraph(GqsGraph& gqs_graph) const final {
    goal_query_ptr_1->buildGqsGraph(gqs_graph);
    goal_query_ptr_2->buildGqsGraph(gqs_graph);
    gqs_graph.setNotState(goal_query_ptr_1->getId());
    gqs_graph.setNotState(goal_query_ptr_2->getId());
  }

  IffStatement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    return new IffStatement<T,V>(0, std::move(goal_query_ptr_1->translate(translate_info)), std::move(goal_query_ptr_2->translate(translate_info)));
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    return GoalQuery<T,V>::to_string(context) + "(" + goal_query_ptr_1->to_string(context) + " <=> " + goal_query_ptr_2->to_string(context) + ")";
  }

private:

  IffGoalQuery<T,V>* doClone() const final { return new IffGoalQuery<T,V>(*this); }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class EqualGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_1;
  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr_2;

public:

  EqualGoalQuery(std::unique_ptr<GoalQuery<T,V>> _goal_query_ptr_1, std::unique_ptr<GoalQuery<T,V>> _goal_query_ptr_2) :
      goal_query_ptr_1{std::move(_goal_query_ptr_1)}, goal_query_ptr_2{std::move(_goal_query_ptr_2)}
  {
    assert(goal_query_ptr_1->getType() == GOAL_QUERY_TYPE::CONSTANT_GOAL_QUERY || goal_query_ptr_1->getType() == GOAL_QUERY_TYPE::VARIABLE_GOAL_QUERY);
    assert(goal_query_ptr_2->getType() == GOAL_QUERY_TYPE::CONSTANT_GOAL_QUERY || goal_query_ptr_2->getType() == GOAL_QUERY_TYPE::VARIABLE_GOAL_QUERY);
  }

  EqualGoalQuery(const EqualGoalQuery& other) :
      GoalQuery<T,V>{other}, goal_query_ptr_1{other.goal_query_ptr_1->clone()}, goal_query_ptr_2{other.goal_query_ptr_2->clone()}
  {
    // do nothing
  }

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::EQUAL_GOAL_QUERY; }
  const std::unique_ptr<GoalQuery<T,V>>& getGoalQueryPtr1() const { return goal_query_ptr_1; }
  const std::unique_ptr<GoalQuery<T,V>>& getGoalQueryPtr2() const { return goal_query_ptr_2; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
    goal_query_ptr_1->setNodeIdByDFS(goal_query_nodes);
    goal_query_ptr_2->setNodeIdByDFS(goal_query_nodes);
  }

  void assignConstantId(NameIdMap& constantIdMap) final { goal_query_ptr_1->assignConstantId(constantIdMap); goal_query_ptr_2->assignConstantId(constantIdMap); }
  void assignVariableId(NameIdMap& variableIdMap) final { goal_query_ptr_1->assignVariableId(variableIdMap); goal_query_ptr_2->assignVariableId(variableIdMap); }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {
    goal_query_ptr_1->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
    goal_query_ptr_2->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
  }

//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {
//    goal_query_ptr_1->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//    goal_query_ptr_2->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//  }

  void buildGqsGraph(GqsGraph& gqs_graph) const final {
    goal_query_ptr_1->buildGqsGraph(gqs_graph);
    goal_query_ptr_2->buildGqsGraph(gqs_graph);
    gqs_graph.setNotState(goal_query_ptr_1->getId());
    gqs_graph.setNotState(goal_query_ptr_2->getId());
  }

  Equal<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    return new Equal<T,V>(0, std::move(unique_cast<Term<T,V>,Statement<T,V>>(goal_query_ptr_1->translate(translate_info))), std::move(unique_cast<Term<T,V>,Statement<T,V>>(goal_query_ptr_2->translate(translate_info))));
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    return GoalQuery<T,V>::to_string(context) + "(" + goal_query_ptr_1->to_string(context) + " == " + goal_query_ptr_2->to_string(context) + ")";
  }

private:

  EqualGoalQuery<T,V>* doClone() const final { return new EqualGoalQuery<T,V>(*this); }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class ForAllGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  std::unique_ptr<VariableGoalQuery<T,V>> variable;
  std::vector<std::unique_ptr<VariableGoalQuery<T,V>>> excluded_variables;
  std::vector<std::unique_ptr<ConstantGoalQuery<T,V>>> excluded_constants;
  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr;

public:

  ForAllGoalQuery(std::unique_ptr<VariableGoalQuery<T,V>> _variable,
                  std::vector<std::unique_ptr<VariableGoalQuery<FPST,FPSV>>>& _excluded_variables,
                  std::vector<std::unique_ptr<ConstantGoalQuery<FPST,FPSV>>>& _excluded_constants,
                  std::unique_ptr<GoalQuery<T,V>> _goal_query_ptr) :
      variable{std::move(_variable)}, goal_query_ptr{std::move(_goal_query_ptr)}
  {
    for(auto& ev : _excluded_variables) {
      excluded_variables.push_back(std::move(ev));
    }
    for(auto& ec : _excluded_constants) {
      excluded_constants.push_back(std::move(ec));
    }
  }

  ForAllGoalQuery(const ForAllGoalQuery& other) :
      GoalQuery<T,V>{other}, variable{unique_cast<VariableGoalQuery<T,V>,GoalQuery<T,V>>(other.variable->clone())}, goal_query_ptr{other.goal_query_ptr->clone()}
  {
    for(auto& ev : other.excluded_variables) {
      excluded_variables.push_back(unique_cast<VariableGoalQuery<T,V>,GoalQuery<T,V>>(ev->clone()));
    }
    for(auto& ec : other.excluded_constants) {
      excluded_constants.push_back(unique_cast<ConstantGoalQuery<T,V>,GoalQuery<T,V>>(ec->clone()));
    }
  }

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::FORALL_GOAL_QUERY; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
    variable->setNodeIdByDFS(goal_query_nodes);
    for(auto& ev : excluded_variables) {
      ev->setNodeIdByDFS(goal_query_nodes);
    }
    for(auto& ec : excluded_constants) {
      ec->setNodeIdByDFS(goal_query_nodes);
    }
    goal_query_ptr->setNodeIdByDFS(goal_query_nodes);
  }

  void assignConstantId(NameIdMap& constantIdMap) final {
    for(auto& ec : excluded_constants) {
      ec->assignConstantId(constantIdMap);
    }
    goal_query_ptr->assignConstantId(constantIdMap);
  }

  void assignVariableId(NameIdMap& variableIdMap) final {
    variable->assignVariableId(variableIdMap);
    for(auto& ev : excluded_variables) {
      ev->assignVariableId(variableIdMap);
    }
    goal_query_ptr->assignVariableId(variableIdMap);
  }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {
    for(auto& ev : excluded_variables) {
      assert(ev->getVariableId() != variable->getVariableId());
      ev->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
    }
    goal_query_ptr->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
    // set variableId2Prefix[variable->getVariableId()] to empty causes it to be ignored in the strong/weak recognizable variables
    variableId2Prefix[variable->getVariableId()] = {};
  }

//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {
//    goal_query_ptr->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//  }

  void buildGqsGraph(GqsGraph& gqs_graph) const final {
    variable->buildGqsGraph(gqs_graph);
    gqs_graph.setNotState(variable->getId());
    for(auto& ev : excluded_variables) {
      ev->buildGqsGraph(gqs_graph);
      gqs_graph.setNotState(ev->getId());
    }
    for(auto& ec : excluded_constants) {
      ec->buildGqsGraph(gqs_graph);
      gqs_graph.setNotState(ec->getId());
    }
    goal_query_ptr->buildGqsGraph(gqs_graph);
    gqs_graph.setNotState(goal_query_ptr->getId());
  }

  ForAllStatement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    std::vector<std::pair<int,std::string>> excluded_variable_idnames = calcExcludedVariableIdnames();
    std::vector<std::pair<int,std::string>> excluded_constant_idnames = calcExcludedConstantIdnames();

    return new ForAllStatement<T,V>(0,
                                    unique_cast<Variable<T,V>,Statement<T,V>>(std::move(variable->translate(translate_info))),
                                    excluded_variable_idnames,
                                    excluded_constant_idnames,
                                    std::move(goal_query_ptr->translate(translate_info)));
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    std::string s = GoalQuery<T,V>::to_string(context) + "(forall " + variable->to_string(context);
    s += calcExcludedCVDescription<T,V>(" not_in ", context, excluded_variables, excluded_constants) + " ";
    s += "[" + goal_query_ptr->to_string(context) + "])";
    return s;
  }

private:

  ForAllGoalQuery<T,V>* doClone() const final { return new ForAllGoalQuery<T,V>(*this); }

  std::vector<std::pair<int,std::string>> calcExcludedVariableIdnames() const {
    std::vector<std::pair<int,std::string>> excluded_variable_idnames;
    for(auto& ev : excluded_variables) {
      excluded_variable_idnames.emplace_back(ev->getVariableId(), ev->getVariableName());
    }
    return excluded_variable_idnames;
  }

  std::vector<std::pair<int,std::string>> calcExcludedConstantIdnames() const {
    std::vector<std::pair<int,std::string>> excluded_constant_idnames;
    for(auto& ec : excluded_constants) {
      excluded_constant_idnames.emplace_back(ec->getConstantId(), ec->getConstantName());
    }
    return excluded_constant_idnames;
  }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class ExistGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  std::unique_ptr<VariableGoalQuery<T,V>> variable;
  std::vector<std::unique_ptr<VariableGoalQuery<T,V>>> excluded_variables;
  std::vector<std::unique_ptr<ConstantGoalQuery<T,V>>> excluded_constants;
  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr;

public:

  ExistGoalQuery(std::unique_ptr<VariableGoalQuery<T,V>> _variable,
                 std::vector<std::unique_ptr<VariableGoalQuery<FPST,FPSV>>>& _excluded_variables,
                 std::vector<std::unique_ptr<ConstantGoalQuery<FPST,FPSV>>>& _excluded_constants,
                 std::unique_ptr<GoalQuery<T,V>> _goal_query_ptr) :
      variable{std::move(_variable)}, goal_query_ptr{std::move(_goal_query_ptr)}
  {
    for(auto& ev : _excluded_variables) {
      excluded_variables.push_back(std::move(ev));
    }
    for(auto& ec : _excluded_constants) {
      excluded_constants.push_back(std::move(ec));
    }
  }

  ExistGoalQuery(const ExistGoalQuery& other) :
      GoalQuery<T,V>{other}, variable{unique_cast<VariableGoalQuery<T,V>,GoalQuery<T,V>>(other.variable->clone())}, goal_query_ptr{other.goal_query_ptr->clone()}
  {
    for(auto& ev : other.excluded_variables) {
      excluded_variables.push_back(unique_cast<VariableGoalQuery<T,V>,GoalQuery<T,V>>(ev->clone()));
    }
    for(auto& ec : other.excluded_constants) {
      excluded_constants.push_back(unique_cast<ConstantGoalQuery<T,V>,GoalQuery<T,V>>(ec->clone()));
    }
  }

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::EXIST_GOAL_QUERY; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
    variable->setNodeIdByDFS(goal_query_nodes);
    for(auto& ev : excluded_variables) {
      ev->setNodeIdByDFS(goal_query_nodes);
    }
    for(auto& ec : excluded_constants) {
      ec->setNodeIdByDFS(goal_query_nodes);
    }
    goal_query_ptr->setNodeIdByDFS(goal_query_nodes);
  }

  void assignConstantId(NameIdMap& constantIdMap) final {
    for(auto& ec : excluded_constants) {
      ec->assignConstantId(constantIdMap);
    }
    goal_query_ptr->assignConstantId(constantIdMap);
  }

  void assignVariableId(NameIdMap& variableIdMap) final {
    variable->assignVariableId(variableIdMap);
    for(auto& ev : excluded_variables) {
      ev->assignVariableId(variableIdMap);
    }
    goal_query_ptr->assignVariableId(variableIdMap);
  }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {
    for(auto& ev : excluded_variables) {
      assert(ev->getVariableId() != variable->getVariableId());
      ev->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
    }
    goal_query_ptr->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
    // set variableId2Prefix[variable->getVariableId()] to empty causes it to be ignored in the strong/weak recognizable variables
    variableId2Prefix[variable->getVariableId()] = {};
  }

//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {
//    goal_query_ptr->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//  }

  void buildGqsGraph(GqsGraph& gqs_graph) const final {
    variable->buildGqsGraph(gqs_graph);
    gqs_graph.setNotState(variable->getId());
    for(auto& ev : excluded_variables) {
      ev->buildGqsGraph(gqs_graph);
      gqs_graph.setNotState(ev->getId());
    }
    for(auto& ec : excluded_constants) {
      ec->buildGqsGraph(gqs_graph);
      gqs_graph.setNotState(ec->getId());
    }
    goal_query_ptr->buildGqsGraph(gqs_graph);
    gqs_graph.setNotState(goal_query_ptr->getId());
  }

  ExistStatement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    std::vector<std::pair<int,std::string>> excluded_variable_idnames = calcExcludedVariableIdnames();
    std::vector<std::pair<int,std::string>> excluded_constant_idnames = calcExcludedConstantIdnames();

    return new ExistStatement<T,V>(0,
                                   unique_cast<Variable<T,V>,Statement<T,V>>(std::move(variable->translate(translate_info))),
                                   excluded_variable_idnames,
                                   excluded_constant_idnames,
                                   std::move(goal_query_ptr->translate(translate_info)));
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    std::string s = GoalQuery<T,V>::to_string(context) + "(exist " + variable->to_string(context);
    s += calcExcludedCVDescription<T,V>(" not_in ", context, excluded_variables, excluded_constants) + " ";
    s += "[" + goal_query_ptr->to_string(context) + "])";
    return s;
  }

private:

  ExistGoalQuery<T,V>* doClone() const final { return new ExistGoalQuery<T,V>(*this); }

  std::vector<std::pair<int,std::string>> calcExcludedVariableIdnames() const {
    std::vector<std::pair<int,std::string>> excluded_variable_idnames;
    for(auto& ev : excluded_variables) {
      excluded_variable_idnames.emplace_back(ev->getVariableId(), ev->getVariableName());
    }
    return excluded_variable_idnames;
  }

  std::vector<std::pair<int,std::string>> calcExcludedConstantIdnames() const {
    std::vector<std::pair<int,std::string>> excluded_constant_idnames;
    for(auto& ec : excluded_constants) {
      excluded_constant_idnames.emplace_back(ec->getConstantId(), ec->getConstantName());
    }
    return excluded_constant_idnames;
  }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class ExcludeGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  std::vector<std::unique_ptr<VariableGoalQuery<FPST,FPSV>>> excluded_variables;
  std::vector<std::unique_ptr<ConstantGoalQuery<FPST,FPSV>>> excluded_constants;

public:

  ExcludeGoalQuery() {}

  ExcludeGoalQuery(std::vector<std::unique_ptr<VariableGoalQuery<FPST,FPSV>>>& _excluded_variables,
                   std::vector<std::unique_ptr<ConstantGoalQuery<FPST,FPSV>>>& _excluded_constants)
  {
    for(auto& ev : _excluded_variables) {
      excluded_variables.push_back(std::move(ev));
    }
    for(auto& ec : _excluded_constants) {
      excluded_constants.push_back(std::move(ec));
    }
  }

  ExcludeGoalQuery(const ExcludeGoalQuery& other) :
      GoalQuery<T,V>{other}
  {
    for(auto& ev : other.excluded_variables) {
      excluded_variables.push_back(unique_cast<VariableGoalQuery<T,V>,GoalQuery<T,V>>(ev->clone()));
    }
    for(auto& ec : other.excluded_constants) {
      excluded_constants.push_back(unique_cast<ConstantGoalQuery<T,V>,GoalQuery<T,V>>(ec->clone()));
    }
  }

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::EXCLUDE_GOAL_QUERY; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
    for(auto& ev : excluded_variables) {
      ev->setNodeIdByDFS(goal_query_nodes);
    }
    for(auto& ec : excluded_constants) {
      ec->setNodeIdByDFS(goal_query_nodes);
    }
  }

  void assignConstantId(NameIdMap& constantIdMap) final {
    for(auto& ec : excluded_constants) {
      ec->assignConstantId(constantIdMap);
    }
  }

  void assignVariableId(NameIdMap& variableIdMap) final {
    for(auto& ev : excluded_variables) {
      ev->assignVariableId(variableIdMap);
    }
  }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {
    for(auto& ev : excluded_variables) {
      ev->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
    }
  }

//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {}

  void buildGqsGraph(GqsGraph& gqs_graph) const final {
    for(auto& ev : excluded_variables) {
      ev->buildGqsGraph(gqs_graph);
      gqs_graph.setNotState(ev->getId());
    }
    for(auto& ec : excluded_constants) {
      ec->buildGqsGraph(gqs_graph);
      gqs_graph.setNotState(ec->getId());
    }
  }

  ForAllStatement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    std::vector<std::string> excluded_variable_names;
    std::vector<std::string> excluded_constant_names;

    for(auto& ev : excluded_variables) {
      excluded_variable_names.push_back(ev->getVariableName());
    }
    for(auto& ec : excluded_constants) {
      excluded_constant_names.push_back(ec->getConstantName());
    }

    return makeExclusiveStatement<T,V>(translate_info.variableIdMap, translate_info.constantIdMap, excluded_variable_names, excluded_constant_names).release();
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    return GoalQuery<T,V>::to_string(context) + "XA" + calcExcludedCVDescription<T,V>("_", context, excluded_variables, excluded_constants);
  }

private:

  ExcludeGoalQuery<T,V>* doClone() const final { return new ExcludeGoalQuery<T,V>(*this); }

};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class EdgeGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  bool is_all;
  bool is_path;
  std::unique_ptr<GoalQuery<T,V>> state_goal_query_ptr;
  std::unique_ptr<GoalQuery<T,V>> cond_goal_query_ptr;   // it can be nullptr

public:

  EdgeGoalQuery(bool is_all, bool is_path, std::unique_ptr<GoalQuery<T,V>> goal_query_ptr) :
      is_all{is_all}, is_path{is_path}, state_goal_query_ptr{std::move(goal_query_ptr)}
  {
    // do nothing
  }

  EdgeGoalQuery(bool is_all, bool is_path, std::unique_ptr<GoalQuery<T,V>> goal_query_ptr, std::unique_ptr<GoalQuery<T,V>> cond_goal_query_ptr) :
      is_all{is_all}, is_path{is_path}, state_goal_query_ptr{std::move(goal_query_ptr)}, cond_goal_query_ptr{std::move(cond_goal_query_ptr)}
  {
    // do nothing
  }

  EdgeGoalQuery(const EdgeGoalQuery& other) :
      GoalQuery<T,V>{other}, is_all{other.is_all}, is_path{other.is_path}, state_goal_query_ptr{other.state_goal_query_ptr->clone()}, cond_goal_query_ptr{(other.cond_goal_query_ptr) ? (other.cond_goal_query_ptr->clone()) : nullptr}
  {
    // do nothing
  }

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::EDGE_GOAL_QUERY; }

  bool isAll() const { return is_all; }
  bool isPath() const { return is_path; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
    if (cond_goal_query_ptr) cond_goal_query_ptr->setNodeIdByDFS(goal_query_nodes);
    state_goal_query_ptr->setNodeIdByDFS(goal_query_nodes);
  }

  void assignConstantId(NameIdMap& constantIdMap) final {
    if (cond_goal_query_ptr) cond_goal_query_ptr->assignConstantId(constantIdMap);
    state_goal_query_ptr->assignConstantId(constantIdMap);
  }

  void assignVariableId(NameIdMap& variableIdMap) final {
    if (cond_goal_query_ptr) cond_goal_query_ptr->assignVariableId(variableIdMap);
    state_goal_query_ptr->assignVariableId(variableIdMap);
  }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {
    current_prefix.push_back(id);
    if (cond_goal_query_ptr) cond_goal_query_ptr->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
    state_goal_query_ptr->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
  }

//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {
//    std::vector<int> all_vid_in_prefix;
//    for(auto node_id : node_id_prefix) {
//      if (nodeId2VariableIds.contains(node_id)) {
//        for(auto vid : nodeId2VariableIds.at(node_id)) {
//          all_vid_in_prefix.push_back(vid);
//        }
//      }
//    }
//    if (nodeId2VariableIds.contains(id)) {
//      for(auto vid : nodeId2VariableIds.at(id)) {
//        assert(dependedVariablesOfEdge[vid].empty());
//        dependedVariablesOfEdge[vid] = all_vid_in_prefix;
//      }
//    }
//
//    node_id_prefix.push_back(id);
//    if (cond_goal_query_ptr) cond_goal_query_ptr->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//    state_goal_query_ptr->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//  }

  bool isEnd() const {
    return (state_goal_query_ptr->getType() == GOAL_QUERY_TYPE::END_GOAL_QUERY);
  }

  bool isPrunable() const {
    return isEnd() && (!cond_goal_query_ptr);
  }

  void buildGqsGraph(GqsGraph& gqs_graph) const final {
    if (cond_goal_query_ptr) {
      cond_goal_query_ptr->buildGqsGraph(gqs_graph);
      gqs_graph.setNotState(cond_goal_query_ptr->getId());
      gqs_graph.setEdgeCond(cond_goal_query_ptr->getId(), id);
    }
    buildGqsGraph_impl(gqs_graph, state_goal_query_ptr);
  }

  void buildGqsGraph_impl(GqsGraph& gqs_graph, const std::unique_ptr<GoalQuery<T,V>>& gq_ptr) const {
    if (gq_ptr->getType() == GOAL_QUERY_TYPE::OR_GOAL_QUERY) {
      gqs_graph.setNotState(gq_ptr->getId());
      auto or_ptr = dynamic_cast<OrGoalQuery<T,V>*>(gq_ptr.get());
      buildGqsGraph_impl(gqs_graph, or_ptr->getGoalQueryPtr1());
      buildGqsGraph_impl(gqs_graph, or_ptr->getGoalQueryPtr2());
    } else {
      if (gq_ptr->getType() == GOAL_QUERY_TYPE::AND_GOAL_QUERY) {
        auto and_ptr = dynamic_cast<AndGoalQuery<T,V>*>(gq_ptr.get());
        if (and_ptr->getIsBetweenStateAndEdge()) {
          gqs_graph.setEdgeNextState(id, and_ptr->getGoalQueryPtr1()->getId());
        } else {
          gqs_graph.setEdgeNextState(id, gq_ptr->getId());
        }
      } else {
        gqs_graph.setEdgeNextState(id, gq_ptr->getId());
      }
      gq_ptr->buildGqsGraph(gqs_graph);
    }
  }

  Statement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    if (isPrunable()) return new TrueStatement<T,V>(0);
    if (is_path) {
      if (!isEnd()) {
        if (is_all) {
          return new AllNextStatement<T,V>(0, std::unique_ptr<Statement<T,V>>(doTranslateWithoutNext(translate_info)));
        } else {
          return new SomeNextStatement<T,V>(0, std::unique_ptr<Statement<T,V>>(doTranslateWithoutNext(translate_info)));
        }
      } else {
        if (is_all) {
          return new OrStatement<T,V>(0, std::make_unique<LastStatement<T,V>>(0), std::make_unique<AllNextStatement<T,V>>(0, std::unique_ptr<Statement<T,V>>(doTranslateWithoutNext(translate_info))));
        } else {
          return new OrStatement<T,V>(0, std::make_unique<LastStatement<T,V>>(0), std::make_unique<SomeNextStatement<T,V>>(0, std::unique_ptr<Statement<T,V>>(doTranslateWithoutNext(translate_info))));
        }
      }
    } else {
      return doTranslateWithoutNext(translate_info);
    }
  }

  Statement<T,V>* doTranslateWithoutNext(TranslateInfo& translate_info) const {
    if (isPrunable()) return new TrueStatement<T,V>(0);   // ignore is_path

    // identify weakly recognizable variable
    std::vector<std::string> weakly_recognizable_vnames;
    std::vector<int> weakly_recognizable_vids;

    if (translate_info.commonParentNodeId2VariableIds.contains(id)) {
      for(auto variable_id : translate_info.commonParentNodeId2VariableIds.at(id)) {
        assert(translate_info.variableId2CommonParentNodeId[variable_id] == id);
        auto variable_name = translate_info.variableIdMap.getNameById(variable_id);
        if (!translate_info.isStronglyRecognizable(variable_name)) {
          weakly_recognizable_vnames.push_back(variable_name);
          weakly_recognizable_vids.push_back(variable_id);
        }
      }
    }

    return doTranslateWithoutNext_impl(translate_info, weakly_recognizable_vnames, weakly_recognizable_vids);
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    std::string s = GoalQuery<T,V>::to_string(context) + ((is_all) ? "A" : "E") + ((is_path) ? "P" : "X");
    if (cond_goal_query_ptr) s += "_{" + cond_goal_query_ptr->to_string(context) + "}";
    s += " " + state_goal_query_ptr->to_string(context);
    return s;
  }

private:

  EdgeGoalQuery<T,V>* doClone() const final { return new EdgeGoalQuery<T,V>(*this); }

  Statement<T,V>* doTranslateWithoutNext_impl(TranslateInfo& translate_info, const std::vector<std::string>& weakly_recognizable_vnames, const std::vector<int>& weakly_recognizable_vids) const {
    // generate the statement
    if (state_goal_query_ptr->getType() != GOAL_QUERY_TYPE::END_GOAL_QUERY) {
      if (is_all) {
        if (!is_path) {
          assert(!cond_goal_query_ptr);
          std::unique_ptr<Statement<T,V>> statement2{appendRecognizableVariables(translate_info, std::move(state_goal_query_ptr->translate(translate_info)), weakly_recognizable_vnames, weakly_recognizable_vids)};
          return new AllNextStatement<T,V>(0, std::move(statement2));
        } else if (!cond_goal_query_ptr) {
          std::unique_ptr<Statement<T,V>> statement2{appendRecognizableVariables(translate_info, std::move(state_goal_query_ptr->translate(translate_info)), weakly_recognizable_vnames, weakly_recognizable_vids)};
          return new AllFinallyStatement<T,V>(0, std::move(statement2));
        } else {
          // *** just a hack. need a better solution ***
          auto cond_statement = cond_goal_query_ptr->translate(translate_info);
          cond_statement->calcBoundFreeVariableIDs({});
          bool isFound = false;
          for(auto weak_vid : weakly_recognizable_vids) {
            if (cond_statement->getFreeVariableIDs().find(weak_vid) != cond_statement->getFreeVariableIDs().end()) {
              isFound = true;
              break;
            }
          }
          if (isFound) {
            // need to separate weakly_recognizable_vids into two groups: The vids in cond_statement->getFreeVariableIDs(), and the rest
            std::vector<std::string> weakly_recognizable_vnames_free;
            std::vector<int> weakly_recognizable_vids_free;
            std::vector<std::string> weakly_recognizable_vnames_other;
            std::vector<int> weakly_recognizable_vids_other;

            for(auto weak_vid : weakly_recognizable_vids) {
              if (cond_statement->getFreeVariableIDs().find(weak_vid) != cond_statement->getFreeVariableIDs().end()) {
                weakly_recognizable_vids_free.push_back(weak_vid);
                weakly_recognizable_vnames_free.push_back(translate_info.variableIdMap.getNameById(weak_vid));
              } else {
                weakly_recognizable_vids_other.push_back(weak_vid);
                weakly_recognizable_vnames_other.push_back(translate_info.variableIdMap.getNameById(weak_vid));
              }
            }

            std::unique_ptr<Statement<T,V>> state_statement{appendRecognizableVariables(translate_info, std::move(state_goal_query_ptr->translate(translate_info)), weakly_recognizable_vnames_other, weakly_recognizable_vids_other)};
            std::unique_ptr<Statement<T,V>> cond_statement{new AllUntilStatement<T,V>(0, std::move(cond_goal_query_ptr->translate(translate_info)), std::move(state_statement))};
            return appendRecognizableVariables(translate_info, std::move(cond_statement), weakly_recognizable_vnames_free, weakly_recognizable_vids_free);
          } else {
            // alright. all weakly_recognizable_vids are really weak
            std::unique_ptr<Statement<T,V>> statement2{appendRecognizableVariables(translate_info, std::move(state_goal_query_ptr->translate(translate_info)), weakly_recognizable_vnames, weakly_recognizable_vids)};
            return new AllUntilStatement<T,V>(0, std::move(cond_goal_query_ptr->translate(translate_info)), std::move(statement2));
          }
        }
      } else {
        if (!is_path) {
          assert(!cond_goal_query_ptr);
          std::unique_ptr<Statement<T,V>> statement2{appendRecognizableVariables(translate_info, std::move(state_goal_query_ptr->translate(translate_info)), weakly_recognizable_vnames, weakly_recognizable_vids)};
          return new SomeNextStatement<T,V>(0, std::move(statement2));
        } else if (!cond_goal_query_ptr) {
          std::unique_ptr<Statement<T,V>> statement2{appendRecognizableVariables(translate_info, std::move(state_goal_query_ptr->translate(translate_info)), weakly_recognizable_vnames, weakly_recognizable_vids)};
          return new SomeFinallyStatement<T,V>(0, std::move(statement2));
        } else {
          // *** just a hack. need a better solution ***
          auto cond_statement = cond_goal_query_ptr->translate(translate_info);
          cond_statement->calcBoundFreeVariableIDs({});
          bool isFound = false;
          for(auto weak_vid : weakly_recognizable_vids) {
            if (cond_statement->getFreeVariableIDs().find(weak_vid) != cond_statement->getFreeVariableIDs().end()) {
              isFound = true;
              break;
            }
          }
          if (isFound) {
            // need to separate weakly_recognizable_vids into two groups: The vids in cond_statement->getFreeVariableIDs(), and the rest
            std::vector<std::string> weakly_recognizable_vnames_free;
            std::vector<int> weakly_recognizable_vids_free;
            std::vector<std::string> weakly_recognizable_vnames_other;
            std::vector<int> weakly_recognizable_vids_other;

            for(auto weak_vid : weakly_recognizable_vids) {
              if (cond_statement->getFreeVariableIDs().find(weak_vid) != cond_statement->getFreeVariableIDs().end()) {
                weakly_recognizable_vids_free.push_back(weak_vid);
                weakly_recognizable_vnames_free.push_back(translate_info.variableIdMap.getNameById(weak_vid));
              } else {
                weakly_recognizable_vids_other.push_back(weak_vid);
                weakly_recognizable_vnames_other.push_back(translate_info.variableIdMap.getNameById(weak_vid));
              }
            }

            std::unique_ptr<Statement<T,V>> state_statement{appendRecognizableVariables(translate_info, std::move(state_goal_query_ptr->translate(translate_info)), weakly_recognizable_vnames_other, weakly_recognizable_vids_other)};
            std::unique_ptr<Statement<T,V>> cond_statement{new SomeUntilStatement<T,V>(0, std::move(cond_goal_query_ptr->translate(translate_info)), std::move(state_statement))};
            return appendRecognizableVariables(translate_info, std::move(cond_statement), weakly_recognizable_vnames_free, weakly_recognizable_vids_free);
          } else {
            // alright. all weakly_recognizable_vids are really weak
            std::unique_ptr<Statement<T,V>> statement2{appendRecognizableVariables(translate_info, std::move(state_goal_query_ptr->translate(translate_info)), weakly_recognizable_vnames, weakly_recognizable_vids)};
            return new SomeUntilStatement<T,V>(0, std::move(cond_goal_query_ptr->translate(translate_info)), std::move(statement2));
          }
        }
      }
    } else {  // state_goal_query_ptr->getType() == GOAL_QUERY_TYPE::END_GOAL_QUERY   =>   cond_goal_query_ptr != nullptr && is_path is true
      assert(cond_goal_query_ptr);
      if (is_all) {
        auto statement_ptr = new AllGloballyStatement<T,V>(0, std::move(cond_goal_query_ptr->translate(translate_info)));
        return appendRecognizableVariables(translate_info, std::unique_ptr<Statement<T,V>>(statement_ptr), weakly_recognizable_vnames, weakly_recognizable_vids);
      } else {
        auto statement_ptr = new SomeGloballyStatement<T,V>(0, std::move(cond_goal_query_ptr->translate(translate_info)));
        return appendRecognizableVariables(translate_info, std::unique_ptr<Statement<T,V>>(statement_ptr), weakly_recognizable_vnames, weakly_recognizable_vids);
      }
    }
  }


};

// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class NilGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

  std::unique_ptr<EdgeGoalQuery<T,V>> edge_goal_query_ptr;  // it should never be nullptr

public:

  NilGoalQuery(std::unique_ptr<EdgeGoalQuery<T,V>> edge_goal_query_ptr) :
      edge_goal_query_ptr{std::move(edge_goal_query_ptr)}
  {
    // do nothing
  }

  NilGoalQuery(const NilGoalQuery& other) :
      GoalQuery<T,V>{other}, edge_goal_query_ptr(unique_cast<EdgeGoalQuery<T,V>,GoalQuery<T,V>>(other.edge_goal_query_ptr->clone()))
  {
    // do nothing
  }

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::NIL_GOAL_QUERY; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
    edge_goal_query_ptr->setNodeIdByDFS(goal_query_nodes);
  }

  void assignConstantId(NameIdMap& constantIdMap) final { edge_goal_query_ptr->assignConstantId(constantIdMap); }
  void assignVariableId(NameIdMap& variableIdMap) final { edge_goal_query_ptr->assignVariableId(variableIdMap); }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {
    edge_goal_query_ptr->calcCommonNodeIdPrefix(variableId2Prefix, current_prefix);
  }

//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {
//    edge_goal_query_ptr->calcDependedVariablesOfEdge(dependedVariablesOfEdge, nodeId2VariableIds, node_id_prefix);
//  }

  void buildGqsGraph(GqsGraph& gqs_graph) const final {
    edge_goal_query_ptr->buildGqsGraph(gqs_graph);  // this is a state
    gqs_graph.setNotState(edge_goal_query_ptr->getId());
    gqs_graph.setEdge(edge_goal_query_ptr->getId());
    gqs_graph.setStateNextEdge(id, edge_goal_query_ptr->getId());
  }

  Statement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    return edge_goal_query_ptr->doTranslateWithoutNext(translate_info);
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    return GoalQuery<T,V>::to_string(context) + "Nil " + edge_goal_query_ptr->to_string(context);
  }

private:

  NilGoalQuery<T,V>* doClone() const final { return new NilGoalQuery<T,V>(*this); }

};


// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class EndGoalQuery : public GoalQuery<T,V> {
  using GoalQuery<T,V>::id;

public:

  EndGoalQuery() {}
  EndGoalQuery(const EndGoalQuery& other) : GoalQuery<T,V>{other} {}

  GOAL_QUERY_TYPE getType() const final { return GOAL_QUERY_TYPE::END_GOAL_QUERY; }

  void setNodeIdByDFS(std::vector<GoalQuery<T,V>*>& goal_query_nodes) final {
    id = goal_query_nodes.size(); goal_query_nodes.push_back(this);
  }

  void assignConstantId(NameIdMap& constantIdMap) final { }
  void assignVariableId(NameIdMap& variableIdMap) final { }

  void calcCommonNodeIdPrefix(std::vector<std::vector<int>>& variableId2Prefix, std::vector<int> current_prefix) const final {}
//  void calcDependedVariablesOfEdge(std::vector<std::vector<int>>& dependedVariablesOfEdge, const std::unordered_map<int,std::vector<int>>& nodeId2VariableIds, std::vector<int> node_id_prefix) const final {}

  void buildGqsGraph(GqsGraph& gqs_graph) const final {}

  Statement<T,V>* doTranslate(TranslateInfo& translate_info) const final {
    return new TrueStatement<T,V>(0);
  }

  std::string to_string(typename GoalQuery<T,V>::ToStringInfo& context) const final {
    return GoalQuery<T,V>::to_string(context) + "End";
  }

private:

  EndGoalQuery<T,V>* doClone() const final { return new EndGoalQuery<T,V>(*this); }

};



// -----------------------------------------------------------------------------------------
// Goal Query Statement
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class GoalQueryStatement {

  std::unique_ptr<GoalQuery<T,V>> goal_query_ptr;
  std::set<std::string> strongly_recognizable_set;
  std::set<std::string> weakly_recognizable_set;
  std::vector<std::pair<std::string,std::string>> unequal_CVs;

  NameIdMap constantIdMap;
  NameIdMap variableIdMap;
  std::vector<GoalQuery<T,V>*> goal_query_nodes;
  std::vector<int> variableId2CommonParentNodeId;
  std::unordered_map<int,std::vector<int>> commonParentNodeId2VariableIds;
//  std::vector<std::vector<int>> dependedVariablesOfEdge;
  std::vector<std::vector<bool>> unequal_variables_matrix;
  std::vector<std::set<int>> unequal_constants;

public:

  GoalQueryStatement() {}

  GoalQueryStatement(std::unique_ptr<GoalQuery<T,V>> _goal_query_ptr,
                     const std::set<std::string> strongly_recognizable_set = {},
                     const std::set<std::string> weakly_recognizable_set = {},
                     const std::vector<std::pair<std::string,std::string>> unequal_CVs = {}) :
     goal_query_ptr{std::move(_goal_query_ptr)}, strongly_recognizable_set{strongly_recognizable_set}, weakly_recognizable_set{weakly_recognizable_set}, unequal_CVs{unequal_CVs}
  {  // don't know why I need to use _goal_query_ptr instead of state_goal_query_ptr
    calcMemberData();
  }

  GoalQueryStatement(const GoalQueryStatement& other) :
      goal_query_ptr{other.goal_query_ptr->clone()}, strongly_recognizable_set{other.strongly_recognizable_set}, weakly_recognizable_set{other.weakly_recognizable_set}, unequal_CVs{other.unequal_CVs}
  {
    calcMemberData();
  }

  GoalQueryStatement& operator=(const GoalQueryStatement& other) {
    goal_query_ptr = other.goal_query_ptr->clone();
    strongly_recognizable_set = other.strongly_recognizable_set;
    weakly_recognizable_set = other.weakly_recognizable_set;
    unequal_CVs = other.unequal_CVs;
    calcMemberData();
    return *this;
  }

  const std::unique_ptr<GoalQuery<T,V>>& getGoalQueryPtr() const { return goal_query_ptr; }
  const std::set<std::string>& getStronglyRecognizableSet() const { return strongly_recognizable_set; }
  const std::vector<std::pair<std::string,std::string>>& getUnequalCVs() const { return unequal_CVs; }

  StatementRecord<T,V> makeStatementRecord() {
    TranslateInfo translate_info{strongly_recognizable_set, constantIdMap, variableIdMap,
                                 variableId2CommonParentNodeId, commonParentNodeId2VariableIds,
                                 unequal_variables_matrix, unequal_constants };
//    TranslateInfo translate_info{strongly_recognizable_set, constantIdMap, variableIdMap,
//                                 variableId2CommonParentNodeId, commonParentNodeId2VariableIds,
//                                 dependedVariablesOfEdge, unequal_variables_matrix, unequal_constants };
    std::unique_ptr<Statement<T,V>> statement_ptr = goal_query_ptr->translate(translate_info);


    std::vector<std::string> strongly_recognizable_vnames;
    std::vector<int> strongly_recognizable_vids;
    for(auto& vname : strongly_recognizable_set) {
      strongly_recognizable_vnames.push_back(vname);
      strongly_recognizable_vids.push_back(variableIdMap.getIdByName(vname));
    }

    // This is just a hack to overcome a problem with inserting weakly recognizable variables.
    // identify free variables in statement_ptr. If they are not in strongly_recognizable_set, they must be weakly recognizable.
    StatementRecord<T,V> sr_tmp{statement_ptr->clone(), constantIdMap, variableIdMap };
    for(auto free_vid : sr_tmp.getStatement().getFreeVariableIDs()) {
      if (std::ranges::find(strongly_recognizable_vids, free_vid) == strongly_recognizable_vids.end()) {
        strongly_recognizable_vnames.push_back(variableIdMap.getNameById(free_vid));
        strongly_recognizable_vids.push_back(free_vid);
      }
    }

    std::unique_ptr<Statement<T,V>> s_statement_ptr{appendRecognizableVariables<T,V>(translate_info, std::move(statement_ptr), strongly_recognizable_vnames, strongly_recognizable_vids)};

    return {std::move(s_statement_ptr), constantIdMap, variableIdMap };
  }


  StatementRecord<T,V> makeExtendedStatementRecord(bool isAP, bool isExcludeAll) {
    auto goal_query_ptr_2 = goal_query_ptr->clone();
    goal_query_ptr_2->setEvalSpecDbKey("depth_if_true_else_min_limit");
    GoalQueryStatement gqs2{std::move(goal_query_ptr_2), strongly_recognizable_set, weakly_recognizable_set, unequal_CVs};

    StatementRecord<T,V> sr = gqs2.makeStatementRecord();
    auto statement_ptr = sr.getStatement().clone();
    statement_ptr->setEvalSpecByDbKey("depth_if_true_else_min_limit");

    if (isAP) {
      if (!isExcludeAll) {
        auto extended_statement_ptr = std::make_unique<AllFinallyStatement<T,V>>(0, std::move(statement_ptr));
        extended_statement_ptr->setEvalSpecByDbKey("max_value");
        return { std::move(extended_statement_ptr), sr.getConstantIdMap(), sr.getVariableIdMap() };
      } else {
        auto extended_constantIdMap = sr.getConstantIdMap();
        auto extended_variableIdMap = sr.getVariableIdMap();
        auto extended_statement_ptr = std::make_unique<AllUntilStatement<T,V>>(0, makeExclusiveStatement<T,V>(extended_variableIdMap, extended_constantIdMap), std::move(statement_ptr));
        extended_statement_ptr->setEvalSpecByDbKey("max_value");
        return { std::move(extended_statement_ptr), extended_constantIdMap, extended_variableIdMap };
      }
    } else {
      if (!isExcludeAll) {
        auto extended_statement_ptr = std::make_unique<SomeFinallyStatement<T,V>>(0, std::move(statement_ptr));
        extended_statement_ptr->setEvalSpecByDbKey("max_value_no_shortcut");
        return { std::move(extended_statement_ptr), sr.getConstantIdMap(), sr.getVariableIdMap() };
      } else {
        auto extended_constantIdMap = sr.getConstantIdMap();
        auto extended_variableIdMap = sr.getVariableIdMap();
        auto extended_statement_ptr = std::make_unique<SomeUntilStatement<T,V>>(0, makeExclusiveStatement<T,V>(extended_variableIdMap, extended_constantIdMap), std::move(statement_ptr));
        extended_statement_ptr->setEvalSpecByDbKey("max_value_no_shortcut");
        return { std::move(extended_statement_ptr), extended_constantIdMap, extended_variableIdMap };
      }
    }
  }


  std::string to_string() const {
    typename GoalQuery<T,V>::ToStringInfo context{strongly_recognizable_set, weakly_recognizable_set};
    std::string s = "[ " + goal_query_ptr->to_string(context) + " ]";
    for(auto& [cv1, cv2] : unequal_CVs) {
      s += "; " + cv1 + " != " + cv2;
    }
    return s;
  }

  void writeDotFile(const std::string& filename) const {
    GqsGraph gqs_graph(goal_query_nodes.size());
    goal_query_ptr->buildGqsGraph(gqs_graph);
//    gqs_graph.debug();

    typename GoalQuery<T,V>::ToStringInfo context{strongly_recognizable_set, weakly_recognizable_set};
    std::ofstream out{filename};

    out << "digraph G {" << std::endl;
    out << R"(  node [shape=ellipse])" << std::endl;
    out << R"(  edge [color="gray",arrowhead=normal])" << std::endl;

    for(auto state_id : gqs_graph.getStates()) {
      if (goal_query_nodes[state_id]->getType() == GOAL_QUERY_TYPE::NIL_GOAL_QUERY) {
        out << "  " << state_id << " [shape=point]" << std::endl;
      } else {
        out << "  " << state_id << " [label=\"" << goal_query_nodes[state_id]->to_string(context) << "\",color=\"black\"]" << std::endl;
      }
    }

    for(auto edge_id : gqs_graph.getEdges()) {
      if (gqs_graph.getNextStateIdsOfEdge(edge_id).size() > 1) {
        out << "  " << edge_id << " [shape=point]" << std::endl;
      }
    }

    for(auto state_id : gqs_graph.getStates()) {
      int edge_id = gqs_graph.getNextEdgeIdOfState(state_id);
      if (edge_id >= 0) {
        auto edge_ptr = dynamic_cast<EdgeGoalQuery<T,V>*>(goal_query_nodes[edge_id]);
        auto is_all = edge_ptr->isAll();
        auto is_path = edge_ptr->isPath();

        // draw the arrows
        assert(gqs_graph.getNextStateIdsOfEdge(edge_id).size()>=1);
        if (gqs_graph.getNextStateIdsOfEdge(edge_id).size()==1) {
          out << "  " << state_id << "->" << gqs_graph.getNextStateIdsOfEdge(edge_id)[0] << " [";
        } else {
          out << "  " << state_id << "->" << edge_id << " [";
        }
        int cond_id = gqs_graph.getEdgeCondIdOfEdge(edge_id);
        if (cond_id >= 0) out << "label=\"" << goal_query_nodes[cond_id]->to_string(context) << "\",";
        out << (is_path?("color=\"black\""):("color=\"black,invis:black\""));
        out << (is_all?(""):(",style=\"dashed\""));
        out << "]" << std::endl;

        if (gqs_graph.getNextStateIdsOfEdge(edge_id).size()>1) {
          for(auto next_state_id : gqs_graph.getNextStateIdsOfEdge(edge_id)) {
            out << "  " << edge_id << "->" << next_state_id << " [";
            out << "color=\"black\"";
            out << ",style=\"dotted\"";
            out << ",arrowhead=none";
            out << "]" << std::endl;
          }
        }
      }
    }

    // check whether we need to add a virtual root
    std::set<int> non_root_states;
    for(auto state_id : gqs_graph.getStates()) {
      int edge_id = gqs_graph.getNextEdgeIdOfState(state_id);
      if (edge_id >= 0) {
        for(auto next_state_id : gqs_graph.getNextStateIdsOfEdge(edge_id)) {
          non_root_states.insert(next_state_id);
        }
      }
    }
    std::set<int> root_states;
    for(auto state_id : gqs_graph.getStates()) {
      if (non_root_states.find(state_id) == non_root_states.end()) {
        root_states.insert(state_id);
      }
    }
    if (root_states.size() > 1) {
      out << "  " << -1 << " [shape=point]" << std::endl;
      for(auto root_state : root_states) {
        out << "  " << -1 << "->" << root_state << " [";
        out << "color=\"black\"";
        out << ",style=\"dotted\"";
        out << ",arrowhead=none";
        out << "]" << std::endl;
      }
    }

    out << "}" << std::endl;

    out.close();
  }

private:

  void calcMemberData() {
    goal_query_ptr->setNodeIdByDFS(goal_query_nodes);
    goal_query_ptr->assignConstantId(constantIdMap);
    goal_query_ptr->assignVariableId(variableIdMap);
    calcVariableId2NodeId();
//    dependedVariablesOfEdge.resize(variableIdMap.size());
//    goal_query_ptr->calcDependedVariablesOfEdge(dependedVariablesOfEdge, commonParentNodeId2VariableIds, {});
    calcUnequalCV();

//    typename GoalQuery<T,V>::ToStringInfo context{strongly_recognizable_set, weakly_recognizable_set};
//    __pp__(goal_query_nodes[0]->to_string(context));

//    for(auto& x : goal_query_nodes) {
//      typename GoalQuery<T,V>::ToStringInfo context{strongly_recognizable_set};
//      __pp__(x->to_string(context));
//    }
  }

  void calcVariableId2NodeId() {  // also calc commonParentNodeId2VariableIds
    std::vector<std::vector<int>> variableId2Prefix(variableIdMap.size());
    goal_query_ptr->calcCommonNodeIdPrefix(variableId2Prefix, {});
    variableId2CommonParentNodeId.resize(variableIdMap.size());
    for(int vid=0; vid < variableIdMap.size(); vid++) {
      if (!variableId2Prefix[vid].empty()) { // must have an edge before the variable, unless it is bounded.
        variableId2CommonParentNodeId[vid] = variableId2Prefix[vid].back();
        commonParentNodeId2VariableIds[variableId2CommonParentNodeId[vid]].push_back(vid);
      }
    }

//    for(auto& [node_id, vids] : commonParentNodeId2VariableIds) {
//      std::cout << "Node " << node_id << ":";
//      for(auto vid : vids) {
//        std::cout << " " << vid;
//      }
//      std::cout << std::endl;
//    }
//    std::cout << std::endl;
  }

  void calcUnequalCV() {
    for(int i=0; i<variableIdMap.size(); i++) {
      unequal_variables_matrix.emplace_back(variableIdMap.size());
    }
    unequal_constants.resize(variableIdMap.size());
    for(auto& [name1, name2] : unequal_CVs) {
      int vid1 = variableIdMap.getIdByName(name1);
      int vid2 = variableIdMap.getIdByName(name2);
      if (vid1 >= 0) {
        if (vid2 >= 0) {
          unequal_variables_matrix[vid1][vid2] = true;
          unequal_variables_matrix[vid2][vid1] = true;
        } else {
          int cid2 = constantIdMap.getIdByName(name2);
          if (cid2 >= 0) {
            unequal_constants[vid1].insert(cid2);
          } else {
            if (!std::islower(name2[0])) {
              int cid2 = constantIdMap.assignIdToName(name2);
              unequal_constants[vid1].insert(cid2);
            } else {
              throw std::runtime_error("Error in GoalQueryStatement::GoalQueryStatement(): Unknown constant " + name2);
            }
          }
        }
      } else { // name1 is not an existing variable
        if (vid2 >= 0) {
          int cid1 = constantIdMap.getIdByName(name1);
          if (cid1 >= 0) {
            unequal_constants[vid2].insert(cid1);
          } else {
            if (!std::islower(name1[0])) {
              int cid1 = constantIdMap.assignIdToName(name1);
              unequal_constants[vid2].insert(cid1);
            } else {
              throw std::runtime_error("Error in GoalQueryStatement::GoalQueryStatement(): Unknown constant " + name1);
            }
          }
        } else {
          throw std::runtime_error("Error in GoalQueryStatement::GoalQueryStatement(): Unknown constants " + name1 + " and " + name2);
        }
      }
    }
  }

};


#endif //FOCTL_GOAL_QUERY_H

