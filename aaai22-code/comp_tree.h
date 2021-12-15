#ifndef FOCTL_COMP_TREE_H
#define FOCTL_COMP_TREE_H

#include <iostream>
#include <vector>
#include <list>
#include <unordered_map>
#include <optional>
#include <bitset>

#include "shared.h"
#include "foctl.h"
#include "environment.h"


// -----------------------------------------------------------------------------------------
// The Computational Tree
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class CompTree {

protected:

  const StatementRecord<T,V>& sr;
  NameIdMap all_constant_id_map;

public:

  CompTree(const Environment& env, const StatementRecord<T,V>& sr) :
      sr{sr}, all_constant_id_map{sr.getConstantIdMap()}
  {
    // add additional goal to the list of constants
    for(auto& constant : env.getGoals()) {
      all_constant_id_map.assignIdToName(constant);
    }
  }

  const std::string& getConstantName(int constant_id) const { return all_constant_id_map.getNameById(constant_id); }

  virtual std::pair<bool,T> run(int start_point_id) = 0;

  virtual const CompTreeContext<T,V>& getCompTreeContext() const = 0;

};




// -----------------------------------------------------------------------------------------
// The Computational Tree for Path-set-based Goal Recognition Environments
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class PathSetCompTree : public CompTree<T,V> {

  using AnnotatedPathPtrList = std::vector<const PathSetEnv::AnnotatedPath*>;

  using CompTree<T,V>::sr;
  using CompTree<T,V>::all_constant_id_map;

  const PathSetEnv& path_set_env;

private:

  class PathSetEnvCompTreeContext : public CompTreeContext<T,V> {

    const PathSetEnv& env;
    const StatementRecord<T,V>& sr;
    const PathSetCompTree<T,V>& comp_tree;

  public:

    PathSetEnvCompTreeContext(const PathSetEnv& env, const StatementRecord<T,V>& sr, const PathSetCompTree<T,V>& comp_tree) :
        env{env}, sr{sr}, comp_tree{comp_tree}
    {
      // do nothing
    }

    std::string getConstantNameFromId(int constant_id) const final {
      return comp_tree.all_constant_id_map.getNameById(constant_id);
    }

    int getVertexIdOfPointId(int point_id) const final {
      return env.getVertexIdOfPoint(point_id);
    }

    std::string getPathStringOfPointId(int point_id) const final {
      std::string s = "[" + std::to_string(env.getStartVertexId());
      for(auto vid : env.getPathPrefixOfPoint(point_id)) {
        s += " "+std::to_string(vid);
      }
      return s + "]";
    }

    std::string to_string(const std::shared_ptr<AnswerBase>& ans) const final {
      if (ans) {
        return std::static_pointer_cast<Answer<T,V>>(ans)->to_string(*this);
      } else {
        return "nil";
      }
    }

    std::string to_string(const T& result) const final {
      std::stringstream out;

      out << result.getValue();
      if (result.getPointId() >= 0) {
        out << "," << getPathStringOfPointId(result.getPointId());
      }
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        out << ",(" << to_string(result.getAnswer()) << ")";
      }

      return out.str();
    }

  };


public:

  class State : public CompTreeState<T,V> {
    const PathSetCompTree& comp_tree;

    int current_vid;
    const AnnotatedPathPtrList& apath_ptr_list;  // stored in the parent state
    int path_depth;
    std::vector<int> subst;
    std::unordered_map<int,AnnotatedPathPtrList> partition;

  public:

    State(const PathSetCompTree& comp_tree, int current_vid, const AnnotatedPathPtrList& apath_ptr_list, int path_depth) :
        comp_tree{comp_tree}, current_vid{current_vid}, apath_ptr_list{apath_ptr_list}, path_depth{path_depth}, subst(comp_tree.sr.getVariableIdMap().size(), -1)
    {
      calcPartition();   // precompute the partition
    }

    State(const PathSetCompTree& comp_tree, int current_vid, const AnnotatedPathPtrList& apath_ptr_list, int path_depth, const std::vector<int>& subst) :
        comp_tree{comp_tree}, current_vid{current_vid}, apath_ptr_list{apath_ptr_list}, path_depth{path_depth}, subst{subst}
    {
      calcPartition();   // precompute the partition
    }

    const AnnotatedPathPtrList& getAnnotatedPathPtrList() const { return apath_ptr_list; }

    int getPathDepth() const final { return path_depth; }

    int getPointId() const final {
      assert(!apath_ptr_list.empty() && "Error in PathSetCompTree::State::getPointId(): apath_ptr_list is empty.");
      return comp_tree.path_set_env.getPointId(apath_ptr_list[0]->getId(), path_depth);
    }

    bool isTerminal() const final { return partition.empty(); }    // must have computed the partition

    std::vector<std::unique_ptr<CompTreeState<T,V>>> getNextStates() const final {
      std::vector<std::unique_ptr<CompTreeState<T,V>>> next_states;
      for(auto& [vid, pps] : partition) {
        next_states.emplace_back(new State{comp_tree, vid, pps, path_depth+1, subst});
      }
      return next_states;
    }

    bool hasGoal(int constant_id) const final {
      return comp_tree.path_set_env.hasGoal(current_vid, comp_tree.all_constant_id_map.getNameById(constant_id));
    }

    int getSubstitute(int variable_id) const final {
      return subst[variable_id];
    }

    int getNumOfConstants() const final {
      return comp_tree.all_constant_id_map.size();
    }

    const std::vector<int>& getSubstitute() const { return subst; }

    void substitute(int variable_id, int constant_id) final {
      subst[variable_id] = constant_id;
    }

    void unsubstitute(int variable_id) final {
      subst[variable_id] = -1;
    }

    std::string getInfo() const final {
      std::string s = "V" + std::to_string(current_vid) + " PD" + std::to_string(path_depth) + " ";
      s += "[";
      for(auto constant_id : subst) {
        if (constant_id >= 0) {
          s += " "+ comp_tree.all_constant_id_map.getNameById(constant_id);
        } else {
          s += " _";
        }
      }
      s += " ]";
      return s;
    }

    const CompTreeContext<T,V>& getCompTreeContext() const final {
      return comp_tree.context;
    }

    void debug() final {
      // do nothing
    }

  private:

    void calcPartition() {
      for(auto& apath_ptr : apath_ptr_list) {
        if (path_depth < apath_ptr->getPath().size()) {
          auto vid = apath_ptr->getPath().at(path_depth);
          partition[vid].push_back(apath_ptr);
        }
      }
    }
  };


  class Cache : public CompTreeCache<T,V> {
    const int MAX_NUMBER_OF_VARIABLES = 100;
    const int MAX_NUM_OF_BITS_FOR_PATH_PROFILE = 32; //  must be less than 32
    const int MAX_NUM_OF_BITS_FOR_SUBSTITUTE = 32;   //  must be less than 32

    const PathSetCompTree& comp_tree;
    const PathSetEnv& path_set_env;
    const StatementRecord<T,V>& sr;
    const NameIdMap& all_constants_id_map;

    int constant_num;
    int max_path_profile_size;
    int max_free_var_num;

    std::vector<std::vector<std::unordered_map<uint64_t,std::pair<bool,T>>>> my_cache;
    int my_cache_size;

  public:

    explicit Cache(const PathSetCompTree& comp_tree) :
        comp_tree{comp_tree}, path_set_env{comp_tree.path_set_env}, sr{comp_tree.sr}, all_constants_id_map{comp_tree.all_constant_id_map}
    {
      int node_num = sr.getNodeNum();
      int point_num = path_set_env.getPointNum();
      constant_num = all_constants_id_map.size();
      max_path_profile_size = MAX_NUM_OF_BITS_FOR_PATH_PROFILE;
      max_free_var_num = calcMaxFreeVarNum(MAX_NUM_OF_BITS_FOR_SUBSTITUTE);

      my_cache.resize(node_num);
      for(int node_id=0; node_id<node_num; node_id++) {
        my_cache[node_id].resize(point_num);
      }
      my_cache_size = 0;
    }

    std::optional<std::pair<bool,T>> get(int foctl_node_id, const CompTreeState<T,V>& state) const {
      auto& s = dynamic_cast<const PathSetCompTree<T,V>::State&>(state);
      // prune away records that are too large to store.
      auto point_id = s.getPointId();
      auto& path_profile = path_set_env.getPathProfile(point_id);
      if (path_profile.size() > max_path_profile_size) return std::nullopt;
      auto& free_var_ids = sr.getNode(foctl_node_id).getFreeVariableIDs();
      if (free_var_ids.size() > max_free_var_num) return std::nullopt;
      // encode
      auto ss_code = encodeSubstitute(s.getSubstitute(), free_var_ids);
      auto pp_code = encodePathProfile(s.getAnnotatedPathPtrList(), path_profile);
      uint64_t sp_code = combine(ss_code, pp_code);
      // find
      try {
        return { my_cache[foctl_node_id][point_id].at(sp_code) };
      } catch(const std::out_of_range& e) {
        return std::nullopt;
      }
    }

    void add(int foctl_node_id, const CompTreeState<T,V>& state, std::pair<bool,T> result) {
      auto& s = dynamic_cast<const PathSetCompTree<T,V>::State&>(state);
      // prune away records that are too large to store.
      auto point_id = s.getPointId();
      auto& path_profile = path_set_env.getPathProfile(point_id);
      if (path_profile.size() > max_path_profile_size) return;
      auto& free_var_ids = sr.getNode(foctl_node_id).getFreeVariableIDs();
      if (free_var_ids.size() > max_free_var_num) return;
      // encode
      auto ss_code = encodeSubstitute(s.getSubstitute(), free_var_ids);
      auto pp_code = encodePathProfile(s.getAnnotatedPathPtrList(), path_profile);
      uint64_t sp_code = combine(ss_code, pp_code);
      // find
      my_cache[foctl_node_id][point_id][sp_code] = result;
      my_cache_size++;
    }

  private:

    int calcMaxFreeVarNum(int max_bit_num) const {
      uint32_t max_value = 0;
      for(int i=0; i<max_bit_num; i++) {
        max_value <<= 1;
        max_value++;
      }
      uint64_t v = 0;  // must be larger than uint32_t
      int count = 0;
      while(v <= max_value) {
        v *= constant_num;
        v += constant_num;
        count++;
        if (count >= MAX_NUMBER_OF_VARIABLES) break;
      }
      return count;
    }

    uint32_t encodeSubstitute(const std::vector<int>& subst, const std::set<int>& free_var_ids) const {
      std::vector<int> vec(free_var_ids.size(), -1);
      for(int i=0; auto free_var_id : free_var_ids) {
        int constant_id = subst[free_var_id];
        assert(0 <= constant_id && constant_id <= constant_num);
        vec[i] = constant_id;
        i++;
      }
      uint32_t result = 0;
      for(auto v : vec) {
        assert (v >= 0);
        result *= constant_num;
        result += v;
      }
      return result;
    }

    uint32_t encodePathProfile(const AnnotatedPathPtrList& apath_ptr_list, const PathSetEnv::PathProfile& path_profile) const {
      std::bitset<32> result;
      for(auto& apath_ptr : apath_ptr_list) {
        int i = 0;
        for(auto path_id : path_profile) {
          if (apath_ptr->getId() == path_id) break;
          i++;
        }
        assert(i < path_profile.size() && "Error in PathSetCompTree::Cache::encodePathProfile()");
        result.set(i);
      }
      return static_cast<uint32_t>(result.to_ulong());
    }

    uint64_t combine(uint32_t ss_code, uint32_t pp_code) const {
      return (static_cast<uint64_t>(ss_code) << 32) | static_cast<uint64_t>(pp_code);
    }

  };

  class NoCache : public CompTreeCache<T,V> {
  public:
    std::optional<std::pair<bool,T>> get(int foctl_node_id, const CompTreeState<T,V>& state) const { return std::nullopt; }
    void add(int foctl_node_id, const CompTreeState<T,V>& state, std::pair<bool,T> result) {}
  };


public:

  PathSetCompTree(const PathSetEnv& path_set_env, const StatementRecord<T,V>& sr, bool is_use_cache = true) :
      CompTree<T,V>{path_set_env,sr}, path_set_env{path_set_env}, cache{*this}, is_use_cache{is_use_cache}, context{path_set_env, sr, *this}
  {
    // do nothing
  }

  std::pair<bool,T> run(int start_point_id) final {
    AnnotatedPathPtrList apath_ptr_list;   // must keep this alive during search
    for(int path_id : path_set_env.getPathProfile(start_point_id)) {
      if (path_set_env.hasPath(path_id)) {
        apath_ptr_list.push_back(&path_set_env.getAnnotatedPath(path_id));
      }
    }
    State s0{*this, path_set_env.getVertexIdOfPoint(start_point_id), apath_ptr_list, path_set_env.getDepthOfPoint(start_point_id)};
    if (is_use_cache) {
      return sr.getStatement().evaluate(s0, cache);
    } else {
      return sr.getStatement().evaluate(s0, no_cache);
    }
  }

  const CompTreeContext<T,V>& getCompTreeContext() const final { return context; }

private:

  Cache cache;
  NoCache no_cache;
  bool is_use_cache;
  PathSetEnvCompTreeContext context;

};


// -----------------------------------------------------------------------------------------
// The Computational Tree for Graph-based Goal Recognition Environments
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class GraphCompTree : public CompTree<T,V> {
  using CompTree<T,V>::sr;
  using CompTree<T,V>::all_constant_id_map;

  const GraphEnv& graph_env;

private:

  class GraphEnvCompTreeContext : public CompTreeContext<T,V> {

    const GraphEnv& env;
    const StatementRecord<T,V>& sr;
    const GraphCompTree<T,V>& comp_tree;

  public:

    GraphEnvCompTreeContext(const GraphEnv& env, const StatementRecord<T,V>& sr, const GraphCompTree<T,V>& comp_tree) :
       env{env}, sr{sr}, comp_tree{comp_tree}
    {
      // do nothing
    }

    std::string getConstantNameFromId(int constant_id) const final {
      return comp_tree.all_constant_id_map.getNameById(constant_id);
    }

    int getVertexIdOfPointId(int point_id) const final {   // *** assume point_id = vertex_id ***
      return point_id;
    }

    std::string getPathStringOfPointId(int point_id) const {
      return "[" + std::to_string(env.getStartVertexId()) + " --> " + std::to_string(getVertexIdOfPointId(point_id)) + "]";
    }

    std::string to_string(const std::shared_ptr<AnswerBase>& ans) const final {
      if (ans) {
        return std::static_pointer_cast<Answer<T,V>>(ans)->to_string(*this);
      } else {
        return "nil";
      }
    }

    std::string to_string(const T& result) const final {
      std::stringstream out;

      out << result.getValue();
      if (result.getPointId() >= 0) {
        out << "," << std::to_string(getVertexIdOfPointId(result.getPointId()));
      }
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        out << "(" << to_string(result.getAnswer()) << ")";
      }

      return out.str();
    }

  };

public:

  class State : public CompTreeState<T,V> {
    const GraphCompTree& comp_tree;

    GraphEnv::Vertex vertex;
    int path_depth;
    std::vector<int> subst;

  public:

    State(const GraphCompTree& comp_tree, GraphEnv::Vertex vertex, int path_depth) :
       comp_tree{comp_tree}, vertex{vertex}, path_depth{path_depth}, subst(comp_tree.sr.getVariableIdMap().size(), -1)
    {
      // do nothing
    }

    State(const GraphCompTree& comp_tree, GraphEnv::Vertex vertex, int path_depth, const std::vector<int>& subst) :
        comp_tree{comp_tree}, vertex{vertex}, path_depth{path_depth}, subst{subst}
    {
      // do nothing
    }

    int getPathDepth() const final { return path_depth; }

    int getPointId() const final { return vertex.get().first; }

    bool isTerminal() const final { return vertex.outgoingEVList().empty(); }

    std::vector<std::unique_ptr<CompTreeState<T,V>>> getNextStates() const final {
      std::vector<std::unique_ptr<CompTreeState<T,V>>> next_states;
      for(auto [e, v] : vertex.outgoingEVList()) {
        next_states.emplace_back(new State{comp_tree, v, path_depth+1, subst});
      }
      return next_states;
    }

    bool hasGoal(int constant_id) const final {
      GraphEnv::GoalList& goal_list = vertex.get().second;
      return std::ranges::find(goal_list, comp_tree.all_constant_id_map.getNameById(constant_id)) != goal_list.end();
    }

    int getSubstitute(int variable_id) const final {
      return subst[variable_id];
    }

    int getNumOfConstants() const final {
      return comp_tree.all_constant_id_map.size();
    }

    const std::vector<int>& getSubstitute() const { return subst; }

    void substitute(int variable_id, int constant_id) final {
      subst[variable_id] = constant_id;
    }

    void unsubstitute(int variable_id) final {
      subst[variable_id] = -1;
    }

    std::string getInfo() const final {
      std::string s = "V" + std::to_string(vertex.get().first) + " PD" + std::to_string(path_depth) + " ";
      s += "[";
      for(auto constant_id : subst) {
        if (constant_id >= 0) {
          s += " "+ comp_tree.all_constant_id_map.getNameById(constant_id);
        } else {
          s += " _";
        }
      }
      s += " ]";
      return s;
    }

    const CompTreeContext<T,V>& getCompTreeContext() const final {
      return comp_tree.context;
    }

    void debug() final {
      __pp__("GraphCompTree::State::vertex ", vertex.get().first);
      std::cout << "  subst = ";
      for(auto constant_id : subst) {
        std::cout << "\"" << comp_tree.all_constant_id_map.getNameById(constant_id) << "\" " << std::endl;
      }
    }
  };

  class NoCache : public CompTreeCache<T,V> {
  public:
    std::optional<std::pair<bool,T>> get(int foctl_node_id, const CompTreeState<T,V>& state) const { return std::nullopt; }
    void add(int foctl_node_id, const CompTreeState<T,V>& state, std::pair<bool,T> result) {}
  };

public:

  GraphCompTree(const GraphEnv& graph_env, const StatementRecord<T,V>& sr) :   // no cache
      CompTree<T,V>{graph_env,sr}, graph_env{graph_env}, context{graph_env, sr, *this}
  {
    // do nothing
  }

  std::pair<bool,T> run(int start_point_id) final {
    // point_id == vertex id
    State s0{*this , graph_env.getVertex(start_point_id), 0};
    return sr.getStatement().evaluate(s0, no_cache);
  }

  const CompTreeContext<T,V>& getCompTreeContext() const final { return context; }

private:

  NoCache no_cache;

  GraphEnvCompTreeContext context;

};




#endif //FOCTL_COMP_TREE_H
