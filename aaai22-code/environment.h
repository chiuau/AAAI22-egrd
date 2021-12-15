#ifndef FOCTL_ENVIRONMENT_H
#define FOCTL_ENVIRONMENT_H

#include <vector>
#include <set>
#include <list>
#include <algorithm>
#include <ranges>
#include <unordered_map>
#include <limits>

#include "graph.h"
#include "name_id_map.h"


// -----------------------------------------------------------------------------------------
// A Goal Recognition Environment
// -----------------------------------------------------------------------------------------

class Environment {
public:

  using Path = std::vector<int>;
  using GoalList = std::vector<std::string>;

  class ModAction {
    int id;
    std::vector<int> add_list;
    std::vector<int> del_list;

  public:

    ModAction(int id) : id{id} {}

    int getId() const { return id; }

    const std::vector<int>& getAddList() const { return add_list; }
    const std::vector<int>& getDelList() const { return del_list; }

    void addToAddList(int edge_id) { add_list.push_back(edge_id); }
    void addToDelList(int edge_id) { del_list.push_back(edge_id); }

    std::string to_string() const {
      std::string s = "M" + std::to_string(id) + "[";
      bool isFirst = true;
      for(int e : add_list) {
        if (!isFirst) s += ",";
        s += "add " + std::to_string(e);
        isFirst = false;
      }
      for(int e : del_list) {
        if (!isFirst) s += ",";
        s += "del " + std::to_string(e);
        isFirst = false;
      }
      s += "]";
      return s;
    }
  };

  class EnvState {

    std::vector<bool> isActiveEdgeId;

  public:

    EnvState(const std::vector<bool>& isActiveEdgeId) : isActiveEdgeId{isActiveEdgeId} {}

    EnvState(const EnvState& other) : isActiveEdgeId{other.isActiveEdgeId} {}

    bool isApplicable(const ModAction& mod_action) const {
      for(auto edge_id : mod_action.getAddList()) {
        if (hasActiveEdge(edge_id)) return false;
      }
      for(auto edge_id : mod_action.getDelList()) {
        if (!hasActiveEdge(edge_id)) return false;
      }
      return true;
    }

    EnvState apply(const ModAction& mod_action) const {
      EnvState env{*this};
      for(auto edge_id : mod_action.getAddList()) {
        env.addActiveEdge(edge_id);
      }
      for(auto edge_id : mod_action.getDelList()) {
        env.deleteActiveEdge(edge_id);
      }
      return env;
    }

    bool operator==(const EnvState& other) const {
      for(int i=0; i<isActiveEdgeId.size(); i++) {
        if (isActiveEdgeId[i] != other.isActiveEdgeId[i]) return false;
      }
      return true;
    }

  private:

    bool hasActiveEdge(int edge_id) const { return isActiveEdgeId[edge_id]; }

    void addActiveEdge(int edge_id) {
      assert(!hasActiveEdge(edge_id) && "Error in Environment::EnvState::addActiveEdge(): edge exists");
      isActiveEdgeId[edge_id] = true;
    }

    void deleteActiveEdge(int edge_id) {
      assert(hasActiveEdge(edge_id) && "Error in Environment::EnvState::deleteActiveEdge(): edge does not exist");
      isActiveEdgeId[edge_id] = false;
    }

  };


protected:

  int start_vid;

  std::unordered_map<int,GoalList> vidToGoals;

  bool is_grid_layout;
  int num_x, num_y;
  std::unordered_map<int,std::pair<int,int>> vertex_pos_in_dot;

public:

  virtual bool empty() const = 0;

  int getStartVertexId() const { return start_vid; }

  void setStartVertexId(int start_vid) { Environment::start_vid = start_vid; }

  virtual int getPointIdOfStartVertex() const = 0;


  const GoalList& getGoalList(int vid) const {
    try {
      return vidToGoals.at(vid);
    } catch(const std::out_of_range& e) {
      return empty_goal_list;
    }
  }

  bool hasGoal(int vid, const std::string& goal) const {
    try {
      auto& goals = vidToGoals.at(vid);
      return std::ranges::find(goals, goal) != goals.end();
    } catch(const std::out_of_range& e) {
      return false;
    }
  }

  void addGoal(int vid, const GoalList& goals) {
    for(auto& g : goals) {
      vidToGoals[vid].push_back(g);
    }
  }

  virtual GoalList getGoals() const {
    std::set<std::string> all_goal_set;
    for(auto& [vid, gl] : vidToGoals) {
      all_goal_set.insert(gl.begin(), gl.end());
    }
    return { all_goal_set.begin(), all_goal_set.end() };
  }

  virtual Path getPathPrefixOfPoint(int point_id) const = 0;

  // ====== mod action ======

  virtual EnvState getEnvState() const = 0;

  virtual int getModActionNum() const = 0;

  virtual const ModAction& getModAction(int mod_id) const = 0;

  virtual bool isModActionApplicable(int mod_id) const = 0;

  virtual bool isModActionUnapplicable(int mod_id) const = 0;

  virtual bool isModIdRelevantToPoint(int mod_id, int point_id) const = 0;

  virtual void apply(int mod_id) = 0;

  virtual void unapply(int mod_id) = 0;

  virtual std::string convertModPlanToString(std::vector<int> mod_plan) const = 0;

  // ====== graph construction ======

  virtual void addPath(const Path& path) = 0;

  virtual void wrap_up() = 0;

  virtual void writeDotFile(const std::string& filename) = 0;

protected:

  void putVertexInGrid(int vid, int num_x, int num_y) {
    vertex_pos_in_dot[vid] = { vid % num_x , vid / num_x };
  }

private:
  static const GoalList empty_goal_list;
};



// -----------------------------------------------------------------------------------------
// A Path-Set-based Goal Recognition Environment  (selected path can be selected)
// -----------------------------------------------------------------------------------------

class PathSetEnv : public Environment {

public:

  using PathProfile = std::vector<int>;   // list of path id

  class AnnotatedPath {
    int id;
    Path path;
  public:
    AnnotatedPath(int id, const Path& path) : id{id}, path{path} {}

    int getId() const { return id; }
    const Path& getPath() const { return path; }
  };

private:

  std::vector<AnnotatedPath> apaths;
  std::vector<std::pair<int,int>> edges_for_decoration;
  std::unordered_map<std::string,int> path_name_to_path_id;
  std::vector<ModAction> mod_action_list;

  // --- statistics ---

  int max_path_size;

  // list of all vertices:
  std::vector<int> vertexIds;
  // edge id
  std::vector<std::pair<int,int>> edgeId2VertexIdPair;
  std::unordered_map<int,std::unordered_map<int,int>> vertexIdPair2EdgeId;
  // point id and path Profile
  std::vector<std::vector<int>> pathId2Depth2PointId;
  std::vector<PathProfile> pointId2PathProfile;
  std::vector<int> pointId2Depth;
  // path to edge
  std::vector<std::vector<int>> pathId2EdgeIdList;
  std::vector<int> pathId2EdgeCount;
  std::vector<std::vector<int>> edgeId2PathIdList;

  // active path and edge
  std::vector<bool> isActiveEdgeId;
  std::vector<bool> isActivePathId;
  std::vector<int> pathId2ActiveEdgeCount;

  // mod action's effect on path
  std::vector<std::vector<bool>> isModActionAddEdgeToPath;
  std::vector<std::vector<bool>> isModActionDelEdgeToPath;

public:

  PathSetEnv() {}

  explicit PathSetEnv(const std::vector<std::string>& all_token_list) { readFromTokens(all_token_list); }

  GoalList getGoals() const final { return Environment::getGoals(); }  // it should remove the goals on active tmp_paths only.

  int getPathNum() const { return apaths.size(); }
  int getEdgeNum() const { return edgeId2VertexIdPair.size(); }
  int getPointNum() const { return pointId2PathProfile.size(); }

  bool empty() const final {
    for(int i=0; i<getPathNum(); i++) {
      if (isActivePathId[i]) return false;
    }
    return true;
  }

  int getPointIdOfStartVertex() const final {
    return pathId2Depth2PointId[0][0];
  }

  int getPointId(int path_id, int depth) const {    // return -1 if this is a terminal node
    return pathId2Depth2PointId[path_id][depth];
  }

  int getVertexIdOfPoint(int point_id) const {
    auto& profile = getPathProfile(point_id);
    if (profile.empty()) {
      return -1;
    } else {
      auto depth = getDepthOfPoint(point_id);
      if (depth==0) {
        return start_vid;
      } else {
        return apaths[profile[0]].getPath()[depth-1];
      }
    }
  }

  int getDepthOfPoint(int point_id) const { return pointId2Depth[point_id]; }

  const PathProfile& getPathProfile(int point_id) const {
    if (point_id >= 0) {
      return pointId2PathProfile[point_id];
    } else {
      return emptyPathProfile;   // which means terminal node
    }
  }

  const PathProfile& getPathProfile(int path_id, int depth) const {
    return getPathProfile(pathId2Depth2PointId[path_id][depth]);
  }

  Path getPathPrefixOfPoint(int point_id) const final {
    auto& profile = getPathProfile(point_id);
    assert(!profile.empty());
    const Path& path = apaths[profile[0]].getPath();
    int depth = pointId2Depth[point_id];

    Path prefix;
    for(int i=0; i<depth; i++) {
      prefix.push_back(path[i]);
    }
    return prefix;
  }

  std::vector<int> getEdgeIdsOnPathPrefixOfPoint(int point_id) const {
    auto& profile = getPathProfile(point_id);
    assert(!profile.empty());
    const Path& path = apaths[profile[0]].getPath();
    int depth = pointId2Depth[point_id];

    std::vector<int> edge_ids;
    int last_vid = start_vid;
    for(int i=0; i<depth; i++) {
      int vid = path[i];
      int edge_id = getEdgeId(last_vid, vid);
      edge_ids.push_back(edge_id);
      last_vid = vid;
    }
    return edge_ids;
  }


  bool hasPath(int path_id) const { return isActivePathId[path_id]; }

  const AnnotatedPath& getAnnotatedPath(int path_id) const { return apaths[path_id]; }

  std::vector<const AnnotatedPath*> getAnnotatedPathPtrList() const;


  // ====== edge management ======

  int getEdgeId(int vid1, int vid2) const {    // return -1 if the edge does not exist
    if (vertexIdPair2EdgeId.contains(vid1)) {
      if (vertexIdPair2EdgeId.at(vid1).contains(vid2)) {
        return vertexIdPair2EdgeId.at(vid1).at(vid2);
      }
    }
    return -1;
  }

  bool hasActiveEdge(int edge_id) const { return isActiveEdgeId[edge_id]; }

  void addActiveEdge(int edge_id);

  void deleteActiveEdge(int edge_id);

  void addActiveEdgesOnAllPaths() {
    for(int path_id=0 ; path_id<getPathNum() ; path_id++) {
      addActiveEdgesOnPath(path_id);
    }
    setActivePaths();
  }

  void addActiveEdgesOnPaths(const std::vector<int>& path_id_list) {
    for(int path_id : path_id_list) {
      addActiveEdgesOnPath(path_id);
    }
    setActivePaths();
  }

  // ====== mod action ======

  EnvState getEnvState() const final { return EnvState(isActiveEdgeId); }

  int getModActionNum() const final { return mod_action_list.size(); }

  const ModAction& getModAction(int mod_id) const final { return mod_action_list[mod_id]; }

  bool isModActionApplicable(int mod_id) const final {
    assert(0 <= mod_id && mod_id < mod_action_list.size());
    const ModAction& mod = mod_action_list[mod_id];
    for(auto edge_id : mod.getAddList()) {
      if (hasActiveEdge(edge_id)) return false;
    }
    for(auto edge_id : mod.getDelList()) {
      if (!hasActiveEdge(edge_id)) return false;
    }
    return true;
  }

  bool isModActionUnapplicable(int mod_id) const final {
    assert(0 <= mod_id && mod_id < mod_action_list.size());
    const ModAction& mod = mod_action_list[mod_id];
    for(auto edge_id : mod.getAddList()) {
      if (!hasActiveEdge(edge_id)) return false;
    }
    for(auto edge_id : mod.getDelList()) {
      if (hasActiveEdge(edge_id)) return false;
    }
    return true;
  }

  bool isModIdRelevantToPoint(int mod_id, int point_id) const final {
    auto& profile = getPathProfile(point_id);
//    __pp__("Path Prefix at Point ", point_id, " is ", getPathPrefixOfPoint(point_id));
//    __pp__("Path Profile at Point ", point_id, " is ", profile);
//    __pp__("mod_id = ", mod_id);
    for(auto path_id : profile) {
      if (isModActionAddEdgeToPath[mod_id][path_id]) return true;
      if (isModActionDelEdgeToPath[mod_id][path_id]) return true;
    }
    return false;
  }

  void apply(int mod_id) final {
    const ModAction& mod = mod_action_list[mod_id];
    for(auto edge_id : mod.getAddList()) {
      addActiveEdge(edge_id);
    }
    for(auto edge_id : mod.getDelList()) {
      deleteActiveEdge(edge_id);
    }
  }

  void unapply(int mod_id) final {
    const ModAction& mod = mod_action_list[mod_id];
    for(auto edge_id : mod.getAddList()) {
      deleteActiveEdge(edge_id);
    }
    for(auto edge_id : mod.getDelList()) {
      addActiveEdge(edge_id);
    }
  }

  std::string convertModPlanToString(std::vector<int> mod_plan) const final {
    std::string s;
    for(bool isFirst=true; auto mod_id : mod_plan) {
      if (!isFirst) s += " ";
      s += mod_action_list[mod_id].to_string();
      isFirst = false;
    }
    return s;
  }


  // ====== environment construction ======

  void addPath(const Path& path) final {
    apaths.emplace_back(apaths.size(), path);
  }

  void wrap_up() final;

  void writeDotFile(const std::string& filename) final;


private:

  void readFromTokens(const std::vector<std::string>& all_token_list);

  void addActiveEdgesOnPath(int path_id);

  void setActivePaths();

  bool checkCycle();

  void calcPathProfile();

  void calcPathProfile_impl(const std::vector<const PathSetEnv::AnnotatedPath*>& apath_ptr_list, int path_depth, int vertex_id);

  void calcModActionAddDelEdgeToPath();

  void addNewEdge(int vid1, int vid2);

  void printEdges();

  void printProfileTable();

private:
  static const PathProfile emptyPathProfile;

};


// -----------------------------------------------------------------------------------------
// A Graph-based Goal Recognition Environment
// -----------------------------------------------------------------------------------------

class GraphEnv : public Environment {

public:

  using Graph = AdjListGraph<std::pair<int,GoalList>,int>;
  using Vertex = Graph::Vertex;

private:

  Graph graph;
  std::unordered_map<int,Vertex> vid2vertex;

  // edge id
  std::vector<std::pair<int,int>> edgeId2VertexIdPair;
  std::unordered_map<int,std::unordered_map<int,int>> vertexIdPair2EdgeId;

  // active edge
  std::vector<bool> isActiveEdgeId;

  // mod actions
  std::vector<ModAction> mod_action_list;

  // temporarily variables.
  std::vector<Path> tmp_paths;  // just for temporarily use before wrap_up
  std::vector<std::pair<int,int>> tmp_edges;  // just for temporarily use before wrap_up

public:

  GraphEnv() {}

  explicit GraphEnv(const std::vector<std::string>& all_token_list) { readFromTokens(all_token_list); }

  bool empty() const final { return graph.getEdgeNum() == 0; }

  int getPointIdOfStartVertex() const final { return start_vid; }

  Vertex getVertex(int vid) const { return vid2vertex.at(vid); }

  int getEdgeNum() const { return edgeId2VertexIdPair.size(); }

  // const Graph& getGraph() const { return graph; }  // may not be a good idea to expose the graph

  Path getPathPrefixOfPoint(int point_id) const final;   // assume it is the shortest path


  // ====== edge management ======

  int getEdgeId(int vid1, int vid2) const {    // return -1 if the edge does not exist
    if (vertexIdPair2EdgeId.contains(vid1)) {
      if (vertexIdPair2EdgeId.at(vid1).contains(vid2)) {
        return vertexIdPair2EdgeId.at(vid1).at(vid2);
      }
    }
    return -1;
  }

  bool hasActiveEdge(int edge_id) const { return isActiveEdgeId[edge_id]; }

  void addActiveEdge(int edge_id) {
    assert(!hasActiveEdge(edge_id) && "Error in GraphEnv::addActiveEdge(): edge exists");
    auto [vid1, vid2] = edgeId2VertexIdPair[edge_id];
    graph.addEdge(getVertex(vid1), getVertex(vid2), 1);
    isActiveEdgeId[edge_id] = true;
  }

  void deleteActiveEdge(int edge_id) {
    assert(hasActiveEdge(edge_id) && "Error in GraphEnv::deleteActiveEdge(): edge does not exist");
    auto [vid1, vid2] = edgeId2VertexIdPair[edge_id];
    graph.deleteEdge(graph.getEdge(getVertex(vid1), getVertex(vid2)));
    isActiveEdgeId[edge_id] = false;
  }

  // ====== mod action ======

  EnvState getEnvState() const final { return EnvState(isActiveEdgeId); }

  int getModActionNum() const final { return mod_action_list.size(); }

  const ModAction& getModAction(int mod_id) const final { return mod_action_list[mod_id]; }

  bool isModActionApplicable(int mod_id) const final {
    assert(0 <= mod_id && mod_id < mod_action_list.size());
    const ModAction& mod = mod_action_list[mod_id];
    for(auto edge_id : mod.getAddList()) {
      if (hasActiveEdge(edge_id)) return false;
    }
    for(auto edge_id : mod.getDelList()) {
      if (!hasActiveEdge(edge_id)) return false;
    }
    return true;
  }

  bool isModActionUnapplicable(int mod_id) const final {
    assert(0 <= mod_id && mod_id < mod_action_list.size());
    const ModAction& mod = mod_action_list[mod_id];
    for(auto edge_id : mod.getAddList()) {
      if (!hasActiveEdge(edge_id)) return false;
    }
    for(auto edge_id : mod.getDelList()) {
      if (hasActiveEdge(edge_id)) return false;
    }
    return true;
  }


  bool isModIdRelevantToPoint(int mod_id, int point_id) const final {
    return true;
  }

  void apply(int mod_id) final {
    const ModAction& mod = mod_action_list[mod_id];
    for(auto edge_id : mod.getAddList()) {
      addActiveEdge(edge_id);
    }
    for(auto edge_id : mod.getDelList()) {
      deleteActiveEdge(edge_id);
    }
  }

  void unapply(int mod_id) final {
    const ModAction& mod = mod_action_list[mod_id];
    for(auto edge_id : mod.getAddList()) {
      deleteActiveEdge(edge_id);
    }
    for(auto edge_id : mod.getDelList()) {
      addActiveEdge(edge_id);
    }
  }

  std::string convertModPlanToString(std::vector<int> mod_plan) const final {
    std::string s;
    for(bool isFirst=true; auto mod_id : mod_plan) {
      if (!isFirst) s += " ";
      s += mod_action_list[mod_id].to_string();
      isFirst = false;
    }
    return s;
  }

  // ====== environment construction ======

  void addPath(const Path& path) final { tmp_paths.push_back(path); }  // Warning: didn't check whether the path is valid yet

  void wrap_up() {
    assert(!vid2vertex.contains(start_vid));
    vid2vertex[start_vid] = graph.addVertex({start_vid, {}});
    addTmpPathToGraph();
    addTmpEdgeToGraph();
    deleteIsolatedVertices();
    assert(vid2vertex.size() > 0 && "The graph has no vertex");
    assignGoalsToVertices();
    if (is_grid_layout) {
      putVerticesInGrid(num_x, num_y);
    }
  }

  void writeDotFile(const std::string& filename) final;

private:

  void readFromTokens(const std::vector<std::string>& all_token_list);
  void addTmpPathToGraph();
  void addTmpEdgeToGraph();
  void deleteIsolatedVertices();
  void assignGoalsToVertices();
  void putVerticesInGrid(int num_x, int num_y);

  void addNewEdge(int vid1, int vid2);

};



#endif //FOCTL_ENVIRONMENT_H

//class ModAction {
//  int id;
//  std::vector<std::pair<int,int>> add_list;
//  std::vector<std::pair<int,int>> del_list;
//public:
//
//  ModAction(int id) : id{id} {}
//
//  const std::vector<std::pair<int,int>>& getAddList() const { return add_list; }
//  const std::vector<std::pair<int,int>>& getDelList() const { return del_list; }
//
//  void addToAddList(int vid1, int vid2) { add_list.push_back({ vid1, vid2 }); }
//  void addToDelList(int vid1, int vid2) { del_list.push_back({ vid1, vid2 }); }
//
//  std::string to_string() const {
//    std::string s = "M" + std::to_string(id) + "[";
//    bool isFirst = true;
//    for(auto [vid1, vid2] : add_list) {
//      if (!isFirst) s += ",";
//      s += "+(" + std::to_string(vid1) + "," + std::to_string(vid2) + ")";
//      isFirst = false;
//    }
//    for(auto [vid1, vid2] : del_list) {
//      if (!isFirst) s += ",";
//      s += "-(" + std::to_string(vid1) + "," + std::to_string(vid2) + ")";
//      isFirst = false;
//    }
//    s += "]";
//    return s;
//  }
//};
//
