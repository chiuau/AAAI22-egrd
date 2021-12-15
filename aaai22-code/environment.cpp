#include <set>
#include <fstream>

#include "string_processing.h"
#include "environment.h"
#include "cassert"

const Environment::GoalList Environment::empty_goal_list;

// -----------------------------------------------------------------------------------------
// A Path-Set-based Goal Recognition Environment
// -----------------------------------------------------------------------------------------

const PathSetEnv::PathProfile PathSetEnv::emptyPathProfile;

std::vector<const PathSetEnv::AnnotatedPath*> PathSetEnv::getAnnotatedPathPtrList() const {
  std::vector<const AnnotatedPath*> apath_ptr_list;
  for(auto& apath : apaths) {
    if (isActivePathId[apath.getId()]) {
      apath_ptr_list.push_back(&apath);
    }
  }
  return apath_ptr_list;
}

void PathSetEnv::addActiveEdge(int edge_id) {
  assert(!hasActiveEdge(edge_id) && "Error in PathSetEnv::addActiveEdge(): edge exists");
  isActiveEdgeId[edge_id] = true;
  for(auto path_id : edgeId2PathIdList[edge_id]) {
    assert(!isActivePathId[path_id] && "Error in PathSetEnv::addActiveEdge(): path_id is not active");
    pathId2ActiveEdgeCount[path_id]++;
    if (pathId2ActiveEdgeCount[path_id] == pathId2EdgeCount[path_id]) {
      isActivePathId[path_id] = true;
    }
  }
}

void PathSetEnv::deleteActiveEdge(int edge_id) {
  assert(hasActiveEdge(edge_id) && "Error in PathSetEnv::deleteActiveEdge(): edge does not exist");
  isActiveEdgeId[edge_id] = false;
  for(auto path_id : edgeId2PathIdList[edge_id]) {
    pathId2ActiveEdgeCount[path_id]--;
    isActivePathId[path_id] = false;  // it must be false
  }
}

void PathSetEnv::wrap_up() {
  assert(apaths.size()>0 && "Error: PathSetEnv::wrap_up(): There is no path.");
  // assert(checkCycle() && "Error: PathSetEnv::wrap_up(): Some paths have duplicated vertices.");

  // (0) find the maximum vertex id
  max_path_size = -1;
  for(auto& apath : apaths) {
    if (static_cast<int>(apath.getPath().size()) > max_path_size) {
      max_path_size = apath.getPath().size();
    }
  }
  assert(max_path_size >= 0);

  // (1) identify all vertices
  vertexIds.push_back(start_vid);
  for(auto& apath : apaths) {
    for(int vid : apath.getPath()) {
      if (std::ranges::find(vertexIds, vid) == vertexIds.end()) {
        vertexIds.push_back(vid);
      }
    }
  }

  // (2) create edges
  for(auto& apath : apaths) {
    int last_vid = start_vid;
    for(int vid : apath.getPath()) {
      addNewEdge(last_vid, vid);
//      if (vertexIdPair2EdgeId[last_vid].find(vid) == vertexIdPair2EdgeId[last_vid].end()) {
//        int edge_id = edgeId2VertexIdPair.size();
//        edgeId2VertexIdPair.emplace_back(last_vid, vid);
//        vertexIdPair2EdgeId[last_vid][vid] = edge_id;
//      }  // else not a new edge, ignore it.
      last_vid = vid;
    }
  }
  // printEdges(); // debug

  // (3) compute pathId2Depth2PointId and pointId2PathProfile
  for(int i=0; i<apaths.size(); i++) {
    pathId2Depth2PointId.emplace_back(max_path_size + 2, -1);  // +2 is necessary
  }
  calcPathProfile();
  // printProfileTable(); // debug

  // (4) compute pathId2EdgeIdList, pathId2EdgeCount, edgeId2PathIdList
  pathId2EdgeIdList.resize(getPathNum());
  pathId2EdgeCount.resize(getPathNum(), 0);
  edgeId2PathIdList.resize(getEdgeNum());
  for(auto& apath : apaths) {
    int path_id = apath.getId();
    int last_vid = start_vid;
    for(int vid : apath.getPath()) {
      int edge_id = vertexIdPair2EdgeId[last_vid][vid];
      pathId2EdgeIdList[path_id].push_back(edge_id);
      pathId2EdgeCount[path_id]++;
      edgeId2PathIdList[edge_id].push_back(path_id);
      last_vid = vid;
    }
  }

  // (5) Initial isActiveEdgeId, isActivePathId, and pathId2ActiveEdgeCount
  isActiveEdgeId.resize(getEdgeNum(), false);
  isActivePathId.resize(getPathNum(), false);
  pathId2ActiveEdgeCount.resize(getPathNum(), 0);

  // (6) Set grid position if necessary
  if (is_grid_layout) {
    for(auto vid : vertexIds) {
      putVertexInGrid(vid, num_x, num_y);
    }
  }
}


void PathSetEnv::readFromTokens(const std::vector<std::string>& all_token_list) {
  if (!isContainKeyword(all_token_list, "PATH_ENV")) {
    throw std::runtime_error("Error: PathSetEnv::readFromTokens(): Cannot find PATH_ENV");
  }

  auto token_partition = partitionTokensByKeywords(all_token_list, {"PATH_ENV", "GRID_LAYOUT", "START_VERTEX", "PATH", "EDGE", "GOAL", "ACTIVE_PATHS", "MOD"});

  is_grid_layout = false;
  for(auto& [keyword, tokens] : token_partition) {
    if (keyword == "GRID_LAYOUT") {
      is_grid_layout = true;
      std::tie(num_x, num_y) = parseTwoTokens<int,int>(tokens);
      break;
    }
  }

  for(auto& [keyword, tokens] : token_partition) {
    if (keyword == "START_VERTEX") {
      setStartVertexId(parseOneToken<int>(tokens));
      break;
    }
  }

  for(auto& [keyword, tokens] : token_partition) {
    if (keyword == "PATH") {
      auto path_name = tokens[0];
      auto path = parseTokenList<int>(remove_front(tokens));
      if (path[0] != getStartVertexId()) throw std::runtime_error("Error: PathSetEnv::readFromFile(): wrong starting vertex for PATH");
      addPath(remove_front(path));
      path_name_to_path_id[path_name] = apaths.size()-1;
    } else if (keyword == "EDGE") {
      auto [ vid1, vid2 ] = parseTwoTokens<int,int>(tokens);
      edges_for_decoration.push_back({vid1, vid2});
    } else if (keyword == "GOAL") {
      auto [ vid, goals ] = parseOneTokenAndTokenList<int,std::string>(tokens);
      addGoal(vid, goals);
    } // else do nothing
  }

  wrap_up();

  std::vector<int> path_id_list;
  bool isFound = false;
  for(auto& [keyword, tokens] : token_partition) {
    if (keyword == "ACTIVE_PATHS") {
      isFound = true;
      for(auto &token : tokens) {
        assert(path_name_to_path_id.contains(token) && "Error in PathSetEnv::readFromTokens(): cannot find path name.");
        path_id_list.push_back(path_name_to_path_id[token]);
      }
      addActiveEdgesOnPaths(path_id_list);
      break;
    }
  }
  if (!isFound) {
    addActiveEdgesOnAllPaths();  // assume all edges in the file has effect
  }

  for(auto& [keyword, tokens] : token_partition) {
    if (keyword == "MOD") {
      ModAction mod_action(mod_action_list.size());
      auto token_partition_2 = partitionTokensByKeywords(tokens, {"ADD", "DEL"});
      for(auto& [keyword_2, tokens_2] : token_partition_2) {
        if (keyword_2 == "ADD") {
          if (tokens_2.size() != 2) throw std::runtime_error("Error in PathSetEnv::readFromTokens(): MOD ADD has wrong numbers of vertices");
          auto [vid1, vid2] = parseTwoTokens<int,int>(tokens_2);
          int edge_id = getEdgeId(vid1, vid2);
          mod_action.addToAddList(edge_id);
        } else if (keyword_2 == "DEL") {
          if (tokens_2.size() != 2) throw std::runtime_error("Error in PathSetEnv::readFromTokens(): MOD DEL has wrong numbers of vertices");
          auto [vid1, vid2] = parseTwoTokens<int,int>(tokens_2);
          int edge_id = getEdgeId(vid1, vid2);
          mod_action.addToDelList(edge_id);
        }   // else ignore it
      }
      mod_action_list.push_back(mod_action);
    }
  }

  calcModActionAddDelEdgeToPath();

}


void PathSetEnv::writeDotFile(const std::string& filename) {
  std::ofstream out{filename};

  out << "digraph G {" << std::endl;
  // header
  out << R"(  node [shape=circle fixedsize=shape])" << std::endl;
  out << R"(  edge [color="gray"])" << std::endl;

  // draw vertices
  out << "  " << start_vid << "[label=\"" << start_vid;
  if (vidToGoals.find(start_vid) != vidToGoals.end()) {
    for(bool isFirst = true; auto& goal : vidToGoals[start_vid]) {
      out << (isFirst?'\n':' ') << goal;
      isFirst = false;
    }
  }
  out << "\"";
  out << R"(, color="blue", style="bold")";
  if (vertex_pos_in_dot.contains(start_vid)) {
    auto [x, y] = vertex_pos_in_dot[start_vid];
    out << R"(, pos=")" << x << "," << y << R"(!")";
  }
  out << "];" << std::endl;

  std::set<int> vertex_set;
  for(auto& apath : apaths) {
    auto& path = apath.getPath();
    for(auto vid : path) {
      if (vertex_set.find(vid) == vertex_set.end()) {  // new vid
        vertex_set.insert(vid);
        out << "  " << vid << " [label=\"" << vid;
        if (vidToGoals.find(vid) == vidToGoals.end()) {
          out << "\"";
        } else {
          for(bool isFirst = true; auto& goal : vidToGoals[vid]) {
            out << (isFirst?'\n':' ') << goal;
            isFirst = false;
          }
          out << "\"";
          out << R"(, color="red", style="bold")";
        }
        // set position
        if (vertex_pos_in_dot.contains(vid)) {
          auto [x, y] = vertex_pos_in_dot[vid];
          out << R"(, pos=")" << x << "," << y << R"(!")";
        }
        out << "];" << std::endl;
      }  // else skip it
    }
  }

  // draw edges
  std::set<std::pair<int,int>> edge_set;

  for(auto& apath : apaths) {
    auto& path = apath.getPath();
    int last_vid = start_vid;
    for(auto vid : path) {
      if (edge_set.find({last_vid, vid}) == edge_set.end()) {  // new edge
        out << "  " << last_vid << "->" << vid << " [";
        int edge_id = vertexIdPair2EdgeId[last_vid][vid];
        out << "label=" << edge_id << ",";
        if (isActiveEdgeId[edge_id]) {
          bool isPartOfActivePath = false;
          for(int path_id : edgeId2PathIdList[edge_id]) {
            if (isActivePathId[path_id]) { isPartOfActivePath = true; break; }
          }
          if (isPartOfActivePath) {
            out << R"(color="black"])" << "\n";
          } else {
            out << R"(color="grey"])" << "\n";
          }
        } else {  // not active
          out << R"(color="grey",style="dotted"])" << "\n";
        }
        edge_set.insert({last_vid, vid});
      }
      last_vid = vid;
    }
  }

  for(auto [vid1, vid2] : edges_for_decoration) {
    if (getEdgeId(vid1, vid2) < 0) {   // edge not exist -> such edge is just a decoration
      out << "  " << vid1 << "->" << vid2 << R"( [color="grey",style="dotted"])" << "\n";
    }  // otherwise, the edge has been drawn.
  }

  out << "}" << std::endl;

  out.close();
}

void PathSetEnv::addActiveEdgesOnPath(int path_id) {
  for(int edge_id : pathId2EdgeIdList[path_id]) {
    isActiveEdgeId[edge_id] = true;
  }
}

void PathSetEnv::setActivePaths() {
  for(int path_id=0; path_id < getPathNum(); path_id++) {
    bool is_active = true;
    int count = 0;
    for(int edge_id : pathId2EdgeIdList[path_id]) {
      if (hasActiveEdge(edge_id)) {
        count++;
      } else {
        is_active = false;
      }
    }
    isActivePathId[path_id] = is_active;
    pathId2ActiveEdgeCount[path_id] = count;
  }
}

bool PathSetEnv::checkCycle() {
  for(auto& apath : apaths) {
    std::set<int> vertex_ids;
    vertex_ids.insert(start_vid);
    for(int vid : apath.getPath()) {
      if (vertex_ids.find(vid) == vertex_ids.end()) {
        vertex_ids.insert(vid);
      } else {
        return false;
      }
    }
  }
  return true;
}

void PathSetEnv::calcPathProfile() {
  int path_depth = 0;
  std::vector<const PathSetEnv::AnnotatedPath*> apath_ptr_list;
  for(auto& apath : apaths) {
    apath_ptr_list.push_back(&apath);
  }
  return calcPathProfile_impl(apath_ptr_list, path_depth, start_vid);
}

void PathSetEnv::calcPathProfile_impl(const std::vector<const PathSetEnv::AnnotatedPath*>& apath_ptr_list, int path_depth, int vertex_id) {
  assert(!apath_ptr_list.empty() && "Error in PathSetEnv::calcPathProfile_impl(): apath_ptr_list is empty");

  std::vector<int> profile;
  for(auto& apath_ptr : apath_ptr_list) {
    profile.push_back(apath_ptr->getId());
  }
  int point_id = static_cast<int>(pointId2PathProfile.size());
  pointId2PathProfile.push_back(profile);
  pointId2Depth.push_back(path_depth);
  for(int path_id : profile) {
    pathId2Depth2PointId[path_id][path_depth] = point_id;
  }
  // recursive calls
  std::unordered_map<int,std::vector<const PathSetEnv::AnnotatedPath*>> partition;
  for(auto& apath_ptr : apath_ptr_list) {
    if (path_depth < apath_ptr->getPath().size()) {
      auto vid = apath_ptr->getPath().at(path_depth);
      partition[vid].push_back(apath_ptr);
    }
  }
  for(auto& [vid, apath_ptr_list_of_vid] : partition) {
    calcPathProfile_impl(apath_ptr_list_of_vid, path_depth+1, vid);
  }
}


void PathSetEnv::calcModActionAddDelEdgeToPath() {
  for(int mod_id = 0; mod_id < mod_action_list.size(); mod_id++) {
    auto& mod_action = mod_action_list[mod_id];
    isModActionAddEdgeToPath.emplace_back(getPathNum(), false);
    isModActionDelEdgeToPath.emplace_back(getPathNum(), false);
    for(int path_id = 0; path_id < getPathNum(); path_id++) {

      for(auto mod_edge_id : mod_action.getAddList()) {
        bool isFound = false;
        for(auto path_edge_id : pathId2EdgeIdList[path_id]) {
          if (path_edge_id == mod_edge_id) {
            isModActionAddEdgeToPath[mod_id][path_id] = true;
            isFound = true;
            break;
          }
        }
        if (isFound) break;
      }

      for(auto mod_edge_id : mod_action.getDelList()) {
        bool isFound = false;
        for(auto path_edge_id : pathId2EdgeIdList[path_id]) {
          if (path_edge_id == mod_edge_id) {
            isModActionDelEdgeToPath[mod_id][path_id] = true;
            isFound = true;
            break;
          }
        }
        if (isFound) break;
      }

    }
  }

  // debug
//  for(int mod_id = 0; mod_id < mod_action_list.size(); mod_id++) {
//    for(int path_id = 0; path_id < getPathNum(); path_id++) {
//      std::cout << (isModActionAddEdgeToPath[mod_id][path_id]?1:0) << " ";
//    }
//    std::cout << std::endl;
//  }
//  std::cout << std::endl;
//  for(int mod_id = 0; mod_id < mod_action_list.size(); mod_id++) {
//    for(int path_id = 0; path_id < getPathNum(); path_id++) {
//      std::cout << (isModActionDelEdgeToPath[mod_id][path_id]?1:0) << " ";
//    }
//    std::cout << std::endl;
//  }
}


void PathSetEnv::addNewEdge(int vid1, int vid2) {
  if (vertexIdPair2EdgeId[vid1].find(vid2) == vertexIdPair2EdgeId[vid1].end()) {
    int edge_id = edgeId2VertexIdPair.size();
    edgeId2VertexIdPair.emplace_back(vid1, vid2);
    vertexIdPair2EdgeId[vid1][vid2] = edge_id;
  }  // else not a new edge, ignore it.
}


void PathSetEnv::printEdges() {
  int edge_id=0;
  for(auto [v1, v2] : edgeId2VertexIdPair) {
    std::cout << edge_id << "=(" << v1 << "," << v2 << ") ";
    assert(vertexIdPair2EdgeId[v1][v2] == edge_id && "Error in PathSetEnv::printEdges(): error in edgeId2VertexIdPair.");
    edge_id++;
  }
  std::cout << std::endl;

  for(auto& [v1, vertexIdPairToEdgeId2] : vertexIdPair2EdgeId) {
    for(auto [v2, edge_id] : vertexIdPairToEdgeId2) {
      std::cout << edge_id << "=(" << v1 << "," << v2 << ") ";
      assert(edgeId2VertexIdPair[edge_id].first == v1 && edgeId2VertexIdPair[edge_id].second == v2 && "Error in PathSetEnv::printEdges(): error in vertexIdPair2EdgeId.");
    }
  }
  std::cout << std::endl;
}

void PathSetEnv::printProfileTable() {
  int path_id=0;
  for(auto& depthToPointId : pathId2Depth2PointId) {
    std::cout << "Path " << path_id << ":";
    int depth=0;  // no use
    for(auto point_id : depthToPointId) {
      std::cout << " " << point_id;
      depth++;
    }
    std::cout << std::endl;
    path_id++;
  }
  std::cout << std::endl;
  int point_id = 0;
  for(auto& profile : pointId2PathProfile) {
    std::cout << "Point " << point_id << ":";
    for(auto path_id : profile) {
      std::cout << " " << path_id;
    }
    std::cout << std::endl;
    point_id++;
  }
  std::cout << std::endl;
}


// -----------------------------------------------------------------------------------------
// A Graph-based Goal Recognition Environment
// -----------------------------------------------------------------------------------------

Environment::Path GraphEnv::getPathPrefixOfPoint(int point_id) const {    // point_id is a vertex
  DijkstraAlgm dijkstra{graph, getVertex(start_vid), getVertex(point_id)};   // find the shortest path

  Environment::Path path;
  for(bool isFirst = true; auto vertex : dijkstra.getPath()) {
    if (!isFirst) {
      path.push_back(vertex.get().first);
      isFirst = false;
    }
  }
  return path;
}


void GraphEnv::readFromTokens(const std::vector<std::string>& all_token_list) {
  if (!isContainKeyword(all_token_list, "GRAPH_ENV")) {
    throw std::runtime_error("Error: GraphEnv::readFromTokens(): Cannot find GRAPH_ENV");
  }

  auto token_partition = partitionTokensByKeywords(all_token_list, {"GRAPH_ENV", "GRID_LAYOUT", "START_VERTEX", "PATH", "EDGE", "GOAL", "MOD"});

  is_grid_layout = false;
  for(auto& [keyword, tokens] : token_partition) {
    if (keyword == "GRID_LAYOUT") {
      is_grid_layout = true;
      std::tie(num_x, num_y) = parseTwoTokens<int,int>(tokens);
      break;
    }
  }

  for(auto& [keyword, tokens] : token_partition) {
    if (keyword == "START_VERTEX") {
      setStartVertexId(parseOneToken<int>(tokens));
      break;
    }
  }
  for(auto& [keyword, tokens] : token_partition) {
    if (keyword == "PATH") {
      // auto path_name = tokens[0];  // ignore it
      auto path = parseTokenList<int>(remove_front(tokens));
      if (path[0] != getStartVertexId()) throw std::runtime_error("Error: GraphEnv::readFromFile(): wrong starting vertex for PATH");
      addPath(remove_front(path));
    } else if (keyword == "EDGE") {
      auto [ vid1, vid2 ] = parseTwoTokens<int,int>(tokens);
      tmp_edges.push_back({vid1, vid2});
    } else if (keyword == "GOAL") {
      auto [ vid, goals ] = parseOneTokenAndTokenList<int,std::string>(tokens);
      addGoal(vid, goals);
    } // else do nothing
  }

  wrap_up();

  for(auto& [keyword, tokens] : token_partition) {
    if (keyword == "MOD") {
      ModAction mod_action(mod_action_list.size());
      auto token_partition_2 = partitionTokensByKeywords(tokens, {"ADD", "DEL"});
      for(auto& [keyword_2, tokens_2] : token_partition_2) {
        if (keyword_2 == "ADD") {
          if (tokens_2.size() != 2) throw std::runtime_error("Error in GraphEnv::readFromTokens(): MOD ADD has wrong numbers of vertices");
          auto [vid1, vid2] = parseTwoTokens<int,int>(tokens_2);
          addNewEdge(vid1, vid2);
          auto edge_id = getEdgeId(vid1, vid2);
          assert(edge_id >= 0);
          mod_action.addToAddList(edge_id);
        } else if (keyword_2 == "DEL") {
          if (tokens_2.size() != 2) throw std::runtime_error("Error in GraphEnv::readFromTokens(): MOD DEL has wrong numbers of vertices");
          auto [vid1, vid2] = parseTwoTokens<int,int>(tokens_2);
          addNewEdge(vid1, vid2);
          auto edge_id = getEdgeId(vid1, vid2);
          assert(edge_id >= 0);
          mod_action.addToDelList(edge_id);
        }   // else ignore it
      }
      mod_action_list.push_back(mod_action);
    }
  }

  // now we know the set of all edges: edgeId2VertexIdPair
  // need to create the set of active edges
  isActiveEdgeId.resize(getEdgeNum(), false);
  for(auto& edge : graph.edges()) {
    auto vid1 = edge.origVertex().get().first;
    auto vid2 = edge.destVertex().get().first;
    auto edge_id = vertexIdPair2EdgeId[vid1][vid2];
    isActiveEdgeId[edge_id] = true;
  }

}


/*
 * DOT (graph description language) writer
 */

void GraphEnv::writeDotFile(const std::string& filename) {
  std::ofstream out{filename};

  // header
  out << "digraph G {" << std::endl;
  out << R"(  node [shape=circle fixedsize=shape])" << std::endl;
  out << R"(  edge [color="gray"])" << std::endl;

  // draw vertices
  for(auto& [vid, vertex] : vid2vertex) {
    // draw label
    out << "  " << vertex.id() << "[label=\"" << vid;
    if (!vertex.get().second.empty()) {
      bool isFirst = true;
      for(auto& goal : vertex.get().second) {
        out << (isFirst?'\n':' ') << goal;
        isFirst = false;
      }
    }
    out << "\"";
    // set color
    if (start_vid == vid) {
      out << R"(,color="blue",style="bold")";
    } else if (!vertex.get().second.empty()) {
      out << R"(,color="red",style="bold")";
    }
    // set position
    if (vertex_pos_in_dot.contains(vid)) {
      auto [x, y] = vertex_pos_in_dot[vid];
      out << R"(,pos=")" << x << "," << y << R"(!")";
    }
    // closing
    out << "];" << std::endl;
  }

  // draw edges
  for(auto edge : graph.edges()) {
    int id1 = edge.origVertex().id();
    int id2 = edge.destVertex().id();
    out << "  " << id1 << "->" << id2;
    int vid1 = edge.origVertex().get().first;
    int vid2 = edge.destVertex().get().first;
    auto edge_id = getEdgeId(vid1, vid2);
    assert(hasActiveEdge(edge_id));
    out << " [label=\"" << edge_id << "\",color=\"black\"];" << std::endl;
  }

  out << "}" << std::endl;

  out.close();
}


void GraphEnv::putVerticesInGrid(int num_x, int num_y) {
  for(auto& [vid, vertex] : vid2vertex) {
    putVertexInGrid(vid, num_x, num_y);
  }
}


void GraphEnv::addTmpPathToGraph() {
  assert(vid2vertex.contains(start_vid));
  for(auto& path : tmp_paths) {
    int last_vid = start_vid;
    for(auto vid : path) {
      // add vertices
      if (!vid2vertex.contains(vid)) {
        vid2vertex[vid] = graph.addVertex({vid, {}});
      }
      // add edge
      auto v1 = vid2vertex[last_vid];
      auto v2 = vid2vertex[vid];
      if (!graph.hasEdge(v1, v2)) {
        graph.addEdge(v1, v2, 1);   // 1 is the cost of the edge
      }
      addNewEdge(last_vid, vid);  // will check duplicated
      last_vid = vid;
    }
  }
}

void GraphEnv::addTmpEdgeToGraph() {
  for(auto [vid1, vid2] : tmp_edges) {
    // add vertices
    if (!vid2vertex.contains(vid1)) {
      vid2vertex[vid1] = graph.addVertex({vid1, {}});
    }
    if (!vid2vertex.contains(vid2)) {
      vid2vertex[vid2] = graph.addVertex({vid2, {}});
    }
    // add edge
    auto v1 = vid2vertex[vid1];
    auto v2 = vid2vertex[vid2];
    if (!graph.hasEdge(v1, v2)) {
      graph.addEdge(v1, v2, 1);   // 1 is the cost of the edge
    }
    addNewEdge(vid1, vid2);  // will check duplicated
  }
}


void GraphEnv::deleteIsolatedVertices() {
  assert(graph.getVertexNum() == vid2vertex.size());
  auto vertices = graph.vertices();
  for(auto vertex : vertices) {
    if (vertex.incomingEVList().empty() && vertex.outgoingEVList().empty()) {
      int vid = vertex.get().first;
      graph.deleteVertex(vertex);
      vid2vertex.erase(vid);
    }
  }
  assert(graph.getVertexNum() == vid2vertex.size());
}


void GraphEnv::assignGoalsToVertices() {
  std::vector<int> missing_vids_in_vidToGoals;
  for(auto& [vid, goals] : vidToGoals) {
    if (vid2vertex.contains(vid)) {
      auto& goal_list = vid2vertex[vid].get().second;
      for(auto& goal : goals) {
        if (std::ranges::find(goal_list, goal) == goal_list.end()) {
          goal_list.push_back(goal);
        }  // ignore duplicated goals
      }
    } else {
      missing_vids_in_vidToGoals.push_back(vid);
    }
  }
  // remove excessive goals
  for(auto vid : missing_vids_in_vidToGoals) {
    vidToGoals.erase(vid);
  }
}


void GraphEnv::addNewEdge(int vid1, int vid2) {
  if (vertexIdPair2EdgeId[vid1].find(vid2) == vertexIdPair2EdgeId[vid1].end()) {
    int edge_id = edgeId2VertexIdPair.size();
    edgeId2VertexIdPair.emplace_back(vid1, vid2);
    vertexIdPair2EdgeId[vid1][vid2] = edge_id;
  }  // else not a new edge, ignore it.
}

