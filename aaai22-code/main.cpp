#include <iostream>

#include "shared.h"
#include "foctl.h"
#include "environment.h"
#include "comp_tree.h"
#include "gqs_reader.h"
#include "string_processing.h"
#include "design.h"


class FoctlCommandLineArgument : public CommandLineArgument {
  bool is_help = false;
  bool is_verbose = false;
  bool is_write_to_dot = false;
  std::string config_filename;
  std::string env_filename;
  std::string gqs_filename;

  bool is_design;
  DesignSearchParameter ds_parameter;

public:

  FoctlCommandLineArgument(int argc, char *argv[]) :
      CommandLineArgument(argc, argv, { { "-h", 0 }, { "-v", 0 }, { "-dot", 0 }, { "-s", 2 }, {"", 3} })
  {
    if (token_partition.empty() || token_partition.contains("-h")) {
      is_help = true;
      return;
    }
    is_verbose = token_partition.contains("-v");
    is_write_to_dot = token_partition.contains("-dot");
    is_design = token_partition.contains("-s");
    if (is_design) {
      ds_parameter = DesignSearchParameter(std::stoi(token_partition["-s"][0]), to_bool(token_partition["-s"][1]));
    }
    if (token_partition.contains("")) {
      config_filename = token_partition[""][0];
      env_filename = token_partition[""][1];
      gqs_filename = token_partition[""][2];
    }
  }

  bool isHelp() const { return is_help; }
  bool isVerbose() const { return is_verbose; }
  bool isWriteToDot() const { return is_write_to_dot; }
  std::string getConfigFilename() const { return config_filename; }
  std::string getEnvFilename() const { return env_filename; }
  std::string getGqsFilename() const { return gqs_filename; }
  bool isDesign() const { return is_design; }
  const DesignSearchParameter& getDesignSearchParameter() const { return ds_parameter; }

  void printHelp() {
    std::cout << "Usage: " << program_name << " -h"<< std::endl;
    std::cout << "       " << program_name << " [-v] [-dot] [-s depth is_use_pruned_reduce] config_filename env_filename gqs_filename"<< std::endl;
    std::cout << std::endl;
    std::cout << "For example, " << std::endl;
    std::cout << std::endl;
    std::cout << "       " << program_name << " -v -dot -s 4 false config.txt env.txt gqs.txt"<< std::endl;
  }
};


class Config {

  bool is_search_all_path = false;
  bool is_exclude_all_goals_in_search_path = false;
  bool is_use_cache = true;

public:

  explicit Config(std::string filepath) {
    auto all_tokens = readTokensFromFile(filepath);
    auto partition = partitionTokensByKeywords(all_tokens, {"SEARCH_ALL_PATH", "EXCLUDE_ALL_GOALS_IN_SEARCH_PATH", "USE_CACHE"});
    for(auto& [keyword, tokens] : partition) {
      if (keyword == "SEARCH_ALL_PATH") {
        is_search_all_path = parseOneToken<bool>(tokens);
      } else if (keyword == "EXCLUDE_ALL_GOALS_IN_SEARCH_PATH") {
        is_exclude_all_goals_in_search_path = parseOneToken<bool>(tokens);
      } else if (keyword == "USE_CACHE") {
        is_use_cache = parseOneToken<bool>(tokens);
      } // ignore other keywords
    }
  }

  bool isSearchAllPath() const { return is_search_all_path; }
  bool isExcludeAllGoalsInSearchPath() const { return is_exclude_all_goals_in_search_path; }
  bool isUseCache() const { return is_use_cache; }

};


int main(int argc, char *argv[]) {
  Shared::init(0, false);
  auto rng = Shared::getInstance().getRng();

  FoctlCommandLineArgument cl_arg{argc, argv};

  if (cl_arg.isHelp()) {
    cl_arg.printHelp();
    exit(EXIT_SUCCESS);
  }

  /***************************************************************************************************/
  /* (1) Prepare the goal query statement */
  /***************************************************************************************************/

  if (cl_arg.isVerbose())
    std::cout << "File containing the goal query statement: " << cl_arg.getGqsFilename() << std::endl;

  std::string g = trim_copy(readStringFromFile(cl_arg.getGqsFilename()));
  if (cl_arg.isVerbose())
    std::cout << "Goal query statement in the file:     " << g << std::endl;

  auto gqs = makeGoalQueryRecord(g);
  if (cl_arg.isVerbose())
    std::cout << "Goal query statement (after parsing): " << gqs.to_string() << std::endl;

  auto sr = gqs.makeStatementRecord();
  if (cl_arg.isVerbose())
    std::cout << "FO-CTL Statement (after translation): " << sr.getStatement() << std::endl;
  assert(sr.getStatement().getFreeVariableIDs().size() == 0 && "Error: The translated statement has free variables");

  Config config(cl_arg.getConfigFilename());
  auto extended_sr = gqs.makeExtendedStatementRecord(config.isSearchAllPath(), config.isExcludeAllGoalsInSearchPath());
  if (cl_arg.isVerbose())
    std::cout << "Extended FO-CTL Statement for WCD:    " << extended_sr.getStatement() << std::endl;

  if (cl_arg.isWriteToDot()) {
    gqs.writeDotFile(replaceFilenameExtension(cl_arg.getGqsFilename(), "dot"));
  }
  //  std::cout << sr.getStatement().getDescription() << std::endl;

  if (cl_arg.isVerbose())
    std::cout << std::endl;

  /***************************************************************************************************/
  /* (2) Prepare the environment and the comp tree */
  /***************************************************************************************************/

  std::unique_ptr<Environment> env;
  std::unique_ptr<CompTree<FPST,FPSV>> comp_tree;

  std::vector<std::string> tokens = readTokensFromFile(cl_arg.getEnvFilename());
  if (isContainKeyword(tokens, "PATH_ENV")) {
    if (cl_arg.isVerbose())
      std::cout << "Creating path-based environment..." << std::endl;
    env = std::make_unique<PathSetEnv>(tokens);
    //    static_cast<PathSetEnv*>(env.get())->deleteActiveEdge(7);
    //    static_cast<PathSetEnv*>(env.get())->deleteActiveEdge(17);
    comp_tree = std::make_unique<PathSetCompTree<FPST,FPSV>>(*static_cast<PathSetEnv*>(env.get()), extended_sr, config.isUseCache());
  } else if (isContainKeyword(tokens, "GRAPH_ENV")) {
    if (cl_arg.isVerbose())
      std::cout << "Creating graph-based environment..." << std::endl;
    env = std::make_unique<GraphEnv>(tokens);
    //    static_cast<GraphEnv*>(env.get())->deleteActiveEdge(7);
    //    static_cast<GraphEnv*>(env.get())->deleteActiveEdge(17);
    comp_tree = std::make_unique<GraphCompTree<FPST,FPSV>>(*static_cast<GraphEnv*>(env.get()), extended_sr);
  } else {
    throw std::runtime_error("Error: unknown environment in " + cl_arg.getEnvFilename());
  }

  if (cl_arg.isWriteToDot()) {
    env->writeDotFile(replaceFilenameExtension(cl_arg.getEnvFilename(), "dot"));
  }

  if (cl_arg.isVerbose())
    std::cout << std::endl;


  /***************************************************************************************************/
  /* (3) Find the solution */
  /***************************************************************************************************/

  if (!cl_arg.isDesign()) {

    auto [ isSolutionFound, result ] = comp_tree->run(env->getPointIdOfStartVertex());

    if (isSolutionFound) {
      if (cl_arg.isVerbose()) {
        std::cout << "Solution found" << std::endl;
        if constexpr (std::is_same_v<FPST, AnswerEvalResult<FPSV>>) {
          std::cout << "Partial solution: " << comp_tree->getCompTreeContext().to_string(result.getAnswer()) << std::endl;
        }
        std::cout << "Path to one of the worst states: " << comp_tree->getCompTreeContext().getPathStringOfPointId(result.getPointId()) << std::endl;
        std::cout << "Path length to the worst states: " << result.getValue() << std::endl;
        std::cout << "The WCD: " << result.getValue() - 1 << std::endl;
      } else {  // just return to path length
        std::cout << result.getValue() << std::endl;
        std::cout << comp_tree->getCompTreeContext().getPathStringOfPointId(result.getPointId()) << std::endl;
      }
    } else {
      if (cl_arg.isVerbose()) {
        std::cout << "No solution" << std::endl;
      } else {  // return -1 to signal no solution
        std::cout << -1 << std::endl;
      }
    }
  } else {

    if (cl_arg.isVerbose()) {
      std::cout << std::endl;
      std::cout << "Before design... " << std::endl;
    }

    auto [ isSolutionFound, result ] = comp_tree->run(env->getPointIdOfStartVertex());
    if (isSolutionFound) {
      if (cl_arg.isVerbose()) {
        std::cout << "Solution found" << std::endl;
        if constexpr (std::is_same_v<FPST, AnswerEvalResult<FPSV>>) {
          std::cout << "Partial solution: " << comp_tree->getCompTreeContext().to_string(result.getAnswer()) << std::endl;
        }
        std::cout << "Path to one of the worst states: " << comp_tree->getCompTreeContext().getPathStringOfPointId(result.getPointId()) << std::endl;
        std::cout << "Path length to the worst states: " << result.getValue() << std::endl;
        if (result.getValue() > 0) {
          std::cout << "The WCD: " << result.getValue() - 1 << std::endl;
        } else {
          std::cout << "The WCD: " << "undefined" << std::endl;
        }
      } else {  // just return to path length
        std::cout << result.getValue() << std::endl;
        std::cout << comp_tree->getCompTreeContext().getPathStringOfPointId(result.getPointId()) << std::endl;
      }
    } else {
      if (cl_arg.isVerbose()) {
        std::cout << "No solution" << std::endl;
      } else {  // return -1 to signal no solution
        std::cout << -1 << std::endl;
      }
    }

    if (cl_arg.isVerbose()) {
      std::cout << std::endl;
      std::cout << "After design... " << std::endl;
    }

    auto [ best_result, mod_plan ] = designSearch<FPST,FPSV>(env, comp_tree, cl_arg.getDesignSearchParameter());

    if (mod_plan.size() > 0) {
      if (cl_arg.isVerbose()) {
        if constexpr (std::is_same_v<FPST, AnswerEvalResult<FPSV>>) {
          std::cout << "Partial solution: " << comp_tree->getCompTreeContext().to_string(best_result.getAnswer()) << std::endl;
        }
        std::cout << "Path to one of the worst states: " << comp_tree->getCompTreeContext().getPathStringOfPointId(best_result.getPointId()) << std::endl;
        std::cout << "Path length to the worst states: " << best_result.getValue() << std::endl;
        if (best_result.getValue() > 0) {
          std::cout << "The WCD: " << best_result.getValue() - 1 << std::endl;
        } else {
          std::cout << "The WCD: " << "undefined" << std::endl;
        }
        // std::cout << "Modification plan's length: " << mod_plan.size() << std::endl;
        std::cout << "Modification plan: " << env->convertModPlanToString(mod_plan) << std::endl;
      } else {
        std::cout << best_result.getValue() << std::endl;
        std::cout << comp_tree->getCompTreeContext().getPathStringOfPointId(best_result.getPointId()) << std::endl;
      }
    } else {
      if (cl_arg.isVerbose()) {
        std::cout << "No better design solution" << std::endl;
      } else {  // return -1 to signal no design solution
        std::cout << -1 << std::endl;
      }
    }

  }

  return EXIT_SUCCESS;
}

