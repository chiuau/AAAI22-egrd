#ifndef FOCTL_DESIGN_H
#define FOCTL_DESIGN_H

#include <vector>
#include <list>

#include "shared.h"
#include "environment.h"
#include "comp_tree.h"


class DesignSearchParameter {
  int max_depth = std::numeric_limits<int>::max();
  bool is_pruned_reduce = true;
public:

  DesignSearchParameter() {}
  DesignSearchParameter(int max_depth, bool is_pruned_reduce) : max_depth{max_depth}, is_pruned_reduce{is_pruned_reduce} {}

  int getMaxDepth() const { return max_depth; }
  bool isPrunedReduce() const { return is_pruned_reduce; }
};


class DesignSearchState {

  std::shared_ptr<const DesignSearchState> parent;
  Environment::EnvState env_state;
  std::vector<int> current_mod_plan;   // short enough to store all of them instead of in the parent

public:

  explicit DesignSearchState(const Environment::EnvState& env_state) : env_state{env_state} {}

  DesignSearchState(std::shared_ptr<const DesignSearchState> other, const Environment::ModAction& mod_action) :
    parent{std::move(other)}, env_state{parent->env_state.apply(mod_action)}, current_mod_plan{parent->current_mod_plan}
  {
    current_mod_plan.push_back(mod_action.getId());
  }

  const std::vector<int>& getCurrentModPlan() const { return current_mod_plan; }

  std::vector<int> findAllApplicableModIds(const std::unique_ptr<Environment>& env, int current_point, bool is_pruned_reduce) const {
    std::vector<int> mod_id_list;
    for(int mod_id = 0; mod_id < env->getModActionNum(); mod_id++) {  // maybe it should be randomized
      auto mod_action = env->getModAction(mod_id);
      if (!env_state.isApplicable(mod_action)) continue;
      if (is_pruned_reduce && !env->isModIdRelevantToPoint(mod_id, current_point)) {
        // __pp__("mod_id ", mod_id, " is irrelevant");
        continue;
      }
      Environment::EnvState new_env_state = env_state.apply(mod_action);
      if (isRepeatedEnvState(new_env_state)) continue;
      mod_id_list.push_back(mod_id);  // got it
    }
    return mod_id_list;
  }

private:

  bool isRepeatedEnvState(const Environment::EnvState& new_env_state) const {
    if (env_state == new_env_state) return true;
    if (parent && parent->isRepeatedEnvState(new_env_state)) return true;
    return false;
  }

};


template<typename T, typename V>
std::pair<T,std::vector<int>> designSearch(std::unique_ptr<Environment>& env, std::unique_ptr<CompTree<FPST,FPSV>>& comp_tree, const DesignSearchParameter& parameter) {
  std::list<std::shared_ptr<const DesignSearchState>> queue;

  V min_value = std::numeric_limits<V>::max();
  T min_result;
  std::vector<int> min_mod_plan;

  queue.emplace_back(new DesignSearchState(env->getEnvState()));  // initial state

  while(!queue.empty()) {
    auto dstate_ptr = queue.front();  // <- lazy

    // __pp__("Process node at depth ", dstate_ptr->getCurrentModPlan().size(), " ", env->convertModPlanToString(dstate_ptr->getCurrentModPlan()));

    // --- apply plan ---
    for(auto mod_id : dstate_ptr->getCurrentModPlan()) {
      assert(env->isModActionApplicable(mod_id));
      env->apply(mod_id);
    }

    if (!env->empty()) {
      auto [ isSolutionFound, result ] = comp_tree->run(env->getPointIdOfStartVertex());

      if (isSolutionFound) {
        // __pp__("Solution Found. Value = ", result.getValue());
        if (result.getValue() < min_value) {
          min_value = result.getValue();
          min_result = result;
          min_mod_plan = dstate_ptr->getCurrentModPlan();
        }
      }
    }

    // even if the solution is not found, we still need to continue to refine the environment

    if (dstate_ptr->getCurrentModPlan().size() < parameter.getMaxDepth()) {
      // __pp__("Expand nodes at depth ", dstate_ptr.getCurrentModPlan().size(), " ", env->convertModPlanToString(dstate_ptr.getCurrentModPlan()));
      for(auto mod_id : dstate_ptr->findAllApplicableModIds(env, min_result.getPointId(), parameter.isPrunedReduce())) {
        // __pp__("M", mod_id);
        auto mod_action = env->getModAction(mod_id);
        queue.emplace_back(new DesignSearchState(dstate_ptr, mod_action));
      }
    }

    // --- rewind plan ---
    for(int i = dstate_ptr->getCurrentModPlan().size() - 1; i >= 0; i--) {
      int mod_id = dstate_ptr->getCurrentModPlan()[i];
      assert(env->isModActionUnapplicable(mod_id));
      env->unapply(mod_id);
    }
    queue.pop_front();  // <- lazy pop
  }

  return { min_result, min_mod_plan };
};


#endif //FOCTL_DESIGN_H
