#include "foctl.h"

bool EvaluatorBase::updateSpecMap(std::vector<std::string>& spec, const std::vector<std::string>& eval_keys, const EvalSpecMap& eval_spec_map) {
  assert(spec.size() == eval_keys.size() && "Error in EvaluatorBase::updateSpecMap(): spec and eval_keys have different size");
  bool is_default = true;
  for(auto& [ key, fun_name ] : eval_spec_map) {
    bool isKeyFound = false;
    for(int i=0; i<eval_keys.size(); i++) {
      if (eval_keys[i] == key) {
        isKeyFound = true;
        if (spec[i] != fun_name) {
          spec[i] = fun_name;
          is_default = false;
          break;
        }
      }
    }
    if (!isKeyFound)
      throw FoctlParseError("Error in EvaluatorBase::updateSpecMap(): spec_map key not found: " + key);
  }
  return is_default;
}


EvaluatorBase::EvalSpecMap EvaluatorBase::empty_eval_spec;
