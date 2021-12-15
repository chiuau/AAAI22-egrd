#ifndef FOCTL_FOCTL_EVAL_H
#define FOCTL_FOCTL_EVAL_H

#include <iostream>
#include <vector>
#include <memory>
#include <optional>

#include "shared.h"

// -----------------------------------------------------------------------------------------
// Data Types for Eval Function
// -----------------------------------------------------------------------------------------

class AnswerBase {};   // just a place holder



class PointInfo {
  int point_id;
  int path_depth;
public:

  PointInfo(int point_id, int path_depth) : point_id{point_id}, path_depth{path_depth} {}

  int getPointId() const { return point_id; }
  int getPathDepth() const { return path_depth; }

  friend std::ostream &operator<<(std::ostream &out, const PointInfo& result) {
    out << "Point " << result.getPointId() << " at Depth " << result.getPathDepth();
    return out;
  }
};


template<typename V>
class PointEvalResult {
protected:
  V value;
  int point_id;
public:

  PointEvalResult() : value{}, point_id{-1} {}
  PointEvalResult(V value, int point_id) : value{value}, point_id{point_id} {}

  // *** required functions ***
  V getValue() const { return value; }
  int getPointId() const { return point_id; }
  virtual std::shared_ptr<AnswerBase> getAnswer() const { return nullptr; }
  virtual PointEvalResult& setAnswer(std::shared_ptr<AnswerBase> ans) { return *this; };
  // **************************

  void setValue(V value) { PointEvalResult::value = value ; }
  void setPointId(int point_id) { PointEvalResult::point_id = point_id ; }

  friend std::ostream &operator<<(std::ostream &out, const PointEvalResult& result) {
    out << result.value;
    if (result.point_id >= 0) {
      out << "@P" << result.point_id;
    }
    return out;
  }
};



template<typename V>
class AnswerEvalResult : public PointEvalResult<V> {

  std::shared_ptr<AnswerBase> ans;

public:

  AnswerEvalResult() {}

  AnswerEvalResult(V value, int point_id) : PointEvalResult<V>(value, point_id) {}

  AnswerEvalResult& setAnswer(std::shared_ptr<AnswerBase> ans) final {
    AnswerEvalResult::ans = std::move(ans); return *this;
  };

  std::shared_ptr<AnswerBase> getAnswer() const final { return ans; }

  friend std::ostream &operator<<(std::ostream &out, const AnswerEvalResult<V>& result) {
    out << dynamic_cast<const PointEvalResult<V>&>(result) << "{ans}";
    return out;
  }

};


// -----------------------------------------------------------------------------------------
// Eval Function Types
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
using ZeroEvalFun = T(*)(const PointInfo& point_info, bool);

template<typename T, typename V>
using OneEvalFun = T(*)(const PointInfo& point_info, bool, std::pair<bool,T>);

template<typename T, typename V>
using TwoEvalFun = T(*)(const PointInfo& point_info, bool, std::pair<bool,T>, std::pair<bool,T>);

template<typename T, typename V>
using ListEvalFun = T(*)(const PointInfo& point_info, bool, const std::vector<std::pair<bool,T>>&);

template<typename T, typename V>
using OneGoalEvalFun = T(*)(const PointInfo& point_info, bool, int, int);  // the second last int is variable_id, which is -1 if there is no associated variable

template<typename T, typename V>
using ListGoalEvalFun = T(*)(const PointInfo& point_info, bool, int, const std::vector<std::tuple<bool,T,int>>&);  // the second last int is variable_id, which is -1 if there is no associated variable


// ---------------------------------------------------------------------------------------------------------
// Eval by passing value directly (only for OneEvalFun)
// ---------------------------------------------------------------------------------------------------------

template<typename T, typename V>
T evalPassValue(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v) {
  return v.second;
}


// ---------------------------------------------------------------------------------------------------------
// Eval by returning min limit
// ---------------------------------------------------------------------------------------------------------

template<typename T, typename V>
T evalMinLimit(const PointInfo& point_info, bool truth_result) {
  return { std::numeric_limits<V>::min(), point_info.getPointId() };
}

template<typename T, typename V>
T evalMinLimit(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v) {
  return { std::numeric_limits<V>::min(), point_info.getPointId() };
}

template<typename T, typename V>
T evalMinLimit(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v1, std::pair<bool,T> v2) {
  return { std::numeric_limits<V>::min(), point_info.getPointId() };
}

template<typename T, typename V>
T evalMinLimit(const PointInfo& point_info, bool truth_result, const std::vector<std::pair<bool,T>>& vs) {
  return { std::numeric_limits<V>::min(), point_info.getPointId() };
}

template<typename T, typename V>
T evalMinLimit(const PointInfo& point_info, bool truth_result, int variable_id, int constant_id) {
  return { std::numeric_limits<V>::min(), point_info.getPointId() };
}

template<typename T, typename V>
T evalMinLimit(const PointInfo& point_info, bool truth_result, int variable_id, const std::vector<std::tuple<bool,T,int>>& vs) {
  return { std::numeric_limits<V>::min(), point_info.getPointId() };
}

// ---------------------------------------------------------------------------------------------------------
// Eval by returning min limit
// ---------------------------------------------------------------------------------------------------------

template<typename T, typename V>
T evalMaxLimit(const PointInfo& point_info, bool truth_result) {
  return { std::numeric_limits<V>::max(), point_info.getPointId() };
}

template<typename T, typename V>
T evalMaxLimit(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v) {
  return { std::numeric_limits<V>::max(), point_info.getPointId() };
}

template<typename T, typename V>
T evalMaxLimit(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v1, std::pair<bool,T> v2) {
  return { std::numeric_limits<V>::max(), point_info.getPointId() };
}

template<typename T, typename V>
T evalMaxLimit(const PointInfo& point_info, bool truth_result, const std::vector<std::pair<bool,T>>& vs) {
  return { std::numeric_limits<V>::max(), point_info.getPointId() };
}

template<typename T, typename V>
T evalMaxLimit(const PointInfo& point_info, bool truth_result, int variable_id, int constant_id) {
  return { std::numeric_limits<V>::max(), point_info.getPointId() };
}

template<typename T, typename V>
T evalMaxLimit(const PointInfo& point_info, bool truth_result, int variable_id, const std::vector<std::tuple<bool,T,int>>& vs) {
  return { std::numeric_limits<V>::max(), point_info.getPointId() };
}


// -----------------------------------------------------------------------------------------
// Eval by returning the depth
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
T evalDepth(const PointInfo& point_info, bool truth_result) {
  return { point_info.getPathDepth(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepth(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v) {
  return { point_info.getPathDepth(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepth(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v1, std::pair<bool,T> v2) {
  return { point_info.getPathDepth(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepth(const PointInfo& point_info, bool truth_result, const std::vector<std::pair<bool,T>>& vs) {
  return { point_info.getPathDepth(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepth(const PointInfo& point_info, bool truth_result, int variable_id, int constant_id) {
  return { point_info.getPathDepth(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepth(const PointInfo& point_info, bool truth_result, int variable_id, const std::vector<std::tuple<bool,T,int>>& vs) {
  return { point_info.getPathDepth(), point_info.getPointId() };
}


// -----------------------------------------------------------------------------------------
// Eval by returning the depth if true else min limit
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
T evalDepthIfTrueElseMinLimit(const PointInfo& point_info, bool truth_result) {
  if (truth_result)
    return { point_info.getPathDepth(), point_info.getPointId() };
  else
    return { std::numeric_limits<V>::min(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepthIfTrueElseMinLimit(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v) {
  if (truth_result)
    return { point_info.getPathDepth(), point_info.getPointId() };
  else
    return { std::numeric_limits<V>::min(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepthIfTrueElseMinLimit(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v1, std::pair<bool,T> v2) {
  if (truth_result)
    return { point_info.getPathDepth(), point_info.getPointId() };
  else
    return { std::numeric_limits<V>::min(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepthIfTrueElseMinLimit(const PointInfo& point_info, bool truth_result, const std::vector<std::pair<bool,T>>& vs) {
  if (truth_result)
    return { point_info.getPathDepth(), point_info.getPointId() };
  else
    return { std::numeric_limits<V>::min(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepthIfTrueElseMinLimit(const PointInfo& point_info, bool truth_result, int variable_id, int constant_id) {
  if (truth_result)
    return { point_info.getPathDepth(), point_info.getPointId() };
  else
    return { std::numeric_limits<V>::min(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepthIfTrueElseMinLimit(const PointInfo& point_info, bool truth_result, int variable_id, const std::vector<std::tuple<bool,T,int>>& vs) {
  if (truth_result)
    return { point_info.getPathDepth(), point_info.getPointId() };
  else
    return { std::numeric_limits<V>::min(), point_info.getPointId() };
}

// -----------------------------------------------------------------------------------------
// Eval by returning the depth if true else min limit
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
T evalDepthIfTrueElseMaxLimit(const PointInfo& point_info, bool truth_result) {
  if (truth_result)
    return { point_info.getPathDepth(), point_info.getPointId() };
  else
    return { std::numeric_limits<V>::max(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepthIfTrueElseMaxLimit(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v) {
  if (truth_result)
    return { point_info.getPathDepth(), point_info.getPointId() };
  else
    return { std::numeric_limits<V>::max(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepthIfTrueElseMaxLimit(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v1, std::pair<bool,T> v2) {
  if (truth_result)
    return { point_info.getPathDepth(), point_info.getPointId() };
  else
    return { std::numeric_limits<V>::max(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepthIfTrueElseMaxLimit(const PointInfo& point_info, bool truth_result, const std::vector<std::pair<bool,T>>& vs) {
  if (truth_result)
    return { point_info.getPathDepth(), point_info.getPointId() };
  else
    return { std::numeric_limits<V>::max(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepthIfTrueElseMaxLimit(const PointInfo& point_info, bool truth_result, int variable_id, int constant_id) {
  if (truth_result)
    return { point_info.getPathDepth(), point_info.getPointId() };
  else
    return { std::numeric_limits<V>::max(), point_info.getPointId() };
}

template<typename T, typename V>
T evalDepthIfTrueElseMaxLimit(const PointInfo& point_info, bool truth_result, int variable_id, const std::vector<std::tuple<bool,T,int>>& vs) {
  if (truth_result)
    return { point_info.getPathDepth(), point_info.getPointId() };
  else
    return { std::numeric_limits<V>::max(), point_info.getPointId() };
}


// ---------------------------------------------------------------------------------------------------------
// Eval by the max true value
// ---------------------------------------------------------------------------------------------------------

template<typename T, typename V>
T evalMaxValueIfTrue(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v1, std::pair<bool,T> v2) {
  // ignore truth_result
  int max_value = std::numeric_limits<V>::min();
  T max_result;
  if (v1.first && v1.second.getValue() > max_value) { max_value = v1.second.getValue(); max_result = v1.second; }
  if (v2.first && v2.second.getValue() > max_value) { max_value = v2.second.getValue(); max_result = v2.second; }
  return max_result;
}

template<typename T, typename V>
T evalMaxValueIfTrue(const PointInfo& point_info, bool truth_result, const std::vector<std::pair<bool,T>>& vs) {
  // ignore truth_result
  int max_value = std::numeric_limits<V>::min();
  T max_result;
  for (auto& [b, r] : vs) {
    if (b) {
      if (r.getValue() > max_value) {
        max_value = r.getValue();
        max_result = r;
      }
    } // else ignore r.
  }
  return max_result;
}

template<typename T, typename V>
T evalMaxValueWithGoalsIfTrue(const PointInfo& point_info, bool truth_result, int variable_id, const std::vector<std::tuple<bool,T,int>>& vsg) {
  int max_value = std::numeric_limits<V>::min();
  T max_result;
  for (auto& [b, r, g] : vsg) {
    if (b) {
      if (r.getValue() > max_value) {
        max_value = r.getValue();
        max_result = r;
      }
    } // else ignore v and g.
  }
  return max_result;
}

// ---------------------------------------------------------------------------------------------------------
// Eval by the min true value
// ---------------------------------------------------------------------------------------------------------

template<typename T, typename V>
T evalMinValueIfTrue(const PointInfo& point_info, bool truth_result, std::pair<bool,T> v1, std::pair<bool,T> v2) {
  // ignore truth_result
  int min_value = std::numeric_limits<V>::max();
  T min_result;
  if (v1.first && v1.second.getValue() < min_value) { min_value = v1.second.getValue(); min_result = v1.second; }
  if (v2.first && v2.second.getValue() < min_value) { min_value = v2.second.getValue(); min_result = v2.second; }
  return min_result;
}

template<typename T, typename V>
T evalMinValueIfTrue(const PointInfo& point_info, bool truth_result, const std::vector<std::pair<bool,T>>& vs) {
  // ignore truth_result
  int min_value = std::numeric_limits<V>::max();
  T min_result;
  for (auto& [b, r] : vs) {
    if (b) {
      if (r.getValue() < min_value) {
        min_value = r.getValue();
        min_result = r;
      }
    } // else ignore r.
  }
  return min_result;
}

template<typename T, typename V>
T evalMinValueWithGoalsIfTrue(const PointInfo& point_info, bool truth_result, int variable_id, const std::vector<std::tuple<bool,T,int>>& vsg) {
  // ignore truth_result
  int min_value = std::numeric_limits<V>::max();
  T min_result;
  for (auto& [b, r, g] : vsg) {
    if (b) {
      if (r.getValue() < min_value) {
        min_value = r.getValue();
        min_result = r;
      }
    } // else ignore v and g.
  }
  return min_result;
}


// ---------------------------------------------------------------------------------------------------------
// Eval Fun Factories
// ---------------------------------------------------------------------------------------------------------

template<typename T, typename V>
std::optional<ZeroEvalFun<T,V>> makeZeroEvalFun(const std::string& fname) {
  if (fname == "min_limit") {
    return evalMinLimit<T,V>;
  } else if (fname == "max_limit") {
    return evalMaxLimit<T,V>;
  } else if (fname == "depth") {
    return evalDepth<T,V>;
  } else if (fname == "depth_if_true_else_min_limit") {
    return evalDepthIfTrueElseMinLimit<T,V>;
  } else if (fname == "depth_if_true_else_max_limit") {
    return evalDepthIfTrueElseMaxLimit<T,V>;
  } else {
    return std::nullopt;
  }
}

template<typename T, typename V>
std::optional<OneEvalFun<T,V>> makeOneEvalFun(const std::string& fname) {
  if (fname == "pass") {
    return evalPassValue<T,V>;
  } else if (fname == "min_limit") {
    return evalMinLimit<T,V>;
  } else if (fname == "max_limit") {
    return evalMaxLimit<T,V>;
  } else if (fname == "depth") {
    return evalDepth<T,V>;
  } else if (fname == "depth_if_true_else_min_limit") {
    return evalDepthIfTrueElseMinLimit<T,V>;
  } else if (fname == "depth_if_true_else_max_limit") {
    return evalDepthIfTrueElseMaxLimit<T,V>;
  } else {
    return std::nullopt;
  }
}

template<typename T, typename V>
std::optional<TwoEvalFun<T,V>> makeTwoEvalFun(const std::string& fname) {
  if (fname == "min_limit") {
    return evalMinLimit<T,V>;
  } else if (fname == "max_limit") {
    return evalMaxLimit<T,V>;
  } else if (fname == "depth") {
    return evalDepth<T,V>;
  } else if (fname == "depth_if_true_else_min_limit") {
    return evalDepthIfTrueElseMinLimit<T,V>;
  } else if (fname == "depth_if_true_else_max_limit") {
    return evalDepthIfTrueElseMaxLimit<T,V>;
  } else if (fname == "min_value_if_true") {
    return evalMinValueIfTrue<T,V>;
  } else if (fname == "max_value_if_true") {
    return evalMaxValueIfTrue<T,V>;
  } else {
    return std::nullopt;
  }
}

template<typename T, typename V>
std::optional<ListEvalFun<T,V>> makeListEvalFun(const std::string& fname) {
  if (fname == "min_limit") {
    return evalMinLimit<T,V>;
  } else if (fname == "max_limit") {
    return evalMaxLimit<T,V>;
  } else if (fname == "depth") {
    return evalDepth<T,V>;
  } else if (fname == "depth_if_true_else_min_limit") {
    return evalDepthIfTrueElseMinLimit<T,V>;
  } else if (fname == "depth_if_true_else_max_limit") {
    return evalDepthIfTrueElseMaxLimit<T,V>;
  } else if (fname == "min_value_if_true") {
    return evalMinValueIfTrue<T,V>;
  } else if (fname == "max_value_if_true") {
    return evalMaxValueIfTrue<T,V>;
  } else {
    return std::nullopt;
  }
}

template<typename T, typename V>
std::optional<OneGoalEvalFun<T,V>> makeOneGoalEvalFun(const std::string& fname) {
  if (fname == "min_limit") {
    return evalMinLimit<T,V>;
  } else if (fname == "max_limit") {
    return evalMaxLimit<T,V>;
  } else if (fname == "depth") {
    return evalDepth<T,V>;
  } else if (fname == "depth_if_true_else_min_limit") {
    return evalDepthIfTrueElseMinLimit<T,V>;
  } else if (fname == "depth_if_true_else_max_limit") {
    return evalDepthIfTrueElseMaxLimit<T,V>;
  } else {
    return std::nullopt;
  }
}

template<typename T, typename V>
std::optional<ListGoalEvalFun<T,V>> makeListGoalEvalFun(const std::string& fname) {
  if (fname == "min_limit") {
    return evalMinLimit<T,V>;
  } else if (fname == "max_limit") {
    return evalMaxLimit<T,V>;
  } else if (fname == "depth") {
    return evalDepth<T,V>;
  } else if (fname == "depth_if_true_else_min_limit") {
    return evalDepthIfTrueElseMinLimit<T,V>;
  } else if (fname == "depth_if_true_else_max_limit") {
    return evalDepthIfTrueElseMaxLimit<T,V>;
  } else if (fname == "min_value_if_true") {
    return evalMinValueWithGoalsIfTrue<T,V>;
  } else if (fname == "max_value_if_true") {
    return evalMaxValueWithGoalsIfTrue<T,V>;
  } else {
    return std::nullopt;
  }
}

/* *********************************************************** */
/* The default choice of T and V                               */
/* *********************************************************** */

using FPSV = int;

#ifdef SHOW_ANSWER
  using FPST = AnswerEvalResult<FPSV>;
#else
  using FPST = PointEvalResult<FPSV>;
#endif


#endif //FOCTL_FOCTL_EVAL_H
