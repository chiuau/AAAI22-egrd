#ifndef CODE_FOCTL_H
#define CODE_FOCTL_H

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <set>
#include <unordered_map>
#include <algorithm>
#include <optional>
#include <memory>

#include "shared.h"
#include "string_processing.h"
#include "name_id_map.h"
#include "foctl_eval.h"



// -----------------------------------------------------------------------------------------
// The interface of CompTree's context
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class Answer;

template<typename T, typename V>
class CompTreeContext {
public:
  virtual std::string getConstantNameFromId(int constant_id) const = 0;
  virtual int getVertexIdOfPointId(int point_id) const = 0;
  virtual std::string getPathStringOfPointId(int point_id) const = 0;
  virtual std::string to_string(const std::shared_ptr<AnswerBase>& ans) const = 0;
  virtual std::string to_string(const T& result) const = 0;
};


// -----------------------------------------------------------------------------------------
// The interface of CompTreeState
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class CompTreeState {
public:
  virtual int getPathDepth() const = 0;
  virtual int getPointId() const = 0;
  virtual bool isTerminal() const = 0;
  virtual std::vector<std::unique_ptr<CompTreeState>> getNextStates() const = 0;
  virtual bool hasGoal(int constant_id) const = 0;
  virtual int getSubstitute(int variable_id) const = 0;    // return -1 if there is no substitute
  virtual int getNumOfConstants() const = 0;
  virtual void substitute(int variable_id, int constant_id) = 0;
  virtual void unsubstitute(int variable_id) = 0;
  virtual std::string getInfo() const = 0;
  virtual const CompTreeContext<T,V>& getCompTreeContext() const = 0;
  virtual void debug() = 0;
};


// -----------------------------------------------------------------------------------------
// CompTreeCache
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class CompTreeCache {
public:

  virtual std::optional<std::pair<bool,T>> get(int foctl_node_id, const CompTreeState<T,V>& state) const = 0;

  virtual void add(int foctl_node_id, const CompTreeState<T,V>& state, std::pair<bool,T> result) = 0;

};


// -----------------------------------------------------------------------------------------
// The interface of Answer
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class Answer : public AnswerBase {
public:
  virtual std::string to_string(const CompTreeContext<T,V>& context) const = 0;
};


// -----------------------------------------------------------------------------------------
// FOCTL_TYPE
// -----------------------------------------------------------------------------------------

enum class FOCTL_TYPE {
  STATEMENT,
  TRUE,
  FALSE,
  NOT,
  AND,
  OR,
  IMPLY,
  IFF,
  ATOM,
  TERM,
  CONSTANT,
  VARIABLE,
  EQUAL,
  FORALL,
  EXIST,
  ALL_FINALLY,
  SOME_FINALLY,
  ALL_GLOBALLY,
  SOME_GLOBALLY,
  ALL_NEXT,
  SOME_NEXT,
  ALL_UNTIL,
  SOME_UNTIL,
  LAST,
};

// -----------------------------------------------------------------------------------------
// Exceptions
// -----------------------------------------------------------------------------------------


class FoctlParseError : public std::runtime_error {
public:
  explicit FoctlParseError(const std::string& what_arg) : std::runtime_error(what_arg) {}
  explicit FoctlParseError(const char* what_arg) : std::runtime_error(what_arg) {}
};

class FoctlEvalError : public std::runtime_error {
public:
  explicit FoctlEvalError(const std::string& what_arg) : std::runtime_error(what_arg) {}
  explicit FoctlEvalError(const char* what_arg) : std::runtime_error(what_arg) {}
};


// -----------------------------------------------------------------------------------------
// Evaluator's base class
// -----------------------------------------------------------------------------------------

class EvaluatorBase {
public:
  using EvalSpecMap = std::unordered_map<std::string,std::string>;

  bool is_default;
  std::string spec_desc;
  std::string full_spec_desc;

  static EvalSpecMap empty_eval_spec;

protected:

  bool updateSpecMap(std::vector<std::string>& spec, const std::vector<std::string>& eval_keys, const EvalSpecMap& eval_spec_map);

};


// -----------------------------------------------------------------------------------------
// Statement Description
// -----------------------------------------------------------------------------------------

struct StatementDescription {
  std::string description;
  std::vector<std::string> footnotes;

  struct Option {
    bool is_show_eval = true;
    bool is_inline_eval = false;
    bool is_show_default_eval = true;
    bool is_show_cv_id = false;
    bool is_show_node_id = false;
  };

  static std::string toNodeIdStr(int node_id, int str_width) {
    std::stringstream ss;
    ss << std::setw(str_width) << std::setfill('0') << node_id;
    return ss.str();
  }

  static std::string toFootnoteLabel(const std::string& node_id_str) {
    return "\"+e" + node_id_str + "+\"";
  }

  static std::string toFootnoteDefinition(const std::string& node_id_str, const std::string& evalulator_spec_desc) {
    return "e" + node_id_str + " = \"" + evalulator_spec_desc + "\"";
  }

  static std::string toSpecDesc(const std::vector<std::string>& eval_keys, const std::vector<std::string>& spec, const std::vector<std::string>& default_spec) {
    assert(eval_keys.size() == default_spec.size() && "Error in EvaluatorBase::makeSpecDesc(): eval_keys and default_spec have different size");
    assert(eval_keys.size() == spec.size() && "Error in EvaluatorBase::makeSpecDesc(): eval_keys and spec have different size");
    std::string s;
    bool isFirst = true;
    for(int i=0; i<eval_keys.size(); i++) {
      if (default_spec[i] != spec[i]) {
        if (!isFirst) s += ",";
        s += eval_keys[i] + "=" + spec[i];
        isFirst = false;
      }
    }
    return s;
  }

  static std::string toFullSpecDesc(const std::vector<std::string>& eval_keys, const std::vector<std::string>& spec){
    assert((eval_keys.size() == spec.size()) && "Error in StatementDescription::toFullSpecDesc(): not enough argument in spec");
    std::string s;
    for(int i=0; i<eval_keys.size(); i++) {
      if (i!=0) s += ", ";
      s += eval_keys[i] + "=" + spec[i];
    }
    return s;
  }

  friend std::ostream &operator<<(std::ostream &out, const StatementDescription& desc) {
    if (desc.footnotes.empty()) {
      out << desc.description;
    } else {
      out << "\"" << desc.description << "\"" << std::endl;
      out << "where" << std::endl;
      for (const std::string &footnote : desc.footnotes) {
        out << "  std::string " << footnote << ";" << std::endl;
      }
//      out << "  return \"" << desc.description << "\";" << std::endl;
    }
    return out;
  }

};

// -----------------------------------------------------------------------------------------
// Statement
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class Statement {
protected:
  int node_id;
  std::string node_id_str;

  std::vector<int> bound_variable_ids;
  std::set<int> free_variable_ids;

public:

  explicit Statement(int node_id, const std::string node_id_str = "") : node_id{node_id}, node_id_str{node_id_str} {}
  virtual ~Statement() = default;

  virtual FOCTL_TYPE getType() const { return FOCTL_TYPE::STATEMENT; }
  int getNodeId() const { return node_id; };

  const std::vector<int>& getBoundVariableIDs() const { return bound_variable_ids; }
  const std::set<int>& getFreeVariableIDs() const { return free_variable_ids; }

  std::unique_ptr<Statement<T,V>> clone() const { return std::unique_ptr<Statement<T,V>>(doClone()); }

  virtual void setEvalSpecByDbKey(const std::string& spec_db_key) = 0;

  virtual void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) = 0;
  virtual void setNodeIdStr(int node_id_str_width) = 0;
  virtual const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) = 0;

  std::pair<bool,T> evaluate(CompTreeState<T,V>& state, CompTreeCache<T,V>& cache) {
    return evaluate(state, 0, cache);
  }

  std::pair<bool,T> evaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) {
    if (IS_SHOW_COMP_TREE) {
      std::cout << indent(search_depth) << state.getInfo() << "  " << *this << std::endl;
    }

    if (auto r_opt = cache.get(node_id, state)) {
      auto result = *r_opt;

      if (IS_SHOW_COMP_TREE) {
        std::cout << indent(search_depth) << "->  " << result.first << "," << state.getCompTreeContext().to_string(result.second) << " cached" << std::endl;
      }

      return result;
    } else {
      auto result = doEvaluate(state, search_depth, cache);
      cache.add(node_id, state, result);

      if (IS_SHOW_COMP_TREE) {
        std::cout << indent(search_depth) << "->  " << result.first << "," << state.getCompTreeContext().to_string(result.second) << "  <- " << state.getInfo() << "  " << *this << std::endl;
      }

      return result;
    }
  }

  StatementDescription getDescription(const StatementDescription::Option opt = StatementDescription::Option()) {
    StatementDescription desc = calcDescription(opt);
    if (opt.is_show_node_id) desc.description = "#" + node_id_str + ":" + desc.description;
    std::ranges::sort(desc.footnotes);
    return desc;
  }

  friend std::ostream& operator<<(std::ostream &out, Statement<T,V> &p) {
    out << p.getDescription({false, true, true, false, false});
    return out;
  }

private:
  virtual Statement* doClone() const = 0;
  virtual StatementDescription calcDescription(const StatementDescription::Option& opt) const = 0;
  virtual std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) = 0;
};


// -----------------------------------------------------------------------------------------
// True
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class TrueStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  class TinyAnswer : public Answer<T,V> {
  public:
    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return "True";
    }
  };

  struct Evaluator : public EvaluatorBase {
    ZeroEvalFun<T,V> true_eval;

    const std::vector<std::string> eval_keys{"true_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"depth"}},
        {"max_value", {"depth"}},
        {"depth_if_true_else_max_limit", {"depth"}},
        {"depth_if_true_else_min_limit", {"depth"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 1)
        throw FoctlParseError("Error in TrueStatement::Evaluator(): Too many arguments in " + spec_desc);
      true_eval = makeZeroEvalFun<T,V>(spec[0]).value_or(nullptr);
      if (!true_eval)
        throw FoctlParseError("Error in TrueStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  explicit TrueStatement(int node_id, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, evaluator{eval_spec}
  {
    // do nothing
  }

  TrueStatement(const TrueStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::TRUE; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final { node_id = nodes.size(); nodes.push_back(this); }
  void setNodeIdStr(int node_id_str_width) final { node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width); }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    // no free variable
    return Statement<T,V>::free_variable_ids;
  }

private:

  TrueStatement<T,V>* doClone() const final { return new TrueStatement<T,V>(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "True" + eval_str;
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) {
    if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
      std::shared_ptr<Answer<T,V>> ans{new TinyAnswer()};
      return { true, evaluator.true_eval( { state.getPointId(), state.getPathDepth() }, true).setAnswer(ans) };
    } else {
      return { true, evaluator.true_eval( { state.getPointId(), state.getPathDepth() }, true) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// False
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class FalseStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  // no TinyAnswer for FalseStatement

  struct Evaluator : public EvaluatorBase {
    ZeroEvalFun<T,V> false_eval;

    const std::vector<std::string> eval_keys{"false_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"max_limit"}},
        {"max_value", {"min_limit"}},
        {"depth_if_true_else_max_limit", {"max_limit"}},
        {"depth_if_true_else_min_limit", {"min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 1)
        throw FoctlParseError("Error in FalseStatement::Evaluator(): Too many arguments in " + spec_desc);
      false_eval = makeZeroEvalFun<T,V>(spec[0]).value_or(nullptr);
      if (!false_eval)
        throw FoctlParseError("Error in FalseStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  explicit FalseStatement(int node_id, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, evaluator{eval_spec}
  {
    // do nothing
  }

  FalseStatement(const FalseStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::FALSE; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final { node_id = nodes.size(); nodes.push_back(this); }
  void setNodeIdStr(int node_id_str_width) final { node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width); }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    // no free variable
    return Statement<T,V>::free_variable_ids;
  }

private:

  FalseStatement<T,V>* doClone() const final { return new FalseStatement<T,V>(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "False" + eval_str;
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) {
    return { false, evaluator.false_eval({ state.getPointId(), state.getPathDepth() }, false) };
  }

};


// -----------------------------------------------------------------------------------------
// Not
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class NotStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement_ptr;


  class TinyAnswer : public Answer<T,V> {
  public:
    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return "!_";
    }
  };


  struct Evaluator : public EvaluatorBase {
    OneEvalFun<T,V> true_eval;
    OneEvalFun<T,V> false_eval;

    const std::vector<std::string> eval_keys{"true_eval", "false_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"pass", "max_limit"}},
        {"max_value", {"pass", "min_limit"}},
        {"depth_if_true_else_max_limit", {"depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 2)
        throw FoctlParseError("Error in NotStatement::Evaluator(): Too many arguments in " + spec_desc);
      true_eval = makeOneEvalFun<T,V>(spec[0]).value_or(nullptr);
      false_eval = makeOneEvalFun<T,V>(spec[1]).value_or(nullptr);
      if (!true_eval || !false_eval)
        throw FoctlParseError("Error in NotStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  NotStatement(int node_id, std::unique_ptr<Statement<T,V>> statement_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement_ptr{std::move(statement_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  NotStatement(const NotStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement_ptr{other.statement_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::NOT; }

  const Statement<T,V>& getStatement() const { return *statement_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids = statement_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids.begin(), free_var_ids.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  NotStatement* doClone() const final { return new NotStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s = statement_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "(!" + eval_str + (eval_str.empty()?"":" ") + desc_s.description + ")";
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s.footnotes.begin(), desc_s.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) {
    auto [b1, v1] = statement_ptr->evaluate(state, search_depth+1, cache);
    if (b1) {
      return { false, evaluator.false_eval({ state.getPointId(), state.getPathDepth() }, false, { b1, v1 }) };
    } else {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer()};
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, { b1, v1 }).setAnswer(ans) };
      } else {
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, { b1, v1 }) };
      }
    }
  }

};


// -----------------------------------------------------------------------------------------
// And
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class AndStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement1_ptr;
  std::unique_ptr<Statement<T,V>> statement2_ptr;


  class TinyAnswer : public Answer<T,V> {
    std::shared_ptr<Answer<T,V>> ans1;
    std::shared_ptr<Answer<T,V>> ans2;
  public:

    TinyAnswer(std::shared_ptr<Answer<T,V>> ans1, std::shared_ptr<Answer<T,V>> ans2) : ans1{std::move(ans1)}, ans2{std::move(ans2)} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return "(" + ans1->to_string(context) + ") && (" + ans2->to_string(context) + ")" ;
    }
  };


  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    OneEvalFun<T,V> shorted_false_eval;
    TwoEvalFun<T,V> true_eval;
    TwoEvalFun<T,V> false_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "shorted_false_eval", "true_eval", "false_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"true", "max_limit", "min_value_if_true", "max_limit"}},
        {"max_value", {"true", "min_limit", "max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"true", "max_limit", "depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"true", "min_limit", "depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 4)
        throw FoctlParseError("Error in AndStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      shorted_false_eval = makeOneEvalFun<T,V>(spec[1]).value_or(nullptr);
      true_eval = makeTwoEvalFun<T,V>(spec[2]).value_or(nullptr);
      false_eval = makeTwoEvalFun<T,V>(spec[3]).value_or(nullptr);
      if (!shorted_false_eval || !true_eval || !false_eval)
        throw FoctlParseError("Error in AndStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  AndStatement(int node_id, std::unique_ptr<Statement<T,V>> statement1_ptr, std::unique_ptr<Statement<T,V>> statement2_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement1_ptr{std::move(statement1_ptr)}, statement2_ptr{std::move(statement2_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  AndStatement(const AndStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement1_ptr{other.statement1_ptr->clone()}, statement2_ptr{other.statement2_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::AND; }

  const Statement<T,V>& getStatement1() const { return *statement1_ptr; }
  const Statement<T,V>& getStatement2() const { return *statement2_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement1_ptr->setNodeIdByDFS(nodes);
    statement2_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement1_ptr->setNodeIdStr(node_id_str_width);
    statement2_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids_1 = statement1_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    auto& free_var_ids_2 = statement2_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids_1.begin(), free_var_ids_1.end());
    Statement<T,V>::free_variable_ids.insert(free_var_ids_2.begin(), free_var_ids_2.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  AndStatement* doClone() const final { return new AndStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s1 = statement1_ptr->getDescription(opt);
    StatementDescription desc_s2 = statement2_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "(" + desc_s1.description + " &&" + eval_str + " " + desc_s2.description + ")";
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s1.footnotes.begin(), desc_s1.footnotes.end());
    desc.footnotes.insert(desc.footnotes.end(), desc_s2.footnotes.begin(), desc_s2.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    auto [b1, v1] = statement1_ptr->evaluate(state, search_depth+1, cache);
    if (!b1 && evaluator.is_shortcut) {
      return { false, evaluator.shorted_false_eval({ state.getPointId(), state.getPathDepth()} , false, {b1, v1 }) };
    }
    auto [b2, v2] = statement2_ptr->evaluate(state, search_depth+1, cache);
    if (b1 && b2) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()),
                                                       std::static_pointer_cast<Answer<T,V>>(v2.getAnswer()))};
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, {b1, v1 }, {b2, v2 }).setAnswer(ans) };
      } else {
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, {b1, v1 }, {b2, v2 }) };
      }
    } else {
      return { false, evaluator.false_eval({ state.getPointId(), state.getPathDepth() }, false, {b1, v1 }, {b2, v2 }) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// Or
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class OrStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement1_ptr;
  std::unique_ptr<Statement<T,V>> statement2_ptr;


  class TinyAnswer : public Answer<T,V> {
    std::shared_ptr<Answer<T,V>> ans;
  public:

    TinyAnswer(std::shared_ptr<Answer<T,V>> ans) : ans{std::move(ans)} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return "(" + ans->to_string(context) + ") || _" ;
    }
  };


  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    OneEvalFun<T,V> shorted_true_eval;
    TwoEvalFun<T,V> true_eval;
    TwoEvalFun<T,V> false_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "shorted_true_eval", "true_eval", "false_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"true", "pass", "min_value_if_true", "max_limit"}},
        {"max_value", {"true", "pass", "max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"true", "depth", "depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"true", "depth", "depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 4)
        throw FoctlParseError("Error in OrStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      shorted_true_eval = makeOneEvalFun<T,V>(spec[1]).value_or(nullptr);
      true_eval = makeTwoEvalFun<T,V>(spec[2]).value_or(nullptr);
      false_eval = makeTwoEvalFun<T,V>(spec[3]).value_or(nullptr);
      if (!shorted_true_eval || !true_eval || !false_eval)
        throw FoctlParseError("Error in OrStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  OrStatement(int node_id, std::unique_ptr<Statement<T,V>> statement1_ptr, std::unique_ptr<Statement<T,V>> statement2_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement1_ptr{std::move(statement1_ptr)}, statement2_ptr{std::move(statement2_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  OrStatement(const OrStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement1_ptr{other.statement1_ptr->clone()}, statement2_ptr{other.statement2_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::OR; }

  const Statement<T,V>& getStatement1() const { return *statement1_ptr; }
  const Statement<T,V>& getStatement2() const { return *statement2_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement1_ptr->setNodeIdByDFS(nodes);
    statement2_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement1_ptr->setNodeIdStr(node_id_str_width);
    statement2_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids_1 = statement1_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    auto& free_var_ids_2 = statement2_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids_1.begin(), free_var_ids_1.end());
    Statement<T,V>::free_variable_ids.insert(free_var_ids_2.begin(), free_var_ids_2.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  OrStatement* doClone() const final { return new OrStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s1 = statement1_ptr->getDescription(opt);
    StatementDescription desc_s2 = statement2_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "(" + desc_s1.description + " ||" + eval_str + " " + desc_s2.description + ")";
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s1.footnotes.begin(), desc_s1.footnotes.end());
    desc.footnotes.insert(desc.footnotes.end(), desc_s2.footnotes.begin(), desc_s2.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    auto [b1, v1] = statement1_ptr->evaluate(state, search_depth+1, cache);
    if (b1 && evaluator.is_shortcut) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()))};
        return { true, evaluator.shorted_true_eval({ state.getPointId(), state.getPathDepth() }, true, { b1, v1 }).setAnswer(ans) };
      } else {
        return { true, evaluator.shorted_true_eval({ state.getPointId(), state.getPathDepth() }, true, { b1, v1 }) };
      }
    }
    auto [b2, v2] = statement2_ptr->evaluate(state, search_depth+1, cache);
    if (b1 || b2) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(b1?(v1.getAnswer()):(v2.getAnswer())))};
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, { b1, v1 }, { b2, v2 }).setAnswer(ans) };
      } else {
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, { b1, v1 }, { b2, v2 }) };
      }
    } else {
      return { false, evaluator.false_eval({ state.getPointId(), state.getPathDepth() }, false, { b1, v1 }, { b2, v2 }) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// Imply
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class ImplyStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement1_ptr;
  std::unique_ptr<Statement<T,V>> statement2_ptr;


  class TinyAnswer : public Answer<T,V> {
    std::shared_ptr<Answer<T,V>> ans1;
    std::shared_ptr<Answer<T,V>> ans2;
  public:

    TinyAnswer() {}
    TinyAnswer(std::shared_ptr<Answer<T,V>> ans1, std::shared_ptr<Answer<T,V>> ans2) : ans1{std::move(ans1)}, ans2{std::move(ans2)} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      if (ans1) {
        return "(" + ans1->to_string(context) + ") => (" + ans2->to_string(context) + ")";
      } else {
        return "False => _" ;
      }
    }
  };


  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    OneEvalFun<T,V> shorted_true_eval;
    TwoEvalFun<T,V> true_eval;
    TwoEvalFun<T,V> false_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "shorted_true_eval", "true_eval", "false_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"true", "pass", "min_value_if_true", "max_limit"}},
        {"max_value", {"true", "pass", "max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"true", "depth", "depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"true", "depth", "depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 4)
        throw FoctlParseError("Error in ImplyStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      shorted_true_eval = makeOneEvalFun<T,V>(spec[1]).value_or(nullptr);
      true_eval = makeTwoEvalFun<T,V>(spec[2]).value_or(nullptr);
      false_eval = makeTwoEvalFun<T,V>(spec[3]).value_or(nullptr);
      if (!shorted_true_eval || !true_eval || !false_eval)
        throw FoctlParseError("Error in ImplyStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  ImplyStatement(int node_id, std::unique_ptr<Statement<T,V>> statement1_ptr, std::unique_ptr<Statement<T,V>> statement2_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement1_ptr{std::move(statement1_ptr)}, statement2_ptr{std::move(statement2_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  ImplyStatement(const ImplyStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement1_ptr{other.statement1_ptr->clone()}, statement2_ptr{other.statement2_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::IMPLY; }

  const Statement<T,V>& getStatement1() const { return *statement1_ptr; }
  const Statement<T,V>& getStatement2() const { return *statement2_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement1_ptr->setNodeIdByDFS(nodes);
    statement2_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement1_ptr->setNodeIdStr(node_id_str_width);
    statement2_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids_1 = statement1_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    auto& free_var_ids_2 = statement2_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids_1.begin(), free_var_ids_1.end());
    Statement<T,V>::free_variable_ids.insert(free_var_ids_2.begin(), free_var_ids_2.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  ImplyStatement* doClone() const final { return new ImplyStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s1 = statement1_ptr->getDescription(opt);
    StatementDescription desc_s2 = statement2_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "(" + desc_s1.description + " =>" + eval_str + " " + desc_s2.description + ")";
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s1.footnotes.begin(), desc_s1.footnotes.end());
    desc.footnotes.insert(desc.footnotes.end(), desc_s2.footnotes.begin(), desc_s2.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    auto [b1, v1] = statement1_ptr->evaluate(state, search_depth+1, cache);
    if (!b1 && evaluator.is_shortcut) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer()};
        return { true, evaluator.shorted_true_eval({ state.getPointId(), state.getPathDepth() }, true, { b1, v1 }).setAnswer(ans) };
      } else {
        return { true, evaluator.shorted_true_eval({ state.getPointId(), state.getPathDepth() }, true, { b1, v1 }) };
      }
    }
    auto [b2, v2] = statement2_ptr->evaluate(state, search_depth+1, cache);
    if (!b1 || b2) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()),
                                                       std::static_pointer_cast<Answer<T,V>>(v2.getAnswer()))};
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, { b1, v1 }, { b2, v2 }).setAnswer(ans) };
      } else {
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, { b1, v1 }, { b2, v2 }) };
      }
    } else {
      return { false, evaluator.false_eval({ state.getPointId(), state.getPathDepth() }, false, { b1, v1 }, { b2, v2 }) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// Iff
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class IffStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement1_ptr;
  std::unique_ptr<Statement<T,V>> statement2_ptr;


  class TinyAnswer : public Answer<T,V> {
    std::shared_ptr<Answer<T,V>> ans1;
    std::shared_ptr<Answer<T,V>> ans2;
  public:

    TinyAnswer() {}

    TinyAnswer(std::shared_ptr<Answer<T,V>> ans1, std::shared_ptr<Answer<T,V>> ans2) : ans1{std::move(ans1)}, ans2{std::move(ans2)} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      if (ans1) {
        return "(" + ans1->to_string(context) + ") <=> (" + ans2->to_string(context) + ")" ;
      } else {
        return "_ <=> _" ;
      }
    }
  };


  struct Evaluator : public EvaluatorBase {
    TwoEvalFun<T,V> true_eval;
    TwoEvalFun<T,V> false_eval;

    const std::vector<std::string> eval_keys{"true_eval", "false_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"min_value_if_true", "max_limit"}},
        {"max_value", {"max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 2)
        throw FoctlParseError("Error in IffStatement::Evaluator(): Too many arguments in " + spec_desc);
      true_eval = makeTwoEvalFun<T,V>(spec[0]).value_or(nullptr);
      false_eval = makeTwoEvalFun<T,V>(spec[1]).value_or(nullptr);
      if (!true_eval || !false_eval)
        throw FoctlParseError("Error in IffStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  IffStatement(int node_id, std::unique_ptr<Statement<T,V>> statement1_ptr, std::unique_ptr<Statement<T,V>> statement2_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement1_ptr{std::move(statement1_ptr)}, statement2_ptr{std::move(statement2_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  IffStatement(const IffStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement1_ptr{other.statement1_ptr->clone()}, statement2_ptr{other.statement2_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::IFF; }

  const Statement<T,V>& getStatement1() const { return *statement1_ptr; }
  const Statement<T,V>& getStatement2() const { return *statement2_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement1_ptr->setNodeIdByDFS(nodes);
    statement2_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement1_ptr->setNodeIdStr(node_id_str_width);
    statement2_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids_1 = statement1_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    auto& free_var_ids_2 = statement2_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids_1.begin(), free_var_ids_1.end());
    Statement<T,V>::free_variable_ids.insert(free_var_ids_2.begin(), free_var_ids_2.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  IffStatement* doClone() const final { return new IffStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s1 = statement1_ptr->getDescription(opt);
    StatementDescription desc_s2 = statement2_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "(" + desc_s1.description + " <=>" + eval_str + " " + desc_s2.description + ")";
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s1.footnotes.begin(), desc_s1.footnotes.end());
    desc.footnotes.insert(desc.footnotes.end(), desc_s2.footnotes.begin(), desc_s2.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    auto [b1, v1] = statement1_ptr->evaluate(state, search_depth+1, cache);
    auto [b2, v2] = statement2_ptr->evaluate(state, search_depth+1, cache);
    if (b1 == b2) {
      if (b1) {
        if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
          std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()),
                                                         std::static_pointer_cast<Answer<T,V>>(v2.getAnswer()))};
          return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, {b1, v1 }, {b2, v2 }).setAnswer(ans) };
        } else {
          return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, {b1, v1 }, {b2, v2 }) };
        }
      } else {
        if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
          std::shared_ptr<TinyAnswer> ans{new TinyAnswer()};
          return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, {b1, v1 }, {b2, v2 }).setAnswer(ans) };
        } else {
          return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, {b1, v1 }, {b2, v2 }) };
        }
      }
    } else {
      return { false, evaluator.false_eval({ state.getPointId(), state.getPathDepth() }, false, {b1, v1 }, {b2, v2 }) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// Atom
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class Atom : public Statement<T,V> {
protected:
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;
public:
  explicit Atom(int node_id, const std::string node_id_str = "") : Statement<T,V>{node_id, node_id_str} {}
  virtual ~Atom() = default;

  virtual FOCTL_TYPE getType() const { return FOCTL_TYPE::ATOM; }
};


// -----------------------------------------------------------------------------------------
// Term
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class Term : public Atom<T,V> {
protected:
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;
public:
  explicit Term(int node_id, const std::string node_id_str = "") : Atom<T,V>{node_id, node_id_str} {}
  virtual ~Term() = default;

  virtual std::string getName() const = 0;
  virtual FOCTL_TYPE getType() const { return FOCTL_TYPE::TERM; }
};


// -----------------------------------------------------------------------------------------
// Constant
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class Constant final : public Term<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::string name;
  int constant_id;


  class TinyAnswer : public Answer<T,V> {
    int constant_id;
  public:

    TinyAnswer(int constant_id) : constant_id{constant_id} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return context.getConstantNameFromId(constant_id);
    }
  };


  struct Evaluator : public EvaluatorBase {
    OneGoalEvalFun<T,V> goal_found_eval;
    OneGoalEvalFun<T,V> goal_not_found_eval;

    const std::vector<std::string> eval_keys{"goal_found_eval", "goal_not_found_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"depth", "max_limit"}},
        {"max_value", {"depth", "min_limit"}},
        {"depth_if_true_else_max_limit", {"depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 2)
        throw FoctlParseError("Error in Constant::Evaluator(): Too many arguments in " + spec_desc);
      goal_found_eval = makeOneGoalEvalFun<T,V>(spec[0]).value_or(nullptr);
      goal_not_found_eval = makeOneGoalEvalFun<T,V>(spec[1]).value_or(nullptr);
      if (!goal_found_eval || !goal_not_found_eval)
        throw FoctlParseError("Error in Constant::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  Constant(int node_id, std::string name, int constant_id, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Term<T,V>{node_id}, name{name}, constant_id{constant_id}, evaluator{eval_spec}
  {
    // do nothing
  }

  Constant(const Constant& other) :
      Term<T,V>{other.node_id, other.node_id_str}, name{other.name}, constant_id{other.constant_id}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::CONSTANT; }

  std::string getName() const final { return name; }

  int getConstantId() const { return constant_id; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    // no free variable
    return Statement<T,V>::free_variable_ids;
  }

private:

  Constant<T,V>* doClone() const final { return new Constant(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = name + (opt.is_show_cv_id?("=C"+std::to_string(constant_id)):"") + eval_str;
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    if (state.hasGoal(constant_id)) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(constant_id)};
        return { true, evaluator.goal_found_eval({ state.getPointId(), state.getPathDepth() }, true, -1, constant_id).setAnswer(ans) };
      } else {
        return { true, evaluator.goal_found_eval({ state.getPointId(), state.getPathDepth() }, true, -1, constant_id) };
      }
    } else {
      return { false, evaluator.goal_not_found_eval({ state.getPointId(), state.getPathDepth() }, false, -1, constant_id) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// Variable
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class Variable final : public Term<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::string name;
  int variable_id;

  class TinyAnswer : public Answer<T,V> {
    int constant_id;
  public:

    TinyAnswer(int constant_id) : constant_id{constant_id} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return context.getConstantNameFromId(constant_id);
    }
  };

  struct Evaluator : public EvaluatorBase {
    OneGoalEvalFun<T,V> goal_found_eval;
    OneGoalEvalFun<T,V> goal_not_found_eval;

    const std::vector<std::string> eval_keys{"goal_found_eval", "goal_not_found_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"depth", "max_limit"}},
        {"max_value", {"depth", "min_limit"}},
        {"depth_if_true_else_max_limit", {"depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 2)
        throw FoctlParseError("Error in Variable::Evaluator(): Too many arguments in " + spec_desc);
      goal_found_eval = makeOneGoalEvalFun<T,V>(spec[0]).value_or(nullptr);
      goal_not_found_eval = makeOneGoalEvalFun<T,V>(spec[1]).value_or(nullptr);
      if (!goal_found_eval || !goal_not_found_eval)
        throw FoctlParseError("Error in Variable::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  Variable(int node_id, std::string name, int variable_id, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Term<T,V>{node_id}, name{name}, variable_id{variable_id}, evaluator{eval_spec}
  {
    // do nothing
  }

  Variable(const Variable& other) :
      Term<T,V>{other.node_id, other.node_id_str}, name{other.name}, variable_id{other.variable_id}, evaluator{other.evaluator}
  {
    // do nothing
  }

  std::string getName() const final { return name; }
  int getVariableId() const { return variable_id; }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::VARIABLE; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    Statement<T,V>::free_variable_ids = { variable_id };
    return Statement<T,V>::free_variable_ids;
  }

private:

  Variable<T,V>* doClone() const final { return new Variable(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = name + (opt.is_show_cv_id?("=v"+std::to_string(variable_id)):"") + eval_str;
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    int constant_id = state.getSubstitute(variable_id);
    if (constant_id >= 0) {
      if (state.hasGoal(constant_id)) {
        if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
          std::shared_ptr<TinyAnswer> ans{new TinyAnswer(constant_id)};
          return { true, evaluator.goal_found_eval({ state.getPointId(), state.getPathDepth() }, true, variable_id, constant_id).setAnswer(ans) };
        } else {
          return { true, evaluator.goal_found_eval({ state.getPointId(), state.getPathDepth() }, true, variable_id, constant_id) };
        }
      } else {
        return { false, evaluator.goal_not_found_eval({ state.getPointId(), state.getPathDepth() }, false, variable_id, constant_id) };
      }
    } else {  // no substitute found
      throw FoctlEvalError("Error in Variable::doEvaluate(): free variable " + std::to_string(variable_id) + " exists.");
    }
  }

};


// -----------------------------------------------------------------------------------------
// Equal
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class Equal final : public Atom<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Term<T,V>> term1_ptr;
  std::unique_ptr<Term<T,V>> term2_ptr;



  class TinyAnswer : public Answer<T,V> {
    int constant_id;
  public:

    TinyAnswer(int constant_id) : constant_id{constant_id} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return context.getConstantNameFromId(constant_id) + "==" + context.getConstantNameFromId(constant_id);
    }
  };


  struct Evaluator : public EvaluatorBase {
    OneGoalEvalFun<T,V> goal_equal_eval;
    ZeroEvalFun<T,V> goal_not_equal_eval;

    const std::vector<std::string> eval_keys{"goal_equal_eval", "goal_not_equal_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"depth", "max_limit"}},
        {"max_value", {"depth", "min_limit"}},
        {"depth_if_true_else_max_limit", {"depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 2)
        throw FoctlParseError("Error in Equal::Evaluator(): Too many arguments in " + spec_desc);
      goal_equal_eval = makeOneGoalEvalFun<T,V>(spec[0]).value_or(nullptr);
      goal_not_equal_eval = makeZeroEvalFun<T,V>(spec[1]).value_or(nullptr);
      if (!goal_equal_eval || !goal_not_equal_eval)
        throw FoctlParseError("Error in Equal::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  Equal(int node_id, std::unique_ptr<Term<T,V>> term1_ptr, std::unique_ptr<Term<T,V>> term2_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Atom<T,V>{node_id}, term1_ptr{std::move(term1_ptr)}, term2_ptr{std::move(term2_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  Equal(const Equal& other) :
      Atom<T,V>{other.node_id, other.node_id_str}, term1_ptr{unique_cast<Term<T,V>,Statement<T,V>>(other.term1_ptr->clone())}, term2_ptr{unique_cast<Term<T,V>,Statement<T,V>>(other.term2_ptr->clone())}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::EQUAL; }

  const Term<T,V>& getTerm1() const { return *term1_ptr; }
  const Term<T,V>& getTerm2() const { return *term2_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    term1_ptr->setNodeIdByDFS(nodes);
    term2_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    term1_ptr->setNodeIdStr(node_id_str_width);
    term2_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids_1 = term1_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    auto& free_var_ids_2 = term2_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids_1.begin(), free_var_ids_1.end());
    Statement<T,V>::free_variable_ids.insert(free_var_ids_2.begin(), free_var_ids_2.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  Equal<T,V>* doClone() const final { return new Equal(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_t1 = term1_ptr->getDescription(opt);
    StatementDescription desc_t2 = term2_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "(" + desc_t1.description + " ==" + eval_str + " " + desc_t2.description + ")";
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_t1.footnotes.begin(), desc_t1.footnotes.end());
    desc.footnotes.insert(desc.footnotes.end(), desc_t2.footnotes.begin(), desc_t2.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    int constant_id_1;
    if (term1_ptr->getType() == FOCTL_TYPE::CONSTANT) {
      auto& term1_constant = dynamic_cast<const Constant<T,V>&>(*term1_ptr);
      constant_id_1 = term1_constant.getConstantId();
    } else if (term1_ptr->getType() == FOCTL_TYPE::VARIABLE) {
      auto& term1_variable = dynamic_cast<const Variable<T,V>&>(*term1_ptr);
      constant_id_1 = state.getSubstitute(term1_variable.getVariableId());
      if (constant_id_1 < 0) {  // no substitute found
        throw FoctlEvalError("Error in Equal::doEvaluate(): free variable exists for term1.");
      }
    } else {
      throw FoctlEvalError("Error in Equal::doEvaluate(): term1 is neither a constant or a variable.");
    }

    int constant_id_2;
    if (term2_ptr->getType() == FOCTL_TYPE::CONSTANT) {
      auto& term2_constant = dynamic_cast<const Constant<T,V>&>(*term2_ptr);
      constant_id_2 = term2_constant.getConstantId();
    } else if (term2_ptr->getType() == FOCTL_TYPE::VARIABLE) {
      auto& term2_variable = dynamic_cast<const Variable<T,V>&>(*term2_ptr);
      constant_id_2 = state.getSubstitute(term2_variable.getVariableId());
      if (constant_id_2 < 0) {  // no substitute found
        throw FoctlEvalError("Error in Equal::doEvaluate(): free variable exists for term2.");
      }
    } else {
      throw FoctlEvalError("Error in Equal::doEvaluate(): term2 is neither a constant or a variable.");
    }

    if (constant_id_1 == constant_id_2) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(constant_id_1)};
        return { true, evaluator.goal_equal_eval({ state.getPointId(), state.getPathDepth() }, true, -1, constant_id_1).setAnswer(ans) };
      } else {
        return { true, evaluator.goal_equal_eval({ state.getPointId(), state.getPathDepth() }, true, -1, constant_id_1) };
      }
    } else {
      return { false, evaluator.goal_not_equal_eval({ state.getPointId(), state.getPathDepth() }, false) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// ForAll
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class ForAllStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Variable<T,V>> variable_ptr;
  std::vector<std::pair<int,std::string>> excluded_variable_idnames;
  std::vector<std::pair<int,std::string>> excluded_constant_idnames;
  std::unique_ptr<Statement<T,V>> statement_ptr;


  class TinyAnswer : public Answer<T,V> {
    int constant_id;
    std::shared_ptr<Answer<T,V>> ans;
  public:

    TinyAnswer() : constant_id{-1} {}
    TinyAnswer(int constant_id, std::shared_ptr<Answer<T,V>> ans) : constant_id{constant_id}, ans{std::move(ans)} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      if (constant_id >= 0) {
        return "forall " + context.getConstantNameFromId(constant_id) + " [" + ans->to_string(context) + "]";
      } else {
        return "forall _";
      }
    }
  };


  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    ListGoalEvalFun<T,V> shorted_false_eval;
    ListGoalEvalFun<T,V> true_eval;
    ListGoalEvalFun<T,V> false_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "shorted_false_eval", "true_eval", "false_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"true", "max_limit", "min_value_if_true", "max_limit"}},
        {"max_value", {"true", "min_limit", "max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"true", "max_limit", "depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"true", "min_limit", "depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      is_shortcut = to_bool(spec[0]);
      if (spec.size() > 4)
        throw FoctlParseError("Error in ForAllStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      shorted_false_eval = makeListGoalEvalFun<T,V>(spec[1]).value_or(nullptr);
      true_eval = makeListGoalEvalFun<T,V>(spec[2]).value_or(nullptr);
      false_eval = makeListGoalEvalFun<T,V>(spec[3]).value_or(nullptr);
      if (!shorted_false_eval || !true_eval || !false_eval) {
        throw FoctlParseError("Error in ForAllStatement::Evaluator(): Cannot parse " + spec_desc);
      }
    }
  };

  Evaluator evaluator;

public:

  ForAllStatement(int node_id, std::unique_ptr<Variable<T,V>> variable_ptr, const std::vector<std::pair<int,std::string>>& excluded_variable_idnames, const std::vector<std::pair<int,std::string>>& excluded_constant_idnames, std::unique_ptr<Statement<T,V>> statement_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, variable_ptr{std::move(variable_ptr)}, excluded_variable_idnames{excluded_variable_idnames}, excluded_constant_idnames{excluded_constant_idnames}, statement_ptr{std::move(statement_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  ForAllStatement(const ForAllStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, variable_ptr{unique_cast<Variable<T,V>,Statement<T,V>>(other.variable_ptr->clone())}, excluded_variable_idnames{other.excluded_variable_idnames}, excluded_constant_idnames{other.excluded_constant_idnames}, statement_ptr{other.statement_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::FORALL; }

  const Variable<T,V>& getVariable() const { return *variable_ptr; }
  const Statement<T,V>& getStatement() const { return *statement_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    // variable_ptr->setNodeIdByDFS(nodes);
    statement_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    // variable_ptr->setNodeIdStr(node_id_str_width);
    statement_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    bound_variable_ids.push_back(variable_ptr->getVariableId());
    variable_ptr->calcBoundFreeVariableIDs(bound_variable_ids);  // ignore the returned free variables, this variable is not a statement
    Statement<T,V>::free_variable_ids = statement_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.erase(variable_ptr->getVariableId());  // the variable is no longer free
    return Statement<T,V>::free_variable_ids;
  }

private:

  ForAllStatement* doClone() const final { return new ForAllStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription::Option opt_v = opt;
    opt_v.is_show_eval = false;
    StatementDescription desc_v = variable_ptr->getDescription(opt_v);
    StatementDescription desc_s = statement_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "(forall" + eval_str + " " + desc_v.description + calcExcludedCVDescription() + " [" + desc_s.description + "])";
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    // desc.footnotes.insert(desc.footnotes.end(), desc_v.footnotes.begin(), desc_v.footnotes.end());
    desc.footnotes.insert(desc.footnotes.end(), desc_s.footnotes.begin(), desc_s.footnotes.end());
    return desc;
  }

  std::string calcExcludedCVDescription() const {
    if (excluded_variable_idnames.empty() && excluded_constant_idnames.empty()) return "";
    std::string s = " not_in {";
    for(bool isFirst=true; auto& [ vid, vname ] : excluded_variable_idnames) {
      if (!isFirst) s += ",";
      s += vname;
      isFirst=false;
    }
    if (!excluded_variable_idnames.empty() && !excluded_constant_idnames.empty()) s += ",";
    for(bool isFirst=true; auto& [ cid, cname ] : excluded_constant_idnames) {
      if (!isFirst) s += ",";
      s += cname;
      isFirst=false;
    }
    s += "}";
    return s;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    int variable_id = variable_ptr->getVariableId();
    if (state.getSubstitute(variable_id) >= 0) {
      throw FoctlEvalError("Error in ForAllStatement::doEvaluate(): variable " + variable_ptr->getName() + " is not free.");
    }
    std::vector<std::tuple<bool,T,int>> results;
    bool bb = true;
    for(int constant_id = 0; constant_id < state.getNumOfConstants(); constant_id++) {
      // check excluded variables
      bool isFound = false;
      for(auto& [vid, vname] : excluded_variable_idnames) {
        int cid = state.getSubstitute(vid);
        assert(cid >= 0);
        if (cid == constant_id) { isFound=true; break; }
      }
      for(auto& [cid, vname] : excluded_constant_idnames) {
        if (cid == constant_id) { isFound=true; break; }
      }
      if (!isFound) {  // not excluded
        state.substitute(variable_id, constant_id);
        auto [b, v] = statement_ptr->evaluate(state, search_depth+1, cache);
        state.unsubstitute(variable_id);
        results.emplace_back(b, v, constant_id);
        bb = bb && b;
        if (!b && evaluator.is_shortcut) {
          return { false, evaluator.shorted_false_eval({ state.getPointId(), state.getPathDepth() }, false, variable_id, results) };
        }
      }
    }

    if (bb) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        if (!results.empty()) {
          auto& [b, selected_t, selected_cid] = results[0];
          std::shared_ptr<TinyAnswer> ans{new TinyAnswer(selected_cid, std::static_pointer_cast<Answer<T,V>>(selected_t.getAnswer()))};
          return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, variable_id, results).setAnswer(ans) };
        } else {   // results is empty, still true but no answer
          std::shared_ptr<TinyAnswer> ans{new TinyAnswer()};
          return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, variable_id, results).setAnswer(ans) };
        }
      } else {
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, variable_id, results) };
      }
    } else {
      return { false, evaluator.false_eval({ state.getPointId(), state.getPathDepth() }, false, variable_id, results) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// Exist
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class ExistStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Variable<T,V>> variable_ptr;
  std::vector<std::pair<int,std::string>> excluded_variable_idnames;
  std::vector<std::pair<int,std::string>> excluded_constant_idnames;
  std::unique_ptr<Statement<T,V>> statement_ptr;


  class TinyAnswer : public Answer<T,V> {
    int constant_id;
    std::shared_ptr<Answer<T,V>> ans;
  public:

    TinyAnswer(int constant_id, std::shared_ptr<Answer<T,V>> ans) : constant_id{constant_id}, ans{std::move(ans)} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return "exist " + context.getConstantNameFromId(constant_id) + " [" + ans->to_string(context) + "]";
    }
  };


  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    ListGoalEvalFun<T,V> shorted_true_eval;
    ListGoalEvalFun<T,V> true_eval;
    ListGoalEvalFun<T,V> false_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "shorted_true_eval", "true_eval", "false_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"true", "min_value_if_true", "min_value_if_true", "max_limit"}},
        {"max_value", {"true", "max_value_if_true", "max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"true", "depth", "depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"true", "depth", "depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      is_shortcut = to_bool(spec[0]);
      if (spec.size() > 4)
        throw FoctlParseError("Error in ExistStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      shorted_true_eval = makeListGoalEvalFun<T,V>(spec[1]).value_or(nullptr);
      true_eval = makeListGoalEvalFun<T,V>(spec[2]).value_or(nullptr);
      false_eval = makeListGoalEvalFun<T,V>(spec[3]).value_or(nullptr);
      if (!shorted_true_eval || !true_eval || !false_eval)
        throw FoctlParseError("Error in ExistStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  ExistStatement(int node_id, std::unique_ptr<Variable<T,V>> variable_ptr, const std::vector<std::pair<int,std::string>>& excluded_variable_idnames, const std::vector<std::pair<int,std::string>>& excluded_constant_idnames, std::unique_ptr<Statement<T,V>> statement_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, variable_ptr{std::move(variable_ptr)}, statement_ptr{std::move(statement_ptr)}, excluded_variable_idnames{excluded_variable_idnames}, excluded_constant_idnames{excluded_constant_idnames}, evaluator{eval_spec}
  {
    // do nothing
  }

  ExistStatement(const ExistStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, variable_ptr{unique_cast<Variable<T,V>,Statement<T,V>>(other.variable_ptr->clone())}, excluded_variable_idnames{other.excluded_variable_idnames}, excluded_constant_idnames{other.excluded_constant_idnames}, statement_ptr{other.statement_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::EXIST; }

  const Variable<T,V>& getVariable() const { return *variable_ptr; }
  const Statement<T,V>& getStatement() const { return *statement_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    // variable_ptr->setNodeIdByDFS(nodes);
    statement_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    // variable_ptr->setNodeIdStr(node_id_str_width);
    statement_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    bound_variable_ids.push_back(variable_ptr->getVariableId());
    variable_ptr->calcBoundFreeVariableIDs(bound_variable_ids);  // ignore the returned free variables, this variable is not a statement
    Statement<T,V>::free_variable_ids = statement_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.erase(variable_ptr->getVariableId());  // the variable is no longer free
    return Statement<T,V>::free_variable_ids;
  }

private:

  ExistStatement* doClone() const final { return new ExistStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription::Option opt_v = opt;
    opt_v.is_show_eval = false;
    StatementDescription desc_v = variable_ptr->getDescription(opt_v);
    StatementDescription desc_s = statement_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "(exist" + eval_str + " " + desc_v.description + calcExcludedCVDescription() + " [" + desc_s.description + "])";
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    // desc.footnotes.insert(desc.footnotes.end(), desc_v.footnotes.begin(), desc_v.footnotes.end());
    desc.footnotes.insert(desc.footnotes.end(), desc_s.footnotes.begin(), desc_s.footnotes.end());
    return desc;
  }

  std::string calcExcludedCVDescription() const {
    if (excluded_variable_idnames.empty() && excluded_constant_idnames.empty()) return "";
    std::string s = " not_in {";
    for(bool isFirst=true; auto& [ vid, vname ] : excluded_variable_idnames) {
      if (!isFirst) s += ",";
      s += vname;
      isFirst=false;
    }
    if (!excluded_variable_idnames.empty() && !excluded_constant_idnames.empty()) s += ",";
    for(bool isFirst=true; auto& [ cid, cname ] : excluded_constant_idnames) {
      if (!isFirst) s += ",";
      s += cname;
      isFirst=false;
    }
    s += "}";
    return s;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    int variable_id = variable_ptr->getVariableId();
    if (state.getSubstitute(variable_id) >= 0) {
      throw FoctlEvalError("Error in ExistStatement::doEvaluate(): variable " + variable_ptr->getName() + " is not free.");
    }
    std::vector<std::tuple<bool,T,int>> results;
    bool bb = false;
    for(int constant_id = 0; constant_id < state.getNumOfConstants(); constant_id++) {
      // check excluded variables
      bool isFound = false;
      for(auto& [vid, vname] : excluded_variable_idnames) {
        int cid = state.getSubstitute(vid);
        assert(cid >= 0);
        if (cid == constant_id) { isFound=true; break; }
      }
      for(auto& [cid, cname] : excluded_constant_idnames) {
        if (cid == constant_id) { isFound=true; break; }
      }
      if (!isFound) {  // not excluded
        state.substitute(variable_id, constant_id);
        auto [b, v] = statement_ptr->evaluate(state, search_depth+1, cache);
        state.unsubstitute(variable_id);
        results.emplace_back(b, v, constant_id);
        bb = bb || b;
        if (b && evaluator.is_shortcut) {
          if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
            std::shared_ptr<TinyAnswer> ans{new TinyAnswer(constant_id, std::static_pointer_cast<Answer<T,V>>(v.getAnswer()))};
            return { true, evaluator.shorted_true_eval({ state.getPointId(), state.getPathDepth() }, true, variable_id, results).setAnswer(ans) };
          } else {
            return { true, evaluator.shorted_true_eval({ state.getPointId(), state.getPathDepth() }, true, variable_id, results) };
          }
        }
      }
    }

    if (bb) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        T selected_t; int selected_cid;
        for(auto& [b, t, cid] : results) {
          if (b) {
            selected_t = t; selected_cid = cid; break;
          }
        }
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(selected_cid, std::static_pointer_cast<Answer<T,V>>(selected_t.getAnswer()))};
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, variable_id, results).setAnswer(ans) };
      } else {
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, variable_id, results) };
      }
    } else {
      return { false, evaluator.false_eval({ state.getPointId(), state.getPathDepth() }, false, variable_id, results) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// All Finally
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class AllFinallyStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement_ptr;


  class TinyAnswer : public Answer<T,V> {
    std::shared_ptr<Answer<T,V>> ans;
    int point_id;
    int depth;
  public:

    TinyAnswer(std::shared_ptr<Answer<T,V>> ans, int point_id, int depth) : ans{std::move(ans)}, point_id{point_id}, depth{depth} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return "AF @" + std::to_string(context.getVertexIdOfPointId(point_id)) + " (" + ans->to_string(context) + ")";
    }

    friend class AllFinallyStatement;
  };

  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    OneEvalFun<T,V> true_eval;
    ZeroEvalFun<T,V> terminal_eval;
    ListEvalFun<T,V> shorted_false_rec_eval;
    ListEvalFun<T,V> true_rec_eval;
    ListEvalFun<T,V> false_rec_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "true_eval", "terminal_eval", "shorted_false_rec_eval","true_rec_eval","false_rec_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"true", "pass", "max_limit", "max_limit", "min_value_if_true", "max_limit"}},
        {"max_value", {"true", "pass", "min_limit", "min_limit", "max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"true", "depth", "max_limit", "max_limit", "depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"true", "depth", "min_limit", "min_limit", "depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 6)
        throw FoctlParseError("Error in AllFinallyStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      true_eval = makeOneEvalFun<T,V>(spec[1]).value_or(nullptr);
      terminal_eval = makeZeroEvalFun<T,V>(spec[2]).value_or(nullptr);
      shorted_false_rec_eval = makeListEvalFun<T,V>(spec[3]).value_or(nullptr);
      true_rec_eval = makeListEvalFun<T,V>(spec[4]).value_or(nullptr);
      false_rec_eval = makeListEvalFun<T,V>(spec[5]).value_or(nullptr);
      if (!true_eval || !terminal_eval || !shorted_false_rec_eval || !true_rec_eval || !false_rec_eval)
        throw FoctlParseError("Error in AllFinallyStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  AllFinallyStatement(int node_id, std::unique_ptr<Statement<T,V>> statement_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement_ptr{std::move(statement_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  AllFinallyStatement(const AllFinallyStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement_ptr{other.statement_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::ALL_FINALLY; }

  const Statement<T,V>& getStatement() const { return *statement_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids = statement_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids.begin(), free_var_ids.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  AllFinallyStatement* doClone() const final { return new AllFinallyStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s = statement_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "AF" + eval_str + " " + desc_s.description;
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s.footnotes.begin(), desc_s.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    auto [b1, v1] = statement_ptr->evaluate(state, search_depth+1, cache);
    if (b1) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()), state.getPointId(), state.getPathDepth())};
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, {b1, v1 }).setAnswer(ans) };
      } else {
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, {b1, v1 }) };
      }
    }
    if (state.isTerminal()) {
      return { false, evaluator.terminal_eval({ state.getPointId(), state.getPathDepth() }, false) };
    }
    auto next_states = state.getNextStates();
    std::vector<std::pair<bool,T>> results;
    bool bb = true;
    for(auto& next_state : next_states) {
      auto [b3, v3] = this->evaluate(*next_state, search_depth+1, cache);
      results.emplace_back(b3, v3);
      bb = bb && b3;
      if (!b3 && evaluator.is_shortcut) {
        return { false, evaluator.shorted_false_rec_eval({ state.getPointId(), state.getPathDepth() }, false, results) };
      }
    }
    if (bb) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        int max_d = -1; T max_t;
        for(auto& [b, t] : results) {
          if (b) {
            int d = static_pointer_cast<TinyAnswer>(t.getAnswer())->depth;
            if (d > max_d) { max_d = d; max_t = t; }
          }
        }
        assert(max_d >= 0);
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(max_t.getAnswer()) };

      } else {
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results) };
      }
    } else {
      return { false, evaluator.false_rec_eval({ state.getPointId(), state.getPathDepth() }, false, results) };
    }
  }

};

// -----------------------------------------------------------------------------------------
// Some Finally
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class SomeFinallyStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement_ptr;


  class TinyAnswer : public Answer<T,V> {
    std::shared_ptr<Answer<T,V>> ans;
    int point_id;
    int depth;
  public:

    TinyAnswer(std::shared_ptr<Answer<T,V>> ans, int point_id, int depth) : ans{std::move(ans)}, point_id{point_id}, depth{depth} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return "EF @" + std::to_string(context.getVertexIdOfPointId(point_id)) + " (" + ans->to_string(context) + ")";
    }

    friend class SomeFinallyStatement;
  };


  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    OneEvalFun<T,V> true_eval;
    ZeroEvalFun<T,V> terminal_eval;
    ListEvalFun<T,V> shorted_true_rec_eval;
    ListEvalFun<T,V> true_rec_eval;
    ListEvalFun<T,V> false_rec_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "true_eval", "terminal_eval", "shorted_true_rec_eval","true_rec_eval","false_rec_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"true", "pass", "max_limit", "min_value_if_true", "min_value_if_true", "max_limit"}},
        {"max_value", {"true", "pass", "min_limit", "max_value_if_true", "max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"true", "depth", "max_limit", "depth", "depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"true", "depth", "min_limit", "depth", "depth", "min_limit"}},
        {"min_value_no_shortcut", {"false", "pass", "max_limit", "min_value_if_true", "min_value_if_true", "max_limit"}},
        {"max_value_no_shortcut", {"false", "pass", "min_limit", "max_value_if_true", "max_value_if_true", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 6)
        throw FoctlParseError("Error in SomeFinallyStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      true_eval = makeOneEvalFun<T,V>(spec[1]).value_or(nullptr);
      terminal_eval = makeZeroEvalFun<T,V>(spec[2]).value_or(nullptr);
      shorted_true_rec_eval = makeListEvalFun<T,V>(spec[3]).value_or(nullptr);
      true_rec_eval = makeListEvalFun<T,V>(spec[4]).value_or(nullptr);
      false_rec_eval = makeListEvalFun<T,V>(spec[5]).value_or(nullptr);
      if (!true_eval || !terminal_eval || !shorted_true_rec_eval || !true_rec_eval || !false_rec_eval)
        throw FoctlParseError("Error in SomeFinallyStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  SomeFinallyStatement(int node_id, std::unique_ptr<Statement<T,V>> statement_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement_ptr{std::move(statement_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  SomeFinallyStatement(const SomeFinallyStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement_ptr{other.statement_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::SOME_FINALLY; }

  const Statement<T,V>& getStatement() const { return *statement_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids = statement_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids.begin(), free_var_ids.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  SomeFinallyStatement* doClone() const final { return new SomeFinallyStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s = statement_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "EF" + eval_str + " " + desc_s.description;
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s.footnotes.begin(), desc_s.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    auto [b1, v1] = statement_ptr->evaluate(state, search_depth+1, cache);
    if (b1) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()), state.getPointId(), state.getPathDepth())};
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, {b1, v1 }).setAnswer(ans) };
      } else {
        return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true, {b1, v1 }) };
      }
    }
    if (state.isTerminal()) {
      return { false, evaluator.terminal_eval({ state.getPointId(), state.getPathDepth() }, false) };
    }
    auto next_states = state.getNextStates();
    std::vector<std::pair<bool,T>> results;
    bool bb = false;
    for(auto& next_state : next_states) {
      auto [b3, v3] = this->evaluate(*next_state, search_depth+1, cache);
      results.emplace_back(b3, v3);
      bb = bb || b3;
      if (b3 && evaluator.is_shortcut) {
        if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
          return { true, evaluator.shorted_true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(v3.getAnswer()) };
        } else {
          return { true, evaluator.shorted_true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results) };
        }
      }
    }
    if (bb) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        int max_d = -1; T max_t;
        for(auto& [b, t] : results) {
          if (b) {
            int d = static_pointer_cast<TinyAnswer>(t.getAnswer())->depth;
            if (d > max_d) { max_d = d; max_t = t; }
          }
        }
        assert(max_d >= 0);
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(max_t.getAnswer()) };
      } else {
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results) };
      }
    } else {
      return { false, evaluator.false_rec_eval({ state.getPointId(), state.getPathDepth() }, false, results) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// All Globally
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class AllGloballyStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement_ptr;


  class TinyAnswer : public Answer<T,V> {
    std::shared_ptr<Answer<T,V>> ans;
    int point_id;
  public:

    TinyAnswer(std::shared_ptr<Answer<T,V>> ans, int point_id) : ans{std::move(ans)}, point_id{point_id} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return "AG @" + std::to_string(context.getVertexIdOfPointId(point_id)) + " (" + ans->to_string(context) + ")";
    }
  };


  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    OneEvalFun<T,V> false_eval;
    ZeroEvalFun<T,V> terminal_eval;
    ListEvalFun<T,V> shorted_false_rec_eval;
    ListEvalFun<T,V> true_rec_eval;
    ListEvalFun<T,V> false_rec_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "false_eval", "terminal_eval", "shorted_false_rec_eval","true_rec_eval","false_rec_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"true", "max_limit", "depth", "max_limit", "min_value_if_true", "max_limit"}},
        {"max_value", {"true", "min_limit", "depth", "min_limit", "max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"true", "max_limit", "depth", "max_limit", "depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"true", "min_limit", "depth", "min_limit", "depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 6)
        throw FoctlParseError("Error in AllGloballyStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      false_eval = makeOneEvalFun<T,V>(spec[1]).value_or(nullptr);
      terminal_eval = makeZeroEvalFun<T,V>(spec[2]).value_or(nullptr);
      shorted_false_rec_eval = makeListEvalFun<T,V>(spec[3]).value_or(nullptr);
      true_rec_eval = makeListEvalFun<T,V>(spec[4]).value_or(nullptr);
      false_rec_eval = makeListEvalFun<T,V>(spec[5]).value_or(nullptr);
      if (!false_eval || !terminal_eval || !shorted_false_rec_eval || !true_rec_eval || !false_rec_eval)
        throw FoctlParseError("Error in AllGloballyStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  AllGloballyStatement(int node_id, std::unique_ptr<Statement<T,V>> statement_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement_ptr{std::move(statement_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  AllGloballyStatement(const AllGloballyStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement_ptr{other.statement_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::ALL_GLOBALLY; }

  const Statement<T,V>& getStatement() const { return *statement_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids = statement_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids.begin(), free_var_ids.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  AllGloballyStatement* doClone() const final { return new AllGloballyStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s = statement_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "AG" + eval_str + " " + desc_s.description;
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s.footnotes.begin(), desc_s.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    auto [b1, v1] = statement_ptr->evaluate(state, search_depth+1, cache);
    if (!b1) {
      return { false, evaluator.false_eval({ state.getPointId(), state.getPathDepth() }, false, { b1, v1 }) };
    }
    if (state.isTerminal()) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()), state.getPointId())};  // only report v1
        return { true, evaluator.terminal_eval({ state.getPointId(), state.getPathDepth() }, true).setAnswer(ans) };   // must be true
      } else {
        return { true, evaluator.terminal_eval({ state.getPointId(), state.getPathDepth() }, true) };   // must be true
      }
    }
    auto next_states = state.getNextStates();
    std::vector<std::pair<bool,T>> results;
    bool bb = true;
    for(auto& next_state : next_states) {
      auto [b3, v3] = this->evaluate(*next_state, search_depth+1, cache);
      results.emplace_back(b3, v3);
      bb = bb && b3;
      if (!b3 && evaluator.is_shortcut) {
        return { false, evaluator.shorted_false_rec_eval({ state.getPointId(), state.getPathDepth() }, false, results) };
      }
    }
    if (bb) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()), state.getPointId())};  // only report v1
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(ans) };
      } else {
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results) };
      }
    } else {
      return { false, evaluator.false_rec_eval({ state.getPointId(), state.getPathDepth() }, false, results) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// Some Globally
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class SomeGloballyStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement_ptr;


  class TinyAnswer : public Answer<T,V> {
    std::shared_ptr<Answer<T,V>> ans;
    int point_id;
  public:

    TinyAnswer(std::shared_ptr<Answer<T,V>> ans, int point_id) : ans{std::move(ans)}, point_id{point_id} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return "EG @" + std::to_string(context.getVertexIdOfPointId(point_id)) + " (" + ans->to_string(context) + ")";
    }
  };


  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    OneEvalFun<T,V> false_eval;
    ZeroEvalFun<T,V> terminal_eval;
    ListEvalFun<T,V> shorted_true_rec_eval;
    ListEvalFun<T,V> true_rec_eval;
    ListEvalFun<T,V> false_rec_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "false_eval", "terminal_eval", "shorted_true_rec_eval","true_rec_eval","false_rec_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"true", "max_limit", "depth", "min_value_if_true", "min_value_if_true", "max_limit"}},
        {"max_value", {"true", "min_limit", "depth", "max_value_if_true", "max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"true", "max_limit", "depth", "depth", "depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"true", "min_limit", "depth", "depth", "depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 6)
        throw FoctlParseError("Error in SomeGloballyStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      false_eval = makeOneEvalFun<T,V>(spec[1]).value_or(nullptr);
      terminal_eval = makeZeroEvalFun<T,V>(spec[2]).value_or(nullptr);
      shorted_true_rec_eval = makeListEvalFun<T,V>(spec[3]).value_or(nullptr);
      true_rec_eval = makeListEvalFun<T,V>(spec[4]).value_or(nullptr);
      false_rec_eval = makeListEvalFun<T,V>(spec[5]).value_or(nullptr);
      if (!false_eval || !terminal_eval || !shorted_true_rec_eval || !true_rec_eval || !false_rec_eval)
        throw FoctlParseError("Error in SomeGloballyStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  SomeGloballyStatement(int node_id, std::unique_ptr<Statement<T,V>> statement_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement_ptr{std::move(statement_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  SomeGloballyStatement(const SomeGloballyStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement_ptr{other.statement_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::SOME_GLOBALLY; }

  const Statement<T,V>& getStatement() const { return *statement_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids = statement_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids.begin(), free_var_ids.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  SomeGloballyStatement* doClone() const final { return new SomeGloballyStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s = statement_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "EG" + eval_str + " " + desc_s.description;
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s.footnotes.begin(), desc_s.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    auto [b1, v1] = statement_ptr->evaluate(state, search_depth+1, cache);
    if (!b1) {
      return { false, evaluator.false_eval({ state.getPointId(), state.getPathDepth() }, false, {b1, v1 }) };
    }
    if (state.isTerminal()) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()), state.getPointId())};  // only report v1
        return { true, evaluator.terminal_eval({ state.getPointId(), state.getPathDepth() }, true).setAnswer(ans) };
      } else {
        return { true, evaluator.terminal_eval({ state.getPointId(), state.getPathDepth() }, true) };
      }
    }
    auto next_states = state.getNextStates();
    std::vector<std::pair<bool,T>> results;
    bool bb = false;
    for(auto& next_state : next_states) {
      auto [b3, v3] = this->evaluate(*next_state, search_depth+1, cache);
      results.emplace_back(b3, v3);
      bb = bb || b3;
      if (b3 && evaluator.is_shortcut) {
        if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
          std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()), state.getPointId())};  // only report v1
          return { true, evaluator.shorted_true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(ans) };
        } else {
          return { true, evaluator.shorted_true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results) };
        }
      }
    }
    if (bb) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()), state.getPointId())};  // only report v1
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(ans) };
      } else {
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results) };
      }
    } else {
      return { false, evaluator.false_rec_eval({ state.getPointId(), state.getPathDepth() }, false, results) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// All Next
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class AllNextStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement_ptr;


  class TinyAnswer : public Answer<T,V> {
    std::shared_ptr<Answer<T,V>> ans;
  public:

    TinyAnswer() {}
    TinyAnswer(std::shared_ptr<Answer<T,V>> ans) : ans{std::move(ans)} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      if (ans) {
        return "AX (" + ans->to_string(context) + ")";
      } else {
        return "AX END";
      }
    }
  };


  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    ZeroEvalFun<T,V> terminal_eval;
    ListEvalFun<T,V> shorted_false_rec_eval;
    ListEvalFun<T,V> true_rec_eval;
    ListEvalFun<T,V> false_rec_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "terminal_eval", "shorted_false_rec_eval","true_rec_eval","false_rec_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"true", "max_limit", "max_limit", "min_value_if_true", "max_limit"}},
        {"max_value", {"true", "min_limit", "min_limit", "max_value_if_true", "min_limit"}},
        // {"min_value", {"true", "depth", "max_limit", "min_value_if_true", "max_limit"}},
        // {"max_value", {"true", "depth", "min_limit", "max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"true", "max_limit", "max_limit", "depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"true", "min_limit", "min_limit", "depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 5)
        throw FoctlParseError("Error in AllNextStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      terminal_eval = makeZeroEvalFun<T,V>(spec[1]).value_or(nullptr);
      shorted_false_rec_eval = makeListEvalFun<T,V>(spec[2]).value_or(nullptr);
      true_rec_eval = makeListEvalFun<T,V>(spec[3]).value_or(nullptr);
      false_rec_eval = makeListEvalFun<T,V>(spec[4]).value_or(nullptr);
      if (!terminal_eval || !shorted_false_rec_eval || !true_rec_eval || !false_rec_eval)
        throw FoctlParseError("Error in AllNextStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  AllNextStatement(int node_id, std::unique_ptr<Statement<T,V>> statement_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement_ptr{std::move(statement_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  AllNextStatement(const AllNextStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement_ptr{other.statement_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::ALL_NEXT; }

  const Statement<T,V>& getStatement() const { return *statement_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids = statement_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids.begin(), free_var_ids.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  AllNextStatement* doClone() const final { return new AllNextStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s = statement_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "AX" + eval_str + " " + desc_s.description;
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s.footnotes.begin(), desc_s.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    if (state.isTerminal()) {
      return { false, evaluator.terminal_eval({ state.getPointId(), state.getPathDepth() }, false) };
//      std::shared_ptr<TinyAnswer> ans{new TinyAnswer()};
//      return { true, evaluator.terminal_eval({ state.getPointId(), state.getPathDepth() }, true).setAnswer(ans) };
    }
    auto next_states = state.getNextStates();
    std::vector<std::pair<bool,T>> results;
    bool bb = true;
    for(auto& next_state : next_states) {
      auto [b3, v3] = statement_ptr->evaluate(*next_state, search_depth+1, cache);
      results.emplace_back(b3, v3);
      bb = bb && b3;
      if (!b3 && evaluator.is_shortcut) {
        return { false, evaluator.shorted_false_rec_eval({ state.getPointId(), state.getPathDepth() }, false, results) };
      }
    }
    if (bb) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        if (results.empty()) {
          std::shared_ptr<TinyAnswer> ans{new TinyAnswer()};
          return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(ans) };
        } else {  // don't check for max depth
          T selected_t = results[0].second;
          std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(selected_t.getAnswer()))};
          return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(ans) };
        }
      } else {
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results) };
      }
    } else {
      return { false, evaluator.false_rec_eval({ state.getPointId(), state.getPathDepth() }, false, results) };
    }
  }

};

// -----------------------------------------------------------------------------------------
// Some Next
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class SomeNextStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement_ptr;


  class TinyAnswer : public Answer<T,V> {
    std::shared_ptr<Answer<T,V>> ans;
  public:

    TinyAnswer() {}
    TinyAnswer(std::shared_ptr<Answer<T,V>> ans) : ans{std::move(ans)} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      if (ans) {
        return "EX (" + ans->to_string(context) + ")";
      } else {
        return "EX END";
      }
    }
  };


  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    ZeroEvalFun<T,V> terminal_eval;
    ListEvalFun<T,V> shorted_true_rec_eval;
    ListEvalFun<T,V> true_rec_eval;
    ListEvalFun<T,V> false_rec_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "terminal_eval", "shorted_true_rec_eval","true_rec_eval","false_rec_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"true", "max_limit", "min_value_if_true", "min_value_if_true", "max_limit"}},
        {"max_value", {"true", "min_limit", "max_value_if_true", "max_value_if_true", "min_limit"}},
        // {"min_value", {"true", "depth", "min_value_if_true", "min_value_if_true", "max_limit"}},
        // {"max_value", {"true", "depth", "max_value_if_true", "max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"true", "max_limit", "depth", "depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"true", "min_limit", "depth", "depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 5)
        throw FoctlParseError("Error in SomeNextStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      terminal_eval = makeZeroEvalFun<T,V>(spec[1]).value_or(nullptr);
      shorted_true_rec_eval = makeListEvalFun<T,V>(spec[2]).value_or(nullptr);
      true_rec_eval = makeListEvalFun<T,V>(spec[3]).value_or(nullptr);
      false_rec_eval = makeListEvalFun<T,V>(spec[4]).value_or(nullptr);
      if (!terminal_eval || !shorted_true_rec_eval || !true_rec_eval || !false_rec_eval)
        throw FoctlParseError("Error in SomeNextStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  SomeNextStatement(int node_id, std::unique_ptr<Statement<T,V>> statement_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement_ptr{std::move(statement_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  SomeNextStatement(const SomeNextStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement_ptr{other.statement_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::SOME_NEXT; }

  const Statement<T,V>& getStatement() const { return *statement_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids = statement_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids.begin(), free_var_ids.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  SomeNextStatement* doClone() const final { return new SomeNextStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s = statement_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "EX" + eval_str + " " + desc_s.description;
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s.footnotes.begin(), desc_s.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    if (state.isTerminal()) {
      return { false, evaluator.terminal_eval({ state.getPointId(), state.getPathDepth() }, false) };
      // std::shared_ptr<TinyAnswer> ans;
      // return { true, evaluator.terminal_eval({ state.getPointId(), state.getPathDepth() }, true).setAnswer(ans) };
    }
    auto next_states = state.getNextStates();
    std::vector<std::pair<bool,T>> results;
    bool bb = false;
    for(auto& next_state : next_states) {
      auto [b3, v3] = statement_ptr->evaluate(*next_state, search_depth+1, cache);
      results.emplace_back(b3, v3);
      bb = bb || b3;
      if (b3 && evaluator.is_shortcut) {
        if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
          std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v3.getAnswer()))};
          return { true, evaluator.shorted_true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(ans) };
        } else {
          return { true, evaluator.shorted_true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results) };
        }
      }
    }
    if (bb) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        bool isFound; T selected_t;
        for(auto& [b, t] : results) {
          if (b) { isFound = true; selected_t = t; break; }
        }
        assert(isFound);
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(selected_t.getAnswer()))};
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(ans) };
      } else {
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results) };
      }
    } else {
      return { false, evaluator.false_rec_eval({ state.getPointId(), state.getPathDepth() }, false, results) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// All Until
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class AllUntilStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement1_ptr;
  std::unique_ptr<Statement<T,V>> statement2_ptr;


  class TinyAnswer : public Answer<T,V> {
    std::shared_ptr<Answer<T,V>> ans_1;
    int point_id_1;
    std::shared_ptr<Answer<T,V>> ans_2;
    int point_id_2;
    int depth;
  public:

    TinyAnswer(std::shared_ptr<Answer<T,V>> ans_2, int point_id_2, int depth) : ans_2{std::move(ans_2)}, point_id_2{point_id_2}, depth{depth} {}
    TinyAnswer(std::shared_ptr<Answer<T,V>> ans_1, int point_id_1, std::shared_ptr<TinyAnswer> until_ans) : ans_1{std::move(ans_1)}, point_id_1{point_id_1}, ans_2{std::move(until_ans->ans_2)}, point_id_2{until_ans->point_id_2}, depth{until_ans->depth} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      if (ans_1) {
        return "A[@" + std::to_string(context.getVertexIdOfPointId(point_id_1)) + " (" + ans_1->to_string(context) +
               ") U @" + std::to_string(context.getVertexIdOfPointId(point_id_2)) +  " (" + ans_2->to_string(context) + ")]";
      } else {
        return "A[_ U @" + std::to_string(context.getVertexIdOfPointId(point_id_2)) +  " (" + ans_2->to_string(context) + ")]";
      }
    }

    friend class AllUntilStatement;
  };


  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    OneEvalFun<T,V> s2_true_eval;
    OneEvalFun<T,V> s1_false_eval;
    ZeroEvalFun<T,V> terminal_eval;
    ListEvalFun<T,V> shorted_false_rec_eval;
    ListEvalFun<T,V> true_rec_eval;
    ListEvalFun<T,V> false_rec_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "s2_true_eval", "s1_false_eval", "terminal_eval", "shorted_false_rec_eval","true_rec_eval","false_rec_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
      {
        {"min_value", {"true", "pass", "max_limit", "max_limit", "max_limit", "min_value_if_true", "max_limit"}},
        {"max_value", {"true", "pass", "min_limit", "min_limit", "min_limit", "max_value_if_true", "min_limit"}},
        {"depth_if_true_else_max_limit", {"true", "depth", "max_limit", "max_limit", "max_limit", "depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"true", "depth", "min_limit", "min_limit", "min_limit", "depth", "min_limit"}},
      };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 7)
        throw FoctlParseError("Error in AllUntilStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      s2_true_eval = makeOneEvalFun<T,V>(spec[1]).value_or(nullptr);
      s1_false_eval = makeOneEvalFun<T,V>(spec[2]).value_or(nullptr);
      terminal_eval = makeZeroEvalFun<T,V>(spec[3]).value_or(nullptr);
      shorted_false_rec_eval = makeListEvalFun<T,V>(spec[4]).value_or(nullptr);
      true_rec_eval = makeListEvalFun<T,V>(spec[5]).value_or(nullptr);
      false_rec_eval = makeListEvalFun<T,V>(spec[6]).value_or(nullptr);
      if (!s2_true_eval || !s1_false_eval || !terminal_eval || !shorted_false_rec_eval || !true_rec_eval || !false_rec_eval)
        throw FoctlParseError("Error in AllUntilStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  AllUntilStatement(int node_id, std::unique_ptr<Statement<T,V>> statement1_ptr, std::unique_ptr<Statement<T,V>> statement2_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement1_ptr{std::move(statement1_ptr)}, statement2_ptr{std::move(statement2_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  AllUntilStatement(const AllUntilStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement1_ptr{other.statement1_ptr->clone()}, statement2_ptr{other.statement2_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::ALL_UNTIL; }

  const Statement<T,V>& getStatement1() const { return *statement1_ptr; }
  const Statement<T,V>& getStatement2() const { return *statement2_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement1_ptr->setNodeIdByDFS(nodes);
    statement2_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement1_ptr->setNodeIdStr(node_id_str_width);
    statement2_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids_1 = statement1_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    auto& free_var_ids_2 = statement2_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids_1.begin(), free_var_ids_1.end());
    Statement<T,V>::free_variable_ids.insert(free_var_ids_2.begin(), free_var_ids_2.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  AllUntilStatement* doClone() const final { return new AllUntilStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s1 = statement1_ptr->getDescription(opt);
    StatementDescription desc_s2 = statement2_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "A[" + desc_s1.description + " U" + eval_str + " " + desc_s2.description + "]";
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s1.footnotes.begin(), desc_s1.footnotes.end());
    desc.footnotes.insert(desc.footnotes.end(), desc_s2.footnotes.begin(), desc_s2.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    auto [b2, v2] = statement2_ptr->evaluate(state, search_depth + 1, cache);
    if (b2) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v2.getAnswer()), state.getPointId(), state.getPathDepth())};
        return { true, evaluator.s2_true_eval({ state.getPointId(), state.getPathDepth() }, true, { b2, v2 }).setAnswer(ans) };
      } else {
        return { true, evaluator.s2_true_eval({ state.getPointId(), state.getPathDepth() }, true, { b2, v2 }) };
      }
    }
    auto [b1, v1] = statement1_ptr->evaluate(state, search_depth + 1, cache);
    if (!b1) {
      return { false, evaluator.s1_false_eval({ state.getPointId(), state.getPathDepth() }, false, { b1, v1 } ) };
    }  // else b1 must be true
    if (state.isTerminal()) {
      return { false, evaluator.terminal_eval({ state.getPointId(), state.getPathDepth() }, false) };
    }
    auto next_states = state.getNextStates();
    std::vector<std::pair<bool,T>> results;
    bool bb = true;
    for(auto& next_state : next_states) {
      auto [b3, v3] = this->evaluate(*next_state, search_depth+1, cache);
      results.emplace_back(b3, v3);
      bb = bb && b3;
      if (!b3 && evaluator.is_shortcut) {
        return { false, evaluator.shorted_false_rec_eval({ state.getPointId(), state.getPathDepth() }, false, results) };
      }
    }
    if (bb) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        assert(!results.empty());   // it is because this state is not terminal
        int max_d = -1; T max_t;
        for(auto& [b, t] : results) {
          if (b) {
            int d = static_pointer_cast<TinyAnswer>(t.getAnswer())->depth;
            if (d > max_d) { max_d = d; max_t = t; }
          }
        }
        assert(max_d >= 0);

        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()),
                                                       state.getPointId(),
                                                       std::static_pointer_cast<TinyAnswer>(max_t.getAnswer()))};

        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(ans) };

      } else {
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results) };
      }
    } else {
      return { false, evaluator.false_rec_eval({ state.getPointId(), state.getPathDepth() }, false, results) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// Some Until
// -----------------------------------------------------------------------------------------


template<typename T, typename V>
class SomeUntilStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;

  std::unique_ptr<Statement<T,V>> statement1_ptr;
  std::unique_ptr<Statement<T,V>> statement2_ptr;

  class TinyAnswer : public Answer<T,V> {
    std::shared_ptr<Answer<T,V>> ans_1;
    int point_id_1;
    std::shared_ptr<Answer<T,V>> ans_2;
    int point_id_2;
    int depth;
  public:

    TinyAnswer(std::shared_ptr<Answer<T,V>> ans_2, int point_id_2, int depth) : ans_2{std::move(ans_2)}, point_id_2{point_id_2}, depth{depth} {}
    TinyAnswer(std::shared_ptr<Answer<T,V>> ans_1, int point_id_1, std::shared_ptr<TinyAnswer> until_ans) : ans_1{std::move(ans_1)}, point_id_1{point_id_1}, ans_2{std::move(until_ans->ans_2)}, point_id_2{until_ans->point_id_2}, depth{until_ans->depth} {}

    std::string to_string(const CompTreeContext<T,V>& context) const final {
      if (ans_1) {
        return "E[@" + std::to_string(context.getVertexIdOfPointId(point_id_1)) + " (" + ans_1->to_string(context) +
        ") U @" + std::to_string(context.getVertexIdOfPointId(point_id_2)) +  " (" + ans_2->to_string(context) + ")]";
      } else {
        return "E[_ U @" + std::to_string(context.getVertexIdOfPointId(point_id_2)) +  " (" + ans_2->to_string(context) + ")]";
      }
    }

    friend class SomeUntilStatement;
  };

  struct Evaluator : public EvaluatorBase {
    bool is_shortcut;
    OneEvalFun<T,V> s2_true_eval;
    OneEvalFun<T,V> s1_false_eval;
    ZeroEvalFun<T,V> terminal_eval;
    ListEvalFun<T,V> shorted_true_rec_eval;
    ListEvalFun<T,V> true_rec_eval;
    ListEvalFun<T,V> false_rec_eval;

    const std::vector<std::string> eval_keys{"is_shortcut", "s2_true_eval", "s1_false_eval", "terminal_eval", "shorted_true_rec_eval","true_rec_eval","false_rec_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
        {
          {"min_value", {"true", "pass", "max_limit", "max_limit", "min_value_if_true", "min_value_if_true", "max_limit"}},
          {"max_value", {"true", "pass", "min_limit", "min_limit", "max_value_if_true", "max_value_if_true", "min_limit"}},
          {"depth_if_true_else_max_limit", {"true", "depth", "max_limit", "max_limit", "depth", "depth", "max_limit"}},
          {"depth_if_true_else_min_limit", {"true", "depth", "min_limit", "min_limit", "depth", "depth", "min_limit"}},
          {"min_value_no_shortcut", {"false", "pass", "max_limit", "max_limit", "min_value_if_true", "min_value_if_true", "max_limit"}},
          {"max_value_no_shortcut", {"false", "pass", "min_limit", "min_limit", "max_value_if_true", "max_value_if_true", "min_limit"}},
        };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 7)
        throw FoctlParseError("Error in SomeUntilStatement::Evaluator(): Too many arguments in " + spec_desc);
      is_shortcut = to_bool(spec[0]);
      s2_true_eval = makeOneEvalFun<T,V>(spec[1]).value_or(nullptr);
      s1_false_eval = makeOneEvalFun<T,V>(spec[2]).value_or(nullptr);
      terminal_eval = makeZeroEvalFun<T,V>(spec[3]).value_or(nullptr);
      shorted_true_rec_eval = makeListEvalFun<T,V>(spec[4]).value_or(nullptr);
      true_rec_eval = makeListEvalFun<T,V>(spec[5]).value_or(nullptr);
      false_rec_eval = makeListEvalFun<T,V>(spec[6]).value_or(nullptr);
      if (!s2_true_eval || !s1_false_eval || !terminal_eval || !shorted_true_rec_eval || !true_rec_eval || !false_rec_eval)
        throw FoctlParseError("Error in SomeUntilStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  SomeUntilStatement(int node_id, std::unique_ptr<Statement<T,V>> statement1_ptr, std::unique_ptr<Statement<T,V>> statement2_ptr, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, statement1_ptr{std::move(statement1_ptr)}, statement2_ptr{std::move(statement2_ptr)}, evaluator{eval_spec}
  {
    // do nothing
  }

  SomeUntilStatement(const SomeUntilStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, statement1_ptr{other.statement1_ptr->clone()}, statement2_ptr{other.statement2_ptr->clone()}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::SOME_UNTIL; }

  const Statement<T,V>& getStatement1() const { return *statement1_ptr; }
  const Statement<T,V>& getStatement2() const { return *statement2_ptr; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final {
    node_id = nodes.size(); nodes.push_back(this);
    statement1_ptr->setNodeIdByDFS(nodes);
    statement2_ptr->setNodeIdByDFS(nodes);
  }

  void setNodeIdStr(int node_id_str_width) final {
    node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width);
    statement1_ptr->setNodeIdStr(node_id_str_width);
    statement2_ptr->setNodeIdStr(node_id_str_width);
  }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    auto& free_var_ids_1 = statement1_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    auto& free_var_ids_2 = statement2_ptr->calcBoundFreeVariableIDs(bound_variable_ids);
    Statement<T,V>::free_variable_ids.insert(free_var_ids_1.begin(), free_var_ids_1.end());
    Statement<T,V>::free_variable_ids.insert(free_var_ids_2.begin(), free_var_ids_2.end());
    return Statement<T,V>::free_variable_ids;
  }

private:

  SomeUntilStatement* doClone() const final { return new SomeUntilStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    StatementDescription desc_s1 = statement1_ptr->getDescription(opt);
    StatementDescription desc_s2 = statement2_ptr->getDescription(opt);
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "E[" + desc_s1.description + " U" + eval_str + " " + desc_s2.description + "]";
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    desc.footnotes.insert(desc.footnotes.end(), desc_s1.footnotes.begin(), desc_s1.footnotes.end());
    desc.footnotes.insert(desc.footnotes.end(), desc_s2.footnotes.begin(), desc_s2.footnotes.end());
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) final {
    auto [b2, v2] = statement2_ptr->evaluate(state, search_depth + 1, cache);
    if (b2) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v2.getAnswer()), state.getPointId(), state.getPathDepth())};
        return { true, evaluator.s2_true_eval({ state.getPointId(), state.getPathDepth() }, true, {b2, v2 }).setAnswer(ans) };
      } else {
        return { true, evaluator.s2_true_eval({ state.getPointId(), state.getPathDepth() }, true, {b2, v2 }) };
      }
    }
    auto [b1, v1] = statement1_ptr->evaluate(state, search_depth + 1, cache);
    if (!b1) {
      return { false, evaluator.s1_false_eval({ state.getPointId(), state.getPathDepth() }, false, {b1, v1 } ) };
    }
    if (state.isTerminal()) {
      return { false, evaluator.terminal_eval({ state.getPointId(), state.getPathDepth() }, false) };
    }
    auto next_states = state.getNextStates();
    std::vector<std::pair<bool,T>> results;
    bool bb = false;
    for(auto& next_state : next_states) {
      auto [b3, v3] = this->evaluate(*next_state, search_depth+1, cache);
      results.emplace_back(b3, v3);
      bb = bb || b3;
      if (b3 && evaluator.is_shortcut) {
        if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
          std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()),
                                                         state.getPointId(),
                                                         std::static_pointer_cast<TinyAnswer>(v3.getAnswer()))};
          return { true, evaluator.shorted_true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(ans) };
        } else {
          return { true, evaluator.shorted_true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results) };
        }
      }
    }
    if (bb) {
      if constexpr (std::is_same_v<T, AnswerEvalResult<V>>) {
        int max_d = -1; T max_t;
        for(auto& [b, t] : results) {
          if (b) {
            int d = static_pointer_cast<TinyAnswer>(t.getAnswer())->depth;
            if (d > max_d) { max_d = d; max_t = t; }
          }
        }
        assert(max_d >= 0);

        std::shared_ptr<TinyAnswer> ans{new TinyAnswer(std::static_pointer_cast<Answer<T,V>>(v1.getAnswer()),
                                                       state.getPointId(),
                                                       std::static_pointer_cast<TinyAnswer>(max_t.getAnswer()))};

        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results).setAnswer(ans) };
      } else {
        return { true, evaluator.true_rec_eval({ state.getPointId(), state.getPathDepth() }, true, results) };
      }
    } else {
      return { false, evaluator.false_rec_eval({ state.getPointId(), state.getPathDepth() }, false, results) };
    }
  }

};


// -----------------------------------------------------------------------------------------
// Last
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class LastStatement final : public Statement<T,V> {
  using Statement<T,V>::node_id;
  using Statement<T,V>::node_id_str;


  class TinyAnswer : public Answer<T,V> {
  public:
    std::string to_string(const CompTreeContext<T,V>& context) const final {
      return "Last";
    }
  };


  struct Evaluator : public EvaluatorBase {
    ZeroEvalFun<T,V> true_eval;
    ZeroEvalFun<T,V> false_eval;

    const std::vector<std::string> eval_keys{"true_eval", "false_eval"};
    const std::unordered_map<std::string,std::vector<std::string>> spec_db =
        {
        {"min_value", {"depth", "max_limit"}},
        {"max_value", {"depth", "min_limit"}},
        {"depth_if_true_else_max_limit", {"depth", "max_limit"}},
        {"depth_if_true_else_min_limit", {"depth", "min_limit"}},
        };
    const std::vector<std::string> default_spec = spec_db.at("min_value");
    std::vector<std::string> spec = default_spec;

    explicit Evaluator(const EvalSpecMap& eval_spec_map) {
      updateSpecMap(spec, eval_keys, eval_spec_map);
      update();
    }

    void setEvalSpecByDbKey(const std::string& spec_db_key) {
      spec = spec_db.at(spec_db_key);
      update();
    }

    void update() {
      is_default = true;
      for(int i=0; i<default_spec.size(); i++) {
        if (spec[i] != default_spec[i]) { is_default = false; break; }
      }
      spec_desc = StatementDescription::toSpecDesc(eval_keys, spec, default_spec);
      full_spec_desc = StatementDescription::toFullSpecDesc(eval_keys, spec);

      if (spec.size() > 2)
        throw FoctlParseError("Error in LastStatement::Evaluator(): Too many arguments in " + spec_desc);
      true_eval = makeZeroEvalFun<T,V>(spec[0]).value_or(nullptr);
      false_eval = makeZeroEvalFun<T,V>(spec[1]).value_or(nullptr);
      if (!true_eval || !false_eval)
        throw FoctlParseError("Error in LastStatement::Evaluator(): Cannot parse " + spec_desc);
    }
  };

  Evaluator evaluator;

public:

  LastStatement(int node_id, const EvaluatorBase::EvalSpecMap& eval_spec = EvaluatorBase::empty_eval_spec) :
      Statement<T,V>{node_id}, evaluator{eval_spec}
  {
    // do nothing
  }

  LastStatement(const LastStatement& other) :
      Statement<T,V>{other.node_id, other.node_id_str}, evaluator{other.evaluator}
  {
    // do nothing
  }

  FOCTL_TYPE getType() const final { return FOCTL_TYPE::LAST; }

  void setEvalSpecByDbKey(const std::string& spec_db_key) final {
    evaluator.setEvalSpecByDbKey(spec_db_key);
  }

  void setNodeIdByDFS(std::vector<Statement<T,V>*>& nodes) final { node_id = nodes.size(); nodes.push_back(this); }
  void setNodeIdStr(int node_id_str_width) final { node_id_str = StatementDescription::toNodeIdStr(node_id, node_id_str_width); }

  const std::set<int>& calcBoundFreeVariableIDs(std::vector<int> bound_variable_ids) final {
    Statement<T,V>::bound_variable_ids = bound_variable_ids;
    // no free variable
    return Statement<T,V>::free_variable_ids;
  }

private:

  LastStatement* doClone() const final { return new LastStatement(*this); }

  StatementDescription calcDescription(const StatementDescription::Option& opt) const final {
    StatementDescription desc;
    std::string eval_str = opt.is_show_eval ? (opt.is_inline_eval ? (opt.is_show_default_eval ? ("_{" + evaluator.full_spec_desc + "}") : (evaluator.is_default ? "" : ("_{" + evaluator.spec_desc + "}"))) : ("_{" + StatementDescription::toFootnoteLabel(node_id_str) + "}")) : "";
    desc.description = "Last" + eval_str;
    if (opt.is_show_eval && !opt.is_inline_eval) desc.footnotes.push_back(StatementDescription::toFootnoteDefinition(node_id_str, (opt.is_show_default_eval) ? (evaluator.full_spec_desc) : (evaluator.spec_desc)));
    return desc;
  }

  std::pair<bool,T> doEvaluate(CompTreeState<T,V>& state, int search_depth, CompTreeCache<T,V>& cache) {
    if (state.isTerminal()) {
      std::shared_ptr<TinyAnswer> ans{new TinyAnswer()};
      return { true, evaluator.true_eval({ state.getPointId(), state.getPathDepth() }, true).setAnswer(ans) };
    } else {
      return { false, evaluator.false_eval({ state.getPointId(), state.getPathDepth() }, false) };
    }
  }

};

// -----------------------------------------------------------------------------------------
// Statement Record
// -----------------------------------------------------------------------------------------

template<typename T, typename V>
class StatementRecord {

  std::unique_ptr<Statement<T,V>> statement_ptr;
  NameIdMap constantIdMap;
  NameIdMap variableIdMap;
  std::vector<Statement<T,V>*> nodes;

public:

  StatementRecord() {}

  StatementRecord(std::unique_ptr<Statement<T,V>> statement_ptr, const NameIdMap& constantIdMap, const NameIdMap& variableIdMap) :
     statement_ptr{std::move(statement_ptr)}, constantIdMap{constantIdMap}, variableIdMap{variableIdMap}
  {
    prepareStatement();
  }

  Statement<T,V>& getStatement() const { return *statement_ptr; }
  int getNodeNum() const { return nodes.size(); }
  Statement<T,V>& getNode(int node_id) const { return *(nodes.at(node_id)); }
  const NameIdMap& getConstantIdMap() const { return constantIdMap; }
  const NameIdMap& getVariableIdMap() const { return variableIdMap; }
  const std::string& getVariableName(int variable_id) const { return variableIdMap.getNameById(variable_id); }

  void setStatement(std::unique_ptr<Statement<T,V>> statement_ptr) {
    StatementRecord::statement_ptr = std::move(statement_ptr);
    prepareStatement();
  }

  int assignConstantIdToName(const std::string& constant_name) { return constantIdMap.assignIdToName(constant_name); }
  int assignVariableIdToName(const std::string& variable_name) { return variableIdMap.assignIdToName(variable_name); }

  void printBoundFreeVariableIds() {
    for(int node_id = 0; node_id<getNodeNum(); node_id++) {
      auto& node = getNode(node_id);
      std::cout << "Node " << node_id << ": " << node << std::endl;
      std::cout << "  Bound: ";
      __pp__(node.getBoundVariableIDs());
      std::cout << "  Free:  ";
      __pp__(node.getFreeVariableIDs());
    }
    std::cout << std::endl;
  }

private:

  void prepareStatement() {
    statement_ptr->setNodeIdByDFS(nodes);
    int node_id_str_width = std::to_string(nodes.size()-1).size();
    statement_ptr->setNodeIdStr(node_id_str_width);
    statement_ptr->calcBoundFreeVariableIDs({});
    // *** Rename variables if we have time ***
  }

};




#endif //CODE_FOCTL_H
