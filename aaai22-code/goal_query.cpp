#include <memory>
#include "goal_query.h"


// -----------------------------------------------------------------------------------------
// Goal Query Statement Maker
// -----------------------------------------------------------------------------------------
//
//template<typename T, typename V>
//GoalQueryStatement<T,V> goalQueryStatementMaker1() {
//
//  // [NIL AP x1^* EP_{XA_{x1,G1}} (x2 and XA_{x2}) AP END] ; x1 != x2 ; x2 != G3 ; EP
//
//  auto end = std::make_unique<EndGoalQuery<T,V>>();
//
//  auto edge3 = std::make_unique<EdgeGoalQuery<T,V>>(true, std::move(end));
//  auto exclude = std::make_unique<ExcludeGoalQuery<T,V>>(std::vector<std::string>{ "x2" }, std::vector<std::string>{});
//  auto x2 = std::make_unique<VariableGoalQuery<T,V>>("x2");
//  auto and3 = std::make_unique<AndGoalQuery<T,V>>(std::move(exclude), std::move(edge3));
//  auto and2 = std::make_unique<AndGoalQuery<T,V>>(std::move(x2), std::move(and3));
//
//  auto edge2_cond = std::make_unique<ExcludeGoalQuery<T,V>>(std::vector<std::string>{ "x1" }, std::vector<std::string>{ "G1" });
//  auto edge2 = std::make_unique<EdgeGoalQuery<T,V>>(false, std::move(and2), std::move(edge2_cond));
//  auto x1 = std::make_unique<VariableGoalQuery<T,V>>("x1");
//  auto and1 = std::make_unique<AndGoalQuery<T,V>>(std::move(x1), std::move(edge2));
//
//  auto edge1 = std::make_unique<EdgeGoalQuery<T,V>>(true,std::move(and1));
//  auto nil1 = std::make_unique<NilGoalQuery<T,V>>(std::move(edge1));
//
//  auto real_start_true = std::make_unique<TrueGoalQuery<T,V>>();
//  auto real_start_and = std::make_unique<AndGoalQuery<T,V>>(std::move(real_start_true), std::move(nil1));
//  auto real_start_edge = std::make_unique<EdgeGoalQuery<T,V>>(false, std::move(real_start_and));
//  auto real_start = std::make_unique<NilGoalQuery<T,V>>(std::move(real_start_edge));
//
//  std::set<std::string> strongly_recognizable_set{"x1"};
//  std::vector<std::pair<std::string,std::string>> unequal_CVs{ {"x1", "x2"} };
//
//  GoalQueryStatement<T,V> gqr{std::move(real_start), strongly_recognizable_set, unequal_CVs};
//
//  return gqr;
//}

