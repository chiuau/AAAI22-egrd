#include <memory>

#include "goal_query.h"
#include "gqs_reader.h"
#include "gqs_parser.tab.hpp"


GoalQueryStatement<FPST,FPSV> GqsReader::readFromStdin() {
  read_mode = READ_MODE::STDIN;
  filename = "std::cin";
  return read();
}

GoalQueryStatement<FPST,FPSV> GqsReader::readFromFile(const std::string &f) {
  read_mode = READ_MODE::FILE;
  filename = f;
  return read();
}

GoalQueryStatement<FPST,FPSV> GqsReader::readFromString(const std::string &s) {
  read_mode = READ_MODE::STRING;
  filename = "std::string";
  input_string = s;
  return read();
}


GoalQueryStatement<FPST,FPSV> GqsReader::read() {
  GoalQueryStatement<FPST,FPSV> gqs;

  location.initialize (&filename);
  scan_begin();
  gqs::GqsParser parser(gqs, *this);
  parser.set_debug_level(trace_parsing);
  int res = parser.parse();
  scan_end();
  if (res == 0)
    return gqs;
  else
    throw std::runtime_error("GqsReader::read(): parser.parse() failed");
}

void GqsReader::error(const gqs::location& l, const std::string& m) {
  // cerr << l << ": " << m << endl;
  throw gqs::GqsParser::syntax_error(l, m);
}

void GqsReader::error(const std::string& m) {
  // cerr << m << endl;
  throw gqs::GqsParser::syntax_error(gqs::location(), m);
}

GoalQueryStatement<FPST,FPSV> makeGoalQueryRecord(const std::string& str) {
  GqsReader reader;
  return reader.readFromString(str);
}

