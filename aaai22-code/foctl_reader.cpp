#include <memory>

#include "foctl.h"
#include "foctl_reader.h"
#include "foctl_parser.tab.hpp"
#include "foctl.h"


StatementRecord<FPST,FPSV> FoctlReader::readFromStdin() {
  read_mode = READ_MODE::STDIN;
  filename = "std::cin";
  return read();
}

StatementRecord<FPST,FPSV> FoctlReader::readFromFile(const std::string &f) {
  read_mode = READ_MODE::FILE;
  filename = f;
  return read();
}

StatementRecord<FPST,FPSV> FoctlReader::readFromString(const std::string &s) {
  read_mode = READ_MODE::STRING;
  filename = "std::string";
  input_string = s;
  return read();
}


StatementRecord<FPST,FPSV> FoctlReader::read() {
  StatementRecord<FPST,FPSV> sr;

  location.initialize (&filename);
  scan_begin();
  foctl::FoctlParser parser(sr, *this);
  parser.set_debug_level(trace_parsing);
  int res = parser.parse();
  scan_end();
  if (res == 0)
    return sr;
  else
    throw FoctlParseError("FoctlReader::read(): parser.parse() failed");
}

void FoctlReader::error(const foctl::location& l, const std::string& m) {
  // cerr << l << ": " << m << endl;
  throw foctl::FoctlParser::syntax_error(l, m);
}

void FoctlReader::error(const std::string& m) {
  // cerr << m << endl;
  throw foctl::FoctlParser::syntax_error(foctl::location(), m);
}

StatementRecord<FPST,FPSV> makeStatementRecord(const std::string& str) {
  FoctlReader reader;
  return reader.readFromString(str);
}

