#ifndef CODE_FOCTL_READER_H
#define CODE_FOCTL_READER_H

#include <string>
#include "foctl_parser.tab.hpp"

#include "foctl.h"

// Tell Flex the lexer's prototype ...
// #define YY_DECL  foctl::FoctlParser::symbol_type foctl_lex(FoctlReader& reader)

// ... and declare it for the parser's sake.
// need to wrap it with extern "C"
// see https://issues.apache.org/jira/browse/THRIFT-2386
// extern "C" {
//   YY_DECL;
// }

class FoctlReader {

  enum class READ_MODE {
    UNKNOWN,
    STDIN,
    FILE,
    STRING
  };

  READ_MODE read_mode = READ_MODE::UNKNOWN;

  int trace_scanning = 0;
  int trace_parsing = 0;

  std::string filename;
  std::string input_string;

  foctl::location location;  // The token's location used by the scanner.

public:

  // Get file name
  std::string& getFilename() { return filename; }

  // Handling the scanner.
  void scan_begin();
  void scan_end();
  foctl::location& getLocation() { return location; }

  // Run the parser.  Return NULL on failure
  StatementRecord<FPST,FPSV> readFromStdin();

  StatementRecord<FPST,FPSV> readFromFile(const std::string& f);

  StatementRecord<FPST,FPSV> readFromString(const std::string& s);

  // Error handling.
  static void error(const foctl::location& l, const std::string& m);


private:

  StatementRecord<FPST,FPSV> read();

  static void error(const std::string& m);

};

// -----------------------------------------------------------------------------------------
// Statement Factory
// -----------------------------------------------------------------------------------------

StatementRecord<FPST,FPSV> makeStatementRecord(const std::string& str);

#endif //CODE_FOCTL_READER_H
