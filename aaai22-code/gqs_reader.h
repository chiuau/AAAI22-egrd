#ifndef FOCTL_GQS_READER_H
#define FOCTL_GQS_READER_H

#include <string>
#include "gqs_parser.tab.hpp"

#include "goal_query.h"

class GqsReader {

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

  gqs::location location;  // The token's location used by the scanner.

public:

  // Get file name
  std::string& getFilename() { return filename; }

  // Handling the scanner.
  void scan_begin();
  void scan_end();
  gqs::location& getLocation() { return location; }

  // Run the parser.  Return NULL on failure
  GoalQueryStatement<FPST,FPSV> readFromStdin();

  GoalQueryStatement<FPST,FPSV> readFromFile(const std::string& f);

  GoalQueryStatement<FPST,FPSV> readFromString(const std::string& s);

  // Error handling.
  static void error(const gqs::location& l, const std::string& m);


private:

  GoalQueryStatement<FPST,FPSV> read();

  static void error(const std::string& m);

};

// -----------------------------------------------------------------------------------------
// Statement Factory
// -----------------------------------------------------------------------------------------

GoalQueryStatement<FPST,FPSV> makeGoalQueryRecord(const std::string& str);


#endif //FOCTL_GQS_READER_H
