%{ /* -*- C++ -*- */

#include <cstdlib>
#include <cerrno>
#include <climits>
#include <string>
#include "gqs_reader.h"
#include "gqs_parser.tab.hpp"

#define yyterminate() return gqs::GqsParser::make_GQS_END(loc);


%}

%option noyywrap nounput noinput batch debug
%option prefix="gqs_"

true_const            (?i:True)
false_const           (?i:False)
not_connective        (?i:not)
and_connective        (?i:and)
or_connective         (?i:or)
imply_connective      =>
iff_connective        <=>
forall                (?i:forall)
exist                 (?i:exist)
NIL                   (?i:NIL)
END                   (?i:END)
AP                    (?i:AP)
EP                    (?i:EP)
AX                    (?i:AX)
EX                    (?i:EX)
XA                    (?i:XA)
equal_comparator      ==
not_equal_comparator  \!=
not_in                (?i:not_in)
constant              [A-Z](\_*[a-zA-Z0-9\-]+)*
variable              [a-z](\_*[a-zA-Z0-9\-]+)*
trace_variable        $(\_*[a-zA-Z0-9\-]+)*
blank                 [ \t]

%{
// Code run each time a pattern is matched.
#define YY_USER_ACTION  loc.columns (yyleng);
%}

%%

%{
  gqs::location& loc = reader.getLocation();     // A handy shortcut to the location held by the driver.
  loc.step();    // Code run each time yylex is called.
%}

{blank}+   loc.step();
\n         { loc.lines(yyleng); loc.step(); return gqs::GqsParser::make_GQS_END(loc); }

"("        return gqs::GqsParser::make_LPAREN(loc);
")"        return gqs::GqsParser::make_RPAREN(loc);
"["        return gqs::GqsParser::make_LBRACKET(loc);
"]"        return gqs::GqsParser::make_RBRACKET(loc);
"{"        return gqs::GqsParser::make_LBRACE(loc);
"}"        return gqs::GqsParser::make_RBRACE(loc);
"_"        return gqs::GqsParser::make_UNDERSCORE(loc);
","        return gqs::GqsParser::make_COMMA(loc);
"="        return gqs::GqsParser::make_EQUAL(loc);
";"        return gqs::GqsParser::make_SEMICOLON(loc);
"^"        return gqs::GqsParser::make_CARET(loc);
"*"        return gqs::GqsParser::make_ASTERISK(loc);


{true_const}         return gqs::GqsParser::make_TRUE_CONST(loc);
{false_const}        return gqs::GqsParser::make_FALSE_CONST(loc);
{not_connective}     return gqs::GqsParser::make_NOT_CONNECTIVE(loc);
{and_connective}     return gqs::GqsParser::make_AND_CONNECTIVE(loc);
{or_connective}      return gqs::GqsParser::make_OR_CONNECTIVE(loc);
{imply_connective}   return gqs::GqsParser::make_IMPLY_CONNECTIVE(loc);
{iff_connective}     return gqs::GqsParser::make_IFF_CONNECTIVE(loc);

{forall}   return gqs::GqsParser::make_FORALL(loc);
{exist}    return gqs::GqsParser::make_EXIST(loc);

{NIL}   return gqs::GqsParser::make_NIL(loc);
{END}   return gqs::GqsParser::make_END(loc);
{AP}    return gqs::GqsParser::make_AP(loc);
{EP}    return gqs::GqsParser::make_EP(loc);
{AX}    return gqs::GqsParser::make_AX(loc);
{EX}    return gqs::GqsParser::make_EX(loc);
{XA}    return gqs::GqsParser::make_XA(loc);

{equal_comparator}       return gqs::GqsParser::make_EQUAL_COMPARATOR(loc);
{not_equal_comparator}   return gqs::GqsParser::make_NOT_EQUAL_COMPARATOR(loc);
{not_in}                 return gqs::GqsParser::make_NOT_IN(loc);

{constant}           return gqs::GqsParser::make_CONSTANT(std::string(yytext), loc);
{variable}           return gqs::GqsParser::make_VARIABLE(std::string(yytext), loc);
{trace_variable}     return gqs::GqsParser::make_TRACE_VARIABLE(std::string(yytext), loc);
.                    reader.error(loc, "invalid character");
<<EOF>>              return gqs::GqsParser::make_GQS_END(loc);

%%


void GqsReader::scan_begin() {
  yy_flex_debug = trace_scanning;

  switch(read_mode) {
    case READ_MODE::UNKNOWN:
      error("GqsReader: unknown read mode");
      // exit(EXIT_FAILURE);
      break;
    case READ_MODE::STDIN:
      yyin = stdin;
      break;
    case READ_MODE::FILE:
      if (!(yyin = fopen(filename.c_str(), "r"))) {
        error("GqsReader: cannot open " + filename + ": " + strerror(errno));
        // exit(EXIT_FAILURE);
      }
      break;
    case READ_MODE::STRING:
      if (!(yyin = fmemopen(const_cast<char *>(input_string.c_str()), input_string.length(), "r"))) {
        error("GqsReader: cannot read from string" + input_string + ": " + strerror(errno));
        // exit(EXIT_FAILURE);
      }
      break;
  }
}

void GqsReader::scan_end() {
  switch(read_mode) {
    case READ_MODE::UNKNOWN:
      error("GqsReader: unknown read mode");
      // exit(EXIT_FAILURE);
      break;
    case READ_MODE::STDIN:
      // do nothing
      break;
    case READ_MODE::FILE:
    case READ_MODE::STRING:
      fclose(yyin);
      break;
  }
}

