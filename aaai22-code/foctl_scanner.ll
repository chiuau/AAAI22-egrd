%{ /* -*- C++ -*- */

#include <cstdlib>
#include <cerrno>
#include <climits>
#include <string>
#include "foctl_reader.h"
#include "foctl_parser.tab.hpp"

#define yyterminate() return foctl::FoctlParser::make_END(loc);


%}

%option noyywrap nounput noinput batch debug
%option prefix="foctl_"

true_const            (?i:True)
false_const           (?i:False)
last_const            (?i:Last)
not_connective        \!
and_connective        \&\&
or_connective         \|\|
imply_connective      =>
iff_connective        <=>
forall                (?i:forall)
exist                 (?i:exist)
AF                    AF
AG                    AG
AX                    AX
EF                    EF
EG                    EG
EX                    EX
AC                    A\ *\[
EC                    E\ *\[
UU                    U
WW                    W
equal_comparator      ==
not_equal_comparator  \!=
not_in                (?i:not_in)
identifier            [a-zA-Z](\_*[a-zA-Z0-9\-]+)*
blank                 [ \t]

%{
// Code run each time a pattern is matched.
#define YY_USER_ACTION  loc.columns (yyleng);
%}

%%

%{
  foctl::location& loc = reader.getLocation();     // A handy shortcut to the location held by the driver.
  loc.step();    // Code run each time yylex is called.
%}

{blank}+   loc.step();
\n         { loc.lines(yyleng); loc.step(); return foctl::FoctlParser::make_END(loc); }

"("        return foctl::FoctlParser::make_LPAREN(loc);
")"        return foctl::FoctlParser::make_RPAREN(loc);
"["        return foctl::FoctlParser::make_LBRACKET(loc);
"]"        return foctl::FoctlParser::make_RBRACKET(loc);
"{"        return foctl::FoctlParser::make_LBRACE(loc);
"}"        return foctl::FoctlParser::make_RBRACE(loc);
"_"        return foctl::FoctlParser::make_UNDERSCORE(loc);
","        return foctl::FoctlParser::make_COMMA(loc);
"="        return foctl::FoctlParser::make_EQUAL(loc);


{true_const}         return foctl::FoctlParser::make_TRUE_CONST(loc);
{false_const}        return foctl::FoctlParser::make_FALSE_CONST(loc);
{last_const}         return foctl::FoctlParser::make_LAST_CONST(loc);
{not_connective}     return foctl::FoctlParser::make_NOT_CONNECTIVE(loc);
{and_connective}     return foctl::FoctlParser::make_AND_CONNECTIVE(loc);
{or_connective}      return foctl::FoctlParser::make_OR_CONNECTIVE(loc);
{imply_connective}   return foctl::FoctlParser::make_IMPLY_CONNECTIVE(loc);
{iff_connective}     return foctl::FoctlParser::make_IFF_CONNECTIVE(loc);

{forall}   return foctl::FoctlParser::make_FORALL(loc);
{exist}    return foctl::FoctlParser::make_EXIST(loc);

{AF}   return foctl::FoctlParser::make_AF(loc);
{AG}   return foctl::FoctlParser::make_AG(loc);
{AX}   return foctl::FoctlParser::make_AX(loc);
{EF}   return foctl::FoctlParser::make_EF(loc);
{EG}   return foctl::FoctlParser::make_EG(loc);
{EX}   return foctl::FoctlParser::make_EX(loc);
{AC}   return foctl::FoctlParser::make_AC(loc);
{EC}   return foctl::FoctlParser::make_EC(loc);
{UU}   return foctl::FoctlParser::make_UU(loc);
{WW}   return foctl::FoctlParser::make_WW(loc);

{equal_comparator}       return foctl::FoctlParser::make_EQUAL_COMPARATOR(loc);
{not_equal_comparator}   return foctl::FoctlParser::make_NOT_EQUAL_COMPARATOR(loc);
{not_in}                 return foctl::FoctlParser::make_NOT_IN(loc);

{identifier}   return foctl::FoctlParser::make_IDENTIFIER(std::string(yytext), loc);
.              reader.error(loc, "invalid character");
<<EOF>>        return foctl::FoctlParser::make_END(loc);

%%


void FoctlReader::scan_begin() {
  yy_flex_debug = trace_scanning;

  switch(read_mode) {
    case READ_MODE::UNKNOWN:
      error("FoctlReader: unknown read mode");
      // exit(EXIT_FAILURE);
      break;
    case READ_MODE::STDIN:
      yyin = stdin;
      break;
    case READ_MODE::FILE:
      if (!(yyin = fopen(filename.c_str(), "r"))) {
        error("FoctlReader: cannot open " + filename + ": " + strerror(errno));
        // exit(EXIT_FAILURE);
      }
      break;
    case READ_MODE::STRING:
      if (!(yyin = fmemopen(const_cast<char *>(input_string.c_str()), input_string.length(), "r"))) {
        error("FoctlReader: cannot read from string" + input_string + ": " + strerror(errno));
        // exit(EXIT_FAILURE);
      }
      break;
  }
}

void FoctlReader::scan_end() {
  switch(read_mode) {
    case READ_MODE::UNKNOWN:
      error("FoctlReader: unknown read mode");
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

