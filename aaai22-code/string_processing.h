#ifndef FOCTL_STRING_PROCESSING_H
#define FOCTL_STRING_PROCESSING_H

#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <string>
#include <sstream>
#include <memory>
#include <cassert>
#include <string_view>
#include <utility>
#include <variant>
#include <filesystem>
#include <algorithm>


/* --------------------------------------------------------------------------------------------------
 * to_string() - convert something to string
 *
 * Usage: to_string(v)
 *
 * -------------------------------------------------------------------------------------------------- */

template<typename T>
std::string to_string_impl(T v) {
  std::stringstream ss;
  ss << v;
  return ss.str();
}

template<typename T>
std::string to_string_impl(const std::vector<T>& vs) {
  std::string s = "[";
  bool isFirst = true;
  for(auto& v : vs) {
    if (isFirst) {
      isFirst = false;
    } else {
      s += ",";
    }
    s += to_string_impl(v);
  }
  s += "]";
  return s;
}

template<typename T>
std::string to_string_impl(const std::set<T>& vs) {
  std::string s = "{";
  bool isFirst = true;
  for(auto& v : vs) {
    if (isFirst) {
      isFirst = false;
    } else {
      s += ",";
    }
    s += to_string_impl(v);
  }
  s += "}";
  return s;
}

template<typename T1, typename T2>
std::string to_string_impl(const std::pair<T1, T2>& p) {
  return "(" + to_string_impl(p.first) + "," + to_string_impl(p.second) + ")";
}


std::string to_string();

template<typename T>
std::string to_string(T&& v) {
  return to_string_impl(std::forward<T>(v));
}

template<typename T, typename... TS>
std::string to_string(T&& v, TS... vs) {
  return to_string_impl(std::forward<T>(v)) + " " + to_string(vs...);
}


/* --------------------------------------------------------------------------------------------------
 * to_bool(str) - Parse a string to a boolean value
 *
 * Usage:  bool b = to_bool("tRuE");
 *
 * See: https://stackoverflow.com/questions/3613284/c-stdstring-to-boolean
 * -------------------------------------------------------------------------------------------------- */

bool to_bool(std::string str);



/* --------------------------------------------------------------------------------------------------
 * indent(n) - generate 2*n empty spaces
 *
 * Usage:  std::cout << indent(n);
 * -------------------------------------------------------------------------------------------------- */

std::string indent(int n);


/* --------------------------------------------------------------------------------------------------
 * readStringFromFile(filename) - read a file and store the content in a string
 *
 * Usage:  auto s = readStringFromFile(filename);
 * -------------------------------------------------------------------------------------------------- */

std::string readStringFromFile(const std::string& filename);


/* --------------------------------------------------------------------------------------------------
 * readTokensFromFile(filename) - read a list of tokens from a file
 *
 * Usage:  std::vector<std::string> tokens = readTokensFromFile(filename);
 * -------------------------------------------------------------------------------------------------- */

std::vector<std::string> readTokensFromFile(const std::string& filename);

/* --------------------------------------------------------------------------------------------------
 * isContainKeyword(tokens, keyword) - check whether a token list contains a keyword
 *
 * Usage:  if (isContainKeyword(tokens, keyword))
 * -------------------------------------------------------------------------------------------------- */

bool isContainKeyword(const std::vector<std::string>& tokens, const std::string& keyword);

/* --------------------------------------------------------------------------------------------------
 * partitionTokens(tokens, keywords) - partition a tokens list according to some keywords
 *
 * Usage:  auto token_partition = partitionTokensByKeywords(tokens, keywords);
 * -------------------------------------------------------------------------------------------------- */

std::vector<std::pair<std::string,std::vector<std::string>>> partitionTokensByKeywords(const std::vector<std::string>& tokens, const std::vector<std::string>& keywords);


/* --------------------------------------------------------------------------------------------------
 * partitionTokens(tokens, sizes) - partition a tokens list according to sizes and types.
 *
 * Default types are "int", "float", "double", "bool" and "string"
 * Other types are keywords that must match exactly
 * size = 0 means match one token without vector
 * size = -1 at the end means matching the rest of the tokens.
 * Throw out_of_range if tokens does not match the sizes or types
 *
 * Usage:  auto token_partition = partitionTokensByType(tokens, { {"bool", 1}, {"string", 3}, {"GRAPH", 0} {"int", -1} });
 * -------------------------------------------------------------------------------------------------- */

using TokenListVariant = std::variant<int, float, double, bool, std::string, std::vector<int>,std::vector<float>,std::vector<double>,std::vector<bool>,std::vector<std::string>>;

std::vector<TokenListVariant> partitionTokensByType(const std::vector<std::string>& tokens, const std::vector<std::pair<std::string,int>>& size_and_types);


/* --------------------------------------------------------------------------------------------------
 * Parse tokens - parse a list of tokens
 * -------------------------------------------------------------------------------------------------- */

template<typename T>
T parseOneToken(const std::vector<std::string>& tokens) {
  if (tokens.size() != 1) throw std::out_of_range("Error in parseOneToken(): incorrect number of tokens: " + to_string(tokens));
  std::istringstream is(tokens[0]);
  T v;
  is >> std::boolalpha >> v;
  return v;
}

template<typename T1, typename T2>
std::pair<T1,T2> parseTwoTokens(const std::vector<std::string>& tokens) {
  if (tokens.size() != 2) throw std::out_of_range("Error in parseTwoTokens(): incorrect number of tokens: " + to_string(tokens));
  std::istringstream is1(tokens[0]);
  T1 v1;
  is1 >> std::boolalpha >> v1;
  std::istringstream is2(tokens[1]);
  T2 v2;
  is2 >> std::boolalpha >> v2;
  return { v1, v2 };
}

template<typename T>
std::vector<T> parseTokenList(const std::vector<std::string>& tokens) {
  std::vector<T> result;
  for(auto& token : tokens) {
    std::istringstream is(token);
    T v;
    is >> std::boolalpha >> v;
    result.push_back(v);
  }
  return result;
}

template<typename T1, typename T2>
std::pair<T1,std::vector<T2>> parseOneTokenAndTokenList(const std::vector<std::string>& tokens) {
  if (tokens.size() < 1) throw std::out_of_range("Error in parseOneTokenAndTokenList(): incorrect number of tokens: " + to_string(tokens));
  std::istringstream is1(tokens[0]);
  T1 v1;
  is1 >> std::boolalpha >> v1;

  std::vector<T2> vs;
  bool isFirst = true;
  for(auto& token : tokens) {
    if (!isFirst) {
      std::istringstream is2(token);
      T2 v;
      is2 >> std::boolalpha >> v;
      vs.push_back(v);
    } else {
      isFirst = false;
    }
  }
  return { v1, vs };
}


/* --------------------------------------------------------------------------------------------------
 * svtoiv(tokens) - convert a string vector into an integer vector
 *
 * Usage:  auto iv = svtoiv(tokens);
 * -------------------------------------------------------------------------------------------------- */

std::vector<int> svtoiv(const std::vector<std::string>& str_list);


/* --------------------------------------------------------------------------------------------------
 * replaceFilenameExtension(filename) - convert a string vector into an integer vector
 *
 * Usage:  auto filename2 = replaceFilenameExtension(filename, "dot");
 * -------------------------------------------------------------------------------------------------- */

std::string replaceFilenameExtension(std::string filename, std::string extension);


/* --------------------------------------------------------------------------------------------------
 * Trim a string
 *
 * See https://stackoverflow.com/questions/216823/how-to-trim-a-stdstring
 * -------------------------------------------------------------------------------------------------- */

// trim from start (in place)
static inline void ltrim(std::string &s) {
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
    return !std::isspace(ch);
  }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
  s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
    return !std::isspace(ch);
  }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
  ltrim(s);
  rtrim(s);
}

// trim from start (copying)
static inline std::string ltrim_copy(std::string s) {
  ltrim(s);
  return s;
}

// trim from end (copying)
static inline std::string rtrim_copy(std::string s) {
  rtrim(s);
  return s;
}

// trim from both ends (copying)
static inline std::string trim_copy(std::string s) {
  trim(s);
  return s;
}

/* --------------------------------------------------------------------------------------------------
 * CommandLineArgument - convert the command line arguments to a tokens list
 *
 * Usage:  auto s = readTokensFromCommandLineArguments(argc, argv);
 * -------------------------------------------------------------------------------------------------- */


class CommandLineArgument {
protected:

  std::string program_name;
  std::vector<std::string> token_list;
  std::unordered_map<std::string,std::vector<std::string>> token_partition;

public:

  CommandLineArgument(int argc, char *argv[], const std::vector<std::pair<std::string,int>> arg_spec) {
    std::filesystem::path p(argv[0]);
    program_name = p.filename().string();

    token_list = convertToTokens(argc, argv);
    auto token_list_copy = token_list;
    // CommandLineArgument(argc, argv, { { "-h", 0 }, { "-v", 0 }, { "-dot", 0 }, {"", 3} })

    // deal with option one by one
    for(auto [option, count] : arg_spec) {
      if (option.empty()) {
        for(auto& token : token_list_copy) {
          token_partition[""].push_back(token);
        }
        break;  // finish
      } else {
        for(int i=0; i < token_list_copy.size(); i++) {
          if (token_list_copy[i] == option) {  // found it
            token_partition[option] = {};
            for(int j=i+1; j<i+1+count; j++) {
              if (j >= token_list_copy.size()) throw std::runtime_error("Error in CommandLineArgument::CommandLineArgument(): wrong number of arguments for option " + option);
              token_partition[option].push_back(token_list_copy[j]);
            }
            token_list_copy = removeTokens(token_list_copy, i, i + count);
            break;  // don't deal with duplicated option
          }  // else do nothing
        }
      }
    }
    // ignore excessive arguments
  }

private:

  std::vector<std::string> convertToTokens(int argc, char *argv[]) {
    std::vector<std::string> result;
    for(int i=1; i<argc; i++) {
      result.emplace_back(argv[i]);
    }
    return result;
  }


  std::vector<std::string> removeTokens(const std::vector<std::string>& token_list, int i1, int i2) {
    std::vector<std::string> result;
    for(int i=0; i<i1; i++) {
      result.push_back(token_list[i]);
    }
    for(int i=i2+1; i<token_list.size(); i++) {
      result.push_back(token_list[i]);
    }
    return result;
  }

};


#endif //FOCTL_STRING_PROCESSING_H
