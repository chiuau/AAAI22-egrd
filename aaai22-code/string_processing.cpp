#include "string_processing.h"
std::string to_string() {
  return "";
}


bool to_bool(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  std::istringstream is(str);
  bool b;
  is >> std::boolalpha >> b;
  return b;
}

//bool stob(std::string s) {
//  if (s == "true" || s == "True" || s == "TRUE") {
//    return true;
//  } else if (s == "false" || s == "False" || s == "FALSE") {
//    return false;
//  } else {
//    throw std::runtime_error("Error in stob(): invalid boolean value: " + s);
//  }
//}

std::string indent(int n) {
  std::string s = "";
  for (int i = 0; i < n; i++) s += "  ";
  return s;
}


std::string readStringFromFile(const std::string& filename) {
  std::ifstream fin{filename, std::ifstream::in};
  if (!fin) throw std::runtime_error("Error in readStringFromFile(): cannot open " + filename);
  std::stringstream buffer;
  buffer << fin.rdbuf();
  return buffer.str();
}


bool isContainKeyword(const std::vector<std::string>& tokens, const std::string& keyword) {
  return std::ranges::find(tokens, keyword) != tokens.end();
}


std::vector<std::string> readTokensFromFile(const std::string& filename) {

  std::vector<std::string> token_list;

  std::ifstream fin{filename, std::ifstream::in};
  if (!fin) throw std::runtime_error("Error in readTokensFromFile(): cannot open " + filename);

  while(fin) {
    std::string token;
    fin >> token;
    if (!fin.fail()) token_list.push_back(token);
  }
  fin.close();

  return token_list;
}


std::vector<std::pair<std::string,std::vector<std::string>>> partitionTokensByKeywords(const std::vector<std::string>& tokens, const std::vector<std::string>& keywords) {
  std::vector<std::pair<std::string,std::vector<std::string>>> result;

  for(int i=0; i<tokens.size(); i++) {
    if (std::ranges::find(keywords, tokens[i]) != keywords.end()) {
      result.emplace_back(tokens[i], std::vector<std::string>());
    } else {
      if (result.empty()) {
        result.emplace_back("", std::vector<std::string>());
      }
      result.back().second.push_back(tokens[i]);
    }
  }

  return result;
}



void partitionTokensByType_impl(std::vector<TokenListVariant>& result, const std::vector<std::string>& tokens, const std::string& type_name, int last_i, int last_i_end) {
  if (type_name=="int") {
    if (last_i != last_i_end) {
      result.push_back(std::vector<int>());
      for(int i=last_i; i<last_i_end; i++) {
        std::get<std::vector<int>>(result.back()).push_back(std::stoi(tokens[i]));
      }
    } else {
      result.push_back(std::stoi(tokens[last_i]));
    }
  } else if (type_name=="float") {
    if (last_i != last_i_end) {
      result.push_back(std::vector<float>());
      for(int i=last_i; i<last_i_end; i++) {
        std::get<std::vector<float>>(result.back()).push_back(std::stof(tokens[i]));
      }
    } else {
      result.push_back(std::stof(tokens[last_i]));
    }
  } else if (type_name=="double") {
    if (last_i != last_i_end) {
      result.push_back(std::vector<double>());
      for(int i=last_i; i<last_i_end; i++) {
        std::get<std::vector<double>>(result.back()).push_back(std::stod(tokens[i]));
      }
    } else {
      result.push_back(std::stod(tokens[last_i]));
    }
  } else if (type_name=="bool") {
    if (last_i != last_i_end) {
      result.push_back(std::vector<bool>());
      for(int i=last_i; i<last_i_end; i++) {
        std::get<std::vector<bool>>(result.back()).push_back(to_bool(tokens[i]));
      }
    } else {
      result.push_back(to_bool(tokens[last_i]));
    }
  } else if (type_name=="string") {
    if (last_i != last_i_end) {
      result.push_back(std::vector<std::string>());
      for(int i=last_i; i<last_i_end; i++) {
        std::get<std::vector<std::string>>(result.back()).push_back(tokens[i]);
      }
    } else {
      result.push_back(tokens[last_i]);
    }
  } else {  // assume an exact match between type_name and tokens[i]
    if (last_i != last_i_end) {
      result.push_back(std::vector<std::string>());
      for(int i=last_i; i<last_i_end; i++) {
        if (tokens[i] == type_name) {
          std::get<std::vector<std::string>>(result.back()).push_back(tokens[i]);
        } else {
          throw std::runtime_error("Error in partitionTokensByType(): token unmatched: " + tokens[i] + " != " + type_name);
        }
      }
    } else {
      if (tokens[last_i] == type_name) {
        result.push_back(tokens[last_i]);
      } else {
        throw std::runtime_error("Error in partitionTokensByType(): token unmatched: " + tokens[last_i] + " != " + type_name);
      }
    }
  }
}


std::vector<TokenListVariant> partitionTokensByType(const std::vector<std::string>& tokens, const std::vector<std::pair<std::string,int>>& size_and_types) {
  std::vector<TokenListVariant> result;

  try {
    int last_i = 0;
    bool isCheckExactSize = true;
    for(auto& [ type_name, size ] : size_and_types) {
      if (!isCheckExactSize) throw std::out_of_range("Error in partitionTokensByType(): too many arguments in size_and_types");
      if (size > 0) {
        if (last_i + size > tokens.size()) throw std::out_of_range("Error in partitionTokensByType(): not enough token");
        partitionTokensByType_impl(result, tokens, type_name, last_i, last_i + size);
        last_i += size;
      } if (size == 0) {
        partitionTokensByType_impl(result, tokens, type_name, last_i, last_i);
        last_i += 1;
      } else { // the last one
        partitionTokensByType_impl(result, tokens, type_name, last_i, tokens.size());
        isCheckExactSize = false;
      }
    }

    if (isCheckExactSize) {
      if (last_i != tokens.size()) throw std::out_of_range("Error in partitionTokensByType(): wrong number of tokens");
    }
  } catch(const std::exception& e) {
    std::cerr << "Current tokens: " << to_string(tokens) << std::endl;
    throw;
  }

  return result;
}


std::vector<int> svtoiv(const std::vector<std::string>& str_list) {
  std::vector<int> result;
  for(auto& s : str_list) {
    result.push_back(std::stoi(s));
  }
  return result;
}


std::string replaceFilenameExtension(std::string filename, std::string extension) {
  int dot_pos = -1;
  for(int i=filename.size()-1; i >= 0; i--) {
    if (filename[i] == '.') { dot_pos = i; break; }
  }
  if (dot_pos >= 0) {
    std::string s;
    for(int i=0; i<=dot_pos; i++) s += filename[i];
    return s + extension;
  } else {
    return filename + "." + extension;
  }
}

