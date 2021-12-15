#ifndef FOCTL_NAME_ID_MAP_H
#define FOCTL_NAME_ID_MAP_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <stdexcept>

// -----------------------------------------------------------------------------------------
// A Name-Id Map
// -----------------------------------------------------------------------------------------

class NameIdMap {

  std::vector<std::string> idToNameMap;
  std::unordered_map<std::string,int> nameToIdMap;

public:

  int size() const { return idToNameMap.size(); }

  const std::vector<std::string>& getNames() const { return idToNameMap; }

  int getIdByName(const std::string& name) const {   //  return -1 if name not found
    try {
      return nameToIdMap.at(name);
    } catch(const std::out_of_range&) {
      return -1;
    }
  }

  const std::string& getNameById(int id) const {       // return an empty string if id not found
    if (0 <= id && id < idToNameMap.size()) {
      return idToNameMap[id];
    } else {
      return empty_name;
    }
  }

  int assignIdToName(const std::string& name) {
    if (name.empty()) throw std::runtime_error("Error in NameIdMap::assignIdToName()");
    if (nameToIdMap.find(name) == nameToIdMap.end()) {
      idToNameMap.push_back(name);
      int id = idToNameMap.size() - 1;
      nameToIdMap[name] = id;
      return id;
    }  else {   // name exists, no need to assign new id to it
      return nameToIdMap[name];
    }
  }

  int makeNewName(std::string prefix = "x") {
    int i=1;
    while(getIdByName(prefix + std::to_string(i)) >= 0) {
      i++;
    }
    std::string new_variable_name = prefix + std::to_string(i);
    return assignIdToName(new_variable_name);
  }

  std::string to_string() {
    std::string s;
    int vid=0;
    for(auto& name : idToNameMap) {
      if (vid != 0) s += " ";
      s += std::to_string(vid) + ":" + name;
      vid++;
    }
    return s;
  }

private:

  static std::string empty_name;
};


#endif //FOCTL_NAME_ID_MAP_H
