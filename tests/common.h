#pragma once

#include <algorithm>
#include <vector>
#include <string>

std::string execute(const std::string& cmd);
std::vector<std::string> str_split(std::string string, const std::string& delimiter);
std::vector<std::string> get_file_names(const std::string& path);