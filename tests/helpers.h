#pragma once

#include <algorithm>
#include <vector>
#include <string>

std::string execute(std::string cmd);
std::vector<std::string> str_split(std::string string, const std::string& delimiter);
std::vector<std::string> get_file_names(const std::string& path);

template <typename T>
std::vector<T> find_nonintersecting_elements(std::vector<T> a, std::vector<T> b)
{
    std::vector<T> result;
    for (const auto& element : a){
        if (std::find(b.begin(), b.end(), element) == b.end()){
            result.push_back(element);
        }
    }

    for (const auto& element : b){
        if (std::find(a.begin(), a.end(), element) == a.end()){
            result.push_back(element);
        }
    }

    return result;
}