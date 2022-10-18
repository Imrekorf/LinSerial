#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include "common.h"

std::string execute(const std::string& cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

std::vector<std::string> str_split(std::string string, const std::string& delimiter)
{
    std::vector<std::string> result;
    size_t pos = 0;
    std::string token;
    while ((pos = string.find(delimiter)) != std::string::npos) {
        token = string.substr(0, pos);
        result.push_back(token);
        string.erase(0, pos + delimiter.length());
    }
    return result;
}

std::vector<std::string> get_file_names(const std::string& path)
{
    std::string response = execute("ls " + path);
    // When failed, sends output to std::err so this is empty. Treat same as no files
    return str_split(response, "\n");
}