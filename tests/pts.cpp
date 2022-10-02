#include <thread>
#include <tuple>
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include "helpers.h"
#include "pts.h"

std::tuple<std::string, std::string> setup_pts()
{
    const std::string INVALID_PORT = "invalid";
    std::tuple<std::string, std::string> tunnel = std::tie(INVALID_PORT, INVALID_PORT);

    const std::string output_file = "socat_output.txt";
    system(("(sh -c '(echo PID: $$; exec socat -d -d pty,raw,echo=0 pty,raw,echo=0) > " + output_file +" 2>&1')&").c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::ifstream is(output_file);
    
    const std::string DEVICE_DIR = "/dev/pts/";
    std::string line;
    while (std::getline(is, line)){
        int offset = line.find(DEVICE_DIR);
        if (offset == std::string::npos) continue;
        std::string device = line.substr(offset, line.length() - offset);
        if (std::get<0>(tunnel) == INVALID_PORT){
            tunnel = std::make_tuple(device, INVALID_PORT);
        } else {
            tunnel = std::make_tuple(std::get<0>(tunnel), device);
        }
    }

    is.close();
    if (std::get<0>(tunnel) == "invalid" || std::get<1>(tunnel) == INVALID_PORT){
        throw std::runtime_error("pts: Could not setup tunnel");
    }

    return tunnel;
}