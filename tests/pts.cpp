#include <thread>
#include <tuple>
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <signal.h>
#include "helpers.h"
#include "pts.h"

const std::string OUTPUT_FILE = "socat_output.txt";
static int socat_pid = -1;

std::tuple<std::string, std::string> setup_pts()
{
    const std::string INVALID_PORT = "invalid";
    std::tuple<std::string, std::string> tunnel = std::tie(INVALID_PORT, INVALID_PORT);

    std::string socat_str = execute("((socat -d -d pty,raw,echo=0 pty,raw,echo=0) > " +  OUTPUT_FILE +" 2>&1)& echo $!");
    socat_pid = std::stoi(socat_str);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::ifstream is(OUTPUT_FILE);
    
    const std::string DEVICE_DIR = "/dev/pts/";
    std::string line;
    while (std::getline(is, line)){
        int offset = line.find(DEVICE_DIR);
        if (offset == std::string::npos) continue;
        std::string device = line.substr(offset);
        if (std::get<0>(tunnel) == INVALID_PORT){
            tunnel = std::make_tuple(device, INVALID_PORT);
        } else {
            tunnel = std::make_tuple(std::get<0>(tunnel), device);
            break;
        }
    }

    is.close();
    if (std::get<0>(tunnel) == "invalid" || std::get<1>(tunnel) == INVALID_PORT){
        throw std::runtime_error("pts: Could not setup tunnel");
    }

    return tunnel;
}

void pts_teardown()
{
    kill(socat_pid, SIGTERM);
}