#include <thread>
#include <fstream>
#include <signal.h>
#include "common.h"
#include "pseudoterminal.h"

pseudoterminal::ConnectedPorts::ConnectedPorts()
{
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
        if (endpoint1 == INVALID_PORT){
            endpoint1 = device;
        } else {
            endpoint2 = device;
            break;
        }
    }

    is.close();
    if (endpoint1 == INVALID_PORT || endpoint2 == INVALID_PORT){
        throw std::runtime_error("pts: Could not setup tunnel - is socat installed?");
    }
}

pseudoterminal::ConnectedPorts::~ConnectedPorts()
{
    kill(socat_pid, SIGTERM);
    remove(OUTPUT_FILE.c_str());
}

std::tuple<std::string, std::string> pseudoterminal::ConnectedPorts::get()
{
    return {endpoint1, endpoint2}; 
}