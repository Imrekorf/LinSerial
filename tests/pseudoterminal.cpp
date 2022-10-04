#include <thread>
#include <tuple>
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <signal.h>
#include <stdio.h>
#include "helpers.h"
#include "pseudoterminal.h"

const std::string OUTPUT_FILE = "socat_output.txt";
static int socat_pid = -1;
static pseudoterminal::Ports connected_ports;
static bool setup_done = false;

void pseudoterminal::setup()
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
        if (connected_ports.endpoint1 == INVALID_PORT){
            connected_ports.endpoint1 = device;
        } else {
            connected_ports.endpoint2 = device;
            break;
        }
    }

    is.close();
    if (connected_ports.endpoint1 == "invalid" || connected_ports.endpoint2 == INVALID_PORT){
        throw std::runtime_error("pts: Could not setup tunnel - is socat installed?");
    }
    setup_done = true;
}

void pseudoterminal::teardown()
{
    kill(socat_pid, SIGTERM);
    remove(OUTPUT_FILE.c_str());
}

pseudoterminal::Ports pseudoterminal::get_connected_ports()
{
    if (!setup_done) setup();
    return connected_ports; 
}