#include <iostream>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <tuple>
#include "doctest.h"
#include "pseudoterminal.h"
#include "LinSerial.h"

TEST_SUITE ("time_out_tests")
{
    TEST_CASE ("time_out_on_read_line")
    {
        pseudoterminal::ConnectedPorts c_ports;
        const auto& [endpoint1, endpoint2] = c_ports.get(); 
        LinSer::Serial receiver(endpoint1.c_str());
        CHECK_THROWS(receiver.readLine());
    }
};