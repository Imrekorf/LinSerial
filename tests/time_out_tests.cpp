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
        pseudoterminal::setup();
        LinSer::Serial receiver(pseudoterminal::get_connected_ports().endpoint1.c_str());
        CHECK_THROWS(receiver.readLine());
        pseudoterminal::teardown();
    }
};