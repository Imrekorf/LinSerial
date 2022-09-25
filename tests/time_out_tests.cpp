#include <iostream>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <tuple>
#include "doctest.h"
#include "pts.h"
#include "LinSerial.h"

TEST_SUITE ("time_out_tests")
{
    TEST_CASE ("time_out_on_read_line")
    {
        auto tunnel = setup_pts();
        LinSer::Serial receiver(std::get<0>(tunnel).c_str());
        CHECK_THROWS(receiver.readLine());
    }
};