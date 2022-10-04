#include <string>
#include "doctest.h"
#include "pseudoterminal.h"
#include "LinSerial.h"

TEST_SUITE ("communication_tests")
{
    TEST_CASE ("read_single_line")
    {
        pseudoterminal::setup();
        LinSer::Serial writer(pseudoterminal::get_connected_ports().endpoint1.c_str());
        LinSer::Serial receiver(pseudoterminal::get_connected_ports().endpoint2.c_str());
        
        auto receiver_thread = std::thread([&receiver](){
            auto start_ts = std::chrono::steady_clock::now(); 
            const int TIMEOUT_MS = 2500;
            while (!receiver.available()){
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_ts).count() > TIMEOUT_MS){
                    //time out, didn't receive message
                    CHECK(false);
                    break;
                }
            }
            CHECK_EQ(receiver.readLine(), "this is a line");
        });
        writer << "this is a line\nthis is another\n";
        
        receiver_thread.join();
        pseudoterminal::teardown();
    }
};