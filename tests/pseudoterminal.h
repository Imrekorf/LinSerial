#pragma once

const std::string INVALID_PORT = "invalid";

namespace pseudoterminal
{
    struct Ports
    {
        std::string endpoint1 = INVALID_PORT;
        std::string endpoint2 = INVALID_PORT;
    };
    void setup();
    void teardown();
    pseudoterminal::Ports get_connected_ports();
}
