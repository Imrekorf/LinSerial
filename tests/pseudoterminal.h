#pragma once

const std::string INVALID_PORT = "invalid";

namespace pseudoterminal
{
    class ConnectedPorts
    {
    public:
        ConnectedPorts();
        ~ConnectedPorts();
        std::tuple<std::string, std::string> get();
        const std::string OUTPUT_FILE = "socat_output.txt";
        int socat_pid = -1;
    private:
        std::string endpoint1 = INVALID_PORT;
        std::string endpoint2 = INVALID_PORT;
    };
}
