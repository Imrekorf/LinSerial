#include <thread>
#include <tuple>
#include "helpers.h"
#include "pts.h"

std::tuple<std::string, std::string> setup_pts()
{
    //todo: check if socat installed
    const std::string DEVICE_DIR = "/dev/pts/";
    std::tuple<std::string, std::string> tunnel = std::tie("", "");
    auto original_files = get_file_names(DEVICE_DIR);

    std::thread([](){
        //todo: shut down?
        execute("socat -d -d pty,raw,echo=0 pty,raw,echo=0");
    }).detach();

    // Hacky way to set up a tunnel which can be used to create LinSerial communication
    auto start_ts = std::chrono::steady_clock::now();
    const int TIMEOUT_MS = 5000;
    while(true){
        std::this_thread::sleep_for (std::chrono::milliseconds(500));
        auto current_files = get_file_names(DEVICE_DIR);
        auto nonintersecting = find_nonintersecting_elements(original_files, current_files); 
        if (nonintersecting.size() == 2){
            tunnel = std::make_tuple(DEVICE_DIR + nonintersecting[0], DEVICE_DIR + nonintersecting[1]);
            break;
        }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_ts).count() > TIMEOUT_MS){
            throw std::runtime_error("Setup failed: timed out setting up tunnel");
            break;
        }
    }

    return tunnel;
}
