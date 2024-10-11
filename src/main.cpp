/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-10-11 03:33:44
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-10-11 03:56:06
 * @ Description: BASIC MAIN LOOP FOR MAPPER_CMD
 */

#include "mapper_cmd.hpp"

std::atomic<bool> running(true);

void signalHandler_ZBFT(int signum) {//VOXL LIBS HAS ITS OWN SIGHANDLER FUNCTION, BUT IT'S NOT USED HERE
    running = false; 
}

int main() {
    std::signal(SIGINT, signalHandler_ZBFT);
    auto mapper_cmd = std::make_unique<MapperCmd>();//SMART POINTER TO AVOID MEMORY LEAKS
    mapper_cmd->initMPA();
    while (running) {
        usleep(50000);//POSIX FUNCTION --> COULD USE CHRONO INSTEAD 
    }
    return 0;
}