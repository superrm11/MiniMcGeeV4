#pragma once
#include <thread>
#include <unistd.h>

class SimpleWatchdog
{
    public:
    SimpleWatchdog(uint time_us, bool isfatal=false);
    
    void feed();
    void start();
    void stop();
    void get_tripped();

};