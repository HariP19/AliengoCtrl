#pragma once

#include <sys/timerfd.h>
#include <sys/mman.h>
#include <cmath>
#include <iostream>
#include <sys/mman.h>
#include <unistd.h>
#include <chrono>

#include <vector>
#include <fstream>
#include <string.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

// #define DEBUG //Un-comment for Debugging

using namespace UNITREE_LEGGED_SDK;

class HardwareInterface
{

public:
    HardwareInterface( const LeggedType& type = LeggedType::Aliengo, const uint8_t& level = LOWLEVEL, const double &loop_rate = 500.0 )
     : safe(type), udp(level)
    {
        if( loop_rate>0 ){
            dt_ = 1.0/loop_rate;
        } else{
            std::cout << "Loop rate should be more than zero! Setting to default 500Hz" << std::endl;
            dt_ = 1.0/500.0;
        }
    }
    virtual ~HardwareInterface(){};

    void Init();
    void Stop();
    void Communicate();
    void Control();

protected:
    virtual void RobotControl() = 0;
    LowCmd cmd = {0};
    LowState data = {0};
    double dt_;

private:
    void InitError( const char* reason, bool printErrno );
    void PrefaultStack();
    void SetupScheduler();
    void SetInitValue();

    Safety safe;
    UDP udp;

    bool comm_ready_;
    bool running_;

    LowCmd cmd_;
    LowState data_;
        
    LowCmd cmd_buf_;
    LowState data_buf_;

    pthread_mutex_t cmd_mut_;
    pthread_mutex_t data_mut_;
};
