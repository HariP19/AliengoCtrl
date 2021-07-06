#include <thread>
#include <signal.h>
#include "hardware_interface.h"

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

class Custom : public HardwareInterface
{

public:
    Custom (const double &loop_rate)
        :HardwareInterface(LeggedType::A1, LOWLEVEL, loop_rate){}
    ~Custom(){};
        
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float qInit[3] = {0};
    float qDes[3]={0};
    long long count = 0;
    int rate_count = 0;
    int sin_count = 0;
    float Kp[3] = {0};  
    float Kd[3] = {0};
    double homing_time = 5.0;
    double sin_period = 2.0;

    double sin_joint1, sin_joint2;



private:
    void RobotControl()
    {   
        count++;
        // printf("%d  %f\n", count, state.motorState[FR_2].q);
        // printf("%d  %f\n", count, data.imu.quaternion[2]);

        // gravity compensation
        cmd.motorCmd[FR_0].tau = -0.65f;
        cmd.motorCmd[FL_0].tau = +0.65f;
        cmd.motorCmd[RR_0].tau = -0.65f;
        cmd.motorCmd[RL_0].tau = +0.65f;

        // if( count >= 100){
        if( count >= 0){
            // first, get record initial position
            // if( count >= 100 && count < 500){
            if( count >= 0 && count < 10){
                qInit[0] = data.motorState[FR_0].q;
                qInit[1] = data.motorState[FR_1].q;
                qInit[2] = data.motorState[FR_2].q;
            }
            // second, move to the origin point of a sine movement with Kp Kd
            // if( count >= 500 && count < 1500){
            if( count*dt_ < homing_time){
                rate_count++;
                double rate = rate_count/200.0;                       // needs count to 200
                Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0; 
                Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
                
                qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
                qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
                qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
            } else {    // last, do sine wave 
                sin_count++;
                sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
                sin_joint2 = -0.6 * sin(1.8*M_PI*sin_count/1000.0);
                qDes[0] = sin_mid_q[0];
                qDes[1] = sin_mid_q[1];
                qDes[2] = sin_mid_q[2] + sin_joint2;
            }

            cmd.motorCmd[FR_0].q = qDes[0];
            cmd.motorCmd[FR_0].dq = 0;
            cmd.motorCmd[FR_0].Kp = Kp[0];
            cmd.motorCmd[FR_0].Kd = Kd[0];
            cmd.motorCmd[FR_0].tau = -0.65f;

            cmd.motorCmd[FR_1].q = qDes[1];
            cmd.motorCmd[FR_1].dq = 0;
            cmd.motorCmd[FR_1].Kp = Kp[1];
            cmd.motorCmd[FR_1].Kd = Kd[1];
            cmd.motorCmd[FR_1].tau = 0.0f;

            cmd.motorCmd[FR_2].q =  qDes[2];
            cmd.motorCmd[FR_2].dq = 0;
            cmd.motorCmd[FR_2].Kp = Kp[2];
            cmd.motorCmd[FR_2].Kd = Kd[2];
            cmd.motorCmd[FR_2].tau = 0.0f;

        }
    }
};

std::shared_ptr<Custom> io;

void signal_callback_handler (int signum)
{
    printf("Interruped with SIGINT: %d", signum);
    io->Stop();
}

int main ()
{
    /* Register signal and signal handler */
    signal ( SIGINT, signal_callback_handler ); // put behind, otherwise will be overwirtten by others such as ROS

    io = std::make_shared<Custom> ( 500 );
    io->Init();
    
    std::thread comm_thread (&Custom::Communicate, io);
    std::thread control_thread (&Custom::Control, io);

    comm_thread.join();
    control_thread.join();

    return 0;
}