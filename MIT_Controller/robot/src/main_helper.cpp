#include "main_helper.h"
#include "RobotRunner.h"
#include <thread>
#include <signal.h>

std::shared_ptr<RobotRunner> robotRunner;

void signal_callback_handler(int signum)
{
    printf("Interruped with SIGINT: %d", signum);
    robotRunner->Stop();
}

int main_helper(RobotController* ctrl)
{
    /* Register signal and signal handler */
    signal ( SIGINT, signal_callback_handler ); // put behind, otherwise will be overwirtten by others such as ROS

    robotRunner = std::make_shared<RobotRunner> (500, ctrl, RobotType::ALIENGO);
    robotRunner -> init();

    std::thread comm_thread (&RobotRunner::Communicate, robotRunner);
    std::thread control_thread (&RobotRunner::Control, robotRunner);

    comm_thread.join();
    control_thread.join();

    printf("Stopping Controller \n");

    return 0;

}