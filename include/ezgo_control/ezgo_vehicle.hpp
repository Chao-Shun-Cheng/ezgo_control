#include <pthread.h>
#include <ros/ros.h>
#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <autoware_msgs/AccelCmd.h>
#include <autoware_msgs/BrakeCmd.h>
#include <autoware_msgs/SteerCmd.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <tablet_socket_msgs/gear_cmd.h>
#include <tablet_socket_msgs/mode_cmd.h>

#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

enum Light { OFF, ON };

enum Shift { FORWARD, REVERSE };

enum Mode { MANUAL, AUTONOMOUS };

typedef struct vehicle_info {
    int throttle;
    int brake;
    float steering_angle;
    float velocity;
    bool control_mode;  // 0 : manual mode, 1 : autonomous mode
    bool shift;         // 0 : Forward, 1 : Reverse
    bool light;         // 0 : OFF, 1 : ON
} vehicle_info_t;

typedef struct vehicle_cmd {
    double linear_x;
    double angular_z;
    int modeValue;  // 0 : manual, 1 : auto pilot, 2 : UI direct control
    int shift;      // 0 : Forward, 1 : Reverse
    int accel_stroke;
    int brake_stroke;
    int steering_angle;
    char light;
} vehicle_cmd_t;

extern vehicle_cmd_t vehicle_cmd;
extern vehicle_info_t vehicle_info;
extern int willExit;

void cmd_reset()
{
    vehicle_cmd.linear_x = 0;
    vehicle_cmd.angular_z = 0;
    vehicle_cmd.modeValue = 0;
    vehicle_cmd.shift = 0;
    vehicle_cmd.accel_stroke = 0;
    vehicle_cmd.brake_stroke = 0;
    vehicle_cmd.steering_angle = 0;
    vehicle_cmd.light = 0;
}

void modeCMDCallback(const tablet_socket_msgs::mode_cmd &mode)
{   
    if (mode.mode == -1 || mode.mode == 0) {
        cmd_reset();
    }
    vehicle_cmd.modeValue = mode.mode;
    std::cout << "In modeCMDCallback, mode = " << vehicle_cmd.modeValue << std::endl;
}

void gearCMDCallback(const tablet_socket_msgs::gear_cmd &gear)
{
    vehicle_cmd.shift = gear.gear;
    std::cout << "In gearCMDCallback, mode = " <<  vehicle_cmd.shift << std::endl;
}

void twistCMDCallback(const geometry_msgs::TwistStamped &msg)
{
    vehicle_cmd.linear_x = msg.twist.linear.x;
    vehicle_cmd.angular_z = msg.twist.angular.z;
}

void steerCMDCallback(const autoware_msgs::SteerCmd &steer)
{
    vehicle_cmd.steering_angle = steer.steer;
}

void accellCMDCallback(const autoware_msgs::AccelCmd &accell)
{
    vehicle_cmd.accel_stroke = accell.accel;
}

void brakeCMDCallback(const autoware_msgs::BrakeCmd &brake)
{
    vehicle_cmd.brake_stroke = brake.brake;
}

void vehicle_control()
{
    // TODO
}

void showVehicleInfo()
{
    if (vehicle_info.control_mode == MANUAL) {
        std::cout << GREEN << "------ manual mode ------" << RESET << std::endl;
        std::cout << "Velocity : " << vehicle_info.velocity << std::endl;
    }

    else if (vehicle_info.control_mode == AUTONOMOUS) {
        std::cout << YELLOW << "------ autonomous mode ------" << RESET << std::endl;
        std::cout << "Throttle : " << vehicle_info.throttle << std::endl;
        std::cout << "Brake : " << vehicle_info.brake << std::endl;
        std::cout << "Velocity : " << vehicle_info.velocity << std::endl;
    }
    if (vehicle_info.light == ON)
        std::cout << "Light : ON" << std::endl;
    else
        std::cout << "Light : OFF" << std::endl;

    if (vehicle_info.shift == FORWARD)
        std::cout << "Shift : Forward" << std::endl;
    else
        std::cout << "Shift : Reverse" << std::endl;
    return;
}

char app_sigaltstack[SIGSTKSZ];

void app_signal_handler(int sig_num)
{
    willExit = 1;
    ros::shutdown();
}

int app_setup_signals()
{
    stack_t sigstack;
    struct sigaction sa;
    int ret = -1;

    sigstack.ss_sp = app_sigaltstack;
    sigstack.ss_size = SIGSTKSZ;
    sigstack.ss_flags = 0;
    if (sigaltstack(&sigstack, NULL) == -1) {
        perror("signalstack()");
        goto END;
    }

    sa.sa_handler = app_signal_handler;
    sa.sa_flags = SA_ONSTACK;
    if (sigaction(SIGINT, &sa, NULL) != 0) {
        perror("sigaction");
        goto END;
    }
    if (sigaction(SIGTERM, &sa, NULL) != 0) {
        perror("sigaction");
        goto END;
    }

    ret = 0;
END:
    return ret;
}
