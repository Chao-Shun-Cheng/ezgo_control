#include <math.h>
#include <pthread.h>
#include <ros/ros.h>
#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>

#include <autoware_msgs/AccelCmd.h>
#include <autoware_msgs/BrakeCmd.h>
#include <autoware_msgs/SteerCmd.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <tablet_socket_msgs/gear_cmd.h>
#include <tablet_socket_msgs/mode_cmd.h>

#define STEERING_OFFSET 0x400
#define BRAKE_OFFSET 26

#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

enum HeadLight { OFF = 0, ON = 1 };

enum TurningLight { NONE = 0, LEFT = 1, RIGHT = 2, BOTH = 3 };

enum Shift { PARKING = 0, REVERSE = 1, NEUTRAL = 2, DRIVE = 3 };

enum Mode { MANUAL = 0, AUTONOMOUS = 1, OVERRIDE = 2, CARINIT = 3 };

typedef struct vehicle_info {
    int throttle;
    int brake;
    float steering_angle;
    float velocity;
    int control_mode;
    int shift;
    int headlight;
    int turninglight;
} vehicle_info_t;

typedef struct vehicle_cmd {
    double linear_x;
    double angular_z;
    int modeValue;  // 0 : manual, 1 : auto pilot, 2 : UI direct control
    int shift;      // 0 : Forward, 1 : Reverse
    int accel_stroke;
    int brake_stroke;
    int steering_angle;
    int headlight;
    int turninglight;
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
    vehicle_cmd.headlight = 0;
    vehicle_cmd.turninglight = 0;
}

bool update_cmd(vehicle_cmd_t &prev_vehicle_cmd)
{
    if (vehicle_cmd.linear_x != prev_vehicle_cmd.linear_x)
        return true;
    if (vehicle_cmd.angular_z != prev_vehicle_cmd.angular_z)
        return true;
    if (vehicle_cmd.modeValue != prev_vehicle_cmd.modeValue)
        return true;
    if (vehicle_cmd.shift != prev_vehicle_cmd.shift)
        return true;
    if (vehicle_cmd.accel_stroke != prev_vehicle_cmd.accel_stroke)
        return true;
    if (vehicle_cmd.brake_stroke != prev_vehicle_cmd.brake_stroke)
        return true;
    if (vehicle_cmd.steering_angle != prev_vehicle_cmd.steering_angle)
        return true;
    if (vehicle_cmd.headlight != prev_vehicle_cmd.headlight)
        return true;
    if (vehicle_cmd.turninglight != prev_vehicle_cmd.turninglight)
        return true;
    return false;
}

void modeCMDCallback(const tablet_socket_msgs::mode_cmd &mode)
{
    if (mode.mode == -1 || mode.mode == 0) {
        cmd_reset();
    }
    vehicle_cmd.modeValue = mode.mode;
}

void gearCMDCallback(const tablet_socket_msgs::gear_cmd &gear)
{
    vehicle_cmd.shift = gear.gear;
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

void vehicle_control() {}

bool ChangeShift() {}

void SteeringControl() {}

void PedalControl() {}

void checkRange() 
{
    vehicle_cmd.steering_angle = vehicle_cmd.steering_angle + STEERING_OFFSET;
    if (vehicle_cmd.steering_angle < 0) vehicle_cmd.steering_angle = 0;
    else if (vehicle_cmd.steering_angle > 1440) vehicle_cmd.steering_angle = 1440;

    vehicle_cmd.brake_stroke = vehicle_cmd.brake_stroke + BRAKE_OFFSET;
    if (vehicle_cmd.brake_stroke < 0) vehicle_cmd.brake_stroke = BRAKE_OFFSET;
    else if (vehicle_cmd.brake_stroke > 255) vehicle_cmd.brake_stroke = 255;

    if (vehicle_cmd.accel_stroke < 0) vehicle_cmd.accel_stroke = 0;
    else if (vehicle_cmd.accel_stroke > 255) vehicle_cmd.accel_stroke = 255;

    if (vehicle_cmd.shift < 0 || vehicle_cmd.shift > 3) {
        vehicle_cmd.shift = 0;
        std::cout << RED << "Shift is out of range." << RESET << std::endl;
    }

    if (vehicle_cmd.turninglight < 0 || vehicle_cmd.turninglight > 3) {
        vehicle_cmd.turninglight = 0;
        std::cout << RED << "Turning light is out of range." << RESET << std::endl;
    }

    if (vehicle_cmd.headlight < 0 || vehicle_cmd.headlight > 3) {
        vehicle_cmd.headlight = 0;
        std::cout << RED << "Head light is out of range." << RESET << std::endl;
    }
    return;
}

void showVehicleInfo()
{
    switch (vehicle_info.control_mode) {
    case MANUAL:
        std::cout << GREEN << "------ manual mode ------" << RESET << std::endl;
        break;
    case AUTONOMOUS:
        std::cout << YELLOW << "------ autonomous mode ------" << RESET << std::endl;
        break;
    case OVERRIDE:
        std::cout << BLUE << "------ override mode ------" << RESET << std::endl;
        break;
    case CARINIT:
        std::cout << RED << "------ car init mode ------" << RESET << std::endl;
        break;
    default:
        break;
    }

    switch (vehicle_info.shift) {
    case PARKING:
        std::cout << "Shift : Parking" << std::endl;
        break;
    case REVERSE:
        std::cout << "Shift : Reverse" << std::endl;
        break;
    case NEUTRAL:
        std::cout << "Shift : Neutral" << std::endl;
        break;
    case DRIVE:
        std::cout << "Shift : Drive" << std::endl;
        break;
    default:
        break;
    }

    std::cout << "Throttle : " << vehicle_info.throttle << std::endl;
    std::cout << "Brake : " << vehicle_info.brake << std::endl;
    std::cout << "Velocity : " << vehicle_info.velocity << std::endl;

    switch (vehicle_info.headlight) {
    case OFF:
        std::cout << "Head Light : OFF" << std::endl;
        break;
    case ON:
        std::cout << "Head Light : ON" << std::endl;
        break;
    default:
        break;
    }

    switch (vehicle_info.turninglight) {
    case NONE:
        std::cout << "Turning Light : OFF" << std::endl;
        break;
    case LEFT:
        std::cout << "Turning Light : Left" << std::endl;
        break;
    case RIGHT:
        std::cout << "Turning Light : Right" << std::endl;
        break;
    case BOTH:
        std::cout << "Turning Light : Both" << std::endl;
        break;
    default:
        break;
    }

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
