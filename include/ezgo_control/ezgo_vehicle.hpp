/*
 *  Copyright (C) MECLAB in National Cheng Kung University, Taiwan.
 *  All rights reserved.
 */

#include <math.h>
#include <pthread.h>
#include <ros/ros.h>
#include <signal.h>
#include <unistd.h>
#include <chrono>
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

#define KMH_TO_MS 3.6       /* [km/hr] -> [m/s] */
#define CHANGE_SHIFT_TIME 3 /* [sec] */
#define VELOCITY_BUFFER 3   /* [km/hr] */

#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

/* Define value meaning. */
enum HeadLight { OFF = 0, ON = 1 };

enum TurningLight { NONE = 0, LEFT = 1, RIGHT = 2, BOTH = 3 };

enum Shift { PARKING = 0, REVERSE = 1, NEUTRAL = 2, DRIVE = 3 };

enum Mode { MANUAL = 0, AUTONOMOUS = 1, OVERRIDE = 2, CARINIT = 3 };

enum SteeringStatus { STEERINGINIT = 0, STEERINGFREE = 1, STEERINGHOLD = 2 };

/*
 * Vehicle configuration.
 * These are defined by yaml file in config folder.
 */
typedef struct vehicle_config {
    float length;
    float hight;
    float width;
    float wheel_base;
    float wheel_angle_max;
    float steering_angle_max;
    int steering_offset;
    int brake_offset;
    double wheel_to_steering;
    float max_velocity;
    float profile_a;
    float profile_b;
    std::string steering_port;
} vehicle_config_t;

/*
 * Vehicle information in real-time.
 * These are provided by CANBus or serial port.
 * [VCU] --> [IPC]
 */
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

/*
 * Vehicle command in real-time.
 * These are send by CANBus or serial port to control vehicle.
 * [IPC] --> [VCU]
 */
typedef struct vehicle_cmd {
    double linear_x;
    double angular_z;
    int modeValue;  // 0 : manual, 1 : auto pilot, 2 : UI direct control
    int shift;
    int accel_stroke;
    int brake_stroke;
    int steering_angle;
    int headlight;
    int turninglight;
} vehicle_cmd_t;

/* Declare in ecu_control.cpp or arduino.cpp. */
extern vehicle_cmd_t vehicle_cmd;
extern vehicle_info_t vehicle_info;
extern vehicle_config_t vehicle_config;
extern int willExit;

bool loading_vehicle_config()
{
    ros::NodeHandle private_nh_("~");
    private_nh_.param<float>("/ecu_ezgo_vehicle_control/vehicle_config/length", vehicle_config.length, 2.4);
    private_nh_.param<float>("/ecu_ezgo_vehicle_control/vehicle_config/hight", vehicle_config.hight, 1.74);
    private_nh_.param<float>("/ecu_ezgo_vehicle_control/vehicle_config/width", vehicle_config.width, 0.97);
    private_nh_.param<float>("/ecu_ezgo_vehicle_control/vehicle_config/wheel_base", vehicle_config.wheel_base, 1.67);
    private_nh_.param<float>("/ecu_ezgo_vehicle_control/vehicle_config/wheel_angle_max", vehicle_config.wheel_angle_max, 1.67);
    private_nh_.param<float>("/ecu_ezgo_vehicle_control/vehicle_config/steering_angle_max", vehicle_config.steering_angle_max, 1.67);
    private_nh_.param<int>("/ecu_ezgo_vehicle_control/vehicle_config/steering_offset", vehicle_config.steering_offset, 1024);
    private_nh_.param<float>("/ecu_ezgo_vehicle_control/vehicle_config/max_velocity", vehicle_config.max_velocity, 30);
    private_nh_.param<float>("/ecu_ezgo_vehicle_control/vehicle_config/profile_a", vehicle_config.profile_a, 0.1989);
    private_nh_.param<float>("/ecu_ezgo_vehicle_control/vehicle_config/profile_b", vehicle_config.profile_b, -8.9443);
    private_nh_.param<std::string>("/ecu_ezgo_vehicle_control/vehicle_config/steering_port", vehicle_config.steering_port, "/dev/ttyACM0");
    private_nh_.param<int>("/ecu_ezgo_vehicle_control/vehicle_config/brake_offset", vehicle_config.brake_offset, 22);
    if (vehicle_config.wheel_angle_max != 0)
        vehicle_config.wheel_to_steering = vehicle_config.steering_angle_max / vehicle_config.wheel_angle_max;
    else
        return false;
    return true;
}

void showConfig() 
{
    std::cout << CYAN << "------- Vehicle Config -------" << std::endl;
    std::cout << "vehicle_config.length : " << vehicle_config.length << "[m]" << std::endl;
    std::cout << "vehicle_config.hight : " << vehicle_config.hight << "[m]" << std::endl;
    std::cout << "vehicle_config.width : " << vehicle_config.width << "[m]" << std::endl;
    std::cout << "vehicle_config.wheel_base : " << vehicle_config.wheel_base << "[m]" << std::endl;
    std::cout << "vehicle_config.wheel_angle_max : " << vehicle_config.wheel_angle_max << "[degrees]" << std::endl;
    std::cout << "vehicle_config.steering_angle_max : " << vehicle_config.steering_angle_max << "[degrees]" << std::endl;
    std::cout << "vehicle_config.max_velocity : " << vehicle_config.max_velocity << "[km/hr]" << std::endl;
    std::cout << "vehicle_config.steering_port : " << vehicle_config.steering_port << std::endl;
    std::cout << "vehicle_config.profile_a : " << vehicle_config.profile_a << std::endl;
    std::cout << "vehicle_config.profile_b : " << vehicle_config.profile_b << std::endl;
    std::cout << "vehicle_config.steering_offset : " << vehicle_config.steering_offset << std::endl;
    std::cout << "vehicle_config.brake_offset : " << vehicle_config.brake_offset << std::endl;
    std::cout << "------- Vehicle Config -------" << RESET << std::endl;
    return;
}

/* Reset vehicle command. */
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

/* Check vehicle command update or not. */
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

/* Callback function. */
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
    vehicle_cmd.brake_stroke = brake.brake + vehicle_config.brake_offset;
}

/*
 * Check if want to change shift.
 * If current velocity direction is not same as command velocity direction and
 * current velocity is not equal to zero, the command direction would be changed
 * to current direction and command velocity would be changed to scale 0.5 of
 * command velocity.
 *
 * @CHANGE_SHIFT_TIME: the waitting time for change shift.
 */
void CheckShift()
{
    if ((vehicle_info.shift == DRIVE && vehicle_cmd.linear_x < 0) || (vehicle_info.shift == REVERSE && vehicle_cmd.linear_x > 0)) {
        if (vehicle_info.velocity != 0) {
            vehicle_cmd.linear_x = vehicle_info.velocity > 1 ? vehicle_info.velocity / 2 : 0;
        } else {
            ros::Time start = ros::Time::now();
            while (1) {
                if (ros::Time::now().toSec() - start.toSec() > CHANGE_SHIFT_TIME)
                    break;
            }
        }
    }

    if (vehicle_cmd.linear_x < 0)
        vehicle_cmd.shift = REVERSE;
    else if (vehicle_cmd.linear_x == 0)
        vehicle_cmd.shift = PARKING;
    else
        vehicle_cmd.shift = DRIVE;
    return;
}

void SteeringControl()
{
    static float pre_cmd_steering_angle = 0.0;
    if (vehicle_cmd.linear_x < 0.1) {
        vehicle_cmd.steering_angle = pre_cmd_steering_angle;
    } else {
        double phi_angle_pi = (vehicle_cmd.angular_z / vehicle_cmd.linear_x);
        double wheel_angle_pi = atan(phi_angle_pi * vehicle_config.wheel_base);
        double wheel_angle = (wheel_angle_pi / M_PI) * 180.0;
        vehicle_cmd.steering_angle = wheel_angle * vehicle_config.wheel_to_steering;
        pre_cmd_steering_angle = vehicle_cmd.steering_angle;
    }
    return;
}

void PedalControl(double cmd_velocity, const double current_velocity)
{
    if (cmd_velocity > vehicle_config.max_velocity) cmd_velocity = vehicle_config.max_velocity; 
    if ((cmd_velocity + VELOCITY_BUFFER) > current_velocity && cmd_velocity != 0) {        /* acceleration */
        std::cout << BLUE << "ACCELERATION" << RESET << std::endl;
        vehicle_cmd.accel_stroke = (cmd_velocity - vehicle_config.profile_b) / vehicle_config.profile_a;
        vehicle_cmd.brake_stroke = 0;
    } else if ((cmd_velocity + VELOCITY_BUFFER) < current_velocity && cmd_velocity > 0) {  /* deceleration */
        std::cout << RED << "DECELERATION" << RESET << std::endl;
        vehicle_cmd.accel_stroke = 0;
        vehicle_cmd.brake_stroke = 40;
    } else if (cmd_velocity == 0 && current_velocity != 0) {                               /* stopping */
        std::cout << RED << "STOPPING" << RESET << std::endl;
        vehicle_cmd.accel_stroke = 0;
        vehicle_cmd.brake_stroke = 80;
    }
    return;
}

void vehicle_control()
{
    CheckShift();
    SteeringControl();
    PedalControl(fabs(vehicle_cmd.linear_x) * KMH_TO_MS, vehicle_info.velocity);
    return;
}

void checkRange()
{
    if (vehicle_cmd.steering_angle < -vehicle_config.steering_angle_max)
        vehicle_cmd.steering_angle = -vehicle_config.steering_angle_max;
    else if (vehicle_cmd.steering_angle > vehicle_config.steering_angle_max)
        vehicle_cmd.steering_angle = vehicle_config.steering_angle_max;

    if (vehicle_cmd.brake_stroke < vehicle_config.brake_offset)
        vehicle_cmd.brake_stroke = vehicle_config.brake_offset;
    else if (vehicle_cmd.brake_stroke > 255)
        vehicle_cmd.brake_stroke = 255;

    if (vehicle_cmd.accel_stroke < 0)
        vehicle_cmd.accel_stroke = 0;
    else if (vehicle_cmd.accel_stroke > 255)
        vehicle_cmd.accel_stroke = 255;

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
    if (vehicle_info.control_mode == MANUAL && vehicle_cmd.modeValue == 0) {
        std::cout << YELLOW << "------ Manual mode ------" << RESET << std::endl;
    } else if (vehicle_info.control_mode == AUTONOMOUS && vehicle_cmd.modeValue == 1) {
        std::cout << GREEN << "------ Autonomous mode ------" << RESET << std::endl;
    } else if (vehicle_info.control_mode == AUTONOMOUS && vehicle_cmd.modeValue == 2) {
        std::cout << GREEN << "------ UI direct control mode ------" << RESET << std::endl;
    } else {
        std::cout << RED << "------ Error mode ------" << RESET << std::endl;
        if (vehicle_info.control_mode == MANUAL)
            std::cout << "Exteral Switch : OFF [Maunal mode]" << std::endl;
        else
            std::cout << "Exteral Switch : ON [Autonomous mode]" << std::endl;
        if (vehicle_cmd.modeValue == 0)
            std::cout << "Software Control Value : 0 [Manual mode]" << std::endl;
        else if (vehicle_cmd.modeValue == 1)
            std::cout << "Software Control Value : 1 [Autonomous mode]" << std::endl;
        else
            std::cout << "Software Control Value : 2 [UI direct control mode]" << std::endl;
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
    
    std::cout << "Throttle [CMD] : " << vehicle_cmd.accel_stroke << ", Throttle : " << vehicle_info.throttle << std::endl;
    std::cout << "Brake [CMD] : " << vehicle_cmd.brake_stroke << ", Brake : " << vehicle_info.brake << std::endl;
    std::cout << "Steering Angle [CMD] : " << vehicle_cmd.steering_angle << ", Steering Angle : " << vehicle_info.steering_angle << std::endl;
    std::cout << "Velocity : " << vehicle_info.velocity << " [km/hr]" << std::endl;

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
