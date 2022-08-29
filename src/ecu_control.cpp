#include <canlib.h>
#include <ezgo_control/SerialPort.hpp>
#include <ezgo_control/ezgo_vehicle.hpp>

using namespace mn::CppLinuxSerial;

#define CAN_CHANNEL 0
#define CAN_BITRATE BAUD_250K
#define CAN_INFO_READ_TIMEOUT_INTERVAL 2
#define CAN_WRITE_ID 0x040
#define CAN_WRITE_DLC 7
#define CAN_READ_ID1 0x060
#define CAN_READ_ID2 0x061

/*
 * pulse to degree
 * This is change pulse of steering motor to steering degree.
 * 360 / 3200 / 6 / 2
 * degree per cycle / pulse per cycle / 1st stage gear ratio / 2nd stage gear
 * ratio
 */
#define pulse_to_degree 0.009375
#define angle_write(pluse) ("abs " + pluse + "@")
#define wait_serial_time 0.05 /* [50 ms] */
#define wait_serial(serial_time) while (GetTimeDiffNow(serial_time) > wait_serial_time)

pthread_mutex_t mutex;
vehicle_info_t vehicle_info;
vehicle_cmd_t vehicle_cmd;
vehicle_config_t vehicle_config;
int willExit = 0;
/* The time is used to make sure the serial write rate > 10 ms */
static struct timespec serial_time;

static void checkCAN(const char *id, canStatus stat)
{
    if (stat != canOK) {
        char buf[50];
        buf[0] = '\0';
        /* Retreive informational text about the status code */
        canGetErrorText(stat, buf, sizeof(buf));
        std::cout << RED << id << " : failed, stat = " << (int) stat << " " << buf << RESET << std::endl;
    }
}

canStatus Kvaser_canbus_write()
{
    static char msg[CAN_WRITE_DLC];
    uint16_t int16_angle = (uint16_t)(vehicle_cmd.steering_angle + vehicle_config.steering_offset);
    msg[0] = vehicle_cmd.brake_stroke;
    msg[1] = vehicle_cmd.accel_stroke;
    msg[4] = vehicle_cmd.shift;
    msg[5] = vehicle_cmd.turninglight;
    msg[6] = vehicle_cmd.headlight;

    /* Byte order: Big-endian */
    msg[2] = (int16_angle >> 8) & 0x00ff;
    msg[3] = int16_angle & 0x00ff;

    static canHandle hnd = -1;
    canStatus stat;

    if (hnd < 0) {
        /* Open channels, parameters and go on bus */
        hnd = canOpenChannel(CAN_CHANNEL, 0);
        if (hnd < 0) {
            checkCAN("canOpenChannel", (canStatus) hnd);
            return canERR_INVHANDLE;
        }
        canSetBusParams(hnd, CAN_BITRATE, 0, 0, 0, 0, 0);
        canSetBusOutputControl(hnd, canDRIVER_NORMAL);
        canBusOn(hnd);
    }

    stat = canWrite(hnd, CAN_WRITE_ID, msg, CAN_WRITE_DLC, CAN_CHANNEL);
    checkCAN("canWrite", stat);
    stat = canWriteSync(hnd, 25);
    checkCAN("canWrite", stat);
    return stat;
}

void serial_steering_write(SerialPort *serialPort)
{
    int pulse = (int) (vehicle_cmd.steering_angle / pulse_to_degree);
    bool findEnd = false;
    pthread_mutex_lock(&mutex);
    while (GetTimeDiffNow(serial_time) < wait_serial_time) 
        ;
    serialPort->Write(angle_write(std::to_string(pulse)));
    GetTickCount(serial_time);
    pthread_mutex_unlock(&mutex);
    return;
}

void serial_steering_read(SerialPort *serialPort)
{
    bool findEnd = false;
    int sign = 0;
    int pluse = 0;
    pthread_mutex_lock(&mutex);
    while (GetTimeDiffNow(serial_time) < wait_serial_time)
        ;
    serialPort->Write("rabs@");
    while (serialPort->Available() && !findEnd) {
        std::string readData;
        serialPort->Read(readData);
        for (int i = 0; i < readData.size(); i++) {
            if (readData[i] == '+')
                sign = 1;
            if (readData[i] == '-')
                sign = -1;
            if (readData[i] >= '0' && readData[i] <= '9')
                pluse = pluse * 10 + (readData[i] - '0');
            if (readData[i] == '!') {
                findEnd = true;
                break;
            }
        }
    }
    GetTickCount(serial_time);
    pthread_mutex_unlock(&mutex);
    if (findEnd && sign != 0)
        vehicle_info.steering_angle = sign * pluse * pulse_to_degree;
    return;
}

void hold_steering(SerialPort *serialPort)
{
    bool findEnd = false;
    pthread_mutex_lock(&mutex);
    while (GetTimeDiffNow(serial_time) < wait_serial_time)
        ;
    serialPort->Write("hold 0@");
    GetTickCount(serial_time);
    pthread_mutex_unlock(&mutex);
    return;
}

void free_steering(SerialPort *serialPort)
{
    bool findEnd = false;
    pthread_mutex_lock(&mutex);
    while (GetTimeDiffNow(serial_time) < wait_serial_time)
        ;
    serialPort->Write("hold 1@");
    GetTickCount(serial_time);
    pthread_mutex_unlock(&mutex);
    return;
}

static void *CAN_SERIAL_Info_Sender(void *args)
{
    std::cout << YELLOW << "ENTER ezgo_control CAN_SERIAL_Info_Sender thread." << RESET << std::endl;
    std::cout << GREEN << "[ezgo_control::CAN_SERIAL_Info_Sender] can open done." << RESET << std::endl;

    SerialPort *serialPort = (SerialPort *) args;

    int steering_status = STEERINGINIT;
    ros::Rate rate(10);
    cmd_reset();

    while (ros::ok() && !willExit) {
        ros::spinOnce();
        switch (vehicle_cmd.modeValue) {
        case 1: /* autonomous driving control */
            if (vehicle_info.control_mode == AUTONOMOUS) {
                vehicle_control();
                checkRange();
                Kvaser_canbus_write();
                if (steering_status != STEERINGHOLD) {
                    hold_steering(serialPort);
                    steering_status = STEERINGHOLD;
                }
                serial_steering_write(serialPort);
            } else {
                if (steering_status != STEERINGFREE) {
                    free_steering(serialPort);
                    steering_status = STEERINGFREE;
                }
                // cmd_reset();
                std::cout << RED << "Check Vehicle Contorl Switch ..." << RESET << std::endl;
            }
            break;
        case 2: /* UI direct control */
            if (vehicle_info.control_mode == AUTONOMOUS) {
                checkRange();
                Kvaser_canbus_write();
                if (steering_status != STEERINGHOLD) {
                    hold_steering(serialPort);
                    steering_status = STEERINGHOLD;
                }
                serial_steering_write(serialPort);
            } else {
                if (steering_status != STEERINGFREE) {
                    free_steering(serialPort);
                    steering_status = STEERINGFREE;
                }
                // cmd_reset();
                std::cout << RED << "Check Vehicle Contorl Switch ..." << RESET << std::endl;
            }
            break;
        default:
            if (steering_status != STEERINGFREE) {
                free_steering(serialPort);
                steering_status = STEERINGFREE;
            }
            cmd_reset();
            break;
        }
        rate.sleep();
    }

    std::cout << YELLOW << "EXIT ezgo_control CAN_Info_Sender thread." << RESET << std::endl;
    return nullptr;
}

static void *SERIAL_Info_Receiver(void *args)
{
    std::cout << YELLOW << "ENTER ezgo_control SERIAL_Info_Receiver thread." << RESET << std::endl;
    SerialPort *serialPort = (SerialPort *) args;
    ros::Rate rate(10);
    while (ros::ok() && !willExit) {
        serial_steering_read(serialPort);
        rate.sleep();
    }
    std::cout << YELLOW << "EXIT ezgo_control SERIAL_Info_Receiver thread." << RESET << std::endl;
    return nullptr;
}

static void *CAN_Info_Receiver(void *args)
{
    std::cout << YELLOW << "ENTER ezgo_control CAN_Info_Receiver thread." << RESET << std::endl;
    canHandle hnd = -1;
    canStatus stat;
    long id;
    unsigned char echo_status = 0;
    unsigned int flag;
    unsigned long time;
    unsigned char msg[8];
    unsigned int dlc;

    hnd = canOpenChannel(CAN_CHANNEL, 0);
    if (hnd < 0) {
        checkCAN("canOpenChannel", (canStatus) hnd);
        return nullptr;
    }
    stat = canSetBusParams(hnd, CAN_BITRATE, 0, 0, 0, 0, 0);
    checkCAN("canSetBusParams", stat);
    canBusOn(hnd);
    std::cout << GREEN << "[ezgo_control::CAN_Info_Receiver] can open done, Channel " << CAN_CHANNEL << RESET << std::endl;

    while (ros::ok() && !willExit) {
        stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, CAN_INFO_READ_TIMEOUT_INTERVAL);
        if (stat == canOK) {
            if (id == CAN_READ_ID1 && dlc == 7) {
                vehicle_info.brake = msg[0];
                vehicle_info.throttle = msg[1];
                // vehicle_info.steering_angle = (float) ((uint16_t)(((msg[2] <<
                // 8) & 0xff00) + msg[3]) - vehicle_config.steering_offset);
                vehicle_info.shift = msg[4];
                vehicle_info.turninglight = msg[5];
                vehicle_info.headlight = msg[6];
            } else if (id == CAN_READ_ID2 && dlc == 5) {
                vehicle_info.control_mode = msg[0];
                vehicle_info.velocity = (float) ((uint16_t)(((msg[1] << 8) & 0xff00) + msg[2]));
                vehicle_info.velocity /= 1000.0;
            } 
        }
        showVehicleInfo();
    }
    std::cout << YELLOW << "EXIT ezgo_control CAN_Info_Receiver thread." << RESET << std::endl;
    return nullptr;
}

bool init_steering_angle(SerialPort *serialPort)
{
    serialPort->Write("plc 2@");
    GetTickCount(serial_time);
    while (GetTimeDiffNow(serial_time) < wait_serial_time)
        ;
    serialPort->Write("sabs 0@");
    GetTickCount(serial_time);
    while (GetTimeDiffNow(serial_time) < wait_serial_time)
        ;
    serialPort->Write("hold 1@");
    GetTickCount(serial_time);
    return true;
}

int main(int argc, char **argv)
{
    int ret = app_setup_signals();
    if (ret == -1) {
        printf("Fail to app_setup_signals\n");
        return -1;
    }

    ros::init(argc, argv, "ecu_ezgo_vehicle_control");
    ros::NodeHandle nh;

    if (!loading_vehicle_config()) {
        ROS_ERROR("Can't load vehicle config!");
        return 0;
    } else {
        showConfig();
    }

    SerialPort serialPort(vehicle_config.steering_port, BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialPort.SetTimeout(-1);
    serialPort.Open();
    std::cout << GREEN << "Open serial port for steering" << RESET << std::endl;

    if (!init_steering_angle(&serialPort)) {
        ROS_ERROR("Can't initialize steering controller!");
        return 0;
    }

    ros::Subscriber sub[6];
    sub[0] = nh.subscribe("/twist_cmd", 1, twistCMDCallback);
    sub[1] = nh.subscribe("/mode_cmd", 1, modeCMDCallback);
    sub[2] = nh.subscribe("/gear_cmd", 1, gearCMDCallback);
    sub[3] = nh.subscribe("/accel_cmd", 1, accellCMDCallback);
    sub[4] = nh.subscribe("/steer_cmd", 1, steerCMDCallback);
    sub[5] = nh.subscribe("/brake_cmd", 1, brakeCMDCallback);

    ros::Publisher vel_pub;
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("can_velocity", 10);

    pthread_t thread_CAN_SERIAL_writer;
    pthread_t thread_CAN_reader;
    pthread_t thread_SERIAL_reader;

    if (pthread_create(&thread_CAN_SERIAL_writer, NULL, CAN_SERIAL_Info_Sender, &serialPort)) {
        perror("could not create thread for CAN_Info_Sender");
        return -1;
    }

    if (pthread_create(&thread_CAN_reader, NULL, CAN_Info_Receiver, NULL)) {
        perror("could not create thread for CAN_Info_Receiver");
        return -1;
    }

    if (pthread_create(&thread_SERIAL_reader, NULL, SERIAL_Info_Receiver, &serialPort)) {
        perror("could not create thread for CAN_Info_Receiver");
        return -1;
    }

    if (pthread_detach(thread_CAN_SERIAL_writer) != 0) {
        std::perror("pthread_detach");
        std::exit(1);
    }

    if (pthread_detach(thread_CAN_reader) != 0) {
        std::perror("pthread_detach");
        std::exit(1);
    }

    if (pthread_detach(thread_SERIAL_reader) != 0) {
        std::perror("pthread_detach");
        std::exit(1);
    }

    ros::Rate rate(50);
    geometry_msgs::TwistStamped velocity;
    while (ros::ok() && !willExit) {
        velocity.header.stamp = ros::Time::now();
        velocity.twist.linear.x = vehicle_info.velocity / KMH_TO_MS;
        vel_pub.publish(velocity);
        rate.sleep();
    }

    ros::waitForShutdown();

    return 0;
}