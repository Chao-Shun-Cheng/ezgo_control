#include <canlib.h>
#include <ezgo_control/ezgo_vehicle.hpp>

#define CAN_CHANNEL 0
#define CAN_BITRATE BAUD_250K
#define CAN_INFO_READ_TIMEOUT_INTERVAL 2
#define CAN_WRITE_ID 0x040
#define CAN_WRITE_DLC 7
#define CAN_READ_ID1 0x060
#define CAN_READ_ID2 0x061



pthread_mutex_t mutex;
vehicle_info_t vehicle_info;
vehicle_cmd_t vehicle_cmd;
int willExit = 0;

static void checkCAN(const char *id, canStatus stat)
{
    if (stat != canOK) {
        char buf[50];
        buf[0] = '\0';
        // Retreive informational text about the status code
        canGetErrorText(stat, buf, sizeof(buf));
        std::cout << RED << id << " : failed, stat = " << (int) stat << " " << buf << RESET << std::endl;
    }
}

canStatus Kvaser_canbus_write()
{
    static char msg[CAN_WRITE_DLC];
    int16_t int16_angle = (int16_t) vehicle_cmd.steering_angle;
    msg[0] = vehicle_cmd.brake_stroke;
    msg[1] = vehicle_cmd.accel_stroke;
    msg[4] = vehicle_cmd.shift;
    msg[5] = vehicle_cmd.turninglight;
    msg[6] = vehicle_cmd.headlight;

    // Byte order: Big-endian
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

static void *CAN_Info_Sender(void *args)
{
    std::cout << YELLOW << "ENTER ezgo_control CAN_Info_Sender thread." << RESET << std::endl;
    std::cout << GREEN << "[ezgo_control::CAN_Info_Sender] can open done." << RESET << std::endl;

    vehicle_cmd_t prev_vehicle_cmd;
    ros::Rate rate(50);
    cmd_reset();

    while (ros::ok() && !willExit) {  // TODO
        ros::spinOnce();
        if (update_cmd(prev_vehicle_cmd)) {
            switch (vehicle_cmd.modeValue) {
            case 0:
                cmd_reset();
                break;
            case 1:  // autonomous mode
                vehicle_control();
                checkRange();
                Kvaser_canbus_write();
                break;
            case 2:  // UI direct control
                checkRange();
                Kvaser_canbus_write();
                break;
            }
            prev_vehicle_cmd = vehicle_cmd;
        }
        rate.sleep();
    }

    std::cout << YELLOW << "EXIT ezgo_control CAN_Info_Sender thread." << RESET << std::endl;
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
                vehicle_info.steering_angle = (float) ((int16_t)(((msg[2] << 8) & 0xff00) + msg[3]) - STEERING_OFFSET);
                vehicle_info.shift = msg[4];
                vehicle_info.turninglight = msg[5];
                vehicle_info.headlight = msg[6];
            } else if (id == CAN_READ_ID2 && dlc == 5) {
                vehicle_info.control_mode = msg[0];
                vehicle_info.velocity = (float) ((int16_t)(((msg[2] << 8) & 0xff00) + msg[3]));
                vehicle_info.velocity /= 1000.0;
            }
            showVehicleInfo();
        }
    }
    std::cout << YELLOW << "EXIT ezgo_control CAN_Info_Receiver thread." << RESET << std::endl;
    return nullptr;
}

int main(int argc, char **argv)
{
    int ret = app_setup_signals();
    if (ret == -1) {
        printf("Fail to app_setup_signals\n");
        return -1;
    }

    ros::init(argc, argv, "ecu_control");
    ros::NodeHandle nh;

    ros::Subscriber sub[6];
    sub[0] = nh.subscribe("/twist_cmd", 1, twistCMDCallback);
    sub[1] = nh.subscribe("/mode_cmd", 1, modeCMDCallback);
    sub[2] = nh.subscribe("/gear_cmd", 1, gearCMDCallback);
    sub[3] = nh.subscribe("/accel_cmd", 1, accellCMDCallback);
    sub[4] = nh.subscribe("/steer_cmd", 1, steerCMDCallback);
    sub[5] = nh.subscribe("/brake_cmd", 1, brakeCMDCallback);

    pthread_t thread_writer;
    pthread_t thread_reader;

    if (pthread_create(&thread_writer, NULL, CAN_Info_Sender, NULL)) {
        perror("could not create thread for CAN_Info_Sender");
        return -1;
    }

    if (pthread_create(&thread_reader, NULL, CAN_Info_Receiver, NULL)) {
        perror("could not create thread for CAN_Info_Receiver");
        return -1;
    }

    if (pthread_detach(thread_writer) != 0) {
        std::perror("pthread_detach");
        std::exit(1);
    }

    if (pthread_detach(thread_reader) != 0) {
        std::perror("pthread_detach");
        std::exit(1);
    }

    ros::waitForShutdown();

    return 0;
}