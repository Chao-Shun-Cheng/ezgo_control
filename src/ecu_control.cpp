#include <canlib.h>
#include <ezgo_control/ezgo_vehicle.hpp>

#define CAN_CHANNEL 0
#define CAN_BITRATE BAUD_500K
#define CAN_INFO_READ_TIMEOUT_INTERVAL 2

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

canStatus Kvaser_canbus_write(long id, char msg[], int len)
{
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

    stat = canWrite(hnd, id, msg, len, CAN_CHANNEL);
    stat = canWriteSync(hnd, 25);
    return stat;
}

static void *CAN_Info_Sender(void *args)
{
    std::cout << YELLOW << "ENTER ezgo_control CAN_Info_Sender thread.\n" << RESET << std::endl;
    std::cout << GREEN << "[ezgo_control::CAN_Info_Sender] can open done." << RESET << std::endl;

    cmd_reset();
    
    while (ros::ok() && !willExit) { // TODO
        switch (vehicle_cmd.modeValue) {
        case 0:
            cmd_reset();
            break;
        case 1:  // autonomous mode
            cmd_reset();
            break;
        case 2:  // UI direct control
            cmd_reset();
            break;
        }
    }
    std::cout << YELLOW << "EXIT ezgo_control CAN_Info_Sender thread.\n" << RESET << std::endl;
    return nullptr;
}

static void *CAN_Info_Receiver(void *args)
{
    std::cout << YELLOW << "ENTER ezgo_control CAN_Info_Receiver thread.\n" << RESET << std::endl;
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
    std::cout << "can info recv: canOpenChannel " << CAN_CHANNEL << std::endl;
    std::cout << GREEN << "[ezgo_control::CAN_Info_Receiver] can open done." << RESET << std::endl;

    while (ros::ok() && !willExit) {
        stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, CAN_INFO_READ_TIMEOUT_INTERVAL);
        if (stat == canOK) {
            // TODO : Parse Can message.
        }
    }
    std::cout << YELLOW << "EXIT ezgo_control CAN_Info_Receiver thread.\n" << RESET << std::endl;
    return nullptr;
}

int main(int argc, char **argv)
{
    int ret = app_setup_signals();
    if (ret == -1) {
        printf("Fail to app_setup_signals\n");
        return -1;
    }

    ros::init(argc, argv, "ecu_cotrol");
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

    ros::AsyncSpinner spinner(4);  // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}