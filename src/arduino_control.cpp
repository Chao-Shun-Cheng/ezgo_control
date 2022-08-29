#include <ezgo_control/SerialPort.hpp>
#include <ezgo_control/ezgo_vehicle.hpp>

#define WRITE_LENGTH 7
#define READ_LENGTH 7
using namespace mn::CppLinuxSerial;

pthread_mutex_t mutex;
vehicle_info_t vehicle_info;
vehicle_cmd_t vehicle_cmd;
vehicle_config_t vehicle_config;
int willExit = 0;

void arduino_serial_write(SerialPort *serialPort)
{
    static std::vector<uint8_t> msg(WRITE_LENGTH, 0);
    int16_t int16_angle = (int16_t) vehicle_cmd.steering_angle;
    msg[0] = vehicle_cmd.brake_stroke;
    msg[1] = vehicle_cmd.accel_stroke;
    msg[4] = vehicle_cmd.shift;
    msg[5] = vehicle_cmd.turninglight;
    msg[6] = vehicle_cmd.headlight;

    // Byte order: Big-endian
    msg[2] = (int16_angle >> 8) & 0x00ff;
    msg[3] = int16_angle & 0x00ff;

    pthread_mutex_lock(&mutex);
    serialPort->WriteBinary(msg);
    pthread_mutex_unlock(&mutex);
    return;
}

static void *writer_handler(void *args)
{
    std::cout << YELLOW << "ENTER ezgo_vehicle_control Writer thread.\n" << RESET << std::endl;
    SerialPort *serialPort = (SerialPort *) args;

    vehicle_cmd_t prev_vehicle_cmd;
    ros::Rate rate(50);
    cmd_reset();

    while (ros::ok() && !willExit) {
        ros::spinOnce();
        if (update_cmd(prev_vehicle_cmd)) {
            switch (vehicle_cmd.modeValue) {
            case 0:
                cmd_reset();
                break;
            case 1:  // autonomous mode
                vehicle_control();
                checkRange();
                arduino_serial_write(serialPort);
                break;
            case 2:  // UI direct control
                checkRange();
                arduino_serial_write(serialPort);
                break;
            }
            prev_vehicle_cmd = vehicle_cmd;
        }
        rate.sleep();
    }
    std::cout << YELLOW << "EXIT ezgo_vehicle_control Writer thread.\n" << RESET << std::endl;
    return nullptr;
}

static void *reader_handler(void *args)
{
    std::cout << YELLOW << "ENTER ezgo_vehicle_control Reader thread.\n" << RESET << std::endl;
    SerialPort *serialPort = (SerialPort *) args;
    while (ros::ok() && !willExit) {
        std::string readData;
        pthread_mutex_lock(&mutex);
        serialPort->Read(readData);
        pthread_mutex_unlock(&mutex);
        if (readData.size() == READ_LENGTH) {
            vehicle_info.brake = readData[0];
            vehicle_info.throttle = readData[1];
            vehicle_info.steering_angle =
                (float) ((int16_t)(((readData[2] << 8) & 0xff00) + readData[3]) - vehicle_config.steering_offset);
            vehicle_info.velocity = (float) ((int16_t)(((readData[4] << 8) & 0xff00) + readData[5]));
            vehicle_info.velocity /= 1000.0;
            vehicle_info.shift = readData[6] & 0x03;
            vehicle_info.turninglight = (readData[6] & 0x0c) >> 2;
            vehicle_info.headlight = (readData[6] & 0x10) >> 4;
            vehicle_info.control_mode = (readData[6] & 0x20) >> 5;
            showVehicleInfo();
        } else {
            std::cout << RED << "Without Receive DATA, Check connect ...." << RESET << std::endl;
        }
    }
    std::cout << YELLOW << "EXIT ezgo_vehicle_control Reader thread.\n" << RESET << std::endl;
    return nullptr;
}

int main(int argc, char **argv)
{
    int ret = app_setup_signals();
    if (ret == -1) {
        printf("Fail to app_setup_signals\n");
        return -1;
    }

    ros::init(argc, argv, "arduino_ezgo_vehicle_control");
    ros::NodeHandle nh;

    if (!loading_vehicle_config()) {
        ROS_ERROR("Can't load vehicle config!");
        return 0;
    }

    ros::Subscriber sub[6];
    sub[0] = nh.subscribe("/twist_cmd", 1, twistCMDCallback);
    sub[1] = nh.subscribe("/mode_cmd", 1, modeCMDCallback);
    sub[2] = nh.subscribe("/gear_cmd", 1, gearCMDCallback);
    sub[3] = nh.subscribe("/accel_cmd", 1, accellCMDCallback);
    sub[4] = nh.subscribe("/steer_cmd", 1, steerCMDCallback);
    sub[5] = nh.subscribe("/brake_cmd", 1, brakeCMDCallback);

    pthread_t thread_writer;
    pthread_t thread_reader;

    // Create serial port object and open serial port
    SerialPort serialPort("/dev/ttyACM0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialPort.SetTimeout(-1);
    serialPort.Open();

    if (pthread_create(&thread_writer, NULL, writer_handler, &serialPort)) {
        perror("could not create thread for writer_handler");
        return -1;
    }

    if (pthread_create(&thread_reader, NULL, reader_handler, &serialPort)) {
        perror("could not create thread for reader_handler");
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
    serialPort.Close();
    return 0;
}
