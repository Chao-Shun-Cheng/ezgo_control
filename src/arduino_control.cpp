#include <ezgo_control/SerialPort.hpp>
#include <ezgo_control/ezgo_vehicle.hpp>

using namespace mn::CppLinuxSerial;

pthread_mutex_t mutex;
vehicle_info_t vehicle_info;
vehicle_cmd_t vehicle_cmd;
int willExit = 0;

static void *writer_handler(void *args)
{
    std::cout << YELLOW << "ENTER ezgo_vehicle_control Writer thread.\n" << RESET << std::endl;
    SerialPort *serialPort = (SerialPort *) args;
	cmd_reset();
	
    while (ros::ok() && !willExit) {
        ros::spinOnce();
        switch (vehicle_cmd.modeValue) {
        case 0:
            // std::cout << "In case 0.\n" << std::endl;
            cmd_reset();
            break;
        case 1:  // autonomous mode
            // std::cout << "In case 1.\n" << std::endl;
            cmd_reset();
            break;
        case 2:  // UI direct control
            // std::cout << "In case 2.\n" << std::endl;
            std::vector<uint8_t> data;
            data.push_back((uint8_t) vehicle_cmd.accel_stroke);
            data.push_back((uint8_t) vehicle_cmd.brake_stroke);
            data.push_back((uint8_t) vehicle_cmd.shift);
            pthread_mutex_lock(&mutex);
            serialPort->WriteBinary(data);
            // for (int i = 0; i < data.size(); i++){
            //     std::cout << "data[" << i << "]: " << unsigned(data[i]) << ", ";
            // }
            std::cout<< std::endl;
            pthread_mutex_unlock(&mutex);
            break;
        }
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
        if (readData.size() == 5) {
            // std::cout << "readData[0]: "<< int(readData[0]) << std::endl;
            vehicle_info.throttle = (uint8_t) readData[0];
            vehicle_info.brake = (uint8_t) readData[1];
            vehicle_info.control_mode = (uint8_t) readData[2] & 0x01;
            vehicle_info.light = (uint8_t) readData[2] & 0x02;
            vehicle_info.shift = (uint8_t) readData[2] & 0x04;
            uint16_t vel = 0;
            vel = (((uint16_t) readData[3]) << 8) & 0xFF00;
            vel |= ((uint16_t) readData[4]) & 0xFF;
            vehicle_info.velocity = ((float) vel) / 1000;
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

    ros::init(argc, argv, "arduino_control");
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

    ros::AsyncSpinner spinner(4);  // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    serialPort.Close();
    return 0;
}
