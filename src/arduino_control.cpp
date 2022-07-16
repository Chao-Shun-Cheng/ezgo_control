#include <ezgo_control/SerialPort.hpp>
#include <ezgo_control/ezgo_vehicle.hpp>

using namespace mn::CppLinuxSerial;

pthread_mutex_t mutex;
vehicle_info_t vehicle_info;
vehicle_cmd_t vehicle_cmd;
int willExit = 0;

static void *writer_handler(void *args) 
{
	printf("ENTER ezgo_vehicle_control Writer thread.\n");
	SerialPort *serialPort = (SerialPort *) args;
	
	while (ros::ok() && !willExit) {
		switch (vehicle_cmd.modeValue) {
		case 0:
			cmd_reset();
			break;
		case 1: // autonomous mode
			cmd_reset();
			break; 
		case 2: // UI direct control
			std::vector<uint8_t> data;
			data.push_back((uint8_t) vehicle_cmd.accel_stroke);
			data.push_back((uint8_t) vehicle_cmd.brake_stroke);
			data.push_back((uint8_t) vehicle_cmd.shift);
			pthread_mutex_lock(&mutex);
			serialPort->WriteBinary(data);
			pthread_mutex_unlock(&mutex);
			break;
		}
	}
	printf("EXIT ezgo_vehicle_control Writer thread.\n");
	return nullptr;
}

static void *reader_handler(void *args) 
{
	printf("ENTER ezgo_vehicle_control Reader thread.\n");
	SerialPort *serialPort = (SerialPort *) args;
	while(ros::ok() && !willExit) {
        std::string readData;
		pthread_mutex_lock(&mutex);
        serialPort->Read(readData);
		pthread_mutex_unlock(&mutex);
		if (readData.size() == 5) {
			vehicle_info.throttle = readData[0];
			vehicle_info.brake = readData[1];
			vehicle_info.control_mode = (bool) readData[2] & 0x1;
			vehicle_info.light = (bool) readData[2] & 0x2;
			vehicle_info.shift = (bool) readData[2] & 0x4;
			uint16_t vel = 0;
			vel = (((uint16_t) readData[3]) << 8) & 0xFF00;
			vel |= ((uint16_t) readData[4]) & 0xFF;
			vehicle_info.velocity = ((float) vel) / 1000; 
			showVehicleInfo();
		} else {
			std::cout << RED << "Without Receive DATA, Check connect ...." << RESET << std::endl;
		}
    }
	printf("EXIT ezgo_vehicle_control Reader thread.\n");
	return nullptr;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "arduino_cotrol");
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

    while (!willExit)
        ros::waitForShutdown();

	serialPort.Close();
	return 0;
}

