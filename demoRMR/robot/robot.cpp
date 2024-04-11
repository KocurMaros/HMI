#include "robot.h"


#ifdef _WIN32
#include <windows.h>

#else
#include <termios.h>
#include <unistd.h>
#include "unistd.h"
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include "signal.h"
#endif

#include <iostream>

std::function<int(TKobukiData)> Robot::do_nothing_robot = [](TKobukiData data) {
	std::cout << "data z kobuki " << std::endl;
	return 0;
};
std::function<int(LaserMeasurement)> Robot::do_nothing_laser = [](LaserMeasurement data) {
	std::cout << "data z rplidar " << std::endl;
	return 0;
};

Robot::~Robot()
{
	ready_promise.set_value();
	robotthreadHandle.join();
	laserthreadHandle.join();
	camerathreadhandle.join();
	skeletonthreadHandle.join();

	close(rob_s);
	close(las_s);
	close(ske_s);
#ifdef _WIN32
	WSACleanup();
#endif
}

Robot::Robot(std::string ipaddressLaser, int laserportRobot, int laserportMe, std::function<int(LaserMeasurement)> &lascallback, std::string ipaddressRobot,
			 int robotportRobot, int robotportMe, std::function<int(TKobukiData)> &robcallback)
	: wasLaserSet(0)
	, wasRobotSet(0)
	, wasCameraSet(0)
	, wasSkeletonSet(0)
	, m_connected(false)
	, m_emgStop(false)
{
	setLaserParameters(ipaddressLaser, laserportRobot, laserportMe, lascallback);
	setRobotParameters(ipaddressRobot, robotportRobot, robotportMe, robcallback);
	readyFuture = ready_promise.get_future();
}

///tato funkcia vas nemusi zaujimat
/// toto je funkcia s nekonecnou sluckou,ktora cita data z robota (UDP komunikacia)
void Robot::robotprocess()
{
#ifdef _WIN32
	WSADATA wsaData = { 0 };
	int iResult = 0;
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
	rob_slen = sizeof(las_si_other);
	if ((rob_s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		std::cerr << __FUNCTION__ << ":" << __LINE__ << ": " << strerror(errno) << std::endl;
		exit(-1);
	}

	char rob_broadcastene = 1;
#ifdef _WIN32
	DWORD timeout = 100;
	::setsockopt(rob_s, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof timeout);
#endif
	::setsockopt(rob_s, SOL_SOCKET, SO_BROADCAST, &rob_broadcastene, sizeof(rob_broadcastene));
	// zero out the structure
	memset((char *)&rob_si_me, 0, sizeof(rob_si_me));

	rob_si_me.sin_family = AF_INET;
	rob_si_me.sin_port = htons(robot_ip_portOut);
	rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	rob_si_posli.sin_family = AF_INET;
	rob_si_posli.sin_port = htons(robot_ip_portIn);
	rob_si_posli.sin_addr.s_addr = inet_addr(robot_ipaddress.data()); //inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
	rob_slen = sizeof(rob_si_me);
	if (::bind(rob_s, (struct sockaddr *)&rob_si_me, sizeof(rob_si_me)) != 0) {
		std::cerr << __FUNCTION__ << ":" << __LINE__ << ": " << strerror(errno) << std::endl;
		exit(-1);
	}


	std::vector<unsigned char> mess = robot.setDefaultPID();
	if (::sendto(rob_s, (char *)mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *)&rob_si_posli, rob_slen) == -1) {
		std::cerr << __FUNCTION__ << ":" << __LINE__ << ": " << strerror(errno) << std::endl;
		exit(-1);
	}
#ifdef _WIN32
	Sleep(100);
#else
	usleep(100 * 1000);
#endif
	mess = robot.setSound(440, 1000);
	if (::sendto(rob_s, (char *)mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *)&rob_si_posli, rob_slen) == -1) { }
	unsigned char buff[50000];

	while (1) {
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
			break;
		memset(buff, 0, 50000 * sizeof(char));
		if ((rob_recv_len = ::recvfrom(rob_s, (char *)&buff, sizeof(char) * 50000, 0, (struct sockaddr *)&rob_si_other, &rob_slen)) == -1) {
			continue;
		}
		if (rob_si_other.sin_addr.s_addr != inet_addr(robot_ipaddress.data())) {
			continue;
		}
		//https://i.pinimg.com/236x/1b/91/34/1b9134e6a5d2ea2e5447651686f60520--lol-funny-funny-shit.jpg
		//tu mame data..zavolame si funkciu

		//	 memcpy(&sens,buff,sizeof(sens));
		struct timespec t;
		//	  clock_gettime(CLOCK_REALTIME,&t);

		int returnval = robot.fillData(sens, (unsigned char *)buff);
		if (returnval == 0) {
			//	 memcpy(&sens,buff,sizeof(sens));

			std::chrono::steady_clock::time_point timestampf = std::chrono::steady_clock::now();

			m_connected = true;

			///---toto je callback funkcia...
			auto thread = std::async(
				std::launch::async, [this](TKobukiData sensdata) { robot_callback(sensdata); }, sens);
		}
	}

	close(rob_s);
	std::cout << "koniec thread2" << std::endl;
}


void Robot::setTranslationSpeed(int mmpersec)
{
	if (m_emgStop) {
		return;
	}
	std::vector<unsigned char> mess = robot.setTranslationSpeed(mmpersec);
	if (::sendto(rob_s, (char *)mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *)&rob_si_posli, rob_slen) == -1) { }
}

void Robot::setRotationSpeed(double radpersec) //left
{
	if (m_emgStop) {
		return;
	}
	std::vector<unsigned char> mess = robot.setRotationSpeed(radpersec);
	if (::sendto(rob_s, (char *)mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *)&rob_si_posli, rob_slen) == -1) { }
}

void Robot::setArcSpeed(int mmpersec, int radius)
{
	if (m_emgStop) {
		return;
	}
	std::vector<unsigned char> mess = robot.setArcSpeed(mmpersec, radius);
	if (::sendto(rob_s, (char *)mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *)&rob_si_posli, rob_slen) == -1) { }
}

///tato funkcia vas nemusi zaujimat
/// toto je funkcia s nekonecnou sluckou,ktora cita data z lidaru (UDP komunikacia)
void Robot::laserprocess()
{
#ifdef _WIN32
	WSADATA wsaData = { 0 };
	int iResult = 0;
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
	las_slen = sizeof(las_si_other);
	if ((las_s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		std::cerr << __FUNCTION__ << ":" << __LINE__ << ": " << strerror(errno) << std::endl;
		exit(-1);
	}

	char las_broadcastene = 1;
#ifdef _WIN32
	DWORD timeout = 100;

	::setsockopt(las_s, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof timeout);
	::setsockopt(las_s, SOL_SOCKET, SO_BROADCAST, &las_broadcastene, sizeof(las_broadcastene));
#else
	::setsockopt(las_s, SOL_SOCKET, SO_BROADCAST, &las_broadcastene, sizeof(las_broadcastene));
#endif
	// zero out the structure
	memset((char *)&las_si_me, 0, sizeof(las_si_me));

	las_si_me.sin_family = AF_INET;
	las_si_me.sin_port = htons(laser_ip_portOut);
	las_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	las_si_posli.sin_family = AF_INET;
	las_si_posli.sin_port = htons(laser_ip_portIn);
	las_si_posli.sin_addr.s_addr = inet_addr(laser_ipaddress.data());
	; //htonl(INADDR_BROADCAST);
	if (::bind(las_s, (struct sockaddr *)&las_si_me, sizeof(las_si_me)) != 0) {
		std::cerr << __FUNCTION__ << ":" << __LINE__ << ": " << strerror(errno) << std::endl;
		exit(-1);
	}

	char command = 0x00;
	//najskor posleme prazdny prikaz
	//preco?
	//https://ih0.redbubble.net/image.74126234.5567/raf,750x1000,075,t,heather_grey_lightweight_raglan_sweatshirt.u3.jpg
	if (::sendto(las_s, &command, sizeof(command), 0, (struct sockaddr *)&las_si_posli, rob_slen)
		== -1) //podla toho vie kam ma robot posielat udaje-odtial odkial mu dosla posledna sprava
	{ }
	LaserMeasurement measure;
	while (1) {
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
			break;
		if ((las_recv_len = ::recvfrom(las_s, (char *)&measure.Data, sizeof(LaserData) * 1000, 0, (struct sockaddr *)&las_si_other, &las_slen)) == -1) {
			continue;
		}
		measure.numberOfScans = las_recv_len / sizeof(LaserData);
		//tu mame data..zavolame si funkciu-- vami definovany callback

		auto t = std::async(
			std::launch::async, [this](LaserMeasurement sensdata) { laser_callback(sensdata); }, measure);
		///ako som vravel,toto vas nemusi zaujimat
	}
	close(las_s);
	std::cout << "koniec thread" << std::endl;
}

void Robot::skeletonprocess()
{
	std::cout << "init skeleton" << std::endl;
#ifdef _WIN32
	WSADATA wsaData = { 0 };
	int iResult = 0;
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
	ske_slen = sizeof(ske_si_other);
	if ((ske_s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		std::cerr << __FUNCTION__ << ":" << __LINE__ << ": " << strerror(errno) << std::endl;
		exit(-1);
	}

	char ske_broadcastene = 1;
#ifdef _WIN32
	DWORD timeout = 100;

	std::cout << ::setsockopt(ske_s, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof timeout) << std::endl;
	std::cout << ::setsockopt(ske_s, SOL_SOCKET, SO_BROADCAST, &ske_broadcastene, sizeof(ske_broadcastene)) << std::endl;
#else
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 10000;
	::setsockopt(ske_s, SOL_SOCKET, SO_BROADCAST, &ske_broadcastene, sizeof(ske_broadcastene));
	::setsockopt(ske_s, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv,sizeof(tv));
#endif
	// zero out the structure
	memset((char *)&ske_si_me, 0, sizeof(ske_si_me));

	ske_si_me.sin_family = AF_INET;
	ske_si_me.sin_port = htons(skeleton_ip_portOut);
	ske_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	ske_si_posli.sin_family = AF_INET;
	ske_si_posli.sin_port = htons(skeleton_ip_portIn);
	ske_si_posli.sin_addr.s_addr = inet_addr(skeleton_ipaddress.data());
	; //htonl(INADDR_BROADCAST);
	rob_slen = sizeof(ske_si_me);

	if (::bind(ske_s, (struct sockaddr *)&ske_si_me, sizeof(ske_si_me)) != 0) {
		std::cerr << __FUNCTION__ << ":" << __LINE__ << ": " << strerror(errno) << std::endl;
		exit(-1);
	}
	char command = 0x00;

	skeleton bbbk;
	double measure[225];
	if (::sendto(ske_s, &command, sizeof(command), 0, (struct sockaddr *)&ske_si_posli, rob_slen)
		== -1) //podla toho vie kam ma robot posielat udaje-odtial odkial mu dosla posledna sprava
	{ }
	while (1) {
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
			break;
		if ((ske_recv_len = ::recvfrom(ske_s, (char *)&bbbk.joints, sizeof(char) * 1800, 0, (struct sockaddr *)&ske_si_other, &ske_slen)) == -1) {
			//    std::cout<<"problem s prijatim"<<std::endl;
			continue;
		}

		// std::cout << "koniec thread" << std::endl;
		auto t = std::async(
			std::launch::async, [this](skeleton skele) { skeleton_callback(skele); }, bbbk);
		// std::cout << "koniec skeleton thread" << std::endl;
	}

	close(ske_s);
}

void Robot::robotStart()
{
	if (wasRobotSet == 1) {
		std::function<void(void)> f = std::bind(&Robot::robotprocess, this);
		robotthreadHandle = std::move(std::thread(f));
	}
	if (wasLaserSet == 1) {
		std::function<void(void)> f2 = std::bind(&Robot::laserprocess, this);
		laserthreadHandle = std::move(std::thread(f2));
	}
	if (wasCameraSet == 1) {
		std::function<void(void)> f3 = std::bind(&Robot::imageViewer, this);
		camerathreadhandle = std::move(std::thread(f3));
	}
	if (wasSkeletonSet == 1) {
		std::function<void(void)> f4 = std::bind(&Robot::skeletonprocess, this);
		skeletonthreadHandle = std::move(std::thread(f4));
	}
}

void Robot::imageViewer()
{
	cv::VideoCapture cap;
	if (!cap.open(camera_link)) {
		std::cerr << "Error opening video stream or file" << std::endl;
		return;
	}

	cv::Mat frameBuf;
	while (1) {
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
			break;
		cap >> frameBuf;


		// std::cout << "doslo toto " << frameBuf.rows << " " << frameBuf.cols << std::endl;


		// tu sa vola callback..
		auto t = std::async(
			std::launch::async, [this](cv::Mat camdata) { camera_callback(camdata.clone()); }, frameBuf);
#ifdef _WIN32
		cv::waitKey(1);
#else
		usleep(1 * 1000);
#endif
	}

	std::cout << "Koniec imageViewer" << std::endl;
	cap.release();
}
