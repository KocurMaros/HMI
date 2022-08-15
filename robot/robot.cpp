#include "robot.h"


#ifdef _WIN32
#include<windows.h>

#else
#include <termios.h>
#include <unistd.h>
#include "unistd.h"
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#endif

std::function<int(TKobukiData)> Robot::do_nothing_robot=[](TKobukiData data){std::cout<<"data z kobuki "<<std::endl; return 0;};
std::function<int(LaserMeasurement)> Robot::do_nothing_laser=[](LaserMeasurement data){std::cout<<"data z rplidar "<<std::endl; return 0;};

Robot::~Robot()
{

    ready_promise.set_value();
    robotthreadHandle.join();
    laserthreadHandle.join();
    camerathreadhandle.join();
#ifdef _WIN32
WSACleanup();
#endif;
}

Robot::Robot(std::string ipaddressLaser,int laserportRobot, int laserportMe,std::function<int(LaserMeasurement)> &lascallback,std::string ipaddressRobot,int robotportRobot, int robotportMe,std::function<int(TKobukiData)> &robcallback): wasLaserSet(0),wasRobotSet(0),wasCameraSet(0)
{

    setLaserParameters(ipaddressLaser,laserportRobot,laserportMe,lascallback);
    setRobotParameters(ipaddressRobot,robotportRobot,robotportMe,robcallback);
    readyFuture=ready_promise.get_future();
}
void Robot::robotprocess()
{
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    rob_slen = sizeof(las_si_other);
    if ((rob_s=::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char rob_broadcastene=1;
    DWORD timeout=100;

    ::setsockopt(rob_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout);
    ::setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(robot_ip_portOut);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(robot_ip_portIn);
    rob_si_posli.sin_addr.s_addr =inet_addr(robot_ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    ::bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (::sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
#ifdef _WIN32
    Sleep(100);
#else
    usleep(100*1000);
#endif
    mess=robot.setSound(440,1000);
    if (::sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    unsigned char buff[50000];
    while(1)
    {

        if(readyFuture.wait_for(std::chrono::seconds(0))==std::future_status::ready)
            break;
        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = ::recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other, &rob_slen)) == -1)
        {

            continue;
        }
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        struct timespec t;
        //      clock_gettime(CLOCK_REALTIME,&t);

        int returnval=robot.fillData(sens,(unsigned char*)buff);
        if(returnval==0)
        {
            //     memcpy(&sens,buff,sizeof(sens));

            std::chrono::steady_clock::time_point timestampf=std::chrono::steady_clock::now();





            ///---toto je callback funkcia...
         //   robot_callback(sens);
            std::async(std::launch::async, [this](TKobukiData sensdata) { robot_callback(sensdata); },sens);

        }


    }

    std::cout<<"koniec thread2"<<std::endl;
}


void Robot::setTranslationSpeed(int mmpersec)
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(mmpersec);
    if (::sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void Robot::setRotationSpeed(double radpersec) //left
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(radpersec);
    if (::sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

 void Robot::setArcSpeed(int mmpersec,int radius)
 {
     std::vector<unsigned char> mess=robot.setArcSpeed(mmpersec,radius);
     if (::sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
     {

     }
 }
void Robot::laserprocess()
{
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    las_slen = sizeof(las_si_other);
    if ((las_s=::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char las_broadcastene=1;
#ifdef _WIN32
    DWORD timeout=100;

    ::setsockopt(las_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout);
    ::setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,&las_broadcastene,sizeof(las_broadcastene));
#else
    ::setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,&las_broadcastene,sizeof(las_broadcastene));
#endif
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(laser_ip_portOut);
    las_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(laser_ip_portIn);
    las_si_posli.sin_addr.s_addr = inet_addr(laser_ipaddress.data());;//htonl(INADDR_BROADCAST);
    ::bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;
    if (::sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, rob_slen) == -1)
    {

    }
    LaserMeasurement measure;
    while(1)
    {

        if(readyFuture.wait_for(std::chrono::seconds(0))==std::future_status::ready)
            break;
        if ((las_recv_len = ::recvfrom(las_s, (char *)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other, &las_slen)) == -1)
        {

            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        //tu mame data..zavolame si funkciu

        std::async(std::launch::async, [this](LaserMeasurement sensdata) { laser_callback(sensdata); },measure);
        //     memcpy(&sens,buff,sizeof(sens));
      //  int returnValue=autonomouslaser(measure);

    }
    std::cout<<"koniec thread"<<std::endl;
}


void Robot::robotStart()
{
    if(wasRobotSet==1)
    {
        std::function<void(void)> f =std::bind(&Robot::robotprocess,this);
        robotthreadHandle=std::move(std::thread(f));
    }
    if(wasLaserSet==1)
    {
        std::function<void(void)> f2 =std::bind(&Robot::laserprocess, this);
        laserthreadHandle=std::move(std::thread(f2));
    }
    if(wasCameraSet==1)
    {
        std::function<void(void)> f3 =std::bind(&Robot::imageViewer, this);
        camerathreadhandle=std::move(std::thread(f3));
    }

}


void Robot::imageViewer()
{
    cv::VideoCapture cap;
    cap.open(camera_link);
    cv::Mat frameBuf;
    while(1)
    {
        if(readyFuture.wait_for(std::chrono::seconds(0))==std::future_status::ready)
            break;
        cap >> frameBuf;






      //  frameBuf.copyTo(robotPicture);
        std::async(std::launch::async, [this](cv::Mat camdata) { camera_callback(camdata.clone()); },frameBuf);
        cv::waitKey(1);

    }
    cap.release();
}
