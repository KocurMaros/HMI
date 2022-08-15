#ifndef ROBOT_H
#define ROBOT_H

#define useCamera

#ifdef useCamera
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#endif

#include "robot_global.h"
#include "rplidar.h"
#include "CKobuki.h"
#include <thread>
#include <functional>
#include <list>
#include <algorithm>
#include <utility>
#include <atomic>
#include <mutex>
#include <future>
#include <random>
#include <iostream>
#include <memory>

class ROBOT_EXPORT Robot
{
public:
    ~Robot();
    Robot(std::string ipaddressLaser="127.0.0.1",int laserportRobot=52999, int laserportMe=5299,std::function<int(LaserMeasurement)> &lascallback=do_nothing_laser,std::string ipaddressRobot="127.0.0.1",int robotportRobot=53000, int robotportMe=5300,std::function<int(TKobukiData)> &robcallback=do_nothing_robot);


    //default functions.. please do not rewrite.. make your own callback
    static  std::function<int(TKobukiData)> do_nothing_robot;
    static std::function<int(LaserMeasurement)> do_nothing_laser;


    void robotStart();
    void setLaserParameters(std::string ipaddress,int laserportRobot, int laserportMe,std::function<int(LaserMeasurement)> callback )
    {
        laser_ip_portOut=laserportRobot;
        laser_ip_portIn=laserportMe;
        laser_ipaddress=ipaddress;
        laser_callback=callback;
        wasLaserSet=1;
    }
    void setRobotParameters(std::string ipaddress,int robotportRobot, int robotportMe,std::function<int(TKobukiData)> callback )
    {
        robot_ip_portOut=robotportRobot;
        robot_ip_portIn=robotportMe;
        robot_ipaddress=ipaddress;
        robot_callback=callback;
        wasRobotSet=1;
    }

    void setTranslationSpeed(int mmpersec);

    void setRotationSpeed(double radpersec);
    void setArcSpeed(int mmpersec,int radius);

    void setCameraParameters(std::string link,std::function<int(cv::Mat)> callback )
    {

        camera_link=link;
        camera_callback=callback;
        wasCameraSet=1;
    }
private:
     std::promise<void> ready_promise;
    std::shared_future<void> readyFuture;
    int wasLaserSet;
    int wasRobotSet;
    int wasCameraSet;
    //veci na laser
    LaserMeasurement copyOfLaserData;
    void laserprocess();
    std::string laser_ipaddress;
    int laser_ip_portOut;
    int laser_ip_portIn;
    std::thread laserthreadHandle;
    std::function<int(LaserMeasurement)> laser_callback=nullptr;

    //veci pre podvozok
    CKobuki robot;
    TKobukiData sens;
    std::string robot_ipaddress;
    int robot_ip_portOut;
    int robot_ip_portIn;
    std::thread robotthreadHandle;
    void robotprocess();
    std::function<int(TKobukiData)> robot_callback=nullptr;


    //veci pre kameru -- pozor na kameru, neotvarat ak nahodou chcete kameru pripojit na detekciu kostry...

    std::string camera_link;
    std::thread camerathreadhandle;
    std::function<int(cv::Mat)> camera_callback=nullptr;
    void Robot::imageViewer();

    ///
    ///
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int las_s,  las_recv_len;


    struct sockaddr_in ske_si_me, ske_si_other,ske_si_posli;

    int ske_s,  ske_recv_len;

    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

    int rob_s,  rob_recv_len;
#ifdef _WIN32
        int rob_slen;
        int las_slen;
        int ske_slen;
#else
         unsigned int rob_slen;
         unsigned int las_slen;
         unsigned int ske_slen;
#endif
};

#endif // ROBOT_H
