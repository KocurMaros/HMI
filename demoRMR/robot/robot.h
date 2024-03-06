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
#include <future>

static const char *klby[]={{"left_wrist"},{"left_thumb_cmc"},{"left_thumb_mcp"},{"left_thumb_ip"},{"left_thumb_tip"},{"left_index_cmc"},{"left_index_mcp"},{"left_index_ip"},{"left_index_tip"},{"left_middle_cmc"},{"left_middle_mcp"},{"left_middle_ip"},{"left_middle_tip"},{"left_ring_cmc"},{"left_ring_mcp"},{"left_ring_ip"},{"left_ringy_tip"},{"left_pinky_cmc"},{"left_pink_mcp"},{"left_pink_ip"},{"left_pink_tip"},{"right_wrist"},{"right_thumb_cmc"},{"right_thumb_mcp"},{"right_thumb_ip"},{"right_thumb_tip"},{"right_index_cmc"},{"right_index_mcp"},{"right_index_ip"},{"right_index_tip"},{"right_middle_cmc"},{"right_middle_mcp"},{"right_middle_ip"},{"right_middle_tip"},{"right_ring_cmc"},{"right_ring_mcp"},{"right_ring_ip"},{"right_ringy_tip"},{"right_pinky_cmc"},{"right_pink_mcp"},{"right_pink_ip"},{"right_pink_tip"},{"nose"},{"left_eye_in"},{"left_eye"},{"left_eye_out"},{"right_eye_in"},{"right_eye"},{"right_eye_out"},{"left_ear"},{"right_ear"},{"mounth_left"},{"mounth_right"},{"left_shoulder"},{"right_shoulder"},{"left_elbow"},{"right_elbow"},{"left_wrist"},{"right_wrist"},{"left_pinky"},{"right_pinky"},{"left_index"},{"right_index"},{"left_thumb"},{"right_thumb"},{"left_hip"},{"right_hip"},{"left_knee"},{"right_knee"},{"left_ankle"},{"right_ankle"},{"left_heel"},{"righ_heel"},{"left+foot_index"},{"right_foot_index"}};
enum jointnames
{
	left_wrist,
   left_thumb_cmc,
   left_thumb_mcp,
   left_thumb_ip,
   left_thumb_tip,
   left_index_cmc,
   left_index_mcp,
   left_index_ip,
   left_index_tip,
   left_middle_cmc,
   left_middle_mcp,
   left_middle_ip,
   left_middle_tip,
   left_ring_cmc,
   left_ring_mcp,
   left_ring_ip,
   left_ringy_tip,
   left_pinky_cmc,
   left_pink_mcp,
   left_pink_ip,
   left_pink_tip,
   right_wrist,
   right_thumb_cmc,
   right_thumb_mcp,
   right_thumb_ip,
   right_thumb_tip,
   right_index_cmc,
   right_index_mcp,
   right_index_ip,
   right_index_tip,
   right_middle_cmc,
   right_middle_mcp,
   right_middle_ip,
   right_middle_tip,
   right_ring_cmc,
   right_ring_mcp,
   right_ring_ip,
   right_ringy_tip,
   right_pinky_cmc,
   right_pink_mcp,
   right_pink_ip,
   right_pink_tip,
   nose,left_eye_in,
   left_eye,
   left_eye_out,
   right_eye_in,
   right_eye,
   right_eye_out,
   left_ear,
   right_ear,
   mounth_left,
   mounth_right,
   left_shoulder,
   right_shoulder,
   left_elbow,
   right_elbow,
   left_wrist_glob,
   right_wrist_glob,
   left_pinky,
   right_pinky,
   left_index,
   right_index,
   left_thumb,
   right_thumb,
   left_hip,
   right_hip,
   left_knee,
   right_knee,
   left_ankle,
   right_ankle,
   left_heel,
   righ_heel,
   left_foot_index,
   right_foot_index
};
typedef struct
{
	double x;
	double y;
	double z;
}klb;

typedef struct
{
	klb joints[75];
}skeleton;

class ROBOT_EXPORT Robot
{
public:
	~Robot();
	Robot(std::string ipaddressLaser = "127.0.0.1", int laserportRobot = 52999, int laserportMe = 5299,
		  std::function<int(LaserMeasurement)> &lascallback = do_nothing_laser, std::string ipaddressRobot = "127.0.0.1", int robotportRobot = 53000,
		  int robotportMe = 5300, std::function<int(TKobukiData)> &robcallback = do_nothing_robot);


	//default functions.. please do not rewrite.. make your own callback
	static std::function<int(TKobukiData)> do_nothing_robot;
	static std::function<int(LaserMeasurement)> do_nothing_laser;


	void robotStart();
	void setLaserParameters(std::string ipaddress, int laserportRobot, int laserportMe, std::function<int(LaserMeasurement)> callback)
	{
		laser_ip_portOut = laserportRobot;
		laser_ip_portIn = laserportMe;
		laser_ipaddress = ipaddress;
		laser_callback = callback;
		wasLaserSet = 1;
	}
	void setRobotParameters(std::string ipaddress, int robotportRobot, int robotportMe, std::function<int(TKobukiData)> callback)
	{
		robot_ip_portOut = robotportRobot;
		robot_ip_portIn = robotportMe;
		robot_ipaddress = ipaddress;
		robot_callback = callback;
		wasRobotSet = 1;
	}

	void setSkeletonParameters(std::string ipaddress,int skeletonportRobot, int skeletonportMe,std::function<int(skeleton)> callback )
	{
		skeleton_ip_portOut=skeletonportRobot;
		skeleton_ip_portIn=skeletonportMe;
		skeleton_ipaddress=ipaddress;
		skeleton_callback=callback;
		wasSkeletonSet=1;
	}

	void setTranslationSpeed(int mmpersec);

	void setRotationSpeed(double radpersec);
	void setArcSpeed(int mmpersec, int radius);

	void setCameraParameters(std::string link, std::function<int(cv::Mat)> callback)
	{
		camera_link = link;
		camera_callback = callback;
		wasCameraSet = 1;
	}

	bool isConnected() const { return m_connected; };
	void setEmgStop(bool stop) { m_emgStop = stop; };
	bool isInEmgStop() const { return m_emgStop; };

	long double tickToMeter = 0.000085292090497737556558; // [m/tick]
	long double b = 0.23; // wheelbase distance in meters, from kobuki manual https://yujinrobot.github.io/kobuki/doxygen/enAppendixProtocolSpecification.html

private:
	std::promise<void> ready_promise;
	std::shared_future<void> readyFuture;
	int wasLaserSet;
	int wasRobotSet;
	int wasCameraSet;
	int wasSkeletonSet;
	//veci na laser
	LaserMeasurement copyOfLaserData;
	void laserprocess();
	std::string laser_ipaddress;
	int laser_ip_portOut;
	int laser_ip_portIn;
	std::thread laserthreadHandle;
	std::function<int(LaserMeasurement)> laser_callback = nullptr;

	//veci pre podvozok
	CKobuki robot;
	TKobukiData sens;
	std::string robot_ipaddress;
	int robot_ip_portOut;
	int robot_ip_portIn;
	std::thread robotthreadHandle;
	void robotprocess();
	std::function<int(TKobukiData)> robot_callback = nullptr;

	//veci na skeleton
	std::thread skeletonthreadHandle;
	std::string skeleton_ipaddress;
	int skeleton_ip_portOut;
	int skeleton_ip_portIn;
	void skeletonprocess();
	std::function<int(skeleton)> skeleton_callback=nullptr;

	//veci pre kameru -- pozor na kameru, neotvarat ak nahodou chcete kameru pripojit na detekciu kostry...

	std::string camera_link;
	std::thread camerathreadhandle;
	std::function<int(cv::Mat)> camera_callback = nullptr;
	void imageViewer();

	///
	///
	struct sockaddr_in las_si_me, las_si_other, las_si_posli;

	int las_s, las_recv_len;


	struct sockaddr_in ske_si_me, ske_si_other, ske_si_posli;

	int ske_s, ske_recv_len;

	//veci na broadcast robot
	struct sockaddr_in rob_si_me, rob_si_other, rob_si_posli;

	int rob_s, rob_recv_len;
#ifdef _WIN32
	int rob_slen;
	int las_slen;
	int ske_slen;
#else
	unsigned int rob_slen;
	unsigned int las_slen;
	unsigned int ske_slen;
#endif

	// Supplied additional parameters.
	bool m_connected;
	bool m_emgStop;
};

#endif // ROBOT_H
