#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#define VX_AXIS 1        // Left Vertical Stick
#define VY_AXIS 0        // Left Horizonal Stick
#define W_AXIS 3         // Right Horizonal Stick
#define EMERG_BTN 4      // L1
#define JS_CTRL_BTN 5    // R1
#define HOME_BTN 2       // Triangle
#define ADD_POINT_BTN 1  // Circle
#define DEL_POINT_BTN 0  // Cross
#define SAVE_POINT_BTN 3 // Square
#define navigation_BTN 9 //start
#define stop_navigation_BTN 8 //back

#define BTN_DEBOUNCE_TIME 0.1 // 100ms

void joyStickCallback(const sensor_msgs::Joy::ConstPtr &msg);
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
void savedPosePubTimerCallback(const ros::TimerEvent &);
void goHome();
void addPoint();
void delPoint();
void savePointsToFile();

double vx_sacler;
double vy_sacler;
double w_sacler;
bool emergencyStopFlag = false;
bool jsCmdVel = false;
bool lastAddBtnState = false;
bool lastDelBtnState = false;
bool lastSaveBtnState = false;
bool lastNavigationState = false; //1101 Eddie add
bool NavigationState = false; //1101 Eddie add
bool lastStopNavigationState = false; //1102 Eddie add
bool StopNavigationState = false; //1102 Eddie add
geometry_msgs::Pose currentPose;
std::vector<geometry_msgs::Pose> savedPose;

ros::Publisher cmdVelPub;
ros::Publisher joycontrolPub;
ros::Publisher emergencyStopPub;
ros::Publisher goalPub;
ros::Publisher savedPosePub;
ros::Publisher navigationPub;
ros::Publisher stopnavigationPub;
ros::Time lastAddTime, lastDelTime, lastSaveTime, lastNavigationTime, lastStopNavigationTime;

int main(int argc, char **argv)
{
    ros::init(argc, argv,"ccp_js_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Subscriber joyStickSub = nh.subscribe("/joy", 1000, joyStickCallback);
    ros::Subscriber cmdVelSub = nh.subscribe("/cmd_vel", 1000, cmdVelCallback);
    ros::Subscriber amclPoseSub = nh.subscribe("/amcl_pose", 1000, amclPoseCallback);
    cmdVelPub = nh.advertise<geometry_msgs::Twist>("omni_base_driver/cmd_vel", 1);
    joycontrolPub = nh.advertise<std_msgs::Bool>("/joy_enable", 1);
    emergencyStopPub = nh.advertise<std_msgs::Bool>("/emergency_stop", 1);
    goalPub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    savedPosePub = nh.advertise<visualization_msgs::Marker>("/saved_point", 1);
    navigationPub = nh.advertise<std_msgs::Int8>("/navigationSend", 1);
    stopnavigationPub = nh.advertise<std_msgs::Int8>("/stopnavigationSend", 1);
    ros::Timer savedPosePubTimer = nh.createTimer(ros::Duration(0.1), savedPosePubTimerCallback);

    nh_private.param<double>("vx_sacler", vx_sacler, 0.5);
    nh_private.param<double>("vy_sacler", vy_sacler, 0.5);
    nh_private.param<double>("w_sacler", w_sacler, 3.14 / 2);

    ros::spin();
    return 0;
}

void joyStickCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    std_msgs::Bool boolMsg;
    std_msgs::Bool boolJoyMsg;
    std_msgs::Int8 NavigationMsg;
    std_msgs::Int8 StopNavigationMsg;

    if (!emergencyStopFlag && msg->buttons[EMERG_BTN])
    {
        emergencyStopFlag = true;
        boolMsg.data = true;
        ROS_INFO("Emergency Stop!");
        emergencyStopPub.publish(boolMsg);
    }

    if (msg->buttons[JS_CTRL_BTN])
    {
        jsCmdVel = !jsCmdVel;
        boolJoyMsg.data = jsCmdVel;
        ROS_INFO_STREAM("Joystick control = " << jsCmdVel);
        // ROS_INFO_STREAM("Joy control boolJoyMsg = " << boolJoyMsg);
        joycontrolPub.publish(boolJoyMsg);
    }

    if (msg->buttons[HOME_BTN])
    {
        goHome();
    }

    ros::Time now = ros::Time::now();
    if (msg->buttons[ADD_POINT_BTN] != lastAddBtnState)
    {
        if (msg->buttons[ADD_POINT_BTN])
        {
            if ((now - lastAddTime).toSec() < BTN_DEBOUNCE_TIME)
                return; // Last add time < 100ms
            addPoint();
            lastAddTime = now;
        }
        lastAddBtnState = msg->buttons[ADD_POINT_BTN];
    }

    if (msg->buttons[DEL_POINT_BTN] != lastDelBtnState)
    {
        if (msg->buttons[DEL_POINT_BTN])
        {
            if ((now - lastDelTime).toSec() < BTN_DEBOUNCE_TIME)
                return; // Last delete time < 100ms
            delPoint();
            lastDelTime = now;
        }
        lastDelBtnState = msg->buttons[DEL_POINT_BTN];
    }

    if (msg->buttons[SAVE_POINT_BTN] != lastSaveBtnState)
    {
        if ((now - lastSaveTime).toSec() < BTN_DEBOUNCE_TIME * 10)
            return; // Last delete time < 100ms
        savePointsToFile();
        lastSaveTime = now;
    }
    lastSaveBtnState = msg->buttons[SAVE_POINT_BTN];

    //11/01 Navigation mode
    if (msg->buttons[navigation_BTN] != lastNavigationState)
    {
        if (msg->buttons[navigation_BTN])
        {
            if ((now - lastNavigationTime).toSec() < BTN_DEBOUNCE_TIME)
                return; // Last delete time < 100ms
            // NavigationState =! NavigationState;
            NavigationMsg.data = 8;
            // NavigationMsg.data = (NavigationMsg.data | NavigationState);
            navigationPub.publish(NavigationMsg);
            lastNavigationTime = now;
        }
        lastNavigationState = msg->buttons[navigation_BTN];
        // ROS_INFO_STREAM("NavigationMsg" << NavigationMsg);
    }

    //11/02 stopNavigation mode
    if (msg->buttons[stop_navigation_BTN] != lastStopNavigationState)
    {
        if (msg->buttons[stop_navigation_BTN])
        {
            if ((now - lastNavigationTime).toSec() < BTN_DEBOUNCE_TIME)
                return; // Last delete time < 100ms
            // NavigationState =! NavigationState;
            StopNavigationMsg.data = 34;
            // NavigationMsg.data = (NavigationMsg.data | NavigationState);
            stopnavigationPub.publish(StopNavigationMsg);
            lastNavigationTime = now;
        }
        lastStopNavigationState = msg->buttons[stop_navigation_BTN];
        // ROS_INFO_STREAM("NavigationMsg" << NavigationMsg);
    }

    if (!jsCmdVel)
        return; // Return if not controlling with joystick


    geometry_msgs::Twist twistMsg;

    twistMsg.linear.x = msg->axes[VX_AXIS] * vx_sacler;
    twistMsg.linear.y = msg->axes[VY_AXIS] * vy_sacler;

    twistMsg.angular.z = msg->axes[W_AXIS] * w_sacler;
    cmdVelPub.publish(twistMsg);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    if (jsCmdVel)
        return;
    cmdVelPub.publish(msg);
}

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    currentPose.position = msg->pose.pose.position;
    currentPose.orientation = msg->pose.pose.orientation;
}

void savedPosePubTimerCallback(const ros::TimerEvent &)
{
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.color.r = 1.0f;
    points.color.g = 1.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0f;
    points.scale.x = 0.25f;
    points.scale.y = 0.25f;

    for (size_t i = 0; i < savedPose.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = savedPose[i].position.x;
        p.y = savedPose[i].position.y;
        p.z = 0;
        points.points.push_back(p);
    }

        savedPosePub.publish(points);
}

void goHome()
{
    ROS_INFO("Going Home!");
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;
    msg.pose.orientation.w = 1;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;

    jsCmdVel = false;
    goalPub.publish(msg);
}

void addPoint()
{
    ROS_INFO("Adding Node %ld!", savedPose.size());
    savedPose.push_back(currentPose);
}

void delPoint()
{
    ROS_INFO("Deleting Point!");
    if (savedPose.size() <= 0)
        return;
    savedPose.pop_back();
}

void savePointsToFile()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
    auto str = oss.str();

    std::string fileName = ros::package::getPath("ccp_js_node") + "/saved_points/" + str + ".csv";
    ROS_INFO_STREAM("Saving Points to " << fileName);
    std::ofstream file;
    file.open(fileName);
    file << "ID,X,Y,W,X,Y,Z\n";
    for (size_t i = 0; i < savedPose.size(); i++)
    {
        file << i << "," << savedPose[i].position.x << "," << savedPose[i].position.y << ","
             << savedPose[i].orientation.x << "," << savedPose[i].orientation.y << "," << savedPose[i].orientation.z << ","
             << savedPose[i].orientation.w
             << std::endl;
    }
    file.close();
}