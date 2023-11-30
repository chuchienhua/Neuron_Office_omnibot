#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <serial/serial.h> // http://wjwwood.io/serial/doc/1.1.0/index.html
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define FRAME_LEN 16 // Length of bytes of a single message frame (including SOF, EOF and CRC16)
#define DATA_LEN 12  // Length of bytes of the data field

using namespace std;

// Message Frame Declaration
typedef struct __attribute__((__packed__))
{
    uint8_t sof;
    union
    {
        uint8_t data[DATA_LEN];
        struct
        {
            float vx;
            uint8_t onesitearrived;
            uint8_t button_status;
        };
    };
    uint16_t checksum;
    uint8_t eof;
} CtrlMsg;

class Teensycommunucation
{
public:
    Teensycommunucation(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Subscribers
    ros::Subscriber emergencyStopSub_;
    ros::Subscriber onesitearrivedSub_;

    // Publishers
    ros::Publisher button_receiver_Pub_;
    // ros::Publisher odomPub_;
    // tf::TransformBroadcaster odomTfBroad_;


    // Timers
    ros::Timer connectionCheckTimer_;
    ros::Timer communicationTimer_;

    // Parameters
    string port_name_;

    // Subscriber Callbacks
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr &msg);
    void onesitearrivedCallback(const std_msgs::UInt8::ConstPtr &msg);

    // Service Callbacks

    // Timer Callbacks
    void connectionCheckCallback(const ros::TimerEvent &event);
    void communicationCallback(const ros::TimerEvent &event);

    // Local Variables
    serial::Serial serialPort;
    bool emergencyStopFlag;
    float vx, vy, w;
    uint8_t button_status;
    uint8_t onesitearrived;

    // Private Functions
    inline uint16_t calculateChecksum(const uint8_t *data, int len)
    {
        uint16_t sum = 0;
        for (int i = 0; i < len / 2; i++)
        {
            sum ^= *(((uint16_t *)data) + i);
        }
        return sum;
    }
};