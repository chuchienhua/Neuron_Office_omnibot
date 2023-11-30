#include "teensycommunucation/teensycommunucation.h"

Teensycommunucation::Teensycommunucation(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{

    // Initialize subscribers
    emergencyStopSub_ = nh_.subscribe("tdk_base_controller/emergency_stop", 1, &Teensycommunucation::emergencyStopCallback, this, ros::TransportHints().tcpNoDelay());
    onesitearrivedSub_ = nh_.subscribe("/onesite_arrived", 1, &Teensycommunucation::onesitearrivedCallback, this, ros::TransportHints().tcpNoDelay());
    
    // Initialize publishers
    button_receiver_Pub_ = nh_.advertise<std_msgs::UInt8>("/button_receiver", 10);

    // Initialize Parameters
    nh_private_.param<string>("device_port", port_name_, "/dev/teensy4");

    // Initialize Timers
    connectionCheckTimer_ = nh_.createTimer(ros::Duration(1.0), &Teensycommunucation::connectionCheckCallback, this); // timer for checking connection
    communicationTimer_ = nh_.createTimer(ros::Duration(0.010), &Teensycommunucation::communicationCallback, this);   // timer for sending control msgs

    vx = 0;
    vy = 0;
    w = 0;
    button_status = 0x00;
    onesitearrived = 0x00;
    emergencyStopFlag = false;
}

void Teensycommunucation::emergencyStopCallback(const std_msgs::Bool::ConstPtr &msg)
{
    emergencyStopFlag = msg->data;
}

void Teensycommunucation::onesitearrivedCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    onesitearrived = msg->data;
}

void Teensycommunucation::connectionCheckCallback(const ros::TimerEvent &event)
{
    if (!serialPort.isOpen())
    {
        try
        {
            serialPort.setPort(port_name_);
            serialPort.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(50); // Set read timeout to 50ms
            serialPort.setTimeout(timeout);
            serialPort.open();
            serialPort.flushInput();
            serialPort.flushOutput();
            vx = 0;
            vy = 0;
            w = 0;
            button_status = 0x00;
            onesitearrived = 0x00;
            ROS_INFO("Connected to teensy successfully!");
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port " << port_name_ << ", " << e.what());
        }
    }
}

void Teensycommunucation::communicationCallback(const ros::TimerEvent &event)
{
    std_msgs::UInt8 button_status_msg;

    if (!serialPort.isOpen())
        return;
    // ROS_INFO("Connected to teensy successfully!!!!");
    CtrlMsg ctrlMsg;
    ctrlMsg.sof = 0xAA; // 0xAA + emergencyStopFlag;
    ctrlMsg.eof = 0xCD;

    ctrlMsg.vx = vx;
    ctrlMsg.onesitearrived = onesitearrived;
    ctrlMsg.button_status = button_status;

    ctrlMsg.checksum = calculateChecksum(ctrlMsg.data, DATA_LEN);
    try
    {

        serialPort.write((uint8_t *)&ctrlMsg, FRAME_LEN);
        uint8_t rxBuf[FRAME_LEN];
        serialPort.read(rxBuf, FRAME_LEN);
        CtrlMsg *rxMsg = (CtrlMsg *)rxBuf;
        if (((rxMsg->sof & 0xFE) == 0xAA) && (rxMsg->eof == 0xCD))
        {
            // TODO: handle response
            // ROS_INFO("Connected to teensy successfully!!!!!!!");
            // ROS_INFO("Button = %2X", rxMsg->button_status);

            button_status = rxMsg->button_status;
            button_status_msg.data = rxMsg->button_status;
            button_receiver_Pub_.publish(button_status_msg);
        }
        else
        {
            ROS_ERROR("Error response! SOF = %2X, EOF = %2X", rxMsg->sof, rxMsg->eof);
        }
    }
    catch (serial::SerialException &e)
    {
        ROS_ERROR_STREAM("Error when communicating to teensy: " << e.what());
        serialPort.close();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Error when communicating to teensy: " << e.what());
        serialPort.close();
    }
}
