#ifndef CAMERA_HEADER_HPP
#define CAMERA_HEADER_HPP
//
// sony camera interface header
//
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
// #include <opencv4/opencv2/core.hpp>
// #include <opencv4/opencv2/videoio.hpp>
// #include <opencv4/opencv2/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
#include <sony_camera_node/CameraCommand.h>

static const int idle = 99;
static const int live = 2;
static const int focus = 3;

class CameraControl
{
public:
    CameraControl()
    {control_value = focus;};
    ~CameraControl(){};
    int get_control_value()
    {
        return control_value;
    }
    void getPicturePathCB(const std_msgs::String &msg)
    {
        picture_path_ = msg;
        new_path_ = true;
    }
    void set_control_value(int value)
    {
      control_value = value;
    }
    bool callback(sony_camera_node::CameraCommandRequest& request,
                  sony_camera_node::CameraCommandResponse& response)
    {
        if (request.command > -1 && request.command <= idle) {
            control_value = request.command;
            ROS_INFO_STREAM("service called, command value = " << (int)request.command);
            return true;
        }
        else {
            return false;
        }
    }

    std_msgs::String picture_path_;
    bool new_path_ = false;
private:
    int control_value;
};

#endif
