#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sony_camera_node/CameraCommand.h>
#include <sstream>
#if defined(USE_EXPERIMENTAL_FS)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#include <iomanip>
#include "CRSDK/CameraRemote_SDK.h"
#include "CameraDevice.h"
#include "Text.h"
#include "sony_camera_interface_header.hpp"

namespace SDK = SCRSDK;

int main(int argc, char **argv)
{
    //
    // initialize ROS and create unique node name
    //
    ros::init(argc, argv, "sony_camera_node");
    //
    // declare a node handle
    //
    ros::NodeHandle nh;

    //
    // instantiate a CameraControl object to enable
    // access to service callback and to encapsulate
    // the request value
    //
    CameraControl camera_control;
    ROS_INFO_STREAM("Camera control object, command value = " << camera_control.get_control_value());

    ros::Subscriber picture_path_sub = nh.subscribe("picture_path", 1000, &CameraControl::getPicturePathCB, &camera_control);
    ros::Publisher current_image_pub = nh.advertise<std_msgs::String>("current_image", 1);

    //
    // instantiate a service to be called to save an image to file
    //
    ros::ServiceServer camera_command_service = nh.advertiseService(
                "sony_camera_node/camera_command",
                &CameraControl::callback,
                &camera_control);
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = 5;
    //
    // create a variable to hold the loop count
    //
    int loop_count = 0;
    int final_count = loop_count;
    //
    // publish the messages in a while loop as long as our node is
    // in good standing as a ROS node
    //
    // create a msg object to hole the string
    //
    std_msgs::String msg;
    std::stringstream msg_ss;
    //
    ////////// Sony camera setup code follows
    //
    // Change global locale to native locale
    //
    std::locale::global(std::locale(""));
    //
    // Make the stream's locale the same as the current global locale
    //
    cli::tin.imbue(std::locale());
    cli::tout.imbue(std::locale());
    auto init_success = SDK::Init();
    if (!init_success) {
        cli::tout << "Failed to initialize Remote SDK. Terminating.\n";
        SDK::Release();
        std::exit(EXIT_FAILURE);
    }
    SDK::ICrEnumCameraObjectInfo* camera_list = nullptr;
    auto enum_status = SDK::EnumCameraObjects(&camera_list);
    if (CR_FAILED(enum_status) || camera_list == nullptr) {
        cli::tout << "No cameras detected. Connect a camera and retry.\n";
        SDK::Release();
        std::exit(EXIT_FAILURE);
    }
    auto ncams = camera_list->GetCount();
    for (CrInt32u i = 0; i < ncams; ++i) {
        auto camera_info = camera_list->GetCameraObjectInfo(i);
        cli::text conn_type(camera_info->GetConnectionTypeName());
        cli::text model(camera_info->GetModel());
        cli::tout << '[' << i + 1 << ']' << model.data() << '\n';
    }
    CrInt32u no = 0;
    if (ncams > 0) {
        no = 1;
    }
    else {
        no = 0;
    }
    if (no == 0) {
        cli::tout << "no cameras detected, exiting...\n";
        SDK::Release();
        std::exit(EXIT_FAILURE);
    }
    auto* camera_info = camera_list->GetCameraObjectInfo(no - 1);
    cli::CameraDevice camera(nullptr, camera_info);
    camera_list->Release();
    auto connect_status = camera.connect();
    if (!connect_status) {
        cli::tout << "Camera connection failed to initiate. Abort.\n";
        SDK::Release();
        std::exit(EXIT_FAILURE);
    }
    cli::tout << "Camera connection successfully initiated!\n\n";
    //
    // instaniate image block and image buffer to hold live view image
    //
    SDK::CrImageDataBlock *image_data = new SDK::CrImageDataBlock();
    CrInt8u* image_buff = nullptr;

    //
    // main ROS loop
    //
    sleep(1);

    while (ros::ok() && loop_count > -1) {
        ++loop_count;
        final_count = loop_count;
        msg_ss << "loop count: \n" << loop_count;
        msg.data = msg_ss.str();
        //
        // acquire the live image and assign it to the image block
        //
        camera.get_image_data(image_data, image_buff);
        // ROS_INFO_STREAM("image data size = " << (int)image_data->GetSize());

        // ROS_INFO_STREAM("test command value = " << (int)camera_control.get_control_value());

        if(camera_control.new_path_)
        {
            camera.path = camera_control.picture_path_.data;
            camera_control.new_path_ = false;
        }

        if (0 == camera_control.get_control_value()) {
            if (camera.connected()) {
                cli::tout << "Initiate disconnect sequence.\n";
                auto disconnect_status = camera.disconnect();
                if (!disconnect_status) {
                    // try again
                    disconnect_status = camera.disconnect();
                }
                if (!disconnect_status) {
                    cli::tout << "Disconnect failed to initiate.\n";
                }
                else {
                    cli::tout << "Disconnect successfully initiated!\n\n";
                }
            }
            loop_count = -2;
        }
        else if (1 == camera_control.get_control_value()) {
            if (camera.set_save_info()) {

                std_msgs::String msg;
                camera.capture_image();
                camera.focus();

                sleep(3);
                msg.data = camera.filename_;
                current_image_pub.publish(msg);

                camera_control.set_control_value(live);
            }
        }
        else if (2 == camera_control.get_control_value()) {
            camera.get_live_view();
        }
        else if (3 == camera_control.get_control_value())
        {
            camera.focus();
            camera_control.set_control_value(live);
        }
        else {
            camera_control.set_control_value(idle);
        }

        ros::spinOnce();
        //
        // sleep or block or go inactive until the next loop iteration
        //
        loop_rate.sleep();
        //
    }// ---- End Loop
    ROS_INFO_STREAM("exited ROS loop, loop count = " << final_count);
    //
    // delete pointers and free allocated memory
    //
    delete[] image_buff; // Release
    delete image_data; // Release

    cli::tout << "Release SDK resources.\n";
    SDK::Release();

    cli::tout << "Exiting application.\n";
    std::exit(EXIT_SUCCESS);
}
