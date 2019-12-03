// include the librealsense C++ header file
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>


//#include <boost/asio.hpp>
//#include <boost/bind.hpp>
//#include <boost/thread.hpp>
//#include <boost/thread/tss.hpp>
//#include <iostream>
//#include <boost/make_shared.hpp>
//#include <boost/function.hpp>
//#include <boost/asio/async_result.hpp>
//#include <boost/thread/future.hpp>
//#include "defines/MacrosInterface.h"

//#include <messagequeue/MessageQueue.h>
//#include <Camera3DPluginInterface.h>
//#include <Camera3DPluginMacrosInterface.h>
//#include <Camera3DCommonMacros.h>
//#include <boost/shared_ptr.hpp>
//#include <defines/MacrosInterface.h>
//#include <EagleCamera3DApi.h>
//#include <Camera3DPluginMacrosInterface.h>
//#include <Camera3DErrorCodeEnum.h>
//#include <vector>
//#include <VisionInputImageStruct.h>

#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <logger/Logger.h>

using namespace std;
using namespace cv;

float getDistance(rs2::depth_frame depth) {
    int width = depth.get_width();
    int height = depth.get_height();
    float distance = 10;
    for (int w = 10; w < width - 10; w=w+width/10) {
        for (int h = 10; h < height-10; h=h+height/10) {
            float dis = depth.get_distance(w, h);
            LOG_INFO << "get distance from " << w << "    " << h << " " << dis;
            if (dis < distance && dis != 0) {
                distance = dis;
            }
        }
    }
    return distance;
}

int main() {

    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);
    for (;;) {
        // Camera warmup - dropping several first frames to let auto-exposure stabilize
        rs2::frameset frames;
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }

        //Get each frame
        rs2::frame color_frame = frames.get_color_frame();

        // Creating OpenCV Matrix from a color image
        Mat color(Size(640, 480), CV_8UC3, (void *) color_frame.get_data(), Mat::AUTO_STEP);
        auto dis = getDistance(frames.get_depth_frame());

        stringstream ss;
        ss << setprecision(3) << dis << "M";

        cv::putText(color, ss.str(), Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 255, 0), 2);
        // Display in a GUI
        namedWindow("color_frame", WINDOW_AUTOSIZE);
        imshow("color_frame", color);

        waitKey(50);


        rs2::frame ir_frame = frames.get_infrared_frame();
        // Creating OpenCV matrix from IR image
        Mat ir(Size(640, 480), CV_8UC1, (void *) ir_frame.get_data(), Mat::AUTO_STEP);
        namedWindow("ir_frame", WINDOW_AUTOSIZE);
        imshow("ir_frame", ir);
        waitKey(50);


        rs2::frame pc_frame = frames.get_depth_frame();
        rs2::pointcloud pc;
//        pc.map_to(color_frame);
        auto points = pc.calculate(pc_frame);
        Mat dp(Size(640, 480), CV_32FC3, (void *) points.get_data(), Mat::AUTO_STEP);
        namedWindow("dp_frame", WINDOW_AUTOSIZE);
        imshow("dp_frame", dp);
        waitKey(50);
    }
    return 0;
}