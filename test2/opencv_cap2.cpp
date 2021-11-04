#include <iostream>
#include <algorithm> 
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

// Intel Realsense Headers
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

// OpenCV Headers
#include <opencv2/opencv.hpp>

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>

constexpr int WIDTH = 1280;
constexpr int HEIGHT = 720;


int main(void)
{
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::colorizer color_map;

    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
    rs2::pipeline_profile profile = pipe.start(cfg); 
    
    //option
    rs2::device selected_device = profile.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        pipe.wait_for_frames();
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        sleep(1);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }

    //initrinsics
    auto depth_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto depth_intr = depth_stream.get_intrinsics();

    while (cv::waitKey(1) < 0) try {
        auto frames = pipe.wait_for_frames();
        rs2::align align(RS2_STREAM_COLOR);
        auto aligned_frames = align.process(frames);
        rs2::video_frame color = aligned_frames.first(RS2_STREAM_COLOR);
        rs2::depth_frame depth = aligned_frames.get_depth_frame();

        int x = 150;
        int y = 150;

        float pixel[2] = {float(x), float(y)};
        float distance = depth.get_distance(pixel[0], pixel[1]);
        float point[3];
        rs2_deproject_pixel_to_point(point, &depth_intr, pixel, distance);

        std::cout << point[0] << ", " << point[1] << ", " << point[2] << std::endl;

        cv::Mat color_frame(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_frame(cv::Size(WIDTH, HEIGHT), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        cv::imshow( "Color", color_frame );
        cv::imshow( "Depth", depth_frame );

        const int32_t key = cv::waitKey( 33 );
        if( key == 'q' ){
            break;
        }
    }
    catch (cv::Exception& e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }
    catch (...) {
        std::cout << "unknown error!" << std::endl;
        return -1;
    }
    pipe.stop();
    cv::destroyAllWindows();
    return 0;
}