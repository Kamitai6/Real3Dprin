#include <iostream>
#include <cstdlib>
#include <cmath> 
#include <tuple>
#include <string>
#include <thread>
#include <chrono>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <Serial.hpp>


cv::Scalar HSV_MIN_1(0, 150, 100);
cv::Scalar HSV_MAX_1(5, 255, 255);

cv::Scalar HSV_MIN_2(170, 150, 100);
cv::Scalar HSV_MAX_2(180, 255, 255);

constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;

constexpr double FIELD_LINE = 3.0;
constexpr double CAMERA_HIGH = 1.4;
constexpr double CAMERA_ANGLE = 20.0 * 0.017453292519943; //degree * (PI / 180)

constexpr int PX_X_HALF = 1280 / 2;
constexpr int PX_Y_HALF = 720 / 2;
constexpr double COEF_X = 674.4192801798158;
constexpr double COEF_Y = 649.4571918977125;

const std::string PORT = "/dev/ttyUSB0";
constexpr int RATE = 115200;

Serial serial;


void send(int value1, int value2) {
    std::vector<unsigned char> send_data{};
    
    std::string a = std::to_string(value1);
    std::string b = std::to_string(value2);
    int l_a = 4 - a.length();
    int l_b = 4 - b.length();
    std::string zero_a{}, zero_b{};
    for (int i{}; i < l_a; ++i) zero_a += "0";
    for (int i{}; i < l_b; ++i) zero_b += "0";
    
    std::string data = "/" + zero_a + a + "." + zero_b + b;
    
    std::vector<char> chars{ std::begin(data), std::end(data) };
    std::transform(std::begin(chars), std::end(chars), std::back_inserter(send_data), [](char c) { return static_cast<unsigned char>(c); });
    serial.write(send_data);
}

cv::Mat detection(cv::Mat color_img) {
    cv::Mat filted_img;
    cv::Size ksize = cv::Size(5, 5);
    cv::GaussianBlur(color_img, filted_img, ksize, 0);

    cv::Mat hsv_img;
    cv::Mat mask_img_1, mask_img_2;
    cv::cvtColor(filted_img, hsv_img, cv::COLOR_BGR2GRAY);
    cv::inRange(hsv_img, HSV_MIN_1, HSV_MAX_1, mask_img_1);
    cv::inRange(hsv_img, HSV_MIN_2, HSV_MAX_2, mask_img_2);

    cv::Mat result_img;
    cv::add(mask_img_1, mask_img_2, result_img);

    //return mask_img_2;
    return result_img;
}

std::tuple<double, int, int> getDistance(cv::Mat depth_img, cv::Mat mask_img) {
    cv::Mat label_img;
    cv::Mat stats;
    cv::Mat centroids;
    int label = cv::connectedComponentsWithStats(mask_img, label_img, stats, centroids);

    int max = 0, max_num = 0;
    for (int i = 1; i < label; ++i) {
        int *param = stats.ptr<int>(i);
        int area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];
		if(max < area) {
            max = area;
            max_num = i;
        }
	}
    double *param = centroids.ptr<double>(max_num);
    int x = static_cast<int>(param[0]);
    int y = static_cast<int>(param[1]);

    double mean = static_cast<double>(cv::mean(depth_img, mask_img)[0]);
    double distance = mean != 0 ? mean : 0.0;

    return {distance, x, y};
}

std::tuple<double, double, double> convertDataXYZ(double distance, int x_px, int y_px, double angle) {
    // get x ratio
    int x_dis = x_px - PX_X_HALF;
    double x_tan = x_dis / COEF_X;

    // get y ratio
    int y_dis = y_px - PX_Y_HALF;
    double y_tan = y_dis / COEF_Y;
    
    double depth_ratio = std::sqrt(std::pow(x_tan, 2) + std::pow(y_tan, 2) + 1);

    // get x (meter)
    double x_m = distance / depth_ratio * x_tan;
    // get y (meter)
    double y_m = distance / depth_ratio * y_tan;
    // get z (meter)
    double z_m = distance / depth_ratio;
    
    double cosine = std::cos(angle);
    double   sine = std::sin(angle);

    y_m *= -1;
    y_m = -z_m *   sine + y_m * cosine;
    z_m =  z_m * cosine + y_m *   sine;

    return {x_m, y_m, z_m};
}

std::tuple<double, double> getFallPoint(double x, double y, double z) {
    static std::chrono::system_clock::time_point last_time;
    static double pre_x, pre_y, pre_z;
    static bool flag = false;

    if (!flag) {
        last_time = std::chrono::system_clock::now();
        pre_x = x; pre_y = y; pre_z = z;
        flag = true;
        return {0.0, 0.0};
    }
    else {
        // 初速を求める。
        std::chrono::system_clock::time_point now_time = std::chrono::system_clock::now();
        double diff_time = std::chrono::duration_cast<std::chrono::milliseconds>( now_time - last_time ).count() / 1000.0;
        last_time = now_time;
        double diff_y = y - pre_y;
        
        // m/s
        double m_per_s = diff_y / diff_time;

        // 何秒後落下するか求める。
        double t = (m_per_s + std::sqrt(std::pow(m_per_s, 2) + 2*9.8*y)) / 9.8;
       
        // 党則直線運動を調べる。
        double diff_x = x - pre_x;
        double diff_z = z - pre_z;

        pre_x = x;
        pre_y = y;
        pre_z = z;

        if (diff_x != 0.0) {
            diff_x = (diff_x * 4 + diff_x) / 5;
        }
        if (diff_z != 0.0) {
            diff_z = (diff_z * 4 + diff_z) / 5;
        }
        double x_m = diff_x / diff_time*t;
        double z_m = diff_z / diff_time*t;

        return {x + x_m, z + z_m};
    }
}


int main() {
    //serial
    auto list = getSerialList();
	for (const auto info : list) {
		std::cout << "device name:" << info.device_name() << std::endl;
		std::cout << "name:" << info.port() << "\n" << std::endl;
	}
	if (!serial.open(list[0].port(), RATE)) return -1;
	SerialInfo info = serial.getInfo();
	std::cout << "open success" << std::endl;

    //realsense
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
    rs2::pipeline_profile selection = pipe.start(cfg);
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        pipe.wait_for_frames();
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
    }

    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        std::this_thread::sleep_for(std::chrono::seconds(1));
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }

    while (cv::waitKey(1) < 0) try {
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto RGB = frames.get_color_frame();
        cv::Mat color_frame(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)RGB.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_frame(cv::Size(WIDTH, HEIGHT), CV_16S, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        cv::Mat mask = detection(color_frame);
        auto [distance, center_x, center_y] = getDistance(depth_frame, mask);
        auto [point_x, point_y, point_z] = convertDataXYZ(distance, center_x, center_y, CAMERA_ANGLE);

        auto [target_x, target_z] = getFallPoint(point_x, point_y + CAMERA_HIGH, point_z);

        if (std::abs(target_x) < FIELD_LINE / 2 && target_z < FIELD_LINE && target_z > 0) {
            int a = static_cast<int>((target_x + FIELD_LINE * 0.5) * 1000);
            int b = static_cast<int>(target_z);
            send(a, b);
            std::cout << a << " " << b << std::endl;
        }
        else {
            int a = 0;
            int b = 0;
            send(a, b);
            std::cout << a << " " << b << std::endl;
        }
        
        cv::imshow( "Color", color_frame );
        cv::imshow( "Depth", depth_frame );
        cv::imshow( "Mask", mask );

        const int32_t key = cv::waitKey( 33 );
        if( key == 'q' ){
            break;
        }
    }
    catch (std::exception e) {
        std::cout << e.what() << std::endl;
    }
    catch (...) {
        std::cout << "unknown error!" << std::endl;
    }
    pipe.stop();
    cv::destroyAllWindows();
    return 0;
}