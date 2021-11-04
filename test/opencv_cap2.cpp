#include <iostream>
#include <algorithm> 
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

// Intel Realsense Headers
#include <librealsense2/rs.hpp>

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


using RGB_Cloud = pcl::PointXYZRGB;
using point_cloud = pcl::PointCloud<RGB_Cloud>;
using cloud_pointer = point_cloud::Ptr;
using prevCloud = point_cloud::Ptr;
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

// Prototypes
void Load_PCDFile(void);
bool userInput(void);

// Global Variables
std::string cloudFile; // .pcd file name
std::string prevCloudFile; // .pcd file name (Old cloud)
int i = 1; // Index for incremental file name

std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = std::min(std::max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color){

    // Object Declaration (Point Cloud)
    cloud_pointer cloud(new point_cloud);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = std::get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = std::get<0>(RGB_Color); // Reference tuple<0>

    }
    
   return cloud; // PCL RGB Point Cloud generated
}

// pcl_ptr points_to_pcl(const rs2::points& points)
// {
//     pcl_ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

//     auto sp = points.get_profile().as<rs2::video_stream_profile>();
//     cloud->width = sp.width();
//     cloud->height = sp.height();
//     cloud->is_dense = false;
//     cloud->points.resize(points.size());
//     auto ptr = points.get_vertices();
//     for (auto& p : cloud->points)
//     {
//         p.x = ptr->x;
//         p.y = ptr->y;
//         p.z = ptr->z;
//         ptr++;
//     }

//     return cloud;
// }

int main(int argc, char * argv[]) try
{
    //======================
    // Variable Declaration
    //======================
    bool captureLoop = true; // Loop control for generating point clouds
   
    //====================
    // Object Declaration
    //====================
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> openCloud;

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //======================
    // Stream configuration with parameters resolved internally. See enable_stream() overloads for extended usage
    //======================
    cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_INFRARED);
    cfg.enable_stream(RS2_STREAM_DEPTH);
    
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
        sleep(1);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }

    // Begin Stream with default configs

    // Loop and take frame captures upon user input
    while(captureLoop == true) {

        // Set loop flag based on user input
        captureLoop = userInput(); 
        if (captureLoop == false) { break; }
        

         // Wait for frames from the camera to settle
        for (int i = 0; i < 30; i++) {
            auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
        }

        // Capture a single frame and obtain depth + RGB values from it    
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto RGB = frames.get_color_frame();

        // Map Color texture to each point
        pc.map_to(RGB);

        // Generate Point Cloud
        auto points = pc.calculate(depth);

        // Convert generated Point Cloud to PCL Formatting
        cloud_pointer cloud = PCL_Conversion(points, RGB);
        
        //========================================
        // Filter PointCloud (PassThrough Method)
        //========================================
        pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
        Cloud_Filter.setInputCloud (cloud);           // Input generated cloud to filter
        Cloud_Filter.setFilterFieldName ("z");        // Set field name to Z-coordinate
        Cloud_Filter.setFilterLimits (0.0, 1.0);      // Set accepted interval values
        Cloud_Filter.filter (*newCloud);              // Filtered Cloud Outputted
        
        cloudFile = "Captured_Frame" + std::to_string(i) + ".pcd";
        
        //==============================
        // Write PC to .pcd File Format
        //==============================
        // Take Cloud Data and write to .PCD File Format
        cout << "Generating PCD Point Cloud File... " << endl;
        pcl::io::savePCDFileASCII(cloudFile, *cloud); // Input cloud to be saved to .pcd
        cout << cloudFile << " successfully generated. " << endl; 
        
        //Load generated PCD file for viewing
        Load_PCDFile();
        i++; // Increment File Name
    }//End-while
   
   
    cout << "Exiting Program... " << endl; 
    return EXIT_SUCCESS;
    
    // const auto window_name = "Display Image";
    // cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    // rs2::colorizer color_map;
    // rs2::pointcloud pc;
    // rs2::points points;
    // rs2::pipeline pipe;
    // pipe.start();

    // auto frames = pipe.wait_for_frames();
    // auto depth = frames.get_depth_frame();
    // points = pc.calculate(depth);
    // auto pcl_points = points_to_pcl(points);

    // pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(pcl_points);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.0, 1.0);
    // pass.filter(*cloud_filtered);

    // while (cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
    // {
    //     if( !cloud_filtered->empty() ){
    //         cv::Mat image ( cloud_filtered->height, cloud_filtered->width, CV_8UC4 );

    //         // pcl::PointCloud to cv::Mat
    //         #pragma omp parallel for
    //         for( int y{}; y < image.rows; ++y ) {
    //             for( int x{}; x < image.cols; ++x ) {
    //                 pcl::PointXYZRGBA point = cloud_filtered->at( x, y );
    //                 image.at<cv::Vec4b>( y, x )[0] = point.b;
    //                 image.at<cv::Vec4b>( y, x )[1] = point.g;
    //                 image.at<cv::Vec4b>( y, x )[2] = point.r;
    //                 image.at<cv::Vec4b>( y, x )[3] = point.a;
    //             }
    //         }
    //         cv::imshow(window_name, image);
    //     }
    // }

    // return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void Load_PCDFile(void)
{
    std::string openFileName;

    // Generate object to store cloud in .pcd file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudView (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    openFileName = "Captured_Frame" + std::to_string(i) + ".pcd";
    pcl::io::loadPCDFile (openFileName, *cloudView); // Load .pcd File
    
    //==========================
    // Pointcloud Visualization
    //==========================
    // Create viewer object titled "Captured Frame"
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Captured Frame"));

    // Set background of viewer to black
    viewer->setBackgroundColor (0, 0, 0); 
    // Add generated point cloud and identify with string "Cloud"
    viewer->addPointCloud<pcl::PointXYZRGB> (cloudView, "Cloud");
    // Default size for rendered points
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
    // Viewer Properties
    viewer->initCameraParameters();  // Camera Parameters for ease of viewing

    cout << endl;
    cout << "Press [Q] in viewer to continue. " << endl;
    
    viewer->spin(); // Allow user to rotate point cloud and view it

    // Note: No method to close PC visualizer, pressing Q to continue software flow only solution.
   
    
}
//========================================
// userInput
// - Prompts user for a char to 
// test for decision making.
// [y|Y] - Capture frame and save as .pcd
// [n|N] - Exit program
//========================================
bool userInput(void){

    bool setLoopFlag;
    bool inputCheck = false;
    char takeFrame; // Utilize to trigger frame capture from key press ('t')
    do {

        // Prompt User to execute frame capture algorithm
        cout << endl;
        cout << "Generate a Point Cloud? [y/n] "; 
        cin >> takeFrame;
        cout << endl;
        // Condition [Y] - Capture frame, store in PCL object and display
        if (takeFrame == 'y' || takeFrame == 'Y') {
            setLoopFlag = true;
            inputCheck = true;
            takeFrame = 0;
        }
        // Condition [N] - Exit Loop and close program
        else if (takeFrame == 'n' || takeFrame == 'N') {
            setLoopFlag = false;
            inputCheck = true;
            takeFrame = 0;
        }
        // Invalid Input, prompt user again.
        else {
            inputCheck = false;
            cout << "Invalid Input." << endl;
            takeFrame = 0;
        }
    } while(inputCheck == false);

    return setLoopFlag; 
}