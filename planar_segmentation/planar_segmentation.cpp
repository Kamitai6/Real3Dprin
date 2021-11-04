
#include <iostream>  
#include <librealsense2/rs.hpp>
#include <pcl/ModelCoefficients.h>  
#include <pcl/io/openni_grabber.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/visualization/cloud_viewer.h>  
  

using RGB_Cloud = pcl::PointXYZRGB;
using point_cloud = pcl::PointCloud<RGB_Cloud>;
using cloud_pointer = point_cloud::Ptr;

void segmentate(pcl::PointCloud<pcl::PointXYZRGB>& cloud, double threshould) {  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
  // Create the segmentation object  
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;  
  // Optional  
  seg.setOptimizeCoefficients (true);  
  // Mandatory  
  seg.setModelType (pcl::SACMODEL_PLANE);  
  seg.setMethodType (pcl::SAC_RANSAC);  
  seg.setDistanceThreshold (threshould);
  seg.setMaxIterations (10);
  
  seg.setInputCloud (cloud.makeShared ());  
  seg.segment (*inliers, *coefficients);  
  
  for (size_t i = 0; i < inliers->indices.size (); ++i) {  
    cloud.points[inliers->indices[i]].r = 255;  
    cloud.points[inliers->indices[i]].g = 0;  
    cloud.points[inliers->indices[i]].b = 0;  
  }  
}  
  
 class SimpleOpenNIViewer  
 {  
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}   
  
     void run ()  
     {  
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
       boost::shared_ptr<pcl::visualization::PCLVisualizer> openCloud;
      rs2::pointcloud pc;
      rs2::pipeline pipe;
      rs2::config cfg;
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
    while (!viewer.wasStopped())  
       {  
         auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto RGB = frames.get_color_frame();
        pc.map_to(RGB);
        auto points = pc.calculate(depth);
        cloud_pointer cloud = PCL_Conversion(points, RGB);
        pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
        Cloud_Filter.setInputCloud (cloud);           // Input generated cloud to filter
        Cloud_Filter.setFilterFieldName ("z");        // Set field name to Z-coordinate
        Cloud_Filter.setFilterLimits (0.0, 1.0);      // Set accepted interval values
        Cloud_Filter.filter (*newCloud);              // Filtered Cloud Outputted
        pcl::PointCloud<pcl::PointXYZRGB> segmented_cloud(*cloud);  
        segmentate(segmented_cloud, 0.05);  
        viewer.showCloud (segmented_cloud.makeShared());  
       }
     }  

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
        cloud->points[i].r = std::get<0>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = std::get<2>(RGB_Color); // Reference tuple<0>

    }
    
   return cloud; // PCL RGB Point Cloud generated
}
  

     pcl::visualization::CloudViewer viewer;  
 };  
  
 int main ()  
 {  
   SimpleOpenNIViewer v;  
   v.run ();  
   return 0;  
 }