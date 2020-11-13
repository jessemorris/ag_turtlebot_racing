#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/console/parse.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/features2d.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/cvstd.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/ply/ply_parser.h>
#include <pcl/io/ply/ply.h>

#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <vtkWindowToImageFilter.h>

#include <pcl/range_image/range_image.h>

#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/features/gasd.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/surface/grid_projection.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/thread/thread.hpp>

#include <vtkSmartPointer.h>
#include <pcl/common/transforms.h>


#include <iostream>
#include <fstream>
#include <string>




typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGB PointTypeColor;

//https://pointclouds.org/documentation/tutorials/range_image_creation.html <- this has a bunch of information

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution_x = 0.1f,
      angular_resolution_y = angular_resolution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool live_update = false;

void compute (const pcl::PointCloud<PointTypeColor>::Ptr input, pcl::PolygonMesh &output,
         int depth, int solver_divide, int iso_divide, float point_weight)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointNormal> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*input, *xyz_cloud);
  pcl::copyPointCloud(*input, *normal_cloud);
//   pcl::fromPCLPointCloud2 (*input, *xyz_cloud);
//   pcl::fromPCLPointCloud2 (*input, *normal_cloud);

  // pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (normal_cloud);
  // ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.3);

  // Compute the features
  ne.compute (*cloud_normals);
  ROS_INFO_STREAM("computed cloud normals size " << cloud_normals->size());
  std::cout << "normal estimation complete" << std::endl;
  std::cout << "reverse normals' direction" << std::endl;

  for(size_t i = 0; i < cloud_normals->size(); ++i){
    cloud_normals->points[i].normal_x *= -1;
    cloud_normals->points[i].normal_y *= -1;
    cloud_normals->points[i].normal_z *= -1;
  }

	//estimate surface normals

	ROS_INFO_STREAM("input size " << xyz_cloud->size());
	ROS_INFO("Using parameters: depth %d, solverDivide %d, isoDivide %d\n", depth, solver_divide, iso_divide);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
	pcl::concatenateFields(*normal_cloud, *cloud_normals, *cloud_smoothed_normals);

	// pcl::Poisson<pcl::PointNormal> poisson;
	// poisson.setDepth (depth);
	// poisson.setSolverDivide (solver_divide);
	// poisson.setIsoDivide (iso_divide);
	// poisson.setPointWeight (point_weight);
	// poisson.setInputCloud (cloud_smoothed_normals);

	// poisson.reconstruct (output);

  	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
  	pcl::GridProjection<pcl::PointNormal> grid_projection;
	grid_projection.setInputCloud(cloud_smoothed_normals);
	grid_projection.setSearchMethod(tree2);
	grid_projection.setResolution(0.005);
	grid_projection.setPaddingSize(3);
	grid_projection.reconstruct(output);

}



void 
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-rx <float>  angular resolution in degrees (default "<<angular_resolution_x<<")\n"
            << "-ry <float>  angular resolution in degrees (default "<<angular_resolution_y<<")\n"
            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
            << "-l           live update - update the range image according to the selected view in the 3D viewer.\n"
            << "-h           this help\n"
            << "\n\n";
}

void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose) {
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

int  main (int argc, char** argv){

	ros::init(argc, argv, "mesh_visualiser");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub_img = it.advertise("mesh_vis", 20);
	sensor_msgs::ImagePtr img_msg;

	int default_depth = 8;
	int default_solver_divide = 8;
	int default_iso_divide = 8;
	float default_point_weight = 4.0f;



	nh.getParam("/mesh_visualiser/default_depth", default_depth);
	nh.getParam("/mesh_visualiser/default_solver_divide", default_solver_divide);
	nh.getParam("/mesh_visualiser/default_iso_divide", default_iso_divide);

	nh.getParam("/mesh_visualiser/default_point_weight", default_point_weight);

	ROS_INFO_STREAM("default depth " << default_depth);
	ROS_INFO_STREAM("default_solver_divide " << default_solver_divide);
	ROS_INFO_STREAM("default_iso_divide " << default_iso_divide);
	ROS_INFO_STREAM("default_point_weight " << default_point_weight);



	angular_resolution_x = pcl::deg2rad (angular_resolution_x);
	angular_resolution_y = pcl::deg2rad (angular_resolution_y);

	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);


	//https://roboticsknowledgebase.com/wiki/programming/eigen-library/
	Eigen::Affine3f scene_sensor_pose(Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));



	pcl::PointCloud<PointTypeColor>::Ptr point_cloud_ptr (new pcl::PointCloud<PointTypeColor> ());
	pcl::PointCloud<PointTypeColor>::Ptr transformed_point_cloud(new pcl::PointCloud<PointTypeColor> ());
	pcl::PointCloud<PointTypeColor>& point_cloud = *point_cloud_ptr;
		
	// pcl::PointCloud<PointTypeColor>& transformed_point_cloud = *transformed_point_cloud_ptr;


    // std::string filename = argv[pcd_filename_indices[0]];
	std::string filename =ros::package::getPath("mesh_visualiser") + std::string("/pc_resources/bunny.pcd");
    if (pcl::io::loadPCDFile(filename, *cloud) == -1)	{
      std::cout << "Was not able to open file \""<<filename<<"\".\n";
      printUsage (argv[0]);
      return 0;
    }
    pcl::fromPCLPointCloud2 (*cloud, point_cloud);

	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	transform_2.rotate(Eigen::AngleAxisf (M_PI, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf (M_PI/4, Eigen::Vector3f::UnitX()) );
	pcl::transformPointCloud(point_cloud, *transformed_point_cloud, transform_2);  

	scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (transformed_point_cloud->sensor_origin_[0] ,
		transformed_point_cloud->sensor_origin_[1],
		transformed_point_cloud->sensor_origin_[2] - 1)) *
		Eigen::Affine3f (Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));

	ROS_INFO_STREAM("made point cloud with " << transformed_point_cloud->size());
	ROS_INFO_STREAM(pcl::getFieldsList (*transformed_point_cloud).c_str ());

	// -----------------------------------------------
	// -----Create RangeImage from the PointCloud-----
	// -----------------------------------------------
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;   
	range_image.createFromPointCloud (*transformed_point_cloud, angular_resolution_x, angular_resolution_y,
										pcl::deg2rad (360.0f), pcl::deg2rad (360.0f),
										scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);


	pcl::visualization::PCLVisualizer mesh_viewer ("3D Mesh Viewer");
  

  

  pcl::PolygonMesh output;
  compute (transformed_point_cloud, output, default_depth, default_solver_divide, default_iso_divide, default_point_weight);

  ROS_INFO_STREAM(output.polygons.size());

  mesh_viewer.addPolygonMesh(output, "polygon");
  mesh_viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING,
                                            pcl::visualization::PCL_VISUALIZER_SHADING_FLAT, "polygon");
   
  // mesh_viewer.addCoordinateSystem (3.0f, "global");

  mesh_viewer.initCameraParameters ();
  setViewerPose(mesh_viewer, range_image.getTransformationToWorldSystem ());


  ROS_INFO_STREAM("made viewer");
//   //can set camea parameters
//   viewer.initCameraParameters ();
//   setViewerPose(viewer, range_image.getTransformationToWorldSystem ());
  
  // --------------------------
  // -----Show range image-----
  // --------------------------
//   pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
//   range_image_widget.showRangeImage (range_image);

  
  //--------------------
  // -----Main loop-----
  //--------------------
  cv::Mat image;
  while (!mesh_viewer.wasStopped()) {
    // range_image_widget.spinOnce ();
    // viewer.spinOnce ();
    mesh_viewer.spinOnce();
    pcl_sleep (0.01);
    
	
	scene_sensor_pose = mesh_viewer.getViewerPose();
	vtkSmartPointer< vtkRenderWindow> vtk_render = mesh_viewer.getRenderWindow();
	// vtk_render->GetRGBAPixelData(0, 0, 640, 460, 1);
	// vtk_render->SetOffScreenRendering(1);
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	windowToImageFilter->SetInput(vtk_render);
	windowToImageFilter->Update();
	vtkSmartPointer<vtkImageData> image_data = windowToImageFilter->GetOutput();

	int dim[3];
	image_data->GetDimensions(dim);

	int imageWidth = dim[0];
	int imageHeight = dim[1];


	// 0, 0, this->w_ - 1, this->h_ - 1, true
	unsigned char *pixels = vtk_render->GetRGBACharPixelData(0, 0, imageWidth - 1, imageHeight - 1, true);


	cv::Mat image = cv::Mat(imageHeight, imageWidth, CV_8UC4, pixels);




      // int* dims = image_data->GetDimensions();

      // unsigned char* image_data_array = static_cast<unsigned char*>(image_data->GetScalarPointer(0,0,0));


      //can trt createFromPointCloudWithKnownSize()
	range_image.createFromPointCloud (*transformed_point_cloud, angular_resolution_x, angular_resolution_y,
									pcl::deg2rad (360.0f), pcl::deg2rad (360.0f),
									scene_sensor_pose, pcl::RangeImage::CAMERA_FRAME, noise_level, min_range, border_size);

	range_image.setUnseenToMaxRange();
	// range_image.getRangeImageWithSmoothedSurface(50, range_image);

	ROS_INFO_STREAM("x " << range_image.width << " y " << range_image.height);

	cv::flip(image,image, 0);
	cv::resize(image, image, cv::Size(640,480));
	// cv::flip(image,image, 1);

	// Flip because of different origins between vtk and OpenCV
	// cv::flip(openCVImage,openCVImage, 0);

	// cv::Mat image(image_data.,range_image.width,CV_8UC3,&image_data_array[0]);
	img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgba8", image).toImageMsg();
	pub_img.publish(img_msg);
	ros::spinOnce();
	ROS_INFO_STREAM(image.rows << " " << image.cols);
	// delete[] ranges;
	// delete[] data;
	// delete[] dims;

        // cv::imshow("frame", image);
        // cv::waitKey(1);
    //   range_image_widget.showRangeImage (range_image);
  }
}



