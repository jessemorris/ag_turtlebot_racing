#include "virtual_object.hpp"
#include "mesh_visualiser/RequestModelView.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/transform_datatypes.h>

#include <memory>
 
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

VirtualObject::VirtualObject(ros::NodeHandle& _nh, const std::string& _object_name, 
	int _global_x, int _global_y, std::string file_suffix) :
        nh(_nh),
		global_x(_global_x),
		global_y(_global_y),
        object_name(_object_name),
		cloud(new pcl::PCLPointCloud2),
		point_cloud(new pcl::PointCloud<PointTypeColor>()),
		viewer("3D Mesh Viewer"),
		image_transport(_nh),
		should_run(true),
		scene_sensor_pose(Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()))
    {
		nh.getParam("/mesh_visualiser/default_depth", default_depth);
		nh.getParam("/mesh_visualiser/default_solver_divide", default_solver_divide);
		nh.getParam("/mesh_visualiser/default_iso_divide", default_iso_divide);
		nh.getParam("/mesh_visualiser/default_point_weight", default_point_weight);

		nh.getParam("/mesh_visualiser/noise_level", noise_level);
		nh.getParam("/mesh_visualiser/min_range", min_range);
		nh.getParam("/mesh_visualiser/border_size", border_size);


		ROS_INFO_STREAM("default depth " << default_depth);
		ROS_INFO_STREAM("default_solver_divide " << default_solver_divide);
		ROS_INFO_STREAM("default_iso_divide " << default_iso_divide);
		ROS_INFO_STREAM("default_point_weight " << default_point_weight);
		resource_path = ros::package::getPath("mesh_visualiser") + std::string("/pc_resources");
		ROS_INFO_STREAM(resource_path);
		file_path = resource_path + "/" + object_name + file_suffix;
		ROS_INFO_STREAM(file_path);

		// cloud = std::unique_ptr<pcl::PCLPointCloud2>();
		// point_cloud = std::unique_ptr<pcl::PointCloud<PointTypeColor>>();

		// *point_cloud = *point_cloud_ptr;

		ROS_INFO_STREAM("made point clounds");



		if (pcl::io::loadPCDFile(file_path, *cloud) == -1)	{
			ROS_WARN_STREAM("Was not able to open file " <<file_path); 
		}

		else {
			ROS_INFO_STREAM("File " << object_name << " was loaded");
			pcl::fromPCLPointCloud2 (*cloud, *point_cloud);
			image_request_service = nh.advertiseService("model_view_" + object_name, &VirtualObject::model_view_callback, this);
			mesh_publisher = image_transport.advertise("mesh_visualiser/" + object_name + "/render", 1);

			angular_resolution_x = 0.1f;
			angular_resolution_y = 0.1f;

			angular_resolution_x = pcl::deg2rad (angular_resolution_x);
			angular_resolution_y = pcl::deg2rad (angular_resolution_y);

			compute_mesh_polygon(cloud, output, default_depth, default_solver_divide, default_iso_divide, default_point_weight);


			ROS_INFO_STREAM(output.polygons.size());

			viewer.addPolygonMesh(output, "polygon");
			viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING,
														pcl::visualization::PCL_VISUALIZER_SHADING_FLAT, "polygon");
			
			// mesh_viewer.addCoordinateSystem (3.0f, "global");

			//set default viewing pose
			scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud->sensor_origin_[0],
						point_cloud->sensor_origin_[1],
						point_cloud->sensor_origin_[2] - 1)) *
						Eigen::Affine3f (Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));

			
			range_image = std::make_unique<pcl::RangeImage>();
			range_image->createFromPointCloud (*point_cloud, angular_resolution_x, angular_resolution_y,
										pcl::deg2rad (360.0f), pcl::deg2rad (360.0f),
										scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

			viewer.initCameraParameters();
			set_viewer_pose(range_image->getTransformationToWorldSystem());

			renderer_thread = std::thread(&VirtualObject::render_thread, this);
			ROS_INFO_STREAM("made thread");



		}


		

    }

VirtualObject::~VirtualObject() {
	should_run = false;
	ros::Duration(1).sleep();

	if(renderer_thread.joinable()) {
		renderer_thread.join();
	}
}

bool VirtualObject::model_view_callback(mesh_visualiser::RequestModelView::Request& request,
                                                mesh_visualiser::RequestModelView::Response& response) 
	{
    	tf2::convert(request.orientation, last_orientation);

		//wait for renderer thread to run
		ros::Duration(0.5).sleep();

		sensor_msgs::ImagePtr img_msg;
		image_mutex.lock();
		img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", rendered_image).toImageMsg();
		ROS_INFO_STREAM(rendered_image.rows << " " << rendered_image.cols);

		// image_test_pub.publish(img_msg);
		mesh_publisher.publish(img_msg);
		response.image = *img_msg;
		image_mutex.unlock();

		return true;



	}



void VirtualObject::compute_mesh_polygon(const pcl::PCLPointCloud2::ConstPtr &input, pcl::PolygonMesh &output,
         int depth, int solver_divide, int iso_divide, float point_weight) {

	pcl::PointCloud<pcl::PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2 (*input, *xyz_cloud);
	pcl::fromPCLPointCloud2 (*input, *normal_cloud);

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




void VirtualObject::set_viewer_pose(const Eigen::Affine3f& viewer_pose) {
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]);

}


bool VirtualObject::create_render(tf2::Quaternion orientation, cv::Mat& image) {

	Eigen::Quaternion<float> q(orientation.x(), orientation.y(), orientation.z(), orientation.w());
	q.normalize();
	ROS_INFO_STREAM("here6");

	scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud->sensor_origin_[0],
						point_cloud->sensor_origin_[1],
						point_cloud->sensor_origin_[2] - 1)) *
						Eigen::Affine3f (Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));
	ROS_INFO_STREAM("here5");
	viewer.spinOnce();

	ROS_INFO_STREAM("here1");

  	// Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();
	// Eigen::Affine3f viewer_pose(rotation_matrix);
	// //update viwwer pose
	// set_viewer_pose(viewer_pose);
    
	// scene_sensor_pose = viewer.getViewerPose();

	range_image->createFromPointCloud (*point_cloud, angular_resolution_x, angular_resolution_y,
									pcl::deg2rad (360.0f), pcl::deg2rad (360.0f),
									scene_sensor_pose, pcl::RangeImage::CAMERA_FRAME, noise_level, min_range, border_size);
	ROS_INFO_STREAM("here2");

	range_image->setUnseenToMaxRange();


    // ros::Duration(0.05).sleep();
	pcl_sleep(0.05);
	vtkSmartPointer< vtkRenderWindow> vtk_render = viewer.getRenderWindow();
	// vtk_render->GetRGBAPixelData(0, 0, 640, 460, 1);viewer
	// vtk_render->SetOffScreenRendering(1);
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	windowToImageFilter->SetInput(vtk_render);
	windowToImageFilter->Update();
	vtkSmartPointer<vtkImageData> image_data = windowToImageFilter->GetOutput();

	ROS_INFO_STREAM("here3");

	int dim[3];
	image_data->GetDimensions(dim);

	int imageWidth = dim[0];
	int imageHeight = dim[1];


	// 0, 0, this->w_ - 1, this->h_ - 1, true
	unsigned char *pixels = vtk_render->GetRGBACharPixelData(0, 0, imageWidth - 1, imageHeight - 1, true);

	ROS_INFO_STREAM("here4");


	image = cv::Mat(imageHeight, imageWidth, CV_8UC4, pixels);
	ROS_INFO_STREAM(rendered_image.rows << " " << rendered_image.cols);

	cv::flip(image,image, 0);
	cv::resize(image, image, cv::Size(640,480));

	return true;
}

void VirtualObject::render_thread() {

	while(should_run && !viewer.wasStopped()) {
		ROS_INFO_STREAM("here");

		create_render(last_orientation, rendered_image);

	}
}

