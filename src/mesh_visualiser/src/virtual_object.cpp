#include "virtual_object.hpp"
#include "mesh_visualiser/RequestModelView.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/transform_datatypes.h>
#include <pcl/common/transforms.h>


#include <memory>



VirtualObject::VirtualObject(ros::NodeHandle& _nh, const std::string& _object_name,
	float _global_x, float _global_y, std::string file_suffix) :
        nh(_nh),
		global_x(_global_x),
		global_y(_global_y),
        object_name(_object_name),
		viewer(object_name + " Viewer"),
		cloud(new pcl::PCLPointCloud2),
		point_cloud(new pcl::PointCloud<PointTypeColor>()),
		transformed_point_cloud(new pcl::PointCloud<PointTypeColor>()),
		image_transport(_nh),
		scene_sensor_pose(Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()))
    {
		nh.getParam("/mesh_visualiser/default_depth", default_depth);
		nh.getParam("/mesh_visualiser/default_solver_divide", default_solver_divide);
		nh.getParam("/mesh_visualiser/default_iso_divide", default_iso_divide);
		nh.getParam("/mesh_visualiser/default_point_weight", default_point_weight);

		nh.getParam("/mesh_visualiser/noise_level", noise_level);
		nh.getParam("/mesh_visualiser/min_range", min_range);
		nh.getParam("/mesh_visualiser/border_size", border_size);

		nh.getParam("/mesh_visualiser/origin_x_offset", origin_x_offset);
		nh.getParam("/mesh_visualiser/origin_y_offset", origin_y_offset);
		nh.getParam("/mesh_visualiser/origin_z_offset", origin_z_offset);

		resource_path = ros::package::getPath("mesh_visualiser") + std::string("/pc_resources");
		file_path = resource_path + "/" + object_name + file_suffix;

		int result = -1;

		if (file_suffix == ".pcd") {
			result = pcl::io::loadPCDFile(file_path, *cloud);
		}
		else if(file_suffix == ".ply") {
			result = pcl::io::loadPLYFile(file_path, *cloud);
		}
		if (result == -1) {
			ROS_INFO_STREAM("File " << file_path << " could not be loaded");
		}
		else {
			ROS_INFO_STREAM("File " << object_name << " was loaded");
			pcl::fromPCLPointCloud2 (*cloud, *point_cloud);

			Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

			transform_2.rotate(Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf (M_PI, Eigen::Vector3f::UnitX()) );
			pcl::transformPointCloud(*point_cloud, *transformed_point_cloud, transform_2);


			angular_resolution_x = 0.1f;
			angular_resolution_y = 0.1f;

			angular_resolution_x = pcl::deg2rad (angular_resolution_x);
			angular_resolution_y = pcl::deg2rad (angular_resolution_y);

			compute_mesh_polygon(transformed_point_cloud, output, default_depth, default_solver_divide, default_iso_divide, default_point_weight);


			vtkSmartPointer< vtkRenderWindow> vtk_render = viewer.getRenderWindow();
			vtk_render->SetOffScreenRendering(1);



			viewer.addPolygonMesh(output,object_name);
			viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING,
														pcl::visualization::PCL_VISUALIZER_SHADING_FLAT,object_name);



			scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (transformed_point_cloud->sensor_origin_[0] +  origin_x_offset,
				transformed_point_cloud->sensor_origin_[1] + origin_y_offset,
				transformed_point_cloud->sensor_origin_[2] + origin_z_offset)) *
				Eigen::Affine3f (Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));

			range_image = std::make_unique<pcl::RangeImage>();

			range_image->createFromPointCloud (*transformed_point_cloud, angular_resolution_x, angular_resolution_y,
										pcl::deg2rad (360.0f), pcl::deg2rad (360.0f),
										scene_sensor_pose, pcl::RangeImage::CAMERA_FRAME, noise_level, min_range, border_size);

			viewer.initCameraParameters();
			set_viewer_pose(range_image->getTransformationToWorldSystem());


			image_request_service = nh.advertiseService("mesh_visualiser/model_view_" + object_name, &VirtualObject::model_view_callback, this);
			mesh_publisher = image_transport.advertise("mesh_visualiser/" + object_name + "/render", 1);

			//set up static transform
			geometry_msgs::TransformStamped static_transformstamped;
			static_transformstamped.header.stamp = ros::Time::now();
			static_transformstamped.header.frame_id = "map";
			static_transformstamped.child_frame_id = object_name;

			static_transformstamped.transform.translation.x = global_x;
			static_transformstamped.transform.translation.y = global_y;
			static_transformstamped.transform.translation.z = 0.0;

			static_transformstamped.transform.rotation.x = 0;
			static_transformstamped.transform.rotation.y = 0;
			static_transformstamped.transform.rotation.z = 0;
			static_transformstamped.transform.rotation.w = 1;

			static_broadcaster.sendTransform(static_transformstamped);


			static_transformstamped.header.stamp = ros::Time::now();
			static_transformstamped.header.frame_id = object_name;
			static_transformstamped.child_frame_id = "rotated_" + object_name;

			static_transformstamped.transform.translation.x = 0;
			static_transformstamped.transform.translation.y = 0;
			static_transformstamped.transform.translation.z = 0.0;


			tf2::Quaternion quat;

			quat.setRPY(-M_PI/2.0, 0, -M_PI/2.0);


            static_transformstamped.transform.rotation.x = quat.x();
            static_transformstamped.transform.rotation.y = quat.y();
            static_transformstamped.transform.rotation.z = quat.z();
            static_transformstamped.transform.rotation.w = quat.w();

			static_broadcaster.sendTransform(static_transformstamped);
		}

    }

VirtualObject::~VirtualObject() {}

bool VirtualObject::model_view_callback(mesh_visualiser::RequestModelView::Request& request,
                                                mesh_visualiser::RequestModelView::Response& response)
	{
		tf2::Quaternion last_orientation;
		cv::Mat image;

    	tf2::convert(request.orientation, last_orientation);
		ros::Duration(0.5).sleep();

		create_render(last_orientation, image);

		sensor_msgs::ImagePtr img_msg;
		img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", image).toImageMsg();

		mesh_publisher.publish(img_msg);
		response.image = *img_msg;

		return true;

	}



void VirtualObject::compute_mesh_polygon(pcl::PointCloud<PointTypeColor>::Ptr input, pcl::PolygonMesh &output,
         int depth, int solver_divide, int iso_divide, float point_weight) {

	pcl::PointCloud<pcl::PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZ>);


	pcl::copyPointCloud(*input, *xyz_cloud);
  	pcl::copyPointCloud(*input, *normal_cloud);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (normal_cloud);

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
	// std::cout << "normal estimation complete" << std::endl;
	// std::cout << "reverse normals' direction" << std::endl;

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
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]);
}


bool VirtualObject::create_render(tf2::Quaternion orientation, cv::Mat& image) {

	ROS_INFO_STREAM("Q: x " << orientation.x() << " y " << orientation.y() << " z " << orientation.z() << " z " << orientation.w());
	Eigen::Quaternion<float> q(orientation.x(), orientation.y(), orientation.z(), orientation.w());
	q.normalize();

	viewer.removePolygonMesh(object_name);

	Eigen::Vector3f euler = Eigen::Matrix3f(q).eulerAngles(0,1,2);

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	transform.rotate(Eigen::AngleAxisf (euler[0], Eigen::Vector3f::UnitX()) *
					Eigen::AngleAxisf (euler[1], Eigen::Vector3f::UnitY()) *
					Eigen::AngleAxisf (euler[2], Eigen::Vector3f::UnitZ()));

	pcl::transformPointCloud(*point_cloud, *transformed_point_cloud, transform);

	//must re-render from orientated point cloud
	compute_mesh_polygon(transformed_point_cloud, output, default_depth, default_solver_divide, default_iso_divide, default_point_weight);


	ROS_INFO_STREAM(output.polygons.size());

	vtkSmartPointer< vtkRenderWindow> vtk_render = viewer.getRenderWindow();
	// vtk_render->SetOffScreenRendering(1);

	viewer.addPolygonMesh(output, object_name);

	range_image->createFromPointCloud (*transformed_point_cloud, angular_resolution_x, angular_resolution_y,
								pcl::deg2rad (360.0f), pcl::deg2rad (360.0f),
								scene_sensor_pose, pcl::RangeImage::CAMERA_FRAME, noise_level, min_range, border_size);

	viewer.initCameraParameters();
	set_viewer_pose(range_image->getTransformationToWorldSystem());

	viewer.spinOnce();
    pcl_sleep (0.01);

	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	windowToImageFilter->SetInput(vtk_render);
	windowToImageFilter->Update();
	vtkSmartPointer<vtkImageData> image_data = windowToImageFilter->GetOutput();

	int dim[3];
	image_data->GetDimensions(dim);

	int imageWidth = dim[0];
	int imageHeight = dim[1];

	unsigned char *pixels = vtk_render->GetRGBACharPixelData(0, 0, imageWidth - 1, imageHeight - 1, true);

	image = cv::Mat(imageHeight, imageWidth, CV_8UC4, pixels);

	cv::flip(image,image, 0);
	cv::resize(image, image, cv::Size(640,480));

	return true;
}
