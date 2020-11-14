#ifndef AR_TURTLEBOT_VIRTUAL_OBJECT
#define AR_TURTLEBOT_VIRTUAL_OBJECT



#include <thread>
#include <sstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
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
#include <vtkCamera.h>


#include "mesh_visualiser/RequestModelView.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include "tf/transform_datatypes.h"


#include <pcl/io/obj_io.h>
#include <pcl/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/texture_mapping.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <memory>
#include <thread>
#include <mutex>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGB PointTypeColor;

class VirtualObject {

    public:
        VirtualObject(ros::NodeHandle& _nh, const std::string& _object_name, 
			int _global_x, int _global_y, std::string file_suffix = ".pcd");
        ~VirtualObject();

        void compute_mesh_polygon(const pcl::PointCloud<PointTypeColor>::Ptr input, pcl::PolygonMesh &output,
        	int depth, int solver_divide, int iso_divide, float point_weight);

        bool model_view_callback(mesh_visualiser::RequestModelView::Request& request,
                                                mesh_visualiser::RequestModelView::Response& response);

    private:

        ros::NodeHandle nh;
        const std::string object_name;
		const int global_x;
		const int global_y;
        std::string file_path;

        std::string resource_path;

		//ros networking
        ros::ServiceServer image_request_service;
		image_transport::ImageTransport image_transport;
		image_transport::Publisher mesh_publisher;

		double origin_x_offset;
		double origin_y_offset;
		double origin_z_offset;

		

        pcl::PCLPointCloud2::Ptr cloud;
        pcl::PointCloud<PointTypeColor>::Ptr point_cloud;
		pcl::PointCloud<PointTypeColor>::Ptr transformed_point_cloud;

		tf2_ros::StaticTransformBroadcaster static_broadcaster;


		float angular_resolution_x;
      	float angular_resolution_y;


		//params for mesh reconstruction
		int default_depth;
		int default_solver_divide;
		int default_iso_divide;
		float default_point_weight;

		float noise_level;
		float min_range;
		int border_size;

		pcl::visualization::PCLVisualizer viewer;
		pcl::PolygonMesh output;

		std::unique_ptr<pcl::RangeImage> range_image;

		//view orientation
		Eigen::Affine3f scene_sensor_pose;


		cv::Mat rendered_image;



	void set_viewer_pose(const Eigen::Affine3f& viewer_pose);
	bool create_render(tf2::Quaternion orientation, cv::Mat& rendered_image);


};

#endif