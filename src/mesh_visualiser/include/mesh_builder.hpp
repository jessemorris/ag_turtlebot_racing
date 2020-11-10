#ifndef AR_TURTLEBOT_MESH_BUILDER
#define AR_TURTLEBOT_MESH_BUILDER



#include <thread>
#include <sstream>

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
// #include <open3d_conversions/open3d_conversions.h>


class MeshBuilder {

    public:
        MeshBuilder(const std::string& _folder_path);
        ~MeshBuilder() {};

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_mesh(const std::string& _object_name);
    private:
        const std::string& folder_path;
};

#endif