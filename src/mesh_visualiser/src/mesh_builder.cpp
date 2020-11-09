#include "mesh_builder.hpp"


MeshBuilder::MeshBuilder(const std::string& _folder_path) :
    folder_path(_folder_path) {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MeshBuilder::extract_mesh(const std::string& _object_name) {
    return nullptr;
}