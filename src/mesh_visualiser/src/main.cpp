#include <ros/ros.h>
#include <image_transport/image_transport.h>


#include <iostream>

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

#include <pcl/range_image/range_image.h>

#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>

#include <pcl/ModelCoefficients.h>

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

#include <boost/filesystem.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/thread/thread.hpp>

#include <iostream>
#include <fstream>
#include <string>




typedef pcl::PointXYZ PointType;

//https://pointclouds.org/documentation/tutorials/range_image_creation.html <- this has a bunch of information

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution_x = 0.1f,
      angular_resolution_y = angular_resolution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool live_update = false;

// --------------
// -----Help-----
// --------------

void create_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,int& surface_mode,int& normal_method,pcl::PolygonMesh& triangles){

     /* ****Translated point cloud to origin**** */
     Eigen::Vector4f centroid;
     pcl::compute3DCentroid(*cloud, centroid);

     Eigen::Affine3f transform = Eigen::Affine3f::Identity();
     transform.translation() << -centroid[0], -centroid[1], -centroid[2];

     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTranslated(new pcl::PointCloud<pcl::PointXYZ>());
     pcl::transformPointCloud(*cloud, *cloudTranslated, transform);

     /* ****kdtree search and msl object**** */
     pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_for_points (new pcl::search::KdTree<pcl::PointXYZ>);
     kdtree_for_points->setInputCloud(cloudTranslated);
     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal> ());

     bool mls_mode = false;
     bool normal_mode = false;

     if(normal_method == 1){
        normal_mode = true;
     }else if(normal_method == 2){
        mls_mode = true;
     }else{
        std::cout << "Select:\n '1' for normal estimation \n '2' for mls normal estimation " << std::endl;
        std::exit(-1);
     }

     bool gp3_mode = false;
     bool poisson_mode = false;

     if(surface_mode == 1){
        poisson_mode = true;
     }else if(surface_mode == 2){
        gp3_mode = true;
     }else{
        std::cout << "Select: \n'1' for surface poisson method \n '2' for surface gp3 method " << std::endl;
        std::exit(-1);
     }

     if(mls_mode){

       std::cout << "Using mls method estimation...";

       pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>());
       pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;       

       //Set parameters
       mls.setComputeNormals(true);  
       mls.setInputCloud(cloudTranslated);
      // mls.setDilationIterations(10);
       //mls.setDilationVoxelSize(0.5);
       //mls.setSqrGaussParam(2.0);
       //mls.setUpsamplingRadius(5);
       //mls.setPolynomialOrder (2); 
       //mls.setPointDensity(30);
       mls.setSearchMethod(kdtree_for_points);
       mls.setSearchRadius(0.03);
       mls.process(*mls_points);  

       pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>());

       for(int i = 0; i < mls_points->points.size(); i++) {

              pcl::PointXYZ pt;
              pt.x = cloud->points[i].x; 
              pt.y = cloud->points[i].y; 
              pt.z = cloud->points[i].z; 

              temp->points.push_back(pt);            
        }
        

       pcl::concatenateFields (*temp, *mls_points, *cloud_with_normals);
       std::cout << "[OK]" << std::endl;

     }else if(normal_mode){

       std::cout << "Using normal method estimation...";

       pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
       pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

       n.setInputCloud(cloudTranslated);
       n.setSearchMethod(kdtree_for_points);
       n.setKSearch(20); //It was 20
       n.compute(*normals);//Normals are estimated using standard method.

       //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
       pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

       std::cout << "[OK]" << std::endl;
     
     }else{
        std::cout << "Select: '1' for normal method estimation \n '2' for mls normal estimation " << std::endl;
        std::exit(-1);
     }

     // Create search tree*
     pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree_for_normals (new pcl::search::KdTree<pcl::PointNormal>);
     kdtree_for_normals->setInputCloud(cloud_with_normals);

     std::cout << "Applying surface meshing...";

     if(gp3_mode){

       std::cout << "Using surface method: gp3 ..." << std::endl;

       int searchK = 100;
       int search_radius = 10;
       int setMU = 5;
       int maxiNearestNeighbors = 30;
       bool normalConsistency = false;

       pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

       gp3.setSearchRadius(search_radius);//It was 0.025
       gp3.setMu(setMU); //It was 2.5
       gp3.setMaximumNearestNeighbors(maxiNearestNeighbors);    //It was 100
       //gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees    //it was 4
       //gp3.setMinimumAngle(M_PI/18); // 10 degrees //It was 18
       //gp3.setMaximumAngle(M_PI/1.5); // 120 degrees        //it was 1.5
       gp3.setNormalConsistency(normalConsistency); //It was false
       gp3.setInputCloud(cloud_with_normals);
       gp3.setSearchMethod(kdtree_for_normals);
       gp3.reconstruct(triangles);

       //vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
       //pcl::PolygonMesh mesh_pcl;
       //pcl::VTKUtils::convertToVTK(triangles,polydata);
       //pcl::VTKUtils::convertToPCL(polydata,mesh_pcl);

       //pcl::io::savePolygonFilePLY("mesh.ply", mesh_pcl);

       std::cout << "[OK]" << std::endl;

     }else if(poisson_mode){
 
        std::cout << "Using surface method: poisson ..." << std::endl;

        int nThreads=8;
        int setKsearch=10;
        int depth=7;
        float pointWeight=2.0;
        float samplePNode=1.5;
        float scale=1.1;
        int isoDivide=8;
        bool confidence=true;
        bool outputPolygons=true;
        bool manifold=true;
        int solverDivide=8;

        pcl::Poisson<pcl::PointNormal> poisson;

        poisson.setDepth(depth);//9
        poisson.setInputCloud(cloud_with_normals);
        poisson.setPointWeight(pointWeight);//4
        poisson.setDegree(2);
        poisson.setSamplesPerNode(samplePNode);//1.5
        poisson.setScale(scale);//1.1
        poisson.setIsoDivide(isoDivide);//8
        poisson.setConfidence(confidence);
        poisson.setOutputPolygons(outputPolygons);
        poisson.setManifold(manifold);
        poisson.setSolverDivide(solverDivide);//8
        poisson.reconstruct(triangles);

        //pcl::PolygonMesh mesh2;
        //poisson.reconstruct(mesh2);
        //pcl::surface::SimplificationRemoveUnusedVertices rem;
        //rem.simplify(mesh2,triangles);

        std::cout << "[OK]" << std::endl;

     }else{
        std::cout << "Select: \n'1' for surface poisson method \n '2' for surface gp3 method " << std::endl;
        std::exit(-1);
     }

 }

 void vizualizeMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,pcl::PolygonMesh &mesh){

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("MAP3D MESH"));

  int PORT1 = 0;
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
  viewer->setBackgroundColor (0, 0, 0, PORT1);
  viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);

  int PORT2 = 0;
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
  viewer->setBackgroundColor (0, 0, 0, PORT2);
  viewer->addText("MESH", 10, 10, "PORT2", PORT2);
  viewer->addPolygonMesh(mesh,"mesh",PORT2);

  viewer->addCoordinateSystem();
  pcl::PointXYZ p1, p2, p3;

  p1.getArray3fMap() << 1, 0, 0;
  p2.getArray3fMap() << 0, 1, 0;
  p3.getArray3fMap() << 0,0.1,1;

  viewer->addText3D("x", p1, 0.2, 1, 0, 0, "x_");
  viewer->addText3D("y", p2, 0.2, 0, 1, 0, "y_");
  viewer->addText3D ("z", p3, 0.2, 0, 0, 1, "z_");

  // if(cloud->points[0].r <= 0 and cloud->points[0].g <= 0 and cloud->points[0].b<= 0 ){
  //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(cloud,255,255,0);
  //   viewer->removeAllPointClouds(0);
  //   viewer->addPointCloud(cloud,color_handler,"original_cloud",PORT1);
  // }else{
  //   viewer->addPointCloud(cloud,"original_cloud",PORT1);
  // }
  viewer->addPointCloud(cloud,"original_cloud",PORT1);
    
  viewer->initCameraParameters ();
  viewer->resetCamera();

  std::cout << "Press [q] to exit!" << std::endl;
  while (!viewer->wasStopped ()){
      viewer->spin();
  }
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

void 
setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mesh_visualiser");
  ros::NodeHandle n;

  // pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>());
  // pcl::PointCloud<pcl::PointXYZ>& point_cloud = *point_cloud_ptr;

  std::string filename = "/home/jesse/Code/src/ar_turtlebot_racing/src/mesh_visualiser/test_data/meshes/textured.obj";
  if (pcl::io::loadOBJFile(filename, *point_cloud) == -1)
  {
    std::cout << "Was not able to open file \""<<filename<<"\".\n";
    printUsage (argv[0]);
    return 0;
  }

  std::cout << point_cloud->size() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*point_cloud,*cloud_xyz);

  pcl::PolygonMesh cloud_mesh;

  int surface_mode = 1;
  int normal_mode = 2;
  create_mesh(cloud_xyz,surface_mode,normal_mode,cloud_mesh);

  vizualizeMesh(point_cloud,cloud_mesh);


}
// int  main (int argc, char** argv){

// ros::init(argc, argv, "mesh_visualiser");
// ros::NodeHandle n;

// image_transport::ImageTransport it(n);
// image_transport::Publisher pub_img = it.advertise("mesh_vis", 20);
// sensor_msgs::ImagePtr img_msg;



// // ros::spin();
//   // --------------------------------------
//   // -----Parse Command Line Arguments-----
//   // --------------------------------------
// //   if (pcl::console::find_argument (argc, argv, "-h") >= 0)
// //   {
// //     printUsage (argv[0]);
// //     return 0;
// //   }
// //   if (pcl::console::find_argument (argc, argv, "-l") >= 0)
// //   {
// //     live_update = true;
// //     std::cout << "Live update is on.\n";
// //   }

//     live_update = true;
//   if (pcl::console::parse (argc, argv, "-rx", angular_resolution_x) >= 0)
//     std::cout << "Setting angular resolution in x-direction to "<<angular_resolution_x<<"deg.\n";
//   if (pcl::console::parse (argc, argv, "-ry", angular_resolution_y) >= 0)
//     std::cout << "Setting angular resolution in y-direction to "<<angular_resolution_y<<"deg.\n";
//   int tmp_coordinate_frame;
//   if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
//   {
//     coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
//     std::cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
//   }
//   angular_resolution_x = pcl::deg2rad (angular_resolution_x);
//   angular_resolution_y = pcl::deg2rad (angular_resolution_y);
  
//   // ------------------------------------------------------------------
//   // -----Read pcd file or create example point cloud if not given-----
//   // ------------------------------------------------------------------
//   pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
//   pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;


// //https://roboticsknowledgebase.com/wiki/programming/eigen-library/
// //   Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
//   Eigen::Affine3f scene_sensor_pose(Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));
// // Eigen::Affine3f scene_sensor_pose(0, -1, 0,
// //                     1, 0, 0,
// //                     0, 0, 1);
//   std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
// //   if (!pcd_filename_indices.empty ())
// if (true)
//   {
//     // std::string filename = argv[pcd_filename_indices[0]];
//     std::string filename = "/home/jesse/Code/src/ar_turtlebot_racing/src/mesh_visualiser/test_data/meshes/textured.obj";
//     if (pcl::io::loadOBJFile(filename, point_cloud) == -1)
//     {
//       std::cout << "Was not able to open file \""<<filename<<"\".\n";
//       printUsage (argv[0]);
//       return 0;
//     }
//     // scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0] + 10,
//     //                                                          point_cloud.sensor_origin_[1],
//     //                                                          point_cloud.sensor_origin_[2] - 50)) *
//     //                     Eigen::Affine3f (point_cloud.sensor_orientation_);
//     scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0] + 10,
//                                                              point_cloud.sensor_origin_[1],
//                                                              point_cloud.sensor_origin_[2] - 50)) *
//                         Eigen::Affine3f (Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));
//   }
//   else
//   {
//     std::cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
//     for (float x=-0.5f; x<=0.5f; x+=0.01f)
//     {
//       for (float y=-0.5f; y<=0.5f; y+=0.01f)
//       {
//         PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
//         point_cloud.push_back (point);
//       }
//     }
//     point_cloud.width = point_cloud.size ();  point_cloud.height = 1;
//   }
  
//   // -----------------------------------------------
//   // -----Create RangeImage from the PointCloud-----
//   // -----------------------------------------------
//   float noise_level = 0.0;
//   float min_range = 0.0f;
//   int border_size = 1;
//   pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
//   pcl::RangeImage& range_image = *range_image_ptr;   
//   range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
//                                     pcl::deg2rad (360.0f), pcl::deg2rad (360.0f),
//                                     scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

//     // range_image.setImageOffsets(0,100);
  
//   // --------------------------------------------
//   // -----Open 3D viewer and add point cloud-----
//   // --------------------------------------------
//   pcl::visualization::PCLVisualizer viewer ("3D Viewer");
//   viewer.setBackgroundColor (1, 1, 1);
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
//   viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
//   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "range image");
//   viewer.addCoordinateSystem (1.0f, "global");
//   pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 0, 0, 0);
//   viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");

//   //can set camea parameters
//   viewer.initCameraParameters ();
//   setViewerPose(viewer, range_image.getTransformationToWorldSystem ());
  
//   // --------------------------
//   // -----Show range image-----
//   // --------------------------
//   pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
//   range_image_widget.showRangeImage (range_image);
  
//   //--------------------
//   // -----Main loop-----
//   //--------------------
//   cv::Mat image;
//   while (!viewer.wasStopped ())
//   {
//     range_image_widget.spinOnce ();
//     viewer.spinOnce ();
//     pcl_sleep (0.01);
    
//     if (live_update)
//     {
//       scene_sensor_pose = viewer.getViewerPose();
//       //can trt createFromPointCloudWithKnownSize()
//       range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
//                                         pcl::deg2rad (360.0f), pcl::deg2rad (360.0f),
//                                         scene_sensor_pose, pcl::RangeImage::CAMERA_FRAME, noise_level, min_range, border_size);

//         range_image.setUnseenToMaxRange();
//         range_image.getRangeImageWithSmoothedSurface(50, range_image);

//         ROS_INFO_STREAM("x " << range_image.width << " y " << range_image.height);
//         float* ranges = range_image.getRangesArray();
//         unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, range_image.width, range_image.height, true); 

//         // for(int i = 0; i< range_image.width; i++) {
//         //     for(int j = 0; j < range_image.height; j++) {
//         //         //There are three kinds of points. Valid points have a real range greater than zero.
//         //         // Unobserved points have x=y=z=NAN and range=-INFINITY. 
//         //         //Far range points have x=y=z=NAN and range=INFINITY.
//         //         // pcl::PointWithRange point = range_image.getPoint(i, j);
//         //         // if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
//         //         //     image.at<unsigned char>(i, j) = 255; 
//         //         // }
//         //         // else {
//         //         //     image.at<unsigned char>(i, j) = 0; 
//         //         // }
//         //         // ROS_INFO_STREAM(point);

//         //     }
//         // }
//         cv::Mat image(range_image.height,range_image.width,CV_8UC3,&rgb_image[0]);
//         img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image).toImageMsg();
//         pub_img.publish(img_msg);
//         ros::spinOnce();
//         ROS_INFO_STREAM(image.rows << " " << image.cols);
//         delete[] ranges;
//         // cv::imshow("frame", image);
//         // cv::waitKey(1);
//     //   range_image_widget.showRangeImage (range_image);
//     }
//   }
// }



