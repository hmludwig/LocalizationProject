#include <iostream> 
#include <filesystem>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace fs = std::filesystem;

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("transformation_matrix:\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}



int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr frame (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bu (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr frame_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  // file pointer
  std::fstream fout;
  
  // opens an existing csv file or creates a new file.
  std::ofstream out("out.csv");

  out << "File" << 'x' << ',' << 'y' << ',' << 'z' << ',' << "roll" << ',' << "pitch" << ',' << "yaw" << '\n';

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../dataset/map.pcd", *map) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file map.pcd \n");
    return (-1);
  }

  std::string directory_path = "/home/harald/LocalizationProject/dataset/frames";
  for (const auto & file: fs::directory_iterator(directory_path)) {
    std::cout << file.path() << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file.path(), *frame) == -1) // load the file
      {
        PCL_ERROR ("Couldn't read file\n");
        return (-1);
      }

    std::cout << "Loaded "
          << frame->width * frame->height
          << " data points from frame.pcd with the following fields: "
          << std::endl;  

    std::cout << "Loaded "
            << map->width * map->height
            << " data points from map.pcd with the following fields: "
            << std::endl;

    std::cout << "Loaded "
              << frame->width * frame->height
              << " data points from frame.pcd with the following fields: "
              << std::endl;

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    std::cerr << "PointCloud before filtering: " << frame->width * frame->height 
       << " data points (" << pcl::getFieldsList (*frame) << ")." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (frame);
    sor.setLeafSize (0.2f, 0.2f, 0.2f);
    sor.filter (*frame_filtered);

    std::cerr << "PointCloud after filtering: " << frame_filtered->width * frame_filtered->height 
          << " data points (" << pcl::getFieldsList (*frame_filtered) << ")." << std::endl;


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(frame_filtered);
    icp.setInputTarget(map);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    transformation_matrix = icp.getFinalTransformation().cast<double>();
    print4x4Matrix (transformation_matrix);

    float roll,pitch,yaw;
    yaw = atan2(transformation_matrix(1,0),transformation_matrix(0,0));
    pitch = atan2(-transformation_matrix(2,0), std::pow(transformation_matrix(2,1) * transformation_matrix(2,1) + transformation_matrix(2,2) * transformation_matrix(2,2), 0.5));
    roll = atan2(transformation_matrix(2,1),transformation_matrix(2,2));

    // pcl::getEulerAngles(transformation, roll, pitch, yaw).reverse();
    std::cout << "roll : " << roll << ", pitch : " << pitch << ", yaw : " << yaw << std::endl;
    out << file.path() << ',' << transformation_matrix(0, 3) << ',' << transformation_matrix(1,3) << ',' << transformation_matrix(2,3) << ',' << roll << ',' << pitch << ',' << yaw << '\n'; 

  }
  out.close();
  return 0;
}
