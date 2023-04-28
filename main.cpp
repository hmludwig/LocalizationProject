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
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
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

  std::string directory_path = "../dataset/frames";

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../dataset/map.pcd", *map) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file map.pcd \n");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../dataset/frames/frame_20.pcd", *frame) == -1) // load the file
        {
          PCL_ERROR ("Couldn't read file frame_0.pcd \n");
          return (-1);
        }

  std::cout << "Loaded "
            << map->width * map->height
            << " data points from map.pcd with the following fields: "
            << std::endl;

  std::cout << "Loaded "
            << frame->width * frame->height
            << " data points from frame.pcd with the following fields: "
            << std::endl;  
  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  double theta = M_PI / 8;  // The angle of rotation in radians
  transformation_matrix (0, 0) = std::cos (theta);
  transformation_matrix (0, 1) = -sin (theta);
  transformation_matrix (1, 0) = sin (theta);
  transformation_matrix (1, 1) = std::cos (theta);

  // A translation on Z axis (0.4 meters)
  transformation_matrix (2, 3) = 0.4;

  // Display in terminal the transformation matrix
  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
  print4x4Matrix (transformation_matrix);


  std::cerr << "PointCloud before filtering: " << frame->width * frame->height 
       << " data points (" << pcl::getFieldsList (*frame) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (frame);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*frame_filtered);

  std::cerr << "PointCloud after filtering: " << frame_filtered->width * frame_filtered->height 
       << " data points (" << pcl::getFieldsList (*frame_filtered) << ")." << std::endl;

  int iterations = 1; 

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(frame);
  icp.setInputTarget(map);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  transformation_matrix = icp.getFinalTransformation().cast<double>();
  print4x4Matrix (transformation_matrix);

  float roll,pitch,yaw;
  std::cout << "\nThe input Euler angles are : " << std::endl;
  std::cout << "roll : " << roll << " ,pitch : " << pitch << " ,yaw : " << yaw << std::endl;

  Eigen::Affine3f transformation;

  transformation (0,0), transformation (0,1), transformation (0,2) = transformation_matrix (0, 0), transformation_matrix (0, 1), transformation_matrix (0, 2);
  transformation (1,0), transformation (1,1), transformation (1,2) = transformation_matrix (1, 0), transformation_matrix (1, 1), transformation_matrix (1, 2);
  transformation (2,0), transformation (2,1), transformation (2,2) = transformation_matrix (2, 0), transformation_matrix (2, 1), transformation_matrix (2, 2);

  pcl::getEulerAngles(transformation,roll,pitch,yaw);
  std::cout << "\nThe output Euler angles (using getEulerAngles function) are : " << std::endl;
  std::cout << "roll : " << roll << " ,pitch : " << pitch << " ,yaw : " << yaw << std::endl;

  return 0;
}
