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

namespace fs = std::filesystem;
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;

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

  std::string directory_path = "../dataset/frames";

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../dataset/map.pcd", *map) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file map.pcd \n");
    return (-1);
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../dataset/frames/frame_0.pcd", *frame) == -1) // load the file
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


// Executing the transformation
  pcl::transformPointCloud (*frame, *cloud_icp, transformation_matrix);
  *cloud_bu = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

  int iterations = 1; 
  pcl::console::TicToc time;
  time.tic ();
  // pcl::PointCloud<pcl::PointXYZ>::PointCloudT
  /*pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations (iterations);
  icp.setInputSource (frame);
  icp.setInputTarget (map);
  icp.align (*frame);
  icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
*/
  /*
  for (const auto & file: fs::directory_iterator(directory_path)) {
      std::cout << file.path() << std::endl;
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (file.path(), *frame) == -1) // load the file
        {
          PCL_ERROR ("Couldn't read file frame_0.pcd \n");
          return (-1);
        }

      std::cout << "Loaded "
            << frame->width * frame->height
            << " data points from frame.pcd with the following fields: "
            << std::endl;  
    }*/
  

  
  /*for (const auto& point_map: *map)
    std::cout << "    " << point_map.x
              << " "    << point_map.y
              << " "    << point_map.z << std::endl;*/

  return (0);
}
