//
// Created by cobot on 19-8-9.
//

#ifndef PROJECT_TESTHOLE_H
#define PROJECT_TESTHOLE_H

#define PCL_NO_PRECOMPILE
#include <iostream>
#include <string>
#include <cstdlib>

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/filters/impl/project_inliers.hpp>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_circle.hpp>
#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_cone.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_sphere.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp>
struct PointXYZRC {
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    int row;
    int col;

    int any1;
    int any2;
    int any3;
    friend std::ostream& operator << (std::ostream& os, const PointXYZRC& p);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

std::ostream& operator << (std::ostream& os, const PointXYZRC& p){
    os<<"x="<<p.x<<"  y="<<p.y<<"   z="<<p.z<<"   r="<<p.row<<" c="<<p.col
    <<" any1="<<p.any1<<"    any2="<<p.any2<<"    any3="<<p.any3;
}

PCL_INSTANTIATE_PCLBase(PointXYZRC)
PCL_INSTANTIATE_PointCloudGeometryHandlerXYZ(PointXYZRC)
PCL_INSTANTIATE_RadiusOutlierRemoval(PointXYZRC)
PCL_INSTANTIATE_KdTree(PointXYZRC)
PCL_INSTANTIATE_KdTreeFLANN(PointXYZRC)
PCL_INSTANTIATE_SampleConsensusModelParallelPlane(PointXYZRC)
//PCL_INSTANTIATE_SampleConsensusModelCylinder(PointXYZRC)
PCL_INSTANTIATE_SampleConsensusModelPerpendicularPlane(PointXYZRC)
PCL_INSTANTIATE_SampleConsensusModelPlane(PointXYZRC)
PCL_INSTANTIATE_SampleConsensusModelNormalPlane(PointXYZRC,pcl::Normal)
PCL_INSTANTIATE_SampleConsensusModelCone(PointXYZRC,pcl::Normal)
PCL_INSTANTIATE_SampleConsensusModelNormalParallelPlane(PointXYZRC,pcl::Normal)
PCL_INSTANTIATE_SampleConsensusModelNormalSphere(PointXYZRC,pcl::Normal)
PCL_INSTANTIATE_ProjectInliers(PointXYZRC)
PCL_INSTANTIATE_SampleConsensusModelSphere(PointXYZRC)
PCL_INSTANTIATE_SampleConsensusModelLine(PointXYZRC)
PCL_INSTANTIATE_SampleConsensusModelParallelLine(PointXYZRC)
PCL_INSTANTIATE_OrganizedNeighbor(PointXYZRC)
PCL_INSTANTIATE_SampleConsensusModelCircle2D(PointXYZRC)
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRC,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, row, row)
                                           (float, any1, any1)
                                           (float, any2, any2)
                                           (float, any3, any3)
                                           (float, col, col)
)


#endif //PROJECT_TESTHOLE_H
