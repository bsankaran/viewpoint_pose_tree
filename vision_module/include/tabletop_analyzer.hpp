/*****************************************************************************************
 *
 *  Copyright (c) 2014, Bharath Sankaran,
 *  Computational Learning and Motor Control Laboratory, University of Southern California
 *  (bsankara@usc.edu)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bharath Sankaran nor CLMC
 *     may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************************/
/**
 * @class tabletop_analyzer
 *
 * @author Bharath Sankaran
 *
 * @brief  A class that preprocesses the input pointcloud.
 *
 * This simple class is meant to preprocess the input pointcloud by extracting tabletop clusters from the dominant
 * table plane
 */
#pragma once

// Ros
#include <ros/ros.h>

// Pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/ModelCoefficients.h>

class tabletop_analyzer
{
public:
    typedef pcl::PointXYZ PointT;
    typedef pcl::search::KdTree<PointT> SearchMethod;

    struct TableInfo
    {
        pcl::ModelCoefficients::Ptr tab_coefs;
        pcl::PointCloud<PointT>::Ptr table_hull_ptr;
        Eigen::Vector3d table_centroid;

        pcl::PointCloud<PointT>::Ptr obj_cld_ptr;
        std::vector<pcl::PointCloud<PointT>::Ptr> obj_surf;	// return downsampled cloud surfaces directly
        //std::vector<pcl::PointIndices> obj_idx;

        pcl::PointCloud<PointT>::Ptr obj_maxp;		// return bounding boxes too
        pcl::PointCloud<PointT>::Ptr obj_minp;
        pcl::PointCloud<PointT>::Ptr obj_centroids;
        pcl::PointCloud<PointT>::Ptr obj_proj_centroids;

        TableInfo() :
            table_hull_ptr( new pcl::PointCloud<PointT>),
            obj_cld_ptr( new pcl::PointCloud<PointT>),
            obj_maxp( new pcl::PointCloud<PointT>),
            obj_minp( new pcl::PointCloud<PointT>),
            obj_centroids( new pcl::PointCloud<PointT>),
            obj_proj_centroids( new pcl::PointCloud<PointT>)
        {}
    };

private:
    //! Min number of inliers for reliable plane detection
    int table_inlier_threshold_;
    //! Size of downsampling grid before performing plane detection
    double table_detection_voxel_size_;
    //! Size of downsampling grid before performing clustering
    double object_detection_voxel_size_;
    //! Filtering of original point cloud along the z axis
    double z_filter_min_, z_filter_max_;
    //! Filtering of point cloud in table frame after table detection
    double table_z_filter_min_, table_z_filter_max_;
    //! Min distance between two clusters
    double object_cluster_distance_;
    double table_cluster_distance_;
    //! Min and max number of points for an object cluster
    int max_object_cluster_size_;
    int min_object_cluster_size_;
    //! Min number of points for a table cluster
    int min_table_cluster_size_;

    // Ros stuff
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

protected:
    virtual ros::NodeHandle& getNodeHandle() { return nh_; }
    virtual ros::NodeHandle& getPrivateNodeHandle() { return private_nh_; }

public:
    tabletop_analyzer();
    tabletop_analyzer( ros::NodeHandle & nh );
    virtual ~tabletop_analyzer() {}

    std::vector<TableInfo> detect_tabletop_objects( const pcl::PointCloud<PointT>::ConstPtr & cld_ptr );

    TableInfo process_table( pcl::ModelCoefficients::Ptr & table_coeffs,
                             const pcl::PointCloud<PointT>::ConstPtr & cld_ptr,
                             const pcl::PointCloud<PointT>::ConstPtr & tab_ptr );

    void obj_cld2clusters( pcl::ModelCoefficients::Ptr & table_coeffs,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_cld,
                           std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & obj_surf,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_maxp,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_minp,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_centroids,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_proj_centroids );

    void obj_cld2clusters( pcl::ModelCoefficients::Ptr & table_coeffs,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr & obj_cld,
                           std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & obj_surf,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_maxp,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_minp,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_centroids,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr & obj_proj_centroids );
private:
    // Area centroid for a convex hull in 2D, any other input will not produce expected result
    template <typename T>
    Eigen::Vector3d getXYAreaCentroid( boost::shared_ptr< pcl::PointCloud<T> > &cld_2d_ptr);
};


template <typename T>
Eigen::Vector3d
tabletop_analyzer::getXYAreaCentroid( boost::shared_ptr< pcl::PointCloud<T> > & cld_2d_ptr)
{
    if( cld_2d_ptr->size() < 3 )
    {
        return Eigen::Vector3d();
    }

    double a_sum = 0, xa_sum = 0, ya_sum = 0;
    double z_sum = cld_2d_ptr->points[0].z;

    for(unsigned int i = 1; i < cld_2d_ptr->size()-1; ++i)
    {
        z_sum += cld_2d_ptr->points[i].z;
        double triangle_x_centroid = (cld_2d_ptr->points[0].x +
                                      cld_2d_ptr->points[i].x +
                                      cld_2d_ptr->points[i+1].x)/3;

        double triangle_y_centroid = (cld_2d_ptr->points[0].y +
                                      cld_2d_ptr->points[i].y +
                                      cld_2d_ptr->points[i+1].y)/3;

        double triangle_area = fabs(
                    (cld_2d_ptr->points[0].x*(cld_2d_ptr->points[i].y-cld_2d_ptr->points[i+1].y) +
                     cld_2d_ptr->points[i].x*(cld_2d_ptr->points[i+1].y-cld_2d_ptr->points[0].y) +
                     cld_2d_ptr->points[i+1].x*(cld_2d_ptr->points[0].y-cld_2d_ptr->points[i].y)) / 2);

        a_sum += triangle_area;
        xa_sum += triangle_x_centroid*triangle_area;
        ya_sum += triangle_y_centroid*triangle_area;
    }

    z_sum += cld_2d_ptr->points.back().z;

    if(a_sum < 1e-8)
    {
        return Eigen::Vector3d();
    }

    return Eigen::Vector3d(xa_sum/a_sum, ya_sum/a_sum, z_sum/cld_2d_ptr->points.size());
}




