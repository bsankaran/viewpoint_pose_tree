
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
 * @class SimplePointCloudAnalyzer
 *
 * @author Bharath Sankaran
 *
 * @brief  A class that preprocesses the input pointcloud.
 *
 * This simple class is meant to preprocess the input pointcloud using the tabletop analyzer to extract the
 * tabletop clusters and return the clustered pointcloud for classification using the Viewpoint Posetree
 */
#pragma once

//! Standard
#include <iostream>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <boost/lexical_cast.hpp>

//! Pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

//! Ros
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

//! Custom Includes
//! quaternion operations
#include <misc.hpp>
//!IO utils
#include <io_utils.hpp>
//! map of the table
#include <metric_map.hpp>
//! Visualization of history data
#include <tabletop_analyzer.hpp>

//!Vision Classifier
#include <vision_module/vtree_user.hpp>


namespace point_cloud_analyzer{

class SimplePointCloudAnalyzer
{
   private:

    Eigen::Vector3d position_;
    Eigen::Vector4d orientation_;
    tf::StampedTransform optical2base_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cld_ptr_;			//! cloud continuously updated by the kinect
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr disp_cld_ptr_;


    //!Object Lists
    std::set<std::string> obj_set_;
    std::string object_list_path_;

    //! Viewpoint Files
    std::string tree_vps_path_;
    Eigen::MatrixXd tree_vps_;

    //! Vision Classifier
    vtree_user vtu_;

    //! Communication
    bool cloud_updated_;
    ros::Subscriber cloud_sub_;
    ros::Time goal_completion_time_;
    //! The listener receives tf transformations and buffers them for up to 10 seconds
    tf::TransformListener listener_;
    std::string fx_frm_;
    std::string opt_frm_;

    //!Tabletop Analyzer
    tabletop_analyzer TA_;


    //! Ros stuff
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

protected:

    virtual ros::NodeHandle& getNodeHandle() { return nh_; }
    virtual ros::NodeHandle& getPrivateNodeHandle() { return private_nh_; }

public:

    //! Constructor
    SimplePointCloudAnalyzer(ros::NodeHandle &nh);

    //! Destructor
    ~SimplePointCloudAnalyzer(){}

    //! Refresh Functions, Needs to be called before calling any other function
    void SimplePointCloudAnalyzerInit();
    void SimplePointCloudAnalyzerSpin();

    //! PointCloud callback
    void CloudCallBack(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud);

private:

    std::string ProcessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cld_ptr_in);

};

}
