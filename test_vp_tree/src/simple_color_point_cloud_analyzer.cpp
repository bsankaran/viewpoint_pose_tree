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
  * Details for all functions declared in the SimplePointCloudAnalyzer class
  */
#include "test_vp_tree/simple_color_point_cloud_analyzer.hpp"
namespace point_cloud_analyzer{

SimplePointCloudAnalyzer::SimplePointCloudAnalyzer(ros::NodeHandle &nh):
    xyz_cld_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>),
    disp_cld_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>),
    cloud_updated_(false), goal_completion_time_(ros::Time::now()),
    listener_(ros::Duration(180.0)), TA_(nh),
    nh_(nh), private_nh_("~"),vtu_("/load","","")
{
    srand((unsigned)time(NULL));
}

void SimplePointCloudAnalyzer::CloudCallBack(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){

    // We receive continuous clouds from the kinect here but will not use them!
    if((!cloud_updated_) && (goal_completion_time_ + ros::Duration(2.0) < cloud->header.stamp))
    {
       try
        {
            // update the pose
            listener_.waitForTransform( opt_frm_, fx_frm_, cloud->header.stamp, ros::Duration(5.0));
            listener_.lookupTransform( fx_frm_, opt_frm_, cloud->header.stamp, optical2base_);

            // ASSUMES optical2map and sensor2map have the same translation!!!
            tf::Vector3 position( optical2base_.getOrigin() );
            position_.x() = position.x();
            position_.y() = position.y();
            position_.z() = position.z();

            // We dont need to look up kinect2map
            tf::Quaternion opt_quat_snsr(0.5, -0.5, 0.5, 0.5);
            tf::Quaternion orientation( optical2base_ * opt_quat_snsr );
            orientation_.x() = orientation.x();
            orientation_.y() = orientation.y();
            orientation_.z() = orientation.z();
            orientation_.w() = orientation.w();

            ROS_INFO_STREAM("position = " << position_.transpose() );
            ROS_INFO_STREAM("orientation = " << orientation_.transpose() );

            // update the cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*cloud, *cld_tmp);

            // cut far away points
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setFilterFieldName ("z"); // check if this cut is in the right direction before rotating the cloud!
            pass.setFilterLimits (0.1, 2.0);
            pass.setInputCloud(cld_tmp);
            pass.filter(*xyz_cld_ptr_);

            cloud_updated_ = true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }
    }
}

void SimplePointCloudAnalyzer::SimplePointCloudAnalyzerInit(){

    std::string vision_module_dir( ros::package::getPath("vision_module") );
    std::string perception_executive_dir(ros::package::getPath("perception_executive"));
    std::string database_dir;
    std::string clouds_dir;

    private_nh_.param( "tree_vps_path", tree_vps_path_, vision_module_dir + "/data/tree_vps.txt" );
    private_nh_.param( "database_dir", database_dir, vision_module_dir + "/data/color" );
    private_nh_.param( "clouds_dir", clouds_dir, vision_module_dir + "/../database/color_cloud_data" );
    private_nh_.param( "object_list_path", object_list_path_, perception_executive_dir + "/data/obj_color_list.txt" );

    //Starting ViewPoint Pose tree
    vtu_.set_database_dir(database_dir);
    vtu_.set_clouds_dir(clouds_dir);
    if(vtu_.start())
        throw std::runtime_error("Loading Vocabulary tree database failed ...\n");

    std::string cld_topic_name;
    private_nh_.param( "cld_topic_name", cld_topic_name, std::string("/XTION/depth/points") );
    private_nh_.param( "fx_frm", fx_frm_, std::string("/BASE") );
    //private_nh_.param( "snsr_frm", snsr_frm_, std::string("/XTION_RGB") );
    private_nh_.param( "opt_frm", opt_frm_, std::string("/XTION") ); //
    cloud_sub_ = nh_.subscribe( cld_topic_name, 1, &SimplePointCloudAnalyzer::CloudCallBack, this);

}

void SimplePointCloudAnalyzer::SimplePointCloudAnalyzerSpin(){

    // Timing
    clock_t tic;
    double toc;

    // 0. Get pose and cloud
    // Wait until time stamps are bigger than goal completion time
    tic = clock();
    while( !cloud_updated_ )
        ros::spinOnce();
    toc = (clock()-tic)*1000/CLOCKS_PER_SEC;
    std::cout << "It took " << toc << " msec to update the pose and cloud!" << std::endl;

    // convert cloud to /BASE frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cld_ptr( new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_ros::transformPointCloud( *xyz_cld_ptr_, *base_cld_ptr, optical2base_);
    base_cld_ptr->header.frame_id = fx_frm_;
    base_cld_ptr->header.stamp = xyz_cld_ptr_->header.stamp;

    // Segment the table out and cluster the objects
    tic = clock();
    std::vector<tabletop_color_analyzer::TableInfo> ti_vec;
    ti_vec = TA_.detect_tabletop_objects( base_cld_ptr );
    bool table_found = (ti_vec.size() > 0);
    toc = (clock()-tic)*1000/CLOCKS_PER_SEC;
    std::cout << "It took " << toc << " msec to segment table!" << std::endl;
    std::cout << ti_vec.size() << " tables were detected!" << std::endl;

    int tab_idx = -1; // The detected table with maximum number of clusters!
    if(table_found){
        int max_obj_num = 0;
        for( int k = 0; k < ti_vec.size(); ++k)
            if( static_cast<int>(ti_vec[k].obj_centroids->size()) > max_obj_num)
            {
                tab_idx = k;
                max_obj_num = ti_vec[k].obj_centroids->size();
            }

        if( tab_idx < 0 )
            table_found = false;
    }
    else{

        std::cout<<"Not tables found in this iteration, returning control"<<std::endl;
        return;
    }


    // Display data
    std::cout << "The table has "
              << ti_vec[tab_idx].obj_centroids->size()
              << " objects on it!" << std::endl;

    //TODO: Dummy process state, Here we simply classify all Objects in the list
    int obj_count = 0;

    for(int i = 0; i < ti_vec[tab_idx].obj_centroids->size(); i++, obj_count++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_view_ptr( new pcl::PointCloud< pcl::PointXYZRGB >); //ti_vec[tab_idx].obj_surf[i]
        *latest_view_ptr = *ti_vec[tab_idx].obj_surf[i];
        std::string classified_output = ProcessPointCloud(latest_view_ptr);
        std::cout<<"Classified Output for object "<<obj_count<<" is "<<classified_output<<std::endl;
        std::string filename("cld_"+ boost::lexical_cast<std::string>(goal_completion_time_.toSec()) + "_object_"+ boost::lexical_cast<std::string>(obj_count)+".pcd");
        pcl::io::savePCDFileASCII(filename,*latest_view_ptr);
    }

    goal_completion_time_ = ros::Time::now();
    cloud_updated_ = false;
}

std::string SimplePointCloudAnalyzer::ProcessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cld_ptr_in){

    // Get the score from the vocabulary tree
    std::cout<<"Searching to get top match"<<std::endl;
    Eigen::MatrixXf temp = cld_ptr_in->getMatrixXfMap();
    std::cout<<"Silly check to see if this works, rows: "<<temp.rows() <<" cols: "<<temp.cols() <<std::endl;
    std::cout<<"Check of points"<<std::endl;
    std::cout<<temp(0,1)<<" "<<temp(1,1)<<" "<<temp(2,1)<<" "<<temp(3,1)
            <<" "<<temp(4,1)<<" "<<temp(5,1)<<" "<<temp(6,1)<<" "<<temp(7,1)<<" "<<std::endl;
    //std::pair<float, std::string> vp_score = vtu_.top_match(cld_ptr_in->getMatrixXfMap());
    std::pair<float, std::string> vp_score = vtu_.top_match(cld_ptr_in);
    ROS_WARN("The top match is %s!",vp_score.second.c_str());

    // convert the match_name to a number representing the obs
    size_t pcd_pos = (vp_score.second).find_last_of(".pcd");
    size_t underscore_pos = (vp_score.second).find_last_of("_");
    size_t slash_pos = (vp_score.second).find_last_of("/");
    if( (pcd_pos == std::string::npos) || (underscore_pos == std::string::npos)
            || (slash_pos == std::string::npos) )
        throw std::runtime_error("Unknown match name...\n");

    std::string tmp( (vp_score.second).substr(slash_pos+1, underscore_pos-slash_pos-1 ) );

    return tmp;

}

}

int SimplePointCloudAnalyzerMain(int argc, char **argv)
{
    ros::init(argc, argv,"simple_color_point_cloud_analyzer");
    ros::NodeHandle nh;
    point_cloud_analyzer::SimplePointCloudAnalyzer pc(nh);
    pc.SimplePointCloudAnalyzerInit();

    while (ros::ok())
    {
        pc.SimplePointCloudAnalyzerSpin();
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char **argv)
{

    return SimplePointCloudAnalyzerMain(argc, argv);
}

