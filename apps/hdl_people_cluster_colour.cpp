#include <mutex>
#include <algorithm>
#include <map>
#include <utility>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <hdl_people_tracking/ClusterPointCloud.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace hdl_people_detection
{

class HdlPeopleClusterColourNodelet : public nodelet::Nodelet
{
public:
    HdlPeopleClusterColourNodelet(){};
    virtual ~HdlPeopleClusterColourNodelet(){};
    void onInit() override 
    {
        _nh = getNodeHandle();
        _mt_nh = getMTNodeHandle();
        _pnh = getPrivateNodeHandle();
        _cluster_sub_name = _pnh.param<std::string>("cluster_topic_name","cluster_vec_points");
        _apriltag_sub_name = _pnh.param<std::string>("apriltag_topic_name","tag_detections");
        _cluster_pub_name = _pnh.param<std::string>("colour_cluster_topic_name","verified_clusters");
        _tag_decay_time = _pnh.param<double>("tag_decay_time",2.0);
        _centroid_distance_tolerance = _pnh.param<double>("centroid_distance_tolerance",0.25); // in meters

        setupUniqueIDs();

        _cluster_pub = _pnh.advertise<sensor_msgs::PointCloud2>(_cluster_pub_name.c_str(),1);
        _timer_loop = _pnh.createTimer(ros::Duration(_tag_decay_time),&HdlPeopleClusterColourNodelet::timerCallback,this);
        _cluster_sub = _mt_nh.subscribe(_cluster_sub_name.c_str(),1,&HdlPeopleClusterColourNodelet::peopleClusterCallback,this);
        _apriltag_sub = _mt_nh.subscribe(_apriltag_sub_name.c_str(),1,&HdlPeopleClusterColourNodelet::apriltagCallback,this);
    }
private:
    ros::NodeHandle _nh;
    ros::NodeHandle _mt_nh;
    ros::NodeHandle _pnh;

    ros::Timer _timer_loop;

    ros::Subscriber _cluster_sub;
    ros::Publisher _cluster_pub;
    ros::Subscriber _apriltag_sub;

    std::string _cluster_sub_name;
    std::string _cluster_pub_name;
    std::string _apriltag_sub_name;

    std::mutex _data_mtx;
    std::map<int,std::pair<std::string, int32_t> > _unique_id_colour;
    const int32_t _whitecolour = rgb(255,255,255);
    std::map<int, std::pair<double,geometry_msgs::Pose> > _apriltag_decay;
    double _tag_decay_time;
    double _centroid_distance_tolerance;

    void peopleClusterCallback(const hdl_people_tracking::ClusterPointCloudConstPtr &msg)
    {
        if(_cluster_pub.getNumSubscribers() == 0)
        {
            ROS_DEBUG("No subscribers. Not publishing.");
            return;
        }
        ros::Time starttime = ros::Time::now();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr accum(new pcl::PointCloud<pcl::PointXYZRGB>);
        //scope for lock guard
        {
            std::lock_guard<std::mutex> lk(_data_mtx);
            // iterate clusters in msg
            for (std::vector<sensor_msgs::PointCloud2>::const_iterator it=msg->clusters.begin(); it!= msg->clusters.end(); it++)
            {
                // convert to PCL. Check centroid
                pcl::PointCloud<pcl::PointXYZRGB> temp;
                pcl::fromROSMsg(*it,temp);
                pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
                for(auto &p: temp.points)
                {
                    centroid.add(p);
                }
                pcl::PointXYZRGB c;
                centroid.get(c);
                // Find closest match with ar marker pose (tolerance)
                double maxdist = 500.0;
                int closest_id = 0;
                for(auto &p: _apriltag_decay)
                {
                    double td = calcDist(c,p.second.second);
                    if(maxdist > td)
                    {
                        maxdist = td;
                        closest_id = p.first;
                    }
                }
                int32_t col = _whitecolour;
                if(maxdist < _centroid_distance_tolerance)
                {
                    // find colour palette of unique ID
                    std::map<int,std::pair<std::string,int32_t> >::iterator un_it = _unique_id_colour.find(closest_id);
                    if(un_it != _unique_id_colour.end())
                    {
                        col = un_it->second.second;
                    }
                }
                for(auto &p: temp.points)
                {
                    // colour PCL pointcloud
                    p.rgb = col;
                }
                // add to accum pointcloud
                *accum+=temp;
            }
        } // end scope for lock guard
        
        //convert to sensor_msgs and publish
        sensor_msgs::PointCloud2 outmsg;
        pcl::toROSMsg(*accum,outmsg);
        outmsg.header.frame_id = msg->header.frame_id;
        outmsg.header.stamp = ros::Time::now();
        _cluster_pub.publish(outmsg);
        ROS_INFO("Time taken for callback: %f",(ros::Time::now()-starttime).toSec());
    }

    void timerCallback(const ros::TimerEvent&)
    {
        // check for decay of recognised tags for cluster detection vis
        ros::Time t = ros::Time::now();
        double start = ((double) t.sec) + ((double) t.nsec/1e9);
        std::lock_guard<std::mutex> lk(_data_mtx);
        std::map<int, std::pair<double,geometry_msgs::Pose> >::iterator next;
        for(std::map<int, std::pair<double,geometry_msgs::Pose> >::iterator it=_apriltag_decay.begin(), next = it;
            it!=_apriltag_decay.end();
            it = next )
        {
            ++next;
            if( (start - it->second.first ) >= _tag_decay_time )
            {
                _apriltag_decay.erase(it);
            }
        }
    }

    void apriltagCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg)
    {
        ros::Time t = ros::Time::now();
        double start = ((double) t.sec) + ((double) t.nsec/1e9);
        // update apriltag location and/or IDs
        std::lock_guard<std::mutex> lk(_data_mtx);
        for(std::vector<apriltag_ros::AprilTagDetection>::const_iterator it=msg->detections.begin();
            it!=msg->detections.end();
            it++)
        {
            // add id and time with its pose
            _apriltag_decay.emplace(it->id[0],std::make_pair(start,it->pose.pose.pose));
        }
    }

    void setupUniqueIDs()
    {
        _unique_id_colour[ 22] = std::make_pair<std::string,int32_t>("PERSON_01",rgb(255,  0,  0));
        _unique_id_colour[ 77] = std::make_pair<std::string,int32_t>("PERSON_02",rgb(255,128,  0));;
        _unique_id_colour[125] = std::make_pair<std::string,int32_t>("PERSON_03",rgb(255,255,  0));;
        _unique_id_colour[191] = std::make_pair<std::string,int32_t>("PERSON_04",rgb(128,255,  0));;
        _unique_id_colour[209] = std::make_pair<std::string,int32_t>("PERSON_05",rgb(  0,255,  0));;
        _unique_id_colour[255] = std::make_pair<std::string,int32_t>("PERSON_06",rgb(  0,255,128));;
        _unique_id_colour[357] = std::make_pair<std::string,int32_t>("PERSON_07",rgb(  0,255,255));;
        _unique_id_colour[372] = std::make_pair<std::string,int32_t>("PERSON_08",rgb(  0,128,255));;
        _unique_id_colour[442] = std::make_pair<std::string,int32_t>("PERSON_09",rgb(  0,  0,255));;
        _unique_id_colour[480] = std::make_pair<std::string,int32_t>("PERSON_10",rgb(128,  0,255));;
        _unique_id_colour[507] = std::make_pair<std::string,int32_t>("PERSON_11",rgb(255,  0,255));;
        _unique_id_colour[573] = std::make_pair<std::string,int32_t>("PERSON_12",rgb(255,  0,128));;
    }

    int32_t rgb(int r, int g, int b)
    {
        return (static_cast<uint32_t>(r) << 16 | 
            static_cast<uint32_t>(g) << 8 | 
            static_cast<uint32_t>(b) );
    }

    double calcDist(pcl::PointXYZRGB centroid,geometry_msgs::Pose pose)
    {
        return std::sqrt(std::pow(centroid.x-pose.position.x,2.0)+std::pow(centroid.y-pose.position.y,2.0)+std::pow(centroid.z-pose.position.z,2.0));
    }
};

}
PLUGINLIB_EXPORT_CLASS(hdl_people_detection::HdlPeopleClusterColourNodelet, nodelet::Nodelet)
