#include <mutex>
#include <algorithm>
#include <map>
#include <utility>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

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

        setupUniqueIDs();
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
    std::map<int, std::pair<double,geometry_msgs::Pose> > _apriltag_decay;
    double _tag_decay_time;

    //private definition of unique colours for each of the 12 identified apriltags
    // everyone else will be white

    void peopleClusterCallback(const hdl_people_tracking::ClusterPointCloudConstPtr &msg)
    {
        if(_cluster_pub.getNumSubscribers() == 0)
        {
            ROS_DEBUG("No subscribers. Not publishing.");
            return;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr accum(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // identify cluster for unique IDs

        // colour PCL point cloud

        // accumulate all clusters in PCL (with diff colours)

        //convert to sensor_msgs and publish
        sensor_msgs::PointCloud2 outmsg;
        pcl::toROSMsg(*accum,outmsg);

        _cluster_pub.publish(outmsg);
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
            // geometry_msgs::Pose tpose = geometry_msgs::Pose(it->pose.pose.pose);
            // _apriltag_decay.emplace(it->id,std::make_pair<double&,geometry_msgs::Pose&>(start,tpose));
        }
    }

    void setupUniqueIDs()
    {
        _unique_id_colour[ 22] = std::make_pair<std::string,int32_t>("PERSON_01",rgb(255,0,0));
        _unique_id_colour[ 77] = std::make_pair<std::string,int32_t>("PERSON_02",rgb(255,0,0));;
        _unique_id_colour[125] = std::make_pair<std::string,int32_t>("PERSON_03",rgb(255,0,0));;
        _unique_id_colour[191] = std::make_pair<std::string,int32_t>("PERSON_04",rgb(255,0,0));;
        _unique_id_colour[209] = std::make_pair<std::string,int32_t>("PERSON_05",rgb(255,0,0));;
        _unique_id_colour[255] = std::make_pair<std::string,int32_t>("PERSON_06",rgb(255,0,0));;
        _unique_id_colour[357] = std::make_pair<std::string,int32_t>("PERSON_07",rgb(255,0,0));;
        _unique_id_colour[372] = std::make_pair<std::string,int32_t>("PERSON_08",rgb(255,0,0));;
        _unique_id_colour[442] = std::make_pair<std::string,int32_t>("PERSON_09",rgb(255,0,0));;
        _unique_id_colour[480] = std::make_pair<std::string,int32_t>("PERSON_10",rgb(255,0,0));;
        _unique_id_colour[507] = std::make_pair<std::string,int32_t>("PERSON_11",rgb(255,0,0));;
        _unique_id_colour[573] = std::make_pair<std::string,int32_t>("PERSON_12",rgb(255,0,0));;
        // _unique_id_colour[ 22] = "PERSON_01";
        // _unique_id_colour[ 77] = "PERSON_02";
        // _unique_id_colour[125] = "PERSON_03";
        // _unique_id_colour[191] = "PERSON_04";
        // _unique_id_colour[209] = "PERSON_05";
        // _unique_id_colour[255] = "PERSON_06";
        // _unique_id_colour[357] = "PERSON_07";
        // _unique_id_colour[372] = "PERSON_08";
        // _unique_id_colour[442] = "PERSON_09";
        // _unique_id_colour[480] = "PERSON_10";
        // _unique_id_colour[507] = "PERSON_11";
        // _unique_id_colour[573] = "PERSON_12";
    }

    int32_t rgb(int r, int g, int b)
    {
        return (static_cast<uint32_t>(r) << 16 | 
            static_cast<uint32_t>(g) << 8 | 
            static_cast<uint32_t>(b) );
    }
};

}
PLUGINLIB_EXPORT_CLASS(hdl_people_detection::HdlPeopleClusterColourNodelet, nodelet::Nodelet)
