#include <ros/ros.h>
#include <ros/serialization.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/Image.h>

#include "intrinsic_tactile_toolbox/TactileInfo.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "collect/AllData.h"
#include "phase_space/AllMarkers.h"

using namespace intrinsic_tactile_toolbox;
namespace collect {

class dataPublisher
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
      
     //! Subscriber for markers readings
    typedef phase_space::AllMarkers::ConstPtr MarkerPtr;
    typedef TactileInfo::ConstPtr thimblePtr;
    typedef sensor_msgs::PointCloud2::ConstPtr scenePtr;
    typedef geometry_msgs::TransformStamped::ConstPtr transPtr;

    typedef message_filters::sync_policies::ApproximateTime<phase_space::AllMarkers ,sensor_msgs::PointCloud2,geometry_msgs::TransformStamped,geometry_msgs::TransformStamped> MySyncPolicy;
//    typedef message_filters::sync_policies::ApproximateTime<phase_space::AllMarkers,geometry_msgs::TransformStamped,geometry_msgs::TransformStamped> MySyncPolicy;
    message_filters::Subscriber<phase_space::AllMarkers> sub_markers_readings;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_camera_readings;
    message_filters::Subscriber<geometry_msgs::TransformStamped> sub_starlink_readings;
    message_filters::Subscriber<geometry_msgs::TransformStamped> sub_wristlink_readings;
 
    /*message_filters::Subscriber<TactileInfo> sub_thimble_readings1;
    message_filters::Subscriber<TactileInfo> sub_thimble_readings2;
    message_filters::Subscriber<TactileInfo> sub_thimble_readings3;
    message_filters::Subscriber<TactileInfo> sub_thimble_readings4;
    message_filters::Subscriber<TactileInfo> sub_thimble_readings5;*/
    
    message_filters::Synchronizer< MySyncPolicy > sync;

    //! Publisher for the data 
    ros::Publisher data_info;

    // There always should be a listener and a broadcaster!
    //! A tf transform listener
    tf::TransformListener tf_listener_;

    //! A tf transform broadcaster
    tf::TransformBroadcaster tf_broadcaster_;

  public:
    //------------------ Callbacks -------------------
    // Callback for performing the tactile computations
  /*  void callback(const MarkerPtr&,const transPtr&,const transPtr&);
    dataPublisher(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"),sub_markers_readings(nh,nh.resolveName("/all_markers"),1),sub_wristlink_readings(nh,"/world_wrist",1),sub_starlink_readings(nh,"/world_star",1),sync(MySyncPolicy(10),sub_markers_readings,sub_wristlink_readings,sub_starlink_readings)
    { 
      sync.registerCallback( boost::bind( &dataPublisher::callback, this, _1, _2, _3 ) );
      data_info = nh_.advertise<AllData>(nh_.resolveName("/data_info"), 10);
    }*/

   void callback(const MarkerPtr& ,const scenePtr&,const transPtr&,const transPtr&);
     dataPublisher(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"),sub_markers_readings(nh,nh.resolveName("/all_markers"),1),sub_camera_readings(nh,"/camera/depth_registered/points",1),sub_starlink_readings(nh,"/world_star",1),sub_wristlink_readings(nh,"/world_wrist",1),sync(MySyncPolicy(10),sub_markers_readings,sub_camera_readings,sub_starlink_readings,sub_wristlink_readings)
    { 
      sync.registerCallback( boost::bind( &dataPublisher::callback, this, _1, _2 ,_3,_4) );
      data_info = nh_.advertise<AllData>(nh_.resolveName("/data_info"), 10);
    }



    //! Empty stub
    ~dataPublisher() {}

};

//Publishe for the markers, the point cloud , the tranformation between world->object
void dataPublisher::callback(const MarkerPtr& markers,const scenePtr& scene,const transPtr& t_w_s,const transPtr& t_w_wr)
//void dataPublisher::callback(const MarkerPtr& markers,const transPtr& t_w_wr,const transPtr& t_w_s)
{
  // read from the topics
  AllData message;
 //
  message.header = markers->header;

  //message.thimbles.resize(5);
  //message.thimbles[0]=*thimble1;
  //message.thimbles[1]=*thimble2;
  //message.thimbles[2]=*thimble3;
  /*message.thimble[3]=*thimble4;
  message.thimble[4]=*thimble5;
  */
  message.LEDs.markers.resize((*markers).LEDs.markers.size());
  message.LEDs=(*markers).LEDs;
  message.scene= *scene;
  message.world_wrist_link=*t_w_wr;
//  message.world_kinect_link=*t_w_k;
  message.object_star_link=*t_w_s;

  data_info.publish(message);
}


} // namespace collect

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "dataPublisher_node");
  ros::NodeHandle nh;

  collect::dataPublisher node(nh);

  ros::spin();  
  
  return 0;
}