#include <ros/ros.h>
/*#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>*/
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


//#include "definitions/PoseEstimation.h"
#include "ParametersPoseEstimation.h"

#include <Eigen/Eigen>
using namespace Eigen;

namespace tf_frames {

class object_calibration
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    //tf::Transform trans_phase_wrist_;
    tf::StampedTransform tf_kinect_world_;
    tf::StampedTransform tf_world_star_;

    std::string phase_frame_name_;
    std::string camera_frame_name_;
    std::string star_frame_name_;
    
    // There always should be a listener and a broadcaster!
    //! A tf transform listener
    tf::TransformListener tf_listener_;
   

    //! A tf transform broadcaster
    tf::TransformBroadcaster tf_broadcaster_;

    //Maual pose estimation
    Eigen::Quaternionf manualQwxyz;
    Eigen::Vector3f manualTxyz;

  public:
    //------------------ Callbacks -------------------
    // Callback for performing the tactile computations
    void calculateTranformation();

    //! Subscribes to and advertises topics
    object_calibration(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      // In case you want to change the name of the gauss frame (child) and the FTframe (parent)
      priv_nh_.param<std::string>("phase_frame_name_", phase_frame_name_, "/world");
      priv_nh_.param<std::string>("star_frame_name", star_frame_name_, "/star");
      priv_nh_.param<std::string>("camera_frame_name", camera_frame_name_, "/camera_rgb_optical_frame");

      std::string txyz_string, quaternionwxyz_string;

      priv_nh_.param<std::string>("manualTxyz", txyz_string, "");
      priv_nh_.param<std::string>("manualQwxyz", quaternionwxyz_string, "");

 
      std::vector<float> elems;
      std::stringstream ss(txyz_string);
      std::string item;
      while (std::getline(ss, item, ',')) {
        elems.push_back(std::atof(item.c_str()));
      }
      manualTxyz(0)=elems[0];
      manualTxyz(1)=elems[1];
      manualTxyz(2)=elems[2];

      elems.clear();
      std::stringstream ss1(quaternionwxyz_string);
      while (std::getline(ss1, item, ',')) {
       elems.push_back(std::atof(item.c_str()));
      }

      manualQwxyz.w()=elems[0];
      manualQwxyz.x()=elems[1];
      manualQwxyz.y()=elems[2];
      manualQwxyz.z()=elems[3];



      try{  tf_listener_.waitForTransform(phase_frame_name_, star_frame_name_, ros::Time(0), ros::Duration(10.0) );
            tf_listener_.lookupTransform(phase_frame_name_, star_frame_name_, ros::Time(0),tf_world_star_);
      }
      catch (tf::TransformException ex){ ROS_ERROR("%s",ex.what()); }

      try{  tf_listener_.waitForTransform(phase_frame_name_, camera_frame_name_, ros::Time(0), ros::Duration(10.0) );
            tf_listener_.lookupTransform(phase_frame_name_, camera_frame_name_, ros::Time(0),tf_kinect_world_);
      }
      catch (tf::TransformException ex){ ROS_ERROR("%s",ex.what()); }
 
      calculateTranformation();
    }

    //! Empty stub
    ~object_calibration() {}

};

Matrix4f calculate_object_pose()
{

 string filename = "/home/kuko/Code/poseEstimation/parametersFiles/config.txt";    
 ParametersPoseEstimation params(filename);
 
 string recognizedObjects_dir = "/home/kuko/Code/poseEstimation/data/recognizedObjects"; 
 I_SegmentedObjects objects(recognizedObjects_dir);
 pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points = params.kinectGrabFrame();

 params.recognizePose(objects,xyz_points);   
 // definitions::Object detected_objects;
  boost::shared_ptr<vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms = objects.getTransforms ();
  Eigen::Matrix4f object_pose = transforms->at(0);
  return object_pose;
}

void object_calibration::calculateTranformation()
{
  //calculate the 4x4 matrix from the /world to /star
  tf::Matrix3x3 rot= tf_world_star_.getBasis();
  tf::Vector3 trasl=tf_world_star_.getOrigin();

  Matrix4f matrix_world_star;
  for (int i=0;i<3;i++)
  {  for(int j=0;j<3;j++)
        matrix_world_star(i,j)=rot[i][j];
      matrix_world_star(i,3)=trasl[i];
  }   
  matrix_world_star(3,0)=matrix_world_star(3,1)=matrix_world_star(3,2)=0;
  matrix_world_star(3,3)=1;

  //calculate the 4x4 matrix from the /kinect /world
  rot= tf_kinect_world_.getBasis();
  trasl=tf_kinect_world_.getOrigin();

  Matrix4f matrix_kinect_world;
  for (int i=0;i<3;i++)
  {  for(int j=0;j<3;j++)
        matrix_kinect_world(i,j)=rot[i][j];
      matrix_kinect_world(i,3)=trasl[i];
  }   
  matrix_kinect_world(3,0)=matrix_kinect_world(3,1)=matrix_kinect_world(3,2)=0;
  matrix_kinect_world(3,3)=1;


 ///manual pose calculated through ikc
Eigen::Matrix3f rot1= manualQwxyz.toRotationMatrix();
Matrix4f trans_new;
for (int i=0;i<3;i++)
  {  for(int j=0;j<3;j++)
        trans_new(i,j)=rot1(i,j);
      trans_new(i,3)=manualTxyz[i];
  }   
  trans_new(3,0)=trans_new(3,1)=trans_new(3,2)=0;
  trans_new(3,3)=1;


/*  Matrix4f matrix_pose_object=calculate_object_pose();
  Eigen::Quaternionf mat_rot1(matrix_pose_object.block<3,3>(0,0));
  Eigen::Vector3f trasl2=matrix_pose_object.block<3,1>(0,3).transpose();
  std::cout << "object_in_camera"<<trasl2(0)<<' '<<trasl2(1)<<' '<<trasl2(2)<<' '<<mat_rot1.x() <<' '<<mat_rot1.y()<<' '<<mat_rot1.z()<<' '<<mat_rot1.w()<< std::endl;  
  
  Matrix4f star_object_trans=matrix_world_star.inverse()*matrix_kinect_world*matrix_pose_object;
  Eigen::Quaternionf mat_rot(star_object_trans.block<3,3>(0,0));
  Eigen::Vector3f trasl1=star_object_trans.block<3,1>(0,3).transpose();
  std::cout << "object_in_star posa automatica:" <<trasl1(0)<<' '<<trasl1(1)<<' '<<trasl1(2)<<' '<<mat_rot.x() <<' '<<mat_rot.y()<<' '<<mat_rot.z()<<' '<<mat_rot.w()<< std::endl;
*/
  Matrix4f star_object_trans1=matrix_world_star.inverse()*matrix_kinect_world*trans_new;
  Eigen::Quaternionf mat_rot2(star_object_trans1.block<3,3>(0,0)); 
  Eigen::Vector3f trasl3=star_object_trans1.block<3,1>(0,3).transpose();
  std::cout << "object_in_star con posa manuale: " <<trasl3(0)<<' '<<trasl3(1)<<' '<<trasl3(2)<<' '<<mat_rot2.x() <<' '<<mat_rot2.y()<<' '<<mat_rot2.z()<<' '<<mat_rot2.w()<< std::endl;


/*
  Matrix4f star_object_transI=(matrix_kinect_world*matrix_pose_object);    
  Eigen::Quaternionf mat_rotI(star_object_transI.block<3,3>(0,0));
  Eigen::Vector3f traslI=star_object_transI.block<3,1>(0,3).transpose();
  std::cout << "object_in_world:" <<traslI(0)<<' '<<traslI(1)<<' '<<traslI(2)<<' '<<mat_rotI.x() <<' '<<mat_rotI.y()<<' '<<mat_rotI.z()<<' '<<mat_rotI.w()<< std::endl;
*/

/*
tf::Transform tf_world_kinect_;
  tf_world_kinect_.setOrigin(tf::Vector3(trasl1(0),trasl1(1),trasl1(2)));
  tf_world_kinect_.setRotation( tf::Quaternion(mat_rot.x(),mat_rot.y(),mat_rot.z(),mat_rot.w()));
 tf_broadcaster_.sendTransform(tf::StampedTransform(tf_world_kinect, ros::Time::now(),  object_frame_,phase_frame_name_));
  */
}

} // namespace intrinsic_tactile_toolbox

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "calibration_node");
  ros::NodeHandle nh;

  tf_frames::object_calibration node(nh);
 
  ros::spin();  
  
  return 0;
}