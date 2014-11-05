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

class system_calibration
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    //tf::Transform trans_phase_wrist_;
    tf::Transform tf_world_kinect;
    tf::StampedTransform tf_world_wrist_;

    std::string phase_frame_name_;
    std::string wrist_frame_name_;
 
    
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
    system_calibration(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      // In case you want to change the name of the gauss frame (child) and the FTframe (parent)
      priv_nh_.param<std::string>("phase_frame_name_", phase_frame_name_, "/world");
      priv_nh_.param<std::string>("wrist_frame_name_", wrist_frame_name_, "/wrist_bracelet");
      


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
     
      
      try{  tf_listener_.waitForTransform(phase_frame_name_, wrist_frame_name_, ros::Time(0), ros::Duration(10.0) );
           

            tf_listener_.lookupTransform(phase_frame_name_, wrist_frame_name_, ros::Time(0),tf_world_wrist_);
      }
      catch (tf::TransformException ex){ ROS_ERROR("%s",ex.what()); }
      
      calculateTranformation();
    }

    //! Empty stub
    ~system_calibration() {}

};

Matrix4f calculate_object_pose()
{

 string filename = "/home/kuko/Code/poseEstimation/parametersFiles/config.txt";    
 ParametersPoseEstimation params(filename);
 
 string recognizedObjects_dir = "/home/kuko/Code/poseEstimation/data/recognizedObjects"; 
 I_SegmentedObjects objects(recognizedObjects_dir);
 pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points = params.kinectGrabFrame();

 params.recognizePose(objects,xyz_points);   
 
 boost::shared_ptr<vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms = objects.getTransforms ();
 Eigen::Matrix4f object_pose = transforms->at(0);
 return object_pose;
}

void system_calibration::calculateTranformation()
{
  //calculate the 4x4 matrix from the /world wrist transformation
  tf::Matrix3x3 rot= tf_world_wrist_.getBasis();
  tf::Vector3 trasl=tf_world_wrist_.getOrigin();




  Matrix4f trans_world_wrist;
  for (int i=0;i<3;i++)
  {  for(int j=0;j<3;j++)
        trans_world_wrist(i,j)=rot[i][j];
      trans_world_wrist(i,3)=trasl[i];
  }   
  trans_world_wrist(3,0)=trans_world_wrist(3,1)=trans_world_wrist(3,2)=0;
  trans_world_wrist(3,3)=1;

 
  
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

 Matrix4f trans1=trans_new*trans_world_wrist.inverse();

  Eigen::Quaternionf mat_rotx(trans1.block<3,3>(0,0));
  Eigen::Vector3f traslx=trans1.block<3,1>(0,3).transpose();
  std::cout << "mondoincamera con posa manuale: " <<traslx(0)<<' '<<traslx(1)<<' '<<traslx(2)<<' '<<mat_rotx.x() <<' '<<mat_rotx.y()<<' '<<mat_rotx.z()<<' '<<mat_rotx.w()<< std::endl;



//automatic pose
 /* Matrix4f pose_wrist=calculate_object_pose();
  Eigen::Quaternionf mat_rot1(pose_wrist.block<3,3>(0,0));
  Eigen::Vector3f trasl2=pose_wrist.block<3,1>(0,3).transpose();
  std::cout << "wristinCamera: " <<trasl2(0)<<' '<<trasl2(1)<<' '<<trasl2(2)<<' '<<mat_rot1.x() <<' '<<mat_rot1.y()<<' '<<mat_rot1.z()<<' '<<mat_rot1.w()<< std::endl;

  Matrix4f trans=pose_wrist*trans_world_wrist.inverse();
  Eigen::Quaternionf mat_rot(trans.block<3,3>(0,0));
  Eigen::Vector3f trasl1=trans.block<3,1>(0,3).transpose();
  std::cout << "mondoincamera con posa automatica: " <<trasl1(0)<<' '<<trasl1(1)<<' '<<trasl1(2)<<' '<<mat_rot.x() <<' '<<mat_rot.y()<<' '<<mat_rot.z()<<' '<<mat_rot.w()<< std::endl;
*/


}

} // namespace intrinsic_tactile_toolbox

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "kinect_calibration_node");
  ros::NodeHandle nh;

  tf_frames::system_calibration node(nh);
  //node.calculateTranformation();
  ros::spin();  
  
  return 0;
}