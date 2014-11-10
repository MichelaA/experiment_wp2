#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


#include <Eigen/Eigen>
using namespace Eigen;

namespace tf_frames {

class hand_calibration
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    //tf::Transform trans_phase_wrist_;
    tf::StampedTransform tf_world_wrist_;
    tf::StampedTransform tf_world_hand_;

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
    hand_calibration(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      // In case you want to change the name of the gauss frame (child) and the FTframe (parent)
      priv_nh_.param<std::string>("phase_frame_name_", phase_frame_name_, "/world");
      priv_nh_.param<std::string>("wrist_frame_name_", wrist_frame_name_, "/wrist_bracelet");

      
      try{  tf_listener_.waitForTransform(phase_frame_name_, wrist_frame_name_, ros::Time(0), ros::Duration(10.0) );
           

            tf_listener_.lookupTransform(phase_frame_name_, wrist_frame_name_, ros::Time(0),tf_world_wrist_);
      }
      catch (tf::TransformException ex){ ROS_ERROR("%s",ex.what()); }
      
      
      try{  tf_listener_.waitForTransform(phase_frame_name_, "/hand", ros::Time(0), ros::Duration(10.0) );
           

            tf_listener_.lookupTransform(phase_frame_name_,  "/hand", ros::Time(0),tf_world_hand_);
      }
      catch (tf::TransformException ex){ ROS_ERROR("%s",ex.what()); }


      calculateTranformation();
    }

    //! Empty stub
    ~hand_calibration() {}

};



void hand_calibration::calculateTranformation()
{
  //calculate the 4x4 matrix from the hand wrist transformation
  tf::Matrix3x3 rot= tf_world_wrist_.getBasis();
  tf::Vector3 trasl=tf_world_wrist_.getOrigin();

 std::cout<<"tf world wrist";
  Matrix4f trans_world_wrist;
  for (int i=0;i<3;i++)
  {  for(int j=0;j<3;j++)
      {  trans_world_wrist(i,j)=rot[i][j];
        std::cout<<trans_world_wrist(i,j)<<' ';
      }
      std::cout<<std::endl;
      trans_world_wrist(i,3)=trasl[i];
  }   
  trans_world_wrist(3,0)=trans_world_wrist(3,1)=trans_world_wrist(3,2)=0;
  trans_world_wrist(3,3)=1;

  Matrix4f trans_world_hand;
  tf::Matrix3x3 rot_hand= tf_world_hand_.getBasis();
  tf::Vector3 trasl_hand=tf_world_hand_.getOrigin();
  std::cout<<"tf world hand";
  for (int i=0;i<3;i++)
  {  for(int j=0;j<3;j++)
      {  trans_world_hand(i,j)=rot_hand[i][j];
          std::cout<<trans_world_hand(i,j)<<' ';
      }
      std::cout<<std::endl;
      trans_world_hand(i,3)=trasl_hand[i];
  }   
  trans_world_hand(3,0)=trans_world_hand(3,1)=trans_world_hand(3,2)=0;
  trans_world_hand(3,3)=1;


  Matrix4f trans1=trans_world_hand*trans_world_wrist.inverse();

  Eigen::Quaternionf mat_rotx(trans1.block<3,3>(0,0));
  Eigen::Vector3f traslx=trans1.block<3,1>(0,3).transpose();
  std::cout << "wrist_in_hand: " <<traslx(0)<<' '<<traslx(1)<<' '<<traslx(2)<<' '<<mat_rotx.x() <<' '<<mat_rotx.y()<<' '<<mat_rotx.z()<<' '<<mat_rotx.w()<< std::endl;


}

} 

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "hand_calibration_node");
  ros::NodeHandle nh;

  tf_frames::hand_calibration node(nh);
  //node.calculateTranformation();
  ros::spin();  
  
  return 0;
}