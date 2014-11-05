#include <fstream>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include "phase_space/AllMarkers.h"

#include "kalman.h"

#include <Eigen/Eigen>

namespace transformations {

class online_tf
{
  private:
    // the node handle
    ros::NodeHandle nh_;

    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // subscribers
    ros::Subscriber read_cad_positions;

    ros::Publisher publish_transforms;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

  	std::string cad_file_path;
  	std::string start_frame;
  	std::string end_frame;
    std::string topic_name;
  	std::vector<int> marker_id;

  	Eigen::MatrixXd cad_points;
	Eigen::MatrixXd read_points;

	kalman filter;

	Eigen::VectorXd transfParameters;


  	void read_cad_coordinates(std::string,int);


  public:
//

    // callback functions
    void calculate_tf(const phase_space::AllMarkers & msg);

   void OptimalRigidTransformation(Eigen::MatrixXd startP, Eigen::MatrixXd finalP);


    // constructor
    online_tf(ros::NodeHandle nh, int argc,char** argv) : nh_(nh), priv_nh_("~"), filter()
    {
        priv_nh_.param<std::string>("cad_file", cad_file_path, "");
        priv_nh_.param<std::string>("start_frame", start_frame, "");
        priv_nh_.param<std::string>("end_frame", end_frame, "");
        priv_nh_.param<std::string>("topic_name", topic_name, "");

 		publish_transforms = nh_.advertise<geometry_msgs::TransformStamped>(nh_.resolveName(topic_name), 10);


        int max_id=-1;
        marker_id.resize(argc-1);
    	for (int i=0;i<argc-1;i++)
    	{	marker_id[i]=std::atoi(argv[i+1]);
    		if (marker_id[i]>max_id)
    			max_id=marker_id[i];
    	}
    	
    	read_cad_coordinates(cad_file_path,max_id);
    	transfParameters.resize(7);
    	    
        // subscribe to topics
    	read_cad_positions = nh_.subscribe(nh_.resolveName("/all_markers"), 
                                                  10, &online_tf::calculate_tf,this);
    
    }

    //! Empty stub
    ~online_tf() {}

};



void online_tf::read_cad_coordinates(std::string cad_file_path,int max_id)
{	
	std::ifstream f_in(cad_file_path.c_str());
	int id;
	
	cad_points=Eigen::MatrixXd::Zero(max_id+1,4);
	
	if (!f_in)
		ROS_ERROR("Error in opening file %s",cad_file_path.c_str());
	while(f_in>>id)
	{	
		if (!(f_in>>cad_points(id,0)>>cad_points(id,1)>>cad_points(id,2)))
			ROS_ERROR("Error in reading file %s",cad_file_path.c_str());
		
	}
}


// this function is called when a new message is received at the topic_name_you_want_to_subscribe_to
//void online_tf::calculate_tf(const visualization_msgs::MarkerArray & msg)
void online_tf::calculate_tf(const  phase_space::AllMarkers & msgX)

{	tf::StampedTransform tr;

	visualization_msgs::MarkerArray msg=msgX.LEDs;
    read_points=Eigen::MatrixXd::Zero(msg.markers.size(),4);
	int numrows=0;
	

	for (int i=0;i<msg.markers.size();i++)
	{	//check if the markers id is in the list and if it has not be read yet
		if((find(marker_id.begin(), marker_id.end(), msg.markers[i].id) != marker_id.end())) // && read_points((msg.markers[i].id,0)==0))	
		//if (read_points((msg.markers[i].id,0)==0))
		{	read_points(numrows,0)=msg.markers[i].id;
			read_points(numrows,1)=msg.markers[i].pose.position.x;
			read_points(numrows,2)=msg.markers[i].pose.position.y;
			read_points(numrows,3)=msg.markers[i].pose.position.z;
			numrows++;
		}

	}	


	Eigen::MatrixXd appo=Eigen::MatrixXd::Zero(numrows,4);
	for (int i=0;i<numrows;i++)
		appo.row(i)=cad_points.row(read_points(i,0));
	
    

	Eigen::MatrixXd startP=appo.block(0,0,numrows,3);

	Eigen::MatrixXd finalP=read_points.block(0,1,numrows,3);
	
	if (filter.getFirst()==0)
	{	
		filter.setFirst(1);
		OptimalRigidTransformation(startP,finalP);
		filter.setKalman_x(transfParameters);


	}		
	else
	{	
		filter.prediction(startP);		
		transfParameters=filter.update(finalP);
	}

	tr.setOrigin( tf::Vector3(transfParameters(0),transfParameters(1),transfParameters(2)));
   	tr.setRotation( tf::Quaternion(transfParameters(3),transfParameters(4),transfParameters(5),transfParameters(6)));

	tf_broadcaster_.sendTransform(tf::StampedTransform(tr, ros::Time::now(), start_frame.c_str(), end_frame.c_str()));

	geometry_msgs::TransformStamped mess;

	mess.header.stamp=msgX.header.stamp;
	mess.header.seq=msgX.header.seq;
	tf::transformStampedTFToMsg(tr, mess);
//  marker.header.stamp = ros::Time::now();
   //  marker.header.seq=std_msgs.msg.Header().seq
	mess.header.stamp=msgX.header.stamp;
	mess.header.seq=msgX.header.seq;
	publish_transforms.publish(mess);


 
}

void online_tf::OptimalRigidTransformation(Eigen::MatrixXd startP, Eigen::MatrixXd finalP)
{	
	Eigen::Matrix4d transf;
	
	if (startP.rows()!=finalP.rows())
	{	ROS_ERROR("The number of rows of startP and finalP have to be the same");
		exit(1);
	}

	Eigen::RowVector3d centroid_startP=Eigen::RowVector3d::Zero(); 
	Eigen::RowVector3d centroid_finalP=Eigen::RowVector3d::Zero(); //= mean(B);
	Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

	//calculate the mean
	for (int i=0;i<startP.rows();i++)
	{	centroid_startP=centroid_startP+startP.row(i);
		centroid_finalP=centroid_finalP+finalP.row(i);
	}
	
	centroid_startP=centroid_startP/startP.rows();
	centroid_finalP=centroid_finalP/startP.rows();

	for (int i=0;i<startP.rows();i++)
		H=H+(startP.row(i)-centroid_startP).transpose()*(finalP.row(i)-centroid_finalP);

   	Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
   
    Eigen::MatrixXd U = svd.matrixU();
   	Eigen::MatrixXd V = svd.matrixV();
  
    if (V.determinant()<0)
   		V.col(2)=-V.col(2)*(-1);

	Eigen::MatrixXd R=V*U.transpose();

	Eigen::Matrix4d C_A = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d C_B = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d R_new = Eigen::Matrix4d::Identity();
			
	C_A.block<3,1>(0,3)=-centroid_startP.transpose();
	R_new.block<3,3>(0,0)=R;
	
	C_B.block<3,1>(0,3)=centroid_finalP.transpose();


	transf = C_B * R_new * C_A;

	//std::cout<<"trans: "<<transf<<std::endl;

	Eigen::Quaterniond mat_rot(transf.block<3,3>(0,0));

	Eigen::Vector3d trasl=transf.block<3,1>(0,3).transpose();

	transfParameters<<trasl(0),trasl(1),trasl(2),mat_rot.x(),mat_rot.y(),mat_rot.z(),mat_rot.w();

}

} // namespace package_name

int main(int argc, char **argv)
{
    ros::init(argc, argv, "online_tf");
 	 	
 	ros::NodeHandle nh;

    transformations::online_tf node(nh,argc,argv);

    while(ros::ok())
    {
 
    ros::spinOnce();
    }

    return 0;
}


