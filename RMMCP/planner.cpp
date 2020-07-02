#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <sstream>
#include <kdl/chain.hpp>
#include <kdl/path_line.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <stdio.h>

using namespace KDL;
Chain chain;
ros::Subscriber sub;
ros::Publisher pub;
std::stringstream errorOut;


bool parseRequest(std::string msg,std::string* type,JntArray* q0 ,Vector* pos, Rotation* rot, int* points) 
{
	//Output variables
	double x,y,z, rx,ry,rz;
	std::string aux;
	//Parse the string
	int wordType=0, wordInitial = 1, wordPos=2, wordRot=3, wordPoints=4;
	std::stringstream sString(msg);
	std::istream_iterator<std::string> begin(sString);
	std::istream_iterator<std::string> end;
	std::vector<std::string> vstrings(begin, end);
	std::string initial, position, rotation;
	//We divided the string in a vector of strings

	if(vstrings.size() != 5) 
	{
		errorOut << "Error, only 5 words allowed in string but " << vstrings.size() << " found" << std::endl;
		ROS_INFO("%s", errorOut.str().c_str());
		errorOut.str("");
		return false;
	}
	*type = vstrings.at(wordType);
	initial = vstrings.at(wordInitial);
	position = vstrings.at(wordPos);
	rotation = vstrings.at(wordRot);
	*points = (int) std::strtol(vstrings.at(wordPoints).c_str(),NULL,10);
	if(*points < 1)
	{
		ROS_INFO("Warning, invalid number of points: %d, setted to 1", *points);
	}
	{
		//Parse q0 
		std::vector<std::string> v;
		Eigen::VectorXd vXd(chain.getNrOfJoints());
    	std::stringstream ss(initial);
		while(std::getline(ss,aux,','))
		{
			v.push_back(aux);
		}
		if(v.size() != chain.getNrOfJoints()) 
		{
			errorOut << "your chain model has " << chain.getNrOfJoints() <<" joints but " << v.size() << " found in q0 field" << std::endl;
			ROS_INFO("Error, %s", errorOut.str().c_str());
			errorOut.str("");
			return false;
		}
		for (int i = 0; i < v.size(); i++)
		{
			vXd[i]=std::strtod(v.at(i).c_str(),NULL);
		}
    	q0->data = vXd;
	}
	{
		//Parse position 
		std::vector<std::string> v;
    	std::stringstream ss(position);
		while(std::getline(ss,aux,','))
		{
			v.push_back(aux);
		}
		if(v.size() != 3) 
		{
			errorOut << "only 3 numbers in position field allowed but " << v.size() << " found" << std::endl;
			ROS_INFO("Error, %s", errorOut.str().c_str());
			errorOut.str("");
			return false;
		}
    	pos->x(std::strtod(v.at(0).c_str(),NULL));
    	pos->y(std::strtod(v.at(1).c_str(),NULL));
    	pos->z(std::strtod(v.at(2).c_str(),NULL));
	}
	{	//Parse rotation
		std::vector<std::string> v;
    	std::stringstream ss(rotation);
		while(std::getline(ss,aux,','))
		{
			v.push_back(aux);
		}
		if(v.size() != 3) 
		{
			errorOut << "only 3 numbers in rotation field allowed but " << v.size() << " found" << std::endl;
			ROS_INFO("Error, %s", errorOut.str().c_str());
			errorOut.str("");
			return false;
		}
    	*rot=rot->EulerZYX(std::strtod(v.at(0).c_str(),NULL),std::strtod(v.at(1).c_str(),NULL),std::strtod(v.at(2).c_str(),NULL));
	
	}	
	if(*type != std::string("L") && *type != std::string("J"))
	{
		errorOut << "traj types allowed are J or L, but " << *type << " found" <<std::endl;
		ROS_INFO("Error, %s", errorOut.str().c_str());
		errorOut.str("");
		return false;
	}
	return true;
}

//This function reads the command received by the python task and...
//Parses it to take the parameters
//
void requestCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Planner heard: [%s]", msg->data.c_str());

	//Parse the request
	std::string type;
	JntArray q0(chain.getNrOfJoints());
	Vector pos;
	Rotation rot;
	int points;
	if (!parseRequest(msg->data, &type, &q0, &pos, &rot, &points))
	{
		errorOut << "found in the message format, request ignored" << std::endl;
		ROS_INFO("Error, %s", errorOut.str().c_str());
		errorOut.str("");
		return;
	}
	JntArray q_out(chain.getNrOfJoints());	

	//What type of path do we want to do?
	std::vector<JntArray> vecQ_out; //Vector with each group of joints position to be reached
	if (type == std::string("J"))
	{
		Frame end(rot,pos);
		//Inverse kinematics solver
		ChainIkSolverPos_LMA solver(chain);
		//We need to do the inverse kinematics of the final point
		JntArray q_target, q_diff;
		solver.CartToJnt(q0,end,q_target);
		Subtract(q_target, q0,q_diff);
		//Interpolate "point" number of values between q0 and q_out
		for	(int i=0; i < points;i++)
		{
			Multiply(q_diff,(double)(i+1)/(double)points,q_out);
			Add(q0,q_out,q_out);
			vecQ_out.push_back(q_out);
		}
	} 
	else if (type == std::string("L"))
	{
		Frame end(rot,pos);
		//Forward kinematics solver
		ChainFkSolverPos_recursive fk(chain); //To solve the starting transform
		//Inverse kinematics solver
		ChainIkSolverPos_LMA solver(chain);
		//Path creation	
		Frame start;		
		fk.JntToCart(q0, start);
		RotationalInterpolation_SingleAxis rotInt;
		Path_Line line(start,end,rotInt.Clone(),1,1);
		double interval = line.PathLength()/(double)points;		
		int i=0;
		for (int i=0; i < points;i++)
		{
			Frame target;
			target=line.Pos(interval*(i+1));
			solver.CartToJnt(q0, target, q_out);
			q0.data=q_out.data;			
			vecQ_out.push_back(q_out);
		}
	}
	//Prepare the message output
	trajectory_msgs::JointTrajectory traj;	
	for (int i = 0; i < points; i++) //For each point
	{
		trajectory_msgs::JointTrajectoryPoint jtPoint;
		JntArray p = vecQ_out.at(i);
		for (int j = 0; j < chain.getNrOfJoints(); j++) //For each joint
		{
			jtPoint.positions.push_back(p(j));
		}
		traj.points.push_back(jtPoint);
	}
	//Publish the message
	pub.publish (traj);
}
int main(int argc, char **argv)
{
  //Definition of your robot, the given example correspond to the image at the assignment guidelines
  //Consider the world frame, this example considers Z axis vertical
  //First argument is type (RotX, RotY,RotZ, TransX, TransY, TransZ)
  //Second argument is scale, use -1 if the angle/translation decrements with joint value
  //Prismatic joints must have a value of scale to transform radians to mm
  //This example model uses mm, joint values are supposed to be equivalent to real measurements
  //beforehand, you can handle this in your python program or specifying a scale in your model
  //Your model must be consistent with what you send via ROS message.
  /**************************************************************/
  // TO DO: Remember that these lines are an example for an specific robot, MODIFY WITH YOURS
  Joint j1(Joint::RotZ); 
  Joint j2(Joint::RotY,-1); //-1 means that joint value decreases at axis direction, put here or in the python code
  Joint j3(Joint::TransZ,-1); //Rot for rotational, Trans for prismatic.

  //Joint j4(Joint::RotZ,-1); //This is a comment, you can build a robot with more than 3 DOF

  //After defining joints, define the kinematic chain
  //This works ading to each joint the link (segment) to the next joint
  //Consider your first joint at the world frame
  //Try to locate the rotational joints where the physical rotation of the link starts unless 
  //it is the first one, in that case try to consider it is at the (0,0,0) position.
  //Try to locate the prismatic joints at the end position of the moving part.

  chain.addSegment(Segment(j1,Frame(Vector(5,-5,40)))); //Means that the position to the next joint (j2)
  //is located at Vector coordinates from the current joint (j1)
  chain.addSegment(Segment(j2,Frame(Vector(0,0,-5)))); // 
  chain.addSegment(Segment(j3,Frame(Vector(0,0,-5))));

  //chain.addSegment(Segment(j4)); //Again, this is a comment
  /**************************************************************/
  //Initialize this ros node
  ros::init(argc, argv, "planner");

  ros::NodeHandle n;
  //Subscribe to topic request, reading strings from your python code 
	sub = n.subscribe<std_msgs::String>("request", 100, requestCallback);
	pub = n.advertise<trajectory_msgs::JointTrajectory>("trajectory", 100);

  //Iterate for listening callbacks 
  ros::spin();
}