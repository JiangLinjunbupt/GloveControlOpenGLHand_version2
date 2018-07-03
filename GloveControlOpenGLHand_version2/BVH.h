#pragma once
#include <vector>
#include <map>
#include <string>
#include <Eigen/Core>
#include <Eigen/LU>
#include "HandStruct.h"
#include <iostream> 
using namespace  std;

class  BVH
{
public:
	enum  ChannelEnum
	{
		X_ROTATION, Y_ROTATION, Z_ROTATION,
		X_POSITION, Y_POSITION, Z_POSITION
	};
	struct  Joint;       //由于Joint的定义再Channel结构体之后，这里起到一个声明的作用。

	struct  Channel
	{

		Joint *              joint;

		ChannelEnum          type;

		int                  index;
	};

	struct  Joint
	{
		string               name;
		int                  index;
		Joint *              parent;
		vector< Joint * >    children;
		double               offset[3];
		vector< Channel * >  channels;

		// local coordiate // 4*4
		Eigen::MatrixXd local;          //模型矩阵（世界坐标等于模型矩阵乘以模型坐标）								
		Eigen::MatrixXd local_inverse;      //模型矩阵的逆

											// global coordinate // 4*4
		Eigen::MatrixXd global;
		// local rotation // 4*4
		Eigen::MatrixXd rotation;
		// transformation to parent: 4*4
		Eigen::MatrixXd trans;
		// trans* rotation
		Eigen::MatrixXd transR;

		// the rotation of x, y, z;
		Pose pose;
		Pose upper;
		Pose lower;

		bool dof[3];
		int corresponds[3]; // the index of parameter that 
		double local_position[3];  // local position

								   // global position
		double global_position[3];

		// initial global position
		double init_global_position[3];

		// target position: used for inverse kinematics
		double target_position[3];

		// scale
		double scale;


	};


private:
	bool                     is_load_success;
	string                   file_name;
	string                   motion_name;
	int                      num_channel;
	vector< Channel * >      channels;

	vector< Joint * >        joints;
	vector<Joint *>          wristBased_joints;
	map< string, Joint * >   joint_index;


public:
	BVH();
	BVH(const char * bvh_file_name);
	~BVH();
	void  Clear();
	void  Load(const char * bvh_file_name);
	void  ChangeToWristBased();
public:
	bool  IsLoadSuccess() const { return is_load_success; }
	const string &  GetFileName() const { return file_name; }
	const string &  GetMotionName() const { return motion_name; }
	const int       GetNumJoint() const { return  joints.size(); }
	const int       GetNumChannel() const { return  channels.size(); }
	const Channel * GetChannel(int no) const { return  channels[no]; }

	Joint *   GetJoint(const string & j) const {
		map< string, Joint * >::const_iterator  i = joint_index.find(j);
		return  (i != joint_index.end()) ? (*i).second : NULL;
	}
	Joint *   GetJoint(const char * j) const {
		map< string, Joint * >::const_iterator  i = joint_index.find(j);
		return  (i != joint_index.end()) ? (*i).second : NULL;
	}
	Joint *   GetJoint(int i) const { return  joints[i]; }

};
