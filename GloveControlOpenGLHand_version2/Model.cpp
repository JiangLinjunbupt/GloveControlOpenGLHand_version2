#include "Model.h"
#include <iostream>
#include <fstream>
#define PI 3.1415926

#include "util.h"
#include <Eigen/LU>
Model::Model() {


}

Model::Model(char* file) {
	bvh_.Load(file);
}

Model::~Model() {
	delete[] OptimizedParams;
	delete[] upper_bound;
	delete[] lower_bound;
	delete[] corresponds_;
}

void Model::set_upper_lower_bound()
{
	//pinkey
	upper_bound[0] = 90; lower_bound[0] = -10;    //low  //弯曲
	upper_bound[1] = 10; lower_bound[1] = -30;           //左右
	upper_bound[2] = 90; lower_bound[2] = 0;    //middle
	upper_bound[20] = 90; lower_bound[20] = 0;   //top

												 //ring
	upper_bound[3] = 90; lower_bound[3] = -10;   //low     //弯曲
	upper_bound[4] = 10; lower_bound[4] = -20;             //左右
	upper_bound[5] = 90; lower_bound[5] = 0;     //middle
	upper_bound[21] = 90; lower_bound[21] = 0;   //top

												 //middle
	upper_bound[6] = 90; lower_bound[6] = -10;    //low      //弯曲
	upper_bound[7] = 0.0; lower_bound[7] = 0.0;               //左右
	upper_bound[8] = 90; lower_bound[8] = 0;     //middle
	upper_bound[22] = 90; lower_bound[22] = 0;   //top

												 //index
	upper_bound[9] = 90; lower_bound[9] = -10;     //low      //弯曲
	upper_bound[10] = 30; lower_bound[10] = -10;              //左右
	upper_bound[11] = 90; lower_bound[11] = 0;    //middle
	upper_bound[23] = 90; lower_bound[23] = 0;    //top

												  //thumb
	upper_bound[12] = 60; lower_bound[12] = 0;            //low x
	upper_bound[13] = 30; lower_bound[13] = -30;            //low z
	upper_bound[18] = 110; lower_bound[18] = 40;            //low y

	upper_bound[14] = 90; lower_bound[14] = 0;           //top
	upper_bound[19] = 90; lower_bound[19] = 0;          //middle

														//global rotation  //这三个我始终没想好
														//upper_bound[15] = 90; lower_bound[15] = -90;           //x
														//upper_bound[16] = 90; lower_bound[16] = -180;           //y
														//upper_bound[17] = 150; lower_bound[17] = -150;            //z
	upper_bound[15] = 1000; lower_bound[15] = -1000;           //x
	upper_bound[16] = 1000; lower_bound[16] = -1000;           //y
	upper_bound[17] = 1000; lower_bound[17] = -1000;            //z

																//global position
	upper_bound[24] = 200; lower_bound[24] = -600;              //x
	upper_bound[25] = 200; lower_bound[25] = -600;              //y
	upper_bound[26] = -500; lower_bound[26] = -1100;          //z
}

void Model::compute_local_inverse() {
	int num_joint = bvh_.GetNumJoint();
	for (int i = 0; i < num_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		joint->local_inverse = joint->local.inverse();
	}

}

void Model::init() {
	set_upper_lower_bound();
	compute_init_global_position();       //这一步承认是以handbone（手掌中心点）的坐标作为手的全局位置。
	compute_local_coordinate();
	compute_parent_child_transform();
	compute_local_inverse();

	this->load_faces(".\\model\\handfaces.txt");
	this->load_vertices(".\\model\\handverts.txt");
	this->load_weight(".\\model\\weight.txt");


	Pose pose(0, 0, 0);
	pose.x = 0; pose.y = 0; pose.z = -200;
	this->set_global_position(pose);

	this->forward_kinematic();
	this->compute_mesh();
}

void Model::transform_matrix(Pose pose, Eigen::MatrixXd& rot) {
	Eigen::MatrixXd x = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd y = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd z = Eigen::MatrixXd::Identity(4, 4);

	double cx = cos(pose.x / 180 * PI);
	double sx = sin(pose.x / 180 * PI);

	double cy = cos(pose.y / 180 * PI);
	double sy = sin(pose.y / 180 * PI);

	double cz = cos(pose.z / 180 * PI);
	double sz = sin(pose.z / 180 * PI);

	x(1, 1) = cx; x(2, 2) = cx;
	x(1, 2) = -sx; x(2, 1) = sx;

	y(0, 0) = cy; y(0, 2) = sy;
	y(2, 0) = -sy; y(2, 2) = cy;

	z(0, 0) = cz; z(1, 1) = cz;
	z(0, 1) = -sz; z(1, 0) = sz;

	rot = x*y*z;   //旋转顺序 z-y-x
}

void Model::forward_kinematic() {

	int number_joint = bvh_.GetNumJoint();
	compute_global();
	Eigen::MatrixXd I;
	Eigen::MatrixXd t;
	BVH::Joint* joint0 = bvh_.GetJoint(0);
	//Eigen::Matr
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		t = Eigen::MatrixXd(4, 1);
		t(0, 0) = joint->init_global_position[0];
		t(1, 0) = joint->init_global_position[1];
		t(2, 0) = joint->init_global_position[2];
		t(3, 0) = 1; /////joint->scale*
					 /*Eigen::MatrixXd SCALE = Eigen::MatrixXd::Identity(4, 4);
					 SCALE(0, 0) = joint->scale;
					 SCALE(1, 1) = joint->scale;
					 SCALE(2, 2) = joint->scale;*/

		I = joint->global* (joint->scale*((joint->local_inverse)*t));
		//I = joint->global* (SCALE*((joint->local_inverse)*t));
		joint->global_position[0] = I(0, 0) + global_position_.x;
		joint->global_position[1] = I(1, 0) + global_position_.y;
		joint->global_position[2] = I(2, 0) + global_position_.z;
	}


	for (int i = 0; i < number_joint; i++) {

		BVH::Joint* joint_handbone = bvh_.GetJoint(1);
		BVH::Joint* joint = bvh_.GetJoint(i);
		t = Eigen::MatrixXd(4, 1);
		t(0, 0) = joint_handbone->init_global_position[0];
		t(1, 0) = joint_handbone->init_global_position[1];
		t(2, 0) = joint_handbone->init_global_position[2];
		t(3, 0) = 1; /////joint->scale*
					 /*Eigen::MatrixXd SCALE = Eigen::MatrixXd::Identity(4, 4);
					 SCALE(0, 0) = joint->scale;
					 SCALE(1, 1) = joint->scale;
					 SCALE(2, 2) = joint->scale;*/

		I = joint_handbone->global* (joint_handbone->scale*((joint_handbone->local_inverse)*t));
		//I = joint->global* (SCALE*((joint->local_inverse)*t));
		joint->global_position[0] = joint->global_position[0] - I(0, 0);
		joint->global_position[1] = joint->global_position[1] - I(1, 0);
		joint->global_position[2] = joint->global_position[2] - I(2, 0);
	}



}

void Model::set_global_position(Pose global_position) {
	global_position_.x = global_position.x;
	global_position_.y = global_position.y;
	global_position_.z = global_position.z;
}

void Model::compute_global() {
	int number_joint = bvh_.GetNumJoint();
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);

		if (i == 2)
		{
			//测试食指变长
			/*Eigen::MatrixXd longer = Eigen::MatrixXd::Identity(4, 4);
			longer(0, 3) = 100;
			joint->transR = joint->trans*longer*joint->rotation;*/
			joint->transR = joint->trans*joint->rotation;
		}
		else
		{
			joint->transR = joint->trans*joint->rotation;  //rotation 在加载bvh文件时候初始化为4*4单位矩阵   //物体变换的顺序应为：大小，旋转，平移
		}
	}
	BVH::Joint* joint0 = bvh_.GetJoint(0);
	joint0->global = joint0->local;
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		BVH::Joint* joint_parent = joint->parent;
		if (joint_parent != nullptr) {
			joint->global = joint_parent->global*joint->transR;        //在joint【0】不旋转的基础上计算的
		}
	}
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		joint->global = joint0->rotation*joint->global;           //加上Joint【0】的旋转
	}
}

void Model::set_one_rotation(Pose pose, int index) {
	BVH::Joint* joint = bvh_.GetJoint(index);
	joint->pose = pose;
	transform_matrix(joint->pose, joint->rotation);
}

void Model::set_hand_rotation(Pose pose)
{
	BVH::Joint *joint = bvh_.GetJoint(0);

	Eigen::MatrixXd z0 = Eigen::MatrixXd::Identity(4, 4);

	double cz0 = cos(-90.0 / 180 * PI);
	double sz0 = sin(-90.0 / 180 * PI);

	z0(0, 0) = cz0;z0(1, 1) = cz0;
	z0(0, 1) = -sz0;z0(1, 0) = sz0;

	Eigen::MatrixXd x = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd y = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd z = Eigen::MatrixXd::Identity(4, 4);

	double cx = cos(pose.x / 180 * PI);
	double sx = sin(pose.x / 180 * PI);

	double cy = cos(-pose.y / 180 * PI);
	double sy = sin(-pose.y / 180 * PI);

	double cz = cos(-pose.z / 180 * PI);
	double sz = sin(-pose.z / 180 * PI);

	x(1, 1) = cx;x(2, 2) = cx;
	x(1, 2) = -sx;x(2, 1) = sx;

	y(0, 0) = cy;y(0, 2) = sy;
	y(2, 0) = -sy;y(2, 2) = cy;

	z(0, 0) = cz;z(1, 1) = cz;
	z(0, 1) = -sz;z(1, 0) = sz;

	joint->rotation = x*y*z*z0; //旋转顺序 z-y-x
}

void Model::set_thumbLower_rotation(Pose pose)
{
	BVH::Joint *joint = bvh_.GetJoint(18);

	Eigen::MatrixXd x = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd y = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd z = Eigen::MatrixXd::Identity(4, 4);

	double cx = cos(-pose.y / 180 * PI);
	double sx = sin(-pose.y / 180 * PI);

	double cy = cos(pose.x / 180 * PI);
	double sy = sin(pose.x / 180 * PI);

	double cz = cos(pose.z / 180 * PI);
	double sz = sin(pose.z / 180 * PI);

	x(1, 1) = cx;x(2, 2) = cx;
	x(1, 2) = -sx;x(2, 1) = sx;

	y(0, 0) = cy;y(0, 2) = sy;
	y(2, 0) = -sy;y(2, 2) = cy;

	z(0, 0) = cz;z(1, 1) = cz;
	z(0, 1) = -sz;z(1, 0) = sz;

	joint->rotation = x*y*z; //旋转顺序 z-y-x

}

void Model::compute_parent_child_transform() {
	int number_joint = bvh_.GetNumJoint();
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		BVH::Joint* joint_parent = joint->parent;
		if (joint->parent != nullptr) {
			joint->trans = joint_parent->local.inverse()* joint->local;
		}
		else {
			joint->trans = joint->local;
		}
	}
}

void Model::compute_local_coordinate() {
	int number_joint = bvh_.GetNumJoint();
	double axisx[3] = { 0,0,0 };
	double axisy[3] = { 0,0,0 };
	double axisz[3] = { 0,0,1 };
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		BVH::Joint* joint_parent = joint->parent;
		axisx[0] = .0; axisx[1] = .0; axisx[2] = .0;
		axisz[0] = .0; axisz[1] = .0; axisz[2] = 1.0;


		BVH::Joint* joint_child = nullptr;

		if (joint->children.size() == 0) {
			joint->local = joint_parent->local;
			joint->local(0, 3) = joint->init_global_position[0];
			joint->local(1, 3) = joint->init_global_position[1];
			joint->local(2, 3) = joint->init_global_position[2];
			continue;
		}
		if (joint->children.size() >1) {
			if (i == 0)
			{
				joint_child = joint->children[0];
			}
			else
			{
				joint_child = joint->children[1];
			}

		}

		if (joint->children.size() == 1) {
			joint_child = joint->children[0];
		}

		axisx[0] = joint_child->init_global_position[0] - joint->init_global_position[0];
		axisx[1] = joint_child->init_global_position[1] - joint->init_global_position[1];
		axisx[2] = joint_child->init_global_position[2] - joint->init_global_position[2];
		normalize(axisx);
		cross_product(axisx, axisz, axisy);
		normalize(axisy);
		cross_product(axisx, axisy, axisz);
		normalize(axisz);
		set_axis(axisx, axisy, axisz, joint->init_global_position, joint->local);
	}
}

void Model::compute_init_global_position() {
	int number_joint = bvh_.GetNumJoint();
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		BVH::Joint* joint_parent = joint->parent;
		if (joint_parent != nullptr) {
			joint->init_global_position[0] = joint_parent->init_global_position[0]
				+ joint->offset[0];

			joint->init_global_position[1] = joint_parent->init_global_position[1]
				+ joint->offset[1];

			joint->init_global_position[2] = joint_parent->init_global_position[2]
				+ joint->offset[2];
		}
	}
}

void Model::load_faces(char* file) {
	ifstream f;
	f.open(file, ios::in);

	f >> num_faces_;
	faces_ = Eigen::MatrixXi::Zero(num_faces_, 3);
	for (int i = 0; i < num_faces_; i++) {
		f >> faces_(i, 0) >> faces_(i, 1) >> faces_(i, 2);
		//cout<< faces_(i,0) <<" "<< faces_(i,1) <<" "<< faces_(i,2)<<endl;
	}
	f.close();
	//cout<< faces_;
}

void Model::load_vertices(char* file) {
	ifstream f;
	f.open(file, ios::in);
	//int number;
	f >> num_vertices_;
	vertices_ = Eigen::MatrixXd::Zero(num_vertices_, 3);
	for (int i = 0; i < num_vertices_; i++) {
		f >> vertices_(i, 0) >> vertices_(i, 1) >> vertices_(i, 2);
		vertices_(i, 0) = vertices_(i, 0) - 79.446518;      //这里加的是handbone相对于wrist的偏移，由于BVH文件最初是以Handbone为基准的，vertices也是以此为基准的，这里相当于对所有的点改变基准坐标
		vertices_(i, 1) = vertices_(i, 1) + 5.274386;
		vertices_(i, 2) = vertices_(i, 2) - 13.494767;
		//cout<< vertices_(i,0)<<" " << vertices_(i,1)<<" "<<vertices_(i,2)<<endl;
	}
	f.close();
	//cout<< vertices_;
}

void Model::load_weight(char* file) {
	fstream f;
	f.open(file, ios::in);
	//int number = 0;
	f >> num_weight_;
	int x = 0, y = 0;
	int num_joint = bvh_.GetNumJoint();
	weight_ = Eigen::MatrixXd::Zero(num_vertices_, num_joint);
	double weight;
	for (int i = 0; i < num_weight_; i++) {
		f >> x >> y >> weight;
		y = y + 1;
		if (y == 22)
		{
			y = 0;
		}
		if (y == 23)
		{
			y = 22;
		}
		weight_(x, y) = weight;

	}
	//cout<<weight_;
	f.close();
}

void Model::compute_mesh() {

	int num_joint = bvh_.GetNumJoint();
	compute_global();
	Eigen::MatrixXd t = Eigen::MatrixXd::Zero(4, num_vertices_);
	Eigen::MatrixXd x = Eigen::MatrixXd::Ones(4, num_vertices_);
	x.block(0, 0, 3, num_vertices_) = vertices_.block(0, 0, num_vertices_, 3).transpose();
	BVH::Joint* joint = nullptr;
	for (int i = 0; i < num_joint; i++) {
		joint = bvh_.GetJoint(i);
		Eigen::MatrixXd y = weight_.block(0, i, num_vertices_, 1);// 在所有顶点 对于 该关节点的weight
		Eigen::MatrixXd y0 = y.replicate(1, 4);    //分别是行重复1遍，列重复4遍，结果为（num_vertices_，4）这么大小的矩阵
		Eigen::MatrixXd z = joint->scale * joint->global * joint->local_inverse * x;
		t = t + z.cwiseProduct(y0.transpose());
	}
	vertices_update_ = t.transpose();
	//cout<<vertices_update_;

	BVH::Joint* joint_handbone = bvh_.GetJoint(1);
	t = Eigen::MatrixXd(4, 1);
	t(0, 0) = joint_handbone->init_global_position[0];
	t(1, 0) = joint_handbone->init_global_position[1];
	t(2, 0) = joint_handbone->init_global_position[2];
	t(3, 0) = 1; /////joint->scale*
				 /*Eigen::MatrixXd SCALE = Eigen::MatrixXd::Identity(4, 4);
				 SCALE(0, 0) = joint->scale;
				 SCALE(1, 1) = joint->scale;
				 SCALE(2, 2) = joint->scale;*/
	Eigen::MatrixXd I;
	I = joint_handbone->global* (joint_handbone->scale*((joint_handbone->local_inverse)*t));

	for (int i = 0; i < vertices_update_.rows(); i++) {
		vertices_update_(i, 0) += global_position_.x - I(0, 0);
		vertices_update_(i, 1) += global_position_.y - I(1, 0);
		vertices_update_(i, 2) += global_position_.z - I(2, 0);
	}
	//cout<<vertices_update_;
}

/////////////////////////////////////////
/////////////////////////////////////////


void Model::GloveParamsConTrollHand(float *handinf)
{

	Pose global_positon(handinf[24], handinf[25], handinf[26]);
	set_global_position(global_positon);
	Pose p_globle(handinf[15], handinf[16], handinf[17]);
	set_hand_rotation(p_globle);

	//thumb
	Pose p_thumb_lower(handinf[12], handinf[18], handinf[13]);
	Pose p_thumb_middle(0, handinf[19], 0);
	Pose p_thumb_top(0, handinf[14], 0);
	set_thumbLower_rotation(p_thumb_lower);
	set_one_rotation(p_thumb_middle, 19);
	set_one_rotation(p_thumb_top, 20);

	//pinkey
	Pose p_pinkey_lower(0, handinf[0], handinf[1]);
	Pose p_pinkey_middle(0, handinf[2], 0);
	Pose p_pinkey_top(0, handinf[20], 0);
	set_one_rotation(p_pinkey_lower, 10);
	set_one_rotation(p_pinkey_middle, 11);
	set_one_rotation(p_pinkey_middle, 12);

	//ring
	Pose p_ring_lower(0, handinf[3], handinf[4]);
	Pose p_ring_middle(0, handinf[5], 0);
	Pose p_ring_top(0, handinf[21], 0);
	set_one_rotation(p_ring_lower, 14);
	set_one_rotation(p_ring_middle, 15);
	set_one_rotation(p_ring_top, 16);

	//middle
	Pose p_middle_lower(0, handinf[6], handinf[7]);
	Pose p_middle_middle(0, handinf[8], 0);
	Pose p_middle_top(0, handinf[22], 0);
	set_one_rotation(p_middle_lower, 6);
	set_one_rotation(p_middle_middle, 7);
	set_one_rotation(p_middle_top, 8);

	//index
	Pose p_index_lower(0, handinf[9], handinf[10]);
	Pose p_index_middle(0, handinf[11], 0);
	Pose p_index_top(0, handinf[23], 0);
	set_one_rotation(p_index_lower, 2);
	set_one_rotation(p_index_middle, 3);
	set_one_rotation(p_index_top, 4);

}
