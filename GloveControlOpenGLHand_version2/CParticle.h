#pragma once
#include<iostream>
#include<vector>
#include<math.h>
#include"Projection.h"
#include "extern.h"
using namespace std;

class CParticle
{
public:
	CParticle() {}
	CParticle(int dim) : dimension(dim)
	{
		initPosition = new float[dimension];
		Vmax = new float[dimension];

		position = new float[dimension];
		velocity = new float[dimension];
		pbest = new float[dimension];
		ParticlerangeLow = new float[dimension];
		ParticlerangeUp = new float[dimension];

		Generated_mat = cv::Mat::zeros(424, 512, CV_16UC1);

		particle_model = new Model(".\\model\\HandBase.bvh");
		particle_model->init();
		particle_project = new Projection(424, 512);
	};


	~CParticle()
	{
		delete[] initPosition;
		delete[] Vmax;
		delete[] position;
		delete[] velocity;
		delete[] pbest;
		delete[] ParticlerangeLow;
		delete[] ParticlerangeUp;
	}
private:
	int dimension;         /*the number of dimensions,类中的常数据成员只能通过构造函数的初始化表对其初始化*/
	float *initPosition;
	float *Vmax; /*粒子速度第i维分量的范围：闭区间[-Vmax[i],Vmax[i]]*/

public:
	float *position;
	float *velocity;
	float fitness;
	float *pbest;//个体极值点
	float fitness_pbest;//个体极值
	float *ParticlerangeLow;      /*rangeLow[dimension]数组，position空间的第i维分量的范围的最小取值*/
	float *ParticlerangeUp;       /*rangeUp[dimension]数组，position空间的第i维分量的范围的最大取值*/

public:
	cv::Mat Generated_mat;
	Model *particle_model;
	Projection *particle_project;


public:
	void particleInit(const float* posit);
	void particleReInit();
	void setPbest(float fit);   /*如果判断出当前粒子位置的fitness值大于pbest，则调用该函数更新pbest和fitness_pbest*/
	void ParticleUpdate(float weight, float factor1, float factor2, const float *gbest);   /*更新速度*/
	void CParticle_Paramsinit(float* Low, float* Up, const float *initpose)
	{
		for (int i = 0; i < dimension; i++)
		{
			initPosition[i] = initpose[i];
			ParticlerangeLow[i] = Low[i];
			ParticlerangeUp[i] = Up[i];
			Vmax[i] = (ParticlerangeUp[i] - ParticlerangeLow[i]) * 0.15f;   /*论文Adaptive particle swarm optimization，IEEE Transactions中采用的值*/
		}
	};
	void  get_objective_func()
	{
		//计算两幅图对应像素的插值-------------------------------------
		particle_model->GloveParamsConTrollHand(this->position);
		particle_model->forward_kinematic();
		particle_model->compute_mesh();

		particle_project->set_color_index(particle_model);
		particle_project->project_3d_to_2d_(particle_model, Generated_mat);

		float E_golden = 0.0f;
		float threshold = 100.0f;      //门限设置位100mm
		for (int i = 0; i < Generated_mat.rows; i++)
		{
			for (int j = 0; j < Generated_mat.cols; j++)
			{
				float difference = abs(Generated_mat.at<ushort>(i, j) - Input_depthMat.at<ushort>(i, j));
				E_golden += difference < threshold ? pow(difference, 2) : pow(threshold, 2);
			}
		}
		E_golden = sqrt(E_golden);

		this->fitness = -E_golden;
	}
};







