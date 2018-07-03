#pragma once
#include<functional>
#include<vector>
#include <iostream>
#include<Windows.h>
#include <algorithm>
#include <fstream>
#include "CParticle.h"
#include"extern.h"
#include"MergeSort.h"

using namespace std;

class APSO
{
public:
	MergeSort *mergesort;
	const int population;  /*粒子总数*/
	const int iteration;   /*迭代数*/
	const int dimension;
	float* bestPosition;  /*最大值点*/
	float* posit_initializer;  //初始化器提供的粒子位置
	vector<CParticle*> CParticle_swarm;
	float *particle_fitness_save;
	float expected_fitness;
	float *APSO_upperBound;
	float *APSO_lowerBound;

public:
	APSO(int popu, int iter,int dim) :population(popu), iteration(iter), dimension(dim), expected_fitness(FLT_MAX)
	{
		mergesort = new MergeSort(population);
		bestPosition = new float[dimension];
		posit_initializer = new float[dimension];
		particle_fitness_save = new float[dimension];
		APSO_upperBound = new float[dimension];
		APSO_lowerBound = new float[dimension];
		CParticle_swarm.resize(population);
		for (vector<CParticle*>::iterator itr = CParticle_swarm.begin();itr != CParticle_swarm.end();itr++)
		{
			*itr = new CParticle(dimension);
		}
	};
	~APSO()
	{
		delete mergesort;
		delete[] bestPosition;
		delete[] posit_initializer;
		delete[] particle_fitness_save;
		delete[] APSO_upperBound;
		delete[] APSO_lowerBound;
		for (vector<CParticle*>::iterator it = CParticle_swarm.begin();it != CParticle_swarm.end();it++)
		{
			if (NULL != *it)
			{
				delete *it;
				*it = NULL;
			}
		}
		CParticle_swarm.clear();
	}
	void  getBestPosByEvolve();
private:
	void swarmInit();

	float fCalculate(int mode);
	int fuzzyDecision(float f, int previous);

};