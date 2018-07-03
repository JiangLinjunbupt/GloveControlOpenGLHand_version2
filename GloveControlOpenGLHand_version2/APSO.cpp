#include"APSO.h"   //使用APSO::
#include <random>

//void  APSO::getBestPosByEvolve()
//{
//	default_random_engine e;
//	uniform_real_distribution<double> u(-0.5, 0.5);
//
//	float inertia = 0.5;    /*惯性权重*/
//	float factor1 = 2;      /*认知因子*/
//	float factor2 = 2;      /*社会因子*/
//	float fit_best = -FLT_MAX;   /*FLT_MAX不是float的最小负值，-FLT_MAX才是*/
//
//	swarmInit();
//
//
//	int it = 0;  /*迭代次数*/
//	while (true)
//	{
//		for (int i = 0; i != population; ++i)
//		{
//			function<void()> task = bind(&CParticle::get_objective_func, CParticle_swarm[i]); //function绑定类的非静态成员函数
//			threadPool.run(task);
//			/*sut->get_fitness(swarm[i]->position, ref(swarm[i]->fitness));
//			this->particle_fitness_save[i] = swarm[i]->fitness;*/
//		}
//
//		threadPool.ensureTaskCompleted(population);
//		cout << "threadpool over!" << endl;
//
//
//		//计算每个粒子的pbest、fitness_pbest、fitness，更新全局极值bestPosition[]数组、全局极值fit_best
//		for (int i = 0; i < this->population; i++)
//		{
//			float fit = CParticle_swarm[i]->fitness;
//			if (fit >= expected_fitness)
//			{
//				cout << "达到最大适应度，停止迭代" << endl;
//				for (int k = 0;k < dimension;k++)
//				{
//					bestPosition[k] = CParticle_swarm[i]->position[k];
//				}
//				//memcpy(bestPosition, Cparticle_swarm[i]->position, sut->dimension * sizeof(float));
//				return;
//			}
//			if (fit > CParticle_swarm[i]->fitness_pbest)  /*更新粒子i的pbest、fitness_pbest*/
//			{
//				CParticle_swarm[i]->setPbest(fit);
//			}
//			if (fit > fit_best)
//			{
//				fit_best = fit;
//				for (int k = 0;k < dimension;k++)
//				{
//					bestPosition[k] = CParticle_swarm[i]->position[k];
//				}
//				//memcpy(bestPosition, Cparticle_swarm[i]->position, sut->dimension * sizeof(float));
//			}
//		}
//
//		cout << "第 " << it << " 代的best fit is :" << fit_best << endl;
//
//		if (it == iteration)
//		{
//			printf("第%d代粒子群的最优适应值fit_best分别是：% 8.4lf\n", it, fit_best);
//			break;
//		}
//
//		for (int i = 0; i < this->population; i++)
//		{
//			CParticle_swarm[i]->velocityUpdate(inertia, factor1, factor2, bestPosition);
//			CParticle_swarm[i]->positionUpdate();
//		}
//
//		cout << "粒子更新完成" << endl;
//
//		it++;
//	} // end while
//}
void  APSO::getBestPosByEvolve()
{

	float inertia = 0.9;    /*惯性权重*/
	float factor1 = 2.0;      /*认知因子*/
	float factor2 = 2.0;      /*社会因子*/
	float factor_max = 2.5;
	float factor_min = 1.5;
	float factorSum_max = 4.0;

	float fit_best = -FLT_MAX;   /*FLT_MAX不是float的最小负值，-FLT_MAX才是*/

	swarmInit();

	
	int it = 0;  /*迭代次数*/
	float f_value = 0;   /*状态因子*/
	int state = 1;        /*粒子的四个阶段，初始时状态因子为0，在搜索阶段*/
	const double sigma_max = 1.0;  /*更新gbest时用到的参数sigma_max、sigma_min*/
	const double sigma_min = 0.1;

	while (true)
	{
		for (int i = 0; i != population; ++i)
		{
			function<void()> task = bind(&CParticle::get_objective_func, CParticle_swarm[i]); //function绑定类的非静态成员函数
			threadPool.run(task);
			/*sut->get_fitness(swarm[i]->position, ref(swarm[i]->fitness));
			this->particle_fitness_save[i] = swarm[i]->fitness;*/
		}

		threadPool.ensureTaskCompleted(population);
		cout << "threadpool over!" << endl;


		//计算每个粒子的pbest、fitness_pbest、fitness，更新全局极值bestPosition[]数组、全局极值fit_best
		for (int i = 0; i < this->population; i++)
		{
			float fit = CParticle_swarm[i]->fitness;
			if (fit >= expected_fitness)
			{
				cout << "达到最大适应度，停止迭代" << endl;
				for (int k = 0;k < dimension;k++)
				{
					bestPosition[k] = CParticle_swarm[i]->position[k];
				}
				//memcpy(bestPosition, Cparticle_swarm[i]->position, sut->dimension * sizeof(float));
				return;
			}
			if (fit > CParticle_swarm[i]->fitness_pbest)  /*更新粒子i的pbest、fitness_pbest*/
			{
				CParticle_swarm[i]->setPbest(fit);
			}
			if (fit > fit_best)
			{
				fit_best = fit;
				for (int k = 0;k < dimension;k++)
				{
					bestPosition[k] = CParticle_swarm[i]->position[k];
				}
				//memcpy(bestPosition, Cparticle_swarm[i]->position, sut->dimension * sizeof(float));
			}
		}

		f_value = fCalculate(2);  //mode=1:全局位移L2范数+角度L1范数; mode=2：L2范数; mode=3：适应值之差的绝对值
		state = fuzzyDecision(f_value, state);

		cout << "第 " << it << " 代的best fit is :" << fit_best << endl;
		cout << "f_value : " << f_value << "  state : " << state << endl;

		if (it == iteration)
		{
			printf("第%d代粒子群的最优适应值fit_best分别是：% 8.4lf\n", it, fit_best);
			break;
		}

		inertia = 1.0 / (1.0 + 1.5 * exp(-2.6 * f_value));

		if (state == 1) /*探索*/
		{
			factor1 = factor1 + (0.05 + (float)(rand() % 51) / 1000.0);  /*[0.05,0.1]*/
			factor2 = factor2 - (0.05 + (float)(rand() % 51) / 1000.0);
		}
		else if (state == 2) /*开发*/
		{
			factor1 = factor1 + 0.5 * (0.05 + (float)(rand() % 51) / 1000.0);    /*[0.025,0.05]*/
			factor2 = factor2 - 0.5 * (0.05 + (float)(rand() % 51) / 1000.0);
		}
		else if (state == 3) /*收敛*/
		{
			factor1 = factor1 + 0.5 * (0.05 + (float)(rand() % 51) / 1000.0);
			factor2 = factor2 + 0.5 * (0.05 + (float)(rand() % 51) / 1000.0);
		}
		else if (state == 4)  /*跳出*/
		{
			factor1 = factor1 - (0.05 + (float)(rand() % 51) / 1000.0);
			factor2 = factor2 + (0.05 + (float)(rand() % 51) / 1000.0);
		}
		else
		{
			cout << "state is :" << state << endl;
			cout << "粒子群所处状态不合理" << endl;
			//exit(EXIT_FAILURE);
		}

		if (factor1 > factor_max) { factor1 = factor_max; }
		if (factor1 < factor_min) { factor1 = factor_min; }
		if (factor2 > factor_max) { factor2 = factor_max; }
		if (factor2 < factor_min) { factor2 = factor_min; }

		if (factor1 + factor2 > factorSum_max)
		{
			factor1 = factorSum_max / (factor1 + factor2)*factor1;
			factor2 = factorSum_max / (factor1 + factor2)*factor2;
		}

		cout << "inertia : " << inertia << "  factor1 : " << factor1 << "   factor2 : " << factor2 << endl;

		for (int i = 0; i < this->population; i++)
		{
			//CParticle_swarm[i]->ParticleUpdate(inertia, factor1, factor2, bestPosition);
			function<void()> task = bind(&CParticle::ParticleUpdate, CParticle_swarm[i],ref(inertia),ref(factor1),ref(factor2), bestPosition); //function绑定类的非静态成员函数
			threadPool.run(task);
		}
		threadPool.ensureTaskCompleted(population);
		cout << "粒子更新完成" << endl;
		cout << endl;

		it++;
	} // end while
}

void APSO::swarmInit()
{

	CParticle_swarm[0]->particleInit(posit_initializer);
	float* recommend = new float[dimension];
	for (int i = 1; i < population; i++)
	{
		for (int v = 0; v < dimension; v++)
		{
			//这里我准备使用均匀撒点来做初始化
			recommend[v] = APSO_lowerBound[v] + (float)i*(APSO_upperBound[v] - APSO_lowerBound[v]) / (float)population;
		}
		CParticle_swarm[i]->particleInit(recommend);
	}
	delete[]recommend;
}


float APSO::fCalculate(int mode)
{
	/*
	mode=1:全局位移L2范数+角度L1范数
	mode=2：L2范数
	mode=3：适应值之差的绝对值
	*/
	float min_dis = FLT_MAX;  /*平均距离的最小值*/
	float max_dis = -FLT_MAX;  /*平均距离的最大值*/
	float g_dis = 0;          /*全局最优粒子到其他所有粒子的平均距离*/

	int row = population;
	int col = population;
	float ** a = new float *[row];
	for (int i = 0; i < row; i++)
		a[i] = new float[col];

	for (int i = 0;i < row;i++)
	{
		for (int j = 0;j < col;j++)
		{
			a[i][j] = 0;
		}
	}
	if (mode == 1)
	{
		;
	}
	else if (mode == 2)
	{
		for (int i = 0; i < row; i++)   /*计算a[][]矩阵的上三角元素*/
		{
			for (int j = i + 1; j < col; j++)
			{
				float tmp = 0;
				for (int k = 0; k < dimension; k++)
				{
					tmp += (CParticle_swarm[i]->position[k] - CParticle_swarm[j]->position[k])*(CParticle_swarm[i]->position[k] - CParticle_swarm[j]->position[k]);
				}
				a[i][j] = sqrt(tmp);
				//cout << a[i][j] << endl;
			}
		}
	}
	else if (mode == 3)
	{
		;
	}
	else
	{
		cout << "model is : " << mode << endl;
		cout << "不存在该粒子距离计算模式" << endl;
		//exit(EXIT_FAILURE);
	}

	for (int i = 0; i < row; i++)
	{
		float meanDist = 0;
		for (int j = 0; j < i; j++)
			meanDist += a[j][i];
		for (int j = i + 1; j < col; j++)
			meanDist += a[i][j];
		meanDist = meanDist / (float)(col - 1);

		if (meanDist < min_dis) { min_dis = meanDist; }
		if (meanDist > max_dis) { max_dis = meanDist; }
	}


	cout << "max dis :" << max_dis << "    min_dis: " << min_dis << endl;
	/*计算全局最优粒子到其它粒子的平均距离*/
	if (mode == 1)
	{
		;
	}
	else if (mode == 2)
	{
		for (auto i : CParticle_swarm)
		{
			float tmp = 0;
			for (int k = 0; k < dimension; k++)
			{
				tmp += (i->position[k] - bestPosition[k])*(i->position[k] - bestPosition[k]);
			}
			g_dis += sqrt(tmp);
		}
		g_dis = g_dis / (float)(population);
	}
	else if (mode == 3)
	{
		;
	}
	else
	{
		cout << "不存在该粒子距离计算模式" << endl;
		//exit(EXIT_FAILURE);
	}


	if (g_dis < min_dis) { min_dis = g_dis; }
	if (g_dis > max_dis) { max_dis = g_dis; }

	/*释放空间*/
	for (int i = 0; i < row; i++)
		delete[col] a[i];
	delete[row] a;

	cout << "max dis :" << max_dis << "    min_dis: " << min_dis << endl;
	return (g_dis - min_dis) / (max_dis - min_dis);
}


int APSO::fuzzyDecision(float f, int previous)
{
	/*
	搜索阶段 S1 = exploration
	利用阶段 S2 = exploitation
	收敛阶段 S3 = convergence
	跳出阶段 S4 = jumping-out
	*/
	if (f <= 0.2)
		return 3;

	if (0.2 < f && f < 0.3)
	{
		if (f < (float)7.0 / (float)30.0)
			return (previous == 1 || previous == 2) ? 2 : 3;
		else
			return (previous == 3) ? 3 : 2;
	}

	if (0.3 <= f && f <= 0.4)
		return 2;

	if (0.4 < f && f < 0.6)
	{
		if (f < 0.5)
			return (previous == 1 || previous == 4) ? 1 : 2;
		else
			return (previous == 2) ? 2 : 1;
	}

	if (0.6 <= f && f <= 0.7)
		return 1;

	if (0.7 < f && f < 0.8)
	{
		if (f < (float)23.0 / (float)30.0)
			return (previous == 4 || previous == 3) ? 4 : 1;
		else
			return  (previous == 1) ? 1 : 4;
	}

	if (f >= 0.8)
		return 4;

	return -1;
}