#include"CParticle.h"
#include"stdlib.h"  /*stdlib.h中包含rand()函数*/
#include <random>
void CParticle::particleInit(const float* posit)    /*初始位置、初始速度、fitness = 0、个体极值点设为自身、fitness_pbest = -DBL_MAX*/
{
	/*修正position各维度值，使落在预设的参数取值范围内*/
	for (int i = 0; i != dimension; i++)
	{
		if (posit[i] < ParticlerangeLow[i])
		{
			position[i] = ParticlerangeLow[i];
		}
		else if (posit[i] > ParticlerangeUp[i])
		{
			position[i] = ParticlerangeUp[i];
		}
		else
		{
			position[i] = posit[i];
		}

		velocity[i] = (2 * ((float)(rand() % 1001) / 1000.0) - 1)*Vmax[i];
		pbest[i] = position[i];
	}

	fitness_pbest = -FLT_MAX;
	fitness = 0.0f;
}

void CParticle::particleReInit()
{
	default_random_engine e;
	uniform_real_distribution<float> u(-0.5, 0.5);

	for (int i = 0;i < dimension;i++)
	{
		position[i] = initPosition[i] + u(e)*(ParticlerangeUp[i] - ParticlerangeLow[i]);
		velocity[i] = (2 * ((float)(rand() % 1001) / 1000.0) - 1)*Vmax[i];
		pbest[i] = position[i];
	}

	fitness_pbest = -FLT_MAX;
	fitness = 0.0f;

}
/*如果判断出当前粒子位置的fitness值大于pbest，则调用该函数更新pbest和fitness_pbest*/
void CParticle::setPbest(float fit)
{
	//memcpy(pbest, position, dimension * sizeof(float));
	for (int i = 0;i < dimension;i++)
	{
		pbest[i] = position[i];
	}
	fitness_pbest = fit;
}

/*速度更新，weight表示惯性权重（对CFPSO为收敛因子）、factor1为认知因子、factor2为社会因子*/
void CParticle::ParticleUpdate(float weight, float factor1, float factor2, const float *gbest)
{
	for (int v = 0; v != dimension; v++)
	{
		float r1 = (float)(rand() % 1001) / 1000.0;   /*[0、1]之间的随机数*/
		float r2 = (float)(rand() % 1001) / 1000.0;
		velocity[v] = weight*velocity[v] + factor1 * r1 * (pbest[v] - position[v]) + factor2 * r2 * (gbest[v] - position[v]);

		/*最大速度限制，与位置限制(即反射墙)配合使用*/
		if (velocity[v] > Vmax[v]) { velocity[v] = Vmax[v]; }
		if (velocity[v] < -1.0*Vmax[v]) { velocity[v] = -1.0*Vmax[v]; }

		position[v] = position[v] + velocity[v];
		/*反射墙Reflecting Walls*/
		if (position[v] > ParticlerangeUp[v]) { position[v] = 2 * ParticlerangeUp[v] - position[v]; }
		if (position[v] < ParticlerangeLow[v]) { position[v] = 2 * ParticlerangeLow[v] - position[v]; }
	}
}



