#include"CParticle.h"
#include"stdlib.h"  /*stdlib.h�а���rand()����*/
#include <random>
void CParticle::particleInit(const float* posit)    /*��ʼλ�á���ʼ�ٶȡ�fitness = 0�����弫ֵ����Ϊ����fitness_pbest = -DBL_MAX*/
{
	/*����position��ά��ֵ��ʹ����Ԥ��Ĳ���ȡֵ��Χ��*/
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
/*����жϳ���ǰ����λ�õ�fitnessֵ����pbest������øú�������pbest��fitness_pbest*/
void CParticle::setPbest(float fit)
{
	//memcpy(pbest, position, dimension * sizeof(float));
	for (int i = 0;i < dimension;i++)
	{
		pbest[i] = position[i];
	}
	fitness_pbest = fit;
}

/*�ٶȸ��£�weight��ʾ����Ȩ�أ���CFPSOΪ�������ӣ���factor1Ϊ��֪���ӡ�factor2Ϊ�������*/
void CParticle::ParticleUpdate(float weight, float factor1, float factor2, const float *gbest)
{
	for (int v = 0; v != dimension; v++)
	{
		float r1 = (float)(rand() % 1001) / 1000.0;   /*[0��1]֮��������*/
		float r2 = (float)(rand() % 1001) / 1000.0;
		velocity[v] = weight*velocity[v] + factor1 * r1 * (pbest[v] - position[v]) + factor2 * r2 * (gbest[v] - position[v]);

		/*����ٶ����ƣ���λ������(������ǽ)���ʹ��*/
		if (velocity[v] > Vmax[v]) { velocity[v] = Vmax[v]; }
		if (velocity[v] < -1.0*Vmax[v]) { velocity[v] = -1.0*Vmax[v]; }

		position[v] = position[v] + velocity[v];
		/*����ǽReflecting Walls*/
		if (position[v] > ParticlerangeUp[v]) { position[v] = 2 * ParticlerangeUp[v] - position[v]; }
		if (position[v] < ParticlerangeLow[v]) { position[v] = 2 * ParticlerangeLow[v] - position[v]; }
	}
}



