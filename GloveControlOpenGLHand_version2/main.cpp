#define NDEBUG
#include "GL/freeglut.h"         //OpenGL的头文件Includ顺序会导致出现编译错误
#include <windows.h>
#include <string.h>
#include <string>
#include <tchar.h>
#include "extern.h"
#include "myKinect.h"
#include "Model.h"
#include "Projection.h"
#include "PointCloud.h"
#include "Viewer.h"
#include "MergeSort.h"
#include "APSO.h"
//#include "GL/freeglut.h"                //这个OpenGL的包含文件放在最后就会出现编译错误

//extern参数定义
cv::Mat Input_depthMat = cv::Mat::zeros(424, 512, CV_16UC1);
ThreadPool threadPool;

//共享内存的相关定义
HANDLE hMapFile;
LPCTSTR pBuf;
#define BUF_SIZE 1024
TCHAR szName[] = TEXT("Global\\MyFileMappingObject");    //指向同一块共享内存的名字
float *GetSharedMemeryPtr;
float *GetGloveData = new float[27];

//main函数中用到的model和projection定义
Model *model = nullptr;
Projection *projection = new Projection(424, 512);

//OpenGL相关定义
VisData _data;
Config config;
Control control;
bool begain_PSO = false;
bool Change_with_glove = true;

//PSO参数设置
const int ParticleDim = 27;        //粒子维数
const int PSOpopulation = 100;     //粒子数量
const int PSOiteration = 50;       //迭代次数
float *PSO_upper_bound = new float[ParticleDim];
float *PSO_lower_bound = new float[ParticleDim];

APSO pso(PSOpopulation, PSOiteration, ParticleDim);

void MixShowResult(cv::Mat input1, cv::Mat input2);
void reset_upper_lower_Bound(float *original_upper, float *original_lower,
	const float *init_params,
	float *output_upper, float *output_lower);
void poseEstimate(const float *initParams, float *upper, float *lower, float* output_dof);

#pragma region OpenGL

#pragma region  Keybroad_event(show mesh or not)

void menu(int op) {

	switch (op) {
	case 'Q':
	case 'q':
		exit(0);
	}
}

/* executed when a regular key is pressed */
void keyboardDown(unsigned char key, int x, int y) {

	switch (key) {
	case 'q':
		config.show_mesh = true;
		config.show_point = false;
		config.show_skeleton = false;
		break;
	case 'w':
		config.show_mesh = false;
		config.show_point = true;
		config.show_skeleton = true;
		break;
	case 'b':
		begain_PSO = true;
		Change_with_glove = false;
		break;
	case 'e':
		begain_PSO = false;
		break;
	case  27:   // ESC
		exit(0);
	}
}

/* executed when a regular key is released */
void keyboardUp(unsigned char key, int x, int y) {

}

/* executed when a special key is pressed */
void keyboardSpecialDown(int k, int x, int y) {

}

/* executed when a special key is released */
void keyboardSpecialUp(int k, int x, int y) {

}
#pragma endregion  Keybroad_event(show mesh or not)


/* reshaped window */
void reshape(int width, int height) {

	GLfloat fieldOfView = 90.0f;
	glViewport(0, 0, (GLsizei)width, (GLsizei)height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fieldOfView, (GLfloat)width / (GLfloat)height, 0.1, 500.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/* executed when button 'button' is put into state 'state' at screen position ('x', 'y') */
void mouseClick(int button, int state, int x, int y) {
	control.mouse_click = 1;
	control.x = x;
	control.y = y;
}

/* executed when the mouse moves to position ('x', 'y') */
/* render the scene */
void draw() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	gluPerspective(180, 1.5, -1000, 1000);
	glLoadIdentity();
	control.gx = model->get_global_position().x;
	control.gy = model->get_global_position().y;
	control.gz = model->get_global_position().z;
	double r = 200;
	double x = r*sin(control.roty)*cos(control.rotx);
	double y = r*sin(control.roty)*sin(control.rotx);
	double z = r*cos(control.roty);
	//cout<< x <<" "<< y <<" " << z<<endl;
	gluLookAt(x + control.gx, y + control.gy, z + control.gz, control.gx, control.gy, control.gz, 0.0, 1.0, 0.0);//个人理解最开始是看向-z的，之后的角度是在global中心上叠加的，所以要加


																												 //画点云，必须放最前面，要是放后面会出错。为什么我也没找到~   -------原因：以前glEnd没加(),程序不会报错，但是运行结果不对。；
	if (pointcloud.pointcloud_vector.size() > 0)
	{
		glPointSize(2);
		glBegin(GL_POINTS);
		glColor3d(0.0, 1.0, 0.0);
		//cout << "the pointcloud size : " << pointcloud.pointcloud_vector.size() << endl;
		for (int i = 0;i < pointcloud.pointcloud_vector.size();i++)
		{
			glVertex3f(pointcloud.pointcloud_vector[i].x, pointcloud.pointcloud_vector[i].y, pointcloud.pointcloud_vector[i].z);
		}
		glEnd();
	}
	/* render the scene here */

	if (config.show_point) {
		glPointSize(2);
		glBegin(GL_POINTS);
		glColor3d(1.0, 0.0, 0.0);
		for (int i = 0; i < model->vertices_update_.rows(); i++) {
			double t = -1e10;
			int idx = 0;
			for (int j = 0; j < model->weight_.cols(); j++) {
				if (t < model->weight_(i, j)) {
					t = model->weight_(i, j);
					idx = j;
				}
			}
			if (idx != 0)         //不显示wrist控制的那些属于arm的点
			{
				glVertex3d(model->vertices_update_(i, 0), model->vertices_update_(i, 1), model->vertices_update_(i, 2));
			}
			//glVertex3d(model->vertices_update_(i, 0), model->vertices_update_(i, 1), model->vertices_update_(i, 2));
			//cout<< model->vertices_(i,0)<< " " << model->vertices_(i,1) <<" "<< model->vertices_(i,2)<<endl;
		}
		glEnd();
	}

	if (config.show_mesh) {
		if (_data.indices == nullptr) return;
		if (_data.vertices == nullptr) return;
		glColor3d(0.0, 0.0, 1.0);
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, _data.vertices);
		glEnableClientState(GL_COLOR_ARRAY);
		glColorPointer(3, GL_FLOAT, 0, _data.colors);
		//glDrawElements(GL_TRIANGLE_STRIP, 12, GL_UNSIGNED_BYTE, indices);
		glDrawElements(GL_TRIANGLES, 3 * _data.num_face, GL_UNSIGNED_INT, _data.indices);

		// deactivate vertex arrays after drawing
		glDisableClientState(GL_VERTEX_ARRAY);
	}
	//glEnable(GL_LIGHTING);
	if (config.show_skeleton) {
		for (int i = 0; i < _data.joints.rows(); i++) {
			//画点开始   //不画arm那个节点
			if (i != 22)
			{
				glColor3f(1.0, 0.0, 0.0);
				glPushMatrix();
				glTranslatef(_data.joints(i, 0), _data.joints(i, 1), _data.joints(i, 2));
				glutSolidSphere(5, 31, 10);
				glPopMatrix();
			}
			//画点结束，使用push和popmatrix是因为保证每个关节点的偏移都是相对于全局坐标中心点做的变换。

			//画线开始  //不画wrist到arm的那条线
			if (i != 0&&i!=22) {
				glLineWidth(5);
				glColor3f(0.0, 1.0, 0);
				glBegin(GL_LINES);
				int ii = _data.joints(i, 3);
				glVertex3f(_data.joints(ii, 0), _data.joints(ii, 1), _data.joints(ii, 2));
				glVertex3f(_data.joints(i, 0), _data.joints(i, 1), _data.joints(i, 2));
				glEnd();
			}

			//画线结束
		}
	}

	glFlush();
	glutSwapBuffers();
}


void mouseMotion(int x, int y) {
	control.rotx = (x - control.x)*0.05;
	control.roty = (y - control.y)*0.05;

	//cout<< control.rotx <<" " << control.roty << endl;
	glutPostRedisplay();
}

/* executed when program is idle */
void idle() {
	if (begain_PSO)
	{
		mykinect.Collectdata();
		pointcloud.DepthMatToPointCloud(mykinect.HandsegmentMat);

		for (int i = 0; i < 24; i++)
		{
			GetGloveData[i] = GetSharedMemeryPtr[i];
		}
		GetGloveData[24] = pointcloud.PointCloud_center_x;
		GetGloveData[25] = pointcloud.PointCloud_center_y;
		GetGloveData[26] = pointcloud.PointCloud_center_z;

		mykinect.HandsegmentMat.copyTo(Input_depthMat);
		poseEstimate(GetGloveData, model->upper_bound, model->lower_bound, model->OptimizedParams);

		model->GloveParamsConTrollHand(model->OptimizedParams);
		model->forward_kinematic();
		model->compute_mesh();

		cv::Mat generated_mat = cv::Mat::zeros(424, 512, CV_16UC1);
		projection->set_color_index(model);
		projection->project_3d_to_2d_(model, generated_mat);
		MixShowResult(mykinect.HandsegmentMat, generated_mat);

		_data.set(model->vertices_update_, model->faces_);
		_data.set_color(model->weight_);
		_data.set_skeleton(model);

		begain_PSO = false;
		glutPostRedisplay();
	}
	if (Change_with_glove)
	{
		mykinect.Collectdata();
		pointcloud.DepthMatToPointCloud(mykinect.HandsegmentMat);


		for (int i = 0; i < 24; i++)
		{
			GetGloveData[i] = GetSharedMemeryPtr[i];
		}
		GetGloveData[24] = pointcloud.PointCloud_center_x;
		GetGloveData[25] = pointcloud.PointCloud_center_y;
		GetGloveData[26] = pointcloud.PointCloud_center_z;

		mykinect.HandsegmentMat.copyTo(Input_depthMat);
		model->GloveParamsConTrollHand(GetGloveData);
		model->forward_kinematic();
		model->compute_mesh();

		cv::Mat generated_mat = cv::Mat::zeros(424, 512, CV_16UC1);
		projection->set_color_index(model);
		projection->project_3d_to_2d_(model, generated_mat);
		MixShowResult(mykinect.HandsegmentMat, generated_mat);


		float E_golden = 0.0;
		float threshold = 100.0;      //门限设置位100mm
		for (int i = 0; i < generated_mat.rows; i++)
		{
			for (int j = 0; j < generated_mat.cols; j++)
			{
				float difference = abs(generated_mat.at<ushort>(i, j) - mykinect.HandsegmentMat.at<ushort>(i, j));
				E_golden += difference < threshold ? pow(difference, 2) : pow(threshold, 2);
			}
		}
		E_golden = sqrt(E_golden);

		cout << -E_golden << endl;

		_data.set(model->vertices_update_, model->faces_);
		_data.set_color(model->weight_);
		_data.set_skeleton(model);

		glutPostRedisplay();
	}

	if (!Change_with_glove && !begain_PSO)
	{
		glutPostRedisplay();
	}
}

/* initialize OpenGL settings */
void initGL(int width, int height) {

	reshape(width, height);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
}

#pragma endregion 
int main(int argc, char** argv)
{
#pragma region SharedMemery
	hMapFile = CreateFileMapping(
		INVALID_HANDLE_VALUE,    // use paging file
		NULL,                    // default security
		PAGE_READWRITE,          // read/write access
		0,                       // maximum object size (high-order DWORD)
		BUF_SIZE,                // maximum object size (low-order DWORD)
		szName);                 // name of mapping object

	if (hMapFile == NULL)
	{
		_tprintf(TEXT("Could not create file mapping object (%d).\n"),
			GetLastError());
		return 1;
	}
	pBuf = (LPTSTR)MapViewOfFile(hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,
		0,
		BUF_SIZE);

	if (pBuf == NULL)
	{
		_tprintf(TEXT("Could not map view of file (%d).\n"),
			GetLastError());

		CloseHandle(hMapFile);

		return 1;
	}

	GetSharedMemeryPtr = (float*)pBuf;
#pragma endregion SharedMemery

#pragma region ThreadPoolinit
	threadPool.setMaxQueueSize(100);
	int hardware_thread = thread::hardware_concurrency();
	if (hardware_thread == 0) { hardware_thread = 2; } //当系统信息无法获取时，函数会返回0
	int threadNum = hardware_thread - 1;
	cout << "threadNum is：" << threadNum << endl;
	threadPool.start(threadNum);  //启动线程池，创建多个线程，从任务队列取任务作为线程入口函数，取任务时，需要判断队列是否非空
#pragma endregion ThreadPoolinit

	mykinect.InitializeDefaultSensor();
	model = new Model(".\\model\\HandBase.bvh");
	model->init();


	pointcloud.pointcloud_vector.clear();
	_data.init(model->vertices_.rows(), model->faces_.rows());
	_data.set(model->vertices_update_, model->faces_);
	_data.set_color(model->weight_);
	_data.set_skeleton(model);

#pragma region OpenGL
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Interactron");

	// register glut call backs
	glutKeyboardFunc(keyboardDown);
	glutKeyboardUpFunc(keyboardUp);
	glutSpecialFunc(keyboardSpecialDown);
	glutSpecialUpFunc(keyboardSpecialUp);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMotion);
	glutReshapeFunc(reshape);
	glutDisplayFunc(draw);
	glutIdleFunc(idle);
	glutIgnoreKeyRepeat(true); // ignore keys held down

	// create a sub menu 
	int subMenu = glutCreateMenu(menu);
	glutAddMenuEntry("Do nothing", 0);
	glutAddMenuEntry("Really Quit", 'q');

	// create main "right click" menu
	glutCreateMenu(menu);
	glutAddSubMenu("Sub Menu", subMenu);
	glutAddMenuEntry("Quit", 'q');
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	initGL(800, 600);

	glutMainLoop();

#pragma endregion

	return 0;
}


void MixShowResult(cv::Mat input1, cv::Mat input2)
{
	int height = input2.rows;
	int width = input2.cols;
	cv::Mat colored_input1 = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat colored_input2 = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat dst;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (input1.at<ushort>(i, j) != 0)
			{
				colored_input1.at < cv::Vec3b>(i, j)[0] = 0;
				colored_input1.at < cv::Vec3b>(i, j)[1] = 0;
				colored_input1.at < cv::Vec3b>(i, j)[2] = 255;
			}
			else
			{

				colored_input1.at < cv::Vec3b>(i, j)[0] = 255;
				colored_input1.at < cv::Vec3b>(i, j)[1] = 255;
				colored_input1.at < cv::Vec3b>(i, j)[2] = 255;

			}

			if (input2.at<ushort>(i, j) != 0)
			{
				colored_input2.at < cv::Vec3b>(i, j)[0] = 0;
				colored_input2.at < cv::Vec3b>(i, j)[1] = 255;
				colored_input2.at < cv::Vec3b>(i, j)[2] = 0;
			}
			else
			{

				colored_input2.at < cv::Vec3b>(i, j)[0] = 255;
				colored_input2.at < cv::Vec3b>(i, j)[1] = 255;
				colored_input2.at < cv::Vec3b>(i, j)[2] = 255;

			}

		}
	}

	cv::addWeighted(colored_input1, 0.5, colored_input2, 0.5, 0.0, dst);
	cv::imshow("Mixed Result", dst);

}


void reset_upper_lower_Bound(float *original_upper, float *original_lower,
	const float *init_params,
	float *output_upper, float *output_lower)
{
	//pinkey
	output_upper[0] = (init_params[0] + 20)<original_upper[0] ? (init_params[0] + 20) : original_upper[0]; output_lower[0] = (init_params[0] - 20)>original_lower[0] ? (init_params[0] - 20) : original_lower[0];    //low  //弯曲
	output_upper[1] = (init_params[1] + 20)<original_upper[1] ? (init_params[1] + 20) : original_upper[1]; output_lower[1] = (init_params[1] - 20)>original_lower[1] ? (init_params[1] - 20) : original_lower[1];           //左右
	output_upper[2] = (init_params[2] + 20)<original_upper[2] ? (init_params[2] + 20) : original_upper[2]; output_lower[2] = (init_params[2] - 20)>original_lower[2] ? (init_params[2] - 20) : original_lower[2];     //middle
	output_upper[20] = (init_params[20] + 20)<original_upper[20] ? (init_params[20] + 20) : original_upper[20]; output_lower[20] = (init_params[20] - 20)>original_lower[20] ? (init_params[20] - 20) : original_lower[20];   //top

	//ring
	output_upper[3] = (init_params[3] + 20) < original_upper[3] ? (init_params[3] + 20) : original_upper[3]; output_lower[3] = (init_params[3] - 20) > original_lower[3] ? (init_params[3] - 20) : original_lower[3];   //low     //弯曲
	output_upper[4] = (init_params[4] + 20) < original_upper[4] ? (init_params[4] + 20) : original_upper[4]; output_lower[4] = (init_params[4] - 20) > original_lower[4] ? (init_params[4] - 20) : original_lower[4];             //左右
	output_upper[5] = (init_params[5] + 20) < original_upper[5] ? (init_params[5] + 20) : original_upper[5]; output_lower[5] = (init_params[5] - 20) > original_lower[5] ? (init_params[5] - 20) : original_lower[5];     //middle
	output_upper[21] = (init_params[21] + 20) < original_upper[21] ? (init_params[21] + 20) : original_upper[21]; output_lower[21] = (init_params[21] - 20) > original_lower[21] ? (init_params[21] - 20) : original_lower[21];   //top

	//middle
	output_upper[6] = (init_params[6] + 20) < original_upper[6] ? (init_params[6] + 20) : original_upper[6]; output_lower[6] = (init_params[6] - 20) > original_lower[6] ? (init_params[6] - 20) : original_lower[6];    //low      //弯曲
	output_upper[7] = 0.0; output_lower[7] = 0.0;               //左右
	output_upper[8] = (init_params[8] + 20) < original_upper[8] ? (init_params[8] + 20) : original_upper[8]; output_lower[8] = (init_params[8] - 20) > original_lower[8] ? (init_params[8] - 20) : original_lower[8];     //middle
	output_upper[22] = (init_params[22] + 20) < original_upper[22] ? (init_params[22] + 20) : original_upper[22]; output_lower[22] = (init_params[22] - 20) > original_lower[22] ? (init_params[22] - 20) : original_lower[22];    //top

	//index
	output_upper[9] = (init_params[9] + 20) < original_upper[9] ? (init_params[9] + 20) : original_upper[9];  output_lower[9] = (init_params[9] - 20) > original_lower[9] ? (init_params[9] - 20) : original_lower[9];     //low      //弯曲
	output_upper[10] = (init_params[10] + 20) < original_upper[10] ? (init_params[10] + 20) : original_upper[10]; output_lower[10] = (init_params[10] - 20) > original_lower[10] ? (init_params[10] - 20) : original_lower[10];              //左右
	output_upper[11] = (init_params[11] + 20) < original_upper[11] ? (init_params[11] + 20) : original_upper[11]; output_lower[11] = (init_params[11] - 20) > original_lower[11] ? (init_params[11] - 20) : original_lower[11];    //middle
	output_upper[23] = (init_params[23] + 20) < original_upper[23] ? (init_params[23] + 20) : original_upper[23]; output_lower[23] = (init_params[23] - 20) > original_lower[23] ? (init_params[23] - 20) : original_lower[23];    //top

	//thumb
	output_upper[12] = (init_params[12] + 20) < original_upper[12] ? (init_params[12] + 20) : original_upper[12]; output_lower[12] = (init_params[12] - 20) > original_lower[12] ? (init_params[12] - 20) : original_lower[12];            //low x
	output_upper[13] = (init_params[13] + 20) < original_upper[13] ? (init_params[13] + 20) : original_upper[13]; output_lower[13] = (init_params[13] - 20) > original_lower[13] ? (init_params[13] - 20) : original_lower[13];            //low y
	output_upper[18] = (init_params[18] + 20) < original_upper[18] ? (init_params[18] + 20) : original_upper[18]; output_lower[18] = (init_params[18] - 20) > original_lower[18] ? (init_params[18] - 20) : original_lower[18];            //low z

	output_upper[14] = (init_params[14] + 20) < original_upper[14] ? (init_params[14] + 20) : original_upper[14]; output_lower[14] = (init_params[14] - 20) > original_lower[14] ? (init_params[14] - 20) : original_lower[14];           //top
	output_upper[19] = (init_params[19] + 20) < original_upper[19] ? (init_params[19] + 20) : original_upper[19]; output_lower[19] = (init_params[19] - 20) > original_lower[19] ? (init_params[19] - 20) : original_lower[19];          //middle


	//global rotation
	output_upper[15] = (init_params[15] + 40) < original_upper[15] ? (init_params[15] + 40) : original_upper[15]; output_lower[15] = (init_params[15] - 40) > original_lower[15] ? (init_params[15] - 40) : original_lower[15];
	output_upper[16] = (init_params[16] + 40) < original_upper[16] ? (init_params[16] + 40) : original_upper[16]; output_lower[16] = (init_params[16] - 40) > original_lower[16] ? (init_params[16] - 40) : original_lower[16];
	output_upper[17] = (init_params[17] + 40) < original_upper[17] ? (init_params[17] + 40) : original_upper[17]; output_lower[17] = (init_params[17] - 40) > original_lower[17] ? (init_params[17] - 40) : original_lower[17];

	//global position
	output_upper[24] = (init_params[24] + 200) < original_upper[24] ? (init_params[24] + 200) : original_upper[24]; output_lower[24] = (init_params[24] - 200) > original_lower[24] ? (init_params[24] - 200) : original_lower[24];
	output_upper[25] = (init_params[25] + 200) < original_upper[25] ? (init_params[25] + 200) : original_upper[25]; output_lower[25] = (init_params[25] - 200) > original_lower[25] ? (init_params[25] - 200) : original_lower[25];
	output_upper[26] = (init_params[26] + 200) < original_upper[26] ? (init_params[26] + 200) : original_upper[26]; output_lower[26] = (init_params[26] - 200) > original_lower[26] ? (init_params[26] - 200) : original_lower[26];
}


void poseEstimate(const float *initParams, float *upper, float *lower, float* output_dof)
{

	//这里的upper和lower需要结合initialparams重新规划。------------------------------------
	reset_upper_lower_Bound(upper, lower, initParams, PSO_upper_bound, PSO_lower_bound);
	//初始化APSO中的初始姿态
	for (int i = 0;i < ParticleDim;i++)
	{
		pso.posit_initializer[i] = initParams[i];
		pso.APSO_lowerBound[i] = PSO_lower_bound[i];
		pso.APSO_upperBound[i] = PSO_upper_bound[i];
	}
	for (int i = 0; i != PSOpopulation; i++)
	{
		pso.CParticle_swarm[i]->CParticle_Paramsinit(PSO_lower_bound, PSO_upper_bound, initParams);   //维度、参数空间边界
	}

	//调用优化方法
	pso.getBestPosByEvolve();

	//传出参数
	for (int i = 0;i < ParticleDim;i++)
	{
		output_dof[i] = pso.bestPosition[i];
	}
	//memcpy(output_dof, pso.bestPosition, sizeof(float)*ParticleDim);
}