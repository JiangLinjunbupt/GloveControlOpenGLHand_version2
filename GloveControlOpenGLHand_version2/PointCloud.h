#pragma once
#include<opencv2/opencv.hpp>    
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<vector>
using namespace std;

struct PointCloud
{
	const float focalx = 381.8452f;const float focaly = 382.1713f;
	const float centerx = 264.0945f;const float centery = 217.1487f;

	vector<cv::Point3f> pointcloud_vector;
	float PointCloud_center_x, PointCloud_center_y, PointCloud_center_z = 0.0f;

	PointCloud() {}
	~PointCloud() {}

	void DepthMatToPointCloud(cv::Mat depthmat)
	{
		pointcloud_vector.clear();
		for (int i = 0; i < depthmat.rows; i++)
		{
			for (int j = 0; j < depthmat.cols; j++)
			{
				if (depthmat.at<ushort>(i, j) != 0)
				{
					cv::Point3f p;

					p.x = (j - centerx) * (-depthmat.at<ushort>(i, j)) / focalx;
					p.y = (i - centery) * (-depthmat.at<ushort>(i, j)) / focaly;
					p.z = -depthmat.at<ushort>(i, j);

					this->PointCloud_center_x += p.x;
					this->PointCloud_center_y += p.y;
					this->PointCloud_center_z += p.z;

					this->pointcloud_vector.push_back(p);

				}
			}
		}

		if (this->pointcloud_vector.size() != 0)
		{
			this->PointCloud_center_x = this->PointCloud_center_x / (float)(this->pointcloud_vector.size());
			this->PointCloud_center_y = this->PointCloud_center_y / (float)(this->pointcloud_vector.size());
			this->PointCloud_center_z = this->PointCloud_center_z / (float)(this->pointcloud_vector.size());
		}
	}

};


static PointCloud pointcloud;