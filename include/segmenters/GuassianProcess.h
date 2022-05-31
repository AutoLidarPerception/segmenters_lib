#ifndef _GUASSIANPROCESS_
#define _GUASSIANPROCESS_

#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <malloc.h>
#include "iterator"
#include <omp.h>
using namespace std;
using namespace Eigen;

#define mypi 3.14159
#define GP_M 72//72
#define GP_N 80//160
#define divider 1
#define R_limit 80
#define binlen 1//(R_limit/GP_N-1)

namespace autosense {
namespace segmenter {

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> Mat;

struct MyPoint
{
	float x;
	float y;
	float z;
	short intesity;
	short type = -1;
	int Index;
	short Isit = -1;
};

struct VEC{
	vector <MyPoint> Point;
};

void GP_INSAC(const pcl::PointCloud<pcl::PointXYZI> &After, pcl::PointCloud<pcl::PointXYZI> &Ob, pcl::PointCloud<pcl::PointXYZI> &Gr);

}
}
#endif