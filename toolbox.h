#pragma once

#include <string>
#include <vector>
//#include <D:\vs_project\repos\R700Robot\source\third\eigen3\Eigen\Dense>
#include <Eigen/Dense>

//#include "GenaricStructure.h"


using namespace std;
using namespace Eigen;


#define Mat MatrixXd

class Toolbox
{
public:
	inline static double CubicTimeScaling(double Tf, double t);
	inline static double QuinticTimeScaling(double Tf, double t);

	inline static double norm(double P1[], double P2[], int size = 6);
	// same as eigen's matrix function "norm()"
	inline static double norm(Mat mat);

	static Mat PoseToTrans(double pose_zyz[]);

	inline static Mat VecToso3(Mat omg);
	   
	static bool NearZero(double val);

	static Mat MatrixLog3(Mat R);

	static Mat so3ToVec(Mat so3Mat);

	static Mat MatrixExp3(Mat so3mat);

	static int TransToPose(Mat transM, double pose_zyz[6]);

	inline static void printVector(vector<double> P);

	static Mat centerCircle3(double pstart[], double pend[], double pvia[]);

	static Mat TransInv(Mat T);

	static Mat MatrixLog6(Mat T);

	static double Trace(Mat M);

	//static double AxisAng3(Mat expc3, out Mat omghat, out double theta)
	static double AxisAng3(Mat expc3, Mat omghat);

	static Mat MatrixExp6(Mat se3mat);

	static Mat array2Mat(double array[], int size);
	static void Mat2array(Mat m, double outArray[]);

	static void Print_Mat(Mat martix_to_print, string marix_name);

	static void T_scaling(double v, double a, double t, double & out_s, double & out_sd);

	// S_Scaling_3(v,a,J,t)
	static void S_Scaling_3(double v, double a, double J, double t, double & out_s, double & out_sd);
};