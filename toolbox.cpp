#pragma once
#include <iostream>

#include "toolbox.h"
#include "GenaricStructure.h"
//#include "SixAxisRobotArm.h"



inline double Toolbox::CubicTimeScaling(double Tf, double t)
{
	// s = 3 * (t / Tf) ^ 2 - 2 * (t / Tf) ^ 3;
	double A = t / Tf;
	double s = 3 * A * A - 2 * A * A * A;
	return s;
}

inline double Toolbox::QuinticTimeScaling(double Tf, double t)
{
	// s = 10 * (t / Tf) ^ 3 - 15 * (t / Tf) ^ 4 + 6 * (t / Tf) ^ 5;
	double A = t / Tf;
	double s = 10 * A * A * A - 15 * A * A * A * A + 6 * A * A * A * A * A;
	return s;
}

inline double Toolbox::norm(double P1[], double P2[], int size)
{
	double result = 0;
	for (int i = 0; i < size; i++)
	{
		double diff = abs(P1[i] - P2[i]);
		if (result < diff) result = diff;
	}
	return result;
}

// same as eigen's matrix function "norm()"

inline double Toolbox::norm(Mat mat) {
	return mat.norm();
}

inline Mat Toolbox::PoseToTrans(double pose_zyz[])
{
	Mat mat_A(4, 4); // this mat is not be initialized 
	mat_A = Mat::Zero(4, 4);
	for (int i = 0; i < 3; i++) mat_A(i, 3) = pose_zyz[i];
	double phi = pose_zyz[3];
	double theta = pose_zyz[4];
	double xsi = pose_zyz[5];
	double c1 = MathCos(phi);
	double c2 = MathCos(theta);
	double c3 = MathCos(xsi);
	double s1 = MathSin(phi);
	double s2 = MathSin(theta);
	double s3 = MathSin(xsi);

	mat_A(0, 0) = c1 * c2 * c3 - s1 * s3;
	mat_A(0, 1) = -c1 * c2 * s3 - s1 * c3;
	mat_A(0, 2) = c1 * s2;
	mat_A(1, 0) = s1 * c2 * c3 + c1 * s3;
	mat_A(1, 1) = -s1 * c2 * s3 + c1 * c3;
	mat_A(1, 2) = s1 * s2;
	mat_A(2, 0) = -s2 * c3;
	mat_A(2, 1) = s2 * s3;
	mat_A(2, 2) = c2;

	mat_A(3, 3) = 1.0;
	return mat_A;
}

inline Mat Toolbox::VecToso3(Mat omg)
{
	Mat so3Mat(3, 3);
	// so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
    so3Mat <<        0, -omg(2, 0),  omg(1, 0),
             omg(2, 0),          0, -omg(0, 0),
            -omg(1, 0),  omg(0, 0),          0;

	return so3Mat;
}



inline bool Toolbox::NearZero(double val)
{
	return (MathAbs(val) < 0.000001);
}

inline Mat Toolbox::MatrixLog3(Mat R)
{
	double trace_sum = R(0, 0) + R(1, 1) + R(2, 2);
	double acosinput = (trace_sum - 1) / 2.0;

	Mat so3mat(3, 3);
	so3mat = Mat::Zero(3, 3);
	if (acosinput >= 1)
	{
		return so3mat;
	}
	else if (acosinput <= -1)
	{
		Mat omg(3, 1);
		if (!NearZero(1 + R(2, 2)))
		{
			double coef = (1.0 / MathSqrt(2.0 * (1.0 + R(2, 2))));
			omg(0, 0) = coef * R(0, 2);
			omg(1, 0) = coef * R(1, 2);
			omg(2, 0) = coef * (1.0 + R(2, 2));
		}
		else if (!NearZero(1 + R(1, 1)))
		{
			double coef = (1.0 / MathSqrt(2.0 * (1.0 + R(1, 1))));
			omg(0, 0) = coef * R(0, 1);
			omg(1, 0) = coef * (1.0 + R(1, 1));
			omg(2, 0) = coef * R(2, 1);
		}
		else
		{
			double coef = (1.0 / MathSqrt(2.0 * (1.0 + R(0, 0))));
			omg(0, 0) = coef * (1.0 + R(0, 0));
			omg(1, 0) = coef * R(1, 0);
			omg(2, 0) = coef * R(2, 0);
		}
		so3mat = VecToso3(MathPI * omg);
	}
	else
	{
		// theta = acos(acosinput);
		// so3mat = theta * (1 / (2 * sin(theta))) * (R - R');

		double theta = MathAcos(acosinput);
		so3mat = theta * (0.5 / MathSin(theta)) * (R - R.transpose());
	}

	return so3mat;
}

inline Mat Toolbox::so3ToVec(Mat so3Mat)
{
	// Mat omg = new Mat(new double[3, 1]{ { so3Mat[2, 1] }, { so3Mat[0, 2] }, { so3Mat[1, 0] } }, true);
	Mat omg(3, 1);
	omg << so3Mat(2, 1), so3Mat(0, 2), so3Mat(1, 0);
	return omg;
}

inline Mat Toolbox::MatrixExp3(Mat so3mat)
{
	Mat omgtheta = so3ToVec(so3mat);
	double norm_v = norm(omgtheta);
	if (NearZero(norm_v) == true)
	{
		// Mat omg = new Mat(new double[3, 1] { { so3Mat[2,1] }, { so3Mat[0, 2] }, { so3Mat[1, 0] } }, true);
		Mat R(3, 3);
		R = Mat::Zero(3, 3);
		for (int i = 0; i < 3; i++) R(i, i) = 1;
		return R;
	}
	else
	{
		// function [omghat, theta] = AxisAng3(expc3)
		double theta = norm_v;
		Mat omgmat = (1.0 / theta) * so3mat;
		// R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;

		Mat R = Mat::Identity(3, 3) + MathSin(theta) * omgmat + (1 - MathCos(theta)) * omgmat * omgmat;
		return R;
	}
}

inline int Toolbox::TransToPose(Mat transM, double pose_zyz[6])
{
	Mat R(3, 3);
	// Mat P = new Mat(3, 1);
	// double[] pose_zyz = new double[6];
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R(i, j) = transM(i, j);
	for (int i = 0; i < 3; i++) pose_zyz[i] = transM(i, 3);
	if (NearZero(R(2, 2) - 1))
	{
		pose_zyz[3] = 0;
		pose_zyz[4] = 0;
		pose_zyz[5] = MathAtan2(-R(0, 1), R(1, 1));
	}
	else if (NearZero(R(2, 2) + 1))
	{
		pose_zyz[3] = 0;
		pose_zyz[4] = MathPI;
		pose_zyz[5] = -MathAtan2(-R(0, 1), R(1, 1));
	}
	else
	{
		pose_zyz[3] = MathAtan2(R(1, 2), R(0, 2));
		double temp = MathSqrt(R(0, 2) * R(0, 2) + R(1, 2) * R(1, 2));
		pose_zyz[4] = MathAtan2(temp, R(2, 2));
		pose_zyz[5] = MathAtan2(R(2, 1), -R(2, 0));
	}

	return 0;
}

inline void Toolbox::printVector(vector<double> P) {
	int size = P.size();
	for (int i = 0; i < size; i++) cout << P[i] << " ";
	cout << "\n";
}

inline Mat Toolbox::centerCircle3(double pstart[], double pend[], double pvia[])
{
	Mat P1(3, 1);
	P1 << pstart[0], pstart[1], pstart[2];
	Mat P2(3, 1);
	P2 << pend[0], pend[1], pend[2];
	Mat P3(3, 1);
	P3 << pvia[0], pvia[1], pvia[2];

	// %% 計算1點和2點的向量，1點和3點的向量
	Mat v21 = P2 - P1;
	Mat v31 = P3 - P1;
	// %% 計算1點和2點的中點，1點和3點的中點
	Mat P21_Center = 0.5 * (P2 + P1);
	Mat P31_Center = 0.5 * (P3 + P1);
	// %% 計算v21和v31的叉乘，即：面的法線
	// vN = cross(v21, v31);
	Vector3d v21_v = { v21(0, 0), v21(1, 0), v21(2, 0) };
	Vector3d v31_v = { v31(0, 0), v31(1, 0), v31(2, 0) };
	Vector3d vN = v21_v.cross(v31_v);

	// if NearZero(norm(vN))
	//     error('P point is colinear with Line(Pstart,Pend) give another one')
	// %% 羅列三元三次方程組
	// %   （1、2）圓心與弦中點組成的向量垂直於弦向量
	// %   （3）圓心和3點共面
	Mat n21 = 1.0 / norm(v21) * v21;
	Mat n31 = 1.0 / norm(v31) * v31;
	Mat nN = 1.0 / norm(vN) * vN;

	Mat A(3, 3);
	A <<
		n21(0, 0), n21(1, 0), n21(2, 0),
		n31(0, 0), n31(1, 0), n31(2, 0),
		nN(0, 0), nN(1, 0), nN(2, 0);
	Mat A_inv = A.inverse();

	double b1 = norm(P21_Center - P1);
	double b2 = norm(P31_Center - P1);
	Mat B(3, 1);
	B << b1, b2, 0;

	Mat Vc1 = A_inv * B;
	Mat c = Vc1 + P1;

	return c;
}

inline Mat Toolbox::TransInv(Mat T)
{
	Mat R(3, 3);
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R(i, j) = T(i, j);
	// Vector2d p = { T(0, 3) ,  T(1, 3) ,  T(2, 3) };
	Mat p(3, 1);
	p << T(0, 3), T(1, 3), T(2, 3);
	// invT = [R', -R' * p; 0, 0, 0, 1];
	Mat R_ = R.transpose();
	Mat R_p = -R_ * p;
	Mat invT = Mat::Identity(4, 4);
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) invT(i, j) = R_(i, j);
	for (int i = 0; i < 3; i++) invT(i, 3) = R_p(i, 0);
	return invT;
}

inline Mat Toolbox::MatrixLog6(Mat T)
{
	// [R, p] = TransToRp(T);
	// omgmat = MatrixLog3(R);
	Mat R(3, 3);
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R(i, j) = T(i, j);
	Mat p(3, 1);
	p << T(0, 3), T(1, 3), T(2, 3);
	Mat omgmat = MatrixLog3(R);

	bool b_all_zero = true;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (!NearZero(omgmat(i, j)))
			{
				b_all_zero = false;
				break;
			}
		}
		if (b_all_zero == false) break;
	}
	if (b_all_zero == true)
	{
		Mat expmat(4, 4);
		for (int i = 0; i < 3; i++) expmat(i, 3) = T(i, 3);
		return expmat;
	}
	else
	{
		double theta = MathAcos((Trace(R) - 1.0) / 2.0);
		Mat tempA1 = Mat::Identity(3, 3) - 0.5 * omgmat;
		// Mat tempA2 = (1.0 / theta - 0.5 / theta / Math.Tan(0.5 * theta)) * omgmat * omgmat;
		Mat tempA2 = (1.0 / theta - 0.5 / MathTan(0.5 * theta)) / theta * omgmat * omgmat;
		Mat tempA = (tempA1 + tempA2) * p;
		Mat expmat(4, 4);
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) expmat(i, j) = omgmat(i, j);
		for (int i = 0; i < 3; i++) expmat(i, 3) = tempA(i, 0);
		return expmat;
	}
}

inline double Toolbox::Trace(Mat M)
{
	int row = M.rows();
	int col = M.cols();
	if (row != col) return 0;
	double sum = 0;
	for (int i = 0; i < row; i++) sum += M(i, i);
	return sum;
}

//static double AxisAng3(Mat expc3, out Mat omghat, out double theta)

inline double Toolbox::AxisAng3(Mat expc3, Mat omghat)
{
	double theta = norm(expc3);
	omghat = 1.0 / theta * expc3;
	return theta;
}

inline Mat Toolbox::MatrixExp6(Mat se3mat)
{
	Mat se3mat_33(3, 3);
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) se3mat_33(i, j) = se3mat(i, j);
	Mat omgtheta = so3ToVec(se3mat_33);

	if (NearZero(norm(omgtheta))) {
		// T = [eye(3), se3mat(1: 3, 4); 0, 0, 0, 1];
		Mat T = Mat::Identity(4, 4);
		for (int i = 0; i < 3; i++) T(i, 3) = se3mat(i, 3);
		return T;
	}
	else
	{
		Mat omghat;
		double theta = AxisAng3(omgtheta, omghat);

		Mat omgmat = 1.0 / theta * se3mat_33;

		/*
		T = [MatrixExp3(se3mat(1: 3, 1: 3)), ...
		s(  eye(3) * theta + (1 - cos(theta)) * omgmat ...
		+ (theta - sin(theta)) * omgmat * omgmat )s ...
		* se3mat(1: 3, 4) / theta;
		0, 0, 0, 1];
		*/
		Mat T = Mat::Identity(4, 4);
		Mat tempA = MatrixExp3(se3mat_33);
		Mat tempB1 = theta * Mat::Identity(3, 3) + (1.0 - MathCos(theta)) * omgmat;
		Mat tempB2 = (theta - MathSin(theta)) * omgmat * omgmat;
		Mat se3mat_p(3, 1);
		for (int i = 0; i < 3; i++) se3mat_p(i, 0) = se3mat(i, 3);
		Mat tempB = (1.0 / theta) * (tempB1 + tempB2) * se3mat_p;
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) T(i, j) = tempA(i, j);
		for (int i = 0; i < 3; i++) T(i, 3) = tempB(i, 0);
		return T;
	}

}

inline Mat Toolbox::array2Mat(double array[], int size)
{
	int len = size;
	Mat mat(len, 1);
	for (int i = 0; i < len; i++) mat(i, 0) = array[i];
	return mat;
}

inline void Toolbox::Mat2array(Mat m, double outArray[])
{
	int len = m.rows();
	for (int i = 0; i < len; i++) outArray[i] = m(i, 0);
}

inline void Toolbox::Print_Mat(Mat martix_to_print, string marix_name)
{
	cout << marix_name << endl;

	for (int i = 0; i < martix_to_print.rows(); i++)
	{
		for (int j = 0; j < martix_to_print.cols(); j++)
			cout << martix_to_print(j, i) << ", ";

		cout << endl;
	}
}

inline void Toolbox::T_scaling(double v, double a, double t, double & out_s, double & out_sd)
{
	// s = 0;
	// sd = 0;
	if (a > v * v)
	{
		double T = (a + v * v) / (a * v);
		if ((0 <= t) && (t <= v / a))
		{
			// double sdd = a;
			out_sd = a * t;
			out_s = 0.5 * a * t * t;
		}
		else if ((v / a < t) && (t <= (T - v / a)))
		{
			// double sdd = 0;
			out_sd = v;
			out_s = v * t - v * v / (2 * a);
		}
		else if (((T - v / a) < t) && (t <= T))
		{
			// double sdd = -a;
			out_sd = v - a * (t - T + v / a);
			out_s = (2 * a * v * T - 2 * v * v - a * a * (t - T) * (t - T)) / (2 * a);
		}
		// end

	}
	else // (a <= v^2)
	{
		double T = 2 / MathSqrt(a);
		if ((0 <= t) && (t < T / 2))
		{
			out_sd = a * t;
			out_s = 0.5 * a * t * t;
		}
		else if ((T / 2 <= t) && (t <= T))
		{
			out_sd = a * (T - t);
			out_s = a * (T / 2) * (T / 2) - (a / 2) * (t - T) * (t - T);
		}

	}
}

// S_Scaling_3(v,a,J,t)

inline void Toolbox::S_Scaling_3(double v, double a, double J, double t, double & out_s, double & out_sd)
{
	// s = 0;
	// sd = 0;

	double T = (a * a * v + J * a + J * v * v) / (J * a * v);

	double t1 = a / J;
	double t2 = v / a;
	double t3 = a / J + v / a;
	double t4 = T - t3;

	double d = (J * t1 * t1 * t1) / 6.0 + (J * t1 * t1 * (t2 - t1)) / 2.0 + (a * (t2 - t1) * (t2 - t1)) / 2.0;
	double c = (J * t1 * t1) / 2.0 + a * (t2 - t1);
	double d2 = (a * t2 * t2) / 3.0 - (2 * a * t2 * t3) / 3.0 - c * t2 + (a * t3 * t3) / 3.0 + c * t3 + d;


	if ((0 <= t) && (t <= t1))
	{
		out_sd = (J * t * t) / 2.0;
		out_s = (J * t * t * t) / 6.0;
	}
	else if ((t1 < t) && (t <= t2))
	{
		out_sd = (J * t1 * t1) / 2.0 + a * (t - t1);
		out_s = (J * t1 * t1 * t1) / 6.0 + (J * t1 * t1 * (t - t1)) / 2.0 + (a * (t - t1) * (t - t1)) / 2.0;
	}
	else if ((t2 < t) && (t <= t3))
	{
		out_sd = (J * t1 * t1) / 2.0 + a * (t2 - t1) + a * ((t - t3) * (t - t3) - (t2 - t3) * (t2 - t3)) / (2.0 * (t2 - t3));
		out_s = d + ((t - t2) * (6 * c * t2 - 6 * c * t3 + a * t * t - 2 * a * t2 * t2 + a * t * t2 - 3 * a * t * t3 + 3 * a * t2 * t3)) / (6.0 * (t2 - t3));
	}
	else if ((t3 < t) && (t <= t4))
	{
		out_sd = v;
		out_s = d2 + v * (t - t3);
	}
	else if ((t4 < t) && (t <= (T - t2)))
	{
		out_sd = (J * t1 * t1) / 2.0 + a * (t2 - t1) + a * (((T - t) - t3) * ((T - t) - t3) - (t2 - t3) * (t2 - t3)) / (2.0 * (t2 - t3));
		out_s = ((t - T + t2) * (6 * c * t2 - 6 * c * t3 - 2 * a * t2 * t2 + a * (T - t) * (T - t) + 3 * a * t2 * t3 + a * t2 * (T - t) - 3 * a * t3 * (T - t))) / (6.0 * t2 - 6 * t3) - d + 1;
	}
	else if (((T - t2) < t) && (t <= T - t1))
	{
		out_sd = (J * t1 * t1) / 2.0 + a * ((T - t) - t1);
		out_s = (J * t1 * t1 * (t - T + t1)) / 2.0 - (J * t1 * t1 * t1) / 6.0 - (a * (t - T + t1) * (t - T + t1)) / 2.0 + 1;
	}
	else if (((T - t1) < t) && (t <= T))
	{
		out_sd = (J * (T - t) * (T - t)) / 2.0;
		out_s = 1 - (J * (T - t) * (T - t) * (T - t)) / 6.0;
	}

}
