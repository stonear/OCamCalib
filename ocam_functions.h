#pragma once

#include <opencv2\opencv.hpp>

#define MAX_POL_LENGTH 64

using namespace cv;
using namespace std;

class ocam_model
{
	public:
		ocam_model();
		~ocam_model();
		double pol[MAX_POL_LENGTH];    // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
		int length_pol;                // length of polynomial
		double invpol[MAX_POL_LENGTH]; // the coefficients of the inverse polynomial
		int length_invpol;             // length of inverse polynomial
		double xc;         // row coordinate of the center
		double yc;         // column coordinate of the center
		double c;          // affine parameter
		double d;          // affine parameter
		double e;          // affine parameter
		int width;         // image width
		int height;        // image height

		int get_ocam_model(string filename);
		void world2cam(double point2D[2], double point3D[3]);
		void cam2world(double point3D[3], double point2D[2]);
		void create_perspecive_undistortion_LUT(Mat &mapx, Mat &mapy, float sf);
		void create_panoramic_undistortion_LUT(Mat &mapx, Mat &mapy, float Rmin, float Rmax);
};