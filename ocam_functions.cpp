#include "ocam_functions.h"

ocam_model::ocam_model()
{
}

ocam_model::~ocam_model()
{
}

int ocam_model::get_ocam_model(string filename)
{
	ifstream file(filename);
	string temp;

	//Read polynomial coefficients
	for (int i = 0; i < 2; i++) getline(file, temp);
	file >> length_pol;
	for (int i = 0; i < length_pol; i++) file >> pol[i];
	
	//Read inverse polynomial coefficients
	for (int i = 0; i < 3; i++) getline(file, temp);
	file >> length_invpol;
	for (int i = 0; i < length_invpol; i++) file >> invpol[i];
	 
	//Read center coordinates
	for (int i = 0; i < 3; i++) getline(file, temp);
	file >> xc;
	file >> yc;

	//Read affine coefficients
	for (int i = 0; i < 3; i++) getline(file, temp);
	file >> c;
	file >> d;
	file >> e;
	
	//Read image size
	for (int i = 0; i < 3; i++) getline(file, temp);
	file >> height;
	file >> width;
	
	file.close();

	return 0;
}

void ocam_model::world2cam(double point2D[2], double point3D[3])
{
	double norm = sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1]);
	double theta = atan(point3D[2] / norm);
	double t, t_i;
	double rho, x, y;
	double invnorm;
	int i;

	if (norm != 0)
	{
		invnorm = 1 / norm;
		t = theta;
		rho = invpol[0];
		t_i = 1;

		for (i = 1; i < length_invpol; i++)
		{
			t_i *= t;
			rho += t_i*invpol[i];
		}

		x = point3D[0] * invnorm * rho;
		y = point3D[1] * invnorm * rho;

		point2D[0] = x * c + y * d + xc;
		point2D[1] = x * e + y + yc;
	}
	else
	{
		point2D[0] = xc;
		point2D[1] = yc;
	}
}

void ocam_model::cam2world(double point3D[3], double point2D[2])
{
	double invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file
	double xp = invdet * ((point2D[0] - xc) - d * (point2D[1] - yc));
	double yp = invdet * (-e * (point2D[0] - xc) + c * (point2D[1] - yc));

	double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
	double zp  = pol[0];
	double r_i = 1;

	for (int i = 1; i < length_pol; i++)
	{
		r_i *= r;
		zp  += r_i*pol[i];
	}
 
	//normalize to unit norm
	double invnorm = 1 / sqrt(xp * xp + yp * yp + zp * zp);

	point3D[0] = invnorm * xp;
	point3D[1] = invnorm * yp; 
	point3D[2] = invnorm * zp;
}

void ocam_model::create_perspecive_undistortion_LUT(Mat &mapx, Mat &mapy, float sf)
{
	Size s = mapx.size();
	int width = s.width;
	int height = s.height;
	float Nxc = height / 2.0;
	float Nyc = width / 2.0;
	float Nz = -width / sf;
	double M[3];
	double m[2];
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			M[0] = (i - Nxc);
			M[1] = (j - Nyc);
			M[2] = Nz;
			world2cam(m, M);
			mapx.at<float>(i, j) = (float)m[1];
			mapy.at<float>(i, j) = (float)m[0];
		}
	}
}

void ocam_model::create_panoramic_undistortion_LUT(Mat &mapx, Mat &mapy, float Rmin, float Rmax)
{
	float theta;
	Size s = mapx.size();
	int width = s.width;
	int height = s.height;
	float rho;

	for (int i = 0; i<height; i++)
		for (int j = 0; j<width; j++)
		{
			theta = -((float)j) / width * 2 * CV_PI; // Note, if you would like to flip the image, just inverte the sign of theta
			rho = Rmax - (Rmax - Rmin) / height * i;
			mapx.at<float>(i, j) = yc + rho * sin(theta);
			mapy.at<float>(i, j) = xc + rho * cos(theta);
		}
}