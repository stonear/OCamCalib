#include "ocam_functions.h"

int main()
{
	ocam_model o, o_cata; // our ocam_models for the fisheye and catadioptric cameras

	/* --------------------------------------------------------------------*/
	/* Read the parameters of the omnidirectional camera from the TXT file */
	/* --------------------------------------------------------------------*/
	o.get_ocam_model("./calib_results_fisheye.txt");
	o_cata.get_ocam_model("./calib_results_catadioptric.txt");

	/* --------------------------------------------------------------------*/
	/* Print ocam_model parameters                                         */
	/* --------------------------------------------------------------------*/  
	cout << "pol =" << endl;
	for (int i=0; i < o.length_pol; i++) cout << o.pol[i] << endl;
	cout << endl;
	cout << "invpol =" << endl;
	for (int i = 0; i < o.length_invpol; i++) cout << o.invpol[i] << endl;
	cout << endl;
	cout << "nxc = " << o.xc << endl;
	cout << "nyc = " << o.yc << endl;
	cout << "width = " << o.width << endl;
	cout << "height = " << o.height << endl;
	
	/* --------------------------------------------------------------------*/
	/* WORLD2CAM projects 3D point into the image                          */
	/* NOTE!!! The coordinates are expressed according the C convention,   */
	/* that is, from the origin (0,0) instead than from 1 (MATLAB).        */
	/* --------------------------------------------------------------------*/
	double point3D[3] = { 100 , 200 , -300 };       // a sample 3D point
	double point2D[2];                              // the image point in pixel coordinates  
	o.world2cam(point2D, point3D);					// The behaviour of this function is the same as in MATLAB

	/* --------------------------------------------------------------------*/
	/* Display re-projected coordinates                                    */
	/* --------------------------------------------------------------------*/
	cout << endl;
	cout << "world2cam: pixel coordinates reprojected onto the image" << endl;
	cout << "m_row= " << point2D[0] << ", m_col= " << point2D[1] << endl;

	/* --------------------------------------------------------------------*/
	/* CAM2WORLD back-projects pixel points on to the unit sphere          */
	/* The behaviour of this function is the same as in MATLAB             */
	/* --------------------------------------------------------------------*/
	o.cam2world(point3D, point2D); 

	/* --------------------------------------------------------------------*/  
	/* Display back-projected normalized coordinates (on the unit sphere)  */
	/* --------------------------------------------------------------------*/
	cout << endl;
	cout << "cam2world: coordinates back-projected onto the unit sphere (x^2+y^2+z^2=1)" << endl;
	cout << "x= " << point3D[0] << ", y=" << point3D[1] << ", z=" << point3D[2] << endl;

	Mat src1, src2;
	src1 = imread("./test_fisheye.jpg");		// source image 1
	src2 = imread("./test_catadioptric.jpg");	// source image 2
	Mat dst_persp(src1.size(), src1.type());	// undistorted perspective and panoramic image
	Size size_pan_image(1200, 400);				// size of the undistorted panoramic image
	Mat dst_pan(size_pan_image, src2.type());	// undistorted panoramic image

	Mat mapx_persp(src1.size(), CV_32FC1);
	Mat mapy_persp(src1.size(), CV_32FC1);
	Mat mapx_pan(size_pan_image, CV_32FC1);
	Mat mapy_pan(size_pan_image, CV_32FC1);

	/* --------------------------------------------------------------------  */
	/* Create Look-Up-Table for perspective undistortion                     */
	/* SF is kind of distance from the undistorted image to the camera       */
	/* (it is not meters, it is justa zoom fator)                            */
	/* Try to change SF to see how it affects the result                     */
	/* The undistortion is done on a  plane perpendicular to the camera axis */
	/* --------------------------------------------------------------------  */
	float sf = 5;
	o.create_perspecive_undistortion_LUT(mapx_persp, mapy_persp, sf);
	
	/* --------------------------------------------------------------------  */
	/* Create Look-Up-Table for panoramic undistortion                       */
	/* The undistortoin is just a simple cartesia-to-polar transformation    */
	/* Note, only the knowledge of image center (xc,yc) is used to undisort the image      */
	/* xc, yc are the row and column coordinates of the image center         */
	/* Note, if you would like to flip the image, just inverte the sign of theta in this function */
	/* --------------------------------------------------------------------  */  
	float Rmax = 470;  // the maximum radius of the region you would like to undistort into a panorama
	float Rmin = 20;   // the minimum radius of the region you would like to undistort into a panorama
	o_cata.create_panoramic_undistortion_LUT(mapx_pan, mapy_pan, Rmin, Rmax);

	/* --------------------------------------------------------------------*/
	/* Undistort using specified interpolation method                      */
	/* For other possible values, see OpenCV doc                           */
	/* --------------------------------------------------------------------*/
	//remap(src1, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
	remap(src1, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR, BORDER_REPLICATE);
	remap(src2, dst_pan, mapx_pan, mapy_pan, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));

	/* --------------------------------------------------------------------*/
	/* Display image                                                       */
	/* --------------------------------------------------------------------*/
	imshow("Original fisheye camera image", src1);
	imshow("Undistorted Perspective Image", dst_persp);
	imshow("Original Catadioptric camera image", src2);
	imshow("Undistorted Panoramic Image", dst_pan);

	/* --------------------------------------------------------------------*/
	/* Wait until key presses                                              */
	/* --------------------------------------------------------------------*/ 
	waitKey();

	return 0;
}