// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
// used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A simple example of using the Ceres minimizer.
//
// Minimize 0.5 (10 - x)^2 using jacobian matrix computed using
// automatic differentiation.

#include <string>

#include "ceres/ceres.h"
#include "ceres/iteration_callback.h"
//#include "ceres/types.h"
#include "ceres/rotation.h"
#include "glog/logging.h"
//#include <ceres/rotation.h>
//#include "ceres_costs_utils.hpp"
//#include "basic_types.h"

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/package.h>

#include <opencv2/highgui/highgui.hpp>
//TODO Why are these deps listed twice?
#include <opencv2/core.hpp>

#include "lidar_reader.cpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

//Debug elements to visualize action of the projector.
cv::Mat * debug_mat;
cv::Mat * the_ground_truth;
class VisualCallback : public ceres::IterationCallback {
public:
	//TODO Are either of these necessary??
	explicit VisualCallback(){}
	~VisualCallback() {}

	ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
		cv::namedWindow("Iteration Projection", CV_GUI_EXPANDED | CV_WINDOW_NORMAL);
		cv::imshow("Iteration Projection", *debug_mat);
		cv::waitKey(1000);
		cv::imwrite("/home/tes77/dbg_img.png", *debug_mat);
		//cv::waitKey();
		
		the_ground_truth->copyTo(*debug_mat);
		return ceres::SOLVER_CONTINUE;
	}
};
//Officially the stupidest thing, this profoundly hacky process is required to
//get the core value out of a ceres::jet since it cannot be accessed
//directly even in a read-only manner. WHY is this functionalty missing from a
//flagship piece of software released by a major corporation for prestigious
//institutions of learning to use?????????
template<typename T>
double val(const T& j){
	std::ostringstream oss;
	oss << j;
	
	//Sometimes a Jet has a big long blob of derivative data attached to it
	//that we need to cut off, and a bracket at the beginning...
	if(oss.str().find_first_of("[") != string::npos){
		return std::stod(oss.str().substr(1, oss.str().find_first_of(";") - 1));
	}
	
	//And SOMETIMES it's just a number.
	else{
		return std::stod(oss.str());
	}
	
	//I have NO idea why.
}

inline double dtor(double d){
	return (d * 3.14159) / 180.0;
}

template <typename T>
inline void transformPoint_rm(const T translation[3], const T rotation[9], const T point_in[3], T point_out[3]);
template <typename T>
inline void transformPoint_rm(const T translation[3], const T rotation[9], const T point_in[3], T point_out[3]){
	/*std::cout << "\tTRANSLATION\n";
	std::cout << "\t" << translation[0] << "\n";
	std::cout << "\t" << translation[1] << "\n";
	std::cout << "\t" << translation[2] << "\n";
	
	std::cout << "\tROTATION\n";
	std::cout << "\t" << rotation[0] << "  " << rotation[1] << "  " << rotation[2] << "\n";
	std::cout << "\t" << rotation[3] << "  " << rotation[4] << "  " << rotation[5] << "\n";
	std::cout << "\t" << rotation[6] << "  " << rotation[7] << "  " << rotation[8] << "\n";*/

	point_out[0] = (point_in[0] * rotation[0] + point_in[1] * rotation[1] + point_in[2] * rotation[2]) + translation[0];
	point_out[1] = (point_in[0] * rotation[3] + point_in[1] * rotation[4] + point_in[2] * rotation[5]) + translation[1];
	point_out[2] = (point_in[0] * rotation[6] + point_in[1] * rotation[7] + point_in[2] * rotation[8]) + translation[2];
}

/*! \brief ceres compliant function to apply a rotation and translation to transform a point
 *@param rotation r, p, w
 *@param translation x, y and z
 *@param point_in, the original point
 *@param point_out, the transformed point
 */
template <typename T>
inline void transformPoint_euler(const T translation[3], const T rotation[3], const T point_in[3], T point_out[3]);
template <typename T>
inline void transformPoint_euler(const T translation[3], const T rotation[3], const T point_in[3], T point_out[3]){
	T R [9];
	ceres::EulerAnglesToRotationMatrix(rotation, 3, R);
	
	transformPoint_rm(translation, R, point_in, point_out);
}

/*! \brief ceres compliant function to apply a rotation and translation to transform a point
 *@param rotation angle axis x, y, z, norm is amount
 *@param translation x, y and z
 *@param point_in, the original point
 *@param point_out, the transformed point
 */
template <typename T>
inline void transformPoint_aa(const T translation[3], const T rotation[3], const T point_in[3], T point_out[3]);
template <typename T>
inline void transformPoint_aa(const T translation[3], const T rotation[3], const T point_in[3], T point_out[3]){
	std::cout << "\t" << translation[0] << "\n";
	std::cout << "\t" << translation[1] << "\n";
	std::cout << "\t" << translation[2] << "\n\n";
	std::cout << "\t" << rotation[0] << "\n";
	std::cout << "\t" << rotation[1] << "\n";
	std::cout << "\t" << rotation[2] << "\n\n";

	std::cout << "\t" << point_in[0] << "\n";
	std::cout << "\t" << point_in[1] << "\n";
	std::cout << "\t" << point_in[2] << "\n\n";
	

	ceres::AngleAxisRotatePoint(rotation, point_in, point_out);
	point_out[0] = point_out[0] + translation[0];
	point_out[1] = point_out[1] + translation[1];
	point_out[2] = point_out[2] + translation[2];
	
	
	std::cout << "\t" << point_out[0] << "\n";
	std::cout << "\t" << point_out[1] << "\n";
	std::cout << "\t" << point_out[2] << "\n\n";
}

/*! \brief ceres compliant function to compute the residual from a distorted pinhole camera model
 *@param point_x the input point x
 *@param point_y the input point y
 *@param point_z the input point z
 *@param k1 radial distortion parameter k1
 *@param k2 radial distortion parameter k2
 *@param k3 radial distortion parameter k3
 *@param p1 tangential distortion parameter p1
 *@param p2 tangential distortion parameter p2
 *@param fx focal length in x
 *@param fy focal length in y
 *@param cx optical center in x
 *@param cy optical center in y
 *@param ox observation in x
 *@param oy observation in y
 *@param residual the output or difference between where the point should appear given the parameters, and where it
 * was observed
 */
template <typename T>
void cameraPntResidualDist(
	const T& point_x, const T& point_y, const T& point_z,
	const T& fx, const T& fy, const T& cx, const T& cy,
	//No distortion in rectified image.
	const T& ox, const T& oy,
	T* residual
);
template <typename T>
inline void cameraPntResidualDist(
	const T& point_x, const T& point_y, const T& point_z,
	const T& fx, const T& fy, const T& cx, const T& cy,
	//No distortion in rectified image
	const T& ox, const T& oy,
	T* residual
){
	/*std::cout<< "Goal u " << ox << "\n";
	std::cout<< "Goal v " << oy << "\n\n";
	
	std::cout<< "fx " << fx << "\n";
	std::cout<< "fy " << fy << "\n";
	std::cout<< "cx " << cx << "\n";
	std::cout<< "cy " << cy << "\n";*/
	
	//Projection
	T xp1 = point_x;
	T yp1 = point_y;
	T zp1 = point_z;
	T up;
	T vp;
	
	//std::cout << point_x << "\n";
	//std::cout << point_y << "\n";
	//std::cout << point_z << "\n\n";
	
	up = cx * zp1 + fx * xp1;
	vp = cy * zp1 + fy * yp1;

	if(zp1 != T(0)){//If the distance to the image plane is zero something is VERY wrong.
		up = up / zp1;
		vp = vp / zp1;
	}
	
	//std::cout<< "u pre-distort " << up << "\n";
	//std::cout<< "v pre-distort " << vp << "\n";
	
	residual[0] = up - ox;
	residual[1] = vp - oy;
	
	//TODO This boundary checking needs to be improved, are there rare
	//edge cases where it can pass out of the range of the cv::Mat?
	//Also parametrize the resolution of the images.
	if(!(up < T(1.0) || up > T(2687.0) || vp < T(1.0) || vp > T(1519.0))){
		debug_mat->at<cv::Vec3b>((int)std::rint(val(vp)) - 1, (int)std::rint(val(up)) - 1)[1] = 255;
		debug_mat->at<cv::Vec3b>((int)std::rint(val(vp)) + 1, (int)std::rint(val(up)) - 1)[1] = 255;
		debug_mat->at<cv::Vec3b>((int)std::rint(val(vp)) - 1, (int)std::rint(val(up)) + 1)[1] = 255;
		debug_mat->at<cv::Vec3b>((int)std::rint(val(vp)) + 1, (int)std::rint(val(up)) + 1)[1] = 255;
		debug_mat->at<cv::Vec3b>((int)std::rint(val(vp)), (int)std::rint(val(up)))[1] = 255;
	}
	//std::getchar();
}

class ExtrinsicCalEntry{
public:
	// A templated cost functor that implements the residual
	//Is given the UNKNOWN TERMS
	template<typename T>
	bool operator()(//TODO Why are all these const / should all these be const?
		const T* CAM_to_FOREARM_translation, const T* CAM_to_FOREARM_rotation,
		const T* BASE_to_TARGET_translation, const T* BASE_to_TARGET_rotation,
	T* residual) const {

		//1: Transform target points into camera frame
		//	CAM_to_POINT = CAM_to_FOREARM * FOREARM_to_BASE_C * BASE_to_TARGET * TARGET_to_POINT
	
		//	1a: TARGET_to_POINT
		T TARGET_to_POINT [3];
		TARGET_to_POINT[0] = T(px);
		TARGET_to_POINT[1] = T(py);
		TARGET_to_POINT[2] = T(pz);
	
		/*std::cout << "TARGET TO POINT\n";
		std::cout << TARGET_to_POINT[0] << "\n";
		std::cout << TARGET_to_POINT[1] << "\n";
		std::cout << TARGET_to_POINT[2] << "\n\n";*/
	
		//	1b: BASE_to_TARGET * TARGET_to_POINT
		T BASE_to_POINT [3];
		transformPoint_euler(BASE_to_TARGET_translation, BASE_to_TARGET_rotation, TARGET_to_POINT, BASE_to_POINT);
		
		/*std::cout << "BASE TO POINT\n";
		std::cout << BASE_to_POINT[0] << "\n";
		std::cout << BASE_to_POINT[1] << "\n";
		std::cout << BASE_to_POINT[2] << "\n\n";*/		
		
		//	1c: FOREARM_C_to_BASE * BASE_to_TARGET * TARGET_to_POINT
		T FOREARM_C_to_POINT[3];
		T FOREARM_C_to_BASE_r [9];
		FOREARM_C_to_BASE_r[0] = T(FOREARM_C_to_BASE_rotation[0]);
		FOREARM_C_to_BASE_r[1] = T(FOREARM_C_to_BASE_rotation[1]);
		FOREARM_C_to_BASE_r[2] = T(FOREARM_C_to_BASE_rotation[2]);
		FOREARM_C_to_BASE_r[3] = T(FOREARM_C_to_BASE_rotation[3]);
		FOREARM_C_to_BASE_r[4] = T(FOREARM_C_to_BASE_rotation[4]);
		FOREARM_C_to_BASE_r[5] = T(FOREARM_C_to_BASE_rotation[5]);
		FOREARM_C_to_BASE_r[6] = T(FOREARM_C_to_BASE_rotation[6]);
		FOREARM_C_to_BASE_r[7] = T(FOREARM_C_to_BASE_rotation[7]);
		FOREARM_C_to_BASE_r[8] = T(FOREARM_C_to_BASE_rotation[8]);
		
		T FOREARM_C_to_BASE_t [3];
		FOREARM_C_to_BASE_t[0] = T(FOREARM_C_to_BASE_translation[0]);
		FOREARM_C_to_BASE_t[1] = T(FOREARM_C_to_BASE_translation[1]);
		FOREARM_C_to_BASE_t[2] = T(FOREARM_C_to_BASE_translation[2]);
		transformPoint_rm(FOREARM_C_to_BASE_t, FOREARM_C_to_BASE_r, BASE_to_POINT, FOREARM_C_to_POINT);
		
		/*std::cout << "FOREARM TO POINT\n";
		std::cout << FOREARM_C_to_POINT[0] << "\n";
		std::cout << FOREARM_C_to_POINT[1] << "\n";
		std::cout << FOREARM_C_to_POINT[2] << "\n\n";*/
		
		//	1d: CAM_to_FOREARM_C * FOREARM_C_to_BASE * BASE_to_TARGET * TARGET_to_POINT
		T CAM_to_POINT [3];
		transformPoint_euler(CAM_to_FOREARM_translation, CAM_to_FOREARM_rotation, FOREARM_C_to_POINT, CAM_to_POINT);
		
		/*std::cout << "CAM TO POINT\n";
		std::cout << CAM_to_POINT[0] << "\n";
		std::cout << CAM_to_POINT[1] << "\n";
		std::cout << CAM_to_POINT[2] << "\n\n";*/


		T ox = T(image_pixels[0]);
		T oy = T(image_pixels[1]);
	
		cameraPntResidualDist(
			CAM_to_POINT[0], CAM_to_POINT[1], CAM_to_POINT[2],
			T(fx_fixed), T(fy_fixed), T(cx_fixed), T(cy_fixed),
			ox, oy,
			residual
		);
		//std::getchar();
		return true;
	}
	
	//Member vars- all the perpoint constants
	double image_pixels[2]; /** observed px location of object in image */
	
	double FOREARM_C_to_BASE_rotation[9];
	double FOREARM_C_to_BASE_translation[3];
	
	double px;
	double py;
	double pz;
	
	double fx_fixed;
	double fy_fixed;
	double cx_fixed;
	double cy_fixed;

	//Having nothing but a constructor inside of Create() is a bit silly...
	ExtrinsicCalEntry(
		const double image_px[2],
		
		const double pnt_x, const double pnt_y, const double pnt_z,
		
		const double FOREARM_C_to_BASE_r[9], const double FOREARM_C_to_BASE_t[3],
		
		const double fx, const double fy, const double cx, const double cy
	){
		image_pixels[0] = image_px[0];
		image_pixels[1] = image_px[1];
		
		px = pnt_x;
		py = pnt_y;
		pz = pnt_z;
		
		for(int i = 0; i < 9; i++){
			FOREARM_C_to_BASE_rotation[i] = FOREARM_C_to_BASE_r[i];
		}
		FOREARM_C_to_BASE_translation[0] = FOREARM_C_to_BASE_t[0];
		FOREARM_C_to_BASE_translation[1] = FOREARM_C_to_BASE_t[1];
		FOREARM_C_to_BASE_translation[2] = FOREARM_C_to_BASE_t[2];
		
		fx_fixed = fx;
		fy_fixed = fy;
		cx_fixed = cx;
		cy_fixed = cy;
	}

	//Takes the per-point constants.
	static ceres::CostFunction* Create(
		const double image_px[2],
		const double pnt_x, const double pnt_y, const double pnt_z,
		const double FOREARM_C_to_BASE_r[9], const double FOREARM_C_to_BASE_t[3],
		const double fx, const double fy, const double cx, const double cy
	){
		return new ceres::AutoDiffCostFunction<ExtrinsicCalEntry, 2,
			3, 3, 3, 3
		>(new ExtrinsicCalEntry(
			image_px,
			
			pnt_x, pnt_y, pnt_z,
			
			FOREARM_C_to_BASE_r, FOREARM_C_to_BASE_t,
			
			fx, fy, cx, cy
		));
	}
};

int main(int argc, char** argv) {

	cv::Mat debug_mat_core = cv::Mat(1520, 2688, CV_8UC3);
	debug_mat = &debug_mat_core;
	cv::Mat ground_truth_core = cv::Mat(1520, 2688, CV_8UC3);
	the_ground_truth = &ground_truth_core;

	//TODO what does this do? Looks ominous.
	google::InitGoogleLogging(argv[0]);

	// Storage for input values constant for each point.
	std::vector<int> img_vec;
	std::vector<int> cor_vec;
	std::vector<Eigen::Affine3d> FOREARM_to_BASE_vec;
	std::vector<Eigen::Vector2d> image_pixel_vec;
	
	//read the image detection file:
	//TODO Parametrize dis
	std::string data_path = ros::package::getPath("roadprintz_camera_calibration");
	std::string fname(data_path + "/white_tape_data/detections.csv");
	if (!read_image_file(fname, img_vec, cor_vec, FOREARM_to_BASE_vec, image_pixel_vec)) {
		cout<<"could not open file "<<fname<<"; quitting"<<endl;
		return 1;
	}
	cout<<"image file has been read in"<<endl;
	
	std::cout << "FOREARM_to_BASE\n" << FOREARM_to_BASE_vec[0].matrix() << "\n\n";
	
	
	std::map<int, Eigen::Vector3d> cloud_point_vec;
	
	fname = data_path + "/white_tape_data/pointcloud_points.csv";
	if (!read_pointcloud_file(fname, cloud_point_vec)){
		cout<<"could not open file "<<fname<<"; quitting"<<endl;
		return 1;
	}
	cout << "pointcloud file has been read in" << endl;
	
	// Build the problem.
	Problem problem;
	int nlines = (int) image_pixel_vec.size();
	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).

	//Known values (one per data point), will be set in loop
	//double image_u,image_v;
	//double FOREARM_to_BASE [16];
	//bool left;
	
	//Known values (constant)
	double projection_matrix[12] = {
		1637.367343,	0.0,		1313.144667,	0.0,
		0.0,		1638.139550,	774.029343,	0.0,
		0.0,		0.0,		1.0,		0.0
	};

	//Unknown values
	double BASE_to_TARGET_t [3];
	double BASE_to_TARGET_r [3];
	
	double CAM_to_FOREARM_t[3];
	double CAM_to_FOREARM_r[3];
	
	//Initialize the unknowns
	//TODO Find a way to inform this parametrically
	//TODO For simulation tests, get these ground truths from the simulation
	//Angle of > 50.0 breaks in sim; 40 does not.
	BASE_to_TARGET_t[0] =  0.0;
	BASE_to_TARGET_t[1] =  0.0;
	BASE_to_TARGET_t[2] =  0.0;
	//Determined with https://www.andre-gaschler.com/rotationconverter/
	//Note that the Euler Angle output should be in Degrees and set to ZYX
	BASE_to_TARGET_r[0] =  0.0;
	BASE_to_TARGET_r[1] =  0.0;
	BASE_to_TARGET_r[2] =  0.0;
	
	CAM_to_FOREARM_t[0] =  -0.1050;
	CAM_to_FOREARM_t[1] =  -1.0620;
	CAM_to_FOREARM_t[2] =  0.067;
	//Determined with https://www.andre-gaschler.com/rotationconverter/
	//Note that the Euler Angle output should be in Degrees and set to ZYX
	CAM_to_FOREARM_r[0] =  0.0;
	CAM_to_FOREARM_r[1] =  0.0;
	CAM_to_FOREARM_r[2] =  90.0;
	
	for (int i=0; i<nlines; i++) {//For each data entry...
		//Add the per-point constants
		double pixvec[2] = {image_pixel_vec[i][0], image_pixel_vec[i][1]};
		
		Eigen::Vector3d vector_F = FOREARM_to_BASE_vec[i].inverse().translation();
		Eigen::Matrix3d matrix_F = FOREARM_to_BASE_vec[i].inverse().linear();
		double translation_F[3] = {
			vector_F.x(),
			vector_F.y(),
			vector_F.z()
		};
		double rotation_F[9] = {
			matrix_F(0,0), matrix_F(0,1), matrix_F(0,2),
			matrix_F(1,0), matrix_F(1,1), matrix_F(1,2),
			matrix_F(2,0), matrix_F(2,1), matrix_F(2,2)
		};
		
		Eigen::Vector3d points_in_space = cloud_point_vec[img_vec[i] * 4 + cor_vec[i]];
		
		CostFunction *cost_function = ExtrinsicCalEntry::Create(
			pixvec,
			points_in_space(0), points_in_space(1), points_in_space(2),
			
			rotation_F, translation_F,
			
			projection_matrix[0], projection_matrix[5], projection_matrix[2], projection_matrix[6]
		);
		
		//Debug output
		ground_truth_core.at<uchar>(std::rint(image_pixel_vec[i][1]) - 1, std::rint(image_pixel_vec[i][0]) - 1, 0) = 255;
		ground_truth_core.at<uchar>(std::rint(image_pixel_vec[i][1]) - 1, std::rint(image_pixel_vec[i][0]), 0) = 255;
		ground_truth_core.at<uchar>(std::rint(image_pixel_vec[i][1]) - 1, std::rint(image_pixel_vec[i][0]) + 1, 0) = 255;
		ground_truth_core.at<uchar>(std::rint(image_pixel_vec[i][1]), std::rint(image_pixel_vec[i][0]) - 1, 0) = 255;
		ground_truth_core.at<uchar>(std::rint(image_pixel_vec[i][1]), std::rint(image_pixel_vec[i][0]) + 1, 0) = 255;
		ground_truth_core.at<uchar>(std::rint(image_pixel_vec[i][1]) + 1, std::rint(image_pixel_vec[i][0]) - 1, 0) = 255;
		ground_truth_core.at<uchar>(std::rint(image_pixel_vec[i][1]) + 1, std::rint(image_pixel_vec[i][0]), 0) = 255;
		ground_truth_core.at<uchar>(std::rint(image_pixel_vec[i][1]) + 1, std::rint(image_pixel_vec[i][0]) + 1, 0) = 255;
		
		//Add the parameters to optimize
		problem.AddResidualBlock(cost_function, NULL,
			CAM_to_FOREARM_t, CAM_to_FOREARM_r,
			BASE_to_TARGET_t, BASE_to_TARGET_r
		);
	}
	
	the_ground_truth->copyTo(*debug_mat);
	
	
	//Bound the rotations.
	problem.SetParameterLowerBound(BASE_to_TARGET_r, 0, -190.0);
	problem.SetParameterUpperBound(BASE_to_TARGET_r, 0,  190.0);
	problem.SetParameterLowerBound(BASE_to_TARGET_r, 1, -190.0);
	problem.SetParameterUpperBound(BASE_to_TARGET_r, 1,  190.0);
	problem.SetParameterLowerBound(BASE_to_TARGET_r, 2, -190.0);
	problem.SetParameterUpperBound(BASE_to_TARGET_r, 2,  190.0);
	
	problem.SetParameterLowerBound(CAM_to_FOREARM_r, 0, -190.0);
	problem.SetParameterUpperBound(CAM_to_FOREARM_r, 0,  190.0);
	problem.SetParameterLowerBound(CAM_to_FOREARM_r, 1, -190.0);
	problem.SetParameterUpperBound(CAM_to_FOREARM_r, 1,  190.0);
	problem.SetParameterLowerBound(CAM_to_FOREARM_r, 2, -190.0);
	problem.SetParameterUpperBound(CAM_to_FOREARM_r, 2,  190.0);
	
	/*std::cout << "" << FOREARM_to_BASE_vec[0].matrix() << "\n\n";
	std::cout << "" << FOREARM_to_BASE_vec[0].linear() << "\n\n";
	
	Eigen::Matrix3d matrix_c = FOREARM_to_BASE_vec[0].linear();
		double rotation_v[9] = {
			matrix_c(0,0), matrix_c(0,1), matrix_c(0,2),
			matrix_c(1,0), matrix_c(1,1), matrix_c(1,2),
			matrix_c(2,0), matrix_c(2,1), matrix_c(2,2)
		};
	printf("%f %f %f\n", rotation_v[0], rotation_v[1], rotation_v[2]);
	printf("%f %f %f\n", rotation_v[3], rotation_v[4], rotation_v[5]);
	printf("%f %f %f\n", rotation_v[6], rotation_v[7], rotation_v[8]);*/

	// Run the solver!
	Solver::Options options;

	options.minimizer_progress_to_stdout = true;
	/*options.linear_solver_type = ceres::DENSE_QR;
	options.preconditioner_type = ceres::CLUSTER_TRIDIAGONAL;
	options.trust_region_strategy_type = ceres::DOGLEG;
	options.dogleg_type = ceres::SUBSPACE_DOGLEG;
	//options.use_inner_iterations = true;*/
	options.max_num_iterations = 10000;
	
	options.callbacks.push_back(new VisualCallback());
	options.update_state_every_iteration = true;

	Solver::Summary summary;
	Solve(options, &problem, &summary);

	std::cout << summary.FullReport() << "\n\n\n";

	std::printf("BASE to TARGET:\n");
	std::printf("\tx = %f\ty = %f\tz = %f\n", BASE_to_TARGET_t[0], BASE_to_TARGET_t[1], BASE_to_TARGET_t[2]);
	std::printf("\tr = %f\tp = %f\tw = %f\n", dtor(BASE_to_TARGET_r[0]), dtor(BASE_to_TARGET_r[1]), dtor(BASE_to_TARGET_r[2]));
	
	std::printf("CAM to FOREARM:\n");
	std::printf("\tx = %f\ty = %f\tz = %f\n", CAM_to_FOREARM_t[0], CAM_to_FOREARM_t[1], CAM_to_FOREARM_t[2]);
	std::printf("\tr = %f\tp = %f\tw = %f\n", dtor(CAM_to_FOREARM_r[0]), dtor(CAM_to_FOREARM_r[1]), dtor(CAM_to_FOREARM_r[2]));
	
	Eigen::Affine3d tmp;
	tmp =
		Eigen::AngleAxisd(dtor(CAM_to_FOREARM_r[0]), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(dtor(CAM_to_FOREARM_r[1]), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(dtor(CAM_to_FOREARM_r[2]), Eigen::Vector3d::UnitX())
	;
	tmp.translation() = Eigen::Vector3d(CAM_to_FOREARM_t[0], CAM_to_FOREARM_t[1], CAM_to_FOREARM_t[2]);
	
	Eigen::Affine3d inv = tmp.inverse();
	
	Eigen::Quaterniond q = Eigen::Quaterniond(inv.rotation());	
	
	std::ofstream lf_os(data_path + "/launch/st_publisher_lidar.launch");
	lf_os << "<launch>\n";
	lf_os << "	<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"stf\" args=\"";
	lf_os << inv.translation()(0) << " " << inv.translation()(1) << " " << inv.translation()(2) << " ";
	lf_os << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
	lf_os << " forearm camera_corrected 1000\"/>\n";
	lf_os << "</launch>";
	lf_os.close();
	
	cv::destroyAllWindows();

	return 0;
}
