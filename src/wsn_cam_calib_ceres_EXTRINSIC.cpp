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

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "glog/logging.h"
//#include <ceres/rotation.h>
//#include "ceres_costs_utils.hpp"
//#include "basic_types.h"

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "calibration_file_reader_EXTRINSIC.cpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// This is how Newman converts to AA. Potentially suspect.
/*T nominal_axis_x[3]; // Nominally we move along camera x, y and z axes
T nominal_axis_y[3];
T nominal_axis_z[3];

nominal_axis_x[0] = T(1.0);
nominal_axis_x[1] = T(0.0);
nominal_axis_x[2] = T(0.0);

nominal_axis_y[0] = T(0.0);
nominal_axis_y[1] = T(1.0);
nominal_axis_y[2] = T(0.0);

nominal_axis_z[0] = T(0.0);
nominal_axis_z[1] = T(0.0);
nominal_axis_z[2] = T(1.0);

T rotation_axis[3]; // Here we skew the axes of motion w/ 3 rotation params
rotation_axis[0] = c_p3[0];
rotation_axis[1] = c_p3[1];
rotation_axis[2] = c_p3[2]; //T(0.0);

T motion_axis_x[3];
T motion_axis_y[3];
T motion_axis_z[3];
ceres::AngleAxisRotatePoint(rotation_axis, nominal_axis_x, motion_axis_x);
ceres::AngleAxisRotatePoint(rotation_axis, nominal_axis_y, motion_axis_y);
ceres::AngleAxisRotatePoint(rotation_axis, nominal_axis_z, motion_axis_z);*/


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
	T aa [3];
	ceres::RotationMatrixToAngleAxis(R, aa);
	ceres::AngleAxisRotatePoint(aa, point_in, point_out);
	point_out[0] = point_out[0] + translation[0];
	point_out[1] = point_out[1] + translation[1];
	point_out[2] = point_out[2] + translation[2];
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
	ceres::AngleAxisRotatePoint(rotation, point_in, point_out);
	point_out[0] = point_out[0] + translation[0];
	point_out[1] = point_out[1] + translation[1];
	point_out[2] = point_out[2] + translation[2];
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
	T& point_x, T& point_y, T& point_z,
	T k1, T k2, T k3, T p1, T p2,
	T fx, T fy, T cx, T cy,
	T ox, T oy,
	T residual[2]
);
template <typename T>
inline void cameraPntResidualDist(
	T& point_x, T& point_y, T& point_z,
	T k1, T k2, T k3, T p1, T p2,
	T fx, T fy, T cx, T cy,
	T ox, T oy,
	T residual[2]
){
	/*std::cout<< "Goal u " << ox << "\n";
	std::cout<< "Goal v " << oy << "\n";*/
	
	//std::cout<< "FL in " << fx << "\n";

	//Projection
	T xp1 = point_x;
	T yp1 = point_y;
	T zp1 = point_z;
	T up;
	T vp;

	up = cx * zp1 + fx * xp1;
	vp = cy * zp1 + fy * yp1;

	if(zp1 != T(0)){
		up = up / zp1;
		vp = vp / zp1;
	}

	//Distortion.
	T up_shrunk = up / cx - T(1.0);
	T vp_shrunk = vp / cy - T(1.0);

	T xp2 = up_shrunk * up_shrunk;/* x^2 */
	T yp2 = vp_shrunk * vp_shrunk;/* y^2 */
	T r2 = xp2 + yp2; /* r^2 radius squared */
	T r4 = r2 * r2; /* r^4 */
	T r6 = r2 * r4; /* r^6 */

	/* apply the distortion coefficients to refine pixel location */
	T xpp = up_shrunk + up_shrunk * (
			k1 * r2 // 2nd order term
			+ k2 * r4// 4th order term
			+ k3 * r6
		)// 6th order term
		+ p2 * (r2 + T(2.0) * xp2)// tangential
		+ p1 * up_shrunk * vp_shrunk * T(2.0);// other tangential term
	;
	T ypp = vp_shrunk + vp_shrunk * (
			k1 * r2 // 2nd order term
			+ k2 * r4// 4th order term
			+ k3 * r6// 6th order term
		)
		+ p1 * (r2 + T(2.0) * yp2)// tangential term
		+ p2 * up_shrunk * vp_shrunk * T(2.0);// other tangential term
	;

	T up_dist = (xpp + T(1.0)) * cx;
	T vp_dist = (ypp + T(1.0)) * cy;

	residual[0] = up_dist - ox;
	residual[1] = vp_dist - oy;

	//std::getchar();
}

class ExtrinsicCalEntry{
public:
	// A templated cost functor that implements the residual
	template<typename T>
	bool operator()(//TODO Why are all these const / should all these be const?
		const T* CAM_to_FLANGE_V_t, const T* CAM_to_FLANGE_V_r, 
		const T* FLANGE_S_to_HEAD_t, const T* FLANGE_S_to_HEAD_r,
	T* residual) const {
	
	//1: Transform target points into camera frame
	//	CAM_to_POINT = CAM_to_FLANGE_V * FLANGE_V_to_BASE * BASE_to_FLANGE_S * FLANGE_S_to_HEAD * HEAD_to_POINT
	
	//	1a: HEAD_to_POINT
	//	Spray always faces straight down and is 0.2 above the surface, so this is constant. 
	T HEAD_to_POINT [3];
	HEAD_to_POINT[0] = T(0.0);
	HEAD_to_POINT[1] = T(0.0);
	HEAD_to_POINT[2] = T(0.2);//TODO Paramertrize this!
	
	//	1b: FLANGE_S_to_HEAD * HEAD_to_POINT
	T FLANGE_S_to_POINT [3];
	transformPoint_euler(FLANGE_S_to_HEAD_t, FLANGE_S_to_HEAD_r, HEAD_to_POINT, FLANGE_S_to_POINT);
	
	//	1c: BASE_to_FLANGE_S * FLANGE_S_to_HEAD * HEAD_to_POINT
	T BASE_to_FLANGE_S_normed [3];
	BASE_to_FLANGE_S_normed[0] = T(BASE_to_FLANGE_S_rx) * T(BASE_to_FLANGE_S_ra);
	BASE_to_FLANGE_S_normed[1] = T(BASE_to_FLANGE_S_ry) * T(BASE_to_FLANGE_S_ra);
	BASE_to_FLANGE_S_normed[2] = T(BASE_to_FLANGE_S_rz) * T(BASE_to_FLANGE_S_ra);
	
	T BASE_to_FLANGE_S_translation [3];
	BASE_to_FLANGE_S_translation[0] = T(BASE_to_FLANGE_S_x);
	BASE_to_FLANGE_S_translation[1] = T(BASE_to_FLANGE_S_y);
	BASE_to_FLANGE_S_translation[2] = T(BASE_to_FLANGE_S_z);
	
	T BASE_to_POINT [3];
	transformPoint_aa(BASE_to_FLANGE_S_translation, BASE_to_FLANGE_S_normed, FLANGE_S_to_POINT, BASE_to_POINT);
	
	//	1d: FLANGE_V_to_BASE * BASE_to_FLANGE_S * FLANGE_S_to_HEAD * HEAD_to_POINT
	T FLANGE_V_to_BASE_normed [3];
	FLANGE_V_to_BASE_normed[0] = T(FLANGE_V_to_BASE_rx) * T(FLANGE_V_to_BASE_ra);
	FLANGE_V_to_BASE_normed[1] = T(FLANGE_V_to_BASE_ry) * T(FLANGE_V_to_BASE_ra);
	FLANGE_V_to_BASE_normed[2] = T(FLANGE_V_to_BASE_rz) * T(FLANGE_V_to_BASE_ra);
	
	T FLANGE_V_to_BASE_translation [3];
	FLANGE_V_to_BASE_translation[0] = T(FLANGE_V_to_BASE_x);
	FLANGE_V_to_BASE_translation[1] = T(FLANGE_V_to_BASE_y);
	FLANGE_V_to_BASE_translation[2] = T(FLANGE_V_to_BASE_z);

	T FLANGE_V_to_POINT [3];
	transformPoint_aa(FLANGE_V_to_BASE_translation, FLANGE_V_to_BASE_normed, BASE_to_POINT, FLANGE_V_to_POINT);

	//	1e: CAM_to_FLANGE_V * FLANGE_V_to_BASE * BASE_to_FLANGE_S * FLANGE_S_to_HEAD * HEAD_to_POINT
	T CAM_to_POINT [3];
	transformPoint_euler(CAM_to_FLANGE_V_t, CAM_to_FLANGE_V_r, FLANGE_V_to_POINT, CAM_to_POINT);

	/** compute project point into image plane and compute residual */
	T ox = T(image_pixel_u);
	T oy = T(image_pixel_v);

	//TODO: Back-check if these are passing through the transform converter properly when not identity.
	cameraPntResidualDist(
		CAM_to_POINT[0], CAM_to_POINT[1], CAM_to_POINT[2],
		T(k1), T(k2), T(k3), T(p1), T(p2),
		T(f_X), T(f_Y), T(c_X), T(c_Y),
		ox, oy,
		residual
	);

	return true;
}
	//Dumbest thing...
	ExtrinsicCalEntry(
		const double image_pixel_u_, const double image_pixel_v_,

		const double FLANGE_V_to_BASE_x_, const double FLANGE_V_to_BASE_y_, const double FLANGE_V_to_BASE_z_,
		const double FLANGE_V_to_BASE_ax_, const double FLANGE_V_to_BASE_ay_, const double FLANGE_V_to_BASE_az_, const double FLANGE_V_to_BASE_aa_,
		
		const double BASE_to_FLANGE_S_x_, const double BASE_to_FLANGE_S_y_, const double BASE_to_FLANGE_S_z_,
		const double BASE_to_FLANGE_S_ax_, const double BASE_to_FLANGE_S_ay_, const double BASE_to_FLANGE_S_az_, const double BASE_to_FLANGE_S_aa_,
		
		const double f_X_, const double f_Y_, const double c_X_, const double c_Y_,
		const double k1_, const double k2_, const double k3_, const double p1_, const double p2_
	):
		image_pixel_u(image_pixel_u_), image_pixel_v(image_pixel_v_),
		
		FLANGE_V_to_BASE_x(FLANGE_V_to_BASE_x_), FLANGE_V_to_BASE_y(FLANGE_V_to_BASE_y_), FLANGE_V_to_BASE_z(FLANGE_V_to_BASE_z_),
		FLANGE_V_to_BASE_rx(FLANGE_V_to_BASE_ax_), FLANGE_V_to_BASE_ry(FLANGE_V_to_BASE_ay_), FLANGE_V_to_BASE_rz(FLANGE_V_to_BASE_az_), FLANGE_V_to_BASE_ra(FLANGE_V_to_BASE_aa_),
		
		BASE_to_FLANGE_S_x(BASE_to_FLANGE_S_x_), BASE_to_FLANGE_S_y(BASE_to_FLANGE_S_y_), BASE_to_FLANGE_S_z(BASE_to_FLANGE_S_z_),
		BASE_to_FLANGE_S_rx(BASE_to_FLANGE_S_ax_), BASE_to_FLANGE_S_ry(BASE_to_FLANGE_S_ay_), BASE_to_FLANGE_S_rz(BASE_to_FLANGE_S_az_), BASE_to_FLANGE_S_ra(BASE_to_FLANGE_S_aa_),
		
		f_X(f_X_), f_Y(f_Y_), c_X(c_X_), c_Y(c_Y_),
		k1(k1_), k2(k2_), k3(k3_), p1(p1_), p2(p2_)
	{}

	static ceres::CostFunction* Create(	
		const double image_pixel_u_, const double image_pixel_v_,
			
		const double FLANGE_V_to_BASE_x_, const double FLANGE_V_to_BASE_y_, const double FLANGE_V_to_BASE_z_,
		const double FLANGE_V_to_BASE_ax_, const double FLANGE_V_to_BASE_ay_, const double FLANGE_V_to_BASE_az_, const double FLANGE_V_to_BASE_aa_,
		
		const double BASE_to_FLANGE_S_x_, const double BASE_to_FLANGE_S_y_, const double BASE_to_FLANGE_S_z_,
		const double BASE_to_FLANGE_S_ax_, const double BASE_to_FLANGE_S_ay_, const double BASE_to_FLANGE_S_az_, const double BASE_to_FLANGE_S_aa_,
		
		//Global projection constants.
		const double f_X_, const double f_Y_, const double c_X_, const double c_Y_,
		const double k1_, const double k2_, const double k3_, const double p1_, const double p2_
	){
		return new ceres::AutoDiffCostFunction<ExtrinsicCalEntry, 2, 3, 3, 3, 3>(new ExtrinsicCalEntry(
			image_pixel_u_, image_pixel_v_,
			
			FLANGE_V_to_BASE_x_, FLANGE_V_to_BASE_y_, FLANGE_V_to_BASE_z_,
			FLANGE_V_to_BASE_ax_, FLANGE_V_to_BASE_ay_, FLANGE_V_to_BASE_az_, FLANGE_V_to_BASE_aa_,
			
			BASE_to_FLANGE_S_x_, BASE_to_FLANGE_S_y_, BASE_to_FLANGE_S_z_,
			BASE_to_FLANGE_S_ax_, BASE_to_FLANGE_S_ay_, BASE_to_FLANGE_S_az_, BASE_to_FLANGE_S_aa_,
			
			f_X_, f_Y_, c_X_, c_Y_,
			k1_, k2_, k3_, p1_, p2_
		));
	}


	//Member vars- all the perpoint constants
	double image_pixel_u; /** observed x location of object in image */
	double image_pixel_v; /** observed y location of object in image */
	
	double FLANGE_V_to_BASE_x, FLANGE_V_to_BASE_y, FLANGE_V_to_BASE_z;
	double FLANGE_V_to_BASE_rx, FLANGE_V_to_BASE_ry, FLANGE_V_to_BASE_rz, FLANGE_V_to_BASE_ra;
	double BASE_to_FLANGE_S_x, BASE_to_FLANGE_S_y, BASE_to_FLANGE_S_z;
	double BASE_to_FLANGE_S_rx, BASE_to_FLANGE_S_ry, BASE_to_FLANGE_S_rz, BASE_to_FLANGE_S_ra;
	
	double f_X, f_Y; /** Projection intrinsics */
	double c_X, c_Y;
	double k1, k2, k3, p1, p2; /** Distortion intrinsics */
	
};

int main(int argc, char** argv) {
	google::InitGoogleLogging(argv[0]);

	// Storage for input values constant for each point.
	std::vector<Eigen::Affine3d> BASE_to_FLANGE_S_vec;
	std::vector<Eigen::Affine3d> FLANGE_V_to_BASE_vec;
	std::vector<Eigen::Vector2d> image_pixel_vec;
	
	//read the calibration file:
	//TODO Parametrize dis
	std::string fname("/home/print/data.csv");
	if (!read_calibration_file(fname, BASE_to_FLANGE_S_vec, FLANGE_V_to_BASE_vec, image_pixel_vec)) {
		cout<<"could not open file "<<fname<<"; quitting"<<endl;
		return 1;
	}
	cout<<"calibration file has been read in"<<endl;
	
	
	// Build the problem.
	Problem problem;
	int nlines = (int) image_pixel_vec.size();
	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).

	//Known values (one per data point)
	double image_u,image_v;
	double BASE_to_FLANGE_S [16];
	double FLANGE_V_to_BASE [16];

	//Known values (constant)
	//TODO Initialize from an external source where the intrinsic calibration results are output.
	double projection_matrix[12] = {
		434.8289178042981,	0.0,			500.5,	0.0,
		0.0,			434.8289178042981,	330.5,	0.0,
		0.0,			0.0,			1.0,	0.0
	};
	double distortion[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

	//Unknown values
	double CAM_to_FLANGE_V_t [3];
	double CAM_to_FLANGE_V_r [3];
	double FLANGE_S_to_HEAD_t [3];
	double FLANGE_S_to_HEAD_r [3];
	
	//Initialize the unknowns
	//TODO Find a way to inform this parametrically
	//TODO For simulation tests, get these ground truths from the simulation
	CAM_to_FLANGE_V_t[0] = 0.0;
	CAM_to_FLANGE_V_t[1] = 0.0;
	CAM_to_FLANGE_V_t[2] = 0.0;
	CAM_to_FLANGE_V_r[0] = 0.0;
	CAM_to_FLANGE_V_r[1] = 0.0;
	CAM_to_FLANGE_V_r[2] = 0.0;
	FLANGE_S_to_HEAD_t[0] = 0.0;
	FLANGE_S_to_HEAD_t[1] = 0.0;
	FLANGE_S_to_HEAD_t[2] = 0.0;
	FLANGE_S_to_HEAD_r[0] = 0.0;
	FLANGE_S_to_HEAD_r[1] = 0.0;
	FLANGE_S_to_HEAD_r[2] = 0.0;
	
	for (int i=0;i<nlines;i++) {//For each data entry...
		//Add the per-point constants
		Eigen::Vector3d translation_v = FLANGE_V_to_BASE_vec[i].translation();
		Eigen::AngleAxisd rotation_v = Eigen::AngleAxisd(FLANGE_V_to_BASE_vec[i].rotation());
		Eigen::Vector3d translation_s = BASE_to_FLANGE_S_vec[i].translation();
		Eigen::AngleAxisd rotation_s = Eigen::AngleAxisd(BASE_to_FLANGE_S_vec[i].rotation());
		
		CostFunction *cost_function = ExtrinsicCalEntry::Create(
			image_pixel_vec[i][0], image_pixel_vec[i][1],
			
			translation_v[0], translation_v[1], translation_v[2],
			rotation_v.axis()[0], rotation_v.axis()[1], rotation_v.axis()[2], rotation_v.angle(),
			
			translation_s[0], translation_s[1], translation_s[2],
			rotation_s.axis()[0], rotation_s.axis()[1], rotation_s.axis()[2], rotation_s.angle(),
			
			//Global projection constants.
			projection_matrix[0], projection_matrix[5], projection_matrix[2], projection_matrix[6],
			distortion[0], distortion[1], distortion[2], distortion[3], distortion[4]
		);
		
		//Add the parameters to optimize
		problem.AddResidualBlock(cost_function, NULL,
			CAM_to_FLANGE_V_t, CAM_to_FLANGE_V_r,
			FLANGE_S_to_HEAD_t, FLANGE_S_to_HEAD_r
		);
	}

	// Run the solver!
	Solver::Options options;

	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 1000;

	Solver::Summary summary;
	Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << "\n\n\n";

	std::printf("CAM_to_FLANGE:\n");
	std::printf("\tx = %f\ty = %f\tz = %f\n", CAM_to_FLANGE_V_t[0], CAM_to_FLANGE_V_t[1], CAM_to_FLANGE_V_t[2]);
	std::printf("\tr = %f\tp = %f\tw = %f\n", CAM_to_FLANGE_V_r[0], CAM_to_FLANGE_V_r[1], CAM_to_FLANGE_V_r[2]);
	std::printf("\n");
	std::printf("FLANGE_to_HEAD:\n");
	std::printf("\tx = %f\ty = %f\tz = %f\n", FLANGE_S_to_HEAD_t[0], FLANGE_S_to_HEAD_t[1], FLANGE_S_to_HEAD_t[2]);
	std::printf("\tr = %f\tp = %f\tw = %f\n", FLANGE_S_to_HEAD_r[0], FLANGE_S_to_HEAD_r[1], FLANGE_S_to_HEAD_r[2]);

	return 0;
}
