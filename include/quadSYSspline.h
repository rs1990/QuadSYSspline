#ifndef QUAD_SYS_SPLINE_H_
#define QUAD_SYS_SPLINE_H_

#include <iostream>
#include <eigen3/Eigen/Dense>

class quadSYSspline{
private:
	int stop;
	double time;
	Eigen::VectorXf current_state;

	double phiDot;
	Eigen::Vector2f thetaDot;
	Eigen::Vector3f siDot;

	//Quadrotor state-space system
	Eigen::Vector3f zeta;
	Eigen::Vector3f eta;
	Eigen::Vector3f zetaDot;
	Eigen::Vector3f etaDot;
	
	//Diff parameters
	Eigen::Vector3f sd1;
	Eigen::Vector3f sd2;
	
	//Integration parameters
	Eigen::Vector3f r1i;
	Eigen::Vector3f r2i;
	
	//NN weight parameters
	//Need to be properly initialized
	Eigen::VectorXf w1col;
	Eigen::VectorXf w2col;
	Eigen::MatrixXf w1;
	Eigen::MatrixXf w2;

	//Trajectory Planning
	Eigen::MatrixXf xa3to5;
	Eigen::MatrixXf ya3to5;
	Eigen::MatrixXf za3to5;

	//Global parameters
	double nn;
	Eigen::Vector3f zetaD1;
	Eigen::Vector3f zetaD2;
public:
	quadSYSspline();
	quadSYSspline(double, Eigen::VectorXf current_state,double nn);
	void calculateNNWeightParameters();
	void trajectoryPlanning(double x0, double y0, double z0);

	//Getters and Setters
	void setW1col(Eigen::VectorXf w1col);
	void setW2col(Eigen::VectorXf w2col);
	void setNN(double nn);


};



#endif /**QUAD_SYS_SPLINE_H_**/
