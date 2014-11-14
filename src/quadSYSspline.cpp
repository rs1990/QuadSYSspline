#include <quadSYSspline.h>

quadSYSspline::quadSYSspline(){
}
quadSYSspline::quadSYSspline(double time, Eigen::VectorXf current_state,double nn){

	this->current_state = current_state;

	setNN(nn);

	//Construct NN weight parameter vector
	//TODO make sure the w1col and w2col are being built properly
	this->w1col = Eigen::VectorXf(nn*3);
	this->w2col = Eigen::VectorXf(nn*3);
	for(int i=0;w1col.cols();i++){
		w1col(i) = current_state(25+i);
		w2col(i) = current_state(25+nn+i);
	}

	//Initiate trajectoryPlanning parameters
}

void quadSYSspline::trajectoryPlanning(double x0, double y0, double z0){
	double xtf = 20;
	double ytf = 5;
	double ztf = 10;
	double tf = 20;

	//X-axis trajectory planning
	xa3to5(0,0) = 3;
	xa3to5(1,0) = 6;
	xa3to5(2,0) = 1;
	xa3to5(0,1) = 4*tf;
	xa3to5(1,1) = 12*tf;
	xa3to5(2,1) = tf;
	xa3to5(0,2) = 5*tf*tf;
	xa3to5(1,2) = 20*tf*tf;
	xa3to5(2,2) = tf*tf;
	Eigen::Vector3f xDiff(0,0,(xtf-x0)/(tf*tf*tf));

	//y-axis trajectory planning
	ya3to5(0,0) = 3;
	ya3to5(1,0) = 6;
	ya3to5(2,0) = 1;
	ya3to5(0,1) = 4*tf;
	ya3to5(1,1) = 12*tf;
	ya3to5(2,1) = tf;
	ya3to5(0,2) = 5*tf*tf;
	ya3to5(1,2) = 20*tf*tf;
	ya3to5(2,2) = tf*tf;
	Eigen::Vector3f yDiff(0,0,(ytf-y0)/(tf*tf*tf));

	//z-axis trajectory planning
	za3to5(0,0) = 3;
	za3to5(1,0) = 6;
	za3to5(2,0) = 1;
	za3to5(0,1) = 4*tf;
	za3to5(1,1) = 12*tf;
	za3to5(2,1) = tf;
	za3to5(0,2) = 5*tf*tf;
	za3to5(1,2) = 20*tf*tf;
	za3to5(2,2) = tf*tf;
	Eigen::Vector3f zDiff(0,0,(ztf-z0)/(tf*tf*tf));

	Eigen::Vector3f xa3to5Vec = xa3to5.inverse()*xDiff;
	Eigen::Vector3f ya3to5Vec = ya3to5.inverse()*yDiff;
	Eigen::Vector3f za3to5Vec = za3to5.inverse()*zDiff;

	float xD = x0 + xa3to5Vec.transpose()*Eigen::Vector3f(tf*tf*tf,tf*tf*tf*tf,tf*tf*tf*tf*tf);
	float yD = y0 + ya3to5Vec.transpose()*Eigen::Vector3f(tf*tf*tf,tf*tf*tf*tf,tf*tf*tf*tf*tf);
	float zD = z0 + za3to5Vec.transpose()*Eigen::Vector3f(tf*tf*tf,tf*tf*tf*tf,tf*tf*tf*tf*tf);

	Eigen::Vector3f zetaD(xD,yD,zD);
	Eigen::Matrix3f zetaD1_term;
	zetaD1_term(0,0) = 3*xa3to5Vec(0);
	zetaD1_term(0,1) = 4*xa3to5Vec(1);
	zetaD1_term(0,2) = 5*xa3to5Vec(2);
	zetaD1_term(1,0) = 3*ya3to5Vec(0);
	zetaD1_term(1,1) = 3*ya3to5Vec(1);
	zetaD1_term(1,2) = 3*ya3to5Vec(2);
	zetaD1_term(2,0) = 3*za3to5Vec(0);
	zetaD1_term(2,1) = 3*za3to5Vec(1);
	zetaD1_term(2,2) = 3*za3to5Vec(2);
	Eigen::Vector3f zetaD1_term_2(tf*tf,tf*tf*tf,tf*tf*tf);
	zetaD1 = zetaD1_term * zetaD1_term_2;

	Eigen::Matrix3f zetaD2_term;
	zetaD2_term(0,0) = 3*xa3to5Vec(0);
	zetaD2_term(0,1) = 4*xa3to5Vec(1);
	zetaD2_term(0,2) = 5*xa3to5Vec(2);
	zetaD2_term(1,0) = 3*ya3to5Vec(0);
	zetaD2_term(1,1) = 3*ya3to5Vec(1);
	zetaD2_term(1,2) = 3*ya3to5Vec(2);
	zetaD2_term(2,0) = 3*za3to5Vec(0);
	zetaD2_term(2,1) = 3*za3to5Vec(1);
	zetaD2_term(2,2) = 3*za3to5Vec(2);
	Eigen::Vector3f zetaD2_term_2(tf*tf,tf*tf*tf,tf*tf*tf);
	zetaD2 = zetaD2_term * zetaD2_term_2;
}

void quadSYSspline::calculateNNWeightParameters(){
	w1 = Eigen::MatrixXf(nn,3);
	w2 = Eigen::MatrixXf(nn,3);

	//Populating w1 and w2 matricies from the vectors
	for(int i=0;i<nn;i++){
		w1(i,1) = w1col(i);
		w1(i,2) = w1col(nn+i);
		w1(i,3) = w1col((2*nn)+i);
		w2(i,1) = w2col(i);
		w2(i,2) = w2col(nn+i);
		w2(i,3) = w2col((2*nn)+i);
	}
}
void quadSYSspline::setW1col(Eigen::VectorXf w1col){
	w1col = Eigen::VectorXf(nn*3);
	this->w1col = w1col;
}

void quadSYSspline::setW2col(Eigen::VectorXf w2col){
	this->w2col = w2col;
}

void quadSYSspline::setNN(double nn){
	this->nn  = nn;
}

int main(){
	quadSYSspline test;
	return 0;
}
