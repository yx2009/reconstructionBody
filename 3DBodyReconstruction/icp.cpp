#include "stdafx.h"
#include "icp.h"

using namespace std;
Icp::Icp (double *M,const int32_t M_num,const int32_t dim) :
  dim(dim), sub_step(10), max_iter(200), min_delta(1e-6) {
  
  // check for correct dimensionality
  if (dim!=2 && dim!=3) {
    cout << "ERROR: LIBICP works only for data of dimensionality 2 or 3" << endl;
    M_tree = 0;
    return;
  }
  
  // check for minimum number of points
  if (M_num<5) {
    cout << "ERROR: LIBICP works only with at least 5 model points" << endl;
    M_tree = 0;
    return;
  }

  // copy model points to M_data
  M_data.resize(boost::extents[M_num][dim]);
  for (int32_t m=0; m<M_num; m++)
    for (int32_t n=0; n<dim; n++)
      M_data[m][n] = (float)M[m*dim+n];

  // build a kd tree from the model point cloud
  M_tree = new kdtree::KDTree(M_data);
}

Icp::~Icp() {
	if (M_tree)
		delete M_tree;
}

void Icp::fit_nonrigid(double *T,double *Tnormal, const int32_t T_num, selfMatrix::Matrix &R, selfMatrix::Matrix &t, const double indist) {

	// make sure we have a model tree
	if (!M_tree) {
		cout << "ERROR: No model available." << endl;
		return;
	}

	// check for minimum number of points
	if (T_num < 5) {
		cout << "ERROR: Icp works only with at least 5 template points" << endl;
		return;
	}

	// coarse matching
	vector<int32_t> active;
	active.clear();
	if (indist <= 0) {
		active.clear();
		for (int32_t i = 0; i < T_num; i++)
			active.push_back(i);
	}
	else {
		active = getInliers_nonrigid(T, Tnormal, T_num, R, t, indist);
	}
	fitStep(T, T_num, R, t, active);

}

void Icp::fit(double *T, const int32_t T_num, selfMatrix::Matrix &R, selfMatrix::Matrix &t, const double indist) {

	// make sure we have a model tree
	if (!M_tree) {
		cout << "ERROR: No model available." << endl;
		return;
	}

	// check for minimum number of points
	if (T_num < 5) {
		cout << "ERROR: Icp works only with at least 5 template points" << endl;
		return;
	}

	// coarse matching
	vector<int32_t> active;
	active.clear();
	for (int32_t i = 0; i < T_num; i += sub_step)
		active.push_back(i);
	fitIterate(T, T_num, R, t, active);

	// fine matching
	if (indist <= 0) {
		active.clear();
		for (int32_t i = 0; i < T_num; i++)
			active.push_back(i);
	}
	else {
		active = getInliers(T, T_num, R, t, indist);
	}
	fitIterate(T, T_num, R, t, active);
}

void Icp::fitIterate(double *T,const int32_t T_num,selfMatrix::Matrix &R,selfMatrix::Matrix &t,const std::vector<int32_t> &active) {
  
  // check if we have at least 5 active points
  if (active.size()<5)
    return;
  
  // iterate until convergence
  for (int32_t iter=0; iter<max_iter; iter++)
    if (fitStep(T,T_num,R,t,active)<min_delta)
      break;
}
