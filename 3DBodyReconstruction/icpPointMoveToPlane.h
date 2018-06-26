#pragma once
#include "icp.h"
#include "Skeleton.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/NonLinearOptimization>
class icpPointMoveToPlane :	public Icp
{
public:
	icpPointMoveToPlane(double *M, const int32_t M_num, const int32_t dim, double *Mnormal, t_Visual *m_Model = nullptr, t_Visual *partModel = nullptr,  int nonrigidTpye = 0, const int32_t num_neighbors = 10, const double flatness = 5.0) : Icp(M, M_num, dim) {
		
			M_normal = computeNormals(num_neighbors, flatness);
					
			M_normalsource = Mnormal;

			pModel = partModel;
			mModel = m_Model;
			NONRIGID_TYPE = nonrigidTpye;
	}
	virtual ~icpPointMoveToPlane()
	{
		delete M_normal;
		M_normalsource = nullptr;

		delete M_normalsource;
		pModel = nullptr;
		delete pModel;
		mModel = nullptr;
		delete mModel;
	}
public:
	// normals of model points
	double *M_normal;
	double *S_normal;
	double *M_normalsource;
	t_Visual *pModel,*mModel;
	int NONRIGID_TYPE=0;//0:icp nonrigid 1:TPS
private:

	double fitStep(double *T, const int32_t T_num, selfMatrix::Matrix &R, selfMatrix::Matrix &t, const std::vector<int32_t> &active);
	std::vector<int32_t> getInliers_nonrigid(double *T, double *Tnormal, const int32_t T_num, const selfMatrix::Matrix &R, const selfMatrix::Matrix &t, const double indist);
	std::vector<int32_t> getInliers(double *T,const int32_t T_num, const selfMatrix::Matrix &R, const selfMatrix::Matrix &t, const double indist);

	// utility functions to compute normals from the model tree
	void computeNormal(const kdtree::KDTreeResultVector &neighbors, double *M_normal, const double flatness);
	double* computeNormals(const int32_t num_neighbors, const double flatness);
	//º∆À„¡⁄”Úµƒµ„
	BOOL NeiberVertex(t_Visual* originalVisual, int pindex, std::vector<int> &neiberIndex);
	double Gauss_distriValue(const double &distan);
	BOOL RotationMatrix(metchPoints* dmetchPoints, int machePnum, Eigen::Matrix3d &R, Eigen::Vector3d &T);
};

