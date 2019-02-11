#pragma once
#define NOMINMAX
#include <igl/colon.h>
#include <igl/directed_edge_orientations.h>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/PI.h>
#include <igl/lbs_matrix.h>
#include <igl/deform_skeleton.h>
#include <igl/dqs.h>
#include <igl/readDMAT.h>
#include <igl/readOFF.h>
#include <igl/arap.h>
#include <time.h>
//#include <igl/opengl/glfw/Viewer.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

#include "Skeleton.h"

using namespace igl;
using namespace std;
typedef
std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
RotationList;

const Eigen::RowVector3d sea_green(70. / 255., 252. / 255., 167. / 255.);


template <
	typename DerivedV,
	typename DerivedF,
	typename Derivedb>
	IGL_INLINE bool arapPrecompute(
		const Eigen::PlainObjectBase<DerivedV> & V,//顶点，n*3
		const Eigen::PlainObjectBase<DerivedF> & F,//三角面片，n*3
		const int dim,//维度一般是3或者2
		const Eigen::PlainObjectBase<Derivedb> & b,//num of constrains
		ARAPData & data)
{
	typedef typename DerivedV::Scalar Scalar;
	// number of vertices
	const int n = V.rows();
	data.n = n;
	assert((b.size() == 0 || b.maxCoeff() < n) && "b out of bounds");
	assert((b.size() == 0 || b.minCoeff() >= 0) && "b out of bounds");

	// remember b
	data.b = b;
	//assert(F.cols() == 3 && "For now only triangles");
	// dimension
	//const int dim = V.cols();
	assert((dim == 3 || dim == 2) && "dim should be 2 or 3");
	data.dim = dim;
	//assert(dim == 3 && "Only 3d supported");
	// Defaults
	data.f_ext = Eigen::MatrixXd::Zero(n, data.dim);

	assert(data.dim <= V.cols() && "solve dim should be <= embedding");
	bool flat = (V.cols() - data.dim) == 1;

	DerivedV plane_V;
	DerivedF plane_F;
	typedef Eigen::SparseMatrix<Scalar> SparseMatrixS;
	SparseMatrixS ref_map, ref_map_dim;
	if (flat)
	{
		project_isometrically_to_plane(V, F, plane_V, plane_F, ref_map);
		repdiag(ref_map, dim, ref_map_dim);
	}
	const Eigen::PlainObjectBase<DerivedV>& ref_V = (flat ? plane_V : V);
	const Eigen::PlainObjectBase<DerivedF>& ref_F = (flat ? plane_F : F);
	SparseMatrixS L;//
	cotmatrix(V, F, L);

	ARAPEnergyType eff_energy = data.energy;
	if (eff_energy == ARAP_ENERGY_TYPE_DEFAULT)
	{
		switch (F.cols())
		{
		case 3:
			if (data.dim == 3)
			{
				eff_energy = ARAP_ENERGY_TYPE_SPOKES_AND_RIMS;
			}
			else
			{
				eff_energy = ARAP_ENERGY_TYPE_ELEMENTS;
			}
			break;
		case 4:
			eff_energy = ARAP_ENERGY_TYPE_ELEMENTS;
			break;
		default:
			assert(false);
		}
	}


	// Get covariance scatter matrix, when applied collects the covariance
	// matrices used to fit rotations to during optimization
	covariance_scatter_matrix(ref_V, ref_F, eff_energy, data.CSM);
	if (flat)
	{
		data.CSM = (data.CSM * ref_map_dim.transpose()).eval();
	}
	assert(data.CSM.cols() == V.rows()*data.dim);

	// Get group sum scatter matrix, when applied sums all entries of the same
	// group according to G
	Eigen::SparseMatrix<double> G_sum;
	if (data.G.size() == 0)
	{
		if (eff_energy == ARAP_ENERGY_TYPE_ELEMENTS)
		{
			speye(F.rows(), G_sum);
		}
		else
		{
			speye(n, G_sum);
		}
	}
	else
	{
		// groups are defined per vertex, convert to per face using mode
		if (eff_energy == ARAP_ENERGY_TYPE_ELEMENTS)
		{
			Eigen::Matrix<int, Eigen::Dynamic, 1> GG;
			Eigen::MatrixXi GF(F.rows(), F.cols());
			for (int j = 0; j < F.cols(); j++)
			{
				Eigen::Matrix<int, Eigen::Dynamic, 1> GFj;
				slice(data.G, F.col(j), GFj);
				GF.col(j) = GFj;
			}
			mode<int>(GF, 2, GG);
			data.G = GG;
		}
		//printf("group_sum_matrix()\n");
		group_sum_matrix(data.G, G_sum);
	}
	Eigen::SparseMatrix<double> G_sum_dim;
	repdiag(G_sum, data.dim, G_sum_dim);
	assert(G_sum_dim.cols() == data.CSM.rows());
	data.CSM = (G_sum_dim * data.CSM).eval();


	arap_rhs(ref_V, ref_F, data.dim, eff_energy, data.K);
	if (flat)
	{
		data.K = (ref_map_dim * data.K).eval();
	}
	assert(data.K.rows() == data.n*data.dim);

	Eigen::SparseMatrix<double> Q = (-L).eval();

	if (data.with_dynamics)
	{
		const double h = data.h;
		assert(h != 0);
		Eigen::SparseMatrix<double> M;
		massmatrix(V, F, MASSMATRIX_TYPE_DEFAULT, data.M);
		const double dw = (1. / data.ym)*(h*h);
		Eigen::SparseMatrix<double> DQ = dw * 1. / (h*h)*data.M;
		Q += DQ;
		// Dummy external forces
		data.f_ext = Eigen::MatrixXd::Zero(n, data.dim);
		data.vel = Eigen::MatrixXd::Zero(n, data.dim);
	}

	return min_quad_with_fixed_precompute(
		Q, b, Eigen::SparseMatrix<double>(), true, data.solver_data);
}

template <
	typename Derivedbc,
	typename DerivedU>
	IGL_INLINE bool arapSolve(
		const Eigen::PlainObjectBase<Derivedbc> & bc,
		ARAPData & data,
		Eigen::PlainObjectBase<DerivedU> & U)
{
	using namespace std;
	assert(data.b.size() == bc.rows());
	if (bc.size() > 0)
	{
		assert(bc.cols() == data.dim && "bc.cols() match data.dim");
	}
	const int n = data.n;
	int iter = 0;
	if (U.size() == 0)
	{
		// terrible initial guess.. should at least copy input mesh
#ifndef NDEBUG
		cerr << "arap_solve: Using terrible initial guess for U. Try U = V." << endl;
#endif
		U = Eigen::MatrixXd::Zero(data.n, data.dim);
	}
	else
	{
		assert(U.cols() == data.dim && "U.cols() match data.dim");
	}
	// changes each arap iteration
	Eigen::MatrixXd U_prev = U;
	// doesn't change for fixed with_dynamics timestep
	Eigen::MatrixXd U0;
	if (data.with_dynamics)
	{
		U0 = U_prev;
	}
	while (iter < data.max_iter)
	{
		U_prev = U;
		// enforce boundary conditions exactly
		for (int bi = 0; bi < bc.rows(); bi++)
		{
			//U.row(data.b(bi)) = bc.row(bi);
		}

		const auto & Udim = U.replicate(data.dim, 1);
		assert(U.cols() == data.dim);
		// As if U.col(2) was 0
		Eigen::MatrixXd S = data.CSM * Udim;
		// THIS NORMALIZATION IS IMPORTANT TO GET SINGLE PRECISION SVD CODE TO WORK
		// CORRECTLY.
		S /= S.array().abs().maxCoeff();

		const int Rdim = data.dim;
		Eigen::MatrixXd R(Rdim, data.CSM.rows());
		if (R.rows() == 2)
		{
			fit_rotations_planar(S, R);
		}
		else
		{
			fit_rotations(S, true, R);
			//#ifdef __SSE__ // fit_rotations_SSE will convert to float if necessary
			//      fit_rotations_SSE(S,R);
			//#else
			//      fit_rotations(S,true,R);
			//#endif
		}
		//for(int k = 0;k<(data.CSM.rows()/dim);k++)
		//{
		//  R.block(0,dim*k,dim,dim) = MatrixXd::Identity(dim,dim);
		//}


		// Number of rotations: #vertices or #elements
		int num_rots = data.K.cols() / Rdim / Rdim;
		// distribute group rotations to vertices in each group
		Eigen::MatrixXd eff_R;
		if (data.G.size() == 0)
		{
			// copy...
			eff_R = R;
		}
		else
		{
			eff_R.resize(Rdim, num_rots*Rdim);
			for (int r = 0; r < num_rots; r++)
			{
				eff_R.block(0, Rdim*r, Rdim, Rdim) =
					R.block(0, Rdim*data.G(r), Rdim, Rdim);
			}
		}

		Eigen::MatrixXd Dl;
		if (data.with_dynamics)
		{
			assert(data.M.rows() == n &&
				"No mass matrix. Call arap_precomputation if changing with_dynamics");
			const double h = data.h;
			assert(h != 0);
			//Dl = 1./(h*h*h)*M*(-2.*V0 + Vm1) - fext;
			// data.vel = (V0-Vm1)/h
			// h*data.vel = (V0-Vm1)
			// -h*data.vel = -V0+Vm1)
			// -V0-h*data.vel = -2V0+Vm1
			const double dw = (1. / data.ym)*(h*h);
			Dl = dw * (1. / (h*h)*data.M*(-U0 - h*data.vel) - data.f_ext);
		}

		Eigen::VectorXd Rcol;
		columnize(eff_R, num_rots, 2, Rcol);
		Eigen::VectorXd Bcol = -data.K * Rcol;
		assert(Bcol.size() == data.n*data.dim);
		for (int c = 0; c < data.dim; c++)
		{
			Eigen::VectorXd Uc, Bc, bcc, Beq;
			Bc = Bcol.block(c*n, 0, n, 1);
			if (data.with_dynamics)
			{
				Bc += Dl.col(c);
			}
			if (bc.size() > 0)
			{
				bcc = bc.col(c);
			}
			min_quad_with_fixed_solve(
				data.solver_data,
				Bc, bcc, Beq,
				Uc);
			U.col(c) = Uc;
		}

		iter++;
	}
	if (data.with_dynamics)
	{
		// Keep track of velocity for next time
		data.vel = (U - U0) / data.h;
	}

	return true;
}

class ARAPDeform
{
public:
	ARAPDeform() {};
	ARAPDeform(t_Visual* model, double* target) {
		int vCnt = model->vertexCnt;
		int fCnt = model->faceCnt;
		int dim = 3;
		//init V
		V.resize(vCnt, dim);
		for (int i = 0; i< vCnt; i++)
		{
			V(i, 0) = model->vertex[i].x;
			V(i, 1) = model->vertex[i].y;
			V(i, 2) = model->vertex[i].z;
		}
		//init target V
		result = target;
		T.resize(vCnt, dim);
		for (int i = 0; i < vCnt; i++)
		{
			T(i, 0) = target[i * 3 + 0];
			T(i, 1) = target[i * 3 + 1];
			T(i, 2) = target[i * 3 + 2];
		}
		//init F
		F.resize(fCnt, dim);
		for (int i = 0; i< fCnt; i++)
		{
			F(i, 0) = model->index[i].v[0];
			F(i, 1) = model->index[i].v[1];
			F(i, 2) = model->index[i].v[2];
		}
		//init arap_data.max_iter
		arap_data.max_iter = 1;
	}
	~ARAPDeform() {};
public:
	bool applyDeform()
	{
		time_t startT = clock();
		time_t endT;
		//// Centroid
		//mid = 0.5*(V.colwise().maxCoeff() + V.colwise().minCoeff());
		// Precomputation
		arap_data.max_iter = 1;
		std::cout << "arap precomputation......";
		bool ret = arapPrecompute(T, F, V.cols(), b, arap_data);
		assert(ret  && "arap precompute false");
		Eigen::MatrixXd bc;
		std::cout << "arap solving......";
		ret = arapSolve(bc, arap_data, V); 
		endT = clock();
		double runtim = (double)(endT - startT) / CLOCKS_PER_SEC;
		
		CString t;
		t.Format(_T("%f"), runtim);
		AfxMessageBox(t);
		for (int i = 0; i < V.rows(); i++)
		{
			result[i * 3 + 0] = T(i, 0);
			result[i * 3 + 1] = T(i, 1);
			result[i * 3 + 2] = T(i, 2);
		}
		return ret;
	}


public:
	Eigen::MatrixXd V, T;
	double *result;
	Eigen::MatrixXi F;
	Eigen::VectorXi S, b;
	Eigen::RowVector3d mid;
	double anim_t = 0.0;
	double anim_t_dir = 0.03;
	igl::ARAPData arap_data;
};

