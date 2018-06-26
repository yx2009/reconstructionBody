#include "stdafx.h"
#include <math.h>
#include "linalg3d.h"
#include "3d-morph.h"
#include "icpPointMoveToPlane.h"
#include <fstream>
#include <vector>
#include <set>
using namespace std;
struct dist_normal
{
	
	double Distance;
	double nx;
	double ny;
	double nz;

};
struct vertexDeform
{
vertexDeform():D_Nlist() {}
	int index;
	int statu=0;//0未移动；1已移动；2已移动且边缘;3外部移动完成的
	double mnx;
	double mny;
	double mnz;
	vector<dist_normal> D_Nlist;
};
double icpPointMoveToPlane::fitStep(double *T, const int32_t T_num, selfMatrix::Matrix &R, selfMatrix::Matrix &t, const std::vector<int32_t> &active) {

	// kd tree query + result
	std::vector<float>         query(dim);
	kdtree::KDTreeResultVector result;

	// init matrix for point correspondences
	selfMatrix::Matrix p_m(active.size(), dim); // model
	selfMatrix::Matrix p_t(active.size(), dim); // template

									// dimensionality 2
	if (dim == 2) {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1];
		double r10 = R.val[1][0]; double r11 = R.val[1][1];
		double t0 = t.val[0][0]; double t1 = t.val[1][0];

		// init A and b
		selfMatrix::Matrix A(active.size(), 3);
		selfMatrix::Matrix b(active.size(), 1);

		// establish correspondences
		for (int32_t i = 0; i < active.size(); i++) {

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = r00*T[idx * 2 + 0] + r01*T[idx * 2 + 1] + t0;
			query[1] = r10*T[idx * 2 + 0] + r11*T[idx * 2 + 1] + t1;

			// search nearest neighbor
			M_tree->n_nearest(query, 1, result);

			// model point
			double dx = M_tree->the_data[result[0].idx][0];
			double dy = M_tree->the_data[result[0].idx][1];

			// model point normal
			double nx = M_normal[result[0].idx * 2 + 0];
			double ny = M_normal[result[0].idx * 2 + 1];

			// template point
			double sx = query[0];
			double sy = query[1];

			// setup least squares system
			A.val[i][0] = ny*sx - nx*sy;
			A.val[i][1] = nx;
			A.val[i][2] = ny;
			b.val[i][0] = nx*dx + ny*dy - nx*sx - ny*sy;
		}
		// linear least square matrices
		selfMatrix::Matrix A_ = ~A*A;
		selfMatrix::Matrix b_ = ~A*b;

		// solve linear system
		if (b_.solve(A_)) {

			// rotation matrix
			selfMatrix::Matrix R_ = selfMatrix::Matrix::eye(2);
			R_.val[0][1] = -b_.val[0][0];
			R_.val[1][0] = +b_.val[0][0];

			// orthonormalized rotation matrix
			selfMatrix::Matrix U, W, V;
			R_.svd(U, W, V);
			R_ = U*~V;

			// translation vector
			selfMatrix::Matrix t_(2, 1);
			t_.val[0][0] = b_.val[1][0];
			t_.val[1][0] = b_.val[2][0];

			// compose: R|t = R_|t_ * R|t
			R = R_*R;
			t = R_*t + t_;
			return max((R_ - selfMatrix::Matrix::eye(2)).l2norm(), t_.l2norm());
		}

		// dimensionality 3
	}
	else {
		vector<int> index_deformededge;//记录边缘已变形点的索引index
		
		//vector<vertexDeform> Deformed_vertex;
		vertexDeform *deform_vertex=new vertexDeform[T_num];// = (vertexDeform *)malloc(T_num * sizeof(vertexDeform));
		vertexDeform *localrigid_vertex = new vertexDeform[T_num];

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];

		// init A and b
		selfMatrix::Matrix A(active.size(), 6);
		selfMatrix::Matrix b(active.size(), 1);

		// establish correspondences
		for (int32_t i = 0; i < active.size(); i++) {

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = r00*T[idx * 3 + 0] + r01*T[idx * 3 + 1] + r02*T[idx * 3 + 2] + t0;
			query[1] = r10*T[idx * 3 + 0] + r11*T[idx * 3 + 1] + r12*T[idx * 3 + 2] + t1;
			query[2] = r20*T[idx * 3 + 0] + r21*T[idx * 3 + 1] + r22*T[idx * 3 + 2] + t2;

			// search nearest neighbor
			M_tree->n_nearest(query, 1, result);

			// model point
			double dx = M_tree->the_data[result[0].idx][0];
			double dy = M_tree->the_data[result[0].idx][1];
			double dz = M_tree->the_data[result[0].idx][2];

			// model point normal
			double nx = M_normal[result[0].idx * 3 + 0];
			double ny = M_normal[result[0].idx * 3 + 1];
			double nz = M_normal[result[0].idx * 3 + 2];

			//原始目标模型数据的顶点向量
			double rnx = M_normalsource[result[0].idx * 3 + 0];
			double rny = M_normalsource[result[0].idx * 3 + 1];
			double rnz = M_normalsource[result[0].idx * 3 + 2];

			// template point
			double sx = query[0];
			double sy = query[1];
			double sz = query[2];



			double px = dx - sx;
			double py = dy - sy;
			double pz = dz - sz;

			//double squarelenth = sqrt(px*px+py*py+pz*pz);//距离
			//计算连接向量到目标点向量的投影长度
			double projLen = px*nx + py*ny + pz*nz;
			//source点到目标点法平面的移动向量
			double s2mx = nx*projLen;
			double s2my = ny*projLen;
			double s2mz = nz*projLen;
			//尝试沿着source向量的移动
			//m=projLen/t_n.dot(m_n)
			if (S_normal!=nullptr)
			{
				double snx = S_normal[idx * 3 + 0];
				double sny = S_normal[idx * 3 + 1];
				double snz = S_normal[idx * 3 + 2];

				double project_sn2mn= snx*nx + sny*ny + snz*nz;
				double m = projLen / project_sn2mn;
			/*	s2mx = m*snx;
				s2my = m*sny;
				s2mz = m*snz;*/
							
			}


			//double mnx = dx - sx; double mny = dy - sy; double mnz = dz - sz;
			if (mModel->vertex[result[0].idx].status==1)//原始模型中顶点是边缘点
			{
				//边缘点对移动方式不同
				//T[idx * 3 + 0] += s2mx;
				//T[idx * 3 + 1] += s2my;
				//T[idx * 3 + 2] += s2mz;

				index_deformededge.push_back(idx);
				deform_vertex[idx].statu = 2;
				deform_vertex[idx].mnx = s2mx; deform_vertex[idx].mny = s2my; deform_vertex[idx].mnz = s2mz;

				//Deformed_vertex.push_back(verdeform);//如果仅仅是利用边缘变形的点，可以只记录边缘点，
			}
			else {
				//点到点的移动，简单的移动
				//T[idx * 3 + 0] = dx;
				//T[idx * 3 + 1] = dy;
				//T[idx * 3 + 2] = dz;

				//T[idx * 3 + 0] += s2mx;
				//T[idx * 3 + 1] += s2my;
				//T[idx * 3 + 2] += s2mz;
				deform_vertex[idx].statu = 1;
				deform_vertex[idx].mnx = s2mx; deform_vertex[idx].mny = s2my; deform_vertex[idx].mnz = s2mz;
			}

			//Deformed_vertex.push_back(verdeform);//此处记录是为了利用更大范围的已变形点。


			// setup least squares system
			A.val[i][0] = nz*sy - ny*sz;
			A.val[i][1] = nx*sz - nz*sx;
			A.val[i][2] = ny*sx - nx*sy;
			A.val[i][3] = nx;
			A.val[i][4] = ny;
			A.val[i][5] = nz;
			b.val[i][0] = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;
		}
		/*fstream ffff;
		ffff.open("statu.txt", ios::app | ios::out);
		if (ffff)
		{
			for (int i=0;i<T_num;i++)
			{
				ffff << deform_vertex[i].statu << endl;
			}

		}
		ffff.close();*/
		//很多点未对应到目标模型的边缘点，没有被定义为边界点
		//此处判断该点是否为边界点
		for (int i = 0; i < T_num; i++)
		{
			if (deform_vertex[i].statu == 1)
			{
				std::vector<int> neiberIndex;
				

				NeiberVertex(pModel, i, neiberIndex);
				for (int j=0;j<neiberIndex.size();j++)
				{
					if (deform_vertex[neiberIndex[j]].statu ==0)
					{
						index_deformededge.push_back(i);
						deform_vertex[i].statu = 2;
						//T[i * 3 + 0] -= deform_vertex[i].mnx;
						//T[i * 3 + 1] -= deform_vertex[i].mny;
						//T[i * 3 + 2] -= deform_vertex[i].mnz;
						break;
					}
				}

			}
		}

		/*CString strrs;
		strrs.Format(_T("%d"), index_deformededge.size());
		AfxMessageBox(strrs);*/
		if (NONRIGID_TYPE == 0)
		{
			while (index_deformededge.size() > 0)//直到所有的点都移动过结束？直到没有边缘动点？
			{
				//清除边缘未动点的列表

				set<int> index_deformingedge;//记录边缘已变形点的索引index
				//遍历边缘已变形的点，查找相邻的点中是否有未移动的点，计算距离，计算高斯分布的影响因子，
				for (int32_t i = 0; i < index_deformededge.size(); i++)
				{
					std::vector<int> neiberIndex;
					int pindex = index_deformededge[i];

					NeiberVertex(pModel, pindex, neiberIndex);
					//给每个相邻未动点赋值距离和参考向量
					for (vector<int>::const_iterator j = neiberIndex.cbegin(); j != neiberIndex.cend(); j++)
					{
						if (deform_vertex[(*j)].statu <= 0)
						{
							//将未动点的距离L和Dlist赋值
							//计算距离和参考向量
							dist_normal DNtemp;
							DNtemp.Distance = sqrt((pModel->vertex[pindex].x - pModel->vertex[*j].x)*(pModel->vertex[pindex].x - pModel->vertex[*j].x)
								+ (pModel->vertex[pindex].y - pModel->vertex[*j].y)*(pModel->vertex[pindex].y - pModel->vertex[*j].y)
								+ (pModel->vertex[pindex].z - pModel->vertex[*j].z)*(pModel->vertex[pindex].z - pModel->vertex[*j].z));
							DNtemp.nx = deform_vertex[pindex].mnx; DNtemp.ny = deform_vertex[pindex].mny; DNtemp.nz = deform_vertex[pindex].mnz;
							deform_vertex[(*j)].D_Nlist.push_back(DNtemp);
							//deform_vertex[*j].statu = 3;
							index_deformingedge.insert((*j));//考虑到重复读取，使用set
						}

					}
					deform_vertex[pindex].statu = 3;//此处的状态变换最好放到邻域点移动完成才执行

				}
				if (index_deformingedge.size() == 0)
				{
					break;//没有将要移动的点，跳出循环
				}
				//清除边缘动点的列表
				index_deformededge.clear();
				index_deformededge.shrink_to_fit();
				//遍历每个刚赋值的未动点，进行变形
				for (set<int>::iterator iter = index_deformingedge.begin(); iter != index_deformingedge.end(); iter++)
				{
					vector<dist_normal> VDtemp = deform_vertex[*iter].D_Nlist;
					double scale = 0.0;
					double movVectorx = 0.0, movVectory = 0.0, movVectorz = 0.0;//该点的移动向量
					for (int i = 0; i < VDtemp.size(); i++)
					{

						double p = Gauss_distriValue(VDtemp[i].Distance);//高斯概率密度值
						scale += p;
						movVectorx += p*VDtemp[i].nx;
						movVectory += p*VDtemp[i].ny;
						movVectorz += p*VDtemp[i].nz;
					}

					movVectorx = movVectorx / scale;
					movVectory = movVectory / scale;
					movVectorz = movVectorz / scale;

					deform_vertex[*iter].mnx = movVectorx;
					deform_vertex[*iter].mny = movVectory;
					deform_vertex[*iter].mnz = movVectorz;

					//移动模型中的点
					//T[*iter * 3 + 0] += movVectorx;
					//T[*iter * 3 + 1] += movVectory;
					//T[*iter * 3 + 2] += movVectorz;

					//将该点变为statu=2边缘动点
					deform_vertex[*iter].statu = 3;
					index_deformededge.push_back(*iter);

				}

			}
		


			/************************************************************************/
			/* start：测试local rigid    20180219                                                                  */
			// 对每个顶点进行local rigid变形
			/************************************************************************/

			for (int i = 0; i < 0/*T_num*/; i++)
			{
				if (deform_vertex[i].statu == 3)
				{
					std::vector<int> neiberIndex;
					NeiberVertex(pModel, i, neiberIndex);
					Eigen::Matrix3d Er;//旋转矩阵R
					Eigen::Vector3d	Et;//平移矩阵T
					int metchNum = neiberIndex.size() + 1;
					metchPoints *dmetchPoints = new metchPoints[metchNum];;//点云匹配点对
					int pt = 1;
					dmetchPoints[0].p1.x = T[i * 3 + 0] + deform_vertex[i].mnx;
					dmetchPoints[0].p1.y = T[i * 3 + 1] + deform_vertex[i].mny;
					dmetchPoints[0].p1.z = T[i * 3 + 2] + deform_vertex[i].mnz;

					dmetchPoints[0].p2.x = T[i * 3 + 0];
					dmetchPoints[0].p2.y = T[i * 3 + 1];
					dmetchPoints[0].p2.z = T[i * 3 + 2];
					for (vector<int>::const_iterator j = neiberIndex.cbegin(); j != neiberIndex.cend(); j++, pt++)
					{
						dmetchPoints[pt].p1.x = T[(*j) * 3 + 0] + deform_vertex[*j].mnx;
						dmetchPoints[pt].p1.y = T[(*j) * 3 + 1] + deform_vertex[*j].mny;
						dmetchPoints[pt].p1.z = T[(*j) * 3 + 2] + deform_vertex[*j].mnz;

						dmetchPoints[pt].p2.x = T[(*j) * 3 + 0];
						dmetchPoints[pt].p2.y = T[(*j) * 3 + 1];
						dmetchPoints[pt].p2.z = T[(*j) * 3 + 2];
					}
					if (RotationMatrix(dmetchPoints, metchNum, Er, Et))
					{
						localrigid_vertex[i].mnx = Et[0];
						localrigid_vertex[i].mny = Et[1];
						localrigid_vertex[i].mnz = Et[2];
					}
					else
					{
						localrigid_vertex[i].mnx = deform_vertex[i].mnx;
						localrigid_vertex[i].mny = deform_vertex[i].mny;
						localrigid_vertex[i].mnz = deform_vertex[i].mnz;
					}

					delete[]dmetchPoints;
					//移动模型中的点
					//T[i * 3 + 0] += deform_vertex[i].mnx;
					//T[i * 3 + 1] += deform_vertex[i].mny;
					//T[i * 3 + 2] += deform_vertex[i].mnz;

				}
			}

			/************************************************************************/
			/* end:   测试local rigid                                                                */
			/************************************************************************/
			//遍历所有已经动的外部点statu=3,进行平滑，求取邻域点move向量的均值yingxiang
			for (int l = 0; l < 10; l++)
			{
				for (int i = 0; i < T_num; i++)
				{
					if (1)//deform_vertex[i].statu == 3)
					{
						std::vector<int> neiberIndex;
						double tempx = 0, tempy = 0, tempz = 0;
						NeiberVertex(pModel, i, neiberIndex);
						for (vector<int>::const_iterator j = neiberIndex.cbegin(); j != neiberIndex.cend(); j++)
						{
							tempx += deform_vertex[*j].mnx;
							tempy += deform_vertex[*j].mny;
							tempz += deform_vertex[*j].mnz;
						}
						deform_vertex[i].mnx = tempx / neiberIndex.size();
						deform_vertex[i].mny = tempy / neiberIndex.size();
						deform_vertex[i].mnz = tempz / neiberIndex.size();

						//移动模型中的点
						//T[i * 3 + 0] += deform_vertex[i].mnx;
						//T[i * 3 + 1] += deform_vertex[i].mny;
						//T[i * 3 + 2] += deform_vertex[i].mnz;

					}
				}

			}

		}else if (NONRIGID_TYPE==1)
		{
			//tps
			std::vector<TPSMatrix::Coord_Diff> *p = new std::vector<TPSMatrix::Coord_Diff>;

			std::auto_ptr< std::vector<TPSMatrix::Coord_Diff> > samples(p);
			if (index_deformededge.size()<=0)
			{
				AfxMessageBox("TPS非刚性变形缺少控制点！");
			}
			for (int i = 0; i < index_deformededge.size(); ++i)
			{
				
						TPSMatrix::Coord_Diff sample;
						int indexP= index_deformededge[i];
						sample.x = T[indexP * 3 + 0];// +deform_vertex[indexP].mnx;
						sample.y = T[indexP * 3 + 1];// +deform_vertex[indexP].mny;
						sample.z = T[indexP * 3 + 2];// +deform_vertex[indexP].mnz;
						sample.dx = deform_vertex[indexP].mnx;
						sample.dy = deform_vertex[indexP].mny;
						sample.dz = deform_vertex[indexP].mnz;
						
						samples->push_back(sample);
			}

			

			TPSMatrix::TPS_Morpher morpher(samples, 1.0);
			std::vector<TPSMatrix::Point> pts;
			for (int i = 0; i < T_num; i++)
			{
				if (deform_vertex[i].statu == 0|| deform_vertex[i].statu==2)
				{
					TPSMatrix::Point pt;
					pt.x = T[i * 3 + 0]; 
					pt.y = T[i * 3 + 1];
					pt.z = T[i * 3 + 2];
					pts.push_back(pt);
					deform_vertex[i].statu == 3;
				}

			}
			morpher.morph(pts);
			//printf("%f %f %f\n", pts[0].x, pts[0].y, pts[0].z);
			int ptsindex = 0;
			for (int i = 0; i < T_num; i++)
			{
				if (deform_vertex[i].statu == 0 || deform_vertex[i].statu == 2)
				{
					deform_vertex[i].mnx = pts[ptsindex].x - T[i * 3 + 0];
					deform_vertex[i].mny = pts[ptsindex].y - T[i * 3 + 1];
					deform_vertex[i].mnz = pts[ptsindex].z - T[i * 3 + 2];
					//T[i * 3 + 0] = pts[ptsindex].x;
					//T[i * 3 + 1] = pts[ptsindex].y;
					//T[i * 3 + 2] = pts[ptsindex].z;
					ptsindex++;
				}

			}
		}
		//error
		//每条边长度变化之和/2作为误差
		double error_all = 0.0;
		int edge_num = 0;
		for (int i = 0; i < T_num; i++)
		{
			
			std::vector<int> neiberIndex;
			NeiberVertex(pModel, i, neiberIndex);
			for (int j = 0; j < neiberIndex.size(); j++)
			{
				double xx = 0.0, yy = 0.0, zz = 0.0;
				double orixx = 0.0, oriyy = 0.0, orizz = 0.0;

				xx = (deform_vertex[i].mnx - deform_vertex[neiberIndex[j]].mnx);
				yy = (deform_vertex[i].mny - deform_vertex[neiberIndex[j]].mny);
				zz = (deform_vertex[i].mnz - deform_vertex[neiberIndex[j]].mnz);

				
				error_all += sqrt(xx*xx + yy*yy + zz*zz);
			}
			edge_num += neiberIndex.size();

		}
		error_all = error_all/(2.0*edge_num);
		CString strrs;
		strrs.Format(_T("%f"), error_all);
		AfxMessageBox(strrs);
		
		//yingxiang 此处可以使用曲率比较进行平滑。
		for (int i = 0; i < T_num; i++)
		{
			if (1)//deform_vertex[i].statu == 3)
			{

				//移动模型中的点
				T[i * 3 + 0] += deform_vertex[i].mnx;
				T[i * 3 + 1] += deform_vertex[i].mny;
				T[i * 3 + 2] += deform_vertex[i].mnz;

			}
		}
		delete []deform_vertex;
		delete []localrigid_vertex;
		// linear least square matrices
		selfMatrix::Matrix A_ = ~A*A;
		selfMatrix::Matrix b_ = ~A*b;

		// solve linear system
		if (b_.solve(A_)) {

			// rotation matrix
			selfMatrix::Matrix R_ = selfMatrix::Matrix::eye(3);
			R_.val[0][1] = -b_.val[2][0];
			R_.val[1][0] = +b_.val[2][0];
			R_.val[0][2] = +b_.val[1][0];
			R_.val[2][0] = -b_.val[1][0];
			R_.val[1][2] = -b_.val[0][0];
			R_.val[2][1] = +b_.val[0][0];

			// orthonormalized rotation matrix
			selfMatrix::Matrix U, W, V;
			R_.svd(U, W, V);
			R_ = U*~V;

			// translation vector
			selfMatrix::Matrix t_(3, 1);
			t_.val[0][0] = b_.val[3][0];
			t_.val[1][0] = b_.val[4][0];
			t_.val[2][0] = b_.val[5][0];

			// compose: R|t = R_|t_ * R|t
			R = R_*R;
			t = R_*t + t_;
			return max((R_ - selfMatrix::Matrix::eye(3)).l2norm(), t_.l2norm());
		}
	}

	// failure
	return 0;
}
double icpPointMoveToPlane::Gauss_distriValue(const double &distan)
{//暂时标准正态分布，需要根据点云的密度确定合适的标准差
	double Sigma = 1.0;//标准差
	double mu = 0.0;//期望值
	return exp(-distan*distan/(2.0* Sigma*Sigma))/(sqrt(2.0*PI)*Sigma);
}
BOOL icpPointMoveToPlane::NeiberVertex(t_Visual* originalVisual, int pindex, std::vector<int> &neiberIndex) 
{
	std::set<int> neiberTemp;
	long tempfindex;
	int temp=0;
	//循环遍历查找与顶点的面索引
	for (int j = 0; j<originalVisual->vfindex[pindex].num; j++)
	{
		tempfindex = originalVisual->vfindex[pindex].findex[j];

		if (originalVisual->index[tempfindex].v[0] == pindex)
		{
			if (neiberTemp.insert(originalVisual->index[tempfindex].v[1]).second)
			{
				neiberIndex.push_back(originalVisual->index[tempfindex].v[1]);
			}
			if (neiberTemp.insert(originalVisual->index[tempfindex].v[2]).second)
			{
				neiberIndex.push_back(originalVisual->index[tempfindex].v[2]);
			}
			temp++;

		}
		else if (originalVisual->index[tempfindex].v[1] == pindex)
		{

			if (neiberTemp.insert(originalVisual->index[tempfindex].v[0]).second)
			{
				neiberIndex.push_back(originalVisual->index[tempfindex].v[0]);
			}
			if (neiberTemp.insert(originalVisual->index[tempfindex].v[2]).second)
			{
				neiberIndex.push_back(originalVisual->index[tempfindex].v[2]);
			}
			temp++;
		}
		else if (originalVisual->index[tempfindex].v[2] == pindex)
		{
			if (neiberTemp.insert(originalVisual->index[tempfindex].v[0]).second)
			{
				neiberIndex.push_back(originalVisual->index[tempfindex].v[0]);
			}
			if (neiberTemp.insert(originalVisual->index[tempfindex].v[1]).second)
			{
				neiberIndex.push_back(originalVisual->index[tempfindex].v[1]);
			}
			temp++;
		}

	}
	if (temp > 0) return TRUE;
	return FALSE;

}
std::vector<int32_t> icpPointMoveToPlane::getInliers_nonrigid(double *T, double *Tn, const int32_t T_num, const selfMatrix::Matrix &R, const selfMatrix::Matrix &t, const double indist) {

	// init inlier vector + query point + query result
	vector<int32_t>            inliers;
	std::vector<float>         query(dim);
	kdtree::KDTreeResultVector neighbor;


	//fstream ff;
	//ff.open("E:\\mtree.txt", ios::out | ios::app);
	//if (ff)
	//{
	//	
	//	for (int32_t i = 0; i < M_tree->N; i++) {
	//		double dx = M_tree->the_data[i][0];
	//		double dy = M_tree->the_data[i][1];
	//		double dz = M_tree->the_data[i][2];
	//		ff << dx << " \t" << dy << " \t" << dz << endl;
	//	}
	//}
	//ff.close();
	// dimensionality 2
	if (dim == 2) {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1];
		double r10 = R.val[1][0]; double r11 = R.val[1][1];
		double t0 = t.val[0][0]; double t1 = t.val[1][0];

		// check for all points if they are inliers
		for (int32_t i = 0; i < T_num; i++) {

			// transform point according to R|t
			double sx = r00*T[i * 2 + 0] + r01*T[i * 2 + 1]; query[0] = sx;
			double sy = r10*T[i * 2 + 0] + r11*T[i * 2 + 1]; query[1] = sy;

			// search nearest neighbor
			M_tree->n_nearest(query, 1, neighbor);

			// model point
			double dx = M_tree->the_data[neighbor[0].idx][0];
			double dy = M_tree->the_data[neighbor[0].idx][1];

			// model point normal
			double nx = M_normal[neighbor[0].idx * 2 + 0];
			double ny = M_normal[neighbor[0].idx * 2 + 1];

			// check if it is an inlier
			if ((sx - dx)*nx + (sy - dy)*ny < indist)
				inliers.push_back(i);
		}

		// dimensionality 3
	}
	else {

		S_normal = Tn;
		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];


		double tempr = 2.0;//点云的平均距离暂设*************************************yingxiang20180408
		double theta = acos(0.7);//最大夹角（0<=theta<=3.1415926）
		double mtemp = 2;// powf(1.0 / theta, 1.0 / tempr);
		// check for all points if they are inliers
		for (int32_t i = 0; i < T_num; i++) {

			// transform point according to R|t
			double sx = r00*T[i * 3 + 0] + r01*T[i * 3 + 1] + r02*T[i * 3 + 2] + t0; query[0] = sx;
			double sy = r10*T[i * 3 + 0] + r11*T[i * 3 + 1] + r12*T[i * 3 + 2] + t1; query[1] = sy;
			double sz = r20*T[i * 3 + 0] + r21*T[i * 3 + 1] + r22*T[i * 3 + 2] + t2; query[2] = sz;
			//
			double snx = Tn[i * 3 + 0];
			double sny = Tn[i * 3 + 1];
			double snz = Tn[i * 3 + 2];

			// search nearest neighbor
			M_tree->n_nearest(query, 1, neighbor);

			// model point
			double dx = M_tree->the_data[neighbor[0].idx][0];
			double dy = M_tree->the_data[neighbor[0].idx][1];
			double dz = M_tree->the_data[neighbor[0].idx][2];

			// model point normal
			double nx = M_normal[neighbor[0].idx * 3 + 0];
			double ny = M_normal[neighbor[0].idx * 3 + 1];
			double nz = M_normal[neighbor[0].idx * 3 + 2];

			// check if it is an inlier 此处判断条件，和法向量的角度过大，不能算。距离元也不行。
			double squarelenth = sqrt((sx - dx)*(sx - dx) + (sy - dy)*(sy - dy) + (sz - dz)*(sz - dz));//距离
			double costheta;
			if (snx==0&&sny==0&&snz==0)
			{
				costheta = 0;
			}else
				costheta = ((dx - sx)*snx + (dy - sy)*sny + (dz - sz)*snz) / (squarelenth*sqrt((snx*snx + sny*sny + snz*snz)));
			if (squarelenth < 20)
			{
				if (squarelenth<=tempr)//匹配点间的距离小于半径
				{
					inliers.push_back(i);
				}else if (fabs(costheta)>0.7)//yingxiang 20180528 costheta 改为 fabs(costheta)
				{
					inliers.push_back(i);
				}
				else if (acos(fabs(costheta))<(PI/2.0)*powf(mtemp,-0.5*(squarelenth-tempr)))//角度和距离的关系，正常的关系：acos(costheta)<=pow(mtemp,-squarelenth)
					inliers.push_back(i);

			}

		}
	}

	// return vector with inliers
	return inliers;
}
std::vector<int32_t> icpPointMoveToPlane::getInliers(double *T,  const int32_t T_num, const selfMatrix::Matrix &R, const selfMatrix::Matrix &t, const double indist) {

	// init inlier vector + query point + query result
	vector<int32_t>            inliers;
	std::vector<float>         query(dim);
	kdtree::KDTreeResultVector neighbor;

	// dimensionality 2
	if (dim == 2) {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1];
		double r10 = R.val[1][0]; double r11 = R.val[1][1];
		double t0 = t.val[0][0]; double t1 = t.val[1][0];

		// check for all points if they are inliers
		for (int32_t i = 0; i < T_num; i++) {

			// transform point according to R|t
			double sx = r00*T[i * 2 + 0] + r01*T[i * 2 + 1]; query[0] = sx;
			double sy = r10*T[i * 2 + 0] + r11*T[i * 2 + 1]; query[1] = sy;

			// search nearest neighbor
			M_tree->n_nearest(query, 1, neighbor);

			// model point
			double dx = M_tree->the_data[neighbor[0].idx][0];
			double dy = M_tree->the_data[neighbor[0].idx][1];

			// model point normal
			double nx = M_normal[neighbor[0].idx * 2 + 0];
			double ny = M_normal[neighbor[0].idx * 2 + 1];

			// check if it is an inlier
			if ((sx - dx)*nx + (sy - dy)*ny < indist)
				inliers.push_back(i);
		}

		// dimensionality 3
	}
	else {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];

		// check for all points if they are inliers
		for (int32_t i = 0; i < T_num; i++) {

			// transform point according to R|t
			double sx = r00*T[i * 3 + 0] + r01*T[i * 3 + 1] + r02*T[i * 3 + 2] + t0; query[0] = sx;
			double sy = r10*T[i * 3 + 0] + r11*T[i * 3 + 1] + r12*T[i * 3 + 2] + t1; query[1] = sy;
			double sz = r20*T[i * 3 + 0] + r21*T[i * 3 + 1] + r22*T[i * 3 + 2] + t2; query[2] = sz;

			// search nearest neighbor
			M_tree->n_nearest(query, 1, neighbor);

			// model point
			double dx = M_tree->the_data[neighbor[0].idx][0];
			double dy = M_tree->the_data[neighbor[0].idx][1];
			double dz = M_tree->the_data[neighbor[0].idx][2];

			// model point normal
			double nx = M_normal[neighbor[0].idx * 3 + 0];
			double ny = M_normal[neighbor[0].idx * 3 + 1];
			double nz = M_normal[neighbor[0].idx * 3 + 2];

			// check if it is an inlier 此处判断条件，和法向量的角度过大，不能算。距离远也不行。
			double squarelenth = sqrt((sx - dx)*(sx - dx) + (sy - dy)*(sy - dy) + (sz - dz)*(sz - dz));//距离

			if (squarelenth < indist)
			{
				if (((dx - sx)*nx + (dy - sy)*ny + (dz - sz)*nz) / (squarelenth*sqrt((nx*nx + ny*ny + nz*nz))) > 0)//角度
					inliers.push_back(i);

			}

		}
	}

	// return vector with inliers
	return inliers;
}

void icpPointMoveToPlane::computeNormal(const kdtree::KDTreeResultVector &neighbors, double *M_normal, const double flatness) {

	// dimensionality 2
	if (dim == 2) {

		// extract neighbors
		selfMatrix::Matrix P(neighbors.size(), 2);
		selfMatrix::Matrix mu(1, 2);
		for (int32_t i = 0; i < neighbors.size(); i++) {
			double x = M_tree->the_data[neighbors[i].idx][0];
			double y = M_tree->the_data[neighbors[i].idx][1];
			P.val[i][0] = x;
			P.val[i][1] = y;
			mu.val[0][0] += x;
			mu.val[0][1] += y;
		}

		// zero mean
		mu = mu / (double)neighbors.size();
		selfMatrix::Matrix Q = P - selfMatrix::Matrix::ones(neighbors.size(), 1)*mu;

		// principal component analysis
		selfMatrix::Matrix H = ~Q*Q;
		selfMatrix::Matrix U, W, V;
		H.svd(U, W, V);

		// normal
		M_normal[0] = U.val[0][1];
		M_normal[1] = U.val[1][1];

		// dimensionality 3
	}
	else {

		// extract neighbors
		selfMatrix::Matrix P(neighbors.size(), 3);
		selfMatrix::Matrix mu(1, 3);
		for (int32_t i = 0; i < neighbors.size(); i++) {
			double x = M_tree->the_data[neighbors[i].idx][0];
			double y = M_tree->the_data[neighbors[i].idx][1];
			double z = M_tree->the_data[neighbors[i].idx][2];
			P.val[i][0] = x;
			P.val[i][1] = y;
			P.val[i][2] = z;
			mu.val[0][0] += x;
			mu.val[0][1] += y;
			mu.val[0][2] += z;
		}

		// zero mean
		mu = mu / (double)neighbors.size();
		selfMatrix::Matrix Q = P - selfMatrix::Matrix::ones(neighbors.size(), 1)*mu;

		// principal component analysis
		selfMatrix::Matrix H = ~Q*Q;
		selfMatrix::Matrix U, W, V;
		H.svd(U, W, V);

		// normal
		M_normal[0] = U.val[0][2];
		M_normal[1] = U.val[1][2];
		M_normal[2] = U.val[2][2];
	}
}
double* icpPointMoveToPlane::computeNormals(const int32_t num_neighbors, const double flatness) {
	double *M_normal = (double*)malloc(M_tree->N*dim * sizeof(double));
	kdtree::KDTreeResultVector neighbors;
	for (int32_t i = 0; i < M_tree->N; i++) {
		M_tree->n_nearest_around_point(i, 0, num_neighbors, neighbors);
		if (dim == 2) computeNormal(neighbors, M_normal + i * 2, flatness);
		else        computeNormal(neighbors, M_normal + i * 3, flatness);
	}
	return M_normal;
}

/* 功能：RotationMatrix计算转换矩阵，利用四元数
参数：dmetchPoints：匹配点对；
*/
/************************************************************************/
BOOL   icpPointMoveToPlane::RotationMatrix(metchPoints* dmetchPoints, int machePnum, Eigen::Matrix3d &R, Eigen::Vector3d &T)
{

	/************************************************************************/
	/* 思路，分别计算匹配点对的质心cp1，cp2，
	分别去中心化，也就是第i个点减去质心得到ci1,ci2,ci1-ci2=(x,y,z),ci1+ci2=(x1,y1,z1)得到矩阵Ai,Ait,  i=(1,2,3……)
	0	 -x	 -y	 -z						0	 x	 y	 z
	Ai=		x	 0	-z1	 y1       Ai的转置Ait=	-x	 0	 z1	-y1
	y	 z1	 0	-x1						-y	-z1	 0	 x1
	z	-y1	 x1	 0						-z	 y1	-x1	 0
	矩阵A=A1t*A1+A2t*A2+A3t*A3+A4t*A4+……+Ait*Ai
	定义一个单位四元数q
	*/
	/************************************************************************/
	Eigen::Matrix4d Ai, Ait, A;
	//A初始化
	A << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;
	//Eigen::Matrix3d R;//旋转向量
	Eigen::Quaterniond Q;
	Eigen::Vector3d P1, P2;//平移向量p1p2为质心
	tVector p1, p2, c1, c2, c, cc;//p1,p2分别是两个模型的质心，c1,c2分别表示去中心化后的点，c=c1-c2,cc=c1+c2
	int loop;
	int num = 0;
	tVector sum1, sum2;
	sum1.x = sum1.y = sum1.z = 0;
	sum2.x = sum2.y = sum2.z = 0;
	//fstream fs1;
	//fs1.open("旋转平移矩阵.txt", ios::app | ios::out);
	//计算质心
	for (loop = 0; loop<machePnum; loop++)
	{
			sum1 += dmetchPoints[loop].p1;
			sum2 += dmetchPoints[loop].p2;
			num++;
	}
	if (num>3)//暂时要求至少4个点
	{
		p1.x = sum1.x / num; p1.y = sum1.y / num; p1.z = sum1.z / num;
		p2.x = sum2.x / num; p2.y = sum2.y / num; p2.z = sum2.z / num;
		P1(0) = p1.x; P1(1) = p1.y; P1(2) = p1.z;
		P2(0) = p2.x; P2(1) = p2.y; P2(2) = p2.z;
	}
	else {

		//AfxMessageBox("没有匹配的点对，无法计算质心");
		return 0;
	}
	//求A
	for (loop = 0; loop<machePnum; loop++)
	{
		
		//去中心化
		c1 = dmetchPoints[loop].p1 - p1;
		c2 = dmetchPoints[loop].p2 - p2;
		c = c1 - c2;
		cc = c1 + c2;
		Ai << 0, -(c.x*0.5), -c.y*0.5, -c.z*0.5,
			c.x*0.5, 0, -cc.z, cc.y,
			c.y*0.5, cc.z, 0, -cc.x,
			c.z*0.5, -cc.y, cc.x, 0;
		//转置
		Ait = Ai.transpose();
		A += (Ait* Ai);

	}
	//求A的最小特征值，及其对应的特征向量
	Eigen::EigenSolver<Eigen::Matrix4d> es(A);
	Eigen::MatrixXcd eVectors = es.eigenvectors();
	Eigen::MatrixXcd eValues = es.eigenvalues();
	Eigen::MatrixXd eValuesReal;
	eValuesReal = eValues.real();
	Eigen::MatrixXf::Index eValuesMin;
	eValuesReal.rowwise().sum().minCoeff(&eValuesMin);
	Eigen::Vector4d q;

	q << eVectors.real()(0, eValuesMin), eVectors.real()(1, eValuesMin),
		eVectors.real()(2, eValuesMin), eVectors.real()(3, eValuesMin);
	q = q.normalized();
	//旋转矩阵
	Q.x() = q(1); Q.y() = q(2); Q.z() = q(3); Q.w() = q(0);
	R = Q.matrix();
	T = P1 - R*P2;
	
	//RTmetchPoints(dmetchPoints, machePnum, R, T);
	return 1;
}
