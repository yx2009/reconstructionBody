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
	int statu=0;//0δ�ƶ���1���ƶ���2���ƶ��ұ�Ե;3�ⲿ�ƶ���ɵ�
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
		vector<int> index_deformededge;//��¼��Ե�ѱ��ε������index
		
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

			//ԭʼĿ��ģ�����ݵĶ�������
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

			//double squarelenth = sqrt(px*px+py*py+pz*pz);//����
			//��������������Ŀ���������ͶӰ����
			double projLen = px*nx + py*ny + pz*nz;
			//source�㵽Ŀ��㷨ƽ����ƶ�����
			double s2mx = nx*projLen;
			double s2my = ny*projLen;
			double s2mz = nz*projLen;
			//��������source�������ƶ�
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
			if (mModel->vertex[result[0].idx].status==1)//ԭʼģ���ж����Ǳ�Ե��
			{
				//��Ե����ƶ���ʽ��ͬ
				//T[idx * 3 + 0] += s2mx;
				//T[idx * 3 + 1] += s2my;
				//T[idx * 3 + 2] += s2mz;

				index_deformededge.push_back(idx);
				deform_vertex[idx].statu = 2;
				deform_vertex[idx].mnx = s2mx; deform_vertex[idx].mny = s2my; deform_vertex[idx].mnz = s2mz;

				//Deformed_vertex.push_back(verdeform);//������������ñ�Ե���εĵ㣬����ֻ��¼��Ե�㣬
			}
			else {
				//�㵽����ƶ����򵥵��ƶ�
				//T[idx * 3 + 0] = dx;
				//T[idx * 3 + 1] = dy;
				//T[idx * 3 + 2] = dz;

				//T[idx * 3 + 0] += s2mx;
				//T[idx * 3 + 1] += s2my;
				//T[idx * 3 + 2] += s2mz;
				deform_vertex[idx].statu = 1;
				deform_vertex[idx].mnx = s2mx; deform_vertex[idx].mny = s2my; deform_vertex[idx].mnz = s2mz;
			}

			//Deformed_vertex.push_back(verdeform);//�˴���¼��Ϊ�����ø���Χ���ѱ��ε㡣


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
		//�ܶ��δ��Ӧ��Ŀ��ģ�͵ı�Ե�㣬û�б�����Ϊ�߽��
		//�˴��жϸõ��Ƿ�Ϊ�߽��
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
			while (index_deformededge.size() > 0)//ֱ�����еĵ㶼�ƶ���������ֱ��û�б�Ե���㣿
			{
				//�����Եδ������б�

				set<int> index_deformingedge;//��¼��Ե�ѱ��ε������index
				//������Ե�ѱ��εĵ㣬�������ڵĵ����Ƿ���δ�ƶ��ĵ㣬������룬�����˹�ֲ���Ӱ�����ӣ�
				for (int32_t i = 0; i < index_deformededge.size(); i++)
				{
					std::vector<int> neiberIndex;
					int pindex = index_deformededge[i];

					NeiberVertex(pModel, pindex, neiberIndex);
					//��ÿ������δ���㸳ֵ����Ͳο�����
					for (vector<int>::const_iterator j = neiberIndex.cbegin(); j != neiberIndex.cend(); j++)
					{
						if (deform_vertex[(*j)].statu <= 0)
						{
							//��δ����ľ���L��Dlist��ֵ
							//�������Ͳο�����
							dist_normal DNtemp;
							DNtemp.Distance = sqrt((pModel->vertex[pindex].x - pModel->vertex[*j].x)*(pModel->vertex[pindex].x - pModel->vertex[*j].x)
								+ (pModel->vertex[pindex].y - pModel->vertex[*j].y)*(pModel->vertex[pindex].y - pModel->vertex[*j].y)
								+ (pModel->vertex[pindex].z - pModel->vertex[*j].z)*(pModel->vertex[pindex].z - pModel->vertex[*j].z));
							DNtemp.nx = deform_vertex[pindex].mnx; DNtemp.ny = deform_vertex[pindex].mny; DNtemp.nz = deform_vertex[pindex].mnz;
							deform_vertex[(*j)].D_Nlist.push_back(DNtemp);
							//deform_vertex[*j].statu = 3;
							index_deformingedge.insert((*j));//���ǵ��ظ���ȡ��ʹ��set
						}

					}
					deform_vertex[pindex].statu = 3;//�˴���״̬�任��÷ŵ�������ƶ���ɲ�ִ��

				}
				if (index_deformingedge.size() == 0)
				{
					break;//û�н�Ҫ�ƶ��ĵ㣬����ѭ��
				}
				//�����Ե������б�
				index_deformededge.clear();
				index_deformededge.shrink_to_fit();
				//����ÿ���ո�ֵ��δ���㣬���б���
				for (set<int>::iterator iter = index_deformingedge.begin(); iter != index_deformingedge.end(); iter++)
				{
					vector<dist_normal> VDtemp = deform_vertex[*iter].D_Nlist;
					double scale = 0.0;
					double movVectorx = 0.0, movVectory = 0.0, movVectorz = 0.0;//�õ���ƶ�����
					for (int i = 0; i < VDtemp.size(); i++)
					{

						double p = Gauss_distriValue(VDtemp[i].Distance);//��˹�����ܶ�ֵ
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

					//�ƶ�ģ���еĵ�
					//T[*iter * 3 + 0] += movVectorx;
					//T[*iter * 3 + 1] += movVectory;
					//T[*iter * 3 + 2] += movVectorz;

					//���õ��Ϊstatu=2��Ե����
					deform_vertex[*iter].statu = 3;
					index_deformededge.push_back(*iter);

				}

			}
		


			/************************************************************************/
			/* start������local rigid    20180219                                                                  */
			// ��ÿ���������local rigid����
			/************************************************************************/

			for (int i = 0; i < 0/*T_num*/; i++)
			{
				if (deform_vertex[i].statu == 3)
				{
					std::vector<int> neiberIndex;
					NeiberVertex(pModel, i, neiberIndex);
					Eigen::Matrix3d Er;//��ת����R
					Eigen::Vector3d	Et;//ƽ�ƾ���T
					int metchNum = neiberIndex.size() + 1;
					metchPoints *dmetchPoints = new metchPoints[metchNum];;//����ƥ����
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
					//�ƶ�ģ���еĵ�
					//T[i * 3 + 0] += deform_vertex[i].mnx;
					//T[i * 3 + 1] += deform_vertex[i].mny;
					//T[i * 3 + 2] += deform_vertex[i].mnz;

				}
			}

			/************************************************************************/
			/* end:   ����local rigid                                                                */
			/************************************************************************/
			//���������Ѿ������ⲿ��statu=3,����ƽ������ȡ�����move�����ľ�ֵyingxiang
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

						//�ƶ�ģ���еĵ�
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
				AfxMessageBox("TPS�Ǹ��Ա���ȱ�ٿ��Ƶ㣡");
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
		//ÿ���߳��ȱ仯֮��/2��Ϊ���
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
		
		//yingxiang �˴�����ʹ�����ʱȽϽ���ƽ����
		for (int i = 0; i < T_num; i++)
		{
			if (1)//deform_vertex[i].statu == 3)
			{

				//�ƶ�ģ���еĵ�
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
{//��ʱ��׼��̬�ֲ�����Ҫ���ݵ��Ƶ��ܶ�ȷ�����ʵı�׼��
	double Sigma = 1.0;//��׼��
	double mu = 0.0;//����ֵ
	return exp(-distan*distan/(2.0* Sigma*Sigma))/(sqrt(2.0*PI)*Sigma);
}
BOOL icpPointMoveToPlane::NeiberVertex(t_Visual* originalVisual, int pindex, std::vector<int> &neiberIndex) 
{
	std::set<int> neiberTemp;
	long tempfindex;
	int temp=0;
	//ѭ�����������붥���������
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


		double tempr = 2.0;//���Ƶ�ƽ����������*************************************yingxiang20180408
		double theta = acos(0.7);//���нǣ�0<=theta<=3.1415926��
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

			// check if it is an inlier �˴��ж��������ͷ������ĽǶȹ��󣬲����㡣����ԪҲ���С�
			double squarelenth = sqrt((sx - dx)*(sx - dx) + (sy - dy)*(sy - dy) + (sz - dz)*(sz - dz));//����
			double costheta;
			if (snx==0&&sny==0&&snz==0)
			{
				costheta = 0;
			}else
				costheta = ((dx - sx)*snx + (dy - sy)*sny + (dz - sz)*snz) / (squarelenth*sqrt((snx*snx + sny*sny + snz*snz)));
			if (squarelenth < 20)
			{
				if (squarelenth<=tempr)//ƥ����ľ���С�ڰ뾶
				{
					inliers.push_back(i);
				}else if (fabs(costheta)>0.7)//yingxiang 20180528 costheta ��Ϊ fabs(costheta)
				{
					inliers.push_back(i);
				}
				else if (acos(fabs(costheta))<(PI/2.0)*powf(mtemp,-0.5*(squarelenth-tempr)))//�ǶȺ;���Ĺ�ϵ�������Ĺ�ϵ��acos(costheta)<=pow(mtemp,-squarelenth)
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

			// check if it is an inlier �˴��ж��������ͷ������ĽǶȹ��󣬲����㡣����ԶҲ���С�
			double squarelenth = sqrt((sx - dx)*(sx - dx) + (sy - dy)*(sy - dy) + (sz - dz)*(sz - dz));//����

			if (squarelenth < indist)
			{
				if (((dx - sx)*nx + (dy - sy)*ny + (dz - sz)*nz) / (squarelenth*sqrt((nx*nx + ny*ny + nz*nz))) > 0)//�Ƕ�
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

/* ���ܣ�RotationMatrix����ת������������Ԫ��
������dmetchPoints��ƥ���ԣ�
*/
/************************************************************************/
BOOL   icpPointMoveToPlane::RotationMatrix(metchPoints* dmetchPoints, int machePnum, Eigen::Matrix3d &R, Eigen::Vector3d &T)
{

	/************************************************************************/
	/* ˼·���ֱ����ƥ���Ե�����cp1��cp2��
	�ֱ�ȥ���Ļ���Ҳ���ǵ�i�����ȥ���ĵõ�ci1,ci2,ci1-ci2=(x,y,z),ci1+ci2=(x1,y1,z1)�õ�����Ai,Ait,  i=(1,2,3����)
	0	 -x	 -y	 -z						0	 x	 y	 z
	Ai=		x	 0	-z1	 y1       Ai��ת��Ait=	-x	 0	 z1	-y1
	y	 z1	 0	-x1						-y	-z1	 0	 x1
	z	-y1	 x1	 0						-z	 y1	-x1	 0
	����A=A1t*A1+A2t*A2+A3t*A3+A4t*A4+����+Ait*Ai
	����һ����λ��Ԫ��q
	*/
	/************************************************************************/
	Eigen::Matrix4d Ai, Ait, A;
	//A��ʼ��
	A << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;
	//Eigen::Matrix3d R;//��ת����
	Eigen::Quaterniond Q;
	Eigen::Vector3d P1, P2;//ƽ������p1p2Ϊ����
	tVector p1, p2, c1, c2, c, cc;//p1,p2�ֱ�������ģ�͵����ģ�c1,c2�ֱ��ʾȥ���Ļ���ĵ㣬c=c1-c2,cc=c1+c2
	int loop;
	int num = 0;
	tVector sum1, sum2;
	sum1.x = sum1.y = sum1.z = 0;
	sum2.x = sum2.y = sum2.z = 0;
	//fstream fs1;
	//fs1.open("��תƽ�ƾ���.txt", ios::app | ios::out);
	//��������
	for (loop = 0; loop<machePnum; loop++)
	{
			sum1 += dmetchPoints[loop].p1;
			sum2 += dmetchPoints[loop].p2;
			num++;
	}
	if (num>3)//��ʱҪ������4����
	{
		p1.x = sum1.x / num; p1.y = sum1.y / num; p1.z = sum1.z / num;
		p2.x = sum2.x / num; p2.y = sum2.y / num; p2.z = sum2.z / num;
		P1(0) = p1.x; P1(1) = p1.y; P1(2) = p1.z;
		P2(0) = p2.x; P2(1) = p2.y; P2(2) = p2.z;
	}
	else {

		//AfxMessageBox("û��ƥ��ĵ�ԣ��޷���������");
		return 0;
	}
	//��A
	for (loop = 0; loop<machePnum; loop++)
	{
		
		//ȥ���Ļ�
		c1 = dmetchPoints[loop].p1 - p1;
		c2 = dmetchPoints[loop].p2 - p2;
		c = c1 - c2;
		cc = c1 + c2;
		Ai << 0, -(c.x*0.5), -c.y*0.5, -c.z*0.5,
			c.x*0.5, 0, -cc.z, cc.y,
			c.y*0.5, cc.z, 0, -cc.x,
			c.z*0.5, -cc.y, cc.x, 0;
		//ת��
		Ait = Ai.transpose();
		A += (Ait* Ai);

	}
	//��A����С����ֵ�������Ӧ����������
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
	//��ת����
	Q.x() = q(1); Q.y() = q(2); Q.z() = q(3); Q.w() = q(0);
	R = Q.matrix();
	T = P1 - R*P2;
	
	//RTmetchPoints(dmetchPoints, machePnum, R, T);
	return 1;
}
