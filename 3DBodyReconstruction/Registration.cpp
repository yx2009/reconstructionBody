#include "stdafx.h"
#include "Registration.h"
#include "icpPointToPlane.h"
#include "icpPointToPoint.h"
#include "icpPointMoveToPlane.h"
//


#include <fstream>
#include <stdio.h>
#include <set>
#include <list>
#include <time.h>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/xfeatures2d/nonfree.hpp>
#include<opencv2/core/core.hpp>



using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
Registration::Registration(void)
{
	tmachePoint = NULL;//
	dmetchPoints = NULL;//
	targetModel = NULL;//
	partModel = NULL;  //
}

Registration::~Registration(void)
{
}

inline tVector Registration::CountNormalofFace(tVector p1, tVector p2, tVector p3)
{
	t_Triangle triangleTemp;
	tVector normal1, normal2, normalofFace;
	normal1.x = p2.x - p1.x;
	normal1.y = p2.y - p1.y;
	normal1.z = p2.z - p1.z;
	normal2.x = p3.x - p1.x;
	normal2.y = p3.y - p1.y;
	normal2.z = p3.z - p1.z;
	normalofFace.x = normal1.y*normal2.z - normal1.z*normal2.y;
	normalofFace.y = normal1.z*normal2.x - normal1.x*normal2.z;
	normalofFace.z = normal1.x*normal2.y - normal1.y*normal2.x;
	double normalLength = sqrt(normalofFace.x*normalofFace.x + normalofFace.y*normalofFace.y + normalofFace.z*normalofFace.z);
	normalofFace.x = normalofFace.x / normalLength;
	normalofFace.y = normalofFace.y / normalLength;
	normalofFace.z = normalofFace.z / normalLength;
	return normalofFace;
}
double Registration::CountTangle(tVector vec1, tVector vec2)
{
	double tangle=0;
	//点积
	double normalMultiply = vec1.x*vec2.x + vec1.y*vec2.y + vec1.z *vec2.z;
	//模长
	double normalLengthMultply1 = vec1.x*vec1.x + vec1.y*vec1.y + vec1.z*vec1.z;
	double normalLengthMultply2 = vec2.x*vec2.x + vec2.y*vec2.y + vec2.z*vec2.z;
	//夹角
	if (normalLengthMultply1>0&& normalLengthMultply2>0)
	{
		tangle= acos(normalMultiply / (sqrt(normalLengthMultply1) *sqrt(normalLengthMultply2)));
		if (_isnan(tangle))
		{
			//AfxMessageBox("tangle is NaN.");
		}
		return tangle;
	}
	
	return tangle;
}
double Registration::CountVecLenth(tVector vec1)
{
	return sqrt(vec1.x*vec1.x + vec1.y*vec1.y + vec1.z*vec1.z);
}

/************************************************************************/
/* 功能：计算配准误差(点到点)                                                                     */
/************************************************************************/
double Registration::errorDistP2P()
{
	double resulterror=0.0;
	int32_t dim = 3;
	Eigen::Matrix3d ER;
	Eigen::Vector3d ET;
	ER << 1, 0, 0,
		  0, 1, 0,
		  0, 0, 1;
	ET << 0, 0, 0;
	/************************************************************************/
	/*MY*/
	//我的头像
	//ER << 0.996336, 0.0733061, -0.0440656,		-0.073741, 0.997243, -0.00832569,		0.0433338, 0.0115446, 0.998994;
	//ET << -55.7596,		67.7527,		31.4949;
	//女头像
	//ER << 0.98327, -0.0100465, 0.181879,		0.000917852, 0.998738, 0.0502054,		-0.182154, -0.0491985, 0.982038;
	//ET << 77.2223,		-3.65279,		-4.23298;
	//teapot
	/*ER << 0.992644, -0.120566, 0.0110687,		0.120493, 0.992689, 0.00699775,		-0.0118315, -0.00561256, 0.999914;
	ET << 47.6547,		13.9512,		-5.58962;*/

	/************************************************************************/
	 /*FPFH*/ 
	 //我的头像
	//ER << 0.9978, 0.0640034, -0.0172906,		-0.0641333, 0.997917, -0.00705019,		0.0168034, 0.00814348, 0.999826;
	//ET << -57.9142, 65.6187, 34.5646;
	//女头像
	//ER << 0.936117, 0.0687426, 0.344904, 		- 0.0928375, 0.994226, 0.0538154,		- 0.339213, -0.0823974, 0.937094;
	//ET << 83.6349, -6.997, -11.9214;
	//teapot
	//ER << 0.996563, -0.0657854, -0.0503626,		0.0706754, 0.992227, 0.102426,		0.0432329, -0.105633, 0.993465; 
	//ET << 39.998, 19.4641, -0.138387;

	
	/************************************************************************/
	/*SUPER4PCS*/
	//我的头像
	//ER << 0.995274, 0.0909121, 0.034127, -0.0920913, 0.995144, 0.0347386, -0.0308031, -0.0377173, 0.998814;
	//ET << -54.3501, 67.611, 35.0852;
	//女头像
	//ER << 0.972344, -0.00246128, 0.233541,		- 0.00848452, 0.998912, 0.045853,		- 0.2334, -0.0465664, 0.971265; 
	//ET << 77.9106, -4.26076, -5.7007;
	//teapot
	//ER << 0.966403, 0.0396788, -0.253949, -0.044786, 0.998893, -0.0143594, 0.253098, 0.0252503, 0.967111;
	//ET << 31.1365, 8.61277, 1.17449;
	/************************************************************************/
	fstream filesss;
	//filesss.open("record001.txt", ios::out | ios::app);

	int ModelvertexNum = targetModel->vertexCnt;//模型点数量
	int GModelvertexNum = partModel->vertexCnt;//转换模型数量
	double* M = (double*)calloc(3 * targetModel->vertexCnt, sizeof(double));
	double* T = (double*)calloc(3 * partModel->vertexCnt, sizeof(double));
	int32_t k = 0;
	

	for (int i = 0; i < (targetModel->vertexCnt); i++)
	{
		M[k * 3 + 0] = targetModel->vertex[i].x;
		M[k * 3 + 1] = targetModel->vertex[i].y;
		M[k * 3 + 2] = targetModel->vertex[i].z;
		k++;
	}
	k = 0;

	RTModel(partModel, ER, ET);
	for (int i = 0; i < (partModel->vertexCnt); i++)
	{
		T[k * 3 + 0] = partModel->vertex[i].x;//x方向平移，此处是为同一模型做实验！！！！
		T[k * 3 + 1] = partModel->vertex[i].y;
		T[k * 3 + 2] = partModel->vertex[i].z;
		k++;
	}

	selfMatrix::Matrix R = selfMatrix::Matrix::eye(3);
	selfMatrix::Matrix t(3, 1);

	IcpPointToPoint icp(M, ModelvertexNum, dim);
	resulterror=icp.averLength(T, GModelvertexNum, R, t, 10);

	
	free(M);
	free(T);

	return resulterror;
}
int Registration::ICP_PointToPlaneRegister()
{
	int32_t dim = 3;
	int32_t num = 10000;
	Eigen::Matrix3d ER;
	Eigen::Vector3d ET;
	fstream filesss;
	filesss.open("旋转平移矩阵.txt", ios::out | ios::app);
	// allocate model and template memory
	/*double* M = (double*)calloc(3 * num, sizeof(double));
	double* T = (double*)calloc(3 * num, sizeof(double));*/

	int ModelvertexNum = targetModel->vertexCnt;//模型点数量
	int GModelvertexNum = partModel->vertexCnt;//转换模型数量
	double* M = (double*)calloc(3*targetModel->vertexCnt, sizeof(double));
	double* T = (double*)calloc(3*partModel->vertexCnt, sizeof(double));
	// set model and template points
	//cout << endl << "Creating model with 10000 points ..." << endl;
	//cout << "Creating template by shifting model by (1,1,1) ..." << endl;
	int32_t k = 0;
	//for (double x = -2; x<2; x += 0.04) {
	//	for (double y = -2; y<2; y += 0.04) {
	//		double z = 5 * x*exp(-x*x - y*y);
	//		M[k * 3 + 0] = x;
	//		M[k * 3 + 1] = y;
	//		M[k * 3 + 2] = z;
	//		T[k * 3 + 0] = x - 1;
	//		T[k * 3 + 1] = y - 1;
	//		T[k * 3 + 2] = z - 1;
	//		k++;
	//	}
	//}
	/************************************************************************/
	/* 取简化的点   100倍                                                                  */
	/************************************************************************/

	//for (int i = 0; i < (m_Model.vertexCnt / 100); i++)
	//{
	//	M[k * 3 + 0] = m_Model.vertex[i * 100].x;
	//	M[k * 3 + 1] = m_Model.vertex[i * 100].y;
	//	M[k * 3 + 2] = m_Model.vertex[i * 100].z;
	//	k++;
	//}
	//k = 0;
	//for (int i = 0; i < (m_GModel.vertexCnt / 100); i++)
	//{
	//	T[k * 3 + 0] = m_GModel.vertex[i * 100].x + 0.001;//x方向平移，此处是为同一模型做实验！！！！
	//	T[k * 3 + 1] = m_GModel.vertex[i * 100].y + 0.002;
	//	T[k * 3 + 2] = m_GModel.vertex[i * 100].z + 0.003;
	//	k++;
	//}	
	/************************************************************************/
		/* 未简化的点   倍                                                                  */
		/************************************************************************/

	for (int i = 0; i < (targetModel->vertexCnt); i++)
	{
		M[k * 3 + 0] = targetModel->vertex[i].x;
		M[k * 3 + 1] = targetModel->vertex[i].y;
		M[k * 3 + 2] = targetModel->vertex[i].z;
		k++;
	}
	k = 0;
	for (int i = 0; i < (partModel->vertexCnt); i++)
	{
		T[k * 3 + 0] = partModel->vertex[i].x ;//x方向平移，此处是为同一模型做实验！！！！
		T[k * 3 + 1] = partModel->vertex[i].y ;
		T[k * 3 + 2] = partModel->vertex[i].z ;
		k++;
	}
	// start with identity as initial transformation
	// in practice you might want to use some kind of prediction here
	selfMatrix::Matrix R = selfMatrix::Matrix::eye(3);
	selfMatrix::Matrix t(3, 1);

	// run point-to-plane ICP (-1 = no outlier threshold)
	//cout << endl << "Running ICP (point-to-plane, no outliers)" << endl;
	IcpPointToPlane icp(M,ModelvertexNum, dim);
	icp.fit(T, GModelvertexNum, R, t, -1);

	// results
	/* cout << endl << "Transformation results:" << endl;
	cout << "R:" << endl << R << endl << endl;
	cout << "t:" << endl << t << endl << endl;*/
	/************************************************************************/
	/* add by yingxiang 20170519                                                                     */
	/************************************************************************/
	if (filesss) {
		filesss << R << endl;
		filesss << t << endl;

	}
	/************************************************************************/
	/* 初始化转换矩阵RT                                                                     */
	/************************************************************************/
	//转换矩阵
	selfMatrix::Matrix RT(4, 4);
	for (int32_t i = 0; i < R.m; i++) {
		for (int32_t j = 0; j < R.n; j++) {

			RT.val[i][j] = R.val[i][j];
			ER(i, j) = R.val[i][j];
		}
	}
	for (int i=0;i<3;i++)
	{
		RT.val[i][3] = t.val[i][0];
		ET(i) = t.val[i][0];
	}
	for (int i=0;i<3;i++)
	{
		RT.val[3][i] = 0;
	}
	RT.val[3][3] = 1;
	RTModel(partModel, ER, ET);
	char buffer[1024];
	for (int i=0;i<RT.m;i++)
	{
		for (int j = 0; j < RT.n;j++)
		{
			sprintf(buffer, "%12.7f ", RT.val[i][j]);
			sscanf(buffer, "%f", &Rt[j*4+i]);

		}
	}
	//drawScene();
	//drawGScene();
	filesss.close();
	// free memory
	free(M);
	free(T);

	// success
	return true;
}
/************************************************************************/
/* 功能：nonrigid 非刚性变形                                                                     */
/************************************************************************/
int Registration:: ICP_Point_nonrigid()
{
	int32_t dim = 3;
	int32_t num = 10000;
	Eigen::Matrix3d ER;
	Eigen::Vector3d ET;
	//fstream filesss;
	//filesss.open("record0001.txt", ios::out | ios::app);
	// allocate model and template memory
	/*double* M = (double*)calloc(3 * num, sizeof(double));
	double* T = (double*)calloc(3 * num, sizeof(double));*/
	if (partModel->normalCnt<=0)
	{
		vertexNormal(partModel);
	}
	if (targetModel->normalCnt <= 0)
	{
		vertexNormal(targetModel);
	}
	int ModelvertexNum = targetModel->vertexCnt;//模型点数量
	int GModelvertexNum = partModel->vertexCnt;//转换模型数量
	double* M = (double*)calloc(3 * targetModel->vertexCnt, sizeof(double));
	double* Mn = (double*)calloc(3 * targetModel->vertexCnt, sizeof(double));
	double* T = (double*)calloc(3 * partModel->vertexCnt, sizeof(double));
	double* Tn = (double*)calloc(3 * partModel->vertexCnt, sizeof(double));
	// set model and template points
	//cout << endl << "Creating model with 10000 points ..." << endl;
	//cout << "Creating template by shifting model by (1,1,1) ..." << endl;
	int32_t k = 0;
	//此处默认模型具有法向量，否则程序错误待改进yingxiang
	for (int i = 0; i < (targetModel->vertexCnt); i++)
	{
		M[k * 3 + 0] = targetModel->vertex[i].x;
		M[k * 3 + 1] = targetModel->vertex[i].y;
		M[k * 3 + 2] = targetModel->vertex[i].z;
		Mn[k * 3 + 0] = targetModel->normal[i].x;
		Mn[k * 3 + 1] = targetModel->normal[i].y;
		Mn[k * 3 + 2] = targetModel->normal[i].z;
		//filesss << Mn[k * 3 + 0] << "  " << Mn[k * 3 + 1] << "   " << Mn[k * 3 + 2] << endl;
		k++;
	}
	k = 0;
	for (int i = 0; i < (partModel->vertexCnt); i++)
	{
		T[k * 3 + 0] = partModel->vertex[i].x;
		T[k * 3 + 1] = partModel->vertex[i].y;
		T[k * 3 + 2] = partModel->vertex[i].z;
		//向量
		Tn[k * 3 + 0] = partModel->normal[i].x;
		Tn[k * 3 + 1] = partModel->normal[i].y;
		Tn[k * 3 + 2] = partModel->normal[i].z;

		k++;
	}
	// start with identity as initial transformation
	// in practice you might want to use some kind of prediction here
	selfMatrix::Matrix R = selfMatrix::Matrix::eye(3);
	selfMatrix::Matrix t(3, 1);

	// run point-to-plane ICP (-1 = no outlier threshold)
	//cout << endl << "Running ICP (point-to-plane, no outliers)" << endl;
	icpPointMoveToPlane icp(M, ModelvertexNum, dim, Mn, targetModel, partModel);
	icp.fit_nonrigid(T,Tn, GModelvertexNum, R, t, 10);//参数根据模型实际情况定
	k = 0;
	ARAPDeform arap(partModel, T);
	arap.applyDeform();
	for (int i = 0; i < (partModel->vertexCnt); i++)
	{
		partModel->vertex[i].x = T[k * 3 + 0];//x方向平移，此处是为同一模型做实验！！！！
		partModel->vertex[i].y = T[k * 3 + 1];
		partModel->vertex[i].z = T[k * 3 + 2];
		k++;
	}
	// results
	/* cout << endl << "Transformation results:" << endl;
	cout << "R:" << endl << R << endl << endl;
	cout << "t:" << endl << t << endl << endl;*/
	/************************************************************************/
	/* add by yingxiang 20170519                                                                     */
	/************************************************************************/
	//if (filesss) {
	//	filesss << R << endl;
	//	filesss << t << endl;

	//}
	/************************************************************************/
	/* 初始化转换矩阵RT                                                                     */
	/************************************************************************/
	//转换矩阵
	/*selfMatrix::Matrix RT(4, 4);
	for (int32_t i = 0; i < R.m; i++) {
		for (int32_t j = 0; j < R.n; j++) {

			RT.val[i][j] = R.val[i][j];
			ER(i, j) = R.val[i][j];
		}
	}
	for (int i = 0; i<3; i++)
	{
		RT.val[i][3] = t.val[i][0];
		ET(i) = t.val[i][0];
	}
	for (int i = 0; i<3; i++)
	{
		RT.val[3][i] = 0;
	}
	RT.val[3][3] = 1;
	RTModel(partModel, ER, ET);
	char buffer[1024];
	for (int i = 0; i<RT.m; i++)
	{
		for (int j = 0; j < RT.n; j++)
		{
			sprintf(buffer, "%12.7f ", RT.val[i][j]);
			sscanf(buffer, "%f", &Rt[j * 4 + i]);

		}
	}*/
	//drawScene();
	//drawGScene();
	//filesss.close();
	// free memory
	free(M);
	free(Mn);
	free(T);
	free(Tn);

	// success
	return 1;
}

/************************************************************************/
/* 功能：计算模型顶点的法向量          20180424修改顶点结构体 加入状态 status 标记点的状态        */
/************************************************************************/
void Registration::vertexNormal(t_Visual *originalVisual) {

	t_Visual myVisual = *originalVisual;
	//fstream filest;
	long tempfindex;
	std::set<int > ppindex;
	int temp;
	int tempsize=0;
	double tempr;
	//fstream fnormal;
	//fnormal.open("fnormal.txt", ios::app | ios::out);

	//filest.open("record001.txt", ios::app | ios::out);

	/************************************************************************/
	/*
	//根据点云源数据中顶点和面片数量的关系，
	//当顶点远大于面片数量：说明需要简化点
	//当顶点和面片数量：不需要简化
	*/
	/************************************************************************/

	if (myVisual.normalCnt <= 0)
	{

		myVisual.normal = (tVector *)malloc(myVisual.vertexCnt * sizeof(tVector));
		originalVisual->normal = (tVector*)malloc(originalVisual->vertexCnt * sizeof(tVector));

	}
	else
		return;
	for (int i = 0; i >= 0 && i<myVisual.vertexCnt; i++)
	{

		vertexsOfonevertex GroupVertex;//定义面片顶点组
		vertexOfFace Vertex4Normal[20];//定义面片顶点组(组内顶点按顺序排列)，以计算顶点法向量
		GroupVertex.numOfGroup = 0;




			//循环遍历查找与顶点相同的点
		for (int j = 0; j<myVisual.vfindex[i].num; j++)
		{


			tempfindex = myVisual.vfindex[i].findex[j];
			temp = GroupVertex.numOfGroup;

			if (myVisual.index[tempfindex].v[0] == i)
			{
				GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[1]].x;
				GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[1]].y;
				GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[1]].z;

				GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[2]].x;
				GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[2]].y;
				GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[2]].z;

				Vertex4Normal[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[0]].x;
				Vertex4Normal[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[0]].y;
				Vertex4Normal[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[0]].z;

				Vertex4Normal[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[1]].x;
				Vertex4Normal[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[1]].y;
				Vertex4Normal[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[1]].z;

				Vertex4Normal[temp].p3.x = myVisual.vertex[myVisual.index[tempfindex].v[2]].x;
				Vertex4Normal[temp].p3.y = myVisual.vertex[myVisual.index[tempfindex].v[2]].y;
				Vertex4Normal[temp].p3.z = myVisual.vertex[myVisual.index[tempfindex].v[2]].z;

				GroupVertex.numOfGroup += 1;
				ppindex.insert(myVisual.index[j].v[1]);
				ppindex.insert(myVisual.index[j].v[2]);

			}
			else if (myVisual.index[tempfindex].v[1] == i)
			{
				GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[0]].x;
				GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[0]].y;
				GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[0]].z;
				GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[2]].x;
				GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[2]].y;
				GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[2]].z;


				Vertex4Normal[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[0]].x;
				Vertex4Normal[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[0]].y;
				Vertex4Normal[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[0]].z;

				Vertex4Normal[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[1]].x;
				Vertex4Normal[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[1]].y;
				Vertex4Normal[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[1]].z;

				Vertex4Normal[temp].p3.x = myVisual.vertex[myVisual.index[tempfindex].v[2]].x;
				Vertex4Normal[temp].p3.y = myVisual.vertex[myVisual.index[tempfindex].v[2]].y;
				Vertex4Normal[temp].p3.z = myVisual.vertex[myVisual.index[tempfindex].v[2]].z;

				GroupVertex.numOfGroup += 1;
				ppindex.insert(myVisual.index[j].v[0]);
				ppindex.insert(myVisual.index[j].v[2]);
			}
			else if (myVisual.index[tempfindex].v[2] == i)
			{
				GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[0]].x;
				GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[0]].y;
				GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[0]].z;
				GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[1]].x;
				GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[1]].y;
				GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[1]].z;


				Vertex4Normal[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[0]].x;
				Vertex4Normal[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[0]].y;
				Vertex4Normal[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[0]].z;

				Vertex4Normal[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[1]].x;
				Vertex4Normal[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[1]].y;
				Vertex4Normal[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[1]].z;

				Vertex4Normal[temp].p3.x = myVisual.vertex[myVisual.index[tempfindex].v[2]].x;
				Vertex4Normal[temp].p3.y = myVisual.vertex[myVisual.index[tempfindex].v[2]].y;
				Vertex4Normal[temp].p3.z = myVisual.vertex[myVisual.index[tempfindex].v[2]].z;

				GroupVertex.numOfGroup += 1;

				ppindex.insert(myVisual.index[j].v[0]);
				ppindex.insert(myVisual.index[j].v[1]);
			}

		}

		if (GroupVertex.numOfGroup <= 2|| (GroupVertex.numOfGroup - ppindex.size()) < 0)//外部边缘点 或者洞边缘
		{
			originalVisual->vertex[i].status = 1;
			MAKEVECTOR(originalVisual->normal[i], 0, 0, 0);// myVisual.normal[i];
			//continue;
		}
		//计算顶点法向量

		myVisual.normal[i].x = 0;
		myVisual.normal[i].y = 0;
		myVisual.normal[i].z = 0;
		for (int m = 0; m < GroupVertex.numOfGroup; m++)
		{
			tVector tempnormal;
			tempnormal = CountNormalofFace(Vertex4Normal[m].p1, Vertex4Normal[m].p2, Vertex4Normal[m].p3);

			myVisual.normal[i].x += tempnormal.x;
			myVisual.normal[i].y += tempnormal.y;
			myVisual.normal[i].z += tempnormal.z;


		}
		NormalizeVector(&(myVisual.normal[i]));
		originalVisual->normal[i] = myVisual.normal[i];
		//fnormal << originalVisual->normal[i].x << "  " << originalVisual->normal[i].y << "  "<<originalVisual->normal[i].z << endl;


	}
	originalVisual->normalCnt = originalVisual->vertexCnt;
	
	
}
//************************************************************************/
/* 计算高斯曲率
    ﹌﹌﹌﹌﹌﹌﹌适合Kinect获取的OBJ文件和普通obj文件*/
//************************************************************************/

void Registration::GuassCurvature(t_Visual *originalVisual)
{

	t_Visual myVisual = *originalVisual;
	fstream filest;
	long tempfindex; 
	int temp;
	double tempr;
	filest.open("record001.txt", ios::app | ios::out);
	//vertexsOfonevertex *GroupVertex =(vertexsOfonevertex *)malloc(originalVisual->vertexCnt * sizeof(vertexsOfonevertex));
	//vertexOfFace *vertexGroup=(vertexOfFace *)malloc(10*sizeof(vertexOfFace));
	/************************************************************************/
	/*
	//根据点云源数据中顶点和面片数量的关系，
	//当顶点远大于面片数量：说明需要简化点
	//当顶点和面片数量：不需要简化
	*/
	/************************************************************************/
	if (0<myVisual.vertexCnt&&myVisual.vertexCnt - myVisual.faceCnt <= myVisual.faceCnt)
	{
		if (myVisual.normalCnt <= 0)
		{

			myVisual.normal = (tVector *)malloc(myVisual.vertexCnt * sizeof(tVector));
			originalVisual->normal = (tVector*)malloc(originalVisual->vertexCnt * sizeof(tVector));

		}
		for (int i = 0; i >= 0 && i<myVisual.vertexCnt; i++)
		{

			vertexsOfonevertex GroupVertex;//定义面片顶点组
			vertexOfFace Vertex4Normal[20];//定义面片顶点组(组内顶点按顺序排列)，以计算顶点法向量
			GroupVertex.numOfGroup = 0;


			if (myVisual.normalCnt>0)
			{
				//循环遍历查找与顶点相同的点
				for (int j = 0; j<myVisual.vfindex[i].num; j++)
				{

					tempfindex = myVisual.vfindex[i].findex[j];
					temp = GroupVertex.numOfGroup;
					if (myVisual.index[tempfindex].v[0] == i)
					{
						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[1]].x;
						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[1]].y;
						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[1]].z;
						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[2]].x;
						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[2]].y;
						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[2]].z;
						GroupVertex.numOfGroup += 1;

					}
					else if (myVisual.index[tempfindex].v[1] == i)
					{
						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[0]].x;
						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[0]].y;
						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[0]].z;
						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[2]].x;
						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[2]].y;
						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[2]].z;
						GroupVertex.numOfGroup += 1;
					}
					else if (myVisual.index[tempfindex].v[2] == i)
					{
						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[0]].x;
						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[0]].y;
						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[0]].z;
						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[1]].x;
						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[1]].y;
						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[1]].z;
						GroupVertex.numOfGroup += 1;
					}

				}

			}
			else
			{

				//循环遍历查找与顶点相同的点
				for (int j = 0; j<myVisual.vfindex[i].num; j++)
				{


					tempfindex = myVisual.vfindex[i].findex[j];
					temp = GroupVertex.numOfGroup;

					if (myVisual.index[tempfindex].v[0] == i)
					{
						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[1]].x;
						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[1]].y;
						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[1]].z;

						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[2]].x;
						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[2]].y;
						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[2]].z;

						Vertex4Normal[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[0]].x;
						Vertex4Normal[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[0]].y;
						Vertex4Normal[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[0]].z;

						Vertex4Normal[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[1]].x;
						Vertex4Normal[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[1]].y;
						Vertex4Normal[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[1]].z;

						Vertex4Normal[temp].p3.x = myVisual.vertex[myVisual.index[tempfindex].v[2]].x;
						Vertex4Normal[temp].p3.y = myVisual.vertex[myVisual.index[tempfindex].v[2]].y;
						Vertex4Normal[temp].p3.z = myVisual.vertex[myVisual.index[tempfindex].v[2]].z;

						GroupVertex.numOfGroup += 1;

					}
					else if (myVisual.index[tempfindex].v[1] == i)
					{
						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[0]].x;
						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[0]].y;
						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[0]].z;
						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[2]].x;
						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[2]].y;
						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[2]].z;


						Vertex4Normal[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[0]].x;
						Vertex4Normal[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[0]].y;
						Vertex4Normal[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[0]].z;

						Vertex4Normal[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[1]].x;
						Vertex4Normal[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[1]].y;
						Vertex4Normal[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[1]].z;

						Vertex4Normal[temp].p3.x = myVisual.vertex[myVisual.index[tempfindex].v[2]].x;
						Vertex4Normal[temp].p3.y = myVisual.vertex[myVisual.index[tempfindex].v[2]].y;
						Vertex4Normal[temp].p3.z = myVisual.vertex[myVisual.index[tempfindex].v[2]].z;

						GroupVertex.numOfGroup += 1;
					}
					else if (myVisual.index[tempfindex].v[2] == i)
					{
						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[0]].x;
						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[0]].y;
						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[0]].z;
						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[1]].x;
						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[1]].y;
						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[1]].z;


						Vertex4Normal[temp].p1.x = myVisual.vertex[myVisual.index[tempfindex].v[0]].x;
						Vertex4Normal[temp].p1.y = myVisual.vertex[myVisual.index[tempfindex].v[0]].y;
						Vertex4Normal[temp].p1.z = myVisual.vertex[myVisual.index[tempfindex].v[0]].z;

						Vertex4Normal[temp].p2.x = myVisual.vertex[myVisual.index[tempfindex].v[1]].x;
						Vertex4Normal[temp].p2.y = myVisual.vertex[myVisual.index[tempfindex].v[1]].y;
						Vertex4Normal[temp].p2.z = myVisual.vertex[myVisual.index[tempfindex].v[1]].z;

						Vertex4Normal[temp].p3.x = myVisual.vertex[myVisual.index[tempfindex].v[2]].x;
						Vertex4Normal[temp].p3.y = myVisual.vertex[myVisual.index[tempfindex].v[2]].y;
						Vertex4Normal[temp].p3.z = myVisual.vertex[myVisual.index[tempfindex].v[2]].z;

						GroupVertex.numOfGroup += 1;
					}

				}

			}
			if (GroupVertex.numOfGroup <= 2)
			{
				//myVisual.vertex[i].z = myVisual.vertexMax_z + 2;
				originalVisual->GuassCurvature[i] = 0.0;
				MAKEVECTOR(originalVisual->normal[i], 0, 0, 0);// myVisual.normal[i];
				continue;
			}

			//计算高斯曲率
			double sumAngle = 0.0;
			double sumArea = 0.0;
			tVector *facevector = new tVector[GroupVertex.numOfGroup];

			//判断是否需要计算顶点法向量
			if (myVisual.normalCnt <= 0)
			{
				myVisual.normal[i].x = 0;
				myVisual.normal[i].y = 0;
				myVisual.normal[i].z = 0;
				for (int m = 0; m < GroupVertex.numOfGroup; m++)
				{
					tVector tempnormal;
					tempnormal = CountNormalofFace(Vertex4Normal[m].p1, Vertex4Normal[m].p2, Vertex4Normal[m].p3);

					myVisual.normal[i].x += tempnormal.x;
					myVisual.normal[i].y += tempnormal.y;
					myVisual.normal[i].z += tempnormal.z;


				}
				NormalizeVector(&(myVisual.normal[i]));
				originalVisual->normal[i] = myVisual.normal[i];
			}
			//tVector *normalOfFace=(tVector *)malloc(myVisual.GroupVertex[i].numOfGroup*sizeof(tVector));
			for (int m = 0; m < GroupVertex.numOfGroup; m++)
			{
				t_Triangle tempTriangle = CountTangle(myVisual.vertex[i], GroupVertex.twoPoints[m].p1, GroupVertex.twoPoints[m].p2);
				sumAngle += tempTriangle.angle;
				sumArea += tempTriangle.area;
				facevector[m].x = tempTriangle.FaceVector.x;
				facevector[m].y = tempTriangle.FaceVector.y;
				facevector[m].z = tempTriangle.FaceVector.z;
			}
			//计算平均曲率
			double sumHcurvature = 0;//未除面积的平均曲率
			tVector sumHVector;
			MAKEVECTOR(sumHVector, 0, 0, 0);
			for (int m = 0; m < GroupVertex.numOfGroup; m++)
			{
				tVector edge1, edge2, edge3, edge4;
				//tVector normal1,normal2;
				double normalangle = 0.0, normalangle1 = 0.0, normalangle2 = 0.0;
				double edgelenth = 0.0;
				double tempH = 0;

				edge1.x = GroupVertex.twoPoints[m].p1.x - myVisual.vertex[i].x;
				edge1.y = GroupVertex.twoPoints[m].p1.y - myVisual.vertex[i].y;
				edge1.z = GroupVertex.twoPoints[m].p1.z - myVisual.vertex[i].z;
				edge2.x = GroupVertex.twoPoints[m].p2.x - myVisual.vertex[i].x;
				edge2.y = GroupVertex.twoPoints[m].p2.y - myVisual.vertex[i].y;
				edge2.z = GroupVertex.twoPoints[m].p2.z - myVisual.vertex[i].z;
				for (int mm = 0; mm < GroupVertex.numOfGroup; mm++)
				{
					if (mm != m)//忽略同一条边
					{
						//利用当前面和这个面的法向量n1、n2求出夹角，算出部分曲率H^
						edge3.x = GroupVertex.twoPoints[mm].p1.x - myVisual.vertex[i].x;
						edge3.y = GroupVertex.twoPoints[mm].p1.y - myVisual.vertex[i].y;
						edge3.z = GroupVertex.twoPoints[mm].p1.z - myVisual.vertex[i].z;
						edge4.x = GroupVertex.twoPoints[mm].p2.x - myVisual.vertex[i].x;
						edge4.y = GroupVertex.twoPoints[mm].p2.y - myVisual.vertex[i].y;
						edge4.z = GroupVertex.twoPoints[mm].p2.z - myVisual.vertex[i].z;
						//判断共边
						if (edge1.x == edge3.x&&edge1.y == edge3.y&&edge1.z == edge3.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角,Dyn的方法
							normalangle1 = CountTangle(edge2, edge2 - edge1);//Meyer的方法
							normalangle2 = CountTangle(edge4, edge4 - edge3);
							//edgelenth = CountVecLenth(edge1);//计算共边的长度
							//sumHcurvature += normalangle*edgelenth;//Dyn的方法
							tempH = 1 / tan(normalangle1) + 1 / tan(normalangle2);
							MAKEVECTOR(sumHVector, sumHVector.x + tempH*edge1.x,
								sumHVector.y + tempH*edge1.y,
								sumHVector.z + tempH*edge1.z);


						}
						else if (edge1.x == edge4.x&&edge1.y == edge4.y&&edge1.z == edge4.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
							normalangle1 = CountTangle(edge2, edge2 - edge1);//Meyer的方法
							normalangle2 = CountTangle(edge3, edge3 - edge4);
							//edgelenth = CountVecLenth(edge1);//计算共边的长度
							//sumHcurvature += normalangle*edgelenth;
							tempH = 1 / tan(normalangle1) + 1 / tan(normalangle2);
							MAKEVECTOR(sumHVector, sumHVector.x + tempH*edge1.x,
								sumHVector.y + tempH*edge1.y,
								sumHVector.z + tempH*edge1.z);


						}
						else if (edge2.x == edge4.x&&edge2.y == edge4.y&&edge2.z == edge4.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
							normalangle1 = CountTangle(edge1, edge1 - edge2);//Meyer的方法
							normalangle2 = CountTangle(edge3, edge3 - edge4);
							//edgelenth = CountVecLenth(edge2);//计算共边的长度
							//sumHcurvature += normalangle*edgelenth;
							tempH = 1 / tan(normalangle1) + 1 / tan(normalangle2);
							MAKEVECTOR(sumHVector, sumHVector.x + tempH*edge2.x,
								sumHVector.y + tempH*edge2.y,
								sumHVector.z + tempH*edge2.z);


						}
						else if (edge2.x == edge3.x&&edge2.y == edge3.y&&edge2.z == edge3.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
							normalangle1 = CountTangle(edge1, edge1 - edge2);//Meyer的方法
							normalangle2 = CountTangle(edge4, edge4 - edge3);
							//edgelenth = CountVecLenth(edge2);//计算共边的长度
							//sumHcurvature += normalangle*edgelenth;
							tempH = 1 / tan(normalangle1) + 1 / tan(normalangle2);
							MAKEVECTOR(sumHVector, sumHVector.x + tempH*edge2.x,
								sumHVector.y + tempH*edge2.y,
								sumHVector.z + tempH*edge2.z);


						}
					}

				}
			}


			delete[] facevector;
			if (sumArea > 0)
			{
				Eigen::Vector3d n1, n2;
				n1 << sumHVector.x, sumHVector.y, sumHVector.z;
				n2 << myVisual.normal[i].x, myVisual.normal[i].y, myVisual.normal[i].z;
				originalVisual->HCurvature[i] = 0.25*(n1.dot(n2)) / sumArea;
				originalVisual->GuassCurvature[i] = (2 * 3.1415926 - sumAngle) / sumArea;
				if (originalVisual->GuassCurvature[i]>0)
				{
					int tempGaussNum = 0;
					for (int m = 0; m < GroupVertex.numOfGroup; m++)
					{
						//求顶点邻边是否与法向量锐角
						double MultiVector1 = myVisual.normal[i].x*(GroupVertex.twoPoints[m].p1.x - myVisual.vertex[i].x)
							+ myVisual.normal[i].y*(GroupVertex.twoPoints[m].p1.y - myVisual.vertex[i].y)
							+ myVisual.normal[i].z*(GroupVertex.twoPoints[m].p1.z - myVisual.vertex[i].z);
						double MultiVector2 = myVisual.normal[i].x*(GroupVertex.twoPoints[m].p2.x - myVisual.vertex[i].x)
							+ myVisual.normal[i].y*(GroupVertex.twoPoints[m].p2.y - myVisual.vertex[i].y)
							+ myVisual.normal[i].z*(GroupVertex.twoPoints[m].p2.z - myVisual.vertex[i].z);
						if (MultiVector1 >= 0 && MultiVector2 >= 0)
						{
							tempGaussNum++;
						}

					}
					if (tempGaussNum >= GroupVertex.numOfGroup)
					{

						originalVisual->GuassCurvature[i] = -originalVisual->GuassCurvature[i];
					}
				}
				else
				{
					originalVisual->GuassCurvature[i] = 0;//马鞍曲面
				}

			}
			else
			{
				originalVisual->HCurvature[i] = 0;
				originalVisual->GuassCurvature[i] = 1.0;
			}
			if (i == 1)
			{
				originalVisual->GuassMaxValue = originalVisual->GuassCurvature[i];
				originalVisual->GuassMinValue = originalVisual->GuassCurvature[i];
			}
			if (originalVisual->GuassCurvature[i]> originalVisual->GuassMaxValue)
			{
				originalVisual->GuassMaxValue = originalVisual->GuassCurvature[i];//取最大值
			}
			else if (originalVisual->GuassCurvature[i] < originalVisual->GuassMinValue)
			{
				originalVisual->GuassMinValue = originalVisual->GuassCurvature[i];//取最小值
			}
			if (filest)
			{
				filest << originalVisual->GuassCurvature[i] << endl; //<<"\t"<<sumAngle<< "\t" << sumArea<< endl;
																	 //filest << originalVisual->HCurvature[i] * originalVisual->HCurvature[i] - originalVisual->GuassCurvature[i] << endl;
			}

		}
		originalVisual->normalCnt = originalVisual->vertexCnt;
		myVisual.normalCnt = originalVisual->vertexCnt;
	}
	if (0<myVisual.vertexCnt&&myVisual.vertexCnt - myVisual.faceCnt>myVisual.faceCnt)
	{
		//循环遍历顶点

		for (int i = 0; i<myVisual.vertexCnt; i++)
		{

			if (myVisual.vertex[i].x == myVisual.vertexMax_x + 10)//跳过已标记顶点 
			{
				originalVisual->GuassCurvature[i] = 0.0;
				if (filest)
				{
					//filest << originalVisual->GuassCurvature[i] << endl;
				}
				continue;
			}
			vertexsOfonevertex GroupVertex;//定义面片顶点组
			GroupVertex.numOfGroup = 0;
			//循环遍历查找与顶点重复的点
			for (int j = 0; j<myVisual.vertexCnt - i - 1; j++)
			{
				//判断下一个点是否已经标记，若标记则进行下次循环
				if (GroupVertex.numOfGroup>8)
				{
					break;
				}
				int sumji = j + i;//当前点的位置的j个位移
				if (myVisual.vertex[sumji].x == myVisual.vertexMax_x + 10)//跳过标记顶点
					continue;

				//从当前顶点位置开始查找判断，以便将当前顶点收入组中
				if (myVisual.vertex[sumji].x == myVisual.vertex[i].x&&
					myVisual.vertex[sumji].y == myVisual.vertex[i].y&&
					myVisual.vertex[sumji].z == myVisual.vertex[i].z)
				{

					//将重复点用当前点云的坐标最大值标记
					if (GroupVertex.numOfGroup>0)
					{
						myVisual.vertex[sumji].x = myVisual.vertexMax_x + 10;
					}
					//遍历面片，将面片的顶点索引修改为所有重复顶点的第一个顶点的索引,将同一顶点面片的其他点信息收入组中

					int temp = GroupVertex.numOfGroup;
					int tempvsite = (sumji) / 3;
					int tempvsiteyu = (sumji) % 3;
					if (myVisual.index[tempvsite].v[0] == sumji)
					{
						myVisual.index[tempvsite].v[0] = i;
						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempvsite].v[1]].x;
						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempvsite].v[1]].y;
						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempvsite].v[1]].z;
						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempvsite].v[2]].x;
						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempvsite].v[2]].y;
						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempvsite].v[2]].z;
						GroupVertex.numOfGroup += 1;

					}
					else if (myVisual.index[tempvsite].v[1] == sumji)
					{
						myVisual.index[tempvsite].v[1] = i;
						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempvsite].v[0]].x;
						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempvsite].v[0]].y;
						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempvsite].v[0]].z;
						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempvsite].v[2]].x;
						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempvsite].v[2]].y;
						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempvsite].v[2]].z;
						GroupVertex.numOfGroup += 1;
					}
					else if (myVisual.index[tempvsite].v[2] == sumji)
					{
						myVisual.index[tempvsite].v[2] = i;
						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempvsite].v[0]].x;
						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempvsite].v[0]].y;
						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempvsite].v[0]].z;
						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempvsite].v[1]].x;
						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempvsite].v[1]].y;
						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempvsite].v[1]].z;
						GroupVertex.numOfGroup += 1;
					}
				}
			}
			if (GroupVertex.numOfGroup <= 2)
			{
				myVisual.vertex[i].z = myVisual.vertexMax_z + 2;
				originalVisual->GuassCurvature[i] = 0.0;
				if (filest)
				{
					//filest << originalVisual->GuassCurvature[i] << endl;
				}
				continue;
			}
			//计算高斯曲率
			double sumAngle = 0.0;
			double sumArea = 0.0;
			tVector *facevector = new tVector[GroupVertex.numOfGroup];
			//tVector *normalOfFace=(tVector *)malloc(myVisual.GroupVertex[i].numOfGroup*sizeof(tVector));

			//for (int m = 0; m < myVisual.GroupVertex[i].numOfGroup; i++)
			//{
			//	normalOfFace[m] = CountNormalofFace(myVisual.vertex[i], myVisual.GroupVertex[i].twoPoints[m].p1, myVisual.GroupVertex[i].twoPoints[m].p2);
			//	/*t_Triangle tempTriangle = CountTangle(myVisual.vertex[i], myVisual.GroupVertex[i].twoPoints[m].p1, myVisual.GroupVertex[i].twoPoints[m].p2);
			//	sumAngle += tempTriangle.angle;
			//	sumArea += tempTriangle.area;*/

			//}
			//计算高斯曲率参数 1、夹角和 2、voronoi面积
			for (int m = 0; m < GroupVertex.numOfGroup; m++)
			{
				t_Triangle tempTriangle = CountTangle(myVisual.vertex[i], GroupVertex.twoPoints[m].p1, GroupVertex.twoPoints[m].p2);
				sumAngle += tempTriangle.angle;
				sumArea += tempTriangle.area;
				facevector[m].x = tempTriangle.FaceVector.x;
				facevector[m].y = tempTriangle.FaceVector.y;
				facevector[m].z = tempTriangle.FaceVector.z;
			}
			//计算平均曲率
			double sumHcurvature = 0;//未除面积的平均曲率

			for (int m = 0; m < GroupVertex.numOfGroup; m++)
			{
				tVector edge1, edge2, edge3, edge4;
				//tVector normal1,normal2;
				double normalangle = 0.0, normalangle1 = 0.0, normalangle2 = 0.0;
				double edgelenth = 0.0;


				edge1.x = GroupVertex.twoPoints[m].p1.x - myVisual.vertex[i].x;
				edge1.y = GroupVertex.twoPoints[m].p1.y - myVisual.vertex[i].y;
				edge1.z = GroupVertex.twoPoints[m].p1.z - myVisual.vertex[i].z;
				edge2.x = GroupVertex.twoPoints[m].p2.x - myVisual.vertex[i].x;
				edge2.y = GroupVertex.twoPoints[m].p2.y - myVisual.vertex[i].y;
				edge2.z = GroupVertex.twoPoints[m].p2.z - myVisual.vertex[i].z;
				for (int mm = 0; mm < GroupVertex.numOfGroup; mm++)
				{
					if (mm != m)//忽略同一条边
					{
						//利用当前面和这个面的法向量n1、n2求出夹角，算出部分曲率H^
						edge3.x = GroupVertex.twoPoints[mm].p1.x - myVisual.vertex[i].x;
						edge3.y = GroupVertex.twoPoints[mm].p1.y - myVisual.vertex[i].y;
						edge3.z = GroupVertex.twoPoints[mm].p1.z - myVisual.vertex[i].z;
						edge4.x = GroupVertex.twoPoints[mm].p2.x - myVisual.vertex[i].x;
						edge4.y = GroupVertex.twoPoints[mm].p2.y - myVisual.vertex[i].y;
						edge4.z = GroupVertex.twoPoints[mm].p2.z - myVisual.vertex[i].z;
						//判断共边
						if (edge1.x == edge3.x&&edge1.y == edge3.y&&edge1.z == edge3.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角,Dyn的方法
							normalangle1 = CountTangle(edge2, edge2 - edge1);//Meyer的方法
							normalangle2 = CountTangle(edge4, edge4 - edge3);
							edgelenth = CountVecLenth(edge1);//计算共边的长度
															 //sumHcurvature += normalangle*edgelenth;//Dyn的方法
							sumHcurvature += 1 / (tan(normalangle1)*tan(normalangle2))*edgelenth;


						}
						else if (edge1.x == edge4.x&&edge1.y == edge4.y&&edge1.z == edge4.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
							normalangle1 = CountTangle(edge2, edge2 - edge1);//Meyer的方法
							normalangle2 = CountTangle(edge3, edge3 - edge4);
							edgelenth = CountVecLenth(edge1);//计算共边的长度
															 //sumHcurvature += normalangle*edgelenth;
							sumHcurvature += 1 / (tan(normalangle1)*tan(normalangle2))*edgelenth;


						}
						else if (edge2.x == edge4.x&&edge2.y == edge4.y&&edge2.z == edge4.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
							normalangle1 = CountTangle(edge1, edge1 - edge2);//Meyer的方法
							normalangle2 = CountTangle(edge3, edge3 - edge4);
							edgelenth = CountVecLenth(edge2);//计算共边的长度
															 //sumHcurvature += normalangle*edgelenth;
							sumHcurvature += 1 / (tan(normalangle1)*tan(normalangle2))*edgelenth;


						}
						else if (edge2.x == edge3.x&&edge2.y == edge3.y&&edge2.z == edge3.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
							normalangle1 = CountTangle(edge1, edge1 - edge2);//Meyer的方法
							normalangle2 = CountTangle(edge4, edge4 - edge3);
							edgelenth = CountVecLenth(edge2);//计算共边的长度
															 //sumHcurvature += normalangle*edgelenth;
							sumHcurvature += 1 / (tan(normalangle1)*tan(normalangle2))*edgelenth;


						}

					}

				}
			}

			delete[] facevector;
			if (sumArea > 0)
			{
				//originalVisual->HCurvature[i]= 0.25*sumHcurvature / sumArea;//Dyn的方法
				originalVisual->HCurvature[i] = 0.5*sumHcurvature / sumArea;
				originalVisual->GuassCurvature[i] = (2 * 3.1415926 - sumAngle) / sumArea;
				if (originalVisual->GuassCurvature[i]>0)
				{
					int tempGaussNum = 0;
					for (int m = 0; m < GroupVertex.numOfGroup; m++)
					{
						//求顶点邻边是否与法向量锐角
						double MultiVector1 = myVisual.normal[i].x*(GroupVertex.twoPoints[m].p1.x - myVisual.vertex[i].x)
							+ myVisual.normal[i].y*(GroupVertex.twoPoints[m].p1.y - myVisual.vertex[i].y)
							+ myVisual.normal[i].z*(GroupVertex.twoPoints[m].p1.z - myVisual.vertex[i].z);
						double MultiVector2 = myVisual.normal[i].x*(GroupVertex.twoPoints[m].p2.x - myVisual.vertex[i].x)
							+ myVisual.normal[i].y*(GroupVertex.twoPoints[m].p2.y - myVisual.vertex[i].y)
							+ myVisual.normal[i].z*(GroupVertex.twoPoints[m].p2.z - myVisual.vertex[i].z);
						if (MultiVector1 >= 0 && MultiVector2 >= 0)
						{
							tempGaussNum++;
						}

					}
					if (tempGaussNum >= GroupVertex.numOfGroup - 2)
					{

						originalVisual->GuassCurvature[i] = -originalVisual->GuassCurvature[i];
					}
				}
				else
				{
					originalVisual->GuassCurvature[i] = 0;//马鞍曲面
				}

			}
			else
			{
				originalVisual->HCurvature[i] = 0;
				originalVisual->GuassCurvature[i] = 1.0;
			}
			if (i == 1)
			{
				originalVisual->GuassMaxValue = originalVisual->GuassCurvature[i];
				originalVisual->GuassMinValue = originalVisual->GuassCurvature[i];
			}
			if (originalVisual->GuassCurvature[i]> originalVisual->GuassMaxValue)
			{
				originalVisual->GuassMaxValue = originalVisual->GuassCurvature[i];//取最大值
			}
			else if (originalVisual->GuassCurvature[i] < originalVisual->GuassMinValue)
			{
				originalVisual->GuassMinValue = originalVisual->GuassCurvature[i];//取最小值
			}
			if (filest)
			{
				//filest << originalVisual->GuassCurvature[i] <<endl;//<<"\t"<<sumAngle<< "\t" << sumArea<< endl;
				filest << originalVisual->HCurvature[i] * originalVisual->HCurvature[i] - originalVisual->GuassCurvature[i] << endl;
			}
			//free(normalOfFace);

		}
	}
	if (myVisual.GuassMaxValue<0)
	{
		myVisual.GuassMaxValue = 0;
	}
	else if (myVisual.GuassMinValue>0)
	{
		myVisual.GuassMinValue = 0;
	}

	/************************************************************************/
	/*                    count mean                                         */
	/************************************************************************/
	if (0 < myVisual.vertexCnt&&myVisual.vertexCnt - myVisual.faceCnt <= myVisual.faceCnt)
	{

		double  *meanGuass = new double[myVisual.vertexCnt];
		double  *meanH = new double[myVisual.vertexCnt];
		for (int i = 0; i >= 0 && i < myVisual.vertexCnt; i++)
		{
			std::set<int> neighberIndexs;

			if (myVisual.normalCnt > 0)
			{
				//循环相邻点
				for (int j = 0; j < myVisual.vfindex[i].num; j++)
				{

					tempfindex = myVisual.vfindex[i].findex[j];
					if (myVisual.index[tempfindex].v[0] == i)
					{
						neighberIndexs.insert(myVisual.index[tempfindex].v[1]);
						neighberIndexs.insert(myVisual.index[tempfindex].v[2]);

					}
					else if (myVisual.index[tempfindex].v[1] == i)
					{

						neighberIndexs.insert(myVisual.index[tempfindex].v[0]);
						neighberIndexs.insert(myVisual.index[tempfindex].v[2]);
					}
					else if (myVisual.index[tempfindex].v[2] == i)
					{

						neighberIndexs.insert(myVisual.index[tempfindex].v[0]);
						neighberIndexs.insert(myVisual.index[tempfindex].v[1]);
					}

				}
				//mean
				double sumG=0.0, sumH=0.0;
				for (std::set<int>::iterator it= neighberIndexs.begin();it!=neighberIndexs.end();it++)
				{
					sumG += originalVisual->GuassCurvature[*it];
					sumH += originalVisual->HCurvature[*it];
				}
				if (neighberIndexs.size()>0)
				{
					meanGuass[i] = sumG / double(neighberIndexs.size());
					meanH[i] = sumH / double(neighberIndexs.size());
				}
				else
				{
					meanGuass[i] = 0;
					meanH[i] =0;
				}

			}

		}
		
		for (int i = 0; i >= 0 && i < myVisual.vertexCnt; i++)
		{
			originalVisual->GuassCurvature[i] = meanGuass[i];
			originalVisual->HCurvature[i] = meanH[i];
		}
		delete[]meanGuass;
	}
	//SaveObjOption(originalVisual);
	//filest.close();
}
//
//void Registration::GuassCurvature(t_Visual *originalVisual) 
//{
//	
//	t_Visual myVisual = *originalVisual;
//	fstream filest;
//	filest.open("record001.txt", ios::app | ios::out);
//	//vertexsOfonevertex *GroupVertex =(vertexsOfonevertex *)malloc(originalVisual->vertexCnt * sizeof(vertexsOfonevertex));
//	//vertexOfFace *vertexGroup=(vertexOfFace *)malloc(10*sizeof(vertexOfFace));
//	/************************************************************************/
//	/* 
//	//根据点云源数据中顶点和面片数量的关系，
//	//当顶点远大于面片数量：说明需要简化点
//	//当顶点和面片数量：不需要简化
//	*/
//	/************************************************************************/
//	if (0<myVisual.vertexCnt&&myVisual.vertexCnt-myVisual.faceCnt<=myVisual.faceCnt)
//	{
//		if (myVisual.normalCnt<=0)
//		{
//		
//			myVisual.normal = (tVector *)malloc(myVisual.vertexCnt * sizeof(tVector));
//
//		}
//		for (int i= 0;i>=0&&i<myVisual.vertexCnt;i++)
//		{
//
//			vertexsOfonevertex GroupVertex;//定义面片顶点组
//			vertexOfFace Vertex4Normal[20];//定义面片顶点组(组内顶点按顺序排列)，以计算顶点法向量
//			GroupVertex.numOfGroup = 0;
//			
//		
//			if (myVisual.normalCnt>0)
//			{
//				//循环遍历查找与顶点相同的点
//				for (int j =0;j<myVisual.faceCnt;j++)
//				{
//					
//					
//					int temp = GroupVertex.numOfGroup;
//					
//					if (myVisual.index[j].v[0] == i)
//					{
//						myVisual.index[j].v[0] = i;
//						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[j].v[1]].x;
//						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[j].v[1]].y;
//						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[j].v[1]].z;
//						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[j].v[2]].x;
//						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[j].v[2]].y;
//						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[j].v[2]].z;
//						GroupVertex.numOfGroup += 1;
//
//					}else if (myVisual.index[j].v[1] == i)
//					{
//						myVisual.index[j].v[1] = i;
//						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[j].v[0]].x;
//						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[j].v[0]].y;
//						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[j].v[0]].z;
//						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[j].v[2]].x;
//						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[j].v[2]].y;
//						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[j].v[2]].z;
//						GroupVertex.numOfGroup += 1;
//					}else if (myVisual.index[j].v[2] == i)
//					{
//						myVisual.index[j].v[2] = i;
//						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[j].v[0]].x;
//						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[j].v[0]].y;
//						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[j].v[0]].z;
//						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[j].v[1]].x;
//						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[j].v[1]].y;
//						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[j].v[1]].z;
//						GroupVertex.numOfGroup += 1;
//					}
//					
//				}
//
//			}
//			else
//			{
//				
//				//循环遍历查找与顶点相同的点
//				for (int j =0;j<myVisual.faceCnt;j++)
//				{
//
//
//					int temp = GroupVertex.numOfGroup;
//
//					if (myVisual.index[j].v[0] == i)
//					{
//						myVisual.index[j].v[0] = i;
//						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[j].v[1]].x;
//						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[j].v[1]].y;
//						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[j].v[1]].z;
//
//						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[j].v[2]].x;
//						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[j].v[2]].y;
//						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[j].v[2]].z;
//
//						Vertex4Normal[temp].p1.x=myVisual.vertex[myVisual.index[j].v[0]].x;
//						Vertex4Normal[temp].p1.y=myVisual.vertex[myVisual.index[j].v[0]].y;
//						Vertex4Normal[temp].p1.z=myVisual.vertex[myVisual.index[j].v[0]].z;
//
//						Vertex4Normal[temp].p2.x=myVisual.vertex[myVisual.index[j].v[1]].x;
//						Vertex4Normal[temp].p2.y=myVisual.vertex[myVisual.index[j].v[1]].y;
//						Vertex4Normal[temp].p2.z=myVisual.vertex[myVisual.index[j].v[1]].z;
//
//						Vertex4Normal[temp].p3.x=myVisual.vertex[myVisual.index[j].v[2]].x;
//						Vertex4Normal[temp].p3.y=myVisual.vertex[myVisual.index[j].v[2]].y;
//						Vertex4Normal[temp].p3.z=myVisual.vertex[myVisual.index[j].v[2]].z;
//
//						GroupVertex.numOfGroup += 1;
//
//					}else if (myVisual.index[j].v[1] == i)
//					{
//						myVisual.index[j].v[1] = i;
//						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[j].v[0]].x;
//						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[j].v[0]].y;
//						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[j].v[0]].z;
//						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[j].v[2]].x;
//						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[j].v[2]].y;
//						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[j].v[2]].z;
//
//
//						Vertex4Normal[temp].p1.x=myVisual.vertex[myVisual.index[j].v[0]].x;
//						Vertex4Normal[temp].p1.y=myVisual.vertex[myVisual.index[j].v[0]].y;
//						Vertex4Normal[temp].p1.z=myVisual.vertex[myVisual.index[j].v[0]].z;
//
//						Vertex4Normal[temp].p2.x=myVisual.vertex[myVisual.index[j].v[1]].x;
//						Vertex4Normal[temp].p2.y=myVisual.vertex[myVisual.index[j].v[1]].y;
//						Vertex4Normal[temp].p2.z=myVisual.vertex[myVisual.index[j].v[1]].z;
//
//						Vertex4Normal[temp].p3.x=myVisual.vertex[myVisual.index[j].v[2]].x;
//						Vertex4Normal[temp].p3.y=myVisual.vertex[myVisual.index[j].v[2]].y;
//						Vertex4Normal[temp].p3.z=myVisual.vertex[myVisual.index[j].v[2]].z;
//
//						GroupVertex.numOfGroup += 1;
//					}else if (myVisual.index[j].v[2] == i)
//					{
//						myVisual.index[j].v[2] = i;
//						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[j].v[0]].x;
//						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[j].v[0]].y;
//						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[j].v[0]].z;
//						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[j].v[1]].x;
//						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[j].v[1]].y;
//						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[j].v[1]].z;
//
//
//						Vertex4Normal[temp].p1.x=myVisual.vertex[myVisual.index[j].v[0]].x;
//						Vertex4Normal[temp].p1.y=myVisual.vertex[myVisual.index[j].v[0]].y;
//						Vertex4Normal[temp].p1.z=myVisual.vertex[myVisual.index[j].v[0]].z;
//
//						Vertex4Normal[temp].p2.x=myVisual.vertex[myVisual.index[j].v[1]].x;
//						Vertex4Normal[temp].p2.y=myVisual.vertex[myVisual.index[j].v[1]].y;
//						Vertex4Normal[temp].p2.z=myVisual.vertex[myVisual.index[j].v[1]].z;
//
//						Vertex4Normal[temp].p3.x=myVisual.vertex[myVisual.index[j].v[2]].x;
//						Vertex4Normal[temp].p3.y=myVisual.vertex[myVisual.index[j].v[2]].y;
//						Vertex4Normal[temp].p3.z=myVisual.vertex[myVisual.index[j].v[2]].z;
//
//						GroupVertex.numOfGroup += 1;
//					}
//
//				}
//
//			}
//			if (GroupVertex.numOfGroup<=2)
//			{
//				myVisual.vertex[i].z = myVisual.vertexMax_z + 2;
//				originalVisual->GuassCurvature[i] = 0.0;
//				continue;
//			}
//			
//			//计算高斯曲率
//			double sumAngle = 0.0;
//			double sumArea = 0.0;
//			tVector *facevector=new tVector[GroupVertex.numOfGroup];
//
//			//判断是否需要计算顶点法向量
//			if (myVisual.normalCnt<=0)
//			{
//				myVisual.normal[i].x=0;
//				myVisual.normal[i].y=0;
//				myVisual.normal[i].z=0;
//				for (int m = 0; m < GroupVertex.numOfGroup; m++)
//				{
//					tVector tempnormal;
//					tempnormal=CountNormalofFace(Vertex4Normal[m].p1,Vertex4Normal[m].p2,Vertex4Normal[m].p3);
//			
//					myVisual.normal[i].x+=tempnormal.x;
//					myVisual.normal[i].y+=tempnormal.y;
//					myVisual.normal[i].z+=tempnormal.z;
//					
//
//				}
//				//originalVisual->normal[i] = myVisual.normal[i];
//			}
//			//tVector *normalOfFace=(tVector *)malloc(myVisual.GroupVertex[i].numOfGroup*sizeof(tVector));
//			for (int m = 0; m < GroupVertex.numOfGroup; m++)
//			{
//				t_Triangle tempTriangle = CountTangle(myVisual.vertex[i], GroupVertex.twoPoints[m].p1, GroupVertex.twoPoints[m].p2);
//				sumAngle += tempTriangle.angle;
//				sumArea += tempTriangle.area;
//				facevector[m].x = tempTriangle.FaceVector.x;
//				facevector[m].y = tempTriangle.FaceVector.y;
//				facevector[m].z = tempTriangle.FaceVector.z;
//			}
//			//计算平均曲率
//			double sumHcurvature = 0;//未除面积的平均曲率
//			tVector sumHVector;
//			MAKEVECTOR(sumHVector, 0, 0, 0);
//			for (int m = 0; m < GroupVertex.numOfGroup; m++)
//			{
//				tVector edge1, edge2,edge3,edge4;
//				//tVector normal1,normal2;
//				double normalangle = 0.0, normalangle1 = 0.0, normalangle2 = 0.0;
//				double edgelenth = 0.0;
//				double tempH = 0;
//
//				edge1.x = GroupVertex.twoPoints[m].p1.x - myVisual.vertex[i].x;
//				edge1.y = GroupVertex.twoPoints[m].p1.y - myVisual.vertex[i].y;
//				edge1.z = GroupVertex.twoPoints[m].p1.z - myVisual.vertex[i].z;
//				edge2.x = GroupVertex.twoPoints[m].p2.x - myVisual.vertex[i].x;
//				edge2.y = GroupVertex.twoPoints[m].p2.y - myVisual.vertex[i].y;
//				edge2.z = GroupVertex.twoPoints[m].p2.z - myVisual.vertex[i].z;
//				for (int mm = 0; mm < GroupVertex.numOfGroup; mm++)
//				{
//					if (mm!=m)//忽略同一条边
//					{
//						//利用当前面和这个面的法向量n1、n2求出夹角，算出部分曲率H^
//						edge3.x = GroupVertex.twoPoints[mm].p1.x - myVisual.vertex[i].x;
//						edge3.y = GroupVertex.twoPoints[mm].p1.y - myVisual.vertex[i].y;
//						edge3.z = GroupVertex.twoPoints[mm].p1.z - myVisual.vertex[i].z;
//						edge4.x = GroupVertex.twoPoints[mm].p2.x - myVisual.vertex[i].x;
//						edge4.y = GroupVertex.twoPoints[mm].p2.y - myVisual.vertex[i].y;
//						edge4.z = GroupVertex.twoPoints[mm].p2.z - myVisual.vertex[i].z;
//						//判断共边
//						if (edge1.x == edge3.x&&edge1.y == edge3.y&&edge1.z == edge3.z)
//						{
//							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角,Dyn的方法
//							normalangle1 = CountTangle(edge2, edge2 - edge1);//Meyer的方法
//							normalangle2 = CountTangle(edge4, edge4 - edge3);
//							//edgelenth = CountVecLenth(edge1);//计算共边的长度
//															 //sumHcurvature += normalangle*edgelenth;//Dyn的方法
//							tempH = 1 / tan(normalangle1) + 1 / tan(normalangle2);
//							MAKEVECTOR(sumHVector, sumHVector.x + tempH*edge1.x,
//								sumHVector.y + tempH*edge1.y,
//								sumHVector.z + tempH*edge1.z);
//
//
//						}
//						else if (edge1.x == edge4.x&&edge1.y == edge4.y&&edge1.z == edge4.z)
//						{
//							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//							normalangle1 = CountTangle(edge2, edge2 - edge1);//Meyer的方法
//							normalangle2 = CountTangle(edge3, edge3 - edge4);
//							//edgelenth = CountVecLenth(edge1);//计算共边的长度
//															 //sumHcurvature += normalangle*edgelenth;
//							tempH = 1 / tan(normalangle1) + 1 / tan(normalangle2);
//							MAKEVECTOR(sumHVector, sumHVector.x + tempH*edge1.x,
//								sumHVector.y + tempH*edge1.y,
//								sumHVector.z + tempH*edge1.z);
//
//
//						}
//						else if (edge2.x == edge4.x&&edge2.y == edge4.y&&edge2.z == edge4.z)
//						{
//							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//							normalangle1 = CountTangle(edge1, edge1 - edge2);//Meyer的方法
//							normalangle2 = CountTangle(edge3, edge3 - edge4);
//							//edgelenth = CountVecLenth(edge2);//计算共边的长度
//															 //sumHcurvature += normalangle*edgelenth;
//							tempH = 1 / tan(normalangle1) + 1 / tan(normalangle2);
//							MAKEVECTOR(sumHVector, sumHVector.x + tempH*edge2.x,
//								sumHVector.y + tempH*edge2.y,
//								sumHVector.z + tempH*edge2.z);
//
//
//						}
//						else if (edge2.x == edge3.x&&edge2.y == edge3.y&&edge2.z == edge3.z)
//						{
//							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//							normalangle1 = CountTangle(edge1, edge1 - edge2);//Meyer的方法
//							normalangle2 = CountTangle(edge4, edge4 - edge3);
//							//edgelenth = CountVecLenth(edge2);//计算共边的长度
//															 //sumHcurvature += normalangle*edgelenth;
//							tempH = 1 / tan(normalangle1) + 1 / tan(normalangle2);
//							MAKEVECTOR(sumHVector, sumHVector.x + tempH*edge2.x,
//								sumHVector.y + tempH*edge2.y,
//								sumHVector.z + tempH*edge2.z);
//
//
//						}
//					}
//
//				}
//			}
//
//
//			delete[] facevector;
//			if (sumArea > 0)
//			{
//				Eigen::Vector3d n1, n2;
//				n1 << sumHVector.x, sumHVector.y, sumHVector.z;
//				n2 << myVisual.normal[i].x, myVisual.normal[i].y, myVisual.normal[i].z;
//				originalVisual->HCurvature[i] = 0.25*(n1.dot(n2)) / sumArea;
//				originalVisual->GuassCurvature[i] = (2 * 3.1415926 - sumAngle) / sumArea;
//				if (originalVisual->GuassCurvature[i]>0)
//				{
//					int tempGaussNum = 0;
//					for (int m = 0; m < GroupVertex.numOfGroup; m++)
//					{
//						//求顶点邻边是否与法向量锐角
//						double MultiVector1 = myVisual.normal[i].x*(GroupVertex.twoPoints[m].p1.x - myVisual.vertex[i].x)
//							+ myVisual.normal[i].y*(GroupVertex.twoPoints[m].p1.y- myVisual.vertex[i].y)
//							+ myVisual.normal[i].z*(GroupVertex.twoPoints[m].p1.z - myVisual.vertex[i].z);
//						double MultiVector2 = myVisual.normal[i].x*(GroupVertex.twoPoints[m].p2.x - myVisual.vertex[i].x)
//							+ myVisual.normal[i].y*(GroupVertex.twoPoints[m].p2.y - myVisual.vertex[i].y)
//							+ myVisual.normal[i].z*(GroupVertex.twoPoints[m].p2.z - myVisual.vertex[i].z);
//						if (MultiVector1>=0&& MultiVector2>=0)
//						{
//							tempGaussNum++;
//						}
//						
//					}
//					if (tempGaussNum >= GroupVertex.numOfGroup)
//					{
//
//						originalVisual->GuassCurvature[i] = -originalVisual->GuassCurvature[i];
//					}
//				}
//				else
//				{
//					originalVisual->GuassCurvature[i] = 0;//马鞍曲面
//				}
//
//			}
//			else 
//			{
//				originalVisual->HCurvature[i] = 0;
//				originalVisual->GuassCurvature[i] = 1.0;
//			}
//			if (i==1)
//			{
//				originalVisual->GuassMaxValue = originalVisual->GuassCurvature[i];
//				originalVisual->GuassMinValue = originalVisual->GuassCurvature[i];
//			}
//			if (originalVisual->GuassCurvature[i]> originalVisual->GuassMaxValue)
//			{
//				originalVisual->GuassMaxValue = originalVisual->GuassCurvature[i];//取最大值
//			}
//			else if (originalVisual->GuassCurvature[i] < originalVisual->GuassMinValue)
//			{
//				originalVisual->GuassMinValue = originalVisual->GuassCurvature[i];//取最小值
//			}
//			if (filest)
//			{
//				filest << originalVisual->GuassCurvature[i] <<endl; //<<"\t"<<sumAngle<< "\t" << sumArea<< endl;
//				//filest << originalVisual->HCurvature[i] * originalVisual->HCurvature[i] - originalVisual->GuassCurvature[i] << endl;
//			}
//			
//		}
//	}
//	if(0<myVisual.vertexCnt&&myVisual.vertexCnt-myVisual.faceCnt>myVisual.faceCnt)
//	{
//		//循环遍历顶点
//		
//		for (int i=0;i<myVisual.vertexCnt;i++)
//		{
//
//			if (myVisual.vertex[i].x == myVisual.vertexMax_x + 10)//跳过已标记顶点 
//			{
//				originalVisual->GuassCurvature[i] = 0.0;
//				if (filest)
//				{
//					//filest << originalVisual->GuassCurvature[i] << endl;
//				}
//				continue;
//			}
//			vertexsOfonevertex GroupVertex;//定义面片顶点组
//			GroupVertex.numOfGroup = 0;
//			//循环遍历查找与顶点重复的点
//			for (int j =0;j<myVisual.vertexCnt-i-1;j++)
//			{
//				//判断下一个点是否已经标记，若标记则进行下次循环
//				if (GroupVertex.numOfGroup>8)
//				{
//					break;
//				}
//				int sumji = j + i;//当前点的位置的j个位移
//				if (myVisual.vertex[sumji].x == myVisual.vertexMax_x + 10)//跳过标记顶点
//					continue;
//
//				//从当前顶点位置开始查找判断，以便将当前顶点收入组中
//				if (myVisual.vertex[sumji].x == myVisual.vertex[i].x&&
//					myVisual.vertex[sumji].y == myVisual.vertex[i].y&&
//					myVisual.vertex[sumji].z == myVisual.vertex[i].z)
//				{
//
//					//将重复点用当前点云的坐标最大值标记
//					if (GroupVertex.numOfGroup>0)
//					{
//						myVisual.vertex[sumji].x = myVisual.vertexMax_x+10;
//					}
//					//遍历面片，将面片的顶点索引修改为所有重复顶点的第一个顶点的索引,将同一顶点面片的其他点信息收入组中
//
//					int temp = GroupVertex.numOfGroup;
//					int tempvsite = (sumji) / 3;
//					int tempvsiteyu = (sumji) % 3;
//					if (myVisual.index[tempvsite].v[0] == sumji)
//					{
//						myVisual.index[tempvsite].v[0] = i;
//						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempvsite].v[1]].x;
//						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempvsite].v[1]].y;
//						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempvsite].v[1]].z;
//						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempvsite].v[2]].x;
//						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempvsite].v[2]].y;
//						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempvsite].v[2]].z;
//						GroupVertex.numOfGroup += 1;
//
//					}else if (myVisual.index[tempvsite].v[1] == sumji)
//					{
//						myVisual.index[tempvsite].v[1] = i;
//						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempvsite].v[0]].x;
//						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempvsite].v[0]].y;
//						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempvsite].v[0]].z;
//						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempvsite].v[2]].x;
//						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempvsite].v[2]].y;
//						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempvsite].v[2]].z;
//						GroupVertex.numOfGroup += 1;
//					}else if (myVisual.index[tempvsite].v[2] == sumji)
//					{
//						myVisual.index[tempvsite].v[2] = i;
//						GroupVertex.twoPoints[temp].p1.x = myVisual.vertex[myVisual.index[tempvsite].v[0]].x;
//						GroupVertex.twoPoints[temp].p1.y = myVisual.vertex[myVisual.index[tempvsite].v[0]].y;
//						GroupVertex.twoPoints[temp].p1.z = myVisual.vertex[myVisual.index[tempvsite].v[0]].z;
//						GroupVertex.twoPoints[temp].p2.x = myVisual.vertex[myVisual.index[tempvsite].v[1]].x;
//						GroupVertex.twoPoints[temp].p2.y = myVisual.vertex[myVisual.index[tempvsite].v[1]].y;
//						GroupVertex.twoPoints[temp].p2.z = myVisual.vertex[myVisual.index[tempvsite].v[1]].z;
//						GroupVertex.numOfGroup += 1;
//					}
//				}
//			}
//			if (GroupVertex.numOfGroup<=2)
//			{
//				myVisual.vertex[i].z = myVisual.vertexMax_z + 2;
//				originalVisual->GuassCurvature[i] = 0.0;
//				if (filest)
//				{
//					//filest << originalVisual->GuassCurvature[i] << endl;
//				}
//				continue;
//			}
//			//计算高斯曲率
//			double sumAngle = 0.0;
//			double sumArea = 0.0;
//			tVector *facevector=new tVector[GroupVertex.numOfGroup];
//			//tVector *normalOfFace=(tVector *)malloc(myVisual.GroupVertex[i].numOfGroup*sizeof(tVector));
//
//			//for (int m = 0; m < myVisual.GroupVertex[i].numOfGroup; i++)
//			//{
//			//	normalOfFace[m] = CountNormalofFace(myVisual.vertex[i], myVisual.GroupVertex[i].twoPoints[m].p1, myVisual.GroupVertex[i].twoPoints[m].p2);
//			//	/*t_Triangle tempTriangle = CountTangle(myVisual.vertex[i], myVisual.GroupVertex[i].twoPoints[m].p1, myVisual.GroupVertex[i].twoPoints[m].p2);
//			//	sumAngle += tempTriangle.angle;
//			//	sumArea += tempTriangle.area;*/
//
//			//}
//			//计算高斯曲率参数 1、夹角和 2、voronoi面积
//			for (int m = 0; m < GroupVertex.numOfGroup; m++)
//			{
//				t_Triangle tempTriangle = CountTangle(myVisual.vertex[i], GroupVertex.twoPoints[m].p1, GroupVertex.twoPoints[m].p2);
//				sumAngle += tempTriangle.angle;
//				sumArea += tempTriangle.area;
//				facevector[m].x = tempTriangle.FaceVector.x;
//				facevector[m].y = tempTriangle.FaceVector.y;
//				facevector[m].z = tempTriangle.FaceVector.z;
//			}
//			//计算平均曲率
//			double sumHcurvature = 0;//未除面积的平均曲率
//
//			for (int m = 0; m < GroupVertex.numOfGroup; m++)
//			{
//				tVector edge1, edge2, edge3, edge4;
//				//tVector normal1,normal2;
//				double normalangle = 0.0, normalangle1 = 0.0, normalangle2 = 0.0;
//				double edgelenth = 0.0;
//
//
//				edge1.x = GroupVertex.twoPoints[m].p1.x - myVisual.vertex[i].x;
//				edge1.y = GroupVertex.twoPoints[m].p1.y - myVisual.vertex[i].y;
//				edge1.z = GroupVertex.twoPoints[m].p1.z - myVisual.vertex[i].z;
//				edge2.x = GroupVertex.twoPoints[m].p2.x - myVisual.vertex[i].x;
//				edge2.y = GroupVertex.twoPoints[m].p2.y - myVisual.vertex[i].y;
//				edge2.z = GroupVertex.twoPoints[m].p2.z - myVisual.vertex[i].z;
//				for (int mm = 0; mm < GroupVertex.numOfGroup; mm++)
//				{
//					if (mm != m)//忽略同一条边
//					{
//						//利用当前面和这个面的法向量n1、n2求出夹角，算出部分曲率H^
//						edge3.x = GroupVertex.twoPoints[mm].p1.x - myVisual.vertex[i].x;
//						edge3.y = GroupVertex.twoPoints[mm].p1.y - myVisual.vertex[i].y;
//						edge3.z = GroupVertex.twoPoints[mm].p1.z - myVisual.vertex[i].z;
//						edge4.x = GroupVertex.twoPoints[mm].p2.x - myVisual.vertex[i].x;
//						edge4.y = GroupVertex.twoPoints[mm].p2.y - myVisual.vertex[i].y;
//						edge4.z = GroupVertex.twoPoints[mm].p2.z - myVisual.vertex[i].z;
//						//判断共边
//						if (edge1.x == edge3.x&&edge1.y == edge3.y&&edge1.z == edge3.z)
//						{
//							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角,Dyn的方法
//							normalangle1 = CountTangle(edge2, edge2 - edge1);//Meyer的方法
//							normalangle2 = CountTangle(edge4, edge4 - edge3);
//							edgelenth = CountVecLenth(edge1);//计算共边的长度
//							//sumHcurvature += normalangle*edgelenth;//Dyn的方法
//							sumHcurvature += 1 / (tan(normalangle1)*tan(normalangle2))*edgelenth;
//
//
//						}
//						else if (edge1.x == edge4.x&&edge1.y == edge4.y&&edge1.z == edge4.z)
//						{
//							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//							normalangle1 = CountTangle(edge2, edge2 - edge1);//Meyer的方法
//							normalangle2 = CountTangle(edge3, edge3 - edge4);
//							edgelenth = CountVecLenth(edge1);//计算共边的长度
//							//sumHcurvature += normalangle*edgelenth;
//							sumHcurvature += 1 / (tan(normalangle1)*tan(normalangle2))*edgelenth;
//
//
//						}
//						else if (edge2.x == edge4.x&&edge2.y == edge4.y&&edge2.z == edge4.z)
//						{
//							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//							normalangle1 = CountTangle(edge1, edge1 - edge2);//Meyer的方法
//							normalangle2 = CountTangle(edge3, edge3 - edge4);
//							edgelenth = CountVecLenth(edge2);//计算共边的长度
//							//sumHcurvature += normalangle*edgelenth;
//							sumHcurvature += 1 / (tan(normalangle1)*tan(normalangle2))*edgelenth;
//
//
//						}
//						else if (edge2.x == edge3.x&&edge2.y == edge3.y&&edge2.z == edge3.z)
//						{
//							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//							normalangle1 = CountTangle(edge1, edge1 - edge2);//Meyer的方法
//							normalangle2 = CountTangle(edge4, edge4 - edge3);
//							edgelenth = CountVecLenth(edge2);//计算共边的长度
//							//sumHcurvature += normalangle*edgelenth;
//							sumHcurvature += 1 / (tan(normalangle1)*tan(normalangle2))*edgelenth;
//
//
//						}
//
//					}
//
//				}
//			}
//
//			delete[] facevector;
//			if (sumArea > 0)
//			{
//				//originalVisual->HCurvature[i]= 0.25*sumHcurvature / sumArea;//Dyn的方法
//				originalVisual->HCurvature[i] = 0.5*sumHcurvature / sumArea;
//				originalVisual->GuassCurvature[i] = (2 * 3.1415926 - sumAngle) / sumArea;
//				if (originalVisual->GuassCurvature[i]>0)
//				{
//					int tempGaussNum = 0;
//					for (int m = 0; m < GroupVertex.numOfGroup; m++)
//					{
//						//求顶点邻边是否与法向量锐角
//						double MultiVector1 = myVisual.normal[i].x*(GroupVertex.twoPoints[m].p1.x - myVisual.vertex[i].x)
//							+ myVisual.normal[i].y*(GroupVertex.twoPoints[m].p1.y- myVisual.vertex[i].y)
//							+ myVisual.normal[i].z*(GroupVertex.twoPoints[m].p1.z - myVisual.vertex[i].z);
//						double MultiVector2 = myVisual.normal[i].x*(GroupVertex.twoPoints[m].p2.x - myVisual.vertex[i].x)
//							+ myVisual.normal[i].y*(GroupVertex.twoPoints[m].p2.y - myVisual.vertex[i].y)
//							+ myVisual.normal[i].z*(GroupVertex.twoPoints[m].p2.z - myVisual.vertex[i].z);
//						if (MultiVector1>=0&& MultiVector2>=0)
//						{
//							tempGaussNum++;
//						}
//						
//					}
//					if (tempGaussNum >= GroupVertex.numOfGroup-2)
//					{
//
//						originalVisual->GuassCurvature[i] = -originalVisual->GuassCurvature[i];
//					}
//				}
//				else
//				{
//					originalVisual->GuassCurvature[i] = 0;//马鞍曲面
//				}
//
//			}
//			else
//			{
//				originalVisual->HCurvature[i] = 0;
//				originalVisual->GuassCurvature[i] = 1.0;
//			}
//			if (i==1)
//			{
//				originalVisual->GuassMaxValue = originalVisual->GuassCurvature[i];
//				originalVisual->GuassMinValue = originalVisual->GuassCurvature[i];
//			}
//			if (originalVisual->GuassCurvature[i]> originalVisual->GuassMaxValue)
//			{
//				originalVisual->GuassMaxValue = originalVisual->GuassCurvature[i];//取最大值
//			}
//			else if (originalVisual->GuassCurvature[i] < originalVisual->GuassMinValue)
//			{
//				originalVisual->GuassMinValue = originalVisual->GuassCurvature[i];//取最小值
//			}
//			if (filest)
//			{
//				//filest << originalVisual->GuassCurvature[i] <<endl;//<<"\t"<<sumAngle<< "\t" << sumArea<< endl;
//				filest << originalVisual->HCurvature[i]* originalVisual->HCurvature[i] - originalVisual->GuassCurvature[i] << endl;
//			}
//			//free(normalOfFace);
//			
//		}
//	}
//	if (myVisual.GuassMaxValue<0)
//	{
//		myVisual.GuassMaxValue = 0;
//	}else if (myVisual.GuassMinValue>0)
//	{
//		myVisual.GuassMinValue = 0;
//	}
//	//filest.close();
//}
/************************************************************************/
/* 计算顶点邻角角度和邻面voronoi面积   除数为零的情况待改进……                                                                  */
/************************************************************************/
t_Triangle Registration::CountTangle(tVector p1, tVector p2, tVector p3)
{

	t_Triangle triangleTemp;
	//vectorC, vectorB, vectorA是三角形顶点C、B、A对应的边向量
	tVector vectorC, vectorB, vectorA, vectorCA, vectorBA, vectorCB;
	vectorC.x = p2.x - p1.x; vectorC.y = p2.y - p1.y; vectorC.z = p2.z - p1.z;
	vectorB.x = p3.x - p1.x; vectorB.y = p3.y - p1.y; vectorB.z = p3.z - p1.z;
	vectorA.x = p2.x - p3.x; vectorA.y = p2.y - p3.y; vectorA.z = p2.z - p3.z;
	//vectorCA,vectorBA,vectorCB是向量叉乘
	vectorCA.x = vectorC.y*vectorA.z - vectorC.z*vectorA.y;
	vectorCA.y = vectorC.z*vectorA.x - vectorC.x*vectorA.z;
	vectorCA.z = vectorC.x*vectorA.y - vectorC.y*vectorA.x;

	vectorBA.x = vectorB.z*vectorA.y - vectorB.y*vectorA.z;
	vectorBA.y = vectorB.x*vectorA.z - vectorB.z*vectorA.x;
	vectorBA.z = vectorB.y*vectorA.x - vectorB.x*vectorA.y;

	vectorCB.x = vectorC.y*vectorB.z - vectorC.z*vectorB.y;
	vectorCB.y = vectorC.z*vectorB.x - vectorC.x*vectorB.z;
	vectorCB.z = vectorC.x*vectorB.y - vectorC.y*vectorB.x;
	//FaceVector是面的法向量
	triangleTemp.FaceVector.x = vectorCB.x;
	triangleTemp.FaceVector.y = vectorCB.y;
	triangleTemp.FaceVector.z = vectorCB.z;
	//点积
	double normalMultiplyBC = vectorC.x*vectorB.x + vectorC.y*vectorB.y + vectorC.z *vectorB.z;
	double normalMultiplyCA = vectorC.x*vectorA.x + vectorC.y*vectorA.y + vectorC.z *vectorA.z;
	double normalMultiplyBA = -vectorA.x*vectorB.x - vectorA.y*vectorB.y - vectorA.z *vectorB.z;
	//叉积长度  例如： |vectorC × vectorA|
	double vectorCALength = sqrt(vectorCA.x*vectorCA.x + vectorCA.y*vectorCA.y + vectorCA.z*vectorCA.z);
	double vectorBALength = sqrt(vectorBA.x*vectorBA.x + vectorBA.y*vectorBA.y + vectorBA.z*vectorBA.z);
	double vectorCBLength = sqrt(vectorCB.x*vectorCB.x + vectorCB.y*vectorCB.y + vectorCB.z*vectorCB.z);

	double normalLengthMultply1 = vectorC.x*vectorC.x + vectorC.y*vectorC.y + vectorC.z*vectorC.z;
	double normalLengthMultply2 = vectorB.x*vectorB.x + vectorB.y*vectorB.y + vectorB.z*vectorB.z;
	double normalLengthMultply3 = vectorA.x*vectorA.x + vectorA.y*vectorA.y + vectorA.z*vectorA.z;

	double averagelength = (sqrt(normalLengthMultply1) + sqrt(normalLengthMultply2) + sqrt(normalLengthMultply3)) / 2;
	triangleTemp.angle = acos(normalMultiplyBC / (sqrt(normalLengthMultply1) *sqrt(normalLengthMultply2)));// *180 / 3.1415926;
																										   //triangleTemp.area = 0.125*(normalLengthMultply2*normalMultiplyCA/ vectorCALength +normalLengthMultply1*normalMultiplyBA / vectorBALength);
	double angle1 = acos(normalMultiplyCA / (sqrt(normalLengthMultply1)*sqrt(normalLengthMultply3)));
	double angle2 = acos(normalMultiplyBA / (sqrt(normalLengthMultply2)*sqrt(normalLengthMultply3)));
	if (normalMultiplyBC > 0)
	{
		if (normalMultiplyCA < 0)
		{
			triangleTemp.area = 0.125*(normalLengthMultply1*vectorCBLength / normalMultiplyBC);
		}
		else if (normalMultiplyBA < 0)
		{
			triangleTemp.area = 0.125*(normalLengthMultply2*vectorCBLength / normalMultiplyBC);
		}
		else
		{
			triangleTemp.area = 0.125*(normalLengthMultply2*normalMultiplyCA / vectorCALength + normalLengthMultply1*normalMultiplyBA / vectorBALength);
		}
	}
	else
	{
		double Sall = sqrt(averagelength*(averagelength - sqrt(normalLengthMultply1))*(averagelength - sqrt(normalLengthMultply2))*(averagelength - sqrt(normalLengthMultply3)));
		double Sb = 0.125*(normalLengthMultply1*vectorCALength / normalMultiplyCA);
		double Sc = 0.125*(normalLengthMultply2*vectorBALength / normalMultiplyBA);
		triangleTemp.area = Sall - Sb - Sc;
	}


	return triangleTemp;
}

/************************************************************************/
/* 计算纹理特征点对                                                                     */
/************************************************************************/
int Registration::texttureKeyPoint(string targetPath,string sPath)
{
	Mat image01 = imread(targetPath, 1);
	Mat image02 = imread(sPath, 1);

	if (!image01.data || !image02.data)
	{
		AfxMessageBox("读取图片出错");
		return 0;
	}
	//namedWindow("匹配效果", WINDOW_NORMAL);
	//namedWindow("原始图1", WINDOW_NORMAL);
	//namedWindow("ransac", WINDOW_NORMAL);
	/*namedWindow("原始图2", WINDOW_NORMAL);
	namedWindow("效果图1", WINDOW_NORMAL);
	namedWindow("效果图2", WINDOW_NORMAL);
	imshow("原始图1", srcImage1);
	imshow("原始图2", srcImage2);*/

	//int minHessian = 1000;//应该是hessian响应值
	//Ptr<SurfFeatureDetector> detector = SurfFeatureDetector::create(minHessian);

	//vector<cv::KeyPoint> key_points_1, key_points_2;

	//Mat dstImage1, dstImage2;
	//detector->detectAndCompute(srcImage1, Mat(), key_points_1, dstImage1);
	//detector->detectAndCompute(srcImage2, Mat(), key_points_2, dstImage2);//可以分成detect和compute

	//Mat img_keypoints_1, img_keypoints_2;
	//drawKeypoints(srcImage1, key_points_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	//drawKeypoints(srcImage2, key_points_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

	//Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
	//vector<DMatch>mach;

	//matcher->match(dstImage1, dstImage2, mach);
	//double Max_dist = 0;
	//double Min_dist = 1000;
	//for (int i = 0; i < dstImage1.rows; i++)
	//{
	//	double dist = mach[i].distance;
	//	if (dist < Min_dist)Min_dist = dist;
	//	if (dist > Max_dist)Max_dist = dist;
	//}

	//vector<DMatch>goodmaches;
	////暂时改一下
	//for (int i = 0; i < dstImage1.rows; i+=100)
	//{
	//	if (mach[i].distance <= 2 * Min_dist)
	//		goodmaches.push_back(mach[i]);
	//}
	//Mat img_maches;
	//drawMatches(srcImage1, key_points_1, srcImage2, key_points_2, goodmaches, img_maches);
	int minHessian = 500;//300;//应该是hessian响应值
						 //imshow("p2", image01);
						 //imshow("p1", image02);

						 //灰度图转换  
	Mat image1, image2;
	cvtColor(image01, image1, CV_RGB2GRAY);
	cvtColor(image02, image2, CV_RGB2GRAY);


	//提取特征点    
	Ptr<SurfFeatureDetector> detector = SurfFeatureDetector::create(minHessian);

	vector<cv::KeyPoint> keyPoint1, keyPoint2;

	Mat imageDesc1, imageDesc2;
	detector->detectAndCompute(image01, Mat(), keyPoint1, imageDesc1);
	detector->detectAndCompute(image02, Mat(), keyPoint2, imageDesc2);//可以分成detect和compute

	Mat img_keypoints_1, img_keypoints_2;
	drawKeypoints(image01, keyPoint1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	drawKeypoints(image02, keyPoint2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

	FlannBasedMatcher matcher;
	vector<vector<DMatch> > matchePoints;
	vector<DMatch> GoodMatchePoints;

	vector<Mat> train_desc(1, imageDesc1);//此处训练1到2的匹配
	matcher.add(train_desc);
	matcher.train();

	matcher.knnMatch(imageDesc2, matchePoints, 2);
	cout << "total match points: " << matchePoints.size() << endl;

	// Lowe's algorithm,获取优秀匹配点
	for (int i = 0; i < matchePoints.size(); i++)
	{
		if (matchePoints[i][0].distance < 0.6 * matchePoints[i][1].distance)
		{
			GoodMatchePoints.push_back(matchePoints[i][0]);
		}
	}

	Mat first_match;
	//drawMatches(image02, keyPoint2, image01, keyPoint1, GoodMatchePoints, first_match);
	if (GoodMatchePoints.size()>0)
	{
		//tmachePoint = (TmachePoints *)malloc(GoodMatchePoints.size() * sizeof(TmachePoints));//为纹理匹配点对分配存储空间
		cv::Point2f pointInterest;
		//为点对数组赋值//归 1 化处理
		for (int j = 0; j < GoodMatchePoints.size(); j++)
		{
			/*	tmachePoint[j].targetPoint.u = (keyPoint1[matchePoints[GoodMatchePoints[j].queryIdx][1].trainIdx].pt.x) / (image01.cols);
			tmachePoint[j].targetPoint.v = (keyPoint1[matchePoints[GoodMatchePoints[j].queryIdx][1].trainIdx].pt.y) / (image01.rows);
			tmachePoint[j].partPoint.u = (keyPoint2[matchePoints[GoodMatchePoints[j].queryIdx][0].trainIdx].pt.x) / (image02.cols);
			tmachePoint[j].partPoint.v = (keyPoint2[matchePoints[GoodMatchePoints[j].queryIdx][0].trainIdx].pt.y) / (image02.rows);*/
			//tmachePoint[j].targetPoint.u = (keyPoint1[GoodMatchePoints[j].trainIdx].pt.x) / (image01.cols);
			//tmachePoint[j].targetPoint.v = (keyPoint1[GoodMatchePoints[j].trainIdx].pt.y) / (image01.rows);
			//tmachePoint[j].partPoint.u = (keyPoint2[GoodMatchePoints[j].queryIdx].pt.x) / (image02.cols);
			//tmachePoint[j].partPoint.v = (keyPoint2[GoodMatchePoints[j].queryIdx].pt.y) / (image02.rows);


			pointInterest.x = keyPoint1[GoodMatchePoints[j].trainIdx].pt.x;
			pointInterest.y = keyPoint1[GoodMatchePoints[j].trainIdx].pt.y;
			cv::circle(image01, pointInterest, 10, cv::Scalar(0, 255, 255), -1);//在图像中画出特征点，10是圆的半径  
			pointInterest.x = keyPoint2[GoodMatchePoints[j].queryIdx].pt.x;
			pointInterest.y = keyPoint2[GoodMatchePoints[j].queryIdx].pt.y;
			cv::circle(image02, pointInterest, 10, cv::Scalar(255, 255, 0), -1);//在图像中画出特征点，10是圆的半径  
		}

	}

	//RANSAC 消除误匹配点
	vector<KeyPoint> R_keypoint01,R_keypoint02;
	for (int j = 0; j < GoodMatchePoints.size(); j ++ )
	{
		R_keypoint01.push_back(keyPoint1[GoodMatchePoints[j].trainIdx]);
		R_keypoint02.push_back(keyPoint2[GoodMatchePoints[j].queryIdx]);

	}
	vector<Point2f> p01, p02;
	for (size_t i=0;i<GoodMatchePoints.size();i++)
	{
		p01.push_back(R_keypoint01[i].pt);
		p02.push_back(R_keypoint02[i].pt);
	}
	//利用基础矩阵剔除误匹配点
	vector<uchar> RansacStatus;
	Mat Fundamental = findFundamentalMat(p01, p02, RansacStatus, FM_RANSAC);
	vector<KeyPoint> RR_keypoint01, RR_keypoint02;
	vector<DMatch> RR_matches;
	int index = 0;
	for (size_t i=0;i<GoodMatchePoints.size();i++)
	{
		if (RansacStatus[i]!=0)
		{
			RR_keypoint01.push_back(R_keypoint01[i]);
			RR_keypoint02.push_back(R_keypoint02[i]);
			GoodMatchePoints[i].queryIdx = index;
			GoodMatchePoints[i].trainIdx = index;
			RR_matches.push_back(GoodMatchePoints[i]);
			index++;
		}
	}
	if (RR_matches.size() > 0)
	{
		tmachePoint = (TmachePoints *)malloc(RR_matches.size() * sizeof(TmachePoints));//为纹理匹配点对分配存储空间
		for (size_t j = 0; j < RR_matches.size(); j++)
		{
			tmachePoint[j].targetPoint.u = (RR_keypoint01[RR_matches[j].trainIdx].pt.x) / (image01.cols);
			tmachePoint[j].targetPoint.v = (RR_keypoint01[RR_matches[j].trainIdx].pt.y) / (image01.rows);
			tmachePoint[j].partPoint.u = (RR_keypoint02[RR_matches[j].queryIdx].pt.x) / (image02.cols);
			tmachePoint[j].partPoint.v = (RR_keypoint02[RR_matches[j].queryIdx].pt.y) / (image02.rows);

		
		}
	}
	Mat img_RR_matches;
	//drawMatches(image02, RR_keypoint02, image01, RR_keypoint01, RR_matches, img_RR_matches);


	
	/*for (int i = 0; i < goodmaches.size(); i++)
	{
	cout << "符合条件的匹配：" << goodmaches[i].queryIdx << "--" << goodmaches[i].trainIdx << endl;
	}*/
	/*imshow("效果图1", img_keypoints_1);
	imshow("效果图2", img_keypoints_2);*/
	//imshow("匹配效果", first_match);
	//imshow("ransac", img_RR_matches);
	//imshow("原始图1", image01);
	//imwrite("E:\\a.jpg", first_match);
	return RR_matches.size();
}

/************************************************************************/
/* 计算纹理特征点对                                                                     */
/************************************************************************/
int Registration::texttureKeyPoint(const cv::Mat &image01, const cv::Mat &image02)
{
	
	if (!image01.data || !image02.data)
	{
		AfxMessageBox("读取图片出错");
		return 0;
	}
	
	int minHessian = 500;//300;//应该是hessian响应值
						 //imshow("p2", image01);
						 //imshow("p1", image02);

						 //灰度图转换  
	Mat image1, image2;
	cvtColor(image01, image1, CV_RGB2GRAY);
	cvtColor(image02, image2, CV_RGB2GRAY);


	//提取特征点    
	Ptr<SurfFeatureDetector> detector = SurfFeatureDetector::create(minHessian);

	vector<cv::KeyPoint> keyPoint1, keyPoint2;

	Mat imageDesc1, imageDesc2;
	detector->detectAndCompute(image01, Mat(), keyPoint1, imageDesc1);
	detector->detectAndCompute(image02, Mat(), keyPoint2, imageDesc2);//可以分成detect和compute

	Mat img_keypoints_1, img_keypoints_2;
	drawKeypoints(image01, keyPoint1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	drawKeypoints(image02, keyPoint2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

	FlannBasedMatcher matcher;
	vector<vector<DMatch> > matchePoints;
	vector<DMatch> GoodMatchePoints;

	vector<Mat> train_desc(1, imageDesc1);//此处训练1到2的匹配
	matcher.add(train_desc);
	matcher.train();

	matcher.knnMatch(imageDesc2, matchePoints, 2);
	cout << "total match points: " << matchePoints.size() << endl;

	// Lowe's algorithm,获取优秀匹配点
	for (int i = 0; i < matchePoints.size(); i++)
	{
		if (matchePoints[i][0].distance < 0.6 * matchePoints[i][1].distance)
		{
			GoodMatchePoints.push_back(matchePoints[i][0]);
		}
	}

	Mat first_match;
	//drawMatches(image02, keyPoint2, image01, keyPoint1, GoodMatchePoints, first_match);
	if (GoodMatchePoints.size()>0)
	{
		//tmachePoint = (TmachePoints *)malloc(GoodMatchePoints.size() * sizeof(TmachePoints));//为纹理匹配点对分配存储空间
		cv::Point2f pointInterest;
		//为点对数组赋值//归 1 化处理
		for (int j = 0; j < GoodMatchePoints.size(); j++)
		{
			/*	tmachePoint[j].targetPoint.u = (keyPoint1[matchePoints[GoodMatchePoints[j].queryIdx][1].trainIdx].pt.x) / (image01.cols);
			tmachePoint[j].targetPoint.v = (keyPoint1[matchePoints[GoodMatchePoints[j].queryIdx][1].trainIdx].pt.y) / (image01.rows);
			tmachePoint[j].partPoint.u = (keyPoint2[matchePoints[GoodMatchePoints[j].queryIdx][0].trainIdx].pt.x) / (image02.cols);
			tmachePoint[j].partPoint.v = (keyPoint2[matchePoints[GoodMatchePoints[j].queryIdx][0].trainIdx].pt.y) / (image02.rows);*/
			//tmachePoint[j].targetPoint.u = (keyPoint1[GoodMatchePoints[j].trainIdx].pt.x) / (image01.cols);
			//tmachePoint[j].targetPoint.v = (keyPoint1[GoodMatchePoints[j].trainIdx].pt.y) / (image01.rows);
			//tmachePoint[j].partPoint.u = (keyPoint2[GoodMatchePoints[j].queryIdx].pt.x) / (image02.cols);
			//tmachePoint[j].partPoint.v = (keyPoint2[GoodMatchePoints[j].queryIdx].pt.y) / (image02.rows);


			//pointInterest.x = keyPoint1[GoodMatchePoints[j].trainIdx].pt.x;
			//pointInterest.y = keyPoint1[GoodMatchePoints[j].trainIdx].pt.y;
			//cv::circle(image01, pointInterest, 10, cv::Scalar(0, 255, 255), -1);//在图像中画出特征点，10是圆的半径  
			//pointInterest.x = keyPoint2[GoodMatchePoints[j].queryIdx].pt.x;
			//pointInterest.y = keyPoint2[GoodMatchePoints[j].queryIdx].pt.y;
			//cv::circle(image02, pointInterest, 10, cv::Scalar(255, 255, 0), -1);//在图像中画出特征点，10是圆的半径  
		}

	}

	//RANSAC 消除误匹配点
	vector<KeyPoint> R_keypoint01,R_keypoint02;
	for (int j = 0; j < GoodMatchePoints.size(); j ++ )
	{
		R_keypoint01.push_back(keyPoint1[GoodMatchePoints[j].trainIdx]);
		R_keypoint02.push_back(keyPoint2[GoodMatchePoints[j].queryIdx]);

	}
	vector<Point2f> p01, p02;
	for (size_t i=0;i<GoodMatchePoints.size();i++)
	{
		p01.push_back(R_keypoint01[i].pt);
		p02.push_back(R_keypoint02[i].pt);
	}
	//利用基础矩阵剔除误匹配点
	vector<uchar> RansacStatus;
	Mat Fundamental = findFundamentalMat(p01, p02, RansacStatus, FM_RANSAC);
	if (RansacStatus.size()==0)
	{
		//return FALSE;
	}
	vector<KeyPoint> RR_keypoint01, RR_keypoint02;
	vector<DMatch> RR_matches;
	int index = 0;
	for (size_t i=0;i<GoodMatchePoints.size()&& RansacStatus.size() != 0;i++)
	{
		if (RansacStatus[i]!=0)
		{
			RR_keypoint01.push_back(R_keypoint01[i]);
			RR_keypoint02.push_back(R_keypoint02[i]);
			GoodMatchePoints[i].queryIdx = index;
			GoodMatchePoints[i].trainIdx = index;
			RR_matches.push_back(GoodMatchePoints[i]);
			index++;
		}
	}


	double height1 = targetModel->vertexMax_y - targetModel->vertexMin_y;
	double height2 = partModel->vertexMax_y - partModel->vertexMin_y;
	double ratios = height1 >= height2 ? height1 / height2 : height2 / height1;
	if (ratios>1.5)
	{
		//dlib face detector
		vector<TmachePoints> F_pointPair;
		haveFc=dlFeaturePoints(image01, image02, F_pointPair);
		vector<DMatch> FacMatchePoints(F_pointPair.size());

		for (size_t i=0;i<F_pointPair.size();i++)
		{
			KeyPoint tar_Point, par_Point;
			tar_Point.pt.x = F_pointPair[i].targetPoint.u;
			tar_Point.pt.y = F_pointPair[i].targetPoint.v;
			par_Point.pt.x = F_pointPair[i].partPoint.u;
			par_Point.pt.y = F_pointPair[i].partPoint.v;
			FacMatchePoints[i].queryIdx = index;
			FacMatchePoints[i].trainIdx = index;

			RR_keypoint01.push_back(tar_Point);
			RR_keypoint02.push_back(par_Point);

			RR_matches.push_back(FacMatchePoints[i]);
			index++;

			//cv::Point2f	pointInterest;
			//pointInterest.x = tar_Point.pt.x;
			//pointInterest.y = tar_Point.pt.y;
			//cv::circle(image01, pointInterest, 10, cv::Scalar(0, 255, 255), -1);//在图像中画出特征点，10是圆的半径  
			//pointInterest.x = par_Point.pt.x;
			//pointInterest.y = par_Point.pt.y;
			//cv::circle(image02, pointInterest, 10, cv::Scalar(255, 255, 0), -1);//在图像中画出特征点，10是圆的半径  

		}
	}

	if (RR_matches.size() > 0)
	{
		tmachePoint = (TmachePoints *)malloc(RR_matches.size() * sizeof(TmachePoints));//为纹理匹配点对分配存储空间
		for (size_t j = 0; j < RR_matches.size(); j++)
		{
			tmachePoint[j].targetPoint.u = (RR_keypoint01[RR_matches[j].trainIdx].pt.x) / (image01.cols);
			tmachePoint[j].targetPoint.v = (RR_keypoint01[RR_matches[j].trainIdx].pt.y) / (image01.rows);
			tmachePoint[j].partPoint.u = (RR_keypoint02[RR_matches[j].queryIdx].pt.x) / (image02.cols);
			tmachePoint[j].partPoint.v = (RR_keypoint02[RR_matches[j].queryIdx].pt.y) / (image02.rows);
		}
	}
	Mat img_RR_matches;
	drawMatches(image02, RR_keypoint02, image01, RR_keypoint01, RR_matches, img_RR_matches);


	
	//for (int i = 0; i < goodmaches.size(); i++)
	//{
	//cout << "符合条件的匹配：" << goodmaches[i].queryIdx << "--" << goodmaches[i].trainIdx << endl;
	//}
	//imshow("效果图1", img_keypoints_1);
	//imshow("效果图2", img_keypoints_2);
	//imshow("匹配效果", first_match);
	cv::namedWindow("ransac", WINDOW_NORMAL);
	imshow("ransac", img_RR_matches);
	//imshow("原始图1", image01);
	//imwrite("E:\\a.jpg", first_match);
	return RR_matches.size();
}


int Registration::dlFeaturePoints(const cv::Mat &image01, const cv::Mat &image02,vector<TmachePoints>& PointPair)
{

	if (!image01.data || !image02.data)
	{
		AfxMessageBox("读取图片出错");
		return 0;
	}
	dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
	/*dlib::shape_predictor sp;
	dlib::deserialize("shape_predictor_68_face_landmarks.dat") >> sp;*/

	//for (int i = 2; i < 0; ++i)
	//{
		
		dlib::cv_image<dlib::rgb_pixel> img1(image01);
		//dlib::array2d<dlib::rgb_pixel> img01();
		//pyramid_up(img1);
		dlib::cv_image<dlib::rgb_pixel> img2(image02);
		//pyramid_up(img2);

		std::vector<dlib::rectangle> dets1 = detector(img1);
		std::vector<dlib::full_object_detection> shapes1;
		for (unsigned long j = 0; j < dets1.size(); ++j)
		{
			dlib::full_object_detection shape = sp(img1, dets1[j]);
			//cout << "number of parts: " << shape.num_parts() << endl;
			//cout << "pixel position of first part:  " << shape.part(0) << endl;
			//cout << "pixel position of second part: " << shape.part(1) << endl;
			shapes1.push_back(shape);

		}

		std::vector<dlib::rectangle> dets2 = detector(img2);
		std::vector<dlib::full_object_detection> shapes2;
		for (unsigned long j = 0; j < dets2.size(); ++j)
		{
			dlib::full_object_detection shape = sp(img2, dets2[j]);
			//cout << "number of parts: " << shape.num_parts() << endl;
			//cout << "pixel position of first part:  " << shape.part(40) << endl;//40\4\22\23\32\55
			//cout << "pixel position of second part: " << shape.part(4) << endl;
			shapes2.push_back(shape);
		}
		int indexs[6] = { 40,49,22,23,32,55 };
		for (int i=0;i<6&& shapes1.size()>0&&shapes2.size()>0;i++)
		{
			TmachePoints pointpair;
			pointpair.targetPoint.u = shapes1[0].part(indexs[i]).x();
			pointpair.targetPoint.v = shapes1[0].part(indexs[i]).y();
			pointpair.partPoint.u = shapes2[0].part(indexs[i]).x();
			pointpair.partPoint.v = shapes2[0].part(indexs[i]).y();
			PointPair.push_back(pointpair);
		}


	

	return PointPair.size();
}
/************************************************************************/
/* 
计算匹配点对的曲率   主要目的是判断边缘点，近边缘的半径为宽度或高度的1/16 还未实现
Visual：模型            
pindex:点的索引                                                    */
/************************************************************************/
//double  Registration::GuassCurvature(t_Visual *originalVisual, int pindex,int *isedge)
//{
//	std::set<int > ppindex ;
//	int tempsize ;
//	vertexsOfonevertex GroupVertex;//定义面片顶点组
//	GroupVertex.numOfGroup = 0;
//	t_Visual *myVisual = originalVisual;
//	double curvature = 0.0;
//	if (0 < myVisual->vertexCnt&&myVisual->vertexCnt - myVisual->faceCnt <= myVisual->faceCnt)
//	{
//		if (myVisual->normalCnt <= 0)
//		{
//
//			myVisual->normal = (tVector *)malloc(myVisual->vertexCnt * sizeof(tVector));
//
//		}
//		int i = pindex;
//
//		vertexOfFace Vertex4Normal[20];//定义面片顶点组(按顺序排列)，以计算顶点法向量
//		GroupVertex.numOfGroup = 0;
//
//
//		if (myVisual->normalCnt > 0)
//		{
//			//循环遍历查找与顶点相同的点
//			for (int j = 0; j < myVisual->faceCnt; j++)
//			{
//
//
//				int temp = GroupVertex.numOfGroup;
//
//				if (myVisual->index[j].v[0] == i)
//				{
//					myVisual->index[j].v[0] = i;
//					GroupVertex.twoPoints[temp].p1 = myVisual->vertex[myVisual->index[j].v[1]];
//					GroupVertex.twoPoints[temp].p2 = myVisual->vertex[myVisual->index[j].v[2]];
//					/*GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[j].v[1]].x;
//					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[j].v[1]].y;
//					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[j].v[1]].z;
//					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[j].v[2]].x;
//					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[j].v[2]].y;
//					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[j].v[2]].z;*/
//					GroupVertex.numOfGroup += 1;
//					ppindex.insert(myVisual->index[j].v[1]);
//					ppindex.insert(myVisual->index[j].v[2]);
//
//				}
//				else if (myVisual->index[j].v[1] == i)
//				{
//					myVisual->index[j].v[1] = i;
//					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
//					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
//					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;
//					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[j].v[2]].x;
//					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[j].v[2]].y;
//					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[j].v[2]].z;
//					GroupVertex.numOfGroup += 1;
//					ppindex.insert(myVisual->index[j].v[0]);
//					ppindex.insert(myVisual->index[j].v[2]);
//				}
//				else if (myVisual->index[j].v[2] == i)
//				{
//					myVisual->index[j].v[2] = i;
//					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
//					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
//					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;
//					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[j].v[1]].x;
//					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[j].v[1]].y;
//					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[j].v[1]].z;
//					GroupVertex.numOfGroup += 1;
//					ppindex.insert(myVisual->index[j].v[0]);
//					ppindex.insert(myVisual->index[j].v[1]);
//				}
//
//			}
//
//		}
//		else
//		{
//
//			//循环遍历查找与顶点相同的点
//			for (int j = 0; j < myVisual->faceCnt; j++)
//			{
//
//
//				int temp = GroupVertex.numOfGroup;
//
//				if (myVisual->index[j].v[0] == i)
//				{
//					myVisual->index[j].v[0] = i;
//					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[j].v[1]].x;
//					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[j].v[1]].y;
//					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[j].v[1]].z;
//
//					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[j].v[2]].x;
//					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[j].v[2]].y;
//					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[j].v[2]].z;
//
//					Vertex4Normal[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
//					Vertex4Normal[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
//					Vertex4Normal[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;
//
//					Vertex4Normal[temp].p2.x = myVisual->vertex[myVisual->index[j].v[1]].x;
//					Vertex4Normal[temp].p2.y = myVisual->vertex[myVisual->index[j].v[1]].y;
//					Vertex4Normal[temp].p2.z = myVisual->vertex[myVisual->index[j].v[1]].z;
//
//					Vertex4Normal[temp].p3.x = myVisual->vertex[myVisual->index[j].v[2]].x;
//					Vertex4Normal[temp].p3.y = myVisual->vertex[myVisual->index[j].v[2]].y;
//					Vertex4Normal[temp].p3.z = myVisual->vertex[myVisual->index[j].v[2]].z;
//
//					GroupVertex.numOfGroup += 1;
//					ppindex.insert(myVisual->index[j].v[1]);
//					ppindex.insert(myVisual->index[j].v[2]);
//
//				}
//				else if (myVisual->index[j].v[1] == i)
//				{
//					myVisual->index[j].v[1] = i;
//					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
//					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
//					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;
//					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[j].v[2]].x;
//					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[j].v[2]].y;
//					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[j].v[2]].z;
//
//
//					Vertex4Normal[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
//					Vertex4Normal[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
//					Vertex4Normal[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;
//
//					Vertex4Normal[temp].p2.x = myVisual->vertex[myVisual->index[j].v[1]].x;
//					Vertex4Normal[temp].p2.y = myVisual->vertex[myVisual->index[j].v[1]].y;
//					Vertex4Normal[temp].p2.z = myVisual->vertex[myVisual->index[j].v[1]].z;
//
//					Vertex4Normal[temp].p3.x = myVisual->vertex[myVisual->index[j].v[2]].x;
//					Vertex4Normal[temp].p3.y = myVisual->vertex[myVisual->index[j].v[2]].y;
//					Vertex4Normal[temp].p3.z = myVisual->vertex[myVisual->index[j].v[2]].z;
//
//					GroupVertex.numOfGroup += 1;
//					ppindex.insert(myVisual->index[j].v[0]);
//					ppindex.insert(myVisual->index[j].v[2]);
//				}
//				else if (myVisual->index[j].v[2] == i)
//				{
//					myVisual->index[j].v[2] = i;
//					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
//					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
//					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;
//					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[j].v[1]].x;
//					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[j].v[1]].y;
//					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[j].v[1]].z;
//
//
//					Vertex4Normal[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
//					Vertex4Normal[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
//					Vertex4Normal[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;
//
//					Vertex4Normal[temp].p2.x = myVisual->vertex[myVisual->index[j].v[1]].x;
//					Vertex4Normal[temp].p2.y = myVisual->vertex[myVisual->index[j].v[1]].y;
//					Vertex4Normal[temp].p2.z = myVisual->vertex[myVisual->index[j].v[1]].z;
//
//					Vertex4Normal[temp].p3.x = myVisual->vertex[myVisual->index[j].v[2]].x;
//					Vertex4Normal[temp].p3.y = myVisual->vertex[myVisual->index[j].v[2]].y;
//					Vertex4Normal[temp].p3.z = myVisual->vertex[myVisual->index[j].v[2]].z;
//
//					GroupVertex.numOfGroup += 1;
//					ppindex.insert(myVisual->index[j].v[0]);
//					ppindex.insert(myVisual->index[j].v[1]);
//				}
//
//			}
//
//		}
//		if (GroupVertex.numOfGroup <= 2)
//		{
//			//myVisual->vertex[i].z = myVisual->vertexMax_z + 2;
//			tempsize = ppindex.size();
//			if (GroupVertex.numOfGroup - tempsize < 0)
//			{
//				*isedge = 1;
//			}
//			else
//				*isedge = 0;
//			curvature = 0.0;
//			return curvature;
//		}
//
//		//计算高斯曲率
//		double sumAngle = 0.0;
//		double sumArea = 0.0;
//		tVector *facevector = new tVector[GroupVertex.numOfGroup];
//
//		//判断是否需要计算顶点法向量
//		if (myVisual->normalCnt <= 0)
//		{
//			myVisual->normal[i].x = 0;
//			myVisual->normal[i].y = 0;
//			myVisual->normal[i].z = 0;
//			for (int m = 0; m < GroupVertex.numOfGroup; m++)
//			{
//				tVector tempnormal;
//				tempnormal = CountNormalofFace(Vertex4Normal[m].p1, Vertex4Normal[m].p2, Vertex4Normal[m].p3);
//
//				myVisual->normal[i].x += tempnormal.x;
//				myVisual->normal[i].y += tempnormal.y;
//				myVisual->normal[i].z += tempnormal.z;
//
//			}
//		}
//		//tVector *normalOfFace=(tVector *)malloc(myVisual->GroupVertex[i].numOfGroup*sizeof(tVector));
//		for (int m = 0; m < GroupVertex.numOfGroup; m++)
//		{
//			t_Triangle tempTriangle = CountTangle(myVisual->vertex[i], GroupVertex.twoPoints[m].p1, GroupVertex.twoPoints[m].p2);
//			sumAngle += tempTriangle.angle;
//			sumArea += tempTriangle.area;
//			facevector[m].x = tempTriangle.FaceVector.x;
//			facevector[m].y = tempTriangle.FaceVector.y;
//			facevector[m].z = tempTriangle.FaceVector.z;
//		}
//		//计算平均曲率
//		double sumHcurvature = 0;//未除面积的平均曲率
//
//		for (int m = 0; m < GroupVertex.numOfGroup; m++)
//		{
//			tVector edge1, edge2, edge3, edge4;
//			//tVector normal1,normal2;
//			double normalangle = 0.0;
//			double edgelenth = 0.0;
//
//
//			edge1.x = GroupVertex.twoPoints[m].p1.x - myVisual->vertex[i].x;
//			edge1.y = GroupVertex.twoPoints[m].p1.y - myVisual->vertex[i].y;
//			edge1.z = GroupVertex.twoPoints[m].p1.z - myVisual->vertex[i].z;
//			edge2.x = GroupVertex.twoPoints[m].p2.x - myVisual->vertex[i].x;
//			edge2.y = GroupVertex.twoPoints[m].p2.y - myVisual->vertex[i].y;
//			edge2.z = GroupVertex.twoPoints[m].p2.z - myVisual->vertex[i].z;
//			for (int mm = 0; mm < GroupVertex.numOfGroup; mm++)
//			{
//				if (mm != m)//忽略同一条边
//				{
//					//利用当前面和这个面的法向量n1、n2求出夹角，算出部分曲率H^
//					edge3.x = GroupVertex.twoPoints[mm].p1.x - myVisual->vertex[i].x;
//					edge3.y = GroupVertex.twoPoints[mm].p1.y - myVisual->vertex[i].y;
//					edge3.z = GroupVertex.twoPoints[mm].p1.z - myVisual->vertex[i].z;
//					edge4.x = GroupVertex.twoPoints[mm].p2.x - myVisual->vertex[i].x;
//					edge4.y = GroupVertex.twoPoints[mm].p2.y - myVisual->vertex[i].y;
//					edge4.z = GroupVertex.twoPoints[mm].p2.z - myVisual->vertex[i].z;
//					//判断共边
//					if (edge1.x == edge3.x&&edge1.y == edge3.y&&edge1.z == edge3.z)
//					{
//						normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//						edgelenth = CountVecLenth(edge1);//计算共边的长度
//						sumHcurvature += normalangle*edgelenth;
//
//
//					}
//					else if (edge1.x == edge4.x&&edge1.y == edge4.y&&edge1.z == edge4.z)
//					{
//						normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//						edgelenth = CountVecLenth(edge1);//计算共边的长度
//						sumHcurvature += normalangle*edgelenth;
//
//
//					}
//					else if (edge2.x == edge4.x&&edge2.y == edge4.y&&edge2.z == edge4.z)
//					{
//						normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//						edgelenth = CountVecLenth(edge2);//计算共边的长度
//						sumHcurvature += normalangle*edgelenth;
//
//
//					}
//					else if (edge2.x == edge3.x&&edge2.y == edge3.y&&edge2.z == edge3.z)
//					{
//						normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//						edgelenth = CountVecLenth(edge2);//计算共边的长度
//						sumHcurvature += normalangle*edgelenth;
//
//
//					}
//
//				}
//
//			}
//		}
//
//		delete[] facevector;
//		if (sumArea > 0)
//		{
//			originalVisual->HCurvature[i] = 0.25*sumHcurvature / sumArea;
//			curvature = (2 * 3.1415926 - sumAngle) / sumArea;
//			if (curvature> 0)
//			{
//				int tempGaussNum = 0;
//				for (int m = 0; m < GroupVertex.numOfGroup; m++)
//				{
//					//求顶点邻边是否与法向量锐角
//					double MultiVector1 = myVisual->normal[i].x*(GroupVertex.twoPoints[m].p1.x - myVisual->vertex[i].x)
//						+ myVisual->normal[i].y*(GroupVertex.twoPoints[m].p1.y - myVisual->vertex[i].y)
//						+ myVisual->normal[i].z*(GroupVertex.twoPoints[m].p1.z - myVisual->vertex[i].z);
//					double MultiVector2 = myVisual->normal[i].x*(GroupVertex.twoPoints[m].p2.x - myVisual->vertex[i].x)
//						+ myVisual->normal[i].y*(GroupVertex.twoPoints[m].p2.y - myVisual->vertex[i].y)
//						+ myVisual->normal[i].z*(GroupVertex.twoPoints[m].p2.z - myVisual->vertex[i].z);
//					if (MultiVector1 >= 0 && MultiVector2 >= 0)
//					{
//						tempGaussNum++;
//					}
//
//				}
//				if (tempGaussNum >= GroupVertex.numOfGroup - 2)
//				{
//
//					curvature = -curvature;
//				}
//			}
//			else
//			{
//				//curvature = 0;//马鞍曲面
//			}
//
//		}
//		else {
//
//			originalVisual->HCurvature[i] = 0;
//			curvature = 1.0;
//		}
//		
//		if (myVisual->normalCnt <= 0)
//		{
//			free(myVisual->normal);
//			myVisual->normal = NULL;
//		}
//
//	}
//	//针对kinect OBJ模型 默认有顶点向量
//	if (0<myVisual->vertexCnt&&myVisual->vertexCnt - myVisual->faceCnt>myVisual->faceCnt)
//	{
//		//循环遍历顶点
//
//		int i = pindex;
//
//		
//		//vertexsOfonevertex GroupVertex;//定义面片顶点组
//		GroupVertex.numOfGroup = 0;
//		//循环遍历查找与顶点重复的点
//		for (int j = 0; j<myVisual->vertexCnt - i - 1; j++)
//		{
//			//判断下一个点是否已经标记，若标记则进行下次循环
//			if (GroupVertex.numOfGroup>19)
//			{
//				break;
//			}
//			int sumji = j + i;//当前点的位置的j个位移
//			
//
//			//从当前顶点位置开始查找判断，以便将当前顶点收入组中
//			if (myVisual->vertex[sumji].x == myVisual->vertex[i].x&&
//				myVisual->vertex[sumji].y == myVisual->vertex[i].y&&
//				myVisual->vertex[sumji].z == myVisual->vertex[i].z)
//			{
//
//				
//				//遍历面片，将面片的顶点索引修改为所有重复顶点的第一个顶点的索引,将同一顶点面片的其他点信息收入组中
//
//				int temp = GroupVertex.numOfGroup;
//				int tempvsite = (sumji) / 3;
//				int tempvsiteyu = (sumji) % 3;//此设计完全是按照kinectOBJ模型数据结构
//				if (myVisual->index[tempvsite].v[0] == sumji)
//				{
//					myVisual->index[tempvsite].v[0] = i;
//					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[tempvsite].v[1]].x;
//					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[tempvsite].v[1]].y;
//					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[tempvsite].v[1]].z;
//					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[tempvsite].v[2]].x;
//					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[tempvsite].v[2]].y;
//					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[tempvsite].v[2]].z;
//					GroupVertex.numOfGroup += 1;
//					ppindex.insert(myVisual->index[tempvsite].v[1]);
//					ppindex.insert(myVisual->index[tempvsite].v[2]);
//
//				}
//				else if (myVisual->index[tempvsite].v[1] == sumji)
//				{
//					myVisual->index[tempvsite].v[1] = i;
//					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[tempvsite].v[0]].x;
//					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[tempvsite].v[0]].y;
//					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[tempvsite].v[0]].z;
//					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[tempvsite].v[2]].x;
//					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[tempvsite].v[2]].y;
//					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[tempvsite].v[2]].z;
//					GroupVertex.numOfGroup += 1;
//					ppindex.insert(myVisual->index[tempvsite].v[0]);
//					ppindex.insert(myVisual->index[tempvsite].v[2]);
//				}
//				else if (myVisual->index[tempvsite].v[2] == sumji)
//				{
//					myVisual->index[tempvsite].v[2] = i;
//					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[tempvsite].v[0]].x;
//					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[tempvsite].v[0]].y;
//					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[tempvsite].v[0]].z;
//					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[tempvsite].v[1]].x;
//					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[tempvsite].v[1]].y;
//					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[tempvsite].v[1]].z;
//					GroupVertex.numOfGroup += 1;
//					ppindex.insert(myVisual->index[tempvsite].v[0]);
//					ppindex.insert(myVisual->index[tempvsite].v[1]);
//				}
//			}
//		}
//		if (GroupVertex.numOfGroup <= 2)
//		{
//			//myVisual->vertex[i].z = myVisual->vertexMax_z + 2;
//			tempsize = ppindex.size();
//			if (GroupVertex.numOfGroup - tempsize < 0)
//			{
//				*isedge = 1;
//			}
//			else
//				*isedge = 0;
//			curvature = 0.0;
//			/*if (filest)
//			{
//			filest << originalVisual->GuassCurvature[i] << endl;
//			}*/
//			return curvature;
//		}
//		//计算高斯曲率
//		double sumAngle = 0.0;
//		double sumArea = 0.0;
//		tVector *facevector = new tVector[GroupVertex.numOfGroup];
//		//tVector *normalOfFace=(tVector *)malloc(myVisual->GroupVertex[i].numOfGroup*sizeof(tVector));
//
//		//for (int m = 0; m < myVisual->GroupVertex[i].numOfGroup; i++)
//		//{
//		//	normalOfFace[m] = CountNormalofFace(myVisual->vertex[i], myVisual->GroupVertex[i].twoPoints[m].p1, myVisual->GroupVertex[i].twoPoints[m].p2);
//		//	/*t_Triangle tempTriangle = CountTangle(myVisual->vertex[i], myVisual->GroupVertex[i].twoPoints[m].p1, myVisual->GroupVertex[i].twoPoints[m].p2);
//		//	sumAngle += tempTriangle.angle;
//		//	sumArea += tempTriangle.area;*/
//
//		//}
//		//计算高斯曲率参数 1、夹角和 2、voronoi面积
//		for (int m = 0; m < GroupVertex.numOfGroup; m++)
//		{
//			t_Triangle tempTriangle = CountTangle(myVisual->vertex[i], GroupVertex.twoPoints[m].p1, GroupVertex.twoPoints[m].p2);
//			sumAngle += tempTriangle.angle;
//			sumArea += tempTriangle.area;
//			facevector[m].x = tempTriangle.FaceVector.x;
//			facevector[m].y = tempTriangle.FaceVector.y;
//			facevector[m].z = tempTriangle.FaceVector.z;
//		}
//		//计算平均曲率
//		double sumHcurvature = 0;//未除面积的平均曲率
//
//		for (int m = 0; m < GroupVertex.numOfGroup; m++)
//		{
//			tVector edge1, edge2, edge3, edge4;
//			//tVector normal1,normal2;
//			double normalangle = 0.0;
//			double edgelenth = 0.0;
//
//
//			edge1.x = GroupVertex.twoPoints[m].p1.x - myVisual->vertex[i].x;
//			edge1.y = GroupVertex.twoPoints[m].p1.y - myVisual->vertex[i].y;
//			edge1.z = GroupVertex.twoPoints[m].p1.z - myVisual->vertex[i].z;
//			edge2.x = GroupVertex.twoPoints[m].p2.x - myVisual->vertex[i].x;
//			edge2.y = GroupVertex.twoPoints[m].p2.y - myVisual->vertex[i].y;
//			edge2.z = GroupVertex.twoPoints[m].p2.z - myVisual->vertex[i].z;
//			for (int mm = 0; mm < GroupVertex.numOfGroup; mm++)
//			{
//				if (mm != m)//忽略同一条边
//				{
//					//利用当前面和这个面的法向量n1、n2求出夹角，算出部分曲率H^
//					edge3.x = GroupVertex.twoPoints[mm].p1.x - myVisual->vertex[i].x;
//					edge3.y = GroupVertex.twoPoints[mm].p1.y - myVisual->vertex[i].y;
//					edge3.z = GroupVertex.twoPoints[mm].p1.z - myVisual->vertex[i].z;
//					edge4.x = GroupVertex.twoPoints[mm].p2.x - myVisual->vertex[i].x;
//					edge4.y = GroupVertex.twoPoints[mm].p2.y - myVisual->vertex[i].y;
//					edge4.z = GroupVertex.twoPoints[mm].p2.z - myVisual->vertex[i].z;
//					//判断共边
//					if (edge1.x == edge3.x&&edge1.y == edge3.y&&edge1.z == edge3.z)
//					{
//						normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//						edgelenth = CountVecLenth(edge1);//计算共边的长度
//						sumHcurvature += normalangle*edgelenth;
//
//
//					}
//					else if (edge1.x == edge4.x&&edge1.y == edge4.y&&edge1.z == edge4.z)
//					{
//						normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//						edgelenth = CountVecLenth(edge1);//计算共边的长度
//						sumHcurvature += normalangle*edgelenth;
//
//
//					}
//					else if (edge2.x == edge4.x&&edge2.y == edge4.y&&edge2.z == edge4.z)
//					{
//						normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//						edgelenth = CountVecLenth(edge2);//计算共边的长度
//						sumHcurvature += normalangle*edgelenth;
//
//
//					}
//					else if (edge2.x == edge3.x&&edge2.y == edge3.y&&edge2.z == edge3.z)
//					{
//						normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
//						edgelenth = CountVecLenth(edge2);//计算共边的长度
//						sumHcurvature += normalangle*edgelenth;
//
//
//					}
//
//				}
//
//			}
//		}
//		delete[] facevector;
//		if (sumArea > 0)
//		{
//			originalVisual->HCurvature[i] = 0.25*sumHcurvature / sumArea;
//			curvature = (2 * 3.1415926 - sumAngle) / sumArea;
//			if (curvature>0)
//			{
//				int tempGaussNum = 0;
//				for (int m = 0; m < GroupVertex.numOfGroup; m++)
//				{
//					//求顶点邻边是否与法向量锐角
//					double MultiVector1 = myVisual->normal[i].x*(GroupVertex.twoPoints[m].p1.x - myVisual->vertex[i].x)
//						+ myVisual->normal[i].y*(GroupVertex.twoPoints[m].p1.y - myVisual->vertex[i].y)
//						+ myVisual->normal[i].z*(GroupVertex.twoPoints[m].p1.z - myVisual->vertex[i].z);
//					double MultiVector2 = myVisual->normal[i].x*(GroupVertex.twoPoints[m].p2.x - myVisual->vertex[i].x)
//						+ myVisual->normal[i].y*(GroupVertex.twoPoints[m].p2.y - myVisual->vertex[i].y)
//						+ myVisual->normal[i].z*(GroupVertex.twoPoints[m].p2.z - myVisual->vertex[i].z);
//					if (MultiVector1 >= 0 && MultiVector2 >= 0)
//					{
//						tempGaussNum++;
//					}
//
//				}
//				if (tempGaussNum >= GroupVertex.numOfGroup - 2)
//				{
//
//					curvature = -curvature;
//				}
//			}
//			else
//			{
//				//curvature = 0;//马鞍曲面
//			}
//
//		}
//		else {
//
//			originalVisual->HCurvature[i] = 0;
//			curvature = 1.0;
//		}
//		
//	
//
//	
//	}
//	tempsize = ppindex.size();
//	if (GroupVertex.numOfGroup - tempsize<0)
//	{
// 		*isedge = 1;
//	}
//	else
//		*isedge = 0;
//	return curvature;
//}
double  Registration::GuassCurvature(t_Visual *originalVisual, int pindex, int *isedge)
{
	std::set<int > ppindex;
	int tempsize;
	double edgeSize = 0;//边缘的范围   宽度或高度的1/16 
	edgeSize = (originalVisual->vertexMax_x - originalVisual->vertexMin_x)/16.0;
	vertexsOfonevertex GroupVertex;//定义面片顶点组
	GroupVertex.numOfGroup = 0;
	t_Visual *myVisual = originalVisual;
	double curvature = 0.0;
	if (0 < myVisual->vertexCnt&&myVisual->vertexCnt - myVisual->faceCnt <= myVisual->faceCnt)
	{
		if (myVisual->normalCnt <= 0)
		{

			myVisual->normal = (tVector *)malloc(myVisual->vertexCnt * sizeof(tVector));

		}
		int i = pindex;

		vertexOfFace Vertex4Normal[20];//定义面片顶点组(按顺序排列)，以计算顶点法向量
		GroupVertex.numOfGroup = 0;


		if (myVisual->normalCnt > 0)
		{
			//循环遍历查找与顶点相同的点
			for (int j = 0; j < myVisual->faceCnt; j++)
			{


				int temp = GroupVertex.numOfGroup;

				if (myVisual->index[j].v[0] == i)
				{
					myVisual->index[j].v[0] = i;
					GroupVertex.twoPoints[temp].p1 = myVisual->vertex[myVisual->index[j].v[1]];
					GroupVertex.twoPoints[temp].p2 = myVisual->vertex[myVisual->index[j].v[2]];
				
					GroupVertex.numOfGroup += 1;
					ppindex.insert(myVisual->index[j].v[1]);
					ppindex.insert(myVisual->index[j].v[2]);

				}
				else if (myVisual->index[j].v[1] == i)
				{
					myVisual->index[j].v[1] = i;
					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;
					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[j].v[2]].x;
					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[j].v[2]].y;
					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[j].v[2]].z;
					GroupVertex.numOfGroup += 1;
					ppindex.insert(myVisual->index[j].v[0]);
					ppindex.insert(myVisual->index[j].v[2]);
				}
				else if (myVisual->index[j].v[2] == i)
				{
					myVisual->index[j].v[2] = i;
					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;

					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[j].v[1]].x;
					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[j].v[1]].y;
					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[j].v[1]].z;
					GroupVertex.numOfGroup += 1;
					ppindex.insert(myVisual->index[j].v[0]);
					ppindex.insert(myVisual->index[j].v[1]);
				}

			}

		}
		else
		{

			//循环遍历查找与顶点相同的点
			for (int j = 0; j < myVisual->faceCnt; j++)
			{


				int temp = GroupVertex.numOfGroup;

				if (myVisual->index[j].v[0] == i)
				{
					myVisual->index[j].v[0] = i;
					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[j].v[1]].x;
					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[j].v[1]].y;
					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[j].v[1]].z;

					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[j].v[2]].x;
					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[j].v[2]].y;
					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[j].v[2]].z;

					Vertex4Normal[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
					Vertex4Normal[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
					Vertex4Normal[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;

					Vertex4Normal[temp].p2.x = myVisual->vertex[myVisual->index[j].v[1]].x;
					Vertex4Normal[temp].p2.y = myVisual->vertex[myVisual->index[j].v[1]].y;
					Vertex4Normal[temp].p2.z = myVisual->vertex[myVisual->index[j].v[1]].z;

					Vertex4Normal[temp].p3.x = myVisual->vertex[myVisual->index[j].v[2]].x;
					Vertex4Normal[temp].p3.y = myVisual->vertex[myVisual->index[j].v[2]].y;
					Vertex4Normal[temp].p3.z = myVisual->vertex[myVisual->index[j].v[2]].z;

					GroupVertex.numOfGroup += 1;
					ppindex.insert(myVisual->index[j].v[1]);
					ppindex.insert(myVisual->index[j].v[2]);

				}
				else if (myVisual->index[j].v[1] == i)
				{
					myVisual->index[j].v[1] = i;
					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;

					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[j].v[2]].x;
					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[j].v[2]].y;
					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[j].v[2]].z;


					Vertex4Normal[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
					Vertex4Normal[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
					Vertex4Normal[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;

					Vertex4Normal[temp].p2.x = myVisual->vertex[myVisual->index[j].v[1]].x;
					Vertex4Normal[temp].p2.y = myVisual->vertex[myVisual->index[j].v[1]].y;
					Vertex4Normal[temp].p2.z = myVisual->vertex[myVisual->index[j].v[1]].z;

					Vertex4Normal[temp].p3.x = myVisual->vertex[myVisual->index[j].v[2]].x;
					Vertex4Normal[temp].p3.y = myVisual->vertex[myVisual->index[j].v[2]].y;
					Vertex4Normal[temp].p3.z = myVisual->vertex[myVisual->index[j].v[2]].z;

					GroupVertex.numOfGroup += 1;
					ppindex.insert(myVisual->index[j].v[0]);
					ppindex.insert(myVisual->index[j].v[2]);
				}
				else if (myVisual->index[j].v[2] == i)
				{
					myVisual->index[j].v[2] = i;
					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;

					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[j].v[1]].x;
					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[j].v[1]].y;
					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[j].v[1]].z;


					Vertex4Normal[temp].p1.x = myVisual->vertex[myVisual->index[j].v[0]].x;
					Vertex4Normal[temp].p1.y = myVisual->vertex[myVisual->index[j].v[0]].y;
					Vertex4Normal[temp].p1.z = myVisual->vertex[myVisual->index[j].v[0]].z;

					Vertex4Normal[temp].p2.x = myVisual->vertex[myVisual->index[j].v[1]].x;
					Vertex4Normal[temp].p2.y = myVisual->vertex[myVisual->index[j].v[1]].y;
					Vertex4Normal[temp].p2.z = myVisual->vertex[myVisual->index[j].v[1]].z;

					Vertex4Normal[temp].p3.x = myVisual->vertex[myVisual->index[j].v[2]].x;
					Vertex4Normal[temp].p3.y = myVisual->vertex[myVisual->index[j].v[2]].y;
					Vertex4Normal[temp].p3.z = myVisual->vertex[myVisual->index[j].v[2]].z;

					GroupVertex.numOfGroup += 1;
					ppindex.insert(myVisual->index[j].v[0]);
					ppindex.insert(myVisual->index[j].v[1]);
				}

			}

		}
		if (GroupVertex.numOfGroup <= 2)
		{
			//myVisual->vertex[i].z = myVisual->vertexMax_z + 2;
			tempsize = ppindex.size();
			if (GroupVertex.numOfGroup - tempsize < 0)
			{
				*isedge = 1;
			}
			else
				*isedge = 0;
			curvature = 0.0;
			return 0;
		}



	}
	//针对kinect OBJ模型 默认有顶点向量
	if (0<myVisual->vertexCnt&&myVisual->vertexCnt - myVisual->faceCnt>myVisual->faceCnt)
	{
		//循环遍历顶点

		int i = pindex;


		//vertexsOfonevertex GroupVertex;//定义面片顶点组
		GroupVertex.numOfGroup = 0;
		//循环遍历查找与顶点重复的点
		for (int j = 0; j<myVisual->vertexCnt - i - 1; j++)
		{
			//判断下一个点是否已经标记，若标记则进行下次循环
			if (GroupVertex.numOfGroup>19)
			{
				break;
			}
			int sumji = j + i;//当前点的位置的j个位移


							  //从当前顶点位置开始查找判断，以便将当前顶点收入组中
			if (myVisual->vertex[sumji].x == myVisual->vertex[i].x&&
				myVisual->vertex[sumji].y == myVisual->vertex[i].y&&
				myVisual->vertex[sumji].z == myVisual->vertex[i].z)
			{


				//遍历面片，将面片的顶点索引修改为所有重复顶点的第一个顶点的索引,将同一顶点面片的其他点信息收入组中

				int temp = GroupVertex.numOfGroup;
				int tempvsite = (sumji) / 3;
				int tempvsiteyu = (sumji) % 3;//此设计完全是按照kinectOBJ模型数据结构
				if (myVisual->index[tempvsite].v[0] == sumji)
				{
					myVisual->index[tempvsite].v[0] = i;
					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[tempvsite].v[1]].x;
					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[tempvsite].v[1]].y;
					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[tempvsite].v[1]].z;
					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[tempvsite].v[2]].x;
					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[tempvsite].v[2]].y;
					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[tempvsite].v[2]].z;
					GroupVertex.numOfGroup += 1;
					ppindex.insert(myVisual->index[tempvsite].v[1]);
					ppindex.insert(myVisual->index[tempvsite].v[2]);

				}
				else if (myVisual->index[tempvsite].v[1] == sumji)
				{
					myVisual->index[tempvsite].v[1] = i;
					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[tempvsite].v[0]].x;
					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[tempvsite].v[0]].y;
					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[tempvsite].v[0]].z;
					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[tempvsite].v[2]].x;
					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[tempvsite].v[2]].y;
					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[tempvsite].v[2]].z;
					GroupVertex.numOfGroup += 1;
					ppindex.insert(myVisual->index[tempvsite].v[0]);
					ppindex.insert(myVisual->index[tempvsite].v[2]);
				}
				else if (myVisual->index[tempvsite].v[2] == sumji)
				{
					myVisual->index[tempvsite].v[2] = i;
					GroupVertex.twoPoints[temp].p1.x = myVisual->vertex[myVisual->index[tempvsite].v[0]].x;
					GroupVertex.twoPoints[temp].p1.y = myVisual->vertex[myVisual->index[tempvsite].v[0]].y;
					GroupVertex.twoPoints[temp].p1.z = myVisual->vertex[myVisual->index[tempvsite].v[0]].z;
					GroupVertex.twoPoints[temp].p2.x = myVisual->vertex[myVisual->index[tempvsite].v[1]].x;
					GroupVertex.twoPoints[temp].p2.y = myVisual->vertex[myVisual->index[tempvsite].v[1]].y;
					GroupVertex.twoPoints[temp].p2.z = myVisual->vertex[myVisual->index[tempvsite].v[1]].z;
					GroupVertex.numOfGroup += 1;
					ppindex.insert(myVisual->index[tempvsite].v[0]);
					ppindex.insert(myVisual->index[tempvsite].v[1]);
				}
			}
		}
		if (GroupVertex.numOfGroup <= 2)
		{
			//myVisual->vertex[i].z = myVisual->vertexMax_z + 2;
			tempsize = ppindex.size();
			if (GroupVertex.numOfGroup - tempsize < 0)
			{
				*isedge = 1;
			}
			else
				*isedge = 0;
			curvature = 0.0;
			/*if (filest)
			{
			filest << originalVisual->GuassCurvature[i] << endl;
			}*/
			return 0;
		}
	}
	tempsize = ppindex.size();
	if (GroupVertex.numOfGroup - tempsize<0)
	{
		*isedge = 1;
	}
	else
		*isedge = 0;
	return 0;
}

/************************************************************************/
/* 功能：判断球形邻域
参数：CenterPoint：中心点；Radius：半径
*/
/************************************************************************/
BOOL Registration::isNeighbourhood(tVector CenterPoint, tVector NeighberPoint, double Radius)
{
	tVector tempVector = CenterPoint - NeighberPoint;
	if (tempVector.x*tempVector.x+tempVector.y*tempVector.y+tempVector.z*tempVector.z<Radius*Radius)
	{
		return TRUE;
	}
	return FALSE;
}	
/************************************************************************/
/* 功能：判断是否在顶点元素是否在list中
参数：lt：LIST；pindex：顶点索引；
*/
/************************************************************************/
BOOL Registration::findListData(list<vertexCurvature> lt, int pindex)
{
	for (list<vertexCurvature>::iterator it = lt.begin(); it != lt.end(); it++)
	{
		if ((*it).index == pindex)
		{
			return TRUE;
		}
	}
	return FALSE;
}
/************************************************************************/
/* 功能：判断是否在顶点元素是否在list中,并删除//针对无重复元素的list，只删除一次！！！！！！！！！
参数：lt：LIST；pindex：顶点索引；
*/
/************************************************************************/
BOOL Registration::DelListData(list<vertexCurvature> lt, int pindex)
{

	for (list<vertexCurvature>::iterator it = lt.begin(); it != lt.end(); )
	{
		if ((*it).index == pindex)
		{
			it = lt.erase(it);
			break;
			return TRUE;
		}
		else
		{
			it++;
		}
	}
	return FALSE;
}
/************************************************************************/
/* 功能：计算领域曲率
参数：*originalVisual：模型；neighborIndex：邻域顶点的索引；pindex：顶点的索引；r：邻域的半径
*/
/************************************************************************/
BOOL Registration::NeibGuassCurvature(t_Visual *originalVisual, list<vertexCurvature > &neighborIndex, int pindex, double r)
{
	std::set<int > ppindex;	//邻点索引
	int tempsize=0;			//ppindex索引set集合的大小
	vertexsOfonevertex GroupVertex;//定义面片顶点组
	GroupVertex.numOfGroup = 0;
	t_Visual *myVisual = originalVisual;
	double Mwidth = (myVisual->vertexMax_x - myVisual->vertexMin_x);//模型的宽度
	double Mheight = (myVisual->vertexMax_y - myVisual->vertexMin_y);//模型的高度
	double curvature = 0.0;
	double R = r;
	vertexCurvature vp;
	vp.index = pindex;
	neighborIndex.push_back(vp);
	list<vertexCurvature>::iterator iter;
	vertexOfFace Vertex4Normal[20];//定义面片顶点组(按顺序排列)，以计算顶点法向量,以下循环中没有清除上一次的值。
	long tempfindex ;
	int temp ;
	double tempr;
	double sumAngle = 0.0;
	double sumArea = 0.0;

	double sumHcurvature = 0;//未除面积的平均曲率
	tVector sumHVector;

	tVector edge1, edge2, edge3, edge4;
	//tVector normal1,normal2;
	double normalangle = 0.0, normalangle1 = 0.0, normalangle2 = 0.0;
	double edgelenth = 0.0;
	double tempH = 0;


	Eigen::Vector3d n1, n2;
	for (iter =neighborIndex.begin();iter!= neighborIndex.end();iter++)
	{
		//思路：
		//查找子相邻顶点，判断是否在r为半径的邻域内，如果在邻域内，插入list中
		//计算其高斯曲率等，更新list中的各项曲率值

		//计算一个点的曲率
		if (0 < myVisual->vertexCnt&&myVisual->vertexCnt - myVisual->faceCnt <= myVisual->faceCnt)
		{
			int i = (*iter).index; 

			if (myVisual->normalCnt == 0)
			{

				myVisual->normal = (tVector *)malloc(myVisual->vertexCnt * sizeof(tVector));
				myVisual->normalCnt = -myVisual->vertexCnt;
				//originalVisual->normal = (tVector *)malloc(originalVisual->vertexCnt * sizeof(tVector));

			}

			//vertexOfFace Vertex4Normal[20];//定义面片顶点组(按顺序排列)，以计算顶点法向量
			GroupVertex.numOfGroup = 0;
			tempsize = 0;
			if (myVisual->normalCnt > 0)
			{
				for (int j=0;j<myVisual->vfindex[i].num;j++)
				{
					//myVisual->vfindex[i].findex[j] 面的索引index
					tempfindex = myVisual->vfindex[i].findex[j];
					temp = GroupVertex.numOfGroup;
					tempr = 0;
					if (i==myVisual->index[tempfindex].v[0])//找到了对应的面
					{
						GroupVertex.twoPoints[temp].p1 = myVisual->vertex[myVisual->index[tempfindex].v[1]];
						GroupVertex.twoPoints[temp].p2 = myVisual->vertex[myVisual->index[tempfindex].v[2]];

						GroupVertex.numOfGroup += 1;
						//ppindex.insert(myVisual->index[tempfindex].v[1]);
						//ppindex.insert(myVisual->index[tempfindex].v[2]);

						//判断是否邻域内

						tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[tempfindex].v[1]]);
						if (tempr <=R)
						{

							vp.index = myVisual->index[tempfindex].v[1];
							if (ppindex.insert(myVisual->index[tempfindex].v[1]).second)//!findListData(neighborIndex, vp.index))
							{
								//vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
						tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[tempfindex].v[2]]);
						if (tempr <=R)
						{
							vp.index = myVisual->index[tempfindex].v[2];
							if (ppindex.insert(myVisual->index[tempfindex].v[2]).second)//!findListData(neighborIndex, vp.index))
							{
								//vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}

					}
					else if (i == myVisual->index[tempfindex].v[1])//找到了对应的面
					{
						GroupVertex.twoPoints[temp].p1 = myVisual->vertex[myVisual->index[tempfindex].v[0]];
						GroupVertex.twoPoints[temp].p2 = myVisual->vertex[myVisual->index[tempfindex].v[2]];

						GroupVertex.numOfGroup += 1;
						//ppindex.insert(myVisual->index[tempfindex].v[0]);
						//ppindex.insert(myVisual->index[tempfindex].v[2]);

						//判断是否邻域内

						tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[tempfindex].v[0]]);
						if (tempr <= R)
						{

							vp.index = myVisual->index[tempfindex].v[0];
							if (ppindex.insert(myVisual->index[tempfindex].v[0]).second)//!findListData(neighborIndex, vp.index))
							{
								//vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
						tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[tempfindex].v[2]]);
						if (tempr <= R)
						{
							vp.index = myVisual->index[tempfindex].v[2];
							if (ppindex.insert(myVisual->index[tempfindex].v[2]).second)//!findListData(neighborIndex, vp.index))
							{
								//vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}

					}
					else if (i == myVisual->index[tempfindex].v[2])//找到了对应的面
					{
						GroupVertex.twoPoints[temp].p1 = myVisual->vertex[myVisual->index[tempfindex].v[0]];
						GroupVertex.twoPoints[temp].p2 = myVisual->vertex[myVisual->index[tempfindex].v[1]];

						GroupVertex.numOfGroup += 1;
						//ppindex.insert(myVisual->index[tempfindex].v[0]);
						//ppindex.insert(myVisual->index[tempfindex].v[1]);

						//判断是否邻域内

						tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[tempfindex].v[0]]);
						if (tempr <= R)
						{

							vp.index = myVisual->index[tempfindex].v[0];
							if (ppindex.insert(myVisual->index[tempfindex].v[0]).second)//!findListData(neighborIndex, vp.index))
							{
								//vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
						tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[tempfindex].v[1]]);
						if (tempr <= R)
						{
							vp.index = myVisual->index[tempfindex].v[1];
							if (ppindex.insert(myVisual->index[tempfindex].v[1]).second)//!findListData(neighborIndex, vp.index))
							{
								//vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}

					}
					else {
						AfxMessageBox("点到面的索引错误！");
					}
				}
			}
			else
			{
				tempr = 0;
				//循环遍历查找与顶点相同的点
				for (int j = 0; j<myVisual->vfindex[i].num; j++)
				{


					tempfindex = myVisual->vfindex[i].findex[j];
					temp = GroupVertex.numOfGroup;

					if (myVisual->index[tempfindex].v[0] == i)
					{
						myVisual->index[tempfindex].v[0] = i;
						GroupVertex.twoPoints[temp].p1 = myVisual->vertex[myVisual->index[tempfindex].v[1]];

						GroupVertex.twoPoints[temp].p2 = myVisual->vertex[myVisual->index[tempfindex].v[2]];

						Vertex4Normal[temp].p1 = myVisual->vertex[myVisual->index[tempfindex].v[0]];

						Vertex4Normal[temp].p2 = myVisual->vertex[myVisual->index[tempfindex].v[1]];

						Vertex4Normal[temp].p3 = myVisual->vertex[myVisual->index[tempfindex].v[2]];

						GroupVertex.numOfGroup += 1;
						//ppindex.insert(myVisual->index[tempfindex].v[1]);
						//ppindex.insert(myVisual->index[tempfindex].v[2]);


						tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[tempfindex].v[1]]);
						if (tempr<=R)
						{

							vp.index = myVisual->index[tempfindex].v[1];
							if (ppindex.insert(myVisual->index[tempfindex].v[1]).second)//!findListData(neighborIndex, vp.index))
							{
								//vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
						tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[tempfindex].v[2]]);
						if (tempr<=R)
						{

							vp.index = myVisual->index[tempfindex].v[2];
							if (ppindex.insert(myVisual->index[tempfindex].v[2]).second)//!findListData(neighborIndex, vp.index))
							{
								//vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}

					}
					else if (myVisual->index[tempfindex].v[1] == i)
					{
						myVisual->index[tempfindex].v[1] = i;
						GroupVertex.twoPoints[temp].p1 = myVisual->vertex[myVisual->index[tempfindex].v[0]];
						GroupVertex.twoPoints[temp].p2 = myVisual->vertex[myVisual->index[tempfindex].v[2]];


						Vertex4Normal[temp].p1 = myVisual->vertex[myVisual->index[tempfindex].v[0]];

						Vertex4Normal[temp].p2 = myVisual->vertex[myVisual->index[tempfindex].v[1]];

						Vertex4Normal[temp].p3 = myVisual->vertex[myVisual->index[tempfindex].v[2]];

						GroupVertex.numOfGroup += 1;
						//ppindex.insert(myVisual->index[tempfindex].v[0]);
						//ppindex.insert(myVisual->index[tempfindex].v[2]);


						tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[tempfindex].v[0]]);
						if (tempr<=R)
						{

							vp.index = myVisual->index[tempfindex].v[0];
							if (ppindex.insert(myVisual->index[tempfindex].v[0]).second)//!findListData(neighborIndex, vp.index))
							{
								//vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
						tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[tempfindex].v[2]]);
						if (tempr<=R)
						{

							vp.index = myVisual->index[tempfindex].v[2];
							if (ppindex.insert(myVisual->index[tempfindex].v[2]).second)//!findListData(neighborIndex, vp.index))
							{
								//vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
					}
					else if (myVisual->index[tempfindex].v[2] == i)
					{
						myVisual->index[tempfindex].v[2] = i;
						GroupVertex.twoPoints[temp].p1 = myVisual->vertex[myVisual->index[tempfindex].v[0]];
						GroupVertex.twoPoints[temp].p2 = myVisual->vertex[myVisual->index[tempfindex].v[1]];


						Vertex4Normal[temp].p1 = myVisual->vertex[myVisual->index[tempfindex].v[0]];

						Vertex4Normal[temp].p2 = myVisual->vertex[myVisual->index[tempfindex].v[1]];

						Vertex4Normal[temp].p3 = myVisual->vertex[myVisual->index[tempfindex].v[2]];

						GroupVertex.numOfGroup += 1;
						//ppindex.insert(myVisual->index[tempfindex].v[0]);
						//ppindex.insert(myVisual->index[tempfindex].v[1]);


						tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[tempfindex].v[0]]);
						if (tempr<=R)
						{

							vp.index = myVisual->index[tempfindex].v[0];
							if (ppindex.insert(myVisual->index[tempfindex].v[0]).second)//!findListData(neighborIndex, vp.index))
							{
								//vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
						tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[tempfindex].v[1]]);
						if (tempr <= R)
						{

							vp.index = myVisual->index[tempfindex].v[1];
							if (ppindex.insert(myVisual->index[tempfindex].v[1]).second)//!findListData(neighborIndex, vp.index))
							{
								//vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
					}
					else
					{
						AfxMessageBox("点到面的索引错误");
					}
				}

			}



			if (GroupVertex.numOfGroup <= 2)
			{
				tempsize = ppindex.size();
				if (findListData(neighborIndex, vp.index))
				{
					return FALSE;
				}
				//myVisual->vertex[i].z = myVisual->vertexMax_z + 2;
				/*	if (GroupVertex.numOfGroup - tempsize < 0)
				{
				*isedge = 1;
				}
				else
				*isedge = 0;*/
				curvature = 0.0;
				(*iter).Gvalue = (*iter).Hvalue = (*iter).k1 = (*iter).k2 = 0.0;
				return FALSE;//边缘点或者洞边缘
			}

			//tempsize = ppindex.size();
			if (GroupVertex.numOfGroup - tempsize < 0)
			{
				return FALSE;//边缘点
			}
			//计算高斯曲率
			sumAngle = 0.0;
			sumArea = 0.0;
			tVector *facevector = new tVector[GroupVertex.numOfGroup];

			//判断是否需要计算顶点法向量
			if (myVisual->normalCnt <= 0)
			{
				myVisual->normal[i].x = 0;
				myVisual->normal[i].y = 0;
				myVisual->normal[i].z = 0;
				for (int m = 0; m < GroupVertex.numOfGroup; m++)
				{
					tVector tempnormal;

					myVisual->normal[i] += CountNormalofFace(Vertex4Normal[m].p1, Vertex4Normal[m].p2, Vertex4Normal[m].p3);

				}
				originalVisual->normal[i] = myVisual->normal[i];
			}
			//tVector *normalOfFace=(tVector *)malloc(myVisual->GroupVertex[i].numOfGroup*sizeof(tVector));
			for (int m = 0; m < GroupVertex.numOfGroup; m++)
			{
				t_Triangle tempTriangle = CountTangle(myVisual->vertex[i], GroupVertex.twoPoints[m].p1, GroupVertex.twoPoints[m].p2);
				sumAngle += tempTriangle.angle;
				sumArea += tempTriangle.area;
				facevector[m] = tempTriangle.FaceVector;
			}
			//计算平均曲率
			sumHcurvature = 0;//未除面积的平均曲率
			MAKEVECTOR(sumHVector, 0, 0, 0);
			for (int m = 0; m < GroupVertex.numOfGroup; m++)
			{
				MAKEVECTOR(edge1, 0, 0, 0); MAKEVECTOR(edge2, 0, 0, 0); MAKEVECTOR(edge3, 0, 0, 0); MAKEVECTOR(edge4, 0, 0, 0);
				//tVector normal1,normal2;
				 normalangle = 0.0, normalangle1 = 0.0, normalangle2 = 0.0;
				 edgelenth = 0.0;
				 tempH = 0;
				//edge1 = GroupVertex.twoPoints[m].p1 - myVisual->vertex[i];
				edge1 = GroupVertex.twoPoints[m].p1 - myVisual->vertex[i];
				edge2 = GroupVertex.twoPoints[m].p2 - myVisual->vertex[i];
				for (int mm = 0; mm < GroupVertex.numOfGroup; mm++)
				{
					if (mm != m)//忽略同一条边
					{
						//利用当前面和这个面的法向量n1、n2求出夹角，算出部分曲率H^
						edge3 = GroupVertex.twoPoints[mm].p1 - myVisual->vertex[i];
						edge4 = GroupVertex.twoPoints[mm].p2 - myVisual->vertex[i];
						//判断共边
						if (edge1.x == edge3.x&&edge1.y == edge3.y&&edge1.z == edge3.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角,Dyn的方法
							normalangle1 = CountTangle(edge2, edge2 - edge1);//Meyer的方法
							normalangle2 = CountTangle(edge4, edge4 - edge3);
							//edgelenth = CountVecLenth(edge1);//计算共边的长度
															 //sumHcurvature += normalangle*edgelenth;//Dyn的方法
							tempH = 1 / tan(normalangle1) + 1 / tan(normalangle2);
							MAKEVECTOR(sumHVector, sumHVector.x + tempH*edge1.x,
								sumHVector.y + tempH*edge1.y,
								sumHVector.z + tempH*edge1.z);


						}
						else if (edge1.x == edge4.x&&edge1.y == edge4.y&&edge1.z == edge4.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
							normalangle1 = CountTangle(edge2, edge2 - edge1);//Meyer的方法
							normalangle2 = CountTangle(edge3, edge3 - edge4);
							//edgelenth = CountVecLenth(edge1);//计算共边的长度
															 //sumHcurvature += normalangle*edgelenth;
							tempH = 1 / tan(normalangle1) + 1 / tan(normalangle2);
							MAKEVECTOR(sumHVector, sumHVector.x + tempH*edge1.x,
								sumHVector.y + tempH*edge1.y,
								sumHVector.z + tempH*edge1.z);


						}
						else if (edge2.x == edge4.x&&edge2.y == edge4.y&&edge2.z == edge4.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
							normalangle1 = CountTangle(edge1, edge1 - edge2);//Meyer的方法
							normalangle2 = CountTangle(edge3, edge3 - edge4);
							//edgelenth = CountVecLenth(edge2);//计算共边的长度
															 //sumHcurvature += normalangle*edgelenth;
							tempH = 1 / tan(normalangle1) + 1 / tan(normalangle2);
							MAKEVECTOR(sumHVector, sumHVector.x + tempH*edge2.x,
								sumHVector.y + tempH*edge2.y,
								sumHVector.z + tempH*edge2.z);


						}
						else if (edge2.x == edge3.x&&edge2.y == edge3.y&&edge2.z == edge3.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
							normalangle1 = CountTangle(edge1, edge1 - edge2);//Meyer的方法
							normalangle2 = CountTangle(edge4, edge4 - edge3);
							//edgelenth = CountVecLenth(edge2);//计算共边的长度
															 //sumHcurvature += normalangle*edgelenth;
							tempH = 1 / tan(normalangle1) + 1 / tan(normalangle2);
							MAKEVECTOR(sumHVector, sumHVector.x + tempH*edge2.x,
								sumHVector.y + tempH*edge2.y,
								sumHVector.z + tempH*edge2.z);


						}
						if (_isnan(sumHcurvature))
						{
							AfxMessageBox("sumHcurvature is NaN.");
						}

					}

				}
			}

			delete[] facevector;
			if (sumArea > 0)
			{
				n1 << sumHVector.x, sumHVector.y, sumHVector.z;
				n2 << originalVisual->normal[i].x, originalVisual->normal[i].y, originalVisual->normal[i].z;
				n2.normalize();
				originalVisual->HCurvature[i] = 0.25*(n1.dot(n2)) / sumArea;
				curvature = (2 * 3.1415926 - sumAngle) / sumArea;
				if (curvature> 0)
				{
					int tempGaussNum = 0;
					for (int m = 0; m < GroupVertex.numOfGroup; m++)
					{
						//求顶点邻边是否与法向量锐角
						double MultiVector1 = myVisual->normal[i].x*(GroupVertex.twoPoints[m].p1.x - myVisual->vertex[i].x)
							+ myVisual->normal[i].y*(GroupVertex.twoPoints[m].p1.y - myVisual->vertex[i].y)
							+ myVisual->normal[i].z*(GroupVertex.twoPoints[m].p1.z - myVisual->vertex[i].z);
						double MultiVector2 = myVisual->normal[i].x*(GroupVertex.twoPoints[m].p2.x - myVisual->vertex[i].x)
							+ myVisual->normal[i].y*(GroupVertex.twoPoints[m].p2.y - myVisual->vertex[i].y)
							+ myVisual->normal[i].z*(GroupVertex.twoPoints[m].p2.z - myVisual->vertex[i].z);
						if (MultiVector1 >= 0 && MultiVector2 >= 0)
						{
							tempGaussNum++;
						}

					}
					if (tempGaussNum >= GroupVertex.numOfGroup - 2)
					{

						curvature = -curvature;
					}
				}
				else
				{
					//curvature = 0;//马鞍曲面
				}

			}
			else {

				originalVisual->HCurvature[i] = 0;
				curvature = 1.0;
			}

			/*if (myVisual->normalCnt <= 0)
			{
				free(myVisual->normal);
				myVisual->normal = NULL;
			}*/
			(*iter).Gvalue = curvature;
			(*iter).Hvalue = originalVisual->HCurvature[i];
			//计算主曲率
			double tempCur = originalVisual->HCurvature[i] * originalVisual->HCurvature[i] - curvature;
			if (tempCur>=0)
			{
				(*iter).k1 = originalVisual->HCurvature[i] - sqrt(tempCur);
				(*iter).k2 = originalVisual->HCurvature[i] + sqrt(tempCur);
			}
			else {
				(*iter).k1 = 0;
				(*iter).k2 = 0;
				//AfxMessageBox("主曲率错误");
			}
		}
		//针对kinect OBJ模型 默认有顶点向量****需要修改yingxiang
		if (0<myVisual->vertexCnt&&myVisual->vertexCnt - myVisual->faceCnt>myVisual->faceCnt)
		{
			//循环遍历顶点
			int i = (*iter).index;

			//vertexsOfonevertex GroupVertex;//定义面片顶点组
			GroupVertex.numOfGroup = 0;
			tempsize = 0;
			//循环遍历查找与顶点重复的点
			for (int j = 0; j<myVisual->vertexCnt - i - 1; j++)
			{
				//判断下一个点是否已经标记，若标记则进行下次循环
				if (GroupVertex.numOfGroup>19)
				{
					break;
				}
				int sumji = j + i;//当前点的位置的j个位移


								  //从当前顶点位置开始查找判断，以便将当前顶点收入组中
				if (myVisual->vertex[sumji].x == myVisual->vertex[i].x&&
					myVisual->vertex[sumji].y == myVisual->vertex[i].y&&
					myVisual->vertex[sumji].z == myVisual->vertex[i].z)
				{


					//遍历面片，将面片的顶点索引修改为所有重复顶点的第一个顶点的索引,将同一顶点面片的其他点信息收入组中
					tempr = 0.0;
					temp = GroupVertex.numOfGroup;
					int tempvsite = (sumji) / 3;
					int tempvsiteyu = (sumji) % 3;//此设计完全是按照kinectOBJ模型数据结构
					if (myVisual->index[tempvsite].v[0] == sumji)
					{
						myVisual->index[tempvsite].v[0] = i;
						GroupVertex.twoPoints[temp].p1 = myVisual->vertex[myVisual->index[tempvsite].v[1]];
						GroupVertex.twoPoints[temp].p2 = myVisual->vertex[myVisual->index[tempvsite].v[2]];
						GroupVertex.numOfGroup += 1;
						ppindex.insert(myVisual->index[tempvsite].v[1]);
						ppindex.insert(myVisual->index[tempvsite].v[2]);

						//判断是否邻域内

						if (isNeighbourhood(myVisual->vertex[pindex], myVisual->vertex[myVisual->index[j].v[1]], R))
						{
							tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[j].v[1]]);
							vp.index = myVisual->index[j].v[1];
							if (!findListData(neighborIndex, vp.index))
							{
								vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
						if (isNeighbourhood(myVisual->vertex[pindex], myVisual->vertex[myVisual->index[j].v[2]], R))
						{
							tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[j].v[2]]);
							vp.index = myVisual->index[j].v[2];
							if (!findListData(neighborIndex, vp.index))
							{
								vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}

					}
					else if (myVisual->index[tempvsite].v[1] == sumji)
					{
						myVisual->index[tempvsite].v[1] = i;
						GroupVertex.twoPoints[temp].p1 = myVisual->vertex[myVisual->index[tempvsite].v[0]];
						GroupVertex.twoPoints[temp].p2 = myVisual->vertex[myVisual->index[tempvsite].v[2]];
						GroupVertex.numOfGroup += 1;
						ppindex.insert(myVisual->index[tempvsite].v[0]);
						ppindex.insert(myVisual->index[tempvsite].v[2]);
						//判断是否邻域内

						if (isNeighbourhood(myVisual->vertex[pindex], myVisual->vertex[myVisual->index[j].v[0]], R))
						{
							tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[j].v[0]]);
							vp.index = myVisual->index[j].v[0];
							if (!findListData(neighborIndex, vp.index))
							{
								vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
						if (isNeighbourhood(myVisual->vertex[pindex], myVisual->vertex[myVisual->index[j].v[2]], R))
						{
							tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[j].v[2]]);
							vp.index = myVisual->index[j].v[2];
							if (!findListData(neighborIndex, vp.index))
							{
								vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
					}
					else if (myVisual->index[tempvsite].v[2] == sumji)
					{
						myVisual->index[tempvsite].v[2] = i;
						GroupVertex.twoPoints[temp].p1 = myVisual->vertex[myVisual->index[tempvsite].v[0]];
						GroupVertex.twoPoints[temp].p2 = myVisual->vertex[myVisual->index[tempvsite].v[1]];
						GroupVertex.numOfGroup += 1;
						ppindex.insert(myVisual->index[tempvsite].v[0]);
						ppindex.insert(myVisual->index[tempvsite].v[1]);
						//判断是否邻域内

						if (isNeighbourhood(myVisual->vertex[pindex], myVisual->vertex[myVisual->index[j].v[0]], R))
						{
							tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[j].v[0]]);
							vp.index = myVisual->index[j].v[0];
							if (!findListData(neighborIndex, vp.index))
							{
								vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
						if (isNeighbourhood(myVisual->vertex[pindex], myVisual->vertex[myVisual->index[j].v[1]], R))
						{
							tempr = CountVecLenth(myVisual->vertex[pindex] - myVisual->vertex[myVisual->index[j].v[1]]);
							vp.index = myVisual->index[j].v[1];
							if (!findListData(neighborIndex, vp.index))
							{
								vp.r = 1 + 10 * tempr / R;
								neighborIndex.push_back(vp); tempsize++;
							}
						}
					}
				}
			}
			if (GroupVertex.numOfGroup <= 2)
			{
				//myVisual->vertex[i].z = myVisual->vertexMax_z + 2;
				tempsize = ppindex.size();
				//if (GroupVertex.numOfGroup - tempsize < 0)
				//{
				//	*isedge = 1;
				//}
				//else
				//	*isedge = 0;
				curvature = 0.0;
				/*if (filest)
				{
				filest << originalVisual->GuassCurvature[i] << endl;
				}*/

				(*iter).Gvalue = (*iter).Hvalue = (*iter).k1 = (*iter).k2 = 0.0;
				return FALSE;
			}

			tempsize = ppindex.size();
			if (GroupVertex.numOfGroup - tempsize < 0)
			{
				return FALSE;//边缘点
			}
			//计算高斯曲率
			double sumAngle = 0.0;
			double sumArea = 0.0;
			tVector *facevector = new tVector[GroupVertex.numOfGroup];
			//tVector *normalOfFace=(tVector *)malloc(myVisual->GroupVertex[i].numOfGroup*sizeof(tVector));
			//for (int m = 0; m < myVisual->GroupVertex[i].numOfGroup; i++)
			//{
			//	normalOfFace[m] = CountNormalofFace(myVisual->vertex[i], myVisual->GroupVertex[i].twoPoints[m].p1, myVisual->GroupVertex[i].twoPoints[m].p2);
			//	/*t_Triangle tempTriangle = CountTangle(myVisual->vertex[i], myVisual->GroupVertex[i].twoPoints[m].p1, myVisual->GroupVertex[i].twoPoints[m].p2);
			//	sumAngle += tempTriangle.angle;
			//	sumArea += tempTriangle.area;*/
			//}
			//计算高斯曲率参数 1、夹角和 2、voronoi面积
			for (int m = 0; m < GroupVertex.numOfGroup; m++)
			{
				t_Triangle tempTriangle = CountTangle(myVisual->vertex[i], GroupVertex.twoPoints[m].p1, GroupVertex.twoPoints[m].p2);
				sumAngle += tempTriangle.angle;
				sumArea += tempTriangle.area;
				facevector[m].x = tempTriangle.FaceVector.x;
				facevector[m].y = tempTriangle.FaceVector.y;
				facevector[m].z = tempTriangle.FaceVector.z;
			}
			//计算平均曲率
			double sumHcurvature = 0;//未除面积的平均曲率

			for (int m = 0; m < GroupVertex.numOfGroup; m++)
			{
				tVector edge1, edge2, edge3, edge4;
				//tVector normal1,normal2;
				double normalangle = 0.0, normalangle1 = 0.0, normalangle2 = 0.0;
				double edgelenth = 0.0;


				edge1.x = GroupVertex.twoPoints[m].p1.x - myVisual->vertex[i].x;
				edge1.y = GroupVertex.twoPoints[m].p1.y - myVisual->vertex[i].y;
				edge1.z = GroupVertex.twoPoints[m].p1.z - myVisual->vertex[i].z;
				edge2.x = GroupVertex.twoPoints[m].p2.x - myVisual->vertex[i].x;
				edge2.y = GroupVertex.twoPoints[m].p2.y - myVisual->vertex[i].y;
				edge2.z = GroupVertex.twoPoints[m].p2.z - myVisual->vertex[i].z;
				for (int mm = 0; mm < GroupVertex.numOfGroup; mm++)
				{
					if (mm != m)//忽略同一条边
					{
						//利用当前面和这个面的法向量n1、n2求出夹角，算出部分曲率H^
						edge3.x = GroupVertex.twoPoints[mm].p1.x - myVisual->vertex[i].x;
						edge3.y = GroupVertex.twoPoints[mm].p1.y - myVisual->vertex[i].y;
						edge3.z = GroupVertex.twoPoints[mm].p1.z - myVisual->vertex[i].z;
						edge4.x = GroupVertex.twoPoints[mm].p2.x - myVisual->vertex[i].x;
						edge4.y = GroupVertex.twoPoints[mm].p2.y - myVisual->vertex[i].y;
						edge4.z = GroupVertex.twoPoints[mm].p2.z - myVisual->vertex[i].z;
						//判断共边
						if (edge1.x == edge3.x&&edge1.y == edge3.y&&edge1.z == edge3.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角,Dyn的方法
							normalangle1 = CountTangle(edge2, edge2 - edge1);//Meyer的方法
							normalangle2 = CountTangle(edge4, edge4 - edge3);
							edgelenth = CountVecLenth(edge1);//计算共边的长度
															 //sumHcurvature += normalangle*edgelenth;//Dyn的方法
							sumHcurvature += 1 / (tan(normalangle1)*tan(normalangle2))*edgelenth;


						}
						else if (edge1.x == edge4.x&&edge1.y == edge4.y&&edge1.z == edge4.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
							normalangle1 = CountTangle(edge2, edge2 - edge1);//Meyer的方法
							normalangle2 = CountTangle(edge3, edge3 - edge4);
							edgelenth = CountVecLenth(edge1);//计算共边的长度
															 //sumHcurvature += normalangle*edgelenth;
							sumHcurvature += 1 / (tan(normalangle1)*tan(normalangle2))*edgelenth;


						}
						else if (edge2.x == edge4.x&&edge2.y == edge4.y&&edge2.z == edge4.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
							normalangle1 = CountTangle(edge1, edge1 - edge2);//Meyer的方法
							normalangle2 = CountTangle(edge3, edge3 - edge4);
							edgelenth = CountVecLenth(edge2);//计算共边的长度
															 //sumHcurvature += normalangle*edgelenth;
							sumHcurvature += 1 / (tan(normalangle1)*tan(normalangle2))*edgelenth;


						}
						else if (edge2.x == edge3.x&&edge2.y == edge3.y&&edge2.z == edge3.z)
						{
							//normalangle = CountTangle(facevector[m], facevector[mm]);//计算法向量夹角
							normalangle1 = CountTangle(edge1, edge1 - edge2);//Meyer的方法
							normalangle2 = CountTangle(edge4, edge4 - edge3);
							edgelenth = CountVecLenth(edge2);//计算共边的长度
															 //sumHcurvature += normalangle*edgelenth;
							sumHcurvature += 1 / (tan(normalangle1)*tan(normalangle2))*edgelenth;


						}

					}

				}
			}
			delete[] facevector;
			if (sumArea > 0)
			{
				originalVisual->HCurvature[i] = 0.25*sumHcurvature / sumArea;
				curvature = (2 * 3.1415926 - sumAngle) / sumArea;
				if (curvature>0)
				{
					int tempGaussNum = 0;
					for (int m = 0; m < GroupVertex.numOfGroup; m++)
					{
						//求顶点邻边是否与法向量锐角
						double MultiVector1 = myVisual->normal[i].x*(GroupVertex.twoPoints[m].p1.x - myVisual->vertex[i].x)
							+ myVisual->normal[i].y*(GroupVertex.twoPoints[m].p1.y - myVisual->vertex[i].y)
							+ myVisual->normal[i].z*(GroupVertex.twoPoints[m].p1.z - myVisual->vertex[i].z);
						double MultiVector2 = myVisual->normal[i].x*(GroupVertex.twoPoints[m].p2.x - myVisual->vertex[i].x)
							+ myVisual->normal[i].y*(GroupVertex.twoPoints[m].p2.y - myVisual->vertex[i].y)
							+ myVisual->normal[i].z*(GroupVertex.twoPoints[m].p2.z - myVisual->vertex[i].z);
						if (MultiVector1 >= 0 && MultiVector2 >= 0)
						{
							tempGaussNum++;
						}

					}
					if (tempGaussNum >= GroupVertex.numOfGroup - 2)
					{

						curvature = -curvature;
					}
				}
				else
				{
					//curvature = 0;//马鞍曲面
				}

			}
			else {

				originalVisual->HCurvature[i] = 0;
				curvature = 1.0;
			}
			(*iter).Gvalue = curvature;
			(*iter).Hvalue = originalVisual->HCurvature[i];
			double tempCur = originalVisual->HCurvature[i] * originalVisual->HCurvature[i] - curvature;
			if (tempCur >= 0)
			{
				(*iter).k1 = originalVisual->HCurvature[i] - sqrt(tempCur);
				(*iter).k2 = originalVisual->HCurvature[i] + sqrt(tempCur);

			}
			else {
				(*iter).k1 = 0;
				(*iter).k2 = 0;
				//AfxMessageBox("主曲率错误");
			}


		}
		//



	}


	
	return TRUE;


}
void pushMinDisPindex(double *mindist)
{

	
}
/************************************************************************/
/* 功能：NZCC（归一化相互关系数），判断邻域相似度
参数：targetVisual：目标模型；sourceVisual：源模型；tindex：目标点的索引；sindex：源模型点的索引；
*/
/************************************************************************/
double Registration::NZCC(t_Visual *targetVisual, t_Visual *sourceVisual, int tindex, int sindex, double tr, double sr)
{
	//思路：
	//分别给定目标模型和源模型以及匹配点的索引，计算邻域（gauss hcurvature k1 k2 ）
	//计算主曲率的均值，然后计算nzcc1、nzcc2，NZCC=nzcc1+nzcc2；
	//设置阈值 ***********************半径未确定****************************
	//邻域半径的确定依据：中心点到其他特征点的距离
	//半径的确定应该先过滤特征点
	fstream fs;
	fs.open("record003.txt", ios::app | ios::out);
	long loop;
	int tempsize = 0;
	double nzcc1 = 0.0, nzcc2 = 0.0;
	double  tempMaxtr = 0, tempMaxsr = 0;//记录邻域半径,去掉距离最远的
	double maxCurvaturek1, minCurvaturek1, maxCurvaturek2, minCurvaturek2;//最大最小
	double etk1=0.0, etk2=0.0, esk1=0.0, esk2=0.0,etg=0.0,esg=0.0;//邻域主曲率、高斯曲率的平均值
	double tempetk1 = 0.0, tempetk2 = 0.0, tempesk1 = 0.0, tempesk2 = 0.0, tempetg = 0.0, tempesg = 0.0;//临时变量

	double nzcc11 = 0.0, nzcc121 = 0.0, nzcc122 = 0.0, nzcc21 = 0.0, nzcc221 = 0.0, nzcc222 = 0.0;//nzcc的分子，分母
	double nzgg11 = 0.0, nzgg12 = 0.0, nzgg21 = 0.0, nzgg22 = 0.0;

	if (isnan(tr)||isinf(tr))
	{
		AfxMessageBox("半径为inf、nan");
	}
	
	list<vertexCurvature > tpNeighborIndex,spNeighborIndex,temptpNeighborIndex;//记录邻域的曲率信息
	//计算邻域的曲率信息
	time_t starT = clock();
	if (!NeibGuassCurvature(targetVisual, tpNeighborIndex, tindex, tr)
		|| !NeibGuassCurvature(sourceVisual, spNeighborIndex, sindex, tr))
	{
		return -3;
	}
	time_t endT = clock();
	char str1[256];
	//sprintf(str1, "%lf", (double)(endT - starT) / CLOCKS_PER_SEC);
	//AfxMessageBox(str1);
	//starT = clock();
	//计算k1,k2的平均值
	list<vertexCurvature >::iterator itt, its,ittemp;
	//k1,k2的平均值取最大值和最小值的平均，不知为啥不用所有值的均值
	//求平均值，和除以个数
	/*for (int i=10;i<=10;i++)
	{*/
		nzcc11 = 0.0, nzcc121 = 0.0, nzcc122 = 0.0, nzcc21 = 0.0, nzcc221 = 0.0, nzcc222 = 0.0; 
		for (itt = tpNeighborIndex.begin(); itt != tpNeighborIndex.end(); itt++)
		{
			//if (itt == tpNeighborIndex.begin())
			//{
			//	maxCurvaturek1 = (*itt).k1;
			//	minCurvaturek1 = (*itt).k1;
			//	maxCurvaturek2 = (*itt).k2;
			//	minCurvaturek2 = (*itt).k2;
			//}
			//if ((*itt).k1 > maxCurvaturek1)
			//{
			//	maxCurvaturek1 = (*itt).k1;//赋大值
			//}
			//else if ((*itt).k1 < minCurvaturek1)
			//{
			//	minCurvaturek1 = (*itt).k1;//赋小值
			//}
			//if ((*itt).k2 > maxCurvaturek2)
			//{
			//	maxCurvaturek2 = (*itt).k2;//赋大值
			//}
			//else if ((*itt).k2 < minCurvaturek2)
			//{
			//	minCurvaturek2 = (*itt).k2;//赋小值
			//}
			//

			//sum etk1 etk2
			etk1 += (*itt).k1;
			etk2 += (*itt).k2;
			etg += (*itt).Gvalue;
		}
		//etk1 = (maxCurvaturek1 + minCurvaturek1) / 2;
		//etk2 = (maxCurvaturek2 + minCurvaturek2) / 2;
		tempetk1 = etk1; //= etk1 / tpNeighborIndex.size();
		tempetk2 = etk2;// = etk2 / tpNeighborIndex.size();
		tempetg = etg; //= etg / tpNeighborIndex.size();
		for (its = spNeighborIndex.begin(); its != spNeighborIndex.end(); its++)
		{
			//if (its == spNeighborIndex.begin())
			//{
			//	maxCurvaturek1 = (*its).k1;
			//	minCurvaturek1 = (*its).k1;
			//	maxCurvaturek2 = (*its).k2;
			//	minCurvaturek2 = (*its).k2;
			//}
			//if ((*its).k1 > maxCurvaturek1)
			//{
			//	maxCurvaturek1 = (*its).k1;//赋大值
			//}
			//else if ((*its).k1 < minCurvaturek1)
			//{
			//	minCurvaturek1 = (*its).k1;//赋小值
			//}
			//if ((*its).k2 > maxCurvaturek2)
			//{
			//	maxCurvaturek2 = (*its).k2;//赋大值
			//}
			//else if ((*its).k2 < minCurvaturek2)
			//{
			//	minCurvaturek2 = (*its).k2;//赋小值
			//}

			//sum esk1 esk2
			esk1 += (*its).k1;
			esk2 += (*its).k2;
			esg += (*its).Gvalue;
		}
		//esk1 = (maxCurvaturek1 + minCurvaturek1) / 2;
		//esk2 = (maxCurvaturek2 + minCurvaturek2) / 2;
		//计算nzcc
		tempesk1=esk1 = esk1 / spNeighborIndex.size();
		tempesk2=esk2 = esk2 / spNeighborIndex.size();
		tempesg =esg = esg / spNeighborIndex.size();
		//必须确定邻域内的点是对应的才能利用NZCC公式计算
		//先计算平移向量在计算旋转矩阵对邻域内的点进行变换，然后找最近点匹配，匹配之后要做标记。
		//开辟内存存储邻域点
		mechIndex *NeiMetchIndex=new mechIndex[tpNeighborIndex.size()];
		tVector *tNeiPoints = new tVector[tpNeighborIndex.size()];
		tVector *sNeiPoints = new tVector[spNeighborIndex.size()];
		tVector transVector = targetVisual->vertex[tindex] - sourceVisual->vertex[sindex];//平移向量
		for (itt=tpNeighborIndex.begin(),loop=0;itt!=tpNeighborIndex.end();itt++)
		{
			tNeiPoints[loop] = targetVisual->vertex[(*itt).index];
			loop++;
		}
		//2.计算旋转矩阵，需要中心点的法向量n1  n2，计算旋转轴和旋转角度
		//n1		targetVisual->normal[tindex];	n2			sourceVisual->normal[sindex];
		Eigen::Vector3d tnormal, snormal;//两点的法向量
		tnormal << targetVisual->normal[tindex].x , targetVisual->normal[tindex].y , targetVisual->normal[tindex].z;
		snormal << sourceVisual->normal[sindex].x , sourceVisual->normal[sindex].y , sourceVisual->normal[sindex].z;
		double theta = CountTangle(targetVisual->normal[tindex], sourceVisual->normal[sindex]);
		Eigen::Vector3d  A;//旋转向量
		A = snormal.cross(tnormal);//向量叉乘，既向目标模型上的点旋转： snormal------> tnormal
		A.normalize();//单位化
		Eigen::AngleAxisd Rtmatrix(theta, A);//以A为旋转轴，旋转theta度
		//实现旋转：

		//平移旋转平移 得到了变换后的点。为每一个邻域内的点找到目标邻域内的一个匹配点。
		for (its=spNeighborIndex.begin(),loop=0;its!=spNeighborIndex.end();its++)
		{
			sNeiPoints[loop] = sourceVisual->vertex[(*its).index];
			sNeiPoints[loop] += transVector;
			sNeiPoints[loop] = sNeiPoints[loop]-targetVisual->vertex[tindex];//平移至原点
			
			snormal << sNeiPoints[loop].x, sNeiPoints[loop].y, sNeiPoints[loop].z;
			snormal=Rtmatrix.matrix()*snormal;//旋转
			sNeiPoints[loop].x = snormal(0);sNeiPoints[loop].y = snormal(1);sNeiPoints[loop].z = snormal(2);
			sNeiPoints[loop] += targetVisual->vertex[tindex];//平移回来

			double tempSum,tempMinsum= tr*tr;
			int tempIndex = -1;
			for (itt=tpNeighborIndex.begin();itt!=tpNeighborIndex.end();itt++)
			{
				tempSum = (sNeiPoints[loop].x - targetVisual->vertex[(*itt).index].x)*(sNeiPoints[loop].x - targetVisual->vertex[(*itt).index].x)
					+ (sNeiPoints[loop].y - targetVisual->vertex[(*itt).index].y)*(sNeiPoints[loop].y - targetVisual->vertex[(*itt).index].y)
					+ (sNeiPoints[loop].z - targetVisual->vertex[(*itt).index].z)*(sNeiPoints[loop].z - targetVisual->vertex[(*itt).index].z);
				
				if (tempMinsum>tempSum)
				{
					tempMinsum = tempSum;
					tempIndex = (*itt).index;
					ittemp = itt;
					//(*itt).r = -(*itt).r-0.0001;//此处将该点的所属半径作为标识符，注意，此后使用该点的r值需要先取其绝对值。
				}
			}
			if (tempIndex>0&& tempMinsum!=tr*tr)
			{
				//从etk1、esk1、etk2、esk2的临时变量中减去匹配的点的k1\k2值
				tempetk1 -= (*ittemp).k1;
				tempetk2 -= (*ittemp).k2;
				tempetg -= (*ittemp).Gvalue;
				temptpNeighborIndex.push_back((*ittemp));//add yingxiang
				//fs <<(*its).index<<"\t "<< (*ittemp).Hvalue << " \t \t" << (*its).Hvalue << endl;
				/*fs << (*its).index << "\ttk1\t" << (*ittemp).k1 << "\tetk1\t" << etk1 << "\t*sk1\t" << (*its).k1 << "\tesk1\t" << esk1*/
				//fs << (*its).index << "\ttk2\t" << (*ittemp).k2 << "\t*sk2" << (*its).k2 <<"距离：  "<< tempMinsum << endl;
				//fs << (*its).index << "\t"<< (*ittemp).k2 - etk2 << "\t\t" << (*its).k2 - esk2 <<"\tbanjing:\t"<< tempMinsum<< endl;
			}

			loop++;

		}
		

		etk1 -= tempetk1;
		etk2 -= tempetk2;
		etg -= tempetg;//减去了未匹配的邻域内点的值

		etk1 = etk1 / temptpNeighborIndex.size();
		etk2 = etk2 / temptpNeighborIndex.size();
		etg = etg / temptpNeighborIndex.size();
		//fs <<  "\tetk2\t" << etk2 << "\tesk2\t" << esk2  << endl;

		if (spNeighborIndex.size()==temptpNeighborIndex.size())
		{
			for (its = spNeighborIndex.begin(), ittemp = temptpNeighborIndex.begin(); its != spNeighborIndex.end() && ittemp != temptpNeighborIndex.end(); its++, ittemp++)
			{
				nzcc11 += ((*ittemp).k1 - etk1)*((*its).k1 - esk1);
				nzcc121 += ((*ittemp).k1 - etk1)*((*ittemp).k1 - etk1);
				nzcc122 += ((*its).k1 - esk1)*((*its).k1 - esk1);

				nzcc21 += ((*ittemp).k2 - etk2)*((*its).k2 - esk2);
				nzcc221 += ((*ittemp).k2 - etk2)*((*ittemp).k2 - etk2);
				nzcc222 += ((*its).k2 - esk2)*((*its).k2 - esk2);

				nzgg11 += ((*ittemp).Gvalue - etg)*((*its).Gvalue - esg);
				nzgg12 += ((*ittemp).Gvalue - etg)*((*ittemp).Gvalue - etg);
				nzgg21 += ((*its).Gvalue - esg)*((*its).Gvalue - esg);
			}
		}
		else
		{
			AfxMessageBox("邻域内的点未匹配完整");
		}
		if (tpNeighborIndex.front().index== 30841)
		{
			targetVisual->tNeiPoints = (tVector *)malloc(temptpNeighborIndex.size() * sizeof(tVector));
			targetVisual->sNeiPoints = (tVector *)malloc(temptpNeighborIndex.size() * sizeof(tVector));
			for (int i=0;i<temptpNeighborIndex.size();i++)
			{
				targetVisual->tNeiPoints[i] = tNeiPoints[i];
				targetVisual->sNeiPoints[i] = sNeiPoints[i];
			}
		}
		//fs << "*********************************************************************************************************" << endl;
		delete []tNeiPoints;
		delete []sNeiPoints;
		delete []NeiMetchIndex;

	
		/*for (itt = tpNeighborIndex.begin(), its = spNeighborIndex.begin();
			itt != tpNeighborIndex.end()  && its != spNeighborIndex.end() ; itt++, its++)
		{
			nzcc11 += ((*itt).k1 - etk1)*((*its).k1 - esk1);
			nzcc121 += ((*itt).k1 - etk1)*((*itt).k1 - etk1);
			nzcc122 += ((*its).k1 - esk1)*((*its).k1 - esk1);

			nzcc21 += ((*itt).k2 - etk2)*((*its).k2 - esk2);
			nzcc221 += ((*itt).k2 - etk2)*((*itt).k2 - etk2);
			nzcc222 += ((*its).k2 - esk2)*((*its).k2 - esk2); 

		}*/
		nzcc1 = nzcc11 / sqrt(nzcc121*nzcc122);
		nzcc2 = nzcc21 / sqrt(nzcc221*nzcc222);
		nzgg22 = nzgg11 / sqrt(nzgg12*nzgg21);
		if (fs)
		{
			fs <<nzgg22<<"\t"<< nzcc1 << "\t" << nzcc2 << "\t~~~~" << nzcc1 + nzcc2 << "\t^^^^^^" << tindex << "\t^^^^^^" << sindex << "\t半径：" << tr << "\t" << sr << "\t size " << tempsize <<"-----"<< endl;
		}
	//}
		endT = clock();

		//sprintf(str1, "%lf", (double)(endT - starT) / CLOCKS_PER_SEC);
		//AfxMessageBox(str1);
	fs.close();
	return nzcc1 + nzcc2;

}
/************************************************************************/
/* 配准 纹理加曲率                                                                     */
/************************************************************************/
BOOL Registration::Regist(t_Visual *targetVisual, t_Visual* sourceVisual) 
{
	t_Visual *tVisual = targetVisual;
	t_Visual *sVisual = sourceVisual;

	double twith = targetVisual->vertexMax_x - targetVisual->vertexMin_x;
	double thigh = targetVisual->vertexMax_y - targetVisual->vertexMin_y;
	double swith = sourceVisual->vertexMax_x - sourceVisual->vertexMin_x;
	double shigh = sourceVisual->vertexMax_y - sourceVisual->vertexMin_y;

	time_t starT=0, endT=0;
	set<int> setpindex;
	int isedge1 = 0;
	int isedge2 = 0;

	//去顶领域半径
	long loop;
	int tempsize = 0;
	double tr = 0, sr = 0, tempMaxtr = 0, tempMaxsr = 0;//记录邻域半径,去掉距离最远的
	fstream fso;
	fso.open("record001.txt", ios::app | ios::out);

	string targetPath = "", partPath="";//定义纹理的路径
	
	if (tVisual->texture != NULL&&tVisual->texture != NULL)
	{
		//Eigen::Matrix3d R;//旋转向量
		//Eigen::Vector3d T;//平移矩阵
		targetPath = tVisual->map;//目标纹理路径
		partPath = sVisual->map;//部分纹理路径
		
		//计算纹理匹配点对
		//string pth1 = "E:\\studfile\\yingxiang\\培养方向\\OBJ\\20170907-1\\1_c.bmp";
		//string pth2 = "E:\\studfile\\yingxiang\\培养方向\\OBJ\\20170907-1\\2_c.bmp";
		//string pth1 = "E:\\studfile\\yingxiang\\培养方向\\OBJ\\茶壶模型\\20171108-茶壶左右\\2_a.bmp";
		//string pth2 = "E:\\studfile\\yingxiang\\培养方向\\OBJ\\茶壶模型\\20171108-茶壶左右\\1_a.bmp";
		//int machePnum = texttureKeyPoint(pth1, pth2);
		starT = clock();
		int machePnum = texttureKeyPoint(tVisual->texMat, sVisual->texMat);

		endT = clock();
		
		char str1[256];
		sprintf(str1, "%lf", (double)(endT - starT) / CLOCKS_PER_SEC);
		//AfxMessageBox(str1);
		//endT = clock();

		starT = clock();
		//计算纹理匹配点对 对应的点云点对
		if (tmachePoint!=NULL)
		{
			int a = sizeof(tmachePoint);
			int b = sizeof(tmachePoint[0]);

			//int machePnum = sizeof(tmachePoint) / sizeof(tmachePoint[0]);
			float dist = 0;
			int tPIndex = 0;//目标模型点索引
			int sPIndex = 0;//源模型点索引
			double *tcurvalue;//存储点的高斯曲率
			tcurvalue = (double *)malloc(machePnum * sizeof(double));
			double *scurvalue;//存储点的高斯曲率
			scurvalue = (double *)malloc(machePnum * sizeof(double));
			float maxdist = 0;//点最大距离
			float mindist;//点最小距离
			float mindists[5] = { 1,1,1,1,1 };//点最小距离数组

			dmetchPoints = (metchPoints *)malloc(machePnum * sizeof(metchPoints));//为3d匹配点对分配内存
			if (tVisual->mechindexs!=NULL)
			{
				free(tVisual->mechindexs);
				tVisual->mechindexs = NULL;
			}
			tVisual->mechindexs = (mechIndex *)malloc(machePnum * sizeof(mechIndex));
			tVisual->mechSize = machePnum;


			//
			for (int i=0;i<machePnum;i++)
			{
	
				mindist = 1;
				//遍历目标模型纹理坐标
				//for (int j = 0; j < tVisual->faceCnt; j++)
				//{
				//	////判断是否相等 几率较小 应该找距离最小的
				//	//if (tmachePoint[i].targetPoint.u == tVisual.texture[j].u&&
				//	//	tmachePoint[i].targetPoint.v == tVisual.texture[j].v)
				//	//{
				//	//	//这里暂时直接使用序列号查找3d点，最好使用面查找对应点
				//	//	dmetchPoints[i].p1.x = tVisual.vertex[j].x;
				//	//	dmetchPoints[i].p1.y = tVisual.vertex[j].y;
				//	//	dmetchPoints[i].p1.z = tVisual.vertex[j].z;
				//	//	break;
				//	//}
				//	int m = 0;
				//	while (m<3)
				//	{
				//		long n = tVisual->index[j].t[m];
				//		dist = (tmachePoint[i].targetPoint.u - tVisual->texture[n].u)*(tmachePoint[i].targetPoint.u - tVisual->texture[n].u) +
				//			(tmachePoint[i].targetPoint.v - tVisual->texture[n].v)*(tmachePoint[i].targetPoint.v - tVisual->texture[n].v);
				//		if (dist < mindist)
				//		{
				//			mindist = dist;
				//			tPIndex = tVisual->index[j].v[m];//记录最近点的位置
				//		}
				//		m++;
				//	}
				//	
				//}

				//遍历点的坐标求得坐标的比例和纹理匹配点的坐标
				for (long j=0;j<tVisual->vertexCnt;j++)
				{
					//坐标比例相同近似
					dist = abs(tmachePoint[i].targetPoint.u - (tVisual->vertex[j].x-tVisual->vertexMin_x )/ twith)+
						abs(tmachePoint[i].targetPoint.v -1+ (tVisual->vertex[j].y - tVisual->vertexMin_y) / thigh);
					if (dist<mindist)
					{
						mindist = dist;
						tPIndex =j;//记录最近点的位置

					}

				}

				dmetchPoints[i].p1.x = tVisual->vertex[tPIndex].x;
				dmetchPoints[i].p1.y = tVisual->vertex[tPIndex].y;
				dmetchPoints[i].p1.z = tVisual->vertex[tPIndex].z;

				//计算匹配点的高斯曲率
				tcurvalue[i]=GuassCurvature(targetVisual, tPIndex,&isedge1);
				
				mindist = 1;
				//遍历部分模型 的纹理坐标
				//for (int j = 0; j < sVisual->faceCnt; j++)
				//{
				//	////判断是否相等
				//	//if (tmachePoint[i].partPoint.u == sVisual.texture[j].u&&
				//	//	tmachePoint[i].partPoint.v == sVisual.texture[j].v)
				//	//{
				//	//	//这里暂时直接使用序列号查找3d点，最好使用面查找对应点
				//	//	dmetchPoints[i].p2.x = sVisual.vertex[j].x;
				//	//	dmetchPoints[i].p2.y = sVisual.vertex[j].y;
				//	//	dmetchPoints[i].p2.z = sVisual.vertex[j].z;
				//	//	break;
				//	//}
				//	int m = 0;
				//	while (m<3)
				//	{
				//		long n= sVisual->index[j].t[m];
				//		dist = (tmachePoint[i].partPoint.u - sVisual->texture[n].u)*(tmachePoint[i].partPoint.u - sVisual->texture[n].u) +
				//			(tmachePoint[i].partPoint.v - sVisual->texture[n].v)*(tmachePoint[i].partPoint.v - sVisual->texture[n].v);
				//		if (dist < mindist)
				//		{
				//			mindist = dist;
				//			sPIndex = sVisual->index[j].v[m];//记录最近点的位置
				//		}
				//		m++;
				//	}
				//}

				//遍历点的坐标求得坐标的比例和纹理匹配点的坐标
				for (long j = 0; j < sVisual->vertexCnt; j++)
				{
					//坐标比例相同近似
					dist = abs(tmachePoint[i].partPoint.u -(sVisual->vertex[j].x- sVisual ->vertexMin_x)/swith)+
						abs(tmachePoint[i].partPoint.v -1+ (sVisual->vertex[j].y - sVisual->vertexMin_y) / shigh);
					if (dist < mindist)
					{
						mindist = dist;
						sPIndex = j;//记录最近点的位置

					}

				}

				dmetchPoints[i].p2.x = sVisual->vertex[sPIndex].x;
				dmetchPoints[i].p2.y = sVisual->vertex[sPIndex].y;
				dmetchPoints[i].p2.z = sVisual->vertex[sPIndex].z;

				//计算匹配点的高斯曲率
				scurvalue[i]=GuassCurvature(sourceVisual, sPIndex,&isedge2);
				
				
				//判断匹配精确度
				dist = fabs(tcurvalue[i] - scurvalue[i]);
				/*if (dist>1)
				{
					tVisual->mechindexs[i].targetIndex = tPIndex;
					tVisual->mechindexs[i].partIndex = sPIndex;
					tVisual->mechindexs[i].statu = 0;

					dmetchPoints[i].statu =0;
				}
				else 
				{*/
					tVisual->mechindexs[i].targetIndex = tPIndex;
					tVisual->mechindexs[i].partIndex = sPIndex;
					tVisual->mechindexs[i].statu = 1;
					if (isedge1||isedge2)//判断是否是边界点
					{
						dmetchPoints[i].statu = 3;
					}else
						dmetchPoints[i].statu = 1;

				/*}*/
				if (fso)
				{
					fso << tPIndex << "\t" << sPIndex << "\t-----"<<dmetchPoints[i].statu<<"***"<<i << endl;
				}
			}

			//确定领域半径
			for (int i = 0; i < tVisual->mechSize - haveFc; i++)
			{
				if (dmetchPoints[i].statu == 1)
				{
					for (loop = 0; loop < targetVisual->mechSize; loop++)
					{
						if (dmetchPoints[loop].statu == 1)
						{
							double templength = 0;
							tVector tempVector = targetVisual->vertex[tVisual->mechindexs[i].targetIndex] - dmetchPoints[loop].p1;
							templength = CountVecLenth(tempVector);
							tr += templength;
							if (templength > tempMaxtr) tempMaxtr = templength;
							tempVector = sourceVisual->vertex[tVisual->mechindexs[i].partIndex] - dmetchPoints[loop].p2;
							templength = CountVecLenth(tempVector);
							sr += templength;
							if (templength > tempMaxsr) tempMaxsr = templength;
							tempsize++;
						}
					}
					//求改点到其他特征点距离的平均距离，想法：N个尺度，缩放2的N次方，N=8（暂设）
					//测试阶段取N为3
				/*	tr = 2 * (tr - tempMaxtr) / ((tempsize - 1) * 8);
					sr = 2 * (sr - tempMaxsr) / ((tempsize - 1) * 8);*/
					tr = 2 * (tr - tempMaxtr) / ((tempsize - 1) * 8);
					sr = 2 * (sr - tempMaxsr) / ((tempsize - 1) * 8);
					break;
				}
			}

			//计算邻域曲率相似度
			int goodmechnum = 0;
			for (int i=0;(i<tVisual->mechSize - haveFc);i++)
			{
				if (dmetchPoints[i].statu == 1 && goodmechnum<6)
				{
					double tempd=NZCC(targetVisual, sourceVisual,
						tVisual->mechindexs[i].targetIndex, tVisual->mechindexs[i].partIndex,tr,sr);
					if (tempd==-3)
					{
						tVisual->mechindexs[i].statu = 0;
						dmetchPoints[i].statu = 3;
					}else if (tempd<0.0)
					{
						tVisual->mechindexs[i].statu = 0;
						dmetchPoints[i].statu = 2;
						
					}
					else
						goodmechnum++;

					fso << tempd << endl;
					//fso << "***************" << tVisual->mechindexs[i].targetIndex << tVisual->mechindexs[i].partIndex << endl;
				}
				else if (goodmechnum>=6)
				{
					dmetchPoints[i].statu = 0;
				}

			}
			//RANSAC
			//思路 排斥次数 边长比例不同 排斥 然后计数 选择最小的
			vector<int> dif_count(tVisual->mechSize);
			//for (int i = 0; i < tVisual->mechSize; i++)
			//{
			//	dif_count[i] = 0;
			//}
			double dif_error = 0.05;

			//计算模型1的边与模型2中边的比例，两条边是对应的
			for (int i = 0; i <0 /*tVisual->mechSize*/; i++)
			{

				for (int j = 0; j < tVisual->mechSize; j++)
				{
					if (i == j) continue;
					tVector tempV1 = dmetchPoints[i].p1 - dmetchPoints[j].p1;
					double length1 = CountVecLenth(tempV1);
					//double edge_corner1=CountTangle()
					tVector tempV2 = dmetchPoints[i].p2 - dmetchPoints[j].p2;
					double length2 = CountVecLenth(tempV2);
					//边长比例
					if (length1 == 0 || length2 == 0)
					{
						dif_count[j] += 1;
						fso << 0 << endl;
						continue;
					}
					double ratio = length1 > length2 ? length2 / length1 : length1 / length2;
					if (ratio < 0.92)
					{
						dif_count[j] += 1;
					}
					fso << ratio << endl;

				}
				fso << "*****************" << endl;
			}
			//计算模型1中的两条边比例
			for (int i = 0; i <tVisual->mechSize&&(haveFc==0&& tVisual->mechSize<15); i++)
			{

				for (int j = 0; j < tVisual->mechSize; j++)
				{
					if (j == i)continue;
					

					
					for (int m=0;m<tVisual->mechSize;m++)
					{
						if (m == i || m == j) continue;
						tVector tempV1 = dmetchPoints[i].p1 - dmetchPoints[j].p1;
						tVector tempV2 = dmetchPoints[i].p2 - dmetchPoints[j].p2;
						tVector tempV3 = dmetchPoints[i].p1 - dmetchPoints[m].p1;
						tVector tempV4 = dmetchPoints[i].p2 - dmetchPoints[m].p2;
						double length1 = CountVecLenth(tempV1);
						double length2 = CountVecLenth(tempV2);
						double length3 = CountVecLenth(tempV3);
						double length4 = CountVecLenth(tempV4);
						//double theta1 = CountTangle(tempV1, tempV3);
						//double theta2 = CountTangle(tempV2, tempV4);
						
						if (length1 == 0 || length2 == 0|| length3 == 0 || length4 == 0)// || theta1==0|| theta2==0)
						{
							dif_count[j] += 1;
							dif_count[m] += 1;
							fso << 0 << endl;
							continue;
						}

						double ratio_length1 = length1 / length3; //length1 > length2 ? length2 / length1 : length1 / length2;
						double ratio_length2 = length2 / length4;//length3 > length4 ? length4 / length3 : length3 / length4;
						double ratio_length = ratio_length1 > ratio_length2 ? ratio_length2 / ratio_length1 : ratio_length1 / ratio_length2;
						//double ratio_tangle = theta1 > theta2 ? theta2 / theta1 : theta1 / theta2;
						double ratio = ratio_length;// *ratio_tangle;
						if (ratio_length < 0.9)
						{
							dif_count[j] += 1;
							dif_count[m] += 1;

						}

						fso << ratio_length << endl;
					}
				}
				fso << "*****************" << endl;
			}
			//找最小的5个
			
			int minindex = 0;
			int *indexsort = new int[tVisual->mechSize];
			int least_pointsNum=5;
			fso << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
			//初始化indexsort[]
			for (size_t i = 0; i < tVisual->mechSize; i++)
			{
				indexsort[i] = i;
				fso << indexsort[i] << "  " << dif_count[i]<<" "<< dmetchPoints[indexsort[i]].statu << endl;
			}
			fso << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
			for (size_t i = 0; i <tVisual->mechSize&&least_pointsNum>0&& (haveFc==0 && tVisual->mechSize<15); i++)
			{

				double min_temp = dif_count[0];
				minindex = i;
				for (int j=i;j<tVisual->mechSize;j++)
				{
					if (dif_count[minindex]>dif_count[j])
					{
						minindex = j;
					}
				}
				
				int tempValue = dif_count[minindex];
				dif_count[minindex] = dif_count[i];
				dif_count[i] = tempValue;

				int tempindex = indexsort[ minindex];
				indexsort[minindex] = indexsort[i];
				indexsort[i] = tempindex;

				if (dmetchPoints[indexsort[i]].statu == 4|| indexsort[i]>(tVisual->mechSize-haveFc))
				{
					continue;
				}
				dmetchPoints[indexsort[i]].statu = 1;
				least_pointsNum--;
				fso << indexsort[i] << "  " << dif_count[i] << endl;
			}
			fso << "*****************" << endl;


			fso << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
			for (size_t i = 0; i < tVisual->mechSize; i++)
			{
				//indexsort[i] = i;
				fso << indexsort[i] << "  " << dif_count[i]<<" "<< dmetchPoints[indexsort[i]].statu << endl;
			}
			if (goodmechnum>=5)
			{
				for (size_t i = tVisual->mechSize-haveFc; i < tVisual->mechSize; i++)
				{
					dmetchPoints[i].statu = 0;
					
				}
				for (size_t i = 5; i < tVisual->mechSize&& (haveFc>0 || tVisual->mechSize<15); i++)
				{
					dmetchPoints[indexsort[i]].statu = 0;
				}
			}
			if(haveFc>0){
				for (size_t i =0; i < (tVisual->mechSize-haveFc); i++)
				{
					dmetchPoints[i].statu = 0;
				
				}
				for (size_t i = tVisual->mechSize - haveFc; i < tVisual->mechSize; i++)
				{
					dmetchPoints[i].statu = 1;

				}
			}
			fso << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;



			RotationMatrix(dmetchPoints, machePnum,ER,ET);
			RTModel(sourceVisual, ER, ET);
			/*if (fso)
			{
				for (int i = 0; i < machePnum; i++)
				{

					fso << tcurvalue[i] << "," << scurvalue[i] << dmetchPoints[i].statu <<"\t\t\t";
					fso << dmetchPoints[i].p1.x << " " << dmetchPoints[i].p1.y << " " << dmetchPoints[i].p1.z << "\t\t\t";
					fso << dmetchPoints[i].p2.x << " " << dmetchPoints[i].p2.y << " " << dmetchPoints[i].p2.z << endl;
				}
			}
			if (fso)
			{
				for (int i = 0; i < machePnum; i++)
				{

					fso << tmachePoint[i].partPoint.u << " " << tmachePoint[i].partPoint.v << endl;
					fso << tmachePoint[i].targetPoint.u << " " << tmachePoint[i].targetPoint.v << endl;
				}
			}*/
			if (fso)
			{
				fso << "最终：R" << endl;
				fso << ER << endl;
				fso << "---------------------------------" << endl;
				fso << "最终：T" << endl;
				fso << ET << endl;
				fso << "*********************************" << endl;
			}
			free(tcurvalue);
			free(scurvalue);

		}
		else
		{
			return FALSE;
		}
		endT = clock();
		double runtim = (double)(endT - starT) / CLOCKS_PER_SEC;
		char str[256];
		sprintf(str, "%lf", runtim);
		//AfxMessageBox(str);

	}
	else
		return FALSE;
	
	return TRUE;
	

}
/************************************************************************/
/* 功能：RotationMatrix计算转换矩阵，利用四元数
参数：dmetchPoints：匹配点对；
*/
/************************************************************************/
double   Registration::RotationMatrix(metchPoints* dmetchPoints, int machePnum, Eigen::Matrix3d &R, Eigen::Vector3d &T)
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
	Eigen::Vector3d P1,P2;//平移向量p1p2为质心
	tVector p1, p2, c1, c2, c,cc;//p1,p2分别是两个模型的质心，c1,c2分别表示去中心化后的点，c=c1-c2,cc=c1+c2
	int loop;
	int num=0;
	tVector sum1, sum2;
	sum1.x = sum1.y = sum1.z = 0;
	sum2.x = sum2.y = sum2.z = 0; 
	fstream fs1;
	fs1.open("旋转平移矩阵.txt", ios::app | ios::out);
	//计算质心
	for (loop=0;loop<machePnum;loop++)
	{
		if (dmetchPoints[loop].statu==1)
		{
			sum1 += dmetchPoints[loop].p1;
			sum2 += dmetchPoints[loop].p2;
			num++;
		}
	}
	if (num>4)//暂时要求至少4个点
	{
		p1.x = sum1.x / num; p1.y = sum1.y / num; p1.z = sum1.z / num;
		p2.x = sum2.x / num; p2.y = sum2.y / num; p2.z = sum2.z / num;
		P1(0) = p1.x; P1(1) = p1.y; P1(2) = p1.z;
		P2(0) = p2.x; P2(1) = p2.y; P2(2) = p2.z;
	}
	else {

		AfxMessageBox("没有匹配的点对，无法计算质心");
		//return 0;
	}
	//求A
	for (loop=0;loop<machePnum;loop++)
	{
		if (dmetchPoints[loop].statu==1)
		{
			//去中心化
			c1 = dmetchPoints[loop].p1 - p1;
			c2 = dmetchPoints[loop].p2 - p2;
			c = c1 - c2;
			cc = c1 + c2;
			/*Ai << 0, -(c.x*0.5), -c.y*0.5, -c.z*0.5,
				c.x*0.5, 0, -cc.z, cc.y,
				c.y*0.5, cc.z, 0, -cc.x,
				c.z*0.5, -cc.y, cc.x, 0;*/
			Ai << 0, -(c.x), -c.y, -c.z,
				c.x, 0, -cc.z, cc.y,
				c.y, cc.z, 0, -cc.x,
				c.z, -cc.y, cc.x, 0;
			//转置
			Ait = Ai.transpose();
			A += (Ait* Ai);
		}
		
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
	if (fs1)
	{
		fs1 <<"R"<< endl;
		fs1 << R << endl;
		fs1 << "---------------------------------" << endl;
		fs1 << "T" << endl;
		fs1 << T << endl;
		fs1 << "*********************************" << endl;
	}
	fs1.close();
	RTmetchPoints(dmetchPoints, machePnum, R, T);
	return 0;
}
/************************************************************************/
/* 功能：RTModel对源模型进行旋转平移操作
参数：dmetchPoints：匹配点对；machePnum：匹配数目；R：旋转矩阵；T：平移矩阵
*/
/************************************************************************/
double  Registration::RTModel(t_Visual *sourceModel, Eigen::Matrix3d R, Eigen::Vector3d T)
{
	//思路：
	//遍历每一个点，进行计算p=R*p+T；
	long loop;
	tVector tempv;
	for (loop=0;loop<sourceModel->vertexCnt;loop++)
	{
		tempv = sourceModel->vertex[loop];
		sourceModel->vertex[loop].x =
			tempv.x*R(0, 0) +
			tempv.y*R(0, 1) +
			tempv.z*R(0, 2) + T(0);
		sourceModel->vertex[loop].y =
			tempv.x*R(1, 0) +
			tempv.y*R(1, 1) +
			tempv.z*R(1, 2) + T(1);
		sourceModel->vertex[loop].z =
			tempv.x*R(2, 0) +
			tempv.y*R(2, 1) +
			tempv.z*R(2, 2) + T(2);
	}
	
	return 0;
}
/************************************************************************/
/* 功能：TPS_nonrigid 非刚性变形                                                                     */
/************************************************************************/
int Registration::TPS_nonrigid()
{
	int32_t dim = 3;
	int32_t num = 10000;
	Eigen::Matrix3d ER;
	Eigen::Vector3d ET;
	//fstream filesss;
	//filesss.open("record0001.txt", ios::out | ios::app);
	// allocate model and template memory
	/*double* M = (double*)calloc(3 * num, sizeof(double));
	double* T = (double*)calloc(3 * num, sizeof(double));*/
	if (partModel->normalCnt <= 0)
	{
		vertexNormal(partModel);
	}
	if (targetModel->normalCnt <= 0)
	{
		vertexNormal(targetModel);
	}
	int ModelvertexNum = targetModel->vertexCnt;//模型点数量
	int GModelvertexNum = partModel->vertexCnt;//转换模型数量
	double* M = (double*)calloc(3 * targetModel->vertexCnt, sizeof(double));
	double* Mn = (double*)calloc(3 * targetModel->vertexCnt, sizeof(double));
	double* T = (double*)calloc(3 * partModel->vertexCnt, sizeof(double));
	double* Tn = (double*)calloc(3 * partModel->vertexCnt, sizeof(double));
	// set model and template points
	//cout << endl << "Creating model with 10000 points ..." << endl;
	//cout << "Creating template by shifting model by (1,1,1) ..." << endl;
	int32_t k = 0;
	//此处默认模型具有法向量，否则程序错误待改进yingxiang
	for (int i = 0; i < (targetModel->vertexCnt); i++)
	{
		M[k * 3 + 0] = targetModel->vertex[i].x;
		M[k * 3 + 1] = targetModel->vertex[i].y;
		M[k * 3 + 2] = targetModel->vertex[i].z;
		Mn[k * 3 + 0] = targetModel->normal[i].x;
		Mn[k * 3 + 1] = targetModel->normal[i].y;
		Mn[k * 3 + 2] = targetModel->normal[i].z;
		//filesss << Mn[k * 3 + 0] << "  " << Mn[k * 3 + 1] << "   " << Mn[k * 3 + 2] << endl;
		k++;
	}
	k = 0;
	for (int i = 0; i < (partModel->vertexCnt); i++)
	{
		T[k * 3 + 0] = partModel->vertex[i].x;
		T[k * 3 + 1] = partModel->vertex[i].y;
		T[k * 3 + 2] = partModel->vertex[i].z;
		//向量
		Tn[k * 3 + 0] = partModel->normal[i].x;
		Tn[k * 3 + 1] = partModel->normal[i].y;
		Tn[k * 3 + 2] = partModel->normal[i].z;

		k++;
	}
	// start with identity as initial transformation
	// in practice you might want to use some kind of prediction here
	selfMatrix::Matrix R = selfMatrix::Matrix::eye(3);
	selfMatrix::Matrix t(3, 1);

	// run point-to-plane ICP (-1 = no outlier threshold)
	//cout << endl << "Running ICP (point-to-plane, no outliers)" << endl;
	icpPointMoveToPlane icp(M, ModelvertexNum, dim, Mn, targetModel, partModel,1);
	icp.fit_nonrigid(T, Tn, GModelvertexNum, R, t, 10);//参数根据模型实际情况定
	k = 0;
	for (int i = 0; i < (partModel->vertexCnt); i++)
	{
		partModel->vertex[i].x = T[k * 3 + 0];//x方向平移，此处是为同一模型做实验！！！！
		partModel->vertex[i].y = T[k * 3 + 1];
		partModel->vertex[i].z = T[k * 3 + 2];
		k++;
	}
	
	// free memory
	free(M);
	free(Mn);
	free(T);
	free(Tn);

	// success
	return 1;
}
/************************************************************************/
/* 功能：RTmetchPoints对匹配点对进行旋转平移操作，以便于迭代求解旋转、平移矩阵
参数：dmetchPoints：匹配点对；machePnum：匹配数目；R：旋转矩阵；T：平移矩阵
*/
/************************************************************************/
double  Registration::RTmetchPoints(metchPoints* dmetchPoints, int machePnum, Eigen::Matrix3d R, Eigen::Vector3d T)
{
	//思路：
	//对匹配点对进行旋转平移
	int loop;
	tVector tempv;
	for (loop=0;loop<machePnum;loop++)
	{
		tempv = dmetchPoints[loop].p2;
		dmetchPoints[loop].p2.x =
			tempv.x*R(0, 0) +
			tempv.y*R(0, 1) +
			tempv.z*R(0, 2) + T(0);
		dmetchPoints[loop].p2.y =
			tempv.x*R(1, 0) +
			tempv.y*R(1, 1) +
			tempv.z*R(1, 2) + T(1);
		dmetchPoints[loop].p2.z =
			tempv.x*R(2, 0) +
			tempv.y*R(2, 1) +
			tempv.z*R(2, 2) + T(2);
	}
	return 0;
}
/************************************************************************/
/* 功能：SaveObjOption保存obj界面
参数：t_Visual：模型指针
*/
/************************************************************************/
void Registration::SaveObjOption(t_Visual *const t_Visual)
{
	// TODO: 在此添加命令处理程序代码

	LPCTSTR dibName = "";
	CFileDialog fileDlg(FALSE);
	CString str;
	char* s;

	fileDlg.m_ofn.lpstrTitle = "OBJ文件另存为";
	fileDlg.m_ofn.lpstrFilter = "OBJ Files(*.obj)\0*.obj\0All File(*.*)\0*.*\0\0";
	//fileDlg.DoModal();

	if (IDOK == fileDlg.DoModal())
	{
		str = fileDlg.GetPathName();
		int len = str.GetLength();
		s = new char[len + 1];
		memset(s, 0, len + 1);
		for (int i = 0; i < len; i++)
			s[i] = str.GetAt(i);
		dibName = s;
	}
	s = new char[1]; memset(s, 0, 1);
	SaveObj(dibName, t_Visual);
	delete[]s;
}
/************************************************************************/
/* 功能：SaveObjOption保存obj
参数：t_Visual：模型指针
*/
/************************************************************************/
BOOL Registration::SaveObj(CString pathname,t_Visual *const t_Visual)
{
	fstream fo(pathname, ios::out);
	if (!fo||fo.fail())
	{
		return FALSE;
	}
	long loop;
	for (loop=0;loop<t_Visual->vertexCnt&&t_Visual->vertexCnt>0;loop++)
	{
		fo << "v " << t_Visual->vertex[loop].x << " " << t_Visual->vertex[loop].y << " "<< t_Visual->vertex[loop].z<<endl;
	}
	for (loop = 0; loop < t_Visual->vertexCnt&&t_Visual->TextureStatu == 3; loop++)
	{
		fo << "vt " << t_Visual->texture[loop].u << " " << t_Visual->texture[loop].v << endl;
	}
	for (loop = 0; loop<t_Visual->normalCnt&&t_Visual->normalCnt>0; loop++)
	{
		fo << "vn " << t_Visual->normal[loop].x << " " << t_Visual->normal[loop].y << " " << t_Visual->normal[loop].z << endl;
	}
	for (loop = 0; loop<t_Visual->faceCnt&&t_Visual->faceCnt>0; loop++)
	{
		if (t_Visual->normalCnt>0&& t_Visual->TextureStatu == 3)
		{
			fo << "f " << t_Visual->index[loop].v[0] + 1 << "/" << t_Visual->index[loop].t[0] + 1 << "/" << t_Visual->index[loop].v[0] + 1 << " "
				<< t_Visual->index[loop].v[1] + 1 << "/" << t_Visual->index[loop].t[1] + 1 << "/" << t_Visual->index[loop].v[1] + 1 << " "
				<< t_Visual->index[loop].v[2] + 1 << "/" << t_Visual->index[loop].t[2] + 1 << "/" << t_Visual->index[loop].v[2] + 1 << endl;
		}
		else if (t_Visual->normalCnt > 0)
		{
			fo << "f " << t_Visual->index[loop].v[0] + 1 << "/" << t_Visual->index[loop].v[0] + 1 << " "
				<< t_Visual->index[loop].v[1] + 1 << "/" << t_Visual->index[loop].v[1] + 1 << " "
				<< t_Visual->index[loop].v[2] + 1 << "/" << t_Visual->index[loop].v[2] + 1 << endl;
		}
		else if (t_Visual->TextureStatu == 2)
		{
			fo << "f " << t_Visual->index[loop].v[0] + 1 << "/" << t_Visual->index[loop].t[0] + 1 << " "
				<< t_Visual->index[loop].v[1] + 1 << "/" << t_Visual->index[loop].t[1] + 1 << " "
				<< t_Visual->index[loop].v[2] + 1 << "/" << t_Visual->index[loop].t[2] + 1 << endl;
		}else
			fo << "f " << t_Visual->index[loop].v[0] + 1 << " " << t_Visual->index[loop].v[1] + 1 << " " << t_Visual->index[loop].v[2] + 1 << endl;
	}
	
}
