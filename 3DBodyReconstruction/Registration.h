#pragma once
#include <GL/gl.h>
#include <GL/glu.h>
#include "Skeleton.h"
#include "ARAPDeform.h"
#include <iostream>
#include <list>
#include <vector>
//#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/NonLinearOptimization>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
using namespace std;
typedef struct
{
	double angle;
	double area;
	tVector FaceVector;//面法向量

}t_Triangle;
typedef struct
{
	t2DCoord targetPoint;
	t2DCoord partPoint;
}TmachePoints;
typedef struct  
{
	int index;
	double Gvalue;
	double Hvalue;
	double k1;
	double k2;
	double r;//半径
}vertexCurvature;
class Registration
{
public:
	Registration(void);
	Registration(t_Visual *tarMdoel,t_Visual *parMdoel):targetModel(tarMdoel),partModel(parMdoel){
		dlib::deserialize("shape_predictor_68_face_landmarks.dat") >> sp;
	};
	~Registration(void);
public:
	TmachePoints *tmachePoint;//纹理匹配点对
	vector<TmachePoints> tmachePointv;
	metchPoints *dmetchPoints;//点云匹配点对
	metchPoints *dgoodmetchPoints;//筛选后点云匹配点对
	int haveFc=0;
	dlib::shape_predictor sp;
	
public:
	t_Visual * targetModel;//目标模型不动
	t_Visual * partModel;  //部分模型动动
	GLfloat Rt[16];//转换矩阵
	Eigen::Matrix3d ER;//旋转矩阵R
	Eigen::Vector3d	ET;//平移矩阵T
public:
	inline tVector CountNormalofFace(tVector, tVector, tVector);
	int ICP_PointToPlaneRegister();
	void GuassCurvature(t_Visual *mVisual);
	double GuassCurvature(t_Visual*, int pindex, int *p);
	BOOL NeibGuassCurvature(t_Visual*, list<vertexCurvature > &neighborIndex, int pindex, double r);
	t_Triangle CountTangle(tVector, tVector, tVector);
	double CountTangle(tVector vec1, tVector vec2);
	double CountVecLenth(tVector vec1);
	int texttureKeyPoint(string s1, string s2);
	int texttureKeyPoint(const cv::Mat &m1,const cv::Mat &m2);
	int dlFeaturePoints(const cv::Mat &m1, const cv::Mat &m2,vector<TmachePoints>& PointPair);
	BOOL Regist(t_Visual *targetVisual, t_Visual* sourceVisual);
	/************************************************************************/
	/* 功能：判断球形邻域
	   参数：CenterPoint：中心点；NeighberPoint：判断对象（顶点）；Radius：半径
	*/
	/************************************************************************/
	BOOL isNeighbourhood(tVector CenterPoint,tVector NeighberPoint,double Radius);
	/************************************************************************/
	/* 功能：判断是否在顶点元素是否在list中
	参数：lt：LIST；pindex：顶点索引；
	*/
	/************************************************************************/
	BOOL findListData(list<vertexCurvature> lt, int pindex);
	/************************************************************************/
	/* 功能：判断是否在顶点元素是否在list中,并删除
	参数：lt：LIST；pindex：顶点索引；
	*/
	/************************************************************************/
	BOOL DelListData(list<vertexCurvature> lt, int pindex);
	/************************************************************************/
	/* 功能：NZCC（归一化相互关系数），判断邻域相似度
	参数：targetVisual：目标模型；sourceVisual：源模型；
	tindex：目标点的索引；sindex：源模型点的索引；tr:目标模型的邻域半径；sr:源模型的邻域半径；
	*/
	/************************************************************************/
	double NZCC(t_Visual *targetVisual, t_Visual *sourceVisual, int tindex, int sindex, double tr,double sr);
	/************************************************************************/
	/* 功能：RotationMatrix计算转换矩阵，利用四元数
	参数：dmetchPoints：匹配点对；machePnum：匹配数目；R：旋转矩阵；T：平移矩阵
	*/
	/************************************************************************/
	double  RotationMatrix(metchPoints* dmetchPoints,int machePnum,Eigen::Matrix3d &R, Eigen::Vector3d &T);
	/************************************************************************/
	/* 功能：RTModel对源模型进行旋转平移操作
	参数：sourceModel：源模型；R：旋转矩阵；T：平移矩阵
	*/
	/************************************************************************/
	double  RTModel(t_Visual *sourceModel, Eigen::Matrix3d R, Eigen::Vector3d T);
	/************************************************************************/
	/* 功能：RTmetchPoints对匹配点对进行旋转平移操作，以便于迭代求解旋转、平移矩阵
	参数：dmetchPoints：匹配点对；machePnum：匹配数目；R：旋转矩阵；T：平移矩阵
	*/
	/************************************************************************/
	double  RTmetchPoints(metchPoints* dmetchPoints, int machePnum, Eigen::Matrix3d R, Eigen::Vector3d T);
	/************************************************************************/
	/* 功能：SaveObjOption保存obj的操作界面
	参数：t_Visual：模型指针
	*/
	/************************************************************************/
	void SaveObjOption(t_Visual *const t_Visual);
	/************************************************************************/
	/* 功能：SaveObjOption保存obj
	参数：t_Visual：模型指针
	*/
	/************************************************************************/
	BOOL SaveObj(CString  pathname,t_Visual * const t_Visual);

	/************************************************************************/
	/* 功能：nonrigid 非刚性变形                                                                     */
	/************************************************************************/
	int ICP_Point_nonrigid();
	/************************************************************************/
	/* 功能：nonrigid 非刚性变形                                                                     */
	/************************************************************************/
	int TPS_nonrigid();
	/************************************************************************/
	/* 功能：计算模型顶点的法向量                                                                     */
	/************************************************************************/
	void vertexNormal(t_Visual *originalVisual);
	/************************************************************************/
	/* 功能：计算配准误差(点到点)                                                                     */
	/************************************************************************/
	double errorDistP2P();
};
