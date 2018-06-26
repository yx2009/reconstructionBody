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
	tVector FaceVector;//�淨����

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
	double r;//�뾶
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
	TmachePoints *tmachePoint;//����ƥ����
	vector<TmachePoints> tmachePointv;
	metchPoints *dmetchPoints;//����ƥ����
	metchPoints *dgoodmetchPoints;//ɸѡ�����ƥ����
	int haveFc=0;
	dlib::shape_predictor sp;
	
public:
	t_Visual * targetModel;//Ŀ��ģ�Ͳ���
	t_Visual * partModel;  //����ģ�Ͷ���
	GLfloat Rt[16];//ת������
	Eigen::Matrix3d ER;//��ת����R
	Eigen::Vector3d	ET;//ƽ�ƾ���T
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
	/* ���ܣ��ж���������
	   ������CenterPoint�����ĵ㣻NeighberPoint���ж϶��󣨶��㣩��Radius���뾶
	*/
	/************************************************************************/
	BOOL isNeighbourhood(tVector CenterPoint,tVector NeighberPoint,double Radius);
	/************************************************************************/
	/* ���ܣ��ж��Ƿ��ڶ���Ԫ���Ƿ���list��
	������lt��LIST��pindex������������
	*/
	/************************************************************************/
	BOOL findListData(list<vertexCurvature> lt, int pindex);
	/************************************************************************/
	/* ���ܣ��ж��Ƿ��ڶ���Ԫ���Ƿ���list��,��ɾ��
	������lt��LIST��pindex������������
	*/
	/************************************************************************/
	BOOL DelListData(list<vertexCurvature> lt, int pindex);
	/************************************************************************/
	/* ���ܣ�NZCC����һ���໥��ϵ�������ж��������ƶ�
	������targetVisual��Ŀ��ģ�ͣ�sourceVisual��Դģ�ͣ�
	tindex��Ŀ����������sindex��Դģ�͵��������tr:Ŀ��ģ�͵�����뾶��sr:Դģ�͵�����뾶��
	*/
	/************************************************************************/
	double NZCC(t_Visual *targetVisual, t_Visual *sourceVisual, int tindex, int sindex, double tr,double sr);
	/************************************************************************/
	/* ���ܣ�RotationMatrix����ת������������Ԫ��
	������dmetchPoints��ƥ���ԣ�machePnum��ƥ����Ŀ��R����ת����T��ƽ�ƾ���
	*/
	/************************************************************************/
	double  RotationMatrix(metchPoints* dmetchPoints,int machePnum,Eigen::Matrix3d &R, Eigen::Vector3d &T);
	/************************************************************************/
	/* ���ܣ�RTModel��Դģ�ͽ�����תƽ�Ʋ���
	������sourceModel��Դģ�ͣ�R����ת����T��ƽ�ƾ���
	*/
	/************************************************************************/
	double  RTModel(t_Visual *sourceModel, Eigen::Matrix3d R, Eigen::Vector3d T);
	/************************************************************************/
	/* ���ܣ�RTmetchPoints��ƥ���Խ�����תƽ�Ʋ������Ա��ڵ��������ת��ƽ�ƾ���
	������dmetchPoints��ƥ���ԣ�machePnum��ƥ����Ŀ��R����ת����T��ƽ�ƾ���
	*/
	/************************************************************************/
	double  RTmetchPoints(metchPoints* dmetchPoints, int machePnum, Eigen::Matrix3d R, Eigen::Vector3d T);
	/************************************************************************/
	/* ���ܣ�SaveObjOption����obj�Ĳ�������
	������t_Visual��ģ��ָ��
	*/
	/************************************************************************/
	void SaveObjOption(t_Visual *const t_Visual);
	/************************************************************************/
	/* ���ܣ�SaveObjOption����obj
	������t_Visual��ģ��ָ��
	*/
	/************************************************************************/
	BOOL SaveObj(CString  pathname,t_Visual * const t_Visual);

	/************************************************************************/
	/* ���ܣ�nonrigid �Ǹ��Ա���                                                                     */
	/************************************************************************/
	int ICP_Point_nonrigid();
	/************************************************************************/
	/* ���ܣ�nonrigid �Ǹ��Ա���                                                                     */
	/************************************************************************/
	int TPS_nonrigid();
	/************************************************************************/
	/* ���ܣ�����ģ�Ͷ���ķ�����                                                                     */
	/************************************************************************/
	void vertexNormal(t_Visual *originalVisual);
	/************************************************************************/
	/* ���ܣ�������׼���(�㵽��)                                                                     */
	/************************************************************************/
	double errorDistP2P();
};
