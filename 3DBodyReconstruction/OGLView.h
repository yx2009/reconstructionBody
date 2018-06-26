
#if !defined(AFX_OGLVIEW_H__2AB46761_27CD_11D1_83A0_004005308EB5__INCLUDED_)
#define AFX_OGLVIEW_H__2AB46761_27CD_11D1_83A0_004005308EB5__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000
// OGLView.h : header file
//

#include <GL/gl.h>
#include <GL/glu.h>
///#include <gltools.h>
#include "ARBMTex.h"		// For ARB Multitexture support
#include "Skeleton.h"
#include <vector>
#include "leftForm.h"
#include "maptex.h"

using namespace std;



//20150511 单元格vector结构
typedef struct point_cloud
{

	double point_x;
	double point_y;
	double point_z;
	int    position;
	int    mark;
}PIT;
typedef struct point_knn
{
	int point;
	double distant;
}KNN;
//typedef struct 
//{
//	double angle;
//	double area;
//	tVector FaceVector;//面法向量
//
//}t_Triangle;
extern	vector<vector<PIT> > vec2;
/////////////////////////////////////////////////////////////////////////////
// COGLView window
typedef double	M3DVector3d[3];
class COGLView : public CWnd
{
// Construction
public:
	COGLView();

	leftForm view_model;
	GLuint	textureyx;
	GLuint textureyx1;
	GLuint textureyx2;
	int NumofObj;
	//GLuint texturegyy3;
	//GLuint texturegyy4;
	//GLuint texturegyy5;
	//GLuint texturegyy6;
// Attributes
public:
	HDC m_hDC;
	HGLRC m_hRC;
	CPoint m_mousepos;
	float m_Grab_Rot_X,m_Grab_Rot_Y,m_Grab_Rot_Z;
	float m_Grab_Trans_X,m_Grab_Trans_Y,m_Grab_Trans_Z;
	CStatusBar  *m_StatusBar;
	int		m_ScreenWidth, m_ScreenHeight;
	BOOL	m_AntiAlias, m_Dragging,m_Silhouette, m_ARBMultiTexture,m_UseMultiTexture;
	t_Bone		m_Camera;					// For the Camera
	t_Visual	m_Model;					// Actual Model to be Drawn
	metchPoints *dmetchPoints=NULL;
	t_Visual    m_GModel;                   //
	tVector4	m_ShadeSrc[32];				// Shade Texture
	tVector		m_ShadeLight;				// Light for Calculating Shade
	tVector		m_SilhouetteColor;			// Color For silhouette line
	float		m_SilhouetteWidth;			// Width of Silhouette line
	unsigned int	m_ShadeTexture;			// Pointer to Shaded texture
	int view_flag;                          //显示模式标志位：flag标识显示状态，0：点；1：mesh；2：纹理；3：……
	int show_flag;                          //显示选择标志位  11:全部显示；  10：显示目标模型； 01：显示源模型； 00：不显示 
	bool show_curvature;                     //显示曲率

	int Minpoint;                                ////add gyy 2014.10.16
	int whichPoint;                              ////add gyy 2014.10.16
	double XminY;                                ////add gyy 2014.10.16
	double Xmin0;                                ////add gyy 2014.10.16
	double BlockPointsX[500];                      ////add gyy 2014.11.01参考点
	double BlockPointsZ[500];                      ////add gyy 2014.11.01参考点
	int refPoints;                               ////add gyy 2014.11.01参考点个数
	GLfloat Rt[16];//转换矩阵

	maptex maped_texture;
// Operations
public:
	float	CalculateShadow(tVector *normal,tVector *light, tMatrix *mat);
	void	LoadShadeTexture(const char *texfile);
	BOOL	SetupPixelFormat(HDC hdc);
	GLvoid	drawModel(t_Visual *model);
	GLvoid	drawModel22(t_Visual *model);
	GLvoid	drawGModel(t_Visual *model);

	GLvoid	drawMechPoints(t_Visual *model1,t_Visual *model2);


	///////////////////////////////////////////////
	GLvoid	drawScene();
	GLvoid  SetCamera();
	GLvoid	drawGScene();
	void	CheckForMultiTexture();
	GLvoid	initializeGL(GLsizei width, GLsizei height);
	void initLights();
	GLvoid	resize( GLsizei width, GLsizei height );
	void	GetGLInfo();
	void	drawGround();
	void	HandleKeyUp(UINT nChar);
	void	HandleKeyDown(UINT nChar);
	BOOL	LoadOBJModel(CString name);
	BOOL	LoadOBJGModel(CString name);
	void	CartoonSettings();
	//void MapTexture(t_Visual *visual);








// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(COGLView)
	
	void m3dFindNormal(M3DVector3d result, const M3DVector3d point1, const M3DVector3d point2, 
		const M3DVector3d point3);//add gyy 

	public:
	virtual BOOL Create(LPCTSTR lpszClassName, LPCTSTR lpszWindowName, DWORD dwStyle, const RECT& rect, CWnd* pParentWnd, UINT nID, CCreateContext* pContext = NULL);
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~COGLView();

	// Generated message map functions
protected:
	PFNGLACTIVETEXTUREARBPROC		glActiveTextureARB;
	PFNGLMULTITEXCOORD2FARBPROC		glMultiTexCoord2fARB;
	PFNGLMULTITEXCOORD1FARBPROC		glMultiTexCoord1fARB;

	void UpdateStatusBar(int mode);
	//{{AFX_MSG(COGLView)
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnDestroy();
	afx_msg void OnPaint();
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnMove(int x, int y);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);

	afx_msg void OnTimer(UINT_PTR nIDEvent);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	/************************************************************************/
	/* add by yingxiang                                                                     */
	/************************************************************************/


	afx_msg void OnBnClickedButtonClearModel();
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Developer Studio will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_OGLVIEW_H__2AB46761_27CD_11D1_83A0_004005308EB5__INCLUDED_)
