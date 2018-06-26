///////////////////////////////////////////////////////////////////////////////
//
// OGLView.cpp : implementation file
//
// Purpose:	Implementation of OpenGL Window of Cartoon Rendering System
//
// Created:
//		JL 1/12/00
//
///////////////////////////////////////////////////////////////////////////////
//
//	Copyright 2000 Jeff Lander, All Rights Reserved.
//  For educational purposes only.
//  Please do not republish in electronic or print form without permission
//  Thanks - jeffl@darwin3d.com
//
///////////////////////////////////////////////////////////////////////////////
/************************************************************************/
/*                                                                      */
/************************************************************************/
#include "stdafx.h"
#include <mmsystem.h>
#include <math.h>
//#include "ToonTex.h"
#include "OGLView.h"
#include "LoadOBJ.h"
#include "LoadTex.h"
#include "ToonSet.h"
#include "Skeleton.h"
#include <vector>
#include <thread>
#include "icpPointToPlane.h"
#include <fstream>
#include "Resource.h"

#include <gl/Gl.h>
#include <gl/Glu.h>
#include <gl/glut.h>
using namespace std;
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#pragma warning (disable:4244)      // I NEED TO CONVERT FROM DOUBLE TO FLOAT

/// Application Definitions ///////////////////////////////////////////////////
#define OGL_AXIS_DLIST		1		// OPENGL DISPLAY LIST ID
#define OGL_SELECTED_DLIST	2		// SELECTED BONE OPENGL DISPLAY LIST
#define ROTATE_SPEED		1.0		// SPEED OF ROTATION
///////////////////////////////////////////////////////////////////////////////

/// Global Variables //////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//点云数量
int PointsNum;
double least_point_y = 0;
double least_point_z = 0;
//图书模型分割的中点0904
double last_x1;
double last_y1;
double last_z1;

double last_x2;
double last_y2;
double last_z2;

double last_x3;
double last_y3;
double last_z3;

double last_x4;
double last_y4;
double last_z4;

double last_x5;
double last_y5;
double last_z5;


int min_x_point = 0;
int max_x_point = 0;
int max_y_point = 0;
int min_y_point = 0;
int max_z_point = 0;
int min_z_point = 0;

double min_x;
double max_x;
double max_y;
double min_y;
double max_z;
double min_z;

//最大，最小值的x,y,z坐标
double x11;
double y11;
double z11;

double x21;
double y21;
double z21;

double x31;
double y31;
double z31;

double x41;
double y41;
double z41;

//顶点存入数组
double **data_v;
double **data_v_two;
double **data_v_three;
double **data_v_four;
double **data_v_2;


//背景平面法向量单位化       
double  m0;
double  n0;
double  w0;
//////////20150507////////////
//栅格划分
//包围框的长和宽

double	long_x;
double	wide_y;
double	hight_z;
//面积
double area_xyz;

//单位面积点云数
double ave_n;
//k邻近
int k = 8;
//分割单元格的长度
double division_long;

//切分的单元格数量

int num_x;
int num_y;
int num_z;
int num_xy;


double ave_middle_x;
double ave_middle_y;
double ave_middle_z;

//创建vector	
vector<vector<PIT> > vec(200);

double  xx = 0;
double  yy = 0;
double  zz = 0;

//除掉书口
double min_x_2 = 0;
double max_x_2 = 0;
double mid_x_2 = 0;

double  middle_xx = 0;
double  middle_yy = 0;
double  middle_zz = 0;

double dis = 0;
//组合点
double mid_book_x = 0;
double mid_book_y = 0;
double mid_book_z = 0;

//到书口出还有的点云
int saved = 0;


double  outcome_1;
double  outcome_2;
double  outcome_3;
//找书口边
double BookPoint = 0;
double least_point_x = 0;



///////////////////////////////////////////////////////////////////////////////
// COGLView

COGLView::COGLView()
{
	memset(Rt, 0, sizeof(GLfloat) * 16);
	// INITIALIZE THE MODE KEYS
	m_StatusBar = NULL;	// CLEAR THIS.  IT IS SET BY MAINFRAME BUT UNTIL THEN MARK IT
	m_AntiAlias = FALSE;
	m_Dragging = FALSE;
	m_Silhouette = TRUE;
	m_ARBMultiTexture = FALSE;		// Is ARBMultiTexture supported
	m_UseMultiTexture = TRUE;		// Do I want to use it?

									// INITIALIZE SOME OF THE CAMERA VARIABLES
	ResetBone(&m_Camera, NULL);
	m_Camera.id = -1;
	strcpy(m_Camera.name, "Camera");
	m_Camera.rot.x = 0.0f;
	m_Camera.rot.y = 0.0f;
	m_Camera.rot.z = 0.0f;
	m_Camera.b_trans.y = 0.0f;
	m_Camera.b_trans.z = 0.0f;
	m_Camera.trans.y = 0.0f;
	m_Camera.trans.z = 0.0f;

	m_Model.vertex = NULL;
	m_Model.m_pRGB = NULL;//add gyy 2014.07.01 19:08
	m_GModel.vertex = NULL;
	m_GModel.m_pRGB = NULL;
	m_GModel.vfindex = NULL;//add yx 

	m_Model.Gvertex = NULL;
	m_Model.normal = NULL;
	m_Model.index = NULL;
	m_Model.vfindex = NULL;
	m_Model.texture = NULL;
	m_Model.deformData = NULL;
	m_Model.vertexColor = NULL;
	m_Model.GuassCurvature = NULL;
	m_Model.HCurvature = NULL;
	m_Model.mechindexs = NULL;

	// Set the Default Light Direction
	m_ShadeLight.x = 0.3f;
	m_ShadeLight.y = 0.1f;
	m_ShadeLight.z = 0.8f;
	NormalizeVector(&m_ShadeLight);	// Normalize it since I know I didn't

	m_SilhouetteColor.r = 0.0f;
	m_SilhouetteColor.g = 0.0f;
	m_SilhouetteColor.b = 0.0f;
	m_SilhouetteWidth = 3;			// Width of Silhouette line
									////////////////////////////////add gyy
	Minpoint = 0;                   //for test 2014.10.16
	whichPoint = 0;                   //for test 2014.10.16
	XminY = 0;                       //for test 2014.10.16
	Xmin0 = 0;                      //for test 2014.10.16
	view_flag = view_model.openGL_view_model; //view_flag标识显示状态，0：点；1：mesh；2：纹理；3：……
	show_flag = view_model.openGL_show_model;
	show_curvature = view_model.showCurvature;

	///////////////////////////////end gyy
}

COGLView::~COGLView()
{
}

BOOL COGLView::Create(LPCTSTR lpszClassName, LPCTSTR lpszWindowName, DWORD dwStyle, const RECT& rect, CWnd* pParentWnd, UINT nID, CCreateContext* pContext)
{
	return CWnd::Create(lpszClassName, lpszWindowName, dwStyle, rect, pParentWnd, nID, pContext);
}



BEGIN_MESSAGE_MAP(COGLView, CWnd)
	//{{AFX_MSG_MAP(COGLView)
	ON_WM_CREATE()
	ON_WM_DESTROY()
	ON_WM_PAINT()
	ON_WM_KEYDOWN()
	ON_WM_KEYUP()
	ON_WM_LBUTTONDOWN()
	ON_WM_RBUTTONDOWN()
	ON_WM_MOUSEMOVE()
	ON_WM_MOVE()
	ON_WM_LBUTTONUP()
	ON_WM_RBUTTONUP()
	ON_WM_MOUSEWHEEL()

	ON_WM_TIMER()
	//}}AFX_MSG_MAP
	//ON_COMMAND(ID_32808, &COGLView::OnFindLeftDownPoint)
	//ON_UPDATE_COMMAND_UI(ID_32808, &COGLView::OnUpdate32808)
	//ON_COMMAND(ID_HEHE_32810, &COGLView::OnHehe32810)
	//ON_COMMAND(ID_CARTOON_TEST, &COGLView::OnCartoonTest)
	//ON_COMMAND(ID_CARTOON_TEST, &COGLView::OnCartoonTest)
	//ON_COMMAND(ID_TEST, &COGLView::OnTest)

	
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// COGLView message handlers

BOOL getTextureWH(char *szPathName, int &lWidthPixels, int &lHeightPixels);
void  MapTexture(t_Visual *m_Model, maptex maped_texture);

BOOL COGLView::SetupPixelFormat(HDC hdc)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	PIXELFORMATDESCRIPTOR pfd, *ppfd;
	int pixelformat;
	///////////////////////////////////////////////////////////////////////////////
	ppfd = &pfd;

	ppfd->nSize = sizeof(PIXELFORMATDESCRIPTOR);
	ppfd->nVersion = 1;
	ppfd->dwFlags = PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
	ppfd->dwLayerMask = PFD_MAIN_PLANE;
	ppfd->iPixelType = PFD_TYPE_RGBA;
	ppfd->cColorBits = 16;
	ppfd->cDepthBits = 16;
	ppfd->cAccumBits = 0;
	ppfd->cStencilBits = 0;

	pixelformat = ChoosePixelFormat(hdc, ppfd);

	if ((pixelformat = ChoosePixelFormat(hdc, ppfd)) == 0) {
		MessageBox("ChoosePixelFormat failed", "Error", MB_OK);
		return FALSE;
	}

	if (pfd.dwFlags & PFD_NEED_PALETTE) {
		MessageBox("Needs palette", "Error", MB_OK);
		return FALSE;
	}

	if (SetPixelFormat(hdc, pixelformat, ppfd) == FALSE) {
		MessageBox("SetPixelFormat failed", "Error", MB_OK);
		return FALSE;
	}

	return TRUE;
}

int COGLView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	RECT rect;
	///////////////////////////////////////////////////////////////////////////////
	if (CWnd::OnCreate(lpCreateStruct) == -1)
		return -1;
	//m_hDC = ::GetDC(m_hWnd);
	m_hDC = ::GetWindowDC(m_hWnd);
	if (!SetupPixelFormat(m_hDC))
		PostQuitMessage(0);

	m_hRC = wglCreateContext(m_hDC);
	wglMakeCurrent(m_hDC, m_hRC);
	GetClientRect(&rect);
	initializeGL(rect.right, rect.bottom);

	// GENERATE THE OPENGL TEXTURE ID
	glGenTextures(1, &m_ShadeTexture);

	LoadShadeTexture("default.shd");

	// CREATE THE DISPLAY LIST FOR AN AXIS WITH ARROWS POINTING IN
	// THE POSITIVE DIRECTION Red = X, Green = Y, Blue = Z
	glNewList(OGL_AXIS_DLIST, GL_COMPILE);
	glBegin(GL_LINES);
	glColor3f(1.0f, 0.0f, 0.0f);	// X AXIS STARTS - COLOR RED
	glVertex3f(-.2f, 0.0f, 0.0f);
	glVertex3f(.2f, 0.0f, 0.0f);
	glVertex3f(0.2f, 0.0f, 0.0f);	// TOP PIECE OF ARROWHEAD
	glVertex3f(0.15f, 0.04f, 0.0f);
	glVertex3f(0.2f, 0.0f, 0.0f);	// BOTTOM PIECE OF ARROWHEAD
	glVertex3f(0.15f, -0.04f, 0.0f);
	glColor3f(0.0f, 1.0f, 0.0f);	// Y AXIS STARTS - COLOR GREEN
	glVertex3f(0.0f, 0.2f, 0.0f);
	glVertex3f(0.0f, -0.2f, 0.0f);
	glVertex3f(0.0f, 0.2f, 0.0f);	// TOP PIECE OF ARROWHEAD
	glVertex3f(0.04f, 0.15f, 0.0f);
	glVertex3f(0.0f, 0.2f, 0.0f);	// BOTTOM PIECE OF ARROWHEAD
	glVertex3f(-0.04f, 0.15f, 0.0f);
	glColor3f(0.0f, 0.0f, 1.0f);	// Z AXIS STARTS - COLOR BLUE
	glVertex3f(0.0f, 0.0f, 0.2f);
	glVertex3f(0.0f, 0.0f, -0.2f);
	glVertex3f(0.0f, 0.0f, 0.2f);	// TOP PIECE OF ARROWHEAD
	glVertex3f(0.0f, 0.04f, 0.15f);
	glVertex3f(0.0f, 0.0f, 0.2f);	// BOTTOM PIECE OF ARROWHEAD
	glVertex3f(0.0f, -0.04f, 0.15f);
	glEnd();
	glEndList();
	SetTimer(1, 1000, NULL);
	drawScene();
	return 0;
}

/* OpenGL code */

///////////////////////////////////////////////////////////////////////////////
// Function:	resize
// Purpose:		This code handles the windows resize for OpenGL
// Arguments:	Width and heights of the view window
///////////////////////////////////////////////////////////////////////////////
GLvoid COGLView::resize(GLsizei width, GLsizei height)
{
	// Local Variables ///////////////////////////////////////////////////////////
	GLfloat aspect;
	///////////////////////////////////////////////////////////////////////////////

	glViewport(0, 0, width, height);

	aspect = (GLfloat)width / (GLfloat)height;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(20.0, aspect, 1, 8000);
	glMatrixMode(GL_MODELVIEW);
	m_ScreenWidth = width;
	m_ScreenHeight = height;

}
/************************************************************************/
/* 绘制地平面                                                                     */
/************************************************************************/
void COGLView::drawGround()
{


	return;
}
//// resize /////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function:	CheckForMultiTexture
// Purpose:		Check if there is support for ARBMultitexture and save pointers
///////////////////////////////////////////////////////////////////////////////
void COGLView::CheckForMultiTexture()
{
	char *strExtensions;

	strExtensions = (char*)glGetString(GL_EXTENSIONS);
	// Check if MultiTexture is supported
	if (strstr(strExtensions, "GL_ARB_multitexture") == 0) {
		MessageBox("GL_ARB_multitexture was not found.\nSingle Pass will be used", "GL Error", MB_OK);
		m_ARBMultiTexture = FALSE;
		return;
	}
	m_ARBMultiTexture = TRUE;

	// Grab pointers to the functions I need
	glActiveTextureARB = (PFNGLACTIVETEXTUREARBPROC)wglGetProcAddress("glActiveTextureARB");
	glMultiTexCoord2fARB = (PFNGLMULTITEXCOORD2FARBPROC)wglGetProcAddress("glMultiTexCoord2fARB");
	glMultiTexCoord1fARB = (PFNGLMULTITEXCOORD1FARBPROC)wglGetProcAddress("glMultiTexCoord1fARB");
}

GLvoid COGLView::initializeGL(GLsizei width, GLsizei height)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	GLfloat aspect;
	///////////////////////////////////////////////////////////////////////////////

	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glClearDepth(1.0);
	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glShadeModel(GL_SMOOTH);

	glEnable(GL_LIGHTING);
	initLights();
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glMatrixMode(GL_PROJECTION);
	aspect = (GLfloat)width / (GLfloat)height;
	// Establish viewing volume
	gluPerspective(60.0, aspect, 10, 8000);
	glMatrixMode(GL_MODELVIEW);

	// SET SOME OGL INITIAL STATES SO THEY ARE NOT DONE IN THE DRAW LOOP
	glPolygonMode(GL_FRONT, GL_FILL);
	glDepthFunc(GL_LESS);
	glEnable(GL_CULL_FACE);
	glPointSize(8.0);		// NICE BEEFY POINTS FOR THE VERTEX SELECTION
	glDisable(GL_TEXTURE_2D);

	//glDisable(GL_LIGHTING);

	glEnable(GL_BLEND);
	//	glBlendFunc(

	CheckForMultiTexture();		// Get ARB_MultiTexture settings
}
void COGLView::initLights()
{
	// set up light colors (ambient, diffuse, specular)
	GLfloat lightKa[] = { 0.5f, 0.4f, 0.2f, 1.0f };      // ambient light
	GLfloat lightKd[] = { .9f, .6f, .5f, 1.0f };      // diffuse light
	GLfloat lightKs[] = { 1, 1, 1, 1 };               // specular light

	GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 0.5 };  //镜面反射参数
	GLfloat mat_shininess[] = { 50.0 };               //高光指数
	glShadeModel(GL_SMOOTH);											  //材质属性
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

	glLightfv(GL_LIGHT0, GL_AMBIENT, lightKa);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightKd);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightKs);

	// position the light in eye space
	float lightPos[4] = { 1, 1, 1, 0 };               // directional light
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	glEnable(GL_LIGHT0);                            // MUST enable each light source after configuration
}
///////////////////////////////////////////////////////////////////////////////
// Function:	CalculateShadow
// Purpose:		Calculate the shadow coordinate value for a normal
// Arguments:	The vertex normal, Light vector, and Object rotation matrix
// Returns:		An index coordinate into the shade table
///////////////////////////////////////////////////////////////////////////////
float COGLView::CalculateShadow(tVector *normal, tVector *light, tMatrix *mat)
{
	//// Local Variables ////////////////////////////////////////////////////////////////
	tVector post;
	float dot;
	/////////////////////////////////////////////////////////////////////////////////////
	// Rotate the normal by the current object matrix
	MultVectorByRotMatrix(mat, normal, &post);
	dot = DotProduct(&post, light);				// Calculate the Dot Product

	if (dot < 0) dot = 0;						// Make sure the Back half dark
	return dot;									// Return the shadow value
}


///////////////////////////////////////////////////////////////////////////////
// Function:	drawModel
// Purpose:		Actually Draws the Model using the Cartoon Settings
// Arguments:	Pointer to the model
///////////////////////////////////////////////////////////////////////////////
void COGLView::m3dFindNormal(M3DVector3d result, const M3DVector3d point1, const M3DVector3d point2,
	const M3DVector3d point3)
{
	M3DVector3d v1, v2;		// Temporary vectors

							// Calculate two vectors from the three points. Assumes counter clockwise
							// winding!
	v1[0] = point1[0] - point2[0];
	v1[1] = point1[1] - point2[1];
	v1[2] = point1[2] - point2[2];

	v2[0] = point2[0] - point3[0];
	v2[1] = point2[1] - point3[1];
	v2[2] = point2[2] - point3[2];

	// Take the cross product of the two vectors to get
	// the normal vector.
	//m3dCrossProduct(result, u v1, v v2);
	result[0] = v1[1] * v2[2] - v2[1] * v1[2];
	result[1] = -v1[0] * v2[2] + v2[0] * v1[2];
	result[2] = v1[0] * v2[1] - v2[0] * v1[1];
}

GLvoid COGLView::drawModel(t_Visual *visual)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	tMatrix mat;		// Needed for Lighting Calc
	int loop;
	t_faceIndex *face;
	float   u;
	double point1[3], point2[3], point3[3], vnormal[3] = { 0 };
	double guassvalue1[3], guassvalue2[3], guassvalue3[3];

	glCallList(OGL_AXIS_DLIST);
	glFlush();
	fstream filessss;
	filessss.open("record001.txt", ios::app | ios::out);
	///////////////////////////////////////////////////////////////////////////////
	if (visual->vertex != NULL)
	{

		glDisable(GL_POLYGON_OFFSET_FILL);
		glPushMatrix();



		glPointSize(1.5);
		if (visual->TextureStatu == 2 && view_flag == 2)//draw texture
		{
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, textureyx);
			glDisable(GL_LIGHTING);
			glBegin(/*GL_POINTS*/GL_TRIANGLES/*GL_LINE_STRIP*/);
			glColor3f(1.0f, 1.0f, 1.0f);
		}else if (visual->TextureStatu == 2 && view_flag == 1)//draw mesh
		{
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, textureyx);
			glDisable(GL_LIGHTING);
			glBegin(/*GL_POINTS*/GL_TRIANGLES/*GL_LINE_STRIP*/);
			glColor3f(1.0f, 1.0f, 1.0f);
		}else if (view_flag == 0)
		{
			//glEnable(GL_LIGHTING);
			glDisable(GL_LIGHTING);
			glDisable(GL_TEXTURE_2D);
			glBegin(GL_POINTS/*GL_TRIANGLES*//*GL_LINE_STRIP*/);
			glColor3f(1.0f, 0.0f, 0.0f);
		}
		else if (visual->TextureStatu == 1 && view_flag == 1)//draw mesh
		{
			glDisable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);
			glBegin(/*GL_POINTS*/GL_TRIANGLES/*GL_LINE_STRIP*/);
			glColor3f(1.0f, 0.0f, 0.0f);
		}
		else if (visual->TextureStatu == 1 && view_flag == 2)//draw color
		{
			glDisable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);
			glBegin(/*GL_POINTS*/GL_TRIANGLES/*GL_LINE_STRIP*/);
			//glColor3f(1.0f, 1.0f, 1.0f);
		}
		else
		{
			glDisable(GL_LIGHTING);
			//glEnable(GL_LIGHTING);
			glDisable(GL_TEXTURE_2D);
			glBegin(/*GL_POINTS*/GL_TRIANGLES/*GL_LINE_STRIP*/);
			glColor3f(1.0f, 1.0f, 1.0f);
		}
		
		/*glDisable(GL_TEXTURE_2D);
		glDeleteTextures(1,&textureyx);*/
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//glPointSize(1.5);
		//glBegin(/*GL_POINTS*/GL_TRIANGLES/*GL_LINE_STRIP*/);							// 开始绘制物体(三角形)
		
		//if (visual->GuassMaxValue==0.0&&visual->GuassMinValue==0.0)//判断是否是加载曲率颜色
		//{
		for (loop = 0; loop < visual->faceCnt; loop++)
		{

			point1[0] = visual->vertex[visual->index[loop].v[0]].x;
			point1[1] = visual->vertex[visual->index[loop].v[0]].y;
			point1[2] = visual->vertex[visual->index[loop].v[0]].z;
			point2[0] = visual->vertex[visual->index[loop].v[1]].x;
			point2[1] = visual->vertex[visual->index[loop].v[1]].y;
			point2[2] = visual->vertex[visual->index[loop].v[1]].z;
			point3[0] = visual->vertex[visual->index[loop].v[2]].x;
			point3[1] = visual->vertex[visual->index[loop].v[2]].y;
			point3[2] = visual->vertex[visual->index[loop].v[2]].z;

			//m3dFindNormal(vnormal, point1, point2, point3);
			//glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
			/*glTexCoord2f(visual->texture[visual->index[loop].t[0]].u,
			visual->texture[visual->index[loop].t[0]].v);*/
			if (visual->normalCnt == visual->vertexCnt)
			{
				glNormal3f(visual->normal[visual->index[loop].v[0]].x, visual->normal[visual->index[loop].v[0]].y, visual->normal[visual->index[loop].v[0]].z);
				//glNormal3f(visual->normal[visual->index[loop].n[0]].x, visual->normal[visual->index[loop].n[0]].y, visual->normal[visual->index[loop].n[0]].z);
			}
			else
			{
				m3dFindNormal(vnormal, point1, point2, point3);
				//visual->normal[visual->index[loop].n[0]].x=vnormal[0];
				//visual->normal[visual->index[loop].n[0]].y=vnormal[0];
				//visual->normal[visual->index[loop].n[0]].z=vnormal[2];
				glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

			}
			if (visual->TextureStatu == 1)//加载顶点颜色
			{
				glColor3d(visual->vertexColor[visual->index[loop].v[0]].r / 255,
					visual->vertexColor[visual->index[loop].v[0]].b / 255,
					visual->vertexColor[visual->index[loop].v[0]].g / 255);
				glVertex3d(point1[0],
					point1[1],
					point1[2]);

				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
				}
				glColor3d(visual->vertexColor[visual->index[loop].v[1]].r / 255,
					visual->vertexColor[visual->index[loop].v[1]].b / 255,
					visual->vertexColor[visual->index[loop].v[1]].g / 255);
				glVertex3d(point2[0],
					point2[1],
					point2[2]);

				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
				}
				glColor3d(visual->vertexColor[visual->index[loop].v[2]].r / 255,
					visual->vertexColor[visual->index[loop].v[2]].b / 255,
					visual->vertexColor[visual->index[loop].v[2]].g / 255);
				glVertex3d(point3[0],
					point3[1],
					point3[2]);
			}
			else if (visual->TextureStatu == 21)
			{

				glVertex3d(point1[0], point1[1], point1[2]);

				glVertex3d(point2[0], point2[1], point2[2]);

				glVertex3d(point3[0], point3[1], point3[2]);

			}
			else if (visual->TextureStatu == 2)
			{
				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[0]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
				if (visual->index[loop].t[0]<0)
				{
					continue;
				}
				glTexCoord2f(visual->texture[visual->index[loop].t[0]].u,
					visual->texture[visual->index[loop].t[0]].v);

				glVertex3d(point1[0],
					point1[1],
					point1[2]);

				glTexCoord2f(visual->texture[visual->index[loop].t[1]].u,
					visual->texture[visual->index[loop].t[1]].v);
				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[1]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/

				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
				}
				else
				{
					m3dFindNormal(vnormal, point1, point2, point3);
					glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

				}
				glVertex3d(point2[0],
					point2[1],
					point2[2]);

				glTexCoord2f(visual->texture[visual->index[loop].t[2]].u,
					visual->texture[visual->index[loop].t[2]].v);
				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[2]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/

				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
				}
				else
				{
					m3dFindNormal(vnormal, point1, point2, point3);
					glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

				}
				glVertex3d(point3[0],
					point3[1],
					point3[2]);

			}
			else if (visual->TextureStatu == 30)
			{
				double tempNum = visual->vertexMax_z + 2;
				for (loop = 0; loop < visual->faceCnt; loop++)
				{
					point1[0] = visual->vertex[visual->index[loop].v[0]].x;
					point1[1] = visual->vertex[visual->index[loop].v[0]].y;
					point1[2] = visual->vertex[visual->index[loop].v[0]].z;
					if (visual->GuassCurvature[visual->index[loop].v[0]]>0 && visual->GuassCurvature[visual->index[loop].v[0]]<1000000)
					{//红
						guassvalue1[0] = 1;//visual->GuassCurvature[visual->index[loop].v[0]] / 5 > 1 ? 1 : visual->GuassCurvature[visual->index[loop].v[0]] / 5;
						guassvalue1[1] = 0;
						guassvalue1[2] = 0;
					}
					else if (visual->GuassCurvature[visual->index[loop].v[0]]<0 && visual->GuassCurvature[visual->index[loop].v[0]]>-1000000)
					{//绿
						guassvalue1[0] = 0;
						guassvalue1[1] = 1;//visual->GuassCurvature[visual->index[loop].v[0]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[0]] / (-4);
						guassvalue1[2] = 0;

					}
					else
					{
						guassvalue1[0] = 0;
						guassvalue1[1] = 0;
						guassvalue1[2] = 1;

					}


					point2[0] = visual->vertex[visual->index[loop].v[1]].x;
					point2[1] = visual->vertex[visual->index[loop].v[1]].y;
					point2[2] = visual->vertex[visual->index[loop].v[1]].z;
					//guassvalue1[1] = visual->GuassCurvature[visual->index[loop].v[1]];
					if (visual->GuassCurvature[visual->index[loop].v[1]] > 0 && visual->GuassCurvature[visual->index[loop].v[1]]<1000000)//此处1000000专为我的模型设置可删除
					{
						guassvalue2[0] = 1;//visual->GuassCurvature[visual->index[loop].v[1]] /5 > 1 ? 1 : visual->GuassCurvature[visual->index[loop].v[1]] / 5;
						guassvalue2[1] = 0;
						guassvalue2[2] = 0;
					}
					else if (visual->GuassCurvature[visual->index[loop].v[1]] < 0 && visual->GuassCurvature[visual->index[loop].v[1]]>-1000000)
					{
						guassvalue2[0] = 0;
						guassvalue2[1] = 1;//visual->GuassCurvature[visual->index[loop].v[1]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[1]] /( -4) ;
						guassvalue2[2] = 0;

					}
					else
					{
						guassvalue2[0] = 0;
						guassvalue2[1] = 0;
						guassvalue2[2] = 1;
					}

					point3[0] = visual->vertex[visual->index[loop].v[2]].x;
					point3[1] = visual->vertex[visual->index[loop].v[2]].y;
					point3[2] = visual->vertex[visual->index[loop].v[2]].z;
					if (visual->GuassCurvature[visual->index[loop].v[2]] > 0 && visual->GuassCurvature[visual->index[loop].v[2]]<1000000)
					{
						guassvalue3[0] = 1;// visual->GuassCurvature[visual->index[loop].v[2]] / 5>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[2]] / 5 ;
						guassvalue3[1] = 0;
						guassvalue3[2] = 0;
					}
					else if (visual->GuassCurvature[visual->index[loop].v[2]] < 0 && visual->GuassCurvature[visual->index[loop].v[2]]>-1000000)
					{
						guassvalue3[0] = 0;
						guassvalue3[1] = 1;//visual->GuassCurvature[visual->index[loop].v[2]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[2]] / (-4) ;
						guassvalue3[2] = 0;

					}
					else
					{
						guassvalue3[0] = 0;
						guassvalue3[1] = 0;
						guassvalue3[2] = 1;
					}
					//guassvalue[2] = visual->GuassCurvature[visual->index[loop].v[2]];
					if (point1[2] == tempNum || point2[2] == tempNum || point3[2] == tempNum)
					{
						continue;
					}

					//m3dFindNormal(vnormal, point1, point2, point3);
					//glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
					/*glTexCoord2f(visual->texture[visual->index[loop].t[0]].u,
					visual->texture[visual->index[loop].t[0]].v);*/
					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[0]].x, visual->normal[visual->index[loop].v[0]].y, visual->normal[visual->index[loop].v[0]].z);
					}
					else
					{
						m3dFindNormal(vnormal, point1, point2, point3);
						glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

					}
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[0]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue1[0], guassvalue1[1], guassvalue1[2]);
					glVertex3d(point1[0], point1[1], point1[2]);


					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
					}
					/*glTexCoord2f(visual->texture[visual->index[loop].t[1]].u,
					visual->texture[visual->index[loop].t[1]].v);*/
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[1]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue2[0], guassvalue2[1], guassvalue2[2]);
					glVertex3d(point2[0], point2[1], point2[2]);


					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
					}
					/*glTexCoord2f(visual->texture[visual->index[loop].t[2]].u,
					visual->texture[visual->index[loop].t[2]].v);*/
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[2]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue3[0], guassvalue3[1], guassvalue3[2]);
					glVertex3d(point3[0], point3[1], point3[2]);
					//if  (filessss)
					//{
					//	filessss << point1[0] << "\t" << point1[1] << "\t" << point1[2] << "\t" << guassvalue1[0] << "\t" << guassvalue1[1] << "\t" << guassvalue1[2] << endl;
					//	filessss << point2[0] << "\t" << point2[1] << "\t" << point2[2] << "\t" << guassvalue2[0] << "\t" << guassvalue2[1] << "\t" << guassvalue2[2] << endl;
					//	filessss << point3[0] << "\t" << point3[1] << "\t" << point3[2] << "\t" << guassvalue3[0] << "\t" << guassvalue3[1] << "\t" << guassvalue3[2] << endl;
					//}
				}

			}
			else if (visual->TextureStatu == 31)
			{
				double tempNum = visual->vertexMax_z + 2;
				for (loop = 0; loop < visual->faceCnt; loop++)
				{
					point1[0] = visual->vertex[visual->index[loop].v[0]].x;
					point1[1] = visual->vertex[visual->index[loop].v[0]].y;
					point1[2] = visual->vertex[visual->index[loop].v[0]].z;
					if (visual->HCurvature[visual->index[loop].v[0]]>10 && visual->HCurvature[visual->index[loop].v[0]]<1000000)
					{//红
						guassvalue1[0] = 1;//visual->GuassCurvature[visual->index[loop].v[0]] / 5 > 1 ? 1 : visual->GuassCurvature[visual->index[loop].v[0]] / 5;
						guassvalue1[1] = 0;
						guassvalue1[2] = 0;
					}
					else if (visual->HCurvature[visual->index[loop].v[0]] <= 10 && visual->HCurvature[visual->index[loop].v[0]]>0.5)
					{//绿
						guassvalue1[0] = 1;
						guassvalue1[1] = 0;//visual->GuassCurvature[visual->index[loop].v[0]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[0]] / (-4);
						guassvalue1[2] = 0;

					}
					else if (visual->HCurvature[visual->index[loop].v[0]] <= 0.5 && visual->HCurvature[visual->index[loop].v[0]] >= 0)
					{
						guassvalue1[0] = 0;
						guassvalue1[1] = 1;
						guassvalue1[2] = 0;

					}
					else
					{
						guassvalue1[0] = 0;
						guassvalue1[1] = 0;
						guassvalue1[2] = 1;
					}


					point2[0] = visual->vertex[visual->index[loop].v[1]].x;
					point2[1] = visual->vertex[visual->index[loop].v[1]].y;
					point2[2] = visual->vertex[visual->index[loop].v[1]].z;
					//guassvalue1[1] = visual->GuassCurvature[visual->index[loop].v[1]];
					if (visual->HCurvature[visual->index[loop].v[1]] > 10 && visual->HCurvature[visual->index[loop].v[1]]<1000000)//此处1000000专为我的模型设置可删除
					{
						guassvalue2[0] = 1;//visual->GuassCurvature[visual->index[loop].v[1]] /5 > 1 ? 1 : visual->GuassCurvature[visual->index[loop].v[1]] / 5;
						guassvalue2[1] = 0;
						guassvalue2[2] = 0;
					}
					else if (visual->HCurvature[visual->index[loop].v[1]] <= 10 && visual->HCurvature[visual->index[loop].v[1]]>0.5)
					{
						guassvalue2[0] = 1;
						guassvalue2[1] = 0;//visual->GuassCurvature[visual->index[loop].v[1]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[1]] /( -4) ;
						guassvalue2[2] = 0;

					}
					else if (visual->HCurvature[visual->index[loop].v[1]] <= 0.5 && visual->HCurvature[visual->index[loop].v[1]] >= 0)
					{
						guassvalue2[0] = 0;
						guassvalue2[1] = 1;
						guassvalue2[2] = 0;
					}
					else
					{
						guassvalue2[0] = 0;
						guassvalue2[1] = 0;
						guassvalue2[2] = 1;
					}

					point3[0] = visual->vertex[visual->index[loop].v[2]].x;
					point3[1] = visual->vertex[visual->index[loop].v[2]].y;
					point3[2] = visual->vertex[visual->index[loop].v[2]].z;
					if (visual->HCurvature[visual->index[loop].v[2]] > 10 && visual->HCurvature[visual->index[loop].v[2]]<1000000)
					{
						guassvalue3[0] = 1;// visual->GuassCurvature[visual->index[loop].v[2]] / 5>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[2]] / 5 ;
						guassvalue3[1] = 0;
						guassvalue3[2] = 0;
					}
					else if (visual->HCurvature[visual->index[loop].v[2]] <= 10 && visual->HCurvature[visual->index[loop].v[2]]>0.5)
					{
						guassvalue3[0] = 1;
						guassvalue3[1] = 0;//visual->GuassCurvature[visual->index[loop].v[2]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[2]] / (-4) ;
						guassvalue3[2] = 0;

					}
					else if (visual->HCurvature[visual->index[loop].v[2]] <= 0.5 && visual->HCurvature[visual->index[loop].v[2]] >= 0)
					{
						guassvalue3[0] = 0;
						guassvalue3[1] = 1;
						guassvalue3[2] = 0;
					}
					else
					{
						guassvalue3[0] = 0;
						guassvalue3[1] = 0;
						guassvalue3[2] = 1;
					}
					//guassvalue[2] = visual->GuassCurvature[visual->index[loop].v[2]];
					if (point1[2] == tempNum || point2[2] == tempNum || point3[2] == tempNum)
					{
						continue;
					}

					//m3dFindNormal(vnormal, point1, point2, point3);
					//glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
					/*glTexCoord2f(visual->texture[visual->index[loop].t[0]].u,
					visual->texture[visual->index[loop].t[0]].v);*/
					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[0]].x, visual->normal[visual->index[loop].v[0]].y, visual->normal[visual->index[loop].v[0]].z);
					}
					else
					{
						m3dFindNormal(vnormal, point1, point2, point3);
						glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

					}
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[0]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue1[0], guassvalue1[1], guassvalue1[2]);
					glVertex3d(point1[0], point1[1], point1[2]);


					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
					}
					else
					{
						m3dFindNormal(vnormal, point1, point2, point3);
						glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

					}
					/*glTexCoord2f(visual->texture[visual->index[loop].t[1]].u,
					visual->texture[visual->index[loop].t[1]].v);*/
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[1]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue2[0], guassvalue2[1], guassvalue2[2]);
					glVertex3d(point2[0], point2[1], point2[2]);


					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
					}
					else
					{
						m3dFindNormal(vnormal, point1, point2, point3);
						glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

					}
					/*glTexCoord2f(visual->texture[visual->index[loop].t[2]].u,
					visual->texture[visual->index[loop].t[2]].v);*/
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[2]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue3[0], guassvalue3[1], guassvalue3[2]);
					glVertex3d(point3[0], point3[1], point3[2]);
					//if  (filessss)
					//{
					//	filessss << point1[0] << "\t" << point1[1] << "\t" << point1[2] << "\t" << guassvalue1[0] << "\t" << guassvalue1[1] << "\t" << guassvalue1[2] << endl;
					//	filessss << point2[0] << "\t" << point2[1] << "\t" << point2[2] << "\t" << guassvalue2[0] << "\t" << guassvalue2[1] << "\t" << guassvalue2[2] << endl;
					//	filessss << point3[0] << "\t" << point3[1] << "\t" << point3[2] << "\t" << guassvalue3[0] << "\t" << guassvalue3[1] << "\t" << guassvalue3[2] << endl;
					//}
				}

			}


			else
			{
				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[0]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/


				glColor3d(1.0f, 0.0f, 0.0f);
				glVertex3d(point1[0],
					point1[1],
					point1[2]);

				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[1]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
				}
				else
				{
					glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
				}
				//glColor3d(1.0f, 1.0f, 1.0f);
				glVertex3d(point2[0],
					point2[1],
					point2[2]);

				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[2]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
				}
				else
				{
					glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
				}
				//glColor3d(1.0f, 1.0f, 1.0f);
				glVertex3d(point3[0],
					point3[1],
					point3[2]);

			}

		}
		//glutSolidSphere(10,40,40);
		//}

		glEnd();
		glPopMatrix();
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(-1.0f, -1.0f);
		glDisable(GL_TEXTURE_2D);
		
		glPushMatrix();
		glBegin(GL_LINE);
		if (dmetchPoints != NULL && 0)
		{
			//for (int i=0;i<25;i++)
			//{
			//	glColor3d(1.0, 0.5, 0.5);
			//	glPushMatrix();
			//	glTranslatef(visual->tNeiPoints[i].x, visual->tNeiPoints[i].y, visual->tNeiPoints[i].z);

			//	glutSolidSphere(0.5, 40, 40);
			//	glPopMatrix();
			//	glColor3d(0.5, 0.5, 1);
			//	glPushMatrix();
			//	glTranslatef(visual->sNeiPoints[i].x, visual->sNeiPoints[i].y, visual->sNeiPoints[i].z);

			//	glutSolidSphere(0.5, 40, 40);
			//	glPopMatrix();
			//}
			glPushMatrix();
			//glTranslatef(visual->vertex[27568].x, visual->vertex[27568].y, visual->vertex[27568].z);
			//glTranslatef(dmetchPoints[m].p1.x, dmetchPoints[m].p1.y, dmetchPoints[m].p1.z);

			//glutSolidSphere(6.0, 40, 40);
			glPopMatrix();
			int mtemp = m_Model.mechSize;// sizeof(*dmetchPoints) / sizeof(dmetchPoints[0]);
			for (int m = 0; m<mtemp; m++)
			{

				if (dmetchPoints[m].statu == 0)
				{
					glColor3d(1.0, 0.0, 0.0);

				}else if (dmetchPoints[m].statu == 3)
				{
					glColor3d(0.0, 1.0, 0.0);
				}else if (dmetchPoints[m].statu == 2)
				{
					glColor3d(0.0, 0.0, 1.0);
				}
				else
					glColor3d(1.0, 1.0, 0.0);
				glPushMatrix();
				glTranslatef(dmetchPoints[m].p1.x, dmetchPoints[m].p1.y, dmetchPoints[m].p1.z);
				//glTranslatef(dmetchPoints[m].p1.x, dmetchPoints[m].p1.y, dmetchPoints[m].p1.z);

				glutSolidSphere(2, 40, 40);
				glPopMatrix();

				if (dmetchPoints[m].statu == 0)
				{
					glColor3d(1.0, 0.0, 0.0);

				}
				else if (dmetchPoints[m].statu == 3)
				{
					glColor3d(0.0, 1.0, 0.0);
				}
				else if (dmetchPoints[m].statu == 2)
				{
					glColor3d(0.0, 0.0, 1.0);
				}
				else
					glColor3d(1.0, 1.0, 0.0);
				glPushMatrix();
				glTranslatef(dmetchPoints[m].p2.x, dmetchPoints[m].p2.y, dmetchPoints[m].p2.z);
				glutSolidSphere(2, 40, 40);
				glPopMatrix();

				/*glVertex3f(dmetchPoints[m].p1.x, dmetchPoints[m].p1.y, dmetchPoints[m].p1.z);
				glVertex3f(dmetchPoints[m].p2.x, dmetchPoints[m].p2.y, dmetchPoints[m].p2.z);*/

			}

		}

		glEnd();
		glPopMatrix();
		/*if (m_Model.mechindexs != NULL&&m_Model.mechSize > 0)
		{

		drawMechPoints(&m_Model, &m_GModel);
		}*/
		glFlush();
		/////////////////////////////////////////add 2014.10.16
		if (Minpoint)
		{
			glColor3f(1, 0, 0);
			glPointSize(8.0);	//set the point size to 4 by 4 pixels
			glEnable(GL_POINT_SMOOTH);//没这句话点就不圆滑
									  //画点
			glBegin(GL_POINTS);
			for (loop = 0; loop < visual->faceCnt; loop++)
			{
				if (visual->vertex[visual->index[loop].v[0]].y == XminY ||
					visual->vertex[visual->index[loop].v[0]].x == Xmin0)
				{
					glVertex3f(visual->vertex[visual->index[loop].v[0]].x,
						visual->vertex[visual->index[loop].v[0]].y,
						visual->vertex[visual->index[loop].v[0]].z);
				}
				if (visual->vertex[visual->index[loop].v[1]].y == XminY ||
					visual->vertex[visual->index[loop].v[1]].x == Xmin0)
				{
					glVertex3f(visual->vertex[visual->index[loop].v[1]].x,
						visual->vertex[visual->index[loop].v[1]].y,
						visual->vertex[visual->index[loop].v[1]].z);
				}
				if (visual->vertex[visual->index[loop].v[2]].y == XminY ||
					visual->vertex[visual->index[loop].v[2]].x == Xmin0)
				{
					glVertex3f(visual->vertex[visual->index[loop].v[2]].x,
						visual->vertex[visual->index[loop].v[2]].y,
						visual->vertex[visual->index[loop].v[2]].z);
				}

			}
			//if(NumofObj<=1)
			glEnd();
			glFlush();
			glColor3f(1.0f, 1.0f, 1.0f);


		}

		////////////////////////////////////////end 2014.10.16	
		//////////////////////20150822/////libin//////////////////////////////////

		//glColor3f(1, 0, 0);
		//glPointSize(10.0);	//set the point size to 4 by 4 pixels
		//glEnable(GL_POINT_SMOOTH);//没这句话点就不圆滑

		////画点
		//glBegin(GL_POINTS);	
		//glVertex3f(last_x1,last_y1,last_z1);
		//glVertex3f(last_x2,last_y2,last_z2);
		//glVertex3f(last_x3,last_y3,last_z3);
		//glVertex3f(last_x4,last_y4,last_z4);
		//glVertex3f(last_x5,last_y5,last_z5);
		//
		//glEnd();

		//glFlush();
		//glColor3f(1.0f, 1.0f, 1.0f);

		//////////////////////////////////////////////////////////////////////////


	}
	//if (visual->vertex != NULL&&visual->normal != NULL)
	//{
	//	glBegin(GL_LINES/*GL_LINE_STRIP*/);							// 开始绘制物体(三角形)
	//	glColor3f(0.60f, 0.0f, 0.00f);
	//	
	//	for (loop = 0; loop < visual->faceCnt; loop++)
	//	{

	//		point1[0] = visual->vertex[visual->index[loop].v[0]].x;
	//		point1[1] = visual->vertex[visual->index[loop].v[0]].y;
	//		point1[2] = visual->vertex[visual->index[loop].v[0]].z;
	//		point2[0] = visual->vertex[visual->index[loop].v[1]].x;
	//		point2[1] = visual->vertex[visual->index[loop].v[1]].y;
	//		point2[2] = visual->vertex[visual->index[loop].v[1]].z;
	//		point3[0] = visual->vertex[visual->index[loop].v[2]].x;
	//		point3[1] = visual->vertex[visual->index[loop].v[2]].y;
	//		point3[2] = visual->vertex[visual->index[loop].v[2]].z;

	//		glColor3f(1.0f, 0.0f, 0.00f);
	//		glVertex3d(point1[0],
	//			point1[1],
	//			point1[2]);
	//		glColor3f(0.00f, 0.0f, 1.0f);

	//		glVertex3d(point1[0] + visual->normal[visual->index[loop].n[0]].x*0.1,
	//			point1[1] + visual->normal[visual->index[loop].n[0]].y*0.1,
	//			point1[2] + visual->normal[visual->index[loop].n[0]].z*0.1);
	//		glColor3f(1.0f, 0.0f, 0.00f);
	//		glVertex3d(point2[0],
	//			point2[1],
	//			point2[2]);
	//		glColor3f(0.00f, 0.0f, 1.0f);
	//		glVertex3d(point2[0] + visual->normal[visual->index[loop].n[1]].x*0.1,
	//			point2[1] + visual->normal[visual->index[loop].n[1]].y*0.1,
	//			point2[2] + visual->normal[visual->index[loop].n[1]].z*0.1);
	//		glColor3f(1.0f, 0.0f, 0.00f);
	//		glVertex3d(point3[0],
	//			point3[1],
	//			point3[2]);
	//		glColor3f(0.00f, 0.0f, 1.0f);
	//		glVertex3d(point3[0] + visual->normal[visual->index[loop].n[2]].x*0.1,
	//			point3[1] + visual->normal[visual->index[loop].n[2]].y*0.1,
	//			point3[2] + visual->normal[visual->index[loop].n[2]].z*0.1);
	//	}
	//	glEnd();

	//}

	filessss.close();
}
GLvoid COGLView::drawModel22(t_Visual *visual)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	tMatrix mat;		// Needed for Lighting Calc
	int loop;
	t_faceIndex *face;
	float   u;
	double point1[3], point2[3], point3[3], vnormal[3] = { 0 };
	double guassvalue1[3], guassvalue2[3], guassvalue3[3];

	glCallList(OGL_AXIS_DLIST);
	glFlush();
	fstream filessss;
	filessss.open("record001.txt", ios::app | ios::out);
	///////////////////////////////////////////////////////////////////////////////
	if (visual->vertex != NULL)
	{

		glPushMatrix();
		glClearColor(1.0, 1.0, 1.0, 0.5);
		glPointSize(1.5);
		glDisable(GL_LIGHTING);

		if (visual->TextureStatu == 2 && view_flag == 2)//draw texture
		{
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, textureyx);
			glDisable(GL_LIGHTING);
			glBegin(GL_TRIANGLES);
			glColor3f(1.0f, 1.0f, 1.0f);
		}
		else if (visual->TextureStatu == 2 && view_flag == 1)//draw mesh
		{
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, textureyx1);
			glDisable(GL_LIGHTING);
			glBegin(GL_TRIANGLES);
			glColor3f(1.0f, 1.0f, 1.0f);

		}
		else if (view_flag == 0)//画点
		{
			glDisable(GL_TEXTURE_2D);
			glColor3f(0.0f, 1.0f, 0.0f);
			glBegin(GL_POINTS);

		}
		else if (visual->TextureStatu == 1)
		{
			//glDisable(GL_LIGHTING);
			glEnable(GL_LIGHTING);
			//glDisable(GL_TEXTURE_2D);
			glColor3f(1.0f, 1.0f, 1.0f);
			glBegin(GL_TRIANGLES);
		}
		else
		{
			//glEnable(GL_LIGHTING);
			//glColor3f(0.0f, 1.0f, 0.0f);
			glBegin(GL_TRIANGLES);

		}

		//if (visual->TextureStatu == 2 && view_flag == 2)
		//{
		//	glEnable(GL_TEXTURE_2D);
		//	glBindTexture(GL_TEXTURE_2D, textureyx);
		//	glDisable(GL_LIGHTING);
		//	glBegin(GL_TRIANGLES);
		//	glColor3f(1.0f, 1.0f, 1.0f);
		//}
		//else if (view_flag == 0)//画点
		//{
		//	//glDisable(GL_TEXTURE_2D);
		//	glColor3f(1.0f, 0.0f, 0.0f);
		//	glBegin(GL_POINTS);

		//}
		//else if (visual->TextureStatu == 1)
		//{
		//	//glDisable(GL_LIGHTING);
		//	//glEnable(GL_LIGHTING);
		//	//glDisable(GL_TEXTURE_2D);
		//	glColor3f(1.0f, 1.0f, 1.0f);
		//	glBegin(GL_TRIANGLES);
		//}
		//else
		//{
		//	glColor3f(1.0f, 0.0f, 0.0f);
		//	glBegin(GL_TRIANGLES);

		//}
		/*glDisable(GL_TEXTURE_2D);
		glDeleteTextures(1,&textureyx);*/
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);							// 开始绘制物体(三角形)
		//if (visual->GuassMaxValue==0.0&&visual->GuassMinValue==0.0)//判断是否是加载曲率颜色
		//{
		for (loop = 0; loop < visual->faceCnt; loop++)
		{

			point1[0] = visual->vertex[visual->index[loop].v[0]].x;
			point1[1] = visual->vertex[visual->index[loop].v[0]].y;
			point1[2] = visual->vertex[visual->index[loop].v[0]].z;
			point2[0] = visual->vertex[visual->index[loop].v[1]].x;
			point2[1] = visual->vertex[visual->index[loop].v[1]].y;
			point2[2] = visual->vertex[visual->index[loop].v[1]].z;
			point3[0] = visual->vertex[visual->index[loop].v[2]].x;
			point3[1] = visual->vertex[visual->index[loop].v[2]].y;
			point3[2] = visual->vertex[visual->index[loop].v[2]].z;

			//m3dFindNormal(vnormal, point1, point2, point3);
			//glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
			/*glTexCoord2f(visual->texture[visual->index[loop].t[0]].u,
			visual->texture[visual->index[loop].t[0]].v);*/
			if (visual->normalCnt == visual->vertexCnt)
			{
				glNormal3f(visual->normal[visual->index[loop].v[0]].x, visual->normal[visual->index[loop].v[0]].y, visual->normal[visual->index[loop].v[0]].z);
				//glNormal3f(visual->normal[visual->index[loop].n[0]].x, visual->normal[visual->index[loop].n[0]].y, visual->normal[visual->index[loop].n[0]].z);
			}
			else
			{
				m3dFindNormal(vnormal, point1, point2, point3);
				//visual->normal[visual->index[loop].n[0]].x=vnormal[0];
				//visual->normal[visual->index[loop].n[0]].y=vnormal[0];
				//visual->normal[visual->index[loop].n[0]].z=vnormal[2];
				//glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

			}
			if (visual->TextureStatu == 1)//加载顶点颜色
			{
				glColor3d(visual->vertexColor[visual->index[loop].v[0]].r / 255,
					visual->vertexColor[visual->index[loop].v[0]].b / 255,
					visual->vertexColor[visual->index[loop].v[0]].g / 255);
				glVertex3d(point1[0],
					point1[1],
					point1[2]);

				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
				}
				glColor3d(visual->vertexColor[visual->index[loop].v[1]].r / 255,
					visual->vertexColor[visual->index[loop].v[1]].b / 255,
					visual->vertexColor[visual->index[loop].v[1]].g / 255);
				glVertex3d(point2[0],
					point2[1],
					point2[2]);

				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
				}
				glColor3d(visual->vertexColor[visual->index[loop].v[2]].r / 255,
					visual->vertexColor[visual->index[loop].v[2]].b / 255,
					visual->vertexColor[visual->index[loop].v[2]].g / 255);
				glVertex3d(point3[0],
					point3[1],
					point3[2]);
			}
			else if (/*visual->TextureStatu == 2&&*/view_flag == 0)
			{

				glVertex3d(point1[0], point1[1], point1[2]);

				glVertex3d(point2[0], point2[1], point2[2]);

				glVertex3d(point3[0], point3[1], point3[2]);

			}
			else if (visual->TextureStatu == 2)
			{
				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[0]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
				glTexCoord2f(visual->texture[visual->index[loop].t[0]].u,
					visual->texture[visual->index[loop].t[0]].v);

				glVertex3d(point1[0],
					point1[1],
					point1[2]);

				glTexCoord2f(visual->texture[visual->index[loop].t[1]].u,
					visual->texture[visual->index[loop].t[1]].v);
				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[1]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/

				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
				}
				else
				{
					//m3dFindNormal(vnormal, point1, point2, point3);
					//glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

				}
				glVertex3d(point2[0],
					point2[1],
					point2[2]);

				glTexCoord2f(visual->texture[visual->index[loop].t[2]].u,
					visual->texture[visual->index[loop].t[2]].v);
				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[2]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/

				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
				}
				else
				{
					//m3dFindNormal(vnormal, point1, point2, point3);
					//glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

				}
				glVertex3d(point3[0],
					point3[1],
					point3[2]);

			}
			else if (visual->TextureStatu == 30)
			{
				double tempNum = visual->vertexMax_z + 2;
				for (loop = 0; loop < visual->faceCnt; loop++)
				{
					point1[0] = visual->vertex[visual->index[loop].v[0]].x;
					point1[1] = visual->vertex[visual->index[loop].v[0]].y;
					point1[2] = visual->vertex[visual->index[loop].v[0]].z;
					if (visual->GuassCurvature[visual->index[loop].v[0]]>0 && visual->GuassCurvature[visual->index[loop].v[0]]<1000000)
					{//红
						guassvalue1[0] = 1;//visual->GuassCurvature[visual->index[loop].v[0]] / 5 > 1 ? 1 : visual->GuassCurvature[visual->index[loop].v[0]] / 5;
						guassvalue1[1] = 0;
						guassvalue1[2] = 0;
					}
					else if (visual->GuassCurvature[visual->index[loop].v[0]]<0 && visual->GuassCurvature[visual->index[loop].v[0]]>-1000000)
					{//绿
						guassvalue1[0] = 0;
						guassvalue1[1] = 1;//visual->GuassCurvature[visual->index[loop].v[0]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[0]] / (-4);
						guassvalue1[2] = 0;

					}
					else
					{
						guassvalue1[0] = 0;
						guassvalue1[1] = 0;
						guassvalue1[2] = 1;

					}


					point2[0] = visual->vertex[visual->index[loop].v[1]].x;
					point2[1] = visual->vertex[visual->index[loop].v[1]].y;
					point2[2] = visual->vertex[visual->index[loop].v[1]].z;
					//guassvalue1[1] = visual->GuassCurvature[visual->index[loop].v[1]];
					if (visual->GuassCurvature[visual->index[loop].v[1]] > 0 && visual->GuassCurvature[visual->index[loop].v[1]]<1000000)//此处1000000专为我的模型设置可删除
					{
						guassvalue2[0] = 1;//visual->GuassCurvature[visual->index[loop].v[1]] /5 > 1 ? 1 : visual->GuassCurvature[visual->index[loop].v[1]] / 5;
						guassvalue2[1] = 0;
						guassvalue2[2] = 0;
					}
					else if (visual->GuassCurvature[visual->index[loop].v[1]] < 0 && visual->GuassCurvature[visual->index[loop].v[1]]>-1000000)
					{
						guassvalue2[0] = 0;
						guassvalue2[1] = 1;//visual->GuassCurvature[visual->index[loop].v[1]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[1]] /( -4) ;
						guassvalue2[2] = 0;

					}
					else
					{
						guassvalue2[0] = 0;
						guassvalue2[1] = 0;
						guassvalue2[2] = 1;
					}

					point3[0] = visual->vertex[visual->index[loop].v[2]].x;
					point3[1] = visual->vertex[visual->index[loop].v[2]].y;
					point3[2] = visual->vertex[visual->index[loop].v[2]].z;
					if (visual->GuassCurvature[visual->index[loop].v[2]] > 0 && visual->GuassCurvature[visual->index[loop].v[2]]<1000000)
					{
						guassvalue3[0] = 1;// visual->GuassCurvature[visual->index[loop].v[2]] / 5>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[2]] / 5 ;
						guassvalue3[1] = 0;
						guassvalue3[2] = 0;
					}
					else if (visual->GuassCurvature[visual->index[loop].v[2]] < 0 && visual->GuassCurvature[visual->index[loop].v[2]]>-1000000)
					{
						guassvalue3[0] = 0;
						guassvalue3[1] = 1;//visual->GuassCurvature[visual->index[loop].v[2]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[2]] / (-4) ;
						guassvalue3[2] = 0;

					}
					else
					{
						guassvalue3[0] = 0;
						guassvalue3[1] = 0;
						guassvalue3[2] = 1;
					}
					//guassvalue[2] = visual->GuassCurvature[visual->index[loop].v[2]];
					if (point1[2] == tempNum || point2[2] == tempNum || point3[2] == tempNum)
					{
						continue;
					}

					//m3dFindNormal(vnormal, point1, point2, point3);
					//glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
					/*glTexCoord2f(visual->texture[visual->index[loop].t[0]].u,
					visual->texture[visual->index[loop].t[0]].v);*/
					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[0]].x, visual->normal[visual->index[loop].v[0]].y, visual->normal[visual->index[loop].v[0]].z);
					}
					else
					{
						m3dFindNormal(vnormal, point1, point2, point3);
						glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

					}
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[0]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue1[0], guassvalue1[1], guassvalue1[2]);
					glVertex3d(point1[0], point1[1], point1[2]);


					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
					}
					/*glTexCoord2f(visual->texture[visual->index[loop].t[1]].u,
					visual->texture[visual->index[loop].t[1]].v);*/
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[1]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue2[0], guassvalue2[1], guassvalue2[2]);
					glVertex3d(point2[0], point2[1], point2[2]);


					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
					}
					/*glTexCoord2f(visual->texture[visual->index[loop].t[2]].u,
					visual->texture[visual->index[loop].t[2]].v);*/
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[2]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue3[0], guassvalue3[1], guassvalue3[2]);
					glVertex3d(point3[0], point3[1], point3[2]);
					//if  (filessss)
					//{
					//	filessss << point1[0] << "\t" << point1[1] << "\t" << point1[2] << "\t" << guassvalue1[0] << "\t" << guassvalue1[1] << "\t" << guassvalue1[2] << endl;
					//	filessss << point2[0] << "\t" << point2[1] << "\t" << point2[2] << "\t" << guassvalue2[0] << "\t" << guassvalue2[1] << "\t" << guassvalue2[2] << endl;
					//	filessss << point3[0] << "\t" << point3[1] << "\t" << point3[2] << "\t" << guassvalue3[0] << "\t" << guassvalue3[1] << "\t" << guassvalue3[2] << endl;
					//}
				}

			}
			else if (visual->TextureStatu == 31)
			{
				double tempNum = visual->vertexMax_z + 2;
				for (loop = 0; loop < visual->faceCnt; loop++)
				{
					point1[0] = visual->vertex[visual->index[loop].v[0]].x;
					point1[1] = visual->vertex[visual->index[loop].v[0]].y;
					point1[2] = visual->vertex[visual->index[loop].v[0]].z;
					if (visual->HCurvature[visual->index[loop].v[0]]>10 && visual->HCurvature[visual->index[loop].v[0]]<1000000)
					{//红
						guassvalue1[0] = 1;//visual->GuassCurvature[visual->index[loop].v[0]] / 5 > 1 ? 1 : visual->GuassCurvature[visual->index[loop].v[0]] / 5;
						guassvalue1[1] = 0;
						guassvalue1[2] = 0;
					}
					else if (visual->HCurvature[visual->index[loop].v[0]] <= 10 && visual->HCurvature[visual->index[loop].v[0]]>0.5)
					{//绿
						guassvalue1[0] = 1;
						guassvalue1[1] = 0;//visual->GuassCurvature[visual->index[loop].v[0]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[0]] / (-4);
						guassvalue1[2] = 0;

					}
					else if (visual->HCurvature[visual->index[loop].v[0]] <= 0.5 && visual->HCurvature[visual->index[loop].v[0]] >= 0)
					{
						guassvalue1[0] = 0;
						guassvalue1[1] = 1;
						guassvalue1[2] = 0;

					}
					else
					{
						guassvalue1[0] = 0;
						guassvalue1[1] = 0;
						guassvalue1[2] = 1;
					}


					point2[0] = visual->vertex[visual->index[loop].v[1]].x;
					point2[1] = visual->vertex[visual->index[loop].v[1]].y;
					point2[2] = visual->vertex[visual->index[loop].v[1]].z;
					//guassvalue1[1] = visual->GuassCurvature[visual->index[loop].v[1]];
					if (visual->HCurvature[visual->index[loop].v[1]] > 10 && visual->HCurvature[visual->index[loop].v[1]]<1000000)//此处1000000专为我的模型设置可删除
					{
						guassvalue2[0] = 1;//visual->GuassCurvature[visual->index[loop].v[1]] /5 > 1 ? 1 : visual->GuassCurvature[visual->index[loop].v[1]] / 5;
						guassvalue2[1] = 0;
						guassvalue2[2] = 0;
					}
					else if (visual->HCurvature[visual->index[loop].v[1]] <= 10 && visual->HCurvature[visual->index[loop].v[1]]>0.5)
					{
						guassvalue2[0] = 1;
						guassvalue2[1] = 0;//visual->GuassCurvature[visual->index[loop].v[1]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[1]] /( -4) ;
						guassvalue2[2] = 0;

					}
					else if (visual->HCurvature[visual->index[loop].v[1]] <= 0.5 && visual->HCurvature[visual->index[loop].v[1]] >= 0)
					{
						guassvalue2[0] = 0;
						guassvalue2[1] = 1;
						guassvalue2[2] = 0;
					}
					else
					{
						guassvalue2[0] = 0;
						guassvalue2[1] = 0;
						guassvalue2[2] = 1;
					}

					point3[0] = visual->vertex[visual->index[loop].v[2]].x;
					point3[1] = visual->vertex[visual->index[loop].v[2]].y;
					point3[2] = visual->vertex[visual->index[loop].v[2]].z;
					if (visual->HCurvature[visual->index[loop].v[2]] > 10 && visual->HCurvature[visual->index[loop].v[2]]<1000000)
					{
						guassvalue3[0] = 1;// visual->GuassCurvature[visual->index[loop].v[2]] / 5>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[2]] / 5 ;
						guassvalue3[1] = 0;
						guassvalue3[2] = 0;
					}
					else if (visual->HCurvature[visual->index[loop].v[2]] <= 10 && visual->HCurvature[visual->index[loop].v[2]]>0.5)
					{
						guassvalue3[0] = 1;
						guassvalue3[1] = 0;//visual->GuassCurvature[visual->index[loop].v[2]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[2]] / (-4) ;
						guassvalue3[2] = 0;

					}
					else if (visual->HCurvature[visual->index[loop].v[2]] <= 0.5 && visual->HCurvature[visual->index[loop].v[2]] >= 0)
					{
						guassvalue3[0] = 0;
						guassvalue3[1] = 1;
						guassvalue3[2] = 0;
					}
					else
					{
						guassvalue3[0] = 0;
						guassvalue3[1] = 0;
						guassvalue3[2] = 1;
					}
					//guassvalue[2] = visual->GuassCurvature[visual->index[loop].v[2]];
					if (point1[2] == tempNum || point2[2] == tempNum || point3[2] == tempNum)
					{
						continue;
					}

					//m3dFindNormal(vnormal, point1, point2, point3);
					//glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
					/*glTexCoord2f(visual->texture[visual->index[loop].t[0]].u,
					visual->texture[visual->index[loop].t[0]].v);*/
					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[0]].x, visual->normal[visual->index[loop].v[0]].y, visual->normal[visual->index[loop].v[0]].z);
					}
					else
					{
						m3dFindNormal(vnormal, point1, point2, point3);
						glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

					}
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[0]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue1[0], guassvalue1[1], guassvalue1[2]);
					glVertex3d(point1[0], point1[1], point1[2]);


					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
					}
					else
					{
						m3dFindNormal(vnormal, point1, point2, point3);
						glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

					}
					/*glTexCoord2f(visual->texture[visual->index[loop].t[1]].u,
					visual->texture[visual->index[loop].t[1]].v);*/
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[1]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue2[0], guassvalue2[1], guassvalue2[2]);
					glVertex3d(point2[0], point2[1], point2[2]);


					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
					}
					else
					{
						m3dFindNormal(vnormal, point1, point2, point3);
						glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

					}
					/*glTexCoord2f(visual->texture[visual->index[loop].t[2]].u,
					visual->texture[visual->index[loop].t[2]].v);*/
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[2]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue3[0], guassvalue3[1], guassvalue3[2]);
					glVertex3d(point3[0], point3[1], point3[2]);
					//if  (filessss)
					//{
					//	filessss << point1[0] << "\t" << point1[1] << "\t" << point1[2] << "\t" << guassvalue1[0] << "\t" << guassvalue1[1] << "\t" << guassvalue1[2] << endl;
					//	filessss << point2[0] << "\t" << point2[1] << "\t" << point2[2] << "\t" << guassvalue2[0] << "\t" << guassvalue2[1] << "\t" << guassvalue2[2] << endl;
					//	filessss << point3[0] << "\t" << point3[1] << "\t" << point3[2] << "\t" << guassvalue3[0] << "\t" << guassvalue3[1] << "\t" << guassvalue3[2] << endl;
					//}
				}

			}
			else
			{
				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[0]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/


				//glColor3d(1.0f, 1.0f, 1.0f);
				glVertex3d(point1[0],
					point1[1],
					point1[2]);

				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[1]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
				}
				else
				{
					glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
				}
				//glColor3d(1.0f, 1.0f, 1.0f);
				glVertex3d(point2[0],
					point2[1],
					point2[2]);

				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[2]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
				}
				else
				{
					glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
				}
				//glColor3d(1.0f, 1.0f, 1.0f);
				glVertex3d(point3[0],
					point3[1],
					point3[2]);

			}

		}
		//glutSolidSphere(10,40,40);
		//}
		glEnd();
		glPopMatrix();
		glDisable(GL_TEXTURE_2D);
		//glColor3d(1.0, 1.0, 0.0);
		glPushMatrix();
		glBegin(GL_LINE);
		if (/*dmetchPoints != NULL )//&&*/ 0)
		{
			//for (int i=0;i<25;i++)
			//{
			//	glColor3d(1.0, 0.5, 0.5);
			//	glPushMatrix();
			//	glTranslatef(visual->tNeiPoints[i].x, visual->tNeiPoints[i].y, visual->tNeiPoints[i].z);

			//	glutSolidSphere(0.5, 40, 40);
			//	glPopMatrix();
			//	glColor3d(0.5, 0.5, 1);
			//	glPushMatrix();
			//	glTranslatef(visual->sNeiPoints[i].x, visual->sNeiPoints[i].y, visual->sNeiPoints[i].z);

			//	glutSolidSphere(0.5, 40, 40);
			//	glPopMatrix();
			//}
			//glPushMatrix();
			//glTranslatef(visual->vertex[27568].x, visual->vertex[27568].y, visual->vertex[27568].z);
			////glTranslatef(dmetchPoints[m].p1.x, dmetchPoints[m].p1.y, dmetchPoints[m].p1.z);

			//glutSolidSphere(6.0, 40, 40);
			//glPopMatrix();
			int mtemp = m_Model.mechSize;// sizeof(*dmetchPoints) / sizeof(dmetchPoints[0]);
			for (int m = 0; m<mtemp; m++)
			{

				if (dmetchPoints[m].statu == 0)
				{
					glColor3d(1.0, 0.0, 0.0);

				}
				else
					glColor3d(1.0, 1.0, 0.0);
				glPushMatrix();
				glTranslatef(dmetchPoints[m].p1.x, dmetchPoints[m].p1.y, dmetchPoints[m].p1.z);
				//glTranslatef(dmetchPoints[m].p1.x, dmetchPoints[m].p1.y, dmetchPoints[m].p1.z);

				glutSolidSphere(2, 40, 40);
				glPopMatrix();

				if (dmetchPoints[m].statu == 0)
				{
					glColor3d(1.0, 0.0, 0.0);

				}
				else
					glColor3d(0.0, 1.0, 1.0);
				glPushMatrix();
				glTranslatef(dmetchPoints[m].p2.x, dmetchPoints[m].p2.y, dmetchPoints[m].p2.z);
				glutSolidSphere(2, 40, 40);
				glPopMatrix();

				/*glVertex3f(dmetchPoints[m].p1.x, dmetchPoints[m].p1.y, dmetchPoints[m].p1.z);
				glVertex3f(dmetchPoints[m].p2.x, dmetchPoints[m].p2.y, dmetchPoints[m].p2.z);*/

			}

		}

		glEnd();
		glPopMatrix();
		/*if (m_Model.mechindexs != NULL&&m_Model.mechSize > 0)
		{

		drawMechPoints(&m_Model, &m_GModel);
		}*/
		glFlush();
		if (Minpoint)
		{
			glColor3f(1, 0, 0);
			glPointSize(8.0);	//set the point size to 4 by 4 pixels
			glEnable(GL_POINT_SMOOTH);//没这句话点就不圆滑
									  //画点
			glBegin(GL_POINTS);
			for (loop = 0; loop < visual->faceCnt; loop++)
			{
				if (visual->vertex[visual->index[loop].v[0]].y == XminY ||
					visual->vertex[visual->index[loop].v[0]].x == Xmin0)
				{
					glVertex3f(visual->vertex[visual->index[loop].v[0]].x,
						visual->vertex[visual->index[loop].v[0]].y,
						visual->vertex[visual->index[loop].v[0]].z);
				}
				if (visual->vertex[visual->index[loop].v[1]].y == XminY ||
					visual->vertex[visual->index[loop].v[1]].x == Xmin0)
				{
					glVertex3f(visual->vertex[visual->index[loop].v[1]].x,
						visual->vertex[visual->index[loop].v[1]].y,
						visual->vertex[visual->index[loop].v[1]].z);
				}
				if (visual->vertex[visual->index[loop].v[2]].y == XminY ||
					visual->vertex[visual->index[loop].v[2]].x == Xmin0)
				{
					glVertex3f(visual->vertex[visual->index[loop].v[2]].x,
						visual->vertex[visual->index[loop].v[2]].y,
						visual->vertex[visual->index[loop].v[2]].z);
				}

			}
			//if(NumofObj<=1)
			glEnd();
			glFlush();
			glColor3f(1.0f, 1.0f, 1.0f);


		}


		//glColor3f(1, 0, 0);
		//glPointSize(10.0);	//set the point size to 4 by 4 pixels
		//glEnable(GL_POINT_SMOOTH);//没这句话点就不圆滑

		////画点
		//glBegin(GL_POINTS);	
		//glVertex3f(last_x1,last_y1,last_z1);
		//glVertex3f(last_x2,last_y2,last_z2);
		//glVertex3f(last_x3,last_y3,last_z3);
		//glVertex3f(last_x4,last_y4,last_z4);
		//glVertex3f(last_x5,last_y5,last_z5);
		//
		//glEnd();

		//glFlush();
		//glColor3f(1.0f, 1.0f, 1.0f);

		//////////////////////////////////////////////////////////////////////////


	}
	//if (visual->vertex != NULL&&visual->normal != NULL)
	//{
	//	glBegin(GL_LINES/*GL_LINE_STRIP*/);							// 开始绘制物体(三角形)
	//	glColor3f(0.60f, 0.0f, 0.00f);
	//	
	//	for (loop = 0; loop < visual->faceCnt; loop++)
	//	{

	//		point1[0] = visual->vertex[visual->index[loop].v[0]].x;
	//		point1[1] = visual->vertex[visual->index[loop].v[0]].y;
	//		point1[2] = visual->vertex[visual->index[loop].v[0]].z;
	//		point2[0] = visual->vertex[visual->index[loop].v[1]].x;
	//		point2[1] = visual->vertex[visual->index[loop].v[1]].y;
	//		point2[2] = visual->vertex[visual->index[loop].v[1]].z;
	//		point3[0] = visual->vertex[visual->index[loop].v[2]].x;
	//		point3[1] = visual->vertex[visual->index[loop].v[2]].y;
	//		point3[2] = visual->vertex[visual->index[loop].v[2]].z;

	//		glColor3f(1.0f, 0.0f, 0.00f);
	//		glVertex3d(point1[0],
	//			point1[1],
	//			point1[2]);
	//		glColor3f(0.00f, 0.0f, 1.0f);

	//		glVertex3d(point1[0] + visual->normal[visual->index[loop].n[0]].x*0.1,
	//			point1[1] + visual->normal[visual->index[loop].n[0]].y*0.1,
	//			point1[2] + visual->normal[visual->index[loop].n[0]].z*0.1);
	//		glColor3f(1.0f, 0.0f, 0.00f);
	//		glVertex3d(point2[0],
	//			point2[1],
	//			point2[2]);
	//		glColor3f(0.00f, 0.0f, 1.0f);
	//		glVertex3d(point2[0] + visual->normal[visual->index[loop].n[1]].x*0.1,
	//			point2[1] + visual->normal[visual->index[loop].n[1]].y*0.1,
	//			point2[2] + visual->normal[visual->index[loop].n[1]].z*0.1);
	//		glColor3f(1.0f, 0.0f, 0.00f);
	//		glVertex3d(point3[0],
	//			point3[1],
	//			point3[2]);
	//		glColor3f(0.00f, 0.0f, 1.0f);
	//		glVertex3d(point3[0] + visual->normal[visual->index[loop].n[2]].x*0.1,
	//			point3[1] + visual->normal[visual->index[loop].n[2]].y*0.1,
	//			point3[2] + visual->normal[visual->index[loop].n[2]].z*0.1);
	//	}
	//	glEnd();

	//}

	filessss.close();
}
//////////////////////////////////////////////////////////////////////////////2014.11.01
GLvoid COGLView::drawGModel(t_Visual *visual)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	tMatrix mat;		// Needed for Lighting Calc
	int loop;
	t_faceIndex *face;
	float   u;
	double point1[3], point2[3], point3[3], vnormal[3] = { 0 };
	double guassvalue1[3], guassvalue2[3], guassvalue3[3];
	///////////////////////////////////////////////////////////////////////////////

	glCallList(OGL_AXIS_DLIST);
	glFlush();
	if (visual->vertex != NULL)
	{

		glPushMatrix();
		
		glPointSize(1.5);

		if (visual->TextureStatu == 2 && view_flag == 2)//draw texture
		{
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, textureyx1);
			glDisable(GL_LIGHTING);
			glBegin(GL_TRIANGLES);
			glColor3f(1.0f, 1.0f, 1.0f);
		}
		else if (visual->TextureStatu == 2 && view_flag == 1)//draw mesh
		{
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, textureyx1);
			glDisable(GL_LIGHTING);
			glBegin(GL_TRIANGLES);
			glColor3f(1.0f, 1.0f, 1.0f);

		}
		else if (view_flag == 0)//画点
		{
			glDisable(GL_TEXTURE_2D);
			glColor3f(0.0f, 1.0f, 0.0f);
			glBegin(GL_POINTS);

		}
		else if (visual->TextureStatu == 1)
		{
			//glDisable(GL_LIGHTING);
			glEnable(GL_LIGHTING);
			//glDisable(GL_TEXTURE_2D);
			glColor3f(1.0f, 1.0f, 1.0f);
			glBegin(GL_TRIANGLES);
		}
		else
		{
			//glEnable(GL_LIGHTING);
			//glColor3f(0.0f, 1.0f, 0.0f);
			glBegin(GL_TRIANGLES);

		}



		for (loop = 0; loop < visual->faceCnt; loop++)
		{

			point1[0] = visual->vertex[visual->index[loop].v[0]].x;
			point1[1] = visual->vertex[visual->index[loop].v[0]].y;
			point1[2] = visual->vertex[visual->index[loop].v[0]].z;
			point2[0] = visual->vertex[visual->index[loop].v[1]].x;
			point2[1] = visual->vertex[visual->index[loop].v[1]].y;
			point2[2] = visual->vertex[visual->index[loop].v[1]].z;
			point3[0] = visual->vertex[visual->index[loop].v[2]].x;
			point3[1] = visual->vertex[visual->index[loop].v[2]].y;
			point3[2] = visual->vertex[visual->index[loop].v[2]].z;

			//m3dFindNormal(vnormal, point1, point2, point3);
			//glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
			/*glTexCoord2f(visual->texture[visual->index[loop].t[0]].u,
			visual->texture[visual->index[loop].t[0]].v);*/
			if (visual->normalCnt == visual->vertexCnt)
			{
				glNormal3f(visual->normal[visual->index[loop].v[0]].x, visual->normal[visual->index[loop].v[0]].y, visual->normal[visual->index[loop].v[0]].z);
				//glNormal3f(visual->normal[visual->index[loop].n[0]].x, visual->normal[visual->index[loop].n[0]].y, visual->normal[visual->index[loop].n[0]].z);
			}
			else
			{
				m3dFindNormal(vnormal, point1, point2, point3);
				//visual->normal[visual->index[loop].n[0]].x=vnormal[0];
				//visual->normal[visual->index[loop].n[0]].y=vnormal[0];
				//visual->normal[visual->index[loop].n[0]].z=vnormal[2];
				glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

			}
			if (visual->TextureStatu == 1)//加载顶点颜色
			{
				glColor3d(visual->vertexColor[visual->index[loop].v[0]].r / 255,
					visual->vertexColor[visual->index[loop].v[0]].b / 255,
					visual->vertexColor[visual->index[loop].v[0]].g / 255);
				glVertex3d(point1[0],
					point1[1],
					point1[2]);

				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
				}
				glColor3d(visual->vertexColor[visual->index[loop].v[1]].r / 255,
					visual->vertexColor[visual->index[loop].v[1]].b / 255,
					visual->vertexColor[visual->index[loop].v[1]].g / 255);
				glVertex3d(point2[0],
					point2[1],
					point2[2]);

				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
				}
				glColor3d(visual->vertexColor[visual->index[loop].v[2]].r / 255,
					visual->vertexColor[visual->index[loop].v[2]].b / 255,
					visual->vertexColor[visual->index[loop].v[2]].g / 255);
				glVertex3d(point3[0],
					point3[1],
					point3[2]);
			}
			else if (visual->TextureStatu == 21)
			{

				glVertex3d(point1[0], point1[1], point1[2]);

				glVertex3d(point2[0], point2[1], point2[2]);

				glVertex3d(point3[0], point3[1], point3[2]);

			}
			else if (visual->TextureStatu == 2)
			{
				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[0]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
				if (visual->index[loop].t[0]<0)
				{
					continue;
				}
				glTexCoord2f(visual->texture[visual->index[loop].t[0]].u,
					visual->texture[visual->index[loop].t[0]].v);

				glVertex3d(point1[0],
					point1[1],
					point1[2]);

				glTexCoord2f(visual->texture[visual->index[loop].t[1]].u,
					visual->texture[visual->index[loop].t[1]].v);
				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[1]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/

				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
				}
				else
				{
					m3dFindNormal(vnormal, point1, point2, point3);
					glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

				}
				glVertex3d(point2[0],
					point2[1],
					point2[2]);

				glTexCoord2f(visual->texture[visual->index[loop].t[2]].u,
					visual->texture[visual->index[loop].t[2]].v);
				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[2]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/

				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
				}
				else
				{
					m3dFindNormal(vnormal, point1, point2, point3);
					glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

				}
				glVertex3d(point3[0],
					point3[1],
					point3[2]);

			}
			else if (visual->TextureStatu == 30)
			{
				double tempNum = visual->vertexMax_z + 2;
				for (loop = 0; loop < visual->faceCnt; loop++)
				{
					point1[0] = visual->vertex[visual->index[loop].v[0]].x;
					point1[1] = visual->vertex[visual->index[loop].v[0]].y;
					point1[2] = visual->vertex[visual->index[loop].v[0]].z;
					if (visual->GuassCurvature[visual->index[loop].v[0]]>0 && visual->GuassCurvature[visual->index[loop].v[0]]<1000000)
					{//红
						guassvalue1[0] = 1;//visual->GuassCurvature[visual->index[loop].v[0]] / 5 > 1 ? 1 : visual->GuassCurvature[visual->index[loop].v[0]] / 5;
						guassvalue1[1] = 0;
						guassvalue1[2] = 0;
					}
					else if (visual->GuassCurvature[visual->index[loop].v[0]]<0 && visual->GuassCurvature[visual->index[loop].v[0]]>-1000000)
					{//绿
						guassvalue1[0] = 0;
						guassvalue1[1] = 1;//visual->GuassCurvature[visual->index[loop].v[0]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[0]] / (-4);
						guassvalue1[2] = 0;

					}
					else
					{
						guassvalue1[0] = 0;
						guassvalue1[1] = 0;
						guassvalue1[2] = 1;

					}


					point2[0] = visual->vertex[visual->index[loop].v[1]].x;
					point2[1] = visual->vertex[visual->index[loop].v[1]].y;
					point2[2] = visual->vertex[visual->index[loop].v[1]].z;
					//guassvalue1[1] = visual->GuassCurvature[visual->index[loop].v[1]];
					if (visual->GuassCurvature[visual->index[loop].v[1]] > 0 && visual->GuassCurvature[visual->index[loop].v[1]]<1000000)//此处1000000专为我的模型设置可删除
					{
						guassvalue2[0] = 1;//visual->GuassCurvature[visual->index[loop].v[1]] /5 > 1 ? 1 : visual->GuassCurvature[visual->index[loop].v[1]] / 5;
						guassvalue2[1] = 0;
						guassvalue2[2] = 0;
					}
					else if (visual->GuassCurvature[visual->index[loop].v[1]] < 0 && visual->GuassCurvature[visual->index[loop].v[1]]>-1000000)
					{
						guassvalue2[0] = 0;
						guassvalue2[1] = 1;//visual->GuassCurvature[visual->index[loop].v[1]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[1]] /( -4) ;
						guassvalue2[2] = 0;

					}
					else
					{
						guassvalue2[0] = 0;
						guassvalue2[1] = 0;
						guassvalue2[2] = 1;
					}

					point3[0] = visual->vertex[visual->index[loop].v[2]].x;
					point3[1] = visual->vertex[visual->index[loop].v[2]].y;
					point3[2] = visual->vertex[visual->index[loop].v[2]].z;
					if (visual->GuassCurvature[visual->index[loop].v[2]] > 0 && visual->GuassCurvature[visual->index[loop].v[2]]<1000000)
					{
						guassvalue3[0] = 1;// visual->GuassCurvature[visual->index[loop].v[2]] / 5>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[2]] / 5 ;
						guassvalue3[1] = 0;
						guassvalue3[2] = 0;
					}
					else if (visual->GuassCurvature[visual->index[loop].v[2]] < 0 && visual->GuassCurvature[visual->index[loop].v[2]]>-1000000)
					{
						guassvalue3[0] = 0;
						guassvalue3[1] = 1;//visual->GuassCurvature[visual->index[loop].v[2]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[2]] / (-4) ;
						guassvalue3[2] = 0;

					}
					else
					{
						guassvalue3[0] = 0;
						guassvalue3[1] = 0;
						guassvalue3[2] = 1;
					}
					//guassvalue[2] = visual->GuassCurvature[visual->index[loop].v[2]];
					if (point1[2] == tempNum || point2[2] == tempNum || point3[2] == tempNum)
					{
						continue;
					}

					//m3dFindNormal(vnormal, point1, point2, point3);
					//glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
					/*glTexCoord2f(visual->texture[visual->index[loop].t[0]].u,
					visual->texture[visual->index[loop].t[0]].v);*/
					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[0]].x, visual->normal[visual->index[loop].v[0]].y, visual->normal[visual->index[loop].v[0]].z);
					}
					else
					{
						m3dFindNormal(vnormal, point1, point2, point3);
						glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

					}
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[0]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue1[0], guassvalue1[1], guassvalue1[2]);
					glVertex3d(point1[0], point1[1], point1[2]);


					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
					}
					/*glTexCoord2f(visual->texture[visual->index[loop].t[1]].u,
					visual->texture[visual->index[loop].t[1]].v);*/
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[1]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue2[0], guassvalue2[1], guassvalue2[2]);
					glVertex3d(point2[0], point2[1], point2[2]);


					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
					}
					/*glTexCoord2f(visual->texture[visual->index[loop].t[2]].u,
					visual->texture[visual->index[loop].t[2]].v);*/
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[2]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue3[0], guassvalue3[1], guassvalue3[2]);
					glVertex3d(point3[0], point3[1], point3[2]);
					//if  (filessss)
					//{
					//	filessss << point1[0] << "\t" << point1[1] << "\t" << point1[2] << "\t" << guassvalue1[0] << "\t" << guassvalue1[1] << "\t" << guassvalue1[2] << endl;
					//	filessss << point2[0] << "\t" << point2[1] << "\t" << point2[2] << "\t" << guassvalue2[0] << "\t" << guassvalue2[1] << "\t" << guassvalue2[2] << endl;
					//	filessss << point3[0] << "\t" << point3[1] << "\t" << point3[2] << "\t" << guassvalue3[0] << "\t" << guassvalue3[1] << "\t" << guassvalue3[2] << endl;
					//}
				}

			}
			else if (visual->TextureStatu == 31)
			{
				double tempNum = visual->vertexMax_z + 2;
				for (loop = 0; loop < visual->faceCnt; loop++)
				{
					point1[0] = visual->vertex[visual->index[loop].v[0]].x;
					point1[1] = visual->vertex[visual->index[loop].v[0]].y;
					point1[2] = visual->vertex[visual->index[loop].v[0]].z;
					if (visual->HCurvature[visual->index[loop].v[0]]>10 && visual->HCurvature[visual->index[loop].v[0]]<1000000)
					{//红
						guassvalue1[0] = 1;//visual->GuassCurvature[visual->index[loop].v[0]] / 5 > 1 ? 1 : visual->GuassCurvature[visual->index[loop].v[0]] / 5;
						guassvalue1[1] = 0;
						guassvalue1[2] = 0;
					}
					else if (visual->HCurvature[visual->index[loop].v[0]] <= 10 && visual->HCurvature[visual->index[loop].v[0]]>0.5)
					{//绿
						guassvalue1[0] = 1;
						guassvalue1[1] = 0;//visual->GuassCurvature[visual->index[loop].v[0]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[0]] / (-4);
						guassvalue1[2] = 0;

					}
					else if (visual->HCurvature[visual->index[loop].v[0]] <= 0.5 && visual->HCurvature[visual->index[loop].v[0]] >= 0)
					{
						guassvalue1[0] = 0;
						guassvalue1[1] = 1;
						guassvalue1[2] = 0;

					}
					else
					{
						guassvalue1[0] = 0;
						guassvalue1[1] = 0;
						guassvalue1[2] = 1;
					}


					point2[0] = visual->vertex[visual->index[loop].v[1]].x;
					point2[1] = visual->vertex[visual->index[loop].v[1]].y;
					point2[2] = visual->vertex[visual->index[loop].v[1]].z;
					//guassvalue1[1] = visual->GuassCurvature[visual->index[loop].v[1]];
					if (visual->HCurvature[visual->index[loop].v[1]] > 10 && visual->HCurvature[visual->index[loop].v[1]]<1000000)//此处1000000专为我的模型设置可删除
					{
						guassvalue2[0] = 1;//visual->GuassCurvature[visual->index[loop].v[1]] /5 > 1 ? 1 : visual->GuassCurvature[visual->index[loop].v[1]] / 5;
						guassvalue2[1] = 0;
						guassvalue2[2] = 0;
					}
					else if (visual->HCurvature[visual->index[loop].v[1]] <= 10 && visual->HCurvature[visual->index[loop].v[1]]>0.5)
					{
						guassvalue2[0] = 1;
						guassvalue2[1] = 0;//visual->GuassCurvature[visual->index[loop].v[1]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[1]] /( -4) ;
						guassvalue2[2] = 0;

					}
					else if (visual->HCurvature[visual->index[loop].v[1]] <= 0.5 && visual->HCurvature[visual->index[loop].v[1]] >= 0)
					{
						guassvalue2[0] = 0;
						guassvalue2[1] = 1;
						guassvalue2[2] = 0;
					}
					else
					{
						guassvalue2[0] = 0;
						guassvalue2[1] = 0;
						guassvalue2[2] = 1;
					}

					point3[0] = visual->vertex[visual->index[loop].v[2]].x;
					point3[1] = visual->vertex[visual->index[loop].v[2]].y;
					point3[2] = visual->vertex[visual->index[loop].v[2]].z;
					if (visual->HCurvature[visual->index[loop].v[2]] > 10 && visual->HCurvature[visual->index[loop].v[2]]<1000000)
					{
						guassvalue3[0] = 1;// visual->GuassCurvature[visual->index[loop].v[2]] / 5>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[2]] / 5 ;
						guassvalue3[1] = 0;
						guassvalue3[2] = 0;
					}
					else if (visual->HCurvature[visual->index[loop].v[2]] <= 10 && visual->HCurvature[visual->index[loop].v[2]]>0.5)
					{
						guassvalue3[0] = 1;
						guassvalue3[1] = 0;//visual->GuassCurvature[visual->index[loop].v[2]] / (-4)>1 ? 1 : visual->GuassCurvature[visual->index[loop].v[2]] / (-4) ;
						guassvalue3[2] = 0;

					}
					else if (visual->HCurvature[visual->index[loop].v[2]] <= 0.5 && visual->HCurvature[visual->index[loop].v[2]] >= 0)
					{
						guassvalue3[0] = 0;
						guassvalue3[1] = 1;
						guassvalue3[2] = 0;
					}
					else
					{
						guassvalue3[0] = 0;
						guassvalue3[1] = 0;
						guassvalue3[2] = 1;
					}
					//guassvalue[2] = visual->GuassCurvature[visual->index[loop].v[2]];
					if (point1[2] == tempNum || point2[2] == tempNum || point3[2] == tempNum)
					{
						continue;
					}

					//m3dFindNormal(vnormal, point1, point2, point3);
					//glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
					/*glTexCoord2f(visual->texture[visual->index[loop].t[0]].u,
					visual->texture[visual->index[loop].t[0]].v);*/
					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[0]].x, visual->normal[visual->index[loop].v[0]].y, visual->normal[visual->index[loop].v[0]].z);
					}
					else
					{
						m3dFindNormal(vnormal, point1, point2, point3);
						glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

					}
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[0]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue1[0], guassvalue1[1], guassvalue1[2]);
					glVertex3d(point1[0], point1[1], point1[2]);


					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
					}
					else
					{
						m3dFindNormal(vnormal, point1, point2, point3);
						glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

					}
					/*glTexCoord2f(visual->texture[visual->index[loop].t[1]].u,
					visual->texture[visual->index[loop].t[1]].v);*/
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[1]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue2[0], guassvalue2[1], guassvalue2[2]);
					glVertex3d(point2[0], point2[1], point2[2]);


					if (visual->normalCnt == visual->vertexCnt)
					{
						glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
					}
					else
					{
						m3dFindNormal(vnormal, point1, point2, point3);
						glNormal3f(vnormal[0], vnormal[1], vnormal[2]);

					}
					/*glTexCoord2f(visual->texture[visual->index[loop].t[2]].u,
					visual->texture[visual->index[loop].t[2]].v);*/
					/*u = CalculateShadow(&visual->normal[visual->index[loop].n[2]], &m_ShadeLight, &mat);
					(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
					glColor3d(guassvalue3[0], guassvalue3[1], guassvalue3[2]);
					glVertex3d(point3[0], point3[1], point3[2]);
					//if  (filessss)
					//{
					//	filessss << point1[0] << "\t" << point1[1] << "\t" << point1[2] << "\t" << guassvalue1[0] << "\t" << guassvalue1[1] << "\t" << guassvalue1[2] << endl;
					//	filessss << point2[0] << "\t" << point2[1] << "\t" << point2[2] << "\t" << guassvalue2[0] << "\t" << guassvalue2[1] << "\t" << guassvalue2[2] << endl;
					//	filessss << point3[0] << "\t" << point3[1] << "\t" << point3[2] << "\t" << guassvalue3[0] << "\t" << guassvalue3[1] << "\t" << guassvalue3[2] << endl;
					//}
				}

			}


			else
			{
				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[0]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/


				glColor3d(0.0f, 1.0f, 0.0f);
				glVertex3d(point1[0],
					point1[1],
					point1[2]);

				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[1]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[1]].x, visual->normal[visual->index[loop].v[1]].y, visual->normal[visual->index[loop].v[1]].z);
				}
				else
				{
					glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
				}
				//glColor3d(1.0f, 1.0f, 1.0f);
				glVertex3d(point2[0],
					point2[1],
					point2[2]);

				/*u = CalculateShadow(&visual->normal[visual->index[loop].n[2]], &m_ShadeLight, &mat);
				(*glMultiTexCoord1fARB)(GL_TEXTURE1_ARB, u);*/
				if (visual->normalCnt == visual->vertexCnt)
				{
					glNormal3f(visual->normal[visual->index[loop].v[2]].x, visual->normal[visual->index[loop].v[2]].y, visual->normal[visual->index[loop].v[2]].z);
				}
				else
				{
					glNormal3f(vnormal[0], vnormal[1], vnormal[2]);
				}
				//glColor3d(1.0f, 1.0f, 1.0f);
				glVertex3d(point3[0],
					point3[1],
					point3[2]);

			}

		}
		glDisable(GL_TEXTURE_2D);
		glEnd();
		glPopMatrix();
		/////////////////////////////////////////add 2014.10.16
		if (Minpoint)
		{
			glColor3f(1, 0, 0);
			glPointSize(8.0);	//set the point size to 4 by 4 pixels
			glEnable(GL_POINT_SMOOTH);//没这句话点就不圆滑
									  //画点
			glBegin(GL_POINTS);
			for (loop = 0; loop < visual->faceCnt; loop++)
			{
				if (visual->vertex[visual->index[loop].v[0]].y == XminY ||
					visual->vertex[visual->index[loop].v[0]].x == Xmin0)
				{
					glVertex3f(visual->vertex[visual->index[loop].v[0]].x,
						visual->vertex[visual->index[loop].v[0]].y,
						visual->vertex[visual->index[loop].v[0]].z);
				}
				if (visual->vertex[visual->index[loop].v[1]].y == XminY ||
					visual->vertex[visual->index[loop].v[1]].x == Xmin0)
				{
					glVertex3f(visual->vertex[visual->index[loop].v[1]].x,
						visual->vertex[visual->index[loop].v[1]].y,
						visual->vertex[visual->index[loop].v[1]].z);
				}
				if (visual->vertex[visual->index[loop].v[2]].y == XminY ||
					visual->vertex[visual->index[loop].v[2]].x == Xmin0)
				{
					glVertex3f(visual->vertex[visual->index[loop].v[2]].x,
						visual->vertex[visual->index[loop].v[2]].y,
						visual->vertex[visual->index[loop].v[2]].z);
				}

			}
			glEnd();
			glFlush();
			glColor3f(1.0f, 1.0f, 1.0f);


		}

		////////////////////////////////////////end 2014.10.16	
		//////////////////////20150822/////libin//////////////////////////////////

		//glColor3f(1, 0, 0);
		//glPointSize(10.0);	//set the point size to 4 by 4 pixels
		//glEnable(GL_POINT_SMOOTH);//没这句话点就不圆滑

		////画点
		//glBegin(GL_POINTS);	
		//glVertex3f(last_x1,last_y1,last_z1);
		//glVertex3f(last_x2,last_y2,last_z2);
		//glVertex3f(last_x3,last_y3,last_z3);
		//glVertex3f(last_x4,last_y4,last_z4);
		//glVertex3f(last_x5,last_y5,last_z5);
		//
		//glEnd();

		//glFlush();
		//glColor3f(1.0f, 1.0f, 1.0f);

		//////////////////////////////////////////////////////////////////////////


	}
}


/************************************************************************/
/* 画匹配点                                                                     */
/************************************************************************/
GLvoid COGLView::drawMechPoints(t_Visual *model1, t_Visual *model2)
{
	long mechsize = 0;
	if (model1->vertex != NULL&&model1->mechindexs != NULL)
	{
		mechsize = model1->mechSize;
		glPushMatrix();
		glColor3f(1, 0, 0);
		glPointSize(8.0);	//set the point size to 4 by 4 pixels

		glEnable(GL_POINT_SMOOTH);//没这句话点就不圆滑
								  //画点
		glBegin(GL_LINE);
		for (int loop = 0; loop < mechsize; loop++)
		{
			if (model1->mechindexs[loop].statu == 0)
			{
				glColor3f(0, 0, 1);
			}
			else glColor3f(1, 0, 0);
			glVertex3f(model1->vertex[model1->mechindexs[loop].targetIndex].x,
				model1->vertex[model1->mechindexs[loop].targetIndex].y,
				model1->vertex[model1->mechindexs[loop].targetIndex].z);
			glVertex3f(model2->vertex[model1->mechindexs[loop].partIndex].x,
				model2->vertex[model1->mechindexs[loop].partIndex].y,
				model2->vertex[model1->mechindexs[loop].partIndex].z);


		}
		//if(NumofObj<=1)
		glEnd();
		glPopMatrix();
		//glFlush();
		//glColor3f(1.0f, 1.0f, 1.0f);
	}

}
///////////////////////////////////////////////////////////////////////////////
// Function:	SetCamera
// Purpose:		set camera parameter
// Arguments:	None
///////////////////////////////////////////////////////////////////////////////
GLvoid COGLView::SetCamera()
{
	/*if (0 < NumofObj)
	{
	m_Camera.trans.x = (m_Model.vertexMax_x + m_Model.vertexMin_x) / 2.0;
	m_Camera.trans.y = (m_Model.vertexMax_y + m_Model.vertexMin_y) / 2.0;
	if ((m_Model.vertexMax_x - m_Model.vertexMin_x)>(m_Model.vertexMax_y - m_Model.vertexMin_y))
	{
	m_Camera.trans.z = (m_Model.vertexMax_z + m_Model.vertexMin_z) / 2.0 - 2 * (m_Model.vertexMax_x - m_Model.vertexMin_x);

	}else
	m_Camera.trans.z = (m_Model.vertexMax_z + m_Model.vertexMin_z) / 2.0 - 2 * (m_Model.vertexMax_y - m_Model.vertexMin_y);


	}*/
	if (0<NumofObj)
	{
		m_Camera.trans.x = -(m_Model.vertexMax_x + m_Model.vertexMin_x) / 2.0;
		m_Camera.trans.y = -(m_Model.vertexMax_y + m_Model.vertexMin_y) / 2.0;
		m_Camera.trans.z = (m_Model.vertexMax_z + m_Model.vertexMin_z) / 2.0;
		double maxtrans = max(m_Model.vertexMax_x - m_Model.vertexMin_x, m_Model.vertexMax_y - m_Model.vertexMin_y);
		maxtrans = max(maxtrans, m_Model.vertexMax_z - m_Model.vertexMin_z);

		m_Camera.trans.z -= 3 * maxtrans;
	}
}
//////SetCamera////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Function:	drawScene
// Purpose:		Actually draw the OpenGL Scene
// Arguments:	None
///////////////////////////////////////////////////////////////////////////////
GLvoid COGLView::drawScene()
{
	/// Local Variables ///////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////

	if (m_Camera.rot.y  > 360.0f) m_Camera.rot.y -= 1.00f;
	if (m_Camera.rot.x   > 360.0f) m_Camera.rot.x -= 1.00f;
	if (m_Camera.rot.z > 360.0f) m_Camera.rot.z -= 1.00f;

	glDisable(GL_DEPTH_TEST);	// TURN OFF DEPTH TEST FOR CLEAR

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_DEPTH_TEST);	// ENABLE DEPTH TESTING
	//glDisable(GL_DEPTH_TEST);	// TURN OFF DEPTH TEST FOR CLEAR


	glPushMatrix();

	// Set camera's orientation and position
	//	glTranslatef(m_Camera.trans.x, m_Camera.trans.y-200, m_Camera.trans.z-1500);

	glTranslatef(m_Camera.trans.x, m_Camera.trans.y, m_Camera.trans.z - 2);
	//glScalef(2000.0, 2000.0, 2000.0);
	glRotatef(m_Camera.rot.y, 0.0f, 1.0f, 0.0f);
	glRotatef(m_Camera.rot.x, 1.0f, 0.0f, 0.0f);
	glRotatef(m_Camera.rot.z, 0.0f, 0.0f, 1.0f);


	//flag标识显示状态，0：点；1：mesh；2：纹理；3：……
	//

	if (view_model.openGL_show_model==11|| view_model.openGL_show_model == 10)
		drawModel(&m_Model);
	

	glPopMatrix();
	glFlush();

	if (NumofObj <= 1)
		SwapBuffers(m_hDC);
	else {
		if (view_model.openGL_show_model == 11 || view_model.openGL_show_model == 1)
		{
			drawGScene();
		}
			SwapBuffers(m_hDC);
		
	}

}
//// drawScene //////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function:	drawScene
// Purpose:		Actually draw the OpenGL Scene
// Arguments:	None
///////////////////////////////////////////////////////////////////////////////
GLvoid COGLView::drawGScene()
{
	/// Local Variables ///////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////

	if (1 >= NumofObj) return;
	if (m_Camera.rot.y > 360.0f) m_Camera.rot.y -= 1.00f;
	if (m_Camera.rot.x > 360.0f) m_Camera.rot.x -= 1.00f;
	if (m_Camera.rot.z > 360.0f) m_Camera.rot.z -= 1.00f;

	glPushMatrix();
	if (Rt[15] == 1)
	{
		//glMatrixMode(GL_MODELVIEW);

		glMultMatrixf(Rt);

	}
	// Set camera's orientation and position
	//	glTranslatef(m_Camera.trans.x, m_Camera.trans.y-200, m_Camera.trans.z-1500);
	glTranslatef(m_Camera.trans.x, m_Camera.trans.y, m_Camera.trans.z - 2);

	glRotatef(m_Camera.rot.y, 0.0f, 1.0f, 0.0f);
	glRotatef(m_Camera.rot.x, 1.0f, 0.0f, 0.0f);
	glRotatef(m_Camera.rot.z, 0.0f, 0.0f, 1.0f);

	// Draw any loaded model
	//flag标识显示状态，0：点；1：mesh；2：纹理；3：……
	//

	drawGModel(&m_GModel);


	glPopMatrix();

	//    glFinish();
	//drawScene();
	//SwapBuffers(m_hDC);


}
//// drawScene //////////////////////////////////////////////////////
void COGLView::OnDestroy()
{
	CWnd::OnDestroy();
	if (m_hRC)
		wglDeleteContext(m_hRC);
	if (m_hDC)
		::ReleaseDC(m_hWnd, m_hDC);
	m_hRC = 0;
	m_hDC = 0;


}

void COGLView::OnPaint()
{
	CPaintDC dc(this); // device context for painting

	drawScene();
	// Do not call CWnd::OnPaint() for painting messages
}

void COGLView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// STORE OFF THE HIT POINT AND SETTINGS FOR THE MOVEMENT LATER
	m_mousepos = point;
	m_Dragging = TRUE;
	m_Grab_Rot_X = m_Camera.rot.x;
	m_Grab_Rot_Y = m_Camera.rot.y;
	m_Grab_Rot_Z = m_Camera.rot.z;
	m_Grab_Trans_X = m_Camera.trans.x;
	m_Grab_Trans_Y = m_Camera.trans.y;
	m_Grab_Trans_Z = m_Camera.trans.z;
	SetFocus();
	CWnd::OnLButtonDown(nFlags, point);
}

void COGLView::OnRButtonDown(UINT nFlags, CPoint point)
{
	// STORE OFF THE HIT POINT AND SETTINGS FOR THE MOVEMENT LATER
	m_mousepos = point;
	m_Dragging = TRUE;
	m_Grab_Rot_X = m_Camera.rot.x;
	m_Grab_Rot_Y = m_Camera.rot.y;
	m_Grab_Rot_Z = m_Camera.rot.z;
	m_Grab_Trans_X = m_Camera.trans.x;
	m_Grab_Trans_Y = m_Camera.trans.y;
	m_Grab_Trans_Z = m_Camera.trans.z;
	SetFocus();
	CWnd::OnRButtonDown(nFlags, point);
}


void COGLView::OnLButtonUp(UINT nFlags, CPoint point)
{
	m_Dragging = FALSE;


	CWnd::OnLButtonUp(nFlags, point);
}

void COGLView::OnRButtonUp(UINT nFlags, CPoint point)
{
	m_Dragging = FALSE;

	CWnd::OnRButtonUp(nFlags, point);
}
BOOL COGLView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{

	// TODO: Add your message handler code here and/or call default

	//AfxMessageBox("鼠标滚轮滑动触发");

	//m_Camera.trans.z = m_Grab_Trans_Z + (.001f *zDelta / 120);
	if (zDelta<=0)
	{
		zDelta -= 1;
	}
	m_Camera.trans.z = m_Grab_Trans_Z + (m_Camera.trans.z / (zDelta+1) * 10);
	m_Grab_Trans_Z = m_Camera.trans.z;
	drawScene();
	//SetFocus();
	return  true;// CWnd::OnMouseWheel(nFlags, zDelta, pt);

}
///////////////////////////////////////////////////////////////////////////////
// Function:	OnMouseMove
// Purpose:		Handler for the mouse.  Handles movement when pressed
// Arguments:	Flags for key masks and point
///////////////////////////////////////////////////////////////////////////////
void COGLView::OnMouseMove(UINT nFlags, CPoint point)
{

	if (!m_Dragging) return;

	//	UpdateStatusBar(0);
	if ((nFlags & MK_LBUTTON) > 0)
	{
		if ((nFlags & MK_CONTROL) > 0)
		{
		}
		// ELSE "SHIFT" MOVE THE BONE IN XY
		else if ((nFlags & MK_SHIFT) > 0)
		{
			UpdateStatusBar(1);
			if ((point.x - m_mousepos.x) != 0)	// Rotate Camera in Z
			{
				m_Camera.rot.z = m_Grab_Rot_Z + ((float)ROTATE_SPEED*0.1  * (point.x - m_mousepos.x));
				drawScene();
				//drawGScene();
			}
		}
		else
		{
			UpdateStatusBar(1);
			if ((point.x - m_mousepos.x) != 0)	// Rotate Camera in Y
			{
				m_Camera.rot.y = m_Grab_Rot_Y + ((float)ROTATE_SPEED*0.1 * (point.x - m_mousepos.x));
				drawScene();
				//drawGScene();
			}
			if ((point.y - m_mousepos.y) != 0)	// Rotate Camera in X
			{
				m_Camera.rot.x = m_Grab_Rot_X + ((float)ROTATE_SPEED *0.1 * (point.y - m_mousepos.y));
				drawScene();
				//drawGScene();
			}
		}
	}
	else if ((nFlags & MK_RBUTTON) == MK_RBUTTON)
	{
		if ((nFlags & MK_CONTROL) > 0)
		{
		}
		else if ((nFlags & MK_SHIFT) > 0)
		{
			UpdateStatusBar(2);
			if ((point.x - m_mousepos.x) != 0)	// Move Camera in X
			{
				m_Camera.trans.x = m_Grab_Trans_X + (.1f * (point.x - m_mousepos.x));
				drawScene();
				//drawGScene();
			}
			if ((point.y - m_mousepos.y) != 0)	// Move Camera in Y
			{
				m_Camera.trans.y = m_Grab_Trans_Y - (.1f * (point.y - m_mousepos.y));
				drawScene();
				//drawGScene();
			}
		}
		// IF I AM HOLDING THE RM BUTTON Translate IN Z
		else
		{
			UpdateStatusBar(2);
			if ((point.x - m_mousepos.x) != 0)	// Move Camera in X
			{
				m_Camera.trans.x = m_Grab_Trans_X + (.01f * (point.x - m_mousepos.x));
				drawScene();
				//drawGScene();
			}
			if ((point.y - m_mousepos.y) != 0)
			{
				m_Camera.trans.z = m_Grab_Trans_Z + ((point.y - m_mousepos.y)*fabs(m_Camera.trans.z / m_ScreenHeight));

				drawScene();
				//drawGScene();
			}
		}
	}

	CWnd::OnMouseMove(nFlags, point);
}
//// OnMouseMove //////////////////////////////////////////////////////

void COGLView::OnMove(int x, int y)
{
	CWnd::OnMove(x, y);

	resize(x, y);

}

// 0 = READY
// 1 = ROTATE
// 2 = TRANSLATE
void COGLView::UpdateStatusBar(int mode)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	char message[80];
	///////////////////////////////////////////////////////////////////////////////
	if (mode == 1)
	{
		sprintf(message, "Rotate (%.2f,%.2f,%.2f)", m_Camera.rot.x, m_Camera.rot.y, m_Camera.rot.z);
	}
	else if (mode == 2)
	{
		sprintf(message, "Translate (%.2f,%.2f,%.2f)", m_Camera.trans.x, m_Camera.trans.y, m_Camera.trans.z);
	}
	else
	{
		strcpy(message, "Ready");
	}
	m_StatusBar->SetPaneText(0, message);
}
void COGLView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	HandleKeyDown(nChar);
	CWnd::OnKeyDown(nChar, nRepCnt, nFlags);
}

void COGLView::OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	CWnd::OnKeyUp(nChar, nRepCnt, nFlags);
	HandleKeyUp(nChar);
}

void COGLView::HandleKeyDown(UINT nChar)
{
}

void COGLView::HandleKeyUp(UINT nChar)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////
	switch (nChar)
	{
	case VK_SPACE:
		break;
	case 'I':
		break;
	case 'W':
		glPolygonMode(GL_FRONT, GL_LINE);
		break;
	case 'F':
		glPolygonMode(GL_FRONT, GL_FILL);
		break;
	case 'A':
		m_Camera.trans.z += 0.01;
		Invalidate(FALSE);
		break;
	case 'S':
		m_Camera.trans.z -= 0.01;
		Invalidate(FALSE);
		break;
	case VK_LEFT:
		m_Camera.trans.x -= 10;
		drawScene();
		break;
	case VK_RIGHT:
		m_Camera.trans.x += 10;
		drawScene();
		break;
	case VK_DOWN:
		m_Camera.trans.y -= 10;
		drawScene();
		break;
	case VK_UP:
		m_Camera.trans.y += 10;
		drawScene();
		break;


	}

	Invalidate(TRUE);

}

///////////////////////////////////////////////////////////////////////////////
// Function:	GetGLInfo
// Purpose:		Get the OpenGL Vendor and Renderer
///////////////////////////////////////////////////////////////////////////////
void COGLView::GetGLInfo()
{
	//// Local Variables ////////////////////////////////////////////////////////////////
	char *who, *which, *ver, *ext, *message;
	int len;
	/////////////////////////////////////////////////////////////////////////////////////
	who = (char *)::glGetString(GL_VENDOR);
	which = (char *)::glGetString(GL_RENDERER);
	ver = (char *)::glGetString(GL_VERSION);
	ext = (char *)::glGetString(GL_EXTENSIONS);

	len = 200 + strlen(who) + strlen(which) + strlen(ver) + strlen(ext);

	message = (char *)malloc(len);
	sprintf(message, "Who:\t%s\nWhich:\t%s\nVersion:\t%s\nExtensions:\t%s",
		who, which, ver, ext);

	::MessageBox(NULL, message, "GL Info", MB_OK);

	free(message);
}
//// GetGLInfo /////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function:	LoadOBJModel
// Purpose:		Load an OBJ Model into the system
// Arguments:	Name of the file to open
///////////////////////////////////////////////////////////////////////////////
BOOL COGLView::LoadOBJModel(CString name)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	char texname[255], *pos;
	tTGAHeader_s header;
	unsigned char *texture;

	///////////////////////////////////////////////////////////////////////////////

	if (m_Model.vertex != NULL)	// 释放数据
	{
		free(m_Model.vertex);
		m_Model.vertex = NULL;
		m_Model.vertexCnt = 0;	// Set the vertex count设置顶点计数
		m_Model.faceCnt = 0;	// Set the vertex count
		m_Model.uvCnt = 0;	// Set the vertex count
		m_Model.normalCnt = 0;	// Set the normal count
	}
	if (m_Model.GuassCurvature != NULL)	// 释放数据
	{
		free(m_Model.GuassCurvature);
		m_Model.GuassCurvature = NULL;
	}
	if (m_Model.HCurvature != NULL)	// 释放数据
	{
		free(m_Model.HCurvature);
		m_Model.HCurvature = NULL;
	}
	if (m_Model.Gvertex != NULL)	// 释放数据
	{
		free(m_Model.Gvertex);
		m_Model.Gvertex = NULL;
	}
	if (m_Model.deformData != NULL)	// 释放数据
	{
		free(m_Model.deformData);
		m_Model.deformData = NULL;
	}
	if (m_Model.normal != NULL)	// 释放数据
	{
		free(m_Model.normal);
		m_Model.normal = NULL;
	}
	if (m_Model.texture != NULL)	// 释放数据
	{
		free(m_Model.texture);
		m_Model.texture = NULL;
	}
	if (m_Model.vertexColor != NULL)	// 释放数据
	{
		free(m_Model.vertexColor);
		m_Model.vertexColor = NULL;
	}
	if (m_Model.index != NULL)	//释放数据
	{
		free(m_Model.index);
		m_Model.index = NULL;
	}
	if (m_Model.vfindex != NULL)	//释放数据
	{
		free(m_Model.vfindex);
		m_Model.vfindex = NULL;
	}
	memset(m_Model.map, 0, 255);
	memset(m_Model.map2, 0, 255);
	BOOL Reb = LoadOBJ((LPCSTR)name, &m_Model);//加载模型数据

	if (strlen(m_Model.map) > 0)
	{
		pos = m_Model.map + strlen(m_Model.map);
		while (*pos != '/' && pos != m_Model.map)
			pos--;

		if (*pos == '/') pos++;

		sprintf(texname, "%s", pos);

		texture = LoadTGAFile(texname, &header);//载入纹理信息

		m_Model.m_pRGB = texture;

		//texture = LoadTGAFile( "BOOK_0",	&header);  



		// GENERATE THE OPENGL TEXTURE ID  生成纹理ID
		//glGenTextures(1, &m_Model.glTex);

		//glBindTexture(GL_TEXTURE_2D, m_Model.glTex);//开始
		//BuildTexture("a_0.jpg",texturegyy5);
		//BuildTexture(m_Model.map, textureyx);//20140915
		maped_texture.BuildTexture(m_Model.map, textureyx, m_hWnd);

											 //getTextureWH(m_Model.map, texW, texH);
											 //t_Visual *ptemp = &m_Model;
		thread mapTexture_m(MapTexture, &m_Model, maped_texture);
		mapTexture_m.detach();
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

		/*
		* Define the 2D texture image.
		*/

		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);	/* Force 4-byte alignment */
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
		glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);

		if (header.d_pixel_size == 32)
		{
			//glTexImage2D(GL_TEXTURE_2D, 0, 4, header.d_width, header.d_height, 0,GL_RGBA, GL_UNSIGNED_BYTE, texture);

		}
		else
		{
			//changed by tonglijing back
			//glTexImage2D(GL_TEXTURE_2D, 0, 3, header.d_width, header.d_height, 0, GL_RGB, GL_UNSIGNED_BYTE, texture);
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, header.d_width, header.d_height, 0,GL_RGB , GL_UNSIGNED_BYTE, texture);

		}//结束
		free(texture);//remove gyy 2014.07.01 20:08
	}
	else
		m_Model.glTex = 0;

	// Load 2nd Texture if it is there
	if (strlen(m_Model.map2) > 0)
	{
		pos = m_Model.map2 + strlen(m_Model.map2);
		while (*pos != '/' && pos != m_Model.map2)
			pos--;

		if (*pos == '/') pos++;

		sprintf(texname, "%s", pos);

		texture = LoadTGAFile(texname, &header);  //

												  // GENERATE THE OPENGL TEXTURE ID
		glGenTextures(1, &m_Model.glTex2);

		glBindTexture(GL_TEXTURE_2D, m_Model.glTex2);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

		/*
		* Define the 2D texture image.
		*/

		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);	/* Force 4-byte alignment */
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
		glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);

		if (header.d_pixel_size == 32)
		{
			glTexImage2D(GL_TEXTURE_2D, 0, 4, header.d_width, header.d_height, 0,
				GL_RGBA, GL_UNSIGNED_BYTE, texture);

		}
		else
		{
			glTexImage2D(GL_TEXTURE_2D, 0, 3, header.d_width, header.d_height, 0,
				GL_RGB, GL_UNSIGNED_BYTE, texture);

		}

		free(texture);
	}
	//  	else
	// 		m_Model.glTex = 0;///remove gyy 2014.09.17
	NumofObj = 1;
	return TRUE;
}
///////////////////////////////////////////////////////////////////////////////
// Function:	LoadOBJModel
// Purpose:		Load an OBJ Model into the system
// Arguments:	Name of the file to open
///////////////////////////////////////////////////////////////////////////////
BOOL COGLView::LoadOBJGModel(CString name)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	char texname[255], *pos;
	tTGAHeader_s header;
	unsigned char *texture;

	///////////////////////////////////////////////////////////////////////////////


	if (m_GModel.vertex != NULL)	// 释放数据
	{
		free(m_GModel.vertex);
		m_GModel.vertex = NULL;
		m_GModel.vertexCnt = 0;	// Set the vertex count
		m_GModel.faceCnt = 0;	// Set the vertex count
		m_GModel.uvCnt = 0;	// Set the vertex count
		m_GModel.normalCnt = 0;	// Set the normal count
	}
	if (m_GModel.GuassCurvature != NULL)	// 释放数据
	{
		free(m_GModel.GuassCurvature);
		m_GModel.GuassCurvature = NULL;
	}
	if (m_GModel.HCurvature != NULL)	// 释放数据
	{
		free(m_GModel.HCurvature);
		m_GModel.HCurvature = NULL;
	}
	if (m_GModel.Gvertex != NULL)	// 释放数据
	{
		free(m_GModel.Gvertex);
		m_GModel.Gvertex = NULL;
	}
	if (m_GModel.deformData != NULL)	// 释放数据
	{
		free(m_GModel.deformData);
		m_GModel.deformData = NULL;
	}
	if (m_GModel.normal != NULL)	// 释放数据
	{
		free(m_GModel.normal);
		m_GModel.normal = NULL;
	}
	if (m_GModel.texture != NULL)	// 释放数据
	{
		free(m_GModel.texture);
		m_GModel.texture = NULL;
	}
	if (m_GModel.vertexColor != NULL)	// 释放数据
	{
		free(m_GModel.vertexColor);
		m_GModel.vertexColor = NULL;
	}
	if (m_GModel.index != NULL)	//释放数据
	{
		free(m_GModel.index);
		m_GModel.index = NULL;
	}
	if (m_GModel.vfindex != NULL)	//释放数据
	{
		free(m_GModel.vfindex);
		m_GModel.vfindex = NULL;
	}
	memset(m_GModel.map, 0, 255);
	memset(m_GModel.map2, 0, 255);
	BOOL Reb = LoadOBJ((LPCSTR)name, &m_GModel);

	if (strlen(m_GModel.map) > 0)
	{
		pos = m_GModel.map + strlen(m_GModel.map);
		while (*pos != '/' && pos != m_GModel.map)
			pos--;

		if (*pos == '/') pos++;

		sprintf(texname, "%s", pos);

		texture = LoadTGAFile(texname, &header);//载入纹理信息gyy 2014.06.30

		m_GModel.m_pRGB = texture;//add gyy 2014.07.01.19:00

								  //texture = LoadTGAFile( "BOOK_0",	&header);  



								  // GENERATE THE OPENGL TEXTURE ID  生成纹理ID
		glGenTextures(1, &m_GModel.glTex);

		//glBindTexture(GL_TEXTURE_2D, m_GModel.glTex);//开始
													 //BuildTexture("a_0.jpg",texturegyy5);
		//BuildTexture(m_GModel.map, textureyx1);//20140915
		maped_texture.BuildTexture(m_GModel.map, textureyx1, m_hWnd);

		//getTextureWH(m_Model.map, texW, texH);
		//t_Visual *ptemp = &m_Model;
		thread mapTexture_g(MapTexture, &m_GModel, maped_texture);//yingxiang此处可能出错#include <mutex> mutex mu ； mu.lock();mu,unlock()未加线程互斥
		mapTexture_g.detach();

		//BuildTexture("BOOK_0.jpg",texturegyy);

		//BuildTexture(m_GModel.map, textureyx2);
		//BuildTexture("BOOK_4.jpg",texturegyy4);



		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

		/*
		* Define the 2D texture image.
		*/

		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);	/* Force 4-byte alignment */
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
		glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);

		if (header.d_pixel_size == 32)
		{
			//glTexImage2D(GL_TEXTURE_2D, 0, 4, header.d_width, header.d_height, 0,GL_RGBA, GL_UNSIGNED_BYTE, texture);

		}
		else
		{
			//changed by tonglijing back
			//glTexImage2D(GL_TEXTURE_2D, 0, 3, header.d_width, header.d_height, 0, GL_RGB, GL_UNSIGNED_BYTE, texture);
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, header.d_width, header.d_height, 0,GL_RGB , GL_UNSIGNED_BYTE, texture);

		}//结束
		free(texture);//remove gyy 2014.07.01 20:08
	}
	else
		m_GModel.glTex = 0;

	// Load 2nd Texture if it is there
	if (strlen(m_GModel.map2) > 0)
	{
		pos = m_GModel.map2 + strlen(m_GModel.map2);
		while (*pos != '/' && pos != m_GModel.map2)
			pos--;

		if (*pos == '/') pos++;

		sprintf(texname, "%s", pos);

		texture = LoadTGAFile(texname, &header);  //

												  // GENERATE THE OPENGL TEXTURE ID
		glGenTextures(1, &m_GModel.glTex2);

		glBindTexture(GL_TEXTURE_2D, m_GModel.glTex2);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

		/*
		* Define the 2D texture image.
		*/

		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);	/* Force 4-byte alignment */
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
		glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);

		if (header.d_pixel_size == 32)
		{
			glTexImage2D(GL_TEXTURE_2D, 0, 4, header.d_width, header.d_height, 0,
				GL_RGBA, GL_UNSIGNED_BYTE, texture);

		}
		else
		{
			glTexImage2D(GL_TEXTURE_2D, 0, 3, header.d_width, header.d_height, 0,
				GL_RGB, GL_UNSIGNED_BYTE, texture);

		}

		free(texture);
	}
	//  	else
	// 		m_GModel.glTex = 0;///remove gyy 2014.09.17
	NumofObj++;
	return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
// Function:	LoadShadeTexture
// Purpose:		Load a shaded environment texture
// Arguments:	Name of the file to open
///////////////////////////////////////////////////////////////////////////////

void COGLView::LoadShadeTexture(const char *texfile)
{
	//// Local Variables ////////////////////////////////////////////////////////////////
	int loop;
	FILE *fp;
	char line[255];
	float value1, value2;
	/////////////////////////////////////////////////////////////////////////////////////

	// Make a Default one One shade with highlight
	for (loop = 0; loop < 32; loop++)
	{

		if (loop < 8)
		{
			MAKEVECTOR4(m_ShadeSrc[loop], 0.4f, 0.4f, 0.4f, 0.0f)
		}
		else if (loop < 28)
		{
			MAKEVECTOR4(m_ShadeSrc[loop], 0.9f, 0.9f, 0.9f, 1.0f)
		}
		else
		{
			MAKEVECTOR4(m_ShadeSrc[loop], 1.0f, 1.0f, 1.0f, 1.0f)
		}
	}

	// Totally simple file format to load a 1D shade table
	// just a list of floats in a text file
	fp = fopen(texfile, "r");
	if (fp)
	{
		for (loop = 0; loop < 32; loop++)
		{
			if (feof(fp))
				break;
			// Get a line from the file
			fgets(line, 255, fp);
			// Convert it to a shade value
			sscanf(line, "%f %f", &value1, &value2);
			m_ShadeSrc[loop].x = m_ShadeSrc[loop].y = m_ShadeSrc[loop].z = value1;
			m_ShadeSrc[loop].w = value2;
		}
		fclose(fp);
	}
	glBindTexture(GL_TEXTURE_1D, m_ShadeTexture);

	// Do not allow bilinear filtering - not for cartoon rendering
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, 32, 0,
		GL_RGBA, GL_FLOAT, (float *)m_ShadeSrc); //visual->texData);
}

///////////////////////////////////////////////////////////////////////////////
// Function:	CartoonSettings
// Purpose:		Adjust Line Settings for Cartoon Render
// Arguments:	Name of the file to open
///////////////////////////////////////////////////////////////////////////////
void COGLView::CartoonSettings()
{
	//// Local Variables ////////////////////////////////////////////////////////////////
	CToonSet	dialog;
	/////////////////////////////////////////////////////////////////////////////////////
	dialog.m_Sil_Red = m_SilhouetteColor.r;
	dialog.m_Sil_Green = m_SilhouetteColor.g;
	dialog.m_Sil_Blue = m_SilhouetteColor.b;
	dialog.m_LineWidth = m_SilhouetteWidth;
	dialog.m_Light_X = m_ShadeLight.x;
	dialog.m_Light_Y = m_ShadeLight.y;
	dialog.m_Light_Z = m_ShadeLight.z;
	if (dialog.DoModal())
	{
		m_SilhouetteColor.r = dialog.m_Sil_Red;
		m_SilhouetteColor.g = dialog.m_Sil_Green;
		m_SilhouetteColor.b = dialog.m_Sil_Blue;
		m_SilhouetteWidth = dialog.m_LineWidth;
		m_ShadeLight.x = dialog.m_Light_X;
		m_ShadeLight.y = dialog.m_Light_Y;
		m_ShadeLight.z = dialog.m_Light_Z;
		NormalizeVector(&m_ShadeLight);	// Normalize it since I know I didn't
	}
	Invalidate(TRUE);
}


BOOL COGLView::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 在此添加专用代码和/或调用基类
#if 0
	if (pMsg->message == WM_KEYDOWN)
	{
		switch (pMsg->wParam)
		{
		case VK_LEFT:
			//
			break;
		case VK_RIGHT:
			//  
			break;
		case VK_UP:
			m_Camera.trans.z += 100;
			Invalidate(FALSE);
			break;
		case VK_DOWN:
			m_Camera.trans.z -= 100;
			Invalidate(FALSE);
			break;
		default:
			break;;

		}
	}
#endif
	return CWnd::PreTranslateMessage(pMsg);
}




void COGLView::OnTimer(UINT_PTR nIDEvent)
{


	//KillTimer(1);
	if (view_model.openGL_view_model != view_flag 
		||view_model.openGL_show_model!= show_flag
		||view_model.showCurvature!=show_curvature)
	{
		view_flag = view_model.openGL_view_model;
		show_flag = view_model.openGL_show_model;
		show_curvature = view_model.showCurvature;
		if (m_Model.GuassCurvature!=NULL&&show_curvature)
		{
			m_Model.TextureStatu = 30;
		}
		else if(m_Model.texture!=NULL)
		{
			m_Model.TextureStatu = 2;
		}
		else if(m_Model.vertexColor!=NULL) {
			m_Model.TextureStatu = 1;
		}
		else {
			m_Model.TextureStatu = 0;
		}
		if (m_GModel.GuassCurvature != NULL&&show_curvature)
		{
			m_GModel.TextureStatu = 30;
		}
		else if (m_GModel.texture != NULL)
		{
			m_GModel.TextureStatu = 2;
		}
		else if (m_GModel.vertexColor != NULL)
		{
			m_GModel.TextureStatu = 1;
		}
		else {
			m_GModel.TextureStatu = 0;
		}
		switch (view_flag)
		{
		case 0:
			glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);           // 设置为点方式
			break;
		case 1:

			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);            // 设置为边缘绘制方式
			break;
		case 2:

			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // 设置两面均为填充方式 
			break;
		default:
			break;
		}


		drawScene();
	}else if (view_model.clearModel==true)
	{
		OnBnClickedButtonClearModel();
		view_model.clearModel = false;
	}




}


void  MapTexture(t_Visual *pm_Model, maptex maped_texture)  //打开OBJ模型函数为GLvoid COGLView::GyydrawModel(t_Visual *visual)
{
	t_Visual m_Model = *pm_Model;
	/*maptex maped_texture;
	maped_texture.JPGWidth = lWidthPixels;
	maped_texture.JPGHeight = lHeightPixels;
	maped_texture.nBytePerLine = (maped_texture.JPGWidth * 32 / 8 + 3) / 4 * 4;*/
	//t_Visual m_Model = visual;
	/////////////////////////计算OBJ模型空间 长度和宽度作为BMP位图大小依据////////////////////////
	double x_min = m_Model.vertex[0].x;  //x y 坐标的最大最小值初始化
	double y_min = m_Model.vertex[0].y;
	double x_max = m_Model.vertex[0].x;
	double y_max = m_Model.vertex[0].y;
	double OBJWidth, OBJHeight;
	int x_maxpoint, x_minpoint, y_maxpoint, y_minpoint;  //x 最大值 最小值 y 最大值 最小值
	for (int j = 0; j < m_Model.vertexCnt; j++)  // 遍历模型顶点
	{

		if (x_min >= m_Model.vertex[j].x)   // 找出x坐标最小值 和 最小值点j
		{
			x_min = m_Model.vertex[j].x;
			x_minpoint = j;
		}

		if (x_max <= m_Model.vertex[j].x)
		{
			x_max = m_Model.vertex[j].x;
			x_maxpoint = j;
		}

		if (y_min >= m_Model.vertex[j].y)
		{
			y_min = m_Model.vertex[j].y;
			y_minpoint = j;
		}

		if (y_max <= m_Model.vertex[j].y)
		{
			y_max = m_Model.vertex[j].y;
			y_maxpoint = j;
		}
	}
	OBJWidth = m_Model.vertex[x_maxpoint].x - m_Model.vertex[x_minpoint].x;
	OBJHeight = m_Model.vertex[y_maxpoint].y - m_Model.vertex[y_minpoint].y;

	/////////////////////////////////////计算结束//////////////////////////////////

	///////////////////////////////计算JPG转成的临时位图数据///////////////////////

	int NewPictureLarge = maped_texture.JPGHeight; //防止内存越界  纹理图大小 （正方形高=宽）
	int OldDataSize = maped_texture.nBytePerLine * maped_texture.JPGHeight;  //纹理图高

																			 ////////////////////////////////////写入BMP文件////////////////////////////////
//#define ENLARGE 20                   //定义从0bj到bmp扩大的倍数
#define PICTURELARGE  maped_texture.JPGHeight //bmp图片大小
	char *pFileName = "obj.bmp";
	//CFile file(pFileName, CFile::modeCreate | CFile::modeWrite);
	try
	{
		//写入文件头、
		maped_texture.m_BmpFileHeader.bfType = 0x4d42;
		maped_texture.m_BmpFileHeader.bfSize = sizeof(BITMAPFILEHEADER);
		maped_texture.m_BmpFileHeader.bfReserved1 = 0;
		maped_texture.m_BmpFileHeader.bfReserved2 = 0;
		maped_texture.m_BmpFileHeader.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);

		//file.Write(&maped_texture.m_BmpFileHeader, sizeof(BITMAPFILEHEADER));

		//写入信息头
		maped_texture.m_BmpInfo.bmiHeader.biBitCount = 24;
		maped_texture.m_BmpInfo.bmiHeader.biClrImportant = 0;
		maped_texture.m_BmpInfo.bmiHeader.biClrUsed = 0;
		maped_texture.m_BmpInfo.bmiHeader.biCompression = BI_RGB; //未经压缩格式
		//规定投影图像的大小H=2046;
		double ENLARGE =  2046.0 / OBJHeight;
		if (OBJHeight*20<2046)
		{
			ENLARGE = 20;
		}
		maped_texture.m_BmpInfo.bmiHeader.biHeight = OBJHeight*ENLARGE; //OBJHeight*ENLARGE;   //空间坐标高扩大倍数作为bmp图像高度
		maped_texture.m_BmpInfo.bmiHeader.biPlanes = 1;
		maped_texture.m_BmpInfo.bmiHeader.biSize = 40;
		maped_texture.m_BmpInfo.bmiHeader.biSizeImage = 0;
		maped_texture.m_BmpInfo.bmiHeader.biWidth = OBJWidth*ENLARGE;     //空间坐标宽扩大倍数作为bmp图像宽度
		if (maped_texture.m_BmpInfo.bmiHeader.biWidth % 4 != 0)  //图片宽度保证是4的倍数（插值时需要）
		{
			maped_texture.m_BmpInfo.bmiHeader.biWidth += 1;
			if (maped_texture.m_BmpInfo.bmiHeader.biWidth % 4 != 0)
			{
				maped_texture.m_BmpInfo.bmiHeader.biWidth += 1;
				if (maped_texture.m_BmpInfo.bmiHeader.biWidth % 4 != 0)
				{
					maped_texture.m_BmpInfo.bmiHeader.biWidth += 1;
				}
			}
		}
		maped_texture.m_BmpInfo.bmiHeader.biXPelsPerMeter = 0;
		maped_texture.m_BmpInfo.bmiHeader.biYPelsPerMeter = 0;
		//file.Write(&maped_texture.m_BmpInfo.bmiHeader, sizeof(BITMAPINFOHEADER));

		//写入调色板
		if (maped_texture.m_BmpInfo.bmiHeader.biBitCount < 24)
		{
			//file.Write(&maped_texture.m_BmpInfo.bmiColors, maped_texture.GetColorNum(maped_texture.m_BmpInfo.bmiHeader) * sizeof(RGBQUAD));
		}

		//从obj中提取顶点像素信息，计算，转化，通过jpg转成的bmp纹理图片，写入bmp图片
		// long nBytePerLine1 = (maped_texture.m_BmpInfo.bmiHeader.biWidth*maped_texture.m_BmpInfo.bmiHeader.biBitCount + 31)/32*4;//图像每行所占实际字节数（须是4的倍数）
		long nBytePerLine1 = maped_texture.m_BmpInfo.bmiHeader.biWidth * 3;
		maped_texture.m_hgImageData1 = GlobalAlloc(GHND, nBytePerLine1 * maped_texture.m_BmpInfo.bmiHeader.biHeight);
		byte *pImageData1 = (byte *)GlobalLock(maped_texture.m_hgImageData1);

		int NewDataSize = nBytePerLine1 * maped_texture.m_BmpInfo.bmiHeader.biHeight;   //防止内存越界  新图大小


																						/////////////////////////////////// //设置底色/////////////////////////////////////////
		for (int k = 0; k < maped_texture.m_BmpInfo.bmiHeader.biHeight; k++)
		{
			for (int l = 0; l < nBytePerLine1; l++)
			{
				*(pImageData1 + l + k*nBytePerLine1) = 255;        //设置底色为白色
			}
		}

		//初始化一个容器 为方便插值像素，初始化为0，填充像素后为1
		vector<int> insert(maped_texture.m_BmpInfo.bmiHeader.biHeight*maped_texture.m_BmpInfo.bmiHeader.biWidth, 0);


		///////////////////////////////////采用遍历三角形的方式 对顶点像素信息存储///////////////////////////////////////
		double VertexU0, VertexV0, VertexU1, VertexV1, VertexU2, VertexV2;                            //初始化三顶点  纹理uv值
		double VertexX0, VertexY0, VertexX1, VertexY1, VertexX2, VertexY2;                            //初始化三顶点 空间坐标xy值  
		int OldBmpHeight0, OldBmpWidth0, OldBmpHeight1, OldBmpWidth1, OldBmpHeight2, OldBmpWidth2; //初始三点在纹理图中的位置
		int NewBmpHeight0, NewBmpWidth0, NewBmpHeight1, NewBmpWidth1, NewBmpHeight2, NewBmpWidth2; //初始三点在bmp图中的位置
		int OldBmpData0, NewBmpData0, OldBmpData1, NewBmpData1, OldBmpData2, NewBmpData2;
		//初始三点在纹理图和bmp图中的数据位置
		int giveup = 0, save = 0; //测试用 用来记录已经保存的所有三角形内部点 和 未保存下来的点的数量
		for (int i = 0; i < m_Model.faceCnt; i++)
		{
			if (m_Model.index[i].t[0]<0)
			{
				continue;
			}
			VertexU0 = m_Model.texture[m_Model.index[i].t[0]].u;                                //获取第一个点的uv值
			VertexV0 = m_Model.texture[m_Model.index[i].t[0]].v;

			VertexX0 = m_Model.vertex[m_Model.index[i].v[0]].x - m_Model.vertex[x_minpoint].x;  //获取第一个点的xy值 并做平移
			VertexY0 = m_Model.vertex[m_Model.index[i].v[0]].y - m_Model.vertex[y_minpoint].y;

			VertexU1 = m_Model.texture[m_Model.index[i].t[1]].u;                                //获取第二个点的uv值
			VertexV1 = m_Model.texture[m_Model.index[i].t[1]].v;

			VertexX1 = m_Model.vertex[m_Model.index[i].v[1]].x - m_Model.vertex[x_minpoint].x;  //获取第二个点的xy值 并做平移
			VertexY1 = m_Model.vertex[m_Model.index[i].v[1]].y - m_Model.vertex[y_minpoint].y;

			VertexU2 = m_Model.texture[m_Model.index[i].t[2]].u;                                //获取第三个点的uv值
			VertexV2 = m_Model.texture[m_Model.index[i].t[2]].v;

			VertexX2 = m_Model.vertex[m_Model.index[i].v[2]].x - m_Model.vertex[x_minpoint].x;  //获取第三个点的xy值 并做平移
			VertexY2 = m_Model.vertex[m_Model.index[i].v[2]].y - m_Model.vertex[y_minpoint].y;

			OldBmpHeight0 = int(VertexV0*PICTURELARGE + 0.5);                   //计算三个点在纹理图像中的位置即xy坐标
			OldBmpWidth0 = int(VertexU0*PICTURELARGE + 0.5);
			OldBmpHeight1 = int(VertexV1*PICTURELARGE + 0.5);
			OldBmpWidth1 = int(VertexU1*PICTURELARGE + 0.5);
			OldBmpHeight2 = int(VertexV2*PICTURELARGE + 0.5);
			OldBmpWidth2 = int(VertexU2*PICTURELARGE + 0.5);

			NewBmpHeight0 = int(VertexY0 * ENLARGE + 0.5);                      //计算三点在bmp图中的位置
			NewBmpWidth0 = int(VertexX0 * ENLARGE + 0.5);
			NewBmpHeight1 = int(VertexY1 * ENLARGE + 0.5);
			NewBmpWidth1 = int(VertexX1 * ENLARGE + 0.5);
			NewBmpHeight2 = int(VertexY2 * ENLARGE + 0.5);
			NewBmpWidth2 = int(VertexX2 * ENLARGE + 0.5);

			//三顶点的保存
			OldBmpData0 = (OldBmpHeight0 * maped_texture.nBytePerLine + OldBmpWidth0 * 4);            //得到三顶点点在乱序bmp中的像素位置
			NewBmpData0 = (NewBmpHeight0 * nBytePerLine1 + NewBmpWidth0 * 3);           //得到三顶点在将要保存的bmp像素位置  
			OldBmpData1 = (OldBmpHeight1 * maped_texture.nBytePerLine + OldBmpWidth1 * 4);
			NewBmpData1 = (NewBmpHeight1 * nBytePerLine1 + NewBmpWidth1 * 3);
			OldBmpData2 = (OldBmpHeight2 * maped_texture.nBytePerLine + OldBmpWidth2 * 4);
			NewBmpData2 = (NewBmpHeight2 * nBytePerLine1 + NewBmpWidth2 * 3);

			if ((NewBmpData0 + 2) <= NewDataSize && (OldBmpData0 + 2) <= OldDataSize) //防止内存溢出
			{
				*(pImageData1 + NewBmpData0 + 0) = *(maped_texture.pImageData + OldBmpData0 + 0);         //修改第一个点像素信息	BGR
				*(pImageData1 + NewBmpData0 + 1) = *(maped_texture.pImageData + OldBmpData0 + 1);
				*(pImageData1 + NewBmpData0 + 2) = *(maped_texture.pImageData + OldBmpData0 + 2);
				insert[NewBmpHeight0*maped_texture.m_BmpInfo.bmiHeader.biWidth + NewBmpWidth0] = 1;
			}

			if ((NewBmpData1 + 2) <= NewDataSize && (OldBmpData1 + 2) <= OldDataSize)
			{
				*(pImageData1 + NewBmpData1 + 0) = *(maped_texture.pImageData + OldBmpData1 + 0);         //修改第二个点像素信息	BGR
				*(pImageData1 + NewBmpData1 + 1) = *(maped_texture.pImageData + OldBmpData1 + 1);
				*(pImageData1 + NewBmpData1 + 2) = *(maped_texture.pImageData + OldBmpData1 + 2);
				insert[NewBmpHeight1*maped_texture.m_BmpInfo.bmiHeader.biWidth + NewBmpWidth1] = 1;
			}

			if ((NewBmpData2 + 2) <= NewDataSize && (OldBmpData2 + 2) <= OldDataSize)
			{

				*(pImageData1 + NewBmpData2 + 0) = *(maped_texture.pImageData + OldBmpData2 + 0);         //修改第三个点像素信息	BGR
				*(pImageData1 + NewBmpData2 + 1) = *(maped_texture.pImageData + OldBmpData2 + 1);
				*(pImageData1 + NewBmpData2 + 2) = *(maped_texture.pImageData + OldBmpData2 + 2);
				insert[NewBmpHeight2*maped_texture.m_BmpInfo.bmiHeader.biWidth + NewBmpWidth2] = 1;
			}


			//判断将要保存bmp中三角形所在矩形区域
			double NewBmpX0 = VertexX0*ENLARGE, NewBmpY0 = VertexY0*ENLARGE,  //计算三定点在bmp图中的坐标
				NewBmpX1 = VertexX1*ENLARGE, NewBmpY1 = VertexY1*ENLARGE,
				NewBmpX2 = VertexX2*ENLARGE, NewBmpY2 = VertexY2*ENLARGE;
			double RectNewMinX, RectNewMaxX, RectNewMinY, RectNewMaxY;     //初始化矩形左下角 右上角坐标
			double DRectNewMinX = NewBmpX0, DRectNewMaxX = NewBmpX0,   //为不损失精度 增加double形先进行比较
				DRectNewMinY = NewBmpY0, DRectNewMaxY = NewBmpY0;
			if (DRectNewMinX > NewBmpX1)
			{
				DRectNewMinX = NewBmpX1;
			}
			if (DRectNewMinX > NewBmpX2)
			{
				DRectNewMinX = NewBmpX2;    //新图左下角x坐标
			}

			if (DRectNewMaxX < NewBmpX1)
			{
				DRectNewMaxX = NewBmpX1;
			}
			if (DRectNewMaxX < NewBmpX2)
			{
				DRectNewMaxX = NewBmpX2;    //新图右上角x坐标
			}

			if (DRectNewMinY > NewBmpY1)
			{
				DRectNewMinY = NewBmpY1;
			}
			if (DRectNewMinY > NewBmpY2)
			{
				DRectNewMinY = NewBmpY2;   //新图左下角y坐标
			}

			if (DRectNewMaxY < NewBmpY1)
			{
				DRectNewMaxY = NewBmpY1;
			}
			if (DRectNewMaxY < NewBmpY2)
			{
				DRectNewMaxY = NewBmpY2;   //新图右上角y坐标
			}

			RectNewMinX = int(DRectNewMinX + 0.5);
			RectNewMaxX = int(DRectNewMaxX + 0.5);
			RectNewMinY = int(DRectNewMinY + 0.5);
			RectNewMaxY = int(DRectNewMaxY + 0.5);
			int NewRectWidth = int(RectNewMaxX - RectNewMinX);      //新bmp矩形长
			int NewRectHeight = int(RectNewMaxY - RectNewMinY);     //新bmp矩形高

																	/////////////////////////////////用坐标变换方法，计算每个三角形的映射矩阵，保存像素点////////////////////////////
			double a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;         //变换矩阵
			double A11, A12, A13, A21, A22, A23, A31, A32, A33;  //伴随矩阵的代数余子式值
			double DET;   //矩阵行列式的值
			double C1, D1, C2, D2, C3, D3;  //纹理图三点矩阵
			double A1, B1, A2, B2, A3, B3;  //bmp三点组成的矩阵
			double a1, b1, a2, b2, a3, b3, c1, c2, c3; //逆矩阵

			A1 = VertexX0*ENLARGE, B1 = VertexY0*ENLARGE,
				A2 = VertexX1*ENLARGE, B2 = VertexY1*ENLARGE,
				A3 = VertexX2*ENLARGE, B3 = VertexY2*ENLARGE;

			C1 = VertexU0*PICTURELARGE, D1 = VertexV0*PICTURELARGE;
			C2 = VertexU1*PICTURELARGE, D2 = VertexV1*PICTURELARGE;
			C3 = VertexU2*PICTURELARGE, D3 = VertexV2*PICTURELARGE;

			DET = A1*B2 + A2*B3 + B1*A3 - B2*A3 - B1*A2 - A1*B3;
			if (DET != 0)
			{
				//计算A*
				A11 = B2 - B3;
				A12 = B3 - B1;
				A13 = B1 - B2;
				A21 = A3 - A2;
				A22 = A1 - A3;
				A23 = A2 - A1;
				A31 = A2*B3 - A3*B2;
				A32 = A3*B1 - A1*B3;
				A33 = A1*B2 - A2*B1;
				//计算逆矩阵
				a1 = double(A11) / DET;
				a2 = double(A21) / DET;
				a3 = double(A31) / DET;
				b1 = double(A12) / DET;
				b2 = double(A22) / DET;
				b3 = double(A32) / DET;
				c1 = double(A13) / DET;
				c2 = double(A23) / DET;
				c3 = double(A33) / DET;
				////计算变换矩阵
				a = C1*a1 + C2*b1 + C3*c1;
				b = C1*a2 + C2*b2 + C3*c2;
				c = C1*a3 + C2*b3 + C3*c3;
				d = D1*a1 + D2*b1 + D3*c1;
				e = D1*a2 + D2*b2 + D3*c2;
				f = D1*a3 + D2*b3 + D3*c3;
			}

			//逆序存储
			//利用叉乘法 判断new三角形所成矩形内的点，是否属于三角形内  
			double Ax, Ay, Bx, By, Cx, Cy;       //初始化三角形三顶点xy坐标，和将要判断点坐标的xy坐标
			int Mx, My;
			double MAx, MAy, MBx, MBy, MCx, MCy;       //初始化 向量MA MB MC 的横纵坐标
			double MAMB, MBMC, MCMA;                //初始化 MA叉乘MB MB叉乘MC MC叉乘MA的值
			Ax = (VertexX0 * ENLARGE);
			Ay = (VertexY0 * ENLARGE);
			Bx = (VertexX1 * ENLARGE);
			By = (VertexY1 * ENLARGE);
			Cx = (VertexX2 * ENLARGE);
			Cy = (VertexY2 * ENLARGE);
			for (int q = 0; q < NewRectHeight; q++)
				for (int w = 0; w < NewRectWidth; w++)
				{
					Mx = int(RectNewMinX + w + 0.5);
					My = int(RectNewMinY + q + 0.5);
					if ((Mx == Ax && My == Ay) || (Mx == Bx && Mx == By) || (Mx == Cx && My == Cy)) //如果判断点是三角形顶点 跳出循环
					{
						break;
					}
					else
					{
						MAx = Ax - Mx;
						MAy = Ay - My;
						MBx = Bx - Mx;
						MBy = By - My;
						MCx = Cx - Mx;
						MCy = Cy - My;

						MAMB = MAx*MBy - MBx*MAy;
						MBMC = MBx*MCy - MCx*MBy;
						MCMA = MCx*MAy - MAx*MCy;

						//判断叉乘均大于或小于0 说明在内部 若有一值为0说明在边上 将点进行保存 否则跳出
						if ((MAMB > 0 && MBMC > 0 && MCMA > 0) || (MAMB < 0 && MBMC < 0 && MCMA < 0) || MAMB == 0 || MBMC == 0 || MCMA == 0)
						{
							int OldBmpMx, OldBmpMy;     //初始化将要保存的点在old图中的位置   将要保存的点在new图中的位置为Mx My
							int OldBmpDataPoint, NewBmpDataPoint; //初始化 old new图中的数据缓冲区位置

																  //计算点在old图中的位置
							OldBmpMx = int(a*Mx + b*My + c + 0.5);
							OldBmpMy = int(d*Mx + e*My + f + 0.5);

							//计算数据缓冲区位置
							NewBmpDataPoint = (My * nBytePerLine1 + Mx * 3);
							OldBmpDataPoint = (OldBmpMy * maped_texture.nBytePerLine + OldBmpMx * 4);

							//修改像素信息
							if ((NewBmpDataPoint + 2) <= NewDataSize && (OldBmpDataPoint + 2) <= OldDataSize && OldBmpDataPoint >= 0) //防止内存溢出
							{
								*(pImageData1 + NewBmpDataPoint + 0) = *(maped_texture.pImageData + OldBmpDataPoint + 0);
								*(pImageData1 + NewBmpDataPoint + 1) = *(maped_texture.pImageData + OldBmpDataPoint + 1);
								*(pImageData1 + NewBmpDataPoint + 2) = *(maped_texture.pImageData + OldBmpDataPoint + 2);
								insert[My*maped_texture.m_BmpInfo.bmiHeader.biWidth + Mx] = 1;
								save++;
							}
							else giveup++;
						}
					}
				}
		}

		///////////////////////////////////下面进行插值算法///////////////////////////////////////
		int count1 = 0; // 实验计数用，count1为已经填充的像素点的个数
		long allpixels = maped_texture.m_BmpInfo.bmiHeader.biWidth*maped_texture.m_BmpInfo.bmiHeader.biHeight;
		int bmpwidth = maped_texture.m_BmpInfo.bmiHeader.biWidth;
		int bmpheight = maped_texture.m_BmpInfo.bmiHeader.biHeight;
		for (int m = 0; m != maped_texture.m_BmpInfo.bmiHeader.biWidth*maped_texture.m_BmpInfo.bmiHeader.biHeight; m++)
		{
			if (insert[m] == 0 && (m * 3 + 2) <= NewDataSize) //判断空白点 若是进行下面插值
			{
				if (m * 3 - nBytePerLine1 > 0 && m * 3 + nBytePerLine1 < NewDataSize && (m * 3) % nBytePerLine1 != 0 &&
					((m + 1) * 3) % nBytePerLine1 != 0)  //非边界点的插值
				{
					//四邻域点均不空的点插值
					if (insert[m - 1] != 0 && insert[m + 1] != 0 && insert[m + bmpwidth] != 0 && insert[m - bmpwidth] != 0)
					{
						*(pImageData1 + m * 3 + 0) = ((*(pImageData1 + m * 3 - 3)) + (*(pImageData1 + m * 3 + 3))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))) / 4;

						*(pImageData1 + m * 3 + 1) = ((*(pImageData1 + m * 3 - 2)) + (*(pImageData1 + m * 3 + 4))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))) / 4;

						*(pImageData1 + m * 3 + 2) = ((*(pImageData1 + m * 3 - 1)) + (*(pImageData1 + m * 3 + 5))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))) / 4;
					}
					//邻域四点有三点不空的情况(上空)
					if (insert[m - 1] != 0 && insert[m + 1] != 0 && insert[m - bmpwidth] != 0 && insert[m + bmpwidth] == 0)
					{
						*(pImageData1 + m * 3 + 0) = ((*(pImageData1 + m * 3 - 3)) + (*(pImageData1 + m * 3 + 3))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))) / 3;

						*(pImageData1 + m * 3 + 1) = ((*(pImageData1 + m * 3 - 2)) + (*(pImageData1 + m * 3 + 4))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))) / 3;

						*(pImageData1 + m * 3 + 2) = ((*(pImageData1 + m * 3 - 1)) + (*(pImageData1 + m * 3 + 5))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))) / 3;
					}
					//邻域四点有三点不空的情况(下空)
					if (insert[m - 1] != 0 && insert[m + 1] != 0 && insert[m + bmpwidth] != 0 && insert[m - bmpwidth] == 0)
					{
						*(pImageData1 + m * 3 + 0) = ((*(pImageData1 + m * 3 - 3)) + (*(pImageData1 + m * 3 + 3))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))) / 3;

						*(pImageData1 + m * 3 + 1) = ((*(pImageData1 + m * 3 - 2)) + (*(pImageData1 + m * 3 + 4))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))) / 3;

						*(pImageData1 + m * 3 + 2) = ((*(pImageData1 + m * 3 - 1)) + (*(pImageData1 + m * 3 + 5))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))) / 3;
					}
					//邻域四点有三点不空的情况(左空)
					if (insert[m - 1] == 0 && insert[m + 1] != 0 && insert[m + bmpwidth] != 0 && insert[m - bmpwidth] != 0)
					{
						*(pImageData1 + m * 3 + 0) = ((*(pImageData1 + m * 3 + 3))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))) / 3;

						*(pImageData1 + m * 3 + 1) = ((*(pImageData1 + m * 3 + 4))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))) / 3;

						*(pImageData1 + m * 3 + 2) = ((*(pImageData1 + m * 3 + 5))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))) / 3;
					}
					//邻域四点有三点不空的情况(右空)
					if (insert[m - 1] != 0 && insert[m + 1] == 0 && insert[m + bmpwidth] != 0 && insert[m - bmpwidth] != 0)
					{
						*(pImageData1 + m * 3 + 0) = ((*(pImageData1 + m * 3 - 3))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))) / 3;

						*(pImageData1 + m * 3 + 1) = ((*(pImageData1 + m * 3 - 2))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))) / 3;

						*(pImageData1 + m * 3 + 2) = ((*(pImageData1 + m * 3 - 1))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))) / 3;
					}
					//邻域四点有两点不空的情况(右下空)
					if (insert[m - 1] != 0 && insert[m + 1] == 0 && insert[m + bmpwidth] != 0 && insert[m - bmpwidth] == 0)
					{
						*(pImageData1 + m * 3 + 0) = ((*(pImageData1 + m * 3 - 3)) +
							(*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))) / 2;

						*(pImageData1 + m * 3 + 1) = ((*(pImageData1 + m * 3 - 2)) +
							(*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))) / 2;

						*(pImageData1 + m * 3 + 2) = ((*(pImageData1 + m * 3 - 1)) +
							(*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))) / 2;
					}
					//邻域四点有两点不空的情况(左下空)
					if (insert[m - 1] == 0 && insert[m + 1] != 0 && insert[m + bmpwidth] != 0 && insert[m - bmpwidth] == 0)
					{
						*(pImageData1 + m * 3 + 0) = ((*(pImageData1 + m * 3 + 3))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))) / 2;

						*(pImageData1 + m * 3 + 1) = ((*(pImageData1 + m * 3 + 4))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))) / 2;

						*(pImageData1 + m * 3 + 2) = ((*(pImageData1 + m * 3 + 5))
							+ (*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))) / 2;
					}
					//邻域四点有两点不空的情况(右上空)
					if (insert[m - 1] != 0 && insert[m + 1] == 0 && insert[m + bmpwidth] == 0 && insert[m - bmpwidth] != 0)
					{
						*(pImageData1 + m * 3 + 0) = ((*(pImageData1 + m * 3 - 3))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))) / 2;

						*(pImageData1 + m * 3 + 1) = ((*(pImageData1 + m * 3 - 2))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))) / 2;

						*(pImageData1 + m * 3 + 2) = ((*(pImageData1 + m * 3 - 1))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))) / 2;
					}
					//邻域四点有两点不空的情况(上下空)
					if (insert[m - 1] != 0 && insert[m + 1] != 0 && insert[m + bmpwidth] == 0 && insert[m - bmpwidth] == 0)
					{
						*(pImageData1 + m * 3 + 0) = ((*(pImageData1 + m * 3 - 3)) + (*(pImageData1 + m * 3 + 3))) / 2;
						*(pImageData1 + m * 3 + 1) = ((*(pImageData1 + m * 3 - 2)) + (*(pImageData1 + m * 3 + 4))) / 2;
						*(pImageData1 + m * 3 + 2) = ((*(pImageData1 + m * 3 - 1)) + (*(pImageData1 + m * 3 + 5))) / 2;
					}
					//邻域四点有两点不空的情况(左右空)
					if (insert[m - 1] == 0 && insert[m + 1] == 0 && insert[m + bmpwidth] != 0 && insert[m - bmpwidth] != 0)
					{
						*(pImageData1 + m * 3 + 0) = ((*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))) / 2;

						*(pImageData1 + m * 3 + 1) = ((*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))) / 2;

						*(pImageData1 + m * 3 + 2) = ((*(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))) / 2;
					}
					//邻域四点有两点不空的情况(左上空)
					if (insert[m - 1] == 0 && insert[m + 1] != 0 && insert[m + bmpwidth] == 0 && insert[m - bmpwidth] != 0)
					{
						*(pImageData1 + m * 3 + 0) = ((*(pImageData1 + m * 3 + 3))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth))) / 2;

						*(pImageData1 + m * 3 + 1) = ((*(pImageData1 + m * 3 + 4))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1))) / 2;

						*(pImageData1 + m * 3 + 2) = ((*(pImageData1 + m * 3 + 5))
							+ (*(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2))) / 2;
					}
					//四邻域点只有一点不空（左不空）
					if (insert[m - 1] != 0 && insert[m + 1] == 0 && insert[m + bmpwidth] == 0 && insert[m - bmpwidth] == 0)
					{
						*(pImageData1 + m * 3 + 0) = *(pImageData1 + m * 3 - 3);
						*(pImageData1 + m * 3 + 1) = *(pImageData1 + m * 3 - 2);
						*(pImageData1 + m * 3 + 2) = *(pImageData1 + m * 3 - 1);
					}
					//四邻域点只有一点不空（右不空）
					if (insert[m - 1] == 0 && insert[m + 1] != 0 && insert[m + bmpwidth] == 0 && insert[m - bmpwidth] == 0)
					{
						*(pImageData1 + m * 3 + 0) = *(pImageData1 + m * 3 + 3);
						*(pImageData1 + m * 3 + 1) = *(pImageData1 + m * 3 + 4);
						*(pImageData1 + m * 3 + 2) = *(pImageData1 + m * 3 + 5);
					}
					//四邻域点只有一点不空（上不空）
					if (insert[m - 1] == 0 && insert[m + 1] == 0 && insert[m + bmpwidth] != 0 && insert[m - bmpwidth] == 0)
					{
						*(pImageData1 + m * 3 + 0) = *(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth);
						*(pImageData1 + m * 3 + 1) = *(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1);
						*(pImageData1 + m * 3 + 2) = *(pImageData1 + m * 3 + 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2);

					}
					//四邻域点只有一点不空（下不空）
					if (insert[m - 1] == 0 && insert[m + 1] == 0 && insert[m + bmpwidth] == 0 && insert[m - bmpwidth] != 0)
					{
						*(pImageData1 + m * 3 + 0) = *(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth);
						*(pImageData1 + m * 3 + 1) = *(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 1);
						*(pImageData1 + m * 3 + 2) = *(pImageData1 + m * 3 - 3 * maped_texture.m_BmpInfo.bmiHeader.biWidth + 2);
					}
					//四邻域点均为空
					if (insert[m - 1] == 0 && insert[m + 1] == 0 && insert[m + bmpwidth] == 0 && insert[m - bmpwidth] == 0)
					{
						*(pImageData1 + m * 3 + 0) = 255;
						*(pImageData1 + m * 3 + 1) = 255;  //绿色
						*(pImageData1 + m * 3 + 2) = 255;
					}

				}
				else //即最外层一周的像素点
				{
					*(pImageData1 + m * 3 + 0) = 255;
					*(pImageData1 + m * 3 + 1) = 255;
					*(pImageData1 + m * 3 + 2) = 255;  //红色
				}
			}
			else
				count1++;
		}


		//file.Write(pImageData1, nBytePerLine1 * maped_texture.m_BmpInfo.bmiHeader.biHeight);

		cv::Mat iMat(bmpheight, bmpwidth, CV_8UC3, pImageData1, nBytePerLine1);
		//cv::Mat rmat;
		//iMat = cv::imread("E:\\studfile\\yingxiang\\培养方向\\CODE\\3DBodyReconstruction\\3DBodyReconstruction\\obj.bmp");
		cv::namedWindow("Projected Image", CV_WINDOW_NORMAL);
		cv::flip(iMat, pm_Model->texMat, 0);//fanzhuan 0 x  1 y
		cv::resize(pm_Model->texMat, pm_Model->texMat, cv::Size(), 0.35, 0.35);
		string s = pm_Model->map;
		s = s + ".jpg";
		cv::imwrite(s, pm_Model->texMat);
		cv::imshow("Projected Image",pm_Model->texMat);
		//cv::Mat imgsaved = cv::imread(s);
		//cv::namedWindow("saved", CV_WINDOW_NORMAL);
		//cv::imshow("saved", imgsaved);
		cv::waitKey(0);

		GlobalUnlock(maped_texture.m_hgImageData1);
		// GlobalFree(maped_texture.m_hgImageData);   //释放内存
		GlobalFree(maped_texture.m_hgImageData1);
		//delete maped_texture.pImageData;  //不能释放内存，否则只能保存一次
		//清除尚未写入的数据并关闭文件
		//file.Flush();
		//file.Close();

	}


	//抛出异常处理
	catch (CFileException *e)
	{
		CString str;
		str.Format("保存数据失败,原因是:%d", e->m_cause);
		AfxMessageBox(str);
		//file.Abort();
		e->Delete();
	}
	////////////////////////////////////BMP保存结束////////////////////////////////
	//drawScene();
}
BOOL getTextureWH(char *szPathName, int &lWidthPixels, int &lHeightPixels)						// 载入图片并转换为纹理
{
	HDC			hdcTemp;												// DC用来保存位图
	HBITMAP		hbmpTemp;												// 保存临时位图
	IPicture	*pPicture;												// 定义IPicture Interface
	OLECHAR		wszPath[MAX_PATH + 1];									// 图片的完全路径
	char		szPath[MAX_PATH + 1];										// 图片的完全路径
	long		lWidth;													// 图像宽度
	long		lHeight;												// 图像高度
																		//long		lWidthPixels;											// 图像的宽带(以像素为单位)
																		//long		lHeightPixels;											// 图像的高带(以像素为单位)
	GLint		glMaxTexDim;											// 保存纹理的最大尺寸

	strcpy(szPath, szPathName);

	MultiByteToWideChar(CP_ACP, 0, szPath, -1, wszPath, MAX_PATH);		// 把ASCII码转化为Unicode标准码
	HRESULT hr = OleLoadPicturePath(wszPath, 0, 0, 0, IID_IPicture, (void**)&pPicture);

	if (FAILED(hr))														// 如果导入失败
	{
		// 图片载入失败出错信息
		MessageBox(HWND_DESKTOP, "图片导入失败1!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// 返回 FALSE
	}

	hdcTemp = CreateCompatibleDC(GetDC(0));								// 建立窗口设备描述表
	if (!hdcTemp)														// 建立失败?
	{
		pPicture->Release();											// 释放IPicture
																		// 图片载入失败出错信息
		MessageBox(HWND_DESKTOP, "图片导入失败2!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// 返回 FALSE
	}

	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &glMaxTexDim);					// 取得支持的纹理最大尺寸

	pPicture->get_Width(&lWidth);										// 取得IPicture 宽度 (转换为Pixels格式)
	lWidthPixels = MulDiv(lWidth, GetDeviceCaps(hdcTemp, LOGPIXELSX), 2540);
	pPicture->get_Height(&lHeight);										// 取得IPicture 高度 (转换为Pixels格式)
	lHeightPixels = MulDiv(lHeight, GetDeviceCaps(hdcTemp, LOGPIXELSY), 2540);


	return TRUE;														// 返回 TRUE
}


void COGLView::OnBnClickedButtonClearModel()
{
	// TODO: 在此添加控件通知处理程序代码
	if (m_Model.vertex != NULL)	// 释放数据
	{
		free(m_Model.vertex);
		m_Model.vertex = NULL;
		m_Model.vertexCnt = 0;	// Set the vertex count
		m_Model.faceCnt = 0;	// Set the vertex count
		m_Model.uvCnt = 0;	// Set the vertex count
		m_Model.normalCnt = 0;	// Set the normal count
	}
	if (m_Model.GuassCurvature != NULL)	// 释放数据
	{
		free(m_Model.GuassCurvature);
		m_Model.GuassCurvature = NULL;
	}
	if (m_Model.HCurvature != NULL)	// 释放数据
	{
		free(m_Model.HCurvature);
		m_Model.HCurvature = NULL;
	}
	if (m_Model.Gvertex != NULL)	// 释放数据
	{
		free(m_Model.Gvertex);
		m_Model.Gvertex = NULL;
	}
	if (m_Model.deformData != NULL)	// 释放数据
	{
		free(m_Model.deformData);
		m_Model.deformData = NULL;
	}
	if (m_Model.normal != NULL)	// 释放数据
	{
		free(m_Model.normal);
		m_Model.normal = NULL;
	}
	if (m_Model.texture != NULL)	// 释放数据
	{
		free(m_Model.texture);
		m_Model.texture = NULL;
	}
	if (m_Model.vertexColor != NULL)	// 释放数据
	{
		free(m_Model.vertexColor);
		m_Model.vertexColor = NULL;
	}
	if (m_Model.index != NULL)	//释放数据
	{
		free(m_Model.index);
		m_Model.index = NULL;
	}
	if (m_Model.vfindex != NULL)	//释放数据
	{
		free(m_Model.vfindex);
		m_Model.vfindex = NULL;
	}
	memset(m_Model.map, 0, 255);
	memset(m_Model.map2, 0, 255);
	if (m_GModel.vertex != NULL)	// 释放数据
	{
		free(m_GModel.vertex);
		m_GModel.vertex = NULL;
		m_GModel.vertexCnt = 0;	// Set the vertex count
		m_GModel.faceCnt = 0;	// Set the vertex count
		m_GModel.uvCnt = 0;	// Set the vertex count
		m_GModel.normalCnt = 0;	// Set the normal count
	}
	if (m_GModel.GuassCurvature != NULL)	// 释放数据
	{
		free(m_GModel.GuassCurvature);
		m_GModel.GuassCurvature = NULL;
	}
	if (m_GModel.HCurvature != NULL)	// 释放数据
	{
		free(m_GModel.HCurvature);
		m_GModel.HCurvature = NULL;
	}
	if (m_GModel.Gvertex != NULL)	// 释放数据
	{
		free(m_GModel.Gvertex);
		m_GModel.Gvertex = NULL;
	}
	if (m_GModel.deformData != NULL)	// 释放数据
	{
		free(m_GModel.deformData);
		m_GModel.deformData = NULL;
	}
	if (m_GModel.normal != NULL)	// 释放数据
	{
		free(m_GModel.normal);
		m_GModel.normal = NULL;
	}
	if (m_GModel.texture != NULL)	// 释放数据
	{
		free(m_GModel.texture);
		m_GModel.texture = NULL;
	}
	if (m_GModel.vertexColor != NULL)	// 释放数据
	{
		free(m_GModel.vertexColor);
		m_GModel.vertexColor = NULL;
	}
	if (m_GModel.index != NULL)	//释放数据
	{
		free(m_GModel.index);
		m_GModel.index = NULL;
	}
	if (m_GModel.vfindex != NULL)	//释放数据
	{
		free(m_GModel.vfindex);
		m_GModel.vfindex = NULL;
	}
	memset(m_GModel.map, 0, 255);
	memset(m_GModel.map2, 0, 255);

	drawScene();
}
