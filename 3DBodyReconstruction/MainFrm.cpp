
// MainFrm.cpp : CMainFrame 类的实现
//

#include "stdafx.h"
#include "OGLView.h"
#include "Registration.h"
#include "mmsystem.h"
#include "3DBodyReconstruction.h"
#include "MainFrm.h"

#include <time.h>
#include <string.h>
#include <math.h>
#include <vector>
using namespace std;
#ifdef _DEBUG
#define new DEBUG_NEW
#endif
#define OGLWIN_START_X	230			// STARTING X POSITION OF OPENGL WINDOW
#define OGLWIN_START_Y	1			// STARTING Y POSITION OF OPENGL WINDOW
#define OGLWIN_WIDTH	4			// WIDTH OF OPENGL WINDOW SUBTRACTED FROM MAX
#define OGLWIN_BOTTOM	20			// BOTTOM BORDER OF OPENGL WINDOW
// CMainFrame

IMPLEMENT_DYNCREATE(CMainFrame, CFrameWnd)

const int  iMaxUserToolbars = 10;
const UINT uiFirstUserToolBarId = AFX_IDW_CONTROLBAR_FIRST + 40;
const UINT uiLastUserToolBarId = uiFirstUserToolBarId + iMaxUserToolbars - 1;

BEGIN_MESSAGE_MAP(CMainFrame, CFrameWnd)
	ON_WM_CREATE()
	ON_WM_SIZE()
	ON_WM_KEYDOWN()
	ON_WM_KEYUP()
	ON_WM_PAINT()
	ON_COMMAND(ID_FILE_OPEN, &CMainFrame::OnFileOpen)
	ON_COMMAND(ID_File_OBJImport, &CMainFrame::OnFileObjimport)
	ON_COMMAND(ID_btn_rudeRegister, &CMainFrame::Onbtnruderegister)
	ON_COMMAND(ID_btn_ICP_PointToPlane, &CMainFrame::OnbtnIcpPointtoplane)
	ON_COMMAND(ID_btn_GuassCurvature, &CMainFrame::OnbtnGuasscurvature)
	ON_COMMAND(ID_btn_ICP_nonrigid, &CMainFrame::OnbtnIcpnonrigid)
	ON_COMMAND(ID_btn_ICP_error, &CMainFrame::OnbtnIcperror)
	ON_COMMAND(ID_TPS_NonRigid, &CMainFrame::OnTpsNonrigid)
END_MESSAGE_MAP()

static UINT indicators[] =
{
	ID_SEPARATOR,           // 状态行指示器
	ID_INDICATOR_CAPS,
	ID_INDICATOR_NUM,
	ID_INDICATOR_SCRL,
};

// CMainFrame 构造/析构

CMainFrame::CMainFrame()
{
	// TODO: 在此添加成员初始化代码
}

CMainFrame::~CMainFrame()
{
}

int CMainFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	RECT rect;
	///////////////////////////////////////////////////////////////////////////////
	GetClientRect(&rect);

	if (CFrameWnd::OnCreate(lpCreateStruct) == -1)
		return -1;

	if (!m_wndStatusBar.Create(this) ||
		!m_wndStatusBar.SetIndicators(indicators,
			sizeof(indicators) / sizeof(UINT)))
	{
		TRACE0("Failed to create status bar\n");
		return -1;      // fail to create
	}

	if (!m_wndleftForm.Create(this, IDD_FORMVIEW_left, CBRS_LEFT, IDD_FORMVIEW_left))
		return -1;
	m_wndleftForm.EnableDocking(CBRS_ALIGN_ANY);
	m_OGLView.Create(NULL, "Render Window", WS_CHILD | WS_VISIBLE,
		CRect(OGLWIN_START_X, OGLWIN_START_Y, rect.right - OGLWIN_WIDTH, rect.bottom - OGLWIN_BOTTOM), this, 104);
	m_OGLView.ShowWindow(TRUE);

	m_OGLView.m_StatusBar = &m_wndStatusBar;

	m_OGLView.Invalidate(TRUE);

	//DockPane(&m_leftDockPane);
	//CPane::DockPane(&m_leftDockPane);
	// TODO: 如果不需要可停靠工具栏，则删除这三行
	//m_wndToolBar.EnableDocking(CBRS_ALIGN_ANY);
	EnableDocking(CBRS_ALIGN_ANY);
	DockControlBar(&m_wndleftForm);
	return 0;
}

BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	if( !CFrameWnd::PreCreateWindow(cs) )
		return FALSE;
	// TODO: 在此处通过修改
	//  CREATESTRUCT cs 来修改窗口类或样式

	return TRUE;
}
//BOOL CMainFrame::PreTranslateMessage(MSG* pMsg)
//{
//	if(pMsg->message == WM_KEYDOWN)
//	{
//		//return m_OGLView.PreTranslateMessage(pMsg);
//		return 0;
//	}
//}
void CMainFrame::OnSize(UINT nType, int cx, int cy)
{
	// RESET THE m_OGLView WINDOW SIZE
	m_OGLView.SetWindowPos(&wndTopMost, OGLWIN_START_X, OGLWIN_START_Y, cx - OGLWIN_WIDTH, cy - OGLWIN_BOTTOM, SWP_NOZORDER);
	//m_OGLView.SetWindowPos(&m_wndleftForm, OGLWIN_START_X, OGLWIN_START_Y, cx - OGLWIN_WIDTH, cy - OGLWIN_BOTTOM, SWP_NOZORDER);
	// RESET THE ACTUAL OPENGL WINDOW SIZE
	m_OGLView.resize(cx - OGLWIN_WIDTH, cy - OGLWIN_BOTTOM);
	CFrameWnd::OnSize(nType, cx, cy);
}
void CMainFrame::OnPaint()
{
	CPaintDC dc(this); // device context for painting

	m_OGLView.drawScene();
}
/// OnPaint
////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Procedure:	OnWhichogl
// Purpose:		Create dialog to Show which version of OGL is running
///////////////////////////////////////////////////////////////////////////////		
void CMainFrame::OnWhichogl()
{
	m_OGLView.GetGLInfo();
}
// OnWhichogl


// CMainFrame 诊断

#ifdef _DEBUG
void CMainFrame::AssertValid() const
{
	CFrameWnd::AssertValid();
}

void CMainFrame::Dump(CDumpContext& dc) const
{
	CFrameWnd::Dump(dc);
}
#endif //_DEBUG


// CMainFrame 消息处理程序



void CMainFrame::OnFileOpen()
{
	// TODO: 在此添加命令处理程序代码
	char BASED_CODE szFilter[] = "Object Mesh OBJ (*.obj)|*.obj||";
	CFileDialog	*dialog;
	CString exten;
	///////////////////////////////////////////////////////////////////////////////
	dialog = new CFileDialog(TRUE, "dcm", NULL, NULL, szFilter);
	if (dialog->DoModal() == IDOK)
	{
		exten = dialog->GetFileExt();
		exten.MakeUpper();
		m_OGLView.LoadOBJModel(dialog->GetPathName());
		m_OGLView.SetCamera();
		m_OGLView.drawScene();
	}
	delete dialog;
}
/************************************************************************/
/* 导入 obj                                                                     */
/************************************************************************/
void CMainFrame::OnFileObjimport()
{
	// TODO: 在此添加命令处理程序代码
	char BASED_CODE szFilter[] = "Object Mesh OBJ (*.obj)|*.obj||";
	CFileDialog	*dialog;
	CString exten;
	///////////////////////////////////////////////////////////////////////////////
	dialog = new CFileDialog(TRUE, "dcm", NULL, NULL, szFilter);
	if (dialog->DoModal() == IDOK)
	{
		exten = dialog->GetFileExt();
		exten.MakeUpper();
		m_OGLView.LoadOBJGModel(dialog->GetPathName());
		
		m_OGLView.drawScene();
	}
	delete dialog;
}
void CMainFrame::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	m_OGLView.HandleKeyDown(nChar);
	CFrameWnd::OnKeyDown(nChar, nRepCnt, nFlags);
}

void CMainFrame::OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	CFrameWnd::OnKeyUp(nChar, nRepCnt, nFlags);
	m_OGLView.HandleKeyUp(nChar);
}



void CMainFrame::Onbtnruderegister()
{
	time_t startT,endT;
	// TODO: 在此添加命令处理程序代码
	if (m_OGLView.NumofObj < 2)
	{ 
		AfxMessageBox("缺少模型！请加载至少两个模型");
		return;
	}
	Registration PointCloudRegister(&m_OGLView.m_Model, &m_OGLView.m_GModel);
	startT=clock();
	if (!PointCloudRegister.Regist(&m_OGLView.m_Model, &m_OGLView.m_GModel))
	{
		AfxMessageBox("配准失败！");
		return;
	}
	else
	{
		endT = clock();
		double runtim = (double)(endT - startT) / CLOCKS_PER_SEC;
		//char str[256];
		//sprintf(str, "%lf", runtim);
		//给richedit控件传递参数
		CString temp;
		temp.Format(_T("%.3f"), runtim);
		CString s="刚性耗时： "+temp+" s\n矩阵：\n" ;

		Eigen::Matrix3d ER= PointCloudRegister.ER;//旋转矩阵R
		Eigen::Vector3d	ET= PointCloudRegister.ET;//平移矩阵T
		for (int i=0;i<3;i++)
		{
			for (int j=0;j<3;j++)
			{
				temp.Format(_T("%.3f"), ER(i, j));
				s = s + temp+"\t";

			}
			temp.Format(_T("%.3f"), ET[i]);
			s = s + temp+"\n\n";
		}
		s = s + "0\t0\t0\t1";
		m_wndleftForm.setRichText(s);
	}
	m_OGLView.dmetchPoints=PointCloudRegister.dmetchPoints;
	m_OGLView.drawScene();
	//AfxMessageBox(str);

}


void CMainFrame::OnbtnIcpPointtoplane()
{
	// TODO: 在此添加命令处理程序代码
	if (m_OGLView.NumofObj < 2)
	{
		AfxMessageBox("缺少模型！请加载至少两个模型");
		return;
	}
	Registration PointCloudRegister(&m_OGLView.m_Model, &m_OGLView.m_GModel);
	if (!PointCloudRegister.ICP_PointToPlaneRegister())
	{

		AfxMessageBox("配准失败！");
	}
	m_OGLView.drawScene();
	//m_OGLView.ICP_PointToPlaneRegister();
	//m_OGLView.m_Model;
}


void CMainFrame::OnbtnGuasscurvature()
{
	// TODO: 在此添加命令处理程序代码
	if (m_OGLView.NumofObj < 1)
	{
		AfxMessageBox("缺少模型！请加载至少一个模型");
		return;
	}
	time_t startT = clock();
	Registration PointCloudRegister;
	if (m_OGLView.NumofObj == 1) {
		PointCloudRegister.GuassCurvature(&m_OGLView.m_Model);
		m_OGLView.m_Model.TextureStatu = 30;
	}
	else if (m_OGLView.NumofObj == 2) {
		PointCloudRegister.GuassCurvature(&m_OGLView.m_Model);
		PointCloudRegister.GuassCurvature(&m_OGLView.m_GModel);
		m_OGLView.m_Model.TextureStatu = 30;
		m_OGLView.m_GModel.TextureStatu = 30;
	}
	time_t endT = clock();

	m_OGLView.drawScene();
	double runtim = (double)(endT - startT) / CLOCKS_PER_SEC; char str[256];
	sprintf(str, "%lf s", runtim);
	AfxMessageBox(str);
}

void CMainFrame::OnbtnIcpnonrigid()
{
	// TODO: 在此添加命令处理程序代码
	// TODO: 在此添加命令处理程序代码
	if (m_OGLView.NumofObj < 2)
	{
		AfxMessageBox("缺少模型！请加载至少两个模型");
		return;
	}
	Registration PointCloudRegister(&m_OGLView.m_Model, &m_OGLView.m_GModel);

	time_t startT=clock();
	time_t endT;
	if (!PointCloudRegister.ICP_Point_nonrigid())
	{

		AfxMessageBox("配准失败！");
	}

	else
	{
		endT = clock();
		double runtim = (double)(endT - startT) / CLOCKS_PER_SEC;
		//char str[256];
		//sprintf(str, "%lf", runtim);
		//给richedit控件传递参数
		CString temp;
		temp.Format(_T("%.3f"), runtim);
		CString s = "\n非刚性耗时： " + temp + " s\n";

		m_wndleftForm.addRichText(s);
	}
	
	//double runtim = (double)(endT - startT) / CLOCKS_PER_SEC; char str[256];
	//sprintf(str, "%lf", runtim);
	//AfxMessageBox(str);
	m_OGLView.drawScene();
}


void CMainFrame::OnbtnIcperror()
{
	// TODO: 在此添加命令处理程序代码
	if (m_OGLView.NumofObj < 2)
	{
		AfxMessageBox("缺少模型！请加载至少两个模型");
		return;
	}
	Registration PointCloudRegister(&m_OGLView.m_Model, &m_OGLView.m_GModel);
	double errorD = PointCloudRegister.errorDistP2P();
	char str[256];
	sprintf(str, "%12.7f", errorD);
	m_OGLView.drawScene();
	AfxMessageBox(str);
	m_OGLView.drawScene();

}


void CMainFrame::OnTpsNonrigid()
{
	// TODO: 在此添加命令处理程序代码
	// TODO: 在此添加命令处理程序代码
	// TODO: 在此添加命令处理程序代码
	if (m_OGLView.NumofObj < 2)
	{
		AfxMessageBox("缺少模型！请加载至少两个模型");
		return;
	}
	Registration PointCloudRegister(&m_OGLView.m_Model, &m_OGLView.m_GModel);

	time_t startT = clock();

	if (!PointCloudRegister.TPS_nonrigid())
	{

		AfxMessageBox("配准失败！");
	}
	time_t endT = clock();
	double runtim = (double)(endT - startT) / CLOCKS_PER_SEC; char str[256];
	sprintf(str, "%lf", runtim);
	AfxMessageBox(str);
	m_OGLView.drawScene();
}
