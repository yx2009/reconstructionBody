
// MainFrm.h : CMainFrame 类的接口
//

#pragma once

#include "Skeleton.h"
#include "OGLView.h"
#include "leftForm.h"
class CMainFrame : public CFrameWnd
{
	
protected: // 仅从序列化创建
	//CMainFrame();
	DECLARE_DYNCREATE(CMainFrame)

// 特性
// Attributes
public:
	CMainFrame();
	CString m_ClassName;
	HCURSOR m_HArrow;
	COGLView m_OGLView;
	BOOL m_Wireframe;
	//////////////////////////////////add gyy 
	double Xmin, Ymin, Xymin;
	int Xcurvertexcnt, Ycurvertexcnt, XcurfaceCnt, YcurfaceCnt;
	double xpoint[10000];
	double zpoint[10000];
	double tempxpoint;
	int countX;
	double szmin, szmax;


// 操作
public:

// 重写
public:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	/*virtual BOOL PreTranslateMessage(MSG* pMsg);*/

// 实现
public:
	virtual ~CMainFrame();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:  // 控件条嵌入成员
	CToolBar          m_wndToolBar;
	CStatusBar        m_wndStatusBar;
	leftForm		  m_wndleftForm;

// 生成的消息映射函数
protected:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnPaint();
	afx_msg void OnWhichogl();
	DECLARE_MESSAGE_MAP()


public:
	afx_msg void OnFileOpen();
	afx_msg void OnFileObjimport();
	afx_msg void Onbtnruderegister();
	afx_msg void OnbtnIcpPointtoplane();
	afx_msg void OnbtnGuasscurvature();
	afx_msg void OnbtnIcpnonrigid();
	afx_msg void OnbtnIcperror();
	afx_msg void OnTpsNonrigid();
};


