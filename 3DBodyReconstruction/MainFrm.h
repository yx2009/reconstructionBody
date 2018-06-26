
// MainFrm.h : CMainFrame ��Ľӿ�
//

#pragma once

#include "Skeleton.h"
#include "OGLView.h"
#include "leftForm.h"
class CMainFrame : public CFrameWnd
{
	
protected: // �������л�����
	//CMainFrame();
	DECLARE_DYNCREATE(CMainFrame)

// ����
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


// ����
public:

// ��д
public:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	/*virtual BOOL PreTranslateMessage(MSG* pMsg);*/

// ʵ��
public:
	virtual ~CMainFrame();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:  // �ؼ���Ƕ���Ա
	CToolBar          m_wndToolBar;
	CStatusBar        m_wndStatusBar;
	leftForm		  m_wndleftForm;

// ���ɵ���Ϣӳ�亯��
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


