#pragma once


// leftForm 对话框

class leftForm : public CDialogBar
{
	DECLARE_DYNAMIC(leftForm)

public:
	leftForm(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~leftForm();
public:
	static int openGL_view_model;
	static int openGL_show_model;//11:全部显示  10：显示目标模型 01：显示源模型 00：不显示
	static bool clearModel;
	static bool showCurvature;
	double richtext[16] = {0.0};
// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_FORMVIEW_left };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButtonpointview();
	afx_msg void OnUpdateButton(CCmdUI *pCmdUI);
	afx_msg void OnBnClickedButtontextureview();
	afx_msg void OnBnClickedButtonmeshview();
public:
	CRichEditCtrl m_rigist_matrix;
public:
	void setRichText(CString text, UINT id = 0);
	void addRichText(CString text, UINT id = 0);
	afx_msg void OnBnClickedCheckmodel();
protected:
	afx_msg LRESULT OnInitDialog(WPARAM wParam, LPARAM lParam);
public:
	afx_msg void OnBnClickedButtonclearmodel();
	afx_msg void OnBnClickedCheckshowcurvature();
};
