#pragma once


// leftForm �Ի���

class leftForm : public CDialogBar
{
	DECLARE_DYNAMIC(leftForm)

public:
	leftForm(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~leftForm();
public:
	static int openGL_view_model;
	static int openGL_show_model;//11:ȫ����ʾ  10����ʾĿ��ģ�� 01����ʾԴģ�� 00������ʾ
	static bool clearModel;
	static bool showCurvature;
	double richtext[16] = {0.0};
// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_FORMVIEW_left };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

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
