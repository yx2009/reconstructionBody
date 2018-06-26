// leftForm.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "3DBodyReconstruction.h"
#include "leftForm.h"
#include "afxdialogex.h"
#include "resource.h"

// leftForm �Ի���
using namespace std;
IMPLEMENT_DYNAMIC(leftForm, CDialogBar)
int leftForm::openGL_view_model = 0; 
int leftForm::openGL_show_model = 11;
bool leftForm::clearModel = false;
bool leftForm::showCurvature = false;
leftForm::leftForm(CWnd* pParent /*=NULL*/)
	: CDialogBar()
{

}

leftForm::~leftForm()
{
}

void leftForm::DoDataExchange(CDataExchange* pDX)
{
	DDX_Control(pDX, IDC_RICHEDIT2_matrix, m_rigist_matrix);
	CDialogBar::DoDataExchange(pDX);

}


BEGIN_MESSAGE_MAP(leftForm, CDialogBar)
	ON_UPDATE_COMMAND_UI(IDC_BUTTON_point_view, OnUpdateButton)
	ON_UPDATE_COMMAND_UI(IDC_BUTTON_texture_view, OnUpdateButton)
	ON_UPDATE_COMMAND_UI(IDC_BUTTON_mesh_view, OnUpdateButton)
	ON_UPDATE_COMMAND_UI(IDC_BUTTON_clearModel, OnUpdateButton)
	ON_BN_CLICKED(IDC_BUTTON_point_view, &leftForm::OnBnClickedButtonpointview)
	ON_BN_CLICKED(IDC_BUTTON_texture_view, &leftForm::OnBnClickedButtontextureview)
	ON_BN_CLICKED(IDC_BUTTON_mesh_view, &leftForm::OnBnClickedButtonmeshview)
	ON_BN_CLICKED(IDC_CHECK_targetModel, &leftForm::OnBnClickedCheckmodel)
	ON_BN_CLICKED(IDC_CHECK_sourceModel, &leftForm::OnBnClickedCheckmodel)
	ON_MESSAGE(WM_INITDIALOG, &leftForm::OnInitDialog)
	ON_BN_CLICKED(IDC_BUTTON_clearModel, &leftForm::OnBnClickedButtonclearmodel)
	
	ON_BN_CLICKED(IDC_CHECK_show_curvature, &leftForm::OnBnClickedCheckshowcurvature)
END_MESSAGE_MAP()


// leftForm ��Ϣ�������

void leftForm::OnUpdateButton(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(TRUE);
}

void leftForm::setRichText(CString text, UINT id) {
	CWnd *pRichEdit = GetDlgItem(IDC_RICHEDIT2_matrix);
	pRichEdit->SetWindowText("");
	m_rigist_matrix.Clear();
	this->RedrawWindow();

	if (NULL != pRichEdit)
	{

		DWORD dwFontStyle = 0;
		dwFontStyle = pRichEdit->SendMessage(EM_GETLANGOPTIONS, NULL, NULL);
		if (dwFontStyle & IMF_AUTOFONT)
		{
			dwFontStyle &= ~IMF_AUTOFONT;
			pRichEdit->SendMessage(EM_SETLANGOPTIONS, NULL, (LPARAM)dwFontStyle);
		}
		pRichEdit->SetWindowText(text);
		//pRichEdit->SetWindowText(text);
	}
}
void leftForm::addRichText(CString text, UINT id) {
	
	CWnd *pRichEdit = GetDlgItem(IDC_RICHEDIT2_matrix);
	CString old_text;
	GetDlgItemTextA(IDC_RICHEDIT2_matrix, old_text);
	text = old_text + text;
	pRichEdit->SetWindowText("");
	m_rigist_matrix.Clear();
	this->RedrawWindow();

	if (NULL != pRichEdit)
	{

		DWORD dwFontStyle = 0;
		dwFontStyle = pRichEdit->SendMessage(EM_GETLANGOPTIONS, NULL, NULL);
		if (dwFontStyle & IMF_AUTOFONT)
		{
			dwFontStyle &= ~IMF_AUTOFONT;
			pRichEdit->SendMessage(EM_SETLANGOPTIONS, NULL, (LPARAM)dwFontStyle);
		}
		pRichEdit->SetWindowText(text);
		//pRichEdit->SetWindowText(text);
	}
}
void leftForm::OnBnClickedButtonpointview()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	openGL_view_model = 0;
}

void leftForm::OnBnClickedButtonmeshview()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	openGL_view_model = 1;
}

void leftForm::OnBnClickedButtontextureview()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	openGL_view_model = 2;
}



void leftForm::OnBnClickedCheckmodel()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	openGL_show_model = 
		((CButton*)GetDlgItem(IDC_CHECK_targetModel))->GetCheck()*10
		+((CButton*)GetDlgItem(IDC_CHECK_sourceModel))->GetCheck();

	
}



void leftForm::OnBnClickedCheckshowcurvature()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	showCurvature=((CButton*)GetDlgItem(IDC_CHECK_show_curvature))->GetCheck();
	//showCurvature = true;
}

afx_msg LRESULT leftForm::OnInitDialog(WPARAM wParam, LPARAM lParam)
{
	((CButton*)GetDlgItem(IDC_CHECK_targetModel))->SetCheck(TRUE);
	((CButton*)GetDlgItem(IDC_CHECK_sourceModel))->SetCheck(TRUE);
	//((CButton*)GetDlgItem(IDC_BUTTON1))->EnableWindow(0);
	return 0;
}


void leftForm::OnBnClickedButtonclearmodel()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	if (MessageBox("ȷʵҪ�����", "��ʾ", MB_OKCANCEL | MB_ICONWARNING) == IDOK)
	{

		clearModel = true;
		CWnd *pRichEdit = GetDlgItem(IDC_RICHEDIT2_matrix);
		pRichEdit->SetWindowText("");
		//m_rigist_matrix.SetWindowText("");
		m_rigist_matrix.Clear();
		this->RedrawWindow();
	}
}


