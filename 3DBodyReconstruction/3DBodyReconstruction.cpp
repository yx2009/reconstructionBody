
// 3DBodyReconstruction.cpp : 定义应用程序的类行为。
//

#include "stdafx.h"
#include "afxwinappex.h"
#include "afxdialogex.h"
#include "3DBodyReconstruction.h"
#include "MainFrm.h"

#include "3DBodyReconstructionDoc.h"
#include "3DBodyReconstructionView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CMy3DBodyReconstructionApp

BEGIN_MESSAGE_MAP(CMy3DBodyReconstructionApp, CWinApp)
	ON_COMMAND(ID_APP_ABOUT, &CMy3DBodyReconstructionApp::OnAppAbout)
	// 基于文件的标准文档命令
	ON_COMMAND(ID_FILE_NEW, &CWinApp::OnFileNew)
	ON_COMMAND(ID_FILE_OPEN, &CWinApp::OnFileOpen)
	// 标准打印设置命令
	ON_COMMAND(ID_FILE_PRINT_SETUP, &CWinApp::OnFilePrintSetup)
END_MESSAGE_MAP()


// CMy3DBodyReconstructionApp 构造

CMy3DBodyReconstructionApp::CMy3DBodyReconstructionApp()
{
	// 支持重新启动管理器
	m_dwRestartManagerSupportFlags = AFX_RESTART_MANAGER_SUPPORT_ALL_ASPECTS;
#ifdef _MANAGED
	// 如果应用程序是利用公共语言运行时支持(/clr)构建的，则: 
	//     1) 必须有此附加设置，“重新启动管理器”支持才能正常工作。
	//     2) 在您的项目中，您必须按照生成顺序向 System.Windows.Forms 添加引用。
	System::Windows::Forms::Application::SetUnhandledExceptionMode(System::Windows::Forms::UnhandledExceptionMode::ThrowException);
#endif

	// TODO: 将以下应用程序 ID 字符串替换为唯一的 ID 字符串；建议的字符串格式
	//为 CompanyName.ProductName.SubProduct.VersionInformation
	SetAppID(_T("3DBodyReconstruction.AppID.NoVersion"));

	// TODO: 在此处添加构造代码，
	// 将所有重要的初始化放置在 InitInstance 中
}

// 唯一的一个 CMy3DBodyReconstructionApp 对象

CMy3DBodyReconstructionApp theApp;


// CMy3DBodyReconstructionApp 初始化

BOOL CMy3DBodyReconstructionApp::InitInstance()
{
	// 如果一个运行在 Windows XP 上的应用程序清单指定要
	// 使用 ComCtl32.dll 版本 6 或更高版本来启用可视化方式，
	//则需要 InitCommonControlsEx()。  否则，将无法创建窗口。
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// 将它设置为包括所有要在应用程序中使用的
	// 公共控件类。
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinApp::InitInstance();


	// 初始化 OLE 库
	if (!AfxOleInit())
	{
		AfxMessageBox(IDP_OLE_INIT_FAILED);
		return FALSE;
	}

	AfxEnableControlContainer();

	EnableTaskbarInteraction(FALSE);

	// 使用 RichEdit 控件需要 AfxInitRichEdit2()	
	AfxInitRichEdit2();

	// 标准初始化
	// 如果未使用这些功能并希望减小
	// 最终可执行文件的大小，则应移除下列
	// 不需要的特定初始化例程
	// 更改用于存储设置的注册表项
	// TODO: 应适当修改该字符串，
	// 例如修改为公司或组织名
	SetRegistryKey(_T("3DBody"));

	LoadStdProfileSettings();  // Load standard INI file options (including MRU)

							   // Register the application's document templates.  Document templates
							   //  serve as the connection between documents, frame windows and views.

	CMainFrame* pFrame = new CMainFrame;

	if (!pFrame->LoadFrame(IDR_MAINFRAME,
		WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN | WS_VISIBLE)) {
		return FALSE;
	}

	m_pMainWnd = pFrame;

	// The one and only window has been initialized, so show and update it.
	//	m_pMainWnd->ShowWindow(SW_SHOW);
	//	m_pMainWnd->UpdateWindow();


	// Enable drag/drop open
	m_pMainWnd->DragAcceptFiles();

	return TRUE;
}

int CMy3DBodyReconstructionApp::ExitInstance()
{
	//TODO: 处理可能已添加的附加资源
	AfxOleTerm(FALSE);

	return CWinApp::ExitInstance();
}

// CMy3DBodyReconstructionApp 消息处理程序


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
	afx_msg LRESULT OnInitDialog(WPARAM wParam, LPARAM lParam);
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
	ON_MESSAGE(WM_INITDIALOG, &CAboutDlg::OnInitDialog)
END_MESSAGE_MAP()

// 用于运行对话框的应用程序命令
void CMy3DBodyReconstructionApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}

// CMy3DBodyReconstructionApp 消息处理程序





afx_msg LRESULT CAboutDlg::OnInitDialog(WPARAM wParam, LPARAM lParam)
{
	return 0;
}
