
// 3DBodyReconstruction.h : 3DBodyReconstruction Ӧ�ó������ͷ�ļ�
//
#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"       // ������


// CMy3DBodyReconstructionApp:
// �йش����ʵ�֣������ 3DBodyReconstruction.cpp
//

class CMy3DBodyReconstructionApp : public CWinApp
{
public:
	CMy3DBodyReconstructionApp();


// ��д
public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();

// ʵ��
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
};

extern CMy3DBodyReconstructionApp theApp;
