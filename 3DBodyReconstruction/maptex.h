#pragma once
#include <gl/GL.h>
//BITMAPINFO����չ
struct EXT_BITMAPINFO
{
	BITMAPINFOHEADER    bmiHeader;
	RGBQUAD             bmiColors[256];//��ɫ�����Ϊ256
};

class maptex
{
public:
	maptex(void) {}
public:
	~maptex(void) {}

public:
	BOOL PlayBmp(CDC *pDC, const CPoint &StartPoint);
	int GetColorNum(const BITMAPINFOHEADER &BmpInf);
	BOOL LoadBmpFile(const CString &sBMPFileName);

	//	virtual ~Liubowen();

public:
	BOOL m_bReady;  //�����Ƿ��Ѵ��ļ���ȡ
	CString m_sBMPFileFullName;  //����·����
	BITMAPFILEHEADER m_BmpFileHeader;
	EXT_BITMAPINFO m_BmpInfo;//��� BITMAPINFOHEADER�͵�ɫ��(�����ɫ)
	HGLOBAL m_hgImageData; //ͼ�����ݾ��
	HGLOBAL m_hgImageData1;
public:
	long		JPGWidth;				// JPGͼ��Ŀ��(������Ϊ��λ) 
	long		JPGHeight;				// JPGͼ��ĸߴ�(������Ϊ��λ)
	BYTE *pImageData;                   //����jpg������Ϣ��
	long nBytePerLine;                  // jpgÿ��������
	BOOL BuildTexture(char *szPathName, GLuint &texid, HWND &m_hwind); 
	BOOL BuildTextureforAll(char *szPathName, GLuint &texid, HWND &m_hwind); 
};


#if !defined(AFX_BMPEDIT_H__CAD01C85_3CBA_4D33_AFDF_14A746B4BD41__INCLUDED_)
#define AFX_BMPEDIT_H__CAD01C85_3CBA_4D33_AFDF_14A746B4BD41__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_BMPEDIT_H__CAD01C85_3CBA_4D33_AFDF_14A746B4BD41__INCLUDED_)
