#pragma once
#include <gl/GL.h>
//BITMAPINFO的扩展
struct EXT_BITMAPINFO
{
	BITMAPINFOHEADER    bmiHeader;
	RGBQUAD             bmiColors[256];//调色板最大为256
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
	BOOL m_bReady;  //数据是否已从文件读取
	CString m_sBMPFileFullName;  //完整路径名
	BITMAPFILEHEADER m_BmpFileHeader;
	EXT_BITMAPINFO m_BmpInfo;//存放 BITMAPINFOHEADER和调色板(非真彩色)
	HGLOBAL m_hgImageData; //图像数据句柄
	HGLOBAL m_hgImageData1;
public:
	long		JPGWidth;				// JPG图像的宽带(以像素为单位) 
	long		JPGHeight;				// JPG图像的高带(以像素为单位)
	BYTE *pImageData;                   //储存jpg像素信息用
	long nBytePerLine;                  // jpg每行数据量
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
