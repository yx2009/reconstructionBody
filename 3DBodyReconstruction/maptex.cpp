#include "StdAfx.h"
#include "maptex.h"
#include <string.h>
#include <math.h>
#include <GL/gl.h>
#include "OGLView.h"  //201654



BOOL maptex::LoadBmpFile(const CString &sBmpFileName)
{
	m_bReady = FALSE;
	//释放之前读取的数据///////
	//释放图像数据
	if (m_hgImageData)
		GlobalFree(m_hgImageData);

	m_sBMPFileFullName = sBmpFileName;
	if (m_sBMPFileFullName.IsEmpty())
		return FALSE;
	CFile BmpFile;
	if (!BmpFile.Open(m_sBMPFileFullName, CFile::modeRead))
	{
		AfxMessageBox("文件打开时出错!");
		return FALSE;
	}

	//读取文件头  格式检查
	if (BmpFile.Read(&m_BmpFileHeader, sizeof(BITMAPFILEHEADER)) < sizeof(BITMAPFILEHEADER))
	{
		AfxMessageBox("文件数据已损坏!");
		BmpFile.Close();
		return FALSE;
	}
	if (m_BmpFileHeader.bfType != 0x4D42)
	{
		AfxMessageBox("非位图文件");
		BmpFile.Close();
		return FALSE;
	}

	//读取信息头
	if (BmpFile.Read(&m_BmpInfo.bmiHeader, sizeof(BITMAPINFOHEADER)) < sizeof(BITMAPINFOHEADER))
	{
		AfxMessageBox("文件数据已损坏!");
		BmpFile.Close();
		return FALSE;
	}
	//SetDIBitsToDevice()和StretchDIBits()会自行处理调色板，不需要再向当前DC中选入调色板了！！！

	//读取调色板
	if (m_BmpInfo.bmiHeader.biBitCount < 24)
		BmpFile.Read(&m_BmpInfo.bmiColors, GetColorNum(m_BmpInfo.bmiHeader) * sizeof(RGBQUAD));

	//读取图像数据
	long nBytePerLine = (m_BmpInfo.bmiHeader.biWidth*m_BmpInfo.bmiHeader.biBitCount + 31) / 32 * 4;//图像每行所占实际字节数（须是4的倍数）
	m_hgImageData = GlobalAlloc(GHND, nBytePerLine * m_BmpInfo.bmiHeader.biHeight);

	if (m_hgImageData == NULL)
	{
		AfxMessageBox("内存不足");
		BmpFile.Close();
		return FALSE;
	}
	byte *pImageData = (byte *)GlobalLock(m_hgImageData);
	BmpFile.Read(pImageData, nBytePerLine * m_BmpInfo.bmiHeader.biHeight);

	GlobalUnlock(m_hgImageData);

	BmpFile.Close();
	m_bReady = TRUE;

	//以下程序完成bmp图像读完后 立刻生成副本bmp图像  by liubowen
	char *pFileName = "myfile.bmp";
	CFile file(pFileName, CFile::modeCreate | CFile::modeWrite);
	if (m_hgImageData == NULL)
		return 0;

	try
	{
		//写入文件头
		/*
		m_BmpFileHeader.bfType = 0x4d42;
		m_BmpFileHeader.bfSize = sizeof(BITMAPFILEHEADER);
		m_BmpFileHeader.bfReserved1 = 0;
		m_BmpFileHeader.bfReserved2 = 0;
		m_BmpFileHeader.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
		*/
		file.Write(&m_BmpFileHeader, sizeof(BITMAPFILEHEADER));

		//写入信息头
		file.Write(&m_BmpInfo.bmiHeader, sizeof(BITMAPINFOHEADER));

		//写入调色板
		if (m_BmpInfo.bmiHeader.biBitCount < 24)
		{
			file.Write(&m_BmpInfo.bmiColors, GetColorNum(m_BmpInfo.bmiHeader) * sizeof(RGBQUAD));
		}

		//add for 遍历缓冲区每个像素点的值 并进行RGB值的修改
		/*
		int k=0;
		for (int i=0;i<m_BmpInfo.bmiHeader.biHeight;i++)
		{
		for (int j=0;j<nBytePerLine;j++)
		{
		*(pImageData+j+i*nBytePerLine)=k++;
		}
		}
		*/

		//写入图像数据
		file.Write(pImageData, nBytePerLine * m_BmpInfo.bmiHeader.biHeight);

		//清除尚未写入的数据并关闭文件
		file.Flush();
		file.Close();
	}

	//抛出异常处理
	catch (CFileException *e)
	{
		CString str;
		str.Format("保存数据失败的原因是:%d", e->m_cause);
		MessageBox(NULL, "str", "发生错误", MB_OK);
		file.Abort();
		e->Delete();
	}
	//生成bmp副本结束

	return TRUE;
}

int maptex::GetColorNum(const BITMAPINFOHEADER &BmpInf)
{
	if (BmpInf.biClrUsed >0)
		return BmpInf.biClrUsed;
	else
		return 1 << BmpInf.biBitCount;
}

BOOL maptex::BuildTexture(char *szPathName, GLuint &texid, HWND &m_hwind) //lbw 2016426
{
	HDC			hdcTemp;												// DC用来保存位图
	HBITMAP		hbmpTemp;												// 保存临时位图
	IPicture	*pPicture;												// 定义IPicture Interface
	OLECHAR		wszPath[MAX_PATH + 1];									// 图片的完全路径
	char		szPath[MAX_PATH + 1];										// 图片的完全路径
	long		lWidth;													// 图像宽度
	long		lHeight;												// 图像高度
																		//long		JPGWidth;	//改为全局										// 图像的宽带(以像素为单位)
																		//long		JPGHeight;											// 图像的高带(以像素为单位)
	GLint		glMaxTexDim;											// 保存纹理的最大尺寸

	//if (strstr(szPathName, "http://"))									// 如果路径包含 http:// 则...
	//{
	//	strcpy(szPath, szPathName);										// 把路径拷贝到 szPath
	//}
	//else																// 否则从文件导入图片
	//{
	//	GetCurrentDirectory(MAX_PATH, szPath);							// 取得当前路径
	//	strcat(szPath, "\\");											// 添加字符"\"
	//	strcat(szPath, szPathName);										// 添加图片的相对路径
	//}

	strcpy(szPath, szPathName);
	MultiByteToWideChar(CP_ACP, 0, szPath, -1, wszPath, MAX_PATH);		// 把ASCII码转化为Unicode标准码
	HRESULT hr = OleLoadPicturePath(wszPath, 0, 0, 0, IID_IPicture, (void**)&pPicture);

	if (FAILED(hr))														// 如果导入失败
	{
		// 图片载入失败出错信息
		MessageBox(HWND_DESKTOP, "图片导入失败1!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// 返回 FALSE
	}

	//hdcTemp = GetDC(m_hwind)/*CreateCompatibleDC(m_hDC)*/;

	hdcTemp = CreateCompatibleDC(GetDC(0));				// 建立窗口设备描述表
	if (!hdcTemp)										// 建立失败?
	{
		pPicture->Release();											// 释放IPicture
																		// 图片载入失败出错信息
		MessageBox(HWND_DESKTOP, "图片导入失败2!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// 返回 FALSE
	}

	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &glMaxTexDim);					// 取得支持的纹理最大尺寸

	pPicture->get_Width(&lWidth);										// 取得IPicture 宽度 (转换为Pixels格式)
	JPGWidth = MulDiv(lWidth, GetDeviceCaps(hdcTemp, LOGPIXELSX), 2540);
	pPicture->get_Height(&lHeight);										// 取得IPicture 高度 (转换为Pixels格式)
	JPGHeight = MulDiv(lHeight, GetDeviceCaps(hdcTemp, LOGPIXELSY), 2540);

	// 调整图片到最好的效果
	if (JPGWidth <= glMaxTexDim)									// 图片宽度是否超过显卡最大支持尺寸
		JPGWidth = 1 << (int)floor((log((double)JPGWidth) / log(2.0f)) + 0.5f);
	else																// 否则,将图片宽度设为显卡最大支持尺寸
		JPGWidth = glMaxTexDim;

	if (JPGHeight <= glMaxTexDim)									// 图片高度是否超过显卡最大支持尺寸
		JPGHeight = 1 << (int)floor((log((double)JPGHeight) / log(2.0f)) + 0.5f);
	else 																// 否则,将图片高度设为显卡最大支持尺寸
		JPGHeight = glMaxTexDim;

	// 建立一个临时位图
	BITMAPINFO	bi = { 0 };												// 位图的类型
	DWORD		*pBits = 0;												// 指向位图Bits的指针

	bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);				// 设置结构大小
	bi.bmiHeader.biBitCount = 32;									// 32 位
	bi.bmiHeader.biWidth = JPGWidth;							// 宽度像素值
	bi.bmiHeader.biHeight = JPGHeight;						// 高度像素值
	bi.bmiHeader.biCompression = BI_RGB;								// RGB 格式
	bi.bmiHeader.biPlanes = 1;									// 一个位平面

																// 建立一个位图这样我们可以指定颜色和深度 并访问每位的值
	hbmpTemp = CreateDIBSection(hdcTemp, &bi, DIB_RGB_COLORS, (void**)&pBits, 0, 0);

	if (!hbmpTemp)														// 建立失败?
	{
		DeleteDC(hdcTemp);												// 删除设备描述表
		pPicture->Release();											// 释放IPicture
																		// 图片载入失败出错信息
		MessageBox(HWND_DESKTOP, "图片导入失败3!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// 返回 FALSE
	}

	SelectObject(hdcTemp, hbmpTemp);									// 选择临时DC句柄和临时位图对象

																		// 在位图上绘制IPicture
	pPicture->Render(hdcTemp, 0, 0, JPGWidth, JPGHeight, 0, lHeight, lWidth, -lHeight, 0);


	pImageData = new BYTE[JPGHeight*JPGWidth * 4];  //为储存像素信息开辟内存
	nBytePerLine = (JPGWidth * 32 / 8 + 3) / 4 * 4;

	// 将BGR转换为RGB　将ALPHA值设为255
	for (long i = 0; i < JPGWidth * JPGHeight; i++)				// 循环遍历所有的像素
	{
		BYTE* pPixel = (BYTE*)(&pBits[i]);							// 获取当前像素
		BYTE  temp = pPixel[0];									// 临时存储第一个颜色像素(蓝色)
		pPixel[0] = pPixel[2];									// 将红色值存到第一位
		pPixel[2] = temp;											// 将蓝色值存到第三位
		pPixel[3] = 255;											// ALPHA值设为255

		pImageData[4 * i + 0] = pPixel[2];                 //2016426  lbw  B
		pImageData[4 * i + 1] = pPixel[1];                  //G
		pImageData[4 * i + 2] = pPixel[0];                  //R
		pImageData[4 * i + 3] = pPixel[3];                  //A
	}

	glGenTextures(1, &texid);											// 创建纹理

																		// 使用来自位图数据生成 的典型纹理
	glBindTexture(GL_TEXTURE_2D, texid);								// 绑定纹理
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// 线形滤波
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);   // 线形滤波

																		// 生成纹理
	glTexImage2D(GL_TEXTURE_2D, 0, 3, JPGWidth, JPGHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, pBits);

	DeleteObject(hbmpTemp);												// 删除对象
	DeleteDC(hdcTemp);													// 删除设备描述表

	pPicture->Release();												// 释放 IPicture

	return TRUE;														// 返回 TRUE
}

BOOL maptex::BuildTextureforAll(char *szPathName, GLuint &texid, HWND &m_hwind) //lbw 2016426
{
	HDC			hdcTemp;												// DC用来保存位图
	HBITMAP		hbmpTemp;												// 保存临时位图
	IPicture	*pPicture;												// 定义IPicture Interface
	OLECHAR		wszPath[MAX_PATH + 1];									// 图片的完全路径
	char		szPath[MAX_PATH + 1];										// 图片的完全路径
	long		lWidth;													// 图像宽度
	long		lHeight;												// 图像高度
																		//long		JPGWidth;	//改为全局										// 图像的宽带(以像素为单位)
																		//long		JPGHeight;											// 图像的高带(以像素为单位)
	GLint		glMaxTexDim;											// 保存纹理的最大尺寸

	if (strstr(szPathName, "http://"))									// 如果路径包含 http:// 则...
	{
		strcpy(szPath, szPathName);										// 把路径拷贝到 szPath
	}
	else																// 否则从文件导入图片
	{
		GetCurrentDirectory(MAX_PATH, szPath);							// 取得当前路径
		strcat(szPath, "\\");											// 添加字符"\"
		strcat(szPath, szPathName);										// 添加图片的相对路径
	}

	MultiByteToWideChar(CP_ACP, 0, szPath, -1, wszPath, MAX_PATH);		// 把ASCII码转化为Unicode标准码
	HRESULT hr = OleLoadPicturePath(wszPath, 0, 0, 0, IID_IPicture, (void**)&pPicture);

	if (FAILED(hr))														// 如果导入失败
	{
		// 图片载入失败出错信息
		MessageBox(HWND_DESKTOP, "图片导入失败1!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// 返回 FALSE
	}

	//hdcTemp = GetDC(m_hwind)/*CreateCompatibleDC(m_hDC)*/;

	hdcTemp = CreateCompatibleDC(GetDC(0));				// 建立窗口设备描述表
	if (!hdcTemp)										// 建立失败?
	{
		pPicture->Release();											// 释放IPicture
																		// 图片载入失败出错信息
		MessageBox(HWND_DESKTOP, "图片导入失败2!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// 返回 FALSE
	}

	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &glMaxTexDim);					// 取得支持的纹理最大尺寸

	pPicture->get_Width(&lWidth);										// 取得IPicture 宽度 (转换为Pixels格式)
	JPGWidth = MulDiv(lWidth, GetDeviceCaps(hdcTemp, LOGPIXELSX), 2540);
	pPicture->get_Height(&lHeight);										// 取得IPicture 高度 (转换为Pixels格式)
	JPGHeight = MulDiv(lHeight, GetDeviceCaps(hdcTemp, LOGPIXELSY), 2540);

	// 调整图片到最好的效果
	if (JPGWidth <= glMaxTexDim)									// 图片宽度是否超过显卡最大支持尺寸
		JPGWidth = 1 << (int)floor((log((double)JPGWidth) / log(2.0f)) + 0.5f);
	else																// 否则,将图片宽度设为显卡最大支持尺寸
		JPGWidth = glMaxTexDim;

	if (JPGHeight <= glMaxTexDim)									// 图片高度是否超过显卡最大支持尺寸
		JPGHeight = 1 << (int)floor((log((double)JPGHeight) / log(2.0f)) + 0.5f);
	else 																// 否则,将图片高度设为显卡最大支持尺寸
		JPGHeight = glMaxTexDim;

	// 建立一个临时位图
	BITMAPINFO	bi = { 0 };												// 位图的类型
	DWORD		*pBits = 0;												// 指向位图Bits的指针

	bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);				// 设置结构大小
	bi.bmiHeader.biBitCount = 32;									// 32 位
	bi.bmiHeader.biWidth = JPGWidth;							// 宽度像素值
	bi.bmiHeader.biHeight = JPGHeight;						// 高度像素值
	bi.bmiHeader.biCompression = BI_RGB;								// RGB 格式
	bi.bmiHeader.biPlanes = 1;									// 一个位平面

																// 建立一个位图这样我们可以指定颜色和深度 并访问每位的值
	hbmpTemp = CreateDIBSection(hdcTemp, &bi, DIB_RGB_COLORS, (void**)&pBits, 0, 0);

	if (!hbmpTemp)														// 建立失败?
	{
		DeleteDC(hdcTemp);												// 删除设备描述表
		pPicture->Release();											// 释放IPicture
																		// 图片载入失败出错信息
		MessageBox(HWND_DESKTOP, "图片导入失败3!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// 返回 FALSE
	}

	SelectObject(hdcTemp, hbmpTemp);									// 选择临时DC句柄和临时位图对象

																		// 在位图上绘制IPicture
	pPicture->Render(hdcTemp, 0, 0, JPGWidth, JPGHeight, 0, lHeight, lWidth, -lHeight, 0);


	pImageData = new BYTE[JPGHeight*JPGWidth];  //为储存像素信息开辟内存
	nBytePerLine = (JPGWidth * 32 / 8 + 3) / 4 * 4;

	// 将BGR转换为RGB　将ALPHA值设为255
	for (long i = 0; i < JPGWidth * JPGHeight; i++)				// 循环遍历所有的像素
	{
		BYTE* pPixel = (BYTE*)(&pBits[i]);							// 获取当前像素
		BYTE  temp = pPixel[0];									// 临时存储第一个颜色像素(蓝色)
		pPixel[0] = pPixel[2];									// 将红色值存到第一位
		pPixel[2] = temp;											// 将蓝色值存到第三位
		pPixel[3] = 255;											// ALPHA值设为255

		pImageData[i] = pPixel[2];                 //2016426  lbw  B
	}

	glGenTextures(1, &texid);											// 创建纹理

																		// 使用来自位图数据生成 的典型纹理
	glBindTexture(GL_TEXTURE_2D, texid);								// 绑定纹理
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// 线形滤波
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);   // 线形滤波

																		// 生成纹理
	glTexImage2D(GL_TEXTURE_2D, 0, 3, JPGWidth, JPGHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, pBits);

	DeleteObject(hbmpTemp);												// 删除对象
	DeleteDC(hdcTemp);													// 删除设备描述表

	pPicture->Release();												// 释放 IPicture

	return TRUE;														// 返回 TRUE
}