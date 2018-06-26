

#include "stdafx.h"
#include "loadtex.h"
#include <GL/gl.h>
#include <string.h>
#include <math.h>


///////////////////////////////////////////////////////////////////////////////
// This stuff forward is the TGA Loading Routines
//
///////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------
// Name: LoadTGAFile()
// Desc: Given a filename of a TGA file, it loads the image in BITMAP form
//-----------------------------------------------------------------------
unsigned char *LoadTGAFile( char * strFilename,	tTGAHeader_s *header)//载入图片，目前出错，估计原因是图片格式不对//gyy 2014.06.30
{
/// Local Variables ///////////////////////////////////////////////////////////
	short			BPP;
	unsigned char	*buffer;
	int bitsize;		/* Total size of bitmap */
	BYTE	*newbits;		/* New RGB bits */
	BYTE	*from, *to;		/* RGB looping vars */
	int		i, j,			/* Looping vars */
	width;			/* Aligned width of bitmap */
    FILE* file;
///////////////////////////////////////////////////////////////////////////////

    // Open the file and read the header
	file = fopen( strFilename, TEXT("rb") );
    if( NULL == file )
        return NULL;

    if ( fread( header, sizeof( tTGAHeader_s ), 1, file ) != 1 )
    {
        fclose( file );
        return NULL;
    }

    // Parse the TGA header
    DWORD dwWidth, dwHeight;
	dwWidth = (DWORD)header->d_width;
	dwHeight = (DWORD)header->d_height;
	BPP = (short)header->d_pixel_size;          // 16, 24, or 32

	// JL TEST SMALL TEXTURES ONLY
	//dwWidth = 2;//remove gyy 2014.06.30
	//dwHeight = 2;//remove gyy 2014.06.30
    // Create a bitmap to load the data into

	bitsize = dwWidth * dwHeight * (BPP/8);//应该没有超范围的gyy2014.06.30
	if ((newbits = (BYTE *)calloc(bitsize, 1)) == NULL)
	{
        fclose( file );
        return NULL;
	}
 	buffer = (unsigned char *)malloc(dwWidth*dwHeight*(BPP / 8));
    if ( fread( buffer, dwWidth*dwHeight*(BPP / 8), 1, file ) != 1 )
	{
        fclose( file );
		free(buffer);
		free(newbits);
        return NULL;
	}

	width   = (BPP / 8) * dwWidth;

    for (i = 0; i < dwHeight; i ++)
		for (j = 0, from = ((BYTE *)buffer) + i * width,to = newbits + i * width;j < dwWidth
			;j ++, from += (BPP / 8), to += (BPP / 8))
        {
				if (BPP == 24)
				{
					to[0] = from[2];
					to[1] = from[1];
					to[2] = from[0];
				}
				else
				{
					to[0] = from[0];
					to[1] = from[1];
					to[2] = from[2];
					to[3] = from[3];
				}
        };
	// SINCE TGA IS UPSIDE DOWN, I HAVE TO REVERSE THIS DAMN THING
	for (int loop = 0; loop < dwHeight; loop++)
	{
		memcpy(	&newbits[loop * dwWidth * (BPP / 8)],
				&buffer[(dwHeight - (loop + 1)) * dwWidth * (BPP / 8)],
				dwWidth * (BPP / 8));
	}

	free(buffer);
	//free(buffer2);
    fclose( file );

    return newbits;
}


// 载入BMP,JPG,GIF等文件  20140624
BOOL BuildTexture(char *szPathName, GLuint &texid)						// 载入图片并转换为纹理
{
	HDC			hdcTemp;												// DC用来保存位图
	HBITMAP		hbmpTemp;												// 保存临时位图
	IPicture	*pPicture;												// 定义IPicture Interface
	OLECHAR		wszPath[MAX_PATH+1];									// 图片的完全路径
	char		szPath[MAX_PATH+1];										// 图片的完全路径
	long		lWidth;													// 图像宽度
	long		lHeight;												// 图像高度
	long		lWidthPixels;											// 图像的宽带(以像素为单位)
	long		lHeightPixels;											// 图像的高带(以像素为单位)
	GLint		glMaxTexDim ;											// 保存纹理的最大尺寸

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

	if(FAILED(hr))														// 如果导入失败
	{
		// 图片载入失败出错信息
		MessageBox (HWND_DESKTOP, "图片导入失败1!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// 返回 FALSE
	}

	hdcTemp = CreateCompatibleDC(GetDC(0));								// 建立窗口设备描述表
	if(!hdcTemp)														// 建立失败?
	{
		pPicture->Release();											// 释放IPicture
		// 图片载入失败出错信息
		MessageBox (HWND_DESKTOP, "图片导入失败2!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// 返回 FALSE
	}

	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &glMaxTexDim);					// 取得支持的纹理最大尺寸

	pPicture->get_Width(&lWidth);										// 取得IPicture 宽度 (转换为Pixels格式)
	lWidthPixels	= MulDiv(lWidth, GetDeviceCaps(hdcTemp, LOGPIXELSX), 2540);
	pPicture->get_Height(&lHeight);										// 取得IPicture 高度 (转换为Pixels格式)
	lHeightPixels	= MulDiv(lHeight, GetDeviceCaps(hdcTemp, LOGPIXELSY), 2540);

	// 调整图片到最好的效果
	if (lWidthPixels <= glMaxTexDim)									// 图片宽度是否超过显卡最大支持尺寸
		lWidthPixels = 1 << (int)floor((log((double)lWidthPixels)/log(2.0f)) + 0.5f); 
	else																// 否则,将图片宽度设为显卡最大支持尺寸
		lWidthPixels = glMaxTexDim;

	if (lHeightPixels <= glMaxTexDim)									// 图片高度是否超过显卡最大支持尺寸
		lHeightPixels = 1 << (int)floor((log((double)lHeightPixels)/log(2.0f)) + 0.5f);
	else																// 否则,将图片高度设为显卡最大支持尺寸
		lHeightPixels = glMaxTexDim;

	// 建立一个临时位图
	BITMAPINFO	bi = {0};												// 位图的类型
	DWORD		*pBits = 0;												// 指向位图Bits的指针

	bi.bmiHeader.biSize			= sizeof(BITMAPINFOHEADER);				// 设置结构大小
	bi.bmiHeader.biBitCount		= 32;									// 32 位
	bi.bmiHeader.biWidth		= lWidthPixels;							// 宽度像素值
	bi.bmiHeader.biHeight		= lHeightPixels;						// 高度像素值
	bi.bmiHeader.biCompression	= BI_RGB;								// RGB 格式
	bi.bmiHeader.biPlanes		= 1;									// 一个位平面

	// 建立一个位图这样我们可以指定颜色和深度 并访问每位的值
	hbmpTemp = CreateDIBSection(hdcTemp, &bi, DIB_RGB_COLORS, (void**)&pBits, 0, 0);

	if(!hbmpTemp)														// 建立失败?
	{
		DeleteDC(hdcTemp);												// 删除设备描述表
		pPicture->Release();											// 释放IPicture
		// 图片载入失败出错信息
		MessageBox (HWND_DESKTOP, "图片导入失败3!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// 返回 FALSE
	}

	SelectObject(hdcTemp, hbmpTemp);									// 选择临时DC句柄和临时位图对象
 	// 在位图上绘制IPicture
	pPicture->Render(hdcTemp, 0, 0, lWidthPixels, lHeightPixels, 0, lHeight, lWidth, -lHeight, 0);

	// 将BGR转换为RGB　将ALPHA值设为255
	for(long i = 0; i < lWidthPixels * lHeightPixels; i++)				// 循环遍历所有的像素
	{
		BYTE* pPixel	= (BYTE*)(&pBits[i]);							// 获取当前像素
		BYTE  temp		= pPixel[0];									// 临时存储第一个颜色像素(蓝色)
		pPixel[0]		= pPixel[2];									// 将红色值存到第一位
		pPixel[2]		= temp;											// 将蓝色值存到第三位
		pPixel[3]		= 255;											// ALPHA值设为255
	}

	glGenTextures(1, &texid);											// 创建纹理

	// 使用来自位图数据生成 的典型纹理
	glBindTexture(GL_TEXTURE_2D, texid);								// 绑定纹理
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// 线形滤波
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);   // 线形滤波
																		// 生成纹理
	glTexImage2D(GL_TEXTURE_2D, 0, 3, lWidthPixels, lHeightPixels, 0, GL_RGBA, GL_UNSIGNED_BYTE, pBits);


	DeleteObject(hbmpTemp);												// 删除对象
	DeleteDC(hdcTemp);													// 删除设备描述表

	pPicture->Release();												// 释放 IPicture

	return TRUE;														// 返回 TRUE
}

