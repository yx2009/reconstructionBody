

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
unsigned char *LoadTGAFile( char * strFilename,	tTGAHeader_s *header)//����ͼƬ��Ŀǰ��������ԭ����ͼƬ��ʽ����//gyy 2014.06.30
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

	bitsize = dwWidth * dwHeight * (BPP/8);//Ӧ��û�г���Χ��gyy2014.06.30
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


// ����BMP,JPG,GIF���ļ�  20140624
BOOL BuildTexture(char *szPathName, GLuint &texid)						// ����ͼƬ��ת��Ϊ����
{
	HDC			hdcTemp;												// DC��������λͼ
	HBITMAP		hbmpTemp;												// ������ʱλͼ
	IPicture	*pPicture;												// ����IPicture Interface
	OLECHAR		wszPath[MAX_PATH+1];									// ͼƬ����ȫ·��
	char		szPath[MAX_PATH+1];										// ͼƬ����ȫ·��
	long		lWidth;													// ͼ����
	long		lHeight;												// ͼ��߶�
	long		lWidthPixels;											// ͼ��Ŀ��(������Ϊ��λ)
	long		lHeightPixels;											// ͼ��ĸߴ�(������Ϊ��λ)
	GLint		glMaxTexDim ;											// ������������ߴ�

	//if (strstr(szPathName, "http://"))									// ���·������ http:// ��...
	//{
	//	strcpy(szPath, szPathName);										// ��·�������� szPath
	//}
	//else																// ������ļ�����ͼƬ
	//{
	//	GetCurrentDirectory(MAX_PATH, szPath);							// ȡ�õ�ǰ·��
	//	strcat(szPath, "\\");											// ����ַ�"\"
	//	strcat(szPath, szPathName);										// ���ͼƬ�����·��
	//}
	strcpy(szPath, szPathName);

	MultiByteToWideChar(CP_ACP, 0, szPath, -1, wszPath, MAX_PATH);		// ��ASCII��ת��ΪUnicode��׼��
	HRESULT hr = OleLoadPicturePath(wszPath, 0, 0, 0, IID_IPicture, (void**)&pPicture);

	if(FAILED(hr))														// �������ʧ��
	{
		// ͼƬ����ʧ�ܳ�����Ϣ
		MessageBox (HWND_DESKTOP, "ͼƬ����ʧ��1!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// ���� FALSE
	}

	hdcTemp = CreateCompatibleDC(GetDC(0));								// ���������豸������
	if(!hdcTemp)														// ����ʧ��?
	{
		pPicture->Release();											// �ͷ�IPicture
		// ͼƬ����ʧ�ܳ�����Ϣ
		MessageBox (HWND_DESKTOP, "ͼƬ����ʧ��2!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// ���� FALSE
	}

	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &glMaxTexDim);					// ȡ��֧�ֵ��������ߴ�

	pPicture->get_Width(&lWidth);										// ȡ��IPicture ��� (ת��ΪPixels��ʽ)
	lWidthPixels	= MulDiv(lWidth, GetDeviceCaps(hdcTemp, LOGPIXELSX), 2540);
	pPicture->get_Height(&lHeight);										// ȡ��IPicture �߶� (ת��ΪPixels��ʽ)
	lHeightPixels	= MulDiv(lHeight, GetDeviceCaps(hdcTemp, LOGPIXELSY), 2540);

	// ����ͼƬ����õ�Ч��
	if (lWidthPixels <= glMaxTexDim)									// ͼƬ����Ƿ񳬹��Կ����֧�ֳߴ�
		lWidthPixels = 1 << (int)floor((log((double)lWidthPixels)/log(2.0f)) + 0.5f); 
	else																// ����,��ͼƬ�����Ϊ�Կ����֧�ֳߴ�
		lWidthPixels = glMaxTexDim;

	if (lHeightPixels <= glMaxTexDim)									// ͼƬ�߶��Ƿ񳬹��Կ����֧�ֳߴ�
		lHeightPixels = 1 << (int)floor((log((double)lHeightPixels)/log(2.0f)) + 0.5f);
	else																// ����,��ͼƬ�߶���Ϊ�Կ����֧�ֳߴ�
		lHeightPixels = glMaxTexDim;

	// ����һ����ʱλͼ
	BITMAPINFO	bi = {0};												// λͼ������
	DWORD		*pBits = 0;												// ָ��λͼBits��ָ��

	bi.bmiHeader.biSize			= sizeof(BITMAPINFOHEADER);				// ���ýṹ��С
	bi.bmiHeader.biBitCount		= 32;									// 32 λ
	bi.bmiHeader.biWidth		= lWidthPixels;							// �������ֵ
	bi.bmiHeader.biHeight		= lHeightPixels;						// �߶�����ֵ
	bi.bmiHeader.biCompression	= BI_RGB;								// RGB ��ʽ
	bi.bmiHeader.biPlanes		= 1;									// һ��λƽ��

	// ����һ��λͼ�������ǿ���ָ����ɫ����� ������ÿλ��ֵ
	hbmpTemp = CreateDIBSection(hdcTemp, &bi, DIB_RGB_COLORS, (void**)&pBits, 0, 0);

	if(!hbmpTemp)														// ����ʧ��?
	{
		DeleteDC(hdcTemp);												// ɾ���豸������
		pPicture->Release();											// �ͷ�IPicture
		// ͼƬ����ʧ�ܳ�����Ϣ
		MessageBox (HWND_DESKTOP, "ͼƬ����ʧ��3!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// ���� FALSE
	}

	SelectObject(hdcTemp, hbmpTemp);									// ѡ����ʱDC�������ʱλͼ����
 	// ��λͼ�ϻ���IPicture
	pPicture->Render(hdcTemp, 0, 0, lWidthPixels, lHeightPixels, 0, lHeight, lWidth, -lHeight, 0);

	// ��BGRת��ΪRGB����ALPHAֵ��Ϊ255
	for(long i = 0; i < lWidthPixels * lHeightPixels; i++)				// ѭ���������е�����
	{
		BYTE* pPixel	= (BYTE*)(&pBits[i]);							// ��ȡ��ǰ����
		BYTE  temp		= pPixel[0];									// ��ʱ�洢��һ����ɫ����(��ɫ)
		pPixel[0]		= pPixel[2];									// ����ɫֵ�浽��һλ
		pPixel[2]		= temp;											// ����ɫֵ�浽����λ
		pPixel[3]		= 255;											// ALPHAֵ��Ϊ255
	}

	glGenTextures(1, &texid);											// ��������

	// ʹ������λͼ�������� �ĵ�������
	glBindTexture(GL_TEXTURE_2D, texid);								// ������
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// �����˲�
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);   // �����˲�
																		// ��������
	glTexImage2D(GL_TEXTURE_2D, 0, 3, lWidthPixels, lHeightPixels, 0, GL_RGBA, GL_UNSIGNED_BYTE, pBits);


	DeleteObject(hbmpTemp);												// ɾ������
	DeleteDC(hdcTemp);													// ɾ���豸������

	pPicture->Release();												// �ͷ� IPicture

	return TRUE;														// ���� TRUE
}

