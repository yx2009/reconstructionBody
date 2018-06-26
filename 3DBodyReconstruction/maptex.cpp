#include "StdAfx.h"
#include "maptex.h"
#include <string.h>
#include <math.h>
#include <GL/gl.h>
#include "OGLView.h"  //201654



BOOL maptex::LoadBmpFile(const CString &sBmpFileName)
{
	m_bReady = FALSE;
	//�ͷ�֮ǰ��ȡ������///////
	//�ͷ�ͼ������
	if (m_hgImageData)
		GlobalFree(m_hgImageData);

	m_sBMPFileFullName = sBmpFileName;
	if (m_sBMPFileFullName.IsEmpty())
		return FALSE;
	CFile BmpFile;
	if (!BmpFile.Open(m_sBMPFileFullName, CFile::modeRead))
	{
		AfxMessageBox("�ļ���ʱ����!");
		return FALSE;
	}

	//��ȡ�ļ�ͷ  ��ʽ���
	if (BmpFile.Read(&m_BmpFileHeader, sizeof(BITMAPFILEHEADER)) < sizeof(BITMAPFILEHEADER))
	{
		AfxMessageBox("�ļ���������!");
		BmpFile.Close();
		return FALSE;
	}
	if (m_BmpFileHeader.bfType != 0x4D42)
	{
		AfxMessageBox("��λͼ�ļ�");
		BmpFile.Close();
		return FALSE;
	}

	//��ȡ��Ϣͷ
	if (BmpFile.Read(&m_BmpInfo.bmiHeader, sizeof(BITMAPINFOHEADER)) < sizeof(BITMAPINFOHEADER))
	{
		AfxMessageBox("�ļ���������!");
		BmpFile.Close();
		return FALSE;
	}
	//SetDIBitsToDevice()��StretchDIBits()�����д����ɫ�壬����Ҫ����ǰDC��ѡ���ɫ���ˣ�����

	//��ȡ��ɫ��
	if (m_BmpInfo.bmiHeader.biBitCount < 24)
		BmpFile.Read(&m_BmpInfo.bmiColors, GetColorNum(m_BmpInfo.bmiHeader) * sizeof(RGBQUAD));

	//��ȡͼ������
	long nBytePerLine = (m_BmpInfo.bmiHeader.biWidth*m_BmpInfo.bmiHeader.biBitCount + 31) / 32 * 4;//ͼ��ÿ����ռʵ���ֽ���������4�ı�����
	m_hgImageData = GlobalAlloc(GHND, nBytePerLine * m_BmpInfo.bmiHeader.biHeight);

	if (m_hgImageData == NULL)
	{
		AfxMessageBox("�ڴ治��");
		BmpFile.Close();
		return FALSE;
	}
	byte *pImageData = (byte *)GlobalLock(m_hgImageData);
	BmpFile.Read(pImageData, nBytePerLine * m_BmpInfo.bmiHeader.biHeight);

	GlobalUnlock(m_hgImageData);

	BmpFile.Close();
	m_bReady = TRUE;

	//���³������bmpͼ������ �������ɸ���bmpͼ��  by liubowen
	char *pFileName = "myfile.bmp";
	CFile file(pFileName, CFile::modeCreate | CFile::modeWrite);
	if (m_hgImageData == NULL)
		return 0;

	try
	{
		//д���ļ�ͷ
		/*
		m_BmpFileHeader.bfType = 0x4d42;
		m_BmpFileHeader.bfSize = sizeof(BITMAPFILEHEADER);
		m_BmpFileHeader.bfReserved1 = 0;
		m_BmpFileHeader.bfReserved2 = 0;
		m_BmpFileHeader.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
		*/
		file.Write(&m_BmpFileHeader, sizeof(BITMAPFILEHEADER));

		//д����Ϣͷ
		file.Write(&m_BmpInfo.bmiHeader, sizeof(BITMAPINFOHEADER));

		//д���ɫ��
		if (m_BmpInfo.bmiHeader.biBitCount < 24)
		{
			file.Write(&m_BmpInfo.bmiColors, GetColorNum(m_BmpInfo.bmiHeader) * sizeof(RGBQUAD));
		}

		//add for ����������ÿ�����ص��ֵ ������RGBֵ���޸�
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

		//д��ͼ������
		file.Write(pImageData, nBytePerLine * m_BmpInfo.bmiHeader.biHeight);

		//�����δд������ݲ��ر��ļ�
		file.Flush();
		file.Close();
	}

	//�׳��쳣����
	catch (CFileException *e)
	{
		CString str;
		str.Format("��������ʧ�ܵ�ԭ����:%d", e->m_cause);
		MessageBox(NULL, "str", "��������", MB_OK);
		file.Abort();
		e->Delete();
	}
	//����bmp��������

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
	HDC			hdcTemp;												// DC��������λͼ
	HBITMAP		hbmpTemp;												// ������ʱλͼ
	IPicture	*pPicture;												// ����IPicture Interface
	OLECHAR		wszPath[MAX_PATH + 1];									// ͼƬ����ȫ·��
	char		szPath[MAX_PATH + 1];										// ͼƬ����ȫ·��
	long		lWidth;													// ͼ����
	long		lHeight;												// ͼ��߶�
																		//long		JPGWidth;	//��Ϊȫ��										// ͼ��Ŀ��(������Ϊ��λ)
																		//long		JPGHeight;											// ͼ��ĸߴ�(������Ϊ��λ)
	GLint		glMaxTexDim;											// ������������ߴ�

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

	if (FAILED(hr))														// �������ʧ��
	{
		// ͼƬ����ʧ�ܳ�����Ϣ
		MessageBox(HWND_DESKTOP, "ͼƬ����ʧ��1!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// ���� FALSE
	}

	//hdcTemp = GetDC(m_hwind)/*CreateCompatibleDC(m_hDC)*/;

	hdcTemp = CreateCompatibleDC(GetDC(0));				// ���������豸������
	if (!hdcTemp)										// ����ʧ��?
	{
		pPicture->Release();											// �ͷ�IPicture
																		// ͼƬ����ʧ�ܳ�����Ϣ
		MessageBox(HWND_DESKTOP, "ͼƬ����ʧ��2!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// ���� FALSE
	}

	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &glMaxTexDim);					// ȡ��֧�ֵ��������ߴ�

	pPicture->get_Width(&lWidth);										// ȡ��IPicture ��� (ת��ΪPixels��ʽ)
	JPGWidth = MulDiv(lWidth, GetDeviceCaps(hdcTemp, LOGPIXELSX), 2540);
	pPicture->get_Height(&lHeight);										// ȡ��IPicture �߶� (ת��ΪPixels��ʽ)
	JPGHeight = MulDiv(lHeight, GetDeviceCaps(hdcTemp, LOGPIXELSY), 2540);

	// ����ͼƬ����õ�Ч��
	if (JPGWidth <= glMaxTexDim)									// ͼƬ����Ƿ񳬹��Կ����֧�ֳߴ�
		JPGWidth = 1 << (int)floor((log((double)JPGWidth) / log(2.0f)) + 0.5f);
	else																// ����,��ͼƬ�����Ϊ�Կ����֧�ֳߴ�
		JPGWidth = glMaxTexDim;

	if (JPGHeight <= glMaxTexDim)									// ͼƬ�߶��Ƿ񳬹��Կ����֧�ֳߴ�
		JPGHeight = 1 << (int)floor((log((double)JPGHeight) / log(2.0f)) + 0.5f);
	else 																// ����,��ͼƬ�߶���Ϊ�Կ����֧�ֳߴ�
		JPGHeight = glMaxTexDim;

	// ����һ����ʱλͼ
	BITMAPINFO	bi = { 0 };												// λͼ������
	DWORD		*pBits = 0;												// ָ��λͼBits��ָ��

	bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);				// ���ýṹ��С
	bi.bmiHeader.biBitCount = 32;									// 32 λ
	bi.bmiHeader.biWidth = JPGWidth;							// �������ֵ
	bi.bmiHeader.biHeight = JPGHeight;						// �߶�����ֵ
	bi.bmiHeader.biCompression = BI_RGB;								// RGB ��ʽ
	bi.bmiHeader.biPlanes = 1;									// һ��λƽ��

																// ����һ��λͼ�������ǿ���ָ����ɫ����� ������ÿλ��ֵ
	hbmpTemp = CreateDIBSection(hdcTemp, &bi, DIB_RGB_COLORS, (void**)&pBits, 0, 0);

	if (!hbmpTemp)														// ����ʧ��?
	{
		DeleteDC(hdcTemp);												// ɾ���豸������
		pPicture->Release();											// �ͷ�IPicture
																		// ͼƬ����ʧ�ܳ�����Ϣ
		MessageBox(HWND_DESKTOP, "ͼƬ����ʧ��3!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// ���� FALSE
	}

	SelectObject(hdcTemp, hbmpTemp);									// ѡ����ʱDC�������ʱλͼ����

																		// ��λͼ�ϻ���IPicture
	pPicture->Render(hdcTemp, 0, 0, JPGWidth, JPGHeight, 0, lHeight, lWidth, -lHeight, 0);


	pImageData = new BYTE[JPGHeight*JPGWidth * 4];  //Ϊ����������Ϣ�����ڴ�
	nBytePerLine = (JPGWidth * 32 / 8 + 3) / 4 * 4;

	// ��BGRת��ΪRGB����ALPHAֵ��Ϊ255
	for (long i = 0; i < JPGWidth * JPGHeight; i++)				// ѭ���������е�����
	{
		BYTE* pPixel = (BYTE*)(&pBits[i]);							// ��ȡ��ǰ����
		BYTE  temp = pPixel[0];									// ��ʱ�洢��һ����ɫ����(��ɫ)
		pPixel[0] = pPixel[2];									// ����ɫֵ�浽��һλ
		pPixel[2] = temp;											// ����ɫֵ�浽����λ
		pPixel[3] = 255;											// ALPHAֵ��Ϊ255

		pImageData[4 * i + 0] = pPixel[2];                 //2016426  lbw  B
		pImageData[4 * i + 1] = pPixel[1];                  //G
		pImageData[4 * i + 2] = pPixel[0];                  //R
		pImageData[4 * i + 3] = pPixel[3];                  //A
	}

	glGenTextures(1, &texid);											// ��������

																		// ʹ������λͼ�������� �ĵ�������
	glBindTexture(GL_TEXTURE_2D, texid);								// ������
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// �����˲�
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);   // �����˲�

																		// ��������
	glTexImage2D(GL_TEXTURE_2D, 0, 3, JPGWidth, JPGHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, pBits);

	DeleteObject(hbmpTemp);												// ɾ������
	DeleteDC(hdcTemp);													// ɾ���豸������

	pPicture->Release();												// �ͷ� IPicture

	return TRUE;														// ���� TRUE
}

BOOL maptex::BuildTextureforAll(char *szPathName, GLuint &texid, HWND &m_hwind) //lbw 2016426
{
	HDC			hdcTemp;												// DC��������λͼ
	HBITMAP		hbmpTemp;												// ������ʱλͼ
	IPicture	*pPicture;												// ����IPicture Interface
	OLECHAR		wszPath[MAX_PATH + 1];									// ͼƬ����ȫ·��
	char		szPath[MAX_PATH + 1];										// ͼƬ����ȫ·��
	long		lWidth;													// ͼ����
	long		lHeight;												// ͼ��߶�
																		//long		JPGWidth;	//��Ϊȫ��										// ͼ��Ŀ��(������Ϊ��λ)
																		//long		JPGHeight;											// ͼ��ĸߴ�(������Ϊ��λ)
	GLint		glMaxTexDim;											// ������������ߴ�

	if (strstr(szPathName, "http://"))									// ���·������ http:// ��...
	{
		strcpy(szPath, szPathName);										// ��·�������� szPath
	}
	else																// ������ļ�����ͼƬ
	{
		GetCurrentDirectory(MAX_PATH, szPath);							// ȡ�õ�ǰ·��
		strcat(szPath, "\\");											// ����ַ�"\"
		strcat(szPath, szPathName);										// ���ͼƬ�����·��
	}

	MultiByteToWideChar(CP_ACP, 0, szPath, -1, wszPath, MAX_PATH);		// ��ASCII��ת��ΪUnicode��׼��
	HRESULT hr = OleLoadPicturePath(wszPath, 0, 0, 0, IID_IPicture, (void**)&pPicture);

	if (FAILED(hr))														// �������ʧ��
	{
		// ͼƬ����ʧ�ܳ�����Ϣ
		MessageBox(HWND_DESKTOP, "ͼƬ����ʧ��1!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// ���� FALSE
	}

	//hdcTemp = GetDC(m_hwind)/*CreateCompatibleDC(m_hDC)*/;

	hdcTemp = CreateCompatibleDC(GetDC(0));				// ���������豸������
	if (!hdcTemp)										// ����ʧ��?
	{
		pPicture->Release();											// �ͷ�IPicture
																		// ͼƬ����ʧ�ܳ�����Ϣ
		MessageBox(HWND_DESKTOP, "ͼƬ����ʧ��2!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// ���� FALSE
	}

	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &glMaxTexDim);					// ȡ��֧�ֵ��������ߴ�

	pPicture->get_Width(&lWidth);										// ȡ��IPicture ��� (ת��ΪPixels��ʽ)
	JPGWidth = MulDiv(lWidth, GetDeviceCaps(hdcTemp, LOGPIXELSX), 2540);
	pPicture->get_Height(&lHeight);										// ȡ��IPicture �߶� (ת��ΪPixels��ʽ)
	JPGHeight = MulDiv(lHeight, GetDeviceCaps(hdcTemp, LOGPIXELSY), 2540);

	// ����ͼƬ����õ�Ч��
	if (JPGWidth <= glMaxTexDim)									// ͼƬ����Ƿ񳬹��Կ����֧�ֳߴ�
		JPGWidth = 1 << (int)floor((log((double)JPGWidth) / log(2.0f)) + 0.5f);
	else																// ����,��ͼƬ�����Ϊ�Կ����֧�ֳߴ�
		JPGWidth = glMaxTexDim;

	if (JPGHeight <= glMaxTexDim)									// ͼƬ�߶��Ƿ񳬹��Կ����֧�ֳߴ�
		JPGHeight = 1 << (int)floor((log((double)JPGHeight) / log(2.0f)) + 0.5f);
	else 																// ����,��ͼƬ�߶���Ϊ�Կ����֧�ֳߴ�
		JPGHeight = glMaxTexDim;

	// ����һ����ʱλͼ
	BITMAPINFO	bi = { 0 };												// λͼ������
	DWORD		*pBits = 0;												// ָ��λͼBits��ָ��

	bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);				// ���ýṹ��С
	bi.bmiHeader.biBitCount = 32;									// 32 λ
	bi.bmiHeader.biWidth = JPGWidth;							// �������ֵ
	bi.bmiHeader.biHeight = JPGHeight;						// �߶�����ֵ
	bi.bmiHeader.biCompression = BI_RGB;								// RGB ��ʽ
	bi.bmiHeader.biPlanes = 1;									// һ��λƽ��

																// ����һ��λͼ�������ǿ���ָ����ɫ����� ������ÿλ��ֵ
	hbmpTemp = CreateDIBSection(hdcTemp, &bi, DIB_RGB_COLORS, (void**)&pBits, 0, 0);

	if (!hbmpTemp)														// ����ʧ��?
	{
		DeleteDC(hdcTemp);												// ɾ���豸������
		pPicture->Release();											// �ͷ�IPicture
																		// ͼƬ����ʧ�ܳ�����Ϣ
		MessageBox(HWND_DESKTOP, "ͼƬ����ʧ��3!\n(TextureLoad Failed!)", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;													// ���� FALSE
	}

	SelectObject(hdcTemp, hbmpTemp);									// ѡ����ʱDC�������ʱλͼ����

																		// ��λͼ�ϻ���IPicture
	pPicture->Render(hdcTemp, 0, 0, JPGWidth, JPGHeight, 0, lHeight, lWidth, -lHeight, 0);


	pImageData = new BYTE[JPGHeight*JPGWidth];  //Ϊ����������Ϣ�����ڴ�
	nBytePerLine = (JPGWidth * 32 / 8 + 3) / 4 * 4;

	// ��BGRת��ΪRGB����ALPHAֵ��Ϊ255
	for (long i = 0; i < JPGWidth * JPGHeight; i++)				// ѭ���������е�����
	{
		BYTE* pPixel = (BYTE*)(&pBits[i]);							// ��ȡ��ǰ����
		BYTE  temp = pPixel[0];									// ��ʱ�洢��һ����ɫ����(��ɫ)
		pPixel[0] = pPixel[2];									// ����ɫֵ�浽��һλ
		pPixel[2] = temp;											// ����ɫֵ�浽����λ
		pPixel[3] = 255;											// ALPHAֵ��Ϊ255

		pImageData[i] = pPixel[2];                 //2016426  lbw  B
	}

	glGenTextures(1, &texid);											// ��������

																		// ʹ������λͼ�������� �ĵ�������
	glBindTexture(GL_TEXTURE_2D, texid);								// ������
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// �����˲�
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);   // �����˲�

																		// ��������
	glTexImage2D(GL_TEXTURE_2D, 0, 3, JPGWidth, JPGHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, pBits);

	DeleteObject(hbmpTemp);												// ɾ������
	DeleteDC(hdcTemp);													// ɾ���豸������

	pPicture->Release();												// �ͷ� IPicture

	return TRUE;														// ���� TRUE
}