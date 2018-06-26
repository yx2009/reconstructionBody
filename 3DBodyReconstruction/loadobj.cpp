///////////////////////////////////////////////////////////////////////////////
//
// LoadOBJ.cpp : implementation file
//
// Purpose:	Implementation of OpenGL Window of OBJ Loader
//
// Created:
//	 JL 9/23/98	 
//
// Notes: This version doesn't used shared vertices in a vertex array. That
//	 would be a faster way of doing things. This creates 3 vertices per
// triangle.
///////////////////////////////////////////////////////////////////////////////
//
//	Copyright 1998 Jeff Lander, All Rights Reserved.
// For educational purposes only.
// Please do not republish in electronic or print form without permission
// Thanks - jeffl@darwin3d.com
//
///////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include "loadOBJ.h"

char *gMatName;

///////////////////////////////////////////////////////////////////////////////
// Function:	ParseString
// Purpose:	 Actually breaks a string of words into individual pieces
// Arguments:	Source string in, array to put the words and the count
///////////////////////////////////////////////////////////////////////////////
void ParseOBJString(char *buffer,CStringArray *words,int *cnt)//��ָ����������������ʹ�������κ�ֵ��ԭ����Ҳ�Ѿ����޸���
{
	/// Local Variables ///////////////////////////////////////////////////////////
	CString in = buffer; 
	CString temp;
	///////////////////////////////////////////////////////////////////////////////

	in.TrimLeft();// Remove all leading whitespace �޼����
	in.TrimRight();//// Remove all trailing whitespace�޼��Ҳ�
	*cnt = 0;//�൱����������һ���ðɣ�
	do 
	{
		temp = in.SpanExcluding(" \t");	 //���ص�һ��\t֮ǰ�������ַ�
		words->Add(temp);
		if (temp == in) break;
		in = in.Right(in.GetLength() - temp.GetLength());
		in.TrimLeft();// Remove all leading whitespace �޼����	 
		*cnt = *cnt + 1;	
	} while (1);
	*cnt = *cnt + 1;
}
//// ParseString //////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Procedure:	LoadMaterialLib
// Purpose:	 Handles the Loading of a Material library
// Arguments:	Name of the Material Library
///////////////////////////////////////////////////////////////////////////////	 
void LoadMaterialLib(CString name,t_Visual *visual)//���ز��ʵĺ���
{
	/// Local Variables ///////////////////////////////////////////////////////////
	int cnt;//����
	char buffer[MAX_STRINGLENGTH];//��255Ϊ���ȵ��ַ����黺����
	CStringArray words;
	CString temp;
	CString filePath = name;
	char c = '\\';
	char *pname=name.GetBuffer();
	const char *ptrr = strrchr(pname, c);
	int pos = ptrr - pname;
	filePath = filePath.Left(pos + 1);
	FILE *fp;//�ļ�ָ��
	int matCnt = 0,curMat = -1;
	///////////////////////////////////////////////////////////////////////////////
	strcpy(visual->map,"");
	fp = fopen((LPCTSTR)name,"r");
	if (fp != NULL)
	{
		// FIRST PASS SETS UP THE NUMBER OF OBJECTS IN THE FILE
		while (!feof(fp))
		{
			fgets(buffer,MAX_STRINGLENGTH,fp);	// GET A STRING FROM THE FILE
			ParseOBJString(buffer,&words,&cnt);	// BREAK THE STRING INTO cnt WORDS
			if (cnt > 0)	 // MAKE SURE SOME WORDS ARE THERE
			{
				temp = words.GetAt(0);	 // CHECK THE FIRST WORK
				if (temp.GetLength() > 0)
				{
					if (temp == "newmtl")	 // AMBIENT �ؼ���
					{
						matCnt++; //�õ����ʵ�����
					}
				}
			}
			words.RemoveAll();	 // CLEAR WORD BUFFER
		}

		gMatName = (char *)malloc(20 * matCnt);
		visual->matColor = (tVector *)malloc(sizeof(tVector) * matCnt);
		fseek(fp,0,SEEK_SET);
		// Get Data
		while (!feof(fp))
		{
			fgets(buffer,MAX_STRINGLENGTH,fp);	// GET A STRING FROM THE FILE
			ParseOBJString(buffer,&words,&cnt);	// BREAK THE STRING INTO cnt WORDS
			if (cnt > 0)	 // MAKE SURE SOME WORDS ARE THERE
			{
				temp = words.GetAt(0); // CHECK THE FIRST WORK
				if (temp.GetLength() > 0)
				{
					if (temp == "newmtl")	 // AMBIENT ������ 
					{
						curMat++;
						strncpy(&gMatName[20 * curMat],words.GetAt(1),20);
					}
					else if (temp == "Ka")	 // AMBIENT ������
					{
						visual->Ka.r = (float)atof(words.GetAt(1));
						visual->Ka.g = (float)atof(words.GetAt(2));
						visual->Ka.b = (float)atof(words.GetAt(3));
					}
					else if (temp == "Kd")	 // DIFFUSE COLOR ��������ɫ
					{
						visual->Kd.r = (float)atof(words.GetAt(1));
						visual->Kd.g = (float)atof(words.GetAt(2));
						visual->Kd.b = (float)atof(words.GetAt(3));
						visual->matColor[curMat].r = visual->Kd.r;
						visual->matColor[curMat].g = visual->Kd.g;
						visual->matColor[curMat].b = visual->Kd.b;
					}
					else if (temp == "Ks")	 // SPECULAR COLOR ���淴����ɫ
					{
						visual->Ks.r = (float)atof(words.GetAt(1));
						visual->Ks.g = (float)atof(words.GetAt(2));
						visual->Ks.b = (float)atof(words.GetAt(3));
					}
					else if (temp == "Ns")	 // SPECULAR COEFFICIENT���淴�����
					{
						visual->Ns = (float)atof(words.GetAt(1));
					}
					else if (temp == "map_Kd")	// TEXTURE MAP NAME �����ļ�·��
					{
						CString sssss22=words.GetAt(1);
						strcpy(&visual->map[curMat * 80],(LPCTSTR)(filePath+words.GetAt(1)));//��ȡjpg�ļ���
					//	BuildTexture(&visual->map[curMat * 80], texture[0]);//20140624
					}
					else if (temp == "map_Ks")	// TEXTURE MAP NAME �����ļ�·��
					{
						strcpy(&visual->map2[curMat * 80],(LPCTSTR)words.GetAt(1));
					}
				}
			}
			words.RemoveAll();	 // CLEAR WORD BUFFER
		}
		fclose(fp);
	}

	visual->matCnt = matCnt;
}

///////////////////////////////////////////////////////////////////////////////
// Procedure:	HandleFace
// Purpose:	 Handles the Face Line in an OBJ file. Extracts index info to 
//	 a face Structure
// Arguments:	Array of words from the face line, place to put the data
// Notes:	 Not an Official OBJ loader as it doesn't handle anything other than
//	 3-4 vertex polygons.
///////////////////////////////////////////////////////////////////////////////	 
void HandleFace(CStringArray *words,t_faceIndex *face)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	int loop,loopcnt;
	CString temp;
	CString vStr,nStr,tStr;	 // HOLD POINTERS TO ELEMENT POINTERS
	int nPos,tPos;
	///////////////////////////////////////////////////////////////////////////////
	loopcnt = words->GetSize();
	if (loopcnt > 4) loopcnt = 4;

	// LOOP THROUGH THE 3 - 4 WORDS OF THE FACELIST LINE, WORD 0 HAS 'f'
	for (loop = 1; loop < loopcnt; loop++)
	{
		temp = words->GetAt(loop);	 // GRAB THE NEXT WORD
		// FACE DATA IS IN THE FORMAT vertex/texture/normal
		tPos = temp.Find('/');	 // FIND THE '/' SEPARATING VERTEX AND TEXTURE
		if (tPos==-1)
		{
			face->v[loop-1]=atoi(temp)-1;
			continue;
		}
		vStr = temp.Left(tPos);	 // GET THE VERTEX NUMBER
	
		tStr = temp.Right(temp.GetLength() - tPos-1);


		//temp.SetAt(tPos,' ');	 // CHANGE THE '/' TO A SPACE SO I CAN TRY AGAIN//remove gyy 2014.07.02 16:08
		//nPos = temp.Find('/');	 // FIND THE '/' SEPARATING TEXTURE AND NORMAL//remove gyy 2014.07.02 16:08
		//tStr = temp.Mid(tPos + 1, nPos - tPos - 1);	 // GET THE TEXTURE NUMBER//remove gyy 2014.07.02 16:08
		//nStr = temp.Right(temp.GetLength() - nPos - 1);	// GET THE NORMAL NUMBER//remove gyy 2014.07.02 16:08
		face->v[loop - 1] =atoi(vStr)-1;	 // STORE OFF THE INDEX FOR THE VERTEX
		
		face->t[loop - 1] =atoi(tStr)-1;	 // STORE OFF THE INDEX FOR THE TEXTURE
		//face->n[loop - 1] = atoi(nStr) - 1;	 // STORE OFF THE INDEX FOR THE NORMAL//remove gyy 2014.07.02 16:08
	}
}
///// HandleFace //////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Procedure:	HandleFaceN
// Purpose:	 Handles the Face Line in an OBJ file. Extracts index info to 
//	 a face Structure
// Arguments:	Array of words from the face line, place to put the data
// Notes:	 Not an Official OBJ loader as it doesn't handle anything other than
//	 3-4 vertex polygons.
///////////////////////////////////////////////////////////////////////////////	 
void HandleFaceN(CStringArray *words, t_faceIndex *face)
{
	/// Local Variables ///////////////////////////////////////////////////////////
	int loop, loopcnt;
	CString temp;
	CString vStr, nStr, tStr;	 // HOLD POINTERS TO ELEMENT POINTERS
	int nPos, tPos;
	///////////////////////////////////////////////////////////////////////////////
	loopcnt = words->GetSize();
	if (loopcnt > 4) loopcnt = 4;

	// LOOP THROUGH THE 3 - 4 WORDS OF THE FACELIST LINE, WORD 0 HAS 'f'
	for (loop = 1; loop < loopcnt; loop++)
	{
		temp = words->GetAt(loop);	 // GRAB THE NEXT WORD
									 // FACE DATA IS IN THE FORMAT vertex/texture/normal
		tPos = temp.Find('/');	 // FIND THE '/' SEPARATING VERTEX AND TEXTURE
		vStr = temp.Left(tPos);	 // GET THE VERTEX NUMBER
		temp = temp.Right(temp.GetLength()-1-tPos);
		tPos = temp.Find('/');
		tStr = temp.Left(tPos);
		nStr = temp.Right(temp.GetLength()-1-tPos);
		//nStr = temp.Right(temp.GetLength() - tPos - 2);


		//temp.SetAt(tPos,' ');	 // CHANGE THE '/' TO A SPACE SO I CAN TRY AGAIN//remove gyy 2014.07.02 16:08
		//nPos = temp.Find('/');	 // FIND THE '/' SEPARATING TEXTURE AND NORMAL//remove gyy 2014.07.02 16:08
		//tStr = temp.Mid(tPos + 1, nPos - tPos - 1);	 // GET THE TEXTURE NUMBER//remove gyy 2014.07.02 16:08
		//nStr = temp.Right(temp.GetLength() - nPos - 1);	// GET THE NORMAL NUMBER//remove gyy 2014.07.02 16:08

		face->v[loop - 1] = atoi(vStr) - 1;	 // STORE OFF THE INDEX FOR THE VERTEX
		face->n[loop - 1] = atoi(nStr) - 1;	 // STORE OFF THE INDEX FOR THE TEXTURE
		face->t[loop - 1] = atoi(tStr) - 1;	 // STORE OFF THE INDEX FOR THE TEXTURE
											 //face->n[loop - 1] = atoi(nStr) - 1;	 // STORE OFF THE INDEX FOR THE NORMAL//remove gyy 2014.07.02 16:08
	}
}
///// HandleFaceN //////////////////////////////////////////////////////////////
int GetCurMat(CString name,t_Visual *visual)
{
	for (int loop = 0; loop < visual->matCnt; loop++)
		if (name == &gMatName[20 * loop])
			return loop;
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Procedure:	LoadOBJ//����obj�ļ�
// Purpose:	 Load an OBJ file into the current bone visual
// Arguments:	Name of 0BJ file and pointer to bone �Ǽܵ�ָ��
// Notes:	 Not an Official OBJ loader as it doesn't handle more than
//	 3 vertex polygons or multiple objects per file.
///////////////////////////////////////////////////////////////////////////////	
BOOL LoadOBJ(const char *filename, t_Visual *visual)//���������ļ�����һ��ָ�򶥵�ṹ��Ķ����ָ�룩
{
	/// Local Variables ///////////////////////////////////////////////////////////
	int loop, loop2, cnt;//��������ѭ���ı�����һ�����ڼ����ı���
	char buffer[MAX_STRINGLENGTH];//������һ��������ַ�������Ϊ���ȵ�char���� #define MAX_STRINGLENGTH	255
	CStringArray words;//һ����Ķ���
	CString temp;//�ַ��������ʱ����
	CString filePath = _T(filename);
	char c = '\\';
	const char *ptrr = strrchr(filename, c);
	int pos = ptrr - filename;
	filePath = filePath.Left(pos + 1);

	FILE *fp;//�ļ���һ��ָ��

	long vCnt = 0, nCnt = 0, tCnt = 0, fCnt = 0;//���� v=���㼯������ vn=���㷨���� vt=������������ f��
	long vnCnt=0,vcCnt=0,vtCnt=0;//��������Զ�����Ϣ�󸽴�������Ϣ����ɫ��Ϣ����������Ϣ�����vncCnt=��������ɫ��vtVnt=��������
	long vPos = 0, nPos = 0, tPos = 0, fPos = 0;//position������ʾ�����еĵڼ����㣩

	t_faceIndex *face = NULL;//�ṹ����� �����ı�����
	float *data;//float ������ָ��
	int	 curMat = 0;//���ͣ�����ʲô��˼��
	float vertexScale = 1.0;//�����ģ=1?


							///////////////////////////////////////////////////////////////////////////////
	fp = fopen(filename, "r");//�����ļ�����r�Ǹ�ʲô�������ļ���ָ��ָ������ļ�

	visual->glTex = 0;//��Ĭ�ϲ���=0ô��
	visual->vertexMax_x = 0.0;
	visual->vertexMax_y = 0.0;
	visual->vertexMax_z = 0.0;
	visual->vertexMin_x = 0.0;
	visual->vertexMin_y = 0.0;
	visual->vertexMin_z = 0.0;
	if (fp != NULL)//���ָ�벻�ǿյ�
	{
		// FIRST PASS SETS UP THE NUMBER OF OBJECTS IN THE FILE
		visual->GuassMaxValue = 0.0;
		visual->GuassMinValue = 0.0;
		while (!feof(fp))//��ô��ô�ļ�ָ��//����ļ��Ƿ�����ˣ������˷��ط���ֵ�����򷵻�0
		{
			fgets(buffer, MAX_STRINGLENGTH, fp);	// GET A STRING FROM THE FILE �õ��ļ���һ���ַ���
			ParseOBJString(buffer, &words, &cnt);	// //��buffer����ַ��������ʷֿ���cntΪ����
													//�������Թؼ���Ϊ�׵ĸ��е�����

			if (cnt > 0)	 // MAKE SURE SOME WORDS ARE THERE ����ʵĸ�������0�Ļ�
			{
				temp = words.GetAt(0);	 // CHECK THE FIRST WORK �ѵ�һ���ʸ���CString�����ʱ����

				if (temp.GetLength() > 0) //�������ʣ����ǵ�һ���ʰɣ���С����0�Ļ�
				{
					if (temp[0] == 'v')	 // ��� ����ʵ�����ĸ��v�����Ƕ��㼯������
					{
						if (temp.GetLength() > 1 && temp[1] == 'n')	 // vn IS A NORMAL ���㷨����
							nCnt++; //��ɼ�������

						else if (temp.GetLength() > 1 && temp[1] == 't')	// vt IS A TEXTURE ������������
							tCnt++;

						else if(words.GetCount()==4)
							vCnt++;		// ���㼯������ 
						else if(words.GetCount()==6)
						{
							vCnt++;		// ���㼯������
							vtCnt++;	 // �ݴ����������� 
						}
						else if(words.GetCount()==7)
						{
							vCnt++;		// ���㼯������
							vnCnt++;	 // �ݴ������������� 
							vcCnt++;	 // �ݴ���ɫ�������� 

						}
							

					}

					else if (temp[0] == 'n')//���㷨����
					{
						nCnt++;
					}
					else if (temp[0] == 'f') //��
						fCnt++;	 // f IS A FACE 
					
					else
					{
						if (temp == "mtllib") // HANDLE THE MATERIAL LIBRARY
						{
							LoadMaterialLib(filePath + words.GetAt(1), visual);//����mtllib���� ���ʿ�
							visual->mtl = temp + " " + words.GetAt(1);//add gyy 2014.12.06

							words.RemoveAll();
							continue;
						}

						visual->matColor = (tVector *)malloc(sizeof(tVector) * 1);
						visual->Ka.r = 1;
						visual->Ka.g =1;
						visual->Ka.b = 0;
						visual->Kd.r =1;
						visual->Kd.g = 1;
						visual->Kd.b =0;
						visual->matColor[curMat].r = visual->Kd.r;
						visual->matColor[curMat].g = visual->Kd.g;
						visual->matColor[curMat].b = visual->Kd.b;
						visual->Ks.r = 1;
						visual->Ks.g = 1;
						visual->Ks.b =0;
						visual->Ns = 1000;
					}
				}
			}
			words.RemoveAll();	 // CLEAR WORD BUFFER
		}

		// �������������ڴ�
		if (vCnt > 0)
		{
			visual->vertex = (tVector *)malloc(vCnt * sizeof(tVector));
			visual->GuassCurvature = (double *)malloc(vCnt * sizeof(double));
			visual->HCurvature = (double *)malloc(vCnt * sizeof(double));
			//visual->GroupVertex = (vertexsOfonevertex *)malloc(vCnt * sizeof(vertexsOfonevertex));
			int s = sizeof(vertexsOfonevertex);
			visual->Gvertex = (tVector *)malloc(vCnt * sizeof(tVector));
			visual->deformData = (tVector *)malloc(sizeof(tVector) * vCnt);
		
			if (nCnt > 0)
			{
				vnCnt=0;
				visual->normal = (tVector *)malloc(nCnt * sizeof(tVector));
			}
			//else if (nCnt==0&&vnCnt==vCnt&&tCnt>0)//����󸽴���Ϣ������
			//{
			//	vcCnt=0;vtCnt=0;
			//	visual->normal = (tVector *)malloc(vnCnt * sizeof(tVector));
			//}

			if (tCnt > 0)
			{
				vtCnt=0;vnCnt=0;vcCnt=0;
				visual->TextureStatu = 2;
				visual->texture = (tVector *)malloc(tCnt * sizeof(tVector));
			}else if (tCnt==0&&vtCnt==vCnt)//����󸽴���Ϣ����������
			{
				vnCnt=0;vcCnt=0;
				visual->TextureStatu = 2;
				visual->texture = (tVector *)malloc(vtCnt * sizeof(tVector));

			}else if(tCnt==0&&nCnt>0&&vcCnt==vCnt)//����󸽴���Ϣ�Ƕ�����ɫ
			{
				vnCnt=0;vtCnt=0;
				visual->TextureStatu = 1;
				visual->vertexColor = (tVector *)malloc(vCnt * sizeof(tVector)); 
			
			}else
			{
				visual->TextureStatu = 0;

			}
			if (fCnt > 0)
			{
				visual->index = (t_faceIndex *)malloc(fCnt * sizeof(t_faceIndex));
				visual->vfindex = (v_faceIndex *)malloc(vCnt * sizeof(v_faceIndex));//Ϊ��������������
				for (int tempi=0;tempi<vCnt;tempi++)
				{
					visual->vfindex[tempi].num = 0;
				}
			}
			

			fseek(fp, 0, SEEK_SET);
			// ��ȡ��ʵ����
			while (!feof(fp))
			{
				fgets(buffer, MAX_STRINGLENGTH, fp);
				ParseOBJString(buffer, &words, &cnt);
				if (cnt > 0)
				{
					temp = words.GetAt(0);
					if (temp.GetLength() > 0)
					{
						if (temp[0] == 'v')	 // WORDS STARTING WITH v
						{
							if (temp.GetLength() > 1 && temp[1] == 'n')	// vn NORMALS���㷨����
							{
								visual->normal[nPos].x = (float)atof(words.GetAt(1));
								visual->normal[nPos].y = (float)atof(words.GetAt(2));
								visual->normal[nPos].z = (float)atof(words.GetAt(3));
								
								nPos++;
							}
							else if (temp.GetLength() > 1 && temp[1] == 't')	// vt TEXTURES ������������
							{
								visual->texture[tPos].u = (float)atof(words.GetAt(1));
								visual->texture[tPos].v = (float)atof(words.GetAt(2));
								tPos++;
							}
							else	 // VERTICES ���㼯������
							{
								if (words.GetCount() ==7&&vnCnt>0)//��ȡ��������
								{
									visual->normal[vPos].x = (float)atof(words.GetAt(4));
									visual->normal[vPos].y = (float)atof(words.GetAt(5));
									visual->normal[vPos].z = (float)atof(words.GetAt(6));

								}else if (words.GetCount() ==7&&vcCnt>0&&tCnt<=0)//��ȡ������ɫ
								{
									visual->vertexColor[vPos].r = atof(words.GetAt(4));
									visual->vertexColor[vPos].g= atof(words.GetAt(5));
									visual->vertexColor[vPos].b = atof(words.GetAt(6));
								}else if (words.GetCount() ==6&&vtCnt>0)
								{
									visual->texture[vPos].u = (float)atof(words.GetAt(4));
									visual->texture[vPos].v = (float)atof(words.GetAt(5));

								}
								visual->vertex[vPos].x = (float)atof(words.GetAt(1)) * vertexScale;
								visual->Gvertex[vPos].x = visual->vertex[vPos].x;//add gyy 2014.11.14
								visual->vertex[vPos].y = (float)atof(words.GetAt(2)) * vertexScale;
								visual->Gvertex[vPos].y = visual->vertex[vPos].y;//add gyy 2014.11.14
								visual->vertex[vPos].z = (float)atof(words.GetAt(3)) * vertexScale;
								visual->Gvertex[vPos].z = visual->vertex[vPos].z;//add gyy 2014.11.14
								if (vPos>0)
								{
									if (visual->vertex[vPos].x > visual->vertexMax_x)
									{
										visual->vertexMax_x = visual->vertex[vPos].x;//ȡ���ֵx
									}else if (visual->vertex[vPos].x < visual->vertexMin_x)
									{
										visual->vertexMin_x = visual->vertex[vPos].x;//ȡ��Сֵx
									}
									if (visual->vertex[vPos].y > visual->vertexMax_y)
									{
										visual->vertexMax_y = visual->vertex[vPos].y;//ȡ���ֵy
									}else if (visual->vertex[vPos].y < visual->vertexMin_y)
									{
										visual->vertexMin_y = visual->vertex[vPos].y;//ȡ��Сֵx
									}
									if (visual->vertex[vPos].z > visual->vertexMax_z)
									{
										visual->vertexMax_z = visual->vertex[vPos].z;//ȡ���ֵz
									}else if (visual->vertex[vPos].z < visual->vertexMin_z)
									{
										visual->vertexMin_z = visual->vertex[vPos].z;//ȡ��Сֵz
									}
								}else
								{
									visual->vertexMax_x = visual->vertex[vPos].x;
									visual->vertexMax_y = visual->vertex[vPos].y;
									visual->vertexMax_z = visual->vertex[vPos].z;
									visual->vertexMin_x = visual->vertex[vPos].x;
									visual->vertexMin_y = visual->vertex[vPos].y;
									visual->vertexMin_z = visual->vertex[vPos].z;
								}
								vPos++;
							}
						}
						else if (temp[0] == 'n')//���㷨����
						{
							visual->normal[nPos].x = (float)atof(words.GetAt(1));
							visual->normal[nPos].y = (float)atof(words.GetAt(2));
							visual->normal[nPos].z = (float)atof(words.GetAt(3));
							nPos++;
						}
						else if (temp[0] == 'f')	 // f v/t/n v/t/n v/t/n	FACE LINE ��
						{
							if (words.GetSize() > 4)
							{
								sprintf(buffer, "Face %d has more than 3 vertices", fPos);
								MessageBox(NULL, buffer, "ERROR", MB_OK);
							}
							if (nCnt == 0) 
							{
								HandleFace(&words, &visual->index[fPos]);
								int numtemp = visual->vfindex[visual->index[fPos].v[0]].num;
								visual->vfindex[visual->index[fPos].v[0]].findex[numtemp] = fPos;
								visual->vfindex[visual->index[fPos].v[0]].num++;

								numtemp = visual->vfindex[visual->index[fPos].v[1]].num;
								visual->vfindex[visual->index[fPos].v[1]].findex[numtemp] = fPos;
								visual->vfindex[visual->index[fPos].v[1]].num++;

								numtemp = visual->vfindex[visual->index[fPos].v[2]].num;
								visual->vfindex[visual->index[fPos].v[2]].findex[numtemp] = fPos;
								visual->vfindex[visual->index[fPos].v[2]].num++;
							}
							else if (nCnt>0)
							{
								HandleFaceN(&words, &visual->index[fPos]);
								int numtemp = visual->vfindex[visual->index[fPos].v[0]].num;
								visual->vfindex[visual->index[fPos].v[0]].findex[numtemp] = fPos;
								visual->vfindex[visual->index[fPos].v[0]].num++;

								numtemp = visual->vfindex[visual->index[fPos].v[1]].num;
								visual->vfindex[visual->index[fPos].v[1]].findex[numtemp] = fPos;
								visual->vfindex[visual->index[fPos].v[1]].num++;

								numtemp = visual->vfindex[visual->index[fPos].v[2]].num;
								visual->vfindex[visual->index[fPos].v[2]].findex[numtemp] = fPos;
								visual->vfindex[visual->index[fPos].v[2]].num++;
							}//���������
																	 //int temp = visual->index[fPos].v[0];
																	 //visual->index[fPos].v[0] = visual->index[fPos].v[2];
																	 //visual->index[fPos].v[2] = temp;
																	 //temp = visual->index[fPos].n[0];
																	 //visual->index[fPos].n[0] = visual->index[fPos].n[2];
																	 //visual->index[fPos].n[2] = temp;
							visual->index[fPos].mat = curMat;
							fPos++;
						}
						else if (temp == "usemtl")	 // f v/t/n v/t/n v/t/n	FACE LINE �Ӵ˿�ʼָ������
						{
							curMat = GetCurMat(words.GetAt(1), visual);
							visual->mtlend0 = temp + " " + words.GetAt(1);//add gyy 2014.12.06
						}
						else if (temp == "scale")	 // f v/t/n v/t/n v/t/n	FACE LINE 
						{
							vertexScale = atof(words.GetAt(1));
						}
					}
				}
				words.RemoveAll();	 // CLEAR WORD BUFFER
			}

			visual->vertexCnt = vPos;	// Set the vertex count���ö������
			visual->faceCnt = fCnt;	// Set the vertex count
			visual->uvCnt = tCnt==0? vtCnt:tCnt;	// Set the vertex count
			visual->normalCnt = nCnt==0? vnCnt:nCnt;	// Set the normal count
		}

		fclose(fp);

		if (gMatName)
			free(gMatName);
	}
	else
		return FALSE;
	return TRUE;
}
///////////////////////////////////////////////////////////////////////////////
// Procedure:	LoadOBJ//����obj�ļ�
// Purpose:	 Load an OBJ file into the current bone visual
// Arguments:	Name of 0BJ file and pointer to bone �Ǽܵ�ָ��
// Notes:	 Not an Official OBJ loader as it doesn't handle more than
//	 3 vertex polygons or multiple objects per file.
///////////////////////////////////////////////////////////////////////////////	
BOOL LoadKinectOBJ(const char *filename, t_Visual *visual)//���������ļ�����һ��ָ�򶥵�ṹ��Ķ����ָ�룩
{
	/// Local Variables ///////////////////////////////////////////////////////////
	int loop, loop2, cnt;//��������ѭ���ı�����һ�����ڼ����ı���
	char buffer[MAX_STRINGLENGTH];//������һ��������ַ�������Ϊ���ȵ�char���� #define MAX_STRINGLENGTH	255
	CStringArray words;//һ����Ķ���
	CString temp;//�ַ��������ʱ����
	CString filePath = _T(filename);
	char c = '\\';
	const char *ptrr = strrchr(filename, c);
	int pos = ptrr - filename;
	filePath = filePath.Left(pos + 1);

	FILE *fp;//�ļ���һ��ָ��

	long vCnt = 0, nCnt = 0, tCnt = 0, fCnt = 0;//���� v=���㼯������ vn=���㷨���� vt=������������ f��
	long vPos = 0, nPos = 0, tPos = 0, fPos = 0;//position������ʾ�����еĵڼ����㣩

	t_faceIndex *face = NULL;//�ṹ����� �����ı�����
	float *data;//float ������ָ��
	int	 curMat = 0;//���ͣ�����ʲô��˼��
	float vertexScale = 1.0;//�����ģ=1?


							///////////////////////////////////////////////////////////////////////////////
	fp = fopen(filename, "r");//�����ļ�����r�Ǹ�ʲô�������ļ���ָ��ָ������ļ�

	visual->glTex = 0;//��Ĭ�ϲ���=0ô��
	if (fp != NULL)//���ָ�벻�ǿյ�
	{
		// FIRST PASS SETS UP THE NUMBER OF OBJECTS IN THE FILE
		while (!feof(fp))//��ô��ô�ļ�ָ��//����ļ��Ƿ�����ˣ������˷��ط���ֵ�����򷵻�0
		{
			fgets(buffer, MAX_STRINGLENGTH, fp);	// GET A STRING FROM THE FILE �õ��ļ���һ���ַ���
			ParseOBJString(buffer, &words, &cnt);	// //��buffer����ַ��������ʷֿ���cntΪ����
													//�������Թؼ���Ϊ�׵ĸ��е�����

			if (cnt > 0)	 // MAKE SURE SOME WORDS ARE THERE ����ʵĸ�������0�Ļ�
			{
				temp = words.GetAt(0);	 // CHECK THE FIRST WORK �ѵ�һ���ʸ���CString�����ʱ����

				if (temp.GetLength() > 0) //�������ʣ����ǵ�һ���ʰɣ���С����0�Ļ�
				{
					if (temp[0] == 'v')	 // ��� ����ʵ�����ĸ��v�����Ƕ��㼯������
					{
						if (temp.GetLength() > 1 && temp[1] == 'n')	 // vn IS A NORMAL ���㷨����
							nCnt++; //��ɼ�������

						else if (temp.GetLength() > 1 && temp[1] == 't')	// vt IS A TEXTURE ������������
							tCnt++;

						else
							vCnt++;	 // v IS A VERTEX ���㼯������ 

					}
					else if (temp[0]=='n')//���㷨����
					{
						nCnt++;
					}
					else if (temp[0] == 'f') //��
						fCnt++;	 // f IS A FACE 
					else if (temp == "mtllib") // HANDLE THE MATERIAL LIBRARY
					{
						LoadMaterialLib(filePath + words.GetAt(1), visual);//����mtllib���� ���ʿ�
						visual->mtl = temp + " " + words.GetAt(1);//add gyy 2014.12.06
					}
				}
			}
			words.RemoveAll();	 // CLEAR WORD BUFFER
		}

		// NOW THAT I KNOW HOW MANY, ALLOCATE ROOM FOR IT �������������ڴ�
		if (vCnt > 0)
		{
			visual->vertex = (tVector *)malloc(vCnt * sizeof(tVector));
			visual->Gvertex = (tVector *)malloc(vCnt * sizeof(tVector));
			visual->deformData = (tVector *)malloc(sizeof(tVector) * vCnt);
			if (nCnt > 0)
			{
				visual->normal = (tVector *)malloc(nCnt * sizeof(tVector));
			}
			if (tCnt > 0)
			{
				visual->texture = (tVector *)malloc(tCnt * sizeof(tVector));
			}
			if (fCnt > 0)
			{
				visual->index = (t_faceIndex *)malloc(fCnt * sizeof(t_faceIndex));
			}
			fseek(fp, 0, SEEK_SET);

			// NOW THAT IT IS ALL ALLOC'ED. GRAB THE REAL DATA ץס��ʵ����
			while (!feof(fp))
			{
				fgets(buffer, MAX_STRINGLENGTH, fp);
				ParseOBJString(buffer, &words, &cnt);
				if (cnt > 0)
				{
					temp = words.GetAt(0);
					if (temp.GetLength() > 0)
					{
						if (temp[0] == 'v')	 // WORDS STARTING WITH v
						{
							if (temp.GetLength() > 1 && temp[1] == 'n')	// vn NORMALS���㷨����
							{
								visual->normal[nPos].x = (float)atof(words.GetAt(1));
								visual->normal[nPos].y = (float)atof(words.GetAt(2));
								visual->normal[nPos].z = (float)atof(words.GetAt(3));
								nPos++;
							}
							else if (temp.GetLength() > 1 && temp[1] == 't')	// vt TEXTURES ������������
							{
								visual->texture[tPos].u = (float)atof(words.GetAt(1));
								visual->texture[tPos].v = (float)atof(words.GetAt(2));
								tPos++;
							}
							else	 // VERTICES ���㼯������
							{
								visual->vertex[vPos].x = (float)atof(words.GetAt(1)) * vertexScale;
								visual->Gvertex[vPos].x = visual->vertex[vPos].x;//add gyy 2014.11.14
								visual->vertex[vPos].y = (float)atof(words.GetAt(2)) * vertexScale;
								visual->Gvertex[vPos].y = visual->vertex[vPos].y;//add gyy 2014.11.14
								visual->vertex[vPos].z = (float)atof(words.GetAt(3)) * vertexScale;
								visual->Gvertex[vPos].z = visual->vertex[vPos].z;//add gyy 2014.11.14
								vPos++;
							}
						}
						else if (temp[0] == 'n')//���㷨����
						{
							visual->normal[nPos].x = (float)atof(words.GetAt(1));
							visual->normal[nPos].y = (float)atof(words.GetAt(2));
							visual->normal[nPos].z = (float)atof(words.GetAt(3));
							nPos++;
						}
						else if (temp[0] == 'f')	 // f v/t/n v/t/n v/t/n	FACE LINE ��
						{
							if (words.GetSize() > 4)
							{
								sprintf(buffer, "Face %d has more than 3 vertices", fPos);
								MessageBox(NULL, buffer, "ERROR", MB_OK);
							}
							if (nCnt == 0)
								HandleFace(&words, &visual->index[fPos]);
							else if (nCnt > 0)
							{
								HandleFaceN(&words, &visual->index[fPos]);
							}//���������
																	 //int temp = visual->index[fPos].v[0];
																	 //visual->index[fPos].v[0] = visual->index[fPos].v[2];
																	 //visual->index[fPos].v[2] = temp;
																	 //temp = visual->index[fPos].n[0];
																	 //visual->index[fPos].n[0] = visual->index[fPos].n[2];
																	 //visual->index[fPos].n[2] = temp;
							visual->index[fPos].mat = curMat;
							fPos++;
						}
						else if (temp == "usemtl")	 // f v/t/n v/t/n v/t/n	FACE LINE �Ӵ˿�ʼָ������
						{
							curMat = GetCurMat(words.GetAt(1), visual);
							visual->mtlend0 = temp + " " + words.GetAt(1);//add gyy 2014.12.06
						}
						else if (temp == "scale")	 // f v/t/n v/t/n v/t/n	FACE LINE 
						{
							vertexScale = atof(words.GetAt(1));
						}
					}
				}
				words.RemoveAll();	 // CLEAR WORD BUFFER
			}

			visual->vertexCnt = vPos;	// Set the vertex count���ö������
			visual->faceCnt = fCnt;	// Set the vertex count
			visual->uvCnt = tCnt;	// Set the vertex count
			visual->normalCnt = nCnt;	// Set the vertex count
		}

		fclose(fp);

		if (gMatName)
			free(gMatName);
	}
	else
		return FALSE;
	return TRUE;
}
