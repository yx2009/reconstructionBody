
// 3DBodyReconstructionView.cpp : CMy3DBodyReconstructionView ���ʵ��
//

#include "stdafx.h"
// SHARED_HANDLERS ������ʵ��Ԥ��������ͼ������ɸѡ�������
// ATL ��Ŀ�н��ж��壬�����������Ŀ�����ĵ����롣
#ifndef SHARED_HANDLERS
#include "3DBodyReconstruction.h"
#endif

#include "3DBodyReconstructionDoc.h"
#include "3DBodyReconstructionView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CMy3DBodyReconstructionView

IMPLEMENT_DYNCREATE(CMy3DBodyReconstructionView, CView)

BEGIN_MESSAGE_MAP(CMy3DBodyReconstructionView, CView)
	// ��׼��ӡ����
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CView::OnFilePrintPreview)
END_MESSAGE_MAP()

// CMy3DBodyReconstructionView ����/����

CMy3DBodyReconstructionView::CMy3DBodyReconstructionView()
{
	// TODO: �ڴ˴���ӹ������

}

CMy3DBodyReconstructionView::~CMy3DBodyReconstructionView()
{
}

BOOL CMy3DBodyReconstructionView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: �ڴ˴�ͨ���޸�
	//  CREATESTRUCT cs ���޸Ĵ��������ʽ

	return CView::PreCreateWindow(cs);
}

// CMy3DBodyReconstructionView ����

void CMy3DBodyReconstructionView::OnDraw(CDC* /*pDC*/)
{
	CMy3DBodyReconstructionDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: �ڴ˴�Ϊ����������ӻ��ƴ���
}


// CMy3DBodyReconstructionView ��ӡ

BOOL CMy3DBodyReconstructionView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// Ĭ��׼��
	return DoPreparePrinting(pInfo);
}

void CMy3DBodyReconstructionView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: ��Ӷ���Ĵ�ӡǰ���еĳ�ʼ������
}

void CMy3DBodyReconstructionView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: ��Ӵ�ӡ����е��������
}


// CMy3DBodyReconstructionView ���

#ifdef _DEBUG
void CMy3DBodyReconstructionView::AssertValid() const
{
	CView::AssertValid();
}

void CMy3DBodyReconstructionView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CMy3DBodyReconstructionDoc* CMy3DBodyReconstructionView::GetDocument() const // �ǵ��԰汾��������
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CMy3DBodyReconstructionDoc)));
	return (CMy3DBodyReconstructionDoc*)m_pDocument;
}
#endif //_DEBUG


// CMy3DBodyReconstructionView ��Ϣ�������
