
// 3DBodyReconstructionView.cpp : CMy3DBodyReconstructionView 类的实现
//

#include "stdafx.h"
// SHARED_HANDLERS 可以在实现预览、缩略图和搜索筛选器句柄的
// ATL 项目中进行定义，并允许与该项目共享文档代码。
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
	// 标准打印命令
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CView::OnFilePrintPreview)
END_MESSAGE_MAP()

// CMy3DBodyReconstructionView 构造/析构

CMy3DBodyReconstructionView::CMy3DBodyReconstructionView()
{
	// TODO: 在此处添加构造代码

}

CMy3DBodyReconstructionView::~CMy3DBodyReconstructionView()
{
}

BOOL CMy3DBodyReconstructionView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: 在此处通过修改
	//  CREATESTRUCT cs 来修改窗口类或样式

	return CView::PreCreateWindow(cs);
}

// CMy3DBodyReconstructionView 绘制

void CMy3DBodyReconstructionView::OnDraw(CDC* /*pDC*/)
{
	CMy3DBodyReconstructionDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: 在此处为本机数据添加绘制代码
}


// CMy3DBodyReconstructionView 打印

BOOL CMy3DBodyReconstructionView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// 默认准备
	return DoPreparePrinting(pInfo);
}

void CMy3DBodyReconstructionView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 添加额外的打印前进行的初始化过程
}

void CMy3DBodyReconstructionView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 添加打印后进行的清理过程
}


// CMy3DBodyReconstructionView 诊断

#ifdef _DEBUG
void CMy3DBodyReconstructionView::AssertValid() const
{
	CView::AssertValid();
}

void CMy3DBodyReconstructionView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CMy3DBodyReconstructionDoc* CMy3DBodyReconstructionView::GetDocument() const // 非调试版本是内联的
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CMy3DBodyReconstructionDoc)));
	return (CMy3DBodyReconstructionDoc*)m_pDocument;
}
#endif //_DEBUG


// CMy3DBodyReconstructionView 消息处理程序
