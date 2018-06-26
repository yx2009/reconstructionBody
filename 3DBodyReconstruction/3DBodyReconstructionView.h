
// 3DBodyReconstructionView.h : CMy3DBodyReconstructionView 类的接口
//

#pragma once


class CMy3DBodyReconstructionView : public CView
{
protected: // 仅从序列化创建
	CMy3DBodyReconstructionView();
	DECLARE_DYNCREATE(CMy3DBodyReconstructionView)

// 特性
public:
	CMy3DBodyReconstructionDoc* GetDocument() const;

// 操作
public:

// 重写
public:
	virtual void OnDraw(CDC* pDC);  // 重写以绘制该视图
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// 实现
public:
	virtual ~CMy3DBodyReconstructionView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
protected:
	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // 3DBodyReconstructionView.cpp 中的调试版本
inline CMy3DBodyReconstructionDoc* CMy3DBodyReconstructionView::GetDocument() const
   { return reinterpret_cast<CMy3DBodyReconstructionDoc*>(m_pDocument); }
#endif

