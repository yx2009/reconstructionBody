
// 3DBodyReconstructionView.h : CMy3DBodyReconstructionView ��Ľӿ�
//

#pragma once


class CMy3DBodyReconstructionView : public CView
{
protected: // �������л�����
	CMy3DBodyReconstructionView();
	DECLARE_DYNCREATE(CMy3DBodyReconstructionView)

// ����
public:
	CMy3DBodyReconstructionDoc* GetDocument() const;

// ����
public:

// ��д
public:
	virtual void OnDraw(CDC* pDC);  // ��д�Ի��Ƹ���ͼ
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// ʵ��
public:
	virtual ~CMy3DBodyReconstructionView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// ���ɵ���Ϣӳ�亯��
protected:
	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // 3DBodyReconstructionView.cpp �еĵ��԰汾
inline CMy3DBodyReconstructionDoc* CMy3DBodyReconstructionView::GetDocument() const
   { return reinterpret_cast<CMy3DBodyReconstructionDoc*>(m_pDocument); }
#endif

