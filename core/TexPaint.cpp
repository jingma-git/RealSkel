#include "TexPaint.h"

TexPaint::TexPaint(void)
{
    m_vFaceInfo.clear();
    m_vImageInTexIdx.clear();
    m_vTexTriangleIdx.clear();
}

TexPaint::~TexPaint(void)
{
    m_vFaceInfo.clear();
    m_vImageInTexIdx.clear();
    m_vTexTriangleIdx.clear();
}

void TexPaint::InitTexPaint(int fSize, CPoint blockPos)
{
    //clear information
    m_vFaceInfo.clear();
    m_vImageInTexIdx.clear();
    m_vTexTriangleIdx.clear();

    //initial
    int i, j;
    Float scale = 1.0 / (Float)m_texWidth;
    Float width0 = m_blockWidth * scale;
    Float width1 = (m_blockWidth - 1) * scale;
    Pnt2 uvBase = Pnt2((blockPos.x() + 0.5) * scale, (blockPos.y() + 0.5) * scale);
    Pnt2 uvTmp;

    vector<int> fids;
    fids.resize(fSize);
    m_vFaceInfo.resize(fSize);
    for (i = 0; i < fSize; i++)
    {
        FaceInfo &faceInfo = m_vFaceInfo[i];
        faceInfo.blockPos = blockPos;
        faceInfo.idxImage = 0; //triangle in texture image 0 at first
        faceInfo.treatFlag = false;

        uvTmp = uvBase;
        for (j = 0; j < 3; j++)
        {
            faceInfo.uv[0 + 4 * j] = uvTmp;
            faceInfo.uv[1 + 4 * j] = Pnt2(uvTmp.x(), uvTmp.y() + width1);
            faceInfo.uv[2 + 4 * j] = Pnt2(uvTmp.x() + width1, uvTmp.y() + width1);
            faceInfo.uv[3 + 4 * j] = Pnt2(uvTmp.x() + width1, uvTmp.y());
            uvTmp.x() += width0;
        }

        fids[i] = i;
    }

    m_vTexTriangleIdx.push_back(fids);

    m_painterWidth = 1.0;

    m_iNumDrawFace = 0;

    m_iNumTexImage = 1;
    m_vImageInTexIdx.push_back(0); // all toys's first texture is tex 0 in scene

    m_iFillColorIdx = 0;
    m_iPrimeColorIdx = -1;
}

void TexPaint::TransDrawFacePos(int fid)
{
    FaceInfo &faceInfo = m_vFaceInfo[fid];

    int numDrawFace = m_iNumDrawFace % ((m_blockSize / 3) * m_blockSize);
    if (numDrawFace == 0) //trans to new texture image
    {
        m_iNumTexImage++;
    }

    //get block Position
    CPoint blockPos;

    blockPos = CPoint((numDrawFace % (m_blockSize / 3)) * m_blockWidth * 3, (numDrawFace / (m_blockSize / 3)) * m_blockWidth);

    faceInfo.blockPos = blockPos;
    faceInfo.idxImage = m_iNumTexImage - 1;

    //get new uv of the triangle
    Float scale = 1.0 / (Float)m_texWidth;
    Float width0 = m_blockWidth * scale;
    Float width1 = (m_blockWidth - 1) * scale;
    Pnt2 uvTmp = Pnt2((faceInfo.blockPos.x() + 0.5) * scale, (faceInfo.blockPos.y() + 0.5) * scale);

    for (int i = 0; i < 3; i++)
    {
        faceInfo.uv[0 + 4 * i] = uvTmp;
        faceInfo.uv[1 + 4 * i] = Pnt2(uvTmp.x(), uvTmp.y() + width1);
        faceInfo.uv[2 + 4 * i] = Pnt2(uvTmp.x() + width1, uvTmp.y() + width1);
        faceInfo.uv[3 + 4 * i] = Pnt2(uvTmp.x() + width1, uvTmp.y());
        uvTmp.x() += width0;
    }

    m_iNumDrawFace++;
}