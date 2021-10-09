#pragma once
#include "common.h"
#include "FreehandLine.h"
struct FaceInfo
{
    CPoint blockPos; //triangle block's position
    bool treatFlag;  //(false/true): (fill/draw) mode
    int idxImage;    //triangle in which texture image
    Pnt2 uv[12];     //uv baryCoord of 3 blocks

    Pnt3 p[7], norm[7]; //control pt and pt's norm of 3 blocks
};

struct ColorFace
{
    int fid;
    int colorIdx;
};

struct DrawBlock
{
    bool drawFlag;
    vector<CPoint> seeds;   //seeds for search
    vector<bool> pixelFlag; //size = blockWidth*blockWidth
};

struct DrawFace
{
    int fid;
    DrawBlock block[3];
};

struct FillFaceOfVert
{
    int vid;
    vector<int> vFid;
};

struct FillFaceSeed
{
    int fid;
    int blockIdx;
    vector<CPoint> seeds;
};

class TexPaint
{

public:
    TexPaint(void);

public:
    ~TexPaint(void);

public:
    void InitTexPaint(int fSize, CPoint blockPos);
    void TransDrawFacePos(int fid); //translate draw face position
    int GetColorIdx(int fid){return 0;}    //get color idx in color blocks

public:
    Float m_painterWidth;        //painter width
    FreehandLine m_texPaintLine; //draw texture paint by the line

    int m_texWidth;   //texture image width, height is the same
    int m_blockWidth; //triangle block width, height is the same
    int m_blockSize;  //triangle block size in a row

    int m_iNumDrawFace;           //the number of faces that mouse move on
    vector<FaceInfo> m_vFaceInfo; //face information for texture paint

    int m_iNumTexImage;           //number of texture image
    vector<int> m_vImageInTexIdx; //texture image idx in XToy's texture array

    int m_iFillColorIdx;  //the current fill color block idx
    int m_iPrimeColorIdx; //the prime color(color of the mouse on ) block idx

    vector<vector<int>> m_vTexTriangleIdx; //compute by m_vInTexImageIdx, collect all triangles in same image to a array, all size = fSize
};