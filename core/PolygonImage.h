#pragma once
#include "common.h"
#include "TMesh.h"

class PolygonImage
{
public:
    int m_polygonMode;         //polygon create mode --- 0: rectangle, 1: ellipse, 2: free hand
    vector<Pnt3> m_polygonPos; //original position
    vector<Pnt2> m_polygonST;  //st of each polygon vert
    Pnt3 m_polygonCenter;

    TMesh m_triMesh;

    Pnt3 m_polygonTrans;
    float m_polygonScale;
    float m_polygonRotate;
    //PixelImage			m_sourceImage;
    PolygonImage()
    {
        m_polygonPos.clear();
        m_polygonST.clear();
        m_triMesh.clear();

        m_polygonMode = 0;
        m_polygonCenter.Set(0, 0, 0);
        m_polygonTrans.Set(0, 0, 0);
        m_polygonScale = 1.0;
        m_polygonRotate = 0.0;
    }

    void clear()
    {
        m_polygonPos.clear();
        m_polygonST.clear();
        m_triMesh.clear();

        m_polygonMode = 0;
        m_polygonCenter.Set(0, 0, 0);
        m_polygonTrans.Set(0, 0, 0);
        m_polygonScale = 1.0;
        m_polygonRotate = 0.0;
    }
};