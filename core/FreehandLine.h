#pragma once
#include "common.h"
class FreehandLine
{
public:
    FreehandLine(){};
    virtual ~FreehandLine(){};

public:
    vector<Pnt3> Pos2d; //2d pos
    vector<Pnt3> Pos3d; //3d pos
    vector<int> fid;    //face id array
    vector<int> eid;    //edge id array
    vector<Pnt3> uvw;   //barycentre array

public:
    void Clear()
    {
        Pos2d.clear();
        Pos3d.clear();
        fid.clear();
        eid.clear();
        uvw.clear();
    }
};
