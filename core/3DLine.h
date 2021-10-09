#pragma once
#include "common.h"

enum RegionType
{
    REGION_JUNCTION,  // zero-term line
    REGION_TRANSTION, // one-term line
    REGION_TERMINAL,  // two-term lines
    REGION_ISLAND     // three-term lines
};
struct Region
{
    void addF(Fd fd)
    {
        faces.push_back(fd);
    }

    void addConnLn(int l)
    {
        if (std::find(connLns.begin(), connLns.end(), l) == connLns.end())
            connLns.push_back(l);
    }

    void addBorderLn(int l)
    {
        borderLns.push_back(l);
    }

    RegionType type;
    Fd centF;
    Pnt3 cent;

    // REGION_JUNCTION, the region is faces connected by juncLns
    vector<Fd> faces;
    vector<int> connLns; // connection lines to traverse among faces among this region
    vector<int> borderLns;
};

class C3DLine
{
public:
    vector<Float> m_vOff; //elevate value

    Pnt3 m_cpt[2];               //start point and end point
    Fd m_cf[2];                  //two face where the start point and end point are located
    int m_cr[2];                 // two regions where the start point and end point are located: see cdt_skel
    IntersectionType m_cType[2]; // intersection types: on_face, on_edge, on_vertex
    int m_cIdx[2];               // if inter_type==on_face m_iIdxs record face_idx, if inter_type==on_edge m_iIdxs record edge_idx

    vector<Hd> m_halfedges;            // halfedges that line goes through
    vector<Ed> m_edges;                // edges that the line goes through
    vector<Pnt3> m_ipt;                //intersection points btw the cutting line and edges
    vector<Fd> m_ifs;                  // the face which the cutting line lies, doesn't include start and end face of the cutting line
    vector<IntersectionType> m_iTypes; // intersection types: on_face, on_edge, on_vertex // ToDO: remove
    vector<int> m_iIdxs;               // if inter_type==on_face m_iIdxs record face_idx, if inter_type==on_edge m_iIdxs record edge_idx,

    vector<Pnt3> m_isectP; //intersection point of the cutting line
public:
    void clear()
    {
        m_cf[0] = SMesh::null_face();
        m_cf[1] = SMesh::null_face();
        m_cr[0] = -1;
        m_cr[1] = -1;

        m_halfedges.clear();
        m_edges.clear();
        m_ipt.clear();
        m_ifs.clear();
    }

    Pnt3 &Getcpt(int i) { return m_cpt[i]; }
    const Pnt3 &Getcpt(int i) const { return m_cpt[i]; }
    void Setcpt(Pnt3 cpt, int i) { m_cpt[i] = cpt; }

    Fd Getcf(int i) { return m_cf[i]; }
    const Fd &Getcf(int i) const { return m_cf[i]; }
    void Setcf(Fd fd, int i) { m_cf[i] = fd; }

    int Getcr(int i) { return m_cr[i]; }
    void Setcr(int region_idx, int i) { m_cr[i] = region_idx; }

    IntersectionType &GetcType(int i) { return m_cType[i]; }
    const IntersectionType &GetcType(int i) const { return m_cType[i]; }
    void SetcType(IntersectionType type, int i) { m_cType[i] = type; }

    int &GetcIdx(int i) { return m_cIdx[i]; }
    const int &GetcIdx(int i) const { return m_cIdx[i]; }
    void SetcIdx(int cIdx, int i) { m_cIdx[i] = cIdx; }

    int GetiptCount() const { return m_ipt.size(); }
    Pnt3 &Getipt(int i) { return m_ipt[i]; }
    const Pnt3 &Getipt(int i) const { return m_ipt[i]; }
    void Setipt(Pnt3 ipt) { m_ipt.push_back(ipt); }

    Fd &Getif(int i) { return m_ifs[i]; }
    const Fd &Getif(int i) const { return m_ifs[i]; }
    void Setif(Fd fd) { m_ifs.push_back(fd); }
    const Fd &GetLastIf() const { return m_ifs.back(); }

    Ed &Getedge(int i) { return m_edges[i]; }
    const Ed &Getedge(int i) const { return m_edges[i]; }
    void Setedges(Ed ed) { m_edges.push_back(ed); }
    int GetedgesCount() const { return m_edges.size(); }

    Hd &GetHalfedge(int i) { return m_halfedges[i]; }
    const Hd &GetHalfedge(int i) const { return m_halfedges[i]; }
    void SetHalfedges(Hd hd) { m_halfedges.push_back(hd); }
    int GetHalfedgesCount() const { return m_halfedges.size(); }

    IntersectionType &GetiType(int i) { return m_iTypes[i]; }
    const IntersectionType &GetiType(int i) const { return m_iTypes[i]; }
    void SetiType(IntersectionType iType) { m_iTypes.push_back(iType); }

    int &GetiIdx(int i) { return m_iIdxs[i]; }
    const int &GetiIdx(int i) const { return m_iIdxs[i]; }
    void SetiIdx(int iIdx) { m_iIdxs.push_back(iIdx); }

    C3DLine &operator=(const C3DLine &other)
    {
        m_cpt[0] = other.m_cpt[0];
        m_cpt[1] = other.m_cpt[1];
        m_cf[0] = other.m_cf[0];
        m_cf[1] = other.m_cf[1];
        m_cr[0] = other.m_cr[0];
        m_cr[1] = other.m_cr[1];

        for (int i = 0; i < other.m_ifs.size(); i++)
        {
            Setif(other.m_ifs[i]);
            Setipt(other.m_ipt[i]);
            SetHalfedges(other.m_halfedges[i]);
            Setedges(other.m_edges[i]);
        }
        return *this;
    }

    bool operator==(const C3DLine &rhs) const
    {
        if (Getcf(0) == rhs.Getcf(0) && Getcf(1) == rhs.Getcf(1))
        {
            if ((Getcpt(0) - rhs.Getcpt(0)).norm() < 1e-5 && (Getcpt(1) - rhs.Getcpt(1)).norm() < 1e-5)
            {
                if (GetiptCount() == rhs.GetiptCount())
                {
                    for (int i = 0; i < GetiptCount(); i++)
                    {
                        if (Getif(i) != rhs.Getif(i))
                        {
                            return false;
                        }
                        if ((Getipt(i) - rhs.Getipt(i)).norm() > 1e-5)
                        {
                            return false;
                        }
                    }
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    bool operator!=(const C3DLine &rhs) const
    {
        return !(*this == rhs);
    }
};

inline ostream &operator<<(ostream &os, const C3DLine &line)
{
    // os << "start from " << line.Getcf(0) << " point " << line.Getcpt(0) << endl;
    // for (size_t i = 0; i < line.m_edges.size(); i++)
    // {
    //     os << "intersect with " << line.m_edges[i] << " on point " << line.Getipt(i) << endl;
    // }
    // os << "end on " << line.Getcf(1) << " point " << line.Getcpt(1) << endl;
    // os << "totally have " << line.m_ifs.size() << " intersections" << endl;
    os << line.Getcf(0) << "-" << line.Getcf(1);
    return os;
}
