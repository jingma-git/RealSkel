#pragma once
#include "TMesh.h"

// CDTSKel works good for Cylinder-like structure but not for Octopus-like strucutre
class CDTSkel
{
public:
    CDTSkel(TMesh &mesh_, vector<Fd> &jFaces_, const vector<C3DLine> &rawLines_);

    void skeletonize(vector<C3DLine> &lines);

private:
    void filter_tiny_faces(vector<Fd> &faces);
    void build_regions();
    int find_longest_line();
    double get_ln_len(const C3DLine &line);

    void reverse_order(C3DLine &line);
    bool is_term_ln(const C3DLine &line);
    bool is_junc_ln(const C3DLine &line);
    void get_jt_lines(const Fd &fd, vector<int> &juncLs, vector<int> &termLs); //j:junction , t: terminal

    bool is_covered_by_circle(const C3DLine &line, C3DLine &pruedLn);
    bool is_pnts_in_circle(const vector<Pnt3> &pnts, const Pnt3 &cent, const double rad);

    // lines that connect one region to the other
    void search(vector<C3DLine> &lines);
    void search_from_junc_regions(const vector<int> &junc_rs, vector<C3DLine> &lines);
    void search_btw_two_term_regions(const vector<int> &term_rs, vector<C3DLine> &lines);
    Fd get_cf(const Region &region, const C3DLine &border_ln);
    void copy_ipts(const C3DLine &ln, C3DLine &line);

    TMesh &cdtMesh;
    const vector<C3DLine> &rawLines;
    map<Fd, double> fAreas;
    map<Fd, vector<int>> fLines; // value: rawLines idxs

    vector<Fd> jFaces; // Junction Faces after filter out tiny faces
    vector<Region> regions;
    vector<int> junc_regions, term_regions, trans_regions;
    vector<C3DLine> mLines;
    vector<C3DLine> regionLns; // line that connects two regions (index in mLines)
};