#pragma once

// ToDO: rename to CDTSkel
// Skeletonization for Cylinder-like component (Torso, Leg, Arm, Head, Tail)
#include "TMesh.h"

enum JunctionFaceType
{
    ZERO_TERMLINE_DISC, // Junction
    TWO_TERMLINE_DISC,  // at the cylinder end: Terminal
    ONE_TERMLINE_DISC,  // at 'trasition' point of cylinder: Sleeve
    NEEDLE,
    TINY
};

class CylinderSkel
{
public:
    CylinderSkel(TMesh &mesh_, vector<Fd> &jFaces_, const vector<C3DLine> &rawLines_);

    void skeletonization(vector<C3DLine> &lines); // one continous line that cross the cylinder-like structure

private:
    void label_junction_face_type();
    int find_longest_line(const Fd &fd); // return line index in rawLines
    int find_longest_line();
    bool IsTerminalLine(const C3DLine &line);
    bool IsJunctionLine(const C3DLine &line);
    void get_jt_lines(const Fd &fd, vector<int> &juncLs, vector<int> &termLs); //j:junction , t: terminal
    void prune();
    bool is_covered_by_circle(const C3DLine &line, C3DLine &pruedLn);
    void merge(vector<C3DLine> &lines);

    struct JunctionRegion
    {
        vector<Fd> faces;
    };

    TMesh &cdtMesh;
    vector<Fd> &jFaces; // Junction Faces
    vector<Fd> nonTinyFs;
    const vector<C3DLine> &rawLines;
    vector<C3DLine> rawLinesCp; //  changed by pruned lines
    map<Fd, JunctionFaceType> fTypes;
    map<Fd, double> fAreas;
    map<Fd, vector<int>> fLines;
    vector<int> pruLines;
    map<Fd, vector<int>> pruFLines;
};