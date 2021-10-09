#include "cylinder_skel.h"

static bool is_pnts_in_circle(const vector<Pnt3> &pnts, const Pnt3 &cent, const double rad)
{
    for (const Pnt3 &p : pnts)
    {
        if ((p - cent).norm() > (rad + 0.1))
        {
            cout << __FILE__ << " " << __LINE__ << ": " << p.transpose() << " dis=" << (p - cent).norm()
                 << " rad=" << rad << endl;
            return false;
        }
    }
    return true;
}

static double GetLineLen(const C3DLine &line)
{
    double sum = 0;
    int num_ipts = line.GetiptCount();
    sum += (line.Getcpt(0) - line.Getipt(0)).norm();
    for (int i = 1; i < num_ipts; i++)
    {
        sum += (line.Getipt(i) - line.Getipt(i - 1)).norm();
    }
    sum += (line.Getipt(num_ipts - 1) - line.Getcpt(1)).norm();
    return sum;
};

static double GetLineArea(const C3DLine &line, TMesh &mesh)
{
    double sum = 0;
    for (int i = 1; i < line.GetiptCount(); i++)
    {
        sum += mesh.Get3DFArea(line.Getif(i));
    }
    return sum;
};

static void reverse_order(C3DLine &line)
{
    C3DLine tmp = line;

    line.clear();
    line.Setcf(tmp.Getcf(1), 0);
    line.Setcpt(tmp.Getcpt(1), 0);
    line.Setcf(tmp.Getcf(0), 1);
    line.Setcpt(tmp.Getcpt(0), 1);
    for (int j = tmp.GetiptCount() - 1; j >= 0; j--)
    {
        line.Setif(tmp.Getif(j));
        line.Setipt(tmp.Getipt(j));
    }
}

CylinderSkel::CylinderSkel(TMesh &mesh_, vector<Fd> &jFaces_, const vector<C3DLine> &rawLines_)
    : cdtMesh(mesh_),
      jFaces(jFaces_),
      rawLines(rawLines_)
{
    for (const Fd &fd : jFaces)
    {
        fAreas[fd] = cdtMesh.Get3DFArea(fd);
        for (int i = 0; i < rawLines.size(); i++)
        {
            const C3DLine &line = rawLines[i];
            if (line.Getcf(0) == fd || line.Getcf(1) == fd)
            {
                fLines[fd].push_back(i);
            }
        }
        assert(fLines[fd].size() == 3);
    }

    for (int i = 0; i < rawLines.size(); i++)
    {
        rawLinesCp.push_back(rawLines[i]);
    }
}

void CylinderSkel::skeletonization(vector<C3DLine> &lines)
{
    C3DLine line;
    if (jFaces.size() == 1)
    {
        cout << __FILE__ << " " << __LINE__ << " only 1 junction faces" << endl;
        line = rawLines[find_longest_line(jFaces[0])];
        lines.push_back(line);
        return;
    }

    // label junction face type
    label_junction_face_type();

    // handle some frequently appearing cases to accelarate
    if (nonTinyFs.size() == 0)
    {
        // the subpart is divideved uniformly so retain the longest line
        cout << __FILE__ << " " << __LINE__ << " nonTinyFs==0" << endl;
        line = rawLines[find_longest_line()];
        lines.push_back(line);
        return;
    }
    else if (nonTinyFs.size() == 1)
    {
        cout << __FILE__ << " " << __LINE__ << " nonTinyFs==1" << endl;
        line = rawLines[find_longest_line(nonTinyFs[0])];
        lines.push_back(line);
        return;
    }

    // if (nonTinyFs.size() == 2)
    // {
    //     // retain the line between two disc;
    //     Fd &fd0 = nonTinyFs[0];
    //     Fd &fd1 = nonTinyFs[1];
    //     if (fTypes[fd0] == TWO_TERMLINE_DISC && fTypes[fd1] == TWO_TERMLINE_DISC)
    //     {
    //         cout << __FILE__ << " " << __LINE__ << " retain the line between two term disc" << endl;
    //         int l0 = find_longest_line(fd0);
    //         int l1 = find_longest_line(fd1);
    //         if (l0 == l1)
    //         {
    //             line = rawLines[l0];
    //             lines.push_back(line);
    //             return;
    //         }
    //         else
    //         {
    //             const C3DLine &ln0 = rawLines[l0];
    //             const C3DLine &ln1 = rawLines[l1];
    //             if (ln0.Getcf(1) == ln1.Getcf(0))
    //             {
    //                 lines.push_back(ln0);
    //                 lines.push_back(ln1);
    //             }
    //             else
    //             {
    //                 lines.push_back(ln1);
    //                 lines.push_back(ln0);
    //             }
    //             return;
    //         }
    //     }
    // }

    // prune
    prune();
    merge(lines);
}

bool CylinderSkel::IsTerminalLine(const C3DLine &line)
{
    return cdtMesh.GetFType(line.Getcf(0)) == TERMINAL ||
           cdtMesh.GetFType(line.Getcf(1)) == TERMINAL;
}

bool CylinderSkel::IsJunctionLine(const C3DLine &line)
{
    return cdtMesh.GetFType(line.Getcf(0)) == JUNCTION &&
           cdtMesh.GetFType(line.Getcf(1)) == JUNCTION;
}

void CylinderSkel::get_jt_lines(const Fd &fd, vector<int> &juncLs, vector<int> &termLs)
{

    for (const int &l : fLines[fd])
    {
        const C3DLine &line = rawLines[l];
        if (IsJunctionLine(line))
        {
            juncLs.push_back(l);
        }
        else
        {
            assert(IsTerminalLine(line));
            termLs.push_back(l);
        }
    }
}

int CylinderSkel::find_longest_line(const Fd &fd)
{
    double maxLen = std::numeric_limits<double>::min();
    int maxIdx = -1;
    for (const int &l : fLines[fd])
    {
        double len = GetLineLen(rawLines[l]);
        if (len > maxLen)
        {
            maxLen = len;
            maxIdx = l;
        }
    }
    return maxIdx;
}

int CylinderSkel::find_longest_line()
{
    double maxLen = std::numeric_limits<double>::min();
    int maxIdx = -1;
    for (int i = 0; i < rawLines.size(); i++)
    {
        double len = GetLineLen(rawLines[i]);
        if (len > maxLen)
        {
            maxLen = len;
            maxIdx = i;
        }
    }
    return maxIdx;
}

void CylinderSkel::label_junction_face_type()
{
    std::sort(jFaces.begin(), jFaces.end(),
              [this](Fd a, Fd b) {
                  return fAreas[a] > fAreas[b];
              });

    double sum = 0;
    int num = 0;
    for (const Fd &fd : cdtMesh.faces())
    {
        if (cdtMesh.GetFType(fd) != JUNCTION)
        {
            sum += cdtMesh.Get3DFArea(fd);
            num++;
        }
    }
    double avgArea = sum / num;
    double realAvgArea = cdtMesh.Get3DArea() / cdtMesh.num_faces();
    double maxArea = fAreas[jFaces[0]];
    double threshArea = 2 * avgArea;
    if (fabs(avgArea - realAvgArea) < 5.0) // triangles are divided uniformly
    {
        threshArea = avgArea;
    }

    double threshLen = 15.0; // edge length that is ignorable
    cout << __FILE__ << " " << __LINE__ << " avgArea=" << avgArea
         << " maxArea=" << maxArea
         << " threshArea=" << threshArea << " realAvgArea=" << realAvgArea << endl;

    // set all tiny junction triangle as terminal triangle
    for (const Fd &fd : jFaces)
    {
        if (fAreas[fd] < threshArea)
        {
            cdtMesh.SetFType(fd, TERMINAL);
            fTypes[fd] = TINY;
            cout << fd << " Tiny" << endl;
        }
        else
        {
            nonTinyFs.push_back(fd);
        }
    }

    if (nonTinyFs.size() == 1)
    {
        return;
    }

    for (const Fd &fd : nonTinyFs)
    {
        std::vector<int> juncLs, termLs; // junction, terminal line indexs
        get_jt_lines(fd, juncLs, termLs);

        cout << __FILE__ << " " << __LINE__ << " " << fd << " area=" << fAreas[fd]
             << " juncLs=" << juncLs.size() << " termLs=" << termLs.size() << endl;
        // junction: area > avgArea && has two terminal lines && one junction line
        const Hd &hd = cdtMesh.halfedge(fd);
        const Hd &nxtHd = cdtMesh.next(hd);
        Pnt3 p0 = cdtMesh.point(cdtMesh.source(hd));
        Pnt3 p1 = cdtMesh.point(cdtMesh.target(hd));
        Pnt3 p2 = cdtMesh.point(cdtMesh.target(nxtHd));

        // sort edges by length
        double len[3];
        int edges[3] = {0, 1, 2};
        len[0] = (p0 - p1).norm();
        len[1] = (p1 - p2).norm();
        len[2] = (p2 - p0).norm();
        std::sort(edges, edges + 3, [&](int i, int j) {
            return len[i] > len[j];
        });
        cout << "sorted edge ";
        for (int i = 0; i < 3; i++)
        {
            cout << edges[i] << " edge_len=" << len[edges[i]] << " | ";
        }
        cout << endl;

        if (juncLs.size() == 2)
        {
            if (len[edges[2]] < threshLen)
            {
                fTypes[fd] = NEEDLE;
                cout << fd << " needle " << endl;
            }
            else
            {
                fTypes[fd] = ONE_TERMLINE_DISC;
                cout << fd << " ONE_TERMLINE_DISC" << endl;
            }
        }
        else if (juncLs.size() == 3)
        {
            fTypes[fd] = ZERO_TERMLINE_DISC;
            cout << fd << " three junc line DISC " << endl;
        }
        else if (termLs.size() == 2)
        {
            fTypes[fd] = TWO_TERMLINE_DISC;
            cout << fd << " two term line DISC " << endl;
        }
        else
        {
            cout << fd << " unrecognized!!!"
                 << " " << (termLs.size() == 3) << " " << (nonTinyFs.size() == 1) << endl;
            // assert(termLs.size() == 3 && nonTinyFs.size() == 1);
        }
        // else if (fAreas[fd] > threshArea &&
        //          fabs(len[edges[1]] - len[edges[2]]) < threshLen &&
        //          (len[edges[2]] / len[edges[0]]) > 0.5)
        // {
        //     // two shortest edges are equi-lateral, shortest_edge / longest_edge
        //     fTypes[fd] = DISC;
        //     cout << fd << " equi-lateral Disc" << endl;
        // }
        // else if (fAreas[fd] > threshArea && termLs.size() == 2)
        // {
        //     fTypes[fd] = DISC;
        //     cout << fd << " non-equi-lateral Disc" << endl;
        // }
        // else
        // {
        //     cout << "Unreconized!!!" << endl;
        // }
        cout << endl
             << endl;
    }
}

void CylinderSkel::prune()
{
    std::map<int, bool> isVisit;
    for (const Fd &fd : nonTinyFs)
    {
        std::vector<int> juncLs, termLs; // junction, terminal line indexs
        get_jt_lines(fd, juncLs, termLs);
        if (fTypes[fd] == NEEDLE || fTypes[fd] == ONE_TERMLINE_DISC)
        {
            cout << __FILE__ << " " << __LINE__ << (fTypes[fd] == NEEDLE ? " NEEDLE " : " ONE_TERMLINE_DISC ") << fd << endl;
            assert(juncLs.size() == 2 && "Needle or ONE_TERMINAL_DISC must have two junction lines!");
            pruFLines[fd].push_back(juncLs[0]);
            pruFLines[fd].push_back(juncLs[1]);
            if (!isVisit[juncLs[0]])
            {
                pruLines.push_back(juncLs[0]);
            }
            if (!isVisit[juncLs[1]])
            {
                pruLines.push_back(juncLs[1]);
            }
            isVisit[juncLs[0]] = true;
            isVisit[juncLs[1]] = true;
            cout << __FILE__ << " " << __LINE__ << " .......end " << fd << " " << pruFLines[fd].size() << endl;
        }
        else if (fTypes[fd] == TWO_TERMLINE_DISC)
        {
            cout << __FILE__ << " " << __LINE__ << " TWO_TERMLINE_DISC " << fd << endl;
            C3DLine pruedLine;
            int t0 = termLs[0];
            int t1 = termLs[1];
            const C3DLine &tl0 = rawLines[t0];
            const C3DLine &tl1 = rawLines[t1];
            if (!is_covered_by_circle(tl0, pruedLine))
            {
                rawLinesCp[t0] = pruedLine;
                pruFLines[fd].push_back(t0);
                if (!isVisit[t0])
                {
                    pruLines.push_back(t0);
                    isVisit[t0] = true;
                }

                cout << __FILE__ << " " << __LINE__ << " retain tl0=" << t0
                     << " rawLines[t0]=" << rawLines[t0].GetiptCount()
                     << " rawLinesCp[t0]=" << rawLinesCp[t0].GetiptCount() << endl;
            }

            if (!is_covered_by_circle(tl1, pruedLine))
            {
                rawLinesCp[t1] = pruedLine;
                pruFLines[fd].push_back(t1);
                if (!isVisit[t1])
                {
                    pruLines.push_back(t1);
                    isVisit[t1] = true;
                }
                cout << __FILE__ << " " << __LINE__ << " retain tl1=" << t1
                     << " rawLines[t1]=" << rawLines[t1].GetiptCount()
                     << " rawLinesCp[t1]=" << rawLinesCp[t1].GetiptCount() << endl;
            }

            pruFLines[fd].push_back(juncLs[0]);
            if (!isVisit[juncLs[0]])
            {
                pruLines.push_back(juncLs[0]);
                pruLines[juncLs[0]] = true;
            }

            cout << __FILE__ << " " << __LINE__ << " TWO_TERMLINE_DISC end " << fd << " " << pruFLines[fd].size() << endl;
        }
        else
        {
            cout << __FILE__ << " " << __LINE__ << " ZERO_TERMLINE_DISC " << fd << " " << juncLs.size() << endl;
            assert(fTypes[fd] == ZERO_TERMLINE_DISC && "ZERO_TERMLINE_DISC must have three juncLs!");
            // choose two longest line
            std::sort(juncLs.begin(), juncLs.end(), [this](int a, int b) {
                const C3DLine &l0 = rawLines[a];
                const C3DLine &l1 = rawLines[b];
                return GetLineLen(l0) > GetLineLen(l1);
            });
            pruFLines[fd].push_back(juncLs[0]);
            pruFLines[fd].push_back(juncLs[1]);
            if (!isVisit[juncLs[0]])
            {
                pruLines.push_back(juncLs[0]);
            }
            if (!isVisit[juncLs[1]])
            {
                pruLines.push_back(juncLs[1]);
            }
            isVisit[juncLs[0]] = true;
            isVisit[juncLs[1]] = true;
            cout << __FILE__ << " " << __LINE__ << " ZERO_TERMLINE_DISC " << fd << " " << pruFLines[fd].size() << endl;
        }
    }
}

bool CylinderSkel::is_covered_by_circle(const C3DLine &line, C3DLine &pruedLn)
{
    Pnt3 p0, p1;
    vector<Pnt3> pnts;
    int n = line.GetiptCount();
    Hd hd = line.GetHalfedge(n - 1);
    Hd nxtHd = cdtMesh.next(hd);
    p0 = cdtMesh.point(cdtMesh.source(hd));
    p1 = cdtMesh.point(cdtMesh.target(hd));
    Pnt3 cent = 0.5 * (p0 + p1);
    double radius = (p0 - p1).norm() * 0.5;
    pnts.push_back(p0);
    pnts.push_back(p1);

    cout << __FILE__ << " " << __LINE__ << " radius=" << radius << ", r0=" << (p0 - cent).norm()
         << " r1=" << (p1 - cent).norm() << " p0=" << p0.transpose() << " p1=" << p1.transpose() << endl;
    if (!is_pnts_in_circle(pnts, cent, radius))
    {
        cout << __FILE__ << " " << __LINE__ << " pnts in circle" << endl;
        pruedLn = line;
        return false;
    }

    for (int i = n - 2; i >= 0; i--)
    {
        hd = line.GetHalfedge(i);
        p0 = cdtMesh.point(cdtMesh.source(hd));
        p1 = cdtMesh.point(cdtMesh.target(hd));
        pnts.push_back(p0);
        pnts.push_back(p1);
        cent = 0.5 * (p0 + p1);
        radius = (p0 - p1).norm() * 0.5;

        if (!is_pnts_in_circle(pnts, cent, radius))
        {
            cout << __FILE__ << " " << __LINE__ << " ............" << i << endl;
            cout << __FILE__ << " " << __LINE__ << " pnts not in circle " << i << endl;
            const Fd &stF = line.Getif(i);
            cdtMesh.SetFType(stF, TERMINAL);
            pruedLn.Setcf(stF, 0);
            pruedLn.Setcpt(cent, 0);

            cout << __FILE__ << " " << __LINE__ << " stF=" << stF << endl;
            i = i - 1;
            for (; i >= 0; i--)
            {
                pruedLn.Setif(line.Getif(i));
                pruedLn.Setipt(line.Getipt(i));
            }
            const Fd &endF = line.Getcf(0);
            cout << __FILE__ << " " << __LINE__ << " endF=" << endF << endl;
            pruedLn.Setcf(endF, 1);
            pruedLn.Setcpt(line.Getcpt(0), 1);
            assert(cdtMesh.GetFType(line.Getcf(0)) == JUNCTION && "The other end must be a JUNCTION Face");
            return false;
        }
    }
    return true;
}

void CylinderSkel::merge(vector<C3DLine> &lines)
{
    cout << __FILE__ << " " << __LINE__ << " merging" << endl;
    std::map<int, bool> isVisit;
    vector<Fd> termFs; // select TWO-TERM-DISC as start
    for (const Fd &fd : nonTinyFs)
    {
        if (fTypes[fd] == TWO_TERMLINE_DISC)
        {
            termFs.push_back(fd);
        }
    }
    assert(termFs.size() == 2 && "There must be at least two TWO_TERMLINE_DISC");
    Fd stF;
    if (cdtMesh.Get3DFArea(termFs[0]) > cdtMesh.Get3DFArea(termFs[1]))
    {
        stF = termFs[0];
    }
    else
    {
        stF = termFs[1];
    }

    bool isLastVisit = false;
    vector<int> orderedLns;
    std::map<int, bool> isLnVisit;
    std::map<Fd, bool> isFVisit;

    Fd curF = stF;
    vector<int> fLns = pruFLines[curF];
    cout << __FILE__ << " " << __LINE__ << " visit " << curF << endl;
    if (fLns.size() == 1)
    {
        C3DLine &l = rawLinesCp[fLns[0]];
        cout << __FILE__ << " " << __LINE__ << " fLns.size() == 1 " << l.Getcf(0) << "-" << l.Getcf(1) << endl;
        orderedLns.push_back(fLns[0]);
        isLnVisit[fLns[0]] = true;
        if (l.Getcf(1) == curF)
        {
            reverse_order(l);
        }

        curF = l.Getcf(1);
    }
    else
    {
        assert(fLns.size() == 2);
        const C3DLine &l0 = rawLinesCp[fLns[0]];
        const C3DLine &l1 = rawLinesCp[fLns[1]];

        if (IsTerminalLine(l0))
        {
            cout << __FILE__ << " " << __LINE__ << " fLns.size() == 2 l0:"
                 << l0.Getcf(0) << "-" << l0.Getcf(1) << " l1:" << l1.Getcf(0) << "-" << l1.Getcf(1) << endl;
            orderedLns.push_back(fLns[0]);
            orderedLns.push_back(fLns[1]);
            isLnVisit[fLns[0]] = true;
            isLnVisit[fLns[1]] = true;
            curF = l1.Getcf(1);
        }
        else
        {
            cout << __FILE__ << " " << __LINE__ << " fLns.size() == 2 l0:"
                 << l0.Getcf(0) << "-" << l0.Getcf(1) << " l1:" << l1.Getcf(0) << "-" << l1.Getcf(1) << endl;
            orderedLns.push_back(fLns[1]);
            orderedLns.push_back(fLns[0]);
            isLnVisit[fLns[1]] = true;
            isLnVisit[fLns[0]] = true;
            curF = l0.Getcf(1);
        }
    }

    do
    {
        vector<int> fLns = pruFLines[curF];
        if (fTypes[curF] == NEEDLE || fTypes[curF] == ONE_TERMLINE_DISC || fTypes[curF] == ZERO_TERMLINE_DISC)
        {
            cout << __FILE__ << " " << __LINE__ << " visit " << curF << endl;
            assert(fLns.size() == 2 && "Must have two lines!");
            const C3DLine &l0 = rawLinesCp[fLns[0]];
            const C3DLine &l1 = rawLinesCp[fLns[1]];
            cout << __FILE__ << " " << __LINE__ << " fLns.size() == 2 l0:"
                 << l0.Getcf(0) << "-" << l0.Getcf(1) << " l1:" << l1.Getcf(0) << "-" << l1.Getcf(1) << endl;
            if (!isLnVisit[fLns[0]])
            {
                orderedLns.push_back(fLns[0]);
                isLnVisit[fLns[0]] = true;
                curF = l0.Getcf(1);
            }
            else
            {
                orderedLns.push_back(fLns[1]);
                isLnVisit[fLns[1]] = true;
                curF = l1.Getcf(1);
            }
        }
        else
        {
            cout << __FILE__ << " " << __LINE__ << " visit " << curF << endl;
            assert(fTypes[curF] == TWO_TERMLINE_DISC);
            if (fLns.size() == 1)
            {
                const C3DLine &l0 = rawLinesCp[fLns[0]];
                cout << __FILE__ << " " << __LINE__ << " fLns.size() == 1 " << l0.Getcf(0) << "-" << l0.Getcf(1) << endl;
                assert(isLnVisit[fLns[0]] && "This line must be added by previous iteration!");
                isLastVisit = true;
                break;
            }
            else
            {
                assert(fLns.size() == 2 && "TWO_TERMLINE_DISC has one/two lines!");
                const C3DLine &l0 = rawLinesCp[fLns[0]];
                const C3DLine &l1 = rawLinesCp[fLns[1]];
                if (!isLnVisit[fLns[0]])
                {
                    orderedLns.push_back(fLns[0]);
                    isLnVisit[fLns[0]] = true;
                    curF = l0.Getcf(1);
                }
                else
                {
                    orderedLns.push_back(fLns[1]);
                    isLnVisit[fLns[1]] = true;
                    curF = l1.Getcf(1);
                }
                isLastVisit = true;
                break;
            }
        }
        isFVisit[curF] = true;
    } while (isLastVisit);

    for (const int lidx : orderedLns)
    {
        lines.push_back(rawLinesCp[lidx]);
    }
}