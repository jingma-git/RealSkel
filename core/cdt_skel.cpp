#include "cdt_skel.h"
#include <queue>

// I forget to add setedges

CDTSkel::CDTSkel(TMesh &mesh_, vector<Fd> &jFaces_, const vector<C3DLine> &rawLines_)
    : cdtMesh(mesh_),
      rawLines(rawLines_)
{
    for (const Fd &fd : jFaces_)
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

    for (const C3DLine &ln : rawLines)
    {
        mLines.push_back(ln);
    }

    filter_tiny_faces(jFaces_);
}

void CDTSkel::filter_tiny_faces(vector<Fd> &faces)
{
    std::sort(faces.begin(), faces.end(),
              [this](Fd a, Fd b) {
                  return fAreas[a] > fAreas[b];
              });
    cout << __FILE__ << " " << __LINE__ << " filter_tiny_faces..." << endl;

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
    double maxArea = fAreas[faces[0]];
    double threshArea = 2 * avgArea;
    if (fabs(avgArea - realAvgArea) < 5.0) // triangles are divided uniformly
    {
        threshArea = avgArea;
    }

    if (maxArea / avgArea > 30)
    {
        threshArea = 2.5 * avgArea;
    }
    cout << __FILE__ << " " << __LINE__ << " avgArea=" << avgArea
         << " maxArea=" << maxArea
         << " threshArea=" << threshArea << " realAvgArea=" << realAvgArea << endl;

    for (const Fd &fd : faces)
    {
        if (fAreas[fd] > threshArea)
        {
            cout << __FILE__ << " " << __LINE__ << " large " << fd << " area=" << fAreas[fd] << endl;
            jFaces.push_back(fd);
        }
        else
        {
            cout << __FILE__ << " " << __LINE__ << " filter out tiny " << fd << " area=" << fAreas[fd] << endl;
            cdtMesh.SetFType(fd, TERMINAL); // My improvement: This is really important!!!
        }
    }
    cout << __FILE__ << " " << __LINE__ << " filter_tiny_faces jFaces.size()=" << jFaces.size() << endl;
}

void CDTSkel::skeletonize(vector<C3DLine> &lines)
{
    if (jFaces.size() <= 1)
    {
        Fd fd = jFaces[0];
        std::sort(fLines[fd].begin(), fLines[fd].end(),
                  [this](int l0, int l1) {
                      return get_ln_len(rawLines[l0]) > get_ln_len(rawLines[l1]);
                  });
        for (const int &l : fLines[fd])
        {
            cout << __FILE__ << " " << __LINE__ << " " << rawLines[l] << " len=" << get_ln_len(rawLines[l]) << endl;
        }
        int l0 = fLines[fd][0];
        int l1 = fLines[fd][1];
        int l2 = fLines[fd][2];
        double len0 = get_ln_len(rawLines[l0]);
        double len1 = get_ln_len(rawLines[l1]);
        double len2 = get_ln_len(rawLines[l2]);
        if (len0 / len1 <= 2.0 && len0 / len2 > 10.0)
        {

            C3DLine line;
            C3DLine &ln0 = mLines[l0];
            C3DLine &ln1 = mLines[l1];
            if (ln0.Getcf(0) == fd)
            {
                reverse_order(ln0);
            }
            if (ln1.Getcf(0) != fd)
            {
                reverse_order(ln1);
            }
            line.Setcf(ln0.Getcf(0), 0);
            line.Setcpt(ln0.Getcpt(0), 0);
            copy_ipts(ln0, line);
            line.Setcf(ln1.Getcf(1), 1);
            line.Setcpt(ln1.Getcpt(1), 1);
            lines.push_back(line);
        }
        else if (fabs(len0 / len1 - 1.0) < 0.2 && fabs(len0 / len1 - 1.0) < 0.4) // three equal-large thin regions
        {
            lines.push_back(mLines[l0]);
            lines.push_back(mLines[l1]);
            lines.push_back(mLines[l2]);
        }
        else
        {
            lines.push_back(rawLines[l0]);
        }
    }
    else
    {
        build_regions();
        search(lines);
    }
}

double CDTSkel::get_ln_len(const C3DLine &line)
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

int CDTSkel::find_longest_line()
{
    double maxLen = std::numeric_limits<double>::min();
    int maxIdx = -1;
    for (int i = 0; i < rawLines.size(); i++)
    {
        double len = get_ln_len(rawLines[i]);
        if (len > maxLen)
        {
            maxLen = len;
            maxIdx = i;
        }
    }
    return maxIdx;
}

void CDTSkel::build_regions()
{
    int cnt = 0;     // region numbers
    int thresh = 10; // short-wide regioin: how many triangles between two JUNCTION triangles
    std::map<Fd, bool> isFVisit;
    cout << __FILE__ << " " << __LINE__ << " build regions..." << endl;
    for (const Fd &fd : jFaces)
    {
        if (isFVisit[fd])
            continue;
        cout << __FILE__ << " " << __LINE__ << " visit " << fd << endl;

        std::vector<int> juncLs, termLs; // junction, terminal line indexs
        get_jt_lines(fd, juncLs, termLs);

        Region region;

        if (termLs.size() == 0)
        {
            cout << __FILE__ << " " << __LINE__ << " REGION_JUNCTION " << fd << endl;
            region.type = REGION_JUNCTION;
            // find all junction triangles connected by 'short' junction lines
            queue<Fd> Q;
            Q.push(fd);

            while (!Q.empty())
            {
                Fd curF = Q.front();
                vector<int> connLns, borderLns;
                for (const int &l : fLines[curF])
                {
                    C3DLine &ln = mLines[l];
                    if (ln.GetiptCount() <= thresh)
                    {
                        region.addConnLn(l);
                        connLns.push_back(l);
                        ln.Setcr(cnt, 0);
                        ln.Setcr(cnt, 1);
                    }
                    else
                    {
                        region.addBorderLn(l);
                        borderLns.push_back(l);
                        if (ln.Getcf(0) == curF)
                            ln.Setcr(cnt, 0);
                        else
                            ln.Setcr(cnt, 1);
                    }
                }

                region.addF(curF);
                Q.pop();
                isFVisit[curF] = true;

                for (const int &l : connLns)
                {
                    const C3DLine &ln = mLines[l];
                    Fd childF = (ln.Getcf(0) == curF ? ln.Getcf(1) : ln.Getcf(0));
                    if (!isFVisit[childF])
                    {
                        Q.push(childF);
                    }
                }
            }
        }
        else if (termLs.size() == 1)
        {
            region.type = REGION_TRANSTION;
            region.centF = fd;
            C3DLine &ln0 = mLines[juncLs[0]];
            ln0.Setcr(cnt, (ln0.Getcf(0) == fd ? 0 : 1));
            C3DLine &ln1 = mLines[juncLs[1]];
            ln1.Setcr(cnt, (ln1.Getcf(0) == fd ? 0 : 1));
            cout << __FILE__ << " " << __LINE__ << " REGION_TRANSTION " << fd
                 << " l0=" << ln0.Getcf(0) << "-" << ln0.Getcf(1)
                 << " rawLn0=" << rawLines[juncLs[0]].Getcf(0) << "-" << rawLines[juncLs[0]].Getcf(1)
                 << " l1=" << ln1.Getcf(0) << "-" << ln1.Getcf(1) << endl;
        }
        else if (termLs.size() == 2)
        {
            region.type = REGION_TERMINAL;
            region.centF = fd;
            C3DLine &ln = mLines[juncLs[0]];
            ln.Setcr(cnt, (ln.Getcf(0) == fd ? 0 : 1));
            cout << __FILE__ << " " << __LINE__ << " REGION_TERMINAL " << fd
                 << " l=" << ln.Getcf(0) << "-" << ln.Getcf(1)
                 << " rawLn=" << rawLines[juncLs[0]].Getcf(0) << "-" << rawLines[juncLs[0]].Getcf(1) << endl;
        }
        else
        {
            assert(termLs.size() == 3);
            region.type = REGION_ISLAND;
            region.centF = fd;
        }

        regions.push_back(region);
        cnt++;
    }
    cout << __FILE__ << " " << __LINE__ << " regions=" << regions.size() << endl;
}

bool CDTSkel::is_term_ln(const C3DLine &line)
{
    return cdtMesh.GetFType(line.Getcf(0)) == TERMINAL ||
           cdtMesh.GetFType(line.Getcf(1)) == TERMINAL;
}

bool CDTSkel::is_junc_ln(const C3DLine &line)
{
    return cdtMesh.GetFType(line.Getcf(0)) == JUNCTION &&
           cdtMesh.GetFType(line.Getcf(1)) == JUNCTION;
}

void CDTSkel::get_jt_lines(const Fd &fd, vector<int> &juncLs, vector<int> &termLs)
{

    juncLs.clear();
    termLs.clear();
    for (const int &l : fLines[fd])
    {
        const C3DLine &line = rawLines[l];
        if (is_term_ln(line))
        {
            termLs.push_back(l);
        }
        else
        {
            assert(is_junc_ln(line));
            juncLs.push_back(l);
        }
    }
}

void CDTSkel::search(vector<C3DLine> &lines)
{
    if (regions.size() == 1 && regions[0].type == REGION_ISLAND)
    {
        lines.push_back(rawLines[find_longest_line()]);
        return;
    }

    for (int i = 0; i < regions.size(); i++)
    {
        if (regions[i].type == REGION_JUNCTION)
        {
            junc_regions.push_back(i);
        }
        else if (regions[i].type == REGION_TRANSTION)
        {
            trans_regions.push_back(i);
        }
        else if (regions[i].type == REGION_TERMINAL)
        {
            term_regions.push_back(i);
        }
    }
    cout << __FILE__ << " " << __LINE__ << " junc_regions=" << junc_regions.size()
         << " term_regions=" << term_regions.size()
         << " trans_regions=" << trans_regions.size()
         << endl;

    if (junc_regions.size() == 0 && term_regions.size() == 2)
    {
        // if one term region is adjacent to a very large 'Transtion region'
        // make the transition region as terminal
        for (int i = 0; i < term_regions.size(); i++)
        {
            const Region &region = regions[term_regions[i]];
            vector<int> juncLs, termLs;
            get_jt_lines(region.centF, juncLs, termLs);
            C3DLine &ln = mLines[juncLs[0]];
            if (ln.GetiptCount() <= 10)
            {
                Fd fd = ln.Getcf(1);
                int ri = ln.Getcr(1);
                if (ln.Getcf(0) != region.centF)
                {
                    fd = ln.Getcf(0);
                    ri = ln.Getcr(0);
                }
                if (cdtMesh.Get3DFArea(region.centF) < 0.2 * cdtMesh.Get3DFArea(fd))
                {
                    cout << __FILE__ << " " << __LINE__ << " region.centF=" << region.centF
                         << " regions[ri]=" << regions[ri].centF << " fd=" << fd << endl;
                    cdtMesh.SetFType(region.centF, TERMINAL);
                    regions[ri].type = REGION_TERMINAL;
                    term_regions[i] = ri;
                }
            }
        }
        search_btw_two_term_regions(term_regions, lines);
    }
    else if (junc_regions.size() >= 1)
    {
        // search_from_junc_regions(junc_regions, lines); // this isn't suitable for octupus-like structure
    }
}

Fd CDTSkel::get_cf(const Region &region, const C3DLine &border_ln)
{
    for (const Fd &fd : region.faces)
    {
        if (border_ln.Getcf(0) == fd || border_ln.Getcf(1) == fd)
        {
            return fd;
        }
    }
}

void CDTSkel::search_from_junc_regions(const vector<int> &junc_rs, vector<C3DLine> &lines)
{
    cout << __FILE__ << " " << __LINE__ << " search_from_junc_regions" << endl;
    std::map<int, bool> isLVisit;

    for (const int &r : junc_rs)
    {
        Region region = regions[r];
        region.cent = cdtMesh.barycent(region.faces);
        cout << __FILE__ << " " << __LINE__ << " ....visit region " << r << endl;
        for (const int &l : region.borderLns)
        {
            if (isLVisit[l])
                continue;

            C3DLine line;
            bool isLastVisit = false;

            C3DLine &stLn = mLines[l];
            int stR = r;
            Fd stF = get_cf(region, stLn);
            if (stLn.Getcr(0) != stR)
            {
                assert(stLn.Getcf(0) != stF && "Control Region and Face must have same order!");
                reverse_order(stLn);
            }
            cout << __FILE__ << " " << __LINE__ << " ....visit borderLn " << stLn.Getcf(0) << "-" << stLn.Getcf(1) << endl;

            line.Setcr(stR, 0);
            line.Setcpt(region.cent, 0);
            line.Setcf(stF, 0);
            copy_ipts(stLn, line);

            int curL = l;
            int curR = stLn.Getcr(1);
            Fd curF = stLn.Getcf(1);
            do
            {
                cout << __FILE__ << " " << __LINE__ << " curF=" << curF << endl;
                vector<int> juncLs, termLs;
                get_jt_lines(curF, juncLs, termLs);
                isLVisit[curL] = true;

                if (regions[curR].type == REGION_TRANSTION)
                {
                    cout << __FILE__ << " " << __LINE__ << " visit REGION_TRANSTION" << curF << endl;
                    assert(juncLs.size() == 2 && "Face on TRANSITON region must have two junction lines!");
                    int nxtL;
                    if (juncLs[0] == curL)
                    {
                        nxtL = juncLs[1];
                    }
                    else
                    {
                        nxtL = juncLs[0];
                    }
                    C3DLine &ln = mLines[nxtL];
                    if (ln.Getcr(0) != curR)
                    {
                        assert(ln.Getcf(0) != curF && "Control Region and Face must have same order!");
                        reverse_order(ln);
                    }
                    copy_ipts(ln, line);
                    curL = nxtL;
                    curR = ln.Getcr(1);
                    curF = ln.Getcf(1);
                }
                else if (regions[curR].type == REGION_TERMINAL)
                {
                    cout << __FILE__ << " " << __LINE__ << " visit REGION_TERMINAL" << curF << endl;
                    assert(juncLs.size() == 1 && "TERMINAL region must have two terminal lines!");
                    assert(juncLs[0] == curL && "TERMINAL region's only JUNCTION_LINE should be added previously");
                    vector<C3DLine> pruTermLns;
                    for (const int &l : termLs)
                    {
                        C3DLine pruLn;
                        if (!is_covered_by_circle(mLines[l], pruLn))
                        {
                            pruTermLns.push_back(pruLn);
                        }
                    }
                    if (pruTermLns.size() == 1)
                    {
                        C3DLine &ln = pruTermLns[0];
                        if (ln.Getcf(0) != curF)
                        {
                            reverse_order(ln);
                        }
                        copy_ipts(ln, line);
                        line.Setcf(ln.Getcf(1), 1);
                        line.Setcpt(ln.Getcpt(1), 1);
                        line.Setcr(-1, 1); // End with Sleeve Triangle
                        cout << __FILE__ << " " << __LINE__ << " End with Sleeve Triangle " << line.Getcf(1) << endl;
                    }
                    else
                    {
                        C3DLine &ln = mLines[curL];
                        Fd endF = ln.Getcf(1);
                        int endR = ln.Getcr(1);
                        regions[endR].cent = cdtMesh.barycent(endF);
                        ln.Setcpt(regions[endR].cent, 1);
                        ln.Setcf(endF, 1);
                        ln.Setcr(endR, 1);
                        cout << __FILE__ << " " << __LINE__ << " End with TERMINAL Triangle " << line.Getcf(1) << endl;
                    }
                    isLastVisit = true;
                }
                else
                {
                    cout << __FILE__ << " " << __LINE__ << " visit REGION_JUNCTION" << curF << endl;
                    assert(regions[curR].type == REGION_JUNCTION && "MUST be a JUNCTION REGION!");
                    assert(juncLs.size() == 0 && "JUNCTION region must have three junction lines!");
                    C3DLine &ln = mLines[curL];
                    int endR = curR;
                    Fd endF = ln.Getcf(1);
                    Region &endRegion = regions[endR];
                    endRegion.cent = cdtMesh.barycent(endRegion.faces);
                    line.Setcf(endF, 1);
                    line.Setcpt(endRegion.cent, 1);
                    line.Setcr(endR, 1);
                    cout << __FILE__ << " " << __LINE__ << " End with REGION_JUNCTION " << line.Getcf(1) << endl;
                    isLastVisit = true;
                    regionLns.push_back(line); // do not know that this is used for
                }

            } while (!isLastVisit);
            lines.push_back(line);
        }
    }
}

void CDTSkel::search_btw_two_term_regions(const vector<int> &term_rs, vector<C3DLine> &lines)
{
    cout << __FILE__ << " " << __LINE__ << " search_btw_two_term_regions" << endl;
    C3DLine line;
    int i0 = term_rs[0];
    Region &r0 = regions[i0];
    vector<int> juncLs, termLs;
    get_jt_lines(r0.centF, juncLs, termLs);
    cout << __FILE__ << " " << __LINE__ << " centF=" << r0.centF << " termLs=" << termLs.size() << endl;
    for (const int &l : termLs)
    {
        cout << __FILE__ << " " << __LINE__ << ": " << mLines[l].Getcf(0) << "-" << mLines[l].Getcf(1) << endl;
    }
    assert(termLs.size() == 2 && "TERMINAL REGION must have two terminal lines!");
    vector<C3DLine> pruTermLns;
    for (const int &l : termLs)
    {
        C3DLine &ln = mLines[l];
        cout << __FILE__ << " " << __LINE__ << " ln=" << ln.Getcf(0) << "-" << ln.Getcf(1) << endl;
        if (ln.Getcf(0) != r0.centF) //terminal line should start from 'JUNCTION face' to terminal face
        {
            reverse_order(ln);
            cout << __FILE__ << " " << __LINE__ << " after reverse ln=" << ln.Getcf(0) << "-" << ln.Getcf(1) << endl;
        }
        C3DLine pruLn;
        if (!is_covered_by_circle(ln, pruLn))
        {
            pruTermLns.push_back(pruLn);
        }
    }

    if (pruTermLns.size() == 1 || pruTermLns.size() == 2)
    {
        C3DLine ln = pruTermLns[0];
        if (pruTermLns.size() == 2 && get_ln_len(pruTermLns[1]) > get_ln_len(ln))
        {
            ln = pruTermLns[1];
        }
        if (ln.Getcf(1) != r0.centF)
        {
            reverse_order(ln);
        }
        line.Setcf(ln.Getcf(0), 0);
        line.Setcpt(ln.Getcpt(0), 0);
        line.Setcr(-1, 0); // start ctrl region is a SLEEVE triangle
        copy_ipts(ln, line);
        cout << __FILE__ << " " << __LINE__ << " add pruLn " << line.Getcf(0) << "-" << line.Getcf(1)
             << " stPt=" << line.Getcpt(0).transpose() << " ipt0=" << line.m_ipt[0].transpose()
             << " endif=" << line.GetLastIf() << endl;
    }

    C3DLine &ln = mLines[juncLs[0]];
    if (ln.Getcf(0) != r0.centF)
    {
        reverse_order(ln);
    }

    if (pruTermLns.size() == 1)
    {
        copy_ipts(ln, line);
        cout << __FILE__ << " " << __LINE__ << " add juncLn " << ln.Getcf(0) << "-" << ln.Getcf(1)
             << " endif=" << line.GetLastIf() << endl;
    }
    else
    {
        line.Setcf(ln.Getcf(0), 0);
        line.Setcpt(ln.Getcpt(0), 0);
        line.Setcr(ln.Getcr(0), 0); // start ctrl region is a JUNCTION triangle
        copy_ipts(ln, line);
        cout << __FILE__ << " " << __LINE__ << " add juncLn " << ln.Getcf(0) << "-" << ln.Getcf(1)
             << " endif=" << line.GetLastIf() << endl;
    }

    Fd curF = ln.Getcf(1);
    int curR = ln.Getcr(1);
    int curL = juncLs[0];
    bool isLastVisit = false;
    do
    {
        cout << __FILE__ << " " << __LINE__ << " ...visit " << curF << endl;
        get_jt_lines(curF, juncLs, termLs);
        if (juncLs.size() == 2 && termLs.size() == 1)
        {
            assert(regions[curR].type == REGION_TRANSTION && "MUST be a TRANSITION REGION!");
            int nxtL;
            if (juncLs[0] == curL)
                nxtL = juncLs[1];
            else
                nxtL = juncLs[0];
            C3DLine &ln = mLines[nxtL];
            if (ln.Getcf(0) != curF)
            {
                reverse_order(ln);
            }
            copy_ipts(ln, line);
            cout << __FILE__ << " " << __LINE__ << " add transLn " << ln.Getcf(0) << "-" << ln.Getcf(1)
                 << " if=" << line.GetLastIf() << endl;
            curF = ln.Getcf(1);
            curL = nxtL;
            curR = ln.Getcr(1);
        }
        else
        {
            assert(termLs.size() == 2 && "MUST have two terminal lines!");
            assert(regions[curR].type == REGION_TERMINAL && "MUST be a TERMINAL REGION!");

            vector<C3DLine> pruTermLns;
            for (const int &l : termLs)
            {
                C3DLine &ln = mLines[l];
                cout << __FILE__ << " " << __LINE__ << " ln=" << ln.Getcf(0) << "-" << ln.Getcf(1) << endl;
                if (ln.Getcf(0) != curF)
                {
                    reverse_order(ln);
                    cout << __FILE__ << " " << __LINE__ << " after reverse ln=" << ln.Getcf(0) << "-" << ln.Getcf(1) << endl;
                }
                C3DLine pruLn;
                if (!is_covered_by_circle(ln, pruLn))
                {
                    pruTermLns.push_back(pruLn);
                }
            }
            if (pruTermLns.size() == 1 || pruTermLns.size() == 2)
            {
                C3DLine ln = pruTermLns[0];
                if (pruTermLns.size() == 2 && get_ln_len(pruTermLns[1]) > get_ln_len(ln))
                {
                    ln = pruTermLns[1];
                }
                cout << __FILE__ << " " << __LINE__ << " the longest pruTermLns " << ln.Getcf(0) << "-" << ln.Getcf(1)
                     << " curF=" << curF << endl;
                if (ln.Getcf(0) != curF)
                {
                    cout << __FILE__ << " " << __LINE__ << " before reverse " << ln.GetiptCount()
                         << " halfedges=" << ln.m_halfedges.size()
                         << " edges=" << ln.m_edges.size() << endl;
                    reverse_order(ln);
                    cout << __FILE__ << " " << __LINE__ << " reverse " << ln.Getcf(0) << "-" << ln.Getcf(1) << endl;
                }
                copy_ipts(ln, line);
                line.Setcf(ln.Getcf(1), 1);
                line.Setcpt(ln.Getcpt(1), 1);
                cout << __FILE__ << " " << __LINE__ << " before Setcr" << endl;
                line.Setcr(-1, 1); // end Region is a SLEEVE TRIANGLE
                cout << __FILE__ << " " << __LINE__ << " add lastPruLn " << line.Getcf(0) << "-" << line.Getcf(1) << endl;
            }
            else
            {
                C3DLine &ln = mLines[curL];
                line.Setcf(ln.Getcf(1), 1);
                line.Setcpt(ln.Getcpt(1), 1);
                line.Setcr(curR, 1); // end Region is REGION_TERMINAL
                cout << __FILE__ << " " << __LINE__ << " set Lastcf " << line.Getcf(0) << "-" << line.Getcf(1) << endl;
            }

            isLastVisit = true;
        }
    } while (!isLastVisit);

    lines.push_back(line);
}

void CDTSkel::copy_ipts(const C3DLine &ln, C3DLine &line)
{
    for (int i = 0; i < ln.GetiptCount(); i++)
    {
        line.Setif(ln.Getif(i));
        line.Setipt(ln.Getipt(i));
        line.SetHalfedges(ln.GetHalfedge(i));
        line.Setedges(ln.Getedge(i));
    }
}

void CDTSkel::reverse_order(C3DLine &line)
{
    C3DLine tmp = line;

    line.clear();
    line.Setcf(tmp.Getcf(1), 0);
    line.Setcr(tmp.Getcr(1), 0);
    line.Setcpt(tmp.Getcpt(1), 0);

    line.Setcf(tmp.Getcf(0), 1);
    line.Setcr(tmp.Getcr(0), 1);
    line.Setcpt(tmp.Getcpt(0), 1);
    for (int j = tmp.GetiptCount() - 1; j >= 0; j--)
    {
        line.Setif(tmp.Getif(j));
        line.Setipt(tmp.Getipt(j));
        line.SetHalfedges(tmp.GetHalfedge(j));
        line.Setedges(tmp.Getedge(j));
    }
}

bool CDTSkel::is_covered_by_circle(const C3DLine &line, C3DLine &pruedLn)
{
    // when caculating chordal axis, we start from the JUNCTION face
    // thus, order should be reversed to make it from TERMINAL face to JUNCTION face
    Pnt3 p0, p1;
    vector<Pnt3> pnts;
    int n = line.GetiptCount();
    Hd hd = line.GetHalfedge(n - 1);
    Vd v0 = cdtMesh.source(hd);
    Vd v1 = cdtMesh.target(hd);
    p0 = cdtMesh.point(v0);
    p1 = cdtMesh.point(v1);
    Pnt3 cent = 0.5 * (p0 + p1);
    double radius = (p0 - p1).norm() * 0.5;
    pnts.push_back(p0);
    pnts.push_back(p1);

    cout << __FILE__ << " " << __LINE__ << " radius=" << radius << ", r0=" << (p0 - cent).norm()
         << " r1=" << (p1 - cent).norm() << " p0=" << p0.transpose() << " p1=" << p1.transpose()
         << " v0=" << v0 << " v1=" << v1 << endl;
    if (!is_pnts_in_circle(pnts, cent, radius))
    {
        cout << __FILE__ << " " << __LINE__ << " pnts in circle" << endl;
        pruedLn = line;
        return false;
    }

    for (int i = n - 2; i >= 0; i--)
    {
        hd = line.GetHalfedge(i);
        v0 = cdtMesh.source(hd);
        v1 = cdtMesh.target(hd);
        p0 = cdtMesh.point(v0);
        p1 = cdtMesh.point(v1);
        cout << __FILE__ << " " << __LINE__ << "...........visit " << i << " " << v0 << "-" << v1 << endl;
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

            cout << __FILE__ << " " << __LINE__ << " stF=" << stF << " ipts=" << n << " edges=" << line.m_edges.size()
                 << " halfedges=" << line.m_halfedges.size() << endl;
            i = i - 1;
            for (; i >= 0; i--)
            {
                pruedLn.Setif(line.Getif(i));
                pruedLn.Setipt(line.Getipt(i));
                pruedLn.SetHalfedges(line.GetHalfedge(i));
                pruedLn.Setedges(line.Getedge(i));
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

bool CDTSkel::is_pnts_in_circle(const vector<Pnt3> &pnts, const Pnt3 &cent, const double rad)
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