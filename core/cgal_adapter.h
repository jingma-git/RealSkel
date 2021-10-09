#pragma once

class TMesh;

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

namespace cgal_adapter
{

    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::Point_3 Point;
    typedef CGAL::Surface_mesh<Point> CMesh; // CGAL Surface mesh

    typedef boost::graph_traits<CMesh>::halfedge_descriptor CHd;
    typedef boost::graph_traits<CMesh>::face_descriptor CFd;
    typedef boost::graph_traits<CMesh>::vertex_descriptor CVd;

    bool fill_hole(TMesh *tmesh);

    void read_obj(const std::string fname, TMesh &tmesh);
    void read_obj(const std::string fname, CMesh &mesh);
    void write_obj(const std::string fname, const TMesh &tmesh);
    void write_obj(const std::string fname, const CMesh &mesh);
    void write_off(const std::string fname, const CMesh &cmesh);

    void create_cmesh(const TMesh &mesh, CMesh &cmesh);

    void create_tmesh(const CMesh &cmesh, TMesh &tmesh);

    int connected_components(TMesh *tmesh);
};