#include "cgal_adapter.h"
#include "TMesh.h"
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>

#include <CGAL/IO/OBJ_reader.h>
#include <CGAL/IO/File_writer_wavefront.h>
#include <CGAL/Surface_mesh/IO.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

namespace cgal_adapter
{
    namespace PMP = CGAL::Polygon_mesh_processing;
    bool fill_hole(TMesh *tmesh)
    {
        CMesh cmesh;
        create_cmesh(*tmesh, cmesh);

        // Incrementally fill the holes
        unsigned int nb_holes = 0;
        for (CHd h : cmesh.halfedges())
        {
            if (cmesh.is_border(h))
            {
                // M_DEBUG << "is_border " << h << endl;
                std::vector<CFd> patch_facets;
                std::vector<CVd> patch_vertices;
                bool success = CGAL::cpp11::get<0>(
                    CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(
                        cmesh,
                        h,
                        std::back_inserter(patch_facets),
                        std::back_inserter(patch_vertices),
                        CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, cmesh)).geom_traits(K())));
                // std::cout << " Number of facets in constructed patch: " << patch_facets.size() << std::endl;
                // std::cout << " Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
                // std::cout << " Fairing : " << (success ? "succeeded" : "failed") << std::endl;
                ++nb_holes;
            }
        }
        // std::cout << std::endl;
        // std::cout << nb_holes << " holes have been filled" << std::endl;
        if (nb_holes >= 1)
        {
            create_tmesh(cmesh, *tmesh);
            return true;
        }
        return false;
    }

    void read_obj(const std::string fname, TMesh &tmesh)
    {
        CMesh cmesh;
        read_obj(fname, cmesh);
        create_tmesh(cmesh, tmesh);
    }

    void read_obj(const std::string fname, CMesh &cmesh)
    {
        std::ifstream input(fname.c_str());
        std::vector<Point> points_ref;
        std::vector<std::vector<std::size_t>> faces_ref;
        if (!input || !CGAL::read_OBJ(input, points_ref, faces_ref))
        {
            return;
        }

        PMP::orient_polygon_soup(points_ref, faces_ref); // optional if your mesh is not correctly oriented
        PMP::polygon_soup_to_polygon_mesh(points_ref, faces_ref, cmesh);
        // cout << "input mesh: vertices=" << cmesh.number_of_vertices() << " faces=" << cmesh.number_of_faces() << endl;
    }

    void write_obj(const std::string fname, const TMesh &tmesh)
    {
        CMesh cmesh;
        create_cmesh(tmesh, cmesh);
        write_obj(fname, cmesh);
    }

    void write_obj(const std::string fname, const CMesh &mesh)
    {
        std::ofstream out(fname.c_str());
        // std::cout << "Mesh written to: " << fname << std::endl;
        CGAL::File_writer_wavefront writer;
        CGAL::generic_print_surface_mesh(out, mesh, writer);
    }

    void write_off(const std::string fname, const CMesh &cmesh)
    {
        std::ofstream out(fname.c_str());
        // std::cout << "Mesh written to: " << fname << std::endl;
        out.precision(17);
        out << cmesh << std::endl;
    }

    void create_cmesh(const TMesh &mesh, CMesh &cmesh)
    {
        std::unordered_map<int, CVd> vmap; // key: old vertex, value: new vertex
        for (Vd vd : mesh.vertices())
        {
            const Pnt3 &p = mesh.point(vd);
            CVd cvd = cmesh.add_vertex(Point(p[0], p[1], p[2]));
            vmap[vd.idx()] = cvd;
            // M_DEBUG << vd << " " << cvd << " point=" << cmesh.point(cvd) << endl;
        }

        for (Fd fd : mesh.faces())
        {
            Hd hd = mesh.halfedge(fd);
            Vd v0 = mesh.source(hd);
            Vd v1 = mesh.target(hd);
            Vd v2 = mesh.target(mesh.next(hd));

            CFd cfd = cmesh.add_face(vmap[v0.idx()], vmap[v1.idx()], vmap[v2.idx()]);
        }
    }

    void create_tmesh(const CMesh &cmesh, TMesh &tmesh)
    {
        tmesh.clear();

        for (CVd cvd : cmesh.vertices())
        {
            const Point &p = cmesh.point(cvd);
            Vd vd = tmesh.add_vertex(Pnt3(p[0], p[1], p[2]));
        }

        for (CFd cfd : cmesh.faces())
        {
            CHd chd = cmesh.halfedge(cfd);
            CVd cv0 = cmesh.source(chd);
            CVd cv1 = cmesh.target(chd);
            CVd cv2 = cmesh.target(cmesh.next(chd));
            Fd fd = tmesh.add_face(Vd(cv0.idx()), Vd(cv1.idx()), Vd(cv2.idx()));
        }
        tmesh.GetVMat();
        tmesh.GetFMat();
    }

    int connected_components(TMesh *tmesh)
    {
        CMesh cmesh;
        create_cmesh(*tmesh, cmesh);

        boost::unordered_map<boost::graph_traits<CMesh>::face_descriptor, int> cc(cmesh.number_of_faces());
        std::size_t num_component = PMP::connected_components(cmesh, boost::make_assoc_property_map(cc));
        return num_component;
    }
}