#include "MeshDecomposer.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

#include <iostream>
#include <fstream>

namespace PMP = CGAL::Polygon_mesh_processing;

MeshDecomposer::MeshDecomposer(Mesh &mesh_) : mesh(mesh_)
{
    auto vnormals = mesh.add_property_map<vertex_descriptor, Vector_3>("v:normals", CGAL::NULL_VECTOR).first;
    auto fnormals = mesh.add_property_map<face_descriptor, Vector_3>("f:normals", CGAL::NULL_VECTOR).first;
    PMP::compute_normals(mesh, vnormals, fnormals);
}

void MeshDecomposer::segmentation()
{
    // calculate Shape Diameter Value
    typedef Mesh::Property_map<face_descriptor, double> Facet_double_map;
    Facet_double_map sdf_property_map;
    sdf_property_map = mesh.add_property_map<face_descriptor, double>("f:sdf").first;
    CGAL::sdf_values(mesh,
                     sdf_property_map,
                     2.0 / 3.0 * CGAL_PI, /*cone angle*/
                     25,                  /*number of rays*/
                     true /*post-process*/);

    // Segmentation
    // create a property-map for segment-ids
    typedef Mesh::Property_map<face_descriptor, std::size_t> Facet_int_map;
    Facet_int_map segment_property_map = mesh.add_property_map<face_descriptor, std::size_t>("f:sid").first;

    std::size_t number_of_segments = CGAL::segmentation_from_sdf_values(mesh,
                                                                        sdf_property_map,
                                                                        segment_property_map,
                                                                        3 /*initial cluster*/);
    // create segmented meshes
    typedef CGAL::Face_filtered_graph<Mesh> Filtered_graph;
    Filtered_graph segment_mesh(mesh, 0, segment_property_map);
    segmented_meshes.resize(number_of_segments);
    for (std::size_t id = 0; id < number_of_segments; ++id)
    {
        if (id > 0)
            segment_mesh.set_selected_faces(id, segment_property_map);
        std::cout << "Segment " << id << "'s area is : " << CGAL::Polygon_mesh_processing::area(segment_mesh) << std::endl;
        CGAL::copy_face_graph(segment_mesh, segmented_meshes[id]);

        std::ofstream os("output/decompose/Segment_" + std::to_string(id) + ".off");
        os << segmented_meshes[id];
    }
}

void MeshDecomposer::find_submesh_boundaries()
{
    // typedef std::pair<Point_3, Vector_3> PointVecPair;

    // auto vnormals = mesh.property_map("v:normals").first;

    // halfedge_descriptor stBndHd;
    // for (halfedge_descriptor hd : mesh.halfedges())
    // {
    //     if (mesh.is_border(hd))
    //     {
    //         stBndHd = hd;
    //         break;
    //     }
    // }
    // bnd_inds.push_back(source(stBndHd));

    // Hd hd = next(stBndHd);
    // while (hd != stBndHd)
    // {
    //     bnd_inds.push_back(source(hd));
    //     hd = next(hd);
    // }
}

void MeshDecomposer::fill_hole()
{
}