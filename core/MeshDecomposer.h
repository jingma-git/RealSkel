#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

class MeshDecomposer
{
public:
    typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    typedef Kernel::Point_3 Point_3;
    typedef Kernel::Vector_3 Vector_3;

    typedef CGAL::Surface_mesh<Point_3> Mesh;

    typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
    typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
    typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;

    MeshDecomposer(Mesh &mesh_);

    void segmentation();
    void find_submesh_boundaries();
    void fill_hole();
    const std::vector<Mesh> &get_segmented_meshes() const { return segmented_meshes; }

private:
    Mesh &mesh;
    std::vector<Mesh> segmented_meshes;
};
