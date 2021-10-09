// std
#include <unordered_map>
// igl
#include <igl/readDMAT.h>
#include <igl/readMESH.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/writeDMAT.h>
#include <igl/writeMESH.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/writePLY.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_corner_normals.h>
#include <igl/pathinfo.h>
#include <igl/is_readable.h>
#include <igl/is_dir.h>
#include <igl/faces_first.h>
#include <igl/lbs_matrix.h>
#include <igl/bbw.h>
#include <igl/partition.h>
#include <igl/repmat.h>
#include <igl/columnize.h>
#include <igl/get_seconds.h>
#include <igl/slice.h>
#include <igl/normalize_row_sums.h>
#include <igl/cat.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/barycenter.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/piecewise_constant_winding_number.h>
#include <igl/unique_edge_map.h>
// boost
#include <boost/filesystem.hpp>
// egl
#include "../egl/time_str.h"
#include "../egl/writeMatrix.h"
#include "../egl/string_util.h"

// my
#include "XToy.h"
#include "XFunctions.h"
#include "straight_skeletor.h"
#include "sskel_to_bone.h"
#include "sskel_subtoy_info_io.h"
#include "cgal_adapter.h"
// Fast Automatic Skinning
#include "copy_bone_roots_data.h"
#include "sort_weights.h"
#include "gather_samples.h"
#include "boundary_conditions.h"
#include "gather_transformations.h"
#include "distribute_transformations.h"
#include "gather_free.h"
#include "gather_fixed.h"
#include "gather_positional_constraints_system.h"
#include "gather_positional_constraints_rhs.h"
#include "uncolumnize.h"

#define TEMP_DIR "./output/skinning/"

#define MAX_NUM_HANDLES 200
#define MAX_NUM_WEIGHTS_PER_VERTEX 16

#define MESH_NAME "mesh"
#define MESH2D_NAME "mesh2D"
#define WEIGHTS_NAME "weights"
#define BONE_ROOTS_NAME "bone_roots"
#define BBW_ITER 30
#define TEX_WIDTH 512
#define BLOCK_WIDTH 16

using namespace Eigen;
using namespace std;
namespace fs = boost::filesystem;

static void copy_sskel(const SSkel<SSNodeSub> &other, SSkel<SSNodeSub> &sskel)
{
    sskel.clear();
    typedef SSNodeSub NodeT;
    typedef SSEdge<NodeT> EdgeT;
    std::unordered_map<NodeT *, NodeT *> node_map;

    for (NodeT *node : other.nodes)
    {
        NodeT *newNode = sskel.add_node(node->pos);
        newNode->set_radius(node->radius);
        node_map[node] = newNode;
        newNode->set_subtoy(node->subToy);
    }

    for (EdgeT *edge : other.edges)
    {
        NodeT *n0 = edge->node0;
        NodeT *n1 = edge->node1;
        sskel.add_edge(node_map[n0], node_map[n1]);
    }
}

int XToy::NUM_INI_SUBTOYS = 16;

std::ostream &operator<<(std::ostream &out, const XToy &toy)
{
    out << "name:" << toy.m_name << "\n"
        << "num_subToys:" << toy.m_subToys.size() << "\n"
        << "curSubToyIdx:" << toy.m_curSubToyIdx << "\n"
        << "parentToyIdx:" << toy.m_parentToyIdx << "\n";
    return out;
}

std::istream &operator>>(std::istream &in, XToy &toy)
{
    std::string line, key, value;
    int position;
    while (std::getline(in, line))
    {
        if (line[0] == '#')
            continue;
        position = line.find(":");
        if (position == line.npos)
            continue;
        key = line.substr(0, position);
        value = line.substr(position + 1, line.length());
        egl::trim(value);
        if (key == "name")
        {
            toy.m_name = value;
        }
        else if (key == "num_subToys")
        {
            int n = std::stoi(value);
            toy.m_subToys.resize(n);
        }
        else if (key == "curSubToyIdx")
        {
            toy.m_curSubToyIdx = std::stoi(value);
        }
        else if (key == "parentToyIdx")
        {
            toy.m_parentToyIdx = std::stoi(value);
        }
    }
	return in;
}

XToy::XToy()
    : m_name(egl::current_time_str()),
      m_texWidth(TEX_WIDTH),
      m_blockWidth(BLOCK_WIDTH),
      anim_timer(0.0f),
      animating(false),
      animation_interp_secs(1.0),
      transition_type(EASE_TRANSITION),
      auto_dof(false),
      dial_in_each_T(false) //Only set to false now, ToDO: set as true, all bones do not move
{
    M_DEBUG << "toyName: " << m_name << endl;
    m_subtoyPool.SetBlockSize(NUM_INI_SUBTOYS);
    skel = new Skeleton<Bone>();
    restSkel = new Skeleton<Bone>();
}

XToy::~XToy()
{
    // Save current session to temp folder
    save(TEMP_DIR);
    delete skel;

    ClearSubToys();
    m_subtoyPool.Clear();
}

bool XToy::save(const string folder_name)
{
    if (!fs::exists(folder_name))
    {
        fs::create_directories(folder_name);
    }

    // basic info
    std::ofstream out;
    out.open(folder_name + "/BasicInfo.txt");
    out << *this;
    out.close();

    // subtoys----------------------
    std::string out_dir = folder_name + "/" + m_name;
    bool dir_created = fs::exists(out_dir);
    if (!dir_created)
    {
        dir_created = fs::create_directories(out_dir);
    }

    if (dir_created)
    {
        for (XSubToy *subToy : m_subToys)
        {
            subToy->Save(out_dir);
        }
    }

    // global ------------------
    // mesh
    WriteOBJ(m_mesh3D, folder_name + "/" + m_name + ".obj");
    WriteOFF(m_mesh3DTmp, folder_name + "/" + m_name + ".off");
    // skeleton
    writeSSkel(folder_name + "/" + m_name + "_sskel.txt", m_sskel);
    writeSSkel(folder_name + "/" + m_name + "_rawSSkel.txt", m_rawSSkel);
    return true;
}

bool XToy::load(const string folder_name)
{
    ClearSubToys();

    std::ifstream in;
    in.open(folder_name + "/BasicInfo.txt");
    if (in.is_open())
    {
        // BasicInfo
        in >> *this;
        in.close();

        M_DEBUG << "Open XToy BasicInfo Successfully" << endl;
        cout << *this;

        // subtoys
        std::string in_dir = folder_name + "/" + m_name;
        int n = static_cast<int>(m_subToys.size());
        for (int i = 0; i < n; ++i)
        {
            m_subToys[i] = m_subtoyPool.Request();

            XSubToy *subToy = m_subToys[i];
            new (subToy) XSubToy();
            subToy->SetID(i);
            subToy->Load(in_dir);
        }

        // parent_child && symmetry relationship
        for (XSubToy *subToy : m_subToys)
        {
            if (subToy->m_parentID != -1)
            {
                XSubToy *parent = m_subToys[subToy->m_parentID];
                subToy->SetParent(parent);
            }
            if (subToy->m_symToyID != -1)
            {
                XSubToy *symToy = m_subToys[subToy->m_symToyID];
                subToy->SetSymToy(symToy);
            }
        }
        ResetRoots();
        M_DEBUG << "root subtoys=" << m_subToyRoots.size() << endl;

        // global ------------------
        // mesh
        ReadOBJ(m_mesh3D, folder_name + "/" + m_name + ".obj");

        // skeleton
        readSSkel(folder_name + "/" + m_name + "_sskel.txt", m_sskel);
        for (SSNodeSub *node : m_sskel.nodes)
        {
            node->set_subtoy(m_subToys[node->subToyID]);
        }
        readSSkel(folder_name + "/" + m_name + "_rawSSkel.txt", m_rawSSkel);
        for (SSNodeSub *node : m_rawSSkel.nodes)
        {
            node->set_subtoy(m_subToys[node->subToyID]);
        }

        visualize_global_sskel();
        M_DEBUG << "load toy successfully!" << endl;
        return true;
    }
    else
    {
        M_DEBUG << "Cannot open " << folder_name << "!\n";
        return false;
    }
}

void XToy::ClearSubToys()
{
    for (XSubToy *subToy : m_subToys)
        m_subtoyPool.Recycle(subToy);
    m_subToys.clear();
}

void XToy::SetupParentChildRelation3D(XSubToy *subToy)
{
    typedef SSEdge<SSNode> EdgeT;
    // find who intersects with subToy
    if (m_subToys.size() <= 1)
    {
        return;
    }

    if (m_usePickedParent)
    {
        subToy->LeaveParent();
        subToy->SetParent(GetParentToy());
        ResetRoots();
    }
    else
    {
        std::unordered_map<XSubToy *, BBox3> sub_bboxes;
        SSkel<SSNode> &csskel = subToy->m_rawSSkel;
        std::unordered_map<XSubToy *, bool> sub_isects;
        std::unordered_map<XSubToy *, SSNode *> cnode_map;
        std::unordered_map<XSubToy *, EdgeT *> pedge_map;

        for (XSubToy *subi : m_subToys)
        {
            if (subi == subToy)
                continue;

            if (subi->m_skelOpt == SKEL_SYM)
            {
                BBox3 bbox = subi->m_curMesh3D.bbox();
                sub_bboxes[subi] = bbox;
            }
        }

        // find minimum distance from subToy's skeleton joint to subi's skeleton edges
        // distance(cjoint, pedge) < radius of pedge, we assume subToy intersect with subi
        for (XSubToy *subi : m_subToys)
        {
            if (subi == subToy)
                continue;

            SSNode *min_cnode = nullptr;
            EdgeT *min_pedge = nullptr;
            double min_dis = std::numeric_limits<double>::max();
            for (SSNode *cnode : csskel.nodes)
            {
                if (cnode->type() == NODE_NORMAL) // only consider junction or terminal node as attachpoint
                    continue;

                SSkel<SSNode> &psskel = subi->m_rawSSkel;
                int n = static_cast<int>(psskel.edges.size());
                for (int i = 0; i < n; ++i) // check whether in the general_cylinder_approximation of subToy i
                {
                    EdgeT *e = static_cast<EdgeT *>(psskel.edges[i]);
                    SSNode *node0 = e->node0;
                    SSNode *node1 = e->node1;
                    double dis, t;
                    DistancePointToLine(node0->pos, node1->pos, cnode->pos, dis, t);
                    if (t < 0)
                    {
                        dis = (node0->pos - cnode->pos).norm();
                    }
                    if (t > 1)
                    {
                        dis = (node1->pos - cnode->pos).norm();
                    }

                    if (dis < min_dis)
                    {
                        min_dis = dis;
                        min_cnode = cnode;
                        min_pedge = e;
                    }
                }
            }

            bool is_inside_subi = false;
            if (subi->m_skelOpt == SKEL_SYM)
            {
                BBox3 bbox = subi->m_curMesh3D.bbox();
                if (IsInsideBBox(sub_bboxes[subi], min_cnode->pos))
                {
                    is_inside_subi = true;
                }
            }
            else
            {
                SSNode *node0 = min_pedge->node0;
                SSNode *node1 = min_pedge->node1;
                double dis, t;
                DistancePointToLine(node0->pos, node1->pos, min_cnode->pos, dis, t);
                if (t < 0)
                {
                    dis = (node0->pos - min_cnode->pos).norm();
                    t = 0;
                }
                if (t > 1)
                {
                    dis = (node1->pos - min_cnode->pos).norm();
                    t = 1;
                }
                double radius = (1 - t) * node0->radius + t * node1->radius;
                if (dis < radius)
                {
                    is_inside_subi = true;
                }
            }

            if (is_inside_subi)
            {
                sub_isects[subi] = true;
                cnode_map[subi] = min_cnode;
                pedge_map[subi] = min_pedge;
            }
        }

        XSubToy *parent = nullptr;
        M_DEBUG << "sub_isects " << sub_isects.size() << endl;
        if (sub_isects.size() == 0)
        {
            subToy->LeaveParent();
        }
        else if (sub_isects.size() == 1)
        {
            auto it = sub_isects.begin();
            parent = it->first;
        }
        else if (sub_isects.size() > 1)
        {
            // choose the largest isect_subtoy as parent
            double max_volume = std::numeric_limits<double>::min();
            for (auto it = sub_isects.begin(); it != sub_isects.end(); ++it)
            {
                XSubToy *subi = it->first;
                bool is_isect = it->second;
                if (is_isect)
                {
                    double volume;
                    if (subi->m_skelOpt == SKEL_SYM)
                    {
                        volume = sub_bboxes[subi].volume();
                    }
                    else
                    {
                        volume = subi->m_curMesh3D.volume();
                    }

                    if (volume > max_volume)
                    {
                        max_volume = volume;
                        parent = subi;
                    }
                }
            }
        }

        if (parent)
        {
            m_parentToyIdx = parent->GetID();
            subToy->SetAttachPoint(cnode_map[parent]->pos);
            M_DEBUG << "SetupParentChildRelation3D " << subToy->name() << " 's parent" << parent->name() << endl;
            subToy->LeaveParent();
            subToy->SetParent(GetParentToy());
        }
        ResetRoots();
    }

    // // the mesh intersection approach, not very fast!!!
    // MatrixXd &VB = subToy->m_curMesh3D.GetVMat();
    // MatrixXi &FB = subToy->m_curMesh3D.GetFMat();
    // M_DEBUG << subToy->name() << " VB:" << VB.rows() << " FB:" << FB.rows() << std::endl;
    // std::list<XSubToy *> Q;
    // for (std::vector<XSubToy *>::iterator it = m_subToyRoots.begin();
    //      it != m_subToyRoots.end(); it++)
    // {
    //     Q.push_back(*it);
    // }

    // while (!Q.empty())
    // {
    //     XSubToy *parent = Q.front();
    //     Q.pop_front();
    //     if (parent == subToy)
    //         continue;
    //     MatrixXd &VA = parent->m_curMesh3D.GetVMat();
    //     MatrixXi &FA = parent->m_curMesh3D.GetFMat();

    //     M_DEBUG << parent->name() << " VA:" << VA.rows() << " FA:" << FA.rows()
    //             << " children=" << parent->GetChildren().size() << std::endl;

    //     MatrixXd VC;
    //     MatrixXi FC;
    //     VectorXi J;
    //     igl::copyleft::cgal::mesh_boolean(VA, FA, VB, FB, igl::MESH_BOOLEAN_TYPE_INTERSECT, VC, FC, J);
    //     if (FC.rows() >= 4)
    //     {
    //         M_DEBUG << parent->name() << " VC:" << VC.rows() << " FC:" << FC.rows() << std::endl;
    //         subToy->m_attachPoint = Centroid(VC, FC);
    //         subToy->SetParent(parent);
    //         M_DEBUG << subToy->name() << " isects with " << parent->name()
    //                 << " at " << subToy->m_attachPoint.transpose() << std::endl;
    //         ResetRoots();
    //         return;
    //     }
    //     for (XSubToy *child : parent->m_children)
    //     {
    //         if (child != subToy)
    //             Q.push_back(child);
    //     }
    // }

    // // not found intersection with other subtoy but have parent
    // if (!subToy->IsRoot())
    // {
    //     subToy->LeaveParent();
    // }
    // ResetRoots();
}

void XToy::ResetRoots()
{
    m_subToyRoots.clear();
    for (XSubToy *subToy : m_subToys)
    {
        if (subToy->IsRoot())
        {
            m_subToyRoots.push_back(subToy);
        }
    }
    M_DEBUG << "ResetRoots m_subToyRoots=" << m_subToyRoots.size() << endl;
}

XSubToy *XToy::AddSubToy()
{
    XSubToy *subToy = m_subtoyPool.Request();
    new (subToy) XSubToy();
    subToy->SetID(m_subToys.size());
    if (m_subToys.size() == 0)
    {
        m_parentToyIdx = subToy->GetID(); // the first subtoy is the default parent subToy
        m_subToyRoots.push_back(subToy);
    }
    m_subToys.push_back(subToy);
    m_curSubToyIdx = subToy->GetID();
    clear_deform_data();
    return subToy;
}

XSubToy *XToy::AddSymSubToy(XSubToy *other, SymetryPlane sym_plane, double plane_pos)
{
    XSubToy *subToy = m_subtoyPool.Request();
    new (subToy) XSubToy(other, sym_plane, plane_pos);
    subToy->SetID(m_subToys.size());
    m_curSubToyIdx = subToy->m_toyID;
    m_subToys.push_back(subToy);
    WriteOBJ(subToy->m_curMesh3D, "output/m_curMesh3D_sym_" + std::to_string(m_curSubToyIdx) + ".obj");

    SetupParentChildRelation3D(subToy); // m_attachpoint
    build_global_sskel();
    visualize_global_sskel();

    M_DEBUG << "AddSymSubToy " << subToy->name() << " successfully!\n";
    return subToy;
}

void XToy::DelSubToy(XSubToy *subToy)
{
    // remove subToy from its parent
    if (subToy->GetParent() != nullptr)
    {
        subToy->LeaveParent();
    }

    if (subToy->GetChildren().size() > 0)
    {
        subToy->DeadToChildren();
    }

    ResetRoots();

    int id = subToy->GetID();
    M_DEBUG << "XToy delete subToy" << id << endl;
    m_subtoyPool.Recycle(subToy);
    if (id != m_subToys.size() - 1)
    {
        XSubToy *subToy_back = m_subToys.back();
        M_DEBUG << "to be deleted subtoy is not the last one, use " << subToy_back << " to replace it" << endl;
        subToy_back->SetID(id);
        m_subToys[id] = subToy_back;
    }
    m_subToys.pop_back();
    M_DEBUG << "pop_back" << endl;
    m_subToyRoots.erase(std::remove(m_subToyRoots.begin(), m_subToyRoots.end(), subToy), m_subToyRoots.end());
    M_DEBUG << "erase from root" << endl;
    global_skel_optim();
    visualize_global_sskel();
}

// bool XToy::load(const string folder_name)
// {
//     bool everything_loaded = true;
//     // Try to load OBJ mesh
//     bool mesh_loaded =
//         load_mesh_from_file(folder_name + "/" + MESH_NAME + ".obj");
//     if (!mesh_loaded)
//     {
//         // try to load OFF mesh
//         mesh_loaded = load_mesh_from_file(folder_name + "/" + MESH_NAME + ".off");
//         if (!mesh_loaded)
//         {
//             // try to load MESH mesh
//             mesh_loaded = load_mesh_from_file(folder_name + "/" + MESH_NAME + ".mesh");
//         }
//     }
//     /***/ printf("%s\n\n", (mesh_loaded ? "succeeded" : "failed"));

//     everything_loaded &= mesh_loaded;
//     //Try to load DMAT weights matrix
//     /***/ printf("Loading weights...\n");
//     bool weights_loaded = load_weights(folder_name + "/" + WEIGHTS_NAME + ".dmat");
//     /***/ printf("%s\n\n", (weights_loaded ? "succeeded" : "failed"));
//     everything_loaded &= weights_loaded;

//     //Try to load BF bone roots file
//     destroy_bone_roots(skel->roots);
//     /***/ printf("Loading bone roots...\n ");
//     bool bone_roots_loaded =
//         load_bone_roots_animation(folder_name + "/" + BONE_ROOTS_NAME + ".bf");
//     /***/ printf("%s\n\n", (bone_roots_loaded ? "succeeded" : "failed"));

//     everything_loaded &= bone_roots_loaded;
//     cout << __FILE__ << " " << __LINE__ << " everything_loaded=" << everything_loaded << endl;
//     // if (everything_loaded)
//     // {
//     //ToDO: load merged mesh and component mesh to subToy
//     m_mesh3D.CreateHalfedgeMesh(V, F);
//     // }

//     return everything_loaded;
// }

// bool XToy::save(const string folder_name)
// {
//     if (!igl::is_dir(folder_name.c_str()))
//     {
//         /***/ printf("Creating new directory %s...", folder_name.c_str());
//         bool folder_created = fs::create_directory(folder_name);
//         if (!folder_created)
//         {
//             fprintf(
//                 stderr,
//                 "^Skinning::save: IOERROR (%d): could not create folder: %s\n",
//                 errno,
//                 folder_name.c_str());
//             return false;
//         }
//     }
//     else
//     {
//         /***/ printf("Directory %s already exists\n", folder_name.c_str());
//     }

//     // Try to save everything to this folder with a default name
//     bool everything_saved = true;

//     // Try to save mesh
//     bool mesh_saved = false;
//     /***/ printf("Saving Mesh...");
//     string obj_file_name(folder_name + "/" + MESH_NAME + ".obj");
//     string mesh_file_name(folder_name + "/" + MESH_NAME + ".mesh");
//     if (Tets.size() == 0)
//     {
//         if (V.size() > 0 && F.size() > 0)
//         {
//             remove(mesh_file_name.c_str());
//             mesh_saved = igl::writeOBJ(obj_file_name.c_str(), V, F);
//         }
//         if (V2D.size() > 0 && F2D.size() > 0)
//         {
//             string mesh2D_file_name(folder_name + "/" + MESH2D_NAME + ".obj");
//             remove(mesh_file_name.c_str());
//             mesh_saved = igl::writeOBJ(mesh2D_file_name.c_str(), V2D, F2D);
//         }
//     }
//     else
//     {
//         remove(obj_file_name.c_str());
//         mesh_saved = igl::writeMESH(folder_name + "/" + MESH_NAME + ".mesh", V, Tets, F);
//     }
//     /***/ printf("%s\n", (mesh_saved ? "succeeded" : "failed"));
//     everything_saved &= mesh_saved;

//     // Try to save weights
//     /***/ printf("Saving weights...");
//     bool weights_saved =
//         igl::writeDMAT((folder_name + "/" + WEIGHTS_NAME + ".dmat").c_str(), OW); // ToDO
//     /***/ printf("%s\n", (weights_saved ? "succeeded" : "failed"));
//     everything_saved &= weights_saved;

//     // Try to save bones
//     if (animation.size() > 0)
//     {
//         bool bone_roots_saved =
//             save_bone_roots_animation((folder_name + "/" + BONE_ROOTS_NAME + ".bf").c_str());
//         everything_saved &= bone_roots_saved;
//     }
//     else
//     {
//         bool bone_roots_saved =
//             save_bone_roots((folder_name + "/" + BONE_ROOTS_NAME + ".bf").c_str());
//         everything_saved &= bone_roots_saved;
//     }
// }

bool XToy::load_mesh_from_file(const std::string mesh_file_name)
{
    // Load mesh file into V,F
    /***/ printf("Loading mesh from file %s...", mesh_file_name.c_str());
    string dirname, basename, extension, filename;
    igl::pathinfo(mesh_file_name, dirname, basename, extension, filename);
    printf("dir=%s, basename=%s, extension=%s, filename=%s\n", dirname.c_str(), basename.c_str(), extension.c_str(), filename.c_str());
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    // Clear old mesh
    V.resize(0, 3);
    F.resize(0, 3);
    Tets.resize(0, 4);
    bool success = false;
    if (extension == "obj")
    {
        Tets.resize(0, 0);
        success = igl::readOBJ(mesh_file_name, V, F);
    }
    else if (extension == "off")
    {
        Tets.resize(0, 0);
        success = igl::readOFF(mesh_file_name, V, F);
    }
    else if (extension == "mesh")
    {
        success = igl::readMESH(mesh_file_name, V, Tets, F);
    }
    else
    {
        fprintf(stderr,
                "Error: load_mesh_from_file: unsupported mesh exenstion %s\n",
                extension.c_str());
        success = false;
    }

    if (!success)
    {
        return false;
    }
    if (V.size() == 0)
    {
        return false;
    }
    return initialize_mesh();
}

bool XToy::initialize_mesh()
{

    if (V.rows() != W.rows())
    {
        // Clear weights
        /***/ printf("Clearing weights since V.rows() (%d) != W.rows() (%d)\n", //ToDO
                     (int)V.rows(),
                     (int)W.rows());
        W.resize(V.rows(), 0);
        OW.resize(V.rows(), 0);
        EW.resize(V.rows(), 0);
        WI.resize(V.rows(), 0);
    }

    cpuV = V;
	return true;
}

bool XToy::load_weights(const std::string weights_file_name)
{
    // read matrix from DMAT file type
    bool read_success = igl::readDMAT(weights_file_name, OW);

    if (!read_success)
    {
        OW.resize(0, 0);
        W.resize(0, 0);
        WI.resize(0, 0);
        return false;
    }
    if (OW.rows() != EW.rows())
    {
        EW.resize(OW.rows(), 0);
    }
    // initialize_depth_offsets(); //ToDO
    return initialize_weights();
}

bool XToy::load_bone_roots_animation(const std::string bone_roots_file_name)
{
    animation.clear();
    load_bone_roots(bone_roots_file_name);

    animation.push_back(
        KeyFrame<BoneBoneCopyMap>(
            BoneBoneCopyMap(skel->roots, false),
            animation_interp_secs,
            transition_type));

    std::cout << "pushed back..." << std::endl;
    // determine basename and extension of file
    string dirname, basename, extension, filename;
    igl::pathinfo(bone_roots_file_name, dirname, basename, extension, filename);
    int i = 1;
    std::cout << "while..." << std::endl;
    while (true)
    {
        stringstream next_name;
        next_name << dirname << "/" << filename << "-" << i << "." << extension;
        // check that this file exists
        if (igl::is_readable(next_name.str().c_str()))
        {
            /***/ printf("next_name: %s\n", next_name.str().c_str());
            // try to load this file as bone roots
            bool load_success = load_bone_roots(next_name.str().c_str(), true, true);
            if (!load_success)
            {
                return false;
            }
            // push current skeleton roots as animation frame
            animation.push_back(
                KeyFrame<BoneBoneCopyMap>(
                    BoneBoneCopyMap(skel->roots, false),
                    animation_interp_secs,
                    transition_type));
        }
        else
        {
            // file didn't exist so we've read all available for animation
            break;
        }
        i++;
    }
    std::cout << "loaded bone roots animation... animation size=" << animation.size() << std::endl;
    return true;
}

bool XToy::load_bone_roots(
    const std::string bone_roots_file_name,
    const bool must_match,
    const bool no_recompute)

{
    // read matrix from BF file type
    vector<Bone *> new_BR;
    bool read_success = read_BF(bone_roots_file_name.c_str(), skel, new_BR);
    // make sure drawing according to last_T is set to match for all bones
    // Check that trees match
    if (!skel->get_editing() && bone_roots_match(new_BR, skel->roots))
    {
        printf("Bones match... Just loading data into existing bones...\n");
        // If bone forests match then just load data (transformations) into
        // existing bones
        copy_bone_roots_data(new_BR, skel->roots);
        if (auto_dof && !no_recompute)
        {
            // reinitialize_auto_dof(); //ToDO
        }
    }
    else if (must_match)
    {
        printf("Bones don't match...\n");
        return false;
    }
    else
    {
        printf("Bones don't match, create new bones...\n");
        destroy_bone_roots(skel->roots);
        skel->roots = new_BR;
        //clear animation stack
        animation.clear();
    }

    initialize_transformations();
    return read_success;
}

bool XToy::save_bone_roots(const string bone_roots_file_name)
{
    // Try to save bones
    /***/ printf("Saving bone forest...");
    bool bone_roots_saved = write_BF(bone_roots_file_name.c_str(), skel->roots);
    /***/ printf("%s\n", (bone_roots_saved ? "succeeded" : "failed"));
    return bone_roots_saved;
}

bool XToy::save_bone_roots_animation(const std::string bone_roots_file_name)
{
    // determine basename and extension of file
    string dirname, basename, extension, filename;
    igl::pathinfo(bone_roots_file_name, dirname, basename, extension, filename);
    /***/ printf("Saving bone forest animation...");
    bool bone_roots_saved = true;

    // Play through animation stack and save each pose
    for (int i = 0; i < (int)animation.size(); i++)
    {
        // Use lerp to load this pose into current skeleton roots
        lerp(animation[i].state, animation[i].state, 0);
        stringstream name;
        if (i == 0)
        {
            // First frame gets input name
            name << bone_roots_file_name;
        }
        else
        {
            name << dirname << "/" << filename << "-" << i << "." << extension;
        }
        /***/ printf("name[%d]: %s\n", i, name.str().c_str());
        bone_roots_saved &= write_BF(name.str().c_str(), skel->roots);
        if (!bone_roots_saved)
        {
            break;
        }
    }
    /***/ printf("%s\n", (bone_roots_saved ? "succeeded" : "failed"));
    return bone_roots_saved;
}

bool XToy::initialize_weights()
{
    assert((OW.cols() + EW.cols()) <= MAX_NUM_HANDLES);
    if (OW.rows() != V.rows())
    {
        printf(
            "Error: OW.rows() (%d) != V.rows() (%d)\n"
            "  Setting OW to []\n",
            (int)OW.rows(),
            (int)V.rows());
        OW.resize(0, 0);
        return false;
    }

    // Need to reinitialize transformations if there was a change in the number
    // of handles
    if ((OW.cols() + EW.cols()) != T.cols() / 4)
    {
        initialize_transformations();
    }

    if (OW.size() > 0)
    {
        M_DEBUG << "construct lbs_matrix" << endl;
        MatrixXd OEW(OW.rows(), OW.cols() + EW.cols());
        if (EW.size() == 0)
        {
            OEW << OW;
        }
        else
        {
            OEW << OW, EW;
        }
        igl::lbs_matrix(V, OEW, M);
        M_DEBUG << " M=" << M.rows() << ", " << M.cols() << endl;
        // cout << M.block(0, 0, 10, M.cols()) << endl;
        egl::writeMatrix("output/M.dmat", M);
        // igl::lbs_matrix_column(V, OEW, Mcol);
    }

    return true;
}

bool XToy::initialize_transformations()
{
    // Stack handle transformations horizontallly
    // Initialize all entries to zero
    int num_handles = OW.cols() + EW.cols();
    T.setZero(3, num_handles * 4);

    return true;
}

void XToy::compute_weights2D(int width, int height, float *mvp)
{
    using namespace Eigen;
    M_DEBUG << "compute_weights2D" << endl;
    for (int i = 0; i < m_subToys.size(); i++)
    {
        XSubToy *subToy = m_subToys[i];
        if (!subToy->m_autoSkel)
            continue;

        TMesh &m_defMesh2D = subToy->m_defMesh2D;
        MatrixXd &VV = m_defMesh2D.GetVMat();
        MatrixXi &FF = m_defMesh2D.GetFMat();
        double avg_len = m_defMesh2D.avg_edge_len();
        VectorXi b;
        MatrixXd bc;
        M_DEBUG << subToy->name() << " num_vertices=" << VV.rows() << endl;

        gather_subB(width, height, mvp, subToy);
        vector<Bone *> &subB = subToy->m_subB;
        int num_bones = 0;
        for (int i = 0; i < subB.size(); i++)
        {
            // subB[0] is root for subtoy
            if ((subToy->m_parent != nullptr && i == 0) || subB[i]->is_root())
                continue;
            num_bones++;
        }

        if (skel->point_handle_mode)
        {
            // boundary_conditions_points(VV, FF, skel->roots, b, bc, subToy->m_toyID); //ToDO
        }
        else
        {
            map<Bone *, int> B2I;
            MatrixXd P(subB.size(), 3);
            MatrixXi BE(num_bones, 2); // assume there is only one root on subtoy
            for (int i = 0; i < subB.size(); i++)
            {
                B2I[subB[i]] = i;
                // M_DEBUG << " " << subB[i] << " " << subB[i]->get_wi() << " B2I=" << B2I[subB[i]] << endl;
            }

            int e = 0;
            for (int i = 0; i < subB.size(); i++)
            {
                // subB[0] is root for subtoy
                P.row(i) = subB[i]->tip_as_drawn().transpose();
                // M_DEBUG << " P" << i << "=" << P.row(i) << endl;
                if ((subToy->m_parent != nullptr && i == 0) || subB[i]->is_root())
                    continue;
                Bone *b = subB[i];
                BE(e, 0) = B2I[b->parent];
                BE(e, 1) = B2I[b];
                e++;
            }

            // M_DEBUG << " BE=" << endl;
            // cout << BE << endl;
            boundary_conditions_edges(VV, FF, P, BE, avg_len, b, bc);
        }

        igl::BBWData bbw_data;
        bbw_data.active_set_params.max_iter = 2 * num_bones;
        bbw_data.verbosity = 0;
        if (igl::bbw(VV, FF, b, bc, bbw_data, subToy->m_skinW2D))
        {
            M_DEBUG << " " << subToy->name() << " compute weights 2D successfully!" << endl;
        }
        igl::normalize_row_sums(subToy->m_skinW2D, subToy->m_skinW2D); // #num_vertices by #num_handles
        igl::lbs_matrix(VV, subToy->m_skinW2D, subToy->m_skinM);       // #num_vertices by #num_handlesx4

        M_DEBUG << " m_skinW2D=" << subToy->m_skinW2D.rows() << ", " << subToy->m_skinW2D.cols()
                << " m_skinM=" << subToy->m_skinM.rows() << ", " << subToy->m_skinM.cols() << endl;
        // cout << "...m_skinM" << endl;
        // cout << subToy->m_skinM.block(0, 0, 10, subToy->m_skinM.cols()) << endl;
        // MatrixXd skinT(subToy->m_skinM.cols(), 3);
        // skinT.setZero();
        // for (int i = 0; i < skinT.rows() / 4; i++)
        // {
        //     skinT.block(i * 4, 0, 3, 3) = Matrix3d::Identity();
        // }
        // MatrixXd tmp = subToy->m_skinM * skinT;
        // igl::writePLY("output/skinMT.ply", tmp, FF);
    }
}

void XToy::compute_weightsTet()
{
    if (V.size() == 0 || skel->roots.size() <= 0)
    {
        return;
    }

    Tets.resize(0, 0);
    OW.resize(0, 0);
    M.resize(0, 0);
    T.resize(0, 0);

    // ----------Gather samples along controls
    MatrixXd S;
    gather_samples(skel->roots, 10, S);

    // ----------Tetralization
#ifdef ToDEL
    // VectorXi IM;
    // igl::faces_first(V, F, IM); // vertices on the surface is reordered to front positions
    // Tets = Tets.unaryExpr(bind1st(mem_fun(static_cast<VectorXi::Scalar &(VectorXi::*)(VectorXi::Index)>(&VectorXi::operator())),
    //                               &IM))
    //            .eval();
#endif

    // Surface vertices
    MatrixXd SV = V.block(0, 0, F.maxCoeff() + 1, V.cols());
    // Remesh at control samples
    MatrixXd VS = igl::cat(1, SV, S);
    // Boundary faces
    MatrixXi BF;

    M_DEBUG << " tetgen begin()" << endl;
    // -Y: preserve the input surface mesh
    // -Q: quiet
    // -p: Tetrahedralize a piecewise linear complex (PLC)
    // -q: refine mesh to improve quality
    // -q1.414: the maximum radius-edge ratio is 1.414
    // int status = igl::copyleft::tetgen::tetrahedralize(VS, F, "YQpq100", V, Tets, BF); //ToDO
    int status = igl::copyleft::tetgen::tetrahedralize(VS, F, "YQpq1.414", V, Tets, BF); //ToDO
    M_DEBUG << " tetgen end() V=" << V.rows() << endl;
    if (BF.rows() != F.rows())
    {
        //assert(BF.maxCoeff() == skinning->F.maxCoeff());
        printf("^%s: Warning: boundary faces != orignal faces\n", __FUNCTION__);
    }

    if (status != 0)
    {
        printf("^%s: tetgen failed. Just meshing convex hull\n", __FUNCTION__);

        status = igl::copyleft::tetgen::tetrahedralize(VS, F, "Qq1.414", V, Tets, BF);
        assert(F.maxCoeff() < V.rows());
        if (status != 0)
        {
            printf("^%s: tetgen failed again.\n", __FUNCTION__);
            return;
        }
    }

    m_mesh3D.CreateHalfedgeMesh(V, F);
    WriteOBJ(m_mesh3D, "output/tet3d.obj");
    M_DEBUG << " write tetralized mesh successfully!" << endl;

    // automatically compute weights according to BBW
    double avg_len = 0;
    for (int i = 0; i < m_subToys.size(); i++)
    {
        XSubToy *subToy = m_subToys[i];
        avg_len += subToy->m_curMesh3D.avg_edge_len();
    }
    avg_len /= (double)m_subToys.size();

    M_DEBUG << "average edge length 3D =" << avg_len << endl;
    VectorXi b;
    MatrixXd bc;
    if (skel->point_handle_mode)
    {
        boundary_conditions_points(V, Tets, skel->roots, b, bc);
    }
    else
    {
        boundary_conditions_edges(V, Tets, skel->roots, avg_len, b, bc);
    }
    // igl::writeDMAT("output/boundVerts.dmat", b.matrix()); //ToDO: cannot compile with static library

    igl::BBWData bbw_data;
    bbw_data.active_set_params.max_iter = BBW_ITER;
    bbw_data.verbosity = 0;
    if (igl::bbw(V, Tets, b, bc, bbw_data, OW))
    {
        M_DEBUG << " compute weights successfully! OW=" << OW.rows() << ", " << OW.cols() << endl;
        egl::writeMatrix("output/W.mat", OW);
    }
    if (OW.rows() != EW.rows())
    {
        EW.resize(OW.rows(), 0);
    }
    igl::normalize_row_sums(OW, OW);
    initialize_weights();
    initialize_transformations();
    cpuV = V;
    // save("output/skinning");
    // load("output/skinning");
}

void XToy::correct_mesh_orientation()
{
    for (size_t i = 0; i < m_subToys.size(); ++i)
    {
        Eigen::MatrixXi E, uE;
        Eigen::VectorXi EMAP;
        std::vector<std::vector<size_t>> uE2E;

        Eigen::MatrixXd &subV = m_subToys[i]->m_curMesh3D.GetVMat();
        Eigen::MatrixXi &F = m_subToys[i]->m_curMesh3D.GetFMat();
        igl::unique_edge_map(F, E, uE, EMAP, uE2E);

        if (!igl::piecewise_constant_winding_number(F, uE, uE2E))
        {
            M_DEBUG << "subToy" << i << " 3DMesh is not PWN" << endl;
            m_subToys[i]->m_curMesh3D.ReverseOrientation();
            WriteOBJ(m_subToys[i]->m_curMesh3D, "output/oriented_" + std::to_string(i) + ".obj");
        }
    }
}

void XToy::correct_mesh_normals()
{
    using namespace Eigen;
    for (size_t i = 0; i < m_subToys.size(); ++i)
    {
        XSubToy *subToy = m_subToys[i];
        TMesh &mesh3d = subToy->m_curMesh3D;
        Eigen::MatrixXd &V = mesh3d.GetVMat();
        Eigen::MatrixXi &F = mesh3d.GetFMat();
        Eigen::MatrixXd N;
        igl::per_vertex_normals(V, F, N);
        Eigen::RowVector3d camera_pos(0, 0, 800);
        int min_row;
        (V.rowwise() - camera_pos).rowwise().squaredNorm().minCoeff(&min_row);
        Eigen::RowVector3d viewdir = camera_pos - V.row(min_row);
        double angle = viewdir.dot(N.row(min_row));
        if (angle < 0)
        {
            m_subToys[i]->m_curMesh3D.ReverseOrientation();
            // WriteOBJ(m_subToys[i]->m_curMesh3D, "output/oriented_normal_" + std::to_string(i) + ".obj");
        }
        // M_DEBUG << "-----------------" << subToy->name() << " angle=" << angle << endl;
    }
}

void XToy::fill_mesh_holes()
{
    typedef cgal_adapter::CMesh CMesh;
    // filling hole to Prevent Input mesh is not PWN error for multi-parts merging
    for (size_t i = 0; i < m_subToys.size(); ++i)
    {
        CMesh cmesh;
        cgal_adapter::create_cmesh(m_subToys[i]->m_curMesh3D, cmesh);
        cgal_adapter::write_off("output/before_fill_hole" + std::to_string(i) + ".off", cmesh);

        if (cgal_adapter::fill_hole(&m_subToys[i]->m_curMesh3D))
        {
            M_DEBUG << i << " has holes" << endl;
            WriteOBJ(m_subToys[i]->m_curMesh3D, "output/filled_" + std::to_string(i) + ".obj");
        }
    }
}

// simply redindex Vertices from subMeshes to the whole mesh
bool XToy::compute_V3D()
{
    if (m_subToys.size() < 1)
        return false;
    fill_mesh_holes();
    correct_mesh_orientation();
    correct_mesh_normals();

    // merge meshes into one
    // ToDO: more than one connected components?
    if (m_isSubToyMoved)
    {
        V = m_subToys[0]->m_curMesh3D.GetVMat();
        F = m_subToys[0]->m_curMesh3D.GetFMat();
        M_DEBUG << " V=" << V.rows() << endl;
        for (size_t i = 1; i < m_subToys.size(); ++i)
        {
            Eigen::VectorXi J;
            const MatrixXd &VB = m_subToys[i]->m_curMesh3D.GetVMat();
            const MatrixXi &FB = m_subToys[i]->m_curMesh3D.GetFMat();
            bool flag = igl::copyleft::cgal::mesh_boolean(V, F, VB, FB, igl::MESH_BOOLEAN_TYPE_UNION, V, F, J);
            M_DEBUG << i << " VB=" << VB.rows() << " V=" << V.rows() << endl;
        }

        m_mesh3D.CreateHalfedgeMesh(V, F);
        if (cgal_adapter::connected_components(&m_mesh3D) != 1)
        {
            M_DEBUG << "Merged mesh is not a single connected component;" << endl;
            WriteOBJ(m_mesh3D, "output/multi_comp_3D.obj");
            m_mesh3D.clear();
            V.resize(0, 3);
            F.resize(0, 3);
            return false;
        }
        m_mesh3DTmp.CreateHalfedgeMesh(V, F);
        WriteOBJ(m_mesh3D, "output/merged3D.obj");
        WriteOFF(m_mesh3DTmp, "output/merged3D.off");
        M_DEBUG << " compute V3D successfully V=" << V.rows() << endl;
    }
    return true;
}

void XToy::gather_subB(int width, int height, float *mvpMatrix, XSubToy *subToy)
{
    subToy->m_subB.clear();
    // Note!!! This is a coarse estimation whether the Bone is in BBox3d of subToy
    std::vector<Bone *> B = gather_bones(skel->roots);
    // M_DEBUG << " gather_subB num_bones=" << B.size() << endl;
    BBox3 bbox = subToy->m_curMesh3D.bbox();
    for (Bone *b : B)
    {

        Pnt3 d;
        Pnt3 tip = b->rest_tip();
        if (tip.x() > bbox.xmin && tip.x() < bbox.xmax &&
            tip.y() > bbox.ymin && tip.y() < bbox.ymax &&
            tip.z() > bbox.zmin && tip.z() < bbox.zmax)
        {
            // M_DEBUG << b << " " << b->get_wi() << " inside " << subToy->name() << endl;
            subToy->m_subB.push_back(b);
            b->subtoy_id = subToy->m_toyID;
        }
    }
}

void XToy::gather_subT(int width, int height, float *mvpMatrix, XSubToy *subToy)
{
    std::vector<Bone *> B = gather_bones(skel->roots);
    std::vector<Bone *> &subB = subToy->m_subB;
    MatrixXd &subT = subToy->m_skinT;

    int num_handles = 0;
    for (int i = 0; i < subB.size(); i++)
    {
        if ((subToy->m_parent != NULL && i == 0) || subB[i]->is_root())
            continue;
        num_handles++;
    }

    subT.resize(3, num_handles * 4);
    subT.setZero();

    // global bone index
    map<const Bone *, int> B2I;
    B2I[NULL] = -1;
    int i = 0;
    for (vector<Bone *>::iterator bit = B.begin(); bit != B.end(); bit++)
    {
        B2I[*bit] = i;
        i++;
    }

    // loop over bones
    int handle_idx = 0;
    for (int i = 0; i < subB.size(); i++)
    {
        Bone *b = subB[i];
        if ((subToy->m_parent != NULL && i == 0) || b->is_root())
            continue;

        int idx = B2I[b] - skel->roots.size();
        // M_DEBUG << "wi=" << b->get_wi() << " B2I=" << B2I[b] << " " << idx << endl;
        // cout << T.block(0, idx * 4, 3, 4) << endl;
        // place transform into T stack
        subT.block(0, handle_idx * 4, 3, 4) = T.block(0, idx * 4, 3, 4);
        handle_idx++;
    }
}

bool XToy::drag_bone(int sx, int sy,
                     int width, int height,
                     float *viewMatrix, float *mvpMatrix,
                     bool right_click, bool shift_down, bool ctrl_down)
{
    int count = 0;
    if (!skel->get_editing()) // BBW and LBS Deformation
    {
        // M_DEBUG << "gather and distribute transformatioins..." << endl;
        // skel->draw_according_to_last_T = dial_in_each_T; //ToDO
        skel->drag_bone(sx, sy, width, height, viewMatrix, mvpMatrix, right_click, shift_down, ctrl_down);
        bool success = gather_transformations(skel->roots, dial_in_each_T, T);
        if (success)
        {

            // M_DEBUG << " T=" << T.rows() << ", " << T.cols() << endl;
            // cout << T.transpose() << endl;

            // 3D LBS
            if (auto_dof)
            {
                transformations();
            }

            cpuV = M * T.transpose();
            m_mesh3D.CreateHalfedgeMesh(cpuV, F);
            // WritePly(m_mesh3D, "output/deformed3D.ply");
        }
        if (!dial_in_each_T)
        {
            distribute_transformations(T, skel->roots);
        }
    }
    else
    {
        skel->drag_bone(sx, sy, width, height, viewMatrix, mvpMatrix, right_click, shift_down, ctrl_down);
    }
	return true;
}

Bone *XToy::pick_bone(int sx, int sy, int width, int height, float *mvpMatrix, bool shift_down, bool ctrl_down)
{
    Bone *b = skel->pick_bone(sx, sy, width, height, mvpMatrix, shift_down, ctrl_down);
    if (b != NULL)
    {
        m_curSubToyIdx = b->subtoy_id;
        cout << __FILE__ << " " << __LINE__ << " sx=" << sx << " sy=" << sy << " m_curSubToyIdx=" << m_curSubToyIdx << endl;
    }
    return b;
}

void XToy::set_rest_pose()
{
    skel->reset();
    for (int i = 0; i < m_subToys.size(); i++)
    {
        XSubToy *subToy = m_subToys[i];
        subToy->m_defMesh2D = subToy->m_curMesh2D;
    }
    initialize_transformations();
    int num_handles = T.cols() / 4;
    for (int i = 0; i < num_handles; i++)
    {
        T.block(0, i * 4, 3, 3) = Matrix3d::Identity();
    }
    cpuV = M * T.transpose();
    m_mesh3D.CreateHalfedgeMesh(cpuV, F);
}

const Eigen::MatrixXi &XToy::elements()
{
    // Decide if solving over volume (tets) or surface (triangles)
    // Indices of elements over which we're solving, either F or T
    if (Tets.size() == 0)
    {
        /***/ printf("Solving over surface...\n");
        return F;
    }
    else
    {
        /***/ printf("Solving over volume...\n");
        return Tets;
    }
}

bool XToy::initialize_auto_dof()
{
    cout << __FILE__ << " " << __LINE__ << " .............initialize_auto_dof..........." << endl;
    // transformations
    gather_transformations(skel->roots, dial_in_each_T, T);

    const int num_handles = W.cols();
    assert(T.cols() / 4 == num_handles);
    const int dim = V.cols();

    // partition into groups
    int k = max(num_handles + 1, min((int)V.rows(), num_groups));
    if (k != num_groups)
    {
        fprintf(stderr,
                "Warning: initialize_auto_dof() num_groups (%d)"
                " invalid, setting to (%d)\n",
                num_groups,
                k);
        num_groups = k;
    }
    // Cluster according to weights
    VectorXi G;
    {
        VectorXi S;
        VectorXd D;
        igl::partition(W, k, G, S, D);
        cout << __FILE__ << " " << __LINE__ << " G=" << G.rows() << ", " << G.cols() << " Seed Indexs=" << S.rows() << ", " << S.cols() << " D=" << D.rows() << ", " << D.cols() << endl;
    }
    igl::arap_dof_precomputation(V, Tets, M, G, arap_dof);
    cout << __FILE__ << " " << __LINE__ << " arap_dof_precompute successfully!" << endl;
    Matrix<int, Dynamic, 1> free(0, 1);
    gather_free(skel->roots, W.cols(), free);
    cout << __FILE__ << " " << __LINE__ << " free handles: " << free.transpose() << endl;
    MatrixXd I = MatrixXd::Identity(dim, dim + 1);
    MatrixXd IGstack;
    igl::repmat(I, 1, num_handles, IGstack);
    // set free handles' initial guesses to zeros
    for (int i = 0; i < free.size(); i++)
    {
        IGstack.block(0, free(i) * (dim + 1), dim, dim + 1).setZero();
    }
    cout << __FILE__ << " " << __LINE__ << " IGstack=\n"
         << IGstack.transpose() << endl;
    igl::columnize(IGstack, num_handles, 2, L);
    return reinitialize_auto_dof();
}

bool XToy::reinitialize_auto_dof()
{
    // number of handles
    const int m = T.cols() / 4;
    // number f dimensions
    const int dim = V.cols();
    // List of fixed *weight functions*, handles or bones are not fixed. their
    // weight functions are fixed
    Eigen::Matrix<int, Eigen::Dynamic, 1> fixed(0, 1);
    Eigen::Matrix<int, Eigen::Dynamic, 1> linear(0, 1);
    // List of free *weight functions*
    Eigen::Matrix<int, Eigen::Dynamic, 1> free(0, 1);
    gather_free(skel->roots, W.cols(), free);
    gather_fixed(skel->roots, W.cols(), DOF_TYPE_FIXED_ALL, fixed);
    gather_fixed(skel->roots, W.cols(), DOF_TYPE_FIXED_LINEAR, linear);
    cout << "free=[" << endl
         << free << endl
         << "]+1;" << endl;
    cout << "fixed=[" << endl
         << fixed << endl
         << "]+1;" << endl;
    cout << "linear=[" << endl
         << linear << endl
         << "]+1;" << endl;
    // Gather list of indices to known values (each dimension of fixed)
    VectorXi fixed_dim(
        fixed.size() * dim * (dim + 1) +
        linear.size() * dim * (dim));

    for (int d = 0; d < dim * (dim + 1); d++)
    {
        for (int i = 0; i < (int)fixed.size(); i++)
        {
            fixed_dim(fixed.size() * d + i) = d * (m) + fixed(i);
        }
    }
    for (int d = 0; d < dim * (dim); d++)
    {
        for (int i = 0; i < (int)linear.size(); i++)
        {
            fixed_dim(fixed.size() * dim * (dim + 1) + linear.size() * d + i) =
                d * (m) + linear(i);
        }
    }

    SparseMatrix<double> A_eq, A_fix_eq;
    gather_positional_constraints_system(skel->roots, m, dim, A_eq); // dim*#constraint_points by m*dim*(dim+1)
    gather_fixed_constraints_system(fixed_dim, dim, m, A_fix_eq);    // fixed_dim.size(), numBones * dim * (dim + 1)
    SparseMatrix<double> A_eq_merged;
    join_constraints_systems(A_eq, A_fix_eq, A_eq_merged);
    igl::arap_dof_recomputation(fixed_dim, A_eq_merged, arap_dof);
    cout << __FILE__ << " " << __LINE__ << " arap_dof_recomputation successfully" << endl;
    return true;
}

bool XToy::initialize_auto_dof1()
{
    auto_dof = true;

    // initialize_weights();
    // initialize_transformations();

    int num_handles = T.cols() / 4;
    int dim = V.cols();

    // ToDO: compute extra weights into TW
    // TW = [OW EW]
    Eigen::MatrixXd TW;
    assert(OW.rows() == EW.rows());
    TW.resize(OW.rows(), OW.cols() + EW.cols());
    TW.block(0, 0, OW.rows(), OW.cols()) = OW;
    TW.block(0, OW.cols(), EW.rows(), EW.cols()) = EW;

    // Partition into groups
    int k = max(num_handles + 1, min((int)V.rows(), num_groups));
    if (k != num_groups)
    {
        fprintf(stderr,
                "Warning: initialize_auto_dof() num_groups (%d)"
                " invalid, setting to (%d)\n",
                num_groups,
                k);
        num_groups = k;
    }
    Matrix<int, Dynamic, 1> G;
    Matrix<int, Dynamic, 1> S;
    Matrix<double, Dynamic, 1> GD;
    igl::partition(TW, k, G, S, GD);
    printf("%s %d partition success! num_groups=%d\n", __FILE__, __LINE__, num_groups);

    // Arap auto dof precomputation
    double before = igl::get_seconds();
    igl::arap_dof_precomputation(V, elements(), Mcol, G, arap_dof);
    printf("%s %d Precomputation time: %g\n", __FILE__, __LINE__, igl::get_seconds() - before);

    // Initialize initial guess to identities, except free handles get zeros
    Eigen::Matrix<int, Eigen::Dynamic, 1> free(0, 1);
    gather_free(skel->roots, (OW.cols() + EW.cols()), free);
    MatrixXd I = MatrixXd::Identity(dim, dim + 1);
    MatrixXd IGstack;
    igl::repmat(I, 1, num_handles, IGstack);
    for (int i = 0; i < free.size(); i++)
    {
        // set free handles' initial guesses to zeros
        IGstack.block(0, free(i) * (dim + 1), dim, dim + 1).setZero();
    }
    igl::columnize(IGstack, num_handles, 2, L);

    printf("%s %d initialize_auto_dof success!\n", __FILE__, __LINE__);
    return reinitialize_auto_dof1();
}

bool XToy::reinitialize_auto_dof1()
{
    auto_dof = true;

    const int m = T.cols() / 4;
    const int dim = V.cols();

    // List of fixed *weight functions*, handles or bones are not fixed. their
    // weight functions are fixed
    Eigen::Matrix<int, Eigen::Dynamic, 1> fixed(0, 1);
    Eigen::Matrix<int, Eigen::Dynamic, 1> linear(0, 1);
    // List of free *weight functions*
    Eigen::Matrix<int, Eigen::Dynamic, 1> free(0, 1);
    gather_free(skel->roots, (OW.cols() + EW.cols()), free);
    gather_fixed(skel->roots, (OW.cols() + EW.cols()), DOF_TYPE_FIXED_ALL, fixed);
    gather_fixed(skel->roots, (OW.cols() + EW.cols()), DOF_TYPE_FIXED_LINEAR, linear);
    // Gather list of indices to known values (each dimension of fixed)
    VectorXi fixed_dim(
        fixed.size() * dim * (dim + 1) +
        linear.size() * dim * (dim));
    for (int d = 0; d < dim * (dim + 1); d++)
    {
        for (int i = 0; i < (int)fixed.size(); i++)
        {
            fixed_dim(fixed.size() * d + i) = d * (m) + fixed(i);
        }
    }
    for (int d = 0; d < dim * (dim); d++)
    {
        for (int i = 0; i < (int)linear.size(); i++)
        {
            fixed_dim(fixed.size() * dim * (dim + 1) + linear.size() * d + i) =
                d * (m) + linear(i);
        }
    }

    // Linear positional constraints system
    SparseMatrix<double> A_eq, A_fix_eq;
    gather_positional_constraints_system(skel->roots, m, dim, A_eq);
    gather_fixed_constraints_system(fixed_dim, dim, m, A_fix_eq);
    SparseMatrix<double> A_eq_merged;
    join_constraints_systems(A_eq, A_fix_eq, A_eq_merged);

    igl::arap_dof_recomputation(fixed_dim, A_eq_merged, arap_dof);
    printf("%s %d reinitialize_auto_dof success!\n", __FILE__, __LINE__);
    return true;
}

bool XToy::transformations()
{
    // Compute transformations into T
    bool gather_success = gather_transformations(skel->roots, dial_in_each_T, T);
    if (!gather_success)
    {
        return false;
    }
    //ToDO
    if (auto_dof)
    {
        // number of handles
        int m = arap_dof.m;
        int dim = arap_dof.dim;
        Matrix<double, Dynamic, 1> B_eq;
        gather_positional_constraints_rhs(skel->roots, m, dim, B_eq);
        // Gather transformations into a column
        MatrixXd Tcol;
        igl::columnize(T.block(0, 0, dim, T.cols()).eval(), m, 2, Tcol);
        // use last solution as initial guess
        MatrixXd L0 = L;
        if (arap_dof.fixed_dim.size() > 0)
        {
            // But also place transformations of fixed handles into initial guess
            Eigen::Matrix<int, Eigen::Dynamic, 1> zero(1);
            zero << 0;
            MatrixXd Tcol_fixed_dim;
            igl::slice(Tcol, arap_dof.fixed_dim, zero, Tcol_fixed_dim);
            igl::slice_into(Tcol_fixed_dim, arap_dof.fixed_dim, zero, L0);

            assert(L0.size() == L.size());
            assert(L0.rows() == L.rows());
            assert(L0.cols() == L.cols());
        }

        int num_runs = 1;
        for (int run = 0; run < num_runs; run++)
        {
            bool update_success = igl::arap_dof_update(arap_dof, B_eq, L0, max_iters, tol, L);
            if (!update_success)
            {
                cout << __FILE__ << " " << __LINE__ << "auto_dof update fail.............\n";
                return false;
            }
            else
            {
                // cout << __FILE__ << " " << __LINE__ << "auto_dof update success.............\n";
            }
        }

        MatrixXd Lstack;
        uncolumnize<double, Dynamic>(L, dim, dim + 1, 2, Lstack);
        T.block(0, 0, Lstack.rows(), Lstack.cols()) = Lstack;
    }

    return true;
}

bool XToy::update_animation(double t)
{
    if (animation.size() == 0)
    {
        return false;
    }

    // indices of current keyframes
    size_t a, b;
    // factor from keyframe a to b where at
    double f;
    bool still_animated = animation.get_frame(t, a, b, f);
    // filter based on transitions
    f = Animation<BoneBoneCopyMap>::filter(
        animation[a].transition, animation[b].transition, f);
    // interpolate bones
    lerp(animation[a].state, animation[b].state, f);
    // this stores the result into Skinning::skel->roots (pointers to skel->roots
    // are setup in _each_ keyframe in "animation")
    return still_animated;
}

bool XToy::start_animating()
{
    bool was_animating = animating;
    if (was_animating)
    {
        /***/ printf("Cannot start animation as we are already animating...\n");
        return false;
    }

    // Always add current frame to stack (it will be popped when animation is
    // stopped)
    animation.push_back(
        KeyFrame<BoneBoneCopyMap>(
            BoneBoneCopyMap(skel->roots, false),
            0,
            transition_type));

    animation_start_seconds = igl::get_seconds();
    anim_timer = 0.0f;
    animating = true;
    cout << __FILE__ << " " << __LINE__ << " start animation" << endl;
    return animating;
}

bool XToy::stop_animating()
{
    bool was_animating = animating;
    if (!was_animating)
    {
        /***/ printf("Cannot stop animation as we are not animating...\n");
        return false;
    }

    // better be animating
    assert(animating);
    // stop animating
    animating = false;

    assert(animation.size() >= 1);
    // Pop last frame
    animation.pop_back();

    // animation better be stopped
    assert(animating == false);
    cout << __FILE__ << " " << __LINE__ << " stop animation" << endl;
    return animating;
}

void XToy::compute_bone_chordal_axis()
{
    cout << __FILE__ << " " << __LINE__ << " compute_bone, m_subToys=" << m_subToys.size() << endl;
    for (int idx = 0; idx < m_subToys.size(); idx++)
    {
        XSubToy *subToy = m_subToys[idx];
        cout << __FILE__ << " " << __LINE__ << " " << subToy->name() << " is_bone_created=" << is_bone_created[subToy] << " chordal=" << subToy->m_vAxisLines.size() << endl;
        if (is_bone_created[subToy] || subToy->m_vAxisLines.size() <= 0)
            continue;

        int layerid = subToy->m_layerID;
        for (int i = 0; i < subToy->m_vAxisLines.size(); i++)
        {
            // layerid * L_THICK
            const Pnt3 &p_st = subToy->m_vAxisLines[i].Getcpt(0);
            const Pnt3 &p_end = subToy->m_vAxisLines[i].Getcpt(1);
            cout << "p_st=" << p_st.transpose() << endl;
            {
                Bone *root = new Bone(skel, NULL, p_st);
                skel->roots.insert(skel->roots.end(), root);
                root->set_wi(-1);
                Bone *parent = root;
                int j = 0;
                Vector3d offset(0, 0, 0);
                for (j = 0; j < subToy->m_vAxisLines[i].GetiptCount(); j++)
                {
                    const Pnt3 &p = subToy->m_vAxisLines[i].Getipt(j);
                    offset = p - parent->rest_tip();
                    Bone *child = new Bone(skel, parent, offset);
                    child->set_wi(j);
                    parent = child;
                    cout << j << " " << p.transpose() << endl;
                }
                offset = p_end - parent->rest_tip();
                Bone *end = new Bone(skel, parent, offset);
                cout << "p_end=" << p_end.transpose() << endl;
            }
            cout << "--------------------\n";
        }
        is_bone_created[subToy] = true;
    }
}

void XToy::compute_bone_medial_axis()
{
    // Construct the whole-body skeleton
    cout << __FILE__ << " " << __LINE__ << " compute_bone, m_subToys=" << m_subToys.size() << endl;
    if (m_subToys.size() <= 1)
    {
        //ToDO: judge the toy's type is torso
        XSubToy *subToy = m_subToys[0];
        cout << __FILE__ << " " << __LINE__ << " autoSkel=" << subToy->m_autoSkel << endl;
        subToy->ConstructSkeleton(skel, subToy->m_layerDepth);
    }
    else
    {
        cout << __FILE__ << " " << __LINE__ << " construct skeleton for all toys" << endl;
        std::list<XSubToy *> Q;
        for (std::vector<XSubToy *>::iterator it = m_subToyRoots.begin();
             it != m_subToyRoots.end(); it++)
        {
            (*it)->ConstructSkeleton(skel, (*it)->m_layerDepth);
            Q.push_back(*it);
        }

        while (!Q.empty())
        {
            XSubToy *parent = Q.front();
            Q.pop_front();

            for (std::vector<XSubToy *>::iterator it = parent->m_children.begin();
                 it != parent->m_children.end(); it++)
            {
                XSubToy *child = (*it);
                child->ConstructSkeleton(skel, child->m_attachPoint, child->m_layerDepth);
                // find attachBone in parent's skel
                // Bone *attachBone = nullptr;
                // double minDis = std::numeric_limits<double>::max();
                // if (parent->m_boneRoot && child->m_boneRoot)
                // {
                //     for (Bone *b : parent->m_boneRoot->children)
                //     {
                //         double dis = (b->rest_tip() - child->m_boneRoot->rest_tip()).norm();
                //         if (dis < minDis)
                //         {
                //             minDis = dis;
                //             attachBone = b;
                //         }
                //     }
                //     child->m_boneRoot->set_parent(attachBone);
                // }
                Q.push_back(child);
            }
        }
    }
}

void XToy::add_sym_skel(SymetryPlane sym_plane, Bone *sel_bone)
{
    if (!sel_bone)
        return;
    // warning: only support copying a bone with its children
    // if copy a portion of bones on SubToy, the parent-children relationship will be wrong
    if (sel_bone && sel_bone->parent) // new bone symetrically
    {
        Pnt3 oPos = sel_bone->rest_tip(); // old pos
        Pnt3 nPos = oPos;                 // new pos
        Pnt3 cent = sel_bone->parent->rest_tip();
        if (sym_plane == SYM_XY_PLANE)
        {
            nPos.z() = 2 * cent.z() - oPos.z();
        }
        else if (sym_plane == SYM_YZ_PLANE)
        {
            nPos.x() = 2 * cent.x() - oPos.x();
        }

        Bone *root = new Bone(skel, sel_bone->parent, nPos - cent);
        std::list<Bone *> Q, Q1;
        Q.push_back(sel_bone);
        Q1.push_back(root);
        while (!Q.empty())
        {
            Bone *parent = Q.front();
            Bone *newParent = Q1.front();
            Q.pop_front();
            Q1.pop_front();

            for (Bone *child : parent->children)
            {
                const Pnt3 &oldPos = child->rest_tip();
                Pnt3 symPos(oldPos.x(), oldPos.y(), oldPos.z());
                if (sym_plane == SYM_XY_PLANE)
                {
                    symPos.z() = 2 * cent.z() - oldPos.z();
                }
                else if (sym_plane == SYM_YZ_PLANE)
                {
                    symPos.x() = 2 * cent.x() - oldPos.x();
                }
                Vector3d off = symPos - newParent->rest_tip();
                Bone *b = new Bone(skel, newParent, off);
                Q.push_back(child);
                Q1.push_back(b);
            }
        }
    }
}

void XToy::add_sym_skel(SymetryPlane sym_plane, Bone *sel_bone, double plane_pos)
{
    if (!sel_bone)
        return;
    // warning: only support copying a bone with its children
    // if copy a portion of bones on SubToy, the parent-children relationship will be wrong
    if (sel_bone) // new bone symetrically
    {
        Pnt3 oPos = sel_bone->rest_tip(); // old pos
        Pnt3 nPos = oPos;                 // new pos
        if (sym_plane == SYM_XY_PLANE)
        {
            nPos.z() = 2 * plane_pos - oPos.z();
        }
        else if (sym_plane == SYM_YZ_PLANE)
        {
            nPos.x() = 2 * plane_pos - oPos.x();
        }

        Bone *root = new Bone(skel, sel_bone->parent, nPos);
        skel->roots.insert(skel->roots.end(), root);
        std::list<Bone *> Q, Q1;
        Q.push_back(sel_bone);
        Q1.push_back(root);
        while (!Q.empty())
        {
            Bone *parent = Q.front();
            Bone *newParent = Q1.front();
            Q.pop_front();
            Q1.pop_front();

            for (Bone *child : parent->children)
            {
                const Pnt3 &oldPos = child->rest_tip();
                Pnt3 symPos(oldPos.x(), oldPos.y(), oldPos.z());
                if (sym_plane == SYM_XY_PLANE)
                {
                    symPos.z() = 2 * plane_pos - oldPos.z();
                }
                else if (sym_plane == SYM_YZ_PLANE)
                {
                    symPos.x() = 2 * plane_pos - oldPos.x();
                }
                Vector3d off = symPos - newParent->rest_tip();
                Bone *b = new Bone(skel, newParent, off);
                Q.push_back(child);
                Q1.push_back(b);
            }
        }
    }
}

static bool pick_mesh(const int sx, const int sy, const int width, const int height, float *mvp, TMesh &mesh,
                      int &fid, Vec3 &bc)
{
    MatrixXd &V = mesh.GetVMat();
    MatrixXi &F = mesh.GetFMat();
    return EGL::unproject_onto_mesh(sx, sy, width, height, mvp, V, F, fid, bc);
}

int XToy::pick_subtoy_impl(const int sx, const int sy,
                           const int width, const int height,
                           float *mvp)
{
    using namespace std;
    using namespace Eigen;

    double minz = std::numeric_limits<double>::max();
    double minz_subtoyid = -1;
    int n = static_cast<int>(m_subToys.size());
    for (int i = 0; i < n; i++)
    {
        XSubToy *subToy = m_subToys[i];
        Pnt3 bc, cent, scent; //screen center of picked point on the picked face
        int fid;
        if (pick_mesh(sx, sy, width, height, mvp, subToy->m_curMesh3D, fid, bc))
        {
            cent = subToy->m_curMesh3D.GetFPos(fid, bc);
            EGL::project(cent.x(), cent.y(), cent.z(), width, height, mvp, scent[0], scent[1], scent[2]);
            if (scent.z() < minz)
            {
                minz = scent.z();
                minz_subtoyid = subToy->GetID();
                m_pickFid = fid; // maybe useful in the future
                m_pickFbc = bc;
                m_pickFPnt = cent;
            }
        }
    }
    return minz_subtoyid;
}

bool XToy::pick_subtoy(const int sx, const int sy,
                       const int width, const int height,
                       float *mvp,
                       bool ctrl_down)
{
    m_curSubToyIdx = pick_subtoy_impl(sx, sy, width, height, mvp);

    if (m_curSubToyIdx >= 0)
    {
        XSubToy *subToy = m_subToys[m_curSubToyIdx];
        bool is_selected = subToy->m_isSelected;
        M_DEBUG << " sel_toyid=" << m_curSubToyIdx
                << " ctrl_down=" << ctrl_down << " is_selected=" << is_selected << endl;
        subToy->m_isSelected = ctrl_down ? (!is_selected) : true;
        M_DEBUG << "subToy->m_isSelected=" << subToy->m_isSelected << endl;
    }
    return m_curSubToyIdx >= 0;
}

bool XToy::pick_parent_subtoy(const int sx, const int sy,
                              const int width, const int height,
                              float *mvp)
{
    m_parentToyIdx = pick_subtoy_impl(sx, sy, width, height, mvp);
    if (m_parentToyIdx)
    {
        m_usePickedParent = true;
    }
    return m_parentToyIdx >= 0;
}

void XToy::clear_deform_data()
{
    OW.resize(0, 0);
    T.resize(0, 0);
    M.resize(0, 0);
}

bool XToy::move_subtoy(const int sx, const int sy,
                       const int width, const int height,
                       float *mvp)
{
    //!!! ToDO: adjust parent-child relationship
    if (m_curSubToyIdx == -1)
        return false;
    XSubToy *subToy = m_subToys[m_curSubToyIdx];
    Pnt3 oPos = subToy->m_curMesh3D.GetFPos(m_pickFid, m_pickFbc); // old position
    Pnt3 sOPos;                                                    // get z depth
    EGL::project(oPos.x(), oPos.y(), oPos.z(), width, height, mvp, sOPos[0], sOPos[1], sOPos[2]);

    Pnt3 nPos; // new position
    EGL::unproject(sx, sy, sOPos[2], width, height, mvp, nPos[0], nPos[1], nPos[2]);
    Vec3 trans_update = nPos - oPos;

    // M_DEBUG << endl
    //      << " oPos=" << oPos.transpose() << endl
    //      << " sOPos=" << sOPos.transpose() << endl
    //      << " nPos=" << nPos.transpose() << endl
    //      << " sx=" << sx << " sy=" << sy
    //      << " trans_update=" << trans_update.transpose() << endl
    //      << endl;

    // contours, meshes, skeleton
    subToy->Translate(trans_update);
    if (subToy->m_autoSkel) // ToDO: temporary bug fix, to be changed in the future if we could extract skeleton from 3D mesh
    {
        SetupParentChildRelation3D(subToy);
        global_skel_optim();
        visualize_global_sskel();
    }
    m_isSubToyMoved = true;
}

void XToy::rotate_subtoy(int axis, double angle)
{
    if (m_curSubToyIdx == -1)
        return;
    XSubToy *subToy = m_subToys[m_curSubToyIdx];
    double rad = angle * M_PI / 180.0;
    // M_DEBUG << "XToy::rotate_subtoy rad=" << rad << " axis" << axis << endl;

    // contours, meshes, skeleton
    subToy->Rotate(axis, rad);
    if (subToy->m_autoSkel) // ToDO: temporary bug fix, to be changed in the future if we could extract skeleton from 3D mesh
    {
        build_global_sskel();
        global_skel_optim();
        visualize_global_sskel();
    }
}

void XToy::scale_subtoy(double scl)
{
    if (m_curSubToyIdx == -1)
        return;
    XSubToy *subToy = m_subToys[m_curSubToyIdx];
    subToy->Scale(scl);

    if (subToy->m_autoSkel) // ToDO: temporary bug fix, to be changed in the future if we could extract skeleton from 3D mesh
    {
        global_skel_optim();
        visualize_global_sskel();
    }
}

void XToy::translate_sskel(XSubToy *subToy, const Vec3 &trans_update)
{
    for (SSNodeSub *node : m_sskel.nodes)
    {
        if (node->subToy == subToy)
        {
            node->pos += trans_update;
        }
    }
}

//------------------------Painting-------------------------------------------------------
UINT XToy::CreateTexture()
{
    Texture *pTex = new Texture();
    m_textures.push_back(pTex);
    return static_cast<UINT>(m_textures.size() - 1);
}

Texture *XToy::GetTexture(UINT i)
{
    if (m_textures.size() < 1 || i >= m_textures.size())
        return NULL;
    return m_textures[i];
}

void XToy::ClearTexture()
{
    for (int i = 0; i < m_textures.size(); ++i)
        delete m_textures[i];
    m_textures.clear();
}

int XToy::GetPaintColorIdx(const ColorI &color)
{
    int colorIdx = -1;

    BYTE r, g, b;
    r = color.r;
    g = color.g;
    b = color.b;

    int texWidth = m_texWidth;
    int blockWidth = m_blockWidth;
    int blockSize = texWidth / blockWidth;
    int colorSize = (int)m_vPaintColor.size();

    for (int i = 0; i < colorSize; i++)
    {
        ColorI colorTmp = m_vPaintColor[i];

        if (r == colorTmp.r && g == colorTmp.g && b == colorTmp.b)
        {
            colorIdx = i;
            break;
        }
    }

    if (colorIdx == -1)
    {
        Texture *tex = GetTexture(0);
        ColorI *imageData = tex->GetImage().GetBuffer();
        CPoint blockPos((colorSize % (blockSize / 3)) * blockWidth * 3, (colorSize / (blockSize / 3)) * blockWidth);
        int position;

        int nSize = tex->GetImage().Height() * tex->GetImage().Width();

        for (int i = 0; i < 3 * blockWidth; i++)
        {
            for (int j = 0; j < blockWidth; j++)
            {
                position = blockPos.x() + i + (blockPos.y() + j) * texWidth;

                if (position >= nSize)
                    continue;

                imageData[position].b = b;
                imageData[position].g = g;
                imageData[position].r = r;
            }
        }

        m_vPaintColor.push_back(color);
        colorIdx = (int)m_vPaintColor.size() - 1;
    }

    return colorIdx;
}

void XToy::InitTexturePaint(const ColorI &color)
{
    if (m_textures.size() == 0)
    {
        int texIdx = CreateTexture();
        Texture *tex = GetTexture(texIdx);
        tex->Reload(false);
        cout << __FILE__ << " " << __LINE__ << " InitTexturePaint" << endl;
    }
    int colorIdx = GetPaintColorIdx(color);
    XSubToy *subToy = m_subToys[m_curSubToyIdx];
    TMesh &mesh = subToy->m_curMesh3D;
    mesh.InitTMeshTexPaint(m_texWidth, m_blockWidth, colorIdx);

    // ToDO: remove
    GetTexture(0)->Save("output/tex0.jpg");
}

void XToy::TexturePaintDraw(const ColorI &paintColor)
{
    XSubToy *subToy = m_subToys[m_curSubToyIdx];
    TMesh &mesh = subToy->m_curMesh3D;
    TexPaint &texPaint = mesh.m_texPaint;
    FreehandLine &texPaintLine = texPaint.m_texPaintLine;
    int colorIdx = GetPaintColorIdx(paintColor);
    texPaint.m_painterWidth = m_brushPainter.m_size / 4.0;
    Float avgLen = mesh.max_edge_len();

    if (m_brushPainter.m_type == 1)
    {
        //ToDO: smooth brush
    }
    else
    {
        // raw brush
        vector<ColorFace> colorFaces;
        vector<DrawFace> drawFaces;
        mesh.TexPaintDrawMesh(colorFaces, drawFaces);
    }
}

void XToy::pick_all_subtoys()
{
    for (XSubToy *subToy : m_subToys)
    {
        subToy->m_isSelected = true;
    }
}

void XToy::unpick_all_subtoys()
{
    m_parentToyIdx = -1;
    m_usePickedParent = false;
    for (XSubToy *subToy : m_subToys)
    {
        subToy->m_isSelected = false;
    }
}

void XToy::pick_current_subtoy()
{
    for (XSubToy *subToy : m_subToys)
    {
        subToy->m_isSelected = false;
    }

    XSubToy *subToy = GetCurToy();
    if (subToy)
        subToy->m_isSelected = true;
}

void XToy::global_skel_optim() // default global optimization
{
    build_global_sskel(); // dp simplfication

    // use default parameter to do merging, trimming
    pick_all_subtoys();
    merge_junction_nodes(m_sskel, m_merge_thresh);
    prune_short_jt_edges(m_sskel, m_prune_thresh);
    collapse_short_edges(m_sskel, m_collapse_thresh);
    pick_current_subtoy();

    visualize_global_sskel();
}

void XToy::merge_prune_collapse()
{
    merge_junction_nodes(m_sskel, m_merge_thresh);
    prune_short_jt_edges(m_sskel, m_prune_thresh);
    collapse_short_edges(m_sskel, m_collapse_thresh);
}

static void find_attach_joint(SSkel<SSNode> &parent, SSkel<SSNode> &child, SSNode *&pnode, SSNode *&cnode)
{
    std::vector<SSNode *> junc_nodes, term_nodes;
    child.collect_nodes(junc_nodes, term_nodes);

    double mindisJ = std::numeric_limits<double>::max(); // to junc
    SSNode *pnode_mindisJ = nullptr;
    SSNode *cnode_mindisJ = nullptr;
    for (SSNode *pn : parent.nodes)
    {
        for (SSNode *cn : junc_nodes)
        {
            double dis = (cn->pos - pn->pos).norm();
            if (dis < mindisJ)
            {
                pnode_mindisJ = pn;
                cnode_mindisJ = cn;
                mindisJ = dis;
            }
        }
    }

    double mindisT = std::numeric_limits<double>::max(); // to terminal
    SSNode *pnode_mindisT = nullptr;
    SSNode *cnode_mindisT = nullptr;
    for (SSNode *pn : parent.nodes)
    {
        for (SSNode *cn : term_nodes)
        {
            double dis = (cn->pos - pn->pos).norm();
            if (dis < mindisT)
            {
                pnode_mindisT = pn;
                cnode_mindisT = cn;
                mindisT = dis;
            }
        }
    }

    if (junc_nodes.size() == 0) // no junc nodes
    {
        pnode = pnode_mindisT;
        cnode = cnode_mindisT;
    }
    else
    {
        // junction node 's distance is similar to terminal node's distance, favors junction node
        if (mindisJ < mindisT + 15)
        {
            pnode = pnode_mindisJ;
            cnode = cnode_mindisJ;
        }
        else
        {
            pnode = pnode_mindisT;
            cnode = cnode_mindisT;
        }
    }
}

void XToy::build_global_sskel()
{
#ifdef SSKEL_DEBUG
    M_DEBUG << "build_global_sskel" << endl;
#endif
    m_sskel.clear();
    for (XSubToy *subToy : m_subToys)
    {
        add_subskel_to_global(subToy);
    }

    std::list<XSubToy *> Q;
    Q.insert(Q.begin(), m_subToyRoots.begin(), m_subToyRoots.end());
    while (!Q.empty()) // this will not work when parts organized as a circle
    {
        XSubToy *parent = Q.front();
        Q.pop_front();
        for (XSubToy *child : parent->GetChildren())
        {
            if (parent->m_autoSkel && child->m_autoSkel)
            {
                connect_subskel(parent, child); // ToDO: this may induce potential bugs in the future
                Q.push_back(child);
            }
        }
    }

    copy_sskel(m_sskel, m_rawSSkel);
    // #ifdef SSKEL_DEBUG
    // M_DEBUG << "build_global_sskel after connect bone" << endl;
    // m_sskel.print();
    // #endif
}

void XToy::change_bone_complexity(int level) // user-controlled branch-level simplification
{
    int dp_thresh = 20 - level;
    M_DEBUG << "Branch Level dp_thresh=" << dp_thresh << endl;
    for (XSubToy *subToy : m_subToys)
    {
        if (subToy->m_isSelected && subToy->m_autoSkel)
        {
            subToy->interactive_simplify_skeleton(dp_thresh);
        }
    }
    build_global_sskel();

    merge_junction_nodes(m_sskel, m_merge_thresh);
    prune_short_jt_edges(m_sskel, m_prune_thresh);
    collapse_short_edges(m_sskel, m_collapse_thresh);

    visualize_global_sskel();
}

double XToy::avg_bone_len()
{
    return m_sskel.avg_edge_len();
}

void XToy::merge_junc_nodes(int merge_thresh)
{
    // 1. merge junction nodes closest to each other
    M_DEBUG << "merge_junc_nodes merge_thresh=" << merge_thresh << endl;
    copy_sskel(m_rawSSkel, m_sskel);
    merge_junction_nodes(m_sskel, merge_thresh);
    // restore state
    prune_short_jt_edges(m_sskel, m_prune_thresh);
    collapse_short_edges(m_sskel, m_collapse_thresh);

    m_merge_thresh = merge_thresh;
    visualize_global_sskel();
}

void XToy::collect_nodes(SSkel<SSNodeSub> &sskel,
                         std::vector<SSNodeSub *> &junc_nodes,
                         std::vector<SSNodeSub *> &term_nodes)
{
    typedef SSNodeSub NodeT;
    typedef SSEdge<NodeT> EdgeT;

    for (NodeT *node : sskel.nodes)
    {
        if (node->subToy->m_isSelected)
        {
            if (node->type() == NODE_JUNCTION)
            {
                junc_nodes.push_back(node);
            }
            else if (node->type() == NODE_TERMINAL)
            {
                term_nodes.push_back(node);
            }
        }
    }
}

void XToy::merge_junction_nodes(SSkel<SSNodeSub> &sskel, double merge_thresh)
{
    typedef SSNodeSub NodeT;
    typedef SSEdge<NodeT> EdgeT;

    std::vector<NodeT *> junc_nodes, term_nodes;
    collect_nodes(sskel, junc_nodes, term_nodes);
    M_DEBUG << "merge_junction_nodes " << junc_nodes.size() << endl;

    std::unordered_map<NodeT *, int> cluster_id;
    int num_cluster = 0;
    for (NodeT *junc_node : junc_nodes)
    {
        cluster_id[junc_node] = -1;
    }

    // 1.a build junction nodes cluster
    for (NodeT *junc_node : junc_nodes)
    {
        if (cluster_id[junc_node] == -1)
        {
            std::list<NodeT *> Q;
            Q.push_back(junc_node);

            while (!Q.empty())
            {
                NodeT *cur_node = Q.front();
                Q.pop_front();
                cluster_id[cur_node] = num_cluster;
                int n = static_cast<int>(cur_node->adj_edges.size());
                for (int i = 0; i < n; ++i)
                {
                    EdgeT *e = static_cast<EdgeT *>(cur_node->adj_edges[i]);
                    if (e->len() < merge_thresh)
                    {
                        NodeT *adj_node = (e->node0 == cur_node) ? e->node1 : e->node0;
                        if (adj_node->type() == NODE_JUNCTION && cluster_id[adj_node] == -1)
                        {
                            Q.push_back(adj_node);
                        }
                    }
                }
            }

            ++num_cluster;
        }
    }

    M_DEBUG << "num_cluster= " << num_cluster << endl;
    std::vector<std::vector<NodeT *>> clusters;
    clusters.resize(num_cluster);
    for (auto it = cluster_id.begin(); it != cluster_id.end(); ++it)
    {
        NodeT *node = it->first;
        int id = it->second;
        clusters[id].push_back(node);
    }

    // 1.b merge junction nodes in one cluster
    int c_id = 0;
    for (std::vector<NodeT *> &cluster : clusters)
    {
        if (cluster.size() > 1)
        {
            // find cluster center, cluster internal edges, cluster peripheral edges
            Pnt3 cent(0, 0, 0);
            double radius = 0;
            for (NodeT *node : cluster)
            {
                cent += node->pos; // center
                radius += node->radius;
            }
            cent /= (double)cluster.size();
            radius /= (double)cluster.size();

            M_DEBUG << "cluster" << c_id << " size=" << cluster.size() << endl;
            // add new node to sskel
            NodeT *newNode = sskel.add_node(cent);
            newNode->set_radius(radius);
            newNode->set_subtoy(cluster[0]->subToy);
            // replace out_es' in_cluster_node as center(newNode)
            std::set<EdgeT *> in_es;
            for (NodeT *node : cluster)
            {
                int n = static_cast<int>(node->adj_edges.size());
                for (int i = 0; i < n; ++i)
                {
                    EdgeT *e = static_cast<EdgeT *>(node->adj_edges[i]);
                    NodeT *adj_node = (e->node0 == node) ? e->node1 : e->node0;
                    if (std::find(cluster.begin(), cluster.end(), adj_node) != cluster.end())
                    {
                        in_es.insert(e);
                    }
                    else
                    {
                        if (e->node0 == node)
                        {
                            e->node0 = newNode;
                        }
                        else if (e->node1 == node)
                        {
                            e->node1 = newNode;
                        }
                        newNode->adj_edges.push_back(e); // peripheral edges
                    }
                }
            }
            // destroy in_cluster_edges
            for (EdgeT *edge : in_es)
            {
                sskel.delete_edge_fromSkel(edge);
            }
            // destroy in_cluster_nodes
            for (NodeT *node : cluster)
            {
                sskel.delete_node_fromSkel(node);
            }
            M_DEBUG << "merge_junction_nodes successfully! cluster" << c_id << endl;
        }

        ++c_id;
    }
}

double XToy::max_jt_edge_len()
{
    double len = m_sskel.max_jt_edge_len();
    if (len < 0)
        return m_prune_thresh;
    else
        return len;
}

void XToy::prune_jt_edges(int prune_thresh)
{
    // 2. prune JT bone
    M_DEBUG << "prune_jt_edges prune_thresh=" << prune_thresh << endl;
    copy_sskel(m_rawSSkel, m_sskel);
    merge_junction_nodes(m_sskel, m_merge_thresh); // restore state
    prune_short_jt_edges(m_sskel, prune_thresh);
    collapse_short_edges(m_sskel, m_collapse_thresh); // restore state
    m_prune_thresh = prune_thresh;

    visualize_global_sskel();
}

void XToy::find_jt_edges(SSkel<SSNodeSub> &sskel, std::vector<SSEdge<SSNodeSub> *> &jt_edges)
{
    typedef SSEdge<SSNodeSub> EdgeT;
    for (EdgeT *e : sskel.edges)
    {
        if (e->node0->subToy->m_isSelected && e->node1->subToy->m_isSelected)
        {
            if (e->node0->type() == NODE_JUNCTION && e->node1->type() == NODE_TERMINAL)
            {
                jt_edges.push_back(e);
            }
            else if (e->node1->type() == NODE_JUNCTION && e->node0->type() == NODE_TERMINAL)
            {
                jt_edges.push_back(e);
            }
        }
    }
}

void XToy::prune_short_jt_edges(SSkel<SSNodeSub> &sskel, int prune_thresh)
{
    typedef SSNodeSub NodeT;
    typedef SSEdge<NodeT> EdgeT;

    M_DEBUG << " prune_short_jt_edges" << endl;

    while (true)
    {

        std::vector<EdgeT *> jt_es;
        find_jt_edges(sskel, jt_es);
        if (jt_es.size() == 0)
            break;

        std::sort(jt_es.begin(), jt_es.end(), [](EdgeT *a, EdgeT *b)
                  { return a->len() < b->len(); });

        EdgeT *e = jt_es[0];
        M_DEBUG << "jt_es[0]=" << jt_es.front()->len() << " jt_es.back()=" << jt_es.back()->len() << endl;
        if (e->len() > prune_thresh)
            break;

        NodeT *junc_node, *adj_node;
        if (e->node0->type() == NODE_JUNCTION)
        {
            junc_node = e->node0;
            adj_node = e->node1;
        }
        else
        {
            junc_node = e->node1;
            adj_node = e->node0;
        }

        junc_node->adj_edges.erase(std::remove(junc_node->adj_edges.begin(), junc_node->adj_edges.end(), e),
                                   junc_node->adj_edges.end());
        sskel.delete_edge_fromSkel(e);
        sskel.delete_node_fromSkel(adj_node);
        M_DEBUG << " prune_short_jt_edges successfully" << endl;
    }
}

double XToy::max_bone_len()
{
    return m_sskel.max_edge_len();
}

void XToy::collapse_short_edges(int collapse_thresh)
{
    // ToDO: if collapsing make the bone outside mesh, do not collapse!!!
    // 3. collapse internal short bones: prune shortest internal edges first
    M_DEBUG << "collapse_short_edges collapse_thresh=" << collapse_thresh << endl;
    collapse_short_edges(m_sskel, collapse_thresh);
    copy_sskel(m_rawSSkel, m_sskel);               // restore state
    merge_junction_nodes(m_sskel, m_merge_thresh); // restore state
    prune_short_jt_edges(m_sskel, m_prune_thresh); // restore state

    collapse_short_edges(m_sskel, collapse_thresh);
    m_collapse_thresh = collapse_thresh;
    visualize_global_sskel();
}

void XToy::collapse_short_edges(SSkel<SSNodeSub> &sskel, int collapse_thresh)
{
    typedef SSNodeSub NodeT;
    typedef SSEdge<NodeT> EdgeT;
    M_DEBUG << "collapse_short_edges " << collapse_thresh << endl;
    // ToDO: do not collapse if the collapsed edge out of shape

    while (true)
    {
        if (sskel.edges.size() == 1) // only has one bone left
            break;

        std::vector<EdgeT *> nn_edges;
        int n = static_cast<int>(sskel.edges.size());
        for (int i = 0; i < n; ++i)
        {
            EdgeT *e = static_cast<EdgeT *>(sskel.edges[i]);
            if (e->node0->subToy->m_isSelected && e->node1->subToy->m_isSelected)
                if (e->node0->type() != NODE_TERMINAL && e->node1->type() != NODE_TERMINAL)
                    nn_edges.push_back(e);
        }

        if (nn_edges.size() < 1)
            break;

        std::sort(nn_edges.begin(), nn_edges.end(), [&](EdgeT *a, EdgeT *b)
                  { return a->len() < b->len(); }); // This is important to prevent duplicate edges or chaos edge order in orignal sskel

        EdgeT *e = nn_edges[0];
        if (e->len() >= collapse_thresh) // the minimum edge's length is greater than thresh
            break;
        M_DEBUG << "collapse edge " << *e << endl;
        const Pnt3 &p0 = e->node0->pos;
        const Pnt3 &p1 = e->node1->pos;
        XSubToy *subToy0 = e->node0->subToy;
        XSubToy *subToy1 = e->node1->subToy;
        SSNodeSub *newnode = sskel.collapse_edge(e);
        const Pnt3 &p = newnode->pos;
        if ((p - p0).norm() < (p - p1).norm())
        {
            newnode->set_subtoy(subToy0);
        }
        else
        {
            newnode->set_subtoy(subToy1);
        }
        // M_DEBUG << "after collapsing " << endl;
        // m_sskel.print();
    }
}

void XToy::visualize_global_sskel()
{
    // M_DEBUG << "visualize_global_sskel" << endl;
    visualize_global_sskel(skel);
    visualize_global_sskel(restSkel);
    clear_deform_data();
}

void XToy::visualize_global_sskel(Skeleton<Bone> *vis_skel)
{
    destroy_bone_roots(vis_skel->roots);
    for (int i = 0; i < m_subToyRoots.size(); ++i)
    {
        M_DEBUG << "root subToy " << m_subToyRoots[i]->name() << endl;
        XSubToy *parent = m_subToyRoots[i];
        SSNodeSub *rootNode = find_root(parent);
        if (rootNode != nullptr)
        {
            Bone *rootBone = new Bone(vis_skel, NULL, rootNode->pos);
            vis_skel->roots.push_back(rootBone);
            sskel_to_bone(m_sskel, rootNode, rootBone);
        }
    }
    M_DEBUG << "visualize_global_sskel successfully!" << endl;
}

SSNodeSub *XToy::find_root(XSubToy *rootToy)
{
    TMesh &mesh = rootToy->m_curMesh3D;
    Pnt3 cent = Centroid(mesh.GetVMat(), mesh.GetFMat());
    SSNodeSub *best = nullptr;
    for (SSNodeSub *node : m_sskel.nodes)
    {
        if (node->subToy == rootToy)
        {
            best = node;
            break;
        }
    }
    // found node with maximum adjacent edges which is near to rootToy's barycenter
    for (SSNodeSub *node : m_sskel.nodes)
    {
        if (node->subToy == rootToy && (node->pos - cent).norm() < (best->pos - cent).norm() &&
            node->adj_edges.size() > best->adj_edges.size())
            best = node;
    }
    // M_DEBUG << "best=" << *best << endl;
    return best;
}

void XToy::add_subskel_to_global(XSubToy *subToy)
{
    // ToDO: temporary bug fix, to be changed in the future if we could extract skeleton from 3D mesh
    if (!subToy->m_autoSkel)
        return;
    typedef typename SSkel<SSNode>::EdgeT SubE;
    SSkel<SSNode> &subSSkel = subToy->m_sskel;
    std::unordered_map<SSNode *, SSNodeSub *> globalNodes; // mapping between global-skeleton and local skeleton
#ifdef SSKEL_DEBUG
    M_DEBUG << "add_subskel_to_global " << subToy->name() << endl;
#endif

    for (SSNode *sn : subSSkel.nodes)
    {
        SSNodeSub *node = m_sskel.add_node(sn->pos);
        node->set_radius(sn->radius);
        node->set_subtoy(subToy);
        globalNodes[sn] = node;
        // M_DEBUG << *sn << " SSNodeSub=" << *node << endl;
    }
    for (int i = 0; i < subSSkel.edges.size(); ++i)
    {
        SubE *se = subSSkel.get_edge(i);
        SSNodeSub *node0 = globalNodes[se->node0];
        SSNodeSub *node1 = globalNodes[se->node1];
        SSEdge<SSNodeSub> *edge = m_sskel.add_edge(node0, node1);
        // M_DEBUG << *se << " SSEdgeSub=" << *edge << endl;
    }

    SSNode *root;
    if (!subToy->IsRoot())
    {
        SSNode *pnode, *cnode;
        find_attach_joint(subToy->m_parent->m_rawSSkel, subToy->m_rawSSkel, pnode, cnode);
        subToy->m_attachPoint = cnode->pos; // raw sskel

        root = subSSkel.find_closest(subToy->m_attachPoint); // sskel for visualization
        subSSkel.set_root(root);
    }
    else
    {
        root = subSSkel.find_node_with_max_branches();
        subSSkel.set_root(root);
    }

    m_subSSkelRoots[subToy] = globalNodes[root];

#ifdef SSKEL_DEBUG
    M_DEBUG << subToy->name() << "  m_subSSkelRoots[subToy]=" << *m_subSSkelRoots[subToy] << endl;
    M_DEBUG << "after add_subskel_to_global" << endl;
    m_sskel.print();
#endif
}

void XToy::connect_subskel(XSubToy *parent, XSubToy *child)
{
    // find_closet_edge(child->GetAttachPoint())
    typedef SSEdge<SSNodeSub> EdgeT;
    SSNodeSub *child_root = m_subSSkelRoots[child];
    const Pnt3 &p2 = child_root->pos;
    EdgeT *e_min = nullptr;
    double dis_min = INT_MAX;
    for (int i = 0; i < m_sskel.edges.size(); ++i)
    {
        EdgeT *e = static_cast<EdgeT *>(m_sskel.get_edge(i));
        if (e->node0->subToy == parent && e->node1->subToy == parent)
        {
            const Pnt3 &p0 = e->node0->pos;
            const Pnt3 &p1 = e->node1->pos;
            Vec3 p01 = p1 - p0;
            Vec3 p02 = p2 - p0;
            Vec3 p12 = p2 - p1;
            double len01 = p01.norm();
            double len02 = p02.norm();
            double len12 = p12.norm();
            double dis;
            if (p01.dot(p02) >= 0 && (p0 - p1).dot(p12) >= 0)
            {
                // p2 lies in othorganal projection area of line p0--p1
                dis = len01 < 1e-3 ? len02 : p01.cross(p02).norm() / len01; // true: line0-1 becomes a point
            }
            else
            { // surpass tail or tip point
                dis = len02 < len12 ? len02 : len12;
            }
            if (dis < dis_min)
            {
                dis_min = dis;
                e_min = e;
            }
        }
    }

    // find closet point on closest edge
    // add_conn_bone_to_global
    const Pnt3 &p0 = e_min->node0->pos;
    const Pnt3 &p1 = e_min->node1->pos;
    Vec3 p01 = p1 - p0;
    Vec3 p02 = p2 - p0;
    Vec3 p12 = p2 - p1;
    double len01 = p01.norm();
    double len02 = p02.norm();
    double len12 = p12.norm();
    // M_DEBUG << " connect_subskel parent_e_min=" << *e_min << endl;
    if (p01.dot(p02) >= 0 && (p0 - p1).dot(p12) >= 0 && len01 > 0.1)
    {
        // p2 lies in othorganal projection area of line p0--p1
        double off = sqrt(len02 * len02 - dis_min * dis_min);
        double t = off / len01;
        Pnt3 p = p0 + p01.normalized() * off;
        double radius = e_min->node0->radius * (1 - t) + e_min->node1->radius * t;
        SSNodeSub *new_node = m_sskel.add_node(p);
        m_sskel.split_edge(e_min, new_node);
        new_node->set_radius(radius);
        new_node->set_subtoy(parent);
        EdgeT *new_edge = m_sskel.add_edge(new_node, child_root);
        M_DEBUG << " new_node " << *new_node << " add_edge=" << *new_edge << endl;
    }
    else
    { // surpass tail or tip point, simply add an edge
        SSNodeSub *p_node = len02 < len12 ? e_min->node0 : e_min->node1;
        EdgeT *e = m_sskel.add_edge(p_node, child_root);
        // M_DEBUG << " p_node " << *p_node << " add_edge=" << *e << endl;
    }
}

void XToy::set_zdepth(XSubToy *subToy, double zdepth)
{
    if (subToy->GetLayerID() == 0)
        return;

    for (SSNode *node : subToy->m_sskel.nodes)
    {
        node->pos(2) = zdepth;
    }

    for (SSNodeSub *node : m_sskel.nodes)
    {
        if (node->subToy == subToy)
            node->pos(2) = zdepth;
    }
}
