#pragma once

#include <iostream>
#include <array>
#include <boost/cstdint.hpp>
#include <boost/iterator/iterator_facade.hpp>

#include "Properties.h"
#include "Euler_operations.h"
#include "graph_circulator.h"
#include "Iterator_range.h"
#include "graph_traits_SMesh.h"

template <typename T>
Iterator_range<T> make_range(const T &b, const T &e)
{
    return Iterator_range<T>(b, e);
}

template <typename T>
Iterator_range<T> make_range(const std::pair<T, T> &p)
{
    return Iterator_range<T>(p.first, p.second);
}

/// Base class for vertex, halfedge, edge, and face index.
///
/// \attention Note that `Index` is not a model of the concept `Handle`,
/// because it cannot be dereferenced.
/// \sa `Vertex_index`, `Halfedge_index`, `Edge_index`, `Face_index`.
template <typename T>
class SM_Index
{
public:
    typedef boost::uint32_t size_type;
    /// Constructor. %Default construction creates an invalid index.
    /// We write -1, which is <a href="https://en.cppreference.com/w/cpp/types/numeric_limits">
    /// <tt>(std::numeric_limits<size_type>::max)()</tt></a>
    /// as `size_type` is an unsigned type.
    explicit SM_Index(size_type _idx = (std::numeric_limits<size_type>::max)()) : idx_(_idx) {}

    /// Get the underlying index of this index
    operator size_type() const { return idx_; }

    /// reset index to be invalid (index=(std::numeric_limits<size_type>::max)())
    void reset() { idx_ = (std::numeric_limits<size_type>::max)(); }

    /// return whether the index is valid, i.e., the index is not equal to `%(std::numeric_limits<size_type>::max)()`.
    bool is_valid() const
    {
        size_type inf = (std::numeric_limits<size_type>::max)();
        return idx_ != inf;
    }

    // Compatibility with OpenMesh handle
    size_type idx() const
    {
        return idx_;
    }

    /// increments the internal index. This operation does not
    /// guarantee that the index is valid or undeleted after the
    /// increment.
    SM_Index &operator++()
    {
        ++idx_;
        return *this;
    }
    /// decrements the internal index. This operation does not
    /// guarantee that the index is valid or undeleted after the
    /// decrement.
    SM_Index &operator--()
    {
        --idx_;
        return *this;
    }

    /// increments the internal index. This operation does not
    /// guarantee that the index is valid or undeleted after the
    /// increment.
    SM_Index operator++(int)
    {
        SM_Index tmp(*this);
        ++idx_;
        return tmp;
    }
    /// decrements the internal index. This operation does not
    /// guarantee that the index is valid or undeleted after the
    /// decrement.
    SM_Index operator--(int)
    {
        SM_Index tmp(*this);
        --idx_;
        return tmp;
    }

    SM_Index operator+=(std::ptrdiff_t n)
    {
        idx_ = size_type(std::ptrdiff_t(idx_) + n);
        return *this;
    }

protected:
    size_type idx_;
};

template <class T>
std::size_t hash_value(const SM_Index<T> &i)
{
    std::size_t ret = i;
    return ret;
}

// Implementation for Surface_mesh::Vertex_index
class SM_Vertex_index
    : public SM_Index<SM_Vertex_index>
{
public:
    SM_Vertex_index() : SM_Index<SM_Vertex_index>((std::numeric_limits<size_type>::max)()) {}

    explicit SM_Vertex_index(size_type _idx) : SM_Index<SM_Vertex_index>(_idx) {}

    template <class T>
    bool operator==(const T &) const = delete;
    template <class T>
    bool operator!=(const T &) const = delete;
    template <class T>
    bool operator<(const T &) const = delete;

    /// are two indices equal?
    bool operator==(const SM_Vertex_index &_rhs) const
    {
        return this->idx_ == _rhs.idx_;
    }

    /// are two indices different?
    bool operator!=(const SM_Vertex_index &_rhs) const
    {
        return this->idx_ != _rhs.idx_;
    }

    /// Comparison by index.
    bool operator<(const SM_Vertex_index &_rhs) const
    {
        return this->idx_ < _rhs.idx_;
    }

    friend std::ostream &operator<<(std::ostream &os, SM_Vertex_index const &v)
    {
        return (os << 'v' << (size_type)v);
    }
};

// Implementation of Surface_mesh::Halfedge_index
class SM_Halfedge_index
    : public SM_Index<SM_Halfedge_index>
{
public:
    // Workaround for a bug in g++4.4 in ADL for function next:
    // we provide the types needed for std::iterator_traits<Surface_mesh::halfedge_index>,
    // although this descriptor is not an iterator.
    typedef void iterator_category;
    typedef void value_type;
    typedef void difference_type;
    typedef void pointer;
    typedef void reference;

    SM_Halfedge_index() : SM_Index<SM_Halfedge_index>((std::numeric_limits<size_type>::max)()) {}

    explicit SM_Halfedge_index(size_type _idx) : SM_Index<SM_Halfedge_index>(_idx) {}

    template <class T>
    bool operator==(const T &) const = delete;
    template <class T>
    bool operator!=(const T &) const = delete;
    template <class T>
    bool operator<(const T &) const = delete;

    /// are two indices equal?
    bool operator==(const SM_Halfedge_index &_rhs) const
    {
        return this->idx_ == _rhs.idx_;
    }

    /// are two indices different?
    bool operator!=(const SM_Halfedge_index &_rhs) const
    {
        return this->idx_ != _rhs.idx_;
    }

    /// Comparison by index.
    bool operator<(const SM_Halfedge_index &_rhs) const
    {
        return this->idx_ < _rhs.idx_;
    }

    friend std::ostream &operator<<(std::ostream &os, SM_Halfedge_index const &h)
    {
        return (os << 'h' << (size_type)h);
    }
};

/// Implementation of Surfae_mesh::Face_index
class SM_Face_index
    : public SM_Index<SM_Face_index>
{
public:
    SM_Face_index() : SM_Index<SM_Face_index>((std::numeric_limits<size_type>::max)()) {}

    explicit SM_Face_index(size_type _idx) : SM_Index<SM_Face_index>(_idx) {}

    template <class T>
    bool operator==(const T &) const = delete;
    template <class T>
    bool operator!=(const T &) const = delete;
    template <class T>
    bool operator<(const T &) const = delete;

    /// are two indices equal?
    bool operator==(const SM_Face_index &_rhs) const
    {
        return this->idx_ == _rhs.idx_;
    }

    /// are two indices different?
    bool operator!=(const SM_Face_index &_rhs) const
    {
        return this->idx_ != _rhs.idx_;
    }

    /// Comparison by index.
    bool operator<(const SM_Face_index &_rhs) const
    {
        return this->idx_ < _rhs.idx_;
    }

    friend std::ostream &operator<<(std::ostream &os, SM_Face_index const &f)
    {
        return (os << 'f' << (size_type)f);
    }
};

/// Implementation of Surface_mesh::Edge_index
class SM_Edge_index
{
public:
    typedef boost::uint32_t size_type;

    SM_Edge_index() : halfedge_((std::numeric_limits<size_type>::max)()) {}

    explicit SM_Edge_index(size_type idx) : halfedge_(idx * 2) {}

    explicit SM_Edge_index(SM_Halfedge_index he) : halfedge_(he) {}

    // returns the internal halfedge.
    SM_Halfedge_index halfedge() const { return halfedge_; }

    // returns the underlying index of this index.
    operator size_type() const { return (size_type)halfedge_ / 2; }

    // compatibility with OpenMesh handles
    size_type idx() const { return (size_type)halfedge_ / 2; }

    // resets index to be invalid (index=(std::numeric_limits<size_type>::max)())
    void reset() { halfedge_.reset(); }

    // returns whether the index is valid, i.e., the index is not equal to (std::numeric_limits<size_type>::max)().
    bool is_valid() const { return halfedge_.is_valid(); }

    template <class T>
    bool operator==(const T &) const = delete;
    template <class T>
    bool operator!=(const T &) const = delete;
    template <class T>
    bool operator<(const T &) const = delete;

    // Are two indices equal?
    bool operator==(const SM_Edge_index &other) const { return (size_type)(*this) == (size_type)other; }

    // Are two indices different?
    bool operator!=(const SM_Edge_index &other) const { return (size_type)(*this) != (size_type)other; }

    // compares by index.
    bool operator<(const SM_Edge_index &other) const { return (size_type)(*this) < (size_type)other; }

    // decrements the internal index. This operation does not
    // guarantee that the index is valid or undeleted after the
    // decrement.
    SM_Edge_index &operator--()
    {
        halfedge_ = SM_Halfedge_index((size_type)halfedge_ - 2);
        return *this;
    }

    // increments the internal index. This operation does not
    // guarantee that the index is valid or undeleted after the
    // increment.
    SM_Edge_index &operator++()
    {
        halfedge_ = SM_Halfedge_index((size_type)halfedge_ + 2);
        return *this;
    }

    // decrements internal index. This operation does not
    // guarantee that the index is valid or undeleted after the
    // decrement.
    SM_Edge_index operator--(int)
    {
        SM_Edge_index tmp(*this);
        halfedge_ = SM_Halfedge_index((size_type)halfedge_ - 2);
        return tmp;
    }

    // increments internal index. This operation does not
    // guarantee that the index is valid or undeleted after the
    // increment.
    SM_Edge_index operator++(int)
    {
        SM_Edge_index tmp(*this);
        halfedge_ = SM_Halfedge_index((size_type)halfedge_ + 2);
        return tmp;
    }

    SM_Edge_index operator+=(std::ptrdiff_t n)
    {
        halfedge_ = SM_Halfedge_index(size_type(std::ptrdiff_t(halfedge_) + 2 * n));
        return *this;
    }

    // prints the index and a short identification string to an ostream.
    friend std::ostream &operator<<(std::ostream &os, SM_Edge_index const &e)
    {
        return (os << 'e' << (size_type)e << " on " << e.halfedge());
    }

    friend std::size_t hash_value(const SM_Edge_index &i)
    {
        return i;
    }

private:
    SM_Halfedge_index halfedge_;
};

template <typename P>
class Surface_mesh
{
    typedef Surface_mesh<P> Self;

public:
    typedef P Point;
    typedef boost::uint32_t size_type;

    typedef SM_Vertex_index Vertex_index;
    typedef SM_Halfedge_index Halfedge_index;
    typedef SM_Face_index Face_index;
    typedef SM_Edge_index Edge_index;

    struct Vertex_connectivity
    {
        // halfedge that points to this vertex
        Halfedge_index halfedge_;
    };

    struct Halfedge_connectivity
    {
        Face_index face_;
        Vertex_index vertex_; // vertex halfedge points to
        Halfedge_index next_halfedge_;
        Halfedge_index prev_halfedge_;
    };

    struct Face_connectivity
    {
        Halfedge_index halfedge_;
    };

    template <typename Index_>
    class Index_iterator
        : public boost::iterator_facade<Index_iterator<Index_>, Index_, std::random_access_iterator_tag>
    {
        typedef boost::iterator_facade<Index_iterator<Index_>, Index_, std::random_access_iterator_tag> Facade;

    public:
        Index_iterator() : hnd_(), mesh_(nullptr) {}
        Index_iterator(const Index_ &h, const Surface_mesh *m)
            : hnd_(h), mesh_(m)
        {

            if (mesh_ && mesh_->has_garbage())
            {
                while (mesh_->has_valid_index(hnd_) && mesh_->is_removed(hnd_))
                {
                    ++hnd_;
                }
            }
        }

    private:
        friend class boost::iterator_core_access;
        void increment()
        {
            // std::cout << __FILE__ << " " << __LINE__ << " hnd_=" << hnd_ << std::endl;
            ++hnd_;
            assert(mesh_ != nullptr);

            if (mesh_->has_garbage())
                while (mesh_->has_valid_index(hnd_) && mesh_->is_removed(hnd_))
                    ++hnd_;
        }

        void decrement()
        {
            --hnd_;
            assert(mesh_ != nullptr);
            if (mesh_->has_garbage())
                while (mesh_->has_valid_index(hnd_) && mesh_->is_removed(hnd_))
                    --hnd_;
        }

        void advance(std::ptrdiff_t n)
        {
            assert(mesh_ != nullptr);

            if (mesh_->has_garbage())
            {
                if (n > 0)
                    for (std::ptrdiff_t i = 0; i < n; ++i)
                        increment();
                else
                    for (std::ptrdiff_t i = 0; i < -n; ++i)
                        decrement();
            }
            else
            {
                // std::cout << __FILE__ << " " << __LINE__ << ": hnd_=" << hnd_ << " n=" << n << std::endl;
                hnd_ += n;
            }
        }

        std::ptrdiff_t distance_to(const Index_iterator &other) const
        {
            if (mesh_->has_garbage())
            {
                bool forward = (other.hnd_ > hnd_);

                std::ptrdiff_t out = 0;
                Index_iterator it = *this;
                while (!it.equal(other))
                {
                    if (forward)
                    {
                        ++it;
                        ++out;
                    }
                    else
                    {
                        --it;
                        --out;
                    }
                }
                return out;
            }

            // else
            return std::ptrdiff_t(other.hnd_) - std::ptrdiff_t(this->hnd_);
        }

        bool equal(const Index_iterator &other) const
        {
            return this->hnd_ == other.hnd_;
        }

        // *iterator to access the real value pointed by this iterator
        Index_ &dereference() const
        {
            // std::cout << __FILE__ << " " << __LINE__ << " dereference=" << const_cast<Index_ &>(hnd_) << std::endl;
            return const_cast<Index_ &>(hnd_);
        }

        Index_ hnd_;
        const Surface_mesh *mesh_;
    };

    typedef Index_iterator<Vertex_index> Vertex_iterator;
    typedef Iterator_range<Vertex_iterator> Vertex_range;
    typedef Index_iterator<Halfedge_index> Halfedge_iterator;
    typedef Iterator_range<Halfedge_iterator> Halfedge_range;
    typedef Index_iterator<Edge_index> Edge_iterator;
    typedef Iterator_range<Edge_iterator> Edge_range;
    typedef Index_iterator<Face_index> Face_iterator;
    typedef Iterator_range<Face_iterator> Face_range;
    // ----------------------------------- Circulator ---------------------------------------------
    typedef EGL::Vertex_around_target_iterator<Self> Vertex_around_target_iterator;
    typedef Iterator_range<Vertex_around_target_iterator> Vertex_around_target_range;

    typedef EGL::Halfedge_around_target_iterator<Self> Halfedge_around_target_iterator;
    typedef Iterator_range<Halfedge_around_target_iterator> Halfedge_around_target_range;

    typedef EGL::Face_around_target_iterator<Self> Face_around_target_iterator;
    typedef Iterator_range<Face_around_target_iterator> Face_around_target_range;

    typedef EGL::Vertex_around_face_iterator<Self> Vertex_around_face_iterator;
    typedef Iterator_range<Vertex_around_face_iterator> Vertex_around_face_range;

    typedef EGL::Halfedge_around_face_iterator<Self> Halfedge_around_face_iterator;
    typedef Iterator_range<Halfedge_around_face_iterator> Halfedge_around_face_range;

    typedef EGL::Face_around_face_iterator<Self> Face_around_face_iterator;
    typedef Iterator_range<Face_around_face_iterator> Face_around_face_range;

    /// \brief This class circulates clockwise through all
    /// one-ring neighbors of a vertex.
    ///  A model of `BidirectionalCirculator` with value type `Vertex_index`.
    /// \sa `Halfedge_around_target_circulator`, `Face_around_target_circulator`
    typedef EGL::Vertex_around_target_circulator<Surface_mesh> Vertex_around_target_circulator;

    /// \brief This class circulates clockwise through all incident faces of a vertex.
    ///  A model of `BidirectionalCirculator` with value type `Face_index`.
    /// \sa `Vertex_around_target_circulator`, `Halfedge_around_target_circulator`
    typedef EGL::Face_around_target_circulator<Surface_mesh> Face_around_target_circulator;

    /// \brief This class circulates clockwise through all halfedges around a vertex that have this vertex as target.
    ///  A model of `BidirectionalCirculator` with value type `Halfedge_index`.
    /// \sa `Vertex_around_target_circulator`, `Halfedge_around_target_circulator`
    typedef EGL::Halfedge_around_target_circulator<Surface_mesh> Halfedge_around_target_circulator;

    /// \brief This class circulates clockwise through all halfedges around a vertex that have this vertex as source.
    ///  A model of `BidirectionalCirculator` with value type `Halfedge_index`.
    /// \sa `Vertex_around_target_circulator`, `Halfedge_around_target_circulator`
    typedef EGL::Halfedge_around_source_circulator<Surface_mesh> Halfedge_around_source_circulator;

    /// \brief This class circulates counterclockwise through all vertices around a face.
    ///  A model of `BidirectionalCirculator` with value type `Vertex_index`.
    typedef EGL::Vertex_around_face_circulator<Surface_mesh> Vertex_around_face_circulator;

    /// \brief This class circulates counterclockwise through all halfedges around a face.
    ///  A model of `BidirectionalCirculator` with value type `Halfedge_index`.
    typedef EGL::Halfedge_around_face_circulator<Surface_mesh> Halfedge_around_face_circulator;

    /// \brief This class circulates counterclockwise through all faces around a face.
    ///  A model of `BidirectionalCirculator` with value type `Face_index`.
    ///  Note that the face index is the same after `operator++`, if the neighboring faces share
    ///  several halfedges.
    typedef EGL::Face_around_face_circulator<Surface_mesh> Face_around_face_circulator;

    // typedefs which make it easier to write the partial specialisation of boost::graph_traits
    typedef Vertex_index vertex_index;
    typedef P vertex_property_type;
    typedef Halfedge_index halfedge_index;
    typedef Edge_index edge_index;
    typedef Face_index face_index;

    typedef Vertex_iterator vertex_iterator;
    typedef Halfedge_iterator halfedge_iterator;
    typedef Edge_iterator edge_iterator;
    typedef Face_iterator face_iterator;
    typedef EGL::In_edge_iterator<Self> In_edge_iterator;
    typedef EGL::Out_edge_iterator<Self> Out_edge_iterator;
    typedef EGL::In_edge_iterator<Self> in_edge_iterator;
    typedef EGL::Out_edge_iterator<Self> out_edge_iterator;

    typedef boost::undirected_tag directed_category;
    typedef boost::disallow_parallel_edge_tag edge_parallel_category;

    struct traversal_category : public virtual boost::bidirectional_graph_tag,
                                public virtual boost::vertex_list_graph_tag,
                                public virtual boost::edge_list_graph_tag
    {
    };

    typedef size_type vertices_size_type;
    typedef size_type halfedges_size_type;
    typedef size_type edges_size_type;
    typedef size_type faces_size_type;
    typedef size_type degree_size_type;

    template <class I, class T>
    struct Property_map : Properties::Property_map_base<I, T, Property_map<I, T>>
    {
        typedef Properties::Property_map_base<I, T, Property_map<I, T>> Base;
        typedef typename Base::reference reference;
        Property_map() : Base() {}
        Property_map(const Base &pm) : Base(pm) {}
    };

    template <typename Key, typename T>
    struct Get_property_map
    {
        typedef Property_map<Key, T> type;
    };

    template <typename, bool = true>
    struct Property_selector
    {
    };

    template <bool dummy>
    struct Property_selector<typename Surface_mesh<P>::Vertex_index, dummy>
    {
        Surface_mesh<P> *m_;
        Property_selector(Surface_mesh<P> *m) : m_(m) {}
        Properties::Property_container<Self,
                                       typename Surface_mesh<P>::Vertex_index> &
        operator()() { return m_->vprops_; }
    };
    template <bool dummy>
    struct Property_selector<typename Surface_mesh<P>::Halfedge_index, dummy>
    {
        Surface_mesh<P> *m_;
        Property_selector(Surface_mesh<P> *m) : m_(m) {}
        Properties::Property_container<Self,
                                       typename Surface_mesh<P>::Halfedge_index> &
        operator()() { return m_->hprops_; }
    };
    template <bool dummy>
    struct Property_selector<typename Surface_mesh<P>::Edge_index, dummy>
    {
        Surface_mesh<P> *m_;
        Property_selector(Surface_mesh<P> *m) : m_(m) {}
        Properties::Property_container<Self,
                                       typename Surface_mesh<P>::Edge_index> &
        operator()() { return m_->eprops_; }
    };
    template <bool dummy>
    struct Property_selector<typename Surface_mesh<P>::Face_index, dummy>
    {
        Surface_mesh<P> *m_;
        Property_selector(Surface_mesh<P> *m) : m_(m) {}
        Properties::Property_container<Self,
                                       typename Surface_mesh<P>::Face_index> &
        operator()() { return m_->fprops_; }
    };

    // ----------------------- Data Allocation -------------------------------------
    Vertex_index add_vertex()
    {
        size_type inf = (std::numeric_limits<size_type>::max)();
        if (vertices_freelist_ != inf)
        {
            // put the new vertex into the most-recent removed vprops_ position
            size_type idx = vertices_freelist_; // get availbe vprops_ position
            vertices_freelist_ = vconn_[Vertex_index(vertices_freelist_)].halfedge_;
            // std::cout << __FILE__ << " " << __LINE__ << " available vidx=" << idx << ", next available vidx=" << vertices_freelist_ << std::endl;
            --removed_vertices_;
            vremoved_[Vertex_index(idx)] = false;
            vprops_.reset(Vertex_index(idx));
            return Vertex_index(idx);
        }
        else
        {
            vprops_.push_back();
            return Vertex_index(vprops_.size() - 1);
        }
    }

    Vertex_index add_vertex(const Point &p)
    {
        Vertex_index v = add_vertex();
        vpoint_[v] = p;
        return v;
    }

    /// adds a new edge, and resizes edge and halfedge properties if necessary.
    Halfedge_index add_edge()
    {
        Halfedge_index h0, h1;
        size_type inf = (std::numeric_limits<size_type>::max)();
        if (edges_freelist_ != inf)
        {
            size_type idx = edges_freelist_;
            edges_freelist_ = (size_type)hconn_[Halfedge_index(edges_freelist_)].next_halfedge_;
            --removed_edges_;
            eremoved_[Edge_index(Halfedge_index(idx))] = false;
            hprops_.reset(Halfedge_index(idx));
            hprops_.reset(opposite(Halfedge_index(idx)));
            eprops_.reset(Edge_index(Halfedge_index(idx)));
            return Halfedge_index(idx);
        }
        else
        {
            eprops_.push_back();
            hprops_.push_back();
            hprops_.push_back();

            return Halfedge_index(num_halfedges() - 2);
        }
    }

    /// adds two opposite halfedges, and resizes edge and halfedge properties if necessary.
    /// Sets the targets of the halfedge to the given vertices, but does not modify the halfedge
    /// associated to the vertices.
    /// \note The function does not check whether there is already an edge between the vertices.
    /// \returns the halfedge with `v1` as target
    Halfedge_index add_edge(Vertex_index v0, Vertex_index v1)
    {
        assert(v0 != v1);
        Halfedge_index h = add_edge();

        set_target(h, v1);
        set_target(opposite(h), v0);

        return h;
    }

    /// adds a new face, and resizes face properties if necessary.
    Face_index add_face()
    {
        size_type inf = (std::numeric_limits<size_type>::max)();
        if (faces_freelist_ != inf)
        {
            size_type idx = faces_freelist_;
            faces_freelist_ = (size_type)fconn_[Face_index(faces_freelist_)].halfedge_;
            --removed_faces_;
            fprops_.reset(Face_index(idx));
            fremoved_[Face_index(idx)] = false;
            return Face_index(idx);
        }
        else
        {
            fprops_.push_back();
            return Face_index(num_faces() - 1);
        }
    }

    Face_index add_face(Vertex_index v0, Vertex_index v1, Vertex_index v2)
    {
        std::array<Vertex_index, 3> V;
        V[0] = v0;
        V[1] = v1;
        V[2] = v2;
        return add_face(V);
    }

    size_type add_face(size_type v0, size_type v1, size_type v2)
    {
       return add_face(Vertex_index(v0), Vertex_index(v1), Vertex_index(v2)).idx();
    }

    template <typename Range>
    Face_index add_face(const Range &vr)
    {
        return Euler::add_face(vr, *this);
    }

    // -- Low-Level Connectivity ---------------------------------------------
    /// returns the vertex the halfedge `h` points to.
    Vertex_index target(Halfedge_index h) const
    {
        return hconn_[h].vertex_;
    }

    /// sets the vertex the halfedge `h` points to to `v`.
    void set_target(Halfedge_index h, Vertex_index v)
    {
        hconn_[h].vertex_ = v;
    }

    /// returns the face incident to halfedge `h`.
    Face_index face(Halfedge_index h) const
    {
        return hconn_[h].face_;
    }

    /// sets the incident face to halfedge `h` to `f`.
    void set_face(Halfedge_index h, Face_index f)
    {
        hconn_[h].face_ = f;
    }

    /// returns the next halfedge within the incident face.
    Halfedge_index next(Halfedge_index h) const
    {
        return hconn_[h].next_halfedge_;
    }

    /// returns the previous halfedge within the incident face.
    Halfedge_index prev(Halfedge_index h) const
    {
        return hconn_[h].prev_halfedge_;
    }

    // sets the next halfedge of `h` within the face to `nh`.
    void set_next_only(Halfedge_index h, Halfedge_index nh)
    {
        hconn_[h].next_halfedge_ = nh;
    }

    void set_prev_only(Halfedge_index h, Halfedge_index nh)
    {
        if (h != null_halfedge())
        {
            hconn_[h].prev_halfedge_ = nh;
        }
    }

    /// sets the next halfedge of `h` within the face to `nh` and
    /// the previous halfedge of `nh` to `h`.
    void set_next(Halfedge_index h, Halfedge_index nh)
    {
        set_next_only(h, nh);
        set_prev_only(nh, h);
    }

    /// returns an incoming halfedge of vertex `v`.
    /// If `v` is a border vertex this will be a border halfedge.
    /// \invariant `target(halfedge(v)) == v`
    Halfedge_index halfedge(Vertex_index v) const
    {
        return vconn_[v].halfedge_;
    }

    /// sets the incoming halfedge of vertex `v` to `h`.
    void set_halfedge(Vertex_index v, Halfedge_index h)
    {
        vconn_[v].halfedge_ = h;
    }

    /// returns a halfedge of face `f`.
    Halfedge_index halfedge(Face_index f) const
    {
        return fconn_[f].halfedge_;
    }

    /// sets the halfedge of face `f` to `h`.
    void set_halfedge(Face_index f, Halfedge_index h)
    {
        fconn_[f].halfedge_ = h;
    }

    /// returns the opposite halfedge of `h`. Note that there is no function `set_opposite()`.
    Halfedge_index opposite(Halfedge_index h) const
    {
        return Halfedge_index(((size_type)h & 1) ? (size_type)h - 1 : (size_type)h + 1);
    }

    /// returns the vertex the halfedge `h` emanates from.
    Vertex_index source(Halfedge_index h) const
    {
        return target(opposite(h));
    }

    /// returns `opposite(next(h))`, that is the next halfedge
    /// "clockwise" around the target vertex of `h`.
    Halfedge_index next_around_target(Halfedge_index h) const
    {
        return opposite(next(h));
    }

    /// returns `prev(opposite(h))`, that is the previous halfedge
    /// "clockwise" around the target vertex of `h`.
    Halfedge_index prev_around_target(Halfedge_index h) const
    {
        return prev(opposite(h));
    }

    /// returns `next(opposite(h))`, that is the next halfedge \ref SurfaceMeshOrientation
    /// "clockwise" around the source vertex of `h`.
    Halfedge_index next_around_source(Halfedge_index h) const
    {
        return next(opposite(h));
    }

    /// returns `opposite(prev(h))`, that is the previous halfedge \ref SurfaceMeshOrientation
    /// "clockwise" around the source vertex of `h`.
    Halfedge_index prev_around_source(Halfedge_index h) const
    {
        return opposite(prev(h));
    }

    /// returns the i'th vertex of edge `e`, for `i=0` or `1`.
    Vertex_index vertex(Edge_index e, unsigned int i) const
    {
        assert(i <= 1);
        return target(halfedge(e, i));
    }

    /// returns the edge that contains halfedge `h` as one of its two halfedges.
    Edge_index edge(Halfedge_index h) const
    {
        return Edge_index(h);
    }

    /// returns the halfedge corresponding to the edge `e`.
    Halfedge_index halfedge(Edge_index e) const
    {
        return Halfedge_index(e.halfedge());
    }

    /// returns the i'th halfedge of edge `e`, for `i=0` or `1`.
    Halfedge_index halfedge(Edge_index e, unsigned int i) const
    {
        assert(i <= 1);
        return Halfedge_index(((size_type)e << 1) + i);
    }

    /// finds a halfedge between two vertices. Returns a default constructed
    /// `Halfedge_index`, if  `source` and  `target` are not connected.
    Halfedge_index halfedge(Vertex_index source, Vertex_index target) const
    {
        assert(has_valid_index(source) && has_valid_index(target));
        Halfedge_index h = halfedge(target);
        const Halfedge_index hh = h;
        if (h.is_valid())
        {
            do
            {
                if (this->source(h) == source)
                    return h;
                h = next_around_target(h);
            } while (h != hh);
        }
        return Halfedge_index();
    }

    // -----------------------Low-Level Removal Functions---------------------------
    ///
    /// Although the elements are only marked as removed
    /// their connectivity and properties should not be used.
    ///
    /// \warning Functions in this group do not adjust any of
    /// connected elements and usually leave the surface mesh in an
    /// invalid state.
    void remove_vertex(Vertex_index v)
    {
        // when doing sequential removals, vconn_[v].halfedge_ actually records previous vertex index that was removed
        vremoved_[v] = true;
        ++removed_vertices_;
        garbage_ = true;
        vconn_[v].halfedge_ = Halfedge_index(vertices_freelist_);
        // std::cout << __FILE__ << " " << __LINE__ << " vidx=" << v << " is now available, vertices_freelist_" << vertices_freelist_
        //           << " vconn_[v].halfedge_=" << vconn_[v].halfedge_ << std::endl;
        vertices_freelist_ = (size_type)v; //vprop_[v] now is available
        // std::cout << __FILE__ << " " << __LINE__ << " vertices_freelist_=" << vertices_freelist_ << std::endl;
    }

    // -----------------------------------------------------------------------------
    /// returns the number of used and removed vertices in the mesh.
    size_type num_vertices() const { return (size_type)vprops_.size(); }

    /// returns the number of used and removed halfedges in the mesh.
    size_type num_halfedges() const { return (size_type)hprops_.size(); }

    /// returns the number of used and removed edges in the mesh.
    size_type num_edges() const { return (size_type)eprops_.size(); }

    /// returns the number of used and removed faces in the mesh.
    size_type num_faces() const { return (size_type)fprops_.size(); }

    size_type degree(Vertex_index v) const
    {
        Halfedge_index h = halfedge(v);
        if (h == null_halfedge())
        {
            return 0;
        }
        size_type count(0);
        Halfedge_index done = h;
        do
        {
            ++count;
            h = opposite(next(h));
        } while (h != done);
        return count;
    }

    size_type degree(Face_index f) const
    {
        size_type count(0);
        if (halfedge(f) == null_halfedge())
        {
            return 0;
        }
        // ToDO:
        Vertex_around_face_circulator fvit(halfedge(f), *this);
        Vertex_around_face_circulator fvend = fvit;
        if (fvit)
            do
            {
                ++count;
            } while (++fvit != fvend);

        return count;
    }

    /// returns the property for the string "v:point".
    Property_map<Vertex_index, Point>
    points() const { return vpoint_; }

    Property_map<Vertex_index, Point> &
    points() { return vpoint_; }

    /// returns the point associated to vertex `v`.
    const Point &
    point(Vertex_index v) const { return vpoint_[v]; }

    /// returns the point associated to vertex `v`.
    Point &
    point(Vertex_index v) { return vpoint_[v]; }

    // ----------------------- Iterator  ---------------------------------------------------------------
    Vertex_iterator vertices_begin() const
    {
        return Vertex_iterator(Vertex_index(0), this);
    }

    Vertex_iterator vertices_end() const
    {
        return Vertex_iterator(Vertex_index(num_vertices()), this);
    }

    Vertex_range vertices() const
    {
        return make_range(vertices_begin(), vertices_end());
    }

    Halfedge_iterator halfedges_begin() const
    {
        return Halfedge_iterator(Halfedge_index(0), this);
    }

    Halfedge_iterator halfedges_end() const
    {
        return Halfedge_iterator(Halfedge_index(num_halfedges()), this);
    }

    Halfedge_range halfedges() const
    {
        return make_range(halfedges_begin(), halfedges_end());
    }

    Edge_iterator edges_begin() const
    {
        return Edge_iterator(Edge_index(0), this);
    }

    Edge_iterator edges_end() const
    {
        return Edge_iterator(Edge_index(num_edges()), this);
    }

    Edge_range edges() const
    {
        return make_range(edges_begin(), edges_end());
    }

    Face_iterator faces_begin() const
    {
        return Face_iterator(Face_index(0), this);
    }

    Face_iterator faces_end() const
    {
        return Face_iterator(Face_index(num_faces()), this);
    }

    Face_range faces() const
    {
        return make_range(faces_begin(), faces_end());
    }

    // ----------------------- Circulator  --------------------------------------------------------------
    /// returns the iterator range for vertices around vertex `target(h)`, starting at `source(h)`.
    Vertex_around_target_range vertices_around_target(Halfedge_index h) const
    {
        return EGL::vertices_around_target(h, *this);
    }

    /// returns the iterator range for incoming halfedges around vertex `target(h)`, starting at `h`.
    Halfedge_around_target_range halfedges_around_target(Halfedge_index h) const
    {
        return EGL::halfedges_around_target(h, *this);
    }

    /// returns the iterator range for faces around vertex `target(h)`, starting at `face(h)`.
    Face_around_target_range faces_around_target(Halfedge_index h) const
    {
        return EGL::faces_around_target(h, *this);
    }

    /// returns the iterator range for vertices around face `face(h)`, starting at `target(h)`.
    Vertex_around_face_range vertices_around_face(Halfedge_index h) const
    {
        return EGL::vertices_around_face(h, *this);
    }

    /// returns the iterator range for halfedges around face `face(h)`, starting at `h`.
    Halfedge_around_face_range halfedges_around_face(Halfedge_index h) const
    {
        return EGL::halfedges_around_face(h, *this);
    }

    /// returns the iterator range for halfedges around face `face(h)`, starting at `h`.
    Face_around_face_range faces_around_face(Halfedge_index h) const
    {
        return EGL::faces_around_face(h, *this);
    }

    // ----------------------- Deletion and garbage collection  -----------------------------------------
    /// returns whether vertex `v` is marked removed.
    /// \sa `collect_garbage()`
    bool is_removed(Vertex_index v) const
    {
        return vremoved_[v];
    }
    /// returns whether halfedge `h` is marked removed.
    /// \sa `collect_garbage()`
    bool is_removed(Halfedge_index h) const
    {
        return eremoved_[edge(h)];
    }
    /// returns whether edge `e` is marked removed.
    /// \sa `collect_garbage()`
    bool is_removed(Edge_index e) const
    {
        return eremoved_[e];
    }
    /// returns whether face `f` is marked removed.
    /// \sa `collect_garbage()`
    bool is_removed(Face_index f) const
    {
        return fremoved_[f];
    }

    bool has_valid_index(Vertex_index v) const
    {
        return ((size_type)v < num_vertices());
    }

    /// returns whether the index of halfedge `h` is valid, that is within the current array bounds.
    bool has_valid_index(Halfedge_index h) const
    {
        return ((size_type)h < num_halfedges());
    }
    /// returns whether the index of edge `e` is valid, that is within the current array bounds.
    bool has_valid_index(Edge_index e) const
    {
        return ((size_type)e < num_edges());
    }
    /// returns whether the index of face `f` is valid, that is within the current array bounds.
    bool has_valid_index(Face_index f) const
    {
        return ((size_type)f < num_faces());
    }

    bool has_garbage() const { return garbage_; }

    void collect_garbage()
    {
        if (!has_garbage())
        {
            return;
        }
        int i, i0, i1, nV(num_vertices()), nE(num_edges()), nH(num_halfedges()), nF(num_faces());

        Vertex_index v;
        Halfedge_index h;
        Face_index f;

        // setup index mapping%
        Property_map<Vertex_index, Vertex_index> vmap = add_property_map<Vertex_index, Vertex_index>("v:garbage-collection").first;
        Property_map<Halfedge_index, Halfedge_index> hmap = add_property_map<Halfedge_index, Halfedge_index>("h:garbage-collection").first;
        Property_map<Face_index, Face_index> fmap = add_property_map<Face_index, Face_index>("f:garbage-collection").first;

        for (i = 0; i < nV; ++i)
            vmap[Vertex_index(i)] = Vertex_index(i);
        for (i = 0; i < nH; ++i)
            hmap[Halfedge_index(i)] = Halfedge_index(i);
        for (i = 0; i < nF; ++i)
            fmap[Face_index(i)] = Face_index(i);

        // really remove vertices
        if (nV > 0)
        {
            i0 = 0;
            i1 = nV - 1;

            while (1)
            {
                // find first removed and last un-removed
                while (!vremoved_[Vertex_index(i0)] && i0 < i1)
                    ++i0;
                while (vremoved_[Vertex_index(i1)] && i0 < i1)
                    --i1;
                if (i0 >= i1)
                    break;

                // swap
                vprops_.swap(i0, i1);
            };

            // remember new size
            nV = vremoved_[Vertex_index(i0)] ? i0 : i0 + 1;
        }

        // really remove edges
        if (nE > 0)
        {
            i0 = 0;
            i1 = nE - 1;

            while (1)
            {
                // find first removed and last un-removed
                while (!eremoved_[Edge_index(i0)] && i0 < i1)
                    ++i0;
                while (eremoved_[Edge_index(i1)] && i0 < i1)
                    --i1;
                if (i0 >= i1)
                    break;

                // swap
                eprops_.swap(i0, i1);
                hprops_.swap(2 * i0, 2 * i1);
                hprops_.swap(2 * i0 + 1, 2 * i1 + 1);
            };

            // remember new size
            nE = eremoved_[Edge_index(i0)] ? i0 : i0 + 1;
            nH = 2 * nE;
        }

        // really remove faces
        if (nF > 0)
        {
            i0 = 0;
            i1 = nF - 1;

            while (1)
            {
                // find 1st removed and last un-removed
                while (!fremoved_[Face_index(i0)] && i0 < i1)
                    ++i0;
                while (fremoved_[Face_index(i1)] && i0 < i1)
                    --i1;
                if (i0 >= i1)
                    break;

                // swap
                fprops_.swap(i0, i1);
            };

            // remember new size
            nF = fremoved_[Face_index(i0)] ? i0 : i0 + 1;
        }

        // update vertex connectivity
        for (i = 0; i < nV; ++i)
        {
            v = Vertex_index(i);
            if (!is_isolated(v))
                set_halfedge(v, hmap[halfedge(v)]);
        }

        // update halfedge connectivity
        for (i = 0; i < nH; ++i)
        {
            h = Halfedge_index(i);
            set_target(h, vmap[target(h)]);
            set_next(h, hmap[next(h)]);
            if (!is_border(h))
                set_face(h, fmap[face(h)]);
        }

        // update indices of faces
        for (i = 0; i < nF; ++i)
        {
            f = Face_index(i);
            set_halfedge(f, hmap[halfedge(f)]);
        }

        // remove index maps
        remove_property_map<Vertex_index>(vmap);
        remove_property_map<Halfedge_index>(hmap);
        remove_property_map<Face_index>(fmap);

        // finally resize arrays
        vprops_.resize(nV);
        vprops_.shrink_to_fit();
        hprops_.resize(nH);
        hprops_.shrink_to_fit();
        eprops_.resize(nE);
        eprops_.shrink_to_fit();
        fprops_.resize(nF);
        fprops_.shrink_to_fit();

        removed_vertices_ = removed_edges_ = removed_faces_ = 0;
        vertices_freelist_ = edges_freelist_ = faces_freelist_ = -1;
        garbage_ = false;
    }

    /// returns whether `v` is isolated, i.e., incident to `Surface_mesh::null_halfedge()`.
    bool is_isolated(Vertex_index v) const
    {
        return !halfedge(v).is_valid();
    }

    // ----------------------- borders  ---------------------------------------------------------------
    /// returns whether `v` is a border vertex.
    /// \EGLAdvancedBegin
    /// With the default value for
    /// `check_all_incident_halfedges` the function iteratates over the incident halfedges.
    /// With `check_all_incident_halfedges == false` the function returns `true`, if the incident
    /// halfedge associated to vertex `v` is a border halfedge, or if the vertex is isolated.
    /// \EGLAdvancedEnd
    /// \attention If the data contained in the `Surface_mesh` is not a 2-manifold, then
    /// this operation is not guaranteed to return the right result.
    bool is_border(Vertex_index v, bool check_all_incident_halfedges = true) const
    {
        Halfedge_index h(halfedge(v));
        if (h == null_halfedge())
        {
            return true;
        }
        if (check_all_incident_halfedges)
        {
            Halfedge_around_target_circulator hatc(h, *this), done(hatc);
            do
            {
                if (is_border(*hatc))
                {
                    return true;
                }
            } while (++hatc != done);
            return false;
        }
        return is_border(h);
    }

    bool is_border(Halfedge_index h) const
    {
        return !face(h).is_valid();
    }

    bool is_border(Edge_index e) const
    {
        return is_border(e.halfedge()) || is_border(opposite(e.halfedge()));
    }

    // eg. vconn_, vpoint_, vremoved_ are added to the vprops_ container
    template <class I, class T>
    std::pair<Property_map<I, T>, bool> add_property_map(std::string name, const T t = T())
    {
        return Property_selector<I>(this)().template add<T>(name, t);
    }

    /// removes property map `p`. The memory allocated for that property map is
    /// freed.
    template <class I, class T>
    void remove_property_map(Property_map<I, T> &p)
    {
        (Property_selector<I>(this)()).template remove<T>(p);
    }

    /// returns a property map named `name` with key type `I` and value type `T`,
    /// and a Boolean that is `true` if the property exists.
    /// In case it does not exist the Boolean is `false` and the behavior of
    /// the property map is undefined.
    template <class I, class T>
    std::pair<Property_map<I, T>, bool> property_map(const std::string &name) const
    {
        return Property_selector<I>(const_cast<Surface_mesh *>(this))().template get<T>(name);
    }

    /// returns `Vertex_index(std::numeric_limits<size_type>::%max())`.
    static Vertex_index null_vertex()
    {
        return vertex_index((std::numeric_limits<size_type>::max)());
    }

    /// returns `Edge_index(std::numeric_limits<size_type>::%max())`.
    static Edge_index null_edge()
    {
        return edge_index((std::numeric_limits<size_type>::max)());
    }
    /// returns `Halfedge_index(std::numeric_limits<size_type>::%max())`.
    static Halfedge_index null_halfedge()
    {
        return halfedge_index((std::numeric_limits<size_type>::max)());
    }
    /// returns `Face_index(std::numeric_limits<size_type>::%max())`.
    static Face_index null_face()
    {
        return face_index((std::numeric_limits<size_type>::max)());
    }

    /// reserves space for vertices, halfedges, edges, faces, and their currently
    /// associated properties.
    void reserve(size_type nvertices,
                 size_type nedges,
                 size_type nfaces)
    {
        vprops_.reserve(nvertices);
        hprops_.reserve(2 * nedges);
        eprops_.reserve(nedges);
        fprops_.reserve(nfaces);
    }

    void resize(size_type nvertices,
                size_type nedges,
                size_type nfaces)
    {
        vprops_.resize(nvertices);
        hprops_.resize(2 * nedges);
        eprops_.resize(nedges);
        fprops_.resize(nfaces);
    }

    Surface_mesh()
    {
        vconn_ = add_property_map<Vertex_index, Vertex_connectivity>("v:connectivity").first;
        hconn_ = add_property_map<Halfedge_index, Halfedge_connectivity>("h:connectivity").first;
        fconn_ = add_property_map<Face_index, Face_connectivity>("f:connectivity").first;
        vpoint_ = add_property_map<Vertex_index, Point>("v:point").first;
        vremoved_ = add_property_map<Vertex_index, bool>("v:removed", false).first;
        eremoved_ = add_property_map<Edge_index, bool>("e:removed", false).first;
        fremoved_ = add_property_map<Face_index, bool>("f:removed", false).first;

        removed_vertices_ = removed_edges_ = removed_faces_ = 0;
        vertices_freelist_ = edges_freelist_ = faces_freelist_ = (std::numeric_limits<size_type>::max)();
        garbage_ = false;
    }

    Surface_mesh(const Surface_mesh &rhs) { *this = rhs; }

    Surface_mesh &operator=(const Surface_mesh &rhs)
    {
        if (this != &rhs)
        {
            // deep copy of property containers
            vprops_ = rhs.vprops_;
            hprops_ = rhs.hprops_;
            eprops_ = rhs.eprops_;
            fprops_ = rhs.fprops_;

            // property handles contain pointers, have to be reassigned
            vconn_ = property_map<Vertex_index, Vertex_connectivity>("v:connectivity").first;
            hconn_ = property_map<Halfedge_index, Halfedge_connectivity>("h:connectivity").first;
            fconn_ = property_map<Face_index, Face_connectivity>("f:connectivity").first;
            vremoved_ = property_map<Vertex_index, bool>("v:removed").first;
            eremoved_ = property_map<Edge_index, bool>("e:removed").first;
            fremoved_ = property_map<Face_index, bool>("f:removed").first;
            vpoint_ = property_map<Vertex_index, P>("v:point").first;

            // how many elements are removed?
            removed_vertices_ = rhs.removed_vertices_;
            removed_edges_ = rhs.removed_edges_;
            removed_faces_ = rhs.removed_faces_;
            vertices_freelist_ = rhs.vertices_freelist_;
            edges_freelist_ = rhs.edges_freelist_;
            faces_freelist_ = rhs.faces_freelist_;
            garbage_ = rhs.garbage_;
        }
		return *this;
    }

    void clear()
    {
        vprops_.clear();
        hprops_.clear();
        eprops_.clear();
        fprops_.clear();

        vprops_.resize(0);
        hprops_.resize(0);
        eprops_.resize(0);
        fprops_.resize(0);

        vprops_.shrink_to_fit();
        hprops_.shrink_to_fit();
        eprops_.shrink_to_fit();
        fprops_.shrink_to_fit();

        vconn_ = add_property_map<Vertex_index, Vertex_connectivity>("v:connectivity").first;
        hconn_ = add_property_map<Halfedge_index, Halfedge_connectivity>("h:connectivity").first;
        fconn_ = add_property_map<Face_index, Face_connectivity>("f:connectivity").first;
        vpoint_ = add_property_map<Vertex_index, Point>("v:point").first;
        vremoved_ = add_property_map<Vertex_index, bool>("v:removed", false).first;
        eremoved_ = add_property_map<Edge_index, bool>("e:removed", false).first;
        fremoved_ = add_property_map<Face_index, bool>("f:removed", false).first;

        removed_vertices_ = removed_edges_ = removed_faces_ = 0;
        vertices_freelist_ = edges_freelist_ = faces_freelist_ = (std::numeric_limits<size_type>::max)();
        garbage_ = false;
    }

    /// Functions to check the number of elements, the amount of space
    /// allocated for elements, and to clear the structure.
    ///@{

    /// returns the number of vertices in the mesh.
    size_type number_of_vertices() const
    {
        return num_vertices() - number_of_removed_vertices();
    }

    /// returns the number of halfedges in the mesh.
    size_type number_of_halfedges() const
    {
        return num_halfedges() - number_of_removed_halfedges();
    }

    /// returns the number of edges in the mesh.
    size_type number_of_edges() const
    {
        return num_edges() - number_of_removed_edges();
    }

    /// returns the number of faces in the mesh.
    size_type number_of_faces() const
    {
        return num_faces() - number_of_removed_faces();
    }

    /// returns `true` iff the mesh is empty, i.e., has no vertices, halfedges and faces.
    bool is_empty() const
    {
        return (num_vertices() == number_of_removed_vertices() && num_halfedges() == number_of_removed_halfedges() && num_faces() == number_of_removed_faces());
    }

    /// returns the number of vertices in the mesh which are marked removed.
    size_type number_of_removed_vertices() const { return removed_vertices_; }

    /// returns the number of halfedges in the mesh which are marked removed.
    size_type number_of_removed_halfedges() const { return 2 * removed_edges_; }

    /// returns the number of edges in the mesh which are marked removed.
    size_type number_of_removed_edges() const { return removed_edges_; }

    /// returns the number offaces in the mesh which are marked removed.
    size_type number_of_removed_faces() const { return removed_faces_; }

private:
    //------------------- Property_container responsible for data allocation-------------
    Properties::Property_container<Self, Vertex_index> vprops_;
    Properties::Property_container<Self, Halfedge_index> hprops_;
    Properties::Property_container<Self, Edge_index> eprops_;
    Properties::Property_container<Self, Face_index> fprops_;

    //------------------- Property_map responsible for data access-----------------------
    Property_map<Vertex_index, Vertex_connectivity> vconn_;
    Property_map<Halfedge_index, Halfedge_connectivity> hconn_;
    Property_map<Face_index, Face_connectivity> fconn_;

    Property_map<Vertex_index, Point> vpoint_;

    Property_map<Vertex_index, bool> vremoved_;
    Property_map<Edge_index, bool> eremoved_;
    Property_map<Face_index, bool> fremoved_;

    size_type removed_vertices_;
    size_type removed_edges_;
    size_type removed_faces_;

    size_type vertices_freelist_; //record the last vertex index that is removed
    size_type edges_freelist_;
    size_type faces_freelist_;
    bool garbage_;
};
