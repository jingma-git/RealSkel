#pragma once
#include <boost/graph/graph_traits.hpp>
#include <boost/optional/optional.hpp>
#include "graph_circulator.h"

namespace EGL{
namespace internal
{
    template <typename Graph>
    bool is_isolated(typename boost::graph_traits<Graph>::vertex_descriptor v,
                     Graph &g)
    {
        return halfedge(v, g) == boost::graph_traits<Graph>::null_halfedge();
    }

    template <typename Graph>
    void adjust_incoming_halfedge(typename boost::graph_traits<Graph>::vertex_descriptor v,
                                  Graph &g)
    {
        typedef typename boost::graph_traits<Graph>::halfedge_descriptor halfedge_descriptor;
        halfedge_descriptor h = halfedge(v, g);
        halfedge_descriptor hh = h;
        // std::cout << v << ": " << h << std::endl;
        if (h != boost::graph_traits<Graph>::null_halfedge())
        {
            if (target(h, g) != v)
            {
                //wrong target, flip
                h = opposite(h, g);
                hh = h;
                set_halfedge(v, h, g);
            }

            do
            {
                if (face(h, g) == boost::graph_traits<Graph>::null_face())
                {
                    set_halfedge(v, h, g);
                    // std::cout << "after adjustment " << v << " h=" << h << std::endl;
                    return;
                }
                h = opposite(next(h, g), g);
            } while (h != hh);
        }
    }
} // namespace internal
}


/*!
   \ingroup PkgBGLHelperFct
    returns `true` if the halfedge `hd` is on a border.
  */
template <typename FaceGraph>
bool is_border(typename boost::graph_traits<FaceGraph>::halfedge_descriptor hd, const FaceGraph &g)
{
    return face(hd, g) == boost::graph_traits<FaceGraph>::null_face();
}

/*!
   \ingroup PkgBGLHelperFct
    returns `true` if the halfedge `hd` or the opposite halfedge is on a border.
  */
template <typename FaceGraph>
bool is_border_edge(typename boost::graph_traits<FaceGraph>::halfedge_descriptor hd, const FaceGraph &g)
{
    return is_border(hd, g) || is_border(opposite(hd, g), g);
}

/*!
   \ingroup PkgBGLHelperFct
    returns `true` if the edge `e` is on a border.
  */
template <typename FaceGraph>
bool is_border(typename boost::graph_traits<FaceGraph>::edge_descriptor ed, const FaceGraph &g)
{
    return is_border_edge(halfedge(ed, g), g);
}

template <typename FaceGraph>
boost::optional<typename boost::graph_traits<FaceGraph>::halfedge_descriptor>
is_border(typename boost::graph_traits<FaceGraph>::vertex_descriptor vd,
          const FaceGraph &g)
{
    EGL::Halfedge_around_target_iterator<FaceGraph> havib, havie;
    for (boost::tie(havib, havie) = EGL::halfedges_around_target(halfedge(vd, g), g); havib != havie; ++havib)
    {
        if (is_border(*havib, g))
        {
            typename boost::graph_traits<FaceGraph>::halfedge_descriptor h = *havib;
            return h;
        }
    }
    // empty
    return boost::optional<typename boost::graph_traits<FaceGraph>::halfedge_descriptor>();
}
