#include "partition.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <list>

namespace igl
{
    namespace copyleft
    {
        namespace cgal
        {
            IGL_INLINE void partition(
                const std::vector<Eigen::Vector3d> &polygon_pts,
                std::vector<std::vector<Eigen::Vector3d>> &partition_polypts)
            {
                // optimal partition
                typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
                typedef CGAL::Partition_traits_2<K> Partition_traits_2;
                typedef Partition_traits_2::Point_2 Point_2;
                typedef Partition_traits_2::Polygon_2 Polygon_2; // a polygon of indices

                Polygon_2 polygon;
                std::vector<Polygon_2> partition_polys;

                for (const Eigen::Vector3d &p : polygon_pts)
                {
                    polygon.push_back(Point_2(p.x(), p.y()));
                }

                CGAL::optimal_convex_partition_2(polygon.vertices_begin(),
                                                 polygon.vertices_end(),
                                                 std::back_inserter(partition_polys));
                assert(CGAL::partition_is_valid_2(polygon.vertices_begin(),
                                                  polygon.vertices_end(),
                                                  partition_polys.begin(),
                                                  partition_polys.end()) &&
                       "Polygon is invalid!");

                partition_polypts.resize(partition_polys.size());
                for (int i = 0; i < partition_polys.size(); i++)
                {
                    const Polygon_2 &poly = partition_polys[i];
                    for (Point_2 p : poly.container())
                    {
                        partition_polypts[i].emplace_back(p.x(), p.y(), 0);
                    }
                }
            }
        }
    }
}