/**
 * @file approximate_boxes.h
 * @brief
 *
 * Copyright (c) 2024 Paul-Otto Müller <https://github.com/paulotto>
 * License: LGPL-3.0-or-later (https://github.com/paulotto/approximate_boxes/blob/main/LICENSE)
 * URL: https://github.com/paulotto/approximate_boxes/blob/main/include/approximate_boxes/approximate_boxes.h
 *
 * @author Paul-Otto Müller
 * @date 10.05.2024
 */

#ifndef APPROXIMATE_BOXES_H
#define APPROXIMATE_BOXES_H

#include <list>
#include <vector>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Octree.h>
#include <CGAL/IO/STL.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Side_of_triangle_mesh.h>

#include "approximate_boxes/hexahedron_mesh.h"


namespace approx_boxes {
    /**
     * @class ApproximateBoxes
     * @brief A class to approximate a volume provided by an OFF file with a set of bounding boxes using an octree.
     * @tparam Kernel The kernel type to use for the octree.
     */
    template<typename Kernel>
    class ApproximateBoxes {
        using Point = typename Kernel::Point_3;
        using Triangle = typename Kernel::Triangle_3;
        using Point_vector = std::vector<Point>;
        using Triangle_vector = std::list<std::array<int, 3> >;
        using Mesh3D = CGAL::Polyhedron_3<Kernel>;
        using SurfaceMesh = CGAL::Surface_mesh<Point>;
        using Octree = CGAL::Octree<Kernel, Point_vector>;
        using Node_vector = std::list<typename Octree::Node>;
        using Preorder_traversal = CGAL::Orthtrees::Preorder_traversal;

        using CartKernel = CGAL::Simple_cartesian<double>;
        using Polyhedron = CGAL::Polyhedron_3<CartKernel>;
        using HalfedgeDS = Polyhedron::HalfedgeDS;

        public:
            explicit ApproximateBoxes(const std::string& file);

            virtual ~ApproximateBoxes() {
                surface_file_stream_.close();
            }

            Polyhedron GetPolyhedronMesh() const {
                return polyhedron_combined_;
            }

            void SetOctreeMaxDepth(unsigned int depth) {
                octree_max_depth_ = depth;
            }

            void SetOctreeMaxInliers(unsigned int inliers) {
                octree_max_inliers_ = inliers;
            }

            virtual void DumpPolyhedronToGmesh(const std::string& filename);

            virtual void DumpPolyhedronToOFF(const std::string& filename);

            virtual void ApproximateGeometry();

            virtual void ExtractNodesInsideBoundary(Octree& octree, Node_vector& nodes_inside);

            virtual void RemoveParentNodes(Octree& octree, Node_vector& nodes_inside);

            static CGAL::Bounded_side IsPointInside(const Point& p, const SurfaceMesh& msh);

            static void CreateSurfaceMeshFromBboxes(const std::vector<CGAL::Bbox_3>& bboxes, SurfaceMesh& mesh);

        protected:
            virtual std::vector<Point> GetAllPoints(const SurfaceMesh& mesh) {
                std::vector<Point> points;
                for (auto v: mesh.vertices()) {
                    points.push_back(mesh.point(v));
                }
                return points;
            }

        private:
            SurfaceMesh mesh_{};
            Polyhedron polyhedron_combined_{};
            std::list<Polyhedron> polyhedron_list_{};

            HexahedronMesh<Polyhedron> hex_mesh_{};

            unsigned int octree_max_depth_{100};
            unsigned int octree_max_inliers_{1};

            std::ifstream surface_file_stream_{""};
    };
} // namespace approx_boxes

#endif //APPROXIMATE_BOXES_H
