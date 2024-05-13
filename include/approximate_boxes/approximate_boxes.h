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
        // Type definitions

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
            /**
             * @brief Constructor.
             * @param file The path to the OFF file to read the mesh from.
             */
            explicit ApproximateBoxes(const std::string& file);

            /**
             * @brief Destructor.
             */
            virtual ~ApproximateBoxes() {
                surface_file_stream_.close();
            }

            /**
             * @brief Get the all approximation boxes combined in one polyhedron.
             * @return The polyhedron combining the surfaces of all boxes.
             */
            Polyhedron GetPolyhedronMesh() const {
                return polyhedron_combined_;
            }

            /**
             * @brief Get the list of all approximation boxes.
             * @return The list of polyhedra representing the approximation boxes.
             */
            std::list<Polyhedron> GetPolyhedronList() const {
                return polyhedron_list_;
            }

            /**
             * @brief Set the number of threads to use for building the hexahedral mesh.
             * If 0, the number of threads is set to the maximum.
             * @param threads The number of threads to use.
             */
            void SetNumberOfThreads(size_t threads = 0) {
                threads_ = threads;
            }

            /**
             * @brief Set the maximum depth of the octree.
             * @param depth The maximum depth of the octree.
             */
            void SetOctreeMaxDepth(unsigned int depth) {
                octree_max_depth_ = depth;
            }

            /**
             * @brief Set the maximum number of inliers for the octree, i.e., the maximum number
             * of points inside the bounding box of a leaf node.
             * @param inliers The maximum number of inliers for the octree.
             */
            void SetOctreeMaxInliers(unsigned int inliers) {
                octree_max_inliers_ = inliers;
            }

            /**
             * @brief Set if the root node should be removed from the list of nodes inside the boundary.
             * @param remove True if the root node should be removed, false otherwise.
             */
            void SetRemoveRootNode(bool remove) {
                remove_root_node_ = remove;
            }

            /**
             * @brief Set if larger boxes should be divided into smaller boxes.
             * Necessary for a completely connected mesh.
             * @param divide True if larger boxes should be divided, false otherwise.
             */
            void SetDivideLargerBboxes(bool divide) {
                divide_larger_bboxes_ = divide;
            }

            /**
             * @brief Dump the polyhedra to a Gmsh file.
             * @param filename The path to the Gmsh file to write the polyhedra to.
             */
            virtual void DumpPolyhedraToGmesh(const std::string& filename);

            /**
             * @brief Dump the combined polyhedron to an OFF file.
             * @param filename The path to the OFF file to write the combined polyhedron to.
             */
            virtual void DumpPolyhedronToOFF(const std::string& filename);

            /**
             * @brief Approximate the geometry of the mesh with bounding boxes.
             */
            virtual void ApproximateGeometry();

            /**
             * @brief Check if a point is inside the mesh.
             * @param p The point to check.
             * @return The side of the point relative to the mesh.
             */
            static CGAL::Bounded_side IsPointInside(const Point& p, const SurfaceMesh& msh);

            /**
             * @brief Find the smallest box size from a list of bounding boxes.
             * @param boxes The list of bounding boxes to find the smallest box size from.
             * @return The smallest box size.
             */
            static double FindSmallestBoxSize(const std::vector<CGAL::Bbox_3>& boxes);

            /**
             * @brief Extract the bounding boxes from the octree.
             * @param octree The octree to extract the bounding boxes from.
             * @param nodes The list of nodes to extract the bounding boxes from.
             * @param bboxes The list of bounding boxes to extract.
             */
            static void ExtractBoundingBoxes(const Octree& octree,
                                             const Node_vector& nodes,
                                             std::vector<CGAL::Bbox_3>& bboxes);

            /**
             * @brief Divide larger boxes into smaller boxes.
             * @param boxes The list of bounding boxes to divide.
             */
            static void DivideLargerBoxes(std::vector<CGAL::Bbox_3>& boxes);

            /**
             * @brief Create a surface mesh from a list of bounding boxes.
             * @param bboxes The list of bounding boxes to create the surface mesh from.
             * @param mesh The surface mesh to create.
             */
            static void CreateSurfaceMeshFromBboxes(const std::vector<CGAL::Bbox_3>& bboxes, SurfaceMesh& mesh);

        protected:
            /**
             * @brief Extract the nodes inside the boundary of the mesh.
             * @param octree The octree to extract the nodes from.
             * @param nodes_inside The list of nodes inside the boundary.
             */
            virtual void ExtractNodesInsideBoundary(Octree& octree, Node_vector& nodes_inside);

            /**
             * @brief Remove the parent nodes from the list of nodes inside the boundary.
             * @param octree The octree to remove the parent nodes from.
             * @param nodes_inside The list of nodes inside the boundary.
             */
            virtual void RemoveParentNodes(Octree& octree, Node_vector& nodes_inside);

            /**
             * @brief Get all points of a surface mesh.
             * @param mesh The mesh to get the points from.
             * @return The list of points of the mesh.
             */
            virtual std::vector<Point> GetAllPoints(const SurfaceMesh& mesh) {
                std::vector<Point> points;
                for (auto v: mesh.vertices()) {
                    points.push_back(mesh.point(v));
                }
                return points;
            }

        private:
            // Number of threads used to build the hexahedral mesh.
            // If 0, the number of threads is set to the maximum.
            size_t threads_{0};
            bool remove_root_node_{true};

            // Necessary for a completely connected mesh. If true, the bounding boxes will all have the same size.
            bool divide_larger_bboxes_{true};

            SurfaceMesh mesh_{};
            Polyhedron polyhedron_combined_{};
            std::list<Polyhedron> polyhedron_list_{};

            HexahedronMesh<Polyhedron> hex_mesh_{};

            unsigned int octree_max_depth_{50};
            unsigned int octree_max_inliers_{1};

            std::ifstream surface_file_stream_{""};
    };
} // namespace approx_boxes

#endif //APPROXIMATE_BOXES_H
