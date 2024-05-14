/**
 * @file approximate_boxes.cpp
 * @brief MIT License
 *
 * Copyright (c) 2024 Paul-Otto Müller <https://github.com/paulotto>
 * License: LGPL-3.0-or-later (https://github.com/paulotto/approximate_boxes/blob/main/LICENSE)
 * URL: https://github.com/paulotto/approximate_boxes/blob/main/src/approximate_boxes.cpp
 *
 * @author Paul-Otto Müller
 * @date 10.05.2024
 */

#include <mutex>
#include <thread>
#include <filesystem>

#include <CGAL/Surface_mesh/IO/OFF.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include "approximate_boxes/utils.h"
#include "approximate_boxes/approximate_boxes.h"
#include "approximate_boxes/build_polyhedron_from_bbox.h"


namespace approx_boxes {
    template<typename Kernel>
    ApproximateBoxes<Kernel>::ApproximateBoxes(const std::string& file)
        : surface_file_stream_(file, std::ifstream::in) {
        std::cout << "[ApproximateBoxes] Reading OFF file: " << file << "...\n";
        const bool success = CGAL::IO::read_OFF(surface_file_stream_, mesh_,
                                                CGAL::parameters::verbose(true));

        if (!success) {
            throw std::runtime_error("[ApproximateBoxes] Failed to read OFF file!");
        }
    }

    template<typename Kernel>
    void ApproximateBoxes<Kernel>::DumpPolyhedraToGmesh(const std::string& filename) {
        hex_mesh_.WriteGmshFile(filename);
    }

    template<typename Kernel>
    void ApproximateBoxes<Kernel>::DumpPolyhedronToOFF(const std::string& filename) {
        if (std::filesystem::exists(filename)) {
            std::cout << "[ApproximateBoxes<Kernel>::DumpPolyhedronToOFF] File already exists: " << filename <<
                    "\n Do you want to overwrite it? (y/[n]): ";
            std::string answer;
            std::getline(std::cin, answer);
            if (answer.find('y') == std::string::npos && answer.find('Y') == std::string::npos) {
                return;
            }
        }

        std::ofstream out(filename);
        // out << polyhedron_combined_;
        CGAL::IO::write_OFF(out, polyhedron_combined_);
        out.close();
    }

    template<typename Kernel>
    void ApproximateBoxes<Kernel>::ApproximateGeometry() {
        std::vector<Point> points = GetAllPoints(mesh_);
        Octree octree{points};

        octree.refine(CGAL::Orthtrees::Maximum_depth_and_maximum_number_of_inliers(
            octree_max_depth_, octree_max_inliers_));
        octree.grade();

        Node_vector nodes_inside;
        ExtractNodesInsideBoundary(octree, nodes_inside);

        RemoveParentNodes(octree, nodes_inside);

        std::vector<CGAL::Bbox_3> bboxes;
        ExtractBoundingBoxes(octree, nodes_inside, bboxes);

        if (divide_larger_bboxes_) DivideLargerBoxes(bboxes);

        std::cout << "[ApproximateBoxes] Building polyhedra from " << bboxes.size() << " bounding boxes..." << '\n';
        std::cout << "[ApproximateBoxes] Using " << threads_ << " threads." << '\n';

        BuildHexahedralMesh(bboxes);
    }

    template<typename Kernel>
    void ApproximateBoxes<Kernel>::ApproximateGeometrySimple(CGAL::Vector_3<Kernel> factor) {
        CGAL::Bbox_3 surface_bbox{};
        ComputeSurfaceMeshBoundingBox(mesh_, surface_bbox);

        std::vector<CGAL::Bbox_3> bboxes_grid;
        bboxes_grid.push_back(surface_bbox);

        DivideLargerBox(factor, bboxes_grid);

        std::vector<CGAL::Bbox_3> bboxes;
        for (const auto& bbox: bboxes_grid) {
            // Center
            double x = (bbox.xmin() + bbox.xmax()) / 2.0;
            double y = (bbox.ymin() + bbox.ymax()) / 2.0;
            double z = (bbox.zmin() + bbox.zmax()) / 2.0;

            if (Point center{x, y, z}; IsPointInside(center, mesh_) == CGAL::ON_BOUNDED_SIDE) {
                bboxes.push_back(bbox);
            }
        }

        std::cout << "[ApproximateBoxes] Building polyhedra from " << bboxes.size() << " bounding boxes..." << '\n';
        std::cout << "[ApproximateBoxes] Using " << threads_ << " threads." << '\n';

        BuildHexahedralMesh(bboxes);
    }

    template<typename Kernel>
    CGAL::Bounded_side ApproximateBoxes<Kernel>::IsPointInside(const Point& p, const SurfaceMesh& msh) {
        const CGAL::Side_of_triangle_mesh<SurfaceMesh, Kernel> inside(msh);

        return inside(p);
    }

    template<typename Kernel>
    double ApproximateBoxes<Kernel>::FindSmallestBoxSize(const std::vector<CGAL::Bbox_3>& boxes) {
        double min_size = std::numeric_limits<double>::max();

        for (const auto& box: boxes) {
            if (const double size = box.xmax() - box.xmin(); size < min_size) {
                min_size = size;
            }
        }

        return min_size;
    }

    template<typename Kernel>
    void ApproximateBoxes<Kernel>::ExtractBoundingBoxes(const Octree& octree,
                                                        const Node_vector& nodes,
                                                        std::vector<CGAL::Bbox_3>& bboxes) {
        for (const auto& node: nodes) {
            bboxes.push_back(octree.bbox(node));
        }
    }

    template<typename Kernel>
    void ApproximateBoxes<Kernel>::DivideLargerBox(CGAL::Vector_3<Kernel> factor, std::vector<CGAL::Bbox_3>& boxes) {
        // Get the original box
        const CGAL::Bbox_3 original_box = boxes.back();
        boxes.pop_back();

        // Calculate the dimensions of the original box
        double original_width = original_box.xmax() - original_box.xmin();
        double original_height = original_box.ymax() - original_box.ymin();
        double original_depth = original_box.zmax() - original_box.zmin();

        // Calculate the dimensions of the smaller boxes
        const double width = original_width / factor.x();
        const double height = original_height / factor.y();
        const double depth = original_depth / factor.z();

        // Calculate the number of smaller boxes in each dimension
        const int num_boxes_x = static_cast<int>(factor.x());
        const int num_boxes_y = static_cast<int>(factor.y());
        const int num_boxes_z = static_cast<int>(factor.z());

        // Create the smaller boxes
        for (int i = 0; i < num_boxes_x; ++i) {
            for (int j = 0; j < num_boxes_y; ++j) {
                for (int k = 0; k < num_boxes_z; ++k) {
                    const double xmin = original_box.xmin() + i * width;
                    const double ymin = original_box.ymin() + j * height;
                    const double zmin = original_box.zmin() + k * depth;

                    CGAL::Bbox_3 new_box(xmin, ymin, zmin, xmin + width, ymin + height, zmin + depth);
                    boxes.push_back(new_box);
                }
            }
        }
    }

    template<typename Kernel>
    void ApproximateBoxes<Kernel>::DivideLargerBoxes(std::vector<CGAL::Bbox_3>& boxes) {
        const double min_size = FindSmallestBoxSize(boxes);

        std::vector<CGAL::Bbox_3> divided_boxes;
        for (const auto& box: boxes) {
            if (const double size = box.xmax() - box.xmin(); size > min_size) {
                // Divide the bounding box into smaller boxes of size min_size
                const int num_divisions = static_cast<int>(size / min_size);
                CGAL::Vector_3<Kernel> factor(num_divisions, num_divisions, num_divisions);
                std::vector<CGAL::Bbox_3> temp_boxes = {box};
                DivideLargerBox(factor, temp_boxes);
                divided_boxes.insert(divided_boxes.end(), temp_boxes.begin(), temp_boxes.end());
            } else {
                divided_boxes.push_back(box);
            }
        }

        boxes = divided_boxes;
    }

    template<typename Kernel>
    void ApproximateBoxes<Kernel>::ComputeSurfaceMeshBoundingBox(const SurfaceMesh& mesh, CGAL::Bbox_3& bbox) {
        for (auto v: mesh.vertices()) {
            bbox += mesh.point(v).bbox();
        }
    }

    template<typename Kernel>
    void ApproximateBoxes<Kernel>::CreateSurfaceMeshFromBboxes(const std::vector<CGAL::Bbox_3>& bboxes,
                                                               SurfaceMesh& mesh) {
        for (const auto& bbox: bboxes) {
            // Create the 8 vertices of the cuboid
            auto v0 = mesh.add_vertex(Point(bbox.xmin(), bbox.ymin(), bbox.zmin()));
            auto v1 = mesh.add_vertex(Point(bbox.xmax(), bbox.ymin(), bbox.zmin()));
            auto v2 = mesh.add_vertex(Point(bbox.xmax(), bbox.ymax(), bbox.zmin()));
            auto v3 = mesh.add_vertex(Point(bbox.xmin(), bbox.ymax(), bbox.zmin()));
            auto v4 = mesh.add_vertex(Point(bbox.xmin(), bbox.ymin(), bbox.zmax()));
            auto v5 = mesh.add_vertex(Point(bbox.xmax(), bbox.ymin(), bbox.zmax()));
            auto v6 = mesh.add_vertex(Point(bbox.xmax(), bbox.ymax(), bbox.zmax()));
            auto v7 = mesh.add_vertex(Point(bbox.xmin(), bbox.ymax(), bbox.zmax()));

            // Create the 12 faces of the cuboid
            mesh.add_face(v0, v1, v2, v3); // bottom face
            mesh.add_face(v4, v5, v6, v7); // top face
            mesh.add_face(v0, v1, v5, v4); // front face
            mesh.add_face(v2, v3, v7, v6); // back face
            mesh.add_face(v1, v2, v6, v5); // right face
            mesh.add_face(v3, v0, v4, v7); // left face
        }
    }

    template<typename Kernel>
    void ApproximateBoxes<Kernel>::ExtractNodesInsideBoundary(Octree& octree,
                                                              Node_vector& nodes_inside) {
        for (const auto& node: octree.template traverse<Preorder_traversal>()) {
            if (const auto& node_coordinates = octree.barycenter(node);
                IsPointInside(node_coordinates, mesh_) == CGAL::ON_BOUNDED_SIDE) {
                nodes_inside.push_back(node);
            }
        }
    }

    template<typename Kernel>
    void ApproximateBoxes<Kernel>::RemoveParentNodes(Octree& octree, Node_vector& nodes_inside) {
        std::set<typename Octree::Node> nodes_to_remove;

        if (remove_root_node_) {
            nodes_inside.erase(std::remove_if(nodes_inside.begin(), nodes_inside.end(),
                                              [&octree](const auto& node) {
                                                  return node == octree.root();
                                              }));
        }

        for (const auto& node: nodes_inside) {
            nodes_to_remove.insert(node.parent());
        }

        for (const auto& node: nodes_to_remove) {
            nodes_inside.erase(std::remove(nodes_inside.begin(), nodes_inside.end(), node), nodes_inside.end());
        }
    }

    template<typename Kernel>
    void ApproximateBoxes<Kernel>::BuildHexahedralMesh(const std::vector<CGAL::Bbox_3>& bboxes) {
        const auto total_bboxes = static_cast<const double>(bboxes.size());
        int processed_bboxes = 0;

        std::mutex polyhedron_list_mutex;

        auto process_bbox = [&](const auto& bbox) {
            BuildPolyhedronFromBbox build_polyhedron(bbox);
            polyhedron_combined_.delegate(build_polyhedron);
            Polyhedron poly{};
            poly.delegate(build_polyhedron);

            polyhedron_list_mutex.lock();
            polyhedron_list_.push_back(poly);
            polyhedron_list_mutex.unlock();

            processed_bboxes++;
            utils::PrintProgress(static_cast<double>(processed_bboxes) / total_bboxes);
        };

        // Divide bboxes into chunks and process each chunk in a separate thread
        const size_t num_threads = threads_;
        const size_t chunk_size = (bboxes.size() + num_threads - 1) / num_threads;

        std::vector<std::thread> threads_vector;
        auto it = bboxes.begin();
        for (size_t i = 0; i < num_threads; ++i) {
            auto end_it = std::next(
                it, std::min(chunk_size, static_cast<size_t>(std::distance(it, bboxes.end()))));

            threads_vector.emplace_back([=, &process_bbox] {
                std::for_each(it, end_it, process_bbox);
            });

            it = end_it;
        }

        // Wait for all threads to finish
        for (auto& thread: threads_vector) {
            thread.join();
        }

        std::cout << "\n\n";

        hex_mesh_.SetPolyhedronList(polyhedron_list_);
        hex_mesh_.BuildMesh(threads_);
    }

    template class ApproximateBoxes<CGAL::Simple_cartesian<double> >;
    template class ApproximateBoxes<CGAL::Exact_predicates_inexact_constructions_kernel>;
} // namespace approx_boxes
