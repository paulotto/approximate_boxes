/**
 * @file hexahedron_mesh.cpp
 * @brief MIT License
 *
 * Copyright (c) 2024 Paul-Otto Müller <https://github.com/paulotto>
 * License: LGPL-3.0-or-later (https://github.com/paulotto/approximate_boxes/blob/main/LICENSE)
 * URL: https://github.com/paulotto/approximate_boxes/blob/main/src/hexahedron_mesh.cpp
 *
 * @author Paul-Otto Müller
 * @date 10.05.2024
 */

#include <mutex>
#include <thread>
#include <iostream>
#include <filesystem>

#include <CGAL/Polyhedron_3.h>

#include "approximate_boxes/utils.h"
#include "approximate_boxes/hexahedron_mesh.h"


namespace approx_boxes {
    template<typename Polyhedron_T>
    HexahedronMesh<Polyhedron_T>::~HexahedronMesh() {
        for (const auto& node: nodes_) {
            delete node;
        }
        nodes_.clear();

        for (const auto& element: elements_) {
            delete element;
        }
        elements_.clear();
    }

    template<typename Polyhedron_T>
    void HexahedronMesh<Polyhedron_T>::WriteGmshFile(const std::string& filename) {
        if (std::filesystem::exists(filename)) {
            std::cout << "[HexahedronMesh<Polyhedron_T>::WriteGmshFile] File already exists: " << filename <<
                    "\n Do you want to overwrite it? (y/[n]): ";
            std::string answer;
            std::getline(std::cin, answer);
            if (answer.find('y') == std::string::npos && answer.find('Y') == std::string::npos) {
                return;
            }
        }

        std::ofstream out(filename);

        CGAL::Bbox_3 bbox_surface;
        for (const auto& poly: polyhedron_list_) {
            bbox_surface += ComputeBoundingBoxPolyhedron(poly);
        }

        // Header
        out << "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n";

        // Entities
        // numPoints(size_t) numCurves(size_t) numSurfaces(size_t) numVolumes(size_t)
        // pointTag(int) X(double) Y(double) Z(double)
        //   numPhysicalTags(size_t) physicalTag(int) ...
        // ...
        // curveTag(int) minX(double) minY(double) minZ(double)
        //   maxX(double) maxY(double) maxZ(double)
        //   numPhysicalTags(size_t) physicalTag(int) ...
        //   numBoundingPoints(size_t) pointTag(int; sign encodes orientation) ...
        // ...
        // surfaceTag(int) minX(double) minY(double) minZ(double)
        //   maxX(double) maxY(double) maxZ(double)
        //   numPhysicalTags(size_t) physicalTag(int) ...
        //   numBoundingCurves(size_t) curveTag(int; sign encodes orientation) ...
        // ...
        // volumeTag(int) minX(double) minY(double) minZ(double)
        //   maxX(double) maxY(double) maxZ(double)
        //   numPhysicalTags(size_t) physicalTag(int) ...
        //   numBoundngSurfaces(size_t) surfaceTag(int; sign encodes orientation) ...
        // out << "$Entities\n" << "0 0 0 " << elements_.size() << '\n';
        // // for (const auto& node: nodes_) {
        // //     out << node->tag << ' ' << node->pos.x() << ' ' << node->pos.y() << ' ' << node->pos.z() << " 0\n";
        // // }
        // for (const auto& element: elements_) {
        //     out << element->tag << ' ' << element->bbox.xmin() << ' ' << element->bbox.ymin() << ' '
        //             << element->bbox.zmin() << ' ' << element->bbox.xmax() << ' ' << element->bbox.ymax() << ' '
        //             << element->bbox.zmax() << " 0 0\n";
        // }
        // out << "$EndEntities\n";

        // Nodes
        // numEntityBlocks(size_t) numNodes(size_t) minNodeTag(size_t) maxNodeTag(size_t)
        //  entityDim(int) entityTag(int) parametric(int) numNodesInBlock(size_t)
        //      nodeTag(size_t)
        //      ...
        //      x(double) y(double) z(double)
        //          < u(double; if parametric and entityDim >= 1) >
        //          < v(double; if parametric and entityDim >= 2) >
        //          < w(double; if parametric and entityDim == 3) >
        //      ...
        out << "$Nodes\n" << "1 " << nodes_.size() << " 1 " << nodes_.size() << '\n';
        out << "3 1 0 " << nodes_.size() << '\n';
        for (const auto& node: nodes_) {
            out << node->tag << '\n';
        }
        for (const auto& node: nodes_) {
            out << node->pos.x() << ' ' << node->pos.y() << ' ' << node->pos.z() << '\n';
        }
        out << "$EndNodes\n";

        // Elements
        // numEntityBlocks(size_t) numElements(size_t) minElementTag(size_t) maxElementTag(size_t)
        //  entityDim(int) entityTag(int) elementType(int) numElementsInBlock(size_t)
        //      elementTag(size_t) nodeTag(size_t) ...
        //  ...
        out << "$Elements\n" << "1 " << elements_.size() << " 1 " << elements_.size() << '\n';
        out << "3 1 5 " << elements_.size() << '\n';
        for (const auto& element: elements_) {
            out << element->tag << ' ';
            auto node_vec = std::vector<Node*>(element->nodes.begin(), element->nodes.end());
            SortNodesForGmsh(node_vec);
            for (const auto& node: node_vec) {
                out << node->tag << ' ';
            }
            out << '\n';
        }
        out << "$EndElements\n";

        out.close();
    }

    template<typename Polyhedron_T>
    CGAL::Bbox_3 HexahedronMesh<Polyhedron_T>::ComputeBoundingBoxPolyhedron(const Polyhedron_T& polyhedron) {
        CGAL::Bbox_3 bbox;

        if (polyhedron.empty()) {
            return bbox;
        }

        auto vertices = polyhedron.points();
        auto first = vertices.begin();
        bbox = first->bbox();

        for (auto it = std::next(first); it != vertices.end(); ++it) {
            bbox = bbox + it->bbox();
        }

        return bbox;
    }

    template<typename Polyhedron_T>
    template<typename K, typename V>
    std::set<K> HexahedronMesh<Polyhedron_T>::GetKeysWithSameValue(const std::multimap<K, V>& mmap, const V& value) {
        std::set<K> keys;
        for (auto it = mmap.begin(); it != mmap.end(); ++it) {
            if (it->second == value) {
                keys.insert(it->first);
            }
        }
        return keys;
    }

    template<typename Polyhedron_T>
    double HexahedronMesh<Polyhedron_T>::SmallestEdgeLength(const Polyhedron_T& polyhedron) {
        double min_length = std::numeric_limits<double>::max();

        for (const auto& edge: polyhedron.edges()) {
            if (const double length = CGAL::squared_distance(edge.vertex()->point(), edge.opposite()->vertex()->point())
                ; length < min_length) {
                min_length = length;
            }
        }

        return std::sqrt(min_length);
    }

    template<typename Polyhedron_T>
    void HexahedronMesh<Polyhedron_T>::SortNodesForGmsh(std::vector<Node*>& nodes) {
        // Calculate the centroid of the nodes
        double cx = 0.0, cy = 0.0, cz = 0.0;
        for (const auto& node: nodes) {
            cx += node->pos.x();
            cy += node->pos.y();
            cz += node->pos.z();
        }
        cx /= nodes.size();
        cy /= nodes.size();
        cz /= nodes.size();

        // Define a lambda function to calculate the angle between a node and the centroid in the XY plane
        auto AngleFromCenter = [&cx, &cy](const Node* node) -> double {
            return std::atan2(node->pos.y() - cy, node->pos.x() - cx);
        };

        // Separate nodes into two layers (bottom and top) based on their Z-coordinate
        std::vector<Node*> bottom_nodes, top_nodes;
        for (const auto& node: nodes) {
            if (node->pos.z() < cz) {
                bottom_nodes.push_back(node);
            } else {
                top_nodes.push_back(node);
            }
        }

        // Sort nodes within each layer based on their angle from the centroid
        std::sort(bottom_nodes.begin(), bottom_nodes.end(), [&](const Node* a, const Node* b) -> bool {
            return AngleFromCenter(a) < AngleFromCenter(b);
        });
        std::sort(top_nodes.begin(), top_nodes.end(), [&](const Node* a, const Node* b) -> bool {
            return AngleFromCenter(a) < AngleFromCenter(b);
        });

        // Concatenate the two layers to form the final sorted list of nodes
        nodes.clear();
        nodes.insert(nodes.end(), bottom_nodes.begin(), bottom_nodes.end());
        nodes.insert(nodes.end(), top_nodes.begin(), top_nodes.end());
    }

    template<typename Polyhedron_T>
    void HexahedronMesh<Polyhedron_T>::AddNodes() {
        double tolerance = CalculateTolerance();
        same_node_tolerance_ = tolerance;

        if (!CheckPolyhedraNodes(polyhedron_list_)) {
            throw std::runtime_error("[HexahedronMesh<Polyhedron_T>::AddNodes] Not all polyhedra have 8 nodes!");
        }

        std::cout << "[HexahedronMesh] Add nodes to the mesh...\n";

        const double total_polyhedrons = polyhedron_list_.size();
        int processed_polyhedrons = 0;

        for (const auto& polyhedron: polyhedron_list_) {
            for (const auto& vertex: polyhedron.points()) {
                auto node_it = std::find_if(nodes_.begin(), nodes_.end(), [&vertex, tolerance](const auto& n) {
                    return CGAL::squared_distance(n->pos, vertex) < tolerance * tolerance;
                });

                if (node_it != nodes_.end()) {
                    node_polyhedron_multimap_.insert({*node_it, &polyhedron});
                } else {
                    auto* node = new Node();
                    node->pos = vertex;
                    nodes_.insert(node);
                    node_polyhedron_multimap_.insert({node, &polyhedron});
                }
            }
            processed_polyhedrons++;
            utils::PrintProgress(static_cast<double>(processed_polyhedrons) / total_polyhedrons);
        }
        std::cout << "\n\n";

        unsigned int tag{1};
        for (const auto& node: nodes_) {
            node->tag = tag++;
        }

#if DEBUG
        std::cout << "[HexahedronMesh<Polyhedron_T>::AddNodes] Number of nodes: " << nodes_.size() << '\n';
#endif
    }

    template<typename Polyhedron_T>
    void HexahedronMesh<Polyhedron_T>::AddNodes(size_t threads) {
        double tolerance = CalculateTolerance();
        same_node_tolerance_ = tolerance;

        if (!CheckPolyhedraNodes(polyhedron_list_)) {
            throw std::runtime_error("[HexahedronMesh<Polyhedron_T>::AddNodes] Not all polyhedra have 8 nodes!");
        }

        std::cout << "[HexahedronMesh] Add nodes to the mesh...\n";

        const double total_polyhedrons = polyhedron_list_.size();
        int processed_polyhedrons = 0;

        std::mutex nodes_mutex;
        std::mutex node_polyhedron_multimap_mutex;

        auto process_polyhedron = [&](const auto& polyhedron) {
            for (const auto& vertex: polyhedron.points()) {
                nodes_mutex.lock();
                auto node_it = std::find_if(nodes_.begin(), nodes_.end(), [&vertex, tolerance](const auto& n) {
                    return CGAL::squared_distance(n->pos, vertex) < tolerance * tolerance;
                });

                if (node_it != nodes_.end()) {
                    node_polyhedron_multimap_mutex.lock();
                    node_polyhedron_multimap_.insert({*node_it, &polyhedron});
                    node_polyhedron_multimap_mutex.unlock();
                } else {
                    auto* node = new Node();
                    node->pos = vertex;
                    nodes_.insert(node);
                    node_polyhedron_multimap_mutex.lock();
                    node_polyhedron_multimap_.insert({node, &polyhedron});
                    node_polyhedron_multimap_mutex.unlock();
                }
                nodes_mutex.unlock();
            }
            processed_polyhedrons++;
            utils::PrintProgress(static_cast<double>(processed_polyhedrons) / total_polyhedrons);
        };

        // Divide polyhedron_list_ into chunks and process each chunk in a separate thread
        const size_t num_threads = threads ? threads : std::thread::hardware_concurrency();
        const size_t chunk_size = (polyhedron_list_.size() + num_threads - 1) / num_threads;

        std::vector<std::thread> threads_vector;
        auto it = polyhedron_list_.begin();
        for (size_t i = 0; i < num_threads; ++i) {
            auto end_it = std::next(
                it, std::min(chunk_size, static_cast<size_t>(std::distance(it, polyhedron_list_.end()))));

            threads_vector.emplace_back([=, &process_polyhedron] {
                std::for_each(it, end_it, process_polyhedron);
            });

            it = end_it;
        }

        // Wait for all threads to finish
        for (auto& thread: threads_vector) {
            thread.join();
        }

        std::cout << "\n\n";

        unsigned int tag{1};
        for (const auto& node: nodes_) {
            node->tag = tag++;
        }
    }

    template<typename Polyhedron_T>
    void HexahedronMesh<Polyhedron_T>::AddElements() {
        unsigned int elem_tag{1};

        std::cout << "[HexahedronMesh] Add elements to the mesh...\n";

        const double total_polyhedrons = polyhedron_list_.size();
        int processed_polyhedrons = 0;

        for (const auto& poly: polyhedron_list_) {
            auto keys = GetKeysWithSameValue<Node*, const Polyhedron_T*>(
                node_polyhedron_multimap_, &poly);

            elements_.push_back(new Element(elem_tag++, keys, ComputeBoundingBoxPolyhedron(poly)));

            processed_polyhedrons++;
            utils::PrintProgress(static_cast<double>(processed_polyhedrons) / total_polyhedrons);
        }
        std::cout << "\n\n";

#if DEBUG
        for (const auto& elem: elements_) {
            if (elem->nodes.size() != 8) {
                std::cerr << "[HexahedronMesh<Polyhedron_T>::AddElements] Element has " << elem->nodes.size()
                        << " nodes instead of 8.\n";
            }
        }
        std::cout << "[HexahedronMesh<Polyhedron_T>::AddElements] Number of elements: " << elements_.size() << '\n';
#endif
    }

    template<typename Polyhedron_T>
    void HexahedronMesh<Polyhedron_T>::AddElements(size_t threads) {
        std::atomic<unsigned int> elem_tag{1};

        std::cout << "[HexahedronMesh] Add elements to the mesh...\n";

        const double total_polyhedrons = polyhedron_list_.size();
        int processed_polyhedrons = 0;

        std::mutex elements_mutex;

        auto process_polyhedron = [&](const auto& poly) {
            auto keys = GetKeysWithSameValue<Node*, const Polyhedron_T*>(
                node_polyhedron_multimap_, &poly);

            elements_mutex.lock();
            elements_.push_back(new Element(elem_tag++, keys, ComputeBoundingBoxPolyhedron(poly)));
            elements_mutex.unlock();

            processed_polyhedrons++;
            utils::PrintProgress(static_cast<double>(processed_polyhedrons) / total_polyhedrons);
        };

        // Divide polyhedron_list_ into chunks and process each chunk in a separate thread
        const size_t num_threads = std::thread::hardware_concurrency();
        const size_t chunk_size = (polyhedron_list_.size() + num_threads - 1) / num_threads;

        std::vector<std::thread> threads_vector;
        auto it = polyhedron_list_.begin();
        for (size_t i = 0; i < num_threads; ++i) {
            auto end_it = std::next(
                it, std::min(chunk_size, static_cast<size_t>(std::distance(it, polyhedron_list_.end()))));

            threads_vector.emplace_back([=, &process_polyhedron] {
                std::for_each(it, end_it, process_polyhedron);
            });

            it = end_it;
        }

        // Wait for all threads to finish
        for (auto& thread: threads_vector) {
            thread.join();
        }

        std::cout << "\n\n";
    }

    template<typename Polyhedron_T>
    double HexahedronMesh<Polyhedron_T>::CalculateTolerance() {
        double min_edge_length = std::numeric_limits<double>::max();

        for (const auto& polyhedron: polyhedron_list_) {
            if (const double edge_length = SmallestEdgeLength(polyhedron); edge_length < min_edge_length) {
                min_edge_length = edge_length;
            }
        }

        // Set the tolerance to a fraction of the smallest edge length
        // The fraction can be adjusted as needed
        return min_edge_length * 0.01;
    }

    template class HexahedronMesh<CGAL::Polyhedron_3<CGAL::Simple_cartesian<double> > >;
} // namespace approx_boxes
