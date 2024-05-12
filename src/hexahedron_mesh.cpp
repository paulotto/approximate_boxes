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

#include <iostream>
#include <filesystem>

#include <CGAL/Polyhedron_3.h>

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
    void HexahedronMesh<Polyhedron_T>::SortNodesForGmsh(std::vector<Node*>& nodes) {
        // First, find the center of the nodes
        double center_x = 0.0, center_y = 0.0, center_z = 0.0;
        for (const auto& node: nodes) {
            center_x += node->pos.x();
            center_y += node->pos.y();
            center_z += node->pos.z();
        }
        center_x /= static_cast<double>(nodes.size());
        center_y /= static_cast<double>(nodes.size());
        center_z /= static_cast<double>(nodes.size());

        Node center;
        center.pos = Point(center_x, center_y, center_z);

        // Then, sort the nodes based on their angle from the center in the XY plane
        std::sort(nodes.begin(), nodes.end(), [&center](const Node* a, const Node* b) {
            double angleA = std::atan2(a->pos.y() - center.pos.y(), a->pos.x() - center.pos.x());
            double angleB = std::atan2(b->pos.y() - center.pos.y(), b->pos.x() - center.pos.x());
            return angleA < angleB;
        });

        // Finally, sort the nodes based on their Z coordinate, keeping the order within each Z level
        std::stable_sort(nodes.begin(), nodes.end(), [](const Node* a, const Node* b) {
            return a->pos.z() < b->pos.z();
        });
    }

    template<typename Polyhedron_T>
    void HexahedronMesh<Polyhedron_T>::AddNodes() {
        for (const auto& polyhedron: polyhedron_list_) {
            for (const auto& vertex: polyhedron.points()) {
                auto node_it = std::find_if(nodes_.begin(), nodes_.end(), [&vertex](const auto& n) {
                    return n->pos == vertex;
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
        }

        unsigned int tag{1};
        for (const auto& node: nodes_) {
            node->tag = tag++;
        }
    }

    template<typename Polyhedron_T>
    void HexahedronMesh<Polyhedron_T>::AddElements() {
        unsigned int elem_tag{1};
        for (const auto& poly: polyhedron_list_) {
            auto keys = GetKeysWithSameValue<Node*, const Polyhedron_T*>(
                node_polyhedron_multimap_, &poly);

            elements_.push_back(new Element(elem_tag++, keys, ComputeBoundingBoxPolyhedron(poly)));
        }
    }

    template class HexahedronMesh<CGAL::Polyhedron_3<CGAL::Simple_cartesian<double> > >;
} // namespace approx_boxes
