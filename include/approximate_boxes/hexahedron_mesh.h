/**
 * @file hexahedron_mesh.h
 * @brief
 *
 * Copyright (c) 2024 Paul-Otto Müller <https://github.com/paulotto>
 * License: LGPL-3.0-or-later (https://github.com/paulotto/approximate_boxes/blob/main/LICENSE)
 * URL: https://github.com/paulotto/approximate_boxes/blob/main/include/approximate_boxes/hexahedron_mesh.h
 *
 * @author Paul-Otto Müller
 * @date 10.05.2024
 */

#ifndef HEXAHEDRON_MESH_H
#define HEXAHEDRON_MESH_H

#include <set>
#include <map>
#include <list>
#include <vector>

#include <CGAL/Simple_cartesian.h>


namespace approx_boxes {
    /**
     * @class HexahedronMesh
     * @brief A class to build a hexahedral mesh from a list of polyhedra.
     * @tparam Polyhedron_T The type of the polyhedra to build the mesh from.
     */
    template<typename Polyhedron_T>
    class HexahedronMesh {
        using Kernel = CGAL::Simple_cartesian<double>;
        using Point = Kernel::Point_3;

        struct Node {
            unsigned int tag{0};
            Point pos{0.0, 0.0, 0.0};

            Node() : Node(0, {0.0, 0.0, 0.0}) {
            }

            Node(unsigned int tag, const Point& pos)
                : tag(tag), pos(pos) {
            }

            friend bool operator<(const Node& lhs, const Node& rhs) {
                return lhs.pos < rhs.pos;
            }

            friend bool operator>(const Node& lhs, const Node& rhs) {
                return rhs < lhs;
            }

            friend bool operator==(const Node& lhs, const Node& rhs) {
                return lhs.pos == rhs.pos;
            }
        };

        struct Element {
            unsigned int tag{0};
            std::set<Node*> nodes{};
            CGAL::Bbox_3 bbox{};

            Element(unsigned int tag, const std::set<Node*>& nodes, const CGAL::Bbox_3& bbox)
                : tag(tag), nodes(nodes), bbox(bbox) {
            }
        };

        public:
            explicit HexahedronMesh()
                : polyhedron_list_({}) {
            }

            explicit HexahedronMesh(const std::list<Polyhedron_T>& polyhedron_list)
                : polyhedron_list_(polyhedron_list) {
            }

            virtual ~HexahedronMesh();

            std::set<Node*> GetNodes() const {
                return nodes_;
            }

            std::vector<Element*> GetElements() const {
                return elements_;
            }

            void SetPolyhedronList(const std::list<Polyhedron_T>& polyhedron_list) {
                polyhedron_list_ = polyhedron_list;
            }

            virtual void BuildMesh() {
                AddNodes();
                AddElements();
            }

            virtual void BuildMesh(const std::string& gmsh_file) {
                BuildMesh();
                WriteGmshFile(gmsh_file);
            }

            virtual void AddNodes();

            virtual void AddElements();

            virtual void WriteGmshFile(const std::string& filename);

            static CGAL::Bbox_3 ComputeBoundingBoxPolyhedron(const Polyhedron_T& polyhedron);

            template<typename K, typename V>
            static std::set<K> GetKeysWithSameValue(const std::multimap<K, V>& mmap, const V& value);

            /**
             * This function assumes that the nodes are already arranged in a way that forms a hexahedron.
             * The nodes are sorted in-place according to the Gmsh ordering for a hexahedron:
             * 1. Bottom face, listed in counter-clockwise order as viewed from above.
             * 2. Top face, listed in counter-clockwise order as viewed from above.
             * @param nodes A vector of nodes
             */
            static void SortNodesForGmsh(std::vector<Node*>& nodes);

        private:
            std::set<Node*> nodes_{};
            std::vector<Element*> elements_{};
            std::list<Polyhedron_T> polyhedron_list_{};
            std::multimap<Node*, const Polyhedron_T*> node_polyhedron_multimap_{};
    };
} // namespace approx_boxes

#endif //HEXAHEDRON_MESH_H
