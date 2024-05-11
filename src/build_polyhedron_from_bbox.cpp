/**
 * @file build_polyhedron_from_bbox.cpp
 * @brief MIT License
 *
 * Copyright (c) 2024 Paul-Otto Müller <https://github.com/paulotto>
 * License: LGPL-3.0-or-later (https://github.com/paulotto/approximate_boxes/blob/main/LICENSE)
 * URL: https://github.com/paulotto/approximate_boxes/blob/main/src/build_polyhedron_from_bbox.cpp
 *
 * @author Paul-Otto Müller
 * @date 10.05.2024
 */

#include <CGAL/Polyhedron_incremental_builder_3.h>

#include "approximate_boxes/build_polyhedron_from_bbox.h"


namespace approx_boxes {
    void BuildPolyhedronFromBbox::operator()(HalfedgeDS& hds) {
        CGAL::Polyhedron_incremental_builder_3<HalfedgeDS> builder(hds, true);
        builder.begin_surface(8, 12);

        // Add vertices
        builder.add_vertex(Point(bbox_.xmin(), bbox_.ymin(), bbox_.zmin()));
        builder.add_vertex(Point(bbox_.xmax(), bbox_.ymin(), bbox_.zmin()));
        builder.add_vertex(Point(bbox_.xmax(), bbox_.ymax(), bbox_.zmin()));
        builder.add_vertex(Point(bbox_.xmin(), bbox_.ymax(), bbox_.zmin()));
        builder.add_vertex(Point(bbox_.xmin(), bbox_.ymin(), bbox_.zmax()));
        builder.add_vertex(Point(bbox_.xmax(), bbox_.ymin(), bbox_.zmax()));
        builder.add_vertex(Point(bbox_.xmax(), bbox_.ymax(), bbox_.zmax()));
        builder.add_vertex(Point(bbox_.xmin(), bbox_.ymax(), bbox_.zmax()));

        // Add facets in counter-clockwise order
        builder.begin_facet();
        builder.add_vertex_to_facet(0);
        builder.add_vertex_to_facet(3);
        builder.add_vertex_to_facet(2);
        builder.add_vertex_to_facet(1);
        builder.end_facet();
        builder.begin_facet();
        builder.add_vertex_to_facet(4);
        builder.add_vertex_to_facet(5);
        builder.add_vertex_to_facet(6);
        builder.add_vertex_to_facet(7);
        builder.end_facet();
        builder.begin_facet();
        builder.add_vertex_to_facet(0);
        builder.add_vertex_to_facet(4);
        builder.add_vertex_to_facet(7);
        builder.add_vertex_to_facet(3);
        builder.end_facet();
        builder.begin_facet();
        builder.add_vertex_to_facet(1);
        builder.add_vertex_to_facet(2);
        builder.add_vertex_to_facet(6);
        builder.add_vertex_to_facet(5);
        builder.end_facet();
        builder.begin_facet();
        builder.add_vertex_to_facet(0);
        builder.add_vertex_to_facet(1);
        builder.add_vertex_to_facet(5);
        builder.add_vertex_to_facet(4);
        builder.end_facet();
        builder.begin_facet();
        builder.add_vertex_to_facet(2);
        builder.add_vertex_to_facet(3);
        builder.add_vertex_to_facet(7);
        builder.add_vertex_to_facet(6);
        builder.end_facet();

        builder.end_surface();
    }
} // namespace approx_boxes
