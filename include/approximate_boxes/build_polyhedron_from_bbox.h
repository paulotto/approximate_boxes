/**
 * @file build_polyhedron_from_bbox.h
 * @brief
 *
 * Copyright (c) 2024 Paul-Otto Müller <https://github.com/paulotto>
 * License: LGPL-3.0-or-later (https://github.com/paulotto/approximate_boxes/blob/main/LICENSE)
 * URL: https://github.com/paulotto/approximate_boxes/blob/main/include/approximate_boxes/build_polyhedron_from_bbox.h
 *
 * @author Paul-Otto Müller
 * @date 10.05.2024
 */

#ifndef BUILD_POLYHEDRON_FROM_BBOX_H
#define BUILD_POLYHEDRON_FROM_BBOX_H

#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>


namespace approx_boxes {
    /**
     * @class BuildPolyhedronFromBbox
     * @brief A class to build a polyhedron from a bounding box
     */
    class BuildPolyhedronFromBbox
            : public CGAL::Modifier_base<CGAL::Polyhedron_3<CGAL::Simple_cartesian<double> >::HalfedgeDS> {
        using Kernel = CGAL::Simple_cartesian<double>;
        using Point = Kernel::Point_3;
        using Polyhedron = CGAL::Polyhedron_3<Kernel>;
        using HalfedgeDS = Polyhedron::HalfedgeDS;

        public:
            explicit BuildPolyhedronFromBbox(const CGAL::Bbox_3& bbox)
                : Modifier_base(), bbox_(bbox) {
            }

            void operator()(HalfedgeDS& hds) override;

        private:
            CGAL::Bbox_3 bbox_;
    };
} // namespace approx_boxes

#endif //BUILD_POLYHEDRON_FROM_BBOX_H
