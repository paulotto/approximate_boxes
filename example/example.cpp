/**
* @file example.cpp
 * @brief MIT License
 *
 * Copyright (c) 2024 Paul-Otto Müller <https://github.com/paulotto>
 * License: LGPL-3.0-or-later (https://github.com/paulotto/approximate_boxes/blob/main/LICENSE)
 * URL: https://github.com/paulotto/approximate_boxes/blob/main/example/example.cpp
 *
 * @author Paul-Otto Müller
 * @date 11.05.2024
 */

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include "approximate_boxes/approximate_boxes.h"


int main(int argc, char* argv[]) {

    using EpicKernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using CartKernel = CGAL::Simple_cartesian<double>;

    using namespace approx_boxes;

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <filename>" << std::endl;
        return 1;
    }

    std::string filename(argv[1]);

    ApproximateBoxes<CartKernel> abox(filename);
    abox.SetNumberOfThreads(0);
    abox.SetDivideLargerBboxes(false);
    abox.ApproximateGeometry();

    filename.erase(filename.size() - 3);
    filename += "msh";
    abox.DumpPolyhedraToGmesh(filename);

    return 0;
}