/**
 * @file utils.h
 * @brief
 *
 * Copyright (c) 2024 Paul-Otto Müller <https://github.com/paulotto>
 * License: LGPL-3.0-or-later (https://github.com/paulotto/approximate_boxes/blob/main/LICENSE)
 * URL: https://github.com/paulotto/approximate_boxes/blob/main/include/approximate_boxes/utils.h
 *
 * @author Paul-Otto Müller
 * @date 13.05.2024
 */

#ifndef UTILS_H
#define UTILS_H

#include <cstdio>


namespace approx_boxes::utils {
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

    /**
     * @brief Prints a progress bar to the console.
     * @param percentage The percentage of the progress.
     */
    inline void PrintProgress(double percentage) {
        const int val = static_cast<int>(percentage * 100);
        const int lpad = static_cast<int>(percentage * PBWIDTH);
        const int rpad = PBWIDTH - lpad;

        printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
        fflush(stdout);
    }
} // namespace approx_boxes

#endif //UTILS_H
