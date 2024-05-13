//
// Created by paul on 13.05.24.
//

#ifndef UTILS_H
#define UTILS_H

#include <cstdio>


namespace approx_boxes::utils {
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

    inline void PrintProgress(double percentage) {
        int val = static_cast<int>(percentage * 100);
        int lpad = static_cast<int>(percentage * PBWIDTH);
        int rpad = PBWIDTH - lpad;
        printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
        fflush(stdout);
    }
} // namespace approx_boxes

#endif //UTILS_H
