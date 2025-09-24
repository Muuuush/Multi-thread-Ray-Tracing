#pragma once

#include "interval.hpp"
#include "vec3.hpp"

using color = vec3;
using uchar = unsigned char;

inline uchar color_f2i(double x)
{
    static const interval intensity(0.0, 1 - epsilon);
    return (uchar)(256 * intensity.clamp(x));
}

inline double gamma_correct(double x)
{
    if (x > 0)
        return std::sqrt(x);
    return 0;
}

inline void
write_color(uchar data[], color c, int x, int y, int w, int n)
{
    for (int i = 0; i < std::min(n, 3); i++)
        data[(y * w + x) * n + i] = color_f2i(gamma_correct(c[i]));
}
