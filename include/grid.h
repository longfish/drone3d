#ifndef GRID3D_H
#define GRID3D_H

#include "drone3d.h"

template <typename T>
class Grid3D
{
protected:
    T *data; // TODO: consider using smart pointer
    unsigned w, h, l;

public:
    Grid3D() {}
    Grid3D(const Grid3D<T> &other);
    Grid3D(unsigned width, unsigned height, unsigned length);
    ~Grid3D() { delete[] data; }
    T &operator()(unsigned x, unsigned y, unsigned z) { return data[z * w * h + y * w + x]; }
    const T &operator()(unsigned x, unsigned y, unsigned z) const { return data[z * w * h + y * w + x]; }
    unsigned size() const { return w * h * l; }

    Grid3D GenerateGrid();
};
#endif