#include "grid.h"

template <typename T>
Grid3D<T>::Grid3D(const std::vector<std::string> &maps){

}

template <typename T>
Grid3D<T>::Grid3D(const Grid3D<T> &other)
{
    unsigned x, size = other.w * other.h * other.l;
    data = new T[size];
    w = other.w;
    h = other.h;
    l = other.l;
    for (x = 0; x < size; ++x)
        data[x] = other.data[x];
}

template <typename T>
Grid3D<T>::Grid3D(unsigned width, unsigned height, unsigned length)
{
    w = width;
    h = height;
    l = length;
    data = new T[w * l * h];
}
