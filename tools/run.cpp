#include "rt/loader.hpp"
#include <iostream>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: run <radar_data.h5>\n";
        return 1;
    }

    std::cout << rt::count_detections(argv[1]) << " detections\n";
    return 0;
}
