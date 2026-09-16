// test.cpp to validate CI review setup
#include <vector>

int* makeArray(int n)
{ // raw new, raw pointer ownership
    int* arr = new int[n];
    for (int i = 0; i < n; i++)
        arr[i] = i;
    return arr;
}

double computeAvg(std::vector<double> v)
{ // unnecessary copy, should be const&
    double sum = 0;
    for (int i = 0; i < v.size(); i++)
    { // manual loop, could use accumulate
        sum += v[i];
    }
    return sum / v.size(); // no guard against empty vector
}
