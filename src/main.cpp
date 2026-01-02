#include "../include/MotionProfiler.h"
#include <vector>
#include <fstream>

int main() {
    // Polynomial coefficients fitted for boundary conditions (from thesis)
    std::vector<double> coeffs = { 0.0, 0.0, 0.0, 2.68, 1.45, 0.12 }; // 100mm in 5s
    MotionProfiler profiler(coeffs, 5.0);

    // 1. Display results in console
    profiler.printProfile();

    // 2. Export CSV for MATLAB/Excel analysis
    std::ofstream csv("motion_profile.csv");
    csv << "time,pos,vel,acc\n" << profiler.toCSV();
    csv.close();
    std::cout << "\n motion_profile.csv exported!\n";

    return 0;
}
