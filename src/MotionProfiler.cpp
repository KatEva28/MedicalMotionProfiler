#include "../include/MotionProfiler.h"
#include <iostream>
#include <iomanip>
#include <fstream>

// ==================================
// CONSTRUCTOR: Load coefficients from thesis
// Example: {0,0,0,2.68,1.45,0.12} = 100mm travel in 5s
// ==================================
MotionProfiler::MotionProfiler(const std::vector<double>& c, double dur)
    : coeffs(Eigen::Map<const Eigen::Vector6d>(c.data())), duration(dur) {}

// POSITION s(t) = a5*t^5 + a4*t^4 + ... + a0
double MotionProfiler::position(double t) const {
    double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
    return coeffs(0) + coeffs(1) * t + coeffs(2) * t2 + coeffs(3) * t3 + coeffs(4) * t4 + coeffs(5) * t5;
}

// 1st DERIVATIVE: v(t) = ds/dt
double MotionProfiler::velocity(double t) const {
    double t2 = t * t, t3 = t2 * t, t4 = t3 * t;
    return coeffs(1) + 2 * coeffs(2) * t + 3 * coeffs(3) * t2 + 4 * coeffs(4) * t3 + 5 * coeffs(5) * t4;
}

// 2nd DERIVATIVE: a(t) = dv/dt
double MotionProfiler::acceleration(double t) const {
    double t2 = t * t, t3 = t2 * t;
    return 2 * coeffs(2) + 6 * coeffs(3) * t + 12 * coeffs(4) * t2 + 20 * coeffs(5) * t3;
}

// 3rd DERIVATIVE: j(t) = da/dt (Jerk)
double MotionProfiler::jerk(double t) const {
    double t2 = t * t;
    return 6 * coeffs(3) + 24 * coeffs(4) * t + 60 * coeffs(5) * t2;
}

// CONSOLE OUTPUT: 8kHz SAMPLING (0.000125s steps)
void MotionProfiler::printProfile() const {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n=== 5th-Order Motion Profile ===\n";

    double dt = 1.0 / 8000.0;  // 0.000125 seconds per sample
    int max_samples = 40;  // Show first 40 samples (5ms) + endpoints only (too many for console)
    std::cout << "Showing first 5ms + endpoints (40k total samples)...\n\n";

    // First 40 samples (5ms)
    for (int i = 0; i < max_samples; i++) 
    {
        double t = i * dt;
        std::cout << "t=" << t * 1000 << "ms | Pos=" << position(t) << "mm | "
            << "Vel=" << velocity(t) << "mm/s | Acc="
            << acceleration(t) << "mm/s²\n";
    }

    std::cout << "\n...[40,000 total samples]... \n";
    std::cout << "t=5.000s | Pos=" << position(duration) << "mm | "
        << "Vel=" << velocity(duration) << "mm/s | Acc="
        << acceleration(duration) << "mm/s² | Jerk=" << jerk(duration) << "mm/s³\n";
}

// CSV EXPORT : 100Hz(practical for Excel / MATLAB)
// Full 8kHz data too large for spreadsheets
std::string MotionProfiler::toCSV() const {
    std::string csv;
    double dt = 0.01;  // 100Hz = 500 samples total (manageable)
    for (double t = 0; t <= duration; t += dt) 
    {
        csv += std::to_string(t) + "," + std::to_string(position(t)) + ","
            + std::to_string(velocity(t)) + "," + std::to_string(acceleration(t)) + "\n";
    }
    return csv;
}
