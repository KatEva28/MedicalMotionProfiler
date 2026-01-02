#pragma once
#include <Eigen/Dense> // Matrix math for polynomials
#include <vector>      
#include <string>

// ======================================================================
// MOTION PROFILER THESIS EXTENSION: 5th-Order MotionProfiler class
// Computes Position/Velocity/Acceleration/Jerk for smooth robot motion
// ======================================================================
class MotionProfiler {
private:
    Eigen::Vector6d coeffs; // a5*t^5 + a4*t^4 + ... + a0 (6 coefficients)
    double duration;        // Total motion duration (e.g. 5.0s)
public:
    MotionProfiler(const std::vector<double>& c, double dur);

    // Core functions: 0th-3rd order derivatives
    double position(double t) const;
    double velocity(double t) const;
    double acceleration(double t) const;
    double jerk(double t) const;

    // Output: Console display + CSV export
    void printProfile() const;
    std::string toCSV() const;
};
