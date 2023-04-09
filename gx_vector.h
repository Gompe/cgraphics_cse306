#pragma once

#include <ostream>
#include <vector>
#include <algorithm>
#include <cmath>

// Vector Declaration
class Vector{
public:
    Vector();
    explicit Vector(double x,double y, double z);

    double operator[](int i) const;
    double& operator[](int i);

    // Inplace operators
    Vector& operator+=(const Vector& rhs);
    Vector& operator-=(const Vector& rhs);
    Vector& operator*=(const Vector& rhs);
    Vector& operator*=(double rhs);
    Vector& operator/=(const Vector& rhs);
    Vector& operator/=(double rhs);
    Vector& operator=(const Vector& rhs);

    double norm2() const;
    double norm() const;
    Vector normalized() const;
    Vector abs() const;
    Vector clip(double min_value, double max_value) const;
    size_t argmin() const;
    size_t argmax() const;
    Vector orthogonal() const;

private:
    double coords[3];
};

// Not inplace operators
Vector operator+(const Vector& a, const Vector& b);
Vector operator-(const Vector& a, const Vector& b);
Vector operator*(const Vector& a, const Vector& b);
Vector operator*(const double a, const Vector& b);
Vector operator*(const Vector& a, const double b);
Vector operator/(const Vector& a, const Vector& b);
Vector operator/(const Vector& a, const double b);
Vector operator-(const Vector& a);

double dot(const Vector& a, const Vector& b);
double cosine_similarity(const Vector& a, const Vector& b);
Vector cross(const Vector& a, const Vector& b);

std::ostream& operator<<(std::ostream& os, const Vector& rhs);

// Ray Declaration
class Ray
{
public:
    Ray(const Vector&O, const Vector& u);
    Vector O;
    Vector u;
};