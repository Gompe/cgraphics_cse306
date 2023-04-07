#include "gx_vector.h"

/* Constructors */
Vector::Vector(){
    coords[0] = 0.;
    coords[1] = 0.;
    coords[2] = 0.;
}
Vector::Vector(double x,double y, double z){
    coords[0] = x;
    coords[1] = y;
    coords[2] = z;
}

/* Getters and Setters*/
double Vector::operator[](int i) const {
    return coords[i];
}

double& Vector::operator[](int i){
    return coords[i];
}

/* Inplace Operators */
Vector& Vector::operator+=(const Vector& rhs){
    for(int i=0; i<3; i++)
        coords[i] += rhs[i];
    return *this;
}
Vector& Vector::operator-=(const Vector& rhs){
    for(int i=0; i<3; i++)
        coords[i] -= rhs[i];
    return *this;
}
Vector& Vector::operator*=(const Vector& rhs){
    for(int i=0; i<3; i++)
        coords[i] *= rhs[i];
    return *this;
}
Vector& Vector::operator*=(double rhs){
    for(int i=0; i<3; i++)
        coords[i] *= rhs;
    return *this;
}
Vector& Vector::operator/=(const Vector& rhs){
    for(int i=0; i<3; i++)
        coords[i] /= rhs[i];
    return *this;
}
Vector& Vector::operator/=(double rhs){
    for(int i=0; i<3; i++)
        coords[i] /= rhs;
    return *this;
}
Vector& Vector::operator=(const Vector& rhs){
    for(int i=0; i<3; i++)
        coords[i] = rhs[i];
    return *this;
}

/* Not inplace operators */
Vector operator+(const Vector& a, const Vector& b){
    return Vector(a[0]+b[0], a[1]+b[1], a[2]+b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
	return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(const Vector& a, const Vector& b){
	/* coordinate-wise product */
	return Vector(a[0]*b[0], a[1]*b[1], a[2]*b[2]);
}
Vector operator*(const double a, const Vector& b) {
	return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const double b) {
	return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator/(const Vector& a, const Vector& b){
    return Vector(a[0]/b[0], a[1]/b[1], a[2]/b[2]);
}
Vector operator/(const Vector& a, const double b) {
	return Vector(a[0] / b, a[1] / b, a[2] / b);
}
Vector operator-(const Vector& a) {
	return Vector(-a[0], -a[1], -a[2]);
}

double dot(const Vector& a, const Vector& b){
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
double cosine_similarity(const Vector& a, const Vector& b){
	double prod_norm = a.norm() * b.norm();
	double dot_prod = dot(a, b);
	return dot_prod/prod_norm;
}
Vector cross(const Vector& a, const Vector& b) {
	return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

/* Utils */
// Norm
double Vector::norm2() const{
    return (
        coords[0] * coords[0] + 
        coords[1] * coords[1] + 
        coords[2] * coords[2]
    );
}
double Vector::norm() const{
    return sqrt(norm2());
}
Vector Vector::normalized() const{
    return *this/norm();
}

// Other operators
Vector Vector::abs() const{
    return Vector(
        std::abs(coords[0]),
        std::abs(coords[1]),
        std::abs(coords[2])
    );
}
Vector Vector::clip(double min_value, double max_value) const{
    Vector output = *this;
    for(int i=0; i<3; i++)
        output[i] = std::max(std::min(output[i], max_value), min_value);
    return output;
}

// Coordinates
size_t Vector::argmin() const{
    if(coords[0] <= coords[1] && coords[0] <= coords[2])
        return 0;
    else if(coords[1] <= coords[2])
        return 1;
    return 2;
}
size_t Vector::argmax() const{
    if(coords[0] >= coords[1] && coords[0] >= coords[2])
        return 0;
    else if(coords[1] >= coords[2])
        return 1;
    return 2;
}

// Other
Vector Vector::orthogonal() const{
    Vector x = *this;
    size_t i = x.abs().argmin();
    x[i] = 0.;
	if(i==0){
		x[1] = -x[1];
		std::swap(x[1], x[2]);
	} 
	else if(i==1){
		x[2] = -x[2];
		std::swap(x[2], x[0]);
	}
	else{
		x[0] = -x[0];
		std::swap(x[0], x[1]);
	}
	return x;
}

/* Print */
std::ostream& operator<<(std::ostream& os, const Vector& rhs){
    os << "(" << rhs[0] << "," << rhs[1] << "," << rhs[2] << ")";
    return os;
}