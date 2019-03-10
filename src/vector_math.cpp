#include "vector_math.hh"


double vec_norm(double &p) {
	x = *(p);
	y = *(p+1);
    return math::sqrt(x*x+y*y)
}

double* vec_sub(double &p1, double &p2) {
	p_new = {*(p1) - *(p2),*(p1+1)-*(p2+1)};
	return p_new;
}

double* vec_add(double &p1, double &p2) {
	p_new = {*(p1) + *(p2),*(p1+1)+*(p2+1)};
	return p_new;
}

double* vec_dot(double &p1, double &p2) {
	p_new = {*(p1) * *(p2),*(p1+1) * *(p2+1)};
	return p_new;
}