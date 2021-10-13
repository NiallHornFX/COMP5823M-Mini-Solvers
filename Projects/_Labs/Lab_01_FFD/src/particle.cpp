#include "particle.h"

#include "spring.h"

particle::particle(const vec3<real> &P, const vec3<std::size_t> &IDX3D, const std::size_t IDX1D, const p_state State)
	: p(P), idx_3d(IDX3D), idx_1d(IDX1D), state(State) {}


void particle::calc_normal(const particle &p1, const particle &p2)
{
	// P1(idx +/- 1) P2(idx +/- pt_N)
	vec3<real> u = p1.p - p; vec3<real> v = p2.p - p;
	n = vec3<real>::cross(u, v).normalize();
}



