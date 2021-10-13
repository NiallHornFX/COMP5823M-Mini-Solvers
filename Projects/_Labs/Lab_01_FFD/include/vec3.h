// Niall Horn - 2020

#ifndef VEC3_H
#define VEC3_H

#include <cmath>
#include <iostream>
#include <type_traits>

#include <immintrin.h>

#define USE_SSE 0 // Use Vectorized SSE Intrinsics where implemented. 
#define VEC3_INLINE __forceinline 

extern const double PI; 

// vec3<T> 3D Vector class with Math Operations. Can be accesed as component wise values of <T>, a Contigous <T> Array[3]  
// or as a 4 byte wise SSE Type __m128 for internal and external simd operations. 
// Deg2Rad and Other util functions return hardcoded single precison floats, T Not used, not relying on T been Floating Point. 


template <class T>
class vec3
{
public:
	vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}
	explicit vec3(T xx) : x(xx), y(xx), z(xx) {}
	explicit vec3(T *arr) { x = arr[0], y = arr[1], z = arr[2]; }
	explicit vec3(__m128 vv) : sa(vv) {}
	vec3() : x((T) 0), y((T) 0), z((T) 0) {}
	~vec3() {}

	// Copy 
	vec3(const vec3 &copy) : x(copy.x), y(copy.y), z(copy.z) {}
	vec3& operator= (const vec3 &copy)
	{
		if (&copy == this) return *this; 
		this->x = copy.x, this->y = copy.y, this->z = copy.z;
		return *this; 
	}

	// UD Op Casts.
	operator vec3<double>() { return vec3<double>((double)x, (double)y, (double)z); }
	operator vec3<float>() { return vec3<float>((float)x, (float)y, (float)z); }
	operator __m128() { return sa; }

	// vec3 Vector Math Operations, Return Scalar. 
	VEC3_INLINE float length() const;
	VEC3_INLINE float dot(const vec3 &b) const;
	VEC3_INLINE float angle(const vec3 &b) const;
	VEC3_INLINE vec3<T> squared() const;

	// vec3 Vector Math Operations, Return modified *this. 
	VEC3_INLINE vec3& normalize();
	vec3& clear(); 

	// Sign Switching
	VEC3_INLINE vec3 operator-() const { return vec3<T>(-this->x, -this->y, -this->z); }

	// LHS vec3 Arithmetic Overloads - Return new vec3 with operation of this and B vec3 completed. Do Not Modifiy this.
	VEC3_INLINE vec3 operator+ (const vec3 &b) const;
	VEC3_INLINE vec3 operator- (const vec3 &b) const;
	VEC3_INLINE vec3 operator/ (T scalar) const;
	VEC3_INLINE float operator*(const vec3 &b) const; // (dot). 

	// LHS Operator Overloads vec3 - Modifiy this return *this. 
	VEC3_INLINE vec3& operator+= (T scalar);
	VEC3_INLINE vec3& operator+= (const vec3 &b);
	VEC3_INLINE vec3& operator-= (T scalar);
	VEC3_INLINE vec3& operator-= (const vec3 &b);
	VEC3_INLINE vec3& operator/= (T scalar);
	VEC3_INLINE vec3& operator/= (const vec3 &b);
	VEC3_INLINE vec3& operator*= (T scalar);

	// Cross Product Return new vec3<T> c = a x b
	static VEC3_INLINE vec3 cross(const vec3 &a, const vec3 &b);

	// Angle Conversion
	static VEC3_INLINE float degtoRad(float deg);
	static VEC3_INLINE float radtoDeg(float rad);

	// Multi-Acess Layout (12-(16) Byte)
	union
	{
		struct { T x, y, z; };
		T va[3];
		__m128 sa;
	};
	
};

// vec3<T> Implementation \\

// _vec3<T> Math Operations \\

// length = Sqrt((this->x * this->x) + (this->y * this->y) + (this->z * this->z))
template <class T>
float vec3<T>::length() const
{
#if USE_SSE == 1
	// SSE Length - 
	__m128 a = _mm_mul_ps(this->sa, this->sa);
	__m128 t1 = _mm_hadd_ps(a, a); // Reduction and Sqrt.
	return _mm_cvtss_f32((_mm_sqrt_ps(_mm_hadd_ps(t1, t1))));
#endif
	return std::sqrtf(std::powf(x, 2.0) + std::powf(y, 2.0) + std::powf(z, 2.0));
}

// vec3<T> Dot Product 
// dot = (this->x * b.x) + (this->y * b.y) + (this->z * b.z)
template <class T>
float vec3<T>::dot(const vec3<T> &b) const
{
#if USE_SSE == 1
	// SSE Dot. 
	__m128 a = _mm_mul_ps(this->sa, this->sa);
	__m128 t1 = _mm_hadd_ps(a, a); 
	return _mm_cvtss_f32( _mm_hadd_ps(t1, t1) );
#endif

	return (float)((x * b.x) + (y * b.y) + (z * b.z));
}
// dot v1*v2 overload = dot = (this->x * b.x) + (this->y * b.y) + (this->z * b.y)
template <class T>
float vec3<T>::operator*(const vec3<T> &b) const
{
	dot(b);
}

// vec3<T> Angle - Angle Between this & b (Radians) 
// theta = acos(dot(normalize(this), normalize(b)))
template <class T>
float vec3<T>::angle(const vec3<T> &b) const
{
	vec3<T> v1 = *this, v2 = b; 
	float theta = std::acosf((v1.normalize()).dot(v2.normalize())); 
	return theta;
}

// vec3<T> Squared 
// this --> (x*x, y*y, z*z)
template <class T>
vec3<T> vec3<T>::squared() const
{
	return vec3<T>(x*x, y*y, z*z);
}

// vec3<T> Normalize 
// this / ||this|| --> *this
template <class T>
vec3<T>& vec3<T>::normalize()
{
	float l = length(); if (l == 0) { x = 0.0f, y = 0.0f, z = 0.0f; return *this; }
	float l_r = 1.0f / l; 
	/* 
	#if USE_SSE == 1
	//this->sa = _mm_div_ps(this->sa, vec3<float>(l).sa);
	this->sa = _mm_mul_ps(this->sa, vec3<float>(l_r).sa);
	return *this; 
	#endif
	*/
	x = x * l_r; y = y * l_r; z = z * l_r;
	return *this; 
}

// vec3<T> Clear
// this->x = 0, this->y = 0, this->z = 0 --> *this
template <class T>
vec3<T>& vec3<T>::clear()
{
	x = (T) 0, y = (T) 0, z = (T) 0;
	return *this; 
}

// _LHS vec3<T> Operand overloads \\

// Vector Additon and Subtraction with LHS and RHS Vector. Return new resulting vec3<T>. 
// v3 = this + v2
template <class T>
vec3<T> vec3<T>::operator+(const vec3<T> &b) const
{
#if USE_SSE == 1
	return vec3<float>(_mm_add_ps(this->sa, b.sa)); 
#endif

	// non SSE (Component Wise) additon - 
	T n_x = x + b.x;
	T n_y = y + b.y;
	T n_z = z + b.z;
	return vec3<T>(n_x, n_y, n_z);
}

// v3 = v2 - this
template <class T>
vec3<T> vec3<T>::operator-(const vec3<T> &b) const
{
#if USE_SSE == 1
	// SSE vec3 subtraction - 
	return vec3<float>(_mm_sub_ps(this->sa, b.sa));
#endif

	// non SSE (Component Wise) subtraction -
	T n_x = x - b.x;
	T n_y = y - b.y;
	T n_z = z - b.z;
	return vec3<T>(n_x, n_y, n_z);
}

// v3 = this / scalar 
template <class T>
vec3<T> vec3<T>::operator/ (T scalar) const
{

	/*
	#if USE_SSE == 1
	return vec3<float>(_mm_div_ps(this->sa, vec3<float>(scalar).sa)); // 4th el 0/0 ?? 
	#endif */

	// non SSE (Component Wise) Division
	vec3<T> t = *this; 
	return vec3<T>(t.x / scalar, t.y / scalar, t.z / scalar); 
}

// _LHS vec3<T> RHS <T> Scalar,vec3<T> --> Return *this modified vec3<T> \\

// vec3<T> Addition by Scalar or vec3<T>
// this->x += scalar, this->y += scalar, this->z += scalar ---> *this
template <class T>
vec3<T>& vec3<T>::operator+=(T scalar)
{
	x += scalar, y += scalar, z += scalar;
	return *this;
}
template <class T>
vec3<T>& vec3<T>::operator+=(const vec3<T> &b)
{
#if USE_SSE
	*this = vec3<T>(_mm_add_ps(this->sa, b.sa)); 
	return *this; 
#endif
	x += b.x, y += b.y, z += b.z;
	return *this;
}

// vec3<T> Subtraction by Scalar or vec3<T> 
// this->x -= scalar, this->y -= scalar, this->z -= scalar ---> *this
template <class T>
vec3<T>& vec3<T>::operator-=(T scalar)
{
	x -= scalar, y -= scalar, z -= scalar;
	return *this;
}
template <class T>
vec3<T>& vec3<T>::operator-=(const vec3<T> &b)
{
#if USE_SSE
	*this = vec3<T>(_mm_sub_ps(this->sa, b.sa));
	return *this;
#endif
	x -= b.x, y -= b.y, z -= b.z;
	return *this;
}

// vec3<T> Multiplicaton by Scalar
// this->x *= scalar, this->y *= scalar, this->z *= scalar ---> *this
template <class T>
vec3<T>& vec3<T>::operator*=(T scalar)
{
	x *= scalar, y *= scalar, z *= scalar;
	return *this;
}

// vec3<T> Division by Scalar or vec3<T> 
// this->x /= scalar, this->y /= scalar, this->z /= scalar ---> *this
template <class T>
vec3<T>& vec3<T>::operator/=(T scalar)
{
	x /= scalar, y /= scalar, z/= scalar;
	return *this;
}
template <class T>
vec3<T>& vec3<T>::operator/=(const vec3<T> &b)
{
	x /= b.x, y /= b.y, z /= b.z;
	return *this;
}

// Static Cross Product --> c = a x b
template <class T>
vec3<T> vec3<T>::cross(const vec3<T> &a, const vec3<T> &b)
{
#if USE_SSE == 1
	// SSE Cross (old) - 
	/*__m128 a_0 = _mm_shuffle_ps(a.sa, a.sa, _MM_SHUFFLE(3, 0, 2, 1));
	__m128 a_1 = _mm_shuffle_ps(a.sa, a.sa, _MM_SHUFFLE(3, 1, 0, 2));
	__m128 b_0 = _mm_shuffle_ps(b.sa, b.sa, _MM_SHUFFLE(3, 1, 0, 2));
	__m128 b_1 = _mm_shuffle_ps(b.sa, b.sa, _MM_SHUFFLE(3, 0, 2, 1));
	return vec3<T>(_mm_sub_ps(_mm_mul_ps(a_0, b_0), _mm_mul_ps(a_1, b_1)));*/

	// SSE Cross - 
	__m128 tmp0 = _mm_shuffle_ps(b.sa, b.sa, _MM_SHUFFLE(3, 0, 2, 1));
	__m128 tmp1 = _mm_shuffle_ps(a.sa, a.sa, _MM_SHUFFLE(3, 0, 2, 1));
	tmp0 = _mm_mul_ps(tmp0, a.sa);
	tmp1 = _mm_mul_ps(tmp1, b.sa);
	__m128 tmp2 = _mm_sub_ps(tmp0, tmp1);
	return vec3<T>(_mm_shuffle_ps(tmp2, tmp2, _MM_SHUFFLE(3, 0, 2, 1)));
#endif 
	float cx = (a.y * b.z) - (a.z * b.y);
	float cy = (a.z * b.x) - (a.x * b.z);
	float cz = (a.x * b.y) - (a.y * b.x);
	return vec3<T>(cx, cy, cz);
}

// Static vec3<T> (Utility) Member Functions \\

// Degrees to Radians
template <class T>
float vec3<T>::degtoRad(float deg)
{
	return (float)deg * (PI / 180.0f);
}

// Radians to Degrees 
template <class T>
float vec3<T>::radtoDeg(float rad)
{
	return (float)rad * (180.0f / PI);
}

// _RHS vec3<T> Operand Global Operator Overload Free Functions \\ 

// Assume Scalar can be a diffrent type, aslong as it is floating_point. 

// s1 * v1 ---> v1;
template <typename F, class T>
VEC3_INLINE vec3<T> operator* (const F mult, const vec3<T> &vec)
{
	if (!std::is_floating_point<F>::value) return vec; 
	vec3<T> tmp = vec; 
	return vec3<T>(tmp.x *= mult, tmp.y *= mult, tmp.z *= mult);
}

// s1 + v1 ---> v1;
template <typename F, class T>
VEC3_INLINE vec3<T> operator+ (const F add, const vec3<T> &vec)
{
	if (!std::is_floating_point<F>::value) return vec;
	vec3<T> tmp = vec;
	return vec3<T>(tmp.x + add, tmp.y + add, tmp.z + add); 
}

#endif