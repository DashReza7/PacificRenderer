#pragma once

class Vec3
{
public:
	Float x, y, z;

	// Constructors
	Vec3() : x(0), y(0), z(0) {}
	Vec3(Float x, Float y, Float z) : x(x), y(y), z(z) {}

	// Access operators
	Float& operator[](int i)
	{
		return i == 0 ? x : (i == 1 ? y : z);
	}

	const Float& operator[](int i) const
	{
		return i == 0 ? x : (i == 1 ? y : z);
	}

	// Unary minus
	Vec3 operator-() const {
		return Vec3(-x, -y, -z);
	}

	// Arithmetic operators
	Vec3& operator+=(const Vec3& v) {
		x += v.x; y += v.y; z += v.z;
		return *this;
	}

	Vec3& operator-=(const Vec3& v) {
		x -= v.x; y -= v.y; z -= v.z;
		return *this;
	}

	Vec3& operator*=(Float s) {
		x *= s; y *= s; z *= s;
		return *this;
	}

	Vec3& operator/=(Float s) {
		Float inv = 1 / s;
		x *= inv; y *= inv; z *= inv;
		return *this;
	}
};


