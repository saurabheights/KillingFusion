#pragma once

#include "Vec.h"
#include "Quat.h"

struct Transform {
	Vec3f translation = Vec3f(0);
	Quat orientation = Quat_Identity();

	Transform() = default;
	Transform(const Vec3f &translation, const Quat &orientation):
		translation(translation), orientation(orientation)
	{
	}

	explicit Transform(const Quat &orientation): orientation(orientation) {}
	explicit Transform(const Vec3f &translation): translation(translation) {}
};

Transform inverse(const Transform &tf);
Mat4 to_mat4(const Transform &tf);

Vec3f transform(const Vec3f &in, const Transform &tr);
