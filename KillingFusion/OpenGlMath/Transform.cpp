#include "Transform.h"

Mat4 to_mat4(const Transform &tf)
{
	return to_mat4(tf.orientation) * Mat4_Translate(tf.translation);
}

Transform inverse(const Transform &tf)
{
	return {-tf.translation, inverse(tf.orientation)};
}

Vec3f transform(const Vec3f &in, const Transform &tr)
{
	return tr.translation + tr.orientation.rotate(in);
}
