#ifndef NAV_TYPES_H
#define NAV_TYPES_H

namespace NavConstants
{
	constexpr float kExtX = 3.0f;
	constexpr float kExtY = 20.0f;
	constexpr float kExtZ = 3.0f;
	constexpr float kLosMaxRange = 60.0f;
	constexpr float kLosStepRange = 40.0f;
}

struct XYZ
{
	float X;
	float Y;
	float Z;

	XYZ() : X(0.0f), Y(0.0f), Z(0.0f) {}

	XYZ(double x, double y, double z)
	{
		X = static_cast<float>(x);
		Y = static_cast<float>(y);
		Z = static_cast<float>(z);
	}
};

#endif
