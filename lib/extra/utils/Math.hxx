#pragma once

namespace math
{
	template <typename T>
	T Clamp(T x, T min, T max)
	{
		return x < min ? min : x < max ? x : max;
	}

	template <typename T>
	T AngleDifference(T a, T b)
	{
		float difference = a - b;
		if (difference > 180) difference -= 360;
		else if (difference < -180) difference += 360;
		return difference;
	}

	inline float Fmod(const float x, const float y)
	{
		const float abs_y = fabsf(y);
		if (abs_y <= 1.e-8f)
		{
			return 0.f;
		}
		const float div = x / y;
		const float quotient = fabsf(div) < 8388608.f ? truncf(div) : div;
		float int_portion = y * quotient;

		if (fabsf(int_portion) > fabsf(x))
		{
			int_portion = x;
		}

		const float result = x - int_portion;
		return Clamp(result, -abs_y, abs_y);
	}

	inline float ClampAngle(float angle)
	{
		angle = Fmod(angle, 360.f);

		if (angle < 0.f)
		{
			angle += 360.f;
		}

		return angle;
	}
}
