// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once

#if _EXECUTION_ENVIRONMENT == 0
#include <tuple>
#else
#include "../../../../../.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/9.2.1/tuple"
#endif

typedef const std::tuple<char, char, char, char> Directions;
typedef const std::tuple<char, char, char, char> Walls;

namespace VDirection
{
	Directions const RIGHT = std::make_tuple(0x01, 0, 0, 0);
	Directions const TOP = std::make_tuple(0, 0x01, 0, 0);
	Directions const LEFT = std::make_tuple(0, 0, 0x01, 0);
	Directions const BOTTOM = std::make_tuple(0, 0, 0, 0x01);
};

inline void copyTuple4(char* arr, Directions direction, int offset = 0)
{
	arr[offset + 0] = std::get<0>(direction);
	arr[offset + 1] = std::get<1>(direction);
	arr[offset + 2] = std::get<2>(direction);
	arr[offset + 3] = std::get<3>(direction);
}

enum direction : int
{
	kRight = 0,
	kTop = 1,
	kLeft = 2,
	kBottom = 3
};

inline direction DirectionFromInt(const int p)
{
	switch (p)
	{
	case 0:
		return kRight;
	case 1:
		return kTop;
	case 2:
		return kLeft;
	default:
		return kBottom;
	}
}
