#pragma once

template <typename T>
struct GeometricPair
{
	GeometricPair()
	{
	}

	GeometricPair(T l, T r)
	{
		left = l;
		right = r;
	}

	T left = 0;
	T right = 0;
};
