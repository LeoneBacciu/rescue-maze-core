#pragma once
#include <exception>

struct StopConnection final : public std::exception
{
	virtual const char* what() const throw () override
	{
		return "C++ Exception";
	}
};
