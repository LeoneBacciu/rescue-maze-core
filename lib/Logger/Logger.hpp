#pragma once


#include <string>
#include <sstream>
#if _EXECUTION_ENVIRONMENT == 0
#else

#include <Arduino.h>
#include <cstdarg>
#include <cstdio>

#endif

enum Source : uint8_t
{
	kBrick,
	kCompass,
	kDrivers,
	kFloor,
	kGyro,
	kLasers,
	kSerial,
	kTemp,
    kGeneric,
};

enum Verbosity : uint8_t
{
	kVerbose,
	kInfo,
	kWarn,
	kError
};

class Logger
{
public:

    static void SetBus(HardwareSerial* hardwareSerial);

	static void SetVerbosity(Verbosity verbosity);

	static void AllowSource(Source source);

	static void DenySource(Source source);

	static void AllowAllSources();

	static void DenyAllSources();

	static void Verbose(Source source, const char* format, ...);

	static void Info(Source source, const char* format, ...);

	static void Warn(Source source, const char* format, ...);

	static void Error(Source source, const char* format, ...);

private:
    static HardwareSerial* serial;
	static char const* source_to_string_[];
	static bool allow_list_[];
	static Verbosity verbosity_;

};

template<typename T> void ToCharArray(T* in, char* out, const uint16_t size)
{
    for (int i = 0; i < size; ++i) {
        if (i != 0) out[3 * i - 1] = ':';
        sprintf(out + 3 * i, "%02X", in[i]);
    }
    out[size*3]= '\0';
//	static const char* digits = "0123456789ABCDEF";
//	for (uint16_t l = 0; l < size; ++l)
//	{
//		if (l != 0) out[3 * l - 1] = ':';
//		for (uint8_t i = 0, j = 4; i < 2; ++i, j -= 4)
//			out[i + 3 * l] = digits[in[l] >> j & 0x0f];
//	}
}
