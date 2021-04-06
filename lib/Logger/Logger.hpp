#pragma once


#if _EXECUTION_ENVIRONMENT == 0
#else

#include <Arduino.h>
#include <cstdarg>
#include <cstdio>

#endif

enum Source : uint8_t {
    kBrick,
    kCompass,
    kDrivers,
    kFloor,
    kGyro,
    kLasers,
    kSerial,
    kTemp
};

enum Verbosity : uint8_t {
    kVerbose,
    kInfo,
    kWarn,
    kError,
};

class Logger {
public:

    static void SetVerbosity(Verbosity verbosity);

    static void AllowSource(Source source);

    static void DenySource(Source source);

    static void AllowAllSources();

    static void DenyAllSources();

    static void Verbose(Source source, const char *format, ...);

    static void Info(Source source, const char *format, ...);

    static void Warn(Source source, const char *format, ...);

    static void Error(Source source, const char *format, ...);

private:
    static char const *source_to_string_[];
    static bool allow_list_[];
    static Verbosity verbosity_;

    static void Print(const char *verb, Source source, const char *format, va_list args);
};
