#pragma once

enum Source
{
	kBrick,
	kCompass,
	kDrivers,
	kFloor,
	kGyro,
	kLasers,
	kSerial,
	kTemp
};

class Logger
{
public:

	static void AllowSource(const Source source);
	static void DenySource(const Source source);
	static void AllowAllSources();
	static void DenyAllSources();
	
	static void Info(const Source source, const char* format, ...);
	static void Warn(const Source source, const char* format, ...);
	static void Error(const Source source, const char* format, ...);

private:
	static char* source_to_string_[];
	static bool allow_list_[];
};
