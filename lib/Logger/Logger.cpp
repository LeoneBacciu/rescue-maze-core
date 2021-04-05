#include "Logger.hpp"

char* Logger::source_to_string_[] = {"brick", "compass", "drivers", "floor", "gyro", "lasers", "serial", "temp"};
bool Logger::allow_list_[] = {false, false, false, false, false, false, false, false};

void Logger::AllowSource(const Source source)
{
	allow_list_[source] = true;
}

void Logger::DenySource(const Source source)
{
	allow_list_[source] = false;
}

void Logger::AllowAllSources()
{
	for (int i = 0; i < sizeof allow_list_ / sizeof allow_list_[0]; ++i)
	{
		allow_list_[i] = true;
	}
}

void Logger::DenyAllSources()
{
	for (int i = 0; i < sizeof allow_list_ / sizeof allow_list_[0]; ++i)
	{
		allow_list_[i] = false;
	}
}

void Logger::Info(const Source source, const char* format, ...)
{
	if (!allow_list_[source]) return;

	char buffer[256];
	va_list args;
	va_start(args, format);
	vsprintf_s(buffer, format, args);
	va_end(args);
	UE_LOG(LogTemp, Display, TEXT("[info] - [%s] - {%s}"), *FString(source_to_string_[source]), *FString(buffer));
}

void Logger::Warn(const Source source, const char* format, ...)
{
	if (!allow_list_[source]) return;

	char buffer[256];
	va_list args;
	va_start(args, format);
	vsprintf_s(buffer, format, args);
	va_end(args);
	UE_LOG(LogTemp, Warning, TEXT("[warn] - [%s] - {%s}"), *FString(source_to_string_[source]), *FString(buffer));
}

void Logger::Error(const Source source, const char* format, ...)
{
	if (!allow_list_[source]) return;

	char buffer[256];
	va_list args;
	va_start(args, format);
	vsprintf_s(buffer, format, args);
	va_end(args);
	UE_LOG(LogTemp, Error, TEXT("[error] - [%s] - {%s}"), *FString(source_to_string_[source]), *FString(buffer));
}
