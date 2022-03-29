#include "Logger.hpp"

char const *Logger::source_to_string_[] = {"brick", "compass", "drivers", "floor", "gyro", "lasers", "serial2", "temp", "generic"};
bool Logger::allow_list_[] = {false, false, false, false, false, false, false, false};
Verbosity Logger::verbosity_ = kInfo;
HardwareSerial* Logger::serial;

void Logger::SetVerbosity(Verbosity verbosity) {
    verbosity_ = verbosity;
}

void Logger::AllowSource(const Source source) {
    allow_list_[source] = true;
}

void Logger::DenySource(const Source source) {
    allow_list_[source] = false;
}

void Logger::AllowAllSources() {
    for (bool &i: allow_list_) i = true;
}

void Logger::DenyAllSources() {
    for (bool &i: allow_list_) i = false;
}

void Logger::Verbose(Source source, const char *format, ...) {
    if (verbosity_ > kVerbose || !allow_list_[source]) return;
    va_list args;
    va_start(args, format);
    Print("verbose", source, format, args);
    va_end(args);
}

void Logger::Info(Source source, const char *format, ...) {
    if (verbosity_ > kInfo || !allow_list_[source]) return;
    va_list args;
    va_start(args, format);
    Print("info", source, format, args);
    va_end(args);
}

void Logger::Warn(Source source, const char *format, ...) {
    if (verbosity_ > kWarn || !allow_list_[source]) return;
    va_list args;
    va_start(args, format);
    Print("warn", source, format, args);
    va_end(args);
}

void Logger::Error(Source source, const char *format, ...) {
    if (verbosity_ > kError || !allow_list_[source]) return;
    va_list args;
    va_start(args, format);
    Print("error", source, format, args);
    va_end(args);
}

#if _EXECUTION_ENVIRONMENT == 0
void Logger::Print(const char *verb, Source source, const char *format, va_list args) {
    char buffer[128];
    vsprintf_s(buffer, format, args);
    UE_LOG(LogTemp, Warning, TEXT("[%s] - [%s] - {%s}"), *FString(verb), *FString(source_to_string_[source]), *FString(buffer));
}
#else

void Logger::Print(const char *verb, Source source, const char *format, va_list args) {
    char buffer[128];
    vsprintf(buffer, format, args);
    serial->printf("[%s] - [%s] - {%s}\n", verb, source_to_string_[source], buffer);
}

void Logger::SetBus(HardwareSerial *hardwareSerial) {
    serial = hardwareSerial;
}

#endif
