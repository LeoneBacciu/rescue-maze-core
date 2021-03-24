// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once

class IMessage
{
public:
    virtual ~IMessage() = default;
    virtual char* toBinary() const = 0;
};
