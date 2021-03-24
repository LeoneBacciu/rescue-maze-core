// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once
#include "Directions.hxx"
#include "IMessage.hh"

class InputMessage final : public IMessage
{
    Directions* direction;
    bool drop;
    bool climb;
    bool old;
public:
    InputMessage(Directions direction, bool drop, bool climb, bool old);

    explicit InputMessage(char* data);

    char* toBinary() const override;
};
