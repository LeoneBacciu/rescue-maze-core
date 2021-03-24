// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once
#include "Directions.hxx"
#include "IMessage.hh"

class OutputMessage : public IMessage
{
    Walls* walls;
    bool black;
    bool checkpoint;
    bool ramp;
    bool obstacle;
public:

    OutputMessage(Walls walls, bool black, bool checkpoint, bool ramp, bool obstacle);

    char* toBinary() const override;
};
