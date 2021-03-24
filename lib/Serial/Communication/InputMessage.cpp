// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#include "InputMessage.hpp"
#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/extra/utils/Constants.hxx"
#else
#include <utils/Constants.hxx>
#endif

InputMessage::InputMessage(Directions direction, bool drop, bool climb, bool old)
    : direction(&direction),
      drop(drop),
      climb(climb),
      old(old)
{
}

InputMessage::InputMessage(char* data)
{
    Directions in_direction = std::make_tuple(data[0], data[1], data[2], data[3]);
    this->direction = &in_direction;
    const int o = communication::flags_offset;
    drop = climb = old = false;
    for (int i = o; i < communication::message_length; ++i)
    {
        if (data[i] == 'D') drop = true;
        if (data[i] == 'C') climb = true;
        if (data[i] == 'O') old = true;
    }
}

char* InputMessage::toBinary() const
{
    static char out[communication::message_length] = {};
    const int o = communication::flags_offset;
    copyTuple4(out, *direction);
    if (drop) out[o + 0] = 'D';
    if (climb) out[o + 1] = 'C';
    if (old) out[o + 2] = 'O';
    return out;
}
