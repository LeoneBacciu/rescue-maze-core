// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#include "OutputMessage.hpp"
#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/utils/Constants.hxx"
#else
#include <utils/Constants.hxx>
#endif

OutputMessage::OutputMessage(Walls walls, bool black, bool checkpoint, bool ramp, bool obstacle)
    : walls(&walls),
      black(black),
      checkpoint(checkpoint),
      ramp(ramp),
      obstacle(obstacle)
{
}

char* OutputMessage::toBinary() const
{
    static char out[communication::message_length] = {};
    const int o = communication::flags_offset;
    copyTuple4(out, *walls);
    if (black) out[o + 0] = 'B';
    if (checkpoint) out[o + 1] = 'C';
    if (ramp) out[o + 2] = 'R';
    if (obstacle) out[o + 3] = 'O';
    return out;
}
