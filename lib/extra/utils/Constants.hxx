#pragma once



#if  _EXECUTION_ENVIRONMENT == 0
typedef uint16 uint16_t;
#endif

namespace dimensions
{
    const float width = 15;
    const float height = 12;
    const float depth = 20;
    const float front_lasers_distance = 10;
}

namespace cell_dimensions
{
    const int depth = 30;
}

namespace communication
{
    const int message_length = 16;
    const int flags_offset = 4;
}
