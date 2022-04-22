#pragma once

#if _EXECUTION_ENVIRONMENT != 0
#include <cstdint>
#endif

namespace dimensions
{
    const uint8_t depth_tolerance = 150;
	const uint8_t width = 110;
	const uint8_t height = 120;
	const uint8_t depth = 190;
	const uint8_t front_lasers_distance = 65;
}

namespace movements {
    const uint8_t touch = 30;
    const uint8_t not_touch = 30;
    const uint8_t arrived = 60;
    const uint8_t really_near = 80;
    const uint8_t near = 100;
    const uint8_t unsafe_wall = 250;
}

namespace cell_dimensions
{
	const uint16_t depth = 320;
    const uint16_t depth_tolerant = 200;
}

namespace communication
{
	const int message_length = 16;
	const int flags_offset = 4;
}
