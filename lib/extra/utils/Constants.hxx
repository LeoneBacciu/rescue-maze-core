#pragma once

#if _EXECUTION_ENVIRONMENT != 0
#include <cstdint>
#endif

namespace dimensions
{
	const uint8_t width = 110;
	const uint8_t height = 120;
	const uint8_t depth = 200;
	const uint8_t front_lasers_distance = 65;
}

namespace movements {
    const uint8_t touch = 20;
    const uint8_t arrived = 50;
    const uint8_t really_near = 80;
    const uint8_t near = 120;
}

namespace cell_dimensions
{
	const uint16_t depth = 300;
}

namespace communication
{
	const int message_length = 16;
	const int flags_offset = 4;
}
