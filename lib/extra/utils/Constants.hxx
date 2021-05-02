#pragma once

#if _EXECUTION_ENVIRONMENT != 0
#include <cstdint>
#endif

namespace dimensions
{
	const uint8_t width = 130;
	const uint8_t height = 120;
	const uint8_t depth = 200;
	const uint8_t front_lasers_distance = 90;
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
