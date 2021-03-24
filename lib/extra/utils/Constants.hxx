#pragma once


namespace dimensions
{
	const uint8_t width = 15;
	const uint8_t height = 12;
	const uint8_t depth = 20;
	const uint8_t front_lasers_distance = 10;
}

namespace cell_dimensions
{
	const uint8_t depth = 30;
}

namespace communication
{
	const int message_length = 16;
	const int flags_offset = 4;
}
