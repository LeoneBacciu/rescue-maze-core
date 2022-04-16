#include "Envelope.hpp"

InputEnvelope *InputEnvelope::FromBytes(uint8_t *data) {
    return new InputEnvelope(DirectionFromBytes(&data[1]), data[5] != 0, data[6]);
}

void OutputEnvelope::ToBytes(uint8_t *bytes) const {
    bytes[0] = 0xfe;
    walls->ToBytes(&bytes[1]);
    bytes[5] = black;
    bytes[6] = checkpoint;
    bytes[7] = sides;
    bytes[8] = 0xff;
}
