#pragma once

#include "Directions.hxx"
#include "Walls.hxx"

class InputEnvelope {
public:
    const Direction direction;
    const bool ignore;
    const uint8_t drop;

    InputEnvelope(const Direction direction, const bool ignore, const uint8_t drop)
            : direction(direction), ignore(ignore), drop(drop) {
    }

    static InputEnvelope *FromBytes(uint8_t *data);
};

class OutputEnvelope {
public:
    const Walls *walls;
    const bool black;
    const bool checkpoint;
    const uint8_t sides;

    OutputEnvelope(const Walls *walls, const bool black, const bool checkpoint, const uint8_t sides)
            : walls(walls), black(black), checkpoint(checkpoint), sides(sides) {
    }

    void ToBytes(uint8_t *bytes) const;
};
