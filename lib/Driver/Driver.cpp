#include "Driver.hpp"


void Driver::Rotate(const bool right) {
    Lasers *lasers = Lasers::Instance();
    Gyro *gyro = Gyro::Instance();

    lasers->StartContinuous();

    Logger::Info(kDrivers, "rotating %s", right ? "right" : "left");
    const int8_t direction_multiplier = right ? 1 : -1;

    AdjustFront(right);

    const float start = gyro->Yaw();
    float goal = math::ClampAngle(start - 70 * direction_multiplier);
    float current = start;
    const float minimum = math::ClampAngle(start + (right ? 10 : -10));

    SetSpeed(kRotateFast * direction_multiplier, -kRotateFast * direction_multiplier);

    const auto turn_condition = right ? RightTurnCondition : LeftTurnCondition;
    uint32_t current_time = millis();
    uint32_t timer_10 = current_time;
    uint32_t timer_100 = current_time;
    while (turn_condition(minimum, current, goal)) {
        current_time = millis();
        if (current_time - timer_100 >= 100) {
            Logger::Verbose(kDrivers, "main rotation -> start: %.2f, obj: %.2f, curr: %F", start, goal, current);
            timer_100 = millis();
        }
        if (current_time - timer_10 >= 9) {
            timer_10 = millis();
            current = gyro->Yaw();
        }
    }
    Logger::Info(kDrivers, "final rotate adjustment");
    uint16_t c = lasers->ReadF(), r = lasers->ReadFR(), l = lasers->ReadFL();
    SetSpeed(kRotateSlow * direction_multiplier, -kRotateSlow * direction_multiplier);
    int16_t diff = lasers->ComputeFrontDifference();

    current_time = millis();
    timer_10 = current_time;
    timer_100 = current_time;

    if (c < cell_dimensions::depth && Lasers::IsValidWall(l, c, r)) {
        while (right ? diff < 1 : diff > -1) {
            current_time = millis();
            if (current_time - timer_100 >= 100) {
                Logger::Verbose(kDrivers, "final rotation (lasers) -> diff: %d", diff);
                timer_100 = millis();
            }
            if (current_time - timer_10 >= 25) {
                timer_10 = millis();
                diff = lasers->ComputeFrontDifference();
            }
        }
    } else {
#if _EXECUTION_ENVIRONMENT == 0
        goal = math::ClampAngle(start - 90 * direction_multiplier);
#else
        goal = math::ClampAngle(start - (right ? 88.f : -85.f));
#endif
        while (turn_condition(start, current, goal)) {
            current_time = millis();
            if (current_time - timer_100 >= 100) {
                Logger::Verbose(kDrivers, "final rotation -> start: %.2f, obj: %.2f, curr: %F", start, goal,
                                current);
                timer_100 = millis();
            }
            if (current_time - timer_10 >= 9) {
                timer_10 = millis();
                current = gyro->Yaw();
            }
        }
    }
    CenterCell();
    Logger::Info(kDrivers, "rotation finished");
    SetSpeed(0, 0);

    lasers->StopContinuous();
}

bool Driver::Go() {
    Lasers *lasers = Lasers::Instance();
    Gyro *gyro = Gyro::Instance();
    Floor *floor = Floor::Instance();
    lasers->StartContinuous();

    uint16_t front_distance, back_distance;
    do {
        delay(100);
        front_distance = lasers->ReadF(), back_distance = lasers->ReadB();
        Serial2.printf("%d, %d\n", front_distance, back_distance);
    } while (!math::InRange<uint16_t>(front_distance, 1, 8000) && !math::InRange<uint16_t>(back_distance, 1, 8000));

//    if (front_distance < cell_dimensions::depth) return false;

    float current_rotation = gyro->Yaw(), delta_angle = 0;
    const float start_rotation = current_rotation;
    const bool use_front = front_distance < back_distance + cell_dimensions::depth;
    uint16_t current_distance = use_front ? front_distance : back_distance;
    const uint8_t cells = current_distance / cell_dimensions::depth;
    const uint16_t objective = std::max(50,
                                        (cells + (use_front ? -1 : 1)) * cell_dimensions::depth +
                                        (cell_dimensions::depth - dimensions::depth) / 2);
    const uint16_t start_cell = std::max(50,
                                         cells * cell_dimensions::depth +
                                         (cell_dimensions::depth - dimensions::depth) / 2);

    Logger::Info(kDrivers, "fd: %d, bd: %d, sr: %.2f, uf: %d, cd: %d, cs: %d, oj: %d, sc: %d",
                 front_distance, back_distance, start_rotation, use_front, current_distance, cells, objective,
                 start_cell);

    int16_t delta_yaw = 0, speed = kMedium;
    bool rear = false, is_valid_wall = true, near = false, really_near = false;
    int16_t front_difference = 0, lateral_difference = 0;
    uint8_t distance_component;

    uint32_t current_time = millis();
    uint32_t timer_10 = current_time;
    uint32_t timer_25 = current_time;
    uint32_t timer_50 = current_time;
    uint32_t timer_100 = current_time;

    while (GoCondition(C_NEGATE(use_front, rear), current_distance, rear ? start_cell : objective)) {
        current_time = millis();

        if (current_time >= timer_100 + 100) {
            Logger::Verbose(kDrivers,
                            "go forward -> delta: %d, distance: %d, obj: %d, d_angle: %.2f, validWall: %d, near: %d, fDifference: %d, lDifference: %d",
                            delta_yaw, current_distance, objective, delta_angle, is_valid_wall, near,
                            front_difference, lateral_difference);
            timer_100 = millis();
        }
        if (current_time >= timer_50 + 50) {
            near = C_NEGATE(use_front, rear) ? current_distance < objective + 100 : current_distance > objective - 100;
            really_near = C_NEGATE(use_front, rear) ? current_distance < objective + 75 : current_distance >
                                                                                          objective - 75;
            distance_component = std::max(1, current_distance / cell_dimensions::depth);

            const uint16_t c = lasers->ReadF(), r = lasers->ReadFR(), l = lasers->ReadFL();
            is_valid_wall = Lasers::IsValidWall(l, c, r);
            lateral_difference = lasers->ComputeLateralDifference(500);


            current_rotation = gyro->Yaw();
            delta_angle = math::AngleDifference(current_rotation, start_rotation);

            delta_yaw = -delta_angle;

            if (!really_near && abs(delta_angle) <= 15) {
                delta_yaw += math::Clamp<int16_t>(lateral_difference * lateral_compensation_multiplier,
                                                  -max_lateral_compensation_speed, max_lateral_compensation_speed);
            } else {
                delta_yaw *= 10;
            }
            delta_yaw *= 2;
            speed = really_near ? kSlow : (near ? kMedium : kFast);
//            if (!near) {
//                speed = kFast;
//            } else {
//            speed = kMedium;
//                delta_yaw *= 3;
//            }

//            if (!is_valid_wall) {
//                delta_yaw *= 2;
//            } else {
//                if (near && cells <= 1) {
//                    if (cells <= 1) {
//                        front_difference = Lasers::FrontDifference(L, R);
//                        delta_yaw += front_difference * 2;
//                    }
//                    delta_yaw /= distance_component;
//                }
//            }
            if (!rear) {
                if (floor->Read() == Floor::kBlack) rear = true;
            }
            timer_50 = millis();
        }
        if (current_time >= timer_25 + 25) {
            auto s = rear ? -speed : speed;
            SetSpeed(s - delta_yaw, s + delta_yaw);
            current_distance = use_front ? lasers->ReadF() : lasers->ReadB();
            if (math::InRange<uint16_t>(current_distance, 1, 8000)) timer_25 = millis();
        }
        if (current_time >= timer_10 + 4) {
            gyro->Update();
            timer_10 = millis();
        }
    }
    SetSpeed(rear ? kBreak : -kBreak, rear ? kBreak : -kBreak);
    delay(250);
    current_rotation = gyro->Yaw();
    delta_angle = math::AngleDifference(current_rotation, start_rotation);
    Logger::Info(kDrivers, "final go adjusting -> diff: %.2f", delta_angle);
    bool align_right = delta_angle > 0;

    SetSpeed(kRotateSlow * (align_right ? 1 : -1), kRotateSlow * (align_right ? -1 : 1));
    auto condition = align_right
                     ? static_cast<bool (*)(float, float)>([](float c, float s) {
                return math::AngleDifference(c, s) > 0;
            })
                     : static_cast<bool (*)(float, float)>([](float c, float s) {
                return math::AngleDifference(c, s) < 0;
            });
    timer_10 = current_time;
    timer_50 = current_time;
    while (condition(current_rotation, start_rotation)) {
        current_time = millis();
        if (current_time >= timer_50 + 50) {
            Logger::Verbose(kDrivers, "Adjusting -> starting: %F, current: %F", start_rotation, current_rotation);
            timer_50 = millis();
        }
        if (current_time >= timer_10 + 9) {
            current_rotation = gyro->Yaw();
            timer_10 = millis();
        }
    }
    SetSpeed(0, 0);
    AdjustFront(delta_yaw < 0);
    CenterCell();
    Logger::Info(kDrivers, "go forward finished%s", rear ? " (aborted)" : "");
    lasers->StopContinuous();
    return !rear;
}

bool Driver::AdjustFront(bool right) {
    delay(100);
    Lasers *lasers = Lasers::Instance();

    const int8_t direction_multiplier = right ? 1 : -1;
    uint16_t c = lasers->ReadF(), r = lasers->ReadFR(), l = lasers->ReadFL();
    if (!Lasers::IsValidWall(l, c, r)) return false;
    Logger::Info(kDrivers, "aligning with front lasers");

    auto first_condition = right ? LeftAdjustCondition : RightAdjustCondition;
    auto second_condition = right ? RightAdjustCondition : LeftAdjustCondition;

    SetSpeed(-kRotateSlow * direction_multiplier, kRotateSlow * direction_multiplier);
    Logger::Verbose(kDrivers, "aligning %s", right ? "left" : "right");
    while (first_condition(l, c, r)) {
        delay(50); // Align with the front wall
        r = lasers->ReadFR(), c = lasers->ReadF(), l = lasers->ReadFL();
        Logger::Verbose(kDrivers, "adjust -> L: %d, c: %d, R: %d", l, c, r);
    }

    SetSpeed(kRotateFast * direction_multiplier, -kRotateFast * direction_multiplier);
    Logger::Verbose(kDrivers, "aligning %s", right ? "right" : "left");
    while (second_condition(l, c, r)) {
        delay(50); // Align with the front wall
        r = lasers->ReadFR(), c = lasers->ReadF(), l = lasers->ReadFL();
        Logger::Verbose(kDrivers, "adjust -> L: %d, c: %d, R: %d", l, c, r);
    }
    SetSpeed(-kBreak * direction_multiplier, kBreak * direction_multiplier);
    delay(100);
    SetSpeed(0, 0);
    Logger::Info(kDrivers, "aligned");
    return true;
}

bool Driver::RightTurnCondition(const float start, const float current, const float goal) {
    if (goal < start) {
        return goal <= current && current <= start;
    }
    return (0 <= current && current <= start) || (goal <= current && current <= 360);
}

bool Driver::RightAdjustCondition(uint16_t l, uint16_t c, uint16_t r) {
    return Lasers::IsValidWall(l, c, r) && Lasers::FrontDifference(l, r) < 0;
}

bool Driver::LeftTurnCondition(const float start, const float current, const float goal) {
    if (goal >= start) {
        return start <= current && current <= goal;
    }
    return (start <= current && current <= 360) || (0 <= current && current <= goal);
}

bool Driver::LeftAdjustCondition(uint16_t l, uint16_t c, uint16_t r) {
    return Lasers::IsValidWall(l, c, r) && Lasers::FrontDifference(l, r) > 0;
}

bool Driver::GoCondition(const bool use_front, const uint16_t current, const uint16_t objective) {
    if (current > 8000) return true;
    bool r = use_front ? current > objective : current < objective;
    return r;
}

void Driver::CenterCell() {
    Lasers *lasers = Lasers::Instance();
    const uint16_t f = lasers->ReadF(), b = lasers->ReadB();
    bool use_front = f < b;
    uint16_t distance = use_front ? f : b;
//    const uint16_t threshold = (distance > (cell_dimensions::depth / 2)) ? 20 : 40;
    if (distance > (cell_dimensions::depth / 2))return;
    int speed = kMedium * (use_front ? 1 : -1);
    Driver::SetSpeed(speed, speed);
    while (distance >= 20) {
        distance = (use_front ? lasers->ReadF() : lasers->ReadB());
        delay(75);
    }
    Driver::SetSpeed(-speed, -speed);
    while (distance <= 25) {
        distance = (use_front ? lasers->ReadF() : lasers->ReadB());
        delay(75);
    }
    Driver::SetSpeed(0, 0);
}

#if _EXECUTION_ENVIRONMENT == 0
void Driver::SetSpeed(int L, int R) {
    GetBus()->SetSpeed(L, R);
}
#else

void Driver::SetSpeed(int l, int r) {
    int abs_l = abs(l), abs_r = abs(r);
    digitalWrite(INV_L1, l >= 0);
    digitalWrite(INV_L2, l <= 0);
    digitalWrite(INV_R1, r >= 0);
    digitalWrite(INV_R2, r <= 0);
    analogWrite(PWM_L, abs_l);
    analogWrite(PWM_R, abs_r);
}


#endif
