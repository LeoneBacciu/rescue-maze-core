#include "Driver.hpp"


void Driver::Rotate(const bool right) {
    Lasers *lasers = Lasers::Instance();
    Gyro *gyro = Gyro::Instance();

    lasers->StartContinuous();

    Logger::Info(kDrivers, "rotating %s", right ? "right" : "left");
    const int8_t direction_multiplier = right ? 1 : -1;

    CenterCell();

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
            Logger::Verbose(kDrivers, "main rotation -> start: %.2f, obj: %.2f, curr: %.2f", start, goal, current);
            timer_100 = millis();
        }
        if (current_time - timer_10 >= 10) {
            timer_10 = millis();
            current = gyro->Yaw();
        }
    }
    Logger::Info(kDrivers, "final rotate adjustment");
    SetSpeed(kRotateSlow * direction_multiplier, -kRotateSlow * direction_multiplier);

    current_time = millis();
    timer_10 = current_time;
    timer_100 = current_time;

    goal = math::ClampAngle(start - 85.f * direction_multiplier);
    while (turn_condition(start, current, goal)) {
        current_time = millis();
        if (current_time - timer_100 >= 100) {
            Logger::Verbose(kDrivers, "final rotation -> start: %.2f, obj: %.2f, curr: %.2f", start, goal,
                            current);
            timer_100 = millis();
        }
        if (current_time - timer_10 >= 10) {
            timer_10 = millis();
            current = gyro->Yaw();
        }
    }
    Break(-kRotateSlow * direction_multiplier, kRotateSlow * direction_multiplier);
    CenterCell();
    Logger::Info(kDrivers, "rotation finished");

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

    if (front_distance < cell_dimensions::depth) return false;

    float current_rotation = gyro->Yaw(), delta_angle = 0;
    const float start_rotation = current_rotation;
    const bool use_front = front_distance < back_distance + cell_dimensions::depth;
    uint16_t current_distance = use_front ? front_distance : back_distance;
    const uint8_t cells = current_distance / cell_dimensions::depth;
    const uint16_t objective = (cells + (use_front ? -1 : 1)) * cell_dimensions::depth + movements::arrived;
    const uint16_t start_cell = cells * cell_dimensions::depth + movements::arrived;

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
                            "go forward -> cd: %d, delta: %d, obj: %d, d_angle: %.2f, validWall: %d, near: %d, fDifference: %d, lDifference: %d, speed: %d",
                            current_distance + 1, delta_yaw, objective, delta_angle, is_valid_wall, near,
                            front_difference, lateral_difference, speed);
            timer_100 = millis();
        }
        if (current_time >= timer_50 + 50) {
            near = C_NEGATE(use_front, rear) ? current_distance < objective + movements::near : current_distance >
                                                                                                objective -
                                                                                                movements::near;
            really_near = C_NEGATE(use_front, rear) ? current_distance < objective + movements::really_near :
                          current_distance >
                          objective - movements::really_near;

            lateral_difference = lasers->ComputeLateralDifference(1000);

            current_rotation = gyro->Yaw();
            delta_angle = math::AngleDifference(current_rotation, start_rotation);

            delta_yaw = delta_angle * -5;

            if (!really_near && abs(delta_angle) <= 20) {
                delta_yaw += math::Clamp<int16_t>(lateral_difference * lateral_compensation_multiplier,
                                                  -max_lateral_compensation_speed, max_lateral_compensation_speed);
            } else {
                delta_yaw *= 5;
            }
            delta_yaw *= 2;
            speed = really_near ? kSlow : (near ? kMedium : kFast);
//            if (!rear) {
//                if (floor->Read() == Floor::kBlack) rear = true;
//            }
            auto s = rear ? -speed : speed;
            SetSpeed(s - delta_yaw, s + delta_yaw);
            current_distance = use_front ? lasers->ReadF() : lasers->ReadB();

            timer_50 = millis();
        }
    }
    ReturnToAngle(start_rotation);
    Logger::Info(kDrivers, "go forward finished%s", rear ? " (aborted)" : "");
    lasers->StopContinuous();
    return !rear;
}

bool Driver::AdjustFront(bool right) {
//    delay(100);
//    Lasers *lasers = Lasers::Instance();
//
//    const int8_t direction_multiplier = right ? 1 : -1;
//    uint16_t c = lasers->ReadF(), r = lasers->ReadFR(), l = lasers->ReadFL();
//    if (!Lasers::IsValidWall(l, c, r)) return false;
//    Logger::Info(kDrivers, "aligning with front lasers");
//
//    auto first_condition = right ? LeftAdjustCondition : RightAdjustCondition;
//    auto second_condition = right ? RightAdjustCondition : LeftAdjustCondition;
//
//    SetSpeed(-kRotateSlow * direction_multiplier, kRotateSlow * direction_multiplier);
//    Logger::Verbose(kDrivers, "aligning %s", right ? "left" : "right");
//    while (first_condition(l, c, r)) {
//        delay(50); // Align with the front wall
//        r = lasers->ReadFR(), c = lasers->ReadF(), l = lasers->ReadFL();
//        Logger::Verbose(kDrivers, "adjust -> L: %d, c: %d, R: %d", l, c, r);
//    }
//
//    SetSpeed(kRotateSlow * direction_multiplier, -kRotateSlow * direction_multiplier);
//    Logger::Verbose(kDrivers, "aligning %s", right ? "right" : "left");
//    while (second_condition(l, c, r)) {
//        delay(50); // Align with the front wall
//        r = lasers->ReadFR(), c = lasers->ReadF(), l = lasers->ReadFL();
//        Logger::Verbose(kDrivers, "adjust -> L: %d, c: %d, R: %d", l, c, r);
//    }
//    Break(-kBreak * direction_multiplier, kBreak * direction_multiplier, 50);
//    Logger::Info(kDrivers, "aligned");
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
    if (!r) Logger::Verbose(kLasers, "END: %d - %d", current, objective);
    return r;
}

void Driver::CenterCell() {
    Lasers *lasers = Lasers::Instance();
    Gyro *gyro = Gyro::Instance();
    const uint16_t f = lasers->ReadF(), b = lasers->ReadB();
    bool use_front = f < b;
    uint16_t distance = use_front ? f : b;

    if (distance > (cell_dimensions::depth / 2)) return;

    int speed = kMedium * (use_front ? 1 : -1);
    Driver::SetSpeed(speed, speed);
    while (distance >= movements::touch) {
        distance = (use_front ? lasers->ReadF() : lasers->ReadB());
        Logger::Verbose(kDrivers, "center first %d", distance);
        delay(75);
    }
    delay(250);
    auto start_angle = gyro->Yaw();
    Driver::SetSpeed(-speed, -speed);
    while (distance <= movements::touch) {
        distance = (use_front ? lasers->ReadF() : lasers->ReadB());
        Logger::Verbose(kDrivers, "center second %d", distance);
        Driver::SetSpeed(-speed, -speed);
        gyro->Update();
        delay(75);
    }
    Break(speed, speed);
    ReturnToAngle(start_angle);
}

void Driver::Break(int l, int r, int time) {
    SetSpeed(l, r);
    delay(time);
    SetSpeed(0, 0);
}

void Driver::ReturnToAngle(float goal) {
    Gyro *gyro = Gyro::Instance();
    auto current_rotation = gyro->Yaw();
    auto delta_angle = math::AngleDifference(current_rotation, goal);
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
    auto current_time = millis();
    auto timer_50 = current_time;
    auto timer_25 = current_time;
    while (condition(current_rotation, goal)) {
        current_time = millis();
        if (current_time >= timer_50 + 50) {
            Logger::Verbose(kDrivers, "Adjusting -> starting: %.2f, current: %.2f, d: %.2f", goal,
                            current_rotation, math::AngleDifference(current_rotation, goal));
            timer_50 = millis();
        }
        if (current_time >= timer_25 + 25) {
            current_rotation = gyro->Yaw();
            timer_25 = millis();
        }
    }
    Break(kBreak * (align_right ? -1 : 1), kBreak * (align_right ? 1 : -1), 200);
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
