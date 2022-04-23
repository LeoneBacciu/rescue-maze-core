#include "Driver.hpp"
#include <Gyro.hpp>
#include <Floor.hpp>
#include <Lasers.hpp>
#include <Logger.hpp>
#include <utils/Math.hxx>

int Driver::speedL = 0;
int Driver::speedR = 0;
bool Driver::center_next = false;

void Driver::Rotate(const bool right, const bool center_start, const bool center_end) {
    Gyro *gyro = Gyro::Instance();

//    if (!gyro->IsTilted(2)) {
//        Logger::Warn(kDrivers, "Resetting gyro");
//        delay(250);
//        gyro->Begin();
//        delay(250);
//    }

    Logger::Info(kDrivers, "rotating %s", right ? "right" : "left");
    const int8_t direction_multiplier = right ? 1 : -1;

    auto tilted = center_start && CenterCell();

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
            if (!tilted) tilted = gyro->IsTilted();
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

    int16_t trials = 100;
    goal = math::ClampAngle(start - 85.f * direction_multiplier);
    while (turn_condition(start, current, goal)) {
        current_time = millis();
        if (current_time - timer_100 >= 100) {
            Logger::Verbose(kDrivers, "final rotation -> start: %.2f, obj: %.2f, curr: %.2f", start, goal,
                            current);
            if (!tilted) tilted = gyro->IsTilted();
            timer_100 = millis();
        }
        if (current_time - timer_10 >= 10) {
            if (--trials < 0) {
                SetSpeed(kRotateFast * direction_multiplier, -kRotateFast * direction_multiplier);
            }
            timer_10 = millis();
            current = gyro->Yaw();
        }
    }
    Break(-kRotateSlow * direction_multiplier, kRotateSlow * direction_multiplier);
    tilted |= center_end && CenterCell();
    if (tilted) center_next = true;
    Logger::Info(kDrivers, "rotation finished (%d)", tilted);
}

bool Driver::Go() {
    Lasers *lasers = Lasers::Instance();
    Gyro *gyro = Gyro::Instance();
    Floor *floor = Floor::Instance();

//    if (!gyro->IsTilted(2)) {
//        Logger::Warn(kDrivers, "Resetting gyro");
//        delay(200);
//        gyro->Begin();
//        delay(200);
//    }

    uint16_t front_distance, back_distance;
    do {
        delay(100);
        front_distance = lasers->ReadFront(), back_distance = lasers->ReadB();
    } while (!math::InRange<uint16_t>(front_distance, 1, 8000) && !math::InRange<uint16_t>(back_distance, 1, 8000));

    if (front_distance < cell_dimensions::depth) return false;

    float current_rotation = gyro->Yaw(), delta_angle = 0;
    const float start_rotation = current_rotation;
    bool use_front = front_distance < back_distance + cell_dimensions::depth;
    Logger::Info(kDrivers, "f: %d, b: %d, d: %d", front_distance, back_distance,
                 (front_distance + back_distance + dimensions::depth) % cell_dimensions::depth);

    if ((front_distance + back_distance + dimensions::depth) % cell_dimensions::depth > movements::unsafe_wall) {
        const int loops = 10;
        int fds[loops], bds[loops];
        int favg = 0, bavg = 0;
        for (int i = 0; i < loops; i++) {
            fds[i] = lasers->ReadF();
            favg += fds[i];
            bds[i] = lasers->ReadB();
            bavg += bds[i];
            delay(100);
        }
        favg /= loops;
        bavg /= loops;
        int fdelta = 0, bdelta = 0;
        for (int i = 0; i < loops; i++) {
            fdelta += abs(fds[i] - favg);
            bdelta += abs(bds[i] - bavg);
        }
        fdelta /= loops;
        bdelta /= loops;
        Logger::Info(kDrivers, "fdelta: %d, bdelta: %d", fdelta, bdelta);
        use_front = fdelta < bdelta;
    }

    uint16_t current_distance = use_front ? front_distance : back_distance;
    const uint8_t cells = current_distance / cell_dimensions::depth;
    uint16_t objective = (cells + (use_front ? -1 : 1)) * cell_dimensions::depth + movements::arrived;
    const uint16_t start_cell = cells * cell_dimensions::depth + movements::arrived;

    Logger::Info(kDrivers, "fd: %d, bd: %d, sr: %.2f, uf: %d, cd: %d, cs: %d, oj: %d, sc: %d",
                 front_distance, back_distance, start_rotation, use_front, current_distance, cells, objective,
                 start_cell);

    int16_t delta = 0, speed = kMedium;
    bool rear = false, is_valid_wall = true, near = false, really_near = false, tilted = false, next_tilted = false;
    int8_t is_ramp = 0;
    int16_t front_difference = 0, lateral_difference = 0;
    float pitch = gyro->Pitch();

    uint32_t current_time = millis();
    uint32_t timer_100 = current_time;
    uint32_t timer_250 = current_time;

    while (GoCondition(C_NEGATE(use_front, rear), current_distance, rear ? start_cell : objective, is_ramp, pitch)) {
        current_time = millis();

        if (current_time >= timer_250 + 250) {
            Logger::Verbose(kDrivers,
                            "go forward -> cd: %d, delta: %d, obj: %d, d_angle: %.2f, validWall: %d, near: %d, fDifference: %d, lDifference: %d, speed: %d, t: %d, r: %d",
                            current_distance + 1, delta, objective, delta_angle, is_valid_wall, near,
                            front_difference, lateral_difference, speed, tilted, is_ramp);
            timer_250 = millis();
        }
        if (current_time >= timer_100 + 100) {
            near = C_NEGATE(use_front, rear) ? current_distance < objective + movements::near : current_distance >
                                                                                                objective -
                                                                                                movements::near;
            really_near = C_NEGATE(use_front, rear) ? current_distance < objective + movements::really_near :
                          current_distance >
                          objective - movements::really_near;

            lateral_difference = lasers->ComputeLateralDifference();

            if (is_ramp == 0) is_ramp = gyro->IsRamp();
//            else {

//                use_front = is_ramp == 1;
//                objective = (cells + (use_front ? -1 : 1)) * cell_dimensions::depth + movements::arrived;
//            }

            pitch = gyro->Pitch();
            current_rotation = gyro->Yaw();
            delta_angle = math::AngleDifference(current_rotation, start_rotation);

            if (really_near && !rear) {
                delta = 0;
            } else if (near) {
                delta = delta_angle * -8;
            } else {
                delta = delta_angle * -5;
            }

            if (abs(delta_angle) <= 20) {
                delta += math::Clamp<int16_t>(lateral_difference * lateral_compensation_multiplier,
                                              -max_lateral_compensation_speed, max_lateral_compensation_speed);
            }

            delta *= 2;

            if (is_ramp != 0) {
                delta /= 5;
            }

            if (!tilted) {
                tilted = gyro->IsTilted();
            }
            if (near) {
                tilted = gyro->IsTilted();
                if (tilted && !next_tilted) next_tilted = true;
            }
            speed = is_ramp == 1 ? kFast : (is_ramp == -1 ? kMedium : (tilted ? kVeryFast : (really_near ? kSlow : (near
                                                                                                                    ? kMedium
                                                                                                                    : kFast))));
            if (!(rear || tilted || next_tilted)) {
                if (floor->Read() == Floor::kBlack) rear = true;
            }
            auto s = rear ? -speed : speed;
            delta = min<int16_t>(delta, abs(speed)); /// Avoid accidental reverse
            SetSpeed(s - delta, s + delta);
            current_distance = use_front ? lasers->ReadFront() : lasers->ReadB();

            timer_100 = millis();
        }
    }
    Break(kBreak * (rear ? 1 : -1), kBreak * (rear ? 1 : -1));

    if (is_ramp != 0) {
        ReturnToAngle(start_rotation);
        PartialCenter(start_rotation, is_ramp == 1);
        ReturnToAngle(start_rotation);
        center_next = false;
        delay(1000);
    } else {
        auto end_diff = math::AngleDifference(start_rotation, current_rotation);
        Logger::Error(kDrivers, "stuff %d %d %.2f", next_tilted, lateral_difference, end_diff);

        if (next_tilted ||
            (lateral_difference < 15 && abs(end_diff) < 15) ||
            !center_next && !ExpensiveCenter()) {
            ReturnToAngle(start_rotation, next_tilted);
            CenterCell();
        }
        if (center_next) {
            ExpensiveCenter();
            center_next = false;
        }
    }


    Logger::Info(kDrivers, "go forward finished%s", rear ? " (aborted)" : "");
    return !rear;
}

bool Driver::AdjustFront(bool right) {
//    delay(100);
//    Lasers *lasers = Lasers::Instance();
//
//    const int8_t direction_multiplier = right ? 1 : -1;
//    uint16_t c = lasers->ReadFront(), r = lasers->ReadFrontR(), l = lasers->ReadFrontL();
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
//        r = lasers->ReadFrontR(), c = lasers->ReadFront(), l = lasers->ReadFrontL();
//        Logger::Verbose(kDrivers, "adjust -> L: %d, c: %d, R: %d", l, c, r);
//    }
//
//    SetSpeed(kRotateSlow * direction_multiplier, -kRotateSlow * direction_multiplier);
//    Logger::Verbose(kDrivers, "aligning %s", right ? "right" : "left");
//    while (second_condition(l, c, r)) {
//        delay(50); // Align with the front wall
//        r = lasers->ReadFrontR(), c = lasers->ReadFront(), l = lasers->ReadFrontL();
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

bool Driver::GoCondition(const bool use_front, const uint16_t current, const uint16_t objective, const uint8_t is_ramp,
                         const float pitch) {
    if (is_ramp != 0) {
        if (abs(pitch - 180) < 5) return false;
        return true;
    }
    if (current > 8000) return true;
    bool r = use_front ? current > objective : current < objective;
    if (!r) Logger::Verbose(kLasers, "END: %d - %d", current, objective);
    return r;
}

bool Driver::CenterCell() {
    Lasers *lasers = Lasers::Instance();
    Gyro *gyro = Gyro::Instance();
    const uint16_t f = lasers->ReadFront(), b = lasers->ReadB();
    bool use_front = f < b;
    uint16_t distance = use_front ? f : b;
    bool tilted = false;

    if (distance > 5000) return false;

    auto start_angle = gyro->Yaw();
    if (distance > (cell_dimensions::depth / 2)) {
        int16_t diff, trials = 50;
        do {
            if (gyro->IsTilted()) {
                tilted = true;
                break;
            }
            diff = lasers->ComputeVerticalDifference();
            auto delta = math::AngleDifference(gyro->Yaw(), start_angle) * -5;
            Logger::Verbose(kDrivers, "center middle %d", diff);
            int speed = abs(diff) > 30 ? kMedium : kSlow;
            SetSpeed(speed * (diff > 0 ? 1 : -1)- delta, speed * (diff > 0 ? 1 : -1) + delta);
            delay(50);
        } while (abs(diff) > 10 && --trials > 0);
        Break(kBreak * (diff > 0 ? -1 : 1), kBreak * (diff > 0 ? -1 : 1), 250);
        return tilted;
    }

    int speed = kMedium * (use_front ? 1 : -1);
    int16_t trials = 25;
    while (distance >= movements::touch && --trials > 0) {
        if (gyro->IsTilted()) {
            tilted = true;
            break;
        }
//        auto delta = math::AngleDifference(gyro->Yaw(), start_angle) * -5;
//        Driver::SetSpeed(speed - delta, speed + delta);
        SetSpeed(speed, speed);
        distance = (use_front ? lasers->ReadFront() : lasers->ReadB());
        Logger::Verbose(kDrivers, "center first %d", distance);
        delay(100);
    }
    trials = 20;
    delay(250);
    start_angle = gyro->Yaw();
    while (distance <= movements::not_touch && --trials > 0) {
        auto delta = math::AngleDifference(gyro->Yaw(), start_angle) * -10;
        Driver::SetSpeed(-speed - delta, -speed + delta);
        distance = (use_front ? lasers->ReadFront() : lasers->ReadB());
        Logger::Verbose(kDrivers, "center second %d", distance);
        delay(100);
    }
    Break(speed, speed);
    ReturnToAngle(start_angle);
    return tilted;
}

void Driver::Break(int l, int r, int time) {
    SetSpeed(l, r);
    delay(time);
    SetSpeed(0, 0);
}

void Driver::ReturnToAngle(float goal, bool fast) {
    Gyro *gyro = Gyro::Instance();
    uint8_t factor = fast ? 10 : 5;
    auto current_rotation = gyro->Yaw();
    auto delta_angle = math::AngleDifference(current_rotation, goal);
    Logger::Info(kDrivers, "final go adjusting -> diff: %.2f", delta_angle);
    if (delta_angle < 2) return;
    bool align_right = delta_angle > 0;
    goal += align_right ? factor : -factor;
    auto speed = fast ? kRotateFast : kRotateSlow;

    SetSpeed(speed * (align_right ? 1 : -1), speed * (align_right ? -1 : 1));
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
    int16_t trials = 40;
    while (condition(current_rotation, goal)) {
        current_time = millis();
        if (current_time >= timer_50 + 50) {
            Logger::Verbose(kDrivers, "Adjusting -> starting: %.2f, current: %.2f, d: %.2f", goal,
                            current_rotation, math::AngleDifference(current_rotation, goal));
            timer_50 = millis();
        }
        if (current_time >= timer_25 + 25) {
            if (--trials < 0) {
                SetSpeed(kRotateFast * (align_right ? 1 : -1), kRotateFast * (align_right ? -1 : 1));
            }
            current_rotation = gyro->Yaw();
            timer_25 = millis();
        }
    }
    Break(kBreak * (align_right ? -1 : 1), kBreak * (align_right ? 1 : -1), 200);
}

bool Driver::ExpensiveCenter() {
    Lasers *lasers = Lasers::Instance();
    Gyro *gyro = Gyro::Instance();
    Logger::Info(kDrivers, "expensive center");
    if (gyro->IsTilted()) return false;
    if (lasers->ReadR() < cell_dimensions::depth) {
        Rotate(false, false);
        Rotate(true);
    } else if (lasers->ReadL() < cell_dimensions::depth) {
        Rotate(true, false);
        Rotate(false);
    } else {
        return false;
    }
    auto start_angle = gyro->Yaw();
    int16_t diff, trials = 50;
    do {
        diff = lasers->ComputeVerticalDifference();
        Logger::Verbose(kDrivers, "center middle %d", diff);
        auto delta = math::AngleDifference(gyro->Yaw(), start_angle) * -5;
        int speed = abs(diff) > 30 ? kMedium : kSlow;
        SetSpeed(speed * (diff > 0 ? 1 : -1) - delta, speed * (diff > 0 ? 1 : -1) + -delta);
        delay(50);
    } while (abs(diff) > 10 && --trials > 0);
    Break(kBreak * (diff > 0 ? -1 : 1), kBreak * (diff > 0 ? -1 : 1), 250);

    return true;
}

void Driver::PartialCenter(const float start_angle, const bool up) {
    Lasers *lasers = Lasers::Instance();
    Gyro *gyro = Gyro::Instance();
    const auto front = lasers->ReadFront();
    const auto cells = front / cell_dimensions::depth;
    const auto obj = cells * cell_dimensions::depth + (up ? movements::arrived : movements::really_near);
    uint16_t current = lasers->ReadFront();
    Logger::Info(kDrivers, "partial center : %d, %d, %d, %d", front, cells, obj, current);
    while (current > obj) {
        current = lasers->ReadFront();
        auto delta = math::AngleDifference(gyro->Yaw(), start_angle) * -5;
        auto lateral_difference = lasers->ComputeLateralDifference();
        delta += math::Clamp<int16_t>(lateral_difference * lateral_compensation_multiplier_ramp,
                                      -max_lateral_compensation_speed_ramp, max_lateral_compensation_speed_ramp);
        Driver::SetSpeed(kMedium - delta, kMedium + delta);
        delay(50);
    }
    Break(-kBreakHard, -kBreakHard);
}

void Driver::Pause() {
    digitalWrite(INV_L1, HIGH);
    digitalWrite(INV_L2, HIGH);
    digitalWrite(INV_R1, HIGH);
    digitalWrite(INV_R2, HIGH);
    analogWrite(PWM_L, LOW);
    analogWrite(PWM_R, LOW);
}

void Driver::Resume() {
    SetSpeed(speedL, speedR);
}

#if _EXECUTION_ENVIRONMENT == 0
void Driver::SetSpeed(int L, int R) {
    GetBus()->SetSpeed(L, R);
}
#else

void Driver::SetSpeed(int l, int r) {
    speedL = l;
    speedR = r;
    int abs_l = abs(l), abs_r = abs(r);
    digitalWrite(INV_L1, l >= 0);
    digitalWrite(INV_L2, l <= 0);
    digitalWrite(INV_R1, r >= 0);
    digitalWrite(INV_R2, r <= 0);
    analogWrite(PWM_L, abs_l);
    analogWrite(PWM_R, abs_r);
}


#endif
