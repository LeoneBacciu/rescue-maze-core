#include "Driver.hpp"


void Driver::Rotate(const bool right)
{
	Lasers* lasers = Lasers::Instance();
	Gyro* gyro = Gyro::Instance();
	Logger::Info(kDrivers, "rotating %s", right ? "right" : "left");
	const int8_t direction_multiplier = right ? 1 : -1;
	float c = lasers->ReadF(), r = lasers->ReadFR(), l = lasers->ReadFL();

	if (Lasers::IsValidWall(l, c, r))
	{
		Logger::Info(kDrivers, "aligning front with lasers");
		if (right)
		{
			SetSpeed(-20, 20);
			while (lasers->ComputeFrontDifference() > 1)
				delayMicroseconds(10); // Align with the front wall

			SetSpeed(50, -50);
			while (lasers->ComputeFrontDifference() < 1)
				delayMicroseconds(10); // Align with the front wall
		}
		else
		{
			SetSpeed(20, -20);
			while (lasers->ComputeFrontDifference() < 1)
				delayMicroseconds(10); // Align with the front wall
			SetSpeed(-50, 50);
			while (lasers->ComputeFrontDifference() > 1)
				delayMicroseconds(10); // Align with the front wall
		}
	}


	gyro->Calibrate();
	const float start = gyro->Yaw();
	float goal = math::ClampAngle(start + 80 * direction_multiplier);
	float current = start;


	const auto turn_condition = right ? RightTurnCondition : LeftTurnCondition;
	while (turn_condition(start, current, goal))
	{
		delayMicroseconds(100);
		current = gyro->Yaw();
		Logger::Verbose(kDrivers, "main rotation -> start: %f, obj: %f, curr: %f", start, goal, current);
	}
	Logger::Info(kDrivers, "final rotate adjustment");
	c = lasers->ReadF(), r = lasers->ReadFR(), l = lasers->ReadFL();
	SetSpeed(10 * direction_multiplier, -10 * direction_multiplier);
	int16_t diff = lasers->ComputeFrontDifference();
	if (c < cell_dimensions::depth && Lasers::IsValidWall(l, c, r))
	{
		while (right ? diff < 1 : diff > -1)
		{
			delayMicroseconds(1);
			diff = lasers->ComputeFrontDifference();
			Logger::Verbose(kDrivers, "final rotation (lasers) -> diff: %d", diff);
		}
	}
	else
	{
		goal = math::ClampAngle(start + 90 * direction_multiplier);
		while (turn_condition(start, current, goal))
		{
			delayMicroseconds(1);
			current = gyro->Yaw();
			Logger::Verbose(kDrivers, "final rotation -> start: %f, obj: %f, curr: %f", start, goal, current);
		}
	}
	Logger::Info(kDrivers, "rotation finished");
	SetSpeed(0, 0);
}

bool Driver::Go()
{
	Lasers* lasers = Lasers::Instance();
	Gyro* gyro = Gyro::Instance();
	Floor* floor = Floor::Instance();
	gyro->Calibrate();

	const uint16_t front_distance = lasers->ReadF(), back_distance = lasers->ReadB();
	const float start_rotation = gyro->Yaw();
	const bool use_front = front_distance < back_distance;
	uint16_t current_distance = use_front ? front_distance : back_distance;
	const uint8_t cells = static_cast<uint8_t>(current_distance) / static_cast<uint8_t>(cell_dimensions::depth);
	const uint16_t objective = std::max(5,
	                                    (cells + (use_front ? -1 : 1)) * cell_dimensions::depth + (
		                                    cell_dimensions::depth - dimensions::depth) / 2);
	const uint16_t start_cell = std::max(5,
	                                     cells * cell_dimensions::depth + (
		                                     cell_dimensions::depth - dimensions::depth) / 2);
	Logger::Info(kDrivers, "go forward -> start: %d, cells: %d, obj: %d, rot: %f", current_distance, cells, objective,
	             start_rotation);
	int8_t delta_yaw = 0;
	bool rear = false;
	while (GoCondition(C_NEGATE(use_front, rear), current_distance, rear ? start_cell : objective))
	{
		const uint16_t c = lasers->ReadF(), r = lasers->ReadFR(), l = lasers->ReadFL();
		const uint16_t lateral = lasers->ComputeLateralDifference();
		const bool near = use_front ? current_distance < objective + 5 : current_distance > objective - 5;
		uint16_t front_distance_component = std::max(1, std::max(l, std::max(c, r)) / cell_dimensions::depth);
		const uint16_t distance_component = std::max(1, current_distance / cell_dimensions::depth);

		const bool is_valid_wall = Lasers::IsValidWall(l, c, r);

		if (!rear) rear = floor->Read() == Floor::kBlack;

		const float current_angle = gyro->Yaw();
		const float delta_angle = math::AngleDifference(current_angle, start_rotation);

		int speed = abs(delta_angle) > 10 ? kSlow : near ? kMedium : kFast;
		delta_yaw = delta_angle;

		if (!is_valid_wall)
		{
			delta_yaw *= 5;
		}
		else
		{
			if (near && cells <= 1)
			{
				delta_yaw += Lasers::FrontDifference(l, r) * frontal_compensation_multiplier;
				delta_yaw /= distance_component;
			}
		}
		if (lateral < lateral_compensation_threshold && lateral > -lateral_compensation_threshold)
		{
			delta_yaw += math::Clamp(lateral * lateral_compensation_multiplier, -max_lateral_compensation_speed,
			                         static_cast<int>(max_lateral_compensation_speed)) / (is_valid_wall ? 1 : 2);
		}

		Logger::Verbose(
			kDrivers, "go forward lasers -> curr: %d, obj: %d, deltayaw: %d, speed: %d, near: %d, valid: %d",
			current_distance, objective, delta_yaw, speed, near, is_valid_wall);
		Logger::Verbose(kDrivers, "go forward gyro   -> start: %f, curr: %f, diff: %f", start_rotation, current_angle,
		                delta_angle);
		if (rear) speed *= -1;
		SetSpeed(speed - delta_yaw, speed + delta_yaw);
		delayMicroseconds(near ? 10 : 20);
		if (is_valid_wall) current_distance = use_front ? lasers->ReadF() : lasers->ReadB();
	}
	float current_angle = gyro->Yaw();
	Logger::Info(kDrivers, "final go adjusting");
	if (math::AngleDifference(current_angle, start_rotation) > 0)
	{
		SetSpeed(-15, 15);
		while (math::AngleDifference(current_angle, start_rotation) > 0)
		{
			current_angle = gyro->Yaw();
			delayMicroseconds(1);
			Logger::Verbose(kDrivers, "Adjusting -> starting: %f, current: %f", start_rotation, current_angle);
		}
	}
	else
	{
		SetSpeed(15, -15);
		while (math::AngleDifference(current_angle, start_rotation) < 0)
		{
			current_angle = gyro->Yaw();
			delayMicroseconds(1);
			Logger::Verbose(kDrivers, "Adjusting -> starting: %f, current: %f", start_rotation, current_angle);
		}
	}
	SetSpeed(0, 0);
	Logger::Info(kDrivers, "go forward finished%s", rear ? " (aborted)" : "");
	return !rear;
}

bool Driver::RightTurnCondition(const float start, const float current, const float goal)
{
	if (goal >= start)
	{
		return start <= current && current <= goal;
	}
	return start <= current && current <= 360 || 0 <= current && current <= goal;
}

bool Driver::LeftTurnCondition(const float start, const float current, const float goal)
{
	if (goal < start)
	{
		return goal <= current && current <= start;
	}
	return 0 <= current && current <= start || goal <= current && current <= 360;
}

bool Driver::GoCondition(const bool use_front, const uint16_t current, const uint16_t objective)
{
	return use_front ? current > objective : current < objective;
}


#if _EXECUTION_ENVIRONMENT == 0
void Driver::SetSpeed(int l, int r)
{
	GetBus()->SetSpeed(l, r);
}
#else
void Driver::SetSpeed(int l, int r) {
    int abs_l = abs(l), abs_r = abs(r);
    digitalWrite(INV_L1, l>=0);
    digitalWrite(INV_L2, l<=0);
    digitalWrite(INV_R1, r>=0);
    digitalWrite(INV_R2, r<=0);
    analogWrite(PWM_L, abs_l);
    analogWrite(PWM_R, abs_r);
}
#endif
