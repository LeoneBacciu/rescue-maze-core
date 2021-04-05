#include "Driver.hpp"


void Driver::Rotate(const bool right)
{
	Lasers* lasers = Lasers::Instance();
	Gyro* gyro = Gyro::Instance();
	const int8_t direction_multiplier = right ? 1 : -1;
	float c = lasers->ReadF(), r = lasers->ReadFR(), l = lasers->ReadFL();

	if (Lasers::IsValidWall(l, c, r))
	{
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
#ifdef UEDebug
		UE_LOG(LogTemp, Warning, TEXT("MAIN -> starting: %f, objective: %f, current: %f"), start, goal, current);
#endif
	}
	c = lasers->ReadF(), r = lasers->ReadFR(), l = lasers->ReadFL();
	SetSpeed(10 * direction_multiplier, -10 * direction_multiplier);
	int16_t diff = lasers->ComputeFrontDifference();
	if (c < cell_dimensions::depth && Lasers::IsValidWall(l, c, r))
	{
		while (right ? diff < 1 : diff > -1)
		{
			delayMicroseconds(1);
			diff = lasers->ComputeFrontDifference();
#ifdef UEDebug
			UE_LOG(LogTemp, Warning, TEXT("FINAL -> difference: %d"), diff);
#endif
		}
	}
	else
	{
		goal = math::ClampAngle(start + 90 * direction_multiplier);
		while (turn_condition(start, current, goal))
		{
			delayMicroseconds(1);
			current = gyro->Yaw();
#ifdef UEDebug
			UE_LOG(LogTemp, Warning, TEXT("FINAL -> starting: %f, objective: %f, current: %f"), start, goal, current);
#endif
		}
	}
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

#ifdef UEDebug
		UE_LOG(LogTemp, Warning,
		       TEXT(
			       "objective: %d, missingcells: %d, distance: %d, speed: %d, deltayaw: %d, dc: %d, fdc: %d, laser: %hs, near: %d"
		       ),
		       objective, cells, current_distance, speed, delta_yaw, distance_component, front_distance_component,
		       use_front?"front":"back", near);
		UE_LOG(LogTemp, Warning, TEXT("valid_wall: %d, startangle: %f, currentangle: %f, anglediff: %f"),
		       is_valid_wall,
		       start_rotation, current_angle, delta_angle);
#endif
		if (rear) speed *= -1;
		SetSpeed(speed - delta_yaw, speed + delta_yaw);
		delayMicroseconds(near ? 10 : 20);
		if (is_valid_wall) current_distance = use_front ? lasers->ReadF() : lasers->ReadB();
	}
	float current_angle = gyro->Yaw();
	if (math::AngleDifference(current_angle, start_rotation) > 0)
	{
		SetSpeed(-15, 15);
		while (math::AngleDifference(current_angle, start_rotation) > 0)
		{
			current_angle = gyro->Yaw();
			delayMicroseconds(1);
#ifdef UEDebug
			UE_LOG(LogTemp, Warning, TEXT("Adjusting -> starting: %f, current: %f"), start_rotation, current_angle);
#endif
		}
	}
	else
	{
		SetSpeed(15, -15);
		while (math::AngleDifference(current_angle, start_rotation) < 0)
		{
			current_angle = gyro->Yaw();
			delayMicroseconds(1);
#ifdef UEDebug
			UE_LOG(LogTemp, Warning, TEXT("Adjusting -> starting: %f, current: %f"), start_rotation, current_angle);
#endif
		}
	}
	SetSpeed(0, 0);
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
