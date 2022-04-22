#include "Lasers.hpp"
#include <Driver.hpp>

#include <utils/Math.hxx>


#if _EXECUTION_ENVIRONMENT == 0
void Lasers::Begin()
{
}

void Lasers::StartContinuous()
{
}

void Lasers::StopContinuous()
{
}

uint16_t Lasers::ReadF()
{
    return Read(GetBus()->GetActor()->GetActorForwardVector()) - dimensions::depth / 2;
}

float Lasers::ComputeFrontAngle()
{
    return -atan((ReadFL() - ReadFR()) / dimensions::front_lasers_distance);
}

int16_t Lasers::ComputeFrontDifference()
{
    return ReadFR() - ReadFL();
}

int16_t Lasers::ComputeLateralDifference(const uint16_t threshold)
{
    const uint16_t L = ReadL(), R = ReadR();
    if (L > threshold || R > threshold) return 0;
    return L % cell_dimensions::depth - R % cell_dimensions::depth;
}

uint16_t Lasers::ReadFL()
{
    return Read(GetBus()->GetActor()->GetActorForwardVector(), -dimensions::front_lasers_distance / 2)
        - dimensions::depth / 2;
}

uint16_t Lasers::ReadFR()
{
    return Read(GetBus()->GetActor()->GetActorForwardVector(), dimensions::front_lasers_distance / 2)
        - dimensions::depth / 2;
}

uint16_t Lasers::ReadL()
{
    return Read(GetBus()->GetActor()->GetActorRightVector() * -1)
        - dimensions::width / 2;
}

uint16_t Lasers::ReadR()
{
    return Read(GetBus()->GetActor()->GetActorRightVector())
        - dimensions::width / 2;
}

uint16_t Lasers::ReadB()
{
    return Read(GetBus()->GetActor()->GetActorForwardVector() * -1)
        - dimensions::depth / 2;
}

bool Lasers::IsValidWall(const uint16_t L, const uint16_t c, const uint16_t R, const int tolerance)
{
    const uint16_t mean = std::min(R, L) + abs(R - L) / 2;
    return abs(L - R) < dimensions::front_lasers_distance && mean - tolerance <= c && c <= mean + tolerance;
}

int16_t Lasers::FrontDifference(const uint16_t L, const uint16_t R)
{
    return R - L;
}

uint16_t Lasers::Read(FVector direction, float delta_y, bool draw) const
{
    FHitResult out_hit;

    FVector start = GetBus()->GetActor()->GetActorLocation();
    if (delta_y != 0) start += GetBus()->GetActor()->GetActorRotation().RotateVector(FVector(0, delta_y, 0));

    FVector end = (start + (direction * 800.0f));

    FCollisionQueryParams collision_params;

    bool is_hit = GetBus()->GetActor()->GetWorld()->LineTraceSingleByChannel(out_hit, start, end, ECC_Visibility,
                                                                             collision_params);

    if (draw)
        DrawDebugLine(GetBus()->GetActor()->GetWorld(), start, FVector(out_hit.Location), FColor::Blue, false, 0.5,
                      0, 0.5);
    float result = 8192;
    if (is_hit)
    {
        result = out_hit.Distance * 10;
    }
    return MakeError(result);
}

float Lasers::MakeError(const float value) const
{
    return value * FMath::RandRange(0.97f, 1.03f);
}

#else

void Lasers::Begin() {
    changeAddress(ADDRESSES::R);
    laserR.init(GetBus());
    laserR.setTimeout(100);
    laserR.setHighPrecision();
    Logger::Verbose(kLasers, "R ok");
    changeAddress(ADDRESSES::FR);
    laserFR.init(GetBus());
    laserFR.setTimeout(100);
    laserFR.setHighPrecision();
    Logger::Verbose(kLasers, "FR ok");
    changeAddress(ADDRESSES::F);
    laserF.init(GetBus());
    laserF.setTimeout(100);
    laserF.setHighPrecision();
    Logger::Verbose(kLasers, "F ok");
    changeAddress(ADDRESSES::FL);
    laserFL.init(GetBus());
    laserFL.setTimeout(100);
    laserFL.setHighPrecision();
    Logger::Verbose(kLasers, "FL ok");
    changeAddress(ADDRESSES::L);
    laserL.init(GetBus());
    laserL.setTimeout(100);
    laserL.setHighPrecision();
    Logger::Verbose(kLasers, "L ok");
    changeAddress(ADDRESSES::B);
    laserB.init(GetBus());
    laserB.setTimeout(100);
    laserB.setHighPrecision();
    Logger::Verbose(kLasers, "B ok");
}

void Lasers::StartContinuous() {
    continuous = true;
    changeAddress(ADDRESSES::R);
    laserR.startContinuous();
    changeAddress(ADDRESSES::FR);
    laserFR.startContinuous();
    changeAddress(ADDRESSES::F);
    laserF.startContinuous();
    changeAddress(ADDRESSES::FL);
    laserFL.startContinuous();
    changeAddress(ADDRESSES::L);
    laserL.startContinuous();
    changeAddress(ADDRESSES::B);
    laserB.startContinuous();
}

void Lasers::StopContinuous() {
    continuous = false;
    changeAddress(ADDRESSES::R);
    laserR.stopContinuous();
    changeAddress(ADDRESSES::FR);
    laserFR.stopContinuous();
    changeAddress(ADDRESSES::F);
    laserF.stopContinuous();
    changeAddress(ADDRESSES::FL);
    laserFL.stopContinuous();
    changeAddress(ADDRESSES::L);
    laserL.stopContinuous();
    changeAddress(ADDRESSES::B);
    laserB.stopContinuous();


}

float Lasers::ComputeFrontAngle() {
    return -atan((ReadFL() - ReadFR()) / dimensions::front_lasers_distance);
}

int16_t Lasers::ComputeFrontDifference() {
    return FrontDifference(ReadFL(), ReadFR());
}

int16_t Lasers::ComputeLateralDifference(const uint16_t threshold, const int16_t bias) {
    uint16_t l, r, trials = 5;
    do {
        l = ReadL();
        r = ReadR();
        trials--;
        delay(1);
    } while (trials > 0 && (!math::InRange<uint16_t>(l, 1, 8000) || !math::InRange<uint16_t>(r, 1, 8000)));
    if (l > threshold || r > threshold) return 0;
    uint16_t cells = std::max<float>((float) l / cell_dimensions::depth + (float) r / cell_dimensions::depth, 1);
    int16_t diff = l % cell_dimensions::depth - r % cell_dimensions::depth + bias;
    return static_cast<float>(diff) / static_cast<float>(cells);
}

int16_t Lasers::ComputeVerticalDifference(uint16_t threshold, int16_t bias) {
    uint16_t f, b, trials = 5;
    do {
        f = ReadF();
        b = ReadB();
        trials--;
        delay(1);
    } while (trials > 0 && (!math::InRange<uint16_t>(f, 1, 8000) || !math::InRange<uint16_t>(b, 1, 8000)));
    if (f > threshold || b > threshold || (f + b + dimensions::depth) % cell_dimensions::depth > movements::unsafe_wall) return 0;

    int32_t f_remainder = f % cell_dimensions::depth;
    int32_t b_remainder = b % cell_dimensions::depth;
    f_remainder = f_remainder > dimensions::depth ? f_remainder - cell_dimensions::depth : f_remainder;
    b_remainder = b_remainder > dimensions::depth ? b_remainder - cell_dimensions::depth : b_remainder;

    return f_remainder - b_remainder + bias;
}

void Lasers::changeAddress(uint8_t laser) {
    delay(10);
    if (laser > 7 || laser == current_bus) return;
    GetBus()->beginTransmission(0x70);
    GetBus()->write(1 << laser);
    GetBus()->endTransmission();
    current_bus = laser;
    delay(10);
}


uint16_t Lasers::ReadR() {
    return Read(laserR, ADDRESSES::R, &rHighPrecision) + BIASES::R;
}

uint16_t Lasers::ReadFR() {
    return Read(laserFR, ADDRESSES::FR, &frHighPrecision) + BIASES::FR;
}

uint16_t Lasers::ReadF() {
    return Read(laserF, ADDRESSES::F, &fHighPrecision) + BIASES::F;
}

uint16_t Lasers::ReadFL() {
    return Read(laserFL, ADDRESSES::FL, &flHighPrecision) + BIASES::FL;
}

uint16_t Lasers::ReadL() {
    return Read(laserL, ADDRESSES::L, &lHighPrecision) + BIASES::L;
}

uint16_t Lasers::ReadB() {
    return Read(laserB, ADDRESSES::B, &bHighPrecision) + BIASES::B;
}

uint16_t Lasers::ReadFrontMin() {
    auto l = ReadFL(), c = ReadF(), r = ReadFR();
    return min(l, min(c, r));
}

bool Lasers::IsValidWall(const uint16_t l, const uint16_t c, const uint16_t r, const uint16_t tolerance) {
    const uint16_t mean = std::min(r, l) + abs(r - l) / 2;
    return abs(l - r) < dimensions::front_lasers_distance &&
           math::InRange<int64_t>(c, mean - tolerance, mean + tolerance);
}

int16_t Lasers::FrontDifference(const uint16_t l, const uint16_t r, const int16_t bias) {
    return (r - l) + bias;
}

uint16_t Lasers::Read(VL53L0X laser, uint8_t address, bool *highPrecision) {
    changeAddress(address);
    uint16_t measure = continuous ? laser.readRangeContinuousMillimeters() : laser.readRangeSingleMillimeters();
    if (measure >= 60000 || measure <= 0) {
        Driver::Pause();
        Logger::Error(kLasers, "%d died", address);
        GetBus()->begin();
        GetBus()->setClock(10000);
        delay(25);
        Begin();
        if (continuous) StartContinuous();
        changeAddress(address);
        measure = continuous ? laser.readRangeContinuousMillimeters() : laser.readRangeSingleMillimeters();
        Logger::Error(kLasers, "%d: %d", address, measure);
        Driver::Resume();
    }
    return measure;
    if (measure < 600) {
        if (!*highPrecision) {
            Logger::Verbose(kLasers, "Setting %d to high precision", address);
            laser.setHighPrecision();
            *highPrecision = true;
            if (continuous) laser.startContinuous();
            measure = continuous ? laser.readRangeContinuousMillimeters() : laser.readRangeSingleMillimeters();
        }
    } else {
        if (*highPrecision) {
            Logger::Verbose(kLasers, "Setting %d to long range", address);
            laser.setLongRange();
            *highPrecision = false;
            if (continuous) laser.startContinuous();
            measure = continuous ? laser.readRangeContinuousMillimeters() : laser.readRangeSingleMillimeters();
        }
    }
    return measure;
}

uint16_t Lasers::ReadFront() {
    frontCounter = ++frontCounter % 3;
    if (frontCounter == 0) return ReadFL();
    if (frontCounter == 2) return ReadFR();
    return ReadF();
}


#endif
