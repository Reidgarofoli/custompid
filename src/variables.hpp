#include "main.h"

double radToDeg(double a){
    return a * 180 / M_PI;
}
double degToRad(double a){
    return a * M_PI / 180;
}

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Controller info(pros::E_CONTROLLER_PARTNER);
pros::Imu inertial(20); // port 10
pros::Optical colorSensor(14);
pros::Distance distSensor(18);
pros::Distance rDist(16);
pros::Distance fDist(14);

pros::Rotation lifterRotation(12);
//pros::Distance front_wall_dist(3);

pros::Rotation vTracking(-15);
pros::Rotation hTracking(18);

pros::MotorGroup LDrive({-4, -6, 7}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
pros::MotorGroup RDrive({2, -8, 11}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);

pros::MotorGroup lifter({9, -10}, pros::MotorGearset::green, pros::MotorUnits::degrees);
pros::Motor intake(-17, pros::MotorGearset::blue, pros::MotorUnits::degrees);

pros::adi::DigitalOut mogo(8); // hi this is good for you ;) 
pros::adi::DigitalOut doinker(2);

pros::adi::Led leds(4, 17+15);

const int lowPos  = 0;
bool autonHappened;
const int midPos  = 100;
const int highPos = 380;
const int outPos = 500;
int currentPosition = lowPos;
bool colorSort = false;
bool mogoValue = false;
bool doinkVal = false;
int auton = 0;
int maxauto = 6;
char team = 'r';
bool confirm = false;
int page = 0;
int pagenums = 2;
int ejectDelay = 50;
std::string space = "                                ";




struct Pose {
    double x;
    double y;
    double theta;

    Pose (double x, double y){
        this->x=x;
        this->y=y;
    }
    Pose (double x, double y, double theta){
        this->x=x;
        this->y=y;
        this->theta=theta;
    }

    Pose operator=(Pose a) {
        x=a.x;
        y=a.y;
        theta=a.theta;
        return a;
    }

    Pose operator+(Pose a) {
        return {a.x+x,a.y+y};
    }
    Pose operator-(Pose a) {
        return {a.x-x,a.y-y};
    }
    float operator*(const Pose& other) const {
        return x * other.x + y * other.y;
    }
    Pose operator*(const float& other) const {
        return Pose(x * other, y * other, theta);
    }

    bool operator==(Pose a) {
       if (a.x==x && a.y==y && a.theta==theta)
          return true;
       else
          return false;
    }
    Pose lerp(Pose other, float t) const {
        return Pose(x + (other.x - x) * t, y + (other.y - y) * t, theta);
    }

    float distance(Pose other) const { return std::hypot(x - other.x, y - other.y); }

    float angle(Pose other) const { return -radToDeg(std::atan2(other.y - y, other.x - x)) + 90; }

    Pose rotate(float angle) const {
        return Pose(x * std::cos(angle) - y * std::sin(angle),
                            x * std::sin(angle) + y * std::cos(angle), theta);
    }
};

struct ExitCondition {
    const float range;
    const int time;
    int startTime = -1;
    bool done = false;
    ExitCondition(const float range, const int time)
        : range(range),
        time(time) {}

    bool getExit() { return done; }

    bool update(const float input) {
        const int curTime = pros::millis();
        if (std::fabs(input) > range) startTime = -1;
        else if (startTime == -1) startTime = curTime;
        else if (curTime >= startTime + time) done = true;
        return done;
    }

    void reset() {
        startTime = -1;
        done = false;
    }
};
class Timer {
    private:
        uint32_t period;
        uint32_t lastTime;
        uint32_t timeWaited = 0;
        bool paused = false;
    public:
        Timer(uint32_t time)
            : period(time) {
            lastTime = pros::millis();
        }
        
        uint32_t getTimeSet() {
            const uint32_t time = pros::millis(); // get time from RTOS
            if (!paused) timeWaited += time - lastTime; // don't update if paused
            lastTime = time; // update last time
            return period;
        }

        uint32_t getTimeLeft() {
            const uint32_t time = pros::millis(); // get time from RTOS
            if (!paused) timeWaited += time - lastTime; // don't update if paused
            lastTime = time; // update last time
            const int delta = period - timeWaited; // calculate how much time is left
            return (delta > 0) ? delta : 0; // return 0 if timer is done
        }

        uint32_t getTimePassed() {
            const uint32_t time = pros::millis(); // get time from RTOS
            if (!paused) timeWaited += time - lastTime; // don't update if paused
            lastTime = time; // update last time;
            return timeWaited;
        }

        bool isDone() {
            const uint32_t time = pros::millis(); // get time from RTOS
            if (!paused) timeWaited += time - lastTime; // don't update if paused
            lastTime = time; // update last time
            const int delta = period - timeWaited; // calculate how much time is left
            return delta <= 0;
        }

        bool isPaused() {
            const uint32_t time = pros::millis(); // get time from RTOS
            if (!paused) timeWaited += time - lastTime; // don't update if paused
            return paused;
        }

        void set(uint32_t time) {
            period = time; // set how long to wait
            reset();
        }

        void reset() {
            timeWaited = 0;
            lastTime = pros::millis();
        }

        void pause() {
            if (!paused) lastTime = pros::millis();
            paused = true;
        }

        void resume() {
            if (paused) lastTime = pros::millis();
            paused = false;
        }

        void waitUntilDone() {
            do pros::delay(5);
            while (!this->isDone());
        }
};