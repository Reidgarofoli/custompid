#include "main.h"
#include "lemlib/api.hpp"

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

lemlib::Drivetrain drivetrain {
	&LDrive, // left motor group
	&RDrive, // right motor group
	12.5, // 14.5 inch track width
	lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
	450, // drivetrain rpm is 450
	8 // Its 8 because we have traction wheels
};

// left tracking using drive motors
lemlib::TrackingWheel vertical_tracking(
	&vTracking, //Look at the left drive
	2.0, //2 inch tracking wheels
	-0.1686 //0.25 incheas left of the center
);

// right tracking using drive motors
lemlib::TrackingWheel horizontal_tracking(
	&hTracking, //Look at the left drive
	2.0, //2 inch tracking wheels
	1.7485 //1.75 incheas back of the center
);

		
// Sensors for odometry 
lemlib::OdomSensors sensors {
	&vertical_tracking, // vertical tracking wheel 1
	nullptr, // vertical tracking wheel 2
	&horizontal_tracking, // horizontal tracking wheel
	nullptr, // no second horizontal tracking wheel
	&inertial // inertial sensor
};


// forward/backward PID
lemlib::ControllerSettings lateralController { 
	12, // proportional gain (kP)
	0, // integral gain (kI)
	60, // derivative gain (kD)
	0, // anti windup
	0.5, // small error range, in inches
	100, // small error range timeout, in milliseconds
	2.5, // large error range, in inches
	500, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
};

// turning PID
lemlib::ControllerSettings angularController {
	5, // proportional gain (kP)
	0, // integral gain (kI)
	48, // derivative gain (kD)
	0, // anti windup
	1, // small error range, in inches
	100, // small error range timeout, in milliseconds
	3, // large error range, in inches
	500, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);


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