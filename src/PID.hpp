#include "main.h"
#include <iostream>
#include "variables.hpp"
#include <ctime>
using namespace std;

class PIDcontroller {
	public:
    	float kp;
		float ki;
    	float kd;

		PIDcontroller(float kp2,float ki2, float kd2){
			kp = kp2;
			kd = kd2;
			ki = ki2;
		}
    	float derivative;
		float integrator;
    	float previousError;

    	float PID(float error,float minspeed = 0, float maxspeed = 127){
    	    derivative = error - previousError;
			integrator += ki*error;
			// if (maxspeed - (kp*error + kd*derivative) < ki*integrator){
			// 	integrator = (maxspeed - (kp*error + kd*derivative))/ki;
			// }
			previousError = error;
    	    float speed = kp*error + kd*derivative + integrator;
			return speed;
    	}

		void resetPID(){
			previousError = 0;
			integrator = 0;
			LDrive.move(0);
			RDrive.move(0);
		}
};

//drivetrain has 12.5" track width, 
//      3.25" wheels 450 rpm

//vertical tracking wheel is 0.1686" left of the center
//horizontal tracking wheel is 1.7485" back from center
//turning pid: 5, 0, 48
//lateral pid: 12, 0, 70
double Tr = 0.1686; // Distance between tracking point and vertical tracking wheel
double Ts = 1.7485; // Distance between tracking point and horizontal tracking wheel
PIDcontroller angularController(
	5, // kP
	0, // kI
	48 // kD
);
PIDcontroller mogoAngularController(
	5, // kP
	0, // kI
	48 // kD
);
ExitCondition angularSmallExit(1, 100);
ExitCondition angularLargeExit(3, 500);

PIDcontroller lateralController(
	12, // kP
	0, // kI
	70 // kD
);
PIDcontroller mogoLateralController(
	12, // kP
	0, // kI
	70 // kD
);

ExitCondition lateralSmallExit(1, 100);
ExitCondition lateralLargeExit(1, 500);


enum class AngularDirection {
    CW_CLOCKWISE, /** turn clockwise */
    CCW_COUNTERCLOCKWISE, /** turn counter-clockwise */
    AUTO /** turn in the direction with the shortest distance to target */
};
constexpr float sanitizeAngle(float angle, bool radians) {
    if (radians) return std::fmod(std::fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    else return std::fmod(std::fmod(angle, 360) + 360, 360);
}
float angleError(float target, float position, bool radians = true, AngularDirection direction = AngularDirection::AUTO) {
    // bound angles from 0 to 2pi or 0 to 360
    target = sanitizeAngle(target, radians);
    target = sanitizeAngle(target, radians);
    const float max = radians ? 2 * M_PI : 360;
    const float rawError = target - position;
    switch (direction) {
        case AngularDirection::CW_CLOCKWISE: // turn clockwise
            return rawError < 0 ? rawError + max : rawError; // add max if sign does not match
        case AngularDirection::CCW_COUNTERCLOCKWISE: // turn counter-clockwise
            return rawError > 0 ? rawError - max : rawError; // subtract max if sign does not match
        default: // choose the shortest path
            return std::remainder(rawError, max);
    }
}
float clamp(float val, float mn, float mx){
	return std::max(std::min(val, mx), mn);
}

float wheelDiameter = 2;
// setting up tracking distances and variables
double lastAngle = 0, currentAngle = 0;
double lastH = 0, currentH = 0;  // current and last rotations of the tracking wheels (horizontal)
double lastV = 0, currentV = 0;  // current and last rotations of the tracking wheels (vertical)
double deltaXLocal = 0, deltaYLocal = 0;
double deltaXGlobal = 0, deltaYGlobal = 0;
double xPosGlobal = 0, yPosGlobal = 0;
double deltaDistS = 0, deltaDistR = 0;
double currentAbsoluteOrientation = 0;
double deltaTheta = 0, avgHeading = 0;
Pose currentPos(0,0,0);
Pose lastPose(0,0,0);
float localX = 0;
float localY = 0;
float lastInertial = 0;
float lastVTracking = 0;
float lastHTracking = 0;

float getAngle(bool radians){
    if (radians){
        return degToRad(currentPos.theta);
    } else {
        return currentPos.theta;
    }
}

// new tracking
float LR = 0, CR = 0, DR = 0;
float LB = 0, CB = 0, DB = 0;
float LA = 0, CA = 0, DA = 0;
float DLX = 0, DLY = 0;
float DGX = 0, DGY = 0;
float Dm = 0;
float getX(){
    return currentPos.x;
}
float getY(){
    return currentPos.y;
}

// new tracking
void tracking(){
    currentPos.x = 0;
    currentPos.y = 0;
    currentPos.theta = 0;
    while (true) {
        currentPos.x = -xPosGlobal;
        currentPos.y = yPosGlobal;
        currentPos.theta += inertial.get_rotation() - lastInertial;
        lastInertial = inertial.get_rotation();

		// pros::lcd::print(0, "x: %f", getX());
        // pros::lcd::print(1, "y: %f", getY());
        // pros::lcd::print(2, "theta: %f", getAngle(false));

        currentAbsoluteOrientation = degToRad(inertial.get_rotation()); // converts to radians
        deltaTheta = currentAbsoluteOrientation - lastAngle;

        currentH = float(hTracking.get_position()) * M_PI / 36000;
        currentV = float(vTracking.get_position()) * M_PI / 36000;
        deltaDistS = (currentH - lastH) * wheelDiameter; // delta y
        deltaDistR = (currentV - lastV) * wheelDiameter; // delta x
        lastH = currentH;
        lastV = currentV;
        avgHeading = lastAngle + (deltaTheta / 2);
        lastAngle = currentAbsoluteOrientation;

        float deltaX = deltaDistS;
        float deltaY = deltaDistR;

        // calculate local x and y
        localX = 0;
        localY = 0;
        if (deltaTheta == 0) { // prevent divide by 0
            localX = deltaX;
            localY = deltaY;
        } else {
            localX = (2 * sin(deltaTheta / 2)) * (deltaX / deltaTheta + Ts);
            localY = (2 * sin(deltaTheta / 2)) * (deltaY / deltaTheta + Tr);
        }

        // calculate global x and y
        xPosGlobal += localY * sin(avgHeading);
        yPosGlobal += localY * cos(avgHeading);
        xPosGlobal += localX * -cos(avgHeading);
        yPosGlobal += localX * sin(avgHeading);

        pros::delay(10);
    }
}
int closestNum(std::vector<int> nums, float number){
    int closest;
    float diff;
    for (int i = 0; i < nums.size(); i++){
        if (fabs(angleError(nums[i], number, false)) < diff || i == 0){
            closest = nums[i];
            diff = fabs(angleError(nums[i], number, false));
        }
    }
    return closest;
}
std::vector<int> angles;
void initializeTracking(){
    angles.push_back(0);
    angles.push_back(90);
    angles.push_back(180);
    angles.push_back(270);
    vTracking.set_position(0);
    hTracking.set_position(0);
    inertial.reset(true);
    while (inertial.is_calibrating()){}
    inertial.set_rotation(0);
    while (inertial.is_calibrating()){}
    pros::Task trackingTask(tracking, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT, "tracking");
}
void setPose(float x, float y, float theta){
    xPosGlobal = x;
    yPosGlobal = y;
    currentPos.theta = theta;
}
void resetPositionFromDistance(int offsetX, int offsetY){
    switch(closestNum(angles, currentPos.theta)){
        case 0:
            setPose((float)lDist.get()/25.4, (float)fDist.get()/-25.4, getAngle(false));
            break;
        case 90:
            setPose((float)fDist.get()/-25.4, (float)lDist.get()/25.4, getAngle(false));
            break;
        case 180:
            setPose((float)lDist.get()/-25.4, (float)fDist.get()/25.4, getAngle(false));
            break;
        case 270:
            setPose((float)fDist.get()/25.4, (float)lDist.get()/-25.4, getAngle(false));
            break;
    }
}


struct TurnToHeadingParams {
    /** the direction the robot should turn in. AUTO by default */
    AngularDirection direction = AngularDirection::AUTO;
    /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
    int maxSpeed = 127;
    /** the minimum speed the robot can turn at. If set to a non-zero value, the `it conditions will switch to less
     * accurate but smoother ones. Value between 0-127. 0 by default */
    int minSpeed = 0;
    /** angle between the robot and target point where the movement will exit. Only has an effect if minSpeed is
     * non-zero.*/
    float earlyExitRange = 0;
};
enum class DriveSide {
    LEFT, /** lock the left side of the drivetrain */
    RIGHT /** lock the right side of the drivetrain */
};
struct SwingToPointParams {
    /** whether the robot should turn to face the point with the front of the robot. True by default */
    bool forwards = true;
    /** the direction the robot should turn in. AUTO by default */
    AngularDirection direction = AngularDirection::AUTO;
    /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
    float maxSpeed = 127;
    /** the minimum speed the robot can turn at. If set to a non-zero value, the exit conditions will switch to less
     * accurate but smoother ones. Value between 0-127. 0 by default */
    float minSpeed = 0;
    /** angle between the robot and target heading where the movement will exit. Only has an effect if minSpeed is
     * non-zero.*/
    float earlyExitRange = 0;
};
struct SwingToHeadingParams {
    /** the direction the robot should turn in. AUTO by default */
    AngularDirection direction = AngularDirection::AUTO;
    /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
    float maxSpeed = 127;
    /** the minimum speed the robot can turn at. If set to a non-zero value, the exit conditions will switch to less
     * accurate but smoother ones. Value between 0-127. 0 by default */
    float minSpeed = 0;
    /** angle between the robot and target heading where the movement will exit. Only has an effect if minSpeed is
     * non-zero.*/
    float earlyExitRange = 0;
};
struct MoveToPointParams {
    /** whether the robot should move forwards or backwards. True by default */
    bool forwards = true;
    /** the maximum speed the robot can travel at. Value between 0-127. 127 by default */
    float maxSpeed = 127;
    /** the minimum speed the robot can travel at. If set to a non-zero value, the exit conditions will switch to
     * less accurate but smoother ones. Value between 0-127. 0 by default */
    float minSpeed = 0;
    /** distance between the robot and target point where the movement will exit. Only has an effect if minSpeed is
     * non-zero.*/
    float earlyExitRange = 0;
};

/* MOVING FUNCTIONS */
template <typename T> constexpr T sgn(T value) { return value < 0 ? -1 : 1; }
void turnToHeading(float theta, int timeout, TurnToHeadingParams params, bool async) {
    params.minSpeed = std::abs(params.minSpeed);
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { turnToHeading(theta, timeout, params, false); });
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float targetTheta;
    float deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getAngle(false);
    bool settling = false;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    std::uint8_t compState = pros::competition::get_status();
    Timer timer(timeout);

    PIDcontroller* angularPID;
    if (mogoValue){
        angularPID = &mogoAngularController;
    } else {
        angularPID = &angularController;
    }
    angularPID->resetPID();

    lateralLargeExit.reset();
    lateralSmallExit.reset();

    // main loop
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit()) {
        // update variables
        Pose pose = {getX(), getY(), getAngle(false)};


        targetTheta = theta;

        // check if settling
        const float rawDeltaTheta = angleError(targetTheta, pose.theta, false);
        if (prevRawDeltaTheta == std::nullopt) prevRawDeltaTheta = rawDeltaTheta;
        if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta)) settling = true;
        prevRawDeltaTheta = rawDeltaTheta;

        // calculate deltaTheta
        if (settling) deltaTheta = angleError(targetTheta, pose.theta, false);
        else deltaTheta = angleError(targetTheta, pose.theta, false, params.direction);
        if (prevDeltaTheta == std::nullopt) prevDeltaTheta = deltaTheta;

        // motion chaining
        if (params.minSpeed != 0 && fabs(deltaTheta) < params.earlyExitRange) break;
        if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta)) break;

        // calculate the speed
        motorPower = angularPID->PID(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // cap the speed
        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;


        LDrive.move(motorPower);
        RDrive.move(-motorPower);

        pros::delay(10);
    }

    // stop the drivetrain
    LDrive.move(0);
    RDrive.move(0);
}

void moveToPoint(float x, float y, int timeout, MoveToPointParams params, bool async) {
    params.earlyExitRange = fabs(params.earlyExitRange);
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveToPoint(x, y, timeout, params, false); });
        pros::delay(10); // delay to give the task time to start
        return;
    }
    
    Pose lastPose = Pose(getX(), getY(), getAngle(false));
    Timer timer(timeout);
    bool close = false;
    float prevLateralOut = 0; // previous lateral power
    float prevAngularOut = 0; // previous angular power
    const int compState = pros::competition::get_status();
    std::optional<bool> prevSide = std::nullopt;

    Pose target(x, y);
    target.theta = lastPose.angle(target);

    // reset PIDs and exit conditions
    PIDcontroller* lateralPID;
    if (mogoValue){
        lateralPID = &mogoLateralController;
    } else {
        lateralPID = &lateralController;
    }
    lateralPID->resetPID();

    PIDcontroller* angularPID;
    if (mogoValue){
        angularPID = &mogoAngularController;
    } else {
        angularPID = &angularController;
    }
    angularPID->resetPID();

    lateralLargeExit.reset();
    lateralSmallExit.reset();

    // main loop
    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close)) {
        // update position

        // update distance traveled
        lastPose = currentPos;

        // calculate distance to the target point
        const float distTarget = currentPos.distance(target);

        // check if the robot is close enough to the target to start settling
        if (distTarget < 7.5 && close == false) {
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60);
        }

        // motion chaining
        const bool side =
            (currentPos.y - target.y) * -sin(target.theta) <= (currentPos.x - target.x) * cos(target.theta) + params.earlyExitRange;
        if (prevSide == std::nullopt) prevSide = side;
        const bool sameSide = side == prevSide;
        // exit if close
        if (!sameSide && params.minSpeed != 0) break;
        prevSide = side;

        // calculate error
        const float adjustedRobotTheta = params.forwards ? currentPos.theta : currentPos.theta + M_PI;
        const float angularError = angleError(adjustedRobotTheta, currentPos.angle(target));
        float lateralError = currentPos.distance(target) * cos(angleError(currentPos.theta, currentPos.angle(target)));

        // update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        // get output from PIDs
        float lateralOut = lateralPID->PID(lateralError);
        float angularOut = angularPID->PID(radToDeg(angularError));
        if (close) angularOut = 0;

        // apply restrictions on angular speed
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);

        // apply restrictions on lateral speed
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

        // prevent moving in the wrong direction
        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

        // constrain lateral output by the minimum speed
        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

        // update previous output
        prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

        // ratio the speeds to respect the max speed
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // move the drivetrain
        LDrive.move(leftPower);
        RDrive.move(rightPower);

        // delay to save resources
        pros::delay(10);
    }

    // stop the drivetrain
    LDrive.move(0);
    RDrive.move(0);
    pros::delay(10);
}