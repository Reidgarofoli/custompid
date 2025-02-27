#include "main.h"
#include "variables.hpp"
std::vector<int> angles;

constexpr float sanitizeAngle(float angle, bool radians) {
    if (radians) return std::fmod(std::fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    else return std::fmod(std::fmod(angle, 360) + 360, 360);
}
float angleError(float target, float position, bool radians = false, AngularDirection direction = AngularDirection::AUTO) {
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
void posecheck(){
    int Rclosenum = closestNum(angles, chassis.getPose().theta + 90);
    float Rangerror = angleError(chassis.getPose().theta + 90, Rclosenum);
    float rDistance = cos((rDist.get() / 25.4)) * Rangerror;
    
    int closenum = closestNum(angles, chassis.getPose().theta);
    float fangerror = angleError(chassis.getPose().theta + 90, closenum);
    float fDistance = cos((fDist.get() / 25.4)) * fangerror;
    //
    //
    //
    rDistance = rDist.get() / 25.4;
    fDistance = fDist.get() / 25.4;
    
    
    // closenum = closestNum(angles, chassis.getPose().theta);
    if (closenum == 0){
        chassis.setPose(-rDistance, -fDistance, chassis.getPose().theta);
    }
    if (closenum == 90){
        chassis.setPose(-fDistance, rDistance, chassis.getPose().theta);
    }
    if (closenum == 180){
        chassis.setPose(rDistance, fDistance, chassis.getPose().theta);
    }
    if (closenum == 270){
        chassis.setPose(fDistance, -rDistance, chassis.getPose().theta);
    }
    
    
    // if (closenum == 0){
    //     chassis.setPose(-rDistance, -fDistance, chassis.getPose().theta);
    // }
    // if (closenum == 90){
    //     chassis.setPose(-fDistance, rDistance, chassis.getPose().theta);
    // }
    // if (closenum == 180){
    //     chassis.setPose(rDistance, fDistance, chassis.getPose().theta);
    // }
    // if (closenum == 270){
    //     chassis.setPose(fDistance, -rDistance, chassis.getPose().theta);
    // }
}
void updatePosFromSensors(bool left, bool right, bool front, bool back){
    
}
void graphing(){
    lemlib::Pose currentPos = chassis.getPose();
    pros::screen::set_pen(0xffffff);
    pros::screen::fill_rect(0,0, 480, 240);
    pros::screen::set_pen(0x000000);
    pros::screen::draw_line(0, -180+240, 480, -180+240);
    pros::screen::set_pen(0x00ff00);
    pros::screen::draw_line(0, 120, 480, 120);
    int x = 0, startTime, delayTime;
    while (true) {
        startTime = pros::millis();
        currentPos = chassis.getPose();
        pros::screen::set_pen(0x0000ff);
        pros::screen::draw_pixel(x, -currentPos.theta+240);
        pros::screen::set_pen(0xff0000);
        pros::screen::draw_pixel(x, -currentPos.y*6+240);
        pros::screen::set_pen(0x00ff00);
        pros::screen::draw_pixel(x, -(LDrive.get_voltage()/1000)+120);
        x++;
        delayTime = std::max((pros::millis() - startTime)-16.6666, 10.0);
        pros::delay(delayTime);
    }
}
void moveToDistance(float distance, float angleToHold, int timeout, MoveToDistanceParams params = {}, bool async = true){
    int targetDist = distance * 25.4;
    params.earlyExitRange = fabs(params.earlyExitRange);
    chassis.requestMotionStart();
    // were all motions cancelled?
    if (!chassis.motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveToDistance(distance, angleToHold, timeout, params, false); });
        chassis.endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    chassis.lateralPID.reset();
    chassis.lateralLargeExit.reset();
    chassis.lateralSmallExit.reset();
    chassis.angularPID.reset();

    Timer timer(timeout);
    float lateralOut;
    float angularOut;
    float error;
    while (!timer.isDone() && (!chassis.lateralSmallExit.getExit() && !chassis.lateralLargeExit.getExit()) && chassis.motionRunning) {
        switch (params.whichDist){
            case 'f':
                error = fDist.get() - targetDist;
                break;
            case 'l':
                error = lDist.get() - targetDist;
                break;
            case 'r':
                error = rDist.get() - targetDist;
                break;
        }
        
        lateralOut = chassis.lateralPID.update(error);
        angularOut = chassis.angularPID.update(lemlib::angleError(angleToHold, chassis.getPose().theta));

        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;

        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        LDrive.move(leftPower);
        RDrive.move(rightPower);
        pros::delay(10);
    }
    // stop the drivetrain
    LDrive.move(0);
    RDrive.move(0);

    chassis.endMotion();
}

void auton6(){//SKILLSSSSS
    if (team == 'r'){
        chassis.setPose(0,0,0);
        currentPosition=midPos;//
        pros::delay(500 );//
        intake.move(127);//
        pros::delay(250);//
        //first part bottom left mogo
        currentPosition = outPos + 20;
        intake.move(-30);//
        pros::delay(400);
        intake.move(0);//
        chassis.moveToPoint(0, -5, 1000, {}, false);
        currentPosition = lowPos;
        chassis.turnToHeading(270, 1000, {}, false);
        chassis.moveToPoint(20, -4, 1000, {.forwards = false, .maxSpeed=50, .minSpeed = 30, .earlyExitRange = 1}, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.turnToHeading(180, 1000, {}, false);
        intake.move(127);
        chassis.moveToPoint(22, -25, 1000, {}, false);
        pros::delay(500);

        //first high stake
        chassis.turnToHeading(135, 1000, {}, false);
        currentPosition = midPos;
        intake.move(127);
        chassis.moveToPoint(40, -45, 2000, {.minSpeed=45, .earlyExitRange=2}, false);
        chassis.swingToHeading(90, DriveSide::LEFT, 1000, {}, false);
        chassis.moveToPoint(60, chassis.getPose().y, 2000, {}, false);
        pros::delay(500);
        currentPosition = highPos;
        intake.move(-30);
        pros::delay(500);

        //secound part bottom left mogo 
        chassis.moveToPoint(43, -50, 1000, {.forwards = false}, false);
        chassis.turnToHeading(0, 1000, {}, false);
        intake.move(127);
        chassis.moveToPoint(47, -21, 1000, {.maxSpeed=60}, false);
        chassis.moveToPoint(47, -7, 1000, {.maxSpeed=60}, false);
        chassis.moveToPoint(46, 8, 1000, {.maxSpeed=60}, false);
        pros::delay(200);
        chassis.turnToHeading(0, 1000, {}, false);
        chassis.setPose(-lDist.get_distance()/25.4, chassis.getPose().y, chassis.getPose().theta);
        chassis.moveToPoint(-20, -4,3000, {.forwards=false}, false);
        chassis.turnToHeading(225, 1000, {}, false);
        chassis.moveToPoint(-4,10,4000,{.forwards=false}, false);
        mogoValue = false;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.moveToPoint(-34,0,4000,{},false);
        chassis.turnToHeading(-90,1000,{},false);
        chassis.setPose(chassis.getPose().x, -lDist.get_distance()/25.4, chassis.getPose().theta);
        currentPosition=lowPos;
        chassis.turnToHeading(90,1000,{},false);
        chassis.moveToPoint(-88,-17,4000,{.forwards=false,.maxSpeed=50},false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.turnToHeading(180,1000,{},false);
        intake.move(127);
        chassis.moveToPoint(-89, -36, 4000, {}, false);
        chassis.turnToHeading(-145, 1000, {}, false);
        chassis.moveToPoint(-105,-60, 4000, {}, false);
        chassis.swingToHeading(270, DriveSide::RIGHT, 1000, {}, false);
        currentPosition = midPos;
        chassis.moveToPoint(-124,chassis.getPose().y, 4000, {}, false);
        pros::delay(500);
        currentPosition = highPos;
        intake.move(-30);
        pros::delay(500);
        chassis.moveToPoint(-110, chassis.getPose().y+1, 2000, {.forwards=false}, false);
        chassis.turnToHeading(0, 1000, {}, false);
        intake.move(127);
        chassis.moveToPoint(-115,-19, 7000, {}, false);
        chassis.moveToPoint(-115,-3, 1000, {}, false);
        chassis.moveToPoint(-115,-18,3000,{.forwards=false}, false);
        chassis.swingToHeading(110,DriveSide::RIGHT, 2000, {},false);
        chassis.moveToPoint(-127,-4,3000,{.forwards=false}, false);
        mogoValue = false;
        mogo.set_value(mogoValue);
        pros::delay(200);//2nd goal in corner  
        intake.move(0);
        chassis.moveToPoint(-110,-10,2000,{},false);
    } else {
        chassis.setPose(0,0,0);
    }
}
void auton0(){ // solo win point not sig
    if (team == 'r'){
        chassis.setPose(0,0,270);
        currentPosition = midPos;
        chassis.moveToPoint(-3, 0, 1000, {}, false);
        intake.move(127);
        chassis.swingToHeading(210, DriveSide::LEFT, 1000, {}, false);
        intake.move(-30);
        currentPosition = outPos+15;
        pros::delay(500);
        chassis.moveToPoint(-0.5, 10, 2000, {.forwards=false}, false);
        chassis.swingToHeading(247, DriveSide::RIGHT, 1000, {}, false);
        doinker.set_value(true);
        pros::delay(200);
        chassis.swingToHeading(180, DriveSide::RIGHT, 1000, {}, false);
        doinker.set_value(false);
        pros::delay(200);
        
        chassis.moveToPoint(3, 30, 1000, {.forwards=false, .maxSpeed=50, .minSpeed=30, .earlyExitRange=1}, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        intake.move(127);
        chassis.moveToPoint(-3, 0, 1000, {}, false);
        chassis.moveToPoint(-3, 20, 1000, {.forwards = false}, false);
        chassis.turnToHeading(90, 1000, {}, false);
        chassis.moveToPoint(22, 25, 1000, {}, false);
        currentPosition = 290;
        chassis.turnToHeading(270, 1000, {}, false);
        chassis.moveToPoint(0, 31, 1000, {}, false);
        currentPosition = highPos;
    } else {
        chassis.setPose(0,0,270);
        currentPosition = midPos;
        chassis.moveToPoint(-3, 0, 1000, {}, false);
        intake.move(127);
        chassis.swingToHeading(210, DriveSide::LEFT, 1000, {}, false);
        intake.move(-30);
        currentPosition = outPos+15;
        pros::delay(500);
        chassis.moveToPoint(-0.5, 10, 2000, {.forwards=false}, false);
        chassis.swingToHeading(247, DriveSide::RIGHT, 1000, {}, false);
        doinker.set_value(true);
        pros::delay(200);
        chassis.swingToHeading(180, DriveSide::RIGHT, 1000, {}, false);
        doinker.set_value(false);
        pros::delay(200);
        
        chassis.moveToPoint(3, 30, 1000, {.forwards=false, .maxSpeed=50, .minSpeed=30, .earlyExitRange=1}, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        intake.move(127);
        chassis.moveToPoint(-3, 0, 1000, {}, false);
        chassis.moveToPoint(-3, 20, 1000, {.forwards = false}, false);
        chassis.turnToHeading(90, 1000, {}, false);
        chassis.moveToPoint(22, 25, 1000, {}, false);
        currentPosition = 290;
        chassis.turnToHeading(270, 1000, {}, false);
        chassis.moveToPoint(0, 31, 1000, {}, false);
        currentPosition = highPos;  
    }
}
void auton1(){ //goal rush OLD

    if (team == 'r'){  //red side goal rush  
        chassis.setPose(0,0,0);
        doinker.set_value(true);
        currentPosition = midPos;
        intake.move(127);
        chassis.moveToPoint(6, 42.5, 2000, {}, false);
        chassis.turnToHeading(77, 1000, {}, false);
        doinker.set_value(false);
        chassis.turnToHeading(81, 1000, {.minSpeed=50, .earlyExitRange=3}, false);
        // chassis.moveToPoint(-1,46.5,4000,{.forwards=false}, false);
        chassis.turnToHeading(81, 1000, {.minSpeed=50, .earlyExitRange=3}, true);
        currentPosition = 665;
        intake.move(-30);
        pros::delay(500);
        chassis.waitUntilDone();
        currentPosition = lowPos;
        chassis.moveToPoint(-14,42,4000,{.forwards=false,.maxSpeed=40, .earlyExitRange=1},false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(100);
        chassis.turnToHeading(-60,1000,{.minSpeed=50, .earlyExitRange=2},false);
        intake.move(0);
        chassis.moveToPoint(-17,48,4000,{},false);
        chassis.turnToHeading(-50,1000,{.minSpeed=30, .earlyExitRange=1},true);
        pros::delay(100);
        doinker.set_value(true);
        chassis.waitUntilDone();
        chassis.moveToPoint(-9, 30, 2000, {.forwards=false, .minSpeed=60, .earlyExitRange=2}, false);
        chassis.turnToHeading(55,1000,{},false);
        doinker.set_value(false);
        intake.move(127);
        chassis.turnToHeading(90,4000,{},false);
        chassis.moveToPoint(0, 28, 4000, {.maxSpeed=50}, false);
        pros::delay(100);

        chassis.moveToPoint(14, 31, 2000, {},false);
        chassis.turnToHeading(-50,2000,{},false);
         mogoValue = false;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.turnToHeading(-150,2000,{},false);
        chassis.moveToPoint(20,51,4000,{.forwards=false,.maxSpeed=50,.earlyExitRange=2},false);
         mogoValue = true;
        mogo.set_value(mogoValue);
    } else {//Blue Side goal rush
        chassis.setPose(0,0,0);
        chassis.moveToPoint(0,30,2000,{.minSpeed=60,.earlyExitRange=2}, false);
        doinkVal = true;
        doinker.set_value(doinkVal);
        chassis.moveToPoint(1,42,2000,{}, false);
        chassis.turnToHeading(90,1000,{},false);
        doinkVal=false;
        doinker.set_value(doinkVal);
        pros::delay(200);
        
        chassis.turnToHeading(270,1000,{},false);
        chassis.moveToPoint(21,46,2000,{.forwards = false,.maxSpeed = 50 }, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        intake.move(127);

        doinkVal = true;
        doinker.set_value(doinkVal);
        chassis.swingToHeading(-165, DriveSide::LEFT, 200,{},false);
        chassis.moveToPoint(-6,6,2000,{},false);
        mogoValue = false;
        mogo.set_value(mogoValue);
        chassis.turnToHeading(90,1000,{},false);
        intake.move(0);
        doinkVal = false;
        doinker.set_value(doinkVal);
        pros::delay(200);
        chassis.turnToHeading(270,1000,{},false);
        intake.move(-127);
        chassis.moveToPoint(25, 10, 2000, {.forwards=false, .minSpeed=100, .earlyExitRange=2}, false);
        chassis.moveToPoint(33, 37, 2000, {.forwards=false, .minSpeed=100, .earlyExitRange=2}, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        chassis.turnToHeading(215,1000,{},false);
        colorSort=true;
        chassis.moveToPoint(11, 30, 2000, {.earlyExitRange=2}, false);
       

        //chassis.turnToHeading(80,1000,{},false);
        // chassis.moveToPoint(3, 41, 2000, {.forwards=false, .earlyExitRange=2}, false);
        // intake.move(127);
        // chassis.moveToPoint(9, 30, 2000, {.earlyExitRange=2}, false);

    }
}

void auton2(){ //JOHN BAPST GOAL RUSH
    if (team == 'r'){
        chassis.setPose(0,0,0);
        doinker.set_value(true);
        chassis.moveToPoint(6, 42.5, 2000, {}, false);
        chassis.turnToHeading(77, 1000, {}, false);
        doinker.set_value(false);
        chassis.turnToHeading(81, 1000, {.minSpeed=50, .earlyExitRange=3}, false);
        // chassis.moveToPoint(-1,46.5,4000,{.forwards=false}, false);
        chassis.turnToHeading(81, 1000, {.minSpeed=50, .earlyExitRange=3}, false);
        chassis.moveToPoint(-14,42,4000,{.forwards=false,.maxSpeed=40, .earlyExitRange=1},false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(100);
        chassis.turnToHeading(-60,1000,{.minSpeed=50, .earlyExitRange=2},false);
        intake.move(0);
        chassis.moveToPoint(-17,48,4000,{},false);
        chassis.turnToHeading(-50,1000,{.minSpeed=30, .earlyExitRange=1},true);
        pros::delay(100);
        doinker.set_value(true);
        chassis.waitUntilDone();
        chassis.moveToPoint(-9, 30, 2000, {.forwards=false, .minSpeed=60, .earlyExitRange=2}, false);
        
        doinker.set_value(false);
        pros::delay(200);
        intake.move(127);
        chassis.moveToPoint(-5,40,4000,{},false);
        chassis.turnToHeading(180,4000,{},false);
        chassis.moveToPoint(-11,13,4000,{},false);
        
        chassis.turnToHeading(270,1000,{},false);
        mogoValue = false;
        mogo.set_value(mogoValue);
        pros::delay(200);
        
        chassis.turnToHeading(270, 1000, {}, false);
        chassis.turnToHeading(180, 1000, {}, false);

        chassis.moveToPoint(-3, 40, 4000, {.forwards=false, .minSpeed=90, .earlyExitRange=2}, false);
        chassis.moveToPoint(19, 50, 4000, {.forwards=false, .maxSpeed=50}, false);

        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.turnToHeading(180, 4000, {}, false);
        colorSort = true;
        chassis.moveToPoint(16, 21, 3000, {}, false);


    } else {       
        chassis.setPose(0, 0, 180);

    }   
}
void auton3(){ // sig solo win point
    if (team == 'r'){
        chassis.setPose(0, 0, 225);
        currentPosition = midPos;
        pros::delay(100);
        intake.move(127);
        chassis.moveToPoint(-2.5, -2.5, 1000, {}, false);
        intake.move(-30);
        currentPosition = outPos;
        pros::delay(500);
        chassis.moveToPoint(7, 27, 2000, {.forwards=false, .maxSpeed=60, .minSpeed=40, .earlyExitRange=2}, false);
        currentPosition = lowPos;
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.turnToHeading(90, 1000, {.minSpeed=50, .earlyExitRange=3}, false);
        intake.move(127);
        chassis.moveToPoint(25, 30, 2000, {.minSpeed=50, .earlyExitRange=2}, false);
        chassis.turnToHeading(235, 1000, {.minSpeed=50, .earlyExitRange=2}, false);
        chassis.moveToPoint(8, 16, 3000, {.minSpeed=90, .earlyExitRange=3}, false);
        mogoValue = false;
        mogo.set_value(mogoValue);
        intake.move(0);
        ringsTillIntake = 2;
        intakeRingToIntake = true;
        chassis.moveToPoint(-12, 6, 3000, {.maxSpeed=60}, false);
        chassis.moveToPoint(-22, 2, 3000, {}, false);
        chassis.turnToHeading(155, 1000, {}, false);
        chassis.moveToPoint(-40, 28, 3000, {.forwards=false, .maxSpeed=50}, false);
        currentPosition = 160;
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.turnToHeading(270, 1000, {}, false);
        intake.move(127);
        chassis.moveToPoint(-57, 28, 3000, {}, false);
        chassis.turnToHeading(90, 1000, {}, false);
        chassis.moveToPoint(-35, 30, 3000, {}, false);
        currentPosition = highPos + 10;
    } else {
        chassis.setPose(0, 0, 225);
        currentPosition = midPos;
        pros::delay(100);
        intake.move(127);
        chassis.moveToPoint(-2.5, -2.5, 1000, {}, false);
        intake.move(-30);
        currentPosition = outPos;
        pros::delay(500);
        chassis.moveToPoint(7, 27, 2000, {.forwards=false, .maxSpeed=60, .minSpeed=40, .earlyExitRange=2}, false);
        currentPosition = lowPos;
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.turnToHeading(90, 1000, {.minSpeed=50, .earlyExitRange=3}, false);
        intake.move(127);
        chassis.moveToPoint(25, 30, 2000, {.minSpeed=50, .earlyExitRange=2}, false);
        chassis.turnToHeading(235, 1000, {.minSpeed=50, .earlyExitRange=2}, false);
        chassis.moveToPoint(8, 16, 3000, {.minSpeed=90, .earlyExitRange=3}, false);
        mogoValue = false;
        mogo.set_value(mogoValue);
        intake.move(0);
        ringsTillIntake = 2;
        intakeRingToIntake = true;
        chassis.moveToPoint(-12, 6, 3000, {.maxSpeed=60}, false);
        chassis.moveToPoint(-22, 2, 3000, {}, false);
        chassis.turnToHeading(155, 1000, {}, false);
        chassis.moveToPoint(-40, 28, 3000, {.forwards=false, .maxSpeed=50}, false);
        currentPosition = 160;
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.turnToHeading(270, 1000, {}, false);
        intake.move(127);
        chassis.moveToPoint(-57, 28, 3000, {}, false);
        chassis.turnToHeading(90, 1000, {}, false);
        chassis.moveToPoint(-35, 30, 3000, {}, false);
        currentPosition = highPos + 10;
    }
}
void auton4(){// GOAL RUSH STATES ELIMINATION
    if (team == 'r'){
        int startTime = pros::millis();
        chassis.setPose(0,0,0);
        ringsTillIntake = 1;
        doinker.set_value(true);
        chassis.moveToPoint(-9, 32, 2000, {.minSpeed=80, .earlyExitRange=1}, true);
        bool running = true;
        while (chassis.isInMotion() && running){
            if (chassis.getPose().y > 30){
                doinker.set_value(false);
                running = false;
            }
        }
        chassis.waitUntilDone();
        doinker.set_value(false);
        chassis.moveToPoint(-6,20,2000,{.forwards=false}, false);
        doinker.set_value(true);
        pros::delay(200);
        chassis.moveToPoint(-6,15,2000,{.forwards=false, .minSpeed=60, .earlyExitRange=1}, false);
        doinker.set_value(false);
        chassis.turnToHeading(135, 1000, {.minSpeed=80, .earlyExitRange=1}, false);
        chassis.moveToPoint(-30,28, 2000, {.forwards=false, .minSpeed=50, .earlyExitRange=1}, false);
        chassis.moveToPoint(-40,30, 2000, {.forwards=false, .maxSpeed=50}, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        intake.move(127);
        chassis.turnToHeading(-48,1000,{},false);
        intake.move(0);
        chassis.moveToPoint(-50,41.5, 2000, {.maxSpeed=50}, false);
        chassis.swingToHeading(-57, DriveSide::RIGHT,1000,{},true);//it was -53
        doinker.set_value(true);
        chassis.waitUntilDone();
        chassis.moveToPoint(-38, 15, 2000, {.forwards = false, .minSpeed=80, .earlyExitRange=2}, false);
        chassis.turnToHeading(135, 1000, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed=80, .earlyExitRange=4}, false);
        doinker.set_value(false);
        pros::delay(400);
        colorSort = true;
        chassis.moveToPose(-10, -4, 100, 2000, {}, true);
        pros::delay(200);
        doinker.set_value(true);
        chassis.waitUntilDone();
        chassis.turnToHeading(50, 1000, {.minSpeed=127, .earlyExitRange=10});
        doinker.set_value(false);
        colorSort = true;
        chassis.moveToPose(0, 25, 0, 4000, {.maxSpeed=60}, true);
        while ((pros::millis() - startTime)/1000 < 14.9) {

        }
        mogoValue = false;
        mogo.set_value(mogoValue);
    } else {
        int startTime = pros::millis();
        chassis.setPose(0,0,0);
        ringsTillIntake = 1;
        doinker.set_value(true);
        chassis.moveToPoint(-9, 32, 2000, {.minSpeed=80, .earlyExitRange=1}, true);
        bool running = true;
        while (chassis.isInMotion() && running){
            if (chassis.getPose().y > 30){
                doinker.set_value(false);
                running = false;
            }
        }
        chassis.waitUntilDone();
        doinker.set_value(false);
        chassis.moveToPoint(-6,20,2000,{.forwards=false}, false);
        doinker.set_value(true);
        pros::delay(200);
        chassis.moveToPoint(-6,15,2000,{.forwards=false, .minSpeed=60, .earlyExitRange=1}, false);
        doinker.set_value(false);
        chassis.turnToHeading(225, 1500, {}, false);
    }
}

void auton5(){ // PID TUNING
    if (team == 'r'){
        chassis.setPose(0,0,0);
        ringsTillIntake = 1;
        doinker.set_value(true);
        chassis.moveToPoint(-11, 34, 2000, {.minSpeed=80,.earlyExitRange=1}, false);
        intake.move(0);
        chassis.moveToPoint(-6,21,2000,{.forwards=false},false);

        doinker.set_value(false);
        pros::delay(300);
        chassis.swingToHeading(90,DriveSide::RIGHT,2000, {.minSpeed=80, .earlyExitRange=3}, false);
        chassis.moveToPoint(-38,31, 2000, {.forwards=false,.maxSpeed=50,}, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        intake.move(127);
        chassis.turnToHeading(-60,1000,{},false);
        intake.move(0);
        chassis.moveToPoint(-47.5,42.5, 2000, {.maxSpeed=60}, false);
        chassis.turnToHeading(-48,1000,{},false);
        doinker.set_value(true);
        pros::delay(200);
        chassis.swingToHeading(-15, DriveSide::RIGHT, 1000, {}, false);
        ldoink.set_value(true);
        pros::delay(200);
        chassis.moveToPoint(-30, 8, 3000, {.forwards=false}, false);
        doinker.set_value(false);
        ldoink.set_value(false);
        pros::delay(400);
        colorSort=true;
        chassis.turnToHeading(30, 1000, {}, false);
        chassis.swingToHeading(135, DriveSide::LEFT, 1000, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed=127, .earlyExitRange=3}, false);
        chassis.moveToPoint(0, -3, 1000, {.minSpeed=80}, false);
    } else {

    }
}

//PID tuning
// void auton5(){
//     if (team == 'r'){
//         pros::Task graphingTask(graphing);
//         chassis.setPose(0,0,0);
//         chassis.turnToHeading(180, 10000000, {}, false);
//         graphingTask.remove();
//     } else {
//         pros::Task graphingTask(graphing);
//         chassis.setPose(0,0,0);
//         chassis.moveToPoint(0, 30, 10000000, {}, false);
//         graphingTask.remove();
//     }
// }