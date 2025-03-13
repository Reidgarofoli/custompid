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
void driveInches(float distance, int timeout, lemlib::MoveToPointParams params = {}, bool async = true){
    float targetX = chassis.getPose().x + sin(chassis.getPose().theta * M_PI/180)*distance;
    float targetY = chassis.getPose().y + cos(chassis.getPose().theta * M_PI/180)*distance;
    if (distance < 0){
        chassis.moveToPoint(targetX, targetY, timeout, {false, params.maxSpeed, params.minSpeed, params.earlyExitRange}, async);
    } else if (distance > 0){
        chassis.moveToPoint(targetX, targetY, timeout, {true, params.maxSpeed, params.minSpeed, params.earlyExitRange}, async);
    }
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
        pros::delay(200);//2nd goal in corner   //edits i made thursday afternoon is everything below here - crosby
        intake.move(0);
        chassis.moveToPoint(-110,-10,2000,{},false);
        chassis.turnToHeading(0,2000,{},false);
        chassis.moveToPoint(-97,-107,4000,{.forwards=false},false);
        chassis.turnToHeading(-45,1000,{},false);
        chassis.moveToPoint(-85,-123,2000,{.forwards=false},false);
         mogoValue = true;//grab first blue goal - doenst actually work 
        mogo.set_value(mogoValue);
        chassis.turnToHeading(-288,1000,{},false);
        chassis.moveToPoint(-130,-122,4000,{.forwards=false},false);
        pros::delay(100);
        mogoValue = false;
        mogo.set_value(mogoValue);//let go 1st blue goal  
        pros::delay(300);
        chassis.moveToPoint(-10,-122,3000,{},false);// this is where it darts over
        currentPosition=lowPos;
        chassis.turnToHeading(-90,1000,{},false);
        chassis.moveToPoint(-67,-112,3000,{.forwards=false},false);///////
        
       
        pros::delay(300);//we are not going for 4th corner. just the 2 rd ones and the back right blue one
        intake.move(0);
        currentPosition=10000;//hang time MAKE THIS EXPOSE HANG idk how to make it ALL the way down. out pos is just for alliance stake but we want it fully down pls do that 
        // bro even I dont know how to make it go all the way down but I do know in the code that controls the movement of the lady brown it stops it from going to far down so this should be fine
        chassis.moveToPoint(-99,-94,4000,{},false);
        chassis.turnToHeading(-496,1000,{},false);//
        chassis.moveToPoint(-70,-70,20000,{.forwards=false,.maxSpeed=67.2},false);

       
        //chassis.moveToPoint()//this is point for the hang needs to be tuned can u just make it drive straight back wards at 60 percent speed for like 2 feet?
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
        currentPosition = outPos;
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
        currentPosition = highPos - 20;
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
void auton1(){ //alliance stake right side

    if (team == 'r'){  //red side alliance stake 
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
        chassis.moveToPoint(25, 30, 2000, {}, false);
        chassis.turnToHeading(270, 1000, {.earlyExitRange=2}, false);
        currentPosition = highPos;
        chassis.moveToPoint(0,31,2000,{},false);
        
    } else {//Blue Side alliance stake
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
        chassis.moveToPoint(25, 30, 2000, {}, false);
        chassis.turnToHeading(270, 1000, {.earlyExitRange=2}, false);
        currentPosition = highPos;
        chassis.moveToPoint(0,31,2000,{},false);
    }
}

void auton2(){ // 4 stack both sides no alliance
    if (team == 'r'){
        chassis.setPose(0,0,180);
    } else {
        chassis.setPose(0,0,180);

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
        colorSort = true;
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
        //end of rush
        //score middle
        chassis.turnToHeading(180, 1000, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed=80, .earlyExitRange=1}, false);
        chassis.moveToPoint(-6,45, 2000, {.forwards=false,.maxSpeed=50, .earlyExitRange=1}, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        intake.move(127);
        pros::delay(500);
        intake.move(0);
        mogoValue= false;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.turnToHeading(80,2000,{},false);
       //pros::delay(15000);
        chassis.moveToPoint(-30,28, 2000, {.forwards=false, .minSpeed=50, .earlyExitRange=1}, false);
        chassis.moveToPoint(-40,33, 2000, {.forwards=false, .maxSpeed=50}, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        colorSort=true;
        chassis.turnToHeading(105, 1000, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed=80, .earlyExitRange=4}, false);
        doinker.set_value(false);
        pros::delay(400);
        colorSort = true;
        chassis.moveToPose(-10.5, -2, 100, 2000, {}, true);
        pros::delay(400);
        doinker.set_value(true);
        chassis.waitUntilDone();
        chassis.turnToHeading(60, 1000, {.minSpeed=127, .earlyExitRange=10});
        doinker.set_value(false);
        colorSort = true;
        chassis.angularSettings.kP+=3;
        chassis.moveToPose(3, 40, 0, 4000, {.maxSpeed=80}, true);
        while ((pros::millis() - startTime)/1000 < 14.9) {
            
        }
        mogoValue = false;
        mogo.set_value(mogoValue);
        chassis.angularSettings.kP-=3;
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

        chassis.moveToPoint(8,30, 2000, {.forwards=false, .maxSpeed=50}, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.turnToHeading(180,2000,{},false);
        chassis.moveToPose(21,0,-225, 2000, { .maxSpeed=80}, true);
        currentPosition = midPos;
        pros::delay(200);
        intake.move(127);
        chassis.waitUntilDone();
        intake.move(-30);
        currentPosition = outPos;
    }
}

void auton5(){ // safer goal rush
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
        currentPosition = midPos;
        chassis.turnToHeading(135, 1000, {.minSpeed=80, .earlyExitRange=1}, false);
        chassis.moveToPoint(-30,28, 2000, {.forwards=false, .minSpeed=50, .earlyExitRange=1}, false);
        chassis.moveToPoint(-40,30, 2000, {.forwards=false, .maxSpeed=50}, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.turnToHeading(180, 1000, {.minSpeed=50, .earlyExitRange=4}, false);
        intake.move(127);
        chassis.moveToPose(-48, 0, 225, 3000, {.maxSpeed=80}, false);
        intake.move(-30);
        currentPosition = outPos; // score on alliance stake
        pros::delay(400);
        chassis.moveToPoint(-42, 6, 1000, {.forwards=false}, false);
        chassis.turnToHeading(100, 1000, {}, false);
        doinkVal = true;
        doinker.set_value(doinkVal);
        chassis.moveToPose(0, -5, 90, 2000, {.maxSpeed=80}, false);
        chassis.turnToHeading(50,2000,{},false);
        doinkVal = false;
        doinker.set_value(doinkVal);
    } else {

    }
}
void auton7(){ // 4 stack left side
    if (team == 'r'){
        chassis.setPose(0,0,180);
    } else {
        chassis.setPose(0,0,180);
    }   
}
void auton8(){ // 4 stack right
    if (team == 'r'){
        chassis.setPose(0,0,180);
        chassis.moveToPoint(0, 20, 1000, {.minSpeed=70, .earlyExitRange=2}, false);
        chassis.moveToPoint(-5, 35, 1000, {.maxSpeed=50}, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        chassis.turnToHeading(90, 1000, {}, false);
    } else {
        chassis.setPose(0,0,180);
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