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
void auton0(){
    if (team == 'r'){
        chassis.setPose(0,0,0);
        currentPosition=midPos;//
        pros::delay(500 );//
        intake.move(127);//
        pros::delay(250);//

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
        chassis.turnToHeading(135, 1000, {}, false);
        currentPosition = midPos;
        intake.move(127);
        chassis.moveToPoint(55, -50, 2000, {}, false);
        chassis.swingToHeading(90, DriveSide::LEFT, 1000, {}, false);
        chassis.moveToPoint(60, -50, 2000, {}, false);
        currentPosition = highPos;
        intake.move(-30);
        pros::delay(500);
        chassis.moveToPoint(43, -50, 1000, {.forwards = false}, false);
        chassis.turnToHeading(0, 1000, {}, false);
        intake.move(127);
        chassis.moveToPoint(47, -21, 1000, {.maxSpeed=60}, false);
        chassis.moveToPoint(47, -7, 1000, {.maxSpeed=60}, false);
        chassis.moveToPoint(46, 8, 1000, {.maxSpeed=60}, false);
        pros::delay(200);
        chassis.moveToPoint(50, -5, 1000, {.maxSpeed=60}, false);
        chassis.turnToHeading(-128, 1000, {}, false);
        chassis.moveToPoint(59, 11, 1000, {.forwards=false, .maxSpeed=60}, false);
        mogoValue = false;
        mogo.set_value(mogoValue);
        pros::delay(200);
        intake.move(0);
        chassis.moveToPoint(42, -1, 1000, {}, false);
        chassis.turnToHeading(90, 1000, {}, false);
        currentPosition = lowPos;
        chassis.moveToPoint(-26, -3, 4000, {.forwards=false, .maxSpeed=60, .minSpeed=30, .earlyExitRange=2}, false);
        mogoValue = true;
        mogo.set_value(mogoValue);
        chassis.turnToHeading(-200, 1000, {}, false);
        intake.move(127);
        chassis.moveToPoint(-24, -25, 4000, {.maxSpeed=60}, false);
        chassis.turnToHeading(-126, 1000, {}, false);
        currentPosition = midPos;
        chassis.moveToPoint(-55, -49, 4000, {}, false);
        chassis.turnToHeading(-90, 1000, {}, false);
        chassis.moveToPoint(-59, -52, 4000, {}, false);
         currentPosition = highPos;
        intake.move(-30);
        pros::delay(500);
        chassis.moveToPoint(-43, -50, 4000, {.forwards=false}, false);
        chassis.turnToHeading(0,200,{},false);
        intake.move(127);
        chassis.moveToPoint(-49,-18,4000,{.maxSpeed=50},false);
        pros::delay(100);
                chassis.moveToPoint(-49,6,4000,{.maxSpeed=50},false);

        chassis.turnToHeading(118,2000,{},false);
        chassis.moveToPoint(-57,11,4000,{.forwards=false},false);
        mogoValue = false;
        mogo.set_value(mogoValue);  //
    } else {
        chassis.setPose(0,0,0);
        chassis.moveToPoint(0, 30, 1000, {}, false);
    }
}
void auton1(){
    if (team == 'r'){    

    } else {       

    }
}

void auton2(){
    if (team == 'r'){    

    } else {       
             
    }   
}
void auton3(){
    if (team == 'r'){    

    } else {       
             
    }
}
void auton4(){
    if (team == 'r'){    

    } else {       
             
    }
}
void auton5(){
    if (team == 'r'){    

    } else {       
             
    }
}
void auton6(){ // skills
    if (team == 'r'){    

    } else {
             
    }
}