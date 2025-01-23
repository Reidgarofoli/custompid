#include "main.h"
#include "PID.hpp"


void auton0(){
    if (team == 'r'){    
        setPose(0,0,0);
        currentPosition=outPos;
        pros::delay(400);
        moveToPoint(0,-7,1000,{.forwards=false}, false);
        pros::delay(200);
        currentPosition=lowPos;
        pros::delay(200);
        turnToHeading(270, 1000, {}, false);
        pros::delay(200);
        moveToPoint(25,-5,1000,{.forwards=false, .minSpeed=50, .earlyExitRange=2}, false);
        pros::delay(200);
        mogoValue = true;
        mogo.set_value(mogoValue);
        pros::delay(200);
        turnToHeading(190, 1000, {}, false);
        pros::delay(200);
        intake.move(127);
        moveToPoint(24, -22, 1000, {}, false);
        pros::delay(1000);
        turnToHeading(135, 1000, {}, false);
        currentPosition = midPos;
        moveToPoint(45, -47, 1000, {.minSpeed = 50, .earlyExitRange = 3}, false);
        moveToPoint(55, -48, 1000, {}, false);
        turnToHeading(-270, 100000, {}, false);
        pros::delay(200);
        // moveToPoint(58, -47, 1000, {}, false);
        pros::delay(1000);
        intake.move(-30);
        currentPosition = highPos;
        } else {       

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