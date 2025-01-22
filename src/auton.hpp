#include "main.h"
#include "PID.hpp"


void auton0(){
    if (team == 'r'){    
        setPose(0,0,0);
        currentPosition=outPos;
        pros::delay(400);
        moveToPoint(0,-10,100000,{.forwards=false}, false);
        currentPosition=lowPos;    
        
        
        
        
        
        
        
        
        
        
        
        
        
        
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