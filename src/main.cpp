#include "main.h"
#include "auton.hpp"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "liblvgl/llemu.hpp"
#include "liblvgl/font/lv_font.h"
#include "liblvgl/draw/lv_draw.h"
#include <cmath>
#include <vector>
int startTime = 0;
double lastColor = 0;
PIDcontroller arm(0.8, 0.0, 1.0);
void updateLeds(){
    if (team == 'r'){
        leds.set_all(0xff0000);
    } else {
        leds.set_all(0x0000ff);
    }
}
void drawImage(std::string filename, int x, int y){
    int width, height, n;
    std::string name = "/usd/" + filename;
    unsigned char *img = stbi_load(name.c_str(), &width, &height, &n, 0);
    if (img == NULL)
    {
        printf("failed to open\n");
        return;
    }
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            pros::screen::set_pen((img[((x) + (y * width)) * 4] * 65536) + (img[((x) + (y * width)) * 4 + 1] * 256) + img[((x) + (y * width)) * 4 + 2]);
            pros::screen::draw_pixel(x, y);
        }
    }
    stbi_image_free(img);
}
void drawField(){
    switch (auton){
        case 0:
            if (team == 'r'){
                drawImage("2stack-red.png", 0, 0);
            } else {
                drawImage("2stack-blue.png", 0, 0);
            }
            break;
        case 1:
            drawImage("solo_wp.png", 0, 0);
            break;
        case 2:
            drawImage("goalRushAuto.png", 0, 0);
            break;
        default:
            pros::screen::set_pen(0x000000);
            pros::screen::fill_rect(0,0,240,240);
            break;
    }
}
void lifting() {
    while (true){
        /*
        ladyBrown code
        */
        if (currentPosition < 0){
            currentPosition = 0;
        }
        if (currentPosition > 680){
            currentPosition = 680;
        }
        //lifter.move_absolute(currentPosition, 200);
        lifter.move(arm.PID(currentPosition - ((float)lifterRotation.get_position()/100)));
        /*
        Color sort
        */
        //hue of 0 is red 240 is blue
        if (colorSort && distSensor.get_distance() < 70){
            pros::delay(ejectDelay);
            if (team == 'r'){ //if we are on red team, and sees a ring at the top of the intake
                if (lastColor < 240) { // if the color is between 0 and 240
                    if (lastColor > 120){ // check if ring is closer to red or blue
                        intake.move(-127);
                    }
                } else { // if color is between 240, 360
                    if (lastColor < 300){ // if the color is closer to blue
                        intake.move(-127);
                    }
                }
            } else if (team == 'b'){ //if we are on red team, and sees a ring at the top of the intake
                if (lastColor < 240) { // if the color is between 0 and 240
                    if (lastColor < 120){ // check if ring is closer to red or blue
                        intake.move(-127);
                    }
                } else { // if color is between 240, 360
                    if (lastColor > 300){ // if the color is closer to blue
                        intake.move(-127);
                    }
                }
            }
        } else if (colorSort){
            intake.move(127);
        }

        if (colorSensor.get_proximity() > 0){
            lastColor = colorSensor.get_hue();
        }

        pros::delay(20);
    }
}


void printing(){
    while (true) {
        if (mogoValue){
            updateLeds();
        } else {
            leds.set_all(0x00ff00);
        }
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 400, 14, "%d", auton);

        pros::c::screen_print_at(pros::E_TEXT_SMALL, 350, 100, "x:%f", getX());
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 350, 120, "y:%f", getY());
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 350, 140, "theta:%f", getAngle(false));
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 350, 160, "fDist:%d", fDist.get());
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 350, 180, "rDist:%d", rDist.get());

        ///pros::lcd::print(1, "x:%f, y:%f, theta:%f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
        // pros::lcd::print(2, "color:%f  proximity:%d", colorSensor.get_hue(), colorSensor.get_proximity());
        // pros::lcd::print(3, "distance:%d", distSensor.get_distance());
        pros::delay(50);
    }
}

void initialize() {
    // chassis.calibrate();
    lifterRotation.set_position(0);
    pros::Task liftTask(lifting);
    pros::Task infoTask(printing);
    updateLeds();
    drawImage("brainBackground.png", 0, 0);
    drawField();
    if (team == 'r'){pros::screen::set_pen(0xff0000);} else {pros::screen::set_pen(0x0000ff);}
    pros::screen::fill_rect(340, 55, 468, 90);
    initializeTracking();
}

void disabled() {}


void competition_initialize() {}


void autonomous() {
    switch (auton){
        case 0:
            auton0();
            break;
        case 1:
            auton1();
            break;
        case 2:
            auton2();
            break;
        case 3:
            auton3();
            break;
        case 4:
            auton4();
            break;
        case 5:
            auton5();
            break;
        case 6:
            auton6();
            break;
        default:
            break;
    } 
}
struct button
{
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    char *name;
};
const int numofbuttons = 4;
button buttons[numofbuttons] = {
    (button){338, 7, 66, 38, (char *)"-"},
    (button){338, 54, 132, 38, (char *)"team"},
    (button){410, 7, 66, 38, (char *)"+"},
};
void getTouched(){
    pros::screen_touch_status_s_t status = pros::screen::touch_status();
    for (int i = 0; i < numofbuttons; i++){
        if (status.x > buttons[i].x && status.y > buttons[i].y && status.x < buttons[i].x + buttons[i].width && status.y < buttons[i].y + buttons[i].height){
            if (buttons[i].name == "+"){
                if (auton != maxauto){
                    auton++;
                } else{
                    auton = 0;
                }
                drawField();
            }
            else if (buttons[i].name == "-"){
                if (auton != 0){
                    auton--;
                } else{
                    auton = maxauto;
                }
                drawField();
            }
            else if (buttons[i].name == "team"){
                if (team == 'r'){team = 'b'; pros::screen::set_pen(0x0000ff);} else {team = 'r'; pros::screen::set_pen(0xff0000);}
                pros::screen::fill_rect(340, 55, 468, 90);
                updateLeds();
                drawField();
			}
        }
    }
}
void opcontrol() {
    currentPosition = lowPos;
    pros::screen::touch_callback(getTouched, pros::E_TOUCH_PRESSED);
    while (true) {
        int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        LDrive.move(dir - turn);
        RDrive.move(dir + turn);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            switch (currentPosition){
                case lowPos:
                    currentPosition = midPos;
                    break;
                case midPos:
                    currentPosition = highPos;
                    break;
                case highPos:
                    currentPosition = lowPos;
                    break;
                default:
                    currentPosition = lowPos;
                    break;
            }
        }

        //intake
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake.move(-127);
        } else {
            intake.move(0);
        }

        //doinker
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            doinkVal = !doinkVal;
            doinker.set_value(doinkVal);
        }

        //mogo
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            mogoValue = !mogoValue;
            mogo.set_value(mogoValue);
            if (mogoValue){
                updateLeds();
            } else {
                leds.set_all(0x00ff00);
            }
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            currentPosition+=10;
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            currentPosition-=10;
        }
        pros::delay(20);
    }
}