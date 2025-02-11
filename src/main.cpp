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
double k = 1.5;
double i = 0;
double d = 1;
lemlib::PID arm(1.5, 0, 1);
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
            pros::c::screen_print_at(pros::E_TEXT_SMALL, 350, 220, "no auton");
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
        if (currentPosition > 267){
            currentPosition = 267;
        }
        //lifter.move_absolute(currentPosition, 200);
        lifter.move(arm.update(currentPosition - ((float)lifterRotation.get_position()/100)));
        /*
        Color sort
        */
        //hue of 0 is red 240 is blue
        if (colorSort && distSensor.get_distance() < 70){
            int startpose = intake.get_position();
            int EjectingColor = colorSensor.get_hue();
            if (team == 'r'){ //if we are on red team, and sees a ring at the top of the intake
                if (EjectingColor < 240) { // if the color is between 0 and 240
                    if (EjectingColor > 120){ // check if ring is closer to red or blue
                        while (234 + startpose > intake.get_position()){intake.move(127); pros::delay(20);}
                        intake.move(-127);
                        pros::delay(50);
                    }
                } else { // if color is between 240, 360
                    if (EjectingColor < 300){ // if the color is closer to blue
                        while (234 + startpose > intake.get_position()){intake.move(127); pros::delay(20);}
                        intake.move(-127);
                        pros::delay(50);
                    }
                }
            } else if (team == 'b'){ //if we are on red team, and sees a ring at the top of the intake
                if (EjectingColor < 240) { // if the color is between 0 and 240
                    if (EjectingColor < 120){ // check if ring is closer to red or blue
                        while (234 + startpose > intake.get_position()){intake.move(127); pros::delay(20);}
                        intake.move(-127);
                        pros::delay(50);
                    }
                } else { // if color is between 240, 360
                    if (EjectingColor > 300){ // if the color is closer to blue
                        while (234 + startpose > intake.get_position()){intake.move(127); pros::delay(20);}
                        intake.move(-127);
                        pros::delay(50);
                    }
                }
            }
        } else if (colorSort){
            intake.move(127);
        }

        // anti jam
        // if (intake.get_efficiency() < 0.01 && currentPosition != midPos){
        //     intake.move(-127);
        //     pros::delay(200);
        //     intake.move(127);
        // }
        pros::delay(20);
    }
}

void printing(){
    while (true) {
        if (auton != 5){
            if (mogoValue){
                updateLeds();
            } else {
                leds.set_all(0x00ff00);
            }
        }

        if (team == 'r'){pros::screen::set_pen(0xff0000);} else {pros::screen::set_pen(0x0000ff);}
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 400, 14, "%d", auton);

        pros::c::screen_print_at(pros::E_TEXT_SMALL, 350, 100, "x:%f", chassis.getPose().x);
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 350, 120, "y:%f", chassis.getPose().y);
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 350, 140, "theta:%f", chassis.getPose().theta);
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 350, 160, "colorProx:%d", colorSensor.get_proximity());
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 350, 180, "kP:%f", arm.kP);
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 350, 200, "kD:%f", arm.kD);

        ///pros::lcd::print(1, "x:%f, y:%f, theta:%f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
        // pros::lcd::print(2, "color:%f  proximity:%d", colorSensor.get_hue(), colorSensor.get_proximity());
        // pros::lcd::print(3, "distance:%d", distSensor.get_distance());
        pros::delay(50);
    }
}
long HSBtoRGB(float _hue, float _sat, float _brightness) {
    float red = 0.0;
    float green = 0.0;
    float blue = 0.0;
    
    if (_sat == 0.0) {
        red = _brightness;
        green = _brightness;
        blue = _brightness;
    } else {
        if (_hue == 360.0) {
            _hue = 0;
        }

        int slice = _hue / 60.0;
        float hue_frac = (_hue / 60.0) - slice;

        float aa = _brightness * (1.0 - _sat);
        float bb = _brightness * (1.0 - _sat * hue_frac);
        float cc = _brightness * (1.0 - _sat * (1.0 - hue_frac));
        
        switch(slice) {
            case 0:
                red = _brightness;
                green = cc;
                blue = aa;
                break;
            case 1:
                red = bb;
                green = _brightness;
                blue = aa;
                break;
            case 2:
                red = aa;
                green = _brightness;
                blue = cc;
                break;
            case 3:
                red = aa;
                green = bb;
                blue = _brightness;
                break;
            case 4:
                red = cc;
                green = aa;
                blue = _brightness;
                break;
            case 5:
                red = _brightness;
                green = aa;
                blue = bb;
                break;
            default:
                red = 0.0;
                green = 0.0;
                blue = 0.0;
                break;
        }
    }

    long ired = red * 255.0;
    long igreen = green * 255.0;
    long iblue = blue * 255.0;
    
    return long((ired << 16) | (igreen << 8) | (iblue));
}

int counter = 0;
int numColors = 255;
void cycle() {
    for (int i = 0; i < 31; i++){
        float colorNumber = (counter + i) % (numColors * 2) > numColors ? (counter + i) % (numColors * 2) - numColors: (counter + i) % (numColors * 2);

        float saturation = 1;
        float brightness = 1;
        float hue = (colorNumber / float(numColors)) * 360;
        long color = HSBtoRGB(hue, saturation, brightness);
        int red = color >> 16 & 255;
        int green = color >> 8 & 255;
        int blue = color & 255;
        leds.set_pixel((red*65536) + (green*256) + blue, i);

                //leds[i] = CRGB ( red, green, blue );
    }
    
    counter = (counter + 1) % (numColors * 2);
}
void initialize() {
    // chassis.calibrate();
    lifterRotation.set_position(0);
    pros::Task liftTask(lifting);
    pros::Task infoTask(printing);
    colorSensor.set_led_pwm(100);
    updateLeds();
    drawImage("brainBackground.png", 0, 0);
    // drawField();
    if (team == 'r'){pros::screen::set_pen(0xff0000);} else {pros::screen::set_pen(0x0000ff);}
    pros::screen::fill_rect(340, 55, 468, 90);
    angles.push_back(0);
    angles.push_back(90);
    angles.push_back(180);
    angles.push_back(270);
    chassis.calibrate();
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
bool intake_moving = false;
void opcontrol() {
    currentPosition = lowPos;
    pros::screen::touch_callback(getTouched, pros::E_TOUCH_PRESSED);
    while (true) {
        if (auton == 5){
            cycle();
        }
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
                    intake_moving = true;
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
            colorSort = false;
            intake.move(127);
            intake_moving=false;
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            colorSort = false;
            intake.move(-127);
            intake_moving=false;
        } else if (!intake_moving){
            intake.move(0);
            colorSort = false;
        }
        if (intake_moving){
            intake.move(-30);
            colorSort = false;
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
        //480 degrees per second
        
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            currentPosition+=10;
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            currentPosition-=10;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            arm.kP = arm.kP + 0.1;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            arm.kP = arm.kP - 0.1;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            arm.kD = arm.kD + 0.1;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            arm.kD = arm.kD - 0.1;
        }

        pros::delay(20);
    }
}