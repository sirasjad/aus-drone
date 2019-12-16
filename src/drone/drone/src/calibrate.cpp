#include <fstream>
#include <iostream>
#include "libnaza/naza_interface_manual.h"
#include "libnaza/pca9685.h"

static naza_interface_manual_c naza;
static PCA9685 pca9685;
static ConfigFile cf("/etc/naza/pwm_config.txt");

int testPWM(){
    std::cout << "Channel: ";
    std::string ch_val; std::getline(std::cin, ch_val);
    int cha_value = atoi(ch_val.c_str());

    while(true){
        std::cout << "Set PWN value: ";
        std::string pwm_val; std::getline(std::cin, pwm_val);
        int pwm_value = atoi(pwm_val.c_str());

        pca9685.Write(CHANNEL(cha_value), VALUE(pwm_value));
    }
}

void recalibrate(){
    naza.recalibrate(cf, pca9685);
}

void resetSticks(){
    naza.set_neutral(cf, pca9685);
}

int main(){
    pca9685.SetFrequency(50);
    naza.init_naza(cf, pca9685);
    recalibrate();

    std::cout << "Selection menu: (1) Test PWM | (2) Recalibrate flight controller | (3) Reset sticks \n";
    int sel; std::cin >> sel;
    switch(sel){
        case 1: { testPWM(); } break;
        case 2: { recalibrate(); } break;
        case 3: { resetSticks(); } break;
        default: { std:cout << "Invalid selection!\n"; } break;
    }
    return 0;
}
