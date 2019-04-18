#include "mbed.h"

class Motor
{
public:

    Motor(PinName p01,PinName p10,PinName pwm1, PinName p02,PinName p20,PinName pwm2,PinName p03,PinName p30,PinName pwm3);
    
    /*******************************************/
    /*初期設定(1周期を引数に取る)*/
    /******************************************/
    void setPwmPeriod(float hz);
    
    /****************************************************/
    /*モーターを動かす*/
    /****************************************************/
    void setPower(float a,float b, float c);
    
    //*********************************************************************************//
    ///////モーター制御θ(degree)に進行方向,powerにPWMの値(0～100),修正値をmodに代入,右回転基準////
    //********************************************************************************://
    void omniWheels(int degree,int power,int mod);

private:
    DigitalOut _p01;
    DigitalOut _p10;
    PwmOut _pwm1;
    DigitalOut _p02;
    DigitalOut _p20;
    PwmOut _pwm2;
    DigitalOut _p03;
    DigitalOut _p30;
    PwmOut _pwm3;
};