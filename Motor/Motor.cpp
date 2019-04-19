#include "mbed.h"
#include "Motor.h"

Motor::Motor(PinName p01,PinName p10,PinName pwm1,PinName p02,PinName p20,PinName pwm2,PinName p03,PinName p30,PinName pwm3):
_p01(p01),_p10(p10),_pwm1(pwm1),_p02(p02),_p20(p20),_pwm2(pwm2),_p03(p03),_p30(p30),_pwm3(pwm3)
{
    _p01 = 0;
    _p10 = 0;
    _pwm1 = 0;
    _p02 = 0;
    _p20 = 0;
    _pwm2 = 0;
    _p03 = 0;
    _p30 = 0;
    _pwm3 = 0;
}

/*モーターのPWM周波数を変更する*/
void Motor::setPwmPeriod(float hz){
    _pwm1.period(hz);
    _pwm2.period(hz);
    _pwm3.period(hz);
}

/*モーターを動かす*/
void Motor::setPower(float a,float b,float c){
    
    a = a / (float)100;
    b = b / (float)100;
    c = c / (float)100;
    
    if(a > 0) { //正回転
        _p01 = 1;
        _p10 = 0;
        _pwm1 = a;
    } else if(a == 0) { //ブレーキ
        _p01 = 1;
        _p10 = 1;
        _pwm1 = 0;
    } else { //負回転
        _p01 = 0;
        _p10 = 1;
        _pwm1 = -1*a;
    }
    if(b > 0) {
        _p02 = 1;
        _p20 = 0;
        _pwm2 = b;
        
    } else if(b == 0) {
        _p02 = 1;
        _p20 = 1;
        _pwm2 = b;
    } else {
        _p02 = 0;
        _p20 = 1;
        _pwm2 = -1*b;
    }
    if(c > 0) {
        _p03 = 1;
        _p30 = 0;
        _pwm3 = c;
    } else if(c == 0) {
        _p03 = 1;
        _p30 = 1;
        _pwm3 = c;
    } else {
        _p03 = 0;
        _p30 = 1;
        _pwm3 = -1*c;

    }
}

//*********************************************************************************/////////
///////3輪オムニホイール制御(進行方向(角度),パワー(スピード),旋回(回転))//////////
//********************************************************************************://///////
/*3輪バージョン*/
void Motor::omniWheels(int degree,int power,int mod){
    double PI = 3.1415926;
    float motor[3];//繝｢繝ｼ繧ｿ繝ｼ逕ｨ螟画焚
    float Max[2];//譛�螟ｧ蛟､逕ｨ螟画焚
    
    if(power == 0){//繝代Ρ繝ｼ0縺ｮ縺ｨ縺�
        
        motor[0] = mod; //繝｢繝ｼ繧ｿ繝ｼ蜿ｳ
        motor[1] = mod; //繝｢繝ｼ繧ｿ繝ｼ蠕�
        motor[2] = mod; //繝｢繝ｼ繧ｿ繝ｼ蟾ｦ
    }
    else{ 
    
        motor[0] = sin((degree-60)*PI/180) + (float)mod * 0.01; //繝｢繝ｼ繧ｿ繝ｼ蜿ｳ
        motor[1] = sin((degree-180)*PI/180) + (float)mod * 0.01; //繝｢繝ｼ繧ｿ繝ｼ蠕�
        motor[2] = sin((degree-300)*PI/180) + (float)mod * 0.01; //繝｢繝ｼ繧ｿ繝ｼ蟾ｦ
        
        if(motor[0]>1){
            motor[0] = 1;
        }
        else if(motor[0] < -1){
            motor[0] = -1;
        }
        if(motor[1]>1){
            motor[1] = 1;
        }
        else if(motor[1] < -1){
            motor[1] = -1;
        }
        if(motor[2]>1){
            motor[2] = 1;
        }
        else if(motor[2] < -1){
            motor[2] = -1;
        }
        
    
        if(fabs(motor[0]) >= fabs(motor[1])){ //繝｢繝ｼ繧ｿ繝ｼ繝代Ρ繝ｼ譛�螟ｧ蛟､險育ｮ�
            Max[0] = fabs(motor[0]);
        }
        else{
            Max[0] = fabs(motor[1]);
        }
        if(fabs(motor[2]) >= Max[0]){
            Max[1] = fabs(motor[2]);
        }
        else{
            Max[1] = Max[0];//譛�螟ｧ蛟､
        }
        
    
        motor[0] = (power*(motor[0]/Max[1]));//繝｢繝ｼ繧ｿ繝代Ρ繝ｼ譛�螟ｧ蛟､菫ｮ豁｣
        motor[1] = (power*(motor[1]/Max[1]));
        motor[2] = (power*(motor[2]/Max[1]));
    }
    
    setPower(motor[0],motor[1],motor[2]);
    
}

