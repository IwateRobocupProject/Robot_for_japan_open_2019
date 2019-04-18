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

/*モーター周波数変更初期設定*/
void Motor::setPwmPeriod(float hz){
    _pwm1.period(hz);
    _pwm2.period(hz);
    _pwm3.period(hz);
}

/*モーター制御*/
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
        _p10 = 1;//0か1だと思う
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
///////モーター制御θ(degree)に進行方向,powerにPWMの値(0～100),修正値をmodに代入,右回転基準//////////
//********************************************************************************://///////
/*3輪用*/
void Motor::omniWheels(int degree,int power,int mod){
    double PI = 3.1415926;
    float motor[3];//モーター用変数
    float Max[2];//最大値用変数
    
    if(power == 0){//パワー0のとき
        
        motor[0] = mod; //モーター右
        motor[1] = mod; //モーター後
        motor[2] = mod; //モーター左
    }
    else{ 
    
        motor[0] = sin((degree-60)*PI/180) + (float)mod * 0.01; //モーター右
        motor[1] = sin((degree-180)*PI/180) + (float)mod * 0.01; //モーター後
        motor[2] = sin((degree-300)*PI/180) + (float)mod * 0.01; //モーター左
        
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
        
    
        if(fabs(motor[0]) >= fabs(motor[1])){ //モーターパワー最大値計算
            Max[0] = fabs(motor[0]);
        }
        else{
            Max[0] = fabs(motor[1]);
        }
        if(fabs(motor[2]) >= Max[0]){
            Max[1] = fabs(motor[2]);
        }
        else{
            Max[1] = Max[0];//最大値
        }
        
    
        motor[0] = (power*(motor[0]/Max[1]));//モータパワー最大値修正
        motor[1] = (power*(motor[1]/Max[1]));
        motor[2] = (power*(motor[2]/Max[1]));
    }
    
    setPower(motor[0],motor[1],motor[2]);
    
}

