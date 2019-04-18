#include "mbed.h"
#include "BNO055.h"
#include "Ping.h"
#include "Motor.h"

//const 
const float PI = 3.1415926;
const int R = 127;
const float tp = 0.4, ti = 0.05, td = 2.5;

//TerminalDisplay(TeraTerm)
Serial pc(SERIAL_TX, SERIAL_RX);

//UltraSonicSensor
Ping uss_left(PA_6);
Ping uss_right(PA_7);
Ping uss_back(PB_6);

//BallSensor&Linesensor
AnalogIn ball_degree(PA_4);
AnalogIn ball_distance(PB_0);
Serial sensor(PC_10, PC_11);

//Motor
Motor motor(PA_12, PA_11, PA_5, PB_2, PB_1, PA_9, PB_15, PB_14, PB_13);

//Timer
Timer timer_PID;

//GyroSensor
I2C i2c(D14, D15);
BNO055 imu(i2c, PC_8);
BNO055_ID_INF_TypeDef bno055_id_inf;
BNO055_EULER_TypeDef euler_angles;

//HoldCheckSensor
DigitalIn hold_check(PC_3);

//kicker
DigitalOut kick(PA_8);

//ToggleSwitch
DigitalIn sw_start(PD_2); //program start switch
DigitalIn sw_reset(PC_12); //gyro sensor reset switch

//variable
char line_value;
int degree_value, distance_value;

//declear prototype (function list)
int PID(double kp, double ki, double kd, int target, int degree, int reset = 0);
void sensor_read();
int get_ball_degree(int A = 0);
int get_ball_distance(int A = 0);
int get_line_degree();
int turn(int degree, int distance);

/*****************************************************************/
/**********************main function******************************/
/*****************************************************************/

int main() {

//**************************************************************//
////////////////////////initialize setting////////////////////////
//**************************************************************//
	wait_ms(200);

	/*Serial Interrupt*/
	sensor.attach(&sensor_read, Serial::RxIrq);

	/*ultra sonic sensor set speed*/
	uss_right.Set_Speed_of_Sound(32); //(cm/ms)
	uss_left.Set_Speed_of_Sound(32); //(cm/ms)
	uss_back.Set_Speed_of_Sound(32); //(cm/ms)

	/*motor pwm frequency set*/
	motor.setPwmPeriod(0.00052);

	/*change Mode IMU,COMPASS,M4G,NDOF_FMC_OFF,NDOF*/
	imu.reset();
	imu.change_fusion_mode(MODE_NDOF);
	wait_ms(100);
	imu.get_Euler_Angles(&euler_angles);
	int init_degree = euler_angles.h;
	imu.reset();
	motor.omniWheels(0, 0, 0);

	/*Variable*/
	int rotation, move, distance, degree, direction;

	/*timer*/
	timer_PID.start();

	while (1) {
//***************************************************************//
////////////////Play mode(you can write this statement)////////////
//***************************************************************//
		timer_PID.reset();
		timer_PID.start();
		PID(tp, ti, td, init_degree, 0, 1);
		while (sw_start == 1) {
			imu.get_Euler_Angles(&euler_angles); //get gyro
			rotation = PID(tp, ti, td, init_degree, euler_angles.h); //PID calcurate

			/*Line control*/
			direction = get_line_degree();
			if (direction != 999) {
				int tmp = direction;

				/*return from line*/
				while (direction != 999 && sw_start == 1) {
					imu.get_Euler_Angles(&euler_angles);
					rotation = PID(tp, ti, td, 0, euler_angles.h);
					tmp = direction;
					if (direction == 90) {
						direction = 240;
					} else if (direction == 270) {
						direction = 120;
					} else if (direction <= 179) {
						direction = direction + 180;
					} else if (direction >= 180) {
						direction = direction - 180;
					}
					motor.omniWheels(direction, 80, rotation);
					direction = get_line_degree();
				}
				/*stop & wait*/
				if (tmp >= 180) {
					tmp = tmp - 360;
				}
				degree = get_ball_degree();
				while ((degree > (tmp - 80)) && (degree < (tmp + 80))
						&& sw_start == 1) {
					imu.get_Euler_Angles(&euler_angles);
					rotation = PID(tp, ti, td, 0, euler_angles.h);
					motor.omniWheels(0, 0, rotation);
					degree = get_ball_degree();
				}
			}

			/*Ball follow control*/
			else {
				/*get ball degree and distance*/
				degree = get_ball_degree();
				distance = get_ball_distance();

				if (distance >= 240) { //ball is not found
					motor.omniWheels(0, 0, rotation);
					kick = 0;
				} else {
					if (hold_check.read() == 0) {
						/*ball hold&kick*/
						kick = 1;
						motor.omniWheels(0, 80, rotation);
					} else {
						/*Speed control*/
						kick = 0;
						move = turn(degree, distance);
						motor.omniWheels(move, 80, rotation);
					}
				}
			}
		}
		timer_PID.stop();

//***************************************************************//
////////////////////////Gyro reset mode////////////////////////////
//***************************************************************//
		if (sw_reset == 1) {
			imu.reset();
			wait_ms(100);
			imu.get_Euler_Angles(&euler_angles);
			init_degree = (int) euler_angles.h;
		}
		motor.omniWheels(0, 0, 0);

//***************************************************************//
//////////////////////////debug mode///////////////////////////////
//***************************************************************//
		if (pc.readable() > 0) {
			if (pc.getc() == 'd') {
				//if 'd' is pressed,debug mode will start.
				while (1) {
					if (pc.readable() > 0) {
						if (pc.getc() == 'r') { //if 'r' is pressed,debug mode will be end.
							break;
						}
					}
					imu.get_Euler_Angles(&euler_angles);
					pc.printf("\f");
					pc.printf("Gyro   degree: %d \r\n", (int) euler_angles.h);
					pc.printf("Ball   degree: %d \r\n", get_ball_degree());
					pc.printf("Ball distance: %d \r\n", get_ball_distance());
					pc.printf("Line   sensor: %d \r\n", get_line_degree());
					pc.printf("Hold   sensor: %d \r\n", hold_check.read());
					pc.printf("USS      left: %d cm\r\n", uss_left.Read_cm());
					pc.printf("USS     right: %d cm\r\n", uss_right.Read_cm());
					pc.printf("USS      back: %d cm\r\n", uss_back.Read_cm());
					sensor.putc('D'); //request Line data
					pc.printf("Line    front: %d\r\n", sensor.getc());
					pc.printf("Line     back: %d\r\n", sensor.getc());
					pc.printf("Line     left: %d\r\n", sensor.getc());
					pc.printf("Line    right: %d\r\n", sensor.getc());
					uss_left.Send();
					uss_right.Send();
					uss_back.Send();
					wait_ms(40);
				}
			}
		}
		/**********************end main function**************************/
	}
}

/////////////////////////////////////////////////////////
/*PID feedback*/
//////////////////////////////////////////////////////////
int PID(double kp, double ki, double kd, int target, int degree, int reset) {
	static double dt, time, pretime;
	static double P = 0, OP = 0, I = 0, D = 0, re = 0;

	if (reset == 1) {
		I = 0;
		time = 0;
		pretime = 0;
	}

	time = timer_PID.read();
	dt = pretime - time; //time
	P = degree - target; //proportional
	if (P <= -180) { //convert to -180 ~ 180
		P = P + 360;
	}
	if (P >= 180) {
		P = P - 360;
	}
	D = (P - OP) / dt; //differential
	I += (P + OP) * dt / 2; //Integral
	pretime = time;
	OP = P;
	re = -1 * (kp * P + ki * I + kd * D);

	if (P >= -5 && P <= 5) {
		I = 0;
	}
	if (re >= 60) {
		re = 60;
	} else if (re <= -60) {
		re = -60;
	}

	return re;
}

/////////////////////////////////////
/*turn control*/
////////////////////////////////////
int turn(int degree, int distance) {

	if (degree >= -45 && degree <= 45) { //ball is found front
		return (2 * degree);
	} else if (degree <= -130 && degree >= 130) { //ball is found back
		if (degree >= 0) {
			return (degree + 90);
		} else {
			return (degree - 90);
		}
	} else {
		if (distance <= R) {
			if (degree >= 0) {
				return (degree + 90); //(180 - 180/PI*asin(kyori / R));
			} else {
				return (degree - 90); //(180 - 180/PI*asin(kyori / R));
			}
		} else {
			if (degree >= 0) {
				return (degree + 180 / PI * asin((double) R / (double) distance)); //ball turn
			} else {
				return (degree - 180 / PI * asin((double) R / (double) distance));
			}
		}
	}
}

/////////////////////////////////////
/****get_sensor_value_for_UART******/
////////////////////////////////////
void sensor_read() {
	char command = sensor.getc(); //get value
	int count = 0;

	/*get line value*/
	if (command == 200) {
		Reget: //reget point
		//wait
		while (sensor.readable() == 0) {
			if (count >= 1000) {
				return;
			}
			wait_us(10);
			count++;
		}
		line_value = sensor.getc();
		//reget
		if (line_value == 200) {
			goto Reget;
		}
	}

	/*get ball value*/
	else if (command == 210) {
		char buffer[3];
		for (int i = 0; i < 3; i++) {
			while (sensor.readable() == 0) { //wait
				if (count >= 1000) {
					return;
				}
				wait_us(10);
				count++;
			}
			buffer[i] = sensor.getc();
			if (buffer[i] == 210) {//Reget
				i = -1;
			}
		}

		//combine value
		degree_value = ((uint16_t) buffer[0] << 8) + (uint8_t) buffer[1] - 180;
		distance_value = buffer[2];
	} else {        //do nothing
		return;
	}
}

/*****************************/
int get_ball_degree(int A) {
	if (A == 1) {
		if (ball_distance.read() >= (double) 0.95) {
			return 999;
		} else {
			return (int) (ball_degree.read() * 360 - 185);
		}
	} else {
		return (int) degree_value;
	}
}

int get_ball_distance(int A) {
	if (A == 1) {
		return (uint8_t) (ball_distance.read() * 255);
	} else {
		return (uint8_t) distance_value;
	}
}

int get_line_degree() {
	switch (line_value) {
	case 'N':
		return 999;
		//break;
	case '0':
		return 0;
		//break;
	case '1':
		return 45;
		//break;
	case '2':
		return 90;
		//break;
	case '3':
		return 135;
		//break;
	case '4':
		return 180;
		//break;
	case '5':
		return 225;
		//break;
	case '6':
		return 270;
		//break;
	case '7':
		return 315;
		//break;
	default:
		return 999;
		//break;
	}
}
/*************************************/
