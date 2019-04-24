#include "mbed.h"
#include "BNO055.h"
#include "Ping.h"
#include "Motor.h"

//constant
#define GK	1
#define FW	0
#define PI	3.1415926

/*調整するときに読んでね*/
/*ロボットを調整する項目は3つあります
 * 以下の調整は試合前に必ずに行ってください
 *
 * 一つ目はラインセンサの調整です
 *　ロボットを起動し、USBケーブルを挿しこんだ後ターミナルを起動して'd'を押してください
 *　各種センサ値が表示されます
 *　コートの緑のマット上でLine_front ~ Line_rightの数値が
 *　75前後　になるように、
 *　ロボットの底面についている可変抵抗(青いやつ)を小さいドライバーで回して調整してください
 *　可変抵抗を時計回りに回すと数値が上がります
 *　キーボード'r'でセンサの値の取得を一時停止することができます
 *
 *　２つ目は回り込み半径の調整です
 * Rを調整することでボールに対してどれくらいの半径を描いて回り込むか調整することができます
 * 上記と同様にターミナルソフトを起動してキーボード'd'を押すと
 * 現在のセンサの数値を見ることができますのでBall distance:
 * の数値を参考にして最適なRを決定してください(distanceの数値=Rにする)
 * キーボード'r'でセンサの値の取得を一時停止することができます
 *
 * ３つ目はスピードとPIDゲイン値の調整です
 * まずラインから出ないようにspeedを調整します
 * そのあとロボットが常に前に向くように(傾いても元の方向にすぐに戻るように)
 * tp ti tdを調整してください
 * P→D→I(またはP→I→D)の順番に調整すると良いかもしれません
 *
 *　以上のことが完了してからプログラムの作成を開始してください
 */

//パラメータ調整項目
const int R = 70; //ロボット回り込み半径(0~255
const int speed = 85; //(0~100)の間で調整
const double tp = 1.5; //比例ゲイン
const double ti = 1.5; //積分ゲイン
const double td = 0.15; //微分ゲイン

//TerminalDisplay(TeraTerm)
Serial pc(SERIAL_TX, SERIAL_RX);

//voltage
AnalogIn voltage(PC_4);

//UltraSonicSensor
DigitalIn change(PA_6);
Ping uss_left(PB_10);
Ping uss_right(PA_7);
Ping uss_back(PB_6);
Timer timer_USS;
Timer timer_volt;

//BallSensor&Linesensor
AnalogIn ball_degree(PA_4);
AnalogIn ball_distance(PB_0);
Serial sensor(PC_10, PC_11, 115200);

//Motor
Motor motor(PB_15, PB_14, PB_13, PB_2, PB_1, PA_9, PA_12, PA_11, PA_5);

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
DigitalIn sw_kick(USER_BUTTON);

//declear prototype (function list)
void uss_send_and_read(); //超音波センサ読み込み
int PID(double kp, double ki, double kd, int target, int degree, int reset = 0); //姿勢制御のPIDで計算する
int get_ball_degree(int A = 0); //ボールの角度を-180~180(ボールなし999)で取得する
int get_ball_distance(int A = 0); //ボールの距離を0~255で取得する
int get_line_degree(); //反応したラインの方向(角度)を0~315(反応なし999)で取得する
int turn(int degree, int distance, int target = 0, int angle = 0); //回り込みの進行方向を計算する
int get_uss_range(char c); //超音波センサの距離を取得する　引数('l','r','b')を代入するとその方向の値が得られる
bool check_voltage(); //電圧をチェックする

/*****************************************************************/
/**********************main function******************************/
/*****************************************************************/

int main() {

//**************************************************************//
////////////////////////initialize setting////////////////////////
//**************************************************************//
	wait_ms(200);
	kick = 0; //キックを解除する

	/*ultra sonic sensor set speed*/
	uss_right.Set_Speed_of_Sound(32); //(cm/ms)
	uss_left.Set_Speed_of_Sound(32); //(cm/ms)
	uss_back.Set_Speed_of_Sound(32); //(cm/ms)

	/*motor pwm frequency set*/
	motor.setPwmPeriod(0.00052);

	/*change Mode IMU,COMPASS,M4G,NDOF_FMC_OFF,NDOF*/
	/*BNO055モードについて
	 * ジャイロセンサーには様々なモードがありますが
	 * 基本的にNDOFモードを使って下さい
	 * 会場の磁場環境がひどい場合はIMUモードを使ってください
	 *　ロボットの電源を起動した直後にキャリブレーションを行ってください
	 *　キャリブレーションをすることでオウンゴールを防ぐことができます。
	 */
	imu.reset();
	imu.change_fusion_mode(MODE_NDOF);

	wait_ms(100);
	imu.get_Euler_Angles(&euler_angles);
	int init_degree = (int) euler_angles.h;
	motor.omniWheels(0, 0, 0);

	/*Variable*/
	int rotation, move, distance, degree, direction;

	/*timer*/

	timer_PID.start();

	while (1) {
//***************************************************************//
////////////////Play mode////////////
//***************************************************************//
		timer_USS.reset();
		timer_USS.start();
		timer_PID.reset();
		timer_PID.start();
		timer_volt.reset();
		timer_volt.start();
		PID(tp, ti, td, init_degree, init_degree, 1); //PIDの積分値をリセットする

		while (sw_start == 1 && check_voltage() == 1) {
			imu.get_Euler_Angles(&euler_angles); //ジャイロの値をしゅとくする
			rotation = PID(tp, ti, td, init_degree, (int) euler_angles.h); //PIDを計算する
			///////////////////////
			////*ラインの制御*///////
			////////////////////
			direction = get_line_degree();
			//if (direction != 999) {
			if (0) {
				int tmp = direction;

				/*ラインが反応したときコートの中に戻る制御プログラム*/
				while (direction != 999 && sw_start == 1 && check_voltage() == 1) {

					imu.get_Euler_Angles(&euler_angles);
					rotation = PID(tp, ti, td, init_degree,
							(int) euler_angles.h, 0);
					tmp = direction;			//踏んだラインの方向を保存
					if (direction == 90) {
						if (get_uss_range('b') >= 50) {
							direction = 240;
						}
					} else if (direction == 270) {
						if (get_uss_range('b') >= 50) {
							direction = 120;
						}
					} else if (direction <= 179) {
						direction = direction + 180;
					} else if (direction >= 180) {
						direction = direction - 180;
					}
					motor.omniWheels(direction, speed, rotation);
					direction = get_line_degree();
				}

				/*コートの中に戻った後、ボールが移動するまでその場で待機する*/
				if (tmp >= 180) {
					tmp = tmp - 360;
				}
				degree = get_ball_degree();
				while ((degree > (tmp - 80)) && (degree < (tmp + 80))
						&& sw_start == 1) {
					imu.get_Euler_Angles(&euler_angles);
					rotation = PID(tp, ti, td, init_degree,
							(int) euler_angles.h);
					motor.omniWheels(0, 0, rotation);
					degree = get_ball_degree();
				}
			}
			////////////////////////////////////
			/*ボールを追いかけるときの制御プログラム*///////
			///////////////////////////////////
			else {
				int P = (int) euler_angles.h - init_degree; //proportional
				if (P <= -180) { //convert to -180 ~ 180
					P = P + 360;
				}
				if (P >= 180) {
					P = P - 360;
				}
				/*ロボット向き修正*/
				if (P <= -35) {
					motor.setPower(70, 70, 70);
				} else if (P >= 30) {
					motor.setPower(-70, -70, -70);
				} else {
					/*ボールの角度と距離を取得する*/
					degree = get_ball_degree();
					distance = get_ball_distance();

					////////////////////
					/*ボールが見つからないとき*/
					////////////////////
					if (distance >= 240) {
						/*
						 * GKモードの場合、ゴール前まで戻ってくる
						 * FWモードの場合、その場で停止する
						 *
						 */
						int mode = FW; //モード切替
						switch (mode) {

						case GK: //GKモード
						{
							int uss_l = get_uss_range('l'), uss_r =
									get_uss_range('r'); //超音波センサの値を取得する
							static float Vx = 0, Vy = 0;

							Vy = get_uss_range('b') - 45; //y座標を取る
							if ((uss_l + uss_r) > 150) { //ロボットが他のロボットと干渉していないときx座標を取る
								Vx = 182 * (float) uss_l
										/ (float) (uss_l + uss_r) - 91;
							} else {
								Vx = 0; //x軸上で座標0にいる判定にする
							}

							if (Vy >= 0) { //ゴールの前にいないとき
								int direction_of_going =
										(int) ((float) 180 / PI)
												* (float) acos(
														Vy
																/ sqrt(
																		Vx * Vx
																				+ Vy
																						* Vy)); //進行方向を計算する
								if (Vx >= 20) { //右側にいるとき
									direction_of_going = 180
											+ direction_of_going;
									motor.omniWheels(direction_of_going, 80,
											rotation); //左後ろに移動
								} else if (Vx <= -20) { //左側にいるとき
									direction_of_going = 180
											- direction_of_going;
									motor.omniWheels(direction_of_going, 80,
											rotation); //右後ろに移動
								} else { //真ん中付近にいるとき
									motor.omniWheels(180, 80, rotation); //後ろへ移動
								}
							} else { //ゴールの前にいるとき
								if (Vx > 30) { //右側にいるとき
									motor.omniWheels(-90, 50, rotation); //左へ移動
								} else if (Vx < -30) { //左側にいるとき
									motor.omniWheels(90, 50, rotation); //右へ移動
								} else { //真ん中にいるとき
									motor.omniWheels(0, 0, rotation); //停止
								}
							}
							break;
						}

						case FW: //FWモード
							motor.omniWheels(0, 0, rotation); //ボールがないとき停止する
							break;
						default:
							motor.omniWheels(0, 0, rotation); //ボールがないとき停止する
							break;
						}
					}
					/////////////////
					/*ボールが見つかった時*/
					/////////////////
					else {

						/*ホールドセンサーが反応したとき*/
						static int count = 0; //タイマー代わりのカウンター
						move = turn(degree, distance); //回り込み方向を計算する
						motor.omniWheels(move, speed, rotation);
						if (hold_check.read() == 0) { //0
							if (count > 70) { //ホールドしてから0.1秒過ぎたとき時
								kick = 1; //キックする
								wait_ms(100); //キック待機時間
								kick = 0; //キックを解除する
								count = 0; //カウンタを0にする
							} else {
								wait_us(1); //マイクロ秒
								count++;
							}
						} else {
							count = 0;
						}
					}
				}
			}
		}
		timer_PID.stop();

//***************************************************************//
////////////////////////Gyro reset mode////////////////////////////
//***************************************************************//
		if (sw_reset == 0) {
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
					pc.printf("////////////////////////////////////////////\r\n");
					pc.printf("/*******************debug******************/\r\n");
					pc.printf("////////////////////////////////////////////\r\n\n");
					imu.get_Euler_Angles(&euler_angles);
					pc.printf("Gyro   degree: %d \r\n\n", (int) euler_angles.h);
					pc.printf("Ball   degree: %d \r\n", get_ball_degree());
					pc.printf("Ball distance: %d \r\n\n", get_ball_distance());
					pc.printf("Line   sensor: %d \r\n", get_line_degree());
					pc.printf("Hold   sensor: %d \r\n\n", hold_check.read());
					pc.printf("USS      left: %d cm\r\n", get_uss_range('l'));
					pc.printf("USS     right: %d cm\r\n", get_uss_range('r'));
					pc.printf("USS      back: %d cm\r\n\n", get_uss_range('b'));
					sensor.putc('D'); //request Line data
					while (sensor.readable() == 0)
						;
					int f = sensor.getc();
					while (sensor.readable() == 0)
						;
					int b = sensor.getc();
					while (sensor.readable() == 0)
						;
					int l = sensor.getc();
					while (sensor.readable() == 0)
						;
					int r = sensor.getc();
					pc.printf("Line    front: %d\r\n", f);
					pc.printf("Line     back: %d\r\n", b);
					pc.printf("Line     left: %d\r\n", l);
					pc.printf("Line    right: %d\r\n\n", r);
					pc.printf("batey voltage: %f\r\n", voltage.read() * 8.18);
					pc.printf("\f");
					wait_ms(300);
				}
			}
		}
//****************************************************************//
//////////////////////kick mode/////////////////////////////////////
//****************************************************************//
		if (sw_kick.read() != 1) {
			kick = 1;
			wait_ms(200);
			kick = 0;
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
	dt = time - pretime; //time
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
int turn(int degree, int distance, int target, int angle) {
	int going;

	angle = angle - target; //proportional
	if (angle <= -180) { //convert to -180 ~ 180
		angle = angle + 360;
	}
	if (angle >= 180) {
		angle = angle - 360;
	}

	degree = degree + angle; //ジャイロの値を加算
	if (degree >= -60 && degree <= 60) { //ボールがまえにあるとき
		going = 2 * degree;
	} else if (degree <= -150 && degree >= 150) { //ボールが真後ろにある時
		if (degree >= 0) {
			going = degree + 90;
		} else {
			going = degree - 90;
		}
	} else {
		if (distance <= R) { //ボールとの距離が近すぎるとき
			if (degree >= 0) {
				going = degree + 90; //(int)(180.0 - ((180.0/PI)*asin((double)distance / (double)R)));
			} else {
				going = degree - 90; //(int)(180.0 - ((180.0/PI)*asin((double)distance / (double)R)));
			}
		} else { //ボールとの距離が離れているとき
			if (degree >= 0) {
				going = degree
						+ (int) ((180.0 / PI)
								* asin((double) R / (double) distance));
			} else {
				going = degree
						- (int) ((180.0 / PI)
								* asin((double) R / (double) distance));
			}
		}
	}
	if (get_uss_range('b') <= 45) { //後ろの壁に近いとき
		if (going > 90) {
			going = 90;
		} else if (going < -90) {
			going = -90;
		}
	}
	return going - angle;
}

//////////////////////////////
/*get ball degree*/
//////////////////////////////
int get_ball_degree(int A) {
	static int value = 0;

	if (A == 1) { //analog read
		if (ball_distance.read() >= (double) 0.95) {
			return 999;
		} else {
			return (int) (ball_degree.read() * 360 - 185);
		}
	} else { //serial read

		/*get ball degree*/
		int count = 0;
		sensor.putc('B');
		char buffer[2];
		for (int i = 0; i < 2; i++) {
			while (sensor.readable() == 0) { //wait
				if (count >= 10000) {
					return value;
				}
				wait_us(1);
				count++;
			}
			buffer[i] = sensor.getc(); //get
		}
		value = ((uint16_t) buffer[0] << 8) + (uint8_t) buffer[1] - 180;

		return value;
	}
}

//////////////////////////////
/*get ball distance*/
//////////////////////////////
int get_ball_distance(int A) {
	static int value = 0;
	if (A == 1) { //analog read
		return (uint8_t) (ball_distance.read() * 255);
	} else { //serial read
		int count = 0;

		/*get ball distance*/
		sensor.putc('C');
		while (sensor.readable() == 0) { //wait
			if (count >= 10000) {
				return value;
			}
			wait_us(1);
			count++;
		}
		value = (int) sensor.getc(); //get
		return value;
	}
}

//////////////////////////////
/*get line degree*/
//////////////////////////////
int get_line_degree() {
	static char value = 0;
	static int degree = 0;
	int count = 0;

	/*get line value*/
	sensor.putc('A');
	while (sensor.readable() == 0) { //wait
		if (count >= 10000) {
			return degree;
		}
		wait_us(1);
		count++;
	}
	value = sensor.getc(); //get

	switch (value) {
	case 'N':
		degree = 999;
		return 999;
		//break;
	case '0':
		degree = 0;
		return 0;
		//break;
	case '1':
		degree = 45;
		return 45;
		//break;
	case '2':
		degree = 90;
		return 90;
		//break;
	case '3':
		degree = 135;
		return 135;
		//break;
	case '4':
		degree = 180;
		return 180;
		//break;
	case '5':
		degree = 225;
		return 225;
		//break;
	case '6':
		degree = 270;
		return 270;
		//break;
	case '7':
		degree = 315;
		return 315;
		//break;
	default:
		degree = 999;
		return 999;
		//break;
	}
}

//////////////////////////////////////
/*USS reading*/
//////////////////////////////////////
/////////////////////////////
/*get uss range cm*/
////////////////////////////
int get_uss_range(char c) {
	static int count = 0;
	static int l = 0, r = 0, b = 0;

	if (timer_USS.read() >= 0.04) {
		l = uss_left.Read_cm();
		r = uss_right.Read_cm();
		b = uss_back.Read_cm();
		uss_left.Send();
		uss_right.Send();
		uss_back.Send();
		timer_USS.reset();
	}

	if (c == 'l' || c == 'L') {
		return l;
	} else if (c == 'r' || c == 'R') {
		return r;
	} else if (c == 'b' || c == 'B') {
		return b;
	} else {
		return 999;
	}
}

//////////////////////////////////////
/*Battery check voltage*/
//////////////////////////////////////
bool check_voltage() {
	static float sum[5] = { 8, 8, 8, 8, 8 };
	static float Ave = 8;
	static bool S = 1;
	if (timer_volt.read() > 0.2) {
		for (int i = 4; i > 0; i--) {
			sum[i] = sum[i - 1];
		}
		sum[0] = voltage.read() * 8.1;
		Ave = (sum[0] + sum[1] + sum[2] + sum[3] + sum[4]) / 5;
		if (Ave < 7.0) { //6.8V以下で自動遮断
			S = 0;
		} else {
			S = 1;

		}
		timer_volt.reset();
	}
	return S;
}

