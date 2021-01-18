/**
 ******************************************************************************
 **	ファイル名 : courceL.c
 **
 **	概要       : 2輪倒立振子ライントレースロボット
 **			PIDパラメータ測定用プログラム
 **
 ******************************************************************************
 **/
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* 倒立振子制御用ヘッダファイル */
#include "nxt_config.h"

/* 下記のパラメータはセンサ個体/環境に合わせてチューニングする必要があります */
#define GYRO_OFFSET	592	/* ジャイロセンサオフセット値(角速度0[deg/sec]時)  */
#define DELTA_t		0.004f	/*サンプリング時間[s] */

#define TAIL_ANGLE_STAND_UP	106	/* 完全停止時の角度[度] */	//高すぎるとダメ
#define TAIL_ANGLE_DRIVE	3	/* バランス走行時の角度[度] */
#define TAIL_ANGLE_GATE		60	/* ルックアップゲート通過時の角度[度] */				//ゲートの高さ変更
#define P_GAIN			2.5f	/* 完全停止用モータ制御比例ゲイン */
#define PWM_ABS_MAX		100	/* 完全停止用モータ制御PWM絶対最大値 */
#define CMD_START		'1'	/* リモートスタートコマンド(変更禁止) */
#define SONAR_ALERT_DISTANCE	26/* 超音波センサによる障害物検知距離[cm] */

/* 本番用 */
#define target		510		/* ラインエッジ閾値 */
#define gray_target 	511	/*グレー値-35*/
#define look_target		580	/* ルックアップ時のラインエッジ閾値 */
#define look_white_target	543	/* 白検索値+15 */
#define look_black_target	608	/* 黒検索値-10 */

#define BT_MAX_RX_BUF_SIZE	32	/* Bluetooth通信用データ受信バッファの大きさ */
#define DEVICE_NAME	"ET343"
int STATE = 1;				/* 状態遷移用フラグ変数 */
char rx_buf[BT_MAX_RX_BUF_SIZE];	/* Bluetooth通信用データ受信バッファ */



//*****************************************************************************
// プロトタイプ宣言
//*****************************************************************************

void speed_set(	signed char *forward);
void STATE_set(void);
void tail_control(signed int angle);
static int remote_start(void);
float Distance( int B_encorder, int C_encorder );
static int sonar_alert(void);


//*****************************************************************************
// 関数名		: ecrobot_device_initialize
// 引数			: none
// 戻り値		: none
// 概要		 	: デバイス初期化用フック関数。この関数はnxtOSEK起動時に実行される。
//				  ユーザーはセンサ、モータなどの各デバイスの初期化関数を実装
//*****************************************************************************
void ecrobot_device_initialize(void)
{
	ecrobot_init_bt_slave(BT_PASS_KEY);
	ecrobot_init_sonar_sensor(NXT_PORT_S2); /* 超音波センサ(I2C通信)を初期化 */
	nxt_motor_set_count(NXT_PORT_A, 0); 	/* 尻尾モータエンコーダリセット */
	nxt_motor_set_count(NXT_PORT_B, 0); 	/* 右モータエンコーダリセット */
	nxt_motor_set_count(NXT_PORT_C, 0); 	/* 左モータエンコーダリセット */
	nxt_motor_set_speed(NXT_PORT_A, 0, 1); 	/* 尻尾モータPWM出力セット */
}


//*****************************************************************************
// 関数名		: ecrobot_device_terminate
// 引数			: none
// 戻り値		: none
// 概要N 		: デバイス終了用フック関数。この関数はSTOPまたはEXITボタンが押された時に実行。
//			ユーザーはセンサ、モータなどの各デバイスの終了関数を実装。
//*****************************************************************************
void ecrobot_device_terminate(void)
{
	ecrobot_set_light_sensor_inactive(NXT_PORT_S3); /* 光センサ赤色LEDをOFF */
	ecrobot_term_sonar_sensor(NXT_PORT_S2); /* 超音波センサ(I2C通信)を終了 */
	ecrobot_term_bt_connection(); /* Bluetooth通信を終了 */
	nxt_motor_set_count(NXT_PORT_A, 0); 	/* 尻尾モータエンコーダリセット */
	nxt_motor_set_count(NXT_PORT_B, 0); 	/* 右モータエンコーダリセット */
	nxt_motor_set_count(NXT_PORT_C, 0); 	/* 左モータエンコーダリセット */
	nxt_motor_set_speed(NXT_PORT_A, 0, 1); 	/* 尻尾モータPWM出力セット */
	nxt_motor_set_speed(NXT_PORT_B, 0, 1); 	/* 右モータPWM出力セット */
	nxt_motor_set_speed(NXT_PORT_C, 0, 1); 	/* 左モータPWM出力セット */
	STATE = 1;

	systick_wait_ms(500); 
}

//*****************************************************************************
// 関数名	: user_1ms_isr_type2
// 引数		: なし
// 戻り値	: なし
// 概要		: 1msec周期割り込みフック関数(OSEK ISR type2カテゴリ)
//		この関数を使用することで、OSEK Alarmカウンタ機能を使用したレートモノトニック
//		スケジューラ等を実装することができます。
//*****************************************************************************
void user_1ms_isr_type2(void){}

//*****************************************************************************
// 関数名	: speed_set
// 引数		: forward(速度)
// 戻り値	: なし
// 概要		: 速度設定用ユーザ関数	左モータで設定
//*****************************************************************************
void speed_set(	signed char *forward)
{
	int count, count_pre=0;
	nxt_motor_set_count( NXT_PORT_B, 0 );

	while(ecrobot_get_touch_sensor( NXT_PORT_S4 ) == 0 )/* もし押したとき偽 */
	{		
		count = nxt_motor_get_count( NXT_PORT_B );
		count = count / 4;
	
		*forward = *forward + ( count - count_pre );		/* 回転量を加算 */

		display_clear(0);		/* 画面表示 */
		display_goto_xy(0, 1);
		display_string("SPEED=");
		display_int(*forward, 4);
		display_update();

		systick_wait_ms(100);
		count_pre = count;
	}
	systick_wait_ms(1000);
}

//*****************************************************************************
// 関数名	: STATE_set
// 引数		: なし
// 戻り値	: なし
// 概要		: ステイト設定用ユーザ関数	左モータで設定
//*****************************************************************************
void STATE_set(void)
{
	display_clear(0);
	display_goto_xy(0, 1);
	display_string("STATE=");
	display_int(STATE, 2);
	display_update();
	
	while(1) {
		if(ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) break; // タッチセンサが押された 
		if(ecrobot_is_ENTER_button_pressed() == 1) {
			STATE = 1;
			display_clear(0);
			display_goto_xy(0, 1);
			display_string("STATE=");
			display_int(STATE, 2);
			display_update();
			systick_wait_ms(1000);
			if(ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) break; // タッチセンサが押された 
		}
	}
	systick_wait_ms(1000);
	return(0);
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void tail_control(signed int angle)
{
	int a_diffrence;
	float pwm;

	a_diffrence = angle - nxt_motor_get_count(NXT_PORT_A);		//モータA角度偏差
	pwm = (float)( P_GAIN * a_diffrence );			/* PI制御 */
	if (pwm > PWM_ABS_MAX)
	{
		pwm = PWM_ABS_MAX;
	}
	else if (pwm < -PWM_ABS_MAX)
	{
		pwm = -PWM_ABS_MAX;
	}

	nxt_motor_set_speed(NXT_PORT_A, (signed char)pwm, 1);
}


//*****************************************************************************
// 関数名 : remote_start
// 引数 : 無し
// 返り値 : 1(スタート)/0(待機)
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
static int remote_start(void)
{
	int i;
	unsigned int rx_len;
	unsigned char start = 0;

	for (i=0; i<BT_MAX_RX_BUF_SIZE; i++)
	{
		rx_buf[i] = 0; /* 受信バッファをクリア */
	}

	rx_len = ecrobot_read_bt(rx_buf, 0, BT_MAX_RX_BUF_SIZE);
	if (rx_len > 0)
	{
		/* 受信データあり */
		if (rx_buf[0] == CMD_START)
		{
			start = 1; /* 走行開始 */
		}
		ecrobot_send_bt(rx_buf, 0, rx_len); /* 受信データをエコーバック */
	}

	return (start);
}

//*****************************************************************************
// 関数名 : Distance
// 引数 : 無し
// 返り値 : distance (距離)
// 概要 : エンコーダ値より距離を求める
//		  送り値のTorFが'0'の時，前回この関数を呼び出してからの距離
//		  '1'の時，走行開始時からの累計距離を返す
//*****************************************************************************
float Distance( int B_encorder, int C_encorder  )
{
	static int B_encorder_bak =0;
	static int C_encorder_bak =0;
	int B_degree, C_degree;
	float B_Distance, C_Distance;
	static unsigned int STATEhold = 0;

	if((STATE==6 || STATE==7 || STATE==8 || STATE==10 || STATE==11) && STATEhold!=STATE) {
		B_encorder_bak = 0;
		C_encorder_bak = 0;
		STATEhold = STATE;
	}
	B_degree = B_encorder - B_encorder_bak;
	C_degree = C_encorder - C_encorder_bak;
	B_Distance = 2.0f * 3.1415f * 42.0f * (float)B_degree / 360.0f;
	C_Distance = 2.0f * 3.1415f * 42.0f * (float)C_degree / 360.0f;

	B_encorder_bak = B_encorder;
	C_encorder_bak = C_encorder;

	return( (B_Distance+C_Distance)/2.0f );
}

//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int sonar_alert(void)
{
	static unsigned int counter = 0;
	static int alert = 0;

	signed int distance;

	if(++counter == 40/4) /* 約40msec周期毎に障害物検知  */
	{

		/*
		 * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
		 * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
		 */
		distance = ecrobot_get_sonar_sensor(NXT_PORT_S2);
		if( (0 <= distance) && (distance <= SONAR_ALERT_DISTANCE) ) {
			alert = 1; /* 障害物を検知 */
		} else if((0 <= distance) && (distance <= SONAR_ALERT_DISTANCE*2)) {
			alert = 2;
		} else {
			alert = 0; /* 障害物無し */
		}
		counter = 0;
	}
	return alert;
}

//*****************************************************************************
// タスク名	: TaskMain
// 概要		: メインタスク
//*****************************************************************************
TASK(TaskMain)
{
	/* 前後進命令: -100(後進)～100(前進) */
	signed char forward[5] = {110, 80, 90, 40, 50};
	/* 旋回命令: -100(左旋回)～100(右旋回) */
	F32 turn;	//turn = forward
	signed char pwm_L, pwm_R;		/* 左右モータPWM出力 */

	/* パラメータ 0:直線走行時 1:カーブ追従時 */
	static F32 KP[2] = {1.21f, 1.21f};
	static F32 KI[2] = {2.12f, 0.51f};
	static F32 KD[2] = {0.48f, 0.12f};
	int setforward = 0;
	int setpid = 0;
	
	int brightness, gyroValue, sonar, bat;	/* 各センサ値読み取り変数 */
	int A_encorder;
	int B_encorder=0, C_encorder=0;	/* 各モータ角度読み取り変数 */
	float distance=0.0f;				/* 距離用変数 */
	
	unsigned int timecounter = 0;
	unsigned int counter = 0;
	unsigned int gyro_high_flag = 0, gyro_low_flag = 0;
	int ghf_time = 0, glf_time = 0;		//ghf=gyro_high_flag, l=low
	int gyromonitor = 0;
	unsigned int tailmonitor = 0;		//ルックアップゲート時尻尾の状態を監視
	unsigned int s_flag = 0;
	int C11_f = 0;

	int light;		/* 光センサ値 */
	float diffrence, diffrence_bak=0.0f;		/* PID用 偏差と1サンプリング前の偏差 */
	static F32 integral=0.0f;		/* PID用 偏差積分値 */
	float fturn;		/* forward = turn */
	
	STATE = 1;				//開始ステイトを指定(テスト用)
	setforward = 0; setpid = 0;	//開始パラメータを指定(テスト用)

	ecrobot_set_light_sensor_active(NXT_PORT_S3);       /* 光センサ赤色LEDをON */
	speed_set(&forward[0]);		/* 速度設定関数 */
	ecrobot_set_bt_device_name(DEVICE_NAME);
	STATE_set();
	
	//******ロボットのスタート準備**************************************************
	while(1)
	{
		brightness = ecrobot_get_light_sensor(NXT_PORT_S3);
		gyroValue = ecrobot_get_gyro_sensor(NXT_PORT_S1);
		A_encorder = nxt_motor_get_count(NXT_PORT_A);
		sonar = ecrobot_get_sonar_sensor(NXT_PORT_S2);
		bat = ecrobot_get_battery_voltage();
		tailmonitor = TAIL_ANGLE_GATE;
		if(ecrobot_is_ENTER_button_pressed() == 1 || counter == 1) {
			tailmonitor = TAIL_ANGLE_STAND_UP;
			counter = 1;
		}
		display_clear(0);
		display_goto_xy(0, 0);
		display_string("STATE:");
		display_int(STATE, 5);
		display_goto_xy(0, 1);
		display_string("brightness:");
		display_int(brightness, 5);
		display_goto_xy(0, 3);
		display_string("Gyro:");
		display_int(gyroValue, 5);
		display_goto_xy(0, 4);
		display_string("Angle:");
		display_int(A_encorder, 5);
		display_goto_xy(0, 6);
		display_string("Bat:");
		display_int(bat, 5);
		display_update();
		systick_wait_ms(40);
		tail_control(tailmonitor);
		
		if (remote_start() == 1) break; /* リモートスタート */
		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1)	break; // タッチセンサが押された 
	}

	//*******************************************************************************

	balance_init();				/* 倒立振子制御に使用する変数の初期化 */
	nxt_motor_set_count(NXT_PORT_B, 0);	/* 右モータエンコーダリセット */
	nxt_motor_set_count(NXT_PORT_C, 0);	/* 左モータエンコーダリセット */
	nxt_motor_set_speed(NXT_PORT_A, 0, 0);	/* 左モータPWM出力セット */
	brightness = ecrobot_get_light_sensor(NXT_PORT_S3);
	gyroValue = ecrobot_get_gyro_sensor(NXT_PORT_S1);
	counter = 0;

	while(1)
	{
		switch (STATE) {
		case 1:
			/* ゴールまでライントレース */
			tail_control( TAIL_ANGLE_DRIVE );
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);

			diffrence =  target-light;	/* 左側ラインをトレース */
			integral += ( ( diffrence + diffrence_bak) / 2.0f ) * DELTA_t;
			/* 操作量=比例ゲイン×偏差 + 積分ゲイン×偏差積分量＋微分ゲイン×偏差変化量 */
			turn = ( KP[setpid] * diffrence + KI[setpid] * integral + KD[setpid] * ( diffrence - diffrence_bak) / DELTA_t );
			diffrence_bak = diffrence;
			
			fturn = forward[setforward];
			/* 上下限値チェック */
			if( fturn < turn )
				turn = fturn;
			else if( turn < (-1*fturn) )
				turn = -1*fturn;
			
				balance_control(
				(float)forward[setforward],
				(float)turn,
				(float)gyromonitor,
				(float)GYRO_OFFSET,
				(float)nxt_motor_get_count(NXT_PORT_C),
				(float)nxt_motor_get_count(NXT_PORT_B),
				(float)ecrobot_get_battery_voltage(),
				&pwm_L, &pwm_R);
			ecrobot_bt_data_logger(STATE, (int)forward[setforward]);
			B_encorder = nxt_motor_get_count(NXT_PORT_B);
			C_encorder = nxt_motor_get_count(NXT_PORT_C);

			distance += Distance( B_encorder, C_encorder );
			/* 動き始めてからの距離をカウントして,パラメータを変更 */
			if(distance > 4634.1f) {
				setforward=1; setpid=1;
				if(C11_f == 0) {
					ecrobot_sound_tone(880, 200, 30);
					C11_f = 1;
				}
			}
			if(distance > 5747.4f) {
				setforward=2; setpid=1;
				if(C11_f == 1) {
					ecrobot_sound_tone(880, 200, 30);
					C11_f = 2;
				}
			}
			if(distance > 6657.7f) {
				setforward=3; setpid = 1;
				if(C11_f == 2) {
					ecrobot_sound_tone(880, 200, 30);
					C11_f = 3;
				}
			}
			if(distance > 8031.2f) {
				setforward=4; setpid=1;
				STATE = 2;		//グレー通過用ステイトへ
				counter = 0;
				if(C11_f == 3) {
					ecrobot_sound_tone(880, 200, 30);
					C11_f = 0;
				}
			}
			break;
			
		case 2:		//グレー通過用ステイト
			tail_control( TAIL_ANGLE_DRIVE );
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);

			diffrence =  gray_target-15-light;	/* 左側ラインをトレース */		//カーブ直後のグレー補正値:-15
			integral += ( ( diffrence + diffrence_bak) / 2.0f ) * DELTA_t;
			/* 操作量=比例ゲイン×偏差 + 積分ゲイン×偏差積分量＋微分ゲイン×偏差変化量 */
			turn = ( KP[setpid] * diffrence + KI[setpid] * integral + KD[setpid] * ( diffrence - diffrence_bak) / DELTA_t );
			diffrence_bak = diffrence;
			
			fturn = forward[setforward];
			/* 上下限値チェック */
			if( fturn < turn )
				turn = fturn;
			else if( turn < (-1*fturn) )
				turn = -1*fturn;

			B_encorder = nxt_motor_get_count(NXT_PORT_B);
			C_encorder = nxt_motor_get_count(NXT_PORT_C);
			
				balance_control(
				(float)forward[setforward],
				(float)turn,
				(float)gyromonitor,
				(float)GYRO_OFFSET,
				(float)nxt_motor_get_count(NXT_PORT_C),
				(float)nxt_motor_get_count(NXT_PORT_B),
				(float)ecrobot_get_battery_voltage(),
				&pwm_L, &pwm_R);
			ecrobot_bt_data_logger(STATE, (int)forward[setforward]);

			distance += Distance( B_encorder, C_encorder );
			counter++;
			/* 動き始めてからの距離をカウントして,パラメータを変更 */
			if(distance > 8331.2) {		//1117
				setforward = 1;	//1117
				setpid = 1;
			}
			if(distance > 8926.3f) {
				ecrobot_sound_tone(1320, 500, 30);
				STATE = 3;
				setforward=3;
				setpid=1;
				counter = 0;
			}
			break;

		case 3:	//グレー通過用ステイト(ソナー)
			tail_control( TAIL_ANGLE_DRIVE );
			setforward = 4; setpid =1;
			if(s_flag == 1) {
				tail_control( TAIL_ANGLE_GATE+20 );
				ecrobot_sound_tone(880, 1000, 30);
				STATE = 4;
			}
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);

			diffrence =  gray_target -15 -light;	/* 左側ラインをトレース */
			integral += ( ( diffrence + diffrence_bak) / 2.0f ) * DELTA_t;
			/* 操作量=比例ゲイン×偏差 + 積分ゲイン×偏差積分量＋微分ゲイン×偏差変化量 */
			turn = ( KP[setpid] * diffrence + KI[setpid] * integral + KD[setpid] * 0 * ( diffrence - diffrence_bak) / DELTA_t );

			fturn = forward[setforward];
			/* 上下限値チェック */
			if( fturn < turn )
				turn = fturn;
			else if( turn < (-1*fturn) )
				turn = -1*fturn;

			if(s_flag != 1) {
				diffrence_bak = diffrence;
				balance_control(
				(float)forward[setforward],
				(float)turn,
				(float)gyromonitor,
				(float)GYRO_OFFSET,
				(float)nxt_motor_get_count(NXT_PORT_C),
				(float)nxt_motor_get_count(NXT_PORT_B),
				(float)ecrobot_get_battery_voltage(),
				&pwm_L, &pwm_R);
			}
			distance += Distance( B_encorder, C_encorder );
			ecrobot_bt_data_logger(STATE, (int)forward[setforward]);
			s_flag = sonar_alert();
				break;
			
		case 4:
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			setforward = 3; setpid = 1;
			if(counter <= 250) {
				tailmonitor = TAIL_ANGLE_GATE+20;
				tail_control( tailmonitor );
				balance_control(
				(float)0,
				(float)0,
				(float)ecrobot_get_gyro_sensor(NXT_PORT_S1),
				(float)GYRO_OFFSET,
				(float)nxt_motor_get_count(NXT_PORT_C),
				(float)nxt_motor_get_count(NXT_PORT_B),
				(float)ecrobot_get_battery_voltage(),
				&pwm_L, &pwm_R);
			} else if( 250<counter && counter<=500 ) {
				tailmonitor = TAIL_ANGLE_GATE+20;
				tail_control( tailmonitor );
				balance_control(
				(float)-10,
				(float)0,
				(float)ecrobot_get_gyro_sensor(NXT_PORT_S1),
				(float)GYRO_OFFSET,
				(float)nxt_motor_get_count(NXT_PORT_C),
				(float)nxt_motor_get_count(NXT_PORT_B),
				(float)ecrobot_get_battery_voltage(),
				&pwm_L, &pwm_R);
			} else if( 500<counter && counter<=700 ) {
				tailmonitor = TAIL_ANGLE_GATE+20;
				tail_control( tailmonitor );
				pwm_L = 0;
				pwm_R = 0;
			} else if( 700<counter && counter<=800 ) {
				tailmonitor = TAIL_ANGLE_GATE+10;
				tail_control( tailmonitor );
				pwm_L = 0;
				pwm_R = 0;
			} else if( 800<counter ) {
				tailmonitor = TAIL_ANGLE_GATE;
				tail_control( tailmonitor );
				pwm_L = 0;
				pwm_R = 0;
				counter = 0;
				C11_f=0;
				STATE = 5;
			}
			ecrobot_bt_data_logger(STATE, tailmonitor);
			counter++;
			break;
			
		case 5:	//白検索
			STATE = 6;	//1116
			tail_control( TAIL_ANGLE_GATE );
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			if( light > look_white_target ) {
				pwm_R=20; pwm_L=-20;
			} else if(light <= look_white_target ) {
				pwm_R=0; pwm_L=0;
				distance = 0;
				STATE = 6;
			}
			ecrobot_bt_data_logger(STATE, pwm_L);
			break;

		case 6:	//黒検索
//			STATE = 7;	//1116
//			tail_control( TAIL_ANGLE_GATE );
//			light = ecrobot_get_light_sensor(NXT_PORT_S3);
//			if( light < look_black_target ) {
//				pwm_R=-20; pwm_L=20;
//			} else if(light >= look_black_target ) {
//				pwm_R=0; pwm_L=0;
				nxt_motor_set_count(NXT_PORT_B, 0);	/* 右モータエンコーダリセット */
				nxt_motor_set_count(NXT_PORT_C, 0);	/* 左モータエンコーダリセット */
				distance = 0;
				STATE = 7;
//			}
			ecrobot_bt_data_logger(STATE, pwm_L);
			break;
			
		case 7:
			tail_control( TAIL_ANGLE_GATE );
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			if( light <= look_target ) {
				pwm_R=25;pwm_L=45;
			} else if(light > look_target ) {
				pwm_R=45;pwm_L=25;
			}
			B_encorder = nxt_motor_get_count(NXT_PORT_B);
			C_encorder = nxt_motor_get_count(NXT_PORT_C);
			distance = distance + Distance( B_encorder, C_encorder);
			ecrobot_bt_data_logger(STATE, pwm_L);
			if( distance > 380.0f ) {
				counter = 0;
				distance = 0;
				nxt_motor_set_count(NXT_PORT_B, 0);	/* 右モータエンコーダリセット */
				nxt_motor_set_count(NXT_PORT_C, 0);	/* 左モータエンコーダリセット */
				STATE = 8;		//450mm進んだらゲートを超えたとみなして次のステイトへ
				if(C11_f == 1) STATE = 12;	//二回目は旋回せず進む
			}
			break;
			
		case 8:
			tail_control( TAIL_ANGLE_GATE );
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			pwm_L=30; pwm_R=-30;
			B_encorder = nxt_motor_get_count(NXT_PORT_B);
			C_encorder = nxt_motor_get_count(NXT_PORT_C);
			distance = distance + Distance( -1*B_encorder, C_encorder);
			if(distance > 285.0f) {					//回転距離
				counter = 0;
				nxt_motor_set_count(NXT_PORT_B, 0);	/* 右モータエンコーダリセット */
				nxt_motor_set_count(NXT_PORT_C, 0);	/* 左モータエンコーダリセット */
				distance = 0;
				STATE = 9;
				if(C11_f == 1) STATE = 5;
				ecrobot_sound_tone(880, 1000, 30); 
			}
			break;
			
		case 9:	//白検索
			STATE = 10;
			tail_control( TAIL_ANGLE_GATE );
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			if( light <= look_white_target ) {
				pwm_R=20; pwm_L=-20;
			} else if(light > look_white_target ) {
				pwm_R=0; pwm_L=0;
				distance = 0;
				STATE = 10;
				ecrobot_sound_tone(1320, 1000, 30);
			}
			ecrobot_bt_data_logger(STATE, pwm_L);
			break;

		case 10:	//黒検索
//			tail_control( TAIL_ANGLE_GATE );
//			light = ecrobot_get_light_sensor(NXT_PORT_S3);
//			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
//			if( light >= look_black_target ) {
//				pwm_R=-20; pwm_L=20;
//			} else if(light < look_black_target ) {
//				pwm_R=0; pwm_L=0;
				nxt_motor_set_count(NXT_PORT_B, 0);	/* 右モータエンコーダリセット */
				nxt_motor_set_count(NXT_PORT_C, 0);	/* 左モータエンコーダリセット */
				distance = 0;
				STATE = 11;
//			}
			ecrobot_bt_data_logger(STATE, pwm_L);
			break;
			
		case 11:	//コースを逆に走行
			tail_control( TAIL_ANGLE_GATE );
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			if( light > look_target ) {
				pwm_R=25;pwm_L=45;
			} else if(light <= look_target ) {
				pwm_R=45;pwm_L=25;
			}
			B_encorder = nxt_motor_get_count(NXT_PORT_B);
			C_encorder = nxt_motor_get_count(NXT_PORT_C);
			distance = distance + Distance( B_encorder, C_encorder);
			ecrobot_bt_data_logger(STATE, pwm_L);
			if( distance > 380.0f ) {				//ゲート下を戻る距離
				counter = 0;
				nxt_motor_set_count(NXT_PORT_B, 0);	/* 右モータエンコーダリセット */
				nxt_motor_set_count(NXT_PORT_C, 0);	/* 左モータエンコーダリセット */
				STATE = 8;		//450mm進んだらゲートを超えたとみなして次のステイトへ
				C11_f = 1;
				distance = 0;
			}
			break;
			
		case 12:	//ゲートを通過しそのままゲートまで直進
			tail_control( TAIL_ANGLE_GATE );
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			if( light <= look_target ) {
				pwm_R=25;pwm_L=45;
			} else if(light > look_target ) {
				pwm_R=45;pwm_L=25;
			}
			B_encorder = nxt_motor_get_count(NXT_PORT_B);
			C_encorder = nxt_motor_get_count(NXT_PORT_C);
			distance = distance + Distance( B_encorder, C_encorder);
			if(distance > 765.0f) STATE = 99;	//70cm
			
			break;
			
		default:	//全動作終了
			pwm_R=0; pwm_L=0;
			tail_control( TAIL_ANGLE_GATE );
			ecrobot_bt_data_logger(99, STATE);
		}
		
		if(gyromonitor <= 300) {	//転倒検知1
			gyro_low_flag = 1;
			glf_time = timecounter;
		}
		if(gyromonitor >= 780) {	//転倒検知2
			gyro_high_flag = 1;
			ghf_time = timecounter;
		}
//		if(STATE != 0) {
//			gyro_high_flag = 0;
//			gyro_low_flag = 0;
//		}
		if(gyro_high_flag==1 && gyro_low_flag==1) {	//転倒検知したら動作停止
			counter = ghf_time - glf_time;
			if(-75<counter && counter<75) tail_control( TAIL_ANGLE_DRIVE );
			STATE = 30;
		}
		
		nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* 左モータPWM出力セット */
		nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* 右モータPWM出力セット */
		systick_wait_ms(4); /* 4msecウエイト */	
		timecounter++;
		
		display_clear(0);
		display_goto_xy(0, 0);
		display_string("distance:");
		display_int((int)distance, 5);
		display_goto_xy(0, 2);
		display_string("STATE:");
		display_int(STATE, 5);
		display_update();

	}
	systick_wait_ms(5000);
	ecrobot_term_bt_connection();
}
