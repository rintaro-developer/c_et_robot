/**
 ******************************************************************************
 **	�t�@�C���� : PIDpar.c
 **
 **	�T�v       : 2�֓|���U�q���C���g���[�X���{�b�g
 **			PID�p�����[�^����p�v���O����
 **
 ******************************************************************************
 **/
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* �|���U�q����p�w�b�_�t�@�C�� */
#include "nxt_config.h"

/* ���L�̃p�����[�^�̓Z���T��/���ɍ��킹�ă`���[�j���O����K�v������܂� */
#define GYRO_OFFSET	592	/* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��)  */
#define DELTA_t		0.004f	/*�T���v�����O����[s] */

#define TAIL_ANGLE_STAND_UP	105	/* ���S��~���̊p�x[�x] */	//��������ƃ_��
#define TAIL_ANGLE_DRIVE	3	/* �o�����X���s���̊p�x[�x] */
#define TAIL_ANGLE_GATE		70	/* ���b�N�A�b�v�Q�[�g�ʉߎ��̊p�x[�x] */
#define P_GAIN			2.5f	/* ���S��~�p���[�^������Q�C�� */
#define PWM_ABS_MAX		100	/* ���S��~�p���[�^����PWM��΍ő�l */
#define CMD_START		'1'	/* �����[�g�X�^�[�g�R�}���h(�ύX�֎~) */

/* �{�ԗp */
#define target		510		/* ���C���G�b�W臒l */
#define gray_target		511	/*�O���[�l-35*/
#define figure_target	500	/* �t���M���AL��ł̃��C���G�b�W臒l(+0) */
#define look_figure_target	580	/* �t���M���AL��ł̃��b�N�A�b�v���̃��C���G�b�W臒l */
#define look_target		580	/* ���b�N�A�b�v���̃��C���G�b�W臒l */


#define BT_MAX_RX_BUF_SIZE	32	/* Bluetooth�ʐM�p�f�[�^��M�o�b�t�@�̑傫�� */
#define DEVICE_NAME	"ET343"
int STATE = 1;				/* ��ԑJ�ڗp�t���O�ϐ� */
char rx_buf[BT_MAX_RX_BUF_SIZE];	/* Bluetooth�ʐM�p�f�[�^��M�o�b�t�@ */



//*****************************************************************************
// �v���g�^�C�v�錾
//*****************************************************************************

void speed_set(	signed char *forward);
void STATE_set(void);
void tail_control(signed int angle);
static int remote_start(void);
float Distance( int B_encorder, int C_encorder );


//*****************************************************************************
// �֐���		: ecrobot_device_initialize
// ����			: none
// �߂�l		: none
// �T�v		 	: �f�o�C�X�������p�t�b�N�֐��B���̊֐���nxtOSEK�N�����Ɏ��s�����B
//				  ���[�U�[�̓Z���T�A���[�^�Ȃǂ̊e�f�o�C�X�̏������֐�������
//*****************************************************************************
void ecrobot_device_initialize(void)
{
	ecrobot_init_bt_slave(BT_PASS_KEY);
	ecrobot_init_sonar_sensor(NXT_PORT_S2); /* �����g�Z���T(I2C�ʐM)�������� */
	nxt_motor_set_count(NXT_PORT_A, 0); 	/* �K�����[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_count(NXT_PORT_B, 0); 	/* �E���[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_count(NXT_PORT_C, 0); 	/* �����[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_speed(NXT_PORT_A, 0, 1); 	/* �K�����[�^PWM�o�̓Z�b�g */
}


//*****************************************************************************
// �֐���		: ecrobot_device_terminate
// ����			: none
// �߂�l		: none
// �T�vN 		: �f�o�C�X�I���p�t�b�N�֐��B���̊֐���STOP�܂���EXIT�{�^���������ꂽ���Ɏ��s�B
//			���[�U�[�̓Z���T�A���[�^�Ȃǂ̊e�f�o�C�X�̏I���֐��������B
//*****************************************************************************
void ecrobot_device_terminate(void)
{
	ecrobot_set_light_sensor_inactive(NXT_PORT_S3); /* ���Z���T�ԐFLED��OFF */
	ecrobot_term_sonar_sensor(NXT_PORT_S2); /* �����g�Z���T(I2C�ʐM)���I�� */
	ecrobot_term_bt_connection(); /* Bluetooth�ʐM���I�� */
	nxt_motor_set_count(NXT_PORT_A, 0); 	/* �K�����[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_count(NXT_PORT_B, 0); 	/* �E���[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_count(NXT_PORT_C, 0); 	/* �����[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_speed(NXT_PORT_A, 0, 1); 	/* �K�����[�^PWM�o�̓Z�b�g */
	nxt_motor_set_speed(NXT_PORT_B, 0, 1); 	/* �E���[�^PWM�o�̓Z�b�g */
	nxt_motor_set_speed(NXT_PORT_C, 0, 1); 	/* �����[�^PWM�o�̓Z�b�g */
	STATE = 1;

	systick_wait_ms(500); 
}

//*****************************************************************************
// �֐���	: user_1ms_isr_type2
// ����		: �Ȃ�
// �߂�l	: �Ȃ�
// �T�v		: 1msec�������荞�݃t�b�N�֐�(OSEK ISR type2�J�e�S��)
//		���̊֐����g�p���邱�ƂŁAOSEK Alarm�J�E���^�@�\���g�p�������[�g���m�g�j�b�N
//		�X�P�W���[�������������邱�Ƃ��ł��܂��B
//*****************************************************************************
void user_1ms_isr_type2(void){}


//*****************************************************************************
// �֐���	: speed_set
// ����		: forward(���x)
// �߂�l	: �Ȃ�
// �T�v		: ���x�ݒ�p���[�U�֐�	�����[�^�Őݒ�
//*****************************************************************************
void speed_set(	signed char *forward)
{
	int count, count_pre=0;
	nxt_motor_set_count( NXT_PORT_B, 0 );

	while(ecrobot_get_touch_sensor( NXT_PORT_S4 ) == 0 )/* �����������Ƃ��U */
	{		
		count = nxt_motor_get_count( NXT_PORT_B );
		count = count / 4;
	
		*forward = *forward + ( count - count_pre );		/* ��]�ʂ����Z */

		display_clear(0);		/* ��ʕ\�� */
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
// �֐���	: STATE_set
// ����		: �Ȃ�
// �߂�l	: �Ȃ�
// �T�v		: �X�e�C�g�ݒ�p���[�U�֐�	�����[�^�Őݒ�
//*****************************************************************************
void STATE_set(void)
{
	display_clear(0);
	display_goto_xy(0, 1);
	display_string("STATE=");
	display_int(STATE, 2);
	display_update();
	
	while(1) {
		if(ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) break; // �^�b�`�Z���T�������ꂽ 
		if(ecrobot_is_ENTER_button_pressed() == 1) {
			STATE = 1;
			display_clear(0);
			display_goto_xy(0, 1);
			display_string("STATE=");
			display_int(STATE, 2);
			display_update();
			systick_wait_ms(1000);
			if(ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) break; // �^�b�`�Z���T�������ꂽ 
		}
	}
	systick_wait_ms(1000);
	return(0);
}

//*****************************************************************************
// �֐��� : tail_control
// ���� : angle (���[�^�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ���s�̊��S��~�p���[�^�̊p�x����
//*****************************************************************************
void tail_control(signed int angle)
{
	int a_diffrence;
	float pwm;

	a_diffrence = angle - nxt_motor_get_count(NXT_PORT_A);		//���[�^A�p�x�΍�
	pwm = (float)( P_GAIN * a_diffrence );			/* PI���� */
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
// �֐��� : remote_start
// ���� : ����
// �Ԃ�l : 1(�X�^�[�g)/0(�ҋ@)
// �T�v : Bluetooth�ʐM�ɂ�郊���[�g�X�^�[�g�B Tera Term�Ȃǂ̃^�[�~�i���\�t�g����A
//       ASCII�R�[�h��1�𑗐M����ƁA�����[�g�X�^�[�g����B
//*****************************************************************************
static int remote_start(void)
{
	int i;
	unsigned int rx_len;
	unsigned char start = 0;

	for (i=0; i<BT_MAX_RX_BUF_SIZE; i++)
	{
		rx_buf[i] = 0; /* ��M�o�b�t�@���N���A */
	}

	rx_len = ecrobot_read_bt(rx_buf, 0, BT_MAX_RX_BUF_SIZE);
	if (rx_len > 0)
	{
		/* ��M�f�[�^���� */
		if (rx_buf[0] == CMD_START)
		{
			start = 1; /* ���s�J�n */
		}
		ecrobot_send_bt(rx_buf, 0, rx_len); /* ��M�f�[�^���G�R�[�o�b�N */
	}

	return (start);
}

//*****************************************************************************
// �֐��� : Distance
// ���� : ����
// �Ԃ�l : distance (����)
// �T�v : �G���R�[�_�l��苗�������߂�
//*****************************************************************************
float Distance( int B_encorder, int C_encorder  )
{
	static int B_encorder_bak =0;
	static int C_encorder_bak =0;
	int B_degree, C_degree;
	float B_Distance, C_Distance;
	static unsigned int STATEhold = 0;

	if((STATE==7 || STATE==9) && STATEhold==0) {
		B_encorder_bak =0;
		C_encorder_bak =0;
		STATEhold = 1;
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
// �^�X�N��	: TaskMain
// �T�v		: ���C���^�X�N
//*****************************************************************************
TASK(TaskMain)
{
	/* �O��i����: -100(��i)�`100(�O�i) */
	signed char forward[8] = {110, 80, 90, 40, 30, 120, 0, 20};
	/* ���񖽗�: -100(������)�`100(�E����) */
	F32 turn;	//turn = forward
	signed char pwm_L, pwm_R;		/* ���E���[�^PWM�o�� */

	/* �p�����[�^ 0:�������s�� 1:�J�[�u�Ǐ]�� */
	static F32 KP[2] = {1.21f, 1.21f};
	static F32 KI[2] = {2.12f, 0.51f};
	static F32 KD[2] = {0.48f, 0.12f};
	int setforward = 0;
	int setpid = 0;
	
	int brightness, gyroValue, sonar, bat;	/* �e�Z���T�l�ǂݎ��ϐ� */
	int A_encorder;
	int B_encorder, C_encorder;	/* �e���[�^�p�x�ǂݎ��ϐ� */
	float distance=0.0f;				/* �����p�ϐ� */

	unsigned int timecounter = 0;
	unsigned int counter = 0;
	unsigned int gyro_high_flag = 0, gyro_low_flag = 0;
	int ghf_time = 0, glf_time = 0;		//ghf=gyro_high_flag, l=low
	int gyromonitor = 0;
	int tailmonitor = 0;
	int alert_f = 0;

	int light;		/* ���Z���T�l */
	float diffrence, diffrence_bak=0.0f;		/* PID�p �΍���1�T���v�����O�O�̕΍� */
	static F32 integral=0.0f;		/* PID�p �΍��ϕ��l */
	float fturn;		/* forward = turn */
	int acc=0;			/* �W���C���Z���T�␳�l */

	STATE = 8;				//�J�n�X�e�C�g���w��(�e�X�g�p)
	setforward=0; setpid=0;	//�J�n�p�����[�^���w��(�e�X�g�p)

	ecrobot_set_light_sensor_active(NXT_PORT_S3);       /* ���Z���T�ԐFLED��ON */
	speed_set(&forward[0]);		/* ���x�ݒ�֐� */
	ecrobot_set_bt_device_name(DEVICE_NAME);
	STATE_set();

	//******���{�b�g�̃X�^�[�g����**************************************************
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
		if (remote_start() == 1) break; /* �����[�g�X�^�[�g */
		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) break; // �^�b�`�Z���T�������ꂽ 
	}

	//*******************************************************************************

	balance_init();				/* �|���U�q����Ɏg�p����ϐ��̏����� */
	nxt_motor_set_count(NXT_PORT_B, 0);	/* �E���[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_count(NXT_PORT_C, 0);	/* �����[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_speed(NXT_PORT_A, 0, 0);	/* �K�����[�^PWM�o�̓Z�b�g */
	brightness = ecrobot_get_light_sensor(NXT_PORT_S3);
	gyroValue = ecrobot_get_gyro_sensor(NXT_PORT_S1);
	counter = 0;

	while(1)
	{
		switch (STATE) {
			case 1:
			/* �S�[���܂Ń��C���g���[�X */
			tail_control( TAIL_ANGLE_DRIVE );
			light = ecrobot_get_light_sensor(NXT_PORT_S3);

			diffrence =  target-light;	/* �������C�����g���[�X */
			integral += ( ( diffrence + diffrence_bak) / 2.0f ) * DELTA_t;
			/* �����=���Q�C���~�΍� + �ϕ��Q�C���~�΍��ϕ��ʁ{�����Q�C���~�΍��ω��� */
			turn = ( KP[setpid] * diffrence + KI[setpid] * integral + KD[setpid] * ( diffrence - diffrence_bak) / DELTA_t );
			diffrence_bak = diffrence;
			
			fturn = forward[setforward];
			/* �㉺���l�`�F�b�N */
			if( fturn < turn )
				turn = fturn;
			else if( turn < (-1*fturn) )
				turn = -1*fturn;

			B_encorder = nxt_motor_get_count(NXT_PORT_B);
			C_encorder = nxt_motor_get_count(NXT_PORT_C);
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			balance_control(
				(float)forward[setforward],
				(float)turn,
				(float)gyromonitor,
				(float)GYRO_OFFSET,
				(float)C_encorder,
				(float)B_encorder,
				(float)ecrobot_get_battery_voltage(),
				&pwm_L, &pwm_R);
			ecrobot_bt_data_logger(STATE, (int)forward[setforward]);
			distance += Distance( B_encorder, C_encorder );
			
			/* �����n�߂Ă���̋������J�E���g����,�p�����[�^��ύX */
			if(distance > 4094.6f) {
				setforward=1; setpid=1;
				if(alert_f == 0) {
					ecrobot_sound_tone(880, 500, 30);
					alert_f = 1;
				}
			}
			if(distance > 5260.1f) {
				setforward=2; setpid=1;
				if(alert_f == 1) {
					ecrobot_sound_tone(880, 500, 30);
					alert_f = 2;
				}
			}
			if(distance > 6162.2f) {
				setforward=3; setpid=1;
				KD[1] = 0.0f;
				if(alert_f == 2) {
					ecrobot_sound_tone(880, 500, 30);
					alert_f = 3;
				}
			}
			if(distance > 7798.9f) {
				setforward=1; setpid=1;
				KD[1] = 0.12f;
				if(alert_f == 3) {
					ecrobot_sound_tone(880, 500, 40);
					alert_f = 4;
				}
			}
			if(distance > 9026.3f) {
				setforward=4; setpid=1;
				if(alert_f == 4) {
					ecrobot_sound_tone(1320, 1000, 40);
					alert_f = 0;
				}
				STATE = 3;		//�O���[�ʉߗp�X�e�C�g��
			}
			break;

		case 2:		//�O���[�ʉߗp�X�e�C�g_���g�p1116
			tail_control( TAIL_ANGLE_DRIVE );
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);

			diffrence =  gray_target-15-light;	/* �������C�����g���[�X */		//�J�[�u����̃O���[�␳�l:-15
			integral += ( ( diffrence + diffrence_bak) / 2.0f ) * DELTA_t;
			/* �����=���Q�C���~�΍� + �ϕ��Q�C���~�΍��ϕ��ʁ{�����Q�C���~�΍��ω��� */
			turn = ( KP[setpid] * diffrence + KI[setpid] * integral + KD[setpid] * ( diffrence - diffrence_bak) / DELTA_t );
			diffrence_bak = diffrence;
			
			fturn = forward[setforward];
			/* �㉺���l�`�F�b�N */
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
			/* �����n�߂Ă���̋������J�E���g����,�p�����[�^��ύX */
			if(distance > 10786.7f) {
				ecrobot_sound_tone(880, 200, 30);
				STATE = 3;		//�������ăt�B�M���A�ɏ��グ��X�e�C�g
				setforward=5;
				setpid=1;
				counter = 0;
			}
			break;


		case 3:	//��Q���֌����đ��x���グ�Đi�ށA���̌���グ�遨���x���グ�Ȃ�������
			tail_control( TAIL_ANGLE_DRIVE );
			setforward = 4; setpid = 1;	//5��4��1116
//			acc=0;	//�W���C���␳�l	1��0��1116
			
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			diffrence =gray_target -15 -light;	/* �������C�����g���[�X */ 	//-15�ǉ�1116
			integral += ( ( diffrence + diffrence_bak) / 2.0f ) * DELTA_t;
			/* �����=���Q�C���~�΍� + �ϕ��Q�C���~�΍��ϕ��ʁ{�����Q�C���~�΍��ω��� */
			turn = ( KP[setpid] * diffrence + KI[setpid] * integral + KD[setpid] * ( diffrence - diffrence_bak) / DELTA_t );

			fturn = forward[setforward];
			/* �㉺���l�`�F�b�N */
			if( fturn < turn )
				turn = fturn;
			else if( turn < (-1*fturn) )
				turn = -1*fturn;

			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			diffrence_bak = diffrence;
			balance_control(
				(float)forward[setforward],
				(float)turn,
				(float)gyromonitor,
				(float)(GYRO_OFFSET),
				(float)nxt_motor_get_count(NXT_PORT_C),
				(float)nxt_motor_get_count(NXT_PORT_B),
				(float)ecrobot_get_battery_voltage(),
				&pwm_L, &pwm_R);
			ecrobot_bt_data_logger(STATE, (int)forward[setforward]);
			//pwm_L += 10;	//�����[�^�␳
			if(acc>=500 && gyromonitor > 720) {	//�Փˌ��m
				counter = 0;
				STATE = 4;
				acc=0;
				ecrobot_sound_tone(880, 250, 40);
			}
			acc++;
			break;
						
		case 4:	//�t�B�M���A��őO�i
			tail_control( TAIL_ANGLE_DRIVE );
			setforward = 4; setpid = 1;		//�������O�i
			gyro_low_flag = 0;
			gyro_high_flag = 0;
			light = ecrobot_get_light_sensor(NXT_PORT_S3);

			diffrence =  figure_target-light;	/* �������C�����g���[�X */
			integral += ( ( diffrence + diffrence_bak) / 2.0f ) * DELTA_t;
			/* �����=���Q�C���~�΍� + �ϕ��Q�C���~�΍��ϕ��ʁ{�����Q�C���~�΍��ω��� */
			turn = ( KP[setpid] * diffrence + KI[setpid] * integral + KD[setpid] * ( diffrence - diffrence_bak) / DELTA_t );

			fturn = forward[setforward];
			/* �㉺���l�`�F�b�N */
			if( fturn < turn )
				turn = fturn;
			else if( turn < (-1*fturn) )
				turn = -1*fturn;

			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			diffrence_bak = diffrence;
			balance_control(
				(float)forward[setforward],
				(float)turn,
				(float)gyromonitor,
				(float)(GYRO_OFFSET),
				(float)nxt_motor_get_count(NXT_PORT_C),
				(float)nxt_motor_get_count(NXT_PORT_B),
				(float)ecrobot_get_battery_voltage(),
				&pwm_L, &pwm_R);
			ecrobot_bt_data_logger(STATE, (int)forward[setforward]);
			counter++;
//			pwm_L += 5;	//�����[�^�␳
			if(250<counter) {	//0.32�b��
				counter = 0;
				STATE = 5;
				acc = 0;
			}
			break;
			
		case 5:	//�t�B�M���A��ŐÎ~
			tail_control( TAIL_ANGLE_GATE+20 );
			setforward = 6; setpid = 1;
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			diffrence =  figure_target-light;	/* �������C�����g���[�X */
			integral += ( ( diffrence + diffrence_bak) / 2.0f ) * DELTA_t;
			/* �����=���Q�C���~�΍� + �ϕ��Q�C���~�΍��ϕ��ʁ{�����Q�C���~�΍��ω��� */
			turn = ( KP[setpid] * diffrence + KI[setpid] * integral + KD[setpid] * ( diffrence - diffrence_bak) / DELTA_t );

			fturn = forward[setforward];
			/* �㉺���l�`�F�b�N */
			if( fturn < turn )
				turn = fturn;
			else if( turn < (-1*fturn) )
				turn = -1*fturn;

			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			diffrence_bak = diffrence;
			balance_control(
				(float)forward[setforward],
				(float)turn,
				(float)gyromonitor,
				(float)(GYRO_OFFSET),
				(float)nxt_motor_get_count(NXT_PORT_C),
				(float)nxt_motor_get_count(NXT_PORT_B),
				(float)ecrobot_get_battery_voltage(),
				&pwm_L, &pwm_R);
			ecrobot_bt_data_logger(STATE, (int)forward[setforward]);
			counter++;
			if(150<counter) {		//0.6�b�o�߂�����
				STATE = 6;
				counter = 0;
			}
			break;

		case 6:		//�t�B�M���A��ŐK�����o���Ĉ��肷��
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			if(counter <= 250) {
				tailmonitor = TAIL_ANGLE_GATE+20;
				tail_control( tailmonitor );
				balance_control(
				(float)0,
				(float)0,
				(float)gyromonitor,
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
				(float)gyromonitor,
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
				counter = -1;
				STATE = 7;
			}
			ecrobot_bt_data_logger(STATE, pwm_L);
			counter++;
			break;

		case 7:		//���b�N�A�b�v���s�ŏ����i��Ń��C���G�b�W��Ɉړ�
			tail_control( TAIL_ANGLE_GATE );
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			if( light <= look_figure_target ) {
				pwm_R=25;pwm_L=45;
			} else if( light > look_figure_target ) {
				pwm_R=45;pwm_L=25;
			}
			counter++;
			if( counter > 350 ) {	//2�b�Ԉړ�
				STATE = 8;
				counter = 0;
				nxt_motor_set_count(NXT_PORT_B, 0);
				nxt_motor_set_count(NXT_PORT_C, 0);
				distance = 0;
			}
			ecrobot_bt_data_logger(STATE, pwm_L);
			break;

		case 8:		//�t�B�M���A��ŃV���O���X�s��
			tail_control( TAIL_ANGLE_GATE );
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			pwm_L=30; pwm_R=-30;
			B_encorder = nxt_motor_get_count(NXT_PORT_B);
			C_encorder = nxt_motor_get_count(NXT_PORT_C);
			distance = distance + Distance( -1*B_encorder, C_encorder);
//			if(distance > 535.0f) {					//��]����	520��535_1116
//				counter = 0;
//				nxt_motor_set_count(NXT_PORT_B, 0);	/* �E���[�^�G���R�[�_���Z�b�g */
//				nxt_motor_set_count(NXT_PORT_C, 0);	/* �����[�^�G���R�[�_���Z�b�g */
//				distance = 0;
//				STATE = 9;
//			}
			if(counter > 3450) {
				counter = 0;
				nxt_motor_set_count(NXT_PORT_B, 0);
				nxt_motor_set_count(NXT_PORT_C, 0);
				distance = 0;
				STATE = 9;
			}
			counter += 4;
			break;
		

		case 9:		//�X�s���セ�̂܂ܑO�ɑO�i���ăt�B�M���A����~���
			tail_control( TAIL_ANGLE_GATE );
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			if( light <= look_figure_target ) {
				pwm_R=25;pwm_L=45;
			} else if( light > look_figure_target ) {
				pwm_R=45;pwm_L=25;
			}
			counter++;
			ecrobot_bt_data_logger(STATE, pwm_L);
			if( gyromonitor<=540 || 620<=gyromonitor) {	//�t�B�M���A����~�肽���̊��m
				ecrobot_sound_tone(1320, 1000, 30);
				counter = 0;
				nxt_motor_set_count(NXT_PORT_B, 0);	/* �E���[�^�G���R�[�_���Z�b�g */
				nxt_motor_set_count(NXT_PORT_C, 0);	/* �����[�^�G���R�[�_���Z�b�g */
				distance = 0;
				STATE = 10;
			}
			break;
			
		case 10:		//�K���[�W�܂őO�i
			tail_control( TAIL_ANGLE_GATE );
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			if( light <= look_target ) {
				pwm_R=25;pwm_L=45;
			} else if( light > look_target ) {
				pwm_R=45;pwm_L=25;
			}
			B_encorder = nxt_motor_get_count(NXT_PORT_B);
			C_encorder = nxt_motor_get_count(NXT_PORT_C);
			distance = distance + Distance( B_encorder, C_encorder);
			if(distance > 883.6f) {		//�t�B�M���A����K���[�W�Ŏ~�܂�܂ł̋���
				STATE = 11;
			}
			counter++;
			break;
				
				
		case 11:		//��~�O����:
			tail_control( TAIL_ANGLE_GATE );
			pwm_R=0; pwm_L=0;
			ecrobot_sound_tone(880, 1000, 30);
			gyro_low_flag = 0;
			gyro_high_flag = 0;
			STATE = 99;	//�X�e�C�g�G���[�R�[�h
			break;

		default:
			pwm_R=0; pwm_L=0;
			tail_control( TAIL_ANGLE_GATE );
			ecrobot_bt_data_logger(99, STATE);
		}

		if(gyromonitor <= 300) {
			gyro_low_flag = 1;
			glf_time = timecounter;
		}
		if(gyromonitor >= 780) {
			gyro_high_flag = 1;
			ghf_time = timecounter;
		}
		if(STATE == 8) {
			gyro_high_flag = 0;
			gyro_low_flag = 0;
		}
		if(gyro_high_flag==1 && gyro_low_flag==1) {
			counter = ghf_time - glf_time;
			if(-75<counter && counter<75) tail_control( TAIL_ANGLE_DRIVE );
			STATE = 30;
		}

		nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* �����[�^PWM�o�̓Z�b�g */
		nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* �E���[�^PWM�o�̓Z�b�g */
		systick_wait_ms(4); /* 4msec�E�G�C�g */	
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
