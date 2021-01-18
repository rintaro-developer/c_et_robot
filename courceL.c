/**
 ******************************************************************************
 **	�t�@�C���� : courceL.c
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

#define TAIL_ANGLE_STAND_UP	106	/* ���S��~���̊p�x[�x] */	//��������ƃ_��
#define TAIL_ANGLE_DRIVE	3	/* �o�����X���s���̊p�x[�x] */
#define TAIL_ANGLE_GATE		60	/* ���b�N�A�b�v�Q�[�g�ʉߎ��̊p�x[�x] */				//�Q�[�g�̍����ύX
#define P_GAIN			2.5f	/* ���S��~�p���[�^������Q�C�� */
#define PWM_ABS_MAX		100	/* ���S��~�p���[�^����PWM��΍ő�l */
#define CMD_START		'1'	/* �����[�g�X�^�[�g�R�}���h(�ύX�֎~) */
#define SONAR_ALERT_DISTANCE	26/* �����g�Z���T�ɂ���Q�����m����[cm] */

/* �{�ԗp */
#define target		510		/* ���C���G�b�W臒l */
#define gray_target 	511	/*�O���[�l-35*/
#define look_target		580	/* ���b�N�A�b�v���̃��C���G�b�W臒l */
#define look_white_target	543	/* �������l+15 */
#define look_black_target	608	/* �������l-10 */

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
static int sonar_alert(void);


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
//		  ����l��TorF��'0'�̎��C�O�񂱂̊֐����Ăяo���Ă���̋���
//		  '1'�̎��C���s�J�n������̗݌v������Ԃ�
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
// �֐��� : sonar_alert
// ���� : ����
// �Ԃ�l : 1(��Q������)/0(��Q������)
// �T�v : �����g�Z���T�ɂ���Q�����m
//*****************************************************************************
static int sonar_alert(void)
{
	static unsigned int counter = 0;
	static int alert = 0;

	signed int distance;

	if(++counter == 40/4) /* ��40msec�������ɏ�Q�����m  */
	{

		/*
		 * �����g�Z���T�ɂ�鋗����������́A�����g�̌��������Ɉˑ����܂��B
		 * NXT�̏ꍇ�́A40msec�������x���o����̍ŒZ��������ł��B
		 */
		distance = ecrobot_get_sonar_sensor(NXT_PORT_S2);
		if( (0 <= distance) && (distance <= SONAR_ALERT_DISTANCE) ) {
			alert = 1; /* ��Q�������m */
		} else if((0 <= distance) && (distance <= SONAR_ALERT_DISTANCE*2)) {
			alert = 2;
		} else {
			alert = 0; /* ��Q������ */
		}
		counter = 0;
	}
	return alert;
}

//*****************************************************************************
// �^�X�N��	: TaskMain
// �T�v		: ���C���^�X�N
//*****************************************************************************
TASK(TaskMain)
{
	/* �O��i����: -100(��i)�`100(�O�i) */
	signed char forward[5] = {110, 80, 90, 40, 50};
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
	int B_encorder=0, C_encorder=0;	/* �e���[�^�p�x�ǂݎ��ϐ� */
	float distance=0.0f;				/* �����p�ϐ� */
	
	unsigned int timecounter = 0;
	unsigned int counter = 0;
	unsigned int gyro_high_flag = 0, gyro_low_flag = 0;
	int ghf_time = 0, glf_time = 0;		//ghf=gyro_high_flag, l=low
	int gyromonitor = 0;
	unsigned int tailmonitor = 0;		//���b�N�A�b�v�Q�[�g���K���̏�Ԃ��Ď�
	unsigned int s_flag = 0;
	int C11_f = 0;

	int light;		/* ���Z���T�l */
	float diffrence, diffrence_bak=0.0f;		/* PID�p �΍���1�T���v�����O�O�̕΍� */
	static F32 integral=0.0f;		/* PID�p �΍��ϕ��l */
	float fturn;		/* forward = turn */
	
	STATE = 1;				//�J�n�X�e�C�g���w��(�e�X�g�p)
	setforward = 0; setpid = 0;	//�J�n�p�����[�^���w��(�e�X�g�p)

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
		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1)	break; // �^�b�`�Z���T�������ꂽ 
	}

	//*******************************************************************************

	balance_init();				/* �|���U�q����Ɏg�p����ϐ��̏����� */
	nxt_motor_set_count(NXT_PORT_B, 0);	/* �E���[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_count(NXT_PORT_C, 0);	/* �����[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_speed(NXT_PORT_A, 0, 0);	/* �����[�^PWM�o�̓Z�b�g */
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
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);

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
			/* �����n�߂Ă���̋������J�E���g����,�p�����[�^��ύX */
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
				STATE = 2;		//�O���[�ʉߗp�X�e�C�g��
				counter = 0;
				if(C11_f == 3) {
					ecrobot_sound_tone(880, 200, 30);
					C11_f = 0;
				}
			}
			break;
			
		case 2:		//�O���[�ʉߗp�X�e�C�g
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

		case 3:	//�O���[�ʉߗp�X�e�C�g(�\�i�[)
			tail_control( TAIL_ANGLE_DRIVE );
			setforward = 4; setpid =1;
			if(s_flag == 1) {
				tail_control( TAIL_ANGLE_GATE+20 );
				ecrobot_sound_tone(880, 1000, 30);
				STATE = 4;
			}
			light = ecrobot_get_light_sensor(NXT_PORT_S3);
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);

			diffrence =  gray_target -15 -light;	/* �������C�����g���[�X */
			integral += ( ( diffrence + diffrence_bak) / 2.0f ) * DELTA_t;
			/* �����=���Q�C���~�΍� + �ϕ��Q�C���~�΍��ϕ��ʁ{�����Q�C���~�΍��ω��� */
			turn = ( KP[setpid] * diffrence + KI[setpid] * integral + KD[setpid] * 0 * ( diffrence - diffrence_bak) / DELTA_t );

			fturn = forward[setforward];
			/* �㉺���l�`�F�b�N */
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
			
		case 5:	//������
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

		case 6:	//������
//			STATE = 7;	//1116
//			tail_control( TAIL_ANGLE_GATE );
//			light = ecrobot_get_light_sensor(NXT_PORT_S3);
//			if( light < look_black_target ) {
//				pwm_R=-20; pwm_L=20;
//			} else if(light >= look_black_target ) {
//				pwm_R=0; pwm_L=0;
				nxt_motor_set_count(NXT_PORT_B, 0);	/* �E���[�^�G���R�[�_���Z�b�g */
				nxt_motor_set_count(NXT_PORT_C, 0);	/* �����[�^�G���R�[�_���Z�b�g */
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
				nxt_motor_set_count(NXT_PORT_B, 0);	/* �E���[�^�G���R�[�_���Z�b�g */
				nxt_motor_set_count(NXT_PORT_C, 0);	/* �����[�^�G���R�[�_���Z�b�g */
				STATE = 8;		//450mm�i�񂾂�Q�[�g�𒴂����Ƃ݂Ȃ��Ď��̃X�e�C�g��
				if(C11_f == 1) STATE = 12;	//���ڂ͐��񂹂��i��
			}
			break;
			
		case 8:
			tail_control( TAIL_ANGLE_GATE );
			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
			pwm_L=30; pwm_R=-30;
			B_encorder = nxt_motor_get_count(NXT_PORT_B);
			C_encorder = nxt_motor_get_count(NXT_PORT_C);
			distance = distance + Distance( -1*B_encorder, C_encorder);
			if(distance > 285.0f) {					//��]����
				counter = 0;
				nxt_motor_set_count(NXT_PORT_B, 0);	/* �E���[�^�G���R�[�_���Z�b�g */
				nxt_motor_set_count(NXT_PORT_C, 0);	/* �����[�^�G���R�[�_���Z�b�g */
				distance = 0;
				STATE = 9;
				if(C11_f == 1) STATE = 5;
				ecrobot_sound_tone(880, 1000, 30); 
			}
			break;
			
		case 9:	//������
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

		case 10:	//������
//			tail_control( TAIL_ANGLE_GATE );
//			light = ecrobot_get_light_sensor(NXT_PORT_S3);
//			gyromonitor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
//			if( light >= look_black_target ) {
//				pwm_R=-20; pwm_L=20;
//			} else if(light < look_black_target ) {
//				pwm_R=0; pwm_L=0;
				nxt_motor_set_count(NXT_PORT_B, 0);	/* �E���[�^�G���R�[�_���Z�b�g */
				nxt_motor_set_count(NXT_PORT_C, 0);	/* �����[�^�G���R�[�_���Z�b�g */
				distance = 0;
				STATE = 11;
//			}
			ecrobot_bt_data_logger(STATE, pwm_L);
			break;
			
		case 11:	//�R�[�X���t�ɑ��s
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
			if( distance > 380.0f ) {				//�Q�[�g����߂鋗��
				counter = 0;
				nxt_motor_set_count(NXT_PORT_B, 0);	/* �E���[�^�G���R�[�_���Z�b�g */
				nxt_motor_set_count(NXT_PORT_C, 0);	/* �����[�^�G���R�[�_���Z�b�g */
				STATE = 8;		//450mm�i�񂾂�Q�[�g�𒴂����Ƃ݂Ȃ��Ď��̃X�e�C�g��
				C11_f = 1;
				distance = 0;
			}
			break;
			
		case 12:	//�Q�[�g��ʉ߂����̂܂܃Q�[�g�܂Œ��i
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
			
		default:	//�S����I��
			pwm_R=0; pwm_L=0;
			tail_control( TAIL_ANGLE_GATE );
			ecrobot_bt_data_logger(99, STATE);
		}
		
		if(gyromonitor <= 300) {	//�]�|���m1
			gyro_low_flag = 1;
			glf_time = timecounter;
		}
		if(gyromonitor >= 780) {	//�]�|���m2
			gyro_high_flag = 1;
			ghf_time = timecounter;
		}
//		if(STATE != 0) {
//			gyro_high_flag = 0;
//			gyro_low_flag = 0;
//		}
		if(gyro_high_flag==1 && gyro_low_flag==1) {	//�]�|���m�����瓮���~
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
