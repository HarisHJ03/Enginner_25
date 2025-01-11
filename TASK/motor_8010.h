#ifndef __GO_M8010_6_H
#define __GO_M8010_6_H

//#include "main.h"
#include "crc.h"
#include "usart.h"
#include "string.h"
#include <math.h>

/**
 * @brief ���ģʽ������Ϣ
 * 
 */
 #define P_MIN 	-95.5f    // Radians
#define P_MAX 	 95.5f        
#define V_MIN 	-45.0f    // Rad/s
#define V_MAX 	 45.0f
#define KP_MIN	 0.0f     // N-m/rad
#define KP_MAX 	 500.0f
#define KD_MIN	 0.0f     // N-m/rad/s
#define KD_MAX	 5.0f
#define T_MIN 	-5.0f
#define T_MAX	 5.0f

static const float go8010_pid[5] = {0.5,0.01,5,0.6,0};//�ڶ�������ʱ����static��ʾ��������ֻ��motor8010.c�ļ����������Ͼͱ�ʾ���д�motor8010.h���ļ��������������壬�ͻ�����ض���ı���

typedef struct     // __attribute__((packed)) <- c++
{
    uint8_t id     :4;      // ���ID: 0,1...,14 15��ʾ�����е���㲥����(��ʱ�޷���)
    uint8_t status :3;      // ����ģʽ: 0.���� 1.FOC�ջ� 2.������У׼ 3.����
    uint8_t none   :1;      // ����λ
} RIS_Mode_t /*__attribute__((packed))*/;   // ����ģʽ 1Byte



/**
 * @brief ���״̬������Ϣ
 * 
 */
typedef struct
{
    int16_t tor_des;        // �����ؽ����Ť�� unit: N.m     (q8)
    int16_t spd_des;        // �����ؽ�����ٶ� unit: rad/s   (q7)
    int32_t pos_des;        // �����ؽ����λ�� unit: rad     (q15)
    uint16_t  k_pos;        // �����ؽڸն�ϵ�� unit: 0.0-1.0 (q15)
    uint16_t  k_spd;        // �����ؽ�����ϵ�� unit: 0.0-1.0 (q15)
    
} RIS_Comd_t;   // ���Ʋ��� 12Byte




/**
 * @brief �������ݰ���ʽ
 * 
 */
typedef struct
{
    uint8_t head[2];    // ��ͷ         2Byte
    RIS_Mode_t mode;    // �������ģʽ  1Byte
    RIS_Comd_t comd;    // ����������� 12Byte
    uint16_t   CRC16;   // CRC          2Byte

} ControlData_t;    // ������������     17Byte




typedef struct {              
        unsigned short id;              // ���ID��0xBB����ȫ�����
        unsigned short mode;            // 0:����, 5:����ת��, 10:�ջ�FOC����
        uint16_t correct;               // ���������Ƿ����� ��1 ������0��������
        int MError;                     // ������ 0.���� 1.���� 2.���� 3.��ѹ 4.����������
        int Temp;                       // �¶�
        float tar_pos;                  // target position 
        float tar_w;                    // target speed
        float T;                        // ��ǰʵ�ʵ���������
        int W;                        // ��ǰʵ�ʵ���ٶȣ����٣�
        float Pos;                      // ��ǰ���λ��
	      float angle;                    //ʵ����Ҫ�ĽǶ�
        int footForce;                  // They dont even know what 7 is this so we dont update this
        uint8_t buffer[17];
        uint8_t Rec_buffer[16];
        ControlData_t  motor_send_data;  
}GO_Motorfield;
extern GO_Motorfield motor_recevie;

typedef struct
{
    // int id;
    float T;
    fp32 Pos;
    float W;
    float K_P;
    float K_W;
} M8010_SendData;
extern M8010_SendData m8010_senddata[4];//8010���ͽṹ���ʼ��

//motor init function 
void GO_M8010_init(GO_Motorfield *GO_motor_info,uint8_t id);

// send motor control message
void GO_M8010_send_data(int id,float T,float W,float Pos,float K_P,float K_W);

//  transmittion interrupt callback
//void uartTxCB(UART_HandleTypeDef *huart);

// data recv to the target motor
void GO_M8010_recv_data(uint8_t* Temp_buffer, GO_Motorfield* motor);

// testing force position mix fucntion 



void go8010_init(void);

// motor array
extern GO_Motorfield GO_motor_info[4];

void go8010_receive(void);


#endif /*__GO_M8010_6_H */

