/*
 * FOC.c
 *
 *  Created on: 25.01.2019
 *      Author: Stancecoke
 */
#include "main.h"
#include "config.h"
#include "FOC.h"
#include "stm32f1xx_hal.h"
#include <arm_math.h>
#include <stdlib.h>
#include <stdbool.h>

#define HFI_VOLTAGE 200

q31_t atan2_LUT(q31_t e_alpha, q31_t e_beta);
extern DMA_HandleTypeDef hdma_m2m;

//q31_t	T_halfsample = 0.00003125;
//q31_t	counterfrequency = 64000000;
//q31_t	U_max = (1/_SQRT3)*_U_DC;
q31_t temp1;
q31_t temp2;
q31_t temp3;
q31_t temp4;
q31_t temp5;
q31_t temp6;

q31_t q31_i_q_fil = 0;
q31_t q31_i_d_fil = 0;

q31_t x1;
q31_t x2;
q31_t teta_obs;

#ifdef FAST_LOOP_LOG
q31_t e_log[300][6];
#endif

q31_t z;
char Obs_flag = 1;
uint8_t ui8_debug_state = 0;

char PI_flag = 0;

//const q31_t _T = 2048;

TIM_HandleTypeDef htim1;

void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta,
		int16_t int16_i_q_target, MotorState_t *MS_FOC);
void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta);


static const q31_t hfi_sin_table[16] = {0,25079,46340,60547,65536,60547,46340,25079,
                                        0,-25079-46340,-60547,-65536,-60547,-46340,-25079 } ; //sin(2*PI/16 *i) * 2^16
static const q31_t hfi_cos_table[16] = {65536,60547,46340,25079,0,-25079,-46340,-60547,-65536,
                                        -60547,-46340,-25079,0,25079,46340,60547}; //cos(2*PI/16 *i) * 2^16

#define FIR_LENGTH 64
#define FIR_REVERSED 0

typedef struct {
     int16_t taps[FIR_LENGTH];
} filter_state_t;

static void queue_dma(uint16_t *dst, uint16_t *src, uint16_t count){
        //TODO: make sure DMA is not busy
        HAL_DMA_Start(&hdma_m2m, src, dst, count);
}

//TODO: has to be a way to optimize this
q31_t q31_low_pass(q31_t v, filter_state_t *filter){
        q31_t ret=0;
        filter->taps[FIR_REVERSED?0:63] = v;

        ret += (filter->taps[0] * 462) >> 8;
        ret += (filter->taps[1] * 438) >> 8;
        ret += (filter->taps[2] * 518) >> 8;
        ret += (filter->taps[3] * 623) >> 8;
        ret += (filter->taps[4] * 741) >> 8;
        ret += (filter->taps[5] * 871) >> 8;
        ret += (filter->taps[6] * 1011) >> 8;
        ret += (filter->taps[7] * 1157) >> 8;
        ret += (filter->taps[8] * 1309) >> 8;
        ret += (filter->taps[9] * 1464) >> 8;
        ret += (filter->taps[10] * 1619) >> 8;
        ret += (filter->taps[11] * 1773) >> 8;
        ret += (filter->taps[12] * 1922) >> 8;
        ret += (filter->taps[13] * 2064) >> 8;
        ret += (filter->taps[14] * 2196) >> 8;
        ret += (filter->taps[15] * 2316) >> 8;
        ret += (filter->taps[16] * 2422) >> 8;
        ret += (filter->taps[17] * 2512) >> 8;
        ret += (filter->taps[18] * 2585) >> 8;
        ret += (filter->taps[19] * 2638) >> 8;
        ret += (filter->taps[20] * 2671) >> 8;
        ret += (filter->taps[21] * 2685) >> 8;
        ret += (filter->taps[22] * 2676) >> 8;
        ret += (filter->taps[23] * 2648) >> 8;
        ret += (filter->taps[24] * 2599) >> 8;
        ret += (filter->taps[25] * 2531) >> 8;
        ret += (filter->taps[26] * 2444) >> 8;
        ret += (filter->taps[27] * 2342) >> 8;
        ret += (filter->taps[28] * 2223) >> 8;
        ret += (filter->taps[29] * 2092) >> 8;
        ret += (filter->taps[30] * 1950) >> 8;
        ret += (filter->taps[31] * 1799) >> 8;
        ret += (filter->taps[32] * 1641) >> 8;
        ret += (filter->taps[33] * 1479) >> 8;
        ret += (filter->taps[34] * 1315) >> 8;
        ret += (filter->taps[35] * 1152) >> 8;
        ret += (filter->taps[36] * 990) >> 8;
        ret += (filter->taps[37] * 834) >> 8;
        ret += (filter->taps[38] * 683) >> 8;
        ret += (filter->taps[39] * 540) >> 8;
        ret += (filter->taps[40] * 406) >> 8;
        ret += (filter->taps[41] * 282) >> 8;
        ret += (filter->taps[42] * 170) >> 8;
        ret += (filter->taps[43] * 70) >> 8;
        ret += (filter->taps[44] * -18) >> 8;
        ret += (filter->taps[45] * -93) >> 8;
        ret += (filter->taps[46] * -156) >> 8;
        ret += (filter->taps[47] * -207) >> 8;
        ret += (filter->taps[48] * -247) >> 8;
        ret += (filter->taps[49] * -275) >> 8;
        ret += (filter->taps[50] * -293) >> 8;
        ret += (filter->taps[51] * -300) >> 8;
        ret += (filter->taps[52] * -301) >> 8;
        ret += (filter->taps[53] * -292) >> 8;
        ret += (filter->taps[54] * -278) >> 8;
        ret += (filter->taps[55] * -257) >> 8;
        ret += (filter->taps[56] * -233) >> 8;
        ret += (filter->taps[57] * -206) >> 8;
        ret += (filter->taps[58] * -176) >> 8;
        ret += (filter->taps[59] * -145) >> 8;
        ret += (filter->taps[60] * -115) >> 8;
        ret += (filter->taps[61] * -85) >> 8;
        ret += (filter->taps[62] * -56) >> 8;
        ret += (filter->taps[63] * -30) >> 8;

        if(FIR_REVERSED)
                queue_dma(filter->taps, filter->taps+1, 63);
        else
                queue_dma(filter->taps + 1, filter->taps, 63);

        return ret >> 8;
}

static bool hfi_on=true;


void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta,
		int16_t int16_i_q_target, MotorState_t *MS_FOC) {
	HAL_GPIO_WritePin(UART1_Tx_GPIO_Port, UART1_Tx_Pin, GPIO_PIN_SET);
	q31_t q31_i_alpha = 0;
	q31_t q31_i_beta = 0;
	q31_t q31_u_alpha = 0;
	q31_t q31_u_beta = 0;
	q31_t q31_i_d = 0;
	q31_t q31_i_q = 0;

	q31_t sinevalue = 0, cosinevalue = 0;

    
        static filter_state_t fundamental_alpha, fundamental_beta, heterodyne_alpha, heterodyne_beta;
	// temp5=(q31_t)int16_i_as;
	// temp6=(q31_t)int16_i_bs;

	// Clark transformation
	arm_clarke_q31((q31_t) int16_i_as, (q31_t) int16_i_bs, &q31_i_alpha,
			&q31_i_beta);

        if(hfi_on){
                q31_i_alpha = q31_low_pass(q31_i_alpha, &fundamental_alpha);
                q31_i_beta = q31_low_pass(q31_i_beta, &fundamental_beta);
        }

	arm_sin_cos_q31(q31_teta, &sinevalue, &cosinevalue);

	// Park transformation
	arm_park_q31(q31_i_alpha, q31_i_beta, &q31_i_d, &q31_i_q, sinevalue,
			cosinevalue);

	q31_i_q_fil -= q31_i_q_fil >> 4;
	q31_i_q_fil += q31_i_q;
	MS_FOC->i_q = q31_i_q_fil >> 4;

	q31_i_d_fil -= q31_i_d_fil >> 4;
	q31_i_d_fil += q31_i_d;
	MS_FOC->i_d = q31_i_d_fil >> 4;

	//Control iq

	PI_flag = 1;
	runPIcontrol();

//set static volatage for hall angle detection
	if (!MS_FOC->hall_angle_detect_flag) {
		MS_FOC->u_q = 0;
		MS_FOC->u_d = 200;
	}

	//inverse Park transformation
	arm_inv_park_q31(MS_FOC->u_d, MS_FOC->u_q, &q31_u_alpha, &q31_u_beta,
			-sinevalue, cosinevalue);


        if(hfi_on){
             static int hfi_index=0;

             //Heterodyne the current with injection signal to frequency shift by injection frequency
             q31_t heteroA = (q31_i_alpha * hfi_sin_table[hfi_index]);
             q31_t heteroB = (q31_i_beta * hfi_sin_table[hfi_index]);

             q31_t loA = q31_low_pass(heteroA, &heterodyne_alpha);
             q31_t loB = q31_low_pass(heteroB, &heterodyne_beta);

             q31_t rotor_angle = atan2_LUT(loA,loB); //That's it

             //Generate the injection
             q31_u_alpha += (hfi_sin_table[hfi_index] * HFI_VOLTAGE) >> 16;
             q31_u_beta += (hfi_cos_table[hfi_index++] * HFI_VOLTAGE) >> 16;
             if(hfi_index==16)
                hfi_index=0;
        }        

#ifdef FAST_LOOP_LOG
	temp1=int16_i_as;
	temp2=int16_i_bs;
	temp3=MS_FOC->i_d;
	temp4=MS_FOC->i_q;
	temp5=MS_FOC->u_d;
	temp6=MS_FOC->u_q;

	if(uint32_PAS_counter < PAS_TIMEOUT&&ui8_debug_state==0)
			{
		e_log[z][0]=temp1;//fl_e_alpha_obs;
		e_log[z][1]=temp2;//fl_e_beta_obs;
		e_log[z][2]=temp3;//(q31_t)q31_teta_obs>>24;
		e_log[z][3]=temp4;
		e_log[z][4]=temp5;
		e_log[z][5]=temp6;
		z++;
		if(z>150) Obs_flag=1;
		if (z>299)
		{z=0;

		ui8_debug_state=2;}
			}
	else {if(ui8_debug_state==2)ui8_debug_state=3;;}

#endif

	//call SVPWM calculation
	svpwm(q31_u_alpha, q31_u_beta);
	//temp6=__HAL_TIM_GET_COUNTER(&htim1);
	HAL_GPIO_WritePin(UART1_Tx_GPIO_Port, UART1_Tx_Pin, GPIO_PIN_RESET);

}
//PI Control for quadrature current iq (torque)
q31_t PI_control_i_q(q31_t ist, q31_t soll) {

	q31_t q31_p; //proportional part
	static q31_t q31_q_i = 0; //integral part
	static q31_t q31_q_dc = 0; // sum of proportional and integral part
	q31_p = ((soll - ist) * P_FACTOR_I_Q);
	q31_q_i += ((soll - ist) * I_FACTOR_I_Q);
	//temp5 = q31_p;
	//temp6 = q31_q_i;

	if (q31_q_i > _U_MAX << 10)
		q31_q_i = _U_MAX << 10;
	if (q31_q_i < -_U_MAX << 10)
		q31_q_i = -_U_MAX << 10;
	if (!READ_BIT(TIM1->BDTR, TIM_BDTR_MOE))
		q31_q_i = 0; //reset integral part if PWM is disabled

	//avoid too big steps in one loop run
	if ((q31_p + q31_q_i) >> 10 > q31_q_dc + 5)
		q31_q_dc += 5;
	else if ((q31_p + q31_q_i) >> 10 < q31_q_dc - 5)
		q31_q_dc -= 5;
	else
		q31_q_dc = (q31_p + q31_q_i) >> 10;

	if (q31_q_dc > _U_MAX)
		q31_q_dc = _U_MAX;
	if (q31_q_dc < -_U_MAX)
		q31_q_dc = -_U_MAX; // allow no negative voltage.

	return (q31_q_dc);
}

//PI Control for direct current id (loss)
q31_t PI_control_i_d(q31_t ist, q31_t soll, q31_t clamp) {
	q31_t q31_p;
	static q31_t q31_d_i = 0;
	static q31_t q31_d_dc = 0;

	q31_p = ((soll - ist) * P_FACTOR_I_D) >> 5;
	q31_d_i += ((soll - ist) * I_FACTOR_I_D) >> 5;

	if (q31_d_i < -clamp + abs(q31_p))
		q31_d_i = -clamp + abs(q31_p);
	if (q31_d_i > clamp - abs(q31_p))
		q31_d_i = clamp - abs(q31_p);

	if (!READ_BIT(TIM1->BDTR, TIM_BDTR_MOE))
		q31_d_i = 0; //reset integral part if PWM is disabled
	//avoid too big steps in one loop run
	if (q31_p + q31_d_i > q31_d_dc + 5)
		q31_d_dc += 5;
	else if (q31_p + q31_d_i < q31_d_dc - 5)
		q31_d_dc -= 5;
	else
		q31_d_dc = q31_p + q31_d_i;

	if (q31_d_dc > _U_MAX)
		q31_d_dc = _U_MAX;
	if (q31_d_dc < -(_U_MAX))
		q31_d_dc = -(_U_MAX);

	return (q31_d_dc);
}

void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta) {

//SVPWM according to chapter 4.9 of UM1052

	q31_t q31_U_alpha = (_SQRT3 * _T * q31_u_alpha) >> 4;
	q31_t q31_U_beta = -_T * q31_u_beta;
	q31_t X = q31_U_beta;
	q31_t Y = (q31_U_alpha + q31_U_beta) >> 1;
	q31_t Z = (q31_U_beta - q31_U_alpha) >> 1;

	//Sector 1 & 4
	if ((Y >= 0 && Z < 0 && X > 0) || (Y < 0 && Z >= 0 && X <= 0)) {
		switchtime[0] = ((_T + X - Z) >> 12) + (_T >> 1); //right shift 11 for dividing by peroid (=2^11), right shift 1 for dividing by 2
		switchtime[1] = switchtime[0] + (Z >> 11);
		switchtime[2] = switchtime[1] - (X >> 11);
		//temp4=1;
	}

	//Sector 2 & 5
	if ((Y >= 0 && Z >= 0) || (Y < 0 && Z < 0)) {
		switchtime[0] = ((_T + Y - Z) >> 12) + (_T >> 1);
		switchtime[1] = switchtime[0] + (Z >> 11);
		switchtime[2] = switchtime[0] - (Y >> 11);
		//temp4=2;
	}

	//Sector 3 & 6
	if ((Y < 0 && Z >= 0 && X > 0) || (Y >= 0 && Z < 0 && X <= 0)) {
		switchtime[0] = ((_T + Y - X) >> 12) + (_T >> 1);
		switchtime[2] = switchtime[0] - (Y >> 11);
		switchtime[1] = switchtime[2] + (X >> 11);
		//temp4=3;
	}

}

