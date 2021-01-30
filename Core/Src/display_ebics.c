/*
 * display_ebics.c
 *
 *  Created on: 12.11.2020
 *      Author: Gaswerke
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "print.h"

UART_HandleTypeDef huart3;
uint8_t ui8_rx_buffer[12];
uint8_t ui8_tx_buffer[12];

void ebics_init() {

	if (HAL_UART_Receive_DMA(&huart3, (uint8_t*) ui8_rx_buffer, 12) != HAL_OK) {
		Error_Handler();
	}
}

void process_ant_page(MotorState_t *MS, MotorParams_t *MP) {

	int chkSum = 0;

	for (uint8_t i = 0; i < 11; i++) {
		chkSum ^= ui8_rx_buffer[i];
	}

	// if(chkSum == ui8_rx_buffer[11]){

	switch (ui8_rx_buffer[3]) {
	case 16: {
		/*
		 message[4] = State.Wheel_Circumference & 0xFF; //Low Byte
		 message[5] = State.Wheel_Circumference >> 8 & 0xFF; // HiByte
		 message[6] = State.Travel_Mode_State;
		 message[7] = State.Display_Command & 0xFF; //Low Byte
		 message[8] = State.Display_Command >> 8 & 0xFF; // HiByte
		 message[9] = State.Manufacturer_ID & 0xFF; //Low Byte
		 message[10] = State.Manufacturer_ID >> 8 & 0xFF; //Hi Byte
		 */
		MP->wheel_cirumference = ui8_rx_buffer[5] << 8 | ui8_rx_buffer[4];
		MS->regen_level = ui8_rx_buffer[6] & 0x07;
		MS->assist_level = ui8_rx_buffer[6] >> 3 & 0x07;


	} // end case 16
		break;

	 case 6:
	 	{
	 		if(ui8_rx_buffer[4])autodetect();
	 		MS->i_q_setpoint = ui8_rx_buffer[8] << 8 | ui8_rx_buffer[7];
	 	}
	 	break;

	default: {
		//do nothing
	}
	} //end switch
	//}// end if chkSum
}	 //end process_ant_page

void send_ant_page(uint8_t page, MotorState_t *MS, MotorParams_t *MP) {

	switch (page) {
	case 1: {
		/*
		 State.Temperature_State=RxAnt[4];
		 State.Travel_Mode_State=RxAnt[5];
		 State.System_State=RxAnt[6];
		 State.Gear_State=RxAnt[7];
		 State.LEV_Error=RxAnt[8];
		 State.Speed=RxAnt[10]<<8|RxAnt[9];
		 */
		uint8_t temperature_state = 1; //to do: set Temperature state Byte according to ANT+LEV
		uint16_t speedx10 = MP->wheel_cirumference
				/ ((MS->Speed * MP->pulses_per_revolution) >> 3) * 36; // *3,6 for km/h then *10 for LEV standard definition.
		speedx10 = 250;
		ui8_tx_buffer[0] = 164; //Sync binary 10100100;
		ui8_tx_buffer[1] = 12;  //MsgLength
		ui8_tx_buffer[2] = 0x4E;  // MsgID for 0x4E for "broadcast Data"
		ui8_tx_buffer[3] = page;

		ui8_tx_buffer[4] = temperature_state;
		ui8_tx_buffer[5] = MS->regen_level | MS->assist_level << 3;
		ui8_tx_buffer[6] = MS->system_state;
		ui8_tx_buffer[7] = MS->gear_state;
		ui8_tx_buffer[8] = MS->error_state;
		ui8_tx_buffer[9] = speedx10 & 0xFF; //low byte of speed
		ui8_tx_buffer[10] = speedx10 >> 8 & 0x07; // lower 3 Bytes of high byte

		int chkSum = 0;

		for (uint8_t i = 0; i < 11; i++) {
			chkSum ^= ui8_tx_buffer[i];
		}
		ui8_tx_buffer[11] = chkSum;

		HAL_UART_Transmit_DMA(&huart3, (uint8_t*) &ui8_tx_buffer, 12);

	} //end case 1
		break;

	default: {
		//do nothing
	}
	}	//end switch

} //end send page
