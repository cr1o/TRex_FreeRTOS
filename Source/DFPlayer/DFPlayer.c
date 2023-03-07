#include "DFPlayer.h"


volatile uint8_t mp3_RXi;
volatile char mp3_RXc;
volatile char mp3_RX_Buf[MP3_RX_BUF_SIZE] = {'\0'};
volatile uint8_t mp3_folder = 1;
volatile uint8_t mp3_cmd_buf[10] = {0x7E, 0xFF, 0x06, 0x00, 0x01, 0x0, 0x0, 0x00, 0x00, 0xEF};
//volatile uint8_t mp3_queue[MP3_QUEUE_LEN] = {MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE};
//volatile int8_t mp3_queue_id = 0;
volatile uint8_t mp3_flag = 0;

//void MP3_ClearRXBuffer(void);


//------------------------------    MP3_init    --------------------------------------------------
void MP3_Init(void){
	/* USART configured as follow:
		          - BaudRate = 9600 baud
		          - Word Length = 8 Bits
		          - One Stop Bit
		          - No parity
		          - Hardware flow control disabled (RTS and CTS signals)
		          - Receive and transmit enabled
		          - USART Clock disabled
		          - USART CPOL: Clock is active low
		          - USART CPHA: Data is captured on the middle
		          - USART LastBit: The clock pulse of the last data bit is not output to
		                           the SCLK pin
	 */
	//UART2_Init();
	RCC->AHBENR|= RCC_AHBENR_GPIOAEN; //Clock Enable Port B

	RCC->APB1ENR|= RCC_APB1ENR_USART2EN; //USART2 clock enable   //?????????? Проверить
	//RCC->AHBENR|= RCC_APB2ENR_USART1EN; //USART2 clock enable


	// INIT TX (PA2) Alternative PUSH PULL
	GPIOA ->MODER |= GPIO_MODER_MODER2_1; // Alternative
	GPIOA ->OTYPER &= ~GPIO_OTYPER_OT_2;  // PUSH PULL
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL2_Msk;
	GPIOA->AFR[0] |= (0x01 << 2*4);

	// INIT RX (PA3) Alternative OPEN DRAIN INPUT
	GPIOA ->MODER |= GPIO_MODER_MODER3_1; //Alternative
	GPIOA ->OTYPER |= GPIO_OTYPER_OT_3; // "RX" - OPEN DRAIN
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL3_Msk;
	GPIOA->AFR[0] |= (0x01<< 3*4);

	//UART_BRR = (Fck + baudrate/2)/baudrate;  Fck - clock AHBENR
	//(8 000 000 + 4800)/9600 = 833 (0x341)
	USART2->BRR = 0x1388; //Baud rate 9600.

	USART2->CR1 |= USART_CR1_RE; //Receiver Enable
	USART2->CR1 |= USART_CR1_TE; //Transmitter Enable
	USART2->CR1 |= USART_CR1_UE; //USART Enable

	USART2->CR1 |= USART_CR1_RXNEIE;//Interrupt Enable Byte receive


	//-------------------------------------------------
	//USART2->ICR |= USART_ICR_FECF;
	//USART2->ICR |= USART_ICR_TCCF;
	//USART2->
	//USART2->RQR |= USART_RQR_RXFRQ;
	//USART2->ISR &= ~USART_ISR_RXNE;
	//--------------------------------------------------
	NVIC_EnableIRQ (USART2_IRQn);   //= 28    USART2 global Interrupt
//	USART2_Send('1');
//	USART2_Send('2');
//	USART2_Send('3');

	//USART2->ICR |= USART_ICR_TCCF;

}
//------------------------------    Calculate checksum    --------------------------------------------------
uint16_t MP3_CheckSum (void) {
	uint16_t sum = 0;
	uint8_t i;
	for (i = 1; i < 7; i ++) {
		sum += mp3_cmd_buf[i];
	}
	return -sum;
}
//------------------------------    SendCmd    --------------------------------------------------
/* Send command to DFPlayer
 */
void MP3_SendCmd (uint8_t cmd, uint16_t high_arg, uint16_t low_arg) {
	uint8_t i;
	uint16_t checksum;

	mp3_cmd_buf[3] = cmd;

	mp3_cmd_buf[5] = high_arg;
	mp3_cmd_buf[6] = low_arg;

	checksum = MP3_CheckSum();
	mp3_cmd_buf[7] = (uint8_t) ((checksum >> 8) & 0x00FF);
	mp3_cmd_buf[8] = (uint8_t) (checksum & 0x00FF);

	// Send command to UART2
	for (i = 0; i < 10; i ++) {
		USART2_Send(mp3_cmd_buf[i]); // Проверить!!!
    }

}
//------------------------------    MP3_set_folder    --------------------------------------------------
void MP3_SetFolder (uint8_t folder) {
	mp3_folder = folder;
}


//------------------------------    ClearRXBuffer    --------------------------------------------------
/* Clear receive data buffer.
 * DFPlayer sends messages when certain events.
 * This buffer is used to receive messages from the player.
*/
void MP3_ClearRXBuffer(void) {
	for (mp3_RXi = 0; mp3_RXi < MP3_RX_BUF_SIZE; mp3_RXi ++)
		mp3_RX_Buf[mp3_RXi] = '\0';
	mp3_RXi = 0;
}

void USART2_IRQHandler(void){

	if (USART2->ISR & USART_ISR_RXNE){      //Флаг приема байта
		USART2->RQR |= USART_RQR_RXFRQ;
		//USART2_Send((USART2->RDR)+1);

		mp3_RXc = USART2->RDR;//USART_ReceiveData(USART2);
		mp3_RX_Buf[mp3_RXi] = mp3_RXc;
		mp3_RXi++;

		if (mp3_RXc != 0xEF) { // End of DFPlayer message
			if (mp3_RXi > MP3_RX_BUF_SIZE-1) {
				MP3_ClearRXBuffer();
			}
		}
		else {
			//if (RX_BUF[3] == 0x3C) { // U-DISK finished playing tracks
			//if (RX_BUF[3] == 0x3E) { // FLASH finished playing tracks
			if (mp3_RX_Buf[3] == 0x3D) { // TF card finished playing tracks
				mp3_flag = 1;
			}
			MP3_ClearRXBuffer();
		}
	}
	//----------   framing error   -------------------
	if (USART2->ISR & USART_ISR_FE){
		USART2->ICR |= USART_ICR_FECF;
	}
}

