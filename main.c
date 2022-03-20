/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
 
 
 
#define PTB0_Pin 0
#define PTB1_Pin 1
#define PTB2_Pin 2
#define PTB3_Pin 3
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

void initPWM(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);

	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3);
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	
	TPM1->MOD = 7500;
	TPM2->MOD = 7500;

	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB (1));
	
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB (1));
	
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB (1));
	
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB (1));
}

void initUART2(uint32_t baud_rate)
{
	uint32_t divisor, bus_clock;
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);

	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
}
	
void UART2_Transmit_Poll(uint8_t data)
{
	while(!(UART2->S1 & UART_S1_TDRE_MASK));
	UART2->D = data;
}
		
uint8_t UART2_Receive_Poll(void)
{
	while(!(UART2->S1 & UART_S1_RDRF_MASK));
	return (UART2->D);
}

static void delay(volatile uint32_t nof){
  while(nof != 0) {
    __asm("NOP");
    nof--;
  }
}

void left_motor (void *argument) {
 
	uint8_t rx_data;
  for (;;) {
		rx_data = UART2_Receive_Poll();
		if(rx_data == 0x40){
			TPM1_C0V = 3750/2;
			TPM1_C1V = 0;
		} else if(rx_data == 0x41) {
			TPM1_C0V = 0;
			TPM1_C1V = 3750/2;
		} else if(rx_data == 0x44) {
			TPM1_C0V = 3750;
			TPM1_C1V = 0;
		} else if(rx_data == 0x45) {
			TPM1_C0V = 0;
			TPM1_C1V = 3750;
		} else if(rx_data == 0x48) {
			TPM1_C0V = 0;
			TPM1_C1V = 0;
		}
	}
}

void right_motor (void *argument) {
 
	uint8_t rx_data;
  for (;;) {
		rx_data = UART2_Receive_Poll();
		if(rx_data == 0x42){
			TPM2_C0V = 3750/2;
			TPM2_C1V = 0;
		} else if(rx_data == 0x43) {
			TPM2_C0V = 0;
			TPM2_C1V = 3750/2;
		} else if(rx_data == 0x46) {
			TPM2_C0V = 3750;
			TPM2_C1V = 0;
		} else if(rx_data == 0x47) {
			TPM2_C0V = 0;
			TPM2_C1V = 3750;
		} else if(rx_data == 0x49) {
			TPM2_C0V = 0;
			TPM2_C1V = 0;
		}
	}
}

void start (void *argument) {

  for (;;) {

	}
}

void end (void *argument) {

  for (;;) {

	}
}

void stop (void *argument) {

	uint8_t rx_data;
  for (;;) {
		rx_data = UART2_Receive_Poll();
		if(rx_data == 0x50) {
			TPM1_C0V = 0;
			TPM1_C1V = 0;
			TPM2_C0V = 0;
			TPM2_C1V = 0;
		}
	}
}

int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initPWM();
	initUART2(BAUD_RATE);
	uint8_t rx_data;
	
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	osThreadNew(left_motor, NULL, NULL);
	osThreadNew(right_motor, NULL, NULL);
	osThreadNew(stop, NULL, NULL);
  osKernelStart();                      // Start thread execution
	
}
