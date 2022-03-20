# include <MKL25Z4.h>

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MASK(x) (1 << (x))

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


void InitGPIO(void)
{
  // Enable Clock to PORTB and PORTD
  SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
  // Configure MUX settings to make all 3 pins GPIO
  PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
  PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
  PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
  PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
  // Set Data Direction Registers for PortB and PortD
  PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
  PTD->PDDR |= MASK(BLUE_LED);
}

void setOff() {
  PTB->PDOR |= MASK(RED_LED);
  PTB->PDOR |= MASK(GREEN_LED);
  PTD->PDOR |= MASK(BLUE_LED);
}

void setRed() {
  PTB->PCOR = MASK(RED_LED);
  PTB->PSOR = MASK(GREEN_LED);
  PTD->PSOR = MASK(BLUE_LED);
}

void setGreen() {
  PTB->PSOR = MASK(RED_LED);
  PTB->PCOR = MASK(GREEN_LED);
  PTD->PSOR = MASK(BLUE_LED);
}

void setBlue() {
  PTB->PSOR = MASK(RED_LED);
  PTB->PSOR = MASK(GREEN_LED);
  PTD->PCOR = MASK(BLUE_LED);
}

int main(void){
	
	InitGPIO();
	uint8_t rx_data = 0x69;
	SystemCoreClockUpdate();
	initUART2(BAUD_RATE);
	setOff();
	while(1){
		//UART2_Transmit_Poll(0x69);
		rx_data = UART2_Receive_Poll();
		//delay(0x80000);
		//rx_data++;
		if(rx_data == 0x31){
			setRed();
		} else if (rx_data == 0x30) {
			setOff();
		} 
	}
}
