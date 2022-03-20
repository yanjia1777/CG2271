# include <MKL25Z4.h>

#define PTB0_Pin 0
#define PTB1_Pin 1
#define C 1431
#define D 1276
#define E 1136
#define F 1074
#define G 956
#define A 852
#define B 759


void initPWM(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);

	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	
	TPM1->MOD = 7500;

	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB (1));
}

static void delay(volatile uint32_t nof){
  while(nof != 0) {
    __asm("NOP");
    nof--;
  }
}

int main(void){
	SystemCoreClockUpdate();
	initPWM();
	while(1){
		TPM1->MOD = 3750;
		TPM1_C0V = 3750/2;

	}
}