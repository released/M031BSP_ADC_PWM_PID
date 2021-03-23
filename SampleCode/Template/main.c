/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include	"project_config.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/

enum
{
	ADC0_CH0 = 0 ,
	ADC0_CH1 , 
	
	ADC0_CH2 , 
	ADC0_CH3 , 
	ADC0_CH4 , 
	ADC0_CH5 , 
	ADC0_CH6 , 
	ADC0_CH7 , 
	ADC0_CH8 , 
	ADC0_CH9 , 
	ADC0_CH10 , 
	ADC0_CH11 , 
	ADC0_CH12 ,
	ADC0_CH13 , 
	ADC0_CH14 , 
	ADC0_CH15 , 
	
	ADC_CH_DEFAULT 	
}ADC_CH_TypeDef;

	
/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;

#define ADC_RESOLUTION						((uint16_t)(4096u))
#define ADC_REF_VOLTAGE						((uint16_t)(3300u))	//(float)(3.3f)

#define ADC_MAX_TARGET						((uint16_t)(4095u))	//(float)(2.612f)
#define ADC_MIN_TARGET						((uint16_t)(0u))	//(float)(0.423f)

#define DUTY_MAX							(uint16_t)(100)
#define DUTY_MIN							(uint16_t)(0)
#define ADC_CONVERT_TARGET					(float)(ADC_MIN_TARGET*ADC_RESOLUTION/ADC_REF_VOLTAGE) //81.92000 
//#define ADC_SUB_TARGET						(float)((ADC_MAX_TARGET-ADC_MIN_TARGET)/(DUTY_MAX-DUTY_MIN)*(ADC_RESOLUTION/ADC_REF_VOLTAGE))//5.60505 
//#define ADCInputV_Sub						(float)	((ADC_MAX_BRIGHT-ADC_MIN_BRIGHT)/(DUTY_MAX-DUTY_MIN)) //0.02737 

#define ADC_SAMPLE_COUNT 					(uint16_t)(16)		// 8
#define ADC_SAMPLE_POWER 					(uint8_t)(4)			//(5)	 	// 3	,// 2 ^ ?

#define ABS(X)  								((X) > 0 ? (X) : -(X)) 

#define ADC_DIGITAL_SCALE(void) 				(0xFFFU >> ((0) >> (3U - 1U)))		//0: 12 BIT 
#define ADC_CALC_DATA_TO_VOLTAGE(DATA,VREF) ((DATA) * (VREF) / ADC_DIGITAL_SCALE())

#define ADCextendSampling 					(0)

uint32_t AVdd = 0;
uint8_t ADC_CH = 0;
volatile uint16_t u16adc_data[2] = {0};
volatile uint16_t u16adc_OldFilterValue[2] = {0};

#define ADC_TARGET								(0)
#define ADC_APPROACH							(1)

double ref_voltage;
double apporach_voltage;
double target_voltage = 1.5;

double proportional_constant = 1.0;
double integral_constant = 0.25;
double differential_constant = 0.01;
double derivative_error;
double cumulative_error;
double last_error;
double current_error;
double out;

/*_____ M A C R O S ________________________________________________________*/

#define SYS_CLK 									(48000000ul)
#define PWM_PSC 								(100)	
#define PWM_FREQ 								(10000)	
#define PWM_DUTY                              	(0)
#define PWM_CHANNEL                           	(0)
#define PWM_CHANNEL_MASK                     (PWM_CH_0_MASK)

#define PWM_RESOLUTION                        	(100)
#define PWM_CNR 								((SYS_CLK/PWM_FREQ)/PWM_PSC - 1)
#define PWM_CMR 								(PWM_DUTY * (PWM_CNR + 1)/PWM_RESOLUTION)

#define CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)	(u32DutyCycle * (PWM_GET_CNR(pwm, u32ChannelNum) + 1) / u32CycleResolution)
#define CalNewDuty(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)		(PWM_SET_CMR(pwm, u32ChannelNum, CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)))


/*_____ F U N C T I O N S __________________________________________________*/

void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

void delay(uint16_t dly)
{
/*
	delay(100) : 14.84 us
	delay(200) : 29.37 us
	delay(300) : 43.97 us
	delay(400) : 58.5 us	
	delay(500) : 73.13 us	
	
	delay(1500) : 0.218 ms (218 us)
	delay(2000) : 0.291 ms (291 us)	
*/

	while( dly--);
}


void delay_ms(uint16_t ms)
{
	TIMER_Delay(TIMER0, 1000*ms);
}

unsigned int LowPassFilter(unsigned int OldFilterValue , unsigned int NewADCValue)
{
    static unsigned int FiltedValue;

	/*
		half Sampling rate=10, BW = 4.41Hz
		Filter factor
		= exp(-pi*BW / Fs)
		= exp (-3.14*4.4/10) = 0.251
		= 256/1024 = (2^8) / (2^10)
		right shift two times
	*/

    if(NewADCValue < OldFilterValue)
    {
        FiltedValue=OldFilterValue-NewADCValue;
        FiltedValue=FiltedValue>>2;
        FiltedValue=OldFilterValue-FiltedValue;
    }
    else if(NewADCValue > OldFilterValue)
    {
        FiltedValue=NewADCValue-OldFilterValue;
        FiltedValue=FiltedValue>>2;
        FiltedValue=OldFilterValue+FiltedValue;
    }

    return FiltedValue;
}

void PWM_Init(void)
{
	/*
		Target : 200K Freq
		DUTY : 50%
		
		SYS_CLK : 48M
		PSC : 1

		48 000 000/200 000 = PSC * (CNR + 1)
		CNR = (SYS_CLK/FREQ)/PSC - 1 = 239

		50 /100 = CMR / (CNR + 1)
		CMR = 50 * (CNR + 1)/100
		
	*/


    /*
      Configure PWM0 channel 0 init period and duty(down counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = CMR / (CNR + 1)
      
      Period = 48 MHz / (1 * (199 + 1)) = 240000 Hz
      Duty ratio = 100 / (199 + 1) = 50%
    */
	
    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, PWM_CHANNEL, PWM_PSC - 1);
	
    /* Set up counter type */
    PWM0->CTL1 &= ~PWM_CTL1_CNTTYPE0_Msk;

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, PWM_CHANNEL, PWM_CNR);
	
    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, PWM_CHANNEL, PWM_CMR);	
	
    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CHANNEL_MASK, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);
	
    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, PWM_CHANNEL_MASK);
	
	PWM_Start(PWM0, PWM_CHANNEL_MASK);
}

__STATIC_INLINE uint32_t FMC_ReadBandGap(void)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;            /* Set ISP Command Code */
    FMC->ISPADDR = 0x70u;                         /* Must keep 0x70 when read Band-Gap */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;           /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                            /* To make sure ISP/CPU be Synchronized */
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {}  /* Waiting for ISP Done */

    return FMC->ISPDAT & 0xFFF;
}

void ADC_IRQHandler(void)
{
	static uint16_t value = 0;
	
	set_flag(flag_ADC_Data_Ready , ENABLE);

	if (ADC_CH == ADC0_CH0)
	{
		u16adc_data[ADC_TARGET] = ADC_GET_CONVERSION_DATA(ADC, ADC0_CH0);
		value = LowPassFilter(u16adc_OldFilterValue[ADC_TARGET],u16adc_data[ADC_TARGET]);
		u16adc_OldFilterValue[ADC_TARGET] = value;
		u16adc_data[ADC_TARGET] = value;		
	}
	if (ADC_CH == ADC0_CH4)
	{
		u16adc_data[ADC_APPROACH] = ADC_GET_CONVERSION_DATA(ADC, ADC0_CH4);
		value = LowPassFilter(u16adc_OldFilterValue[ADC_APPROACH],u16adc_data[ADC_APPROACH]);
		u16adc_OldFilterValue[ADC_APPROACH] = value;
		u16adc_data[ADC_APPROACH] = value;
	}	

	
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
}

void ADC_ReadAVdd(void)
{
    int32_t  i32ConversionData;
    int32_t  i32BuiltInData;

    ADC_POWER_ON(ADC);
    CLK_SysTickDelay(10000);

	
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT29);
    ADC_SetExtendSampleTime(ADC, 0, 71);
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

	set_flag(flag_ADC_Data_Ready , DISABLE);
    ADC_ENABLE_INT(ADC, ADC_ADF_INT);
    NVIC_EnableIRQ(ADC_IRQn);	
    ADC_START_CONV(ADC);

	while(!is_flag_set(flag_ADC_Data_Ready));
	
    ADC_DISABLE_INT(ADC, ADC_ADF_INT);
		
    i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 29);
    SYS_UnlockReg();
    FMC_Open();
    i32BuiltInData = FMC_ReadBandGap();	

	AVdd = 3072*i32BuiltInData/i32ConversionData;

	printf("%s : %d,%d,%d\r\n",__FUNCTION__,AVdd, i32ConversionData,i32BuiltInData);

    NVIC_DisableIRQ(ADC_IRQn);
	
}

void ADC_InitChannel(uint8_t ch)
{
    ADC_POWER_ON(ADC);
	
    /* Set input mode as single-end, and Single mode*/
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE,(uint32_t) 0x1 << ch);

    /* To sample band-gap precisely, the ADC capacitor must be charged at least 3 us for charging the ADC capacitor ( Cin )*/
    /* Sampling time = extended sampling time + 1 */
    /* 1/24000000 * (Sampling time) = 3 us */
	/*
	    printf("+----------------------------------------------------------------------+\n");
	    printf("|   ADC clock source -> PCLK1  = 48 MHz                                |\n");
	    printf("|   ADC clock divider          = 2                                     |\n");
	    printf("|   ADC clock                  = 48 MHz / 2 = 24 MHz                   |\n");
	    printf("|   ADC extended sampling time = 71                                    |\n");
	    printf("|   ADC conversion time = 17 + ADC extended sampling time = 88         |\n");
	    printf("|   ADC conversion rate = 24 MHz / 88 = 272.7 ksps                     |\n");
	    printf("+----------------------------------------------------------------------+\n");
	*/

    /* Set extend sampling time based on external resistor value.*/
    ADC_SetExtendSampleTime(ADC,(uint32_t) NULL, ADCextendSampling);

	ADC_EnableHWTrigger(ADC, ADC_ADCR_TRGS_PWM, 0);

    /* Select ADC input channel */
    ADC_SET_INPUT_CHANNEL(ADC, 0x1 << ch);

	ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
	ADC_ENABLE_INT(ADC, ADC_ADF_INT);
	NVIC_EnableIRQ(ADC_IRQn);

    /* Start ADC conversion */
    ADC_START_CONV(ADC);
	
}

void PID_Init(void)
{
	cumulative_error = 0;
	derivative_error = 0;
	last_error = 0;
	current_error = 0;
}


void PID_Calculation(void)
{
	/*
		PID formal is: out =(kp*E)+(ki*se)+(kd*de) 
		kp: proportional constant 
		ki: integral constant 
		kd: differential constant 
		E: current error 
		se: cumulative error 
		de: last error 

	*/
	uint16_t abs_value = 0;
	uint16_t abs_volt = 0;
	
	ref_voltage = AVdd;

	while (1)
	{
   	 	target_voltage = ADC_CALC_DATA_TO_VOLTAGE(u16adc_data[ADC_TARGET],ref_voltage);    			//Volume Range: 0 ~ 4095   		
	    apporach_voltage = ADC_CALC_DATA_TO_VOLTAGE(u16adc_data[ADC_APPROACH],ref_voltage);         	//desire voltage
	    
		abs_value = ABS (u16adc_data[ADC_TARGET] - u16adc_data[ADC_APPROACH]);
		abs_volt = ABS (target_voltage - apporach_voltage);
		
		#if (_debug_log_PID_ == 1)	//debug	
//	    printf("voltage = %4.3f (%3.3f, %dmv) , current_error = %4.3f  ,out = %6.3f\r\n",apporach_voltage ,ref_voltage,AVdd ,current_error , out );
//	    printf("voltage = %4.3f (%4.3f ) , current_error = %4.3f  ,out = %6.3f\r\n",apporach_voltage , target_voltage ,current_error , out );

		printf("target = %4dmv  , apporach = %4dmv  , ref  = %4dmv , ABS : %3d / %3d ", (uint16_t) target_voltage, (uint16_t)apporach_voltage,AVdd ,abs_value,abs_volt);

		if (target_voltage > apporach_voltage)
		{
			printf("+++\r\n");
		}
		else
		{
			printf(" ---\r\n");
		}
		#endif

		
		if (abs_value < 20 )
		{			
			break;
		}
		else
		{

		    current_error = (target_voltage - apporach_voltage) / ( ref_voltage / 4095);		//SET is target voltage
		    cumulative_error += current_error;
		    derivative_error = current_error - last_error;
		    out = (proportional_constant * current_error) + (integral_constant * cumulative_error) + (differential_constant * derivative_error);
		    last_error = current_error;
			
			CalNewDuty(PWM0, PWM_CHANNEL, out / 4095, PWM_RESOLUTION);	
		}
	}
	
}

void GPIO_Init (void)
{
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);

    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	
}


void TMR1_IRQHandler(void)
{
//	static uint32_t LOG = 0;
	static uint8_t flag = 0;
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
			PB14 ^= 1;
		}

		if ((get_tick() % 10) == 0)
		{
			flag ^= 1;

			if (flag == 0)
			{
				ADC_CH = ADC0_CH0;
			}
			else
			{
				ADC_CH = ADC0_CH4;
			}
			
			ADC_InitChannel(ADC_CH);
		}	
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res == 'x' || res == 'X')
	{
		NVIC_SystemReset();
	}

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}

void UART02_IRQHandler(void)
{

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_HXTSTB_Msk);

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk|CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk|CLK_STATUS_LXTSTB_Msk);	

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(ADC_MODULE);	
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(2));

    CLK_EnableModuleClock(PWM0_MODULE);
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);
	
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    SYS->GPB_MFPL = (SYS->GPB_MFPL &~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk )) \
                    | (SYS_GPB_MFPL_PB0MFP_ADC0_CH0 | SYS_GPB_MFPL_PB4MFP_ADC0_CH4) ;

    /* Set PB.0 ~ PB.3 to input mode */
    GPIO_SetMode(PB, BIT0|BIT4, GPIO_MODE_INPUT);

    /* Disable the PB0 ~ PB3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0|BIT4);

    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_PWM0_CH0;

   /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	UART0_Init();
	GPIO_Init();

	PID_Init();
	PWM_Init();	
	
	ADC_ReadAVdd();
	
	TIMER1_Init();
	
    /* Got no where to go, just loop forever */
    while(1)
    {
		PID_Calculation();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
