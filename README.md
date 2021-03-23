# M031BSP_ADC_PWM_PID
 M031BSP_ADC_PWM_PID


update @ 2021/03/23

1. simple PID exercise , with ADC#1 input , and adjust PWM output duty , covert to DC voltage and detect by ADC#2 , 

with ADC#2 apporach to ADC#1 by PID solution

ADC#1 --> MCU -----> PWM 

			^	   	  |
			
			|	      |
			
			|	      v
			
		  ADC#2 <-- RC circuit


2. initial PB.5 (PWM0_CH0) , with external RC circuit , to convert DC voltage

3. initial PB.0 (ADC0_CH0) as TARGET voltage , PB.4 (ADC0_CH4) as APPROACH voltage , trigger with PWM

use lowpass filter to sampling ADC value

4. Adjust PWM duty base on ADC#2 (APPROACH voltage) , to meet ADC#1 (TARGET voltage)

5. by connect PWM pin and covert with RC for DC voltage , below is waveform capture 

up : PWM 

down :external RC circuit , convert to DC volage , as ADC#2 input

![image](https://github.com/released/M031BSP_ADC_PWM_PID/blob/main/waveform.jpg)

6. below to log message capture

![image](https://github.com/released/M031BSP_ADC_PWM_PID/blob/main/terminal.jpg)

