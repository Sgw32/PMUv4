#include <math.h>
#include <stdlib.h>
#include <float.h>
#include "stm32f1xx_hal.h"

#define INJECTOR_MIN 1500
#define BAUDRATE     57600            // Baud rate of UART in bps
#define PCA0_MKS 1

char RK_code[66], nByte = 0, KontrSumma = 0, NPackage = 0;

float bat;
float bat3_pr;
float cx[4];
float current;
float smoothCurrent;
float smoothCurrentMax;
float Wh;

float cap;
char SteckPoint;
float chgCurrent;
float chgCurrSmooth;
float maxChg;
float temperature=25;
//unsigned int time;

float injector_pwm = INJECTOR_MIN;//1640
float pwm3 = 1000;
//float pwm = 1000;
float pwm1 = 1000;
float pwm2 = 1000;
float pwm4 = 700*PCA0_MKS;

char startDvs;
char phase = 0;
char CountRun=0;

void OutModem1(unsigned char Data, char i);
void OutModem2(unsigned int Data, char i);
void OutModem4(unsigned long int Data, char i);

#define NBFM 		50
char BuferFromModem [NBFM]; // Для анализа с последовательного порта
char wBFM = 0, rBFM = 0, marBFM = 0;

#define SIZE_BUFFER0	61
char BufferInModem[SIZE_BUFFER0]; // Для отправки в последовательный порт
int r0, rk;

char flTransmiter, flRun;
char flMem;
char rgAnswer;
char rst_src, rst_count=0;

//UART 8051 emulate
char TI0;

void processWriteUART(UART_HandleTypeDef huart3)
{
	int i;
	if(flRun)
	{
		flRun = 0;
	}
	//	rgAnswer = 1;
	if(flTransmiter)
	;
	else
	{
		if(rgAnswer == 1)
		{
			rgAnswer = 0;

			BufferInModem[0] = 1 | 0x40;
			OutModem2( 100*bat, 1);
			OutModem2( 10*smoothCurrent, 3);
			OutModem2( 10*cap, 5);
			OutModem1( 4 | (char)flMem , 7);
			OutModem1( rst_src , 8);
			OutModem1( rst_count , 9);

			BufferInModem[10] = 0;
			for (i = 0; i < 10; i++ )
				BufferInModem[10] = BufferInModem[10] ^ BufferInModem[i];
			OutModem1(BufferInModem[10], 10);
			BufferInModem[11] = 0;
			r0 = 0;
			rk = 11;

			HAL_UART_Transmit_DMA(&huart3, BufferInModem, 12);
			//flTransmiter = 1;
			//SFRPAGE = UART0_PAGE;
			//TI0 = 1;
		}
		if(rgAnswer == 4)
		{
			rgAnswer = 0;

			BufferInModem[0] = 4 | 0x40;
			OutModem2( 100*bat, 1);
			OutModem2( 10*smoothCurrent, 3);
			OutModem2( 10*cap, 5);
			OutModem2( 100*chgCurrSmooth, 7);
			OutModem2( 10*temperature, 9);
			OutModem2( injector_pwm, 11);
			OutModem1( startDvs , 13);
			OutModem1( rst_src , 14);
			OutModem1( rst_count , 15);

			BufferInModem[16] = 0;
			for (i = 0; i < 16; i++ )
			BufferInModem[16] = BufferInModem[16] ^ BufferInModem[i];
			OutModem1(BufferInModem[16], 16);
			BufferInModem[17] = 0;
			r0 = 0;
			rk = 17;

			HAL_UART_Transmit_DMA(&huart3, BufferInModem, 17);
			//flTransmiter = 1;
			//SFRPAGE = UART0_PAGE;
			//TI0 = 1;
		}
	}
}

void processReadUART()
{
	if(rBFM < (wBFM+marBFM*NBFM))
	{
		if ((BuferFromModem[rBFM] & 0xC0) == 0x40)
		{
			nByte = 0;
			KontrSumma = 0;
			NPackage = BuferFromModem[rBFM] & 0x3f;
		}

		RK_code[nByte] = BuferFromModem[rBFM] & 0x7f;
		KontrSumma = KontrSumma^RK_code[nByte];
		if (++nByte > 65)
		nByte = 65;

		if ( (nByte == 3) && (KontrSumma == 0) )
		{
			if (NPackage == 1)
			{
				rgAnswer = 1;
			}
			if (NPackage == 2)//start dvs
			{
				rgAnswer = 2;
				startDvs=1;
			}
			if (NPackage == 3)//stop dvs
			{
				rgAnswer = 3;
				startDvs=0;
			}
			if (NPackage == 4)//ICE telemetry
			{
				rgAnswer = 4;
			}
			if (NPackage == 5)//ICE injector ++
			{
				rgAnswer = 5;
				injector_pwm+=10;
			}
			if (NPackage == 6)//ICE injector --
			{
				rgAnswer = 6;
				injector_pwm-=10;
			}
			/*if (NPackage == 8)//ICE debug
			{
				rgAnswer = 8;
			}*/

		}
		if ( (nByte == 4) && (KontrSumma == 0) )
		{
			if (NPackage == 7)//ICE injector set
			{

				injector_pwm= ((int)RK_code[2])<<7 | (int)RK_code[3];
			}
		}
		rBFM++;
		if(rBFM >= NBFM)
		{
			rBFM = 0;
			marBFM = 0;
		}
	}
}

unsigned char UART0_emulate_isr(char RI0,unsigned char SBUF0,UART_HandleTypeDef huart3)
{
	unsigned char SBUF1=0;
	if (RI0)  //-------RX Get Byte-------//
	{
		BuferFromModem [wBFM++] = SBUF0;
		if(wBFM >= NBFM)
		{
			wBFM = 0;
			marBFM = 1;
		}
		RI0 = 0;
	}
	/*if (TI0)  //-------TX Set Byte-------//
	{
		if(r0 < rk)
		{
		SBUF1 = BufferInModem[r0++];
		HAL_UART_Transmit_IT(&huart3, SBUF1, 1);
		}
		else
		flTransmiter = 0;

		TI0 = 0;
	}*/
	return SBUF1;
}


//------------------------------------------------------------------------------
void OutModem1(unsigned char Data, char i)
{
	BufferInModem[i] = Data | 0x80;
}
//------------------------------------------------------------------------------
void OutModem2(unsigned int Data, char i)
{
	BufferInModem[i] = (Data & 0x007f)| 0x80;
	BufferInModem[i+1] = ((Data & 0x3f80) >> 7)| 0x80;
}
