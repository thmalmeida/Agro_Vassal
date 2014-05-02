#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include <Arduino.h>
#include <string.h>

#include "fonts/SystemFont5x7.h"       // system font
#include "glcd.h"    // Graphics LCD library
#include "fonts/allFonts.h"

#include "main.h"
#include "DS1307RTC.h"
#include "Time.h"

#include "MemoryFree.h"

tmElements_t tm;

#define k1_on()		PORTG |=  (1<<5);	// Triac 1 is the starter
#define k1_off()	PORTG &= ~(1<<5);	// Triac 1 is the starter
#define k2_on()		PORTE |=  (1<<5);	// Triac 1 is the starter
#define k2_off()	PORTE &= ~(1<<5);	// Triac 1 is the starter
#define k3_on()		PORTE |=  (1<<4);
#define k3_off()	PORTE &= ~(1<<4);

#define s11_on()	PORTC |=  (1<<2);	// S11
#define s11_off()	PORTC &= ~(1<<2);	// S11
#define	s10_on()	PORTC |=  (1<<1);	// S10
#define	s10_off()	PORTC &= ~(1<<1);	// S10
#define s09_on()	PORTC |=  (1<<0);	// S09
#define s09_off()	PORTC &= ~(1<<0);	// S09
#define s08_on()	PORTD |=  (1<<7);	// S08
#define s08_off()	PORTD &= ~(1<<7);	// S08
#define s07_on()	PORTG |=  (1<<2);	// S07
#define s07_off()	PORTG &= ~(1<<2);	// S07
#define s06_on()	PORTG |=  (1<<1);	// S06
#define s06_off()	PORTG &= ~(1<<1);	// S06
#define s05_on()	PORTG |=  (1<<0);	// S05
#define s05_off()	PORTG &= ~(1<<0);	// S05
#define s04_on()	PORTL |=  (1<<7);	// S04
#define s04_off()	PORTL &= ~(1<<7);	// S04
#define s03_on()	PORTL |=  (1<<6);	// S03
#define s03_off()	PORTL &= ~(1<<6);	// S03
#define s02_on()	PORTL |=  (1<<5);	// S02
#define s02_off()	PORTL &= ~(1<<5);	// S02
#define s01_on()	PORTL |=  (1<<4);	// S01
#define s01_off()	PORTL &= ~(1<<4);	// S01

#define f01_on()	PORTB |=  (1<<4);	// f01
#define f01_off()	PORTB &= ~(1<<4);	// f01
#define f02_on()	PORTB |=  (1<<5);	// f02
#define f02_off()	PORTB &= ~(1<<5);	// f02

// Contactors input
//#define k3_read		~(PINE & 0b00001000)
#define k3_read (~PINE & 0b00001000)
//#define k3_read	bit_is_set(PINE, 3)
//bit_is_set(PIND, 3)

const char *monthName[12] = {
	"Jan", "Feb", "Mar", "Apr", "May", "Jun",
	"Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

enum states01 {
	redTime,
	greenTime
};
enum states01 periodState = redTime;
//enum states_sector {
//	s01,
//	s02,
//	s03,
//	s04,
//	s05,
//	s06,
//	s07,
//	s08,
//	s09,
//	s10,
//	s11
//};
//enum states_sector stateSector = s01;

uint8_t stateSector =0;

enum statesMode {
	manual,
	programmed,
	automatic,
	valveTesting
};
enum statesMode stateMode = manual;
// Time sector in minutes
//#define st11 5
//#define st10 5
//#define st09 5
//#define st08 5
//#define st07 5
//#define st06 5
//#define st05 5
//#define st04 5
//#define st03 5
//#define st02 5
//#define st01 5

//#define Length 3*160.0
const int Ncycles = 3;
const int Length = Ncycles*160;		// 3 cycles
//const int Length = 480;		// 3 cycles
//const int Length = 640;		// 4 cycles
//const int Length = 800;		// 5 cycles
//const int Length = 960;		// 6 cycles
//const int Length = 1120;		// 7 cycles
//const int Length = 1280;		// 8 cycles

//uint8_t motorTime = 0;
uint8_t flag_motorStart = 0;
uint8_t timeCounter = 0;

uint8_t sector = 0;

uint16_t timeSectorSet = 4*60;
volatile uint16_t timeSector = 0;
uint8_t timeSectorVectorMin[11];
uint8_t flag_1s;
//uint8_t flag_SMS = 0;
//uint8_t flag_valve = 0;
uint8_t valveOnTest = 0;
uint8_t motorStatus = 0;
uint8_t valveStatus[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
char buffer[180];

char c;
uint8_t flag_sector=1;
volatile uint8_t flag_timeOVF=0;
char buffer_GLCD[40];
char buffer_SIM900[65];
uint16_t sample = 0;

// Time decision variables
uint8_t HourOn  = 21;
uint8_t MinOn   = 30;
uint8_t HourOff = 6;
uint8_t MinOff  = 0;

uint8_t flag_timeMatch = 0;
uint8_t flag_Started = 0;

uint8_t flag01 = 0;
uint8_t flag02 = 0;
uint8_t flag03 = 0;
uint8_t flag04 = 0;
uint8_t flag05 = 0;

volatile uint8_t flag_summaryGLCD = 0;
// Bluetooth variables
char inChar, aux[3], aux2[5], sInstr[15];
uint8_t k=0, rLength, opcode;
char sInstrSIM900[65];
char sInstrBluetooth[20];

uint8_t enableTranslate_BT = 0;
uint8_t enableTranslate_SIM900 = 0;
uint8_t enableSIM900_Send = 0;
uint8_t enableDecode = 0;

volatile uint8_t flag_30s = 1;
volatile uint8_t count30s = 0;



void print2digits(int number)
{
	if (number >= 0 && number < 10)
	{
		Serial.write('0');
	}
	Serial.print(number);
}
bool getTime(const char *str)
{
	int Hour, Min, Sec;

	if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
	tm.Hour = Hour;
	tm.Minute = Min;
	tm.Second = Sec;
	return true;
}
bool getDate(const char *str)
{
	char Month[12];
	int Day, Year;
	uint8_t monthIndex;

	if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
	for (monthIndex = 0; monthIndex < 12; monthIndex++)
	{
		if (strcmp(Month, monthName[monthIndex]) == 0) break;
	}
	if (monthIndex >= 12) return false;
	tm.Day = Day;
	tm.Month = monthIndex + 1;
	tm.Year = CalendarYrToTm(Year);

	return true;
}

void init_SIM900()
{
	DDRH |= (1<<PH5);	// Reset pin
	DDRH |= (1<<PH6);	// Power pin

	PORTH &= ~(1<<PH5);
	PORTH &= ~(1<<PH6);
}
void init_valves()
{
	DDRC |= (1<<2);	// S11
	DDRC |= (1<<1); // S10
	DDRC |= (1<<0); // S09
	DDRD |= (1<<7); // S08
	DDRG |= (1<<2);	// S07
	DDRG |= (1<<1);	// S06
	DDRG |= (1<<0);	// S05
	DDRL |= (1<<7);	// S04
	DDRL |= (1<<6); // S03
	DDRL |= (1<<5); // S02
	DDRL |= (1<<4); // S01

	DDRB |= (1<<4);	// f01
	DDRB |= (1<<5);	// f02
}
void init_contactors()
{
	// K3 as input
	DDRE &= ~(1<<3);
}
void init_motorDriver()
{
	DDRE |= (1<<4);	// k1
	DDRE |= (1<<5);	// k2
	DDRG |= (1<<5);	// k3
}
void init_Timer1_1Hz()
{
	// Timer 1 with 16 bit time counter. On a Fast PWM
	// TCCR1A <==	COM1A1	COM1A0	COM1B1	COM1B0	COM1C1	COM1C0	WGM11	WGM10
	TCCR1A = 0b00000010;

	// TCCR1B <==	ICNC1	ICES1	�		WGM13	WGM12	CS12	CS11	CS10
	TCCR1B = 0b00011101;	// Start timer at Fcpu/1024

	// TIMSK1 <==	�		�		ICIE1	�		OCIE1C	OCIE1B	OCIE1A	TOIE1
//	TIMSK1 |= (1 << OCIE1A);
	TIMSK1 = 0b00000010;

	ICR1 = 15624;	// To obtain 1Hz clock.
}
void init_ADC()
{
//	ADCSRA ==> ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);	// Set 128 division clock
	ADCSRA |= (1<<ADEN); 							// Enable module

//	ADCSRB ==>	�	ACME	�	�	MUX5	ADTS2	ADTS1	ADTS0
	ADCSRB &= ~(1<<ADTS2);							// Free running mode.
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS0);

	ADCSRB &= ~(1<<MUX5);							// To select ADC0;

//	ADMUX ==> REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
	ADMUX &= ~(1<<REFS1);							// AVCC is the Vref
	ADMUX |=  (1<<REFS0);

	ADMUX &= ~(1<<ADLAR);							// Left Adjustment. To ADCH register.


	ADMUX &= ~(1<<MUX4);							// Select ADC0
	ADMUX &= ~(1<<MUX3);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX0);
}

float calcIrms()
{
	int i;
	uint8_t high, low;

//	int adcSamples[Length];
	int *adcSamples = NULL;
	adcSamples = (int*)malloc(Length * sizeof(int));

//	Serial.print("Declarado: ");
//	Serial.println(freeMemory());

	// Do an ADC conversion
	for(i=0;i<Length;i++)
	{
		ADCSRA |= (1<<ADSC);				// Start conversion;
		while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

		low  = ADCL;
		high = ADCH;

		adcSamples[i] = (high << 8) | low;
	}

	float *vs = NULL;
	vs = (float*)malloc(Length * sizeof(float));

//	for(i=0;i<Length;i++)
//	{
//		Serial.println(adcSamples[i]);
//	}

	for(i=0;i<Length;i++)
	{
		vs[i] = (adcSamples[i]*5.0)/1024.0;
	}

	free(adcSamples);

	// Offset remove.
	float Vmean = 0.0;
	for(i=0;i<Length;i++)
		Vmean += vs[i];

	Vmean = Vmean/Length;

	for(i=0;i<Length;i++)
		vs[i] = vs[i] - Vmean;

	float *vs2 = NULL;
	vs2 = (float*)malloc(Length * sizeof(float));

	// Power signal
	for(i=0;i<Length;i++)
		vs2[i] = vs[i]*vs[i];

	free(vs);

	float sum=0;
	float V2mean;

	// mean finder
	for(i=0;i<Length;i++)
		sum += vs2[i];
	V2mean = sum/Length;

	free(vs2);

	float I = 0.0;
	float k = 2020.0;
	float R = 310.0;

	// RMS equation
	I = (k*sqrt(V2mean))/R;

	return I;
}

void motor_start()
{
	k1_on();
	k3_on();
	_delay_ms(5000);

	k3_off();
	uint32_t count = 0;
	while(k3_read)
	{
		count++;
		if(count>=50000)
		{
			k1_off();
			k2_off();
			k3_off();
			return;

		}
	}
	_delay_ms(25);
	k2_on();

	Serial1.print("Count = ");
	Serial1.println(count);

	motorStatus = 1;
}
void motor_stop()
{
	k1_off();
	k2_off();

	motorStatus = 0;
}
void valveInstr(uint8_t sector, uint8_t status)
{
	switch (sector)
	{
		case 1:
			if(status)
			{
				s01_on();
				valveStatus[0] = 1;
			}
			else
			{
				s01_off();
				valveStatus[0] = 0;
			}
			break;

		case 2:
			if(status)
			{
				s02_on();
				valveStatus[1] = 1;
			}
			else
			{
				s02_off();
				valveStatus[1] = 0;
			}
			break;

		case 3:
			if(status)
			{
				s03_on();
				valveStatus[2] = 1;
			}
			else
			{
				s03_off();
				valveStatus[2] = 0;
			}
			break;

		case 4:
			if(status)
			{
				s04_on();
				valveStatus[3] = 1;
			}
			else
			{
				s04_off();
				valveStatus[3] = 0;
			}
			break;

		case 5:
			if(status)
			{
				s05_on();
				valveStatus[4] = 1;
			}
			else
			{
				s05_off();
				valveStatus[4] = 0;
			}
			break;

		case 6:
			if(status)
			{
				s06_on();
				valveStatus[5] = 1;
			}
			else
			{
				s06_off();
				valveStatus[5] = 0;
			}
			break;

		case 7:
			if(status)
			{
				s07_on();
				valveStatus[6] = 1;
			}
			else
			{
				s07_off();
				valveStatus[6] = 0;
			}
			break;

		case 8:
			if(status)
			{
				s08_on();
				valveStatus[7] = 1;
			}
			else
			{
				s08_off();
				valveStatus[7] = 0;
			}
			break;

		case 9:
			if(status)
			{
				s09_on();
				valveStatus[8] = 1;
			}
			else
			{
				s09_off();
				valveStatus[8] = 0;
			}
			break;

		case 10:
			if(status)
			{
				s10_on();
				valveStatus[9] = 1;
			}
			else
			{
				s10_off();
				valveStatus[9] = 0;
			}
			break;

		case 11:
			if(status)
			{
				s11_on();
				valveStatus[10] = 1;
			}
			else
			{
				s11_off();
				valveStatus[10] = 0;
			}
			break;

		case 12:
			if(status)
			{
				f01_on();
				valveStatus[11] = 1;
			}
			else
			{
				f01_off();
				valveStatus[11] = 0;
			}
			break;

		case 13:
			if(status)
			{
				f02_on();
				valveStatus[12] = 1;
			}
			else
			{
				f02_off();
				valveStatus[12] = 1;
			}
			break;
	}
}
void turnAll_OFF()
{
	motor_stop();
	int i;
	for(i=1;i<14;i++)
	{
		valveInstr(i,0);
	}
}

uint16_t timeSectorMemory(uint8_t sector)
{
	return 60*timeSectorVectorMin[sector-1];
//	return timeSectorVectorMin[sector-1];
}

void SIM900_sendSMS(char *smsbuffer)
{
	Serial2.println("AT+CMGF=1\r");    //Because we want to send the SMS in text mode
	delay(200);
	Serial2.println("AT+CMGS=\"27988081875\"");//send sms message, be careful need to add a country code before the cellphone number
	delay(200);
	Serial2.println(smsbuffer);//the content of the message
//	SerialGSM.print(buffer_to_send);

	delay(1000);
	Serial2.println((char)26);//the ASCII code of the ctrl+z is 26
	delay(2000);
	Serial2.println();
}
void SIM900_power()	// GSM AND GPRS Functions
{
	pinMode(9, OUTPUT);
	digitalWrite(9,LOW);
	delay(1000);
	digitalWrite(9,HIGH);
	delay(2000);
	digitalWrite(9,LOW);
	delay(3000);

	delay(2000);
	SIM900_sendSMS("SIM900 Turned On!");
}
void SIM900_reset()
{
	PORTH |= (1<<PH5);
	_delay_ms(20);
	PORTH &= ~(1<<PH5);
}
int SIM900_checkAlive()
{
	int enableCompare =0, r=0;
	char str[3];

	Serial2.println("at");
	_delay_ms(1200);

	while((Serial2.available()>0))	// Reading from serial
	{
		inChar = Serial2.read();
		sInstrSIM900[k] = inChar;
		k++;

		if(inChar=='K')
		{
			rLength = k;
			k = 0;
			enableCompare = 1;
		}

		if(!Serial2.available())
		{
//			Serial.print(sInstrSIM900);
			k =0;
		}
	}

	if(enableCompare)
	{
		enableCompare = 0;

		char *p;
		p = strchr(sInstrSIM900,'O');

		str[0] = p[0];
		str[1] = p[1];
		str[2] = '\0';
//		Serial1.print("Str: ");
//		Serial1.println(str);

		if(!strcmp(str,"OK"))
		{
//			Serial1.println("ALIVE!");
			r = 1;
		}
	}
	else
	{
		SIM900_power();
		r = 0;
	}

	return r;
}

uint8_t valveTest(uint8_t sector)
{
	float I0a=0.0, I0b=0.0, I0c=0.0, Ia=0.0, Ib=0.0, Ic=0.0;
	int I0m=0, Im=0;
	uint8_t status = 0, tentativas=4;

//	while(!status||!limit)
//	{
	do{
		// Measurement of current without load
		I0a = calcIrms();	// Read currently current;
		_delay_ms(100);
		I0b = calcIrms();	// Read currently current;
		_delay_ms(100);
		I0c = calcIrms();	// Read currently current;
		_delay_ms(100);

		I0m = (int) (1.2*1000.0*(I0a+I0b+I0c)/3.0);

		// Put load
		valveInstr(sector,1);
		_delay_ms(1000);

		// Measurement of current without load
		Ia = calcIrms();	// Read currently current;
		_delay_ms(100);
		Ib = calcIrms();	// Read currently current;
		_delay_ms(100);
		Ic = calcIrms();	// Read currently current;
		_delay_ms(100);

		Im = (int) (1000.0*(Ia+Ib+Ic)/3.0);

		valveInstr(sector,0);

		// Check if it is okay.
		if(Im<=I0m)
			status = 0;
		else
			status = 1;

		tentativas--;

	}while(status&&tentativas);

//	sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sistema Desligado!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year));
//	SIM900_sendSMS(buffer_SIM900);

	return status;
}
void verifyAllValves()
{

}
void verifyValve()
{
	if(motorStatus)
	{
		float Ia=0.0, Ib=0.0, Ic=0.0;
		int Im=0;

		Ia = calcIrms();	// Read currently current;
		_delay_ms(100);
		Ib = calcIrms();	// Read currently current;
		_delay_ms(100);
		Ic = calcIrms();	// Read currently current;
		_delay_ms(100);
		Im = (int) (1000.0*(Ia+Ib+Ic)/3.0);

//		Serial1.print("Im = ");
//		Serial1.println(Im);

		if(Im<85)
		{
			sprintf(buffer,"Sistema desligado durante o setor[%.2d]!",stateSector);
			SIM900_sendSMS(buffer);
			turnAll_OFF();
			stateMode = manual;
		}
	}
}
uint8_t verifyNextValve(uint8_t sector)
{
	uint8_t nextSector = 0;
	float I0a=0.0, I0b=0.0, I0c=0.0, Ia=0.0, Ib=0.0, Ic=0.0;
	int I0m=0, Im=0;
	uint8_t i=0;

	while(Im<=I0m)
	{
//		sprintf(buffer,"-- Sector[%d] --:",sector+i);
//		Serial1.println(buffer);
		I0a = calcIrms();	// Read currently current;
		_delay_ms(100);
		I0b = calcIrms();	// Read currently current;
		_delay_ms(100);
		I0c = calcIrms();	// Read currently current;
		_delay_ms(100);
//		Serial1.print("I0a: ");
//		Serial1.print(I0a);
//		Serial1.print("  I0b: ");
//		Serial1.print(I0b);
//		Serial1.print("  I0c: ");
//		Serial1.println(I0c);

		I0m = (int) (1.05*1000.0*(I0a+I0b+I0c)/3.0);
//		Serial1.print("I0m: ");
//		Serial1.println(I0m);


		valveInstr(sector+i,1);
		_delay_ms(1000);

		Ia = calcIrms();	// Read currently current;
		_delay_ms(100);
		Ib = calcIrms();	// Read currently current;
		_delay_ms(100);
		Ic = calcIrms();	// Read currently current;
		_delay_ms(100);
//		Serial1.print("Ia: ");
//		Serial1.print(Ia);
//		Serial1.print("  Ib: ");
//		Serial1.print(Ib);
//		Serial1.print("  Ic: ");
//		Serial1.println(Ic);

		Im = (int) (1000.0*(Ia+Ib+Ic)/3.0);
//		Serial1.print("Im: ");
//		Serial1.println(Im);

//		sprintf(buffer,"I[%.2d]: ",sector+i);
//		Serial1.print(buffer);
//		Serial1.println(Im);

		if(Im<=I0m)
		{
			valveInstr(sector+i,0);
//			sprintf(buffer,"Sector[%.2d]: Down!",sector+i);
//			Serial1.println(buffer);
			i++;
		}
		else
		{
			nextSector = sector+i;
		}

		if((sector+i) >= 12)
		{
			nextSector = 0;
			Im = 2.0*I0m;

			flag_sector = 1;

			valveInstr(sector-1,0);	// Desliga atual
			valveInstr(sector,0);		// Desliga o que ligou
			turnAll_OFF();

			sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sistema Desligado!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year));
			SIM900_sendSMS(buffer_SIM900);

			flag_timeMatch = 0;

//			if(stateMode == automatic)
			stateMode = manual;


//			sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n System Stoped!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year));
//			SIM900_sendSMS(buffer_SIM900);
		}
	}

	return nextSector;
}

void process_Working()
{
	// 1- Check valve working before start motor
	if(!motorStatus)
	{
		stateSector = verifyNextValve(1);

		if(stateSector>0)
		{
			valveInstr(stateSector,1);
			motor_start();

			_delay_ms(100);
			GLCD.Init();

			flag_sector = 0;
			flag_timeOVF = 0;
		}
		else
		{
			Serial1.println("Out of order!");
			stateMode = manual;
		}

	}

	// 2- Verifica se pode trocar de setor
	if(!flag_sector)
	{
		flag_sector = 1;

		uint8_t i=0;
		for(i=1;i<stateSector;i++)		// Desliga todos setores anteriores.
		{
			valveInstr(i,0);
		}
		valveInstr(stateSector,1);

		if(stateMode == automatic)
			timeSector = timeSectorSet;
		else
			timeSector = timeSectorMemory(stateSector);

		sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
		SIM900_sendSMS(buffer_SIM900);
	}

	// Verifica o proximo setor
	if(flag_timeOVF)
	{
		flag_sector = 0;
		flag_timeOVF = 0;
		timeSector = 10;		// Para nao gerar interrupcao e voltar aqui de novo pulando setor
		stateSector = verifyNextValve(stateSector+1);
	}

	// Check if valve is opened
	verifyValve();
}
void process_Programmed()
{

	if(((tm.Hour == HourOn) && (tm.Minute == MinOn)))
	{
		flag_timeMatch = 1;
	}

	if(((tm.Hour == HourOff) && (tm.Minute == MinOff)))
	{
		flag_timeMatch = 0;
	}


	if(flag_timeMatch)
	{
		process_Working();
	}
	else
	{
		if(motorStatus)
		{
			turnAll_OFF();
			stateSector = 1;
		}

	}

}
void process_valveTest()
{
	if(valveTest(valveOnTest))
	{
		sprintf(buffer,"Sector[%.2d]: Ok!",valveOnTest);
		SIM900_sendSMS(buffer);

		stateMode = manual;
	}
}
void process_Mode()
{
	switch(stateMode)
	{
		case manual:
			// Manual Process
//			process_Manual();
			break;

		case programmed:
			process_Programmed();
			break;

		case automatic:
			// Automatic Process
			process_Working();
			break;

		case valveTesting:
			// Automatic Process
			process_valveTest();
			break;
	}
}

void periodVerify0()
{
	if (((tm.Hour == HourOn) && (tm.Minute >= MinOn)) || (tm.Hour > HourOn)
			|| (tm.Hour < HourOff)
			|| ((tm.Hour == HourOff) && (tm.Minute < MinOff)))
	{
		periodState = greenTime;
		flag04 = 1;
		flag05 = 0;
	}

	if (((tm.Hour == HourOff) && (tm.Minute >= MinOff))
			|| ((tm.Hour > HourOff) && (tm.Hour < HourOn))
			|| ((tm.Hour == HourOn) && (tm.Minute < MinOn)))
	{
		periodState = redTime;
		flag04 = 0;
		flag05 = 1;
	}
}
void refreshVariables()
{
	if(flag_30s)
	{
		SIM900_checkAlive();

		flag_30s = 0;
		count30s = 30;
	}

	if (flag_1s)
	{
		flag_1s = 0;

		RTC.read(tm);
		periodVerify0();
	}
}
void refreshTimeSectors()
{
	int i;
	for(i=0;i<11;i++)
		timeSectorVectorMin[i] = eeprom_read_byte((uint8_t *)(i+1+10));
}

void comm_SIM900()
{
	// Rx - Always listening
	while((Serial2.available()>0))	// Reading from serial
	{
		inChar = Serial2.read();
		sInstrSIM900[k] = inChar;
		k++;

		if(inChar==';')
		{
			rLength = k;
			k = 0;

			enableTranslate_SIM900 = 1;
		}

		if(!Serial2.available())
		{
			Serial.print(sInstrSIM900);
			k =0;
		}
	}


	if(enableTranslate_SIM900)
	{
		enableTranslate_SIM900 = 0;

		char *pi, *pf;
		pi = strchr(sInstrSIM900,'$');
		pf = strchr(sInstrSIM900,';');

		uint8_t l=0;
		l = pf - pi;

		int i;
		for(i=1;i<=l;i++)
		{
			sInstr[i+1] = pi[i];
		}

		enableDecode = 1;
		enableSIM900_Send = 1;
	}
}
void comm_SIM900_SerialPC()
{
	while(Serial2.available() > 0)
		Serial.write(Serial2.read());

//	while(Serial.available() > 0)
//		Serial2.write(Serial.read());
}
void comm_SIM900_Bluetooth()
{
	while(Serial2.available() > 0)
		Serial1.write(Serial2.read());

	while(Serial1.available() > 0)
		Serial2.write(Serial1.read());
}
void comm_Bluetooth_NEW()
{
	while((Serial1.available()>0))	// Reading from serial
	{
		inChar = Serial1.read();
		sInstrBluetooth[k] = inChar;
		k++;

		if(inChar==';')
		{
			rLength = k;
			k = 0;

			enableTranslate_BT = 1;

			Serial1.println("Entrou0!");
		}

		if(!Serial1.available())
		{
			k =0;
		}
	}

	if(enableTranslate_BT)
	{
		enableTranslate_BT = 0;

		char *pi, *pf;
		pi = strchr(sInstrBluetooth,'$');
		pf = strchr(sInstrBluetooth,';');

		sprintf(buffer,"%c",*pi);
		Serial1.println(buffer);

		Serial1.println("Entrou1!");

//		if((*pi=='$')&&(*pf==';'))
//		{
			Serial1.println("Entrou2!");
			uint8_t l=0;
			l = pf - pi + 1;

			int i;
			for(i=1;i<=l;i++)
			{
				sInstr[i+1] = pi[i-1];
			}

			enableDecode = 1;
//		}
	}

}
void comm_Bluetooth()
{
	// Rx - Always listening
	while((Serial1.available()>0))	// Reading from serial
	{
		inChar = Serial1.read();
		sInstr[k] = inChar;
		k++;

		if(inChar==';')
		{
			rLength = k;
			k = 0;

			enableDecode = 1;
		}
	}
}

void summary_Print(uint8_t opt)
{
	switch (opt)
	{
		case 0:
			sprintf(buffer,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d Motor:%d, Mode:%d Uptime: %.2d:%.2d:%.2d, %d d, %d m, %d y, t1:%d, t2:%d, t3:%d, t4:%d, t5:%d, t6:%d, t7:%d, t8:%d, t9:%d, t10:%d, t11:%d",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), motorStatus, stateMode,hour(), minute(), second(), day()-1, month()-1, year()-1970, timeSectorVectorMin[0], timeSectorVectorMin[1], timeSectorVectorMin[2], timeSectorVectorMin[3], timeSectorVectorMin[4], timeSectorVectorMin[5], timeSectorVectorMin[6], timeSectorVectorMin[7], timeSectorVectorMin[8], timeSectorVectorMin[9],timeSectorVectorMin[10]);
			if(enableSIM900_Send)
			{
				enableSIM900_Send = 0;
				SIM900_sendSMS(buffer);
			}
			else
			{
				Serial1.println(buffer);
			}

			break;

		case 1:
			sprintf(buffer,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d, Motor: %d, s1:%d, s2:%d, s3:%d, s4:%d, s5:%d, s6:%d, s7:%d, s8:%d, s9:%d, s10:%d, s11:%d, f01:%d, f02:%d",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), motorStatus, valveStatus[0], valveStatus[1], valveStatus[2], valveStatus[3], valveStatus[4], valveStatus[5], valveStatus[6], valveStatus[7], valveStatus[8], valveStatus[9],valveStatus[10], valveStatus[11], valveStatus[12]);
			if(enableSIM900_Send)
			{
				enableSIM900_Send = 0;
				SIM900_sendSMS(buffer);
			}
			else
			{
				Serial1.println(buffer);
			}

			break;

		case 9:

			sprintf(buffer,"Comando n�o implementado!");
			if(enableSIM900_Send)
			{
				enableSIM900_Send = 0;
				SIM900_sendSMS(buffer);
			}
			else
			{
				Serial1.println(buffer);
			}




//			sprintf(buffer,"t[01]:%d, t[02]:%d, t[03]:%d, t[04]:%d, t[05]:%d, t[06]:%d, t[07]:%d, t[08]:%d, t[09]:%d, t[10]:%d, t[11]:%d",timeSectorVectorMin[0], timeSectorVectorMin[1], timeSectorVectorMin[2], timeSectorVectorMin[3], timeSectorVectorMin[4], timeSectorVectorMin[5], timeSectorVectorMin[6], timeSectorVectorMin[7], timeSectorVectorMin[8], timeSectorVectorMin[9],timeSectorVectorMin[10], timeSectorVectorMin[11]);
//			Serial1.println(buffer);

//			int i;
//			for(i=0;i<11;i++)
//			{
//				sprintf(buffer,"ts[%.d]: %d",i+1, timeSectorVectorMin[i]);
//				Serial1.println(buffer);
//			}
			break;
	}
}

void handleMessage()
{
/*	0;			Verificar detalhes
	1123030;	Ajustar a hora
	201012014;	ajustar a dada
	31;			ligar (31) ou desligar (30) o motor
	4s01:1;		acionamento das valvulas
	5t01:09;	tempo em minutos para cada setor
	60;			Modo de funcionamento
		60; 	- Manual
		61;		- Automatico
		62;		- Executa automatico 1x

*/

	// Tx - Transmitter
	if(enableDecode)
	{
		enableDecode = 0;

		// Getting the opcode
		aux[0] = '0';
		aux[1] = sInstr[2];
		aux[2] = '\0';
		opcode = (uint8_t) atoi(aux);
//		Serial1.println("Got!");

		switch (opcode)
		{
			case 0:		// Check status

				summary_Print(0);

				break;

			case 1:		// Set-up clock

				// Getting the parameters
				aux[0] = sInstr[3];
				aux[1] = sInstr[4];
				aux[2] = '\0';
				tm.Hour = (uint8_t) atoi(aux);

				aux[0] = sInstr[5];
				aux[1] = sInstr[6];
				aux[2] = '\0';
				tm.Minute = (uint8_t) atoi(aux);

				aux[0] = sInstr[7];
				aux[1] = sInstr[8];
				aux[2] = '\0';
				tm.Second = (uint8_t) atoi(aux);

				RTC.write(tm);

				summary_Print(0);

				break;

			case 2:		// Set-up date

				// Getting the parameters
				aux[0] = sInstr[3];
				aux[1] = sInstr[4];
				aux[2] = '\0';
				tm.Day = (uint8_t) atoi(aux);

				aux[0] = sInstr[5];
				aux[1] = sInstr[6];
				aux[2] = '\0';
				tm.Month = (uint8_t) atoi(aux);

				char aux2[5];
				aux2[0] = sInstr[7];
				aux2[1] = sInstr[8];
				aux2[2] = sInstr[9];
				aux2[3] = sInstr[10];
				aux2[4] = '\0';
				tm.Year = (uint8_t) (atoi(aux2)-1970);

				RTC.write(tm);

				summary_Print(0);

				break;

			case 3:		// Set motor ON/OFF

				uint8_t motorCommand;
				aux[0] = '0';
				aux[1] = sInstr[3];
				aux[2] = '\0';
				motorCommand = (uint8_t) atoi(aux);

				if (motorCommand&&(!motorStatus))
					motor_start();
				else
					motor_stop();

				summary_Print(0);

				break;

			case 4:	// ON OFF sectors
				if(sInstr[3] == 's')
				{
					aux[0] = sInstr[4];
					aux[1] = sInstr[5];
					aux[2] = '\0';
					sector = (uint8_t) atoi(aux);


					uint8_t sectorCommand;
					// sInstr[6] == :
					aux[0] = '0';
					aux[1] = sInstr[7];
					aux[2] = '\0';
					sectorCommand = (uint8_t) atoi(aux);

					valveInstr(sector, sectorCommand);
					sprintf(buffer,"Sector%.2d: [%d], Time: %.2d:%.2d:%.2d,",sector, sectorCommand, tm.Hour, tm.Minute, tm.Second);
					Serial1.println(buffer);

//					summary_Print(1);
				}
				break;

			case 5:
//				5t01:23;
				if(sInstr[3] == 't')
				{
					aux[0] = sInstr[4];
					aux[1] = sInstr[5];
					aux[2] = '\0';
					sector = (uint8_t) atoi(aux);


					uint8_t sectorTimeChange;
					aux[0] = sInstr[7];
					aux[1] = sInstr[8];
					aux[2] = '\0';
					sectorTimeChange = (uint8_t) atoi(aux);

					eeprom_write_byte(( uint8_t *)(sector+10), sectorTimeChange);

					refreshTimeSectors();
					summary_Print(0);
//					printTimeSectors();

//					eeprom_update_byte(( uint8_t *)46 , sectorTimeChange);

//					ByteOfData = eeprom_read_byte (( uint8_t *) 46) ;

//					valveInstr(sector, sectorCommand);
//					sprintf(buffer,"Sector%.2d: [%d], Time: %.2d:%.2d:%.2d,",sector, sectorCommand, tm.Hour, tm.Minute, tm.Second);
//					Serial1.println(buffer);
				}

				break;

			case 6:
				uint8_t setCommand;
				aux[0] = '0';
				aux[1] = sInstr[3];
				aux[2] = '\0';
				setCommand = (uint8_t) atoi(aux);

				switch (setCommand)
				{
					case 0:
						stateMode = manual;
						if(motorStatus)
						{
							turnAll_OFF();
							stateSector = 1;
						}
						break;

					case 1:
						stateMode = programmed;
						break;

					case 2:
						stateMode = automatic;
						break;

					case 3:
	//				63:s01;
					if((sInstr[4] == ':')&&(sInstr[5] == 's'))
					{
						aux[0] = sInstr[6];
						aux[1] = sInstr[7];
						aux[2] = '\0';
						valveOnTest = (uint8_t) atoi(aux);

						stateMode = valveTesting;

//						sprintf(buffer,"Testing Sector[%.2d]...",valveOnTest);
//						SIM900_sendSMS(buffer);
					}
					break;

					default:
						Serial1.println("Comando n�o implementado!");
						break;
				}
//				summary_Print(0);

				break;

			case 7:
				summary_Print(1);
			break;

			case 8:
				GLCD.Init();
			break;


			default:
				summary_Print(9);
				break;


		}
	}
}

void summary_GLCD()
{
	if(flag_summaryGLCD)
	{
		flag_summaryGLCD = 0;

		sprintf(buffer_GLCD,"%.2d:%.2d:%.2d, %d/%d/%d",tm.Hour,tm.Minute,tm.Second,tm.Day,tm.Month,tmYearToCalendar(tm.Year));
		GLCD.CursorTo(0,0);
		GLCD.print(buffer_GLCD);

		sprintf(buffer_GLCD,"Mode: %d",stateMode);
		GLCD.CursorTo(0,2);
		GLCD.print(buffer_GLCD);

		sprintf(buffer_GLCD,"Setor%.2d: %.4d",stateSector, timeSector);
		GLCD.CursorTo(0,3);
		GLCD.print(buffer_GLCD);

		int I=(int) (1000.0*calcIrms());
		sprintf(buffer_GLCD,"Is= %d mA",I);
		GLCD.CursorTo(20,4);
		GLCD.print(buffer_GLCD);
	}
}

ISR(TIMER1_COMPA_vect)
{
	if(!count30s)
		flag_30s = 1;
	else
		count30s--;

	if(timeSector == 0)
		flag_timeOVF = 1;
	else
		timeSector--;

	flag_1s =1;
	flag_summaryGLCD = 1;
}


int main()
{
	// Initialize arduino hardware requirements.
	init();
	init_valves();
	init_contactors();
	init_motorDriver();
	init_ADC();
	init_SIM900();
	init_Timer1_1Hz();

	GLCD.Init();
	GLCD.SelectFont(SystemFont5x7);

	Serial.begin(9600);		// Debug
	Serial1.begin(38400);	// Bluetooth
	Serial2.begin(9600);	// Connected to SIM900

	Serial1.println("- Raiden Controller Started! -");
	refreshTimeSectors();

	sInstr[0] = '0';
	sInstr[1] = '0';
	sInstr[2] = '0';
	sInstr[3] = ';';

	while (1)
	{
		// Refrash all variables to compare and take decisions;
		refreshVariables();

		// Main process.
		process_Mode();

		// SIM900 <--> uC
		comm_SIM900();

		// SIM900 <-> Serial PC
//		comm_SIM900_SerialPC();

		// Bluetooth communication
		comm_Bluetooth();

		// Message Manipulation
		handleMessage();

//		GLD Screen Informations
		summary_GLCD();

//		Serial.println(freeMemory());
	}
}



























//				sprintf(buffer," Per.: %d, Motor: %d",periodo, motorStatus);
//				Serial1.println(buffer);

//				sprintf(buffer,"Uptime: %.2d:%.2d:%.2d, %d day(s), %d month(s), %d year(s)", hour(), minute(), second(), day()-1, month()-1, year()-1970);
//				Serial1.println(buffer);
//
//				sprintf(buffer,"Ligou_01___: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",hourLog_ON[0], minuteLog_ON[0], secondLog_ON[0], dayLog_ON[0], monthLog_ON[0], tmYearToCalendar(YearLog_ON[0]), distanceLog_ON[0]);
//				Serial1.println(buffer);
//
//				sprintf(buffer,"Ligou_02___: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",hourLog_ON[1], minuteLog_ON[1], secondLog_ON[1], dayLog_ON[1], monthLog_ON[1], tmYearToCalendar(YearLog_ON[1]), distanceLog_ON[1]);
//				Serial1.println(buffer);
//
//				sprintf(buffer,"Desligou_01: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",hourLog_OFF[0], minuteLog_OFF[0], secondLog_OFF[0], dayLog_OFF[0], monthLog_OFF[0], tmYearToCalendar(YearLog_OFF[0]), distanceLog_OFF[0]);
//				Serial1.println(buffer);
//
//				sprintf(buffer,"Desligou_02: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",hourLog_OFF[1], minuteLog_OFF[1], secondLog_OFF[1], dayLog_OFF[1], monthLog_OFF[1], tmYearToCalendar(YearLog_OFF[1]), distanceLog_OFF[1]);
//				Serial1.println(buffer);





// state machine
//	switch(stateSector)
//	{
//		case 1:
//			if(!flag_sector)
//			{
//				flag_sector = 1;
//				valveInstr(1,1);
//
//				if(stateMode == automatic)
//					timeSector = timeSectorSet;
//				else
//					timeSector = timeSectorMemory(stateSector);
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = verifyNextValve(stateSector+1);
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 2:
//			if(!flag_sector)
//			{
//				flag_sector = 1;
//
//				int i;
//				for(i=1;i<stateSector;i++)
//				{
//					valveInstr(i,0);
//				}
//				valveInstr(stateSector,1);
//
//				if(stateMode == automatic)
//					timeSector = timeSectorSet;
//				else
//					timeSector = timeSectorMemory(stateSector);
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = verifyNextValve(stateSector+1);
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 3:
//			if(!flag_sector)
//			{
//				flag_sector = 1;
//
//				int i;
//				for(i=1;i<stateSector;i++)
//				{
//					valveInstr(i,0);
//				}
//				valveInstr(stateSector,1);
//
//				if(stateMode == automatic)
//					timeSector = timeSectorSet;
//				else
//					timeSector = timeSectorMemory(stateSector);
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = verifyNextValve(stateSector+1);
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 4:
//			if(!flag_sector)
//			{
//				flag_sector = 1;
//
//				int i;
//				for(i=1;i<stateSector;i++)
//				{
//					valveInstr(i,0);
//				}
//				valveInstr(stateSector,1);
//
//				if(stateMode == automatic)
//					timeSector = timeSectorSet;
//				else
//					timeSector = timeSectorMemory(stateSector);
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = verifyNextValve(stateSector+1);
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 5:
//			if(!flag_sector)
//			{
//				flag_sector = 1;
//
//				int i;
//				for(i=1;i<stateSector;i++)
//				{
//					valveInstr(i,0);
//				}
//				valveInstr(stateSector,1);
//
//				if(stateMode == automatic)
//					timeSector = timeSectorSet;
//				else
//					timeSector = timeSectorMemory(stateSector);
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = verifyNextValve(stateSector+1);
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 6:
//			if(!flag_sector)
//			{
//				flag_sector = 1;
//
//				int i;
//				for(i=1;i<stateSector;i++)
//				{
//					valveInstr(i,0);
//				}
//				valveInstr(stateSector,1);
//
//				if(stateMode == automatic)
//					timeSector = timeSectorSet;
//				else
//					timeSector = timeSectorMemory(stateSector);
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = verifyNextValve(stateSector+1);
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 7:
//			if(!flag_sector)
//			{
//				flag_sector = 1;
//
//				int i;
//				for(i=1;i<stateSector;i++)
//				{
//					valveInstr(i,0);
//				}
//				valveInstr(stateSector,1);
//
//				if(stateMode == automatic)
//					timeSector = timeSectorSet;
//				else
//					timeSector = timeSectorMemory(stateSector);
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = verifyNextValve(stateSector+1);
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 8:
//			if(!flag_sector)
//			{
//				flag_sector = 1;
//
//				int i;
//				for(i=1;i<stateSector;i++)
//				{
//					valveInstr(i,0);
//				}
//				valveInstr(stateSector,1);
//
//				if(stateMode == automatic)
//					timeSector = timeSectorSet;
//				else
//					timeSector = timeSectorMemory(stateSector);
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = verifyNextValve(stateSector+1);
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 9:
//			if(!flag_sector)
//			{
//				flag_sector = 1;
//
//				int i;
//				for(i=1;i<stateSector;i++)
//				{
//					valveInstr(i,0);
//				}
//				valveInstr(stateSector,1);
//
//				if(stateMode == automatic)
//					timeSector = timeSectorSet;
//				else
//					timeSector = timeSectorMemory(stateSector);
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = verifyNextValve(stateSector+1);
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 10:
//			if(!flag_sector)
//			{
//				flag_sector = 1;
//
//				int i;
//				for(i=1;i<stateSector;i++)
//				{
//					valveInstr(i,0);
//				}
//				valveInstr(stateSector,1);
//
//				if(stateMode == automatic)
//					timeSector = timeSectorSet;
//				else
//					timeSector = timeSectorMemory(stateSector);
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = verifyNextValve(stateSector+1);
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 11:
//			if(!flag_sector)
//			{
//				flag_sector = 1;
//
//				int i;
//				for(i=1;i<stateSector;i++)
//				{
//					valveInstr(i,0);
//				}
//				valveInstr(stateSector,1);
//
//				if(stateMode == automatic)
//					timeSector = timeSectorSet;
//				else
//					timeSector = timeSectorMemory(stateSector);
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = verifyNextValve(stateSector+1);
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//	}




//void process_Automatic()
//{
//	if(!motorStatus)
//	{
//		valveInstr(1,1);
//		motor_start();
//
//		_delay_ms(100);
//		GLCD.Init();
//
//		flag_sector = 0;
//		flag_timeOVF = 0;
////		stateSector = verifyNextValve(1);
//	}
//
////	if(!flag_sector)
////	{
////		sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
////		SIM900_sendSMS(buffer_SIM900);
////	}
//
//	switch(stateSector)
//	{
//		case 1:
//			if(!flag_sector)
//			{
//				valveInstr(1,1);
//				timeSector = timeSectorSet;
//				flag_sector = 1;
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = 2;
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 2:
//			if(!flag_sector)
//			{
//				valveInstr(2,1);
//				valveInstr(1,0);
//				timeSector = timeSectorSet;
//				flag_sector = 1;
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = 3;
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 3:
//			if(!flag_sector)
//			{
//				valveInstr(3,1);
//				valveInstr(2,0);
//				timeSector = timeSectorSet;
//				flag_sector = 1;
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = 4;
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 4:
//			if(!flag_sector)
//			{
//				valveInstr(4,1);
//				valveInstr(3,0);
//				timeSector = timeSectorSet;
//				flag_sector = 1;
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = 5;
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 5:
//			if(!flag_sector)
//			{
//				valveInstr(5,1);
//				valveInstr(4,0);
//				timeSector = timeSectorSet;
//				flag_sector = 1;
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = 6;
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 6:
//			if(!flag_sector)
//			{
//				valveInstr(6,1);
//				valveInstr(5,0);
//				timeSector = timeSectorSet;
//				flag_sector = 1;
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = 7;
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 7:
//			if(!flag_sector)
//			{
//				valveInstr(7,1);
//				valveInstr(6,0);
//				timeSector = timeSectorSet;
//				flag_sector = 1;
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = 8;
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 8:
//			if(!flag_sector)
//			{
//				valveInstr(8,1);
//				valveInstr(7,0);
//				timeSector = timeSectorSet;
//				flag_sector = 1;
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = 9;
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 9:
//			if(!flag_sector)
//			{
//				valveInstr(9,1);
//				valveInstr(8,0);
//				timeSector = timeSectorSet;
//				flag_sector = 1;
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = 10;
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 10:
//			if(!flag_sector)
//			{
//				valveInstr(10,1);
//				valveInstr(9,0);
//				timeSector = timeSectorSet;
//				flag_sector = 1;
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = 11;
//				flag_sector = 0;
//				flag_timeOVF = 0;
//			}
//			break;
//
//		case 11:
//			if(!flag_sector)
//			{
//				valveInstr(11,1);
//				valveInstr(10,0);
//				timeSector = timeSectorSet;
//				flag_sector = 1;
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			if(flag_timeOVF)
//			{
//				stateSector = 1;
//				flag_sector = 0;
//				flag_timeOVF = 0;
//
//				valveInstr(11,0);
//
//				motor_stop();
//				stateMode = manual;
//
//
//				sprintf(buffer_SIM900,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n System Stoped!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year));
//				SIM900_sendSMS(buffer_SIM900);
//			}
//			break;
//	}
//}



//void comm_Bluetooth()
//{
//	 char data [21];
//	 int number_of_bytes_received;
//
//	 if(Serial1.available() > 0)
//	 {
//	   number_of_bytes_received = Serial1.readBytesUntil (13,data,20); // read bytes (max. 20) from buffer, untill <CR> (13). store bytes in data. count the bytes recieved.
//	   data[number_of_bytes_received] = 0; // add a 0 terminator to the char array
//
//		 bool result = strcmp (data, "whatever");
//
//		 if (result == false)
//		 {
//		   Serial1.println("data matches whatever");
//		 }
//		 else
//		 {
//		   Serial1.println("data does not match whatever");
//		 }
//		 Serial1.flush();
//	 }
//}

//int main() {
//
//	// Initialize arduino hardware requirements.
//	init();
//	init_valves();
//	init_contactors();
//	init_motorDriver();
//	init_ADC();
//	init_SIM900();
//	init_Timer1_1Hz();
//
//	Serial1.begin(38400);	// Bluetooth
//	Serial.begin(9600);		// Debug
//	Serial2.begin(9600);	// Connected to SIM900
//
//	GLCD.Init();
//	GLCD.SelectFont(SystemFont5x7);
//
//	// Welcome!
//	GLCD.CursorTo(0,7);
//	GLCD.print("Motor Testing Mode! v0.0");
//	Serial1.println("- Raiden Power -");
//
////	uint8_t sector;
////	char c = '0';
////	uint8_t k=0;
////	char buffer[5];
//	SIM900_checkAlive();
////	SIM900_sendSMS("Turning ON!");
//
//	while(1)
//	{
////		Refresh all variables - Electrical par, Time period;
//		refreshVariables();
//
////		Communication between SIM900 and serial port computer
//		comm_SIM900();
//
////		Communication between Raiden and Bluetooth terminal cellphone
//		comm_Bluetooth2();
//
////		Command Decisions
////		control_decision();
//
//		comm_SIM900_Bluetooth();
//
////		Do process
////		process_main();
//
////		GLD Screen Informations
//		summary_GLCD();
//	}
//}