
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#define LCD_RS 0 // LCD RS
#define LCD_RW 1 // LCD RW
#define LCD_EN 2 // LCD EN

// chon PC3 la ngo ra motor
#define output1 PORTC
#define output2 PORTB
const uint8_t p_out = 1;
const uint8_t p_err = 2; // pc2 is error's led

unsigned int moist_value;	  // moisture value
unsigned char sec, min, hour; // time value

volatile unsigned char mode = 0;																// 0: display, 1: moisture, 2: timer
unsigned char selected_value = 0;																// 0: high/low for moisture
unsigned char selected_value2 = 0;																// hour/minute/second for timer
unsigned char high_moisture_L = 0, high_moisture_H = 0, low_moisture_L = 0, low_moisture_H = 0; // do am cai dat
unsigned char set_hour_L = 0, set_hour_H = 0, set_minute_L = 0, set_minute_H = 0;				// thoi gian hen

void delay_us(unsigned char num); // delay 1us X (num + 1) using int clk
void delay_ms(unsigned char num); // delay 128us X (num + 1) using int clk
void adc_init(void);
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(unsigned char data);
unsigned char i2c_read(void);
void i2c_nak(void);
void lcdWrite(unsigned char data);
void lcd_reset(void);
void lcd_init(void);
void lcdCommand(unsigned char cmd);
void lcdData(unsigned char data);
void rtc_read(void);
void motor(void);
unsigned char ascii_0(unsigned char val);
unsigned char ascii_1(unsigned char val);

void EEPROM_writeByte(uint16_t addr, uint8_t data);

uint8_t EEPROM_readByte(uint16_t addr);

// bien tam thoi
uint32_t T_P;
uint32_t T_RTC;
uint32_t RH_P_LL;
uint32_t RH_P_HL;
uint32_t temp;
uint8_t count_temp = 0;
unsigned int temp_moist;
unsigned int count_moist;

const unsigned int addr_time_eep = 0x01FF;
const unsigned int addr_mois_eep = 0x02FF;
ISR(ADC_vect)
{
	unsigned int adc_low = ADCL;
	unsigned int adc_high = ADCH;
	unsigned int adc_value = adc_low + adc_high * 256;
	if (adc_value >= 550)
		moist_value = 0;
	else if (adc_value <= 370)
		moist_value = 99;
	else
		moist_value = 305 - (100 * adc_value) / 180; // moisture value
	ADCSRA |= (1 << ADSC);							 // start conversion
}

ISR(TIMER1_COMPA_vect)
{
	if (moist_value <= (temp + 1))
		count_temp++;
	if (count_temp == 5) // co the chinh lai nho hon de de test
	{
		output1 |= (1 << p_err);  // enable error's led
		output2 &= ~(1 << p_out); // disable motor
		TCCR1B &= ~7;			  // stop Timer1
	}
	if (moist_value > RH_P_HL)
	{
		output2 &= ~(1 << p_out); // disable motor
		TCCR1B &= ~7;			  // stop Timer1
	}
}

ISR(INT1_vect)
{
	if ((output1 & (1 << p_err)) == 0)
		motor();			  // don't turn on motor if error's led on
	output1 &= ~(1 << p_err); // disable error's led
}
ISR(INT0_vect)
{
	mode++;
	if (mode == 3)
		mode = 0;
}

int main(void)
{
	sei();
	adc_init();
	DDRB = 0x0F;
	DDRB |= (1 << p_out);
	DDRC |= (1 << p_err); // Set outputs
	PORTC = 0;
	DDRD = 0;													 // Set buttons as inputs
	PORTD = ((1 << PD2) | (1 << PD6) | (1 << PD3) | (1 << PD5)); // Set pull-ups
	EICRA = (1 << ISC11) | (1 << ISC01);						 // falling edge interrupt
	EIMSK = (1 << INT1) | (1 << INT0);							 // external interrupt enable
	OCR1A = 16000;												 // 128us x (number + 1), thoi gian kiem tra la khoang 2s, co the chinh lai nho hon de de test
	TCCR1B = (1 << WGM12);										 // mode CTC
	TIMSK1 = (1 << OCIE1A);										 // Timer1 COMPA	interrupt enable
	// uint8_t lastButton1State = 0;
	uint8_t lastButton2State = 0;
	uint8_t lastButton3State = 0;
	i2c_init();
	lcd_reset();
	lcd_init();
	mode = 0;
	temp_moist = moist_value;

	// read value stored in eeprom
	high_moisture_H = EEPROM_readByte(addr_mois_eep);
	high_moisture_L = EEPROM_readByte(addr_mois_eep + 1);
	low_moisture_H = EEPROM_readByte(addr_mois_eep + 2);
	low_moisture_L = EEPROM_readByte(addr_mois_eep + 3);
	set_hour_H = EEPROM_readByte(addr_time_eep);
	set_hour_L = EEPROM_readByte(addr_time_eep + 1);
	set_minute_H = EEPROM_readByte(addr_time_eep + 2);
	set_minute_L = EEPROM_readByte(addr_time_eep + 3);
	/*	i2c_start();
		i2c_write(0b11010000);
		i2c_write(0x00);
		i2c_write(0x00);
		i2c_write(0x00);
		i2c_write(0x00);
		i2c_nak();
		i2c_stop();*/
	while (1)
	{
		if (moist_value != temp_moist)
		{
			count_moist++;
			if (count_moist == 5)
			{
				temp_moist = moist_value;
				count_moist = 0;
			}
		}
		// uint8_t currentButton1State = PIND & (1 << PD2);
		uint8_t currentButton2State = PIND & (1 << PD5);
		uint8_t currentButton3State = PIND & (1 << PD6);

		if (mode == 1)
		{
			if (currentButton2State && !lastButton2State)
			{
				if (PIND & (1 << PD5))
				{ // Second button pressed
					delay_ms(255);
					delay_ms(255);
					if (selected_value == 0)
					{
						low_moisture_L = (low_moisture_L + 1) % 10;
						EEPROM_writeByte(addr_mois_eep + 3, low_moisture_L);
					}
					if (selected_value == 1)
					{
						low_moisture_H = (low_moisture_H + 1) % 10;
						EEPROM_writeByte(addr_mois_eep + 2, low_moisture_H);
					}
					if (selected_value == 2)
					{
						high_moisture_L = (high_moisture_L + 1) % 10;
						EEPROM_writeByte(addr_mois_eep + 1, high_moisture_L);
					}
					if (selected_value == 3)
					{
						high_moisture_H = (high_moisture_H + 1) % 10;
						EEPROM_writeByte(addr_mois_eep, high_moisture_H);
					}
				}
			}
		}
		if (mode == 1)
		{
			if (currentButton3State && !lastButton3State)
			{
				if (PIND & (1 << PD6))
				{ // Second button pressed
					delay_ms(100);
					// delay_ms(255);
					selected_value++;
					if (selected_value == 4)
						selected_value = 0;
				}
			}
		}

		if (mode == 2)
		{
			if (currentButton2State && !lastButton2State)
			{
				if (PIND & (1 << PD5))
				{ // Second button pressed
					delay_ms(100);
					// delay_ms(255);
					if (selected_value2 == 0)
					{
						set_minute_L = (set_minute_L + 1) % 10;
						EEPROM_writeByte(addr_time_eep + 3, set_minute_L);
					}
					if (selected_value2 == 1)
					{
						set_minute_H++;
						if (set_minute_H == 6)
							set_minute_H = 0;
						EEPROM_writeByte(addr_time_eep + 2, set_minute_H);
					}
					if (selected_value2 == 2)
					{
						set_hour_L = (set_hour_L + 1) % 10;
						EEPROM_writeByte(addr_time_eep + 1, set_hour_L);
					}
					if (selected_value2 == 3)
					{
						set_hour_H++;
						if (set_hour_H == 3)
							set_hour_H = 0;
						EEPROM_writeByte(addr_time_eep, set_hour_H);
					}
				}
			}
		}

		if (mode == 2)
		{
			if (currentButton3State && !lastButton3State)
			{
				if (PIND & (1 << PD6))
				{ // Second button pressed
					delay_ms(100);
					// delay_ms(255);
					selected_value2++;
					if (selected_value2 == 4)
						selected_value2 = 0;
				}
			}
		}

		switch (mode)
		{
		case 0:
		{
			rtc_read();
			lcdCommand(0x0C);
			lcdCommand(0x80);
			char dis1[] = "Moisture: ";
			char dis2[] = "Time: ";

			for (int i = 0; i < strlen(dis1); i++)
			{
				lcdData(dis1[i]);
			}
			lcdData(temp_moist / 10 + 48);
			lcdData(temp_moist % 10 + 48);
			lcdData('%');
			lcdCommand(0xC0);
			for (int i = 0; i < strlen(dis2); i++)
			{
				lcdData(dis2[i]);
			}
			lcdData(hour / 10 + 48);
			lcdData(hour % 10 + 48);
			lcdData(':');
			lcdData(min / 10 + 48);
			lcdData(min % 10 + 48);
			lcdData(':');
			lcdData(sec / 10 + 48);
			lcdData(sec % 10 + 48);
			lcdData(' ');
			break;
		}
		case 1:
		{
			lcdCommand(0x0C);
			lcdCommand(0x80);
			char dis[] = "H:";
			for (int i = 0; i < strlen(dis); i++)
			{
				lcdData(dis[i]);
			}
			lcdData(high_moisture_H + '0');
			lcdData(high_moisture_L + '0');
			lcdData('%');
			strcpy(dis, "     L:");
			for (int i = 0; i < strlen(dis); i++)
			{
				lcdData(dis[i]);
			}
			lcdData(low_moisture_H + '0');
			lcdData(low_moisture_L + '0');
			lcdData('%');
			lcdCommand(0xC0);
			strcpy(dis, "1/Inc  2/Switch ");
			for (int i = 0; i < strlen(dis); i++)
			{
				lcdData(dis[i]);
			}
			lcdCommand(0x0E);
			if (selected_value == 0)
				lcdCommand(0x8D);
			if (selected_value == 1)
				lcdCommand(0x8C);
			if (selected_value == 2)
				lcdCommand(0x83);
			if (selected_value == 3)
				lcdCommand(0x82);
			delay_ms(200);
			// delay_ms(255);
			// delay_ms(255);
			// delay_ms(255);
			break;
		}
		case 2:
		{
			lcdCommand(0x0C);
			lcdCommand(0x80);
			char dis[2] = "";
			for (int i = 0; i < strlen(dis); i++)
			{
				lcdData(dis[i]);
			}
			lcdData(set_hour_H + '0');
			lcdData(set_hour_L + '0');
			strcpy(dis, ":");
			for (int i = 0; i < strlen(dis); i++)
			{
				lcdData(dis[i]);
			}
			lcdData(set_minute_H + '0');
			lcdData(set_minute_L + '0');
			strcpy(dis, ":");
			for (int i = 0; i < strlen(dis); i++)
			{
				lcdData(dis[i]);
			}
			lcdData('0');
			lcdData('0');
			strcpy(dis, "         ");
			for (int i = 0; i < strlen(dis); i++)
			{
				lcdData(dis[i]);
			}
			lcdCommand(0xC0);
			strcpy(dis, "1/Inc  2/Switch ");
			for (int i = 0; i < strlen(dis); i++)
			{
				lcdData(dis[i]);
			}
			lcdCommand(0x0E);
			if (selected_value2 == 0)
				lcdCommand(0x84);
			if (selected_value2 == 1)
				lcdCommand(0x83);
			if (selected_value2 == 2)
				lcdCommand(0x81);
			if (selected_value2 == 3)
				lcdCommand(0x80);
			delay_ms(200);
			// delay_ms(255);
			// delay_ms(255);
			// delay_ms(255);
			break;
		}
		}

		// lastButton1State = currentButton1State;
		lastButton2State = currentButton2State;
		lastButton3State = currentButton3State;
		T_RTC = hour * 60 + min;
		T_P = (set_hour_H * 10 + set_hour_L) * 60 + set_minute_H * 10 + set_minute_L;
		// addr = 0x01ff;

		RH_P_LL = low_moisture_H * 10 + low_moisture_L;
		RH_P_HL = high_moisture_H * 10 + high_moisture_L;
		if ((T_RTC == T_P) && ((output1 & (1 << p_err)) == 0))
			motor();
		// PORTB = mode;
		delay_ms(200);
		// delay_ms(255);
		// delay_ms(255);
		// delay_ms(255);
	}
}

void delay_us(unsigned char num) // delay 1us X (num + 1) using int clk
{
	OCR0A = num;
	TCCR0A = (1 << WGM01); // Timer 0, CTC mode, int clk, prescaler 8
	TCCR0B = (1 << CS01);  // start Timer
	while ((TIFR0 & (1 << OCF0A)) == 0)
		;				  // wait until OCF0A is set
	TCCR0B = 0;			  // stop Timer
	TIFR0 = (1 << OCF0A); // erase OCF0A
}

void delay_ms(unsigned char num) // delay 128us X (num + 1) using int clk
{
	OCR0A = num;
	TCCR0A = (1 << WGM01);				// Timer 0, CTC mode, int clk, prescaler 1024
	TCCR0B = (1 << CS02) | (1 << CS00); // start Timer
	while ((TIFR0 & (1 << OCF0A)) == 0)
		;				  // wait until OCF0A is set
	TCCR0B = 0;			  // stop Timer
	TIFR0 = (1 << OCF0A); // erase OCF0A
}

void adc_init(void)
{
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //
	ADMUX = (1 << REFS0);
	ADCSRA |= (1 << ADSC); // start conversion
}

void i2c_init(void)
{
	TWBR = 0x08; // set SCL f = 100KHz
	TWSR = 1;
	TWCR = (1 << TWEN); // module TWI enabled
}

void i2c_start(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0)
		;
}

void i2c_stop(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void i2c_write(unsigned char data)
{
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0)
		;
}

unsigned char i2c_read(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while ((TWCR & (1 << TWINT)) == 0)
		;
	return TWDR;
}

void i2c_nak(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0)
		;
}

void lcdWrite(unsigned char data)
{
	i2c_start();
	i2c_write(0b01001110);
	i2c_write(data);
	i2c_stop();
}

void lcdCommand(unsigned char cmd)
{
	i2c_start();
	i2c_write(0b01001110);
	i2c_write((0 << LCD_RS) | (0 << LCD_RW) | (1 << LCD_EN) | (cmd & 0xF0) | 0x08);
	i2c_write((0 << LCD_RS) | (0 << LCD_RW) | (0 << LCD_EN) | (cmd & 0xF0) | 0x08);
	delay_us(100); // delay 100us
	i2c_write((0 << LCD_RS) | (0 << LCD_RW) | (1 << LCD_EN) | (cmd << 4) | 0x08);
	i2c_write((0 << LCD_RS) | (0 << LCD_RW) | (0 << LCD_EN) | (cmd << 4) | 0x08);
	delay_us(100);
	i2c_stop();
}

void lcdData(unsigned char data)
{
	// PORTB |= (1 << PB5);
	i2c_start();
	// PORTB |= (1 << PB5);
	i2c_write(0b01001110);
	i2c_write((1 << LCD_RS) | (0 << LCD_RW) | (1 << LCD_EN) | (data & 0xF0) | 0x08);
	i2c_write((1 << LCD_RS) | (0 << LCD_RW) | (0 << LCD_EN) | (data & 0xF0) | 0x08);
	delay_us(100); // delay 100us
	i2c_write((1 << LCD_RS) | (0 << LCD_RW) | (1 << LCD_EN) | (data << 4) | 0x08);
	i2c_write((1 << LCD_RS) | (0 << LCD_RW) | (0 << LCD_EN) | (data << 4) | 0x08);
	delay_us(100);
	i2c_stop();
}

void lcd_reset(void) // 4 bits LCD reset
{
	delay_ms(196); // delay 25ms
	delay_ms(196); // delay 25ms
	i2c_start();
	i2c_write(0b01001110);
	i2c_write(0b00110100);
	i2c_write(0b00110000);
	delay_ms(32); // delay 4.2ms
	i2c_write(0b00110100);
	i2c_write(0b00110000);
	delay_us(200); // delay 200us
	i2c_write(0b00110100);
	i2c_write(0b00110000);
	delay_us(200); // delay 200us
	i2c_write(0b00100100);
	delay_us(1);
	i2c_write(0b00100000);
	i2c_stop();
}

void lcd_init(void)
{
	lcdCommand(0x28); // 4 bits operation, 2 line
	lcdCommand(0x01); // clear display
	delay_ms(16);	  // delay 2ms
	lcdCommand(0x0C); // display on, no cursor
	lcdCommand(0x06); // cursor shift right
}

void rtc_read(void)
{
	unsigned char val;
	i2c_start();
	i2c_write(0b11010000);
	i2c_write(0x00);
	i2c_start();
	i2c_write(0b11010001);
	val = i2c_read();
	sec = (val & 0x0F) + ((val >> 4) * 10);
	val = i2c_read();
	min = (val & 0x0F) + ((val >> 4) * 10);
	val = i2c_read() & 0b00111111;
	hour = (val & 0x0F) + ((val >> 4) * 10);
	i2c_nak();
	i2c_stop();
}
void motor(void)
{
	if ((TCCR1B & 7) == 0)
	{
		temp = moist_value;
		count_temp = 0;
	}
	if (moist_value < RH_P_LL)
	{
		output2 |= (1 << p_out);
		TCCR1B |= (1 << CS12) | (1 << CS10); // he so chia clk la 1024, start Timer1
	}
}

void EEPROM_writeByte(uint16_t addr, uint8_t data)
{
	// Wait for completion of previous write
	while (EECR & (1 << EEPE))
		;

	// Set up address and data registers
	EEAR = addr;
	EEDR = data;

	// Write logical one to EEMPE
	EECR |= (1 << EEMPE);

	// Start EEPROM write by setting EEPE
	EECR |= (1 << EEPE);
}

uint8_t EEPROM_readByte(uint16_t addr)
{
	// Wait for completion of previous write
	while (EECR & (1 << EEPE))
		;

	// Set up address register
	EEAR = addr;

	// Start EEPROM read by setting EERE
	EECR |= (1 << EERE);

	// Return data from data register
	return EEDR;
}