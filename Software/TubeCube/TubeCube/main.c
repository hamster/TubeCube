#include <atmel_start.h>
#include <util/delay.h>
#include <string.h>
#include "characters.h"

#define BRIGHTNESS_NORMAL (22*3)
#define BRIGHTNESS_LOW	  (3)

#define NUM_BANNERS 11
#define INTERBANNER_CHARS 10
#define INTERCHAR_TIME	250

char *banners[NUM_BANNERS] = {
	" @hamster ",
	" SAINTCON 2019 ",
	" RUN DRC ",
	" Now with 50% more magic smoke ",
	" BLINKY ",
	" DCZia ",
	" dc801 ",
	" MARV WAS NOT HERE ",
	" THIS SPACE FOR RENT ",
	" git push --force ",
	" HACKING TIME ...      ERROR! HACKING TOO MUCH TIME! "
};

/************************************************************************/
/* Write a char out to the tube                                         */
/************************************************************************/
void SetChar(unsigned int character){
	
	// Not an ASCII printable char, ignore it
	if(character > 127 || character < 32){
		return;
	}
	
	// Can't really do new lines or returns so just make it a blanking char
	if(character == '\n'){
		character = ' ';
	}
	if(character == '\r'){
		character = ' ';
	}
		
	if(character == ' '){
		PWM_0_load_duty_cycle_ch2(BRIGHTNESS_LOW);
	}
	else{
		PWM_0_load_duty_cycle_ch2(BRIGHTNESS_NORMAL);
	}
	
	// Strobe is chip select for the HV5812
	Strobe_set_level(true);
		
	char data[3] = { 0, 0, 0 };
		
	// 0x20 is to turn on the grid
	data[2] = TubeChar[character - ' '][2] | 0x20;	
	data[1] = TubeChar[character - ' '][1];
	data[0] = TubeChar[character - ' '][0];

	SPI_0_write_block(data, 3);

	Strobe_set_level(false);
	
}

/************************************************************************/
/* Spin the block around the display.  I suppose I could abstract the   */
/* actual write...                                                      */
/************************************************************************/
void SetSpin(unsigned int pos){
	
	Strobe_set_level(true);
	
	char data[3];
	data[2] = TubeSpin[pos][2] | 0x20;
	data[1] = TubeSpin[pos][1];
	data[0] = TubeSpin[pos][0];
	SPI_0_write_block(data, 3);
	
	Strobe_set_level(false);
	
}


int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	//// Enable hardware
	PWM_0_enable();
	USART_0_enable();
	SPI_0_enable();
		
	// Setup the HVPS
	// This will switch at about 10khz
	PWM_0_load_counter(0);
	PWM_0_load_top(300);
	PWM_0_enable_output_ch2();
	
	PWM_0_load_duty_cycle_ch2(BRIGHTNESS_NORMAL);
	
	// Delay a tick to let the HVPS come up to power
	_delay_ms(50);
	
	// Make sure the display is clear
	SetChar(' ');
	
	// Display my handle, woo woo
	const char handle[] = " hamster ";
	for(int i = 0; i < sizeof(handle) - 1; i++){
		SetChar(handle[i]);
		_delay_ms(150);
	}
	
	// Spin the display and detect if we are the banner generator
	
	bool isGenerator = true;
	
	PWM_0_load_duty_cycle_ch2(BRIGHTNESS_NORMAL);
	for(int j = 0; j < 4; j++){
		for(int i = 0; i < 8; i++){
			USART_0_write(23);
			if(USART_0_is_rx_ready()){
				if(USART_0_get_data() == 23){
					isGenerator = false;
				}
			}
			SetSpin(i);
			_delay_ms(40);
		}
	}
	
	// Give an indication if we detected that we are the generator
	for(int i = 0; i < 6; i++){
		if((i % 2) == 0){
			SetChar('.');
		}
		else{
			if(isGenerator){
				SetChar('{');
			}
			else{
				SetChar('-');
			}
		}
		_delay_ms(50);
	}

	// Dim the display
	PWM_0_load_duty_cycle_ch2(BRIGHTNESS_LOW);
	
	unsigned char retChar;
	unsigned char oldChar = ' ';
	uint8_t pos = 0;
	char *msg = banners[0];
	
	while (1) {
		
		if(isGenerator){
			// Strobe the message out
			
			// Write the current position to the tube, and then 
			// write the last char out the serial port
			SetChar(msg[pos]);
			if(pos > 0){
				USART_0_write(msg[pos - 1]);
			}
			else{
				USART_0_write(' ');
			}
					
			pos++;
			if(pos >= strlen(msg)){
				
				// Blank the tubes for a bit
				SetChar(' ');
				for(int i = 0; i < INTERBANNER_CHARS; i++){
					_delay_ms(INTERCHAR_TIME);
					USART_0_write(' ');
				}
				
				// Grab a different banner
				pos = 0;
				int randBanner = rand() % (NUM_BANNERS);
				msg = banners[randBanner];
			}
			
			// If we start getting data on the serial port, stop generating banner
			if(USART_0_is_rx_ready()){
				if(USART_0_get_data() == ' '){
					isGenerator = false;
				}
			}
			_delay_ms(INTERCHAR_TIME);
		}
		else{
			// We're downstream of the generator
			// Wait for a char to be received
			// Once we get something, clock what were displaying out to the next unit
			if(USART_0_is_rx_ready()){
				USART_0_write(oldChar);
				retChar = USART_0_get_data();
				SetChar(retChar);
				oldChar = retChar;
			}
			_delay_ms(25);
		}
		
	}
	
}
