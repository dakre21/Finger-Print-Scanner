/*
 * FinalProject.c
 *
 * Created: 4/7/2015 2:16:46 PM
 *  Author: David
 */ 
#define Get_Pin			0
#define Check_Pin		1
#define Authorize		2
#define Scan_Finger		3
#define Validate_User	4
#define Enroll_User		5
#define user_length		28
#define id_length		10
#define pin_length		4
#define F_CPU 8000000UL // Defines 8Mhz clock
#define BAUD 9600
#define BUADRATE F_CPU/16/BAUD-1

#include <avr/io.h>	// Define io/sram registers
#include <util/delay.h>	// Define delay routines
#include <avr/pgmspace.h> // Define flash memory space
#include <avr/interrupt.h>
#include "lcd.h"
#include "uart.h"

/*
Declaration of the variables used throughout this program
*/

int curr_state = Get_Pin;
int swipe_event = 0;
int pin_event = 0;
int swipe_valid = 0;
int user_pos = 0;
int counter_pin = 0;
int counter = 0;
int flag = 0;
char key;
char tmp2;
char tmp3;

unsigned char tmp = 0;

char curr_userID[id_length];
char curr_userPin[pin_length];
char final_userPin[pin_length-1];
unsigned char sequence;
unsigned char next_bit;
char check_bit;

const char pin[pin_length] PROGMEM = "8989";

// Interrupt for Keypad
ISR(PCINT2_vect){
	tmp2 = PORTD;
	tmp2 &= ~(0x06);	// Clear PD2/PD3
	PORTD = tmp2;
	tmp2 = PORTC;
	tmp2 &= ~(0x16);
	PORTC = tmp2;	// Clear PC4
	
	_delay_ms(100);
	
	if((curr_state == Get_Pin)||(curr_state == Check_Pin)){
		PORTC |= (1 << 5);	// Turn LED off -- Sets bit
		if(bit_is_clear(PIND,4)){
			//PORTD &= ~(1 << 0);
			PORTC |= (1 << 4);
			PORTD |= (1 << 2);
			_delay_loop_2(150);
		
			if(bit_is_clear(PIND,4)){
	
					//curr_userPin[flag] = key;
					lcd_puts("*");
					flag++;
					key = '1';
		
			}else{
		
			PORTD |= (1 << 3);
	
			PORTC &= ~(1 << 4);
		
			_delay_loop_2(150);
		
			if(bit_is_clear(PIND,4)){

					//curr_userPin[flag] = key;
					lcd_puts("*");
					flag++;
					key = '2';
		
			}else{
				
			PORTC |= (1 << 4);
			PORTD &= ~(1 << 2);
		
			_delay_loop_2(150);
		
			if(bit_is_clear(PIND,4)){
	
					//curr_userPin[flag] = key;
					lcd_puts("*");
					flag++;
					key = '3';
			}
			PORTD &= ~(1 << 3);
			PORTC &= ~(1 << 5);
			PORTD &= ~(1 << 2);
		
			}

		}		
	
	}else if(bit_is_clear(PIND,5)){

	//PORTD &= ~(1 << 0);
	PORTC |= (1 << 4);
	PORTD |= (1 << 2);
	_delay_loop_2(100);

	if(bit_is_clear(PIND,5)){
	
		//curr_userPin[flag] = key;
		lcd_puts("*");
		flag++;
		key = '4';
	
		}else{
	
		PORTD |= (1 << 3);
	
		PORTC &= ~(1 << 4);
	
		_delay_loop_2(150);
	
		if(bit_is_clear(PIND,5)){

			//curr_userPin[flag] = key;
			lcd_puts("*");
			flag++;
			key = '5';
		
			}else{
		
			PORTC |= (1 << 4);
			PORTD &= ~(1 << 2);
		
			_delay_loop_2(150);
		
			if(bit_is_clear(PIND,5)){
			
				//curr_userPin[flag] = key;
				lcd_puts("*");
				flag++;
				key = '6';
			}
			PORTD &= ~(1 << 3);
			PORTC &= ~(1 << 4);
			PORTD &= ~(1 << 2);
		
		}

	}

	}else if(bit_is_clear(PIND,6)){

	//PORTD &= ~(1 << 0);
	PORTC |= (1 << 4);
	PORTD |= (1 << 2);
	_delay_loop_2(150);

	if(bit_is_clear(PIND,6)){
	
		//curr_userPin[flag] = key;
		lcd_puts("*");
		flag++;
		key = '7';
	
		}else{
	
		PORTD |= (1 << 3);
	
		PORTC &= ~(1 << 4);
	
		_delay_loop_2(150);
	
		if(bit_is_clear(PIND,6)){

			//curr_userPin[flag] = key;
			lcd_puts("*");
			flag++;
			key = '8';
		
			}else{
		
			PORTC |= (1 << 4);
			PORTD &= ~(1 << 2);
		
			_delay_loop_2(150);
		
			if(bit_is_clear(PIND,6)){
				
				//curr_userPin[flag] = key;
				lcd_puts("*");
				flag++;
				key = '9';
			}
			PORTD &= ~(1 << 3);
			PORTC &= ~(1 << 4);
			PORTD &= ~(1 << 2);
		
		}

	}

	}

	if(bit_is_clear(PIND,7)){

	//PORTD &= ~(1 << 0);
	PORTC |= (1 << 4);
	PORTD |= (1 << 2);
	_delay_loop_2(150);

	if(bit_is_clear(PIND,7)){
	
		//curr_userPin[flag] = key;
		lcd_puts("*");
		flag++;
		key = '*';
	
		}else{
	
		PORTD |= (1 << 3);
	
		PORTC &= ~(1 << 4);
	
		_delay_loop_2(250);
	
		if(bit_is_clear(PIND,7)){

			//curr_userPin[flag] = key;
			lcd_puts("*");
			flag++;
			key = '0';
		
			}else{
		
			PORTC |= (1 << 4);
			PORTD &= ~(1 << 2);
		
			_delay_loop_2(150);
		
			if(bit_is_clear(PIND,7)){
			
				//curr_userPin[flag] = key;
				lcd_puts("*");
				flag++;
				key = '#';
			}
			PORTD &= ~(1 << 3);
			PORTC &= ~(1 << 4);
			PORTD &= ~(1 << 2);
		
		}

	}

	}
	
	_delay_loop_2(150);
	
	if(key != ' '){
		curr_userPin[flag] = key;
		//flag++;
		//lcd_puts("*");
		//PORTC &= ~(1 << 5);
	}
	
	if(flag >= 4){
		//PORTC |= (1 << 5);
		lcd_clrscr;
		lcd_home();
		for(int i = 1; i < 5; i ++){
			//lcd_putc(curr_userPin[i]);
			key = curr_userPin[i];
			final_userPin[i-1] = key;
		}
	
		pin_event = 1;
		flag = 0;
//		check_pin();
	}

}
}

//Data received on the serial port interrupt -- RX
ISR(USART_RX_vect){
	if(curr_state == Validate_User){
		PORTC &= ~(1 << 5);	// Turn LED on
	}
}

void USART_Init(){
	UBRR0H = (unsigned char)(BUADRATE>>8); // Sets up baudrate
	UBRR0L = (unsigned char)BUADRATE;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);	// Enable receiver and transmitter
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);	// Set frame format: 8data/2stop bit
}

int main(void)            // Main Loop
{
	
	DDRB = 0b00101000;
	DDRC = 0b01111111;	// PC5 for keypad (before PD1)
	DDRD = 0b00001110;	// PD0 (RX) input & PD1 (TX) output for serial/ PD3 is now PD0 for keypad
	//DDRD &= ~(1 << PD3);	// Set PD3 as input for mag card
	PORTC |= (1 << 5);	// Turn LED off -- Sets bit
	//PORTB |= (1 << 1);
	PORTD = 0xF0;
	
	USART_Init();
	
	// Enables pin change interrupt 
	tmp3 = PCICR;
	tmp3 = 1 << (PCIE2) | 0 << (PCIE1) | 0 << (PCIE0);
	PCICR = tmp3;
	tmp3 = PCMSK2;
	tmp3 = 1 << (PCINT23) | 1 << (PCINT22) | 1 << (PCINT21) | 1 << (PCINT20) | 0 << (PCINT19) | 0 << (PCINT18) | 0 << (PCINT17) | 0 << (PCINT16);
	PCMSK2 = tmp3;
	
	sei();                // Enables global interrupts

	
	lcd_init(LCD_DISP_ON);
	
	 while(1)
	 {
		 switch(curr_state){
			 
			 case Get_Pin:
				lcd_clrscr();
				//lcd_puts("Print Valid ");
				//lcd_putc('\n');
				lcd_puts("Enter PIN: ");
				while(pin_event != 1){
					_delay_us(50);
					 // Forever loop until keypad enters in 4 numbers for pin
			    }   
				pin_event = 0;
				curr_state = Check_Pin;
			 break;
			 
			 case Check_Pin:
				if(strncmp_P(final_userPin, pin, pin_length) == 0){
					curr_state = Authorize;
				}else{
					lcd_clrscr();
					lcd_puts("Pin invalid");
					_delay_ms(2500);
					lcd_clrscr();
					lcd_puts("Enter PIN:");
					while(pin_event != 1){
						_delay_us(50);
						// Forever loop until keypad enters in 4 numbers for pin
					}
					pin_event = 0;
					curr_state = Check_Pin;
				}
				
			 break;
			 
			 case Authorize:
				lcd_clrscr();
				lcd_puts("Access Granted");
				
				PORTC &= ~(1 << 5);	// Turn LED on for 5 seconds
				_delay_ms(1000);
				swipe_event = 0;
				curr_state = Scan_Finger;
				PORTC |= (1 << 5);	// Turn LED off
			
			  break;
			  
			  case Scan_Finger:
			  lcd_clrscr();	// Initialize LCD with text
			  lcd_puts("Scan Finger");
			  _delay_ms(2500);
			  if(swipe_event == 0){
				  curr_state = Scan_Finger;	// No card swipe
				  }else{
				  curr_state = Validate_User;
			  }
			  break;
			  
			  case Validate_User:
			  
			  break;
			  
			  case Enroll_User:
			  
			  break;
		 }
	 }
}

// Checks fingerprint module for fingerprint match
/*
int check_cardID(){
	for(int count = 0; count < user_length; count++){
		if(strncmp_P(curr_userID, id[count], id_length) == 0){
			user_pos = count;	// Used for correct associated pin #
			swipe_valid = 1;	// Correct swipe => Get Pin #
			return swipe_valid;
		}
	}
	return 0;
}
*/

