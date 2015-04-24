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
#define Enroll_User		4
#define EXIT			5
#define pin_length		4
#define F_CPU 8000000UL // Defines 8Mhz clock
#define BAUD 9600
#define BAUDRATE F_CPU/16/BAUD-1

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
int finger_event = 0;
int pin_event = 0;
int swipe_valid = 0;
int user_pos = 0;
int counter_pin = 0;
int counter = 0;
int count_finger = 0;
int flag = 0;
int count = 0;
int enroll_id = 0;
int checksumLow = 0;
char key;
char tmp2;
char tmp3;

unsigned char tmp = 0;

char curr_userPin[pin_length];
char final_userPin[pin_length-1];
unsigned char data;
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
	
	if((curr_state == Get_Pin)||(curr_state == Check_Pin)||(curr_state == Scan_Finger)){
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
		finger_event = 2;
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
		data = UDR0;
		
		if((curr_state == Check_Pin)&&(count == 4)){
			if(data != 0){
				enroll_id = data;
				enroll_id++;
			}else if(data == 0){
				enroll_id = data;
			}
			lcd_clrscr();
			lcd_puts("get enroll count");
			_delay_ms(1000);
		}
		if((curr_state == Scan_Finger)&&(data == 0x01)){
			finger_event = 1;
			lcd_clrscr();
			lcd_puts("finger pressed");
			_delay_ms(1000);
		}
		if(data == 0x31){
			lcd_clrscr();
			lcd_puts("response error");
			_delay_ms(1000);
			finger_event = 2;
		}
		if(data == 0x30){
			lcd_clrscr();
			lcd_puts("ack good");
			_delay_ms(1000);
		}
		lcd_clrscr();
		
		if(curr_state == Check_Pin){
			count++;
			if(count > 11){
				count == 0;
			}
		}
		/*
		if(curr_state == Get_Pin){
			count_finger++;
			if(count_finger > 11){
				count_finger = 0;
			}
		}*/
		_delay_ms(10);
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

	// Enables pin change interrupt  
	tmp3 = PCICR;
	tmp3 = 1 << (PCIE2) | 0 << (PCIE1) | 0 << (PCIE0);
	PCICR = tmp3;
	tmp3 = PCMSK2;
	tmp3 = 1 << (PCINT23) | 1 << (PCINT22) | 1 << (PCINT21) | 1 << (PCINT20) | 0 << (PCINT19) | 0 << (PCINT18) | 0 << (PCINT17) | 0 << (PCINT16);
	PCMSK2 = tmp3;
	sei();                // Enables global interrupts
	lcd_init(LCD_DISP_ON);
	uart_init(BAUDRATE);
	//uart_init(51);
	//USART_Init();
	_delay_ms(100);
	initialize_fps();
	_delay_ms(100);
	led_on();
	_delay_ms(100);
	
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
				getEnrollCount();
				_delay_ms(100);
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
				
				PORTC &= ~(1 << 5);	// Turn LED on 
				_delay_ms(2500);
				curr_state = Scan_Finger;
			
				PORTC |= (1 << 5);	// Turn LED off
			 break;
			  
			case Scan_Finger:
			  fingerPressed();
			  _delay_ms(100);
			  lcd_clrscr();	// Initialize LCD with text
			  lcd_puts("Scan Finger");
			  _delay_ms(2500);
			  count_finger = 0;
			  if(finger_event == 0){
				  curr_state = Scan_Finger;	// No finger pressed
				  }else if (finger_event == 1){
					 curr_state = Enroll_User;
				 }else{
					 curr_state = EXIT;
				 }
			break;
			  
			case Enroll_User:
			  enrollStart();
			break;
			
			case EXIT:
				lcd_clrscr();
				lcd_puts("User exit");
				_delay_ms(2000);
				finger_event = 0;
				curr_state = Get_Pin;
			break;
		 }
	 }
}

void initialize_fps(){
	// Open connection 

	uart_transmit(0x55);	// Start code 1
	uart_transmit(0xAA);	// Start code 2
	uart_transmit(0x01);	// Device ID
	uart_transmit(0x00);	// Second part of ID
	uart_transmit(0x01);	// Input parameter byte 1 (largest byte of param)
	uart_transmit(0x00);	// Input parameter byte 2 (second largest byte of param)
	uart_transmit(0x00);	// Input parameter byte 3
	uart_transmit(0x00);	// Input parameter byte 4	
	uart_transmit(0x01);	// Byte 1 of command
	uart_transmit(0x00);	// Byte 2 of command
	uart_transmit(0x02);	// Low byte checksum
	uart_transmit(0x01);	// High byte checksum
	
	//uart_receive();
}

// Turn FPS LED on
void led_on(){
	
	uart_transmit(0x55);	// Start code 1
	uart_transmit(0xAA);	// Start code 2
	uart_transmit(0x01);	// Device ID
	uart_transmit(0x00);	// Second part of ID
	uart_transmit(0x01);	// Input parameter byte 1 -- LED ON
	uart_transmit(0x00);	// Input parameter byte 2
	uart_transmit(0x00);	// Input parameter byte 3
	uart_transmit(0x00);	// Input parameter byte 4
	uart_transmit(0x12);	// Byte 1 of command
	uart_transmit(0x00);	// Byte 2 of command
	uart_transmit(0x13);	// Low byte checksum
	uart_transmit(0x01);	// High byte checksum
	
	//uart_receive();
}

void getEnrollCount(){
	uart_transmit(0x55);	// Start code 1
	uart_transmit(0xAA);	// Start code 2
	uart_transmit(0x01);	// Device ID
	uart_transmit(0x00);	// Second part of ID
	uart_transmit(0x01);	// Input parameter byte 1 
	uart_transmit(0x00);	// Input parameter byte 2
	uart_transmit(0x00);	// Input parameter byte 3
	uart_transmit(0x00);	// Input parameter byte 4
	uart_transmit(0x20);	// Byte 1 of command
	uart_transmit(0x00);	// Byte 2 of command
	uart_transmit(0x21);	// Low byte checksum
	uart_transmit(0x01);	// High byte checksum
}

void fingerPressed(){
	uart_transmit(0x55);	// Start code 1
	uart_transmit(0xAA);	// Start code 2
	uart_transmit(0x01);	// Device ID
	uart_transmit(0x00);	// Second part of ID
	uart_transmit(0x01);	// Input parameter byte 1
	uart_transmit(0x00);	// Input parameter byte 2
	uart_transmit(0x00);	// Input parameter byte 3
	uart_transmit(0x00);	// Input parameter byte 4
	uart_transmit(0x26);	// Byte 1 of command
	uart_transmit(0x00);	// Byte 2 of command
	uart_transmit(0x27);	// Low byte checksum
	uart_transmit(0x01);	// High byte checksum
}

void enrollStart(){
	checksumLow = enroll_id + 22;
	uart_transmit(0x55);	// Start code 1
	uart_transmit(0xAA);	// Start code 2
	uart_transmit(0x01);	// Device ID
	uart_transmit(0x00);	// Second part of ID
	uart_transmit(enroll_id);	// Input parameter byte 1
	uart_transmit(0x00);	// Input parameter byte 2
	uart_transmit(0x00);	// Input parameter byte 3
	uart_transmit(0x00);	// Input parameter byte 4
	uart_transmit(0x22);	// Byte 1 of command
	uart_transmit(0x00);	// Byte 2 of command
	uart_transmit(checksumLow);	// Low byte checksum
	uart_transmit(0x01);	// High byte checksum
}

void uart_transmit(unsigned char data){
	//_delay_ms(10);
	cli();
	uart_putc(data);
	//while(!(UCSR0A & (1 << UDRE0)));
	//UDR0 = data;
	//while((UCSR0A & (1 << TXC0)) == 0){};
	sei();
}
/*
void uart_receive(){
	while(!(UCSR0A & (1 << RXEN0)));
	// Error
	if(UDR0 == 0x31){
		PORTC &= ~(1 << 5);	// Turn LED on 
	}
	//return UDR0;
}

void USART_Init(){
	UCSR0A = (1<<U2X0);  //Enable 2x speed 
	UBRR0H = (unsigned char)(BAUDRATE>>8); // Sets up baudrate
	UBRR0L = (unsigned char)BAUDRATE;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);	// Enable receiver and transmitter
	UCSR0C |= (3<<UCSZ00)|(0<<USBS0);	// FPS 8 data, no parity, 1 stop bit
}*/