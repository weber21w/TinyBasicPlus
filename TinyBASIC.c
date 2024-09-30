#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <uzebox.h>
#include <fatfs/ffconf.h>
#include <fatfs/ff.h>
#include <fatfs/diskio.h>

#define Wait200ns() asm volatile("lpm\n\tlpm\n\t");
#define Wait100ns() asm volatile("lpm\n\t");

#define kAutorunFilename "AUTORUN.BAS"
/*
static void turnOffPWM(uint8_t timer){

	switch (timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   cbi(TCCR1A, COM1C1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
		#endif
	}
}
*/

//uint8_t analog_reference = DEFAULT;
void analogReference(uint8_t mode){
/*
	// can't actually set the register here because the default setting
	// will connect AVCC and the AREF pin, which would cause a short if
	// there's something connected to AREF.
	analog_reference = mode;
*/
}

u16 analogRead(u8 pin){
/*
#if defined(analogPinToChannel)

	pin = analogPinToChannel(pin);
	if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#endif

#if defined(ADCSRB) && defined(MUX5)
	// the MUX5 bit of ADCSRB selects whether we're reading from channels
	// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif
  
	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
	// to 0 (the default).
#if defined(ADMUX)
	ADMUX = (analog_reference << 6) | (pin & 0x07);
#endif

	// without a delay, we seem to read from the wrong channel
	//delay(1);

#if defined(ADCSRA) && defined(ADC)
	// start the conversion
	sbi(ADCSRA, ADSC);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// ADC macro takes care of reading ADC register.
	// avr-gcc implements the proper reading order: ADCL is read first.
	return ADC;
#endif
*/
	return 0;
}
u8 digitalRead(u8 pin){/*
	uint8_t timer = digitalPinToTimer(pin);
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);

	if (port == NOT_A_PIN) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
*/
	return 0;
}

void analogWrite(u8 pin, u16 val){
/*
	// We need to make sure the PWM output is enabled for those pins
	// that support it, as we turn it off when digitally reading or
	// writing with them.  Also, make sure the pin is in output mode
	// for consistenty with Wiring, which doesn't require a pinMode
	// call for the analog output pins.
	pinMode(pin, OUTPUT);
	if (val == 0){
		digitalWrite(pin, LOW);
	}else if (val == 255){
		digitalWrite(pin, HIGH);
	}else{
		switch(digitalPinToTimer(pin)){
			// XXX fix needed for atmega8
			#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
			case TIMER0A:
				// connect pwm to pin on timer 0
				sbi(TCCR0, COM00);
				OCR0 = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR0A) && defined(COM0A1)
			case TIMER0A:
				// connect pwm to pin on timer 0, channel A
				sbi(TCCR0A, COM0A1);
				OCR0A = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR0A) && defined(COM0B1)
			case TIMER0B:
				// connect pwm to pin on timer 0, channel B
				sbi(TCCR0A, COM0B1);
				OCR0B = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR1A) && defined(COM1A1)
			case TIMER1A:
				// connect pwm to pin on timer 1, channel A
				sbi(TCCR1A, COM1A1);
				OCR1A = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR1A) && defined(COM1B1)
			case TIMER1B:
				// connect pwm to pin on timer 1, channel B
				sbi(TCCR1A, COM1B1);
				OCR1B = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR1A) && defined(COM1C1)
			case TIMER1C:
				// connect pwm to pin on timer 1, channel C
				sbi(TCCR1A, COM1C1);
				OCR1C = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR2) && defined(COM21)
			case TIMER2:
				// connect pwm to pin on timer 2
				sbi(TCCR2, COM21);
				OCR2 = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR2A) && defined(COM2A1)
			case TIMER2A:
				// connect pwm to pin on timer 2, channel A
				sbi(TCCR2A, COM2A1);
				OCR2A = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR2A) && defined(COM2B1)
			case TIMER2B:
				// connect pwm to pin on timer 2, channel B
				sbi(TCCR2A, COM2B1);
				OCR2B = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR3A) && defined(COM3A1)
			case TIMER3A:
				// connect pwm to pin on timer 3, channel A
				sbi(TCCR3A, COM3A1);
				OCR3A = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR3A) && defined(COM3B1)
			case TIMER3B:
				// connect pwm to pin on timer 3, channel B
				sbi(TCCR3A, COM3B1);
				OCR3B = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR3A) && defined(COM3C1)
			case TIMER3C:
				// connect pwm to pin on timer 3, channel C
				sbi(TCCR3A, COM3C1);
				OCR3C = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR4A)
			case TIMER4A:
				//connect pwm to pin on timer 4, channel A
				sbi(TCCR4A, COM4A1);
				#if defined(COM4A0)		// only used on 32U4
				cbi(TCCR4A, COM4A0);
				#endif
				OCR4A = val;	// set pwm duty
				break;
			#endif
			
			#if defined(TCCR4A) && defined(COM4B1)
			case TIMER4B:
				// connect pwm to pin on timer 4, channel B
				sbi(TCCR4A, COM4B1);
				OCR4B = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR4A) && defined(COM4C1)
			case TIMER4C:
				// connect pwm to pin on timer 4, channel C
				sbi(TCCR4A, COM4C1);
				OCR4C = val; // set pwm duty
				break;
			#endif
				
			#if defined(TCCR4C) && defined(COM4D1)
			case TIMER4D:				
				// connect pwm to pin on timer 4, channel D
				sbi(TCCR4C, COM4D1);
				#if defined(COM4D0)		// only used on 32U4
				cbi(TCCR4C, COM4D0);
				#endif
				OCR4D = val;	// set pwm duty
				break;
			#endif

							
			#if defined(TCCR5A) && defined(COM5A1)
			case TIMER5A:
				// connect pwm to pin on timer 5, channel A
				sbi(TCCR5A, COM5A1);
				OCR5A = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR5A) && defined(COM5B1)
			case TIMER5B:
				// connect pwm to pin on timer 5, channel B
				sbi(TCCR5A, COM5B1);
				OCR5B = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR5A) && defined(COM5C1)
			case TIMER5C:
				// connect pwm to pin on timer 5, channel C
				sbi(TCCR5A, COM5C1);
				OCR5C = val; // set pwm duty
				break;
			#endif

			case NOT_ON_TIMER:
			default:
				if(val < 128){
					digitalWrite(pin, LOW);
				}else{
					digitalWrite(pin, HIGH);
				}
		}
	}
*/
}

void digitalWrite(u8 pin, u8 val){
/*
	uint8_t timer = digitalPinToTimer(pin);
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *out;

	if (port == NOT_A_PIN) return;

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	out = portOutputRegister(port);

	uint8_t oldSREG = SREG;
	cli();

	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}

	SREG = oldSREG;
*/
}

void tone(u8 f, u8 d){

}

void noTone(){

}
#define PM_INPUT	0
#define PM_OUTPUT	1
void pinMode(u8 pin, u8 mode){
/*
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	//TODO VERIFY PIN
	volatile uint8_t *reg = portModeRegister(port);
	volatile uint8_t *out = portOutputRegister(port);

	if(mode == INPUT){ 
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out &= ~bit;
		SREG = oldSREG;
	}else if (mode == INPUT_PULLUP){
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out |= bit;
		SREG = oldSREG;
	}else{
		uint8_t oldSREG = SREG;
                cli();
		*reg |= bit;
		SREG = oldSREG;
	}
*/
}

void delay(u8 ms){
	s16 time = ms;
	while(time > 0){
		if(GetVsyncFlag()){
			WaitVsync(1);
			time -= 16;
			continue;
		}
		for(u16 i=0;i<1000;i++){
			for(u8 j=0;j<5;j++){
				Wait200ns();
			}
		}
		time--;
	}
}

u8 con_x = 0;
u8 con_y = 0;
void ConsolePrintChar(char c){
	if(c == 0xFE){while(1);}
	if(c == 8){//backspace
	//	PrintChar(con_x, con_y, ' ');
	//	con_x--;//eat the increment done already
		if(0){//con_x > 1){
			con_x-=2;
		}else if(con_x > 0){
			con_x--;
		}else if(0){//con_y){
			con_y--;
		}else
			return;
		PrintChar(con_x, con_y, ' ');
		return;
	}else if(c == 9){//clear screen
		ClearVram();
		con_x = 0;
		con_y = 0;
		return;
	}if(c == '\r'){
		con_x = 0;
	}else if(c == '\n'){
		if(++con_y >= SCREEN_TILES_V){
			ClearVram();
			con_x = 0;
			con_y = 0;
		}
	}else if(c == '\t'){
		con_x += 4;
		if(con_x >= SCREEN_TILES_H)
			con_x = 0;
	}else{
		PrintChar(con_x++, con_y, c);
	}
}

#define KB_SEND_KEY 0x00
#define KB_SEND_END 0x01
#define KB_SEND_DEVICE_ID 0x02
#define KB_SEND_FIRMWARE_REV 0x03
#define KB_RESET 0x7f

#define KB_EXT_FLAG_EXT 0x100
#define KB_CTRL_FLAG 	0x200
#define KB_ALT_FLAG 	0x400


// Unshifted characters
const unsigned char unshifted[][2] PROGMEM = {
	{0x0d,9},
	{0x0e,'`'},
	{0x15,'q'},
	{0x16,'1'},
	{0x1a,'z'},
	{0x1b,'s'},
	{0x1c,'a'},
	{0x1d,'w'},
	{0x1e,'2'},
	{0x21,'c'},
	{0x22,'x'},
	{0x23,'d'},
	{0x24,'e'},
	{0x25,'4'},
	{0x26,'3'},
	{0x29,' '},
	{0x2a,'v'},
	{0x2b,'f'},
	{0x2c,'t'},
	{0x2d,'r'},
	{0x2e,'5'},
	{0x31,'n'},
	{0x32,'b'},
	{0x33,'h'},
	{0x34,'g'},
	{0x35,'y'},
	{0x36,'6'},
	{0x3a,'m'},
	{0x3b,'j'},
	{0x3c,'u'},
	{0x3d,'7'},
	{0x3e,'8'},
	{0x41,','},
	{0x42,'k'},
	{0x43,'i'},
	{0x44,'o'},
	{0x45,'0'},
	{0x46,'9'},
	{0x49,'.'},
	{0x4a,'/'},
	{0x4b,'l'},
	{0x4c,';'},
	{0x4d,'p'},
	{0x4e,'-'},
	{0x52,'\''},
	{0x54,'['},
	{0x55,'='},
	{0x5a,13},
	{0x5b,']'},
	{0x5d,'\\'},
	{0x66,8},
	{0x69,'1'},
	{0x6b,'4'},
	{0x6c,'7'},
	{0x70,'0'},
	{0x71,'.'},
	{0x72,'2'},
	{0x73,'5'},
	{0x74,'6'},
	{0x75,'8'},
	{0x79,'+'},
	{0x7a,'3'},
	{0x7b,'-'},
	{0x7c,'*'},
	{0x7d,'9'},
	{0x76,27},  //ESC

	{0x05,128}, //F1
	{0x06,129}, //F2
	{0x04,130}, //F3
	{0x0c,131}, //F4
	{0x03,132}, //F5
	{0x0b,133}, //F6
	{0x83,134}, //F7
	{0x0a,135}, //F8
	{0x01,136}, //F9
	{0x09,137}, //F10
	{0,0}
};

// Shifted characters
const unsigned char shifted[][2] PROGMEM= {
	{0x0d,9},
	{0x0e,'~'},
	{0x15,'Q'},
	{0x16,'!'},
	{0x1a,'Z'},
	{0x1b,'S'},
	{0x1c,'A'},
	{0x1d,'W'},
	{0x1e,'@'},
	{0x21,'C'},
	{0x22,'X'},
	{0x23,'D'},
	{0x24,'E'},
	{0x25,'$'},
	{0x26,'#'},
	{0x29,' '},
	{0x2a,'V'},
	{0x2b,'F'},
	{0x2c,'T'},
	{0x2d,'R'},
	{0x2e,'%'},
	{0x31,'N'},
	{0x32,'B'},
	{0x33,'H'},
	{0x34,'G'},
	{0x35,'Y'},
	{0x36,'^'},
	{0x3a,'M'},
	{0x3b,'J'},
	{0x3c,'U'},
	{0x3d,'&'},
	{0x3e,'*'},
	{0x41,'<'},
	{0x42,'K'},
	{0x43,'I'},
	{0x44,'O'},
	{0x45,')'},
	{0x46,'('},
	{0x49,'>'},
	{0x4a,'?'},
	{0x4b,'L'},
	{0x4c,':'},
	{0x4d,'P'},
	{0x4e,'_'},
	{0x52,'\"'},
	{0x54,'{'},
	{0x55,'+'},
	{0x5a,13},
	{0x5b,'}'},
	{0x5d,'|'},
	{0x66,8},
	{0x69,'1'},
	{0x6b,'4'},
	{0x6c,'7'},
	{0x70,'0'},
	{0x71,'>'},
	{0x72,'2'},
	{0x73,'5'},
	{0x74,'6'},
	{0x75,'8'},
	{0x79,'+'},
	{0x7a,'3'},
	{0x7b,'-'},
	{0x7c,'*'},
	{0x7d,'9'},
	{0x76,27},  //ESC
	{0x05,128}, //F1
	{0x06,129}, //F2
	{0x04,130}, //F3
	{0x0c,131}, //F4
	{0x03,132}, //F5
	{0x0b,133}, //F6
	{0x83,134}, //F7
	{0x0a,135}, //F8
	{0x01,136}, //F9
	{0x09,137}, //F10
	{0,0}
};

u8 is_up=0, shift = 0, mode = 0;
u16 ctrl,alt;
u8 is_extended=0;

u8 GetKeyboardChar(u8 command){
	static u8 state=0;
	u8 data=0;

	unsigned char i;

	if(state==0){

		//ready to transmit condition
		JOYPAD_OUT_PORT&=~(_BV(JOYPAD_CLOCK_PIN));
		JOYPAD_OUT_PORT|=_BV(JOYPAD_LATCH_PIN);
		Wait200ns();
		Wait200ns();
		Wait200ns();
		Wait200ns();
		Wait200ns();
		JOYPAD_OUT_PORT&=~(_BV(JOYPAD_LATCH_PIN));
		JOYPAD_OUT_PORT|=_BV(JOYPAD_CLOCK_PIN);
		Wait200ns();
		Wait200ns();
		Wait200ns();
		Wait200ns();
		Wait200ns();

		if(command==KB_SEND_END){
			state=0;
		}else{
			state=1;
		}
	}

	//read button states
	for(i=0;i<8;i++){

		data<<=1;

		//data out
		if(command&0x80){
			JOYPAD_OUT_PORT|=_BV(JOYPAD_LATCH_PIN);
		}else{
			JOYPAD_OUT_PORT&=~(_BV(JOYPAD_LATCH_PIN));
		}

		//pulse clock pin
		JOYPAD_OUT_PORT&=~(_BV(JOYPAD_CLOCK_PIN));

		command<<=1;
		Wait100ns();

		if((JOYPAD_IN_PORT&(1<<JOYPAD_DATA2_PIN))!=0) data|=1;

		JOYPAD_OUT_PORT|=_BV(JOYPAD_CLOCK_PIN);

	}

	JOYPAD_OUT_PORT&=~(_BV(JOYPAD_LATCH_PIN));

	//return data;
	u8 sc = data;
	if(sc==0xe0){
		is_extended=1;
		return 0;
	}else{
		unsigned char i,c;
		if (!is_up)// Last data received was the up-key identifier
		{
			switch (sc)
			{
				case 0xF0 :// The up-key identifier
					is_up = 1;
					break;
				case 0x08 ://backspace
					return 8;
				case 0x12 :// Left SHIFT
				case 0x59 :// Right SHIFT
					shift = 1;
					break;

				case 0x14 :// Left Control (right ctrl is extended 0x0e+0x14)
					ctrl = KB_CTRL_FLAG; //ctlr flag bit
					break;

				case 0x11 :// Left Alt (right Alt is extended 0x0e+0x11)
					alt = KB_ALT_FLAG; //alt flag bit
					break;

				default:
					if(is_extended){
						is_extended=0;
						return alt|ctrl|0x0100|sc;
					}else{

						if(!shift)// If shift not pressed,
						{ // do a table look-up
							for(i = 0; pgm_read_byte(&(unshifted[i][0]))!=sc && pgm_read_byte(&(unshifted[i][0])); i++);
							if (pgm_read_byte(&(unshifted[i][0])) == sc) {
								c=pgm_read_byte(&(unshifted[i][1]));

								return alt|ctrl|c;
							}
						} else {// If shift pressed
							for(i = 0; pgm_read_byte(&(shifted[i][0]))!=sc && pgm_read_byte(&(shifted[i][0])); i++);
							if (pgm_read_byte(&(shifted[i][0])) == sc) {
								c=pgm_read_byte(&(shifted[i][1]));

								return alt|ctrl|c;
							}
						}
					}
					break;
			}

		} else {
			is_up = 0;// Two 0xF0 in a row not allowed
			switch (sc)
			{
				case 0x12 :// Left SHIFT
				case 0x59 :// Right SHIFT
					shift = 0;
					break;

				case 0x14 :// Left control (right ctrl is extended 0x0e+0x14)
					ctrl = 0;
					break;

				case 0x11 :// Left Alt (right Alt is extended 0x0e+0x11)
					alt = 0; //alt flag bit
					break;
			}
		}

	}
	is_extended=0;
	return 0; //invalid char

}



#define kConsoleBaud 9600
#define kVersion "0.2"
#define kRamSize 512
FATFS fs;
FIL f;
//DIR d;
u16 bytesWritten;
u16 bytesRead;
u8 sd_initialized = 0;


//#define NO_SPI_RAM 1

#ifndef NO_SPI_RAM
	#include <spiram.h>

	uint16_t spiram_cursor = 0;
	uint16_t spiram_state = 0;

	uint8_t SpiRamCursorInit(){
		spiram_cursor = kRamSize
		spiram_state = 0;
		if(!SpiRamInit())
			return 0;
		SpiRamSeqReadStart((kRamSize<<16)&0xFF, (uint16_t)kRamSize&0xFFFF);
		return 1;
	}

	uint8_t SpiRamCursorRead(uint16_t addr){
		if(spiram_state){//in a sequential write?
			SpiRamSeqWriteEnd();
			asm("nop");asm("nop");
			SpiRamSeqReadStart(0, addr);
			asm("nop");asm("nop");
			spiram_state = 0;
			spiram_cursor = addr+1;
			return SpiRamSeqReadU8();
		}
		if(spiram_cursor != addr){//current sequential read position doesn't match?
			SpiRamSeqReadEnd();
			asm("nop");asm("nop");
			SpiRamSeqReadStart(0, addr);
			asm("nop");asm("nop");
			spiram_cursor = addr+1;
			return SpiRamSeqReadU8();
		}
		spiram_cursor++;
		return SpiRamSeqReadU8();
	}

	void SpiRamCursorWrite(uint16_t addr, uint8_t val){
		if(!spiram_state){//in a sequential read?
			SpiRamSeqReadEnd();
			asm("nop");asm("nop");
			SpiRamSeqWriteStart(0, addr);
			spiram_state = 1;
			spiram_cursor = addr+1;
			asm("nop");asm("nop");
			SpiRamSeqWriteU8(val);
			return;
		}
		if(spiram_cursor != addr){//current sequential write position doesn't match?
			SpiRamSeqWriteEnd();
			asm("nop");asm("nop");
			SpiRamSeqWriteStart(0, addr);
			spiram_cursor = addr+1;
			asm("nop");asm("nop");
			SpiRamSeqWriteU8(val);
			return;
		}
		spiram_cursor++;
		SpiRamSeqWriteU8(val);
	}
#endif


////////////////////
	//functions defined elsehwere
	void cmd_Files();
	char *filenameWord();

//some settings based things

u8 inhibitOutput = 0;
static u8 runAfterLoad = 0;
static u8 triggerRun = 0;

//these will select, at runtime, where IO happens through for load/save
enum{
	kStreamSerial = 0,
	kStreamFile,
	kStreamKeyboard,
	kStreamScreen
};
static u8 inStream = kStreamKeyboard;
static u8 outStream = kStreamScreen;


////////////////////////////////////////////////////////////////////////////////
//ASCII Characters
#define CR	'\r'
#define NL	'\n'
#define LF	0x0a
#define TAB	'\t'
#define BELL	'\b'
#define SPACE	 ' '
#define SQUOTE	'\''
#define DQUOTE	'\"'
#define CTRLC	0x03
#define CTRLH	0x08
#define CTRLS	0x13
#define CTRLX	0x18

typedef u16 LINENUM;

static u8 program[kRamSize];
//static const char *sentinel = "HELLO";
static u8 *txtpos,*list_line, *tmptxtpos;
static u8 expression_error;
static u8 *tempsp;

/***********************************************************/
//Keyword table and constants - the last character has 0x80 added to it
const static u8 keywords[] PROGMEM = {
	'L','I','S','T'+0x80,
	'L','O','A','D'+0x80,
	'N','E','W'+0x80,
	'R','U','N'+0x80,
	'S','A','V','E'+0x80,
	'N','E','X','T'+0x80,
	'L','E','T'+0x80,
	'I','F'+0x80,
	'G','O','T','O'+0x80,
	'G','O','S','U','B'+0x80,
	'R','E','T','U','R','N'+0x80,
	'R','E','M'+0x80,
	'F','O','R'+0x80,
	'I','N','P','U','T'+0x80,
	'P','R','I','N','T'+0x80,
	'P','O','K','E'+0x80,
	'S','T','O','P'+0x80,
	'B','Y','E'+0x80,
	'F','I','L','E','S'+0x80,
	'M','E','M'+0x80,
	'?'+ 0x80,
	'\''+ 0x80,
	'A','W','R','I','T','E'+0x80,
	'D','W','R','I','T','E'+0x80,
	'D','E','L','A','Y'+0x80,
	'E','N','D'+0x80,
	'R','S','E','E','D'+0x80,
	'C','H','A','I','N'+0x80,
	'T','O','N','E','W'+0x80,
	'T','O','N','E'+0x80,
	'N','O','T','O','N','E'+0x80,
	0
};
		
enum{//by moving the command list to an enum, we can easily remove sections above and below simultaneously to selectively obliterate functionality.
	KW_LIST = 0,
	KW_LOAD, KW_NEW, KW_RUN, KW_SAVE,
	KW_NEXT, KW_LET, KW_IF,
	KW_GOTO, KW_GOSUB, KW_RETURN,
	KW_REM,
	KW_FOR,
	KW_INPUT, KW_PRINT,
	KW_POKE,
	KW_STOP, KW_BYE,
	KW_FILES,
	KW_MEM,
	KW_QMARK, KW_QUOTE,
	KW_AWRITE, KW_DWRITE,
	KW_DELAY,
	KW_END,
	KW_RSEED,
	KW_CHAIN,
	KW_TONEW, KW_TONE, KW_NOTONE,
	KW_DEFAULT /* always the final one*/
};

struct stack_for_frame{
	char frame_type;
	char for_var;
	s16 terminal;
	s16 step;
	u8 *current_line;
	u8 *txtpos;
};

struct stack_gosub_frame{
	char frame_type;
	u8 *current_line;
	u8 *txtpos;
};

const static u8 func_tab[] PROGMEM = {
'P','E','E','K'+0x80,
'A','B','S'+0x80,
'A','R','E','A','D'+0x80,
'D','R','E','A','D'+0x80,
'R','N','D'+0x80,
0
};
#define FUNC_PEEK	0
#define FUNC_ABS	1
#define FUNC_AREAD	2
#define FUNC_DREAD	3
#define FUNC_RND	4
#define FUNC_UNKNOWN	5

const static u8 to_tab[] PROGMEM = {
	'T','O'+0x80,
	0
};

const static u8 step_tab[] PROGMEM = {
	'S','T','E','P'+0x80,
	0
};

const static u8 relop_tab[] PROGMEM = {
	'>','='+0x80,
	'<','>'+0x80,
	'>'+0x80,
	'='+0x80,
	'<','='+0x80,
	'<'+0x80,
	'!','='+0x80,
	0
};

#define RELOP_GE	0
#define RELOP_NE	1
#define RELOP_GT	2
#define RELOP_EQ	3
#define RELOP_LE	4
#define RELOP_LT	5
#define RELOP_NE_BANG	6
#define RELOP_UNKNOWN	7

const static u8 highlow_tab[] PROGMEM = { 
	'H','I','G','H'+0x80,
	'H','I'+0x80,
	'L','O','W'+0x80,
	'L','O'+0x80,
	0
};
#define HIGHLOW_HIGH	1
#define HIGHLOW_UNKNOWN	4

#define STACK_DEPTH	5
#define STACK_SIZE (sizeof(struct stack_for_frame)*STACK_DEPTH)
#define VAR_SIZE sizeof(s16)//Size of variables in bytes

static u8 *stack_limit;
static u8 *program_start;
static u8 *program_end;
///////static u8 *stack;//Software stack for things that should go on the CPU stack
static u8 *variables_begin;
static u8 *current_line;
static u8 *sp;
#define STACK_GOSUB_FLAG 'G'
#define STACK_FOR_FLAG 'F'
static u8 table_index;
static LINENUM linenum;

static const char okmsg[]		PROGMEM = "OK";
static const char whatmsg[]		PROGMEM = "What? ";
static const char howmsg[]		PROGMEM = "How?";
static const char sorrymsg[]		PROGMEM = "Sorry!";
static const char initmsg[]		PROGMEM = "UzeBASIC " kVersion;
static const char memorymsg[]		PROGMEM = " bytes free.";

static const char breakmsg[]		PROGMEM = "break!";
//static const char unimplimentedmsg[]	PROGMEM = "Unimplemented";
static const char backspacemsg[]	PROGMEM = "\b \b";
static const char indentmsg[]		PROGMEM = "    ";
static const char sderrormsg[]		PROGMEM = "ERROR: Failed to initialize SD Card, read/write is disabled.";
static const char sdsuccessmsg[]	PROGMEM = "SUCCESS: SD is initialized";
static const char sdfilemsg[]		PROGMEM = "ERROR: File Operation failed.";
static const char dirextmsg[]		PROGMEM = "(dir)";
static const char slashmsg[]		PROGMEM = "/";
static const char spacemsg[]		PROGMEM = " ";

static s16 inchar();
static void outchar(char c);
static void line_terminator();
static s16 expression();
static char breakcheck();
/***************************************************************************/
static void ignore_blanks(){
	while(*txtpos == SPACE || *txtpos == TAB)
		txtpos++;
}


/***************************************************************************/
static void scantable(const u8 *table){
	s16 i = 0;
	table_index = 0;
	while(1){
		//if(GetVsyncFlag()) WaitVsync(1);
		if(pgm_read_byte(table) == 0)//run out of table entries?
			return;

		if(txtpos[i] == pgm_read_byte(table)){//do we match this character?
			i++;
			table++;
		}else{//do we match the last character of keywork (with 0x80 added)? If so, return
			if(txtpos[i]+0x80 == pgm_read_byte(table)){
				txtpos += i+1;//Advance the pointer to following the keyword
				ignore_blanks();
				return;
			}
			while((pgm_read_byte(table) & 0x80) == 0)//Forward to the end of this keyword
				table++;

			table++;////Now move on to the first character of the next word...
			table_index++;
			ignore_blanks();//...and reset the position index
			i = 0;
		}
	}
}

/***************************************************************************/
static void pushb(u8 b){
	sp--;
	*sp = b;
}

/***************************************************************************/
static u8 popb(){
	u8 b;
	b = *sp;
	sp++;
	return b;
}

/***************************************************************************/
void printnum(s16 num){
	s16 digits = 0;

	if(num < 0){
		num = -num;
		outchar('-');
	}
	do{
		pushb(num%10+'0');
		num = num/10;
		digits++;
	}
	while (num > 0);

	while(digits > 0){
		outchar(popb());
		digits--;
	}
}

void printUnum(u16 num){
	s16 digits = 0;

	do{
		pushb(num%10+'0');
		num = num/10;
		digits++;
	}
	while(num > 0);

	while(digits > 0){
		outchar(popb());
		digits--;
	}
}

/***************************************************************************/
static u16 testnum(){
	u16 num = 0;
	ignore_blanks();

	while(*txtpos>= '0' && *txtpos <= '9'){
		if(num >= 0xFFFF/10){//trap overflows
			num = 0xFFFF;
			break;
		}

		num = num *10 + *txtpos - '0';
		txtpos++;
	}
	return num;
}

/***************************************************************************/
static u8 print_quoted_string(){
	s16 i=0;
	u8 delim = *txtpos;
	if(delim != '"' && delim != '\'')
		return 0;
	txtpos++;

	while(txtpos[i] != delim){//check we have a closing delimiter
		if(txtpos[i] == NL)
			return 0;
		i++;
	}

	while(*txtpos != delim){//print the characters
		outchar(*txtpos);
		txtpos++;
	}
	txtpos++;//skip over the last delimiter
	return 1;
}


/***************************************************************************/
void printmsgNoNL(const char *msg){
	while(pgm_read_byte(msg) != 0){
		outchar(pgm_read_byte(msg++));
	}
}

/***************************************************************************/
void printmsg(const char *msg){
	printmsgNoNL(msg);
	line_terminator();
}

/***************************************************************************/
static void getln(char prompt){
	outchar(prompt);
	txtpos = program_end+sizeof(LINENUM);

	while(1){
		if(GetVsyncFlag()) WaitVsync(1);
		char c = inchar();
		if(0){//c == 8){
			if(txtpos > variables_begin-2){
				txtpos[0] = ' ';
				txtpos--;
				outchar(' ');
			}
			continue;
		}
		switch(c){
		case NL:
			//break;
		case CR:
			line_terminator();
			txtpos[0] = NL;//Terminate all strings with a NL
			return;
		case CTRLH:
			if(txtpos == program_end)
				break;
			txtpos--;

			printmsg(backspacemsg);
			break;
		default://We need to leave at least one space to allow us to shuffle the line into order
			if(txtpos == variables_begin-2){
				outchar(BELL);
			}else{
				txtpos[0] = c;
				txtpos++;
				outchar(c);
			}
		}
	}
}

/***************************************************************************/
static u8 *findline(){
	u8 *line = program_start;
	while(1){
		if(line == program_end)
			return line;

		if(((LINENUM *)line)[0] >= linenum)
			return line;

		line += line[sizeof(LINENUM)];//Add the line length onto the current address, to get to the next line
	}
}

/***************************************************************************/
static void toUppercaseBuffer(){
	u8 *c = program_end+sizeof(LINENUM);
	u8 quote = 0;

	while(*c != NL){
		//Are we in a quoted string?
		if(*c == quote)
			quote = 0;
		else if(*c == '"' || *c == '\'')
			quote = *c;
		else if(quote == 0 && *c >= 'a' && *c <= 'z')
			*c = *c + 'A' - 'a';
		c++;
	}
}

/***************************************************************************/
void printline(){
	LINENUM line_num;

	line_num = *((LINENUM *)(list_line));
	list_line += sizeof(LINENUM) + sizeof(char);

	//Output the line
	printnum(line_num);
	outchar(' ');
	while(*list_line != NL){
		outchar(*list_line);
		list_line++;
	}
	list_line++;
	line_terminator();
}

/***************************************************************************/
static s16 expr4(){
	//fix provided by Jurg Wullschleger wullschleger@gmail.com for whitespace and unary operations
	ignore_blanks();

	if(*txtpos == '-'){
		txtpos++;
		return -expr4();
	}
	//end fix

	if(*txtpos == '0'){
		txtpos++;
		return 0;
	}

	if(*txtpos >= '1' && *txtpos <= '9'){
		s16 a = 0;
		do{
			a = a*10 + *txtpos - '0';
			txtpos++;
		} 
		while(*txtpos >= '0' && *txtpos <= '9');
		return a;
	}

	//Is it a function or variable reference?
	if(txtpos[0] >= 'A' && txtpos[0] <= 'Z'){
		s16 a;
		//Is it a variable reference (single alpha)
		if(txtpos[1] < 'A' || txtpos[1] > 'Z'){
			a = ((s16 *)variables_begin)[*txtpos - 'A'];
			txtpos++;
			return a;
		}

		//Is it a function with a single parameter
		scantable(func_tab);
		if(table_index == FUNC_UNKNOWN)
			goto EXPR4_ERROR;

		u8 f = table_index;

		if(*txtpos != '(')
			goto EXPR4_ERROR;

		txtpos++;
		a = expression();
		if(*txtpos != ')')
			goto EXPR4_ERROR;
		txtpos++;
		switch(f){
		case FUNC_PEEK:
			if(a < kRamSize){
				return program[a];
			}else{
				return SpiRamCursorRead(a);
			}
		case FUNC_ABS:
			if(a < 0) 
				return -a;
			return a;

		case FUNC_AREAD:
			pinMode(a, PM_INPUT);
			return analogRead(a);
		case FUNC_DREAD:
			pinMode(a, PM_INPUT);
			return digitalRead(a);

		case FUNC_RND:
			return(GetPrngNumber(0) % a);
		}
	}

	if(*txtpos == '('){
		txtpos++;
		s16 a = expression();
		if(*txtpos != ')')
			goto EXPR4_ERROR;

		txtpos++;
		return a;
	}

EXPR4_ERROR:
	expression_error = 1;
	return 0;
}

/***************************************************************************/
static s16 expr3(){
	s16 a = expr4();
	ignore_blanks();//fix for eg:	100 a = a + 1

	while(1){
		s16 b;
		if(*txtpos == '*'){
			txtpos++;
			b = expr4();
			a *= b;
		}else if(*txtpos == '/'){
			txtpos++;
			b = expr4();
			if(b != 0)
				a /= b;
			else
				expression_error = 1;
		}else
			return a;
	}
}

/***************************************************************************/
static s16 expr2(){
	s16 a;

	if(*txtpos == '-' || *txtpos == '+')
		a = 0;
	else
		a = expr3();

	while(1){
		s16 b;
		if(*txtpos == '-'){
			txtpos++;
			b = expr3();
			a -= b;
		}else if(*txtpos == '+'){
			txtpos++;
			b = expr3();
			a += b;
		}else
			return a;
	}
}
/***************************************************************************/
static s16 expression(){
	s16 a = expr2();
	s16 b;

	//Check if we have an error
	if(expression_error)	return a;

	scantable(relop_tab);
	if(table_index == RELOP_UNKNOWN)
		return a;

	switch(table_index){
	case RELOP_GE:
		b = expr2();
		if(a >= b) return 1;
		break;
	case RELOP_NE:
	case RELOP_NE_BANG:
		b = expr2();
		if(a != b) return 1;
		break;
	case RELOP_GT:
		b = expr2();
		if(a > b) return 1;
		break;
	case RELOP_EQ:
		b = expr2();
		if(a == b) return 1;
		break;
	case RELOP_LE:
		b = expr2();
		if(a <= b) return 1;
		break;
	case RELOP_LT:
		b = expr2();
		if(a < b) return 1;
		break;
	}
	return 0;
}

/***************************************************************************/
int main(){

	GetPrngNumber(GetTrueRandomSeed());
	if(GetPrngNumber(0) == 0)
		GetPrngNumber(0xACE);

//	InitMusicPlayer(patches);
	SetMasterVolume(224);
	DDRA |= (1<<PA6);//enable Uzenet module
	PORTD |=(1<<PD3);//reset module
	UBRR0H = 0;
	UBRR0L = 185;//9600
	UCSR0A = (1<<U2X0);//double speed
	UCSR0C = (1<<UCSZ01)+(1<<UCSZ00)+(0<<USBS0);//8N1
	UCSR0B = (1<<RXEN0)+(1<<TXEN0);//enable TX & RX

	/////Serial.println(sentinel);
	printmsg(initmsg);

	sd_initialized = 0;//try 10 times to initialize else fail
	printmsgNoNL(PSTR("Initializing SD..."));
	for(u8 i=0;i<10;i++){
		if(f_mount(0, &fs) != FR_OK){//if(f_mount(&fs, "", 1) != FR_OK){
			PORTD &= ~(1<<6);//deassert card
			WaitVsync(30);//wait
			continue;
		}else{
			printmsg(PSTR("Success!"));
			sd_initialized = 1;
			break;
		}
	}
	if(!sd_initialized){
		printmsg(PSTR("ERROR"));
	}

	outStream = kStreamScreen;
	inStream = kStreamKeyboard;
	inhibitOutput = 0;

	printmsgNoNL(PSTR("Searching for "));
	printmsgNoNL(PSTR(kAutorunFilename));
	printmsgNoNL(PSTR("..."));
	if(f_open(&f, kAutorunFilename, FA_OPEN_EXISTING|FA_READ) == FR_OK){//try to load autorun file if present
		printmsg(PSTR("Loaded"));
		program_end = program_start;
		inStream = kStreamFile;
		inhibitOutput = 1;
		runAfterLoad = 1;
	}else
		printmsg(PSTR("Not Found"));
	u8 *start;
	u8 *newEnd;
	u8 linelen;
	u8 isDigital;
	u8 alsoWait = 0;
	s16 val,val2;
	u8 var;
	char *filename;

	program_start = program;
	program_end = program_start;
	sp = program+sizeof(program);	//Needed for printnum
	stack_limit = program+sizeof(program)-STACK_SIZE;
	variables_begin = stack_limit - 27*VAR_SIZE;

	//memory free
	printnum(variables_begin-program_end);
	printmsg(memorymsg);

WARMSTART:
	//this signifies that it is running in 'direct' mode.
	current_line = 0;
	sp = program+sizeof(program);
	printmsg(okmsg);

PROMPT:
	if(triggerRun){
		triggerRun = 0;
		current_line = program_start;
		goto EXECLINE;
	}

	getln('>');
	toUppercaseBuffer();
//printmsg(PSTR("."));
	txtpos = program_end+sizeof(u16);
	while(*txtpos != NL)//find the end of the freshly entered line
		txtpos++;

	u8 *dest;//move it to the end of program_memory
	dest = variables_begin-1;
	while(1){
		*dest = *txtpos;
		if(txtpos == program_end+sizeof(u16))
			break;
		dest--;
		txtpos--;
	}
	txtpos = dest;

	linenum = testnum();//now see if we have a line number
	ignore_blanks();
	if(linenum == 0)
		goto DIRECT;

	if(linenum == 0xFFFF)
		goto QHOW;

	linelen = 0;
	while(txtpos[linelen] != NL)//find the length of what's left, including the (not yet populated) line header
		linelen++;
	linelen++;//Include the NL in the line length
	linelen += sizeof(u16)+sizeof(char);//Add space for the line number and line length

	//Now we have the number, add the line header.
	txtpos -= 3;

	*((u16 *)txtpos) = linenum;
	txtpos[sizeof(LINENUM)] = linelen;


	//Merge it into the rest of the program
	start = findline();

	//If a line with that number exists, then remove it
	if(start != program_end && *((LINENUM *)start) == linenum){
		u8 *dest, *from;
		unsigned tomove;

		from = start + start[sizeof(LINENUM)];
		dest = start;

		tomove = program_end - from;
		while(tomove > 0){
			*dest = *from;
			from++;
			dest++;
			tomove--;
		}	
		program_end = dest;
	}

	if(txtpos[sizeof(LINENUM)+sizeof(char)] == NL)//If the line has no txt, it was just a delete
		goto PROMPT;



	//Make room for the new line, either all in one hit or lots of little shuffles
	while(linelen > 0){
		u16 tomove;
		u8 *from,*dest;
		u16 space_to_make;

		space_to_make = txtpos - program_end;

		if(space_to_make > linelen)
			space_to_make = linelen;
		newEnd = program_end+space_to_make;
		tomove = program_end - start;


		//Source and destination - as these areas may overlap we need to move bottom up
		from = program_end;
		dest = newEnd;
		while(tomove > 0){
			from--;
			dest--;
			*dest = *from;
			tomove--;
		}

		//Copy over the bytes into the new space
		for(tomove = 0; tomove < space_to_make; tomove++){
			*start = *txtpos;
			txtpos++;
			start++;
			linelen--;
		}
		program_end = newEnd;
	}
	goto PROMPT;

//UNIMPLEMENTED:
//	printmsg(unimplimentedmsg);
//	goto PROMPT;

QHOW:	
	printmsg(howmsg);
	goto PROMPT;

QWHAT:	
	printmsgNoNL(whatmsg);
	if(current_line != NULL){
		u8 tmp = *txtpos;
		if(*txtpos != NL)
			*txtpos = '^';
		list_line = current_line;
		printline();
		*txtpos = tmp;
	}
	line_terminator();
	goto PROMPT;

QSORRY:	
	printmsg(sorrymsg);
	goto WARMSTART;

RUN_NEXT_STATEMENT:
	while(*txtpos == ':')
		txtpos++;
	ignore_blanks();
	if(*txtpos == NL)
		goto EXECNEXTLINE;
	goto INTERPRET_AT_TXT_POS;

DIRECT: 
	txtpos = program_end+sizeof(LINENUM);
	if(*txtpos == NL)
		goto PROMPT;

INTERPRET_AT_TXT_POS:
	if(breakcheck()){
		printmsg(breakmsg);
		goto WARMSTART;
	}

	scantable(keywords);

	switch(table_index){
	case KW_DELAY:
			expression_error = 0;
			val = expression();
			delay(val);
			goto EXECNEXTLINE;
	case KW_FILES:
		goto FILES;
	case KW_LIST:
		goto LIST;
	case KW_CHAIN:
		goto CHAIN;
	case KW_LOAD:
		goto LOAD;
	case KW_MEM:
		goto MEM;
	case KW_NEW:
		if(txtpos[0] != NL)
			goto QWHAT;
		program_end = program_start;
		goto PROMPT;
	case KW_RUN:
		current_line = program_start;
		goto EXECLINE;
	case KW_SAVE:
		goto SAVE;
	case KW_NEXT:
		goto NEXT;
	case KW_LET:
		goto ASSIGNMENT;
	case KW_IF:
		expression_error = 0;
		s16 val = expression();
		if(expression_error || *txtpos == NL)
			goto QHOW;
		if(val != 0)
			goto INTERPRET_AT_TXT_POS;
		goto EXECNEXTLINE;

	case KW_GOTO:
		expression_error = 0;
		linenum = expression();
		if(expression_error || *txtpos != NL)
			goto QHOW;
		current_line = findline();
		goto EXECLINE;

	case KW_GOSUB:
		goto GOSUB;
	case KW_RETURN:
		goto GOSUB_RETURN; 
	case KW_REM:
	case KW_QUOTE:
		goto EXECNEXTLINE;	//Ignore line completely
	case KW_FOR:
		goto FORLOOP; 
	case KW_INPUT:
		goto INPUT; 
	case KW_PRINT:
	case KW_QMARK:
		goto PRINT;
	case KW_POKE:
		goto POKE;
	case KW_END:
	case KW_STOP:
		//This is the easy way to end - set the current line to the end of program attempt to run it
		if(txtpos[0] != NL)
			goto QWHAT;
		current_line = program_end;
		goto EXECLINE;
	case KW_BYE://Leave the basic interperater
		goto WARMSTART;

	case KW_AWRITE:	//AWRITE <pin>, HIGH|LOW
		isDigital = 0;
		goto AWRITE;
	case KW_DWRITE:	//DWRITE <pin>, HIGH|LOW
		isDigital = 1;
		goto DWRITE;

	case KW_RSEED:
		goto RSEED;

	case KW_TONEW:
		alsoWait = 1;
	case KW_TONE:
		goto TONEGEN;
	case KW_NOTONE:
		goto TONESTOP;


	case KW_DEFAULT:
		goto ASSIGNMENT;
	default:
		break;
	}

EXECNEXTLINE:
	if(GetVsyncFlag()) WaitVsync(1);
	if(current_line == NULL) goto PROMPT;//Processing direct commands?
	current_line +=	 current_line[sizeof(LINENUM)];

EXECLINE:
	if(GetVsyncFlag()) WaitVsync(1);
	if(current_line == program_end) goto WARMSTART;//Out of lines to run
	txtpos = current_line+sizeof(LINENUM)+sizeof(char);
	goto INTERPRET_AT_TXT_POS;

INPUT:
		ignore_blanks();
		if(*txtpos < 'A' || *txtpos > 'Z') goto QWHAT;
		var = *txtpos;
		txtpos++;
		ignore_blanks();
		if(*txtpos != NL && *txtpos != ':') goto QWHAT;
INPUTAGAIN:
		tmptxtpos = txtpos;
		getln('?');
		toUppercaseBuffer();
		txtpos = program_end+sizeof(u16);
		ignore_blanks();
		expression_error = 0;
		val = expression();
		if(expression_error)
			goto INPUTAGAIN;
		((s16 *)variables_begin)[var-'A'] = val;
		txtpos = tmptxtpos;

		goto RUN_NEXT_STATEMENT;

FORLOOP:
		ignore_blanks();
		if(*txtpos < 'A' || *txtpos > 'Z') goto QWHAT;
		var = *txtpos;
		txtpos++;
		ignore_blanks();
		if(*txtpos != '=') goto QWHAT;
		txtpos++;
		ignore_blanks();

		expression_error = 0;
		s16 initial = expression();
		if(expression_error) goto QWHAT;

		scantable(to_tab);
		if(table_index != 0) goto QWHAT;

		s16 terminal = expression();
		if(expression_error) goto QWHAT;

		scantable(step_tab);
		s16 step;
		if(table_index == 0){
			step = expression();
			if(expression_error) goto QWHAT;
		}else{
			step = 1;
		}
		ignore_blanks();
		if(*txtpos != NL && *txtpos != ':') goto QWHAT;


		if(!expression_error && *txtpos == NL){
			struct stack_for_frame *f;
			if(sp + sizeof(struct stack_for_frame) < stack_limit) goto QSORRY;

			sp -= sizeof(struct stack_for_frame);
			f = (struct stack_for_frame *)sp;
			((s16 *)variables_begin)[var-'A'] = initial;
			f->frame_type = STACK_FOR_FLAG;
			f->for_var = var;
			f->terminal = terminal;
			f->step		 = step;
			f->txtpos	 = txtpos;
			f->current_line = current_line;
			goto RUN_NEXT_STATEMENT;
		}
	goto QHOW;

GOSUB:
	expression_error = 0;
	linenum = expression();
	if(!expression_error && *txtpos == NL){
		struct stack_gosub_frame *f;
		if(sp + sizeof(struct stack_gosub_frame) < stack_limit)
			goto QSORRY;

		sp -= sizeof(struct stack_gosub_frame);
		f = (struct stack_gosub_frame *)sp;
		f->frame_type = STACK_GOSUB_FLAG;
		f->txtpos = txtpos;
		f->current_line = current_line;
		current_line = findline();
		goto EXECLINE;
	}
	goto QHOW;

NEXT:
	ignore_blanks();//find the variable name
	if(*txtpos < 'A' || *txtpos > 'Z') goto QHOW;
	txtpos++;
	ignore_blanks();
	if(*txtpos != ':' && *txtpos != NL) goto QWHAT;

GOSUB_RETURN:
	tempsp = sp;
	while(tempsp < program+sizeof(program)-1){//walk up the stack frames and find the frame we want(if present)
		switch(tempsp[0]){
		case STACK_GOSUB_FLAG:
			if(table_index == KW_RETURN){
				struct stack_gosub_frame *f = (struct stack_gosub_frame *)tempsp;
				current_line	= f->current_line;
				txtpos			= f->txtpos;
				sp += sizeof(struct stack_gosub_frame);
				goto RUN_NEXT_STATEMENT;
			}
			//This is not the loop you are looking for... so Walk back up the stack
			tempsp += sizeof(struct stack_gosub_frame);
			break;
		case STACK_FOR_FLAG:
			//Flag, Var, Final, Step
			if(table_index == KW_NEXT){
				struct stack_for_frame *f = (struct stack_for_frame *)tempsp;
				//Is the the variable we are looking for?
				if(txtpos[-1] == f->for_var){
					s16 *varaddr = ((s16 *)variables_begin) + txtpos[-1] - 'A'; 
					*varaddr = *varaddr + f->step;
					//Use a different test depending on the sign of the step increment
					if((f->step > 0 && *varaddr <= f->terminal) || (f->step < 0 && *varaddr >= f->terminal)){
						//We have to loop so don't pop the stack
						txtpos = f->txtpos;
						current_line = f->current_line;
						goto RUN_NEXT_STATEMENT;
					}
					//We've run to the end of the loop. drop out of the loop, popping the stack
					sp = tempsp + sizeof(struct stack_for_frame);
					goto RUN_NEXT_STATEMENT;
				}
			}
			//This is not the loop you are looking for... so Walk back up the stack
			tempsp += sizeof(struct stack_for_frame);
			break;
		default:
			printmsg(PSTR("Stack is stuffed!\n"));
			goto WARMSTART;
		}
	}
	//Didn't find the variable we've been looking for
	goto QHOW;

ASSIGNMENT:
	if(*txtpos < 'A' || *txtpos > 'Z') goto QHOW;
	s16 *pvar = (s16 *)variables_begin + *txtpos - 'A';
	txtpos++;
	ignore_blanks();

	if (*txtpos != '=') goto QWHAT;
	txtpos++;
	ignore_blanks();
	expression_error = 0;
	val = expression();
	if(expression_error) goto QWHAT;
	if(*txtpos != NL && *txtpos != ':') goto QWHAT;//check that we are at the end of the statement
	*pvar = val;
	goto RUN_NEXT_STATEMENT;
POKE:
	expression_error = 0;
	val = expression();//work out where to put it
	if(expression_error) goto QWHAT;
	//u8 *address = (u8 *)val;

	ignore_blanks();//check for a comma
	if (*txtpos != ',') goto QWHAT;
	txtpos++;
	ignore_blanks();
	expression_error = 0;
	val = expression();//get the value to assign
	if(expression_error) goto QWHAT;
	//printf("Poke %p value %i\n",address, (u8)value);
	//Check that we are at the end of the statement
	if(*txtpos != NL && *txtpos != ':') goto QWHAT;
	goto RUN_NEXT_STATEMENT;

LIST:
	linenum = testnum();//Retuns 0 if no line found.

	//Should be EOL
	if(txtpos[0] != NL)
		goto QWHAT;

	//Find the line
	list_line = findline();
	while(list_line != program_end)
		printline();
	goto WARMSTART;

PRINT:
	//If we have an empty list then just put out a NL
	if(*txtpos == ':'){
		line_terminator();
		txtpos++;
		goto RUN_NEXT_STATEMENT;
	}
	if(*txtpos == NL){
		goto EXECNEXTLINE;
	}

	while(1){
		ignore_blanks();
		if(print_quoted_string()){
			;
		}else if(*txtpos == '"' || *txtpos == '\''){
			goto QWHAT;
		}else{
			s16 e;
			expression_error = 0;
			e = expression();
			if(expression_error)
				goto QWHAT;
			printnum(e);
		}

		//At this point we have three options, a comma or a new line
		if(*txtpos == ','){
			txtpos++;	//Skip the comma and move onto the next
		}else if(txtpos[0] == ';' && (txtpos[1] == NL || txtpos[1] == ':')){
			txtpos++;//This has to be the end of the print - no newline
			break;
		}else if(*txtpos == NL || *txtpos == ':'){
			line_terminator();	//The end of the print statement
			break;
		}else
			goto QWHAT;	
	}
	goto RUN_NEXT_STATEMENT;

MEM:
	//memory free
	printnum(variables_begin-program_end);
	printmsg(memorymsg);
	goto RUN_NEXT_STATEMENT;


	/*************************************************/
AWRITE://AWRITE <pin>,val
DWRITE:
	expression_error = 0;
	s16 pinNo = expression();//get the pin number
	if(expression_error) goto QWHAT;

	ignore_blanks();//check for a comma
	if (*txtpos != ',') goto QWHAT;
	txtpos++;
	ignore_blanks();

	//u8 *txtposBak = txtpos; 
	scantable(highlow_tab);
	if(table_index != HIGHLOW_UNKNOWN){
		if(table_index <= HIGHLOW_HIGH){
			val = 1;
		}else{
			val = 0;
		}
	}else{//and the value (numerical)
		expression_error = 0;
		val = expression();
		if(expression_error) goto QWHAT;
	}
	pinMode(pinNo, PM_OUTPUT);
	if(isDigital){
		digitalWrite(pinNo, val);
	}else{
		analogWrite(pinNo, val);
	}
	goto RUN_NEXT_STATEMENT;

FILES:
	cmd_Files();
	goto WARMSTART;

CHAIN:
	runAfterLoad = 1;

LOAD:
	program_end = program_start;//clear the program
	expression_error = 0;
	filename = filenameWord();//work out the filename
	if(expression_error) goto QWHAT;

	if(f_open(&f, (const char*)filename, FA_READ) == FR_OK){
		inStream = kStreamFile;//this will kickstart a series of events to read in from the file.
		inhibitOutput = 1;
	}else{
		printmsg(sdfilemsg);
	}
		
	goto WARMSTART;

SAVE:
	expression_error = 0;
	filename = filenameWord();//work out the filename
	if(expression_error) goto QWHAT;

	//open the file(overwrite if existing), switch over to file output
	if(f_open(&f, (const char *)filename, FA_WRITE) == FR_OK){//|FA_CREATE_ALWAYS
		outStream = kStreamFile;
	}else{
		printmsg(sdfilemsg);
	}

	list_line = findline();//copied from "List"
	while(list_line != program_end)
		printline();

	outStream = kStreamScreen;//go back to standard output, close the file
	f_close(&f);
	goto WARMSTART;

RSEED:
	expression_error = 0;
	val = expression();//get pin number
	if(expression_error) goto QWHAT;

	GetPrngNumber(val);
	goto RUN_NEXT_STATEMENT;

TONESTOP:
	noTone();
	goto RUN_NEXT_STATEMENT;

TONEGEN://TONE freq, duration
	expression_error = 0;
	val = expression();//get the frequency(if 0, turn off tone)
	if(expression_error) goto QWHAT;
	if(val == 0) goto TONESTOP;
	ignore_blanks();
	if (*txtpos != ',') goto QWHAT;
	txtpos++;
	ignore_blanks();
	expression_error = 0;
	val2 = expression();//get the duration(if 0, turn off tone0
	if(expression_error) goto QWHAT;
	if(val2 == 0) goto TONESTOP;

	tone(val, val2);//frequency, duration
	if(alsoWait){
		delay(val2);
		alsoWait = 0;
	}

	goto RUN_NEXT_STATEMENT;
	return 0;
}

//returns 1 if the character is valid in a filename
static s16 isValidFnChar(char c){
	if(c >= '0' && c <= '9') return 1;//number
	if(c >= 'A' && c <= 'Z') return 1;//LETTER
	if(c >= 'a' && c <= 'z') return 1;//letter (for completeness)
	if(c == '_') return 1;
	if(c == '+') return 1;
	if(c == '.') return 1;
	if(c == '~') return 1;	//Window~1.txt

	return 0;
}

char *filenameWord(){
	//SDL - I wasn't sure if this functionality existed above, so I figured i'd put it here
	u8 * ret = txtpos;
	expression_error = 0;

	//make sure there are no quotes or spaces, search for valid characters
	//while(*txtpos == SPACE || *txtpos == TAB || *txtpos == SQUOTE || *txtpos == DQUOTE) txtpos++;
	while(!isValidFnChar(*txtpos)) txtpos++;
	ret = txtpos;

	if(*ret == '\0'){
		expression_error = 1;
		return (char *)ret;
	}

	//now, find the next nonfnchar
	txtpos++;
	while(isValidFnChar(*txtpos)) txtpos++;
	if(txtpos != ret) *txtpos = '\0';

	//set the error code if we've got no string
	if(*ret == '\0'){
		expression_error = 1;
	}

	return (char *)ret;
}

/***************************************************************************/
static void line_terminator(){
	outchar(NL);
	outchar(CR);
}

/***********************************************************/
static char breakcheck(){
	if(UartUnreadCount())
		return UartReadChar() == CTRLC;
	return 0;
}
/***********************************************************/
static s16 inchar(){
	s16 v;
	switch(inStream){
	case(kStreamKeyboard):
		do{
			if(GetVsyncFlag()) WaitVsync(1);
			v = GetKeyboardChar(KB_SEND_END);
		}while(!v);
		return v;
		break;
	case(kStreamFile):
		if(GetVsyncFlag()) WaitVsync(1);
		f_read(&f, &v, 1, &bytesRead);
		if(bytesRead != 1){
			f_close(&f);
			goto INCHAR_LOADFINISH;
		}
		if(v == NL) v=CR;//file translate

		return v;		
		break;
	 case(kStreamSerial):
	default:
		while(1){
			if(GetVsyncFlag()) WaitVsync(1);
			if(UartUnreadCount())
				return UartReadChar();
		}
	}
	
INCHAR_LOADFINISH:
	inStream = kStreamKeyboard;
	inhibitOutput = 0;

	if(runAfterLoad){
		runAfterLoad = 0;
		triggerRun = 1;
	}
	return NL;//trigger a prompt.
}

/***********************************************************/
static void outchar(char c){
	if(inhibitOutput) return;

	if(outStream == kStreamScreen){
		ConsolePrintChar(c);
	}else if(outStream == kStreamFile){
		f_write(&f, &c, 1, &bytesWritten);
	}else{
		while(IsUartTxBufferFull());
		UartSendChar(c);
	}
}

void cmd_Files(){
	DIR d;
	if(f_opendir(&d, "/") != FR_OK)
		return;
	FILINFO entry;

	while(1){
		if(GetVsyncFlag()) WaitVsync(1);
		if(f_readdir(&d, &entry) != FR_OK || entry.fname[0] == 0)
			break;
		//common header
		printmsgNoNL(indentmsg);
		printmsgNoNL((const char *)entry.fname);
		if(entry.fattrib & AM_DIR){
			printmsgNoNL(slashmsg);
			u8 found_end = 0;
			for(u8 i=0; i<13 ; i++){
				if(entry.fname[i] == '\0')
					found_end = 1;
				if(found_end)
					printmsgNoNL(spacemsg);
			}
			printmsgNoNL(dirextmsg);
		}else{//file ending
			u8 found_end = 0;
			for(u8 i=0; i<13 ; i++){
				if(entry.fname[i] == '\0')
					found_end = 1;
				if(found_end)
					printmsgNoNL(spacemsg);
			}
			printUnum(entry.fsize);
		}
		line_terminator();
	}
	f_close(&f);
}