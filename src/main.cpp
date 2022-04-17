#include <Arduino.h>
#include <SoftwareSerial.h>
#define FCLK_8M
// #define M_DEBUG

#ifndef FASTPWM
    #ifdef FCLK_8M //Set the sampling rate as 360Hz on fclk=8MHz
    #define C_TCNT1H 169
    #define C_TCNT1L 48
    #else //Set the sampling rate as 360Hz on fclk=16MHz
    #define C_TCNT1H 82
    #define C_TCNT1L 98
    #endif
#else
    #define C_OCR1AH 173
    #define C_OCR1AL 158
#endif

//

#define BT_RX 6
#define BT_TX 5
#define BAUD_RATE 38400
#define BLE_WR_EN 0x5A
#define BLE_WR_DIS 0x55
#define BUFF_SIZE 64
#define FS 360

float st_time = 0.0;
SoftwareSerial BT(BT_RX, BT_TX);

const uint16_t LAST_BUFF_IDX = BUFF_SIZE - 1;
uint32_t cnt;
int period_us = (1.0/FS)*1000000;
uint16_t samp1_buff[BUFF_SIZE];
uint16_t rd_pos = 0;
uint16_t wr_pos = 0;

bool sample_en = false;

inline void send(int samp_idx){
	char a[] = {0x80 | (samp1_buff[samp_idx]>>6), 0x40 | (samp1_buff[samp_idx] & 0x003f)};
	BT.write(a, 2);
#ifdef M_DEBUG
	Serial.println(samp1_buff[samp_idx]);
#endif
}

ISR(TIMER1_OVF_vect) {
  TCNT1H = C_TCNT1H;
  TCNT1L = C_TCNT1L;
}

ISR(ADC_vect){
    samp1_buff[wr_pos] = ADCL | (ADCH << 8);
    wr_pos = !(wr_pos == LAST_BUFF_IDX) ? wr_pos + 1 : 0;
    cnt++;
}

void init_timer1(){
	TIMSK1 |= 1 << TOIE1;	
  	TCCR1A = 0X00;
//FAST PWM MODE
//   TCCR1A |= (1 << WGM11) | (1 << WGM10);
//   TCCR1B |= (1 << WGM13) | (1 << WGM12);
//   OCR1AH = C_OCR1AH;
//   OCR1AL = C_OCR1AL;
}

void init_adc0(){
	ADCSRA |=  (1 << ADPS2) | (1 << ADPS0);
	ADCSRA &=  ~(1 << ADPS1);  // 0
	ADCSRB |= (1 << ADTS2) | (1 << ADTS1); //TIMER/COUNTER1 overflow
	ADCSRA |= 1 << ADATE; //AUTO TRIGGER ENABLE
	ADMUX  = 0b01000000;      // C2:: Use AVCC and A0 as input to ADC
}

void samp_ctl(bool en){
	if(en){
		// TCNT1H = C_TCNT1H;
  		// TCNT1L = C_TCHT1L;
  		ADCSRA |= 1 << ADEN;
  		ADCSRA |= 1 << ADIE;
		TCCR1B = 0X01;
		sei();
	}else{
		ADCSRA &= ~(1 << ADEN);
		ADCSRA &= ~(1 << ADIE);
  		TCCR1B = 0X00;
	}
}

void setup() {
	Serial.begin(115200);
	BT.begin(BAUD_RATE);

	pinMode(13, OUTPUT);
	pinMode(A0, INPUT);
	digitalWrite(13, HIGH);
	init_adc0();
	init_timer1();
	delay(500);
	digitalWrite(13, LOW);
	
  	rd_pos = 0;
    wr_pos = 0;
	Serial.println("setup");
}

void loop() {
	if (BT.available()) {
    	char v = BT.read();
		if (v == BLE_WR_EN) {
			rd_pos = 0;
    		wr_pos = 0;
			cnt = 0;
			sample_en = true;
			digitalWrite(13, HIGH);
			st_time = millis();
			Serial.println(F("START"));
		} else if (v == BLE_WR_DIS) {
			sample_en = false;
			digitalWrite(13, LOW);
			Serial.print(F("Done: "));
			Serial.print((millis() - st_time)/1000.0);
			Serial.print(F(", act:"));
			Serial.println(((double)cnt*period_us)/1000000);
		}
		samp_ctl(sample_en);
  	}
	delay(50);
  	if(sample_en){
		uint16_t t_wr_pos = wr_pos;
		if(rd_pos < t_wr_pos){
			for(; rd_pos < t_wr_pos; rd_pos++)
				send(rd_pos);

		}else if(rd_pos > t_wr_pos){
			for(; rd_pos <= LAST_BUFF_IDX; rd_pos++)
				send(rd_pos);

			for(rd_pos=0; rd_pos < t_wr_pos; rd_pos++)
				send(rd_pos);
		}
	}
}