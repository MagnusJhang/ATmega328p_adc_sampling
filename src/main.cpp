#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TimerOne.h>
#define M_DEBUG

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
ISR(ADC_VECT){
	samp1_buff[wr_pos] = analogRead(A0);
	wr_pos = !(wr_pos == LAST_BUFF_IDX) ? wr_pos + 1 : 0;
	cnt++;
}

void sampling(){
	
	ADCSRA |= 1 << ADSC;

	
}

void setup() {
#ifdef M_DEBUG
	Serial.begin(115200);
#endif
	BT.begin(BAUD_RATE);
	pinMode(13, OUTPUT);
	pinMode(A0, INPUT);
	digitalWrite(13, HIGH);
	delay(500);
	digitalWrite(13, LOW);
	
	ADCSRA |= 1 << ADEN;
	ADCSRA |= 1 << ADIE;
	//ADC_ENABLE True
	ADCSRA |= 1 << ADPS0;
	ADCSRA |= 1 << ADPS1; 
	//prescalar: 16
	ADMUX &= 0xf0; //ADC0
	ADMUX |= 1 << ADLAR;
	
	//HIGH BYTE FIRST

	Timer1.initialize(period_us);
	Timer1.attachInterrupt(sampling); 
	Timer1.stop(); 
  	rd_pos = 0;
    wr_pos = 0;
	
}

void loop() {
	if (BT.available()) {
    	char v = BT.read();
		if (v == BLE_WR_EN) {
			rd_pos = 0;
    		wr_pos = 0;
			cnt = 0;
			Timer1.start();
			sample_en = true;
			digitalWrite(13, HIGH);
			st_time = millis();
		} else if (v == BLE_WR_DIS) {
			Timer1.stop();
			sample_en = false;
			digitalWrite(13, LOW);
			Serial.print(F("Done: "));
			Serial.print((millis() - st_time)/1000.0);
			Serial.print(", act:");
			Serial.println(((double)cnt*period_us)/1000000);
		}
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