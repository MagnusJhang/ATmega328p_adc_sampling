# Hardware requirements
1. ATmega328p
2. BLE Module: HM-10, HM-19, etc. (Baudrate: 38400 bps)

# Pin Map Connection
1. ATMega328p.D6 <--> BLE Module.RX
2. ATMega328p.D5 <--> BLE Module.TX
3. ATMega328p.A0 <--> Analog Device (eg. ECG Sensor, Potentialmetter, etc.)

# UUID:
1. The HM-10, HM-19 device 


# Supported Commands
There are two commands that could be sent after connecting the BLE module.
1. Starting Sampling: 0x5A ( Please send the command from peripheral device after connected)
2. Stoping Sampling: 0x55 ( Please send the command from peripheral device before disconnected)

# Specification of Sampling
1. Sampling rate: 360Hz
2. ADC resolution: 10 bit

# Configuration of Sampling
* If your ATmega328 is working under 8 MHz, please define the FLAG  **#define FCLK_8M**. Otherwise, the firmware would achieve under the 16 MHz fclk.
* IF you would like to sampling under the [ADC Noise Reduction Mode](https://microchipdeveloper.com/8avr:adcnoisereduce), please enable the flag by **#define ADC_NOISE_REDUCTION**

# Data package
* The 10 bit sampling value (a0) is separated into high byte (HB) and low byte (LB) by following rules. 

``` C
uint16_t a0 = analogRead(A0);
char package[] = {0x80 | (a0>>6), 0x40 | (a0 & 0x003f)};
BT.write(package, 2);
```

| HB[7] | HB[6] | HB[5] | HB[4] | HB[3] | HB[2] | HB[1] | HB[0] | LB[7] | LB[6] | LB[5] | LB[4] | LB[3] | LB[2] | LB[1] | LB[0]
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 0 | N/A | a0[9] | a0[8] | a0[7] |a0[6] |a0[5] | 0 | 1 | N/A | a0[4] | a0[3] |a0[2] |a0[1] |a0[0] |


