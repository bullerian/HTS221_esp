#ifndef __HTS221_H
#define __HTS221_H

//#define DECIMAL_DIGITS 2

#define HTS221_addr	 0xBF
#define HTS221SR	 0x27
#define HTS221tOut	 0x2A
#define HTS221hOut	 0x28
#define HTS221CTRL1	 0x20
#define HTS221CTRL2	 0x21
#define HTS221CTRL3	 0x22
#define HTS221aVconf 0x10

#define HTS221CTRL1_PD			0b10000000 // PD: power down control (0: power-down mode; 1: active mode)
#define HTS221CTRL1_BDU 		0b00000100 // BDU: block data update (0: continuous update; 1: output registers not updated until MSB and LSB reading)

#define HTS221CTRL2_HEATER 		0b00000010 //  Heater (0: heater disable; 1: heater enable)
#define HTS221CTRL2_ONE_SHOT 	0b00000001 //  One shot enable (0: waiting for start of conversion; 1: start for a new dataset)

#define HTS221CTRL3_DRDY_EN 	0b00000100 // DRDY_EN: Data Ready enable (0: Data Ready disabled - default;1: Data Ready signal available on pin 3)
#define HTS221CTRL3_DRDY_H_L 	0b10000000 //  DRDY_H_L: Data Ready output signal active high, low (0: active high -default;1: active low)
#define HTS221CTRL3_PP_OD   	0b01000000 //  PP_OD: Push-pull / Open Drain selection on pin 3 (DRDY) (0: push-pull - default; 1: open drain)

#define HTS221SR_H_DA 			0b00000010 //H_DA: Humidity data available. (0: new data for Humidity is not yet available; 1: new data for Humidity is available)
#define HTS221SR_T_DA			0b00000001 //T_DA
#define HTS221SR_TH_DA			0b00000011

#define hts221_busy				2
#define hts221_ok				0
#define hts221_error			1

#define pow 100 //pow(10,DECIMAL_DIGITS)

unsigned char HTS221_init();
uint8_t HTS221startConv();
uint8_t HTS221dr();
void HTS221getTH();

#endif
