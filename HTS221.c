#include "HTS221.h"

//#define HTS_first_time

#ifdef HTS_first_time
	int H0_T0_OUT,  H1_T0_OUT,  T0_OUT,  T1_OUT, temperature_t;
	float T0_degC, T1_degC, H0_rh, H1_rh;
	unsigned int humidity_t;
#else
	const int H0_T0_OUT=2; const int H1_T0_OUT=-11056; const int  T0_OUT=-5; const int T1_OUT=674;
	const float T0_degC=20.625; const float T1_degC=32.5; const float H0_rh=29; const float H1_rh=71;
	const float Ta=11.875; const uint16_t Tb=679;
	const uint8_t Ha=42; const int16_t Hb=-11058;
	int8_t HTS_T, HTS_H;
#endif

unsigned char HTS221_init(){
	unsigned char i;

	I2CInit(); //I2C init
	i=I2C_ReadByte(HTS221_addr, HTS221CTRL1);
	if (!(i & HTS221CTRL1_PD)){
		I2C_SendByte(HTS221_addr,HTS221CTRL1,(i | HTS221CTRL1_PD | HTS221CTRL1_BDU)); // BDU=1, PD=1
	}

#ifdef HTS_first_time
	unsigned char T1T0;
	T1T0=I2C_ReadByte(HTS221_addr, 0x35);
	T0_degC=((T1T0 & 0x03)<<8)+I2C_ReadByte(HTS221_addr, 0x32);
	T0_degC /=8;
	T1_degC=((T1T0 & 0x0C)<<6)+I2C_ReadByte(HTS221_addr, 0x33);
	T1_degC /=8;

	T0_OUT=(I2C_ReadByte(HTS221_addr,0x3D)<<8)+I2C_ReadByte(HTS221_addr,0x3C);
	T1_OUT=(I2C_ReadByte(HTS221_addr,0x3F)<<8)+I2C_ReadByte(HTS221_addr,0x3E);

	H0_rh=((float)I2C_ReadByte(HTS221_addr, 0x30))/2;
	H1_rh=((float)I2C_ReadByte(HTS221_addr, 0x31))/2;

	H0_T0_OUT=(I2C_ReadByte(HTS221_addr,0x37)<<8)+I2C_ReadByte(HTS221_addr,0x36);
	H1_T0_OUT=(I2C_ReadByte(HTS221_addr,0x3B)<<8)+I2C_ReadByte(HTS221_addr,0x3A);

    if (H0_T0_OUT & 0x8000) H0_T0_OUT = -(0x8000-(0x7fff & H0_T0_OUT));
    if (H1_T0_OUT & 0x8000) H1_T0_OUT = -(0x8000-(0x7fff & H1_T0_OUT));       //((~H1_T0_OUT)+1);//
    if (T0_OUT & 0x8000) T0_OUT = -(0x8000-(0x7fff & T0_OUT));
    if (T1_OUT & 0x8000) T1_OUT = -(0x8000-(0x7fff & T1_OUT));
#endif

	return hts221_ok;
}


uint8_t HTS221startConv(){
	uint8_t tmp_byte;
	tmp_byte=I2C_ReadByte(HTS221_addr, HTS221CTRL2); // get hts221ctrl2 reg
	if (tmp_byte & HTS221CTRL2_ONE_SHOT) return hts221_busy; //if ONE_SHOT bit is set return BUSY
	I2C_SendByte(HTS221_addr, HTS221CTRL2, tmp_byte | HTS221CTRL2_ONE_SHOT);// else start convention
	return hts221_ok;
}


uint8_t HTS221dr(){  //data ready= return 1
	return (I2C_ReadByte(HTS221_addr, HTS221SR) & HTS221SR_H_DA); //check if hum data is available, return 2 if so
}


/*****************************
/Output: Two float values of temp and hum
/
********************************/
void HTS221getTH(){
	int H_raw, T_raw;
	//TODO: DONT USE READWORD
	H_raw=(I2C_ReadByte(HTS221_addr,0x29)<<8)+I2C_ReadByte(HTS221_addr, 0x28);
	T_raw=(I2C_ReadByte(HTS221_addr,0x2B)<<8)+I2C_ReadByte(HTS221_addr, 0x2A);

    if (H_raw&0x8000) H_raw = -(0x8000-(0x7fff&H_raw));   //((~H_OUT)+1);;
    if (T_raw&0x8000) T_raw  = -(0x8000-(0x7fff&T_raw));

#ifndef HTS_first_time
    HTS_T = ((T_raw - T0_OUT))*Ta/Tb+T0_degC; //T_degC = (float)((T_raw - T0_OUT))*(float)(T1_degC-T0_degC)/(T1_OUT-T0_OUT)+T0_degC;  old eqv

    //  T_raw = (float)T_degC * pow;
//	T_degC =(int)T_raw/pow;

	HTS_H = ((H_raw - H0_T0_OUT))*Ha/Hb+H0_rh;
	if (HTS_H>100) HTS_H=100;
//	H_raw = (unsigned int)(H_rh * pow);
//	H_rh=(float)(humidity_t/pow);
#endif
}
