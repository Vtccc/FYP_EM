void Initial_ST7305(void)
{
	IOSET = nRESB;	
	Delay(100);
	IOCLR = nRESB;
	Delay(10000);	
	IOSET = nRESB;	
	Delay(10000);


// 7305
Write_Register(0xD6); //NVM Load Control 
	Write_Parameter(0X17); 
	Write_Parameter(0X02);
	 
	Write_Register(0xD1); //Booster Enable 
	Write_Parameter(0X01); 

	Write_Register(0xC0); //Gate Voltage Setting 
	Write_Parameter(0X08); //VGH 00:8V  04:10V  08:12V   0E:15V
	Write_Parameter(0X06); //VGL 00:-5V   04:-7V   0A:-10V 


// VLC=3.6V (12/-5)(delta Vp=0.6V)		
	Write_Register(0xC1); //VSHP Setting (4.8V)	
	Write_Parameter(0X3C); //VSHP1 	
	Write_Parameter(0X3C); //VSHP2 	
	Write_Parameter(0X3C); //VSHP3 	
	Write_Parameter(0X3C); //VSHP4	
		
	Write_Register(0xC2); //VSLP Setting (0.98V)	
	Write_Parameter(0X23); //VSLP1 	
	Write_Parameter(0X23); //VSLP2 	
	Write_Parameter(0X23); //VSLP3 	
	Write_Parameter(0X23); //VSLP4 	
		
	Write_Register(0xC4); //VSHN Setting (-3.6V)	
	Write_Parameter(0X5A); //VSHN1	
	Write_Parameter(0X5A); //VSHN2 	
	Write_Parameter(0X5A); //VSHN3 	
	Write_Parameter(0X5A); //VSHN4 	
		
	Write_Register(0xC5); //VSLN Setting (0.22V)	
	Write_Parameter(0X37); //VSLN1 	
	Write_Parameter(0X37); //VSLN2 	
	Write_Parameter(0X37); //VSLN3 	
	Write_Parameter(0X37); //VSLN4

	Write_Register(0xD8); //HPM=32Hz
	Write_Parameter(0XA6); //~51Hz
	Write_Parameter(0XE9); //~1Hz

/*-- HPM=32hz ; LPM=> 0x15=8Hz 0x14=4Hz 0x13=2Hz 0x12=1Hz 0x11=0.5Hz 0x10=0.25Hz---*/
	Write_Register(0xB2); //Frame Rate Control 
	Write_Parameter(0X12); //HPM=32hz ; LPM=1hz 

	Write_Register(0xB3); //Update Period Gate EQ Control in HPM 
	Write_Parameter(0XE5); 
	Write_Parameter(0XF6); 
	Write_Parameter(0X05); //HPM EQ Control 
	Write_Parameter(0X46); 
	Write_Parameter(0X77); 
	Write_Parameter(0X77); 
	Write_Parameter(0X77); 
	Write_Parameter(0X77); 
	Write_Parameter(0X76); 
	Write_Parameter(0X45); 

	Write_Register(0xB4); //Update Period Gate EQ Control in LPM 
	Write_Parameter(0X05); //LPM EQ Control 
	Write_Parameter(0X46); 
	Write_Parameter(0X77); 
	Write_Parameter(0X77); 
	Write_Parameter(0X77); 
	Write_Parameter(0X77); 
	Write_Parameter(0X76); 
	Write_Parameter(0X45); 

	Write_Register(0x62); //Gate Timing Control
	Write_Parameter(0X32);
	Write_Parameter(0X03);
	Write_Parameter(0X1F);

	Write_Register(0xB7); //Source EQ Enable 
	Write_Parameter(0X13); 

	Write_Register(0xB0); //Gate Line Setting 
	Write_Parameter(0X60); //384 line 

	Write_Register(0x11); //Sleep out 
	Delay(255); 

	Write_Register(0xC9); //Source Voltage Select  
	Write_Parameter(0X00); //VSHP1; VSLP1 ; VSHN1 ; VSLN1
	
	Write_Register(0x36); //Memory Data Access Control
	Write_Parameter(0X48); //MX=1 ; DO=1 

	Write_Register(0x3A); //Data Format Select 
	Write_Parameter(0X10); //10:4write for 24bit ; 11: 3write for 24bit

	Write_Register(0xB9); //Gamma Mode Setting 
	Write_Parameter(0X20); //20: Mono 00:4GS 

	Write_Register(0xB8); //Panel Setting 
	Write_Parameter(0X09); // Panel Setting Frame inversion  09:column 29:dot_1-Frame 25:dot_1-Line

	//WRITE RAM 168*384
	Write_Register(0x2A); //Column Address Setting 
	Write_Parameter(0X17); 
	Write_Parameter(0X24); 

	Write_Register(0x2B); //Row Address Setting 
	Write_Parameter(0X00); 
	Write_Parameter(0XBF); 
/*
	Write_Register(0x72); //de-stress off 
	Write_Parameter(0X13);
*/
	Write_Register(0x35); //TE
	Write_Parameter(0X00); //

	Write_Register(0xD0); //Auto power dowb
	Write_Parameter(0XFF); //


	Write_Register(0x39); //LPM


	Write_Register(0x29); //DISPLAY ON  
  
}	

/*====================================================*/
void address(void)
{             
     Write_Register(0x2A);////Column Address Setting S61~S182
     Write_Parameter(0x17);
     Write_Parameter(0x24);    

     Write_Register(0x2B);////Row Address Setting G1~G250
     Write_Parameter(0x00);
     Write_Parameter(0xBF); 
    
    Write_Register(0x2C);   //write image data
 }	
