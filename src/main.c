/***

Field Programmable Robot ver 0.1 alpha
Copyright (C) 2018 Yuri213212
Site:https://github.com/Yuri213212/FieldProgrammableRobot
Email: yuri213212@vip.qq.com
License: CC BY-NC-SA 4.0
https://creativecommons.org/licenses/by-nc-sa/4.0/

***/

#pragma arm section zidata="non_initialized"

#include "LPC1100L.h"
#include "ioport.h"
#include "buzzer.h"
#include "i2c.h"
#include "oledchar.h"

#define wallclock (int16_t)TMR16B0TC

#define buttonwait 375		//10ms
#define i2cresetwait 375	//10ms

#define exestartwait 0x1E00	//3s
#define exeerrorwait 0x1400	//2s
#define exefdbkwait 0x1420	//2s
#define exeltrtwait 0x0920	//0.9s

uint16_t state,movewait;
int movebuffer;
uint8_t count;
uint16_t opcommand[16];
uint8_t command[16];

int16_t buttonwaitto;
uint8_t buttonstate,buttonsample;

uint8_t phase,duty;
uint8_t const *patternp;

int8_t i2cbusy,i2ciorefresh,i2coledrefresh,i2coledmax,i2coledi,i2coledfont,i2ccolorrefersh;
uint16_t i2ciooutdata;
uint8_t i2cioout[7],i2coledoutbuf[129],i2coledout[21],i2ccolorinbuf[1];
struct i2ctransaction i2ccontrol[3],*i2ccp;

//0x40,0x00,2,0x14,<0x55>,<0xAA>,0x10
__inline void ioout(uint16_t data){
	i2cioout[4]=data&0xFF;
	i2cioout[5]=data>>8;
}

__inline uint8_t toHex(register uint8_t x){
	return x<10?x+'0':x+55;
}

void str2buf(uint8_t *buf,char const *s){
	register int i;

	for (i=0;s[i];++i){
		buf[i]=(uint8_t)s[i];
	}
	buf[i]=0;
}

//0x78,0x00,6,0x80,<0x02>,0x80,<0x10>,0x80,<0xB0>,0x40,0x00,7,<0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77>,0x10	//0xE3=NOP
void oledprintdata(int8_t pos,uint8_t const *data){
	register int i;

	if (pos>=0){
		i2coledout[4]=((pos<<3)&0x08)+2;
		i2coledout[6]=((pos>>1)&0x07)|0x10;
		i2coledout[8]=((pos>>4)&0x07)|0xB0;
	}else{
		i2coledout[4]=0xE3;
		i2coledout[6]=0xE3;
		i2coledout[8]=0xE3;
	}
	for (i=0;i<8;++i){
		i2coledout[i+12]=data[i];
	}
}

//0x78,0x00,6,0x80,<0x02>,0x80,<0x10>,0x80,<0xB0>,0x40,0x00,7,<0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77>,0x10	//0xE3=NOP
void oledprinthex(int8_t pos,uint8_t c){
	register int i;

	if (pos>=0){
		i2coledout[4]=((pos<<3)&0x08)+2;
		i2coledout[6]=((pos>>1)&0x07)|0x10;
		i2coledout[8]=((pos>>4)&0x07)|0xB0;
	}else{
		i2coledout[4]=0xE3;
		i2coledout[6]=0xE3;
		i2coledout[8]=0xE3;
	}
	for (i=0;i<4;++i){
		i2coledout[i+12]=oledhex8[(c>>4<<2)|i];
	}
	for (i=0;i<4;++i){
		i2coledout[i+16]=oledhex8[((c&0x0F)<<2)|64|i];
	}
}

void Reset_Handler(){
//clock settings
	SYSPLLCTRL=0x23;			//set PLL to 48MHz(12MHz*4)
	PDRUNCFG&=~(0x80);			//system PLL power-on
	while (!(SYSPLLSTAT&0x01));	//wait until PLL lock
	MAINCLKSEL=0x03;			//select main clock source to PLL output
	MAINCLKUEN=0x01;
	MAINCLKUEN=0x00;
	MAINCLKUEN=0x01;			//update main clock source
	SYSAHBCLKDIV=0x01;			//set system clock to 48MHz(48MHz/1)
	SYSAHBCLKCTRL=0x187FF;		//enable clock for peripherals (using GPIO,WDT,timers,I2C)
	WDTCLKSEL=0x01;				//select WDT clock source to main clock
	WDTCLKUEN=0x01;
	WDTCLKUEN=0x00;
	WDTCLKUEN=0x01;				//update WDT clock source
	WDTCLKDIV=0x01;				//set WDT clock to 12MHz(48MHz/1/4)
	PRESETCTRL=0x02;			//clear reset for I2C
//IO port settings
	//IOCON_RESET_PIO0_0=0xD0;	//pulled up input for reset
	IOCON_PIO0_1=0xF0;			//pulled up input for button 1
	//IOCON_PIO0_2=0xD0;		//reserved
	IOCON_PIO0_3=0xF0;;			//pulled up input for signal from color sensor
	IOCON_PIO0_4=0x201;			//open drain io for I2C_SCL
	IOCON_PIO0_5=0x201;			//open drain io for I2C_SDA
	IOCON_PIO0_6=0x4C0;			//open drain output for LEDr
	IOCON_PIO0_7=0xC0;			//standard output for buzzer
	IOCON_PIO0_8=0x4C0;			//open drain output for motor 1n
	IOCON_PIO0_9=0x4C0;			//open drain output for motor 1p
	IOCON_SWCLK_PIO0_10=0x4C1;	//open drain output for motor 2n
	IOCON_R_PIO0_11=0x4C1;		//open drain output for motor 2p
	//IOCON_R_PIO1_0=0xD0;		//reserved
	//IOCON_R_PIO1_1=0xD0;		//reserved
	//IOCON_R_PIO1_2=0xD0;		//reserved
	//IOCON_SWDIO_PIO1_3=0xD0;	//reserved
	//IOCON_PIO1_4=0xD0;		//reserved
	//IOCON_PIO1_5=0xD0;		//reserved
	IOCON_PIO1_6=0xD1;			//pulled up input for UART_RXD
	IOCON_PIO1_7=0xC1;			//standard output for UART_TXD
	//IOCON_PIO1_8=0xD0;		//reserved
	//IOCON_PIO1_9=0xD0;		//reserved
	GPIO0DIR=0xFD0;
	GPIO1DIR=0x080;				//set IO direction
	GPIO0DATA[0xFD0]=0xF50;
	GPIO1DATA[0x080]=0x080;		//init output
//previous error handling
	if (SYSRSTSTAT&0x04){		//reset by WDT
		GPIO0DATA[P0_LEDr]=0;	//red LED on
		for (;;);				//halt
	}
//WDT settings
	WDTC=1199999;				//time-out after 0.1s
	WDMOD=0x3;					//enanble WDT (start on first feed)
//wall clock settings
	TMR16B0PR=1279;				//set period 1280(37500Hz)
	TMR16B0TCR=0x1;				//start counter
//interrupt settings
	ISER=0x000F8000;			//enable interrupts (for timers,I2C)
	IPR3=0xC0000000;			//set priority (I2C=-3)
	IPR4=0x00800000;			//set priority (T3=-2,T2=0)
	SHPR3=0x40000000;			//set priority (T0=-1)
//timer settings
	TMR16B1MR3=1;				//avoid instant interrupt
	TMR16B1MCR=0x600;			//set interrupt and reset on match #3
	TMR32B0MR3=4799999;			//set period 4.8M(10Hz)
	TMR32B0MCR=0x600;			//set interrupt and reset on match #3
//I2C settings
	I2C0SCLH=45;
	I2C0SCLL=75;				//set SCL to 400KHz(48MHz/(45+75))
	//I2C0CONCLR=0x6C;			//reset state
	I2C0CONSET=0x40;			//enable I2C

/*** idle process start ***/

	state=0;
	movewait=0x0A00;
	movebuffer=0xFF0000;
	count=0;

	buttonwaitto=0;
	buttonstate=0;

	i2cbusy=-1;
	i2coledrefresh=2;
	i2ciorefresh=1;
	i2ccolorrefersh=0;
	i2cioout[0]=0x40;
	i2cioout[1]=0x00;
	i2cioout[2]=2;
	i2cioout[3]=0x14;
	i2cioout[6]=0x10;
	i2ciooutdata=0xFFFF;
	i2coledout[0]=0x78;
	i2coledout[1]=0x00;
	i2coledout[2]=6;
	i2coledout[3]=0x80;
	i2coledout[5]=0x80;
	i2coledout[7]=0x80;
	i2coledout[9]=0x40;
	i2coledout[10]=0x00;
	i2coledout[11]=7;
	i2coledout[20]=0x10;
	str2buf(i2coledoutbuf,"state=0x        command count=                                  ");
	i2coledmax=-1;
	i2coledi=0;
	i2coledfont=2;
	i2ccontrol[0].state=0x81;
	i2ccontrol[0].cmd=i2cioinit;
	i2ccontrol[0].next=&i2ccontrol[1];
	i2ccontrol[1].state=0x81;
	i2ccontrol[1].cmd=i2coledinit;
	i2ccontrol[1].next=&i2ccontrol[2];
	i2ccontrol[2].state=0x00;
	i2ccontrol[2].cmd=i2ccolorin;
	i2ccontrol[2].in=i2ccolorinbuf;
	i2ccontrol[2].next=&i2ccontrol[0];
	i2ccp=&i2ccontrol[2];

	for (;;){//main loop
		register int temp;

		WDFEED=0xAA;
		WDFEED=0x55;			//feed watch dog

		switch (i2ccontrol[0].state){
		case 0x01:
			i2ccontrol[0].cmd=i2cioout;
			i2ccontrol[0].state=0x00;
			break;
		case 0x00:
			if (i2ciorefresh){
				ioout(i2ciooutdata);
				i2ccontrol[0].state=0x80;
				i2ciorefresh=0;
			}
		}

		switch (i2ccontrol[1].state){
		case 0x01:
			i2ccontrol[1].cmd=i2coledout;
			i2ccontrol[1].state=0x00;
		case 0x00:
			if (i2coledi>=0){
				if (i2coledmax>=0&&i2coledmax<i2coledi){
					i2coledi=-1;
					break;
				}
				if (i2coledfont&0x2){//0=half height,2=fullheight
					temp=i2coledoutbuf[i2coledi>>1];
				}else{
					temp=i2coledoutbuf[i2coledi];
				}
				if (!temp&&i2coledmax<0){
					i2coledi=-1;
					break;
				}
				switch (i2coledfont){//0=char,1=hex
				case 0:
					oledprintdata(i2coledi,&oledchar8[temp<<3]);
					break;
				case 1:
					oledprinthex(i2coledi,temp);
					break;
				case 2:
					oledprintdata((i2coledi&~0x1F)+((i2coledi&1)<<4)+((i2coledi>>1)&0x0F),&oledchar16[i2coledi&0x01][temp<<3]);
					break;
				case 3:
					oledprintdata((i2coledi&~0x1F)+((i2coledi&1)<<4)+((i2coledi>>1)&0x0F),&oledhex16[(i2coledi&1?(temp&0x0F)|0x10:temp>>4)<<3]);
				}
				++i2coledi;
				i2ccontrol[1].state=0x80;
			}else switch (i2coledrefresh){
			case 0:
				break;
			case 4://dataHi
				i2coledoutbuf[count+0x1F]=opcommand[count-1]>>8;
				i2coledmax=(count<<1)+0x3F;
				i2coledi=(count<<1)+0x3E;
				i2coledfont=3;
				i2coledrefresh=3;
				break;
			case 3://dataLo
				i2coledoutbuf[count+0x2F]=opcommand[count-1]&0xFF;
				i2coledmax=(count<<1)+0x5F;
				i2coledi=(count<<1)+0x5E;
				i2coledfont=3;
				i2coledrefresh=2;
				break;
			case 2://count
				if (count>9){
					i2coledoutbuf[0x1E]='1';
					i2coledoutbuf[0x1F]='1'+count-11;
				}else{
					i2coledoutbuf[0x1E]='0';
					i2coledoutbuf[0x1F]='0'+count;
				}
				i2coledmax=0x3F;
				i2coledi=0x3C;
				i2coledfont=2;
				i2coledrefresh=1;
				break;
			case 1://state
				i2coledoutbuf[0x08]=toHex((state>>12)&0x0F);
				i2coledoutbuf[0x09]=toHex((state>>8)&0x0F);
				i2coledoutbuf[0x0A]=toHex((state>>4)&0x0F);
				i2coledoutbuf[0x0B]=toHex(state&0x0F);
				i2coledmax=0x17;
				i2coledi=0x10;
				i2coledfont=2;
				i2coledrefresh=0;
				break;
			}
		}

		switch (i2ccontrol[2].state){
		case 0x00:
			if (i2ccolorrefersh){
				i2ccontrol[2].state=0x80;
				i2ccolorrefersh=0;
			}
		}

		if (i2cbusy<0&&(int16_t)(i2cresetwait-wallclock)<=0){
			i2cbusy=0;
		}
		if (!i2cbusy){
			for (temp=3;temp;--temp){
				i2ccp=i2ccp->next;
				if (i2ccp->state&0x80){
					i2cbusy=1;
					i2ccp->cmdi=0;
					I2C0CONSET=0x20;	//start I2C
					break;
				}
			}
		}

		GPIO0DATA[P0_LEDr]=GPIO0DATA[P0_BUTTON]?P0_LEDr:0;

		if (0x01&~buttonstate){
			if (!buttonstate){
				temp=P0_ALL&~GPIO0DATA[P0_ALL];
				if (temp){
					buttonsample=temp;
					buttonstate=1;
					buttonwaitto=wallclock+buttonwait;
				}
			}else if (!(buttonstate&~GPIO0DATA[P0_ALL])){
				buttonstate|=1;
				buttonsample=0;
				buttonwaitto=wallclock+buttonwait;
			}
		}else if ((int16_t)(buttonwaitto-wallclock)>0){
			if (buttonsample){
				temp=buttonsample&~GPIO0DATA[P0_ALL];
				if (temp){
					buttonsample=temp;
				}else{
					buttonstate=0;
				}
			}else if (!(GPIO0DATA[P0_ALL]&buttonstate)){
				buttonstate&=~1;
			}
		}else{
			temp=(-buttonsample)&buttonsample;
			switch (temp){	//button down
			case P0_BUTTON:
				state=count?0x0040:0x0080;
				TMR32B0TCR=0x1;	//start counter
				break;
			case P0_SIG:
				i2ccolorrefersh=1;
				break;
			default:
				switch (buttonstate&~1){	//button up
				/*case P0_BUTTON:
					break;*/
				case P0_SIG:;
					if (count<16){
						temp=0;
						command[count]=0;
						opcommand[count]=0;
						if (i2ccolorinbuf[0]&0x03){
							command[count]|=0x01;
							opcommand[count]|=0x0001;
						}
						if (i2ccolorinbuf[0]&0x0C){
							command[count]|=0x02;
							opcommand[count]|=0x0010;
						}
						if (i2ccolorinbuf[0]&0x30){
							command[count]|=0x04;
							opcommand[count]|=0x0100;
						}
						if (i2ccolorinbuf[0]&0xC0){
							command[count]|=0x08;
							opcommand[count]|=0x1000;
						}
						++count;
						i2ciooutdata<<=1;
						i2ciorefresh=1;
						i2coledrefresh=4;
						duty=8;
						SYST_CVR=SYST_RVR=2999999;	//set period 2.9M(120BPM)
						patternp=&pattern[0x28];	//beepHi
						SYST_CSR=0x7;	//enable interrupt and start counter
					}else{
						phase=0;
						duty=4;
						SYST_CVR=SYST_RVR=2399999;	//set period 2.4M(150BPM)
						patternp=&pattern[0x60];	//error
						SYST_CSR=0x7;	//enable interrupt and start counter
					}
					break;
				}
			}
			buttonstate=temp;
		}
	}
}

void SysTick_Handler(){
	register int temp,cdata=*(patternp++);

	temp=cdata&0x0F;
	cdata>>=4;
	if (temp<0x0C){
		TMR16B1PR=(1<<(9-cdata))-1;
		TMR16B1MR3=period[temp];
		TMR16B1TCR=0x1;	//start counter
	}else{
		if (temp!=0x0F){
			TMR16B1TCR=0x2;	//stop and reset counter
			GPIO0DATA[P0_BUZZER]=0;
		}
		if (temp<0x0E){
			SYST_CSR=4;	//stop counter
			phase=0;
			GPIO0DATA[P0_BUZZER]=0;
		}
	}
}

void TIMER16_1_IRQHandler(){
	TMR16B1IR=TMR16B1IR;		//acknowledgement
	GPIO0DATA[P0_BUZZER]=(phase&0xF)<duty?P0_BUZZER:0;
	++phase;
}

void TIMER32_0_IRQHandler(){
	register int temp;

	TMR32B0IR=TMR32B0IR;		//acknowledgement
	if (state>=0x100){//wait/sustain
		state-=0x100;//wait(8),empty,start,move,stop,index(4)//0x0080 for empty, 0x0040 for exec
		i2coledrefresh=1;
		return;
	}
	if (state&0x80){//empty command
		if (state&0x10){
			TMR32B0TCR=0x2;	//stop and reset counter
			state=0;
			i2ciooutdata=0xFFFF;
		}else{
			i2ciooutdata=~(0x8000>>(state&0x0F));
			++state;
		}
		i2ciorefresh=1;
		i2coledrefresh=1;
		return;
	}
	if (state&0x40){//start
		state=exestartwait;
		GPIO0DATA[0xF00]=0xF00;
		i2ciooutdata=0xFFFF;
		i2ciorefresh=1;
		i2coledrefresh=1;
		duty=8;
		SYST_CVR=SYST_RVR=2999999;	//set period 2.9M(120BPM)
		patternp=&pattern[0x00];	//start
		SYST_CSR=0x7;	//enable interrupt and start counter
		return;
	}
	if (state&0x20){//release move
		state=state&~0x20;
		GPIO0DATA[0xF00]=0xF00;
		i2coledrefresh=1;
		return;
	}
	if (state>=count){//end
		TMR32B0TCR=0x2;	//stop and reset counter
		state=0;
		GPIO0DATA[0xF00]=0xF00;
		i2ciooutdata=0xFFFF<<count;
		i2ciorefresh=1;
		i2coledrefresh=1;
		phase=0;
		duty=8;
		SYST_CVR=SYST_RVR=2999999;	//set period 2.9M(120BPM)
		patternp=&pattern[0x30];	//end
		SYST_CSR=0x7;	//enable interrupt and start counter
		return;
	}
	temp=command[state];
	switch (temp){
	default:
		state|=exeerrorwait;
		duty=4;
		SYST_CVR=SYST_RVR=2399999;	//set period 2.4M(150BPM)
		patternp=&pattern[0x60];	//error
		SYST_CSR=0x7;	//enable interrupt and start counter
		break;
	case 0x07:
		state|=movewait=exefdbkwait;
		GPIO0DATA[0xF00]=movebuffer=0xA00;
		goto beep;
	case 0x0B:
		state|=movewait=exefdbkwait;
		GPIO0DATA[0xF00]=movebuffer=0x500;
		goto beep;
	case 0x0D:
		state|=movewait=exeltrtwait;
		GPIO0DATA[0xF00]=movebuffer=0x600;
		goto beep;
	case 0x0E:
		state|=movewait=exeltrtwait;
		GPIO0DATA[0xF00]=movebuffer=0x900;
		goto beep;
	case 0x0F:
		state|=movewait;
		GPIO0DATA[0xF00]=movebuffer;
beep:
		duty=8;
		SYST_CVR=SYST_RVR=2999999;	//set period 2.9M(120BPM)
		patternp=&pattern[0x58];	//beepLo
		SYST_CSR=0x7;	//enable interrupt and start counter
	}
	i2ciooutdata=~(1<<(state&0x0F));
	++state;
	i2ciorefresh=1;
	i2coledrefresh=1;
}

void I2C_IRQHandler(){
	register int c;

	switch (I2C0STAT){
	default:	//error
		I2C0CONSET=0x10;
		i2cbusy=0;
		break;
	case 0x08:	//start
	case 0x10:	//restart
		i2ccp->count=0;
		i2ccp->ini=0;
		I2C0DAT=i2ccp->cmd[i2ccp->cmdi++];
		I2C0CONCLR=0x20;
		break;
	case 0x18:	//acceptW
	case 0x28:	//dataW
		if (!i2ccp->count){
			c=i2ccp->cmd[i2ccp->cmdi++];
			if (c){
				if (c==0x10){
					i2ccp->state&=~0x80;
					i2cbusy=0;
				}
				I2C0CONSET=c;
			}else{
				i2ccp->count=i2ccp->cmd[i2ccp->cmdi++];
				I2C0DAT=i2ccp->cmd[i2ccp->cmdi++];
			}
		}else{
			I2C0DAT=i2ccp->cmd[i2ccp->cmdi++];
			--i2ccp->count;
		}
		break;
	case 0x40:	//acceptR
		i2ccp->count=i2ccp->cmd[i2ccp->cmdi++];
		if (i2ccp->count){
			I2C0CONSET=0x04;
		}
		break;
	case 0x50:	//dataR
		i2ccp->in[i2ccp->ini++]=I2C0DATA_BUFFER;
		if (!(--i2ccp->count)){
			I2C0CONCLR=0x04;
		}
		break;
	case 0x58:	//disconnectR
		i2ccp->in[i2ccp->ini++]=I2C0DATA_BUFFER;
		c=i2ccp->cmd[i2ccp->cmdi++];
		if (c==0x10){
			i2ccp->state&=~0x80;
			i2cbusy=0;
		}
		I2C0CONSET=c;
	}
	I2C0CONCLR=0x08;			//acknoledgement
}
