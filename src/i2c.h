//device 0x40	MCP23017	16 bit I/O expander
//device 0x78	SH1106		128*64 OLED dot matrix screen
//device 0x20	???			color sensor

#include <stdint.h>

struct i2ctransaction{
	uint8_t state;		//loaded,phase(7)
	uint8_t cmdi;		//set 0 on start
	uint8_t count;		//set 0 on start/restart
	uint8_t ini;		//set 0 on start/restart
	uint8_t const *cmd;	//{addr(e),ctrl,[max,[data],ctrl]},{addr(o),max,ctrl}	//ctrl:0x20=restart,0x10=stop,0x00=data//max:datalen-1
	uint8_t *in;
	struct i2ctransaction *next;
};

const uint8_t i2cioinit[]={0x40,0x00,4,0x14,0xFF,0xFF,0x00,0x00,0x10};
const uint8_t i2coledinit[]={0x78,0x00,3,0x00,0xD5,0xF0,0xAF,0x10};
const uint8_t i2ccolorin[]={0x21,0,0x10};
