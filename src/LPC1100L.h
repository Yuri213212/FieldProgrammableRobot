#include <stdint.h>

//Chapter 3: system control block
#define SYSMEMREMAP			(*(volatile uint32_t*)0x40048000)	//RW,def=0x02
#define PRESETCTRL			(*(volatile uint32_t*)0x40048004)	//RW,def=0x0
#define SYSPLLCTRL			(*(volatile uint32_t*)0x40048008)	//RW,def=0x0
#define SYSPLLSTAT			(*(volatile uint32_t*)0x4004800C)	//R ,def=0x0
#define SYSOSCCTRL			(*(volatile uint32_t*)0x40048020)	//RW,def=0x0
#define WDTOSCCTRL			(*(volatile uint32_t*)0x40048024)	//RW,def=0x0
#define IRCCTRL				(*(volatile uint32_t*)0x40048028)	//RW,def=0x80
#define SYSRSTSTAT			(*(volatile uint32_t*)0x40048030)	//RW,def=0x0
#define SYSPLLCLKSEL		(*(volatile uint32_t*)0x40048040)	//RW,def=0x0
#define SYSPLLCLKUEN		(*(volatile uint32_t*)0x40048044)	//RW,def=0x0
#define MAINCLKSEL			(*(volatile uint32_t*)0x40048070)	//RW,def=0x0
#define MAINCLKUEN			(*(volatile uint32_t*)0x40048074)	//RW,def=0x0
#define SYSAHBCLKDIV		(*(volatile uint32_t*)0x40048078)	//RW,def=0x01
#define SYSAHBCLKCTRL		(*(volatile uint32_t*)0x40048080)	//RW,def=0x85F
#define SSP0CLKDIV			(*(volatile uint32_t*)0x40048094)	//RW,def=0x0
#define UARTCLKDIV			(*(volatile uint32_t*)0x40048098)	//RW,def=0x0
#define SSP1CLKDIV			(*(volatile uint32_t*)0x4004809C)	//RW,def=0x0
#define WDTCLKSEL			(*(volatile uint32_t*)0x400480D0)	//RW,def=0x0
#define WDTCLKUEN			(*(volatile uint32_t*)0x400480D4)	//RW,def=0x0
#define WDTCLKDIV			(*(volatile uint32_t*)0x400480D8)	//RW,def=0x0
#define CLKOUTCLKSEL		(*(volatile uint32_t*)0x400480E0)	//RW,def=0x0
#define CLKOUTUEN			(*(volatile uint32_t*)0x400480E4)	//RW,def=0x0
#define CLKOUTCLKDIV		(*(volatile uint32_t*)0x400480E8)	//RW,def=0x0
#define PIOPORCAP0			(*(volatile uint32_t*)0x40048100)	//R ,def=-
#define PIOPORCAP1			(*(volatile uint32_t*)0x40048104)	//R ,def=-
#define BODCTRL				(*(volatile uint32_t*)0x40048150)	//RW,def=0x0
#define SYSTCKCAL			(*(volatile uint32_t*)0x40048154)	//RW,def=0x04
#define NMISRC				(*(volatile uint32_t*)0x40048174)	//RW,def=0x0
#define STARTAPRP0			(*(volatile uint32_t*)0x40048200)	//RW,def=0x0
#define STARTERP0			(*(volatile uint32_t*)0x40048204)	//RW,def=0x0
#define STARTRSRP0CLR		(*(volatile uint32_t*)0x40048208)	// W,def=-
#define STARTSRP0			(*(volatile uint32_t*)0x4004820C)	//R ,def=-
#define PDSLEEPCFG			(*(volatile uint32_t*)0x40048230)	//RW,def=0x0
#define PDAWAKECFG			(*(volatile uint32_t*)0x40048234)	//RW,def=0xEDF0
#define PDRUNCFG			(*(volatile uint32_t*)0x40048238)	//RW,def=0xEDF0
#define DEVICE_ID			(*(volatile uint32_t*)0x400483F4)	//R ,def=?

//Chapter 4: PMU
#define PCON				(*(volatile uint32_t*)0x40038000)	//RW,def=0x0
#define GPREG				( (volatile uint32_t*)0x40038004)
#define GPREG0				(*(volatile uint32_t*)0x40038004)	//RW,def=0x0
#define GPREG1				(*(volatile uint32_t*)0x40038008)	//RW,def=0x0
#define GPREG2				(*(volatile uint32_t*)0x4003800C)	//RW,def=0x0
#define GPREG3				(*(volatile uint32_t*)0x40038010)	//RW,def=0x0
#define GPREG4				(*(volatile uint32_t*)0x40038014)	//RW,def=0x0

//Chapter 7: I/O configuration
#define IOCON_PIO2_6		(*(volatile uint32_t*)0x40044000)	//RW,def=0xD0
#define IOCON_PIO2_0		(*(volatile uint32_t*)0x40044008)	//RW,def=0xD0
#define IOCON_RESET_PIO0_0	(*(volatile uint32_t*)0x4004400C)	//RW,def=0xD0
#define IOCON_PIO0_1		(*(volatile uint32_t*)0x40044010)	//RW,def=0xD0
#define IOCON_PIO1_8		(*(volatile uint32_t*)0x40044014)	//RW,def=0xD0
#define IOCON_PIO0_2		(*(volatile uint32_t*)0x4004401C)	//RW,def=0xD0
#define IOCON_PIO2_7		(*(volatile uint32_t*)0x40044020)	//RW,def=0xD0
#define IOCON_PIO2_8		(*(volatile uint32_t*)0x40044024)	//RW,def=0xD0
#define IOCON_PIO2_1		(*(volatile uint32_t*)0x40044028)	//RW,def=0xD0
#define IOCON_PIO0_3		(*(volatile uint32_t*)0x4004402C)	//RW,def=0xD0
#define IOCON_PIO0_4		(*(volatile uint32_t*)0x40044030)	//RW,def=0x0
#define IOCON_PIO0_5		(*(volatile uint32_t*)0x40044034)	//RW,def=0x0
#define IOCON_PIO1_9		(*(volatile uint32_t*)0x40044038)	//RW,def=0xD0
#define IOCON_PIO3_4		(*(volatile uint32_t*)0x4004403C)	//RW,def=0xD0
#define IOCON_PIO2_4		(*(volatile uint32_t*)0x40044040)	//RW,def=0xD0
#define IOCON_PIO2_5		(*(volatile uint32_t*)0x40044044)	//RW,def=0xD0
#define IOCON_PIO3_5		(*(volatile uint32_t*)0x40044048)	//RW,def=0xD0
#define IOCON_PIO0_6		(*(volatile uint32_t*)0x4004404C)	//RW,def=0xD0
#define IOCON_PIO0_7		(*(volatile uint32_t*)0x40044050)	//RW,def=0xD0
#define IOCON_PIO2_9		(*(volatile uint32_t*)0x40044054)	//RW,def=0xD0
#define IOCON_PIO2_10		(*(volatile uint32_t*)0x40044058)	//RW,def=0xD0
#define IOCON_PIO2_2		(*(volatile uint32_t*)0x4004405C)	//RW,def=0xD0
#define IOCON_PIO0_8		(*(volatile uint32_t*)0x40044060)	//RW,def=0xD0
#define IOCON_PIO0_9		(*(volatile uint32_t*)0x40044064)	//RW,def=0xD0
#define IOCON_SWCLK_PIO0_10	(*(volatile uint32_t*)0x40044068)	//RW,def=0xD0
#define IOCON_PIO1_10		(*(volatile uint32_t*)0x4004406C)	//RW,def=0xD0
#define IOCON_PIO2_11		(*(volatile uint32_t*)0x40044070)	//RW,def=0xD0
#define IOCON_R_PIO0_11		(*(volatile uint32_t*)0x40044074)	//RW,def=0xD0
#define IOCON_R_PIO1_0		(*(volatile uint32_t*)0x40044078)	//RW,def=0xD0
#define IOCON_R_PIO1_1		(*(volatile uint32_t*)0x4004407C)	//RW,def=0xD0
#define IOCON_R_PIO1_2		(*(volatile uint32_t*)0x40044080)	//RW,def=0xD0
#define IOCON_PIO3_0		(*(volatile uint32_t*)0x40044084)	//RW,def=0xD0
#define IOCON_PIO3_1		(*(volatile uint32_t*)0x40044088)	//RW,def=0xD0
#define IOCON_PIO2_3		(*(volatile uint32_t*)0x4004408C)	//RW,def=0xD0
#define IOCON_SWDIO_PIO1_3	(*(volatile uint32_t*)0x40044090)	//RW,def=0xD0
#define IOCON_PIO1_4		(*(volatile uint32_t*)0x40044094)	//RW,def=0xD0
#define IOCON_PIO1_11		(*(volatile uint32_t*)0x40044098)	//RW,def=0xD0
#define IOCON_PIO3_2		(*(volatile uint32_t*)0x4004409C)	//RW,def=0xD0
#define IOCON_PIO1_5		(*(volatile uint32_t*)0x400440A0)	//RW,def=0xD0
#define IOCON_PIO1_6		(*(volatile uint32_t*)0x400440A4)	//RW,def=0xD0
#define IOCON_PIO1_7		(*(volatile uint32_t*)0x400440A8)	//RW,def=0xD0
#define IOCON_PIO3_3		(*(volatile uint32_t*)0x400440AC)	//RW,def=0xD0
#define IOCON_SCK_LOC		(*(volatile uint32_t*)0x400440B0)	//RW,def=0x0
#define IOCON_DSR_LOC		(*(volatile uint32_t*)0x400440B4)	//RW,def=0x0
#define IOCON_DCD_LOC		(*(volatile uint32_t*)0x400440B8)	//RW,def=0x0
#define IOCON_RI_LOC		(*(volatile uint32_t*)0x400440BC)	//RW,def=0x0

//Chapter 12: GPIO
#define GPIO0DATA			( (volatile uint32_t*)0x50000000)
#define GPIO0DATA_ALL		(*(volatile uint32_t*)0x50003FFC)	//RW,def=-
#define GPIO0DIR			(*(volatile uint32_t*)0x50008000)	//RW,def=0x0
#define GPIO0IS				(*(volatile uint32_t*)0x50008004)	//RW,def=0x0
#define GPIO0IBE			(*(volatile uint32_t*)0x50008008)	//RW,def=0x0
#define GPIO0IEV			(*(volatile uint32_t*)0x5000800C)	//RW,def=0x0
#define GPIO0IE				(*(volatile uint32_t*)0x50008010)	//RW,def=0x0
#define GPIO0RIS			(*(volatile uint32_t*)0x50008014)	//R ,def=0x0
#define GPIO0MIS			(*(volatile uint32_t*)0x50008018)	//R ,def=0x0
#define GPIO0IC				(*(volatile uint32_t*)0x5000801C)	// W,def=0x0
#define GPIO1DATA			( (volatile uint32_t*)0x50010000)
#define GPIO1DATA_ALL		(*(volatile uint32_t*)0x50013FFC)	//RW,def=-
#define GPIO1DIR			(*(volatile uint32_t*)0x50018000)	//RW,def=0x0
#define GPIO1IS				(*(volatile uint32_t*)0x50018004)	//RW,def=0x0
#define GPIO1IBE			(*(volatile uint32_t*)0x50018008)	//RW,def=0x0
#define GPIO1IEV			(*(volatile uint32_t*)0x5001800C)	//RW,def=0x0
#define GPIO1IE				(*(volatile uint32_t*)0x50018010)	//RW,def=0x0
#define GPIO1RIS			(*(volatile uint32_t*)0x50018014)	//R ,def=0x0
#define GPIO1MIS			(*(volatile uint32_t*)0x50018018)	//R ,def=0x0
#define GPIO1IC				(*(volatile uint32_t*)0x5001801C)	// W,def=0x0
#define GPIO2DATA			( (volatile uint32_t*)0x50020000)
#define GPIO2DATA_ALL		(*(volatile uint32_t*)0x50023FFC)	//RW,def=-
#define GPIO2DIR			(*(volatile uint32_t*)0x50028000)	//RW,def=0x0
#define GPIO2IS				(*(volatile uint32_t*)0x50028004)	//RW,def=0x0
#define GPIO2IBE			(*(volatile uint32_t*)0x50028008)	//RW,def=0x0
#define GPIO2IEV			(*(volatile uint32_t*)0x5002800C)	//RW,def=0x0
#define GPIO2IE				(*(volatile uint32_t*)0x50028010)	//RW,def=0x0
#define GPIO2RIS			(*(volatile uint32_t*)0x50028014)	//R ,def=0x0
#define GPIO2MIS			(*(volatile uint32_t*)0x50028018)	//R ,def=0x0
#define GPIO2IC				(*(volatile uint32_t*)0x5002801C)	// W,def=0x0
#define GPIO3DATA			( (volatile uint32_t*)0x50030000)
#define GPIO3DATA_ALL		(*(volatile uint32_t*)0x50033FFC)	//RW,def=-
#define GPIO3DIR			(*(volatile uint32_t*)0x50038000)	//RW,def=0x0
#define GPIO3IS				(*(volatile uint32_t*)0x50038004)	//RW,def=0x0
#define GPIO3IBE			(*(volatile uint32_t*)0x50038008)	//RW,def=0x0
#define GPIO3IEV			(*(volatile uint32_t*)0x5003800C)	//RW,def=0x0
#define GPIO3IE				(*(volatile uint32_t*)0x50038010)	//RW,def=0x0
#define GPIO3RIS			(*(volatile uint32_t*)0x50038014)	//R ,def=0x0
#define GPIO3MIS			(*(volatile uint32_t*)0x50038018)	//R ,def=0x0
#define GPIO3IC				(*(volatile uint32_t*)0x5003801C)	// W,def=0x0

//Chapter 13: UART
#define	U0RBR				(*(volatile uint32_t*)0x40008000)	//R ,def=-
#define	U0THR				(*(volatile uint32_t*)0x40008000)	// W,def=-
#define	U0DLL				(*(volatile uint32_t*)0x40008000)	//RW,def=0x01
#define	U0DLM				(*(volatile uint32_t*)0x40008004)	//RW,def=0x0
#define	U0IER				(*(volatile uint32_t*)0x40008004)	//RW,def=0x0
#define	U0IIR				(*(volatile uint32_t*)0x40008008)	//R ,def=0x01
#define	U0FCR				(*(volatile uint32_t*)0x40008008)	// W,def=0x0
#define	U0LCR				(*(volatile uint32_t*)0x4000800C)	//RW,def=0x0
#define	U0MCR				(*(volatile uint32_t*)0x40008010)	//RW,def=0x0
#define	U0LSR				(*(volatile uint32_t*)0x40008014)	//R ,def=0x60
#define	U0MSR				(*(volatile uint32_t*)0x40008018)	//R ,def=0x0
#define	U0SCR				(*(volatile uint32_t*)0x4000801C)	//RW,def=0x0
#define	U0ACR				(*(volatile uint32_t*)0x40008020)	//RW,def=0x0
#define	U0FDR				(*(volatile uint32_t*)0x40008028)	//RW,def=0x10
#define	U0TER				(*(volatile uint32_t*)0x40008030)	//RW,def=0x80
#define	U0RS485CTRL			(*(volatile uint32_t*)0x4000804C)	//RW,def=0x0
#define	U0RS485ADRMATCH		(*(volatile uint32_t*)0x40008050)	//RW,def=0x0
#define	U0RS485DLY			(*(volatile uint32_t*)0x40008054)	//RW,def=0x0

//Chapter 14: SPI
#define	SSP0CR0				(*(volatile uint32_t*)0x40040000)	//RW,def=0x0
#define	SSP0CR1				(*(volatile uint32_t*)0x40040004)	//RW,def=0x0
#define	SSP0DR				(*(volatile uint32_t*)0x40040008)	//RW,def=0x0
#define	SSP0SR				(*(volatile uint32_t*)0x4004000C)	//R ,def=0x03
#define	SSP0CPSR			(*(volatile uint32_t*)0x40040010)	//RW,def=0x0
#define	SSP0IMSC			(*(volatile uint32_t*)0x40040014)	//RW,def=0x0
#define	SSP0RIS				(*(volatile uint32_t*)0x40040018)	//R ,def=0x08
#define	SSP0MIS				(*(volatile uint32_t*)0x4004001C)	//R ,def=0x0
#define	SSP0ICR				(*(volatile uint32_t*)0x40040020)	// W,def=-
#define	SSP1CR0				(*(volatile uint32_t*)0x40058000)	//RW,def=0x0
#define	SSP1CR1				(*(volatile uint32_t*)0x40058004)	//RW,def=0x0
#define	SSP1DR				(*(volatile uint32_t*)0x40058008)	//RW,def=0x0
#define	SSP1SR				(*(volatile uint32_t*)0x4005800C)	//R ,def=0x03
#define	SSP1CPSR			(*(volatile uint32_t*)0x40058010)	//RW,def=0x0
#define	SSP1IMSC			(*(volatile uint32_t*)0x40058014)	//RW,def=0x0
#define	SSP1RIS				(*(volatile uint32_t*)0x40058018)	//R ,def=0x08
#define	SSP1MIS				(*(volatile uint32_t*)0x4005801C)	//R ,def=0x0
#define	SSP1ICR				(*(volatile uint32_t*)0x40058020)	// W,def=-

//Chapter 15: I2C
#define	I2C0CONSET			(*(volatile uint32_t*)0x40000000)	//RW,def=0x0
#define	I2C0STAT			(*(volatile uint32_t*)0x40000004)	//R ,def=0xF8
#define	I2C0DAT				(*(volatile uint32_t*)0x40000008)	//RW,def=0x0
#define	I2C0ADR0			(*(volatile uint32_t*)0x4000000C)	//RW,def=0x0
#define	I2C0SCLH			(*(volatile uint32_t*)0x40000010)	//RW,def=0x04
#define	I2C0SCLL			(*(volatile uint32_t*)0x40000014)	//RW,def=0x04
#define	I2C0CONCLR			(*(volatile uint32_t*)0x40000018)	// W,def=-
#define	I2C0MMCTRL			(*(volatile uint32_t*)0x4000001C)	//RW,def=0x0
#define	I2C0ADR				( (volatile uint32_t*)0x4000001C)
#define	I2C0ADR1			(*(volatile uint32_t*)0x40000020)	//RW,def=0x0
#define	I2C0ADR2			(*(volatile uint32_t*)0x40000024)	//RW,def=0x0
#define	I2C0ADR3			(*(volatile uint32_t*)0x40000028)	//RW,def=0x0
#define	I2C0DATA_BUFFER		(*(volatile uint32_t*)0x4000002C)	//R ,def=0x0
#define	I2C0MASK			( (volatile uint32_t*)0x40000030)
#define	I2C0MASK0			(*(volatile uint32_t*)0x40000030)	//RW,def=0x0
#define	I2C0MASK1			(*(volatile uint32_t*)0x40000034)	//RW,def=0x0
#define	I2C0MASK2			(*(volatile uint32_t*)0x40000038)	//RW,def=0x0
#define	I2C0MASK3			(*(volatile uint32_t*)0x4000003C)	//RW,def=0x0

//Chapter 18: 16-bit counter/timer
#define	TMR16B0IR			(*(volatile uint32_t*)0x4000C000)	//RW,def=0x0
#define	TMR16B0TCR			(*(volatile uint32_t*)0x4000C004)	//RW,def=0x0
#define	TMR16B0TC			(*(volatile uint32_t*)0x4000C008)	//RW,def=0x0
#define	TMR16B0PR			(*(volatile uint32_t*)0x4000C00C)	//RW,def=0x0
#define	TMR16B0PC			(*(volatile uint32_t*)0x4000C010)	//RW,def=0x0
#define	TMR16B0MCR			(*(volatile uint32_t*)0x4000C014)	//RW,def=0x0
#define	TMR16B0MR			( (volatile uint32_t*)0x4000C018)
#define	TMR16B0MR0			(*(volatile uint32_t*)0x4000C018)	//RW,def=0x0
#define	TMR16B0MR1			(*(volatile uint32_t*)0x4000C01C)	//RW,def=0x0
#define	TMR16B0MR2			(*(volatile uint32_t*)0x4000C020)	//RW,def=0x0
#define	TMR16B0MR3			(*(volatile uint32_t*)0x4000C024)	//RW,def=0x0
#define	TMR16B0CCR			(*(volatile uint32_t*)0x4000C028)	//RW,def=0x0
#define	TMR16B0CR0			(*(volatile uint32_t*)0x4000C02C)	//R ,def=0x0
#define	TMR16B0EMR			(*(volatile uint32_t*)0x4000C03C)	//RW,def=0x0
#define	TMR16B0CTCR			(*(volatile uint32_t*)0x4000C070)	//RW,def=0x0
#define	TMR16B0PWMC			(*(volatile uint32_t*)0x4000C074)	//RW,def=0x0
#define	TMR16B1IR			(*(volatile uint32_t*)0x40010000)	//RW,def=0x0
#define	TMR16B1TCR			(*(volatile uint32_t*)0x40010004)	//RW,def=0x0
#define	TMR16B1TC			(*(volatile uint32_t*)0x40010008)	//RW,def=0x0
#define	TMR16B1PR			(*(volatile uint32_t*)0x4001000C)	//RW,def=0x0
#define	TMR16B1PC			(*(volatile uint32_t*)0x40010010)	//RW,def=0x0
#define	TMR16B1MCR			(*(volatile uint32_t*)0x40010014)	//RW,def=0x0
#define	TMR16B1MR			( (volatile uint32_t*)0x40010018)
#define	TMR16B1MR0			(*(volatile uint32_t*)0x40010018)	//RW,def=0x0
#define	TMR16B1MR1			(*(volatile uint32_t*)0x4001001C)	//RW,def=0x0
#define	TMR16B1MR2			(*(volatile uint32_t*)0x40010020)	//RW,def=0x0
#define	TMR16B1MR3			(*(volatile uint32_t*)0x40010024)	//RW,def=0x0
#define	TMR16B1CCR			(*(volatile uint32_t*)0x40010028)	//RW,def=0x0
#define	TMR16B1CR0			(*(volatile uint32_t*)0x4001002C)	//R ,def=0x0
#define	TMR16B1EMR			(*(volatile uint32_t*)0x4001003C)	//RW,def=0x0
#define	TMR16B1CTCR			(*(volatile uint32_t*)0x40010070)	//RW,def=0x0
#define	TMR16B1PWMC			(*(volatile uint32_t*)0x40010074)	//RW,def=0x0

//Chapter 20: 32-bit counter/timer
#define	TMR32B0IR			(*(volatile uint32_t*)0x40014000)	//RW,def=0x0
#define	TMR32B0TCR			(*(volatile uint32_t*)0x40014004)	//RW,def=0x0
#define	TMR32B0TC			(*(volatile uint32_t*)0x40014008)	//RW,def=0x0
#define	TMR32B0PR			(*(volatile uint32_t*)0x4001400C)	//RW,def=0x0
#define	TMR32B0PC			(*(volatile uint32_t*)0x40014010)	//RW,def=0x0
#define	TMR32B0MCR			(*(volatile uint32_t*)0x40014014)	//RW,def=0x0
#define	TMR32B0MR			( (volatile uint32_t*)0x40014018)
#define	TMR32B0MR0			(*(volatile uint32_t*)0x40014018)	//RW,def=0x0
#define	TMR32B0MR1			(*(volatile uint32_t*)0x4001401C)	//RW,def=0x0
#define	TMR32B0MR2			(*(volatile uint32_t*)0x40014020)	//RW,def=0x0
#define	TMR32B0MR3			(*(volatile uint32_t*)0x40014024)	//RW,def=0x0
#define	TMR32B0CCR			(*(volatile uint32_t*)0x40014028)	//RW,def=0x0
#define	TMR32B0CR0			(*(volatile uint32_t*)0x4001402C)	//R ,def=0x0
#define	TMR32B0EMR			(*(volatile uint32_t*)0x4001403C)	//RW,def=0x0
#define	TMR32B0CTCR			(*(volatile uint32_t*)0x40014070)	//RW,def=0x0
#define	TMR32B0PWMC			(*(volatile uint32_t*)0x40014074)	//RW,def=0x0
#define	TMR32B1IR			(*(volatile uint32_t*)0x40018000)	//RW,def=0x0
#define	TMR32B1TCR			(*(volatile uint32_t*)0x40018004)	//RW,def=0x0
#define	TMR32B1TC			(*(volatile uint32_t*)0x40018008)	//RW,def=0x0
#define	TMR32B1PR			(*(volatile uint32_t*)0x4001800C)	//RW,def=0x0
#define	TMR32B1PC			(*(volatile uint32_t*)0x40018010)	//RW,def=0x0
#define	TMR32B1MCR			(*(volatile uint32_t*)0x40018014)	//RW,def=0x0
#define	TMR32B1MR			( (volatile uint32_t*)0x40018018)
#define	TMR32B1MR0			(*(volatile uint32_t*)0x40018018)	//RW,def=0x0
#define	TMR32B1MR1			(*(volatile uint32_t*)0x4001801C)	//RW,def=0x0
#define	TMR32B1MR2			(*(volatile uint32_t*)0x40018020)	//RW,def=0x0
#define	TMR32B1MR3			(*(volatile uint32_t*)0x40018024)	//RW,def=0x0
#define	TMR32B1CCR			(*(volatile uint32_t*)0x40018028)	//RW,def=0x0
#define	TMR32B1CR0			(*(volatile uint32_t*)0x4001802C)	//R ,def=0x0
#define	TMR32B1EMR			(*(volatile uint32_t*)0x4001803C)	//RW,def=0x0
#define	TMR32B1CTCR			(*(volatile uint32_t*)0x40018070)	//RW,def=0x0
#define	TMR32B1PWMC			(*(volatile uint32_t*)0x40018074)	//RW,def=0x0

//Chapter 22: Watchdog timer
#define	WDMOD				(*(volatile uint32_t*)0x40004000)	//RW,def=0x0
#define	WDTC				(*(volatile uint32_t*)0x40004004)	//RW,def=0xFF
#define	WDFEED				(*(volatile uint32_t*)0x40004008)	// W,def=-
#define	WDTV				(*(volatile uint32_t*)0x4000400C)	//R ,def=0xFF
#define	WDWARNINT			(*(volatile uint32_t*)0x40004014)	//RW,def=0x0
#define	WDWINDOW			(*(volatile uint32_t*)0x40004018)	//RW,def=0xFFFFFF

//Chapter 24: SysTick timer
#define	SYST_CSR			(*(volatile uint32_t*)0xE000E010)	//RW,def=0x0
#define	SYST_RVR			(*(volatile uint32_t*)0xE000E014)	//RW,def=0x0
#define	SYST_CVR			(*(volatile uint32_t*)0xE000E018)	//RW,def=0x0
#define	SYST_CALIB			(*(volatile uint32_t*)0xE000E01C)	//RW,def=0x04

//Chapter 25: ADC
#define	AD0CR				(*(volatile uint32_t*)0x4001C000)	//RW,def=0x0
#define	AD0GDR				(*(volatile uint32_t*)0x4001C004)	//RW,def=-
#define	AD0INTEN			(*(volatile uint32_t*)0x4001C00C)	//RW,def=0x100
#define	AD0DR				( (volatile uint32_t*)0x4001C010)
#define	AD0DR0				(*(volatile uint32_t*)0x4001C010)	//RW,def=-
#define	AD0DR1				(*(volatile uint32_t*)0x4001C014)	//RW,def=-
#define	AD0DR2				(*(volatile uint32_t*)0x4001C018)	//RW,def=-
#define	AD0DR3				(*(volatile uint32_t*)0x4001C01C)	//RW,def=-
#define	AD0DR4				(*(volatile uint32_t*)0x4001C020)	//RW,def=-
#define	AD0DR5				(*(volatile uint32_t*)0x4001C024)	//RW,def=-
#define	AD0DR6				(*(volatile uint32_t*)0x4001C028)	//RW,def=-
#define	AD0DR7				(*(volatile uint32_t*)0x4001C02C)	//RW,def=-
#define	AD0STAT				(*(volatile uint32_t*)0x4001C030)	//R ,def=0x0

//Chapter 26: FLASHCFG
#define FLASHCFG			(*(volatile uint32_t*)0x4003C010)	//RW,def=0x02

//Chapter 26: FMC
#define FMSSTART			(*(volatile uint32_t*)0x4003C020)	//RW,def=0x0
#define FMSSTOP				(*(volatile uint32_t*)0x4003C024)	//RW,def=0x0
#define FMSW				( (volatile uint32_t*)0x4003C02C)
#define FMSW0				(*(volatile uint32_t*)0x4003C02C)	//R ,def=-
#define FMSW1				(*(volatile uint32_t*)0x4003C030)	//R ,def=-
#define FMSW2				(*(volatile uint32_t*)0x4003C034)	//R ,def=-
#define FMSW3				(*(volatile uint32_t*)0x4003C038)	//R ,def=-
#define FMSTAT				(*(volatile uint32_t*)0x4003CFE0)	//R ,def=0x0
#define FMSTATCLR			(*(volatile uint32_t*)0x4003CFE8)	// W,def=-

//Chapter 28: NVIC
#define ISER				(*(volatile uint32_t*)0xE000E100)	//RW,def=0x0
#define ICER				(*(volatile uint32_t*)0xE000E180)	//RW,def=0x0
#define ISPR				(*(volatile uint32_t*)0xE000E200)	//RW,def=0x0
#define ICPR				(*(volatile uint32_t*)0xE000E280)	//RW,def=0x0
#define IPR					( (volatile uint32_t*)0xE000E400)
#define IPR0				(*(volatile uint32_t*)0xE000E400)	//RW,def=0x0
#define IPR1				(*(volatile uint32_t*)0xE000E404)	//RW,def=0x0
#define IPR2				(*(volatile uint32_t*)0xE000E408)	//RW,def=0x0
#define IPR3				(*(volatile uint32_t*)0xE000E40C)	//RW,def=0x0
#define IPR4				(*(volatile uint32_t*)0xE000E410)	//RW,def=0x0
#define IPR5				(*(volatile uint32_t*)0xE000E414)	//RW,def=0x0
#define IPR6				(*(volatile uint32_t*)0xE000E418)	//RW,def=0x0
#define IPR7				(*(volatile uint32_t*)0xE000E41C)	//RW,def=0x0

//Chapter 28: SCB
#define	CPUID				(*(volatile uint32_t*)0xE000ED00)	//R ,def=0x410CC200
#define	ICSR				(*(volatile uint32_t*)0xE000ED04)	//RW,def=0x0
#define	AIRCR				(*(volatile uint32_t*)0xE000ED0C)	//RW,def=0xFA050000
#define	SCR					(*(volatile uint32_t*)0xE000ED10)	//RW,def=0x0
#define	CCR					(*(volatile uint32_t*)0xE000ED14)	//R ,def=0x204
#define	SHPR2				(*(volatile uint32_t*)0xE000ED1C)	//RW,def=0x0
#define	SHPR3				(*(volatile uint32_t*)0xE000ED20)	//RW,def=0x0

//IRQ number
#define	Reset_IRQn			(-15)
#define	NMI_IRQn			(-14)
#define	HardFault_IRQn		(-13)
#define	SVC_IRQn			(-5)
#define	PendSV_IRQn			(-2)
#define	SysTick_IRQn		(-1)
#define	WAKEUP0_IRQn		0
#define	WAKEUP1_IRQn		1
#define	WAKEUP2_IRQn		2
#define	WAKEUP3_IRQn		3
#define	WAKEUP4_IRQn		4
#define	WAKEUP5_IRQn		5
#define	WAKEUP6_IRQn		6
#define	WAKEUP7_IRQn		7
#define	WAKEUP8_IRQn		8
#define	WAKEUP9_IRQn		9
#define	WAKEUP10_IRQn		10
#define	WAKEUP11_IRQn		11
#define	WAKEUP12_IRQn		12
#define	C_CAN_IRQn			13
#define	SPI1_IRQn			14
#define	I2C_IRQn			15
#define	CT16B0_IRQn			16
#define	CT16B1_IRQn			17
#define	CT32B0_IRQn			18
#define	CT32B1_IRQn			19
#define	SPI0_IRQn			20
#define	UART_IRQn			21
#define	ADC_IRQn			24
#define	WDT_IRQn			25
#define	BOD_IRQn			26
#define	PIO_3_IRQn			28
#define	PIO_2_IRQn			29
#define	PIO_1_IRQn			30
#define	PIO_0_IRQn			31

//SVC command
#define NMI() ICSR=0x80000000
__svc(0) void SVC(void);
#define PSV() ICSR=0x10000000

//Entry point	//Exception 1
void Reset_Handler(void) __attribute__((noreturn));
