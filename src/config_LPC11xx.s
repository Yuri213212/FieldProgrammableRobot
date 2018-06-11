                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
__Vectors       DCD     0x10001000                ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler
                DCD     WAKEUP_IRQHandler         ; 16+ 0: Wakeup PIO0.0
                DCD     WAKEUP_IRQHandler         ; 16+ 1: Wakeup PIO0.1
                DCD     WAKEUP_IRQHandler         ; 16+ 2: Wakeup PIO0.2
                DCD     WAKEUP_IRQHandler         ; 16+ 3: Wakeup PIO0.3
                DCD     WAKEUP_IRQHandler         ; 16+ 4: Wakeup PIO0.4
                DCD     WAKEUP_IRQHandler         ; 16+ 5: Wakeup PIO0.5
                DCD     WAKEUP_IRQHandler         ; 16+ 6: Wakeup PIO0.6
                DCD     WAKEUP_IRQHandler         ; 16+ 7: Wakeup PIO0.7
                DCD     WAKEUP_IRQHandler         ; 16+ 8: Wakeup PIO0.8
                DCD     WAKEUP_IRQHandler         ; 16+ 9: Wakeup PIO0.9
                DCD     WAKEUP_IRQHandler         ; 16+10: Wakeup PIO0.10
                DCD     WAKEUP_IRQHandler         ; 16+11: Wakeup PIO0.11
                DCD     WAKEUP_IRQHandler         ; 16+12: Wakeup PIO1.0
                DCD     CAN_IRQHandler            ; 16+13: CAN
                DCD     SSP1_IRQHandler           ; 16+14: SSP1
                DCD     I2C_IRQHandler            ; 16+15: I2C
                DCD     TIMER16_0_IRQHandler      ; 16+16: 16-bit Counter-Timer 0
                DCD     TIMER16_1_IRQHandler      ; 16+17: 16-bit Counter-Timer 1
                DCD     TIMER32_0_IRQHandler      ; 16+18: 32-bit Counter-Timer 0
                DCD     TIMER32_1_IRQHandler      ; 16+19: 32-bit Counter-Timer 1
                DCD     SSP0_IRQHandler           ; 16+20: SSP0
                DCD     UART_IRQHandler           ; 16+21: UART
                DCD     USB_IRQHandler            ; 16+22: USB IRQ
                DCD     USB_FIQHandler            ; 16+24: USB FIQ
                DCD     ADC_IRQHandler            ; 16+24: A/D Converter
                DCD     WDT_IRQHandler            ; 16+25: Watchdog Timer
                DCD     BOD_IRQHandler            ; 16+26: Brown Out Detect
                DCD     FMC_IRQHandler            ; 16+27: IP2111 Flash Memory Controller
                DCD     PIOINT3_IRQHandler        ; 16+28: PIO INT3
                DCD     PIOINT2_IRQHandler        ; 16+29: PIO INT2
                DCD     PIOINT1_IRQHandler        ; 16+30: PIO INT1
                DCD     PIOINT0_IRQHandler        ; 16+31: PIO INT0

                AREA    |.ARM.__at_0x02FC|, CODE, READONLY
CRP_Key         DCD     0xFFFFFFFF

                AREA    |.text|, CODE, READONLY

; Dummy Exception Handlers (infinite loops which can be modified)                
Default_Handler PROC
                EXPORT  Reset_Handler             [WEAK]
                EXPORT  NMI_Handler               [WEAK]
                EXPORT  HardFault_Handler         [WEAK]
                EXPORT  SVC_Handler               [WEAK]
                EXPORT  PendSV_Handler            [WEAK]
                EXPORT  SysTick_Handler           [WEAK]
                EXPORT  WAKEUP_IRQHandler         [WEAK]
                EXPORT  CAN_IRQHandler            [WEAK]
                EXPORT  SSP1_IRQHandler           [WEAK]
                EXPORT  I2C_IRQHandler            [WEAK]
                EXPORT  TIMER16_0_IRQHandler      [WEAK]
                EXPORT  TIMER16_1_IRQHandler      [WEAK]
                EXPORT  TIMER32_0_IRQHandler      [WEAK]
                EXPORT  TIMER32_1_IRQHandler      [WEAK]
                EXPORT  SSP0_IRQHandler           [WEAK]
                EXPORT  UART_IRQHandler           [WEAK]
                EXPORT  USB_IRQHandler            [WEAK]
                EXPORT  USB_FIQHandler            [WEAK]
                EXPORT  ADC_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  FMC_IRQHandler            [WEAK]
                EXPORT  PIOINT3_IRQHandler        [WEAK]
                EXPORT  PIOINT2_IRQHandler        [WEAK]
                EXPORT  PIOINT1_IRQHandler        [WEAK]
                EXPORT  PIOINT0_IRQHandler        [WEAK]
Reset_Handler
NMI_Handler
HardFault_Handler
SVC_Handler
PendSV_Handler
SysTick_Handler
WAKEUP_IRQHandler
CAN_IRQHandler
SSP1_IRQHandler
I2C_IRQHandler
TIMER16_0_IRQHandler
TIMER16_1_IRQHandler
TIMER32_0_IRQHandler
TIMER32_1_IRQHandler
SSP0_IRQHandler
UART_IRQHandler
USB_IRQHandler
USB_FIQHandler
ADC_IRQHandler
WDT_IRQHandler
BOD_IRQHandler
FMC_IRQHandler
PIOINT3_IRQHandler  
PIOINT2_IRQHandler 
PIOINT1_IRQHandler
PIOINT0_IRQHandler
                B       .
                ENDP

                ALIGN

                END
