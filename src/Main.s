            TTL Exercise Ten - Timer Driver Input Timing
;****************************************************************
;
;Name:  Chris Abajian
;Date:  11/1/2018
;Class:  CMPE-250
;Section:  01L3, Thursday, 11AM
;---------------------------------------------------------------
;Keil Template for KL46
;R. W. Melton
;February 5, 2018
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;---------------------------------------------------------------
;Characters
CR          EQU  0x0D
LF          EQU  0x0A
NULL        EQU  0x00
;Max string character length
MAX_STRING	EQU	 79
;Queue structure displacements
IN_PTR		EQU		0
OUT_PTR		EQU		4
BUF_STRT	EQU		8
BUF_PAST	EQU		12
BUF_SIZE	EQU		16
NUM_ENQD	EQU		17
;Queue structure sizes
Q_REC_SZ	EQU		18
Q_BUF_SZ	EQU		6
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << UART0_PRI_POS)
;--UART0
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU  (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU  (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at 24 MHz count rate
;0.01 s * 24,000,000 Hz = 240,000
;TSV = 240,000 - 1
PIT_LDVAL_10ms  EQU  239999
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRLn:  PIT timer control register n
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;0x38->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
Reset_Handler  PROC  {}
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL46 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
		;init UART0 IRQ
			 BL		Init_UART0_IRQ			;initialize UART0
		;initialize stopwatch and count to zero	
			MOVS	R0,#0
			LDR		R1,=Count
			STR		R0,[R1,#0]
			LDR		R1,=RunStopWatch
			STRB	R0,[R1,#0]
		;init PIT IRQ
			BL		Init_PIT_IRQ
Prompt	;main prompt
			LDR		R0,=MAIN_PRM			;get prompt address
			LDR		R1,=MAX_STRING			;get MAX_STRING
			BL		PutStringSB
		;linefeed, put '>'
			LDR		R0,=CRLF
			BL		PutStringSB
			MOVS	R0,'>'
			BL		PutChar
		;clear count, set stopwatch boolean
			MOVS	R0,#0
			LDR		R1,=Count
			STR		R0,[R1,#0]
			MOVS	R0,#1
			LDR		R1,=RunStopWatch
			STRB	R0,[R1,#0]
		;get user input
			LDR		R0,=OutStr
			MOVS	R1,#MAX_STRING
			BL		GetStringSB
		;clear stopwatch boolean
			MOVS	R0,#0
			LDR		R1,=RunStopWatch
			STRB	R0,[R1,#0]
		;linefeed, put '<'
			LDR		R0,=CRLF
			BL		PutStringSB
			MOVS	R0,'<'
			BL		PutChar
		;put timer count
			LDR		R0,=Count
			LDR		R0,[R0,#0]
			BL		PutNumU
		;put remaining count information
			LDR		R0,=COUNT_INFO
			MOVS	R1,#MAX_STRING
			BL		PutStringSB
		;linefeed
			LDR		R0,=CRLF
			BL		PutStringSB
		;load input & password
			LDR		R0,=OutStr
			LDR		R1,=PASSWORD
Loop	;load first characters of input and password
			LDRB	R2,[R0,#0]
			LDRB	R3,[R1,#0]
		;check if characters are equal
			CMP		R2,R3
		;if not, access denied
			BNE		Denied
		;if yes, increment pointers, check if end of input
			ADDS	R0,R0,#1
			ADDS	R1,R1,#1
			CMP		R2,#NULL
			BNE		Loop
		;if end and all characters match, check timer count
			LDR		R0,=Count
			LDR		R0,[R0,#0]
			MOVS	R1,#250
			LSLS	R1,R1,#1
			CMP		R0,R1
			BLS		Granted
Denied	;put denied statement
			LDR		R0,=DENIED
			MOVS	R1,#MAX_STRING
			BL		PutStringSB
		;linefeed x2
			LDR		R0,=CRLF
			BL		PutStringSB
			BL		PutStringSB
		;return to start
			B		Prompt
Granted	;put granted statement
			LDR		R0,=GRANTED
			MOVS	R1,#MAX_STRING
			BL		PutStringSB
EndChck	;linefeed, put mission complete
			LDR		R0,=CRLF
			BL		PutStringSB
			LDR		R0,=COMPLETED
			BL		PutStringSB
			LDR		R0,=CRLF
			BL		PutStringSB
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<
Init_PIT_IRQ	PROC	{R0-R14}
;Interrupt service request handler to initialize
;the PIT to generate an interrupt every 0.01 s
;from PIT channel 0.
;	Inputs:
;		-
;	OutPuts:
;		-
;	Modified Registers
;		APSR
				PUSH	{R0-R3}
			;Enable clock for PIT module
				LDR		R0,=SIM_SCGC6
				LDR		R1,=SIM_SCGC6_PIT_MASK
				LDR		R2,[R0,#0]
				ORRS	R2,R2,R1
				STR		R2,[R0,#0]
			;Disable PIT timer 0
				LDR		R0,=PIT_CH0_BASE
				LDR		R1,=PIT_TCTRL_TEN_MASK
				LDR		R2,[R0,#PIT_TCTRL_OFFSET]
				BICS	R2,R2,R1
				STR		R2,[R0,#PIT_TCTRL_OFFSET]
			;Set PIT interrupt priority
				LDR		R0,=PIT_IPR
				LDR		R1,=NVIC_IPR_PIT_MASK
				;LDR	R2,=NVIC_IPR_PIT_PRI_0
				LDR		R3,[R0,#0]
				BICS	R3,R3,R1
				;ORRS	R3,R3,R2
				STR		R3,[R0,#0]
			;Clear any pending PIT interrupts
				LDR		R0,=NVIC_ICPR
				LDR		R1,=NVIC_ICPR_PIT_MASK
				STR		R1,[R0,#0]
			;Unmask PIT interrupts
				LDR		R0,=NVIC_ISER
				LDR		R1,=NVIC_ISER_PIT_MASK
				STR		R1,[R0,#0]
			;Enable PIT module
				LDR		R0,=PIT_BASE
				LDR		R1,=PIT_MCR_EN_FRZ
				STR		R1,[R0,#PIT_MCR_OFFSET]
			;Set PIT timer 0 period for 0.01 s
				LDR		R0,=PIT_CH0_BASE
				LDR		R1,=PIT_LDVAL_10ms 
				STR		R1,[R0,#PIT_LDVAL_OFFSET]
			;Enable PIT timer 0 interrupt
				LDR		R1,=PIT_TCTRL_CH_IE
				STR		R1,[R0,#PIT_TCTRL_OFFSET]
				
				POP		{R0-R3}
				BX		LR
				ENDP	
				
				
				
PIT_ISR			PROC	{R0-R14}
;Handler for an interrupt service request generated
;by periodic interrupt timer (PIT). If the stop watch
;is enabled, the count variable increases by 1. The
;interrupt is cleared before returning.
;	Inputs:
;		-
;	OutPuts:
;		-
;	Modified Registers
;		APSR
				PUSH	{R0-R1}
			;load stopwatch
				LDR		R0,=RunStopWatch
				LDRB	R0,[R0,#0]
			;if runstopwatch enabled (non-zero)
				CMP		R0,#0
				BEQ		PIT_ISR_Skip
			;load and increment count
				LDR		R0,=Count
				LDR		R1,[R0,#0]
				ADDS	R1,R1,#1
				STR		R1,[R0,#0]
PIT_ISR_Skip	
			;clear interrupt
				LDR		R0,=PIT_CH0_BASE
				LDR		R1,=PIT_TFLG_TIF_MASK
				STR		R1,[R0,#PIT_TFLG_OFFSET]
			;restore registers, return
				POP		{R0-R1}
				BX		LR
				ENDP	
					
					
					
GetChar		PROC	{R1-R14}
;Attempts to get a character from the receive queue
;by dequeueing until successful.
;	Inputs:
;		-
;	Outputs:
;		R0: character dequeued from receive queue
;	Modified Registers:
;		R0, APSR
			PUSH	{LR, R1}
		;mask all other interrupts
PollRxQ		CPSID	I
		;attempt to dequeue character
			LDR		R1,=RxQRecord
			BL		Dequeue
		;unmask all other interrupts
			CPSIE	I
		;if carry is clear-- return
			BCC		GetCharEnd
		;if carry is set-- re-poll queue until character is available
			B		PollRxQ
		;return
GetCharEnd	POP		{PC, R1}
			BX		LR
			ENDP
;end GetChar PROC


PutChar		PROC	{R0-R14}
;Attempts to put a character into the transmit queue
;by enqueueing the character until successful.
;	Inputs:
;		R0: character to transmit
;	Outputs:
;		-
;	Modified Registers:
;		APSR
		;store modified registers
			PUSH	{R0,R1,LR}
		;mask all other interrupts
PollTxQ		CPSID	I
		;attempt to enqueue character
			LDR		R1,=TxQRecord
			BL		Enqueue
		;unmask all other interrupts
			CPSIE	I
		;if carry is set-- re-poll queue until character is enqueued
			BCS		PollTxQ
		;enable TxInterrupt
PutCharEnd	LDR		R0,=UART0_BASE
			MOVS	R1,#UART0_C2_TI_RI
			STRB	R1,[R0,#UART0_C2_OFFSET]
		;restore preserved registers and return
			POP		{R0,R1,PC}
			BX		LR
			ENDP
;end PutChar PROC


UART0_ISR	PROC	{R0-R14}
;Interrupt Service Routine for UART0. Handles
;request according to if transmit interrupt is
;enabled, or otherwise if receive interrupt is
;enabled.
;	Inputs:
;		-
;	Outputs:
;		-
;	Modified Registers:
;		APSR
		;mask interrupts
			CPSID	I
		;push modified registers
			PUSH	{LR}
		;load UART0 base
			LDR		R0,=UART0_BASE
		;if transmit interrupt is enabled
			MOVS	R1,#UART0_C2_TIE_MASK
			LDRB	R2,[R0,#UART0_C2_OFFSET]
			ANDS	R1,R1,R2
			BEQ		ISRRx
		;if transmit interrupt
			MOVS	R1,#UART0_S1_TDRE_MASK
			LDRB	R2,[R0,#UART0_S1_OFFSET]
			ANDS	R1,R1,R2
			BEQ		ISRRx
		;dequeue character from TxQueue
			LDR		R1,=TxQRecord
			BL		Dequeue
		;if successful dequeue...
			LDR		R1,=UART0_BASE
			BCS		ISRElse
		;...write char to UART0 data register
			STRB	R0,[R1,#UART0_D_OFFSET]
			B		ISRRx
		;...else, disable TxInterrupt
ISRElse		MOVS	R2,#UART0_C2_T_RI
			STRB	R2,[R1,#UART0_C2_OFFSET]
ISRRx	;if RxInterrupt is enabled...
			MOVS	R1,#UART0_S1_RDRF_MASK
			LDR		R2,=UART0_BASE
			LDRB	R2,[R2,#UART0_S1_OFFSET]
			ANDS	R1,R1,R2
			BEQ		ISREnd
		;...read char from UART0 data register
			LDR		R2,=UART0_BASE
			LDRB	R0,[R2,#UART0_D_OFFSET]
			LDR		R1,=RxQRecord
			BL		Enqueue
ISREnd	;unmask other interrupts
			CPSIE	I
		;restore preserved registers
			POP		{PC}
			ENDP
;end ISR PROC

			
Init_UART0_IRQ		PROC	{R0-R12}
;Initializzes the KL46 for interrupt serial I/O
;with UART0 through port A pins 1 and 2 using
;the format 8N1 (8 data bits, no parity, one stop
;bit) at 9600 baud.
;	Inputs:
;		-
;	Outputs:
;		-
;	Modified Registers:
;		APSR
		;store {R0-R3,LR} in stack
			PUSH	{R0-R3,LR}
		;init queue record structures
			LDR		R0,=RxQBuffer
			LDR		R1,=RxQRecord
			MOVS	R2,#Q_BUF_SZ
			BL		InitQueue
			LDR		R0,=TxQBuffer
			LDR		R1,=TxQRecord
			MOVS	R2,#Q_BUF_SZ
			BL		InitQueue
		;select MCGPLLCLK / 2 as clock source for UART0
			LDR		R0,=SIM_SOPT2
			LDR		R1,=SIM_SOPT2_UART0SRC_MASK
			LDR		R2,[R0,#0]
			BICS	R2,R2,R1
			LDR		R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
			ORRS	R2,R2,R1
			STR		R2,[R0,#0]
		;enable external connections for UART0
			LDR		R0,=SIM_SOPT5
			LDR		R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR		R2,[R0,#0]
			BICS	R2,R2,R1
			STR		R2,[R0,#0]
		;enable clock for UART0
			LDR		R0,=SIM_SCGC4
			LDR		R1,=SIM_SCGC4_UART0_MASK
			LDR		R2,[R0,#0]
			ORRS	R2,R2,R1
			STR		R2,[R0,#0]
		;connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
			LDR     R0,=PORTA_PCR1
			LDR     R1,=PORT_PCR_SET_PTA1_UART0_RX
			STR     R1,[R0,#0]
		;connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)
			LDR     R0,=PORTA_PCR2
			LDR     R1,=PORT_PCR_SET_PTA2_UART0_TX
			STR     R1,[R0,#0]
		;disable UART0 receiver and transmitter
			LDR		R0,=UART0_BASE
			MOVS	R1,#UART0_C2_T_R
			LDRB	R2,[R0,#UART0_C2_OFFSET]
			BICS	R2,R2,R1
			STRB	R2,[R0,#UART0_C2_OFFSET]
		;initialize NVIC for UART0 interrupts
		;Set UART0 IRQ priority
			LDR		R0,=UART0_IPR
			;LDR 	R1,=NVIC_IPR_UART0_MASK
			LDR		R2,=NVIC_IPR_UART0_PRI_3
			LDR 	R3,[R0,#0]
			;BICS 	R3,R3,R1
			ORRS 	R3,R3,R2
			STR 	R3,[R0,#0]
		;Clear any pending UART0 interrupts
			LDR 	R0,=NVIC_ICPR
			LDR 	R1,=NVIC_ICPR_UART0_MASK
			STR 	R1,[R0,#0]
		;Unmask UART0 interrupts
			LDR 	R0,=NVIC_ISER
			LDR 	R1,=NVIC_ISER_UART0_MASK
			STR 	R1,[R0,#0]
		;set UART0 for 9600 baud, 8N1 protocol
			LDR		R0,=UART0_BASE
			MOVS	R1,#UART0_BDH_9600
			STRB	R1,[R0,#UART0_BDH_OFFSET]
			MOVS	R1,#UART0_BDL_9600
			STRB	R1,[R0,#UART0_BDL_OFFSET]
			MOVS	R1,#UART0_C1_8N1
			STRB	R1,[R0,#UART0_C1_OFFSET]
			MOVS	R1,#UART0_C3_NO_TXINV
			STRB	R1,[R0,#UART0_C3_OFFSET]
			MOVS	R1,#UART0_C4_NO_MATCH_OSR_16
			STRB	R1,[R0,#UART0_C4_OFFSET]
			MOVS	R1,#UART0_C5_NO_DMA_SSR_SYNC
			STRB	R1,[R0,#UART0_C5_OFFSET]
			MOVS	R1,#UART0_S1_CLEAR_FLAGS
			STRB	R1,[R0,#UART0_S1_OFFSET]
			MOVS	R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
			STRB	R1,[R0,#UART0_S2_OFFSET]
		;enable UART0 receiver and transmitter
			MOVS	R1,#UART0_C2_T_RI
			STRB	R1,[R0,#UART0_C2_OFFSET]
			
		;restore {R0-R3} from stack
			POP		{R0-R3,PC}
					
			BX		LR
			ENDP
;end Init_UART0_IRQ PROC
				
				
InitQueue		PROC	{R0-R14}
;Initializes the queue record structure at
;the provided address in memory.
;Inputs:
;	R0: queue buffer address
;	R1: queue structure address
;	R2: queue character capacity
;Outputs:
;	-
;Modified Registers:
;	APSR	
			;preserve modified register values
				PUSH	{R0}
			;store queue record data
				STR		R0,[R1,#IN_PTR]				;store InPointer to queue record
				STR		R0,[R1,#OUT_PTR]			;store OutPointer to queue record
				STR		R0,[R1,#BUF_STRT]			;store BufferStart to queue record
				ADDS	R0,R0,R2					;increment buffer address by max buffer size
				STR		R0,[R1,#BUF_PAST]			;store BufferPast to queue record
				STRB	R2,[R1,#BUF_SIZE]			;store BufferSize to queue record
				MOVS	R0,#0						;set NumberEnqueued to 0
				STRB	R0,[R1,#NUM_ENQD]			;store NumberEnqueued to queue record
			;restore preserved register values
				POP		{R0}
				BX		LR
				ENDP
;end InitQueue PROC
				
				
Dequeue			PROC	{R1-R14}
;Attempts to get a character from the queue and
;returns the character and clears PSR C flag if successful.
;If unsuccessful, PSR C flag is set.
;Inputs:
;	R1: queue structure address
;Outputs:
;	R0: dequeue'd character
;	PSR C flag: Success(0) or Failure(1)
;Modified Registers:
;	R0, APSR
			;preserve modified register values
				PUSH	{R1-R5}
			;load queue record data
				LDR		R2,[R1,#OUT_PTR]		;R2 <- OutPointer
				LDR		R3,[R1,#BUF_STRT]		;R3 <- BufferStart
				LDR		R4,[R1,#BUF_PAST]		;R4 <- BufferPast
				LDRB	R5,[R1,#NUM_ENQD]		;R5 <- NumberEnqueued
			;check if queue is empty
				CMP		R5,#0
				BEQ		DeQElse
			;remove element from queue
				LDRB	R0,[R2,#0]
			;decrement NumberEnqueued by 1 character
				SUBS	R5,R5,#1
				STRB	R5,[R1,#NUM_ENQD]
			;increment OutPointer by 1 character
				ADDS	R2,R2,#1
				STR		R2,[R1,#OUT_PTR]
			;check if OutPointer points outside buffer
				CMP		R2,R4
				BLT		DeQSkip
				MOVS	R2,R3					;set OutPointer to BufferStart
				STR		R2,[R1,#OUT_PTR]		;store OutPointer
DeQSkip		;clear carry flag
				PUSH	{R0}
				MRS		R0,APSR
				MOVS	R1,#0x20
				LSLS	R1,R1,#24
				BICS	R0,R0,R1
				MSR		APSR,R0
				POP		{R0}
				B		DeQEnd
DeQElse		;set carry flag
				PUSH	{R0}
				MRS		R0,APSR
				MOVS	R1,#0x20
				LSLS	R1,R1,#24
				ORRS	R0,R0,R1
				MSR		APSR,R0
				POP		{R0}
DeQEnd		;restore preserved register values
				POP		{R1-R5}
				BX		LR
				ENDP
;end Deueue PROC
				
				
Enqueue			PROC	{R1-R14}
;Attempts to put a character in the queue and
;clears PSR C flag if successful. If unsuccessful,
;PSR C flag is set.
;Inputs:
;	R0: character to enqueue
;	R1: queue structure address
;Outputs:
;	PSR C flag: Success(0) or Failure(1)
;Modified Registers:
;	APSR
			;preserve modified register values
				PUSH	{R0-R6}
			;load queue record data
				LDR		R2,[R1,#IN_PTR]			;R2 <- InPointer
				LDR		R3,[R1,#BUF_STRT]		;R3 <- BufferStart
				LDR		R4,[R1,#BUF_PAST]		;R4 <- BufferPast
				MOVS	R5,#0
				LDRB	R5,[R1,#BUF_SIZE]		;R5 <- BufferSize
				LDRB	R6,[R1,#NUM_ENQD]		;R6 <- NumberEnqueued
			;check if queue is full
				CMP		R6,R5
				BHS		EnQElse
			;put element in queue
				STRB	R0,[R2,#0]
			;increment NumberEnqueued by 1 character
				ADDS	R6,R6,#1
				STRB	R6,[R1,#NUM_ENQD]
			;increment InPointer by 1 character
				ADDS	R2,R2,#1
				STR		R2,[R1,#IN_PTR]
			;check if InPointer points outside buffer
				CMP		R2,R4
				BLT		EnQSkip
				MOVS	R2,R3					;set InPointer to BufferStart
				STR		R2,[R1,#IN_PTR]			;store InPointer
EnQSkip		;clear carry flag
				MRS		R0,APSR
				MOVS	R1,#0x20
				LSLS	R1,R1,#24
				BICS	R0,R0,R1
				MSR		APSR,R0
				B		EnQEnd
EnQElse		;set carry flag
				MRS		R0,APSR
				MOVS	R1,#0x20
				LSLS	R1,R1,#24
				ORRS	R0,R0,R1
				MSR		APSR,R0
EnQEnd		;restore preserved register values
				POP		{R0-R6}
				BX		LR
				ENDP
;end Enqueue PROC
			
			
GetStringSB		PROC	{R0-R14}
;Gets a string of characters typed to the terminal and
;displays them as well as storing them in a string buffer.
;If the string is longer than the capacity, any characters
;typed are not saved or displayed.
;	Inputs:
;		R0: Output string reference
;		R1: Buffer Capacity
;	Outputs:
;		-
;	Modified Registers:
;		APSR
;	Uses:
;		GetChar
		;store modified registers, LR
			PUSH	{R0-R2,LR}
		;store string ref into R2
			MOVS	R2,R0
		;get first character
			BL		GetChar
GSSBCondCR	CMP		R0,#CR			;carriage return condition
			BEQ		GSSBEnd			;	if (R0 == 0x0D) return
GSSBCondSB  SUBS	R1,R1,#1		;string buffer condition
			BEQ		GSSBWait		;	if (remainingBuffer == 0) wait for carriage return
GSSBBody	BL		PutChar
			STRB	R0,[R2,#0]		;store character
			ADDS	R2,R2,#1		;increment character count
			BL		GetChar			;get next character
			B		GSSBCondCR
GSSBWait	BL		GetChar
			CMP		R0,#CR			;carriage return condition
			BNE		GSSBWait		;	if (R0 == 0x0D) return
GSSBEnd		MOVS	R0,#0
			STRB	R0,[R2,#0]
		;restore modified registers, PC
			POP		{R0-R2,PC}
		;return
			BX		LR
			ENDP
;end GetStringSB PROC



PutStringSB		PROC	{R0-R14}
;Displays a string specified at the input reference
;to the terminal. If the string exceeds the capacity,
;the characters exceeding the buffer capacity are not
;displayed.
;	Inputs:
;		R0: Input string reference
;		R1: Buffer Capacity
;	Outputs:
;		-
;	Modified Registers:
;		APSR
;	Uses:
;		PutChar
		;preserve modified registers, LR
			PUSH	{R0-R2,LR}
		;check if buffer is zero
			CMP		R1,#0
			BEQ		PSSBEnd
		;get first character
			LDRB	R2,[R0,#0]
PSSBCondCR	CMP		R2,#NULL		;null terminated condition
			BEQ		PSSBEnd			;	if (R2 == 0) return
PSSBCondSB  SUBS	R1,R1,#1		;string buffer condition
			BEQ		PSSBEnd			;	if (remainingBuffer == 0) return
PSSBBody	PUSH	{R0}			;preserve string reference
			MOVS	R0,R2			;move character to R0
			BL		PutChar			;display character
			POP		{R0}			;restore string reference
			ADDS	R0,R0,#1		;increment string reference
			LDRB	R2,[R0,#0]		;load next character
			B		PSSBCondCR
PSSBEnd	;restore modified registers, PC
			POP		{R0-R2,PC}
		;return
			BX		LR
			ENDP
;end PutStringSB PROC
				
			
			
PutNumU		PROC	{R0-R14}
;Displays a text decimal representation of an
;unsigned word input.
;	Inputs:
;		R0: Unsigned word value
;	Outputs:
;		-
;	Modified Registers:
;		APSR
;	Uses:
;		DIVU
;		PutChar
		;store modified registers, LR
			PUSH	{R0-R2,LR}
			MOVS	R2,#0
PNULoop	;move quotient to DIVU dividend input, R1
			MOVS	R1,R0
		;set divisor to 10
			MOVS	R0,#10
		;divide
			BL		DIVU
		;convert and store remainder
			ADDS	R1,R1,#48
		;push value to stack
			PUSH	{R1}
		;increment counter
			ADDS	R2,R2,#1
PNUCond	;repeat while quotient != 0
			CMP		R0,#0
			BNE		PNULoop
PNULoop2 ;display stored values
			POP		{R0}
			BL		PutChar
			SUBS	R2,R2,#1
			BNE		PNULoop2
		;restore modified registers, PC
			POP		{R0-R2,PC}
		;return
			BX		LR
			ENDP
;end PutNumU PROC



DIVU			PROC	{R2-R14}
;Divides the value in R1 by the value in R0, 
;thus performing R1/R0. The result is placed
;in R0 with a remainder in R1.
;Parameters:
;  Inputs:
;	  R1:	Dividend (positive integer)
;	  R0:	Divisor (positive, non-zero integer)
;  Outputs:
;	  R0:	Quotient (positive integer)
;	  R1:	Remainder (positive integer)
;  DESTRUCTIVE: Inputs R0,R1 are modified			
DIVUIfErZ	CMP		R0,#0			;if (R0 == 0) {
			BEQ		DIVUEndIfEr		;	endif: zero divisor; }
			CMP		R1,#0			;if (R1 == 0) {
			BEQ		DIVUEndIfZ		;	endif: zero dividend; }
			B		DIVUInit		;else { init division }

DIVUEndIfEr	PUSH	{R0}
			PUSH	{R1}
			MRS		R0,APSR
			MOVS	R1,#0x20
			LSLS	R1,R1,#24
			ORRS	R0,R0,R1
			MSR		APSR,R0
			POP		{R1}
			POP		{R0}
			B		DIVUReturn		;Set Carry flag, Return
			

DIVUEndIfZ	MOVS	R0,#0			;Set Divisor to 0
			PUSH	{R0}
			PUSH	{R1}
			MRS		R0,APSR
			MOVS	R1,#0x20
			LSLS	R1,R1,#24
			BICS	R0,R0,R1
			MSR		APSR,R0
			POP		{R1}
			POP		{R0}
			B		DIVUReturn		;Clear Carry flag, Return
			
DIVUInit	PUSH	{R2}			;Store R2 value in stack
			MOVS	R2,#0			;Set R2 (quotient counting register) to zero
			CMP		R1,R0			;Set flags for R1 - R0 (Dividend - Divisor)
			
DIVULoop	BLO		DIVUEndLoop		;while (Dividend >= Divisor) {
DIVUBody	ADDS	R2,R2,#1		;	Quotient += 1
			SUBS	R1,R1,R0		;	Dividend = Dividend - Divisor
			CMP		R1,R0			;
			B		DIVULoop		;}

DIVUEndLoop	MOVS	R0,R2			;Move Quotient (R2) to output (R0)
			POP		{R2}			;Retrieve preserved R2 value from stack
			PUSH	{R0}			;Clear carry
			PUSH	{R1}
			MRS		R0,APSR
			MOVS	R1,#0x20
			LSLS	R1,R1,#24
			BICS	R0,R0,R1
			MSR		APSR,R0
			POP		{R1}
			POP		{R0}
DIVUReturn	BX		LR
			ENDP
;end DIVU PROC
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
            IMPORT  HardFault_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    HardFault_Handler  ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendableSrvReq (pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 xfer complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 xfer complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 xfer complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 xfer complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:command complete; read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:I2C1
            DCD    Dummy_Handler      ;26:SPI0 (all IRQ sources)
            DCD    Dummy_Handler      ;27:SPI1 (all IRQ sources)
            DCD    UART0_ISR	      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:UART1 (status; error)
            DCD    Dummy_Handler      ;30:UART2 (status; error)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:TPM2
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    PIT_ISR		      ;38:PIT (all IRQ sources)
            DCD    Dummy_Handler      ;39:I2S0
            DCD    Dummy_Handler      ;40:USB0
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:Segment LCD
            DCD    Dummy_Handler      ;46:PORTA pin detect
            DCD    Dummy_Handler      ;47:PORTC and PORTD pin detect
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
MAIN_PRM		DCB		"Enter the access code.", NULL
GRANTED			DCB		"--Access granted", NULL
DENIED			DCB		"--Access denied", NULL
COMPLETED		DCB		"Mission completed!", NULL
COUNT_INFO		DCB		" x 0.01 s", NULL
PASSWORD		DCB		"opensesame", NULL
CRLF			DCB		CR, LF, NULL

;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
;Queue struture addresses
RxQRecord		SPACE	Q_REC_SZ
RxQBuffer		SPACE	Q_BUF_SZ
TxQRecord		SPACE	Q_REC_SZ
TxQBuffer		SPACE	Q_BUF_SZ
;timer variables
Count			SPACE	4
RunStopWatch	SPACE	1
;Output string address
OutStr			SPACE	MAX_STRING
;>>>>>   end variables here <<<<<
            ALIGN
            END