;
; Version 1.00
;

; This code is the bootloader for ECE 449 course project.
; It reads user's object file, byte by byte, and stores it 
; into the RAM module. 
;
; When the board powers up, the CPU should fetch intsructions
; from this ROM unit (botloader). It waits to receive 0xAAAA 
; on the input port. When the loader programm is run on the PC, 
; it first sends 0xAAAA to indicate the start of the object file
; transfer. This follows by 0x0000 to reset the data and wait for 
; a valid data.
;
; After the start of transfer is initiated, the CPU reads the 
; first byte that is the size of the object file in bytes.
; After the completion of the transfer, the CPU should start 
; executing instructions from the RAM. To switch from ROM to RAM,
; the MSB of the PC (the 9th bit) is connected to the enable pins
; of the ROM and RAM. If it is '0', the ROM is enable, if it is '1'
; the RAM will be accessed. Therefore, at the end of th etransfer, the 
; (0x100+ code section start address) is added to the PC to enable the RAM and 
; point the PC to the start of the code section in the memory.
; So, after the transfer of object file to the RAM is complete, one more 
; byte is received which is the start of the code section.
;
; Register Usage:
;
; r7 - Temporary register
; r6 - Bit mask for load signal
; r5 - Pointer to Display
; r4 - Number of bytes in program ( maximum 255 )
; r3 - Address to store the program
; r2 - Data from port 
; r1 - 
; r0 -
;


RamStart:	equ		0x0400
LedDisplay:	equ		0xFFF2

;
; Word addressable constants
;

;BootVector:	equ		0x0400
;BootVector_1:	equ		0x0401
;BootVector_2:	equ		0x0402
;StepSize:	equ		0x0001

;
; Byte addressable constants
;

BootVector:	equ		0x0400
BootVector_1:	equ		0x0402
BootVector_2:	equ		0x0404
StepSize:	equ		0x0002




;++
;
;
; Reset and interrupt vectors
;
;
;++
		org		0x0000
		brr		ResetExecute
		brr		ResetLoad		
		brr		Interrupt

Interrupt:
WaitForever:	brr		WaitForever	

;++
;
;
; Reset Execute test code
;
;
;++

;
; Verify that the jump instructions are valid
;

	
ResetExecute:	loadimm.upper	BootVector.hi
		loadimm.lower	BootVector.lo
		load		r7, r7
		nop
		nop
		loadimm.lower	0x00
		mov		r2, r7
		loadimm.upper	0x25
		loadimm.lower	0x00
		sub		r2, r2, r7
		test		r2
		brr.z		ResetExecute_1
		brr		WaitForever
ResetExecute_1:
		loadimm.upper	BootVector_1.hi
		loadimm.lower	BootVector_1.lo
		load		r7, r7
		nop
		nop
		loadimm.lower	0x00
		mov		r2, r7
		loadimm.upper	0x24
		loadimm.lower	0x00
		sub		r2, r2, r7
		test		r2
		brr.z		ResetExecute_2
		brr		WaitForever

ResetExecute_2:	
		loadimm.upper	BootVector_2.hi
		loadimm.lower	BootVector_2.lo
		load		r2, r7
		nop
		nop
		loadimm.upper	0x87
		loadimm.lower	0xc0
		sub		r2, r2, r7
		test		r2
		brr.z		ResetExecute_3
		brr		WaitForever

ResetExecute_3:
		loadimm.upper	BootVector.hi
		loadimm.lower	BootVector.lo
		br		r7,0

;+
;
; Bootloader code
;
;
;+

ResetLoad:	loadimm.upper	0x00			; Bit mask for external data available signal
		loadimm.lower	0x80
		mov		r6, r7

;+
;
;
; Wait for start of transfer header (0xAA)
;
;
;+

WaitFor_AA:	in		r2			; wait for data available to go high
		nand		r2, r2, r6
		nand		r2, r2, r2
		test		r2
		brr.z		WaitFor_AA

		loadimm.upper	0xFF
		loadimm.lower	0x00
		in		r2
		nand		r2, r2, r7
		nand		r2, r2, r2
		LOADIMM.UPPER	0xAA
		sub		r2, r2, r7
		test		r2
		brr.z		Got_AA
		brr		WaitFor_AA

Got_AA:		loadimm.lower	0x01
		nop
		nop
		nop
		out		r7

WaitForEnd_AA:
		in		r2
		nand		r2, r2, r6
		nand		r2, r2, r2	
		test		r2
		brr.z		Done_AA
		brr		WaitForEnd_AA

Done_AA:	loadimm.lower	0x00
		nop
		nop
		nop
		out		r7


		loadimm.upper	RamStart.hi
		loadimm.lower	RamStart.lo
		mov		r6, r7
		loadimm.upper	0x00
		loadimm.lower	0x02
		store		r6, r7

;++
;
;
; Wait for second part of the header 0x55
;
;
;++

		loadimm.upper	0x00			; Load data bit
		loadimm.lower	0x80
		mov		r6, r7

WaitFor_55:	in		r2			; wait for load signal
		nand		r2, r2, r6
		nand		r2, r2, r2
		test		r2
		brr.z		WaitFor_55

		loadimm.upper	0xFF
		loadimm.lower	0x00
		in		r2
		nand		r2, r2, r7
		nand		r2, r2, r2
		LOADIMM.UPPER	0x55
		sub		r2, r2, r7
		test		r2
		brr.z		Got_55
		brr		WaitFor_55

Got_55:		loadimm.lower	0x01
		nop
		nop
		nop
		out		r7


WaitForEnd_55:
		in		r2
		nand		r2, r2, r6
		nand		r2, r2, r2	
		test		r2
		brr.z		Done_55
		brr		WaitForEnd_55

Done_55:	loadimm.lower	0x00
		nop
		nop
		nop
		out		r7

		loadimm.upper	0x00
		loadimm.lower	0x80
		mov		r6, r7

;++
;
;
; Get the size of the program ( maximum 255 packets )
;
;
;++

WaitForSize:	in		r2			; wait for load signal
		mov		r4, r2
		nand		r2, r2, r6
		nand		r2, r2, r2
		test		r2
		brr.z		WaitForSize

		shr		r4, 8			; R4 has the size of the program

		loadimm.lower	0x01
		nop
		nop
		nop
		out		r7


WaitForSizeEnd:
		in		r2
		nand		r2, r2, r6
		nand		r2, r2, r2
		test		r2
		brr.z		DoneSize
		brr		WaitForSizeEnd

DoneSize:	loadimm.lower	0x00
		nop
		nop
		nop
		out		r7

		loadimm.upper	0x02			; Address to store the code
		loadimm.lower	0x00
		mov		r3, r7

;++
;
;
; Get the program and load address ( maximum 255 packets )
;
;
;++


GetProgram:	loadimm.upper	LedDisplay.hi		; Display Number of packets left to download
		loadimm.lower	LedDisplay.lo
		nop			
		nop
		nop
		nop
		nop
		store		r7, r4

		test		r4			; Transfer complete ?
		brr.z		WaitForever		; Done the transfer so halt the app

WaitForHighByte:
		in		r2			; wait for load signal
		mov		r1, r2
		nand		r2, r2, r6
		nand		r2, r2, r2
		test		r2
		brr.z		WaitForHighByte

		shr		r1, 8			; Clear the lower half of r1
		shl		r1, 8

		loadimm.lower	0x01
		nop
		nop
		nop
		out		r7


WaitForHighByteEnd:
		in		r2
		nand		r2, r2, r6
		nand		r2, r2, r2
		test		r2
		brr.z		DoneHighByte
		brr		WaitForHighByteEnd

DoneHighByte:	
		loadimm.lower	0x00
		nop
		nop
		nop
		out		r7
		mov		r0, r1			; save the upper 8 bits of the instruction

WaitForLowByte:
		in		r2			; wait for load signal
		mov		r1, r2
		nand		r2, r2, r6
		nand		r2, r2, r2
		test		r2
		brr.z		WaitForLowByte

		shr		r1, 8			; R1 has the lower 8 bits of the instruction

		loadimm.lower	0x01
		nop
		nop
		nop
		out		r7

WaitForLowByteEnd:
		in		r2
		nand		r2, r2, r6
		nand		r2, r2, r2
		test		r2
		brr.z		DoneLowByte
		brr		WaitForLowByteEnd

DoneLowByte: 	in		r2			; See if the packet contains an address or instruction
		loadimm.lower	0x00
		nop
		nop
		nop
		out		r7

		add		r1, r1, r0		; merge the upper and lower bytes of the instruction
		shl		r2, 9			; test to see if the packet is and address or instruction
		shr		r2, 15
		test		r2
		brr.z		GotInstruction

		mov		r3, r1			; Got starting address of program

;
; Create a jump vector to the origin of the test code
;
		loadimm.upper	StepSize.hi
		loadimm.lower	StepSize.lo
		mov		r2, r7

		loadimm.upper	BootVector.hi		; Location of the vector
		loadimm.lower	BootVector.lo
		mov		r0, r7

		mov		r7, r3			; loadimm.upper instruction
		shr		r7, 8
		loadimm.upper	0x25
		store		r0, r7

		add		r0, r0, r2

		mov		r7, r3			; loaddimm.lower instruction
		loadimm.upper	0x24
		store		r0, r7

		add		r0, r0, r2		; brr r7,0 instruction
		loadimm.upper	0x87
		loadimm.lower	0xC0
		store		r0, r7

		brr		DecrementCount

GotInstruction:	store		r3, r1

;		loadimm.upper	0xFF			; Display the instruction we just received
;		loadimm.lower	0xF2
;		store		r7, r1
		
		loadimm.upper	StepSize.hi
		loadimm.lower	StepSize.lo
		add		r3, r3, r7		; Advance to the next address

DecrementCount:
		loadimm.upper	0x00
		loadimm.lower	0x01
		sub		r4, r4, r7
		brr		GetProgram
							
		
