LedDisplay:	equ		0xFFF2
DipSwitches:	equ		0xFFF0


;.DATA
;.CODE
		org		0x456
start:		loadimm.upper	LedDisplay.hi
		loadimm.lower	LedDisplay.lo
		mov		r5, r7
		loadimm.upper	0x00
		loadimm.lower	0x00
		mov		r6, r7
loop:		nop
		nop
		nop
		store		r5, r6
		loadimm.upper	0x00
		loadimm.lower	0x01
		add		r6, r6, r7

		loadimm.lower	0x0F
		nand		r4, r6, r7
		nand		r4, r4, r4
		loadimm.lower	0x0a
		sub		r4, r4, r7
		test		r4
		brr.z		ones_zero
		brr		loop	

ones_zero:	loadimm.upper	0xff
		loadimm.lower	0xf0
		nand		r6, r6, r7
		nand		r6, r6, r6

		loadimm.upper	0x00
		loadimm.lower	0x10
		add		r6, r6, r7

		loadimm.upper	0x00
		loadimm.lower	0xf0
		nand		r4, r6, r7
		nand		r4, r4, r4

		loadimm.upper	0x00
		loadimm.lower	0xA0
		sub		r4, r4, r7
		test		r4
		brr.z		tens_zero
		brr		loop

tens_zero:	loadimm.upper	0x0F
		loadimm.lower	0x00
		nand		r6, r6, r7
		nand		r6, r6, r6		

		loadimm.upper	0x01
		loadimm.lower	0x00
		add		r6, r6, r7

		loadimm.upper	0x0A
		sub		r4, r6, r7
		test		r4
		brr.z		start
		brr		loop


