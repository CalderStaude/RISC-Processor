LedDisplay:	equ		0xFFF2
DipSwitches:	equ		0xFFF0
DipSwitchMask:	equ		07			; Binary multiple as a mask


main:		loadimm.upper	DipSwitches.hi
		loadimm.lower	DipSwitches.lo
		load		r6, r7


		loadimm.upper	DipSwitchMask.hi
		loadimm.lower	DipSwitchMask.lo

		nand		r6, r6, r7		; Nand the switch settings
		nand		r6, r6, r6		; one's compliment result to and it


		loadimm.upper	0x00				
		loadimm.lower	0x01
		mov		r4, r7
		mov		r3, r7

		test		r6
		brr.z		Done

		sub		r6, r6, r3		; 0 and 1 factorial should return 1

		test		r6
		brr.z		Done

		loadimm.upper	0x00
		loadimm.lower	0x02
		mov		r5, r7

loop:		mul		r4, r4, r5
		add		r5, r5, r3

		sub		r6, r6, r3
		test		r6
		brr.z		Done
		brr		loop

Done:		loadimm.upper	LedDisplay.hi
		loadimm.lower	LedDisplay.lo

		store		r7, r4
		brr		Done

		end


