library IEEE;
use IEEE.STD_LOGIC_1164.all;

package custom_types is

    type REG_SOURCE_TYPE is (NONE, ALU, PC_PLUS2, MEMORY, IMM_REG, IN_PORT);
    type INSTRUCTION_TYPE is (A0, A1, A2, A3, B1, B2, B3, L1, L2, ERR);
    type FORMAT_TYPE is (A, B, L, ERR);
    type HAZARD_SOURCE is (NONE, ALU, MEM);
    type FW_TYPE is (NONE, FW_INPUT_A, FW_INPUT_B, FW_BOTH);
    type MNEMONIC_TYPE is (NOP, ADD, SUB, MUL, N_AND, S_HL, S_HR, TEST, OUT_I, IN_I, BRR, BRR_N, BRR_Z, BR, BR_N, BR_Z, BR_SUB, RETURN_I, LOAD, STORE, LOADIMM, MOV, ERR);

end custom_types;

package body custom_types is

end custom_types;
