--IF/ID Latch Logic
--Rev 2
--March 11th 2021

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.std_logic_unsigned.all;
use IEEE.NUMERIC_STD.ALL;

library work;
use work.custom_types.all;

-- VHDL code for Control Unit of the Processor
-- This code is responsible for parsing the instructions coming from mem and decoding them to preform the requested actions

entity IF_ID is
port (
    -- INPUTS
    PC_IN: in std_logic_vector(15 downto 0);
    INSTR_IN: in std_logic_vector(15 downto 0);
    CLK: in std_logic;
    RST: in std_logic;
    
    -- OUTPUTS
    MNEMONIC_OUT : out MNEMONIC_TYPE;
    -- Hazard handling
    STALL_OUT : out std_logic;                              -- stops PC from incrementing when stall necessary
    ALU_FW_OUT : out FW_TYPE;
    EX_FW_OUT : out FW_TYPE;
    MEM_FW_OUT : out FW_TYPE;
    -- ALU signals
    ALU_MODE_OUT : out std_logic_vector(2 downto 0);
    INPUT_A_ADDR_OUT : out std_logic_vector(2 downto 0);    -- ALU input A
    INPUT_B_ADDR_OUT : out std_logic_vector(2 downto 0);    -- ALU input B
    SHFT_OUT : out std_logic_vector(3 downto 0);            -- For Shift Operation
    -- REG and MEM signals
    REG_WR_ADDR_OUT : out std_logic_vector(2 downto 0);     -- reg_file write address for alu result or memory output
    REG_WR_EN_OUT : out std_logic;
    REG_SOURCE_SEL_OUT : out std_logic;                     -- select mem over alu for reg write source when set
    IMM_DATA_OUT : out std_logic_vector(7 downto 0);        -- the data passed in an immediate instruction for storing in r7
    IMM_UP_SIDE_SEL_OUT : out std_logic;                    -- select the upper or lower half of the register to write when doing a load immediate instruction
    MEM_WR_EN_OUT : out std_logic;
	-- Branching signals
    PC_OUT: out std_logic_vector(15 downto 0);
    DISPLACEMENT_OUT : out std_logic_vector(15 downto 0));
end IF_ID;

architecture Behavioral of IF_ID is

-- instruction type signals
signal mnemonic : MNEMONIC_TYPE;
signal format_t : FORMAT_TYPE;
signal instr_t : INSTRUCTION_TYPE;

-- signals to parse instruction into
signal op_code : std_logic_vector(6 downto 0);          --bits 15 to 9
signal alu_mode : std_logic_vector(2 downto 0);         --bits 3 to 0 of opcode
signal input_a_addr : std_logic_vector(2 downto 0);     --bits 5 to 4 or 8 to 6
signal input_b_addr : std_logic_vector(2 downto 0);     --bits 3 to 0
signal shift_val : std_logic_vector(3 downto 0);        --bits 3 to 0
signal reg_wr_addr : std_logic_vector(2 downto 0);       --bits 8 to 6
signal reg_wr_en : std_logic;
signal reg_source_sel : std_logic;
signal reg_in_select : REG_SOURCE_TYPE;
signal imm_data : std_logic_vector(7 downto 0);
signal imm_up_side_sel : std_logic;
signal mem_wr_en : std_logic;

-- branching signals
signal displacement_unshifted_B1 : std_logic_vector(8 downto 0);
signal displacement_unshifted_B2 : std_logic_vector(5 downto 0);
signal displacement_shifted : std_logic_vector(15 downto 0);

-- hazard detection
type reg_adr_array is array (integer range 0 to 1) of std_logic_vector(2 downto 0);
type instr_hist_array is array (integer range 0 to 1) of HAZARD_SOURCE;
signal reg_wr_history : reg_adr_array := (others=>(others=>'0'));
signal instr_history: instr_hist_array := (others=>NONE);
signal hazard_type : HAZARD_SOURCE;
signal stall : std_logic;
signal alu_fw : FW_TYPE;
signal ex_fw : FW_TYPE;
signal mem_fw : FW_TYPE;

begin

op_code <= INSTR_IN(15 downto 9);                                                               -- opcode bits 15 to 9
instr_t <=  A0 when(op_code = 0) else
            A1 when(op_code >= 1 and op_code <= 4) else
            A2 when(op_code >= 5 and op_code <= 6) else
            A3 when(op_code = 7 or op_code = 32 or op_code = 33) else
            B1 when(op_code >= 64 and op_code <= 66) else
            B2 when(op_code >= 67 and op_code <= 70) else
            B3 when(op_code = 71) else
            L1 when(op_code = 18) else 
            L2 when(op_code = 16 or op_code = 17 or op_code = 19) else ERR;
            
format_t <= A when(instr_t = A0 or instr_t = A1 or instr_t = A2 or instr_t = A3) else
            B when(instr_t = B1 or instr_t = B2) else
            L when(instr_t = L1 or instr_t = L2) else ERR;
            
hazard_type <=  ALU when(instr_t = A1 or instr_t = A2 or mnemonic = MOV or mnemonic = IN_I) else
                MEM when(mnemonic = LOAD or mnemonic = LOADIMM) else NONE;
            
mnemonic <= NOP when(op_code = 0) else
            ADD when(op_code = 1) else
            SUB when(op_code = 2) else
            MUL when(op_code = 3) else
            N_AND when(op_code = 4) else
            S_HL when(op_code = 5) else
            S_HR when(op_code = 6) else
            TEST when(op_code = 7) else
            OUT_I when(op_code = 32) else
            IN_I when(op_code = 33) else
            BRR when(op_code = 64) else
            BRR_N when(op_code = 65) else
            BRR_Z when(op_code = 66) else
            BR when(op_code = 67) else
            BR_N when(op_code = 68) else
            BR_Z when(op_code = 69) else
            BR_SUB when(op_code = 70) else
            RETURN_I when(op_code = 71) else
            LOAD when(op_code = 16) else
            STORE when(op_code = 17) else
            LOADIMM when(op_code = 18) else
            MOV when(op_code = 19) else ERR;

alu_mode <= op_code(2 downto 0) when(instr_t = A1 or instr_t = A2 or mnemonic = TEST) else              -- alu_mode sent when A1, A2 or TEST instruction
            "001" when (instr_t = B1 or instr_t = B2) else "000";                                       -- alu_mode is add for branch instructions
input_a_addr <= INSTR_IN(5 downto 3) when(instr_t = A1 or mnemonic = STORE or mnemonic = MOV) else        -- reg data_a is rb when A1, STORE, or MOVE instructions and ra when A2, TEST, or OUT instructions
                INSTR_IN(8 downto 6) when(instr_t = A2 or mnemonic = TEST or mnemonic = OUT_I or instr_t = B2) else
                "111" when(mnemonic = RETURN_I) else "000";
input_b_addr <= INSTR_IN(8 downto 6) when(mnemonic = STORE) else                                        -- reg data_b address is ra for STORE instructions, rb for LOAD instructions
                INSTR_IN(5 downto 3) when(mnemonic = LOAD) else                                        -- and rc for A1 instructions
                INSTR_IN(2 downto 0) when(instr_t = A1) else "000";
shift_val <= INSTR_IN(3 downto 0) when (instr_t = A2) else "0000";                                   -- alu shift value is 3 downto 0
reg_wr_addr <= INSTR_IN(8 downto 6) when(   instr_t = A1 or                                         -- store address is always input_a for arithmetic instructions
                                            instr_t = A2 or                                         -- and for LOAD, MOVE, and IN instructions
                                            mnemonic = LOAD or 
                                            mnemonic = MOV or 
                                            mnemonic = IN_I) else
               "111" when(mnemonic = BR_SUB or mnemonic = LOADIMM) else "000";                          -- store address is r7 for sub and LOADIMM instructions
reg_wr_en <= '1' when(  instr_t = A1 or                                                             -- register_write_enable set when A1, A2, IN or any L instruction except STORE
                        instr_t = A2 or 
                        mnemonic = IN_I or 
                        mnemonic = LOAD or 
                        mnemonic = LOADIMM or 
                        mnemonic = MOV or
                        mnemonic = BR_SUB) else '0';
reg_source_sel <= '1' when(mnemonic = LOAD) else '0';                                                   -- reg file will only be written to from mem when doing a LOAD instruction, otherwise source will be ALU_RESULT
imm_data <= INSTR_IN(7 downto 0) when(mnemonic = LOADIMM) else x"00";                                   -- imm data is passed if running load_imm instruction
imm_up_side_sel <= INSTR_IN(8) when(mnemonic = LOADIMM) else '0';                                                                         -- for load_imm instructions selects the upper or lower half of r7 to store the imm value
mem_wr_en <= '1' when(mnemonic = STORE) else '0';                                                       -- mem write is enabled only for STORE and OUT instructions
displacement_unshifted_B1 <= INSTR_IN(8 downto 0) when(instr_t = B1) else "000000000";
displacement_unshifted_B2 <= INSTR_IN(5 downto 0) when(instr_t = B2) else "000000";
displacement_shifted <= std_logic_vector(resize(signed(displacement_unshifted_B1) * 2, displacement_shifted'length)) when(instr_t = B1) else
                        std_logic_vector(resize(signed(displacement_unshifted_B2) * 2, displacement_shifted'length)) when(instr_t = B2) else x"0000";

-- hazard handling
stall <= '1' when(instr_history(0) = MEM and (reg_wr_history(0) = input_a_addr or reg_wr_history(0) = input_b_addr)) else '0';  -- need to stall when trying to pull from write address of LOAD, LOADIMM, and IN
alu_fw <=   FW_BOTH when(instr_history(0) = ALU and reg_wr_history(0) = input_a_addr and reg_wr_history(0) = input_b_addr) else
            FW_INPUT_A when(instr_history(0) = ALU and reg_wr_history(0) = input_a_addr) else
            FW_INPUT_B when(instr_history(0) = ALU and reg_wr_history(0) = input_b_addr) else NONE;
ex_fw <=    FW_BOTH when(instr_history(1) = ALU and reg_wr_history(1) = input_a_addr and reg_wr_history(1) = input_b_addr) else
            FW_INPUT_A when(instr_history(1) = ALU and reg_wr_history(1) = input_a_addr) else
            FW_INPUT_B when(instr_history(1) = ALU and reg_wr_history(1) = input_b_addr) else NONE;
mem_fw <=   FW_BOTH when(instr_history(1) = MEM and reg_wr_history(1) = input_a_addr and reg_wr_history(1) = input_b_addr) else
            FW_INPUT_A when(instr_history(1) = MEM and reg_wr_history(1) = input_a_addr) else
            FW_INPUT_B when(instr_history(1) = MEM and reg_wr_history(1) = input_b_addr) else NONE;

process(CLK, RST)
begin
    if(RST = '1') then
        ALU_MODE_OUT <= "000";
        INPUT_A_ADDR_OUT <= "000";
        INPUT_B_ADDR_OUT <= "000";
        SHFT_OUT <= "0000";
        REG_WR_ADDR_OUT <= "000";
        REG_WR_EN_OUT <= '0';
        REG_SOURCE_SEL_OUT <= '0';
        IMM_DATA_OUT <= x"00";
        IMM_UP_SIDE_SEL_OUT <= '0';
        MEM_WR_EN_OUT <= '0';
        ALU_FW_OUT <= NONE;
        EX_FW_OUT <= NONE;
        MEM_FW_OUT <= NONE;
        MNEMONIC_OUT <= NOP;
        DISPLACEMENT_OUT <= x"0000";
        PC_OUT <= x"0000";
        for i in 0 to 1 loop
            reg_wr_history(i)<= (others => '0'); 
            instr_history(i) <= NONE;
        end loop;
        
    elsif falling_edge(CLK) then
        if(stall = '1') then 
            ALU_MODE_OUT <= "000";
            INPUT_A_ADDR_OUT <= "000";
            INPUT_B_ADDR_OUT <= "000";
            SHFT_OUT <= "0000";
            REG_WR_ADDR_OUT <= "000";
            REG_WR_EN_OUT <= '0';
            REG_SOURCE_SEL_OUT <= '0';
            IMM_DATA_OUT <= x"00";
            IMM_UP_SIDE_SEL_OUT <= '0';
            MEM_WR_EN_OUT <= '0';
            ALU_FW_OUT <= NONE;
            EX_FW_OUT <= NONE;
            MEM_FW_OUT <= NONE;
            MNEMONIC_OUT <= NOP;
            DISPLACEMENT_OUT <= x"0000";
            PC_OUT <= PC_IN;
            reg_wr_history(1) <= reg_wr_history(0);
            reg_wr_history(0) <= "000";
            instr_history(1) <= instr_history(0);
            instr_history(0) <= NONE;
        else
	        ALU_MODE_OUT <= alu_mode;
	        INPUT_A_ADDR_OUT <= input_a_addr;
	        INPUT_B_ADDR_OUT <= input_b_addr;
	        SHFT_OUT <= shift_val;
	        REG_WR_ADDR_OUT <= reg_wr_addr;
	        REG_WR_EN_OUT <= reg_wr_en;
            REG_SOURCE_SEL_OUT <= reg_source_sel;
        	IMM_DATA_OUT <= imm_data;
            IMM_UP_SIDE_SEL_OUT <= imm_up_side_sel;
        	MEM_WR_EN_OUT <= mem_wr_en;
            ALU_FW_OUT <= alu_fw;
        	EX_FW_OUT <= ex_fw;
            MEM_FW_OUT <= mem_fw;
            MNEMONIC_OUT <= mnemonic;
            DISPLACEMENT_OUT <= displacement_shifted;
            PC_OUT <= PC_IN;
            reg_wr_history(1) <= reg_wr_history(0);
            reg_wr_history(0) <= reg_wr_addr;
            instr_history(1) <= instr_history(0);
            instr_history(0) <= hazard_type;
        end if;
        
        

    end if;
end process;

STALL_OUT <= stall;

end Behavioral;