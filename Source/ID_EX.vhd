library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.std_logic_unsigned.all;
use IEEE.NUMERIC_STD.ALL;

library work;
use work.custom_types.all;

entity ID_EX is
port (
    --Inputs
    RST : in std_logic;
    CLK : in std_logic;
    MNEMONIC_IN : in MNEMONIC_TYPE;                         --Instruction being processed
    INPUT_PORT_IN : in std_logic_vector(15 downto 0);
    -- ALU Inputs
    ALU_MODE_IN : in std_logic_vector(2 downto 0);
    INPUT_A_DATA_IN : in std_logic_vector(15 downto 0);     --Input A for ALU
    INPUT_B_DATA_IN : in std_logic_vector(15 downto 0);     --Input B for ALU
    SHFT_IN : in std_logic_vector(3 downto 0);
    -- Branch Inputs
    DISPLACEMENT_IN : in std_logic_vector(15 downto 0);     --Displacement of branch instructions
    PC_IN : in std_logic_vector(15 downto 0);               --PC (Current Instr Addr)
    -- Reg and Mem Inputs
    REG_WR_ADDR_IN : in std_logic_vector(2 downto 0);       --Stores Reg Address
    REG_WR_EN_IN : in std_logic;
    REG_SOURCE_SEL_IN : in std_logic;
    IMM_DATA_IN : in std_logic_vector(7 downto 0);
    IMM_UP_SIDE_SEL_IN : in std_logic;
    MEM_WR_EN_IN : in std_logic;
    -- Hazard handling inputs
    ALU_FW_IN : in FW_TYPE;
    ALU_FW_DATA_IN : in std_logic_vector(15 downto 0);
    EX_FW_IN : in FW_TYPE;
    EX_FW_DATA_IN : in std_logic_vector(15 downto 0);
    MEM_FW_IN : in FW_TYPE;
    MEM_FW_DATA_IN : in std_logic_vector(15 downto 0);
    
    -- OUTPUTS
    MNEMONIC_OUT : out MNEMONIC_TYPE;                       --Instruction being processed
    OUTPUT_PORT_OUT : out std_logic_vector(15 downto 0);
    -- ALU outputs
    ALU_MODE_OUT : out std_logic_vector(2 downto 0);
    INPUT_A_DATA_OUT : out std_logic_vector(15 downto 0);
    INPUT_B_DATA_OUT : out std_logic_vector(15 downto 0);
    SHFT_OUT : out std_logic_vector(3 downto 0);
    -- Branch outputs
    PC_OUT : out std_logic_vector(15 downto 0);
    -- Reg and Mem Outputs
    REG_WR_ADDR_OUT : out std_logic_vector(2 downto 0);        --Stores Reg Address
    REG_WR_EN_OUT : out std_logic;
    REG_SOURCE_SEL_OUT : out std_logic;
    ALU_BYPASS_OUT : out std_logic_vector(15 downto 0);
    IMM_UP_SIDE_SEL_OUT : out std_logic;
    MEM_ADDR_OUT : out std_logic_vector(15 downto 0);
    MEM_WR_EN_OUT : out std_logic);
end ID_EX;

architecture Behavioral of ID_EX is

signal mnemonic: MNEMONIC_TYPE; 
signal out_port : std_logic_vector(15 downto 0);

-- signals for ALU
signal alu_mode : std_logic_vector(2 downto 0);       
signal alu_input_a : std_logic_vector(15 downto 0);   
signal alu_input_b : std_logic_vector(15 downto 0); 
signal shft_val : std_logic_vector(3 downto 0);
 
-- signals for branching
signal pc : std_logic_vector(15 downto 0);

-- signals for Mem/Reg
signal reg_wr_addr : std_logic_vector(2 downto 0);
signal reg_wr_en : std_logic;
signal reg_source_sel : std_logic;
signal alu_bypass : std_logic_vector(15 downto 0);
signal imm_data : std_logic_vector(15 downto 0) := x"0000";
signal imm_up_side_sel : std_logic;
signal mem_addr : std_logic_vector(15 downto 0);
signal mem_wr_en : std_logic;

begin

-- basic latch assignments for alu inputs and register store values
mnemonic <= MNEMONIC_IN;
out_port <= alu_input_a when(mnemonic = OUT_I) else x"0000";

alu_mode <= ALU_MODE_IN;
alu_input_a <=  PC_IN when( (mnemonic = BRR)    or 
                            (mnemonic = BRR_N)  or 
                            (mnemonic = BRR_Z))  else
                ALU_FW_DATA_IN when(ALU_FW_IN = FW_INPUT_A or ALU_FW_IN = FW_BOTH) else
                EX_FW_DATA_IN when(EX_FW_IN = FW_INPUT_A or EX_FW_IN = FW_BOTH) else
                MEM_FW_DATA_IN when(MEM_FW_IN = FW_INPUT_A or MEM_FW_IN = FW_BOTH) else
                INPUT_A_DATA_IN;
alu_input_b <=  alu_input_a when(ALU_FW_IN = FW_BOTH or EX_FW_IN = FW_BOTH or MEM_FW_IN = FW_BOTH) else
                DISPLACEMENT_IN when(   (mnemonic = BRR)    or 
                                        (mnemonic = BRR_N)  or 
                                        (mnemonic = BRR_Z)  or
                                        (mnemonic = BR)     or 
                                        (mnemonic = BR_Z)   or
                                        (mnemonic = BR_N)   or 
                                        (mnemonic = BR_SUB)) else
                ALU_FW_DATA_IN when(ALU_FW_IN = FW_INPUT_B) else
                EX_FW_DATA_IN when(EX_FW_IN = FW_INPUT_B) else
                MEM_FW_DATA_IN when(MEM_FW_IN = FW_INPUT_B) else
                INPUT_B_DATA_IN;
shft_val <= SHFT_IN;
pc <= PC_IN;
reg_wr_addr <= REG_WR_ADDR_IN;
reg_wr_en <= REG_WR_EN_IN;
reg_source_sel <= REG_SOURCE_SEL_IN;
imm_data(7 downto 0) <= IMM_DATA_IN;
alu_bypass <=   alu_input_a when(mnemonic = MOV or mnemonic = STORE or mnemonic = RETURN_I) else   -- register value bypasses alu for MOV, STORE, and RETURN instructions
                imm_data when(mnemonic = LOADIMM) else                      -- imm data bypasses alu for LOADIMM instructions
                INPUT_PORT_IN when(mnemonic = IN_I) else x"0000";           -- input port value bypasses alu for IN instructions
imm_up_side_sel <= IMM_UP_SIDE_SEL_IN;
mem_addr <= alu_input_b when((mnemonic = LOAD or mnemonic = STORE)) else x"0000";      -- mem address is input_B for LOAD and STORE instructions
mem_wr_en <= MEM_WR_EN_IN;

    process(CLK, RST)
    begin
        if(RST = '1') then
            MNEMONIC_OUT <= NOP;
            OUTPUT_PORT_OUT <= x"0000";
            -- ALU outputs
            ALU_MODE_OUT <= "000";
            INPUT_A_DATA_OUT <= "0000000000000000";
            INPUT_B_DATA_OUT <= "0000000000000000";
            SHFT_OUT <= "0000";
            -- branch outputs
            PC_OUT <= x"0000";
            -- reg and mem outputs
            REG_WR_ADDR_OUT <= "000";
            REG_WR_EN_OUT <= '0';
            REG_SOURCE_SEL_OUT <= reg_source_sel;
		    IMM_UP_SIDE_SEL_OUT <= '0';
		    ALU_BYPASS_OUT <= x"0000";
            MEM_ADDR_OUT <= x"0000";
            MEM_WR_EN_OUT <= '0';
            
        elsif falling_edge(CLK) then
            MNEMONIC_OUT <= mnemonic;
            OUTPUT_PORT_OUT <= out_port;
            -- ALU outputs
            ALU_MODE_OUT <= alu_mode;
            INPUT_A_DATA_OUT <= alu_input_a; 
            INPUT_B_DATA_OUT <= alu_input_b;
            SHFT_OUT <= shft_val;
            -- branch outputs
            PC_OUT <= pc;
            -- reg and mem outputs
            REG_WR_ADDR_OUT <= reg_wr_addr;
            REG_WR_EN_OUT <= reg_wr_en;
            REG_SOURCE_SEL_OUT <= reg_source_sel;
            IMM_UP_SIDE_SEL_OUT <= imm_up_side_sel;
            ALU_BYPASS_OUT <= alu_bypass;
            MEM_ADDR_OUT <= mem_addr;
            MEM_WR_EN_OUT <= mem_wr_en;
        end if;
    end process;
end Behavioral;