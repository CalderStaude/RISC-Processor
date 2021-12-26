--Execute and Memory Access Latch
--Rev 3
--March 28 2021

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

library work;
use work.custom_types.all;

entity EX_MEM is
port (
    --Inputs
    RST : in std_logic;
    CLK : in std_logic;
    MNEMONIC_IN: in MNEMONIC_TYPE;                      --Instruction being processed
    -- ALU result inputs
    ALU_BYPASS_IN : in std_logic_vector(15 downto 0);
    ALU_RESULT_IN : in std_logic_vector(15 downto 0);
    Z_FLAG: in std_logic;                               --Zero flag                              
    N_FLAG: in std_logic;                               --Negative flag
    O_FLAG: in std_Logic;                               --Overflow flag
    -- Branching inputs
    PC_IN : in std_logic_vector(15 downto 0);           --PC (Current Instr Addr)
    -- Reg/mem input signals
    REG_WR_ADDR_IN : in std_logic_vector(2 downto 0);
    REG_WR_EN_IN : in std_logic;
    REG_SOURCE_SEL_IN : in std_logic;
    IMM_UP_SIDE_SEL_IN : in std_logic;
    MEM_ADDR_IN : in std_logic_vector(15 downto 0);
    MEM_WR_EN_IN : in std_logic;
    
    -- OUTPUTS
    MNEMONIC_OUT : out MNEMONIC_TYPE;
    BR_TKN_OUT : out std_logic;
    DESTINATION_ADDRESS_OUT : out std_logic_vector(15 downto 0);
    ALU_RESULT_OUT : out std_logic_vector(15 downto 0);
    REG_WR_ADDR_OUT : out std_logic_vector(2 downto 0);
    REG_WR_EN_OUT : out std_logic;
    REG_SOURCE_SEL_OUT : out std_logic;
    IMM_UP_SIDE_SEL_OUT : out std_logic;
    MEM_ADDR_OUT : out std_logic_vector(15 downto 0);
    MEM_WR_EN_OUT : out std_logic;
    ALU_FW_DATA_OUT : out std_logic_vector(15 downto 0));
end EX_MEM;

architecture Behavioral of EX_MEM is

--signals for ALU   
signal mnemonic : MNEMONIC_TYPE;
signal br_tkn : std_logic;
signal destination_address : std_logic_vector(15 downto 0);
signal alu_result : std_logic_vector(15 downto 0);
signal reg_wr_addr : std_logic_vector(2 downto 0);
signal reg_wr_en : std_logic;
signal reg_source_sel : std_logic;
signal imm_up_side_sel : std_logic;
signal mem_addr : std_logic_vector(15 downto 0);
signal mem_wr_en : std_logic;

begin

mnemonic <= MNEMONIC_IN;
br_tkn <= '1' when  (mnemonic = BRR) or 
                    (mnemonic = BR) or
                    ((mnemonic = BRR_N) and (N_FLAG = '1')) or
                    ((mnemonic = BRR_Z) and (Z_FLAG = '1')) or
                    ((mnemonic = BR_N) and (N_FLAG = '1')) or
                    ((mnemonic = BR_Z) and (Z_FLAG = '1')) or
                    (mnemonic = RETURN_I) or
                    (mnemonic = BR_SUB) else '0';
destination_address <= ALU_RESULT_IN when(br_tkn = '1' and mnemonic /= RETURN_I) else
                       ALU_BYPASS_IN when(br_tkn = '1' and mnemonic = RETURN_I) else x"0000";
alu_result <=   PC_IN when(mnemonic = BR_SUB) else              -- alu_result is PC for branch instructions so it can be stored in the reg file as the return addr, otherwise it's ALU_RESULT
                ALU_BYPASS_IN when(ALU_BYPASS_IN /= x"0000") else ALU_RESULT_IN;
reg_wr_addr <= REG_WR_ADDR_IN;
reg_wr_en <= REG_WR_EN_IN;
reg_source_sel <= REG_SOURCE_SEL_IN;
imm_up_side_sel <= IMM_UP_SIDE_SEL_IN;
mem_addr <= MEM_ADDR_IN;
mem_wr_en <= MEM_WR_EN_IN;

Process(CLK, RST)
begin
    if (RST = '1') then
        MNEMONIC_OUT <= NOP;
        BR_TKN_OUT <= '0';
        DESTINATION_ADDRESS_OUT <= x"0000";
        ALU_RESULT_OUT <= x"0000";
        REG_WR_ADDR_OUT <= "000";
        REG_WR_EN_OUT <= '0';
        REG_SOURCE_SEL_OUT <= '0';
        IMM_UP_SIDE_SEL_OUT <= '0';
        MEM_ADDR_OUT <= x"0000";
        MEM_WR_EN_OUT <= '0';
    elsif falling_edge(CLK) then
        BR_TKN_OUT <= br_tkn;
        DESTINATION_ADDRESS_OUT <= destination_address;
        if(Br_Tkn = '1') then
            MNEMONIC_OUT <= NOP;
	        ALU_RESULT_OUT <= x"0000";
	        REG_WR_ADDR_OUT <= "000";
	        REG_WR_EN_OUT <= '0';
	        REG_SOURCE_SEL_OUT <= '0';
	        IMM_UP_SIDE_SEL_OUT <= '0';
	        MEM_ADDR_OUT <= x"0000";
	        MEM_WR_EN_OUT <= '0';
        else
            MNEMONIC_OUT <= mnemonic;
	        ALU_RESULT_OUT <= alu_result;
	        REG_WR_ADDR_OUT <= reg_wr_addr;
	        REG_WR_EN_OUT <= reg_wr_en;
	        REG_SOURCE_SEL_OUT <= reg_source_sel;
	        IMM_UP_SIDE_SEL_OUT <= imm_up_side_sel;
	        MEM_ADDR_OUT <= mem_addr;
	        MEM_WR_EN_OUT <= mem_wr_en;
    end if;
    end if;
end Process;

ALU_FW_DATA_OUT <= alu_result;   -- asynchronous forward alu_result (which includes alu_bypass) to IF_ID

end Behavioral;
