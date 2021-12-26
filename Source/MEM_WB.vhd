
--Memory Access And Write Back Latch
--Rev 1
--March 8 2021

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library work;
use work.custom_types.all;

library work;
use work.custom_types.all;

entity MEM_WB is
port (
    --INPUTS
    RST : in std_logic;     --Reset the Latch
    CLK : in std_logic;     --Clock used to release data on rising edge
    MNEMONIC_IN : in MNEMONIC_TYPE;
    ALU_RESULT_IN : in std_logic_vector(15 downto 0);       -- Result from ALU operation
    MEM_DATA_IN : in std_logic_vector(15 downto 0);         -- result from memory load
    REG_WR_ADDR_IN : in std_logic_vector(2 downto 0);       -- Stores Reg Address
    REG_WR_EN_IN : in std_logic;                            -- enable register write
    REG_SOURCE_SEL_IN : in std_logic;                       -- select ALU or memory load to store in reg
    IMM_UP_SIDE_SEL_IN : in std_logic;
    IMM_REG_IN : in std_logic_vector(15 downto 0);          -- feedback from reg7 for LOADIMM instructions
    --OUTPUTS
    REG_WR_DATA_OUT : out std_logic_vector(15 downto 0);    -- the data to be stored in the register. Either ALU result, memory load or immediate value
    REG_WR_ADDR_OUT : out std_logic_vector(2 downto 0);
    REG_WR_EN_OUT : out std_logic;
    MEM_FW_DATA_OUT : out std_logic_vector(15 downto 0));   -- asynchronous mem wb for data hazard handling
end MEM_WB;

architecture Behavioral of MEM_WB is

signal mnemonic : MNEMONIC_TYPE;
signal reg_wr_data : std_logic_vector(15 downto 0);
signal reg_wr_addr : std_logic_vector(2 downto 0);    
signal reg_wr_en : std_logic;
signal imm_up_side_sel : std_logic;
signal imm_reg : std_logic_vector(15 downto 0);

begin

mnemonic <= MNEMONIC_IN;
reg_wr_data <=  imm_reg when(mnemonic = LOADIMM) else
                MEM_DATA_IN when(REG_SOURCE_SEL_IN = '1') else ALU_RESULT_IN;
reg_wr_addr <= REG_WR_ADDR_IN;
reg_wr_en <= REG_WR_EN_IN;
imm_up_side_sel <= IMM_UP_SIDE_SEL_IN;
imm_reg <=  ALU_RESULT_IN(7 downto 0) & IMM_REG_IN(7 downto 0) when(mnemonic = LOADIMM and imm_up_side_sel = '1') else  -- concatenate 8 bit IMM value with bottom half of reg7 if loading upper side
            IMM_REG_IN(15 downto 8) & ALU_RESULT_IN(7 downto 0) when(mnemonic = LOADIMM and imm_up_side_sel = '0');     -- concatenate top half og reg7 with 8 bit IMM value if loading lower side

Process(CLK, RST)
begin
    if (RST = '1') then                 --Reset local values then write them to the output
        REG_WR_DATA_OUT <= x"0000";
        REG_WR_ADDR_OUT <= "000";
        REG_WR_EN_OUT <= '0';       
    elsif falling_edge(CLK) then         --Write input into latch memory to be reteived by the next stage of the pipeline. 
        REG_WR_DATA_OUT <= reg_wr_data;
        REG_WR_ADDR_OUT <= reg_wr_addr;
        REG_WR_EN_OUT <= reg_wr_en;
    end if;
end Process;

MEM_FW_DATA_OUT <= imm_reg when(mnemonic = LOADIMM) else MEM_DATA_IN;

end Behavioral;