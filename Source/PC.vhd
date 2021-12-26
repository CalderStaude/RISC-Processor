--PC for Format B
--Rev 2
--March 25

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;   --For Adder

entity PC is
Port ( 
        clk: in std_logic;
        RST_LD : in std_logic;
        RST_EX : in std_logic;
        BR_TKN_IN: in std_logic;                               --Flag for if a conditional branch should be taken
        DESTINATION_ADDRESS_IN: in std_logic_vector(15 downto 0);       -- The output of the ALU used for branching 
        STALL_IN: in std_logic;                                -- Set when a stall is required, stops PC from incrementing
        PC_OUT: out std_logic_vector(15 downto 0));            --The instruction address for the memery to get the next instruction and for the IF_ID latch for branch instructions
end PC;

architecture Behavioral of PC is
signal Instruction_Address: std_logic_vector(15 downto 0):= x"0000";
signal br_tkn : std_logic;
signal destination_address: std_logic_vector(15 downto 0);
begin
                     

process(clk, RST_EX, RST_LD)
begin   
if(RST_EX = '1') then
    Instruction_Address <= x"0000";
elsif(RST_LD = '1') then
    Instruction_Address <= x"0002";
else
    if falling_edge(clk) then   
        if(STALL_IN = '1') then
            -- NOP
        elsif (br_tkn = '1') then
            Instruction_Address <= destination_address;
        else
            Instruction_Address <= std_logic_vector(unsigned(Instruction_Address) + 2);
        end if;  
    end if;
    if rising_edge(clk) then
        br_tkn <= BR_TKN_IN;
        destination_address <= DESTINATION_ADDRESS_IN;
    end if;
end if;
end process;
PC_OUT <= Instruction_Address;
end Behavioral;
