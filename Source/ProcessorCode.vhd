-------------------------------------------
--slightly modified hw_cpucore VHDL File
-------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity hw_CpuCore is
port (
        display_clock: in STD_LOGIC;
        clock: in STD_LOGIC;
        local_leds: out std_logic_vector( 15 downto 0 );
        local_seven_segment: out std_logic_vector( 6 downto 0 );
        local_digit_select: out std_logic_vector( 3 downto 0 );
        in_port: in std_logic_vector( 15 downto 5 );            --changed from 5 to 0
        ack_signal: out std_logic;

        debug_clock : in std_logic;
        copy_transfer: in std_logic;
        data_in: in std_logic;
        data_out: out std_logic;
        
        AuxStep: out std_logic;
        AuxNext: out std_logic;
        AuxResume: out std_logic
);
end hw_CpuCore;

architecture Behavioral of hw_CpuCore is

component CPU is
	Port ( 
        IN_PORT : in std_logic_vector(15 downto 0);
        OUT_PORT : out std_logic_vector(15 downto 0);       
        MM_IN_PORT : in std_logic_vector(15 downto 0);      
        MM_OUT_PORT : out std_logic_vector(15 downto 0);    

		INSTRUCTIONS_OUT: out std_logic_vector(15 downto 0);  --For LED's
        ALU_RESULT_OUTPUT : out std_logic_vector(15 downto 0);
        PC_OUTPUT: out std_logic_vector(15 downto 0);	
		
        --Resets
        RST_EX: in std_logic;    --Reset and load (PC <= 0x0002)
        RST_LD: in std_logic;      --Reset and execute (PC <= 0x0000)

        leds: out std_logic_vector( 15 downto 0 );

        clock : in std_logic
);
end component CPU;

component display_controller is
port (
    clk, reset: in std_logic;                                   --clk is the 50Mhz system clock FPGA pin B8, reset is is FPGA pin B18
    hex3, hex2, hex1, hex0: in std_logic_vector(3 downto 0);    --hex3, hex2,hex1 and hex0 are four bit arrays used to store hex display character
    an: out std_logic_vector(3 downto 0);                       --an (an0,an1, an2, an3) four bit array that connect to the display transistors
    sseg: out std_logic_vector(6 downto 0)                      --sseg 7bit array that stores the individual segments of the hex display
);
end component display_controller;

signal instructions: std_logic_vector(15 downto 0);
signal pc: std_logic_vector(15 downto 0);
signal alu_result: std_logic_vector(15 downto 0);
signal Data_Display: std_logic_vector(15 downto 0);
	
signal input_stream: std_logic_vector( 39 downto 0 );
signal output_stream: std_logic_vector( 59 downto 0 );
signal input_count: unsigned ( 7 downto 0 ); 
signal output_count: unsigned ( 7 downto 0 ); 
signal Digit4: std_logic_vector( 6 downto 0 );
signal Digit3: std_logic_vector( 6 downto 0 );
signal Digit2: std_logic_vector( 6 downto 0 );
signal Digit1: std_logic_vector( 6 downto 0 );
signal HexKeyboard: std_logic_vector( 15 downto 0 );

signal dip_switches: std_logic_vector ( 15 downto 0 );
signal btnC: std_logic;
signal btnU: STD_LOGIC;
signal btnD: STD_LOGIC;
signal btnL: STD_LOGIC;
signal btnR: STD_LOGIC;
signal keyboard_col_data: std_logic_vector ( 3 downto 0 );
signal keyboard_row_data: std_logic_vector ( 3 downto 0 );

signal digit_select: STD_LOGIC_VECTOR( 3 downto 0 );
signal seven_segment: STD_LOGIC_VECTOR( 6 downto 0 ); 
signal leds: STD_LOGIC_VECTOR( 15 downto 0 );
signal JC0: STD_LOGIC;
signal ack: STD_LOGIC;

--For STM Board
signal mm_input_port : std_logic_vector(15 downto 0);
signal mm_output_port : std_logic_vector(15 downto 0);
signal cpu_input_port : std_logic_vector(15 downto 0);
signal cpu_output_port : std_logic_vector(15 downto 0);
signal disp_rst : std_logic;
--
-- Receive switch data
--

begin

cpu_input_port(15 downto 5) <= in_port;
cpu_input_port(4 downto 0) <= "00000";
mm_input_port <= dip_switches;
ack <= cpu_output_port(0);
disp_rst <= btnC or btnU;

Core: CPU port map
( 
        IN_PORT => cpu_input_port,
        OUT_PORT => cpu_output_port,
        MM_IN_PORT => mm_input_port,
        MM_OUT_PORT => mm_output_port,
        clock => clock, 
        RST_EX => btnC,
        RST_LD => btnU,
	ALU_RESULT_OUTPUT => alu_result,
        INSTRUCTIONS_OUT => instructions,
	PC_OUTPUT => pc,
        leds => leds
);

Display_Module: display_controller port map(
    --INPUTS
    clk => display_clock,                   --IN
    reset => disp_rst,                      --IN
    hex3 => Data_Display(15 downto 12),   --IN
    hex2 => Data_Display(11 downto 8),    --IN
    hex1 => Data_Display(7 downto 4),     --IN
    hex0 => Data_Display(3 downto 0),     --IN
    --OUTPUTS
    an => digit_select,                     --Out
    sseg => seven_segment                   --Out
);

    process( display_clock, digit_select, seven_segment )
    begin
        if ( rising_edge( display_clock )) then
            if ( digit_select = "1110" ) then
                Digit1 <= not seven_segment;
            end if;

            if ( digit_select = "1101" ) then
                Digit2 <= not seven_segment;
            end if;

            if ( digit_select = "1011" ) then
                Digit3 <= not seven_segment;
            end if;

            if ( digit_select = "0111" ) then
                Digit4 <= not seven_segment;
            end if;
        end if;
    end process;


    process( debug_clock, copy_transfer, data_in )
    begin
        if rising_edge( debug_clock ) then
            if ( copy_transfer = '0' ) then
                input_count <= x"00";
            else
                if ( input_count < 40 ) then
                    input_stream(39 downto 0 ) <= input_stream( 38 downto 0 ) & data_in;
                    input_count <= input_count + x"01";
                else
                    AuxStep <= input_stream( 39 );
                    AuxNext <= input_stream( 38 );
                    AuxResume <= input_stream( 37 );
                    dip_switches <= input_stream( 36 downto 21 );
                    btnU <= input_stream( 20 );
                    btnC <= input_stream( 19 );
                    btnD <= input_stream( 18 );
                    btnL <= input_stream( 17 );
                    btnR <= input_stream( 16 );
                    HexKeyboard <= input_stream( 15 downto 0 );
                end if;
            end if;

         end if;
    end process;

--
-- Send LED data
--

    process( debug_clock, copy_transfer )
    begin
        if rising_edge( debug_clock ) then
            if ( copy_transfer = '0' ) then
                output_count <= x"00";

                output_stream( 59 ) <= clock;
                output_stream( 58 ) <= in_port( 7 );
                output_stream( 57 ) <= ack;
                output_stream( 56 ) <= in_port( 6 ); 
                output_stream( 55 downto 48 ) <= in_port( 15 downto 8 );

                output_stream( 47 downto 32 ) <= leds;
                output_stream( 31 downto 24 ) <= "0" & Digit4;
                output_stream( 23 downto 16 ) <= "0" & Digit3;
                output_stream( 15 downto  8 ) <= "0" & Digit2;
                output_stream(  7 downto  0 ) <= "0" & Digit1;

             else
                if ( output_count < ( 59 + 1 )) then
                    output_stream(59 downto 0) <= output_stream(58 downto 0) & "0";
                    output_count <= output_count + x"01";
                end if;
            end if;

         end if;
    end process;

    process ( display_clock, keyboard_col_data, HexKeyboard )
    begin
        if ( keyboard_col_data = "0111" ) then
            keyboard_row_data <= not HexKeyboard( 15 downto 12 );

        elsif( keyboard_col_data = "1011" ) then
            keyboard_row_data <= not HexKeyboard( 11 downto 8 );

        elsif( keyboard_col_data = "1101" ) then
            keyboard_row_data <= not HexKeyboard(  7 downto 4 );

        elsif ( keyboard_col_data = "1110" ) then
            keyboard_row_data <= not HexKeyboard( 3 downto 0 );

        else
            keyboard_row_data <= "1111";
        end if;
     end process;

     process(btnD, btnL, btnR)
    	begin
    
	    if (btnD = '0') and (btnL = '0') and (btnR = '0') then
		leds <= pc;          
		Data_Display <= mm_output_port;
	    end if;
	    if (btnD = '1') and (btnL = '0') and (btnR = '0') then
		leds <= instructions;             
		Data_Display <= instructions;
	    end if;
	    if (btnD = '0') and (btnL = '1') and (btnR = '0') then
		leds <= instructions;             
		Data_Display <= pc;
	    end if;
	    if (btnD = '0') and (btnL = '0') and (btnR = '1') then
		leds <= instructions;            
		Data_Display <= alu_result;
	    end if;
       end process;
			
    data_out <= output_stream(59);
    local_digit_select <= digit_select;
    local_seven_segment <= seven_segment; 
    local_leds <= leds;
    ack_signal <= ack;

end Behavioral;



-------------------------------------------
--unmodified 7 Segment Display Controller VHDL File
-------------------------------------------
--<--------------------PROVIDED CODE - DO NOT EDIT ------------------------------>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- clk is the 50Mhz system clock FPGA pin B8
-- reset is is FPGA pin B18
--hex3, hex2,hex1 and hex0 are four bit arrays used to store hex display character
--an (an0,an1, an2, an3) four bit array that connect to the display transistors
--sseg 7bit array that stores the individual segments of the hex display
-- refer to ucf file and Nexys2 manual

entity display_controller is
	port(
		clk, reset: in std_logic;
		hex3, hex2, hex1, hex0: in std_logic_vector(3 downto 0);
		an: out std_logic_vector(3 downto 0);
		sseg: out std_logic_vector(6 downto 0)
	);
end display_controller;

-- This architecture below describes the behavior of the multiplexer using its individual processes as well as
-- its seven-segment decoder to display numbers on the anodes using cathodes. 

architecture arch of display_controller is 
	
	-- each 7-seg led enabled (2^18/4)*25 ns (40 ms)
	constant N: integer:=18;
	
	-- These signals are operations within the FPGA that help the processes overall but that we cannot see.
	--q_reg and q_next are 18 bit arrays, in this application N=18
        --sel is a 2 bit array that selects the hex display
        --hex is a 4 bit array the contains the hex value to be displayed

	signal q_reg, q_next: unsigned(1 downto 0);
	signal sel: std_logic_vector(1 downto 0);
	signal hex: std_logic_vector(3 downto 0);


begin

	-- This process controls the reset button of the clock.

	process(clk, reset)
	begin
		if reset='1' then                       --If button pressed all  bits in q_reg set to 0
			q_reg <= (others=>'0');
		elsif (clk'event and clk='1') then       --If clock is rising q_next assigned to q_reg
			q_reg <= q_next;
		end if;
	end process;
	
	-- State logic for the counter                  --Increment q_reg and assign to q_next
	q_next <= q_reg + 1;
	
	-- 2 MSBs of counter to control 4-to-1 multiplexing 
	--assign bits17 and 16 to sel array
        sel <= std_logic_vector(q_reg);   
	
	-- This is the 2:4 decoder, which converts a two-bit input into a four-bit input for the seven-segment decoder.
	
        --The value in the sel array will determine which of the Case statements is selected. Since 
        -- the sel array is two bits then there are only four possible cases 0,1,2,3.
        --The value in an will determine which transistor is turned on that selects one of four hex
        --displays.
        --The hex0, hex1, hex2, and hex3 value comes from the Counter.vhd file

	process(sel, hex0, hex1, hex2, hex3)
	begin
		case sel is
			when "00" =>
				an <= "1110";
				hex <= hex0;
			when "01" =>
				an <= "1101";
				hex <= hex1;
			when "10" =>
				an <= "1011";
				hex <= hex2;
			when others =>
				an <= "0111";
				hex <= hex3;
				
		end case;
	end process;
	
	-- The value that was assigned to the hex array is used to assign the sseg array. Example: If   
        -- the hex array contained the value 1001 (9) then the sseg array would be assigned the 
        -- value 0010000 which would display a 9 on the selected hex display. See reference 
        --if needed
	
	with hex select
		sseg(6 downto 0) <=
			"1000000" when "0000", -- 0
			"1111001" when "0001", -- 1
			"0100100" when "0010", -- 2
			"0110000" when "0011", -- 3
			"0011001" when "0100", -- 4
			"0010010" when "0101", -- 5
			"0000010" when "0110", -- 6
			"1111000" when "0111", -- 7
			"0000000" when "1000", -- 8
			"0010000" when "1001", -- 9
			"0001000" when "1010", -- A, which signifies "10"
			"0000011" when "1011", -- B, which signifies "11"
			"1000110" when "1100", -- C, which signifies "12"
			"0100001" when "1101", -- D, which signifies "13"
			"0000110" when "1110", -- E, which signifies "14"
			"0001110" when others;  -- F, which signifies "15"
			
end arch;



-------------------------------------------
--Custom types VHDL File
-------------------------------------------
-- used to make code easier to follow by passing along mnemonic rather than opcode
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



-------------------------------------------
--Main CPU VHDL File
-------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

library work;
use work.custom_types.all;

entity cpu is
port (
    IN_PORT : in std_logic_vector(15 downto 0);
    OUT_PORT : out std_logic_vector(15 downto 0);
    MM_IN_PORT : in std_logic_vector(15 downto 0);
    MM_OUT_PORT : out std_logic_vector(15 downto 0);

    ALU_RESULT_OUTPUT : out std_logic_vector(15 downto 0);
    instructions_OUT: out std_logic_vector(15 downto 0);
    PC_OUTPUT: out std_logic_vector(15 downto 0);
	
    --Resets
    RST_EX: in std_logic;    --Reset and load (PC <= 0x0002)
    RST_LD: in std_logic;      --Reset and execute (PC <= 0x0000)

    leds: out std_logic_vector( 15 downto 0 );

    clock : in std_logic                                --Set to clk_sys              
);
end cpu;

architecture Behavioral of cpu is

component PC
Port(
    clk: in std_logic;
    RST_LD : in std_logic;
    RST_EX : in std_logic;
    BR_TKN_IN: in std_logic;                               	--Flag for if a conditional branch should be taken
    DESTINATION_ADDRESS_IN: in std_logic_vector(15 downto 0);   --The output of the ALU used for branching
    STALL_IN: in std_logic;
    PC_OUT: out std_logic_vector(15 downto 0)              	--The instruction address for the memery to get the next instruction and for the IF_ID latch for branch instructions
);
end component;

component memory_interface
Port ( 
    rst : in std_logic;
    clk: in std_logic;
    ReadA_OUT : out std_logic_vector(15 downto 0);
    AddrA_IN : in std_logic_vector(15 downto 0);
    WriteA_IN : in std_logic_vector(15 downto 0);
    WeA_IN : in std_logic;
    ReadB_OUT : out std_logic_vector(15 downto 0);
    AddrB_IN : in std_logic_vector(15 downto 0);
    MM_IN_PORT : in std_logic_vector(15 downto 0);
    MM_OUT_PORT : out std_logic_vector(15 downto 0)
    );
end component;

component IF_ID
Port(
    --Inputs
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
end component;

component register_file
Port(
    RST : in std_logic;
    CLK: in std_logic;
    --read signals
    RD_INDEX_A: in std_logic_vector(2 downto 0);    -- the register to read from for data A
    RD_INDEX_B: in std_logic_vector(2 downto 0);    -- the register to read from for data B
    RD_DATA_A: out std_logic_vector(15 downto 0);   -- the data output from the register specified by RD_INDEX_A
    RD_DATA_B: out std_logic_vector(15 downto 0);   -- the data output from the register specified by RD_INDEX_B
    --write signals
    WR_INDEX: in std_logic_vector(2 downto 0);      -- the register to write to if WR_EN is set
    WR_DATA: in std_logic_vector(15 downto 0);      -- data to be written to WR_INDEX register if WR_EN is set
    WR_EN: in std_logic;                            -- enable writing to register specified by WR_INDEX
    --lOADIMM feedback signal
    IMM_REG_OUT: out std_logic_vector(15 downto 0));
end component;

component ID_EX
Port(
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
    MNEMONIC_OUT : out MNEMONIC_TYPE;                   	--Instruction being processed
    OUTPUT_PORT_OUT : out std_logic_vector(15 downto 0);
    -- ALU outputs
    ALU_MODE_OUT : out std_logic_vector(2 downto 0);
    INPUT_A_DATA_OUT : out std_logic_vector(15 downto 0);
    INPUT_B_DATA_OUT : out std_logic_vector(15 downto 0);
    SHFT_OUT : out std_logic_vector(3 downto 0);
    -- Branch outputs
    PC_OUT : out std_logic_vector(15 downto 0);
    -- Reg and Mem Outputs
    REG_WR_ADDR_OUT : out std_logic_vector(2 downto 0);        	--Stores Reg Address
    REG_WR_EN_OUT : out std_logic;
    REG_SOURCE_SEL_OUT : out std_logic;
    ALU_BYPASS_OUT : out std_logic_vector(15 downto 0);
    IMM_UP_SIDE_SEL_OUT : out std_logic;
    MEM_ADDR_OUT : out std_logic_vector(15 downto 0);
    MEM_WR_EN_OUT : out std_logic);
end component;
    
component ALU
Port(
    --Inputs
    CLK: in std_logic;
    RST: in std_logic;
    ALU_MODE_IN: in std_logic_vector( 2 downto 0 );   
    INPUT_A: in std_logic_vector( 15 downto 0 );
    INPUT_B: in std_logic_vector( 15 downto 0 );
    SHIFT_IN: in std_logic_vector(3 downto 0);
    --Outputs
    RESULT: out std_logic_vector( 15 downto 0 );
    Z_FLAG: Out std_logic;                             
    N_FLAG: Out std_logic;
    O_FLAG: Out std_Logic);
end component;

component EX_MEM
Port (
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
end component;

component MEM_WB
Port(
    --INPUTS
    RST : in std_logic;     --Reset the Latch
    CLK : in std_logic;     --Clock used to release data on rising edge
    MNEMONIC_IN : in MNEMONIC_TYPE;
    ALU_RESULT_IN : in std_logic_vector(15 downto 0);   	-- Result from ALU operation
    MEM_DATA_IN : in std_logic_vector(15 downto 0);     	-- result from memory load
    REG_WR_ADDR_IN : in std_logic_vector(2 downto 0);       	-- Stores Reg Address
    REG_WR_EN_IN : in std_logic;                        	-- enable register write
    REG_SOURCE_SEL_IN : in std_logic;                       	-- select ALU or memory load to store in reg
    IMM_UP_SIDE_SEL_IN : in std_logic;
    IMM_REG_IN : in std_logic_vector(15 downto 0);          	-- feedback from reg7 for LOADIMM instructions
    --OUTPUTS
    REG_WR_DATA_OUT : out std_logic_vector(15 downto 0);    	-- the data to be stored in the register. Either ALU result, memory load or immediate value
    REG_WR_ADDR_OUT : out std_logic_vector(2 downto 0);
    REG_WR_EN_OUT : out std_logic;
    MEM_FW_DATA_OUT : out std_logic_vector(15 downto 0));   	-- asynchronous mem wb for data hazard handling
end component;

--Input output ports
signal mm_input_port : std_logic_vector(15 downto 0);
signal mm_output_port : std_logic_vector(15 downto 0);
signal input_port : std_logic_vector(15 downto 0);
signal output_port : std_logic_vector(15 downto 0);

--Local Reset and Clock Signals used for resetting hardware and controlling latches
signal rst_sys : std_logic := '0';
signal clk_sys : std_logic := '0';
signal rst_considering_branch : std_logic := '0';

--Program Counter Ports
signal pc_sys : std_logic_vector(15 downto 0);

--Memory Ports
signal instruction : std_logic_vector(15 downto 0);
signal mem_data_out : std_logic_vector(15 downto 0);

--IF/ID CTRL Latch
signal mnemonic_1 : MNEMONIC_TYPE;
signal alu_mode_1 : std_logic_vector(2 downto 0);
signal input_a_addr : std_logic_vector(2 downto 0);
signal input_b_addr : std_logic_vector(2 downto 0);
signal shft_1 : std_logic_vector(3 downto 0);
signal reg_wr_addr_1 : std_logic_vector(2 downto 0);
signal reg_wr_en_1 : std_logic;
signal reg_source_select_1 : std_logic;
signal imm_data : std_logic_vector(7 downto 0);
signal imm_up_side_sel_1 : std_logic;
signal mem_wr_en_1 : std_logic;
signal stall : std_logic;
signal alu_fw : FW_TYPE;
signal ex_fw : FW_TYPE;
signal mem_fw : FW_TYPE;
signal pc_sys_1 : std_logic_vector(15 downto 0);
signal displacement : std_logic_vector(15 downto 0);

--Register File Ports
signal input_a_data_1 : std_logic_vector(15 downto 0);
signal input_b_data_1 : std_logic_vector(15 downto 0);
signal imm_reg_feedback : std_logic_vector(15 downto 0);

--ID/EX CTRL Latch
signal mnemonic_2 : MNEMONIC_TYPE;
signal alu_bypass_2 : std_logic_vector(15 downto 0);
signal alu_mode_2 : std_logic_vector(2 downto 0);
signal input_a_data_2 : std_logic_vector(15 downto 0);
signal input_b_data_2 : std_logic_vector(15 downto 0);
signal shft_2 : std_logic_vector(3 downto 0);
signal reg_wr_addr_2 : std_logic_vector(2 downto 0);
signal reg_wr_en_2 : std_logic;
signal reg_source_select_2 : std_logic;
signal imm_up_side_sel_2 : std_logic;
signal mem_addr_2 : std_logic_vector(15 downto 0);
signal mem_wr_en_2 : std_logic;
signal pc_sys_2 : std_logic_vector(15 downto 0);

--ALU Ports
signal alu_result_2 : std_logic_vector(15 downto 0);
signal Z_Flag_ALU : std_logic;
signal N_Flag_ALU : std_logic;
signal O_Flag_ALU : std_logic;

--EX/MEM CTRL Latch
signal mnemonic_3 : MNEMONIC_TYPE;
signal alu_result_3 : std_logic_vector(15 downto 0);
signal reg_wr_addr_3 : std_logic_vector(2 downto 0);
signal reg_wr_en_3 : std_logic;
signal reg_source_select_3 : std_logic;
signal imm_up_side_sel_3 : std_logic;
signal mem_addr_3 : std_logic_vector(15 downto 0);
signal mem_wr_en_3 : std_logic;
signal destination_address : std_logic_vector(15 downto 0);
signal br_tkn : std_logic;
signal alu_fw_data : std_logic_vector(15 downto 0);

--MEM/WB CTRL Latch
signal reg_wr_data_4 : std_logic_vector(15 downto 0);
signal reg_wr_addr_4 : std_logic_vector(2 downto 0);
signal reg_wr_en_4 : std_logic;
signal mem_fw_data : std_logic_vector(15 downto 0);

--NOTE the rest of the latches ports' have the same signals as written above.

begin
    -- cpu inputs/outputs
    rst_sys <= RST_LD or RST_EX;        --Reset
    clk_sys <= clock;
    input_port <= IN_PORT;
    OUT_PORT <= output_port;
    mm_input_port <= MM_IN_PORT;
    MM_OUT_PORT <= mm_output_port;
    rst_considering_branch <= rst_sys or br_tkn;   --rst or br tkn
    ALU_RESULT_OUTPUT <= reg_wr_data_4;
    instructions_OUT <= instruction;
    PC_OUTPUT <= pc_sys;

    --Port Map Program Counter
    cpu_PC : PC port map (
        RST_LD => RST_LD,
        RST_EX => RST_EX,
        clk => clk_sys,
        PC_OUT => pc_sys,
        BR_TKN_IN => br_tkn,
        STALL_IN => stall,
        DESTINATION_ADDRESS_IN => destination_address
    );
    
    cpu_memory_interface : memory_interface port map (
        rst => rst_sys,
        clk => clk_sys,
        ReadB_OUT => instruction,
        AddrB_IN => pc_sys,
        ReadA_OUT => mem_data_out,
        AddrA_IN => mem_addr_3,
        WriteA_IN => alu_result_3,
        WeA_IN => mem_wr_en_3,
        MM_IN_PORT => mm_input_port,
        MM_OUT_PORT => mm_output_port
    );
    
    cpu_IF_ID : IF_ID port map(
        RST => rst_considering_branch,
        CLK => clk_sys,
        INSTR_IN => instruction,
        PC_IN => pc_sys,
        MNEMONIC_OUT => mnemonic_1,
        STALL_OUT => stall,
        ALU_FW_OUT => alu_fw,
        EX_FW_OUT => ex_fw,
        MEM_FW_OUT => mem_fw,
        ALU_MODE_OUT => alu_mode_1,
        INPUT_A_ADDR_OUT => input_a_addr,
        INPUT_B_ADDR_OUT => input_b_addr,
        SHFT_OUT => shft_1,
        REG_WR_ADDR_OUT => reg_wr_addr_1,
        REG_WR_EN_OUT => reg_wr_en_1,
        REG_SOURCE_SEL_OUT => reg_source_select_1,
        IMM_DATA_OUT => imm_data,
        IMM_UP_SIDE_SEL_OUT => imm_up_side_sel_1,
        MEM_WR_EN_OUT => mem_wr_en_1,
        PC_OUT => pc_sys_1,
        DISPLACEMENT_OUT => displacement
    );
    
    --Port Map Processor Components
    cpu_register_file : register_file port map (
        RST => rst_sys,
        CLK => clk_sys,
        RD_INDEX_A => input_a_addr,
        RD_INDEX_B => input_b_addr,
        RD_DATA_A => input_a_data_1,
        RD_DATA_B => input_b_data_1,
        WR_INDEX => reg_wr_addr_4,
        WR_DATA => reg_wr_data_4,
        WR_EN => reg_wr_en_4,
        IMM_REG_OUT => imm_reg_feedback
    );
    
    cpu_ID_EX : ID_EX port map (
        RST => rst_considering_branch,
        CLK => clk_sys,
        INPUT_PORT_IN => input_port,
        DISPLACEMENT_IN => displacement,
        IMM_DATA_IN => imm_data,
        ALU_FW_IN => alu_fw,
        ALU_FW_DATA_IN => alu_fw_data,
        EX_FW_IN => ex_fw,
        EX_FW_DATA_IN => alu_result_3,
        MEM_FW_IN => mem_fw,
        MEM_FW_DATA_IN => mem_fw_data,
        
        MNEMONIC_IN => mnemonic_1,
        MNEMONIC_OUT => mnemonic_2,
        PC_IN => pc_sys_1,
        PC_OUT => pc_sys_2,
        ALU_MODE_IN  => alu_mode_1,
        ALU_MODE_OUT => alu_mode_2,
        INPUT_A_DATA_IN => input_a_data_1,
        INPUT_A_DATA_OUT => input_a_data_2,
        INPUT_B_DATA_IN => input_b_data_1,
        INPUT_B_DATA_OUT => input_b_data_2,
        SHFT_IN => shft_1,
        SHFT_OUT => shft_2,
        REG_WR_EN_IN => reg_wr_en_1,
        REG_WR_EN_OUT => reg_wr_en_2,
        REG_WR_ADDR_IN => reg_wr_addr_1,
        REG_WR_ADDR_OUT => reg_wr_addr_2,
        MEM_WR_EN_IN => mem_wr_en_1,
        MEM_WR_EN_OUT => mem_wr_en_2,
        REG_SOURCE_SEL_IN => reg_source_select_1,
        REG_SOURCE_SEL_OUT => reg_source_select_2,
        IMM_UP_SIDE_SEL_IN => imm_up_side_sel_1,
        IMM_UP_SIDE_SEL_OUT => imm_up_side_sel_2,
        
        MEM_ADDR_OUT => mem_addr_2,
        ALU_BYPASS_OUT => alu_bypass_2,
        OUTPUT_PORT_OUT => output_port
    );

    cpu_ALU : ALU port map (
        CLK => clk_sys, 
        RST => rst_sys,
        ALU_MODE_IN => alu_mode_2,
        INPUT_A => input_a_data_2,
        INPUT_B => input_b_data_2,
        SHIFT_IN => shft_2,
        RESULT => alu_result_2,
        Z_FLAG => Z_Flag_ALU,             
        N_FLAG => N_Flag_ALU,
        O_FLAG => O_Flag_ALU
    );

    cpu_EX_MEM : EX_MEM port map (
        RST => rst_sys,    --Reset the Latch
        CLK => clk_sys,     --Clock used to release data on rising edge
        PC_IN => pc_sys_2,
        ALU_BYPASS_IN => alu_bypass_2,
        
        MNEMONIC_IN => mnemonic_2,
        MNEMONIC_OUT => mnemonic_3,
        ALU_Result_IN => alu_result_2,
        ALU_Result_OUT => alu_result_3,
        Z_FLAG => Z_Flag_ALU,
        N_FLAG => N_Flag_ALU,
        O_FLAG => O_Flag_ALU,
        REG_WR_ADDR_IN => reg_wr_addr_2,
        REG_WR_ADDR_OUT => reg_wr_addr_3,
        REG_WR_EN_IN => reg_wr_en_2,
        REG_WR_EN_OUT => reg_wr_en_3,
        REG_SOURCE_SEL_IN => reg_source_select_2,
        REG_SOURCE_SEL_OUT => reg_source_select_3,
        IMM_UP_SIDE_SEL_IN => imm_up_side_sel_2,
        IMM_UP_SIDE_SEL_OUT => imm_up_side_sel_3,
        MEM_ADDR_IN => mem_addr_2,
        MEM_ADDR_OUT => mem_addr_3,
        MEM_WR_EN_IN => mem_wr_en_2,
        MEM_WR_EN_OUT => mem_wr_en_3,

        DESTINATION_ADDRESS_OUT => destination_address,
        BR_TKN_OUT => br_tkn,
        ALU_FW_DATA_OUT => alu_fw_data
    );
    
    cpu_MEM_WB : MEM_WB port map(
        RST => rst_sys,
        CLK => clk_sys,
        MNEMONIC_IN => mnemonic_3,
        ALU_RESULT_IN => alu_result_3,
        MEM_DATA_IN => mem_data_out,
        REG_SOURCE_SEL_IN => reg_source_select_3,
        IMM_UP_SIDE_SEL_IN => imm_up_side_sel_3,
        IMM_REG_IN => imm_reg_feedback,
        
        REG_WR_ADDR_IN => reg_wr_addr_3,
        REG_WR_ADDR_OUT => reg_wr_addr_4,
        REG_WR_EN_IN => reg_wr_en_3,
        REG_WR_EN_OUT => reg_wr_en_4,
        
        REG_WR_DATA_OUT => reg_wr_data_4,
        MEM_FW_DATA_OUT => mem_fw_data
    );

end behavioral;



-------------------------------------------
--Program Counter VHDL File
-------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;   --For Adder

entity PC is
Port ( 
        clk: in std_logic;
        RST_LD : in std_logic;
        RST_EX : in std_logic;
        BR_TKN_IN: in std_logic;                               		--Flag for if a conditional branch should be taken
        DESTINATION_ADDRESS_IN: in std_logic_vector(15 downto 0);       -- The output of the ALU used for branching 
        STALL_IN: in std_logic;                                		-- Set when a stall is required, stops PC from incrementing
        PC_OUT: out std_logic_vector(15 downto 0));            		--The instruction address for the memery to get the next instruction and for the IF_ID latch for branch instructions
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



-------------------------------------------
--Register File VHDL File
-------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity register_file is
port(
    RST : in std_logic;
    CLK: in std_logic;
    --read signals
    RD_INDEX_A: in std_logic_vector(2 downto 0);    -- the register to read from for data A
    RD_INDEX_B: in std_logic_vector(2 downto 0);    -- the register to read from for data B
    RD_DATA_A: out std_logic_vector(15 downto 0);   -- the data output from the register specified by RD_INDEX_A
    RD_DATA_B: out std_logic_vector(15 downto 0);   -- the data output from the register specified by RD_INDEX_B
    --write signals
    WR_INDEX: in std_logic_vector(2 downto 0);      -- the register to write to if WR_EN is set
    WR_DATA: in std_logic_vector(15 downto 0);      -- data to be written to WR_INDEX register if WR_EN is set
    WR_EN: in std_logic;                            -- enable writing to register specified by WR_INDEX
    --lOADIMM feedback signal
    IMM_REG_OUT: out std_logic_vector(15 downto 0));
end register_file;

architecture behavioural of register_file is

type reg_array is array (integer range 0 to 7) of std_logic_vector(15 downto 0);
--internals signals
signal reg_file : reg_array := (others=>(others=>'0'));
begin
--write operation 
process(CLK)
begin
    if(CLK='1' and CLK'event) then if(RST='1') then
        for i in 0 to 7 loop
            reg_file(i)<= (others => '0'); 
        end loop;
    elsif(WR_EN='1') then
        case WR_INDEX(2 downto 0) is
            when "000" => reg_file(0) <= WR_DATA;
            when "001" => reg_file(1) <= WR_DATA;
            when "010" => reg_file(2) <= WR_DATA;
            when "011" => reg_file(3) <= WR_DATA;
            when "100" => reg_file(4) <= WR_DATA;
            when "101" => reg_file(5) <= WR_DATA;
            when "110" => reg_file(6) <= WR_DATA;
            when "111" => reg_file(7) <= WR_DATA;
            --fill this part
            when others => NULL; 
        end case;
    end if;
    end if;
end process;

--read operation
RD_DATA_A <=
reg_file(0) when(RD_INDEX_A="000") else
reg_file(1) when(RD_INDEX_A="001") else
reg_file(2) when(RD_INDEX_A="010") else
reg_file(3) when(RD_INDEX_A="011") else
reg_file(4) when(RD_INDEX_A="100") else
reg_file(5) when(RD_INDEX_A="101") else
reg_file(6) when(RD_INDEX_A="110") else reg_file(7);

--Read for second array
RD_DATA_B <=	
reg_file(0) when(RD_INDEX_B="000") else
reg_file(1) when(RD_INDEX_B="001") else
reg_file(2) when(RD_INDEX_B="010") else
reg_file(3) when(RD_INDEX_B="011") else
reg_file(4) when(RD_INDEX_B="100") else
reg_file(5) when(RD_INDEX_B="101") else
reg_file(6) when(RD_INDEX_B="110") else reg_file(7);

IMM_REG_OUT <= reg_file(7);

end behavioural;



-------------------------------------------
--ALU VHDL File
-------------------------------------------
library ieee; 
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_SIGNED.all;
use IEEE.NUMERIC_STD.all;
 
entity ALU is
 
port (
        --Inputs
        CLK: in std_logic;
        RST: in std_logic;
        ALU_MODE_IN: in std_logic_vector( 2 downto 0 );   
        INPUT_A: in std_logic_vector( 15 downto 0 );
        INPUT_B: in std_logic_vector( 15 downto 0 );
        SHIFT_IN: in std_logic_vector(3 downto 0);
        --Outputs
        RESULT: out std_logic_vector( 15 downto 0 );
        Z_FLAG: Out std_logic;  --Zero flag                              
        N_FLAG: Out std_logic;  --Negative flag
        O_FLAG: Out std_Logic   --Overflow flag
    );
 
end ALU;
 
architecture Behavioral of ALU is
    signal ALU_Result : std_logic_vector (31 downto 0);
    signal Zero_Flag : std_logic;
    signal inputA_Negative_Flag : std_logic;
    signal Overflow_Flag : std_logic;
    
    begin
        process(RST, ALU_MODE_IN, INPUT_A, INPUT_B, SHIFT_IN)
            variable tmp_result : std_logic_vector(31 downto 0);
        begin
            if (RST = '1') then   
                tmp_result := "00000000000000000000000000000000"; --Output nothing
                Zero_Flag <= '0';
                InputA_Negative_Flag <= '0';
                Overflow_Flag <= '0';
            else
                
                case(ALU_MODE_IN) is
                when "000" => --NOP
                    tmp_result := "00000000000000000000000000000000";
                when "001" => -- Add
                    tmp_result := std_logic_vector(resize(signed(INPUT_A) + signed(INPUT_B), tmp_result'length));
                when "010" => -- Sub
                    tmp_result := std_logic_vector(resize(signed(INPUT_A) - signed(INPUT_B), tmp_result'length));
                when "011" => -- MUL Multiplication
                    tmp_result := std_logic_vector(resize((signed(INPUT_A) * signed(INPUT_B)), tmp_result'length));        
                when "100" => -- Logical nand 
                    tmp_result := std_logic_vector(resize((signed(INPUT_A) NAND signed(INPUT_B)), tmp_result'length));
                when "101" => --SHL Shift Left Logical
                    case(SHIFT_IN) is
                    when "0001" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 1)), tmp_result'length));
                    when "0010" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 2)), tmp_result'length));
                    when "0011" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 3)), tmp_result'length));
                    when "0100" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 4)), tmp_result'length));
                    when "0101" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 5)), tmp_result'length));
                    when "0110" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 6)), tmp_result'length));
                    when "0111" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 7)), tmp_result'length));
                    when "1000" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 8)), tmp_result'length));
                    when "1001" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 9)), tmp_result'length));
                    when "1010" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 10)), tmp_result'length));
                    when "1011" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 11)), tmp_result'length));
                    when "1100" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 12)), tmp_result'length));
                    when "1101" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 13)), tmp_result'length));
                    when "1110" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 14)), tmp_result'length));
                    when "1111" =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 15)), tmp_result'length));
                    when others =>
                        tmp_result := std_logic_vector(resize((shift_left(signed(INPUT_A), 0)), tmp_result'length));
                    end case;
                when "110" => --SHR Shift Right Logical
                    case(SHIFT_IN) is
                    when "0001" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 1)), tmp_result'length));
                    when "0010" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 2)), tmp_result'length));
                    when "0011" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 3)), tmp_result'length));
                    when "0100" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 4)), tmp_result'length));
                    when "0101" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 5)), tmp_result'length));
                    when "0110" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 6)), tmp_result'length));
                    when "0111" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 7)), tmp_result'length));
                    when "1000" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 8)), tmp_result'length));
                    when "1001" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 9)), tmp_result'length));
                    when "1010" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 10)), tmp_result'length));
                    when "1011" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 11)), tmp_result'length));
                    when "1100" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 12)), tmp_result'length));
                    when "1101" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 13)), tmp_result'length));
                    when "1110" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 14)), tmp_result'length));
                    when "1111" =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 15)), tmp_result'length));
                    when others =>
                        tmp_result := std_logic_vector(resize((shift_right(signed(INPUT_A), 0)), tmp_result'length));
                    end case;
                when "111" => --Test
                     tmp_result := std_logic_vector(resize(signed(INPUT_A), tmp_result'length));
                    if(signed(INPUT_A) > 0) then        --Check if INPUT_A is positive
                        InputA_Negative_Flag <= '0';    --Set negative flag to 0
                        Zero_Flag <= '0';               --Reset the Zero Flag
                    elsif(signed(INPUT_A) < 0) then     --Check if INPUT_A is negative
                        InputA_Negative_Flag <= '1';    --Set negative flag to 1 or TRUE
                        Zero_Flag <= '0';               --Reset the Zero Flag
                    else                                --Check if INPUT_A is equal to zero
                        Zero_Flag <= '1';               --Set zero flag to 1 or TRUE
                        InputA_Negative_Flag <= '0';    --Reset negative flag 
                    end if; 
                when others => 
                    tmp_result := "00000000000000000000000000000000";  --Error Handeling 
                end case;

                if(signed(tmp_result) > 32767 or signed(tmp_result) < -32768) then
                Overflow_Flag <= '1';
                tmp_result := "00000000000000000000000000000000";
                end if;
                
            end if;
            ALU_Result <= tmp_result;
        end process;

        process(CLK)
        begin
        if rising_edge(CLK) then
            Z_Flag <= Zero_Flag;
            N_Flag <= InputA_Negative_Flag;
            O_Flag <= Overflow_Flag;
            Result <= std_logic_vector(resize(signed(ALU_Result), Result' length));
        end if;
        end process;
end Behavioral;



-------------------------------------------
--Memory Interface VHDL File
-------------------------------------------
Library xpm;
use xpm.vcomponents.all;
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use IEEE.NUMERIC_STD.ALL;

entity memory_interface is
port(
    rst : in std_logic;
    clk: in std_logic;
    ReadA_OUT : out std_logic_vector(15 downto 0) := "0000000000000000";	-- output from portA
    AddrA_IN : in std_logic_vector(15 downto 0);				-- address of portA
    WriteA_IN : in std_logic_vector(15 downto 0);				-- data into portA
    WeA_IN : in std_logic;							-- write enable of portA
    ReadB_OUT : out std_logic_vector(15 downto 0) := "0000000000000000";    	-- instruction out
    AddrB_IN : in std_logic_vector(15 downto 0);                            	-- instruction address in (PC)
    MM_IN_PORT : in std_logic_vector(15 downto 0);
    MM_OUT_PORT : out std_logic_vector(15 downto 0));
end memory_interface;


architecture Behavioral of memory_interface is


signal weA_v : std_logic_vector(0 downto 0);

signal ram_douta : std_logic_vector(15 downto 0);
signal ram_addra : std_logic_vector(15 downto 0);
signal ram_doutb : std_logic_vector(15 downto 0);
signal ram_addrb : std_logic_vector(15 downto 0);
signal ram_dina : std_logic_vector(15 downto 0);
signal ram_ena : std_logic;
signal ram_enb : std_logic;

signal rom_douta : std_logic_vector(15 downto 0);
signal rom_addra : std_logic_vector(15 downto 0);
signal rom_ena : std_logic;

signal in_port : std_logic_vector(15 downto 0):= x"0002";
signal out_port : std_logic_vector(15 downto 0);

begin

   xpm_memory_dpdistram_inst : xpm_memory_dpdistram
   generic map (
      ADDR_WIDTH_A => 16,               -- DECIMAL
      ADDR_WIDTH_B => 16,               -- DECIMAL
      BYTE_WRITE_WIDTH_A => 16,        -- DECIMAL
      CLOCKING_MODE => "common_clock", -- String
      MEMORY_INIT_FILE => "none",      -- String
      MEMORY_INIT_PARAM => "",        -- String
      MEMORY_OPTIMIZATION => "true",   -- String
      MEMORY_SIZE => 8192,             -- DECIMAL
      MESSAGE_CONTROL => 0,            -- DECIMAL
      READ_DATA_WIDTH_A => 16,         -- DECIMAL
      READ_DATA_WIDTH_B => 16,         -- DECIMAL
      READ_LATENCY_A => 1,             -- DECIMAL
      READ_LATENCY_B => 1,             -- DECIMAL
      READ_RESET_VALUE_A => "0",       -- String
      READ_RESET_VALUE_B => "0",       -- String
      --RST_MODE_A => "SYNC",            -- String   --This feature only exists in vervado 2020 and not 2017
      --RST_MODE_B => "SYNC",            -- String   --This feature only exists in vervado 2020 and not 2017
      --SIM_ASSERT_CHK => 0,             -- DECIMAL; 0=disable simulation messages, 1=enable simulation messages
      USE_EMBEDDED_CONSTRAINT => 0,    -- DECIMAL
      USE_MEM_INIT => 1,               -- DECIMAL
      WRITE_DATA_WIDTH_A => 16         -- DECIMAL
   )
   port map (
      douta => ram_douta,   -- READ_DATA_WIDTH_A-bit output: Data output for port A read operations.
      doutb => ram_doutb,   -- READ_DATA_WIDTH_B-bit output: Data output for port B read operations.
      addra => ram_addra,   -- ADDR_WIDTH_A-bit input: Address for port A write and read operations.
      addrb => ram_addrb,   -- ADDR_WIDTH_B-bit input: Address for port B write and read operations.
      clka => clk,     -- 1-bit input: Clock signal for port A. Also clocks port B when parameter
      clkb => clk,     -- 1-bit input: Clock signal for port B when parameter CLOCKING_MODE is
                        -- "independent_clock". Unused when parameter CLOCKING_MODE is "common_clock".
                        
      dina => ram_dina,     -- WRITE_DATA_WIDTH_A-bit input: Data input for port A write operations.
      ena => ram_ena,       -- 1-bit input: Memory enable signal for port A. Must be high on clock cycles when read
                        -- or write operations are initiated. Pipelined internally.

      enb => ram_enb,       -- 1-bit input: Memory enable signal for port B. Must be high on clock cycles when read
                        -- or write operations are initiated. Pipelined internally.

      regcea => '1', -- 1-bit input: Clock Enable for the last register stage on the output data path.
      regceb => '1', -- 1-bit input: Do not change from the provided value.
      rsta => rst,     -- 1-bit input: Reset signal for the final port A output register stage. Synchronously
                        -- resets output port douta to the value specified by parameter READ_RESET_VALUE_A.

      rstb => rst,     -- 1-bit input: Reset signal for the final port B output register stage. Synchronously
                        -- resets output port doutb to the value specified by parameter READ_RESET_VALUE_B.

      wea => weA_v        -- WRITE_DATA_WIDTH_A/BYTE_WRITE_WIDTH_A-bit input: Write enable vector for port A
                        -- input data port dina. 1 bit wide when word-wide writes are used. In byte-wide write
                        -- configurations, each bit controls the writing one byte of dina to address addra. For
                        -- example, to synchronously write only bits [15-8] of dina when WRITE_DATA_WIDTH_A is
                        -- 32, wea would be 4'b0010.

   );

-- <-----Cut code below this line and paste into the architecture body---->

   -- xpm_memory_sprom: Single Port ROM
   -- Xilinx Parameterized Macro, version 2020.2

   xpm_memory_sprom_inst : xpm_memory_sprom
   generic map (
      ADDR_WIDTH_A => 16,              -- DECIMAL
      AUTO_SLEEP_TIME => 0,           -- DECIMAL
      --CASCADE_HEIGHT => 0,            -- DECIMAL
      ECC_MODE => "no_ecc",           -- String
      MEMORY_INIT_FILE => "InstructionSetA.mem",     -- String
      MEMORY_INIT_PARAM => "",       -- String
      MEMORY_OPTIMIZATION => "true",  -- String
      MEMORY_PRIMITIVE => "auto",     -- String
      MEMORY_SIZE => 8192,            -- DECIMAL
      MESSAGE_CONTROL => 0,           -- DECIMAL
      READ_DATA_WIDTH_A => 16,        -- DECIMAL
      READ_LATENCY_A => 1,            -- DECIMAL
      READ_RESET_VALUE_A => "0",      -- String
      --RST_MODE_A => "SYNC",           -- String  --This feature only exists in vervado 2020 and not 2017
      --SIM_ASSERT_CHK => 0,            -- DECIMAL; 0=disable simulation messages, 1=enable simulation messages --This feature only exists in vervado 2020 and not 2017
      USE_MEM_INIT => 1,              -- DECIMAL
      WAKEUP_TIME => "disable_sleep"  -- String
   )
   port map (
      dbiterra => open,             -- 1-bit output: Leave open.
      douta => rom_douta,                   -- READ_DATA_WIDTH_A-bit output: Data output for port A read operations.
      sbiterra => open,             -- 1-bit output: Leave open.
      addra => rom_addra,                   -- ADDR_WIDTH_A-bit input: Address for port A read operations.
      clka => clk,                     -- 1-bit input: Clock signal for port A.
      ena => rom_ena,                       -- 1-bit input: Memory enable signal for port A. Must be high on clock
                                        -- cycles when read operations are initiated. Pipelined internally.

      injectdbiterra => '0', -- 1-bit input: Do not change from the provided value.
      injectsbiterra => '0', -- 1-bit input: Do not change from the provided value.
      regcea => '1',                 -- 1-bit input: Do not change from the provided value.
      rsta => rst,                     -- 1-bit input: Reset signal for the final port A output register
                                        -- stage. Synchronously resets output port douta to the value specified
                                        -- by parameter READ_RESET_VALUE_A.

      sleep => '0'                    -- 1-bit input: sleep signal to enable the dynamic power saving feature.
   );

   -- End of xpm_memory_sprom_inst instantiation	
  
-- ramB and romA connected to portB for instructon fetch
-- ramA, input and output connected to portA for load and store

rom_ena <= '1' when (AddrB_IN < 1024) else '0';							--ROM is only used for port B and will only be enabled for addresses between 0 and 1024
ram_ena <= '1' when (AddrA_IN >= 1024 and AddrA_IN < 2048) else '0';	--RAM is enabled if the input address is between 1024 and 2048
ram_enb <= '1' when (AddrB_IN >= 1024 and AddrB_IN < 2048) else '0';
rom_addra <= std_logic_vector(unsigned(AddrB_IN) srl 1) when (AddrB_IN < 1024) else (others => '0');	--ROM address is shifted to map the addresses so that the processor is byte addressable
ram_addra <= std_logic_vector(unsigned(AddrA_IN - 1024) srl 1) when(AddrA_IN >= 1024 and AddrA_IN < 2048) else (others => '0');		--RAM address is mapped to the macro addressing by subtracting 1024 and shifting left
ram_addrb <= std_logic_vector(unsigned(AddrB_IN - 1024) srl 1) when(AddrB_IN >= 1024 and AddrB_IN < 2048) else (others => '0');
out_port <= WriteA_IN when(WeA_IN = '1' and AddrA_IN = X"FFF2");											--OUT port is written to when write enable is high and address is FFF2
ram_dina <= WriteA_IN when (AddrA_IN >= 1024 and AddrA_IN < 2048 and WeA_IN = '1');							
ReadA_OUT <= ram_douta when (AddrA_IN >= 1024 and AddrA_IN < 2048 and WeA_IN = '0' and clk /= '0') else 	--ReadOUT is set depending on what module (ROM RAM or IN-PORT) the input address corresponds to
             in_port when(AddrA_IN = X"FFF0" and WeA_IN = '0' and clk /= '0') else                      
             (others => '0') when (clk /= '0'); 
ReadB_OUT <= rom_douta when (AddrB_IN < 1024 and clk /= '0') else 
             ram_doutb when (AddrB_IN >= 1024 and AddrB_IN < 2048 and clk /= '0') else 
             in_port when(AddrB_IN = X"FFF0" and clk /= '0') else 
             (others => '0') when (clk /= '0'); 
weA_v <= "1" when(WeA_IN = '1') else "0";

process(clk)
begin   
    if rising_edge(clk) then
        MM_OUT_PORT <= out_port;
        in_port <= MM_IN_PORT;
    end if;
end process;

end Behavioral;



-------------------------------------------
--IF_ID latch and logic VHDL File
-------------------------------------------
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
signal reg_wr_addr : std_logic_vector(2 downto 0);      --bits 8 to 6
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
input_a_addr <= INSTR_IN(5 downto 3) when(instr_t = A1 or mnemonic = STORE or mnemonic = MOV) else      -- reg data_a is rb when A1, STORE, or MOVE instructions and ra when A2, TEST, or OUT instructions
                INSTR_IN(8 downto 6) when(instr_t = A2 or mnemonic = TEST or mnemonic = OUT_I or instr_t = B2) else
                "111" when(mnemonic = RETURN_I) else "000";
input_b_addr <= INSTR_IN(8 downto 6) when(mnemonic = STORE) else                                        -- reg data_b address is ra for STORE instructions, rb for LOAD instructions
                INSTR_IN(5 downto 3) when(mnemonic = LOAD) else                                        	-- and rc for A1 instructions
                INSTR_IN(2 downto 0) when(instr_t = A1) else "000";
shift_val <= INSTR_IN(3 downto 0) when (instr_t = A2) else "0000";                                   	-- alu shift value is 3 downto 0
reg_wr_addr <= INSTR_IN(8 downto 6) when(   instr_t = A1 or                                         	-- store address is always input_a for arithmetic instructions
                                            instr_t = A2 or                                         	-- and for LOAD, MOVE, and IN instructions
                                            mnemonic = LOAD or 
                                            mnemonic = MOV or 
                                            mnemonic = IN_I) else
               "111" when(mnemonic = BR_SUB or mnemonic = LOADIMM) else "000";                          -- store address is r7 for sub and LOADIMM instructions
reg_wr_en <= '1' when(  instr_t = A1 or                                                             	-- register_write_enable set when A1, A2, IN or any L instruction except STORE
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



-------------------------------------------
--ID_EX latch and logic VHDL File
-------------------------------------------
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
    MNEMONIC_OUT : out MNEMONIC_TYPE;                    	--Instruction being processed
    OUTPUT_PORT_OUT : out std_logic_vector(15 downto 0);
    -- ALU outputs
    ALU_MODE_OUT : out std_logic_vector(2 downto 0);
    INPUT_A_DATA_OUT : out std_logic_vector(15 downto 0);
    INPUT_B_DATA_OUT : out std_logic_vector(15 downto 0);
    SHFT_OUT : out std_logic_vector(3 downto 0);
    -- Branch outputs
    PC_OUT : out std_logic_vector(15 downto 0);
    -- Reg and Mem Outputs
    REG_WR_ADDR_OUT : out std_logic_vector(2 downto 0);      	--Stores Reg Address
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



-------------------------------------------
--EX_MEM latch and logic VHDL File
-------------------------------------------
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
alu_result <=   PC_IN when(mnemonic = BR_SUB) else	-- alu_result is PC for branch instructions so it can be stored in the reg file as the return addr, otherwise it's ALU_RESULT
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



-------------------------------------------
--MEM_WB latch and logic VHDL File
-------------------------------------------
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
    if (RST = '1') then                 	--Reset local values then write them to the output
        REG_WR_DATA_OUT <= x"0000";
        REG_WR_ADDR_OUT <= "000";
        REG_WR_EN_OUT <= '0';       
    elsif falling_edge(CLK) then         	--Write input into latch memory to be reteived by the next stage of the pipeline. 
        REG_WR_DATA_OUT <= reg_wr_data;
        REG_WR_ADDR_OUT <= reg_wr_addr;
        REG_WR_EN_OUT <= reg_wr_en;
    end if;
end Process;

MEM_FW_DATA_OUT <= imm_reg when(mnemonic = LOADIMM) else MEM_DATA_IN;

end Behavioral;
