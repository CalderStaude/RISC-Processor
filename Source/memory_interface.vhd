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
    AddrA_IN : in std_logic_vector(15 downto 0);							-- address of portA
    WriteA_IN : in std_logic_vector(15 downto 0);							-- data into portA
    WeA_IN : in std_logic;													-- write enable of portA
    ReadB_OUT : out std_logic_vector(15 downto 0) := "0000000000000000";    -- instruction out
    AddrB_IN : in std_logic_vector(15 downto 0);                            -- instruction address in (PC)
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

rom_ena <= '1' when (AddrB_IN < 1024) else '0';
ram_ena <= '1' when (AddrA_IN >= 1024 and AddrA_IN < 2048) else '0';
ram_enb <= '1' when (AddrB_IN >= 1024 and AddrB_IN < 2048) else '0';
rom_addra <= std_logic_vector(unsigned(AddrB_IN) srl 1) when (AddrB_IN < 1024) else (others => '0'); 
ram_addra <= std_logic_vector(unsigned(AddrA_IN - 1024) srl 1) when(AddrA_IN >= 1024 and AddrA_IN < 2048) else (others => '0');
ram_addrb <= std_logic_vector(unsigned(AddrB_IN - 1024) srl 1) when(AddrB_IN >= 1024 and AddrB_IN < 2048) else (others => '0');
out_port <= WriteA_IN when(WeA_IN = '1' and AddrA_IN = X"FFF2");
ram_dina <= WriteA_IN when (AddrA_IN >= 1024 and AddrA_IN < 2048 and WeA_IN = '1');
ReadA_OUT <= ram_douta when (AddrA_IN >= 1024 and AddrA_IN < 2048 and WeA_IN = '0' and clk /= '0') else 
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
