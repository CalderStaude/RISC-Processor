LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
 
ENTITY CPU_Testbench IS
END CPU_Testbench;
 
ARCHITECTURE behavior OF CPU_Testbench IS 

    COMPONENT cpu   --Component being tested
    PORT (          
        IN_PORT : in std_logic_vector(15 downto 0);
        OUT_PORT : out std_logic_vector(15 downto 0);
        MM_IN_PORT : in std_logic_vector(15 downto 0);
        MM_OUT_PORT : out std_logic_vector(15 downto 0);
    
        --Resets
        RST_EX: in std_logic;    --Reset and load (PC <= 0x0002)
        RST_LD: in std_logic;      --Reset and execute (PC <= 0x0000)
    
        leds: out std_logic_vector( 15 downto 0 );
    
        clock : in std_logic                                --Set to clk_sys      
    );
    END COMPONENT;


   signal rst : std_logic;
   signal clk : std_logic;
 
BEGIN
 
    --Directly Mapped ports
    uut: cpu PORT MAP (
        RST_EX => rst,
        RST_LD => rst,
        clock => clk,
        IN_PORT => x"0005",
        MM_IN_PORT => x"0005"
    );
    
    -- Stimulus process
    stim_proc: process
    begin  
        rst <= '0';
        clk <= '0';    
        
        for i in 0 to 10 loop     
            clk <= '0';
            wait for 10 ns;
            clk <= '1';
            wait for 10 ns;
            
        end loop;

   end process;
END;
