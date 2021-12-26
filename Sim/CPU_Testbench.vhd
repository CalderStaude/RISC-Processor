LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY CPU_Testbench IS
END CPU_Testbench;
 
ARCHITECTURE behavior OF CPU_Testbench IS 

    COMPONENT cpu   --Component being tested
    PORT (          
         --Inputs
        RST : in std_logic; 
        CLK : in std_logic
    );
    END COMPONENT;


   signal rst : std_logic;
   signal clk : std_logic;
 
BEGIN
 
    --Directly Mapped ports
    uut: cpu PORT MAP (
        rst => rst,
        clk => clk
    );
    
    -- Stimulus process
    stim_proc: process
    begin  
        rst <= '0';
        clk <= '0';    
        
        for i in 0 to 30 loop     
            clk <= '0';
            wait for 10 ns;
            clk <= '1';
            wait for 10 ns;
            
        end loop;

   end process;
END;
