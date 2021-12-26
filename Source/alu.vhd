--ECE 449 ALU
--March 28 2021
--Rev 7
 
 
library ieee; 
--use IEEE.STD_LOGIC_ARITH.ALL;  --May not need
use IEEE.STD_LOGIC_1164.ALL;
--use IEEE.STD_LOGIC_UNSIGNED.ALL;
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
                    tmp_result := std_logic_vector(resize((signed(INPUT_A) NAND signed(INPUT_B)), tmp_result'length)); --std_logic_vector(signed(INPUT_A) NAND signed(INPUT_B));
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
