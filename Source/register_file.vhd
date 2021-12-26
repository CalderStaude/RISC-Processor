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