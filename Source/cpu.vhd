--Overall Processor control
--Rev 9

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
    BR_TKN_IN: in std_logic;                               --Flag for if a conditional branch should be taken
    DESTINATION_ADDRESS_IN: in std_logic_vector(15 downto 0);       --The output of the ALU used for branching 
    STALL_IN: in std_logic;
    PC_OUT: out std_logic_vector(15 downto 0)              --The instruction address for the memery to get the next instruction and for the IF_ID latch for branch instructions
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
    ALU_RESULT_IN : in std_logic_vector(15 downto 0);   -- Result from ALU operation
    MEM_DATA_IN : in std_logic_vector(15 downto 0);     -- result from memory load
    REG_WR_ADDR_IN : in std_logic_vector(2 downto 0);       -- Stores Reg Address
    REG_WR_EN_IN : in std_logic;                        -- enable register write
    REG_SOURCE_SEL_IN : in std_logic;                       -- select ALU or memory load to store in reg
    IMM_UP_SIDE_SEL_IN : in std_logic;
    IMM_REG_IN : in std_logic_vector(15 downto 0);          -- feedback from reg7 for LOADIMM instructions
    --OUTPUTS
    REG_WR_DATA_OUT : out std_logic_vector(15 downto 0);    -- the data to be stored in the register. Either ALU result, memory load or immediate value
    REG_WR_ADDR_OUT : out std_logic_vector(2 downto 0);
    REG_WR_EN_OUT : out std_logic;
    MEM_FW_DATA_OUT : out std_logic_vector(15 downto 0));   -- asynchronous mem wb for data hazard handling
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
