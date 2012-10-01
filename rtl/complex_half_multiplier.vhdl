
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use ieee.math_real.all;

library floatfixlib;
use floatfixlib.fixed_pkg.all;

use work.fft_lib.all;


library UNISIM;
use UNISIM.Vcomponents.all;



entity complex_half_multiplier is

  port (
    clk, reset_n : in  std_logic;
    A_top, B_top : in  std_logic_vector(17 downto 0);
    A_bot, B_bot : in  std_logic_vector(17 downto 0);
    subtract     : in  std_logic;
    P            : out std_logic_vector(47 downto 0)
    );

end complex_half_multiplier;







architecture spartan6 of complex_half_multiplier is
  
  signal P_top : std_logic_vector(47 downto 0);

  signal PC_carry : std_logic_vector(47 downto 0);
  signal BC_carry : std_logic_vector(17 downto 0);

  signal PC_bot : std_logic_vector(47 downto 0);
  signal BC_bot : std_logic_vector(17 downto 0);

  signal C : std_logic_vector(47 downto 0);
  signal D : std_logic_vector(17 downto 0);
  
  signal ce, reset : std_logic;
  signal top_opmode : std_logic_vector(7 downto 0);
  signal carry : std_logic;
begin  -- rtl

  P <= P_top;

  ce    <= '1';
  reset <= not reset_n;

  C <= (others => '0');
  D <= (others => '0');

  top_opmode <= subtract & "0000101";
  
  DSP48A1_top : DSP48A1
   generic map (
      A0REG => 1,              -- First stage A input pipeline register (0/1)
      A1REG => 1,              -- Second stage A input pipeline register (0/1)
      B0REG => 1,              -- First stage B input pipeline register (0/1)
      B1REG => 1,              -- Second stage B input pipeline register (0/1)
      CARRYINREG => 0 ,         -- CARRYIN input pipeline register (0/1)
      CARRYINSEL => "OPMODE5", -- Specify carry-in source, "CARRYIN" or "OPMODE5" 
      CARRYOUTREG => 1,        -- CARRYOUT output pipeline register (0/1)
      CREG => 1,               -- C input pipeline register (0/1)
      DREG => 1,               -- D pre-adder input pipeline register (0/1)
      MREG => 1,               -- M pipeline register (0/1)
      OPMODEREG => 1,          -- Enable=1/disable=0 OPMODE input pipeline registers
      PREG => 1,               -- P output pipeline register (0/1)
      RSTTYPE => "SYNC"        -- Specify reset type, "SYNC" or "ASYNC" 
   )
   port map (
      -- Cascade Ports: 18-bit (each) Ports to cascade from one DSP48 to another
      BCOUT => open,           -- 18-bit B port cascade output
      PCOUT => open,           -- 48-bit P cascade output (if used, connect to PCIN of another DSP48A1)
      -- Data Ports: 1-bit (each) Data input and output ports
      CARRYOUT => open,     -- 1-bit carry output (if used, connect to CARRYIN pin of another DSP48A1)
      CARRYOUTF => open,   -- 1-bit fabric carry output
      M => open,                   -- 36-bit fabric multiplier data output
      P => P_top,                   -- 48-bit data output
      -- Cascade Ports: 48-bit (each) Ports to cascade from one DSP48 to another
      PCIN => PC_carry,             -- 48-bit P cascade input (if used, connect to PCOUT of another DSP48A1)
      -- Control Input Ports: 1-bit (each) Clocking and operation mode
      CLK => clk,               -- 1-bit clock input
      OPMODE => top_opmode,         -- 8-bit operation mode input
      -- Data Ports: 18-bit (each) Data input and output ports
      A => A_top,                   -- 18-bit A data input
      B => B_top,                   -- 18-bit B data input (connected to fabric or BCOUT of adjacent DSP48A1)
      C => C,                   -- 48-bit C data input
      CARRYIN => carry,       -- 1-bit carry input signal (if used, connect to CARRYOUT pin of another
                                -- DSP48A1)

      D => D,                   -- 18-bit B pre-adder data input
      -- Reset/Clock Enable Input Ports: 1-bit (each) Reset and enable input ports
      CEA => ce,               -- 1-bit active high clock enable input for A registers
      CEB => ce,               -- 1-bit active high clock enable input for B registers
      CEC => ce,               -- 1-bit active high clock enable input for C registers
      CECARRYIN => ce,   -- 1-bit active high clock enable input for CARRYIN registers
      CED => ce,               -- 1-bit active high clock enable input for D registers
      CEM => ce,               -- 1-bit active high clock enable input for multiplier registers
      CEOPMODE => ce,     -- 1-bit active high clock enable input for OPMODE registers
      CEP => ce,               -- 1-bit active high clock enable input for P registers
      RSTA => reset,             -- 1-bit reset input for A pipeline registers
      RSTB => reset,             -- 1-bit reset input for B pipeline registers
      RSTC => reset,             -- 1-bit reset input for C pipeline registers
      RSTCARRYIN => reset, -- 1-bit reset input for CARRYIN pipeline registers
      RSTD => reset,             -- 1-bit reset input for D pipeline registers
      RSTM => reset,             -- 1-bit reset input for M pipeline registers
      RSTOPMODE => reset,   -- 1-bit reset input for OPMODE pipeline registers
      RSTP => reset              -- 1-bit reset input for P pipeline registers
   );
  

   DSP48A1_bot : DSP48A1
   generic map (
      A0REG => 1,              -- First stage A input pipeline register (0/1)
      A1REG => 0,              -- Second stage A input pipeline register (0/1)
      B0REG => 1,              -- First stage B input pipeline register (0/1)
      B1REG => 0,              -- Second stage B input pipeline register (0/1)
      CARRYINREG => 1,         -- CARRYIN input pipeline register (0/1)
      CARRYINSEL => "OPMODE5", -- Specify carry-in source, "CARRYIN" or "OPMODE5" 
      CARRYOUTREG => 1,        -- CARRYOUT output pipeline register (0/1)
      CREG => 1,               -- C input pipeline register (0/1)
      DREG => 1,               -- D pre-adder input pipeline register (0/1)
      MREG => 1,               -- M pipeline register (0/1)
      OPMODEREG => 1,          -- Enable=1/disable=0 OPMODE input pipeline registers
      PREG => 1,               -- P output pipeline register (0/1)
      RSTTYPE => "SYNC"        -- Specify reset type, "SYNC" or "ASYNC" 
   )
   port map (
      -- Cascade Ports: 18-bit (each) Ports to cascade from one DSP48 to another
      BCOUT => open,           -- 18-bit B port cascade output
      PCOUT => PC_carry,           -- 48-bit P cascade output (if used, connect to PCIN of another DSP48A1)
      -- Data Ports: 1-bit (each) Data input and output ports
      CARRYOUT => carry,     -- 1-bit carry output (if used, connect to CARRYIN pin of another DSP48A1)
      CARRYOUTF => open,   -- 1-bit fabric carry output
      M => open,                   -- 36-bit fabric multiplier data output
      P => open,                   -- 48-bit data output
      -- Cascade Ports: 48-bit (each) Ports to cascade from one DSP48 to another
      PCIN => PC_bot,             -- 48-bit P cascade input (if used, connect to PCOUT of another DSP48A1)
      -- Control Input Ports: 1-bit (each) Clocking and operation mode
      CLK => clk,               -- 1-bit clock input
      OPMODE => "00000001",         -- 8-bit operation mode input
      -- Data Ports: 18-bit (each) Data input and output ports
      A => A_bot,                   -- 18-bit A data input
      B => B_bot,                   -- 18-bit B data input (connected to fabric or BCOUT of adjacent DSP48A1)
      C => C,                   -- 48-bit C data input
      CARRYIN => '0',       -- 1-bit carry input signal (if used, connect to CARRYOUT pin of another
                                -- DSP48A1)

      D => D,                   -- 18-bit B pre-adder data input
      -- Reset/Clock Enable Input Ports: 1-bit (each) Reset and enable input ports
      CEA => ce,               -- 1-bit active high clock enable input for A registers
      CEB => ce,               -- 1-bit active high clock enable input for B registers
      CEC => ce,               -- 1-bit active high clock enable input for C registers
      CECARRYIN => ce,   -- 1-bit active high clock enable input for CARRYIN registers
      CED => ce,               -- 1-bit active high clock enable input for D registers
      CEM => ce,               -- 1-bit active high clock enable input for multiplier registers
      CEOPMODE => ce,     -- 1-bit active high clock enable input for OPMODE registers
      CEP => ce,               -- 1-bit active high clock enable input for P registers
      RSTA => reset,             -- 1-bit reset input for A pipeline registers
      RSTB => reset,             -- 1-bit reset input for B pipeline registers
      RSTC => reset,             -- 1-bit reset input for C pipeline registers
      RSTCARRYIN => reset, -- 1-bit reset input for CARRYIN pipeline registers
      RSTD => reset,             -- 1-bit reset input for D pipeline registers
      RSTM => reset,             -- 1-bit reset input for M pipeline registers
      RSTOPMODE => reset,   -- 1-bit reset input for OPMODE pipeline registers
      RSTP => reset              -- 1-bit reset input for P pipeline registers
   );

  

  BC_bot <= (others => '0');
  PC_bot <= (others => '0');
  
end spartan6;








