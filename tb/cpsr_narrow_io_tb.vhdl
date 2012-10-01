--                                    
--                              COPYRIGHT                             
--  
--  High-performance FFT library 
--  Copyright (c) 2010, The University of Waikato, New Zealand
--  All rights reserved.
--  
--  Author: Anthony Blake (a@anthonix.com)
--  
--  Redistribution and use in source and binary forms, with or without
--  modification, are permitted provided that the following conditions are met:
--      * Redistributions of source code must retain the above copyright
--        notice, this list of conditions and the following disclaimer.
--      * Redistributions in binary form must reproduce the above copyright
--        notice, this list of conditions and the following disclaimer in the
--        documentation and/or other materials provided with the distribution.
--      * Neither the name of the organization nor the
--        names of its contributors may be used to endorse or promote products
--        derived from this software without specific prior written permission.
--  
--  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
--  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
--  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
--  DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF WAIKATO BE LIABLE FOR ANY
--  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
--  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
--  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
--  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
--  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.fft_lib.all;
--use work.math_lib.all;

library floatfixlib;
use floatfixlib.fixed_pkg.all;

-------------------------------------------------------------------------------

entity cpsr_narrow_io_tb is

end cpsr_narrow_io_tb;

-------------------------------------------------------------------------------

architecture tb of cpsr_narrow_io_tb is

  component cpsr_narrow_io
    generic (
      MAX_N : integer);
    port (
      clk, reset_n : in  std_logic;
      wr_data      : in  complex;
      wr_addr      : in  unsigned(log2(MAX_N)-1 downto 0);
      wr_en        : in  std_logic;
      rd_data      : out complex;
      rd_addr      : in  unsigned(log2(MAX_N)-1 downto 0);
      ready        : out std_logic;
      enable       : in  std_logic;
      finished     : out std_logic;
      n            : in  unsigned(log2(MAX_N)-1 downto 0));
  end component;

  -- component generics
  constant MAX_N : integer := 1024;

  -- component ports
  signal clk, reset_n : std_logic := '1';
  signal wr_data      : complex;
  signal wr_addr      : unsigned(log2(MAX_N)-1 downto 0);
  signal wr_en        : std_logic;
  signal rd_data      : complex;
  signal rd_addr      : unsigned(log2(MAX_N)-1 downto 0);
  signal ready        : std_logic;
  signal enable       : std_logic;
  signal finished     : std_logic;
  signal n            : unsigned(log2(MAX_N)-1 downto 0);

  signal simulation_active : std_logic := '0';

  signal rd_re, rd_im : real;
begin  -- tb

  -- component instantiation
  DUT: cpsr_narrow_io
    generic map (
      MAX_N => MAX_N)
    port map (
      clk      => clk,
      reset_n  => reset_n,
      wr_data  => wr_data,
      wr_addr  => wr_addr,
      wr_en    => wr_en,
      rd_data  => rd_data,
      rd_addr  => rd_addr,
      ready    => ready,
      enable   => enable,
      finished => finished,
      n        => n);

  -- clock generation
  clk <= not clk after 5 ns;

  p_gen_reset : process
  begin  -- process p_get_reset
    wait for 100 ns;
    reset_n           <= '0';
    wait for 100 ns;
    reset_n           <= '1';
    wait until clk'event and clk = '1';
    simulation_active <= '1';
    wait;
  end process p_gen_reset;

  rd_re <= to_real(rd_data.re);
  rd_im <= to_real(rd_data.im);

  readcmd : process
  begin

    wr_addr <= to_unsigned(0, wr_addr'length);
    wr_en  <= '0';
    enable <= '0';
    n      <= to_unsigned(128, n'length);


    wait until simulation_active = '1';


    for i in 0 to to_integer(n)-1 loop
      wait until clk'event and clk = '1';
      wait for 2 ns;

        wr_data <= CMPLX(real(i), 0.0);

      wr_addr <= to_unsigned(i, wr_addr'length);
      wr_en   <= '1';

    end loop;

    wait until clk'event and clk = '1';
    wr_en <= '0';

    enable <= '1';
    wait until clk'event and clk = '1';
    enable <= '0';

    

    wait until finished'event and finished='1';

    for i in 0 to 15 loop
      wait until clk'event and clk = '1';      
    end loop;  -- i


    for i in 0 to to_integer(n)-1 loop
      wait until clk'event and clk = '1';
      wait for 2 ns;
      rd_addr <= to_unsigned(i, rd_addr'length);
    end loop;
      


    wait;
  end process;


  

end tb;

-------------------------------------------------------------------------------

configuration cpsr_narrow_io_tb_tb_cfg of cpsr_narrow_io_tb is
  for tb
  end for;
end cpsr_narrow_io_tb_tb_cfg;

-------------------------------------------------------------------------------
