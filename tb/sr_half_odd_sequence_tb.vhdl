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

-------------------------------------------------------------------------------

entity sr_half_odd_sequence_tb is

end sr_half_odd_sequence_tb;

-------------------------------------------------------------------------------

architecture tb of sr_half_odd_sequence_tb is

  component sr_half_odd_sequence
    generic (
      N_CORES   : integer;
      THIS_CORE : integer;
      MAX_N     : integer);
    port (
      clk, reset_n : in  std_logic;
      clk_en       : in  std_logic;
      n            : in  integer;
      rd_en        : in  std_logic;
      dout         : out integer;
      sync         : out std_logic;
      valid        : out std_logic);
  end component;
  component sr_half_odd_seq_ram
    generic (
      N_CORES   : integer;
      THIS_CORE : integer;
      MAX_N     : integer);
    port (
      clk, reset_n : in  std_logic;
      clk_en       : in  std_logic;
      n            : in  integer;
      rd_en        : in  std_logic;
      dout         : out integer;
      sync         : out std_logic;
      valid        : out std_logic);
  end component;
  
  -- component generics
  constant N_CORES   : integer := 1;
  constant THIS_CORE : integer := 0;
  constant MAX_N     : integer := 1024;

  -- component ports
  signal clk, reset_n : std_logic := '1'; 
  signal clk_en       : std_logic;
  signal n            : integer;
  signal rd_en        : std_logic;

  signal dout0         : integer;
  signal sync0         : std_logic;
  signal valid0        : std_logic;

  signal dout1         : integer;
  signal sync1         : std_logic;
  signal valid1        : std_logic;



begin  -- tb

  -- component instantiation
  DUT: sr_half_odd_sequence
    generic map (
      N_CORES   => N_CORES,
      THIS_CORE => THIS_CORE,
      MAX_N     => MAX_N)
    port map (
      clk     => clk,
      reset_n => reset_n,
      clk_en  => clk_en,
      n       => n,
      rd_en   => rd_en,
      dout    => dout0,
      sync    => sync0,
      valid   => valid0
      );

  sr_half_odd_seq_ram_1: sr_half_odd_seq_ram
    generic map (
      N_CORES   => N_CORES,
      THIS_CORE => THIS_CORE,
      MAX_N     => MAX_N)
    port map (
      clk     => clk,
      reset_n => reset_n,
      clk_en  => clk_en,
      n       => n,
      rd_en   => rd_en,
      dout    => dout1,
      sync    => sync1,
      valid   => valid1
      );
  
  -- clock generation
  clk <= not clk after 5 ns;

  -- waveform generation
  WaveGen_Proc: process
  begin
    n <= 128;
    clk_en <= '1';
    rd_en <= '0';
    wait for 50 ns;
    reset_n <= '0';
    wait for 50 ns;
    reset_n <= '1';
    wait for 50 ns;
    rd_en <= '1';
	 wait for 10 ns;
	 rd_en <= '0';
    wait;

  end process WaveGen_Proc;

  

end tb;

-------------------------------------------------------------------------------

configuration sr_half_odd_sequence_tb_tb_cfg of sr_half_odd_sequence_tb is
  for tb
  end for;
end sr_half_odd_sequence_tb_tb_cfg;

-------------------------------------------------------------------------------
