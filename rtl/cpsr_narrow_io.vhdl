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

entity cpsr_narrow_io is

  generic (
    MAX_N : integer := 1024);
  port (
    clk, reset_n : in  std_logic;
    wr_data      : in  complex;
    wr_addr      : in  unsigned(log2(MAX_N)-1 downto 0);
    wr_en        : in  std_logic;
    rd_data      : out complex;
    rd_addr      : in unsigned(log2(MAX_N)-1 downto 0);
    ready        : out std_logic;
    enable       : in  std_logic;
    finished     : out std_logic;
    n            : in  unsigned(log2(MAX_N)-1 downto 0)
    );

end cpsr_narrow_io;


architecture rtl of cpsr_narrow_io is

  component cpsr
    generic (
      MAX_N : integer);
    port (
      clk, reset_n : in  std_logic;
      wr_data      : in  complex_vector(3 downto 0);
      wr_addr      : in  int_vec(3 downto 0);
      wr_en        : in  std_logic_vector(3 downto 0);
      rd_data      : out complex_vector(3 downto 0);
      rd_addr      : in  int_vec(3 downto 0);
      ready        : out std_logic;
      enable       : in  std_logic;
      finished     : out std_logic;
      n            : in  integer);
  end component;

  signal n_int       : integer;
  signal wr_addr_int : int_vec(3 downto 0);
  signal rd_addr_int : int_vec(3 downto 0);

  signal wr_data_int : complex_vector(3 downto 0);
  signal rd_data_int : complex_vector(3 downto 0);

  signal wr_en_int : std_logic_vector(3 downto 0);
  
begin  -- rtl

  wr_data_int(0) <= wr_data;
  wr_data_int(1) <= CZERO;
  wr_data_int(2) <= CZERO;
  wr_data_int(3) <= CZERO;

  wr_addr_int(0) <= to_integer(wr_addr);
  wr_addr_int(1) <= 0;
  wr_addr_int(2) <= 0;
  wr_addr_int(3) <= 0;

  rd_addr_int(0) <= to_integer(rd_addr);
  rd_addr_int(1) <= 0;
  rd_addr_int(2) <= 0;
  rd_addr_int(3) <= 0;

  n_int       <= to_integer(n);

  wr_en_int <=  '0' & '0' & '0' & wr_en;
  
  rd_data <= rd_data_int(0);
  
  cpsr_1 : cpsr
    generic map (
      MAX_N => MAX_N)
    port map (
      clk      => clk,
      reset_n  => reset_n,
      wr_data  => wr_data_int,
      wr_addr  => wr_addr_int,
      wr_en    => wr_en_int,
      rd_data  => rd_data_int,
      rd_addr  => rd_addr_int,
      ready    => ready,
      enable   => enable,
      finished => finished,
      n        => n_int);



end rtl;
