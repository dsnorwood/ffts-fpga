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

library ieee_proposed;
use ieee_proposed.fixed_pkg.all;

use work.fft_lib.all;

entity controller is

  generic (
    MAX_N         : integer := 1024;
    MEM_WR_DELAYS : integer := 3;
    MEM_RD_DELAYS : integer := 3;
    BFLY_DELAYS   : integer := 7
    );
  port (
    clk, reset_n : in std_logic;
    enable       : in std_logic;

    n : in unsigned(log2(MAX_N) downto 0);

    wr_addr : out int_vec(3 downto 0);
    rd_addr : out int_vec(3 downto 0);
    wr_en   : out std_logic_vector(3 downto 0);

    w_index : out integer;
    mode    : out sr_mode_t;
    eof     : out std_logic

    );

end controller;

architecture rtl of controller is

  component sr_generator
    generic (
      MAX_N         : integer;
      MEM_WR_DELAYS : integer := 3;
      MEM_RD_DELAYS : integer := 3;
      BFLY_DELAYS   : integer := 7
      );
    port (
      clk, reset_n : in  std_logic;
      enable       : in  std_logic;
      n            : in  unsigned(log2(MAX_N) downto 0);
      base         : out integer;
      stride       : out integer;
      w_index      : out integer;
      mode         : out sr_mode_t;
      eof          : out std_logic);
  end component;

  signal base        : integer;
  signal stride      : integer;
  signal w_index_int : integer;
  signal mode_int    : sr_mode_t;
  signal valid_int   : std_logic;

  type   mode_vec is array (natural range<>) of sr_mode_t;
  signal mode_delay_vec : mode_vec(MEM_RD_DELAYS-1 downto 0);

  signal w_index_delay_vec : int_vec(MEM_RD_DELAYS-1 downto 0);

  type   wr_en_vec is array (natural range<>) of std_logic_vector(3 downto 0);
  signal wr_en_delay_vec : wr_en_vec(MEM_RD_DELAYS+BFLY_DELAYS-1 downto 0);

  type   addr_vec is array (natural range<>) of int_vec(3 downto 0);
  signal wr_addr_delay_vec : addr_vec(MEM_RD_DELAYS+BFLY_DELAYS-1 downto 0);

  signal rd_addr_int : int_vec(3 downto 0);

  signal clk_int : std_logic;

  signal eof_delay_vec : std_logic_vector(MEM_WR_DELAYS+MEM_RD_DELAYS+BFLY_DELAYS downto 0);
  signal eof_int       : std_logic;

  signal processing : std_logic;

  signal processing_delay : std_logic_vector(6 downto 0);
  
begin  -- rtl

  p_proc : process (clk, reset_n)
  begin  -- process p_proc
    if reset_n = '0' then               -- asynchronous reset (active low)
      processing_delay <= (others => '0');
    elsif clk'event and clk = '1' then  -- rising clock edge

      if processing_delay(0) = '0' and enable = '1' then
        processing_delay(0) <= '1';
      elsif processing_delay(0) = '1' then
        if eof_int = '1' then
          processing_delay <= (others => '0');
        else
          for i in 1 to 6 loop
            processing_delay(i) <= processing_delay(i-1);
          end loop;  -- i
          
        end if;

      end if;

      
    end if;
  end process p_proc;

  processing <= processing_delay(6);

  clk_int <= clk;

  rd_addr <= rd_addr_int;

  p_rd_addr : process(base, stride, mode_int)
  begin  -- process p_rd_addr
    if mode_int = SRFFT_MODE_DUAL_R2 then
      rd_addr_int(0) <= base;
      rd_addr_int(2) <= base + stride;
      rd_addr_int(1) <= base + stride + stride;
      rd_addr_int(3) <= base + stride + stride + stride;
    else
      rd_addr_int(0) <= base;
      rd_addr_int(1) <= base + stride;
      rd_addr_int(2) <= base + stride + stride;
      rd_addr_int(3) <= base + stride + stride + stride;
    end if;
    
  end process p_rd_addr;

  controller_inst : sr_generator
    generic map (
      MAX_N         => MAX_N,
      MEM_WR_DELAYS => MEM_WR_DELAYS,
      MEM_RD_DELAYS => MEM_RD_DELAYS,
      BFLY_DELAYS   => BFLY_DELAYS
      )
    port map (
      clk     => clk_int,
      reset_n => reset_n,
      enable  => enable,
      n       => n,
      base    => base,
      stride  => stride,
      w_index => w_index_int,
      mode    => mode_int,
      eof     => eof_int
      );

  p_bfly_delays : process (clk)
  begin  -- process p_delays     
    if clk'event and clk = '1' then     -- rising clock edge
      mode_delay_vec(0)    <= mode_int;
      w_index_delay_vec(0) <= w_index_int;

      for i in 1 to MEM_RD_DELAYS-1 loop
        mode_delay_vec(i)    <= mode_delay_vec(i-1);
        w_index_delay_vec(i) <= w_index_delay_vec(i-1);
      end loop;  -- i
    end if;
  end process p_bfly_delays;

  mode    <= mode_delay_vec(MEM_RD_DELAYS-1);
  w_index <= w_index_delay_vec(MEM_RD_DELAYS-1);

  p_valid_int : process (mode_int, processing)
  begin  -- process p_valid_int
    if mode_int = SRFFT_MODE_NOP then
      valid_int <= '0';
    else
      valid_int <= processing;
    end if;
  end process p_valid_int;

  p_mem_wr_delays : process (clk)
  begin  -- process p_mem_wr_delays    
    if clk'event and clk = '1' then     -- rising clock edge
      wr_en_delay_vec(0)   <= valid_int & valid_int & valid_int & valid_int;
      wr_addr_delay_vec(0) <= rd_addr_int;
--        rd_addr_int(3) & rd_addr_int(2) & rd_addr_int(1) & rd_addr_int(0);

      for i in 1 to MEM_RD_DELAYS+BFLY_DELAYS-1 loop
        wr_en_delay_vec(i)   <= wr_en_delay_vec(i-1);
        wr_addr_delay_vec(i) <= wr_addr_delay_vec(i-1);
      end loop;  -- i
    end if;
  end process p_mem_wr_delays;

  wr_en   <= wr_en_delay_vec(MEM_RD_DELAYS+BFLY_DELAYS-1);
  wr_addr <= wr_addr_delay_vec(MEM_RD_DELAYS+BFLY_DELAYS-1);

  p_eof_delay : process (clk)
  begin  -- process p_eof_delay
    if clk'event and clk = '1' then     -- rising clock edge
      eof_delay_vec(0) <= eof_int;
      for i in 1 to MEM_WR_DELAYS+MEM_RD_DELAYS+BFLY_DELAYS loop
        eof_delay_vec(i) <= eof_delay_vec(i-1);
      end loop;  -- i
    end if;
  end process p_eof_delay;

  eof <= eof_delay_vec(MEM_WR_DELAYS+MEM_RD_DELAYS+BFLY_DELAYS);
  
end rtl;
