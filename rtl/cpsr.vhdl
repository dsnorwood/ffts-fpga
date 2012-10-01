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

library floatfixlib;
use floatfixlib.fixed_pkg.all;


entity cpsr is
  generic (
    MAX_N        :    integer := 1024);
  port (
    clk, reset_n : in std_logic;

    wr_data : in complex_vector(3 downto 0);
    wr_addr : in int_vec(3 downto 0);
    wr_en   : in std_logic_vector(3 downto 0);

    rd_data : out complex_vector(3 downto 0);
    rd_addr : in  int_vec(3 downto 0);

    ready  : out std_logic;
    enable : in  std_logic;
    finished : out std_logic;
    n      : in  integer

    );

end cpsr;

architecture rtl of cpsr is

  component controller
    generic (
      MAX_N         : integer;
      MEM_WR_DELAYS : integer;
      MEM_RD_DELAYS : integer;
      BFLY_DELAYS   : integer);
    port (
      clk, reset_n : in  std_logic;
      enable       : in  std_logic;
      n            : in  integer;
      wr_addr      : out int_vec(3 downto 0);
      rd_addr      : out int_vec(3 downto 0);
      wr_en        : out std_logic_vector(3 downto 0);
      w_index      : out integer;
      mode         : out sr_mode_t;
      eof          : out std_logic);
  end component;
  
  component splitradix_generator
    generic (
      MAX_N        :     integer);
    port (
      clk, reset_n : in  std_logic;
      enable       : in  std_logic;
      n            : in  integer;
      base         : out integer;
      stride       : out integer;
      w_index      : out integer;
      mode         : out sr_mode_t;
      valid        : out std_logic);
  end component;

  component quad_interleaved_memory
    generic (
      DATA_WIDTH :     integer;
      DEPTH      :     integer);
    port (
      clk        : in  std_logic;
      reset_n    : in  std_logic;
      wr_data0   : in  std_logic_vector(DATA_WIDTH-1 downto 0);
      wr_addr0   : in  integer;
      wr_data1   : in  std_logic_vector(DATA_WIDTH-1 downto 0);
      wr_addr1   : in  integer;
      wr_data2   : in  std_logic_vector(DATA_WIDTH-1 downto 0);
      wr_addr2   : in  integer;
      wr_data3   : in  std_logic_vector(DATA_WIDTH-1 downto 0);
      wr_addr3   : in  integer;
      wr_en      : in  std_logic_vector(3 downto 0);
      rd_data0   : out std_logic_vector(DATA_WIDTH-1 downto 0);
      rd_addr0   : in  integer;
      rd_data1   : out std_logic_vector(DATA_WIDTH-1 downto 0);
      rd_addr1   : in  integer;
      rd_data2   : out std_logic_vector(DATA_WIDTH-1 downto 0);
      rd_addr2   : in  integer;
      rd_data3   : out std_logic_vector(DATA_WIDTH-1 downto 0);
      rd_addr3   : in  integer;
      rd_en      : in  std_logic_vector(3 downto 0);
      rd_valid   : out std_logic_vector(3 downto 0));
  end component;

  component subtransform
    generic (
      MAX_N        :     integer);
    port (
      clk, reset_n : in  std_logic;
      xs           : in  complex_vector(3 downto 0);
      ys           : out complex_vector(3 downto 0);
      w_addr       : in  integer;
      mode         : in  sr_mode_t;
      n            : in  integer);
  end component;

  component cp_bitreverse
    generic (
      MAX_N : integer);
    port (
      clk      : in  std_logic;
      addr_in  : in  integer;
      addr_out : out integer;
      n        : in  integer);
  end component;
  
  signal ram_rd_data : complex_vector(3 downto 0);
  signal ram_wr_data : complex_vector(3 downto 0);

  signal ram_rd_addr : int_vec(3 downto 0);
  signal ram_wr_addr : int_vec(3 downto 0);

  signal ram_wr_en : std_logic_vector(3 downto 0);

  signal ram_rd_en    : std_logic_vector(3 downto 0);
  signal ram_rd_valid : std_logic_vector(3 downto 0);

  signal main_wr_addr  : int_vec(3 downto 0);
  signal input_wr_addr : int_vec(3 downto 0);
  
  signal main_wr_data  : complex_vector(3 downto 0);
  signal input_wr_data : complex_vector(3 downto 0);
  
  signal main_wr_en    : std_logic_vector(3 downto 0);
  signal input_wr_en   : std_logic_vector(3 downto 0);

  signal main_rd_addr   : int_vec(3 downto 0);
  signal output_rd_addr : int_vec(3 downto 0);

  signal ram_wr_sel : std_logic;
  signal ram_rd_sel : std_logic;

  signal fly_mode    : sr_mode_t;
  signal fly_w_index : integer;

  type raw_data_vec is array (3 downto 0) of std_logic_vector(COMPLEX_WIDTH-1 downto 0);

  signal ram_rd_raw_data : raw_data_vec;


  signal main_ctrl_en : std_logic;
  signal main_eof    : std_logic;

  type state_t is (IO, PROCESSING);
  signal state : state_t;


  constant MEM_RD_DELAYS : integer := 5;
  constant MEM_WR_DELAYS : integer := 3;
  constant BFLY_DELAYS : integer := 8;
  
begin  -- rtl


  p_fsm : process (clk)
  begin  -- process p_fsm   
    if clk'event and clk = '1' then     -- rising clock edge
      if reset_n = '0' then
        state     <= IO;
      else
        if state = IO then
          if enable = '1' then
            state <= PROCESSING;
          end if;
        else
          if main_eof = '1' then
            state <= IO;
          end if;
        end if;
      end if;
    end if;
  end process p_fsm;

  p_mux_sels : process (state)
  begin  -- process p_mux_sels
    if state = IO then
      ram_wr_sel <= '1';
      ram_rd_sel <= '1';
      ready <= '1';
    else
      ram_wr_sel <= '0';
      ram_rd_sel <= '0';
      ready <= '0';
    end if;
  end process p_mux_sels;

  input_wr_addr <= wr_addr;
  input_wr_data <= wr_data;
  input_wr_en   <= wr_en;

  cp_bitreverse_0: cp_bitreverse
    generic map (
      MAX_N => MAX_N)
    port map (
      clk => clk,
      addr_in  => rd_addr(0),
      addr_out => output_rd_addr(0),
      n        => n);
  
  cp_bitreverse_1: cp_bitreverse
    generic map (
      MAX_N => MAX_N)
    port map (
      clk => clk,
      addr_in  => rd_addr(1),
      addr_out => output_rd_addr(1),
      n        => n);
  
  cp_bitreverse_2: cp_bitreverse
    generic map (
      MAX_N => MAX_N)
    port map (
      clk => clk,
      addr_in  => rd_addr(2),
      addr_out => output_rd_addr(2),
      n        => n);

  cp_bitreverse_3: cp_bitreverse
    generic map (
      MAX_N => MAX_N)
    port map (
      clk => clk,
      addr_in  => rd_addr(3),
      addr_out => output_rd_addr(3),
      n        => n);
  
--  output_rd_addr <= rd_addr;
  rd_data        <= ram_rd_data;

  p_ram_wr_data_mux : process(ram_wr_sel, input_wr_data, main_wr_data)
  begin  -- process p_ram_wr_mux
    if ram_wr_sel = '1' then
      ram_wr_data <= input_wr_data;
    else
      ram_wr_data <= main_wr_data;
    end if;
  end process p_ram_wr_data_mux;

  p_ram_wr_addr_mux : process(ram_wr_sel, input_wr_addr, main_wr_addr)
  begin  -- process p_ram_wr_mux
    if ram_wr_sel = '1' then
      ram_wr_addr <= input_wr_addr;
    else
      ram_wr_addr <= main_wr_addr;
    end if;
  end process p_ram_wr_addr_mux;

  p_ram_wr_en_mux : process(ram_wr_sel, input_wr_en, main_wr_en)
  begin  -- process p_ram_wr_mux
    if ram_wr_sel = '1' then
      ram_wr_en <= input_wr_en;
    else
      ram_wr_en <= main_wr_en;
    end if;
  end process p_ram_wr_en_mux;

  p_ram_rd_addr_mux : process(ram_rd_sel, output_rd_addr, main_rd_addr)
  begin  -- process p_ram_rd_mux
    if ram_rd_sel = '1' then
      ram_rd_addr <= output_rd_addr;
    else
      ram_rd_addr <= main_rd_addr;
    end if;
  end process p_ram_rd_addr_mux;

  ram_rd_en <= (others => '1');

  ram_inst : quad_interleaved_memory
    generic map (
      DATA_WIDTH => COMPLEX_WIDTH,
      DEPTH      => MAX_N)
    port map (
      clk        => clk,
      reset_n    => reset_n,
      wr_data0   => to_slv(ram_wr_data(0)),
      wr_addr0   => ram_wr_addr(0),
      wr_data1   => to_slv(ram_wr_data(1)),
      wr_addr1   => ram_wr_addr(1),
      wr_data2   => to_slv(ram_wr_data(2)),
      wr_addr2   => ram_wr_addr(2),
      wr_data3   => to_slv(ram_wr_data(3)),
      wr_addr3   => ram_wr_addr(3),
      wr_en      => ram_wr_en,
      rd_data0   => ram_rd_raw_data(0),
      rd_addr0   => ram_rd_addr(0),
      rd_data1   => ram_rd_raw_data(1),
      rd_addr1   => ram_rd_addr(1),
      rd_data2   => ram_rd_raw_data(2),
      rd_addr2   => ram_rd_addr(2),
      rd_data3   => ram_rd_raw_data(3),
      rd_addr3   => ram_rd_addr(3),
      rd_en      => ram_rd_en,
      rd_valid   => ram_rd_valid
      );

  p_convert_ram_rd_data : process (ram_rd_raw_data)
  begin  -- process p_convert_ram_rd_data
    for i in 3 downto 0 loop
      ram_rd_data(i) <= to_complex(ram_rd_raw_data(i));
    end loop;  -- i
  end process p_convert_ram_rd_data;


  SUBTRANSFORM0 : subtransform
    generic map (
      MAX_N   => MAX_N)
    port map (
      clk     => clk,
      reset_n => reset_n,
      xs      => ram_rd_data,
      ys      => main_wr_data,
      w_addr  => fly_w_index,
      mode    => fly_mode,
      n       => n
      );

  main_ctrl_en <= enable;
  
--   main_controller : splitradix_generator
--     generic map (
--       MAX_N   => MAX_N)
--     port map (
--       clk     => clk,
--       reset_n => reset_n,
--       enable  => main_ctrl_en,
--       n       => n,
--       base    => main_base,
--       stride  => main_stride,
--       w_index => main_w_index,
--       mode    => main_mode,
--       valid   => main_valid
--       );

  main_controller: controller
    generic map (
        MAX_N         => MAX_N,
        MEM_WR_DELAYS => MEM_WR_DELAYS,
        MEM_RD_DELAYS => MEM_RD_DELAYS,
        BFLY_DELAYS   => BFLY_DELAYS)
    port map (
        clk     => clk,
        reset_n => reset_n,
        enable  => main_ctrl_en,
        n       => n,
        wr_addr => main_wr_addr,
        rd_addr => main_rd_addr,
        wr_en   => main_wr_en,
        w_index => fly_w_index,
        mode    => fly_mode,
        eof    => main_eof
        );

  finished <= main_eof;
  
end rtl;
