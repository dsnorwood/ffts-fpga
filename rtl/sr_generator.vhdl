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

entity sr_generator is

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

-- stage_syncs : inout std_logic_vector(N_CORES-1 downto 0);

--    stage_sync : out std_logic;
--    all_synced : in  std_logic;

    base    : out integer;
    stride  : out integer;
    w_index : out integer;
    mode    : out sr_mode_t;
    eof     : out std_logic
    );

end sr_generator;

architecture rtl of sr_generator is
  
  component sr_init_sequence
    generic (
      N_CORES      : integer;
      THIS_CORE    : integer;
      MAX_N        : integer;
      PHASE_OFFSET : integer);
    port (
      clk, reset_n : in  std_logic;
      clk_en       : in  std_logic;
      n            : in  unsigned(log2(MAX_N) downto 0);
      rd_en        : in  std_logic;
      dout         : out integer;
      valid        : out std_logic;
      sync         : out std_logic);
  end component;

  component sr_half_odd_sequence
    generic (
      N_CORES   : integer;
      THIS_CORE : integer;
      MAX_N     : integer);
    port (
      clk, reset_n : in  std_logic;
      clk_en       : in  std_logic;
      n            : in  unsigned(log2(MAX_N) downto 0);
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
      n            : in  unsigned(log2(MAX_N) downto 0);
      rd_en        : in  std_logic;
      dout         : out integer;
      sync         : out std_logic;
      valid        : out std_logic);
  end component;

  component sr_fifo
    generic (
      N_CORES   : integer;
      THIS_CORE : integer;
      MAX_N     : integer);
    port (
      clk, reset_n : in  std_logic;
      clk_en       : in  std_logic;
      rd_en        : in  std_logic;
      wr_en        : in  std_logic;
      stage_in     : in  std_logic;
      addr_in      : in  integer;
      pos_in       : in  integer;
      sync         : out std_logic;
      valid        : out std_logic;
      empty        : out std_logic;
      stage_out    : out std_logic;
      addr_out     : out integer;
      pos_out      : out integer);
  end component;

  component sr_next_bfly
    generic (
      MAX_N : integer);
    port (
      stage_in          : in  std_logic;
      pos_in, addr_in   : in  integer;
      stage_out         : out std_logic;
      pos_out, addr_out : out integer;
      wr_en             : out std_logic);
  end component;

  component sr_decode_bfly
    generic (
      MAX_N     : integer;
      N_CORES   : integer;
      THIS_CORE : integer);
    port (
		clk             : in std_logic;
      last_stage      : in  std_logic;
      n_in            : in  unsigned(log2(MAX_N) downto 0);
      pos_in, addr_in : in  integer;
      base_out        : out integer;
      stride_out      : out integer;
      w_index_out     : out integer;
      mode_out        : out sr_mode_t);
  end component;

  constant UNSIGNED_MAX : integer := log2(MAX_N) + 1;

  type bfly_state_t is
  record
    addr  : unsigned(UNSIGNED_MAX-1 downto 0);
    pos   : unsigned(UNSIGNED_MAX-1 downto 0);
    stage : std_logic;
  end record;

  constant INIT_PHASE_OFFSET : integer := 1;



  signal half_odd_valid : std_logic;
  signal half_odd_sync  : std_logic;
  signal half_odd_addr  : integer;
  signal half_odd_rd_en : std_logic;


  signal init_sync  : std_logic;
  signal init_rd_en : std_logic;
  signal init_addr  : integer;
  signal init_valid : std_logic;

  signal fifo_din   : bfly_state_t;
  signal fifo_dout  : bfly_state_t;
  signal fifo_sync  : std_logic;
  signal fifo_valid : std_logic;
  signal fifo_empty : std_logic;

  signal fifo_rd_en : std_logic;
  signal fifo_wr_en : std_logic;

  signal fifo_finish_pulse : std_logic;

  type   state_t is (IDLE, INIT_SEQ0, INIT_SEQ1, RUN_FIFO, HALF_ODD_SEQ0, HALF_ODD_SEQ1);
  signal state    : state_t;
  signal state_r0 : state_t;
  signal state_r1 : state_t;

  type   mux_sel_t is (HALF_ODD, INIT, FIFO);
  signal mux_sel : mux_sel_t;


  signal init_dout     : bfly_state_t;
  signal half_odd_dout : bfly_state_t;
  signal bfly_muxed    : bfly_state_t;
  signal bfly_muxed_r  : bfly_state_t;

  signal base_int    : integer;
  signal stride_int  : integer;
  signal mode_int    : sr_mode_t;
  signal w_index_int : integer;

  signal last_stage_pulse : std_logic;

  signal init_sync_pulse : std_logic;

  signal stage_sync_pulse : std_logic;

  signal cur_stage_sync : std_logic;

  signal clk_en : std_logic;

  signal counter : unsigned(UNSIGNED_MAX-1 downto 0);

  type mode_delay_vec is array (natural range<>) of sr_mode_t;

  signal base_delays    : int_vec(3 downto 0);
  signal stride_delays  : int_vec(3 downto 0);
  signal w_index_delays : int_vec(3 downto 0);
  signal mode_delays    : mode_delay_vec(3 downto 0);

  signal clk_en_delays : std_logic_vector(4 downto 0);
  signal eof_delays    : std_logic_vector(4 downto 0);

  signal eof_int : std_logic;

  signal log2n_uint : unsigned(UNSIGNED_MAX-1 downto 0);

  signal fifo_out_addr : integer;
  signal fifo_out_pos  : integer;

  signal next_bfly_addr  : integer;
  signal next_bfly_pos   : integer;
  signal next_bfly_stage : std_logic;
  signal next_bfly_wr_en : std_logic;

  signal n_int : unsigned(log2(MAX_N) downto 0);
  
begin-- rtl


  
  p_mode_mux : process (clk_en_delays(3), mode_delays(3))
  begin  -- process p_mode_mux
    if clk_en_delays(3) = '1' then
      mode <= mode_delays(3);
    else
      mode <= SRFFT_MODE_NOP;
    end if;
  end process p_mode_mux;

  base    <= base_delays(3);
  stride  <= stride_delays(3);
  w_index <= w_index_delays(3);
  eof     <= eof_delays(4);

  p_clk_en_delays : process (clk, reset_n)
  begin  -- process p_clk_en_delays
    if reset_n = '0' then               -- asynchronous reset (active low)
      clk_en_delays <= (others => '0');
      eof_delays    <= (others => '0');
      
    elsif clk'event and clk = '1' then  -- rising clock edge
      clk_en_delays(0) <= clk_en;
      eof_delays(0)    <= eof_int;
      for i in 1 to 4 loop
        clk_en_delays(i) <= clk_en_delays(i-1);
        eof_delays(i)    <= eof_delays(i-1);
      end loop;  -- i
    end if;
  end process p_clk_en_delays;

  p_delays : process (clk, reset_n)
  begin  -- process p_delays
    if reset_n = '0' then               -- asynchronous reset (active low)
      base_delays    <= (others => 0);
      stride_delays  <= (others => 0);
      w_index_delays <= (others => 0);
      mode_delays    <= (others => SRFFT_MODE_NOP);
      
      
    elsif clk'event and clk = '1'  then  -- rising clock edge
      if clk_en_delays(3) = '1' then
      base_delays(0)    <= base_int;
      stride_delays(0)  <= stride_int;
      w_index_delays(0) <= w_index_int;
      mode_delays(0)    <= mode_int;


      for i in 1 to 3 loop
        base_delays(i)    <= base_delays(i-1);
        stride_delays(i)  <= stride_delays(i-1);
        w_index_delays(i) <= w_index_delays(i-1);
        mode_delays(i)    <= mode_delays(i-1);

      end loop;  -- i
    end if;
    end if;
  end process p_delays;

  p_counter : process (clk, reset_n)
  begin  -- process p_counter
    if reset_n = '0' then               -- asynchronous reset (active low)
      counter <= to_unsigned(0, counter'length);
    elsif clk'event and clk = '1' then  -- rising clock edge

      if to_integer(counter) /= 0 then
        counter <= counter - to_unsigned(1, counter'length);
      else
        if bfly_muxed.addr = 0 then
          counter <= to_unsigned(BFLY_DELAYS + MEM_WR_DELAYS + MEM_RD_DELAYS -1, counter'length);
        end if;
      end if;
      
    end if;
  end process p_counter;

  p_clken : process (clk, reset_n)
  begin  -- process p_clken
    if reset_n = '0' then               -- asynchronous reset (active low)
      clk_en <= '1';
    elsif clk'event and clk = '1' then  -- rising clock edge

      if clk_en = '0' then
        if counter = 0 then
          clk_en <= '1';
        end if;
      elsif bfly_muxed.addr = 0 and to_integer(counter) /= 0 then
        clk_en <= '0';
      else
        clk_en <= '1';
      end if;

    end if;
  end process p_clken;

  p_stage_sync_pulse : process (bfly_muxed.stage, cur_stage_sync)
  begin  -- process p_stage_sync_pulse
    if bfly_muxed.stage /= cur_stage_sync then
      stage_sync_pulse <= '1';
    else
      stage_sync_pulse <= '0';
    end if;
  end process p_stage_sync_pulse;

  p_stage_sync : process (clk, reset_n)
  begin  -- process p_stage_sync
    if reset_n = '0' then               -- asynchronous reset (active low)
      cur_stage_sync <= '0';
    elsif clk'event and clk = '1' then  -- rising clock edge

      if state = IDLE then
        cur_stage_sync <= '0';
      else
        if stage_sync_pulse = '1' then
          cur_stage_sync <= bfly_muxed.stage;
        end if;
        
      end if;
    end if;
  end process p_stage_sync;

--  stage_sync <= stage_sync_pulse;



  init_dout.stage <= '0';
  init_dout.pos   <= n_int srl 2;

  log2n_uint <= to_unsigned(log2(n_int), UNSIGNED_MAX);

  half_odd_dout.stage <= not (log2n_uint(0));
  half_odd_dout.pos   <= to_unsigned(1, half_odd_dout.pos'length);

-- p_valid : process (state, state_r0)
-- begin                                -- process p_valid
--     if state = IDLE or state = INIT_SEQ0 or state_r0 = INIT_SEQ0 then
--       valid_int <= '0';
--     else
--       valid_int <= '1';
--     end if;
--   end process p_valid;

  p_multiplexor : process (clk, reset_n)
  begin  -- process p_multiplexor
    if reset_n = '0' then               -- asynchronous reset (active low)

      init_dout.addr <= to_unsigned(0, init_dout.addr'length);

      half_odd_dout.addr <= to_unsigned(0, half_odd_dout.addr'length);

    elsif clk'event and clk = '1' then  -- rising clock edge

      init_dout.addr     <= to_unsigned(init_addr, init_dout.addr'length);
      half_odd_dout.addr <= to_unsigned(half_odd_addr, half_odd_dout.addr'length);


      case state_r0 is

        when INIT_SEQ0 | INIT_SEQ1 =>
          bfly_muxed <= init_dout;
        when HALF_ODD_SEQ0 | HALF_ODD_SEQ1 =>
          bfly_muxed <= half_odd_dout;
        when others =>
          bfly_muxed <= fifo_dout;
      end case;


    end if;
  end process p_multiplexor;

  p_state_delay : process (clk, reset_n)
  begin  -- process p_state_delay
    if reset_n = '0' then               -- asynchronous reset (active low)
      state_r0 <= IDLE;
      state_r1 <= IDLE;
    elsif clk'event and clk = '1' then  -- rising clock edge
      state_r0 <= state;
      state_r1 <= state_r0;
    end if;
  end process p_state_delay;

  p_fifo_rd_en : process (state, init_sync)
  begin  -- process p_fifo_rd_en
    if init_sync = '1' or state = RUN_FIFO then
      fifo_rd_en <= '1';
    else
      fifo_rd_en <= '0';
    end if;
  end process p_fifo_rd_en;

  p_init_rd_en : process (state)
  begin  -- process p_init_rd_en
    if state = INIT_SEQ0 or state = INIT_SEQ1 then
      init_rd_en <= '1';
    else
      init_rd_en <= '0';
    end if;
  end process p_init_rd_en;


  p_half_odd_rd_en : process (state)
  begin  -- process p_half_odd_seq_rd_en
    if state = HALF_ODD_SEQ0 or state = HALF_ODD_SEQ1 then
      half_odd_rd_en <= '1';
    else
      half_odd_rd_en <= '0';
    end if;
  end process p_half_odd_rd_en;

  p_last_stage_pulse : process(clk, reset_n)
  begin  -- process p_last_stage_pulse
    if reset_n = '0' then
      last_stage_pulse <= '0';
    elsif clk'event and clk = '1' then
      if state_r0 = HALF_ODD_SEQ0 or state_r0 = HALF_ODD_SEQ1 or (state_r1 = HALF_ODD_SEQ1) then
        last_stage_pulse <= '1';
              
      else
        last_stage_pulse <= '0';
                      
      end if;

    end if;
  end process p_last_stage_pulse;

  p_fsm : process (clk, reset_n)
  begin  -- process p_fsm
    if reset_n = '0' then               -- asynchronous reset (active low)
      state   <= IDLE;
      eof_int <= '0';
    elsif clk'event and clk = '1' then  -- rising clock edge

      eof_int <= '0';

      case state is
        when IDLE =>
          if enable = '1' then
            state <= INIT_SEQ0;
          end if;
        when INIT_SEQ0 =>
          state <= INIT_SEQ1;
        when INIT_SEQ1 =>
          if init_sync = '1' then
            state <= RUN_FIFO;
          end if;
        when RUN_FIFO =>
          if fifo_finish_pulse = '1' then
            state <= HALF_ODD_SEQ0;
          end if;
        when HALF_ODD_SEQ0 =>
          state <= HALF_ODD_SEQ1;
        when HALF_ODD_SEQ1 =>
          if half_odd_sync = '1' then
            eof_int <= '1';
            state   <= IDLE;
          end if;

        when others => state <= IDLE;
      end case;

    end if;
  end process p_fsm;

  fifo_finish_pulse <= fifo_empty;

  init_seq_inst : sr_init_sequence
    generic map (
      N_CORES      => 1,
      THIS_CORE    => 0,
      MAX_N        => MAX_N,
      PHASE_OFFSET => INIT_PHASE_OFFSET
      )
    port map (
      clk     => clk,
      reset_n => reset_n,
      clk_en  => clk_en,
      n       => n_int,
      rd_en   => init_rd_en,
      dout    => init_addr,
      valid   => init_valid,
      sync    => init_sync

      );


  --half_odd_seq_inst : sr_half_odd_sequence
  --  generic map (
  --    N_CORES   => 1,
  --    THIS_CORE => 0,
  --    MAX_N     => MAX_N)
  --  port map (
  --    clk     => clk,
  --    reset_n => reset_n,
  --    clk_en => clk_en,
  --    n       => n,
  --    rd_en   => half_odd_rd_en,
  --    dout    => half_odd_addr,
  --    sync    => half_odd_sync,
  --    valid   => half_odd_valid
  --    );
  sr_half_odd_seq_ram_1 : sr_half_odd_seq_ram
    generic map (
      N_CORES   => 1,
      THIS_CORE => 0,
      MAX_N     => MAX_N)
    port map (
      clk     => clk,
      reset_n => reset_n,
      clk_en  => clk_en,
      n       => n_int,
      rd_en   => half_odd_rd_en,
      dout    => half_odd_addr,
      sync    => half_odd_sync,
      valid   => half_odd_valid);

  fifo_inst : sr_fifo
    generic map (
      N_CORES   => 1,
      THIS_CORE => 0,
      MAX_N     => MAX_N)
    port map (
      clk       => clk,
      reset_n   => reset_n,
      clk_en    => clk_en,
      rd_en     => fifo_rd_en,
      wr_en     => fifo_wr_en,
      stage_in  => fifo_din.stage,
      addr_in   => to_integer(fifo_din.addr),
      pos_in    => to_integer(fifo_din.pos),
      sync      => fifo_sync,
      valid     => fifo_valid,
      empty     => fifo_empty,
      stage_out => fifo_dout.stage,
      addr_out  => fifo_out_addr,
      pos_out   => fifo_out_pos
      );

  fifo_dout.addr <= to_unsigned(fifo_out_addr, fifo_dout.addr'length);
  fifo_dout.pos  <= to_unsigned(fifo_out_pos, fifo_dout.pos'length);

  p_bfly_muxed_r : process (clk)
  begin  -- process p_bfly_muxed_r
    if clk'event and clk = '1' then     -- rising clock edge
      bfly_muxed_r <= bfly_muxed;
    end if;
  end process p_bfly_muxed_r;

  next_bfly_inst : sr_next_bfly
    generic map (
      MAX_N => MAX_N)
    port map (
      stage_in  => bfly_muxed_r.stage,
      pos_in    => to_integer(bfly_muxed_r.pos),
      addr_in   => to_integer(bfly_muxed_r.addr),
      stage_out => next_bfly_stage,
      pos_out   => next_bfly_pos,
      addr_out  => next_bfly_addr,
      wr_en     => next_bfly_wr_en
      );

  p_fifo : process (clk)
  begin  -- process p_fifo
    if clk'event and clk = '1' then     -- rising clock edge
      fifo_wr_en     <= next_bfly_wr_en;
      fifo_din.stage <= next_bfly_stage;
      fifo_din.addr  <= to_unsigned(next_bfly_addr, fifo_din.addr'length);
      fifo_din.pos   <= to_unsigned(next_bfly_pos, fifo_din.pos'length);
      
    end if;
  end process p_fifo;

  decode_bfly_inst : sr_decode_bfly
    generic map (
      MAX_N     => MAX_N,
      N_CORES   => 1,
      THIS_CORE => 0)
    port map (
		clk         => clk,
      last_stage  => last_stage_pulse,
      n_in        => n_int,
      pos_in      => to_integer(bfly_muxed_r.pos),
      addr_in     => to_integer(bfly_muxed_r.addr),
      base_out    => base_int,
      stride_out  => stride_int,
      w_index_out => w_index_int,
      mode_out    => mode_int
      );


  p_n_int : process (clk)
  begin  -- process p_n_int
    if clk'event and clk = '1' then     -- rising clock edge
      n_int <= n;
    end if;
  end process p_n_int;
end rtl;
