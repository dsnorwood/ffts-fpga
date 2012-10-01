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

entity sr_half_odd_sequence is

  generic (
    N_CORES   : integer := 1;
    THIS_CORE : integer := 0;
    MAX_N     : integer := 1024);

  port (
    clk, reset_n : in  std_logic;
    clk_en : in std_logic;
    n            : in  integer;
    rd_en        : in  std_logic;
    dout         : out integer;
    sync         : out std_logic;
    valid        : out std_logic
    );

end sr_half_odd_sequence;

architecture rtl of sr_half_odd_sequence is

  function count_leading_ones (X : integer) return integer is
    variable x_int               : unsigned(log2(MAX_N)-1 downto 0);
  begin
    x_int := to_unsigned(X, x_int'length);
    for i in 0 to log2(MAX_N)-1 loop
      if x_int(i) = '0' then
        return i;
      end if;
    end loop;  -- i
    return 0;
  end count_leading_ones;

  function final_stage_next_state (X : integer) return integer is
  begin
    if X = 0 then
      return 1;
    end if;
    if count_leading_ones(X+2) mod 2 = 1 then
      return X+2;
    else
      return X+4;
    end if;
  end final_stage_next_state;

  function swap_one_zero (X : integer) return integer is

  begin
    if X = 0 then
      return 1;
    elsif X = 1 then
      return 0;

    end if;
    return X;
  end swap_one_zero;

  signal cur : integer;

  constant UNSIGNED_MAX : integer := log2(MAX_N/4);


  signal n_div_4 : integer;

  signal sync_int : std_logic;

  signal wait_bit : std_logic;

  signal valid_int : std_logic;

  signal running : std_logic;

  signal reset_pulse : std_logic;

  constant THIS_CORE_INT : integer := THIS_CORE;  --swap_one_zero(THIS_CORE);
begin  -- rtl

  sync <= sync_int and valid_int;

  n_div_4 <= to_integer(to_unsigned(n, log2(MAX_N)+1) srl (2 + log2(N_CORES)));

  dout <= cur;

-- valid_int <= not wait_bit;
-- valid <= valid_int;
  valid     <= '1';
  valid_int <= '1';

  p_reset_pulse : process(cur, n_div_4)
  begin  -- process p_reset_pulse
    if final_stage_next_state(cur) = ((THIS_CORE_INT+1) * n_div_4)+1 then
      reset_pulse <= '1';
    else
      reset_pulse <= '0';
    end if;
  end process p_reset_pulse;

  p_seq : process (clk, reset_n)
  begin  -- process p_seq
    if reset_n = '0' then               -- asynchronous reset (active low)
      cur      <= (THIS_CORE_INT * n_div_4) + 1;
      wait_bit <= '0';
      running  <= '0';

    elsif clk'event and clk = '1' and clk_en='1' then  -- rising clock edge

      if running = '0' then
        if rd_en = '1' then
          running <= '1';
        end if;
      else
        if reset_pulse = '1' then
          running <= '0';
        end if;
      end if;


      wait_bit <= '0';

      if (rd_en = '1' or running = '1') and wait_bit = '0' then

        if reset_pulse = '1' then
          cur <= (THIS_CORE_INT * n_div_4) + 1;
-- if count_leading_ones(THIS_CORE_INT) mod 2 = 0 then
-- wait_bit <= '1';
-- end if;
        else
          cur <= final_stage_next_state(cur);

        end if;

      end if;
    end if;
  end process p_seq;

  p_sync : process (cur, n_div_4)
  begin  -- process p_sync
    if cur = (THIS_CORE_INT * n_div_4) + 1 then
      sync_int <= '1';
    else
      sync_int <= '0';
    end if;
  end process p_sync;

end rtl;
