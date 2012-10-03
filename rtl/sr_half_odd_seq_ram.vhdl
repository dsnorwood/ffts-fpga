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

entity sr_half_odd_seq_ram is

  generic (
    N_CORES   : integer := 1;
    THIS_CORE : integer := 0;
    MAX_N     : integer := 1024);

  port (
    clk, reset_n : in  std_logic;
    clk_en       : in  std_logic;
    clk_en_adv   : in std_logic;
    n            : in  unsigned(log2(MAX_N) downto 0);
    rd_en        : in  std_logic;
    dout         : out integer;
    sync         : out std_logic;
    valid        : out std_logic
    );

end sr_half_odd_seq_ram;

architecture rtl of sr_half_odd_seq_ram is

  function count_leading_ones (X : integer) return integer is
    variable x_int : unsigned(log2(MAX_N)-1 downto 0);
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
    elsif count_leading_ones(X+2) mod 2 = 1 then
      return X+2;
    else
      return X+4;
    end if;
  end final_stage_next_state;

  function sequence_length (N : integer) return integer is
    variable value : integer := 0;
    variable count : integer := 0;
  begin

    while value < N loop
      value := final_stage_next_state(value);
      count := count + 1;
    end loop;
    return count-1;
  end sequence_length;

  type ram_t is array (0 to sequence_length(MAX_N/4)-1) of unsigned(log2(MAX_N)-1 downto 0);


  impure function generate_half_odd_sequence (N : in integer)
    return ram_t is
    variable temp_ram : ram_t;
    variable value    : integer := 1;
  begin

    for I in 0 to N-1 loop
      
      temp_ram(I) := to_unsigned(value, temp_ram(I)'length);
      value       := final_stage_next_state(value);
    end loop;

    return temp_ram;
  end function;

  signal ram : ram_t := generate_half_odd_sequence(sequence_length(MAX_N/4));

  function swap_one_zero (X : integer) return integer is

  begin
    if X = 0 then
      return 1;
    elsif X = 1 then
      return 0;

    end if;
    return X;
  end swap_one_zero;

  constant UNSIGNED_MAX : integer := log2(MAX_N/4);

  signal addr_int : integer := 0;
  signal data_int : unsigned(log2(MAX_N)-1 downto 0);

  signal dout_int : integer;

  signal n_div_4 : integer;

  signal sync_int : std_logic;

  signal valid_int : std_logic;

  signal running : std_logic := '0';

  signal reset_pulse : std_logic;

  constant THIS_CORE_INT : integer := THIS_CORE;  --swap_one_zero(THIS_CORE);
  
  signal n_div_4_mod : unsigned(log2(MAX_N) downto 0);
  
begin  -- rtl

  process(clk)
  begin
    if clk'event and clk='1' then
      if to_unsigned(log2(n), log2(MAX_N))(0)='0' then
        n_div_4_mod <= to_unsigned(n_div_4, n_div_4_mod'length) - to_unsigned(3, n_div_4_mod'length);    
      else
        n_div_4_mod <= to_unsigned(n_div_4, n_div_4_mod'length) - to_unsigned(1, n_div_4_mod'length);    
      end if; 
    end if;
  end process;
  
  p_ram : process (clk)
  begin  -- process p_ram
    if clk'event and clk = '1' then     -- rising clock edge
      data_int <= ram(addr_int);
      n_div_4 <= to_integer(n srl (2 + log2(N_CORES)));
    end if;
  end process p_ram;

  p_address_logic : process (clk, reset_n)
  begin  -- process p_address_logic
    if reset_n = '0' then               -- asynchronous reset (active low)
      addr_int <= 0;
    elsif clk'event and clk = '1'  then  -- rising clock edge

  if clk_en_adv='1' then
      if reset_pulse='1' then
        addr_int <= 0;
--      end if;
--      if rd_en = '1' and running='0' then
--        addr_int <= 2;
      elsif running='1' or (rd_en='1' and running='0') then
        addr_int <= addr_int + 1;
      else
        addr_int <= 0;
      end if;
    end if;
    end if;
  end process p_address_logic;

	dout <= to_integer(data_int);
	valid <= '1';
	sync <= not (running or reset_pulse);
	
  p_out_regs: process (clk, reset_n)
  begin  -- process p_out_regs
    if reset_n = '0' then               -- asynchronous reset (active low)
     
      dout_int <= 0;
--      valid <= '0';
--      sync <= '0';
    elsif clk'event and clk = '1'  then  -- rising clock edge
  if clk_en ='1' then
      if running='1' then
        dout_int <= to_integer(data_int);
      else
        dout_int <= 1;
      end if;
    end if;
      

--      if running='1' then
--        valid <= '1';
--      else
--        valid <= '0';
--      end if;

--      if reset_pulse='1' then
--        sync <= '1';
--      else
--        sync <= '0';
--      end if;
      
    end if;
  end process p_out_regs;



-- valid_int <= not wait_bit;
-- valid <= valid_int;
  --valid     <= '1';
  --valid_int <= '1';

  p_reset_pulse : process(data_int, n_div_4_mod)
  begin  -- process p_reset_pulse
    if to_integer(data_int) = n_div_4_mod then
      reset_pulse <= '1';
    else
      reset_pulse <= '0';
    end if;
  end process p_reset_pulse;

  p_seq : process (clk, reset_n)
  begin  -- process p_seq
    if reset_n = '0' then               -- asynchronous reset (active low)
      running  <= '0';

    elsif clk'event and clk = '1'  then  -- rising clock edge

  if clk_en='1' then
      if running = '0' then
        if rd_en = '1' then
          running <= '1';
        end if;
      else
        if reset_pulse = '1' then
          running <= '0';
        end if;
      end if;

end if;
    end if;
  end process p_seq;

 
end rtl;
