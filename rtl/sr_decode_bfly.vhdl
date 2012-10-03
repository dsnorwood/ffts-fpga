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

entity sr_decode_bfly is

  generic (
    MAX_N     : integer := 1024;
    N_CORES   : integer := 1;
    THIS_CORE : integer := 0
    );

  port (

	clk : in std_logic;
    last_stage : in std_logic;
    
    n_in            : in unsigned(log2(MAX_N) downto 0);
    pos_in, addr_in : in integer;

    base_out    : out integer;
    stride_out  : out integer;
    w_index_out : out integer;
    mode_out    : out sr_mode_t
    
    );

end sr_decode_bfly;

architecture rtl of sr_decode_bfly is

  constant UNSIGNED_MAX : integer := log2(MAX_N)+1;

  function base_decode(addr : integer; pos : integer) return integer is
    variable temp           : integer;
    variable rval           : integer;
  begin
    temp := to_integer(unsigned(-to_signed(pos, UNSIGNED_MAX)));
    rval := to_integer(((to_unsigned(temp, UNSIGNED_MAX) and to_unsigned(addr, UNSIGNED_MAX)) sll 2)
                       or (not(to_unsigned(temp, UNSIGNED_MAX)) and to_unsigned(addr, UNSIGNED_MAX)));
    return rval;
  end base_decode;


  function log2_lsb(x : integer) return integer is
    variable x_int    : unsigned(UNSIGNED_MAX-1 downto 0) := to_unsigned(x, UNSIGNED_MAX);
  begin
    for i in 0 to x_int'length-1 loop
      if x_int(i) = '1' then
        return i+1;
      end if;
    end loop;
    return 0;
  end log2_lsb;

  function w_decode(addr : integer; pos : integer; n : integer) return integer is
  begin
    return to_integer((to_unsigned(addr, UNSIGNED_MAX) sll (log2_lsb(n)-log2_lsb(pos))) and to_unsigned(n-1, UNSIGNED_MAX));
  end w_decode;

  signal base_out_int    : integer;
  signal mode_out_int    : sr_mode_t;
  signal w_index_out_int : integer;
  signal stride_out_int  : integer;

--   constant MODE_DOUBLE_RADIX2         : integer := 3;
--   constant MODE_SPLITRADIX_AND_RADIX2 : integer := 2;
--   constant MODE_SPLITRADIX            : integer := 0;
	signal n_int : unsigned(log2(MAX_N) downto 0);
begin  -- rtl 

  base_out    <= base_out_int;
  stride_out  <= stride_out_int;
  w_index_out <= w_index_out_int;
  mode_out    <= mode_out_int;

  base_out_int    <= base_decode(addr_in, pos_in);
  stride_out_int <= pos_in;
-- stride_out_int <= pos_in;
  w_index_out_int <= w_decode(addr_in, pos_in, to_integer(n_int srl 2));--/4/N_CORES);

	process (clk) 
	begin
	if clk'event and clk='1' then
		n_int <= n_in;--to_unsigned(MAX_N, n_int'length);
		
	end if;
	end process;

  p_mode : process (w_index_out_int, last_stage, n_int, addr_in, pos_in)
  begin  -- process p_mode

    
--  if stage_in = log2(n_in)-2 then     
    if last_stage = '1' then
      if addr_in = 0 then
        mode_out_int <= SRFFT_MODE_CPSR_R2;--MODE_SPLITRADIX_AND_RADIX2;
      else
        mode_out_int <= SRFFT_MODE_DUAL_R2;--MODE_DOUBLE_RADIX2;
      end if;

--      stride_out_int <= 1;


    else
      if pos_in = 1 and w_index_out_int = 0 then
        mode_out_int <= SRFFT_MODE_CPSR_R2;--MODE_SPLITRADIX_AND_RADIX2;
      else
        mode_out_int <= SRFFT_MODE_CPSR;--MODE_SPLITRADIX;
      end if;

  

    end if;
  end process p_mode;

end rtl;

