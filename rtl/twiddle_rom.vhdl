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

use ieee.math_real.all;

use work.fft_lib.all;

library floatfixlib;
use floatfixlib.fixed_pkg.all;

--library ieee_proposed;
--use ieee_proposed.fixed_pkg.all;

entity twiddle_rom is

  generic (
    MAX_N : integer := 1024
    );
  port (
    clk, reset_n : in  std_logic;
    addr         : in  integer;
    w            : out complex;
    
    n            : in  integer
    );

end twiddle_rom;

architecture rtl of twiddle_rom is

  type ram_t is array(0 to MAX_N/4-1) of word_t;

  function twiddle (k : in integer; n : in integer)
    return word_t is
  begin
    return to_sfixed(cos(-2.0 * MATH_PI * real(k) / real(n)), WORD_HINDEX, WORD_LINDEX);
  end twiddle;

  impure function generate_twiddles (N : in integer)
    return ram_t is

    variable temp_ram : ram_t;

  begin

    for I in 0 to N/4-1 loop
      temp_ram(I) := twiddle(I, N);
    end loop;

    return temp_ram;
  end function;

  signal ram : ram_t := generate_twiddles(MAX_N);

  signal w_a, w_b : word_t;

  signal cur_n : integer;

  signal addr_int : unsigned(log2(MAX_N/4)-1 downto 0);
 
  signal shiftamt : integer;
  
  signal n_div_4 : unsigned(log2(MAX_N/4)-1 downto 0);
  
  constant log2_max_n : integer := log2(MAX_N);
  
begin  -- rtl

  p_n_reg : process (clk, reset_n)
  begin
    if reset_n='0' then
      cur_n <= 0;

    elsif clk'event and clk = '1' then     -- rising clock edge
      cur_n <= n;
    end if;
  end process p_n_reg;

  p_addr: process (cur_n, addr)
  begin  -- process p_addr

    shiftamt <= log2_max_n - log2(cur_n);

    addr_int <= to_unsigned(addr, log2(MAX_N/4)) sll (log2_max_n - log2(cur_n));
    n_div_4 <= to_unsigned(cur_n, n_div_4'length) srl 2;

  end process p_addr;

  w <= COMPLEX'(w_a, resize(-w_b, w_b));

  p_ram : process (clk)
  begin
    if clk'event and clk = '1' then
      w_a <= ram(to_integer(addr_int));

      if addr_int = 0 then
        w_b <= to_sfixed(0, w_b);
      else

        w_b <= ram(to_integer( (n_div_4 sll (log2_max_n - log2(cur_n))) - addr_int ));

      end if;


    end if;
  end process p_ram;


end rtl;
