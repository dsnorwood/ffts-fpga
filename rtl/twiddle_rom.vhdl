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

library ieee_proposed;
use ieee_proposed.fixed_pkg.all;

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
    
    n            : in  unsigned(log2(MAX_N) downto 0)
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

  signal w_a, w_b, w_a0, w_b0 : word_t;

  signal cur_n : unsigned(log2(MAX_N) downto 0);

  signal addr_int, addr_int_r, addr_int_sub : unsigned(log2(MAX_N/4)-1 downto 0);
 
  signal shiftamt : unsigned(log2(MAX_N) downto 0);
  
  signal n_div_4, addr_b : unsigned(log2(MAX_N) downto 0);
  
  constant log2_max_n : integer := log2(MAX_N);
  
  signal addr_probe : integer;
begin  -- rtl

  p_n_reg : process (clk, reset_n)
  begin
    if reset_n='0' then
      cur_n <= to_unsigned(0, cur_n'length);

    elsif clk'event and clk = '1' then     -- rising clock edge
      cur_n <= n;
      addr_b <= (n_div_4 sll (log2_max_n - log2(cur_n))) ;
      
      shiftamt <= to_unsigned(log2_max_n, shiftamt'length) - log2(cur_n);
      n_div_4 <= cur_n srl 2;
      
      addr_int     <= to_unsigned(addr, log2(MAX_N/4)) sll to_integer(shiftamt);
      addr_int_sub <= resize(addr_b - (to_unsigned(addr, addr_b'length) sll to_integer(shiftamt)), addr_int_sub'length);
    end if;
  end process p_n_reg;

  p_addr: process (shiftamt, addr)
  begin  -- process p_addr

   

    
    

  end process p_addr;

addr_probe <= to_integer( unsigned( std_logic_vector(addr_b - addr_int) and std_logic_vector(to_unsigned(MAX_N/4-1, addr_b'length)) ) );
  
  
  
  p_ram : process (clk)
  begin
    if clk'event and clk = '1' then
      w_a0 <= ram(to_integer(addr_int));
      
--      w_b0 <= ram(to_integer( unsigned( std_logic_vector(addr_b - addr_int) and std_logic_vector(to_unsigned(MAX_N/4-1, addr_b'length))) )); 
      w_b0 <= ram(to_integer(addr_int_sub )); 
      addr_int_r <= addr_int;
      if addr_int_r = 0 then
        w <= COMPLEX'(w_a0, resize(to_sfixed(0, w_b), w_b));
      else
        --w <= COMPLEX'(w_a0, resize(-w_b0, w_b));
        w <= COMPLEX'(w_a0, w_b0);
      end if;
      
  --    w_a <= ram(to_integer(addr_int));
      
  --    if addr_int = 0 then
  --      w_b <= to_sfixed(0, w_b);
  --    else
  --      w_b <= ram(to_integer( addr_b - addr_int )); 
  --    end if;

      
     -- w <= COMPLEX'(w_a, resize(-w_b, w_b));
      --w_a <= w_a0;
   
    end if;
  end process p_ram;


end rtl;
