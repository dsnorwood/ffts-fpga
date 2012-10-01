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

entity cp_bitreverse is

  generic (
    MAX_N : integer := 1024
    );
  port (
    clk      : in  std_logic;
    addr_in  : in  integer;
    addr_out : out integer;
    n        : in  integer
    );

end cp_bitreverse;

architecture rtl of cp_bitreverse is

  function bitreverse(x : std_logic_vector(log2(MAX_N)-1 downto 0); n : integer)
    return std_logic_vector is
    variable rval : std_logic_vector(log2(MAX_N)-1 downto 0) := (others => '0');
  begin
    for i in 0 to log2(n)-1 loop
      rval(i) := x(log2(n)-1-i);
    end loop;
    return rval;
  end bitreverse;



  signal din, dout : std_logic_vector(log2(MAX_N)-1 downto 0);
  signal dout_rev : std_logic_vector(log2(MAX_N)-1 downto 0);
  signal din_masked : std_logic_vector(log2(MAX_N)-1 downto 0);
begin  -- rtkl

  p_dout_rev: process (dout, n)
  begin  -- process p_dout_rev

    for i in 0 to log2(MAX_N)-1 loop
      if i<log2(n) then
        dout_rev(i) <= dout(log2(n)-1-i);
      else
        dout_rev(i) <= '0';        
      end if;
    end loop;  -- i
    
  end process p_dout_rev;
  
  din <= std_logic_vector(to_unsigned(addr_in, din'length));

  din_masked <= din and std_logic_vector(to_unsigned(3, din_masked'length));

  p_dout: process (din, dout)
  begin  -- process p_dout
     for i in 0 to dout'length-1 loop
        if i=0 or i=1 then
          dout(i) <= din(i);
        else
          dout(i) <= din(i) xor ((din(i-1) and din(i-2)) or (din(i-1) and (not din(i-2)) and dout(i-1) and dout(i-2)));          
        end if;

      end loop;  -- i      
    
  end process p_dout;
  
--  dout_rev <= bitreverse(dout, n);
  addr_out <= to_integer(unsigned(dout_rev));
  
end rtl;
