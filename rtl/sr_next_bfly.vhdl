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

entity sr_next_bfly is
  generic (
    MAX_N : integer := 1024);

  port (
-- stage_in : in integer;
    stage_in        : in std_logic;
    pos_in, addr_in : in integer;

    stage_out         : out std_logic;
    pos_out, addr_out : out integer;
    wr_en             : out std_logic

    );

end sr_next_bfly;

architecture rtl of sr_next_bfly is
  signal wr_en_int        : std_logic;
  signal radix2_wr_en_int : std_logic;

  signal pos_out_int  : integer;
  signal addr_out_int : integer;

  constant UNSIGNED_MAX : integer := log2(MAX_N)+1;

begin  -- rt; 

  stage_out <= not stage_in;

  pos_out  <= pos_out_int;
  addr_out <= addr_out_int;

  wr_en <= wr_en_int;
-- radix2_wr_en <= radix2_wr_en_int;

  p_wr : process (pos_in, addr_in)
  begin  -- process p_next_state

    wr_en_int <= '0';

    if (to_unsigned(addr_in, UNSIGNED_MAX) and (to_unsigned(pos_in, UNSIGNED_MAX) srl 1)) > 0 then

      if to_integer(to_unsigned(pos_in, UNSIGNED_MAX) srl 2) > 0 then
        wr_en_int <= '1';
      end if;

    else

      if to_integer(to_unsigned(pos_in, UNSIGNED_MAX) srl 1) > 0 then
        wr_en_int <= '1';
      end if;

    end if;
  end process p_wr;


  p_next_state : process (pos_in, addr_in)
  begin  -- process p_next_state

    if (to_unsigned(addr_in, UNSIGNED_MAX) and (to_unsigned(pos_in, UNSIGNED_MAX) srl 1)) > 0 then

-- if stage_in = 0 then
-- pos_out_int <= pos_in;
-- addr_out_int <= addr_in;
-- else
      pos_out_int  <= to_integer(to_unsigned(pos_in, UNSIGNED_MAX) srl 2);
      addr_out_int <= addr_in;
-- end if;
    else

-- if stage_in = 0 then
-- pos_out_int <= pos_in;
-- addr_out_int <= addr_in;
-- else
      pos_out_int  <= to_integer(to_unsigned(pos_in, UNSIGNED_MAX) srl 1);
      addr_out_int <= addr_in;
-- end if;
    end if;
  end process p_next_state;



end rtl;

