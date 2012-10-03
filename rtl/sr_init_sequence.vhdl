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

entity sr_init_sequence is

  generic (
    N_CORES      : integer := 1;
    THIS_CORE    : integer := 0;
    MAX_N        : integer := 1024;
    PHASE_OFFSET : integer := 1
    );

  port (
    clk, reset_n : in  std_logic;
    clk_en : in std_logic;
    n            : in  unsigned(log2(MAX_N) downto 0);
    rd_en        : in  std_logic;
    dout         : out integer;
    valid        : out std_logic;
    sync         : out std_logic

    );

end sr_init_sequence;

architecture rtl of sr_init_sequence is

--TODO: comment this and link from fft_lib
  
--   function perm_mm (A  : in std_logic_vector) return std_logic_vector is
--     constant N_PORTS   :    integer                                    := N_CORES * 4;
--     variable rval      :    std_logic_vector(log2(N_PORTS)-1 downto 0) := (others => '0');
--     variable remainder :    integer                                    := 0;
--   begin

--     for i in 0 to A'length/log2(N_PORTS)-1 loop
--       rval := rval xor A(i*log2(N_PORTS)+log2(N_PORTS)-1 downto i*log2(N_PORTS));
--     end loop;

--     remainder := A'length - (A'length/log2(N_PORTS)*log2(N_PORTS));

--     if remainder > 0 then
--       rval(remainder-1 downto 0) := rval(remainder-1 downto 0) xor A(A'length-1 downto A'length-remainder);
--     end if;

--     return rval;
--   end perm_mm;

--   function perm_addr (A : in std_logic_vector) return std_logic_vector is
--     constant N_PORTS    :    integer                               := N_CORES * 4;
--     variable rval       :    std_logic_vector(A'length-1 downto 0) := (others => '0');
--   begin

--     rval := A(A'high downto log2(N_PORTS)) & perm_mm(A);

--     return rval;
--   end perm_addr;

  signal cur     : unsigned(log2(MAX_N) downto 0);
  signal counter : unsigned(log2(MAX_N) downto 0);

  signal n_div_4   : unsigned(log2(MAX_N) downto 0);
  signal max_count : unsigned(log2(MAX_N) downto 0);

  signal running : std_logic;

  signal sync_int  : std_logic;
  signal valid_int : std_logic;

  constant UNSIGNED_MAX : integer := log2(MAX_N);
begin  -- rtl

  sync  <= sync_int;
  valid <= valid_int;

  valid_int <= '1';

  process (clk) 
  begin
    if clk'event and clk='1' then
    n_div_4   <= n srl (2 + log2(N_CORES)); 
    max_count <= (n srl 2) - to_unsigned(N_CORES - THIS_CORE, n'length);
  end if;
  end process;

  

-- dout <= to_integer(to_unsigned(cur, log2(MAX_N/4)) sll log2(N_CORES));

 

--  dout <= to_integer(unsigned( perm_addr(std_logic_vector(to_unsigned(cur, UNSIGNED_MAX))) ));
  dout <= to_integer(cur);
  
  p_sync_int : process (counter)
  begin  -- process p_sync_int
    if counter = 0 then
      sync_int <= '1';
    else
      sync_int <= '0';
    end if;
  end process p_sync_int;

  p_seq : process (clk, reset_n)
  begin  -- process p_seq
    if reset_n = '0' then               -- asynchronous reset (active low)
      cur     <= to_unsigned(THIS_CORE, cur'length);
-- sync_int <= '1';
      running <= '0';
      counter <= to_unsigned(PHASE_OFFSET, counter'length);
    elsif clk'event and clk = '1'  then  -- rising clock edge

if clk_en='1' then
      if running = '0' then
        if rd_en = '1' then
          running <= '1';
        end if;
      else
        if cur = max_count then
          running <= '0';
        end if;
      end if;


      if running = '1' or rd_en = '1' then
        if cur = max_count then
          cur       <= to_unsigned(THIS_CORE, cur'length);
          counter   <= to_unsigned(PHASE_OFFSET, counter'length);
-- sync_int <= '1';
        else
          cur       <= cur + to_unsigned(N_CORES, cur'length);
          if counter = n_div_4-1 then
            counter <= to_unsigned(0, counter'length);
          else
            counter <= counter + 1;
          end if;

-- sync_int <= '0';
        end if;
      end if;
end if;
    end if;
  end process p_seq;

end rtl;
