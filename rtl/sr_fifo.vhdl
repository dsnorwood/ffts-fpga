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


entity sr_fifo is

  generic (
    N_CORES   : integer := 1;
    THIS_CORE : integer := 0;
    MAX_N     : integer := 1024

    );

  port (
    clk, reset_n : in std_logic;
    clk_en : in std_logic;
    
    rd_en : in std_logic;
    wr_en : in std_logic;

    stage_in : in std_logic;
    addr_in  : in integer;
    pos_in   : in integer;

    sync  : out std_logic;
    valid : out std_logic;
    empty : out std_logic;

    stage_out : out std_logic;
    addr_out  : out integer;
    pos_out   : out integer

    );

end sr_fifo;


architecture rtl of sr_fifo is


  type state_t is
    record
      addr  : signed(log2(MAX_N) downto 0);
      pos   : signed(log2(MAX_N) downto 0);
      stage : std_logic;
    end record;

	

  type state_vec is array (natural range<>) of std_logic_vector(2 * log2(MAX_N) + 3 downto 0);

  signal state_ram_dout : state_t;--std_logic_vector(2 * log2(MAX_N) + 3 downto 0);
  signal state_ram_din  : state_t;--std_logic_vector(2 * log2(MAX_N) + 3 downto 0);

  type state_ram_t is array (0 to MAX_N/4/N_CORES-1) of state_t;--std_logic_vector(2 * log2(MAX_N) + 3 downto 0);
  signal state_ram : state_ram_t;

  signal read_ptr  : unsigned(log2(MAX_N/4/N_CORES)-1 downto 0);
  signal write_ptr : unsigned(log2(MAX_N/4/N_CORES)-1 downto 0);
  
  signal empty_int : std_logic;

  signal sync_int  : std_logic;
  signal valid_int : std_logic;

begin  -- rtl

  valid_int <= not empty_int;
  empty     <= empty_int;

  valid <= valid_int;
  sync  <= sync_int;

  state_ram_din.addr  <= to_signed(addr_in, state_ram_din.addr'length);
  state_ram_din.pos   <= to_signed(pos_in, state_ram_din.pos'length);
  state_ram_din.stage <= stage_in;
--  state_ram_din(2*log2(MAX_N)+2 downto log2(MAX_N)+2) <= std_logic_vector(to_unsigned(addr_in, log2(MAX_N)+1));
--  state_ram_din(log2(MAX_N)+1 downto 1)               <= std_logic_vector(to_unsigned(pos_in, log2(MAX_N)+1));
--  state_ram_din(0)                                    <= stage_in;

  p_out_regs : process (clk)
  begin  -- process p_out_regs      
    if clk'event and clk = '1' and clk_en='1' then     -- rising clock edge


	if empty_int ='1' and sync_int='0' then
		addr_out <= -1;
		pos_out <= 0;
		stage_out <= '0';
 else	
			addr_out  <= to_integer(state_ram_dout.addr);
			pos_out   <= to_integer(state_ram_dout.pos);
			stage_out <= state_ram_dout.stage;

--			addr_out  <= to_integer(unsigned(state_ram_dout(2*log2(MAX_N)+2 downto log2(MAX_N)+2)));
--			pos_out   <= to_integer(unsigned(state_ram_dout(log2(MAX_N)+1 downto 1)));
--			stage_out <= state_ram_dout(0);
  end if;
    end if;
  end process p_out_regs;

  p_fifo : process (clk)
  begin  -- process p_fifo
    if reset_n = '0' then               -- asynchronous reset (active low)
      read_ptr   <= (others => '0');
      write_ptr  <= (others => '0');
    elsif clk'event and clk = '1' and clk_en='1' then  -- rising clock edge
		
      if empty_int = '0' and rd_en = '1' then
        read_ptr <= read_ptr + 1;
      end if;

      if wr_en = '1' then
        write_ptr <= write_ptr + 1;
      end if;
	
    end if;
  end process p_fifo;

  p_sync : process (state_ram_dout)
  begin  -- process p_sync
    if empty_int='0' and to_integer(state_ram_dout.addr) = 0 then
--    if unsigned(state_ram_dout(2*log2(MAX_N)+2 downto log2(MAX_N)+2)) = to_unsigned(0, log2(MAX_N)+1) then
      sync_int <= '1';
    else
      sync_int <= '0';
    end if;
  end process p_sync;

  p_ram : process (clk)
  begin  -- process p_ra  
    if clk'event and clk = '1' and clk_en='1' then     -- rising clock edge

		
      if wr_en = '1' then
        state_ram(to_integer(write_ptr)) <= state_ram_din;
      end if;

      state_ram_dout <= state_ram(to_integer(read_ptr));
	
    end if;
  end process p_ram;



  p_empty : process (read_ptr, write_ptr)
  begin  -- process p_empty
    if read_ptr = write_ptr then
      empty_int <= '1';
    else
      empty_int <= '0';
    end if;
  end process p_empty;

end rtl;
