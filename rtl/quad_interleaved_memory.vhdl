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

entity crossbar_switch_slice is

  generic (
    PHIT_WIDTH : integer := 16;
    THIS_PORT  : integer := 0);

  port (
    i1, i2, i3, i4                 : in std_logic_vector(PHIT_WIDTH-1 downto 0);
    dest1, dest2, dest3, dest4     : in std_logic_vector(1 downto 0);
    valid1, valid2, valid3, valid4 : in std_logic;

    output       : out std_logic_vector(PHIT_WIDTH-1 downto 0);
    output_valid : out std_logic
    );

end crossbar_switch_slice;

architecture rtl of crossbar_switch_slice is

begin  -- rtl

  p_req : process (dest1, dest2, dest3, dest4, valid1, valid2, valid3, valid4, i1, i2, i3, i4)
  begin  -- process p_req
    if valid1 = '1' and dest1 = std_logic_vector(to_unsigned(THIS_PORT, dest1'length)) then
      output       <= i1;
      output_valid <= '1';
    elsif valid2 = '1' and dest2 = std_logic_vector(to_unsigned(THIS_PORT, dest2'length)) then
      output       <= i2;
      output_valid <= '1';
    elsif valid3 = '1' and dest3 = std_logic_vector(to_unsigned(THIS_PORT, dest3'length)) then
      output       <= i3;
      output_valid <= '1';
    elsif valid4 = '1' and dest4 = std_logic_vector(to_unsigned(THIS_PORT, dest4'length)) then
      output       <= i4;
      output_valid <= '1';
    else
      output       <= (others => '0');
      output_valid <= '0';
    end if;
  end process p_req;


end rtl;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity crossbar_switch is

  generic (
    PHIT_WIDTH   :    integer := 16);
  port (
    clk, reset_n : in std_logic;

    i1, i2, i3, i4             : in std_logic_vector(PHIT_WIDTH-1 downto 0);
    dest1, dest2, dest3, dest4 : in std_logic_vector(1 downto 0);
    e1, e2, e3, e4             : in std_logic;

    o1, o2, o3, o4     : out std_logic_vector(PHIT_WIDTH-1 downto 0);
    dv1, dv2, dv3, dv4 : out std_logic
    );

end crossbar_switch;

architecture rtl of crossbar_switch is

  component crossbar_switch_slice
    generic (
      PHIT_WIDTH                     :     integer;
      THIS_PORT                      :     integer);
    port (
      i1, i2, i3, i4                 : in  std_logic_vector(PHIT_WIDTH-1 downto 0);
      dest1, dest2, dest3, dest4     : in  std_logic_vector(1 downto 0);
      valid1, valid2, valid3, valid4 : in  std_logic;
      output                         : out std_logic_vector(PHIT_WIDTH-1 downto 0);
      output_valid                   : out std_logic);
  end component;


  constant N_PORTS : integer := 4;

  type phit_vec is array (N_PORTS-1 downto 0) of std_logic_vector(PHIT_WIDTH-1 downto 0);

  signal din_int     : phit_vec;
  signal dout_int    : phit_vec;
  signal enables_int : std_logic_vector(N_PORTS-1 downto 0);

  signal dv_int : std_logic_vector(N_PORTS-1 downto 0);

  type dest_vec is array (3 downto 0) of std_logic_vector(1 downto 0);

  signal dests_int : dest_vec;
  
begin  -- rtl

  p_enable_regs : process (clk, reset_n)
  begin  -- process p_enable_regs
    if reset_n = '0' then               -- asynchronous reset (active low)
      enables_int <= (others => '0');
    elsif clk'event and clk = '1' then  -- rising clock edge
      enables_int <= e4 & e3 & e2 & e1;
    end if;
  end process p_enable_regs;

  p_regs : process (clk)
  begin  -- process p_regs

    if clk'event and clk = '1' then     -- rising clock edge
      din_int(0) <= i1;
      din_int(1) <= i2;
      din_int(2) <= i3;
      din_int(3) <= i4;

      dests_int(0) <= dest1;
      dests_int(1) <= dest2;
      dests_int(2) <= dest3;
      dests_int(3) <= dest4;

      o1 <= dout_int(0);
      o2 <= dout_int(1);
      o3 <= dout_int(2);
      o4 <= dout_int(3);

      dv1 <= dv_int(0);
      dv2 <= dv_int(1);
      dv3 <= dv_int(2);
      dv4 <= dv_int(3);

    end if;
  end process p_regs;


  gen_ports : for i in 0 to 3 generate

    xbar_port_inst : crossbar_switch_slice
      generic map (
        PHIT_WIDTH => PHIT_WIDTH,
        THIS_PORT  => i)
      port map (
        i1         => din_int(0),
        i2         => din_int(1),
        i3         => din_int(2),
        i4         => din_int(3),
        dest1      => dests_int(0),
        dest2      => dests_int(1),
        dest3      => dests_int(2),
        dest4      => dests_int(3),
        valid1     => enables_int(0),
        valid2     => enables_int(1),
        valid3     => enables_int(2),
        valid4     => enables_int(3),

        output       => dout_int(i),
        output_valid => dv_int(i)
        );

  end generate gen_ports;


end rtl;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.fft_lib.all;

entity dual_port_ram is

  generic (
    DATA_WIDTH   :    integer := 16;
    DEPTH        :    integer := 1024
    );
  port (
    clk, reset_n : in std_logic;

    rd_addr : in  unsigned(log2(DEPTH)-1 downto 0);
    wr_addr : in  unsigned(log2(DEPTH)-1 downto 0);
    wr_en   : in  std_logic;
    din     : in  std_logic_vector(DATA_WIDTH-1 downto 0);
    dout    : out std_logic_vector(DATA_WIDTH-1 downto 0)
    );

end dual_port_ram;

architecture rtl of dual_port_ram is

  type ram_t is array(0 to DEPTH-1) of std_logic_vector(DATA_WIDTH-1 downto 0);
  signal ram : ram_t;

begin

  p_bram : process (clk)
  begin
    if clk'event and clk = '1' then

      if wr_en = '1' then
        ram(to_integer(wr_addr)) <= din;
      end if;

      dout <= ram(to_integer(rd_addr));
    end if;
  end process p_bram;

end rtl;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.fft_lib.all;


entity quad_interleaved_memory is

  generic (
    DATA_WIDTH : integer := 16;
    DEPTH      : integer := 1024
    );

  port (
    clk     : in std_logic;
    reset_n : in std_logic;

    wr_data0 : in std_logic_vector(DATA_WIDTH-1 downto 0);
    wr_addr0 : in integer;

    wr_data1 : in std_logic_vector(DATA_WIDTH-1 downto 0);
    wr_addr1 : in integer;

    wr_data2 : in std_logic_vector(DATA_WIDTH-1 downto 0);
    wr_addr2 : in integer;

    wr_data3 : in std_logic_vector(DATA_WIDTH-1 downto 0);
    wr_addr3 : in integer;

    wr_en : in std_logic_vector(3 downto 0);

    rd_data0 : out std_logic_vector(DATA_WIDTH-1 downto 0);
    rd_addr0 : in  integer;

    rd_data1 : out std_logic_vector(DATA_WIDTH-1 downto 0);
    rd_addr1 : in  integer;

    rd_data2 : out std_logic_vector(DATA_WIDTH-1 downto 0);
    rd_addr2 : in  integer;

    rd_data3 : out std_logic_vector(DATA_WIDTH-1 downto 0);
    rd_addr3 : in  integer;

    rd_en    : in  std_logic_vector(3 downto 0);
    rd_valid : out std_logic_vector(3 downto 0)

    );

end quad_interleaved_memory;

architecture rtl of quad_interleaved_memory is

  component dual_port_ram
    generic (
      DATA_WIDTH   :     integer;
      DEPTH        :     integer);
    port (
      clk, reset_n : in  std_logic;
      rd_addr      : in  unsigned(log2(DEPTH)-1 downto 0);
      wr_addr      : in  unsigned(log2(DEPTH)-1 downto 0);
      wr_en        : in  std_logic;
      din          : in  std_logic_vector(DATA_WIDTH-1 downto 0);
      dout         : out std_logic_vector(DATA_WIDTH-1 downto 0));
  end component;

  component crossbar_switch
    generic (
      PHIT_WIDTH                 :     integer);
    port (
      clk, reset_n               : in  std_logic;
      i1, i2, i3, i4             : in  std_logic_vector(PHIT_WIDTH-1 downto 0);
      dest1, dest2, dest3, dest4 : in  std_logic_vector(1 downto 0);
      e1, e2, e3, e4             : in  std_logic;
      o1, o2, o3, o4             : out std_logic_vector(PHIT_WIDTH-1 downto 0);
      dv1, dv2, dv3, dv4         : out std_logic);
  end component;

  function perm_mm (A  : in std_logic_vector) return std_logic_vector is
    constant N_PORTS   :    integer                                    := 4;
    variable rval      :    std_logic_vector(log2(N_PORTS)-1 downto 0) := (others => '0');
    variable remainder :    integer                                    := 0;
  begin

    for i in 0 to A'length/log2(N_PORTS)-1 loop
      rval := rval xor A(i*log2(N_PORTS)+log2(N_PORTS)-1 downto i*log2(N_PORTS));
    end loop;

    remainder := A'length - (A'length/log2(N_PORTS)*log2(N_PORTS));

    if remainder > 0 then
      rval(remainder-1 downto 0) := rval(remainder-1 downto 0) xor A(A'length-1 downto A'length-remainder);
    end if;

    return rval;
  end perm_mm;

  function perm_addr (A : in std_logic_vector) return std_logic_vector is
    constant N_PORTS    :    integer                                             := 4;
    variable rval       :    std_logic_vector(A'length-log2(N_PORTS)-1 downto 0) := (others => '0');
  begin

    rval := A(A'high downto log2(N_PORTS));

    return rval;
  end perm_addr;

  constant WR_PHIT_WIDTH : integer := DATA_WIDTH + log2(DEPTH/4);

  type wr_phit_vec is array (3 downto 0) of std_logic_vector(WR_PHIT_WIDTH-1 downto 0);

  signal wr_en_xbar_out   : std_logic_vector(3 downto 0);
  signal wr_phit_xbar_in  : wr_phit_vec;
  signal wr_phit_xbar_out : wr_phit_vec;

  type mm_vec is array (3 downto 0) of std_logic_vector(1 downto 0);

  signal wr_mm : mm_vec;
  signal rd_mm : mm_vec;

  constant RD_ADDR_PHIT_WIDTH : integer := log2(DEPTH/4) + 2;

  type rd_addr_phit_vec is array (3 downto 0) of std_logic_vector(RD_ADDR_PHIT_WIDTH-1 downto 0);

  signal rd_addr_phit_in : rd_addr_phit_vec;
  signal rd_addr_phit_out : rd_addr_phit_vec;

  signal rd_addr_valid : std_logic_vector(3 downto 0);

  type rd_data_vec is array (3 downto 0) of std_logic_vector(DATA_WIDTH-1 downto 0);

  signal rd_data_phit_in : rd_data_vec;
  signal rd_data_phit_out : rd_data_vec;
  
  type addr_vec is array (3 downto 0) of unsigned(log2(DEPTH)-1 downto 0);

  signal wr_addr_int : addr_vec;

  type port_id_vec is array (3 downto 0) of std_logic_vector(1 downto 0);

  signal port_id_r : port_id_vec;

  signal rd_addr_valid_r : std_logic_vector(3 downto 0);
  
begin  -- rtl


  wr_addr_int(0) <= to_unsigned(wr_addr0, log2(DEPTH));
  wr_addr_int(1) <= to_unsigned(wr_addr1, log2(DEPTH));
  wr_addr_int(2) <= to_unsigned(wr_addr2, log2(DEPTH));
  wr_addr_int(3) <= to_unsigned(wr_addr3, log2(DEPTH));

  p_wr_mm: process (wr_addr_int)
  begin  -- process p_wr_mm
    for i in 0 to 3 loop
      wr_mm(i) <= perm_mm(std_logic_vector(wr_addr_int(i)));
    end loop;  -- i
  end process p_wr_mm;

  wr_phit_xbar_in(0) <= perm_addr(std_logic_vector(wr_addr_int(0))) & wr_data0;
  wr_phit_xbar_in(1) <= perm_addr(std_logic_vector(wr_addr_int(1))) & wr_data1;
  wr_phit_xbar_in(2) <= perm_addr(std_logic_vector(wr_addr_int(2))) & wr_data2;
  wr_phit_xbar_in(3) <= perm_addr(std_logic_vector(wr_addr_int(3))) & wr_data3;


  write_xbar : crossbar_switch
    generic map (
      PHIT_WIDTH => WR_PHIT_WIDTH)
    port map (
      clk        => clk,
      reset_n    => reset_n,
      i1         => wr_phit_xbar_in(0),
      i2         => wr_phit_xbar_in(1),
      i3         => wr_phit_xbar_in(2),
      i4         => wr_phit_xbar_in(3),
      dest1      => wr_mm(0),
      dest2      => wr_mm(1),
      dest3      => wr_mm(2),
      dest4      => wr_mm(3),
      e1         => wr_en(0),
      e2         => wr_en(1),
      e3         => wr_en(2),
      e4         => wr_en(3),
      o1         => wr_phit_xbar_out(0),
      o2         => wr_phit_xbar_out(1),
      o3         => wr_phit_xbar_out(2),
      o4         => wr_phit_xbar_out(3),
      dv1        => wr_en_xbar_out(0),
      dv2        => wr_en_xbar_out(1),
      dv3        => wr_en_xbar_out(2),
      dv4        => wr_en_xbar_out(3)
      );

  gen_memories : for i in 0 to 3 generate

    mem_inst : dual_port_ram
      generic map (
        DATA_WIDTH => DATA_WIDTH,
        DEPTH      => DEPTH/4)
      port map (
        clk        => clk,
        reset_n    => reset_n,

        wr_addr => unsigned(wr_phit_xbar_out(i)(WR_PHIT_WIDTH-1 downto DATA_WIDTH)),
        wr_en   => wr_en_xbar_out(i),
        din     => wr_phit_xbar_out(i)(DATA_WIDTH-1 downto 0),

        rd_addr => unsigned(rd_addr_phit_out(i)(RD_ADDR_PHIT_WIDTH-1 downto 2)),
        dout    => rd_data_phit_in(i)
        );

  end generate gen_memories;

  rd_mm(0) <= perm_mm(std_logic_vector(to_unsigned(rd_addr0, log2(DEPTH))));
  rd_mm(1) <= perm_mm(std_logic_vector(to_unsigned(rd_addr1, log2(DEPTH))));
  rd_mm(2) <= perm_mm(std_logic_vector(to_unsigned(rd_addr2, log2(DEPTH))));
  rd_mm(3) <= perm_mm(std_logic_vector(to_unsigned(rd_addr3, log2(DEPTH))));

  rd_addr_phit_in(0) <=
    perm_addr(std_logic_vector(to_unsigned(rd_addr0, log2(DEPTH)))) & "00";
  rd_addr_phit_in(1) <=
    perm_addr(std_logic_vector(to_unsigned(rd_addr1, log2(DEPTH)))) & "01";
  rd_addr_phit_in(2) <=
    perm_addr(std_logic_vector(to_unsigned(rd_addr2, log2(DEPTH)))) & "10";
  rd_addr_phit_in(3) <=
    perm_addr(std_logic_vector(to_unsigned(rd_addr3, log2(DEPTH)))) & "11";

  p_port_id_delays: process (clk)
  begin      
    if clk'event and clk = '1' then  -- rising clock edge
      if reset_n='0' then
        port_id_r <= (others => (others => '0'));
        rd_addr_valid_r <= (others => '0');
      else
        port_id_r(0) <= rd_addr_phit_out(0)(1 downto 0);
        port_id_r(1) <= rd_addr_phit_out(1)(1 downto 0);
        port_id_r(2) <= rd_addr_phit_out(2)(1 downto 0);
        port_id_r(3) <= rd_addr_phit_out(3)(1 downto 0);
        rd_addr_valid_r <= rd_addr_valid(3) & rd_addr_valid(2) & rd_addr_valid(1) & rd_addr_valid(0);
      end if;
    end if;
  end process p_port_id_delays;

  read_addr_xbar: crossbar_switch
    generic map (
        PHIT_WIDTH => RD_ADDR_PHIT_WIDTH)
    port map (
        clk     => clk,
        reset_n => reset_n,
        
        i1      => rd_addr_phit_in(0),
        i2      => rd_addr_phit_in(1),
        i3      => rd_addr_phit_in(2),
        i4      => rd_addr_phit_in(3),
        dest1   => rd_mm(0),
        dest2   => rd_mm(1),
        dest3   => rd_mm(2),
        dest4   => rd_mm(3),
        e1      => rd_en(0),
        e2      => rd_en(1),
        e3      => rd_en(2),
        e4      => rd_en(3),        
        o1      => rd_addr_phit_out(0),
        o2      => rd_addr_phit_out(1),
        o3      => rd_addr_phit_out(2),
        o4      => rd_addr_phit_out(3),
        dv1     => rd_addr_valid(0),
        dv2     => rd_addr_valid(1),
        dv3     => rd_addr_valid(2),
        dv4     => rd_addr_valid(3)
        );

  read_data_xbar: crossbar_switch
    generic map (
        PHIT_WIDTH => DATA_WIDTH)
    port map (
        clk     => clk,
        reset_n => reset_n,
        
        i1      => rd_data_phit_in(0),
        i2      => rd_data_phit_in(1),
        i3      => rd_data_phit_in(2),
        i4      => rd_data_phit_in(3),
        dest1   => port_id_r(0),
        dest2   => port_id_r(1),
        dest3   => port_id_r(2),
        dest4   => port_id_r(3),
        e1      => rd_addr_valid_r(0),
        e2      => rd_addr_valid_r(1),
        e3      => rd_addr_valid_r(2),
        e4      => rd_addr_valid_r(3),
        o1      => rd_data_phit_out(0),
        o2      => rd_data_phit_out(1),
        o3      => rd_data_phit_out(2),
        o4      => rd_data_phit_out(3),
        dv1     => rd_valid(0),
        dv2     => rd_valid(1),
        dv3     => rd_valid(2),
        dv4     => rd_valid(3)

        );

  rd_data0 <= rd_data_phit_out(0);
  rd_data1 <= rd_data_phit_out(1);
  rd_data2 <= rd_data_phit_out(2);
  rd_data3 <= rd_data_phit_out(3);
  
  
end rtl;
