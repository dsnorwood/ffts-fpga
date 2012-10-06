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

-- library ieee;
-- use ieee.std_logic_1164.all;
-- use ieee.numeric_std.all;

-- use work.fft_lib.all;

-- entity complex_multiplier is
--   generic (
--     N_STAGES     :     integer := 3);
--   port (
--     clk, reset_n : in  std_logic;
--     a, b         : in  complex;
--     y            : out complex
--     );

-- end complex_multiplier;

-- architecture rtl of complex_multiplier is

--   signal a_int, b_int : complex;
--   signal y_int : complex_vector(N_STAGES-2 downto 0);

-- begin  -- rtl

--   p_regs : process (clk)
--   begin  -- process p_regs     
--     if clk'event and clk = '1' then     -- rising clock edge
--       a_int    <= a;
--       b_int    <= b;
--       y_int(0) <= a * b;
--       y_int(1) <= y_int(0);
--     end if;
--   end process p_regs;

--   y <= y_int(1);

-- end rtl;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library ieee_proposed;
use ieee_proposed.fixed_pkg.all;

use work.fft_lib.all;

entity subtransform is
  generic (
    MAX_N        :     integer := 1024
    );
  port (
    clk, reset_n : in  std_logic;
    xs           : in  complex_vector(3 downto 0);
    ys           : out complex_vector(3 downto 0);
    w_addr       : in  integer;
    mode         : in  sr_mode_t;

    n : in unsigned(log2(MAX_N) downto 0)

    );

end subtransform;

architecture rtl of subtransform is

  component complex_multiplier
    port (
      clk, reset_n : in  std_logic;
      A_rel, A_img : in  std_logic_vector(17 downto 0);
      B_rel, B_img : in  std_logic_vector(17 downto 0);
      P_rel, P_img : out std_logic_vector(47 downto 0));
  end component;
  
  component complex_conj_multiplier
    port (
      clk, reset_n : in  std_logic;
      A_rel, A_img : in  std_logic_vector(17 downto 0);
      B_rel, B_img : in  std_logic_vector(17 downto 0);
      P_rel, P_img : out std_logic_vector(47 downto 0));
  end component;
  
  component twiddle_rom
    generic (
      MAX_N : integer);
    port (
      clk, reset_n : in  std_logic;
      addr         : in  integer;
      w            : out complex;
      n            : in  unsigned(log2(MAX_N) downto 0)
		);
  end component;
  
  constant MULT_DELAYS : integer := 4;

  type sr_mode_delay is array (0 to MULT_DELAYS-1) of sr_mode_t;
  signal mode_delays : sr_mode_delay;
  signal mode_int, mode_int_r1, mode_int_r2 : sr_mode_t;
  
  signal w1, w2, w2_conj     : complex;
  signal w_addr_int : integer;

  signal e, f : complex;
  
  signal xs_int, xs_int_r1, xs_int_r2 : complex_vector(3 downto 0);
  signal stage1, stage1_r1 : complex_vector(3 downto 0);
  signal stage2            : complex_vector(3 downto 0);

  type complex_delay is array (0 to MULT_DELAYS-1) of complex_vector(3 downto 0);
  signal stage2_delays : complex_delay;
  signal stage1_delays : complex_delay;
  signal xs_delays     : complex_delay;

  signal e_rel, e_img : std_logic_vector(47 downto 0);
  signal f_rel, f_img : std_logic_vector(47 downto 0);

  signal mult2_a_rel, mult2_a_img : std_logic_vector(17 downto 0);
  signal mult2_b_rel, mult2_b_img : std_logic_vector(17 downto 0);
  
begin  -- rtl

  w_addr_int <= w_addr;
     

  p_reg_inputs : process (clk)
  begin  -- process p_w_addr_reg     
    if clk'event and clk = '1' then     -- rising clock edge
      xs_int     <= xs;
      mode_int <= mode;
    end if;
  end process p_reg_inputs;

  twiddle_rom_inst : twiddle_rom
    generic map (
      MAX_N   => MAX_N)
    port map (
      clk     => clk,
      reset_n => reset_n,
      addr    => w_addr_int,
      w       => w1,
      n       => n
      );

  p_first_stage : process (clk)
  begin  -- process p_first_stage

    if clk'event and clk = '1' then
      stage1(0) <= xs_int(0) + xs_int(2);
      stage1(1) <= xs_int(1) + xs_int(3);
      stage1(2) <= xs_int(0) - xs_int(2);
      stage1(3) <= xs_int(1) - xs_int(3);

      xs_int_r1 <= xs_int;
      mode_int_r1 <= mode_int;
    end if;

  end process p_first_stage;


  p_second_stage : process (clk)
  begin  -- process p_second_stage

    if clk'event and clk = '1' then
      stage2(0) <= stage1(0) + stage1(1);
      stage2(1) <= stage1(0) - stage1(1);
      stage2(2) <= stage1(2) - (stage1(3)*CBASE_i);
      stage2(3) <= stage1(2) + (stage1(3)*CBASE_i);


      --stage2(2).re <= resize(stage1(2).re + stage1(3).im, stage2(2).re'high, stage2(2).re'low);
      --stage2(2).im <= resize(stage1(2).im - stage1(3).re, stage2(2).im'high, stage2(2).im'low);
      
      --stage2(3).re <= resize(stage1(2).re - stage1(3).im, stage2(3).re'high, stage2(3).re'low);
      --stage2(3).im <= resize(stage1(2).im + stage1(3).re, stage2(3).im'high, stage2(3).im'low);
      
      w2 <= w1;
     
      
      xs_int_r2 <= xs_int_r1;
      stage1_r1 <= stage1;
      mode_int_r2 <= mode_int_r1;
    end if;

  end process p_second_stage;
  
  mult1: complex_conj_multiplier
    port map (
        clk     => clk,
        reset_n => reset_n,
        A_rel   => to_slv(stage2(2).re),
        A_img   => to_slv(stage2(2).im),
        B_rel   => to_slv(w2.re),
        B_img   => to_slv(w2.im),
        P_rel   => e_rel,
        P_img   => e_img
        );

  e <= COMPLEX'(to_sfixed(e_rel(26 downto 9), 8, -9), to_sfixed(e_img(26 downto 9), 8, -9));
  
--   mult1 : complex_multiplier
--     port map (
--       clk     => clk,
--       reset_n => reset_n,
--       a       => stage2(2),
--       b       => w2,
--       y       => e
--       );

  
 --  w2_conj <= CONJ(w2);
 -- mult2_b_rel <= to_slv(w2_conj.re);
 -- mult2_b_img <= to_slv(w2_conj.im);
  
  mult2: complex_multiplier
    port map (
        clk     => clk,
        reset_n => reset_n,
        A_rel   => to_slv(stage2(3).re),
        A_img   => to_slv(stage2(3).im),
        B_rel   => to_slv(w2.re),
        B_img   => to_slv(w2.im),
        P_rel   => f_rel,
        P_img   => f_img
        );

  f <= COMPLEX'(to_sfixed(f_rel(26 downto 9), 8, -9), to_sfixed(f_img(26 downto 9), 8, -9));

--   mult2 : complex_multiplier
--     port map (
--       clk     => clk,
--       reset_n => reset_n,
--       a       => stage2(3),
--       b       => CONJ(w2),
--       y       => f
--       );


  p_delays : process (clk)
  begin
    if clk'event and clk = '1' then
      stage1_delays(0) <= stage1_r1;
      stage2_delays(0) <= stage2;
      xs_delays(0)     <= xs_int_r2;
      mode_delays(0)   <= mode_int_r2;

      for i in 1 to MULT_DELAYS-1 loop
        stage1_delays(i) <= stage1_delays(i-1);
        stage2_delays(i) <= stage2_delays(i-1);
        xs_delays(i)     <= xs_delays(i-1);
        mode_delays(i)   <= mode_delays(i-1);
      end loop;  -- i
    end if;
  end process p_delays;

  p_out_muxes : process (clk)
  begin  -- process p_out_muxes

    if clk'event and clk = '1' then
      case mode_delays(MULT_DELAYS-1) is
        when SRFFT_MODE_DUAL_R2 =>
          ys(0) <= stage1_delays(MULT_DELAYS-1)(0) sra 1;
          ys(1) <= stage1_delays(MULT_DELAYS-1)(1) sra 1;
          ys(2) <= stage1_delays(MULT_DELAYS-1)(2) sra 1;
          ys(3) <= stage1_delays(MULT_DELAYS-1)(3) sra 1;
        when SRFFT_MODE_CPSR_R2 =>
          ys(0) <= stage2_delays(MULT_DELAYS-1)(0) sra 2;
          ys(1) <= stage2_delays(MULT_DELAYS-1)(1) sra 2;
          ys(2) <= e sra 2;
          ys(3) <= f sra 2;
        when SRFFT_MODE_NOP     =>
          ys(0) <= xs_delays(MULT_DELAYS-1)(0);
          ys(1) <= xs_delays(MULT_DELAYS-1)(1);
          ys(2) <= xs_delays(MULT_DELAYS-1)(2);
          ys(3) <= xs_delays(MULT_DELAYS-1)(3);
        when others             =>
          ys(0) <= stage1_delays(MULT_DELAYS-1)(0) sra 1;
          ys(1) <= stage1_delays(MULT_DELAYS-1)(1) sra 1;
          ys(2) <= e sra 2;
          ys(3) <= f sra 2;
      end case;

    end if;

  end process p_out_muxes;



end rtl;
