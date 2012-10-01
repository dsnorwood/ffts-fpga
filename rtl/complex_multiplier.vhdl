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

entity complex_multiplier is
  
  port (
    clk, reset_n : in std_logic;
    A_rel, A_img : in  std_logic_vector(17 downto 0);
    B_rel, B_img : in  std_logic_vector(17 downto 0);
    P_rel, P_img : out std_logic_vector(47 downto 0)
    );

end complex_multiplier;

architecture rtl of complex_multiplier is
  component complex_half_multiplier
    port (
      clk, reset_n : in  std_logic;
      A_top, B_top : in  std_logic_vector(17 downto 0);
      A_bot, B_bot : in  std_logic_vector(17 downto 0);
      subtract     : in  std_logic;
      P            : out std_logic_vector(47 downto 0));
  end component;
  
begin  -- rtl

  MULT0 : complex_half_multiplier
    port map (
      clk     => clk,
      reset_n => reset_n,
      A_top   => A_img,
      B_top   => B_img,
      A_bot   => A_rel,
      B_bot   => B_rel,
      subtract => '1',
      P       => P_rel);


  MULT1 : complex_half_multiplier
    port map (
      clk     => clk,
      reset_n => reset_n,
      A_top   => A_img,
      B_top   => B_rel,
      A_bot   => A_rel,
      B_bot   => B_img,
      subtract => '0',
      P       => P_img);


end rtl;
