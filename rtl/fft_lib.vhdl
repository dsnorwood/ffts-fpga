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

library floatfixlib;
use floatfixlib.fixed_pkg.all;


package fft_lib is

  type int_vec is array (natural range<>) of integer;
  
  constant WORD_LENGTH : integer := 18;
  constant WORD_HINDEX : integer := 8;
  constant WORD_LINDEX : integer := -9;

  constant COMPLEX_WIDTH : integer := 2*WORD_LENGTH;

  subtype word_t is sfixed(WORD_HINDEX downto WORD_LINDEX);

  type complex is
    record
      re : word_t;
      im : word_t;
    end record;

  type complex_vector is array (integer range <>) of COMPLEX;

  function CMPLX(X : in real; Y : in real := 0.0) return complex;

  type real_complex is
    record
      re : real;
      im : real;
    end record;

  type real_complex_vector is array (natural range<>) of real_complex;

  function to_real_complex(X : in complex) return real_complex;
  function to_slv(X          : in complex) return std_logic_vector;
  function to_complex(X      : in std_logic_vector) return complex;

  constant CBASE_1 : complex :=
    COMPLEX'(to_sfixed(1.0, WORD_HINDEX, WORD_LINDEX), to_sfixed(0.0, WORD_HINDEX, WORD_LINDEX));
  constant CBASE_i : complex :=
    COMPLEX'(to_sfixed(0.0, WORD_HINDEX, WORD_LINDEX), to_sfixed(1.0, WORD_HINDEX, WORD_LINDEX));
  constant CZERO   : complex :=
    COMPLEX'(to_sfixed(0.0, WORD_HINDEX, WORD_LINDEX), to_sfixed(0.0, WORD_HINDEX, WORD_LINDEX));

  function "-" (Z : in complex) return complex;

  function "sra" (L : in complex; R : in integer) return complex;

  function CONJ (Z : in complex) return complex;

  function "+" (L : in complex; R : in complex) return complex;

  function "-" (L : in complex; R : in complex) return complex;

  function "*" (L : in complex; R : in complex) return complex;



  subtype sr_mode_t is std_logic_vector(1 downto 0);
  constant SRFFT_MODE_CPSR    : sr_mode_t := "00";
  constant SRFFT_MODE_NOP     : sr_mode_t := "01";
  constant SRFFT_MODE_CPSR_R2 : sr_mode_t := "10";
  constant SRFFT_MODE_DUAL_R2 : sr_mode_t := "11";
  
  function log2(A : integer) return integer;
  
end fft_lib;

package body fft_lib is

  function "-" (Z : in complex ) return complex is
  begin
    return COMPLEX'(resize(-z.Re, z.re'high, z.re'low), resize(-z.Im, z.im'high, z.im'low));
  end "-";

  function "sra" ( L : in complex; R : in integer ) return complex is
  begin
    return COMPLEX'(L.re sra R, L.im sra R);
  end "sra";

  function "+" ( L : in complex; R : in complex ) return complex is
  begin
    return COMPLEX'(resize(L.re + R.re, L.re'high, L.re'low), resize(L.im + R.im, L.im'high, L.im'low));
  end "+";

  function "-" ( L : in complex; R : in complex ) return complex is
  begin
    return COMPLEX'(resize(L.re - R.re, L.re'high, L.re'low), resize(L.im - R.im, L.re'high, L.re'low));
  end "-";

  function "*" ( L : in complex; R : in complex ) return complex is
  begin
    return COMPLEX'(resize(L.re * R.re - L.im * R.im, L.re'high, L.re'low),
                    resize(L.re * R.im + L.im * R.re, L.re'high, L.re'low));
  end "*";

  function CMPLX(X : in real; Y : in real := 0.0) return complex is
    -- returns complex number X + iY
  begin
    return COMPLEX'(to_sfixed(X, WORD_HINDEX, WORD_LINDEX), to_sfixed(Y, WORD_HINDEX, WORD_LINDEX));
  end CMPLX;

  function CONJ (Z : in complex) return complex is
    -- returns complex conjugate (x-jy for z = x+ jy)
  begin
    return COMPLEX'(z.Re, resize(-z.Im, z.im'high, z.im'low));
  end CONJ;

  function to_real_complex(X : in complex) return real_complex is
    -- returns complex number X + iY
  begin
    return REAL_COMPLEX'(to_real(X.re), to_real(X.im));
  end to_real_complex;

  function to_slv(X : in complex) return std_logic_vector is
    -- returns complex number X + iY
    variable rval   :    std_logic_vector(COMPLEX_WIDTH-1 downto 0);
  begin
    rval := to_slv(X.re) & to_slv(X.im);
    return rval;
  end to_slv;

  function to_complex(X : in std_logic_vector) return complex is
    -- returns complex number X + iY
  begin
    return COMPLEX'( to_sfixed(X(COMPLEX_WIDTH-1 downto COMPLEX_WIDTH/2), WORD_HINDEX, WORD_LINDEX),
                     to_sfixed(X(COMPLEX_WIDTH/2-1 downto 0), WORD_HINDEX, WORD_LINDEX) );
  end to_complex;

  function log2(A : integer) return integer is
  begin

    if A<0 then
      return 0;
    end if;
    
    for I in 1 to 30 loop               -- Works for up to 32 bit integers
      if(2**I > A) then return(I-1);
      end if;
    end loop;
    return(30);
  end function log2;

end fft_lib;
