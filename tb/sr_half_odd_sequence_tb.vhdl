-------------------------------------------------------------------------------
-- Title      : Testbench for design "sr_half_odd_sequence"
-- Project    : NTP S-1 Appliance
-------------------------------------------------------------------------------
-- File       : sr_half_odd_sequence_tb.vhdl
-- Author     :   <a@anthonix.com>
-- Company    : 
-- Created    : 2010-12-28
-- Last update: 2011-01-10
-- Platform   : 
-- Standard   : VHDL'87
-------------------------------------------------------------------------------
-- This file is part of the NTP S-1 appliance project. For more information 
-- see http://www.wand.net.nz/~amb33/ntp
-------------------------------------------------------------------------------
-- Description: 
-------------------------------------------------------------------------------
-- Copyright (c) 2010 
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author  Description
-- 2010-12-28  1.0      anthonix	Created
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

-------------------------------------------------------------------------------

entity sr_half_odd_sequence_tb is

end sr_half_odd_sequence_tb;

-------------------------------------------------------------------------------

architecture tb of sr_half_odd_sequence_tb is

  component sr_half_odd_sequence
    generic (
      N_CORES   : integer;
      THIS_CORE : integer;
      MAX_N     : integer);
    port (
      clk, reset_n : in  std_logic;
      clk_en       : in  std_logic;
      n            : in  integer;
      rd_en        : in  std_logic;
      dout         : out integer;
      sync         : out std_logic;
      valid        : out std_logic);
  end component;
  component sr_half_odd_seq_ram
    generic (
      N_CORES   : integer;
      THIS_CORE : integer;
      MAX_N     : integer);
    port (
      clk, reset_n : in  std_logic;
      clk_en       : in  std_logic;
      n            : in  integer;
      rd_en        : in  std_logic;
      dout         : out integer;
      sync         : out std_logic;
      valid        : out std_logic);
  end component;
  
  -- component generics
  constant N_CORES   : integer := 1;
  constant THIS_CORE : integer := 0;
  constant MAX_N     : integer := 1024;

  -- component ports
  signal clk, reset_n : std_logic := '1'; 
  signal clk_en       : std_logic;
  signal n            : integer;
  signal rd_en        : std_logic;

  signal dout0         : integer;
  signal sync0         : std_logic;
  signal valid0        : std_logic;

  signal dout1         : integer;
  signal sync1         : std_logic;
  signal valid1        : std_logic;



begin  -- tb

  -- component instantiation
  DUT: sr_half_odd_sequence
    generic map (
      N_CORES   => N_CORES,
      THIS_CORE => THIS_CORE,
      MAX_N     => MAX_N)
    port map (
      clk     => clk,
      reset_n => reset_n,
      clk_en  => clk_en,
      n       => n,
      rd_en   => rd_en,
      dout    => dout0,
      sync    => sync0,
      valid   => valid0
      );

  sr_half_odd_seq_ram_1: sr_half_odd_seq_ram
    generic map (
      N_CORES   => N_CORES,
      THIS_CORE => THIS_CORE,
      MAX_N     => MAX_N)
    port map (
      clk     => clk,
      reset_n => reset_n,
      clk_en  => clk_en,
      n       => n,
      rd_en   => rd_en,
      dout    => dout1,
      sync    => sync1,
      valid   => valid1
      );
  
  -- clock generation
  clk <= not clk after 5 ns;

  -- waveform generation
  WaveGen_Proc: process
  begin
    n <= 128;
    clk_en <= '1';
    rd_en <= '0';
    wait for 50 ns;
    reset_n <= '0';
    wait for 50 ns;
    reset_n <= '1';
    wait for 50 ns;
    rd_en <= '1';
	 wait for 10 ns;
	 rd_en <= '0';
    wait;

  end process WaveGen_Proc;

  

end tb;

-------------------------------------------------------------------------------

configuration sr_half_odd_sequence_tb_tb_cfg of sr_half_odd_sequence_tb is
  for tb
  end for;
end sr_half_odd_sequence_tb_tb_cfg;

-------------------------------------------------------------------------------
