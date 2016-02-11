--
--
--  ZPUINO implementation on Gadget Factory 'Papilio Pro' Board
-- 
--  Copyright 2011 Alvaro Lopes <alvieboy@alvie.com>
-- 
--	 Vanilla Variant
--
--  Version: 1.0
-- 
--  The FreeBSD license
--  
--  Redistribution and use in source and binary forms, with or without
--  modification, are permitted provided that the following conditions
--  are met:
--  
--  1. Redistributions of source code must retain the above copyright
--     notice, this list of conditions and the following disclaimer.
--  2. Redistributions in binary form must reproduce the above
--     copyright notice, this list of conditions and the following
--     disclaimer in the documentation and/or other materials
--     provided with the distribution.
--  
--  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY
--  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
--  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
--  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
--  ZPU PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
--  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
--  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
--  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
--  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
--  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
--  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
--  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--  
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ZPUino_Papilio_ProNOSD_LX9_V2 is
  port (
	 --32Mhz input clock is converted to a 96Mhz clock
--    CLK:        in std_logic;
	 
	 --Clock outputs to be used in schematic
	 clk_96Mhz:        out std_logic;	--This is the clock that the system runs on.
	 clk_1Mhz:        out std_logic;		--This is a 1Mhz clock for symbols like the C64 SID chip.
	 clk_osc_32Mhz:   out std_logic;		--This is the 32Mhz clock from external oscillator.

    -- Connection to the main SPI flash
--    SPI_SCK:    out std_logic;
--    SPI_MISO:   in std_logic;
--    SPI_MOSI:   out std_logic;
--    SPI_CS:     out std_logic;
	 
	 ext_pins_in : in std_logic_vector(100 downto 0);
	 ext_pins_out : out std_logic_vector(100 downto 0);
	 ext_pins_inout : inout std_logic_vector(100 downto 0);

	 gpio_bus_in : in std_logic_vector(200 downto 0);
	 gpio_bus_out : out std_logic_vector(200 downto 0);

    -- UART (FTDI) connection
--    TXD:        out std_logic;
--    RXD:        in std_logic;

--    DRAM_ADDR   : OUT   STD_LOGIC_VECTOR (12 downto 0);
--     DRAM_BA      : OUT   STD_LOGIC_VECTOR (1 downto 0);
--     DRAM_CAS_N   : OUT   STD_LOGIC;
--     DRAM_CKE      : OUT   STD_LOGIC;
--     DRAM_CLK      : OUT   STD_LOGIC;
--     DRAM_CS_N   : OUT   STD_LOGIC;
--     DRAM_DQ      : INOUT STD_LOGIC_VECTOR(15 downto 0);
--     DRAM_DQM      : OUT   STD_LOGIC_VECTOR(1 downto 0);
--     DRAM_RAS_N   : OUT   STD_LOGIC;
--     DRAM_WE_N    : OUT   STD_LOGIC;

    -- The LED
--    LED:        out std_logic;
	
	 --There are more bits in the address for this wishbone connection
	 wishbone_slot_video_in : in std_logic_vector(100 downto 0);
	 wishbone_slot_video_out : out std_logic_vector(100 downto 0);
	 vgaclkout: out std_logic;	

	 --Input and output reversed for the master
	 wishbone_slot_5_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_5_out : in std_logic_vector(100 downto 0);
	 
	 wishbone_slot_6_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_6_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_8_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_8_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_9_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_9_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_10_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_10_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_11_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_11_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_12_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_12_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_13_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_13_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_14_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_14_out : in std_logic_vector(100 downto 0)

--	 wishbone_slot_15_in : out std_logic_vector(61 downto 0);
--	 wishbone_slot_15_out : in std_logic_vector(33 downto 0)	 
	
	
 	
  );

end entity ZPUino_Papilio_ProNOSD_LX9_V2;

architecture behave of ZPUino_Papilio_ProNOSD_LX9_V2 is

constant wordPower			: integer := 5;
constant wordSize			: integer := 2**wordPower;
constant maxAddrBitIncIO		: integer := 27;
constant maxIOBit: integer := maxAddrBitIncIO - 1;
constant minIOBit: integer := 2;
constant maxAddrBitBRAM		: integer := 22;

  
	COMPONENT ZPUino_Papilio_ProNoSD_V2_blackbox
 port (
    CLK:        in std_logic;
    --RST:        in std_logic; -- No reset on papilio
	 
	 --Clock outputs to be used in schematic
	 clk_96Mhz:        out std_logic;	--This is the clock that the system runs on.
	 clk_1Mhz:        out std_logic;		--This is a 1Mhz clock for symbols like the C64 SID chip.
	 clk_osc_32Mhz:   out std_logic;		--This is the 32Mhz clock from external oscillator.

	 -- Connection to the main SPI flash
    SPI_SCK:    out std_logic;
    SPI_MISO:   in std_logic;
    SPI_MOSI:   out std_logic;
    SPI_CS:     inout std_logic;
	 
	 gpio_bus_in : in std_logic_vector(200 downto 0);
	 gpio_bus_out : out std_logic_vector(200 downto 0);
		
	 -- UART (FTDI) connection
    TXD:        out std_logic;
    RXD:        in std_logic;

	 --There are more bits in the address for this wishbone connection
	 wishbone_slot_video_in : in std_logic_vector(100 downto 0);
	 wishbone_slot_video_out : out std_logic_vector(100 downto 0);
	 vgaclkout: out std_logic;	

	 wishbone_slot_5_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_5_out : in std_logic_vector(100 downto 0);
	 
	 wishbone_slot_6_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_6_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_8_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_8_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_9_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_9_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_10_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_10_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_11_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_11_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_12_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_12_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_13_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_13_out : in std_logic_vector(100 downto 0);

	 wishbone_slot_14_in : out std_logic_vector(100 downto 0);
	 wishbone_slot_14_out : in std_logic_vector(100 downto 0)
  );
	END COMPONENT;  
	
  signal SPI_SCK, SPI_MOSI, SPI_CS, TXD:	std_logic;

begin

	ext_pins_out(0) <= SPI_SCK;
	ext_pins_out(1) <= SPI_MOSI;
	ext_pins_out(2) <= SPI_CS;
	ext_pins_out(3) <= TXD;

--	SPI_SCK <= ext_pins_out(0);
--	SPI_MOSI <= ext_pins_out(1);
--	SPI_CS <= ext_pins_out(2);
--	TXD <= ext_pins_out(3);
	ext_pins_out <= (others => 'Z');

	Inst_ZPUino_Papilio_ProNoSD_V2_blackbox: ZPUino_Papilio_ProNoSD_V2_blackbox PORT MAP(
		CLK => ext_pins_in(0),
		clk_96Mhz => clk_96Mhz,
		clk_1Mhz => clk_1Mhz,
		clk_osc_32Mhz => clk_osc_32Mhz,
		SPI_SCK => SPI_SCK,
		SPI_MISO => ext_pins_in(1),
		SPI_MOSI => SPI_MOSI,
		SPI_CS => SPI_CS,
		gpio_bus_in => gpio_bus_in,
		gpio_bus_out => gpio_bus_out,
		TXD => TXD,
		RXD => ext_pins_in(2),

		wishbone_slot_video_in => wishbone_slot_video_in,
		wishbone_slot_video_out => wishbone_slot_video_out,
		vgaclkout => vgaclkout,
		wishbone_slot_5_in => wishbone_slot_5_in,
		wishbone_slot_5_out => wishbone_slot_5_out,
		wishbone_slot_6_in => wishbone_slot_6_in,
		wishbone_slot_6_out => wishbone_slot_6_out,
		wishbone_slot_8_in => wishbone_slot_8_in,
		wishbone_slot_8_out => wishbone_slot_8_out,
		wishbone_slot_9_in => wishbone_slot_9_in,
		wishbone_slot_9_out => wishbone_slot_9_out,
		wishbone_slot_10_in => wishbone_slot_10_in,
		wishbone_slot_10_out => wishbone_slot_10_out,
		wishbone_slot_11_in => wishbone_slot_11_in,
		wishbone_slot_11_out => wishbone_slot_11_out,
		wishbone_slot_12_in => wishbone_slot_12_in,
		wishbone_slot_12_out => wishbone_slot_12_out,
		wishbone_slot_13_in => wishbone_slot_13_in,
		wishbone_slot_13_out => wishbone_slot_13_out,
		wishbone_slot_14_in => wishbone_slot_14_in,
		wishbone_slot_14_out => wishbone_slot_14_out
	);

end behave;
