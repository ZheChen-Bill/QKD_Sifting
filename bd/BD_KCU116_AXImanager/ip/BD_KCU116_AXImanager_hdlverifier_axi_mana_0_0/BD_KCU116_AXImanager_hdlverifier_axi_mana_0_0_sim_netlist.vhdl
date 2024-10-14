-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2022.1 (win64) Build 3526262 Mon Apr 18 15:48:16 MDT 2022
-- Date        : Wed Dec 20 07:23:21 2023
-- Host        : DESKTOP-H9LB9O9 running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode funcsim
--               d:/QKD_HW_project/matlab_test/kcu116_sifting_AXImanager_v1/kcu116_sifting_AXImanager_v1.gen/sources_1/bd/BD_KCU116_AXImanager/ip/BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0/BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_sim_netlist.vhdl
-- Design      : BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xcku5p-ffvb676-2-e
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_vendor_ip1 is
  port (
    SR : out STD_LOGIC_VECTOR ( 0 to 0 );
    CLK : out STD_LOGIC;
    tdi : out STD_LOGIC;
    shift_dr : out STD_LOGIC;
    E : out STD_LOGIC_VECTOR ( 0 to 0 );
    \FSM_sequential_state_reg_reg[2]\ : out STD_LOGIC_VECTOR ( 0 to 0 );
    \FSM_sequential_state_reg_reg[2]_0\ : out STD_LOGIC_VECTOR ( 0 to 0 );
    \FSM_sequential_state_reg_reg[0]\ : out STD_LOGIC;
    \FSM_sequential_state_reg_reg[0]_0\ : out STD_LOGIC_VECTOR ( 0 to 0 );
    \tdi_shift_reg_reg[31]\ : out STD_LOGIC_VECTOR ( 0 to 0 );
    \FSM_sequential_state_reg_reg[1]\ : out STD_LOGIC_VECTOR ( 0 to 0 );
    D : out STD_LOGIC_VECTOR ( 3 downto 0 );
    u_BSCANE2_0 : out STD_LOGIC_VECTOR ( 0 to 0 );
    u_BSCANE2_1 : out STD_LOGIC_VECTOR ( 0 to 0 );
    u_BSCANE2_2 : out STD_LOGIC_VECTOR ( 0 to 0 );
    \FSM_sequential_state_reg_reg[3]\ : out STD_LOGIC;
    u_BSCANE2_3 : out STD_LOGIC;
    \FSM_sequential_state_reg_reg[2]_1\ : out STD_LOGIC;
    u_BSCANE2_4 : out STD_LOGIC;
    \FSM_sequential_state_reg_reg[0]_1\ : out STD_LOGIC;
    \counter_reg[0]\ : in STD_LOGIC;
    Q : in STD_LOGIC_VECTOR ( 3 downto 0 );
    \pad_len_reg[0]\ : in STD_LOGIC;
    \FSM_sequential_state_reg_reg[0]_2\ : in STD_LOGIC;
    \FSM_sequential_state_reg_reg[0]_3\ : in STD_LOGIC;
    \FSM_sequential_state_reg_reg[1]_0\ : in STD_LOGIC;
    \reg_wdata_reg[0]\ : in STD_LOGIC;
    \reg_wdata_reg[0]_0\ : in STD_LOGIC_VECTOR ( 2 downto 0 );
    \read_reg_reg[5]\ : in STD_LOGIC;
    \reg_addr_reg[0]\ : in STD_LOGIC;
    \reg_addr_reg[0]_0\ : in STD_LOGIC_VECTOR ( 1 downto 0 );
    \reg_addr_reg[0]_1\ : in STD_LOGIC;
    \FSM_sequential_state_reg_reg[2]_2\ : in STD_LOGIC;
    \FSM_sequential_state_reg_reg[0]_4\ : in STD_LOGIC;
    u_BSCANE2_5 : in STD_LOGIC_VECTOR ( 0 to 0 );
    u_BSCANE2_6 : in STD_LOGIC_VECTOR ( 2 downto 0 );
    u_BSCANE2_7 : in STD_LOGIC;
    reg_write_reg : in STD_LOGIC;
    reg_write : in STD_LOGIC;
    shift_out_en_reg : in STD_LOGIC;
    shift_in_en_reg : in STD_LOGIC;
    shift_in_data_reg : in STD_LOGIC;
    shift_in_data_reg_0 : in STD_LOGIC_VECTOR ( 0 to 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_vendor_ip1 : entity is "hdlverifier_jtag_vendor_ip1";
end BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_vendor_ip1;

architecture STRUCTURE of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_vendor_ip1 is
  signal \FSM_sequential_state_reg[0]_i_3_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[3]_i_3_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[3]_i_4_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[3]_i_6_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[3]_i_7_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[3]_i_8_n_0\ : STD_LOGIC;
  signal \^fsm_sequential_state_reg_reg[0]\ : STD_LOGIC;
  signal \^sr\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal cdr : STD_LOGIC;
  signal \reg_addr[4]_i_3_n_0\ : STD_LOGIC;
  signal reg_write_i_2_n_0 : STD_LOGIC;
  signal sdr : STD_LOGIC;
  signal sel : STD_LOGIC;
  signal \^shift_dr\ : STD_LOGIC;
  signal shift_out_en_i_2_n_0 : STD_LOGIC;
  signal \^tdi\ : STD_LOGIC;
  signal tdo : STD_LOGIC;
  signal udr : STD_LOGIC;
  signal NLW_u_BSCANE2_DRCK_UNCONNECTED : STD_LOGIC;
  signal NLW_u_BSCANE2_RUNTEST_UNCONNECTED : STD_LOGIC;
  signal NLW_u_BSCANE2_TMS_UNCONNECTED : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \FSM_sequential_state_reg[0]_i_3\ : label is "soft_lutpair35";
  attribute SOFT_HLUTNM of \FSM_sequential_state_reg[3]_i_4\ : label is "soft_lutpair32";
  attribute SOFT_HLUTNM of \command_reg[7]_i_2\ : label is "soft_lutpair35";
  attribute SOFT_HLUTNM of \pad_len[2]_i_2\ : label is "soft_lutpair34";
  attribute SOFT_HLUTNM of \read_reg[21]_i_1\ : label is "soft_lutpair34";
  attribute SOFT_HLUTNM of \reg_addr[4]_i_3\ : label is "soft_lutpair32";
  attribute SOFT_HLUTNM of \reg_wdata[31]_i_1\ : label is "soft_lutpair33";
  attribute SOFT_HLUTNM of reg_write_i_2 : label is "soft_lutpair33";
  attribute SOFT_HLUTNM of \tdi_shift_reg[31]_i_1\ : label is "soft_lutpair36";
  attribute SOFT_HLUTNM of \tdi_shift_reg[31]_i_2\ : label is "soft_lutpair36";
  attribute BOX_TYPE : string;
  attribute BOX_TYPE of u_BSCANE2 : label is "PRIMITIVE";
begin
  \FSM_sequential_state_reg_reg[0]\ <= \^fsm_sequential_state_reg_reg[0]\;
  SR(0) <= \^sr\(0);
  shift_dr <= \^shift_dr\;
  tdi <= \^tdi\;
\FSM_sequential_state_reg[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00000003050F000F"
    )
        port map (
      I0 => \FSM_sequential_state_reg_reg[0]_4\,
      I1 => \FSM_sequential_state_reg[0]_i_3_n_0\,
      I2 => Q(3),
      I3 => Q(0),
      I4 => Q(2),
      I5 => Q(1),
      O => D(0)
    );
\FSM_sequential_state_reg[0]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"FB"
    )
        port map (
      I0 => \^tdi\,
      I1 => \reg_addr_reg[0]_0\(0),
      I2 => \reg_addr_reg[0]_0\(1),
      O => \FSM_sequential_state_reg[0]_i_3_n_0\
    );
\FSM_sequential_state_reg[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"1111111111F11111"
    )
        port map (
      I0 => \FSM_sequential_state_reg_reg[1]_0\,
      I1 => Q(1),
      I2 => \reg_addr_reg[0]_1\,
      I3 => \^tdi\,
      I4 => \reg_addr_reg[0]_0\(0),
      I5 => \reg_addr_reg[0]_0\(1),
      O => D(1)
    );
\FSM_sequential_state_reg[2]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"888F8F8888888888"
    )
        port map (
      I0 => Q(2),
      I1 => \FSM_sequential_state_reg_reg[2]_2\,
      I2 => \reg_addr_reg[0]_0\(0),
      I3 => \^tdi\,
      I4 => \reg_addr_reg[0]_0\(1),
      I5 => \reg_addr_reg[0]_1\,
      O => D(2)
    );
\FSM_sequential_state_reg[3]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFFFFFEAFF"
    )
        port map (
      I0 => \FSM_sequential_state_reg[3]_i_3_n_0\,
      I1 => \FSM_sequential_state_reg[3]_i_4_n_0\,
      I2 => \FSM_sequential_state_reg_reg[0]_2\,
      I3 => \FSM_sequential_state_reg[3]_i_6_n_0\,
      I4 => \FSM_sequential_state_reg[3]_i_7_n_0\,
      I5 => \FSM_sequential_state_reg[3]_i_8_n_0\,
      O => \FSM_sequential_state_reg_reg[2]\(0)
    );
\FSM_sequential_state_reg[3]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000010000000000"
    )
        port map (
      I0 => Q(0),
      I1 => Q(2),
      I2 => \read_reg_reg[5]\,
      I3 => \reg_addr_reg[0]_0\(0),
      I4 => \reg_addr_reg[0]_0\(1),
      I5 => \^tdi\,
      O => D(3)
    );
\FSM_sequential_state_reg[3]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"000000000000F200"
    )
        port map (
      I0 => Q(2),
      I1 => \FSM_sequential_state_reg_reg[0]_3\,
      I2 => \pad_len_reg[0]\,
      I3 => \^shift_dr\,
      I4 => Q(1),
      I5 => \FSM_sequential_state_reg_reg[1]_0\,
      O => \FSM_sequential_state_reg[3]_i_3_n_0\
    );
\FSM_sequential_state_reg[3]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0080"
    )
        port map (
      I0 => sel,
      I1 => sdr,
      I2 => Q(1),
      I3 => Q(3),
      O => \FSM_sequential_state_reg[3]_i_4_n_0\
    );
\FSM_sequential_state_reg[3]_i_6\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFEFFFFFFFFFFFF"
    )
        port map (
      I0 => \reg_wdata_reg[0]\,
      I1 => \reg_wdata_reg[0]_0\(0),
      I2 => \reg_wdata_reg[0]_0\(1),
      I3 => \reg_wdata_reg[0]_0\(2),
      I4 => Q(0),
      I5 => \FSM_sequential_state_reg[3]_i_4_n_0\,
      O => \FSM_sequential_state_reg[3]_i_6_n_0\
    );
\FSM_sequential_state_reg[3]_i_7\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0024000000000000"
    )
        port map (
      I0 => Q(1),
      I1 => Q(3),
      I2 => Q(2),
      I3 => Q(0),
      I4 => udr,
      I5 => sel,
      O => \FSM_sequential_state_reg[3]_i_7_n_0\
    );
\FSM_sequential_state_reg[3]_i_8\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00000008"
    )
        port map (
      I0 => cdr,
      I1 => sel,
      I2 => Q(3),
      I3 => Q(0),
      I4 => Q(1),
      O => \FSM_sequential_state_reg[3]_i_8_n_0\
    );
\command_reg[7]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000D00000005"
    )
        port map (
      I0 => Q(1),
      I1 => \^shift_dr\,
      I2 => Q(3),
      I3 => Q(0),
      I4 => Q(2),
      I5 => \reg_addr_reg[0]\,
      O => \FSM_sequential_state_reg_reg[1]\(0)
    );
\command_reg[7]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \^tdi\,
      I1 => Q(1),
      O => u_BSCANE2_2(0)
    );
\counter[7]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0E0F00000C030003"
    )
        port map (
      I0 => \counter_reg[0]\,
      I1 => Q(0),
      I2 => Q(3),
      I3 => Q(2),
      I4 => \^shift_dr\,
      I5 => Q(1),
      O => \FSM_sequential_state_reg_reg[0]_0\(0)
    );
\pad_len[2]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"1000100010001111"
    )
        port map (
      I0 => Q(1),
      I1 => Q(3),
      I2 => \pad_len_reg[0]\,
      I3 => \^shift_dr\,
      I4 => Q(0),
      I5 => Q(2),
      O => E(0)
    );
\pad_len[2]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \^tdi\,
      I1 => Q(0),
      O => u_BSCANE2_1(0)
    );
\read_reg[21]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"D0"
    )
        port map (
      I0 => Q(0),
      I1 => Q(3),
      I2 => \^fsm_sequential_state_reg_reg[0]\,
      O => \FSM_sequential_state_reg_reg[0]_1\
    );
\read_reg[28]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00000000FF200000"
    )
        port map (
      I0 => Q(0),
      I1 => \read_reg_reg[5]\,
      I2 => \^shift_dr\,
      I3 => \FSM_sequential_state_reg[3]_i_8_n_0\,
      I4 => Q(2),
      I5 => \^sr\(0),
      O => \^fsm_sequential_state_reg_reg[0]\
    );
\reg_addr[4]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000022000000000"
    )
        port map (
      I0 => \reg_addr_reg[0]\,
      I1 => \reg_addr[4]_i_3_n_0\,
      I2 => \reg_addr_reg[0]_0\(1),
      I3 => \reg_addr_reg[0]_0\(0),
      I4 => \^tdi\,
      I5 => \reg_addr_reg[0]_1\,
      O => \tdi_shift_reg_reg[31]\(0)
    );
\reg_addr[4]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"BF"
    )
        port map (
      I0 => \^sr\(0),
      I1 => sel,
      I2 => sdr,
      O => \reg_addr[4]_i_3_n_0\
    );
\reg_wdata[31]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"01"
    )
        port map (
      I0 => \FSM_sequential_state_reg[3]_i_6_n_0\,
      I1 => Q(2),
      I2 => \^sr\(0),
      O => \FSM_sequential_state_reg_reg[2]_0\(0)
    );
reg_write_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"33BB333000880000"
    )
        port map (
      I0 => \^shift_dr\,
      I1 => reg_write_i_2_n_0,
      I2 => reg_write_reg,
      I3 => Q(3),
      I4 => Q(1),
      I5 => reg_write,
      O => \FSM_sequential_state_reg_reg[3]\
    );
reg_write_i_2: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => Q(2),
      I1 => \FSM_sequential_state_reg[3]_i_6_n_0\,
      O => reg_write_i_2_n_0
    );
shift_in_data_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFBFFF00008000"
    )
        port map (
      I0 => \^tdi\,
      I1 => shift_in_data_reg,
      I2 => sdr,
      I3 => sel,
      I4 => \^sr\(0),
      I5 => shift_in_data_reg_0(0),
      O => u_BSCANE2_4
    );
shift_in_en_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFEFF00000200"
    )
        port map (
      I0 => \^shift_dr\,
      I1 => Q(2),
      I2 => Q(0),
      I3 => Q(3),
      I4 => Q(1),
      I5 => shift_in_en_reg,
      O => \FSM_sequential_state_reg_reg[2]_1\
    );
shift_out_en_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0080FFFF00800000"
    )
        port map (
      I0 => sel,
      I1 => sdr,
      I2 => Q(2),
      I3 => Q(3),
      I4 => shift_out_en_i_2_n_0,
      I5 => shift_out_en_reg,
      O => u_BSCANE2_3
    );
shift_out_en_i_2: unisim.vcomponents.LUT6
    generic map(
      INIT => X"000000000070000F"
    )
        port map (
      I0 => \counter_reg[0]\,
      I1 => \^shift_dr\,
      I2 => Q(1),
      I3 => Q(3),
      I4 => Q(2),
      I5 => Q(0),
      O => shift_out_en_i_2_n_0
    );
\tdi_shift_reg[31]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"EA"
    )
        port map (
      I0 => \^sr\(0),
      I1 => cdr,
      I2 => sel,
      O => u_BSCANE2_0(0)
    );
\tdi_shift_reg[31]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => sdr,
      I1 => sel,
      O => \^shift_dr\
    );
u_BSCANE2: unisim.vcomponents.BSCANE2
    generic map(
      DISABLE_JTAG => "FALSE",
      JTAG_CHAIN => 3
    )
        port map (
      CAPTURE => cdr,
      DRCK => NLW_u_BSCANE2_DRCK_UNCONNECTED,
      RESET => \^sr\(0),
      RUNTEST => NLW_u_BSCANE2_RUNTEST_UNCONNECTED,
      SEL => sel,
      SHIFT => sdr,
      TCK => CLK,
      TDI => \^tdi\,
      TDO => tdo,
      TMS => NLW_u_BSCANE2_TMS_UNCONNECTED,
      UPDATE => udr
    );
u_BSCANE2_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFEF0020"
    )
        port map (
      I0 => u_BSCANE2_5(0),
      I1 => u_BSCANE2_6(0),
      I2 => u_BSCANE2_6(2),
      I3 => u_BSCANE2_6(1),
      I4 => u_BSCANE2_7,
      O => tdo
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram is
  port (
    D : out STD_LOGIC_VECTOR ( 63 downto 0 );
    \bitcount_reg[3]\ : out STD_LOGIC;
    tck : in STD_LOGIC;
    aclk : in STD_LOGIC;
    readram_rd : in STD_LOGIC;
    Q : in STD_LOGIC_VECTOR ( 7 downto 0 );
    ADDRBWRADDR : in STD_LOGIC_VECTOR ( 7 downto 0 );
    axi4m_rdata : in STD_LOGIC_VECTOR ( 63 downto 0 );
    axi4m_rvalid : in STD_LOGIC;
    \shift_reg_reg[62]\ : in STD_LOGIC_VECTOR ( 62 downto 0 );
    \shift_reg_reg[0]\ : in STD_LOGIC;
    ram_reg_0 : in STD_LOGIC_VECTOR ( 4 downto 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram : entity is "jtag_master_dcram";
end BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram;

architecture STRUCTURE of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram is
  signal readram_q : STD_LOGIC_VECTOR ( 63 downto 0 );
  signal NLW_ram_reg_CASOUTDBITERR_UNCONNECTED : STD_LOGIC;
  signal NLW_ram_reg_CASOUTSBITERR_UNCONNECTED : STD_LOGIC;
  signal NLW_ram_reg_DBITERR_UNCONNECTED : STD_LOGIC;
  signal NLW_ram_reg_SBITERR_UNCONNECTED : STD_LOGIC;
  signal NLW_ram_reg_CASDINA_UNCONNECTED : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal NLW_ram_reg_CASDINB_UNCONNECTED : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal NLW_ram_reg_CASDINPA_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_ram_reg_CASDINPB_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_ram_reg_CASDOUTA_UNCONNECTED : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal NLW_ram_reg_CASDOUTB_UNCONNECTED : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal NLW_ram_reg_CASDOUTPA_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_ram_reg_CASDOUTPB_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_ram_reg_DOUTPADOUTP_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_ram_reg_DOUTPBDOUTP_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_ram_reg_ECCPARITY_UNCONNECTED : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal NLW_ram_reg_RDADDRECC_UNCONNECTED : STD_LOGIC_VECTOR ( 8 downto 0 );
  attribute \MEM.PORTA.DATA_BIT_LAYOUT\ : string;
  attribute \MEM.PORTA.DATA_BIT_LAYOUT\ of ram_reg : label is "p0_d64";
  attribute \MEM.PORTB.DATA_BIT_LAYOUT\ : string;
  attribute \MEM.PORTB.DATA_BIT_LAYOUT\ of ram_reg : label is "p0_d64";
  attribute METHODOLOGY_DRC_VIOS : string;
  attribute METHODOLOGY_DRC_VIOS of ram_reg : label is "{SYNTH-6 {cell *THIS*}}";
  attribute RDADDR_COLLISION_HWCONFIG : string;
  attribute RDADDR_COLLISION_HWCONFIG of ram_reg : label is "DELAYED_WRITE";
  attribute RTL_RAM_BITS : integer;
  attribute RTL_RAM_BITS of ram_reg : label is 16384;
  attribute RTL_RAM_NAME : string;
  attribute RTL_RAM_NAME of ram_reg : label is "inst/readram/ram_reg";
  attribute RTL_RAM_TYPE : string;
  attribute RTL_RAM_TYPE of ram_reg : label is "RAM_SDP";
  attribute ram_addr_begin : integer;
  attribute ram_addr_begin of ram_reg : label is 0;
  attribute ram_addr_end : integer;
  attribute ram_addr_end of ram_reg : label is 511;
  attribute ram_offset : integer;
  attribute ram_offset of ram_reg : label is 256;
  attribute ram_slice_begin : integer;
  attribute ram_slice_begin of ram_reg : label is 0;
  attribute ram_slice_end : integer;
  attribute ram_slice_end of ram_reg : label is 63;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \shift_reg[0]_i_1\ : label is "soft_lutpair20";
  attribute SOFT_HLUTNM of \shift_reg[10]_i_1\ : label is "soft_lutpair22";
  attribute SOFT_HLUTNM of \shift_reg[11]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \shift_reg[12]_i_1\ : label is "soft_lutpair25";
  attribute SOFT_HLUTNM of \shift_reg[13]_i_1\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \shift_reg[14]_i_1\ : label is "soft_lutpair28";
  attribute SOFT_HLUTNM of \shift_reg[15]_i_1\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \shift_reg[16]_i_1\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \shift_reg[17]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \shift_reg[18]_i_1\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \shift_reg[19]_i_1\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \shift_reg[1]_i_1\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \shift_reg[20]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \shift_reg[21]_i_1\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \shift_reg[22]_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \shift_reg[23]_i_1\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \shift_reg[24]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \shift_reg[25]_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \shift_reg[26]_i_1\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \shift_reg[27]_i_1\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \shift_reg[28]_i_1\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \shift_reg[29]_i_1\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \shift_reg[2]_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \shift_reg[30]_i_1\ : label is "soft_lutpair25";
  attribute SOFT_HLUTNM of \shift_reg[31]_i_1\ : label is "soft_lutpair26";
  attribute SOFT_HLUTNM of \shift_reg[32]_i_1\ : label is "soft_lutpair31";
  attribute SOFT_HLUTNM of \shift_reg[33]_i_1\ : label is "soft_lutpair30";
  attribute SOFT_HLUTNM of \shift_reg[34]_i_1\ : label is "soft_lutpair26";
  attribute SOFT_HLUTNM of \shift_reg[35]_i_1\ : label is "soft_lutpair24";
  attribute SOFT_HLUTNM of \shift_reg[36]_i_1\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \shift_reg[37]_i_1\ : label is "soft_lutpair28";
  attribute SOFT_HLUTNM of \shift_reg[38]_i_1\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \shift_reg[39]_i_1\ : label is "soft_lutpair24";
  attribute SOFT_HLUTNM of \shift_reg[3]_i_1\ : label is "soft_lutpair21";
  attribute SOFT_HLUTNM of \shift_reg[40]_i_1\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \shift_reg[41]_i_1\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \shift_reg[42]_i_1\ : label is "soft_lutpair20";
  attribute SOFT_HLUTNM of \shift_reg[43]_i_1\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \shift_reg[44]_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \shift_reg[45]_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \shift_reg[46]_i_1\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \shift_reg[47]_i_1\ : label is "soft_lutpair22";
  attribute SOFT_HLUTNM of \shift_reg[48]_i_1\ : label is "soft_lutpair29";
  attribute SOFT_HLUTNM of \shift_reg[49]_i_1\ : label is "soft_lutpair31";
  attribute SOFT_HLUTNM of \shift_reg[4]_i_1\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \shift_reg[50]_i_1\ : label is "soft_lutpair23";
  attribute SOFT_HLUTNM of \shift_reg[51]_i_1\ : label is "soft_lutpair29";
  attribute SOFT_HLUTNM of \shift_reg[52]_i_1\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \shift_reg[53]_i_1\ : label is "soft_lutpair27";
  attribute SOFT_HLUTNM of \shift_reg[54]_i_1\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \shift_reg[55]_i_1\ : label is "soft_lutpair19";
  attribute SOFT_HLUTNM of \shift_reg[56]_i_1\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \shift_reg[57]_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \shift_reg[58]_i_1\ : label is "soft_lutpair30";
  attribute SOFT_HLUTNM of \shift_reg[59]_i_1\ : label is "soft_lutpair19";
  attribute SOFT_HLUTNM of \shift_reg[5]_i_1\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \shift_reg[60]_i_1\ : label is "soft_lutpair23";
  attribute SOFT_HLUTNM of \shift_reg[61]_i_1\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \shift_reg[62]_i_1\ : label is "soft_lutpair27";
  attribute SOFT_HLUTNM of \shift_reg[63]_i_2\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \shift_reg[6]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \shift_reg[7]_i_1\ : label is "soft_lutpair21";
  attribute SOFT_HLUTNM of \shift_reg[8]_i_1\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \shift_reg[9]_i_1\ : label is "soft_lutpair1";
begin
ram_reg: unisim.vcomponents.RAMB36E2
    generic map(
      CASCADE_ORDER_A => "NONE",
      CASCADE_ORDER_B => "NONE",
      CLOCK_DOMAINS => "INDEPENDENT",
      DOA_REG => 0,
      DOB_REG => 0,
      ENADDRENA => "FALSE",
      ENADDRENB => "FALSE",
      EN_ECC_PIPE => "FALSE",
      EN_ECC_READ => "FALSE",
      EN_ECC_WRITE => "FALSE",
      INIT_A => X"000000000",
      INIT_B => X"000000000",
      INIT_FILE => "NONE",
      RDADDRCHANGEA => "FALSE",
      RDADDRCHANGEB => "FALSE",
      READ_WIDTH_A => 72,
      READ_WIDTH_B => 0,
      RSTREG_PRIORITY_A => "RSTREG",
      RSTREG_PRIORITY_B => "RSTREG",
      SIM_COLLISION_CHECK => "ALL",
      SLEEP_ASYNC => "FALSE",
      SRVAL_A => X"000000000",
      SRVAL_B => X"000000000",
      WRITE_MODE_A => "WRITE_FIRST",
      WRITE_MODE_B => "WRITE_FIRST",
      WRITE_WIDTH_A => 0,
      WRITE_WIDTH_B => 72
    )
        port map (
      ADDRARDADDR(14) => '1',
      ADDRARDADDR(13 downto 6) => Q(7 downto 0),
      ADDRARDADDR(5 downto 0) => B"111111",
      ADDRBWRADDR(14) => '1',
      ADDRBWRADDR(13 downto 6) => ADDRBWRADDR(7 downto 0),
      ADDRBWRADDR(5 downto 0) => B"111111",
      ADDRENA => '1',
      ADDRENB => '1',
      CASDIMUXA => '0',
      CASDIMUXB => '0',
      CASDINA(31 downto 0) => NLW_ram_reg_CASDINA_UNCONNECTED(31 downto 0),
      CASDINB(31 downto 0) => NLW_ram_reg_CASDINB_UNCONNECTED(31 downto 0),
      CASDINPA(3 downto 0) => NLW_ram_reg_CASDINPA_UNCONNECTED(3 downto 0),
      CASDINPB(3 downto 0) => NLW_ram_reg_CASDINPB_UNCONNECTED(3 downto 0),
      CASDOMUXA => '0',
      CASDOMUXB => '0',
      CASDOMUXEN_A => '1',
      CASDOMUXEN_B => '1',
      CASDOUTA(31 downto 0) => NLW_ram_reg_CASDOUTA_UNCONNECTED(31 downto 0),
      CASDOUTB(31 downto 0) => NLW_ram_reg_CASDOUTB_UNCONNECTED(31 downto 0),
      CASDOUTPA(3 downto 0) => NLW_ram_reg_CASDOUTPA_UNCONNECTED(3 downto 0),
      CASDOUTPB(3 downto 0) => NLW_ram_reg_CASDOUTPB_UNCONNECTED(3 downto 0),
      CASINDBITERR => '0',
      CASINSBITERR => '0',
      CASOREGIMUXA => '0',
      CASOREGIMUXB => '0',
      CASOREGIMUXEN_A => '1',
      CASOREGIMUXEN_B => '1',
      CASOUTDBITERR => NLW_ram_reg_CASOUTDBITERR_UNCONNECTED,
      CASOUTSBITERR => NLW_ram_reg_CASOUTSBITERR_UNCONNECTED,
      CLKARDCLK => tck,
      CLKBWRCLK => aclk,
      DBITERR => NLW_ram_reg_DBITERR_UNCONNECTED,
      DINADIN(31 downto 0) => axi4m_rdata(31 downto 0),
      DINBDIN(31 downto 0) => axi4m_rdata(63 downto 32),
      DINPADINP(3 downto 0) => B"1111",
      DINPBDINP(3 downto 0) => B"1111",
      DOUTADOUT(31 downto 0) => readram_q(31 downto 0),
      DOUTBDOUT(31 downto 0) => readram_q(63 downto 32),
      DOUTPADOUTP(3 downto 0) => NLW_ram_reg_DOUTPADOUTP_UNCONNECTED(3 downto 0),
      DOUTPBDOUTP(3 downto 0) => NLW_ram_reg_DOUTPBDOUTP_UNCONNECTED(3 downto 0),
      ECCPARITY(7 downto 0) => NLW_ram_reg_ECCPARITY_UNCONNECTED(7 downto 0),
      ECCPIPECE => '1',
      ENARDEN => readram_rd,
      ENBWREN => '1',
      INJECTDBITERR => '0',
      INJECTSBITERR => '0',
      RDADDRECC(8 downto 0) => NLW_ram_reg_RDADDRECC_UNCONNECTED(8 downto 0),
      REGCEAREGCE => '1',
      REGCEB => '1',
      RSTRAMARSTRAM => '0',
      RSTRAMB => '0',
      RSTREGARSTREG => '0',
      RSTREGB => '0',
      SBITERR => NLW_ram_reg_SBITERR_UNCONNECTED,
      SLEEP => '0',
      WEA(3 downto 0) => B"0000",
      WEBWE(7) => axi4m_rvalid,
      WEBWE(6) => axi4m_rvalid,
      WEBWE(5) => axi4m_rvalid,
      WEBWE(4) => axi4m_rvalid,
      WEBWE(3) => axi4m_rvalid,
      WEBWE(2) => axi4m_rvalid,
      WEBWE(1) => axi4m_rvalid,
      WEBWE(0) => axi4m_rvalid
    );
\ram_reg_i_2__0\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00000001"
    )
        port map (
      I0 => ram_reg_0(2),
      I1 => ram_reg_0(3),
      I2 => ram_reg_0(1),
      I3 => ram_reg_0(0),
      I4 => ram_reg_0(4),
      O => \bitcount_reg[3]\
    );
\shift_reg[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(0),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(0),
      O => D(0)
    );
\shift_reg[10]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(10),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(10),
      O => D(10)
    );
\shift_reg[11]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(11),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(11),
      O => D(11)
    );
\shift_reg[12]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(12),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(12),
      O => D(12)
    );
\shift_reg[13]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(13),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(13),
      O => D(13)
    );
\shift_reg[14]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(14),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(14),
      O => D(14)
    );
\shift_reg[15]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(15),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(15),
      O => D(15)
    );
\shift_reg[16]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(16),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(16),
      O => D(16)
    );
\shift_reg[17]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(17),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(17),
      O => D(17)
    );
\shift_reg[18]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(18),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(18),
      O => D(18)
    );
\shift_reg[19]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(19),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(19),
      O => D(19)
    );
\shift_reg[1]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(1),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(1),
      O => D(1)
    );
\shift_reg[20]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(20),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(20),
      O => D(20)
    );
\shift_reg[21]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(21),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(21),
      O => D(21)
    );
\shift_reg[22]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(22),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(22),
      O => D(22)
    );
\shift_reg[23]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(23),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(23),
      O => D(23)
    );
\shift_reg[24]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(24),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(24),
      O => D(24)
    );
\shift_reg[25]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(25),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(25),
      O => D(25)
    );
\shift_reg[26]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(26),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(26),
      O => D(26)
    );
\shift_reg[27]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(27),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(27),
      O => D(27)
    );
\shift_reg[28]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(28),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(28),
      O => D(28)
    );
\shift_reg[29]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(29),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(29),
      O => D(29)
    );
\shift_reg[2]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(2),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(2),
      O => D(2)
    );
\shift_reg[30]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(30),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(30),
      O => D(30)
    );
\shift_reg[31]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(31),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(31),
      O => D(31)
    );
\shift_reg[32]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(32),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(32),
      O => D(32)
    );
\shift_reg[33]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(33),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(33),
      O => D(33)
    );
\shift_reg[34]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(34),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(34),
      O => D(34)
    );
\shift_reg[35]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(35),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(35),
      O => D(35)
    );
\shift_reg[36]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(36),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(36),
      O => D(36)
    );
\shift_reg[37]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(37),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(37),
      O => D(37)
    );
\shift_reg[38]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(38),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(38),
      O => D(38)
    );
\shift_reg[39]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(39),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(39),
      O => D(39)
    );
\shift_reg[3]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(3),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(3),
      O => D(3)
    );
\shift_reg[40]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(40),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(40),
      O => D(40)
    );
\shift_reg[41]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(41),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(41),
      O => D(41)
    );
\shift_reg[42]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(42),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(42),
      O => D(42)
    );
\shift_reg[43]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(43),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(43),
      O => D(43)
    );
\shift_reg[44]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(44),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(44),
      O => D(44)
    );
\shift_reg[45]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(45),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(45),
      O => D(45)
    );
\shift_reg[46]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(46),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(46),
      O => D(46)
    );
\shift_reg[47]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(47),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(47),
      O => D(47)
    );
\shift_reg[48]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(48),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(48),
      O => D(48)
    );
\shift_reg[49]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(49),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(49),
      O => D(49)
    );
\shift_reg[4]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(4),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(4),
      O => D(4)
    );
\shift_reg[50]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(50),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(50),
      O => D(50)
    );
\shift_reg[51]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(51),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(51),
      O => D(51)
    );
\shift_reg[52]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(52),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(52),
      O => D(52)
    );
\shift_reg[53]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(53),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(53),
      O => D(53)
    );
\shift_reg[54]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(54),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(54),
      O => D(54)
    );
\shift_reg[55]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(55),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(55),
      O => D(55)
    );
\shift_reg[56]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(56),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(56),
      O => D(56)
    );
\shift_reg[57]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(57),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(57),
      O => D(57)
    );
\shift_reg[58]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(58),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(58),
      O => D(58)
    );
\shift_reg[59]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(59),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(59),
      O => D(59)
    );
\shift_reg[5]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(5),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(5),
      O => D(5)
    );
\shift_reg[60]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(60),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(60),
      O => D(60)
    );
\shift_reg[61]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(61),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(61),
      O => D(61)
    );
\shift_reg[62]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(62),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(62),
      O => D(62)
    );
\shift_reg[63]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => readram_q(63),
      I1 => \shift_reg_reg[0]\,
      O => D(63)
    );
\shift_reg[6]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(6),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(6),
      O => D(6)
    );
\shift_reg[7]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(7),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(7),
      O => D(7)
    );
\shift_reg[8]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(8),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(8),
      O => D(8)
    );
\shift_reg[9]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \shift_reg_reg[62]\(9),
      I1 => \shift_reg_reg[0]\,
      I2 => readram_q(9),
      O => D(9)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram_0 is
  port (
    axi4m_wdata : out STD_LOGIC_VECTOR ( 63 downto 0 );
    D : out STD_LOGIC_VECTOR ( 7 downto 0 );
    aclk : in STD_LOGIC;
    CLK : in STD_LOGIC;
    writeram_rd : in STD_LOGIC;
    Q : in STD_LOGIC_VECTOR ( 7 downto 0 );
    ram_reg_0 : in STD_LOGIC_VECTOR ( 63 downto 0 );
    WEBWE : in STD_LOGIC_VECTOR ( 0 to 0 );
    ram_reg_1 : in STD_LOGIC_VECTOR ( 7 downto 0 );
    axi4m_wready : in STD_LOGIC;
    ram_reg_2 : in STD_LOGIC
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram_0 : entity is "jtag_master_dcram";
end BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram_0;

architecture STRUCTURE of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram_0 is
  signal \^d\ : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal ram_reg_i_10_n_0 : STD_LOGIC;
  signal ram_reg_i_9_n_0 : STD_LOGIC;
  signal NLW_ram_reg_CASOUTDBITERR_UNCONNECTED : STD_LOGIC;
  signal NLW_ram_reg_CASOUTSBITERR_UNCONNECTED : STD_LOGIC;
  signal NLW_ram_reg_DBITERR_UNCONNECTED : STD_LOGIC;
  signal NLW_ram_reg_SBITERR_UNCONNECTED : STD_LOGIC;
  signal NLW_ram_reg_CASDINA_UNCONNECTED : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal NLW_ram_reg_CASDINB_UNCONNECTED : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal NLW_ram_reg_CASDINPA_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_ram_reg_CASDINPB_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_ram_reg_CASDOUTA_UNCONNECTED : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal NLW_ram_reg_CASDOUTB_UNCONNECTED : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal NLW_ram_reg_CASDOUTPA_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_ram_reg_CASDOUTPB_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_ram_reg_DOUTPADOUTP_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_ram_reg_DOUTPBDOUTP_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_ram_reg_ECCPARITY_UNCONNECTED : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal NLW_ram_reg_RDADDRECC_UNCONNECTED : STD_LOGIC_VECTOR ( 8 downto 0 );
  attribute \MEM.PORTA.DATA_BIT_LAYOUT\ : string;
  attribute \MEM.PORTA.DATA_BIT_LAYOUT\ of ram_reg : label is "p0_d64";
  attribute \MEM.PORTB.DATA_BIT_LAYOUT\ : string;
  attribute \MEM.PORTB.DATA_BIT_LAYOUT\ of ram_reg : label is "p0_d64";
  attribute METHODOLOGY_DRC_VIOS : string;
  attribute METHODOLOGY_DRC_VIOS of ram_reg : label is "{SYNTH-6 {cell *THIS*}}";
  attribute RDADDR_COLLISION_HWCONFIG : string;
  attribute RDADDR_COLLISION_HWCONFIG of ram_reg : label is "DELAYED_WRITE";
  attribute RTL_RAM_BITS : integer;
  attribute RTL_RAM_BITS of ram_reg : label is 16384;
  attribute RTL_RAM_NAME : string;
  attribute RTL_RAM_NAME of ram_reg : label is "inst/writeram/ram_reg";
  attribute RTL_RAM_TYPE : string;
  attribute RTL_RAM_TYPE of ram_reg : label is "RAM_SDP";
  attribute ram_addr_begin : integer;
  attribute ram_addr_begin of ram_reg : label is 0;
  attribute ram_addr_end : integer;
  attribute ram_addr_end of ram_reg : label is 511;
  attribute ram_offset : integer;
  attribute ram_offset of ram_reg : label is 256;
  attribute ram_slice_begin : integer;
  attribute ram_slice_begin of ram_reg : label is 0;
  attribute ram_slice_end : integer;
  attribute ram_slice_end of ram_reg : label is 63;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of ram_reg_i_2 : label is "soft_lutpair54";
  attribute SOFT_HLUTNM of ram_reg_i_3 : label is "soft_lutpair54";
  attribute SOFT_HLUTNM of ram_reg_i_6 : label is "soft_lutpair53";
  attribute SOFT_HLUTNM of ram_reg_i_7 : label is "soft_lutpair55";
  attribute SOFT_HLUTNM of ram_reg_i_8 : label is "soft_lutpair55";
  attribute SOFT_HLUTNM of ram_reg_i_9 : label is "soft_lutpair53";
begin
  D(7 downto 0) <= \^d\(7 downto 0);
ram_reg: unisim.vcomponents.RAMB36E2
    generic map(
      CASCADE_ORDER_A => "NONE",
      CASCADE_ORDER_B => "NONE",
      CLOCK_DOMAINS => "INDEPENDENT",
      DOA_REG => 0,
      DOB_REG => 0,
      ENADDRENA => "FALSE",
      ENADDRENB => "FALSE",
      EN_ECC_PIPE => "FALSE",
      EN_ECC_READ => "FALSE",
      EN_ECC_WRITE => "FALSE",
      INIT_A => X"000000000",
      INIT_B => X"000000000",
      INIT_FILE => "NONE",
      RDADDRCHANGEA => "FALSE",
      RDADDRCHANGEB => "FALSE",
      READ_WIDTH_A => 72,
      READ_WIDTH_B => 0,
      RSTREG_PRIORITY_A => "RSTREG",
      RSTREG_PRIORITY_B => "RSTREG",
      SIM_COLLISION_CHECK => "ALL",
      SLEEP_ASYNC => "FALSE",
      SRVAL_A => X"000000000",
      SRVAL_B => X"000000000",
      WRITE_MODE_A => "WRITE_FIRST",
      WRITE_MODE_B => "WRITE_FIRST",
      WRITE_WIDTH_A => 0,
      WRITE_WIDTH_B => 72
    )
        port map (
      ADDRARDADDR(14) => '1',
      ADDRARDADDR(13 downto 6) => \^d\(7 downto 0),
      ADDRARDADDR(5 downto 0) => B"111111",
      ADDRBWRADDR(14) => '1',
      ADDRBWRADDR(13 downto 6) => Q(7 downto 0),
      ADDRBWRADDR(5 downto 0) => B"111111",
      ADDRENA => '1',
      ADDRENB => '1',
      CASDIMUXA => '0',
      CASDIMUXB => '0',
      CASDINA(31 downto 0) => NLW_ram_reg_CASDINA_UNCONNECTED(31 downto 0),
      CASDINB(31 downto 0) => NLW_ram_reg_CASDINB_UNCONNECTED(31 downto 0),
      CASDINPA(3 downto 0) => NLW_ram_reg_CASDINPA_UNCONNECTED(3 downto 0),
      CASDINPB(3 downto 0) => NLW_ram_reg_CASDINPB_UNCONNECTED(3 downto 0),
      CASDOMUXA => '0',
      CASDOMUXB => '0',
      CASDOMUXEN_A => '1',
      CASDOMUXEN_B => '1',
      CASDOUTA(31 downto 0) => NLW_ram_reg_CASDOUTA_UNCONNECTED(31 downto 0),
      CASDOUTB(31 downto 0) => NLW_ram_reg_CASDOUTB_UNCONNECTED(31 downto 0),
      CASDOUTPA(3 downto 0) => NLW_ram_reg_CASDOUTPA_UNCONNECTED(3 downto 0),
      CASDOUTPB(3 downto 0) => NLW_ram_reg_CASDOUTPB_UNCONNECTED(3 downto 0),
      CASINDBITERR => '0',
      CASINSBITERR => '0',
      CASOREGIMUXA => '0',
      CASOREGIMUXB => '0',
      CASOREGIMUXEN_A => '1',
      CASOREGIMUXEN_B => '1',
      CASOUTDBITERR => NLW_ram_reg_CASOUTDBITERR_UNCONNECTED,
      CASOUTSBITERR => NLW_ram_reg_CASOUTSBITERR_UNCONNECTED,
      CLKARDCLK => aclk,
      CLKBWRCLK => CLK,
      DBITERR => NLW_ram_reg_DBITERR_UNCONNECTED,
      DINADIN(31 downto 0) => ram_reg_0(31 downto 0),
      DINBDIN(31 downto 0) => ram_reg_0(63 downto 32),
      DINPADINP(3 downto 0) => B"1111",
      DINPBDINP(3 downto 0) => B"1111",
      DOUTADOUT(31 downto 0) => axi4m_wdata(31 downto 0),
      DOUTBDOUT(31 downto 0) => axi4m_wdata(63 downto 32),
      DOUTPADOUTP(3 downto 0) => NLW_ram_reg_DOUTPADOUTP_UNCONNECTED(3 downto 0),
      DOUTPBDOUTP(3 downto 0) => NLW_ram_reg_DOUTPBDOUTP_UNCONNECTED(3 downto 0),
      ECCPARITY(7 downto 0) => NLW_ram_reg_ECCPARITY_UNCONNECTED(7 downto 0),
      ECCPIPECE => '1',
      ENARDEN => writeram_rd,
      ENBWREN => '1',
      INJECTDBITERR => '0',
      INJECTSBITERR => '0',
      RDADDRECC(8 downto 0) => NLW_ram_reg_RDADDRECC_UNCONNECTED(8 downto 0),
      REGCEAREGCE => '1',
      REGCEB => '1',
      RSTRAMARSTRAM => '0',
      RSTRAMB => '0',
      RSTREGARSTREG => '0',
      RSTREGB => '0',
      SBITERR => NLW_ram_reg_SBITERR_UNCONNECTED,
      SLEEP => '0',
      WEA(3 downto 0) => B"0000",
      WEBWE(7) => WEBWE(0),
      WEBWE(6) => WEBWE(0),
      WEBWE(5) => WEBWE(0),
      WEBWE(4) => WEBWE(0),
      WEBWE(3) => WEBWE(0),
      WEBWE(2) => WEBWE(0),
      WEBWE(1) => WEBWE(0),
      WEBWE(0) => WEBWE(0)
    );
ram_reg_i_10: unisim.vcomponents.LUT2
    generic map(
      INIT => X"7"
    )
        port map (
      I0 => ram_reg_2,
      I1 => axi4m_wready,
      O => ram_reg_i_10_n_0
    );
\ram_reg_i_1__0\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"6AAAAAAAAAAAAAAA"
    )
        port map (
      I0 => ram_reg_1(7),
      I1 => ram_reg_1(5),
      I2 => ram_reg_1(3),
      I3 => ram_reg_i_9_n_0,
      I4 => ram_reg_1(4),
      I5 => ram_reg_1(6),
      O => \^d\(7)
    );
ram_reg_i_2: unisim.vcomponents.LUT5
    generic map(
      INIT => X"6AAAAAAA"
    )
        port map (
      I0 => ram_reg_1(6),
      I1 => ram_reg_1(4),
      I2 => ram_reg_i_9_n_0,
      I3 => ram_reg_1(3),
      I4 => ram_reg_1(5),
      O => \^d\(6)
    );
ram_reg_i_3: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6AAA"
    )
        port map (
      I0 => ram_reg_1(5),
      I1 => ram_reg_1(3),
      I2 => ram_reg_i_9_n_0,
      I3 => ram_reg_1(4),
      O => \^d\(5)
    );
ram_reg_i_4: unisim.vcomponents.LUT6
    generic map(
      INIT => X"AA6AAAAAAAAAAAAA"
    )
        port map (
      I0 => ram_reg_1(4),
      I1 => ram_reg_1(2),
      I2 => ram_reg_1(1),
      I3 => ram_reg_i_10_n_0,
      I4 => ram_reg_1(0),
      I5 => ram_reg_1(3),
      O => \^d\(4)
    );
ram_reg_i_5: unisim.vcomponents.LUT6
    generic map(
      INIT => X"6AAAAAAAAAAAAAAA"
    )
        port map (
      I0 => ram_reg_1(3),
      I1 => ram_reg_1(0),
      I2 => ram_reg_2,
      I3 => axi4m_wready,
      I4 => ram_reg_1(1),
      I5 => ram_reg_1(2),
      O => \^d\(3)
    );
ram_reg_i_6: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFF8000"
    )
        port map (
      I0 => ram_reg_1(0),
      I1 => ram_reg_2,
      I2 => axi4m_wready,
      I3 => ram_reg_1(1),
      I4 => ram_reg_1(2),
      O => \^d\(2)
    );
ram_reg_i_7: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6AAA"
    )
        port map (
      I0 => ram_reg_1(1),
      I1 => axi4m_wready,
      I2 => ram_reg_2,
      I3 => ram_reg_1(0),
      O => \^d\(1)
    );
ram_reg_i_8: unisim.vcomponents.LUT3
    generic map(
      INIT => X"6A"
    )
        port map (
      I0 => ram_reg_1(0),
      I1 => axi4m_wready,
      I2 => ram_reg_2,
      O => \^d\(0)
    );
ram_reg_i_9: unisim.vcomponents.LUT5
    generic map(
      INIT => X"80000000"
    )
        port map (
      I0 => ram_reg_1(2),
      I1 => ram_reg_1(1),
      I2 => axi4m_wready,
      I3 => ram_reg_2,
      I4 => ram_reg_1(0),
      O => ram_reg_i_9_n_0
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_core is
  port (
    jtag_reset : out STD_LOGIC;
    tck : out STD_LOGIC;
    shift_out_en : out STD_LOGIC;
    shift_in_en : out STD_LOGIC;
    D : out STD_LOGIC_VECTOR ( 0 to 0 );
    E : out STD_LOGIC_VECTOR ( 0 to 0 );
    tck_reset_reg : out STD_LOGIC_VECTOR ( 0 to 0 );
    readram_rd : out STD_LOGIC;
    shift_out_en_reg_0 : out STD_LOGIC_VECTOR ( 0 to 0 );
    \write_count_reg[15]\ : out STD_LOGIC;
    tck_reset_reg_0 : out STD_LOGIC;
    shift_in_en_reg_0 : out STD_LOGIC_VECTOR ( 0 to 0 );
    shift_in_state : out STD_LOGIC;
    shift_out_en_reg_1 : out STD_LOGIC_VECTOR ( 0 to 0 );
    shift_in_state_d1_reg : out STD_LOGIC;
    \reg_wdata_reg[0]_0\ : out STD_LOGIC;
    \reg_wdata_reg[31]_0\ : out STD_LOGIC_VECTOR ( 31 downto 0 );
    SR : out STD_LOGIC_VECTOR ( 0 to 0 );
    status : in STD_LOGIC_VECTOR ( 1 downto 0 );
    aresetn : in STD_LOGIC;
    Q : in STD_LOGIC_VECTOR ( 0 to 0 );
    tck_reset : in STD_LOGIC;
    ram_reg : in STD_LOGIC;
    ram_reg_0 : in STD_LOGIC_VECTOR ( 0 to 0 );
    u_BSCANE2 : in STD_LOGIC_VECTOR ( 0 to 0 );
    S : in STD_LOGIC_VECTOR ( 1 downto 0 );
    writeram_wr_reg : in STD_LOGIC_VECTOR ( 4 downto 0 );
    \readram_raddr_reg[0]\ : in STD_LOGIC;
    writeram_wr_reg_0 : in STD_LOGIC;
    shift_in_state_d1 : in STD_LOGIC;
    start_write : in STD_LOGIC;
    trigger_tck_d1 : in STD_LOGIC;
    start_read_reg : in STD_LOGIC
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_core : entity is "hdlverifier_jtag_core";
end BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_core;

architecture STRUCTURE of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_core is
  signal \^d\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal \FSM_sequential_state_reg[0]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[1]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[2]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[2]_i_3_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[3]_i_10_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[3]_i_11_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[3]_i_12_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[3]_i_5_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state_reg[3]_i_9_n_0\ : STD_LOGIC;
  signal \burst_length[7]_i_2_n_0\ : STD_LOGIC;
  signal command_reg : STD_LOGIC_VECTOR ( 7 downto 5 );
  signal command_reg_1 : STD_LOGIC;
  signal counter : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal \counter[0]_i_1_n_0\ : STD_LOGIC;
  signal \counter[1]_i_1_n_0\ : STD_LOGIC;
  signal \counter[2]_i_1_n_0\ : STD_LOGIC;
  signal \counter[3]_i_1_n_0\ : STD_LOGIC;
  signal \counter[4]_i_1_n_0\ : STD_LOGIC;
  signal \counter[5]_i_1_n_0\ : STD_LOGIC;
  signal \counter[6]_i_1_n_0\ : STD_LOGIC;
  signal \counter[7]_i_2_n_0\ : STD_LOGIC;
  signal \counter[7]_i_3_n_0\ : STD_LOGIC;
  signal \counter[7]_i_4_n_0\ : STD_LOGIC;
  signal \counter[7]_i_5_n_0\ : STD_LOGIC;
  signal counter_0 : STD_LOGIC;
  signal \^jtag_reset\ : STD_LOGIC;
  signal p_0_in : STD_LOGIC_VECTOR ( 2 downto 0 );
  signal pad_len : STD_LOGIC;
  signal \pad_len[0]_i_1_n_0\ : STD_LOGIC;
  signal \pad_len[1]_i_1_n_0\ : STD_LOGIC;
  signal \pad_len[2]_i_3_n_0\ : STD_LOGIC;
  signal \pad_len_reg_n_0_[0]\ : STD_LOGIC;
  signal \pad_len_reg_n_0_[1]\ : STD_LOGIC;
  signal \pad_len_reg_n_0_[2]\ : STD_LOGIC;
  signal read_reg : STD_LOGIC_VECTOR ( 28 downto 0 );
  signal \read_reg[16]_i_1_n_0\ : STD_LOGIC;
  signal \read_reg[18]_i_1_n_0\ : STD_LOGIC;
  signal \read_reg[19]_i_1_n_0\ : STD_LOGIC;
  signal \read_reg[1]_i_2_n_0\ : STD_LOGIC;
  signal \read_reg[1]_i_3_n_0\ : STD_LOGIC;
  signal \read_reg[22]_i_1_n_0\ : STD_LOGIC;
  signal \read_reg[23]_i_1_n_0\ : STD_LOGIC;
  signal \read_reg[24]_i_1_n_0\ : STD_LOGIC;
  signal \read_reg[25]_i_1_n_0\ : STD_LOGIC;
  signal \read_reg[26]_i_1_n_0\ : STD_LOGIC;
  signal \read_reg[27]_i_1_n_0\ : STD_LOGIC;
  signal \read_reg[28]_i_3_n_0\ : STD_LOGIC;
  signal \read_reg[3]_i_2_n_0\ : STD_LOGIC;
  signal \read_reg[5]_i_1_n_0\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[0]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[10]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[11]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[12]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[13]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[14]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[15]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[16]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[17]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[18]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[19]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[1]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[20]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[21]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[22]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[23]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[24]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[25]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[26]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[27]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[28]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[2]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[3]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[4]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[5]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[6]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[7]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[8]\ : STD_LOGIC;
  signal \read_reg_reg_n_0_[9]\ : STD_LOGIC;
  signal reg_addr : STD_LOGIC_VECTOR ( 4 downto 0 );
  signal \reg_addr[4]_i_2_n_0\ : STD_LOGIC;
  signal \^reg_wdata_reg[31]_0\ : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal reg_write : STD_LOGIC;
  signal reg_write_i_3_n_0 : STD_LOGIC;
  signal shift_dr : STD_LOGIC;
  signal shift_in_data_i_2_n_0 : STD_LOGIC;
  signal \^shift_in_en\ : STD_LOGIC;
  signal \^shift_out_en\ : STD_LOGIC;
  signal start_read_i_2_n_0 : STD_LOGIC;
  signal \state_reg__0\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \state_reg__1\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \^tck\ : STD_LOGIC;
  signal \^tck_reset_reg_0\ : STD_LOGIC;
  signal tdi : STD_LOGIC;
  signal tdi_shift_reg : STD_LOGIC_VECTOR ( 31 downto 25 );
  signal tdi_shift_reg_next : STD_LOGIC_VECTOR ( 23 downto 0 );
  signal u_jtag_n_15 : STD_LOGIC;
  signal u_jtag_n_16 : STD_LOGIC;
  signal u_jtag_n_18 : STD_LOGIC;
  signal u_jtag_n_19 : STD_LOGIC;
  signal u_jtag_n_20 : STD_LOGIC;
  signal u_jtag_n_21 : STD_LOGIC;
  signal u_jtag_n_22 : STD_LOGIC;
  signal u_jtag_n_5 : STD_LOGIC;
  signal u_jtag_n_6 : STD_LOGIC;
  signal u_jtag_n_7 : STD_LOGIC;
  signal u_jtag_n_9 : STD_LOGIC;
  signal writeram_wr_i_2_n_0 : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \FSM_sequential_state_reg[0]_i_2\ : label is "soft_lutpair37";
  attribute SOFT_HLUTNM of \FSM_sequential_state_reg[1]_i_2\ : label is "soft_lutpair40";
  attribute SOFT_HLUTNM of \FSM_sequential_state_reg[2]_i_2\ : label is "soft_lutpair50";
  attribute SOFT_HLUTNM of \FSM_sequential_state_reg[2]_i_3\ : label is "soft_lutpair47";
  attribute SOFT_HLUTNM of \FSM_sequential_state_reg[3]_i_9\ : label is "soft_lutpair50";
  attribute FSM_ENCODED_STATES : string;
  attribute FSM_ENCODED_STATES of \FSM_sequential_state_reg_reg[0]\ : label is "STATE_WRITE_REG:0011,STATE_READ_REG:0111,STATE_CMD:0010,STATE_HEADER:0001,STATE_IDLE:0000,STATE_READ_DATA:0110,STATE_WR_TRANSIT:0100,STATE_WRITE_DATA:1000,STATE_WAIT_PAD:0101";
  attribute FSM_ENCODED_STATES of \FSM_sequential_state_reg_reg[1]\ : label is "STATE_WRITE_REG:0011,STATE_READ_REG:0111,STATE_CMD:0010,STATE_HEADER:0001,STATE_IDLE:0000,STATE_READ_DATA:0110,STATE_WR_TRANSIT:0100,STATE_WRITE_DATA:1000,STATE_WAIT_PAD:0101";
  attribute FSM_ENCODED_STATES of \FSM_sequential_state_reg_reg[2]\ : label is "STATE_WRITE_REG:0011,STATE_READ_REG:0111,STATE_CMD:0010,STATE_HEADER:0001,STATE_IDLE:0000,STATE_READ_DATA:0110,STATE_WR_TRANSIT:0100,STATE_WRITE_DATA:1000,STATE_WAIT_PAD:0101";
  attribute FSM_ENCODED_STATES of \FSM_sequential_state_reg_reg[3]\ : label is "STATE_WRITE_REG:0011,STATE_READ_REG:0111,STATE_CMD:0010,STATE_HEADER:0001,STATE_IDLE:0000,STATE_READ_DATA:0110,STATE_WR_TRANSIT:0100,STATE_WRITE_DATA:1000,STATE_WAIT_PAD:0101";
  attribute SOFT_HLUTNM of \bitcount[5]_i_1\ : label is "soft_lutpair44";
  attribute SOFT_HLUTNM of \burst_length[7]_i_1\ : label is "soft_lutpair42";
  attribute SOFT_HLUTNM of \command_reg[5]_i_1\ : label is "soft_lutpair52";
  attribute SOFT_HLUTNM of \command_reg[6]_i_1\ : label is "soft_lutpair51";
  attribute SOFT_HLUTNM of \counter[0]_i_1\ : label is "soft_lutpair48";
  attribute SOFT_HLUTNM of \counter[1]_i_1\ : label is "soft_lutpair48";
  attribute SOFT_HLUTNM of \counter[2]_i_1\ : label is "soft_lutpair38";
  attribute SOFT_HLUTNM of \counter[3]_i_1\ : label is "soft_lutpair38";
  attribute SOFT_HLUTNM of \counter[6]_i_1\ : label is "soft_lutpair39";
  attribute SOFT_HLUTNM of \counter[7]_i_2\ : label is "soft_lutpair39";
  attribute SOFT_HLUTNM of \counter[7]_i_3\ : label is "soft_lutpair46";
  attribute SOFT_HLUTNM of \pad_len[0]_i_1\ : label is "soft_lutpair52";
  attribute SOFT_HLUTNM of \pad_len[1]_i_1\ : label is "soft_lutpair51";
  attribute SOFT_HLUTNM of ram_reg_i_1 : label is "soft_lutpair37";
  attribute SOFT_HLUTNM of \read_reg[0]_i_1\ : label is "soft_lutpair40";
  attribute SOFT_HLUTNM of \read_reg[1]_i_2\ : label is "soft_lutpair49";
  attribute SOFT_HLUTNM of \read_reg[1]_i_3\ : label is "soft_lutpair42";
  attribute SOFT_HLUTNM of \read_reg[28]_i_3\ : label is "soft_lutpair41";
  attribute SOFT_HLUTNM of \read_reg[2]_i_1\ : label is "soft_lutpair41";
  attribute SOFT_HLUTNM of \reg_addr[4]_i_2\ : label is "soft_lutpair46";
  attribute SOFT_HLUTNM of reg_write_i_3 : label is "soft_lutpair49";
  attribute SOFT_HLUTNM of shift_in_data_i_2 : label is "soft_lutpair47";
  attribute SOFT_HLUTNM of \shift_in_reg[63]_i_1\ : label is "soft_lutpair43";
  attribute SOFT_HLUTNM of shift_in_state_d1_i_2 : label is "soft_lutpair45";
  attribute SOFT_HLUTNM of \shift_reg[63]_i_1\ : label is "soft_lutpair44";
  attribute SOFT_HLUTNM of start_write_i_1 : label is "soft_lutpair45";
  attribute SOFT_HLUTNM of \write_count[0]_i_1\ : label is "soft_lutpair43";
begin
  D(0) <= \^d\(0);
  jtag_reset <= \^jtag_reset\;
  \reg_wdata_reg[31]_0\(31 downto 0) <= \^reg_wdata_reg[31]_0\(31 downto 0);
  shift_in_en <= \^shift_in_en\;
  shift_out_en <= \^shift_out_en\;
  tck <= \^tck\;
  tck_reset_reg_0 <= \^tck_reset_reg_0\;
\FSM_sequential_state_reg[0]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"04"
    )
        port map (
      I0 => p_0_in(0),
      I1 => p_0_in(2),
      I2 => p_0_in(1),
      O => \FSM_sequential_state_reg[0]_i_2_n_0\
    );
\FSM_sequential_state_reg[1]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"B"
    )
        port map (
      I0 => \state_reg__0\(3),
      I1 => \state_reg__0\(0),
      O => \FSM_sequential_state_reg[1]_i_2_n_0\
    );
\FSM_sequential_state_reg[2]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \state_reg__0\(3),
      I1 => \state_reg__0\(1),
      O => \FSM_sequential_state_reg[2]_i_2_n_0\
    );
\FSM_sequential_state_reg[2]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0002"
    )
        port map (
      I0 => \state_reg__0\(1),
      I1 => \state_reg__0\(3),
      I2 => \state_reg__0\(2),
      I3 => \state_reg__0\(0),
      O => \FSM_sequential_state_reg[2]_i_3_n_0\
    );
\FSM_sequential_state_reg[3]_i_10\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFFFFFFFFE"
    )
        port map (
      I0 => counter(4),
      I1 => counter(3),
      I2 => counter(5),
      I3 => counter(6),
      I4 => counter(7),
      I5 => \FSM_sequential_state_reg[3]_i_12_n_0\,
      O => \FSM_sequential_state_reg[3]_i_10_n_0\
    );
\FSM_sequential_state_reg[3]_i_11\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFFFFFE"
    )
        port map (
      I0 => counter(7),
      I1 => counter(6),
      I2 => counter(5),
      I3 => counter(3),
      I4 => counter(4),
      O => \FSM_sequential_state_reg[3]_i_11_n_0\
    );
\FSM_sequential_state_reg[3]_i_12\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"6FF6FFFFFFFF6FF6"
    )
        port map (
      I0 => counter(1),
      I1 => \pad_len_reg_n_0_[1]\,
      I2 => counter(0),
      I3 => \pad_len_reg_n_0_[0]\,
      I4 => \pad_len_reg_n_0_[2]\,
      I5 => counter(2),
      O => \FSM_sequential_state_reg[3]_i_12_n_0\
    );
\FSM_sequential_state_reg[3]_i_5\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000080"
    )
        port map (
      I0 => counter(0),
      I1 => counter(1),
      I2 => counter(2),
      I3 => \FSM_sequential_state_reg[3]_i_11_n_0\,
      I4 => \state_reg__0\(2),
      I5 => \state_reg__0\(0),
      O => \FSM_sequential_state_reg[3]_i_5_n_0\
    );
\FSM_sequential_state_reg[3]_i_9\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"B"
    )
        port map (
      I0 => \state_reg__0\(3),
      I1 => \state_reg__0\(1),
      O => \FSM_sequential_state_reg[3]_i_9_n_0\
    );
\FSM_sequential_state_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_5,
      D => \state_reg__1\(0),
      Q => \state_reg__0\(0),
      R => \^jtag_reset\
    );
\FSM_sequential_state_reg_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_5,
      D => \state_reg__1\(1),
      Q => \state_reg__0\(1),
      R => \^jtag_reset\
    );
\FSM_sequential_state_reg_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_5,
      D => \state_reg__1\(2),
      Q => \state_reg__0\(2),
      R => \^jtag_reset\
    );
\FSM_sequential_state_reg_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_5,
      D => \state_reg__1\(3),
      Q => \state_reg__0\(3),
      R => \^jtag_reset\
    );
\address[31]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000020"
    )
        port map (
      I0 => reg_write,
      I1 => reg_addr(1),
      I2 => reg_addr(0),
      I3 => reg_addr(2),
      I4 => reg_addr(3),
      I5 => reg_addr(4),
      O => E(0)
    );
\bitcount[5]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFEF"
    )
        port map (
      I0 => tck_reset,
      I1 => p_0_in(1),
      I2 => p_0_in(2),
      I3 => p_0_in(0),
      O => SR(0)
    );
\burst_length[7]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00000020"
    )
        port map (
      I0 => \burst_length[7]_i_2_n_0\,
      I1 => tck_reset,
      I2 => reg_addr(2),
      I3 => reg_addr(4),
      I4 => reg_addr(3),
      O => tck_reset_reg(0)
    );
\burst_length[7]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"02"
    )
        port map (
      I0 => reg_write,
      I1 => reg_addr(0),
      I2 => reg_addr(1),
      O => \burst_length[7]_i_2_n_0\
    );
\command_reg[5]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tdi_shift_reg(30),
      I1 => \state_reg__0\(1),
      O => command_reg(5)
    );
\command_reg[6]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tdi_shift_reg(31),
      I1 => \state_reg__0\(1),
      O => command_reg(6)
    );
\command_reg_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => command_reg_1,
      D => command_reg(5),
      Q => p_0_in(0),
      R => \^jtag_reset\
    );
\command_reg_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => command_reg_1,
      D => command_reg(6),
      Q => p_0_in(1),
      R => \^jtag_reset\
    );
\command_reg_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => command_reg_1,
      D => command_reg(7),
      Q => p_0_in(2),
      R => \^jtag_reset\
    );
\counter[0]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => counter(0),
      I1 => \counter[7]_i_5_n_0\,
      O => \counter[0]_i_1_n_0\
    );
\counter[1]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"14"
    )
        port map (
      I0 => \counter[7]_i_5_n_0\,
      I1 => counter(0),
      I2 => counter(1),
      O => \counter[1]_i_1_n_0\
    );
\counter[2]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1540"
    )
        port map (
      I0 => \counter[7]_i_5_n_0\,
      I1 => counter(1),
      I2 => counter(0),
      I3 => counter(2),
      O => \counter[2]_i_1_n_0\
    );
\counter[3]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"15554000"
    )
        port map (
      I0 => \counter[7]_i_5_n_0\,
      I1 => counter(0),
      I2 => counter(1),
      I3 => counter(2),
      I4 => counter(3),
      O => \counter[3]_i_1_n_0\
    );
\counter[4]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"1555555540000000"
    )
        port map (
      I0 => \counter[7]_i_5_n_0\,
      I1 => counter(2),
      I2 => counter(1),
      I3 => counter(0),
      I4 => counter(3),
      I5 => counter(4),
      O => \counter[4]_i_1_n_0\
    );
\counter[5]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"41"
    )
        port map (
      I0 => \counter[7]_i_5_n_0\,
      I1 => \counter[7]_i_4_n_0\,
      I2 => counter(5),
      O => \counter[5]_i_1_n_0\
    );
\counter[6]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"5104"
    )
        port map (
      I0 => \counter[7]_i_5_n_0\,
      I1 => counter(5),
      I2 => \counter[7]_i_4_n_0\,
      I3 => counter(6),
      O => \counter[6]_i_1_n_0\
    );
\counter[7]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"0000BF40"
    )
        port map (
      I0 => \counter[7]_i_4_n_0\,
      I1 => counter(5),
      I2 => counter(6),
      I3 => counter(7),
      I4 => \counter[7]_i_5_n_0\,
      O => \counter[7]_i_2_n_0\
    );
\counter[7]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FDFF"
    )
        port map (
      I0 => counter(0),
      I1 => counter(1),
      I2 => \FSM_sequential_state_reg[3]_i_11_n_0\,
      I3 => counter(2),
      O => \counter[7]_i_3_n_0\
    );
\counter[7]_i_4\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFFFFFF"
    )
        port map (
      I0 => counter(3),
      I1 => counter(0),
      I2 => counter(1),
      I3 => counter(2),
      I4 => counter(4),
      O => \counter[7]_i_4_n_0\
    );
\counter[7]_i_5\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFF000CFFFF55FF"
    )
        port map (
      I0 => \FSM_sequential_state_reg[3]_i_10_n_0\,
      I1 => \reg_addr[4]_i_2_n_0\,
      I2 => \state_reg__0\(2),
      I3 => \state_reg__0\(0),
      I4 => \state_reg__0\(3),
      I5 => \state_reg__0\(1),
      O => \counter[7]_i_5_n_0\
    );
\counter_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => counter_0,
      D => \counter[0]_i_1_n_0\,
      Q => counter(0),
      R => \^jtag_reset\
    );
\counter_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => counter_0,
      D => \counter[1]_i_1_n_0\,
      Q => counter(1),
      R => \^jtag_reset\
    );
\counter_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => counter_0,
      D => \counter[2]_i_1_n_0\,
      Q => counter(2),
      R => \^jtag_reset\
    );
\counter_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => counter_0,
      D => \counter[3]_i_1_n_0\,
      Q => counter(3),
      R => \^jtag_reset\
    );
\counter_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => counter_0,
      D => \counter[4]_i_1_n_0\,
      Q => counter(4),
      R => \^jtag_reset\
    );
\counter_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => counter_0,
      D => \counter[5]_i_1_n_0\,
      Q => counter(5),
      R => \^jtag_reset\
    );
\counter_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => counter_0,
      D => \counter[6]_i_1_n_0\,
      Q => counter(6),
      R => \^jtag_reset\
    );
\counter_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => counter_0,
      D => \counter[7]_i_2_n_0\,
      Q => counter(7),
      R => \^jtag_reset\
    );
\pad_len[0]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tdi_shift_reg(30),
      I1 => \state_reg__0\(0),
      O => \pad_len[0]_i_1_n_0\
    );
\pad_len[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tdi_shift_reg(31),
      I1 => \state_reg__0\(0),
      O => \pad_len[1]_i_1_n_0\
    );
\pad_len[2]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000100000000000"
    )
        port map (
      I0 => tdi_shift_reg(26),
      I1 => tdi_shift_reg(28),
      I2 => tdi_shift_reg(27),
      I3 => tdi_shift_reg(29),
      I4 => \state_reg__0\(2),
      I5 => tdi_shift_reg(25),
      O => \pad_len[2]_i_3_n_0\
    );
\pad_len_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => pad_len,
      D => \pad_len[0]_i_1_n_0\,
      Q => \pad_len_reg_n_0_[0]\,
      R => \^jtag_reset\
    );
\pad_len_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => pad_len,
      D => \pad_len[1]_i_1_n_0\,
      Q => \pad_len_reg_n_0_[1]\,
      R => \^jtag_reset\
    );
\pad_len_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => pad_len,
      D => u_jtag_n_16,
      Q => \pad_len_reg_n_0_[2]\,
      R => \^jtag_reset\
    );
ram_reg_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00000020"
    )
        port map (
      I0 => ram_reg,
      I1 => p_0_in(0),
      I2 => p_0_in(2),
      I3 => p_0_in(1),
      I4 => ram_reg_0(0),
      O => readram_rd
    );
\read_reg[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"4F444444"
    )
        port map (
      I0 => \read_reg[3]_i_2_n_0\,
      I1 => status(0),
      I2 => \state_reg__0\(3),
      I3 => \state_reg__0\(0),
      I4 => \read_reg_reg_n_0_[1]\,
      O => read_reg(0)
    );
\read_reg[16]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00FF00F9000000F9"
    )
        port map (
      I0 => reg_addr(0),
      I1 => reg_addr(1),
      I2 => \read_reg[1]_i_3_n_0\,
      I3 => \state_reg__0\(3),
      I4 => \state_reg__0\(0),
      I5 => \read_reg_reg_n_0_[17]\,
      O => \read_reg[16]_i_1_n_0\
    );
\read_reg[18]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00FF00F9000000F9"
    )
        port map (
      I0 => reg_addr(0),
      I1 => reg_addr(1),
      I2 => \read_reg[1]_i_3_n_0\,
      I3 => \state_reg__0\(3),
      I4 => \state_reg__0\(0),
      I5 => \read_reg_reg_n_0_[19]\,
      O => \read_reg[18]_i_1_n_0\
    );
\read_reg[19]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00FF00F9000000F9"
    )
        port map (
      I0 => reg_addr(0),
      I1 => reg_addr(1),
      I2 => \read_reg[1]_i_3_n_0\,
      I3 => \state_reg__0\(3),
      I4 => \state_reg__0\(0),
      I5 => \read_reg_reg_n_0_[20]\,
      O => \read_reg[19]_i_1_n_0\
    );
\read_reg[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"CCCCCCCCCCCCCFEC"
    )
        port map (
      I0 => status(1),
      I1 => \read_reg[1]_i_2_n_0\,
      I2 => reg_addr(0),
      I3 => reg_addr(1),
      I4 => \read_reg[28]_i_3_n_0\,
      I5 => \read_reg[1]_i_3_n_0\,
      O => read_reg(1)
    );
\read_reg[1]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"08"
    )
        port map (
      I0 => \read_reg_reg_n_0_[2]\,
      I1 => \state_reg__0\(0),
      I2 => \state_reg__0\(3),
      O => \read_reg[1]_i_2_n_0\
    );
\read_reg[1]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"FE"
    )
        port map (
      I0 => reg_addr(2),
      I1 => reg_addr(3),
      I2 => reg_addr(4),
      O => \read_reg[1]_i_3_n_0\
    );
\read_reg[22]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00FF00F9000000F9"
    )
        port map (
      I0 => reg_addr(0),
      I1 => reg_addr(1),
      I2 => \read_reg[1]_i_3_n_0\,
      I3 => \state_reg__0\(3),
      I4 => \state_reg__0\(0),
      I5 => \read_reg_reg_n_0_[23]\,
      O => \read_reg[22]_i_1_n_0\
    );
\read_reg[23]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00FF00F9000000F9"
    )
        port map (
      I0 => reg_addr(0),
      I1 => reg_addr(1),
      I2 => \read_reg[1]_i_3_n_0\,
      I3 => \state_reg__0\(3),
      I4 => \state_reg__0\(0),
      I5 => \read_reg_reg_n_0_[24]\,
      O => \read_reg[23]_i_1_n_0\
    );
\read_reg[24]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00FF00F9000000F9"
    )
        port map (
      I0 => reg_addr(0),
      I1 => reg_addr(1),
      I2 => \read_reg[1]_i_3_n_0\,
      I3 => \state_reg__0\(3),
      I4 => \state_reg__0\(0),
      I5 => \read_reg_reg_n_0_[25]\,
      O => \read_reg[24]_i_1_n_0\
    );
\read_reg[25]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00FF00F9000000F9"
    )
        port map (
      I0 => reg_addr(0),
      I1 => reg_addr(1),
      I2 => \read_reg[1]_i_3_n_0\,
      I3 => \state_reg__0\(3),
      I4 => \state_reg__0\(0),
      I5 => \read_reg_reg_n_0_[26]\,
      O => \read_reg[25]_i_1_n_0\
    );
\read_reg[26]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00FF00F9000000F9"
    )
        port map (
      I0 => reg_addr(0),
      I1 => reg_addr(1),
      I2 => \read_reg[1]_i_3_n_0\,
      I3 => \state_reg__0\(3),
      I4 => \state_reg__0\(0),
      I5 => \read_reg_reg_n_0_[27]\,
      O => \read_reg[26]_i_1_n_0\
    );
\read_reg[27]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00FF00F9000000F9"
    )
        port map (
      I0 => reg_addr(0),
      I1 => reg_addr(1),
      I2 => \read_reg[1]_i_3_n_0\,
      I3 => \state_reg__0\(3),
      I4 => \state_reg__0\(0),
      I5 => \read_reg_reg_n_0_[28]\,
      O => \read_reg[27]_i_1_n_0\
    );
\read_reg[28]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"5555555455545555"
    )
        port map (
      I0 => \read_reg[28]_i_3_n_0\,
      I1 => reg_addr(2),
      I2 => reg_addr(3),
      I3 => reg_addr(4),
      I4 => reg_addr(1),
      I5 => reg_addr(0),
      O => read_reg(28)
    );
\read_reg[28]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \state_reg__0\(3),
      I1 => \state_reg__0\(0),
      O => \read_reg[28]_i_3_n_0\
    );
\read_reg[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"1F111111"
    )
        port map (
      I0 => \read_reg[3]_i_2_n_0\,
      I1 => aresetn,
      I2 => \state_reg__0\(3),
      I3 => \state_reg__0\(0),
      I4 => \read_reg_reg_n_0_[3]\,
      O => read_reg(2)
    );
\read_reg[3]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"1F111111"
    )
        port map (
      I0 => \read_reg[3]_i_2_n_0\,
      I1 => Q(0),
      I2 => \state_reg__0\(3),
      I3 => \state_reg__0\(0),
      I4 => \read_reg_reg_n_0_[4]\,
      O => read_reg(3)
    );
\read_reg[3]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFFFFFFFEF"
    )
        port map (
      I0 => \read_reg[28]_i_3_n_0\,
      I1 => reg_addr(1),
      I2 => reg_addr(0),
      I3 => reg_addr(2),
      I4 => reg_addr(3),
      I5 => reg_addr(4),
      O => \read_reg[3]_i_2_n_0\
    );
\read_reg[5]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00FF00F9000000F9"
    )
        port map (
      I0 => reg_addr(0),
      I1 => reg_addr(1),
      I2 => \read_reg[1]_i_3_n_0\,
      I3 => \state_reg__0\(3),
      I4 => \state_reg__0\(0),
      I5 => \read_reg_reg_n_0_[6]\,
      O => \read_reg[5]_i_1_n_0\
    );
\read_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => read_reg(0),
      Q => \read_reg_reg_n_0_[0]\,
      R => '0'
    );
\read_reg_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[11]\,
      Q => \read_reg_reg_n_0_[10]\,
      R => u_jtag_n_22
    );
\read_reg_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[12]\,
      Q => \read_reg_reg_n_0_[11]\,
      R => u_jtag_n_22
    );
\read_reg_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[13]\,
      Q => \read_reg_reg_n_0_[12]\,
      R => u_jtag_n_22
    );
\read_reg_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[14]\,
      Q => \read_reg_reg_n_0_[13]\,
      R => u_jtag_n_22
    );
\read_reg_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[15]\,
      Q => \read_reg_reg_n_0_[14]\,
      R => u_jtag_n_22
    );
\read_reg_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[16]\,
      Q => \read_reg_reg_n_0_[15]\,
      R => u_jtag_n_22
    );
\read_reg_reg[16]\: unisim.vcomponents.FDSE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg[16]_i_1_n_0\,
      Q => \read_reg_reg_n_0_[16]\,
      S => '0'
    );
\read_reg_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[18]\,
      Q => \read_reg_reg_n_0_[17]\,
      R => u_jtag_n_22
    );
\read_reg_reg[18]\: unisim.vcomponents.FDSE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg[18]_i_1_n_0\,
      Q => \read_reg_reg_n_0_[18]\,
      S => '0'
    );
\read_reg_reg[19]\: unisim.vcomponents.FDSE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg[19]_i_1_n_0\,
      Q => \read_reg_reg_n_0_[19]\,
      S => '0'
    );
\read_reg_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => read_reg(1),
      Q => \read_reg_reg_n_0_[1]\,
      R => '0'
    );
\read_reg_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[21]\,
      Q => \read_reg_reg_n_0_[20]\,
      R => u_jtag_n_22
    );
\read_reg_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[22]\,
      Q => \read_reg_reg_n_0_[21]\,
      R => u_jtag_n_22
    );
\read_reg_reg[22]\: unisim.vcomponents.FDSE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg[22]_i_1_n_0\,
      Q => \read_reg_reg_n_0_[22]\,
      S => '0'
    );
\read_reg_reg[23]\: unisim.vcomponents.FDSE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg[23]_i_1_n_0\,
      Q => \read_reg_reg_n_0_[23]\,
      S => '0'
    );
\read_reg_reg[24]\: unisim.vcomponents.FDSE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg[24]_i_1_n_0\,
      Q => \read_reg_reg_n_0_[24]\,
      S => '0'
    );
\read_reg_reg[25]\: unisim.vcomponents.FDSE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg[25]_i_1_n_0\,
      Q => \read_reg_reg_n_0_[25]\,
      S => '0'
    );
\read_reg_reg[26]\: unisim.vcomponents.FDSE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg[26]_i_1_n_0\,
      Q => \read_reg_reg_n_0_[26]\,
      S => '0'
    );
\read_reg_reg[27]\: unisim.vcomponents.FDSE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg[27]_i_1_n_0\,
      Q => \read_reg_reg_n_0_[27]\,
      S => '0'
    );
\read_reg_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => read_reg(28),
      Q => \read_reg_reg_n_0_[28]\,
      R => '0'
    );
\read_reg_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => read_reg(2),
      Q => \read_reg_reg_n_0_[2]\,
      R => '0'
    );
\read_reg_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => read_reg(3),
      Q => \read_reg_reg_n_0_[3]\,
      R => '0'
    );
\read_reg_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[5]\,
      Q => \read_reg_reg_n_0_[4]\,
      R => u_jtag_n_22
    );
\read_reg_reg[5]\: unisim.vcomponents.FDSE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg[5]_i_1_n_0\,
      Q => \read_reg_reg_n_0_[5]\,
      S => '0'
    );
\read_reg_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[7]\,
      Q => \read_reg_reg_n_0_[6]\,
      R => u_jtag_n_22
    );
\read_reg_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[8]\,
      Q => \read_reg_reg_n_0_[7]\,
      R => u_jtag_n_22
    );
\read_reg_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[9]\,
      Q => \read_reg_reg_n_0_[8]\,
      R => u_jtag_n_22
    );
\read_reg_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_7,
      D => \read_reg_reg_n_0_[10]\,
      Q => \read_reg_reg_n_0_[9]\,
      R => u_jtag_n_22
    );
\readram_raddr[7]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => \^shift_out_en\,
      I1 => \readram_raddr_reg[0]\,
      O => shift_out_en_reg_1(0)
    );
\reg_addr[4]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"4000"
    )
        port map (
      I0 => \FSM_sequential_state_reg[3]_i_11_n_0\,
      I1 => counter(2),
      I2 => counter(1),
      I3 => counter(0),
      O => \reg_addr[4]_i_2_n_0\
    );
\reg_addr_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_9,
      D => tdi_shift_reg(25),
      Q => reg_addr(0),
      R => '0'
    );
\reg_addr_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_9,
      D => tdi_shift_reg(26),
      Q => reg_addr(1),
      R => '0'
    );
\reg_addr_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_9,
      D => tdi_shift_reg(27),
      Q => reg_addr(2),
      R => '0'
    );
\reg_addr_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_9,
      D => tdi_shift_reg(28),
      Q => reg_addr(3),
      R => '0'
    );
\reg_addr_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_9,
      D => tdi_shift_reg(29),
      Q => reg_addr(4),
      R => '0'
    );
\reg_wdata_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(0),
      Q => \^reg_wdata_reg[31]_0\(0),
      R => '0'
    );
\reg_wdata_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(10),
      Q => \^reg_wdata_reg[31]_0\(10),
      R => '0'
    );
\reg_wdata_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(11),
      Q => \^reg_wdata_reg[31]_0\(11),
      R => '0'
    );
\reg_wdata_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(12),
      Q => \^reg_wdata_reg[31]_0\(12),
      R => '0'
    );
\reg_wdata_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(13),
      Q => \^reg_wdata_reg[31]_0\(13),
      R => '0'
    );
\reg_wdata_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(14),
      Q => \^reg_wdata_reg[31]_0\(14),
      R => '0'
    );
\reg_wdata_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(15),
      Q => \^reg_wdata_reg[31]_0\(15),
      R => '0'
    );
\reg_wdata_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(16),
      Q => \^reg_wdata_reg[31]_0\(16),
      R => '0'
    );
\reg_wdata_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(17),
      Q => \^reg_wdata_reg[31]_0\(17),
      R => '0'
    );
\reg_wdata_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(18),
      Q => \^reg_wdata_reg[31]_0\(18),
      R => '0'
    );
\reg_wdata_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(19),
      Q => \^reg_wdata_reg[31]_0\(19),
      R => '0'
    );
\reg_wdata_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(1),
      Q => \^reg_wdata_reg[31]_0\(1),
      R => '0'
    );
\reg_wdata_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(20),
      Q => \^reg_wdata_reg[31]_0\(20),
      R => '0'
    );
\reg_wdata_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(21),
      Q => \^reg_wdata_reg[31]_0\(21),
      R => '0'
    );
\reg_wdata_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(22),
      Q => \^reg_wdata_reg[31]_0\(22),
      R => '0'
    );
\reg_wdata_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(23),
      Q => \^reg_wdata_reg[31]_0\(23),
      R => '0'
    );
\reg_wdata_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg(25),
      Q => \^reg_wdata_reg[31]_0\(24),
      R => '0'
    );
\reg_wdata_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg(26),
      Q => \^reg_wdata_reg[31]_0\(25),
      R => '0'
    );
\reg_wdata_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg(27),
      Q => \^reg_wdata_reg[31]_0\(26),
      R => '0'
    );
\reg_wdata_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg(28),
      Q => \^reg_wdata_reg[31]_0\(27),
      R => '0'
    );
\reg_wdata_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg(29),
      Q => \^reg_wdata_reg[31]_0\(28),
      R => '0'
    );
\reg_wdata_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg(30),
      Q => \^reg_wdata_reg[31]_0\(29),
      R => '0'
    );
\reg_wdata_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(2),
      Q => \^reg_wdata_reg[31]_0\(2),
      R => '0'
    );
\reg_wdata_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg(31),
      Q => \^reg_wdata_reg[31]_0\(30),
      R => '0'
    );
\reg_wdata_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi,
      Q => \^reg_wdata_reg[31]_0\(31),
      R => '0'
    );
\reg_wdata_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(3),
      Q => \^reg_wdata_reg[31]_0\(3),
      R => '0'
    );
\reg_wdata_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(4),
      Q => \^reg_wdata_reg[31]_0\(4),
      R => '0'
    );
\reg_wdata_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(5),
      Q => \^reg_wdata_reg[31]_0\(5),
      R => '0'
    );
\reg_wdata_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(6),
      Q => \^reg_wdata_reg[31]_0\(6),
      R => '0'
    );
\reg_wdata_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(7),
      Q => \^reg_wdata_reg[31]_0\(7),
      R => '0'
    );
\reg_wdata_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(8),
      Q => \^reg_wdata_reg[31]_0\(8),
      R => '0'
    );
\reg_wdata_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => u_jtag_n_6,
      D => tdi_shift_reg_next(9),
      Q => \^reg_wdata_reg[31]_0\(9),
      R => '0'
    );
reg_write_i_3: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \state_reg__0\(0),
      I1 => \state_reg__0\(2),
      O => reg_write_i_3_n_0
    );
reg_write_reg: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => '1',
      D => u_jtag_n_18,
      Q => reg_write,
      R => \^jtag_reset\
    );
shift_in_data_i_2: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0010"
    )
        port map (
      I0 => \state_reg__0\(2),
      I1 => \state_reg__0\(0),
      I2 => \state_reg__0\(3),
      I3 => \state_reg__0\(1),
      O => shift_in_data_i_2_n_0
    );
shift_in_data_reg: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => '1',
      D => u_jtag_n_21,
      Q => \^d\(0),
      R => '0'
    );
shift_in_en_reg: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => '1',
      D => u_jtag_n_20,
      Q => \^shift_in_en\,
      R => \^jtag_reset\
    );
\shift_in_reg[63]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00000080"
    )
        port map (
      I0 => \^shift_in_en\,
      I1 => p_0_in(0),
      I2 => p_0_in(2),
      I3 => p_0_in(1),
      I4 => tck_reset,
      O => shift_in_en_reg_0(0)
    );
shift_in_state_d1_i_2: unisim.vcomponents.LUT3
    generic map(
      INIT => X"08"
    )
        port map (
      I0 => p_0_in(0),
      I1 => p_0_in(2),
      I2 => p_0_in(1),
      O => shift_in_state
    );
shift_out_en_reg: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => '1',
      D => u_jtag_n_19,
      Q => \^shift_out_en\,
      R => \^jtag_reset\
    );
\shift_reg[63]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00000020"
    )
        port map (
      I0 => \^shift_out_en\,
      I1 => p_0_in(0),
      I2 => p_0_in(2),
      I3 => p_0_in(1),
      I4 => tck_reset,
      O => shift_out_en_reg_0(0)
    );
start_read_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"A3A0"
    )
        port map (
      I0 => \^reg_wdata_reg[31]_0\(0),
      I1 => trigger_tck_d1,
      I2 => start_read_i_2_n_0,
      I3 => start_read_reg,
      O => \reg_wdata_reg[0]_0\
    );
start_read_i_2: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000010"
    )
        port map (
      I0 => reg_addr(1),
      I1 => reg_addr(0),
      I2 => reg_write,
      I3 => reg_addr(4),
      I4 => reg_addr(3),
      I5 => reg_addr(2),
      O => start_read_i_2_n_0
    );
start_write_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"EFFF8AAA"
    )
        port map (
      I0 => shift_in_state_d1,
      I1 => p_0_in(1),
      I2 => p_0_in(2),
      I3 => p_0_in(0),
      I4 => start_write,
      O => shift_in_state_d1_reg
    );
\tdi_shift_reg_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(10),
      Q => tdi_shift_reg_next(9),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(11),
      Q => tdi_shift_reg_next(10),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(12),
      Q => tdi_shift_reg_next(11),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(13),
      Q => tdi_shift_reg_next(12),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(14),
      Q => tdi_shift_reg_next(13),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(15),
      Q => tdi_shift_reg_next(14),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(16),
      Q => tdi_shift_reg_next(15),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(17),
      Q => tdi_shift_reg_next(16),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(18),
      Q => tdi_shift_reg_next(17),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(19),
      Q => tdi_shift_reg_next(18),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(1),
      Q => tdi_shift_reg_next(0),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(20),
      Q => tdi_shift_reg_next(19),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(21),
      Q => tdi_shift_reg_next(20),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(22),
      Q => tdi_shift_reg_next(21),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(23),
      Q => tdi_shift_reg_next(22),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg(25),
      Q => tdi_shift_reg_next(23),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg(26),
      Q => tdi_shift_reg(25),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg(27),
      Q => tdi_shift_reg(26),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg(28),
      Q => tdi_shift_reg(27),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg(29),
      Q => tdi_shift_reg(28),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg(30),
      Q => tdi_shift_reg(29),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(2),
      Q => tdi_shift_reg_next(1),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg(31),
      Q => tdi_shift_reg(30),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi,
      Q => tdi_shift_reg(31),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(3),
      Q => tdi_shift_reg_next(2),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(4),
      Q => tdi_shift_reg_next(3),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(5),
      Q => tdi_shift_reg_next(4),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(6),
      Q => tdi_shift_reg_next(5),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(7),
      Q => tdi_shift_reg_next(6),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(8),
      Q => tdi_shift_reg_next(7),
      R => u_jtag_n_15
    );
\tdi_shift_reg_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => \^tck\,
      CE => shift_dr,
      D => tdi_shift_reg_next(9),
      Q => tdi_shift_reg_next(8),
      R => u_jtag_n_15
    );
u_jtag: entity work.BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_vendor_ip1
     port map (
      CLK => \^tck\,
      D(3 downto 0) => \state_reg__1\(3 downto 0),
      E(0) => pad_len,
      \FSM_sequential_state_reg_reg[0]\ => u_jtag_n_7,
      \FSM_sequential_state_reg_reg[0]_0\(0) => counter_0,
      \FSM_sequential_state_reg_reg[0]_1\ => u_jtag_n_22,
      \FSM_sequential_state_reg_reg[0]_2\ => \FSM_sequential_state_reg[3]_i_5_n_0\,
      \FSM_sequential_state_reg_reg[0]_3\ => \FSM_sequential_state_reg[3]_i_10_n_0\,
      \FSM_sequential_state_reg_reg[0]_4\ => \FSM_sequential_state_reg[0]_i_2_n_0\,
      \FSM_sequential_state_reg_reg[1]\(0) => command_reg_1,
      \FSM_sequential_state_reg_reg[1]_0\ => \FSM_sequential_state_reg[1]_i_2_n_0\,
      \FSM_sequential_state_reg_reg[2]\(0) => u_jtag_n_5,
      \FSM_sequential_state_reg_reg[2]_0\(0) => u_jtag_n_6,
      \FSM_sequential_state_reg_reg[2]_1\ => u_jtag_n_20,
      \FSM_sequential_state_reg_reg[2]_2\ => \FSM_sequential_state_reg[2]_i_2_n_0\,
      \FSM_sequential_state_reg_reg[3]\ => u_jtag_n_18,
      Q(3 downto 0) => \state_reg__0\(3 downto 0),
      SR(0) => \^jtag_reset\,
      \counter_reg[0]\ => \counter[7]_i_3_n_0\,
      \pad_len_reg[0]\ => \pad_len[2]_i_3_n_0\,
      \read_reg_reg[5]\ => \FSM_sequential_state_reg[3]_i_9_n_0\,
      \reg_addr_reg[0]\ => \reg_addr[4]_i_2_n_0\,
      \reg_addr_reg[0]_0\(1 downto 0) => tdi_shift_reg(31 downto 30),
      \reg_addr_reg[0]_1\ => \FSM_sequential_state_reg[2]_i_3_n_0\,
      \reg_wdata_reg[0]\ => \counter[7]_i_4_n_0\,
      \reg_wdata_reg[0]_0\(2 downto 0) => counter(7 downto 5),
      reg_write => reg_write,
      reg_write_reg => reg_write_i_3_n_0,
      shift_dr => shift_dr,
      shift_in_data_reg => shift_in_data_i_2_n_0,
      shift_in_data_reg_0(0) => \^d\(0),
      shift_in_en_reg => \^shift_in_en\,
      shift_out_en_reg => \^shift_out_en\,
      tdi => tdi,
      \tdi_shift_reg_reg[31]\(0) => u_jtag_n_9,
      u_BSCANE2_0(0) => u_jtag_n_15,
      u_BSCANE2_1(0) => u_jtag_n_16,
      u_BSCANE2_2(0) => command_reg(7),
      u_BSCANE2_3 => u_jtag_n_19,
      u_BSCANE2_4 => u_jtag_n_21,
      u_BSCANE2_5(0) => u_BSCANE2(0),
      u_BSCANE2_6(2 downto 0) => p_0_in(2 downto 0),
      u_BSCANE2_7 => \read_reg_reg_n_0_[0]\
    );
\write_count[0]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"EFFF"
    )
        port map (
      I0 => tck_reset,
      I1 => p_0_in(1),
      I2 => p_0_in(2),
      I3 => p_0_in(0),
      O => \^tck_reset_reg_0\
    );
writeram_wr_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"01000000"
    )
        port map (
      I0 => \^tck_reset_reg_0\,
      I1 => writeram_wr_i_2_n_0,
      I2 => S(1),
      I3 => writeram_wr_reg(3),
      I4 => writeram_wr_reg(1),
      O => \write_count_reg[15]\
    );
writeram_wr_i_2: unisim.vcomponents.LUT6
    generic map(
      INIT => X"DFFFFFFFFFFFFFFF"
    )
        port map (
      I0 => \^shift_in_en\,
      I1 => S(0),
      I2 => writeram_wr_reg_0,
      I3 => writeram_wr_reg(0),
      I4 => writeram_wr_reg(2),
      I5 => writeram_wr_reg(4),
      O => writeram_wr_i_2_n_0
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager is
  port (
    axi4m_awaddr : out STD_LOGIC_VECTOR ( 31 downto 0 );
    axi4m_awprot : out STD_LOGIC_VECTOR ( 2 downto 0 );
    axi4m_awvalid : out STD_LOGIC;
    axi4m_awready : in STD_LOGIC;
    axi4m_wdata : out STD_LOGIC_VECTOR ( 63 downto 0 );
    axi4m_wvalid : out STD_LOGIC;
    axi4m_wready : in STD_LOGIC;
    axi4m_wstrb : out STD_LOGIC_VECTOR ( 7 downto 0 );
    axi4m_awid : out STD_LOGIC_VECTOR ( 0 to 0 );
    axi4m_awlen : out STD_LOGIC_VECTOR ( 7 downto 0 );
    axi4m_awsize : out STD_LOGIC_VECTOR ( 2 downto 0 );
    axi4m_awburst : out STD_LOGIC_VECTOR ( 1 downto 0 );
    axi4m_awlock : out STD_LOGIC;
    axi4m_awcache : out STD_LOGIC_VECTOR ( 3 downto 0 );
    axi4m_wlast : out STD_LOGIC;
    axi4m_awqos : out STD_LOGIC_VECTOR ( 3 downto 0 );
    axi4m_bresp : in STD_LOGIC_VECTOR ( 1 downto 0 );
    axi4m_bvalid : in STD_LOGIC;
    axi4m_bid : in STD_LOGIC_VECTOR ( 0 to 0 );
    axi4m_bready : out STD_LOGIC;
    axi4m_araddr : out STD_LOGIC_VECTOR ( 31 downto 0 );
    axi4m_arprot : out STD_LOGIC_VECTOR ( 2 downto 0 );
    axi4m_arvalid : out STD_LOGIC;
    axi4m_arready : in STD_LOGIC;
    axi4m_arid : out STD_LOGIC_VECTOR ( 0 to 0 );
    axi4m_arlen : out STD_LOGIC_VECTOR ( 7 downto 0 );
    axi4m_arsize : out STD_LOGIC_VECTOR ( 2 downto 0 );
    axi4m_arburst : out STD_LOGIC_VECTOR ( 1 downto 0 );
    axi4m_arlock : out STD_LOGIC;
    axi4m_arcache : out STD_LOGIC_VECTOR ( 3 downto 0 );
    axi4m_arqos : out STD_LOGIC_VECTOR ( 3 downto 0 );
    axi4m_rdata : in STD_LOGIC_VECTOR ( 63 downto 0 );
    axi4m_rlast : in STD_LOGIC;
    axi4m_rresp : in STD_LOGIC_VECTOR ( 1 downto 0 );
    axi4m_rvalid : in STD_LOGIC;
    axi4m_rready : out STD_LOGIC;
    axi4m_rid : in STD_LOGIC_VECTOR ( 0 to 0 );
    aclk : in STD_LOGIC;
    aresetn : in STD_LOGIC
  );
  attribute APP_TYPE : string;
  attribute APP_TYPE of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is "4'b0000";
  attribute AXIMATER_IP_CONFIG : string;
  attribute AXIMATER_IP_CONFIG of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is "32'b00000000000000000000000000000010";
  attribute AXI_ADDR_WIDTH : integer;
  attribute AXI_ADDR_WIDTH of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is 32;
  attribute AXI_DATA_WIDTH : integer;
  attribute AXI_DATA_WIDTH of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is 64;
  attribute AXImaster_ADDRS_64 : string;
  attribute AXImaster_ADDRS_64 of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is "1'b0";
  attribute AXImaster_DATA_64 : string;
  attribute AXImaster_DATA_64 of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is "1'b1";
  attribute AXImaster_IP_ID0 : string;
  attribute AXImaster_IP_ID0 of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is "32'b00011111110011010000000000100000";
  attribute ID_WIDTH : integer;
  attribute ID_WIDTH of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is 1;
  attribute JTAG_ID : integer;
  attribute JTAG_ID of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is 3;
  attribute MAJOR_VER : string;
  attribute MAJOR_VER of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is "8'b00000010";
  attribute MINOR_VER : string;
  attribute MINOR_VER of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is "4'b0000";
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is "hdlverifier_axi_manager";
  attribute SIGNATURE : string;
  attribute SIGNATURE of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is "16'b0001111111001101";
  attribute log2AXI_DATA_WIDTH : integer;
  attribute log2AXI_DATA_WIDTH of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager : entity is 6;
end BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager;

architecture STRUCTURE of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager is
  signal \<const0>\ : STD_LOGIC;
  signal \aclk_cnt[3]_i_1_n_0\ : STD_LOGIC;
  signal aclk_cnt_reg : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal aclk_reset : STD_LOGIC;
  signal aclk_reset0 : STD_LOGIC;
  signal address : STD_LOGIC;
  signal \address_reg_n_0_[0]\ : STD_LOGIC;
  signal \address_reg_n_0_[10]\ : STD_LOGIC;
  signal \address_reg_n_0_[11]\ : STD_LOGIC;
  signal \address_reg_n_0_[12]\ : STD_LOGIC;
  signal \address_reg_n_0_[13]\ : STD_LOGIC;
  signal \address_reg_n_0_[14]\ : STD_LOGIC;
  signal \address_reg_n_0_[15]\ : STD_LOGIC;
  signal \address_reg_n_0_[16]\ : STD_LOGIC;
  signal \address_reg_n_0_[17]\ : STD_LOGIC;
  signal \address_reg_n_0_[18]\ : STD_LOGIC;
  signal \address_reg_n_0_[19]\ : STD_LOGIC;
  signal \address_reg_n_0_[1]\ : STD_LOGIC;
  signal \address_reg_n_0_[20]\ : STD_LOGIC;
  signal \address_reg_n_0_[21]\ : STD_LOGIC;
  signal \address_reg_n_0_[22]\ : STD_LOGIC;
  signal \address_reg_n_0_[23]\ : STD_LOGIC;
  signal \address_reg_n_0_[24]\ : STD_LOGIC;
  signal \address_reg_n_0_[25]\ : STD_LOGIC;
  signal \address_reg_n_0_[26]\ : STD_LOGIC;
  signal \address_reg_n_0_[27]\ : STD_LOGIC;
  signal \address_reg_n_0_[28]\ : STD_LOGIC;
  signal \address_reg_n_0_[29]\ : STD_LOGIC;
  signal \address_reg_n_0_[2]\ : STD_LOGIC;
  signal \address_reg_n_0_[30]\ : STD_LOGIC;
  signal \address_reg_n_0_[31]\ : STD_LOGIC;
  signal \address_reg_n_0_[3]\ : STD_LOGIC;
  signal \address_reg_n_0_[4]\ : STD_LOGIC;
  signal \address_reg_n_0_[5]\ : STD_LOGIC;
  signal \address_reg_n_0_[6]\ : STD_LOGIC;
  signal \address_reg_n_0_[7]\ : STD_LOGIC;
  signal \address_reg_n_0_[8]\ : STD_LOGIC;
  signal \address_reg_n_0_[9]\ : STD_LOGIC;
  signal \^axi4m_araddr\ : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal \^axi4m_arburst\ : STD_LOGIC_VECTOR ( 1 downto 0 );
  signal \^axi4m_arvalid\ : STD_LOGIC;
  signal axi4m_arvalid0 : STD_LOGIC;
  signal axi4m_arvalid_i_1_n_0 : STD_LOGIC;
  signal \^axi4m_awlen\ : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal \^axi4m_awvalid\ : STD_LOGIC;
  signal axi4m_awvalid_i_1_n_0 : STD_LOGIC;
  signal \^axi4m_rready\ : STD_LOGIC;
  signal axi4m_rready_i_1_n_0 : STD_LOGIC;
  signal axi4m_wlast_INST_0_i_1_n_0 : STD_LOGIC;
  signal axi4m_wlast_INST_0_i_2_n_0 : STD_LOGIC;
  signal axi4m_wlast_INST_0_i_3_n_0 : STD_LOGIC;
  signal axi4m_wlast_INST_0_i_4_n_0 : STD_LOGIC;
  signal \^axi4m_wvalid\ : STD_LOGIC;
  signal axi4m_wvalid_i_1_n_0 : STD_LOGIC;
  signal axi4m_wvalid_i_2_n_0 : STD_LOGIC;
  signal \bitcount[0]_i_1_n_0\ : STD_LOGIC;
  signal bitcount_reg : STD_LOGIC_VECTOR ( 5 downto 0 );
  signal burst_length : STD_LOGIC;
  signal \burst_length_reg_n_0_[0]\ : STD_LOGIC;
  signal \burst_length_reg_n_0_[1]\ : STD_LOGIC;
  signal \burst_length_reg_n_0_[2]\ : STD_LOGIC;
  signal \burst_length_reg_n_0_[3]\ : STD_LOGIC;
  signal \burst_length_reg_n_0_[4]\ : STD_LOGIC;
  signal \burst_length_reg_n_0_[5]\ : STD_LOGIC;
  signal \burst_length_reg_n_0_[6]\ : STD_LOGIC;
  signal \burst_length_reg_n_0_[7]\ : STD_LOGIC;
  signal burst_type : STD_LOGIC_VECTOR ( 1 downto 0 );
  signal data1 : STD_LOGIC_VECTOR ( 62 downto 0 );
  signal jtag_reset : STD_LOGIC;
  signal jtag_reset_d1 : STD_LOGIC;
  signal p_0_in : STD_LOGIC_VECTOR ( 1 downto 0 );
  signal p_0_in11_in : STD_LOGIC;
  signal \p_0_in__0\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \p_0_in__1\ : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal \p_0_in__2\ : STD_LOGIC_VECTOR ( 5 downto 1 );
  signal \p_0_in__3\ : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal read_count : STD_LOGIC;
  signal \read_count[0]_i_1_n_0\ : STD_LOGIC;
  signal \read_count[0]_i_4_n_0\ : STD_LOGIC;
  signal read_count_reg : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal \read_count_reg[0]_i_3_n_1\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_10\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_11\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_12\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_13\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_14\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_15\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_2\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_3\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_4\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_5\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_6\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_7\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_8\ : STD_LOGIC;
  signal \read_count_reg[0]_i_3_n_9\ : STD_LOGIC;
  signal readram_n_64 : STD_LOGIC;
  signal readram_raddr : STD_LOGIC;
  signal \readram_raddr[7]_i_3_n_0\ : STD_LOGIC;
  signal readram_raddr_reg : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal readram_rd : STD_LOGIC;
  signal sel : STD_LOGIC;
  signal shift_in_data : STD_LOGIC;
  signal shift_in_en : STD_LOGIC;
  signal shift_in_reg : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[0]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[10]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[11]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[12]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[13]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[14]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[15]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[16]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[17]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[18]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[19]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[1]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[20]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[21]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[22]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[23]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[24]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[25]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[26]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[27]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[28]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[29]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[2]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[30]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[31]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[32]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[33]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[34]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[35]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[36]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[37]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[38]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[39]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[3]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[40]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[41]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[42]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[43]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[44]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[45]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[46]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[47]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[48]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[49]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[4]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[50]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[51]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[52]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[53]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[54]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[55]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[56]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[57]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[58]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[59]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[5]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[60]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[61]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[62]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[63]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[6]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[7]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[8]\ : STD_LOGIC;
  signal \shift_in_reg_reg_n_0_[9]\ : STD_LOGIC;
  signal shift_in_state : STD_LOGIC;
  signal shift_in_state_d1 : STD_LOGIC;
  signal shift_in_state_d1_i_1_n_0 : STD_LOGIC;
  signal shift_out_en : STD_LOGIC;
  signal shift_reg : STD_LOGIC_VECTOR ( 63 downto 0 );
  signal \shift_reg[63]_i_3_n_0\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[0]\ : STD_LOGIC;
  signal start_read_aclk : STD_LOGIC;
  signal start_read_reg_n_0 : STD_LOGIC;
  signal start_write : STD_LOGIC;
  signal start_write_d1 : STD_LOGIC;
  signal start_write_d2 : STD_LOGIC;
  signal state_write : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \state_write[0]_i_1_n_0\ : STD_LOGIC;
  signal \state_write[0]_i_2_n_0\ : STD_LOGIC;
  signal \state_write[1]_i_1_n_0\ : STD_LOGIC;
  signal \state_write[2]_i_2_n_0\ : STD_LOGIC;
  signal \state_write[2]_i_3_n_0\ : STD_LOGIC;
  signal \state_write_reg_n_0_[0]\ : STD_LOGIC;
  signal \state_write_reg_n_0_[1]\ : STD_LOGIC;
  signal \state_write_reg_n_0_[2]\ : STD_LOGIC;
  signal status : STD_LOGIC_VECTOR ( 1 downto 0 );
  signal \status[0]_i_1_n_0\ : STD_LOGIC;
  signal \status[1]_i_1_n_0\ : STD_LOGIC;
  signal tck : STD_LOGIC;
  signal tck_reset : STD_LOGIC;
  signal tmp1 : STD_LOGIC;
  signal tmp2 : STD_LOGIC;
  signal tmp3 : STD_LOGIC;
  signal tmp4 : STD_LOGIC;
  signal trigger : STD_LOGIC;
  signal trigger_d1 : STD_LOGIC;
  signal trigger_tck : STD_LOGIC;
  signal trigger_tck_d1 : STD_LOGIC;
  signal u_jtag_n_10 : STD_LOGIC;
  signal u_jtag_n_14 : STD_LOGIC;
  signal u_jtag_n_15 : STD_LOGIC;
  signal u_jtag_n_16 : STD_LOGIC;
  signal u_jtag_n_17 : STD_LOGIC;
  signal u_jtag_n_18 : STD_LOGIC;
  signal u_jtag_n_19 : STD_LOGIC;
  signal u_jtag_n_20 : STD_LOGIC;
  signal u_jtag_n_21 : STD_LOGIC;
  signal u_jtag_n_22 : STD_LOGIC;
  signal u_jtag_n_23 : STD_LOGIC;
  signal u_jtag_n_24 : STD_LOGIC;
  signal u_jtag_n_25 : STD_LOGIC;
  signal u_jtag_n_26 : STD_LOGIC;
  signal u_jtag_n_27 : STD_LOGIC;
  signal u_jtag_n_28 : STD_LOGIC;
  signal u_jtag_n_29 : STD_LOGIC;
  signal u_jtag_n_30 : STD_LOGIC;
  signal u_jtag_n_31 : STD_LOGIC;
  signal u_jtag_n_32 : STD_LOGIC;
  signal u_jtag_n_33 : STD_LOGIC;
  signal u_jtag_n_34 : STD_LOGIC;
  signal u_jtag_n_35 : STD_LOGIC;
  signal u_jtag_n_36 : STD_LOGIC;
  signal u_jtag_n_37 : STD_LOGIC;
  signal u_jtag_n_40 : STD_LOGIC;
  signal u_jtag_n_41 : STD_LOGIC;
  signal u_jtag_n_42 : STD_LOGIC;
  signal u_jtag_n_43 : STD_LOGIC;
  signal u_jtag_n_44 : STD_LOGIC;
  signal u_jtag_n_45 : STD_LOGIC;
  signal u_jtag_n_46 : STD_LOGIC;
  signal u_jtag_n_48 : STD_LOGIC;
  signal u_jtag_n_8 : STD_LOGIC;
  signal u_jtag_n_9 : STD_LOGIC;
  signal \write_count[0]_i_3_n_0\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_0\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_1\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_10\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_11\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_12\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_13\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_14\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_15\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_2\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_3\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_4\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_5\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_6\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_7\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_8\ : STD_LOGIC;
  signal \write_count_reg[0]_i_2_n_9\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_1\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_10\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_11\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_12\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_13\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_14\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_15\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_2\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_3\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_4\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_5\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_6\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_7\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_8\ : STD_LOGIC;
  signal \write_count_reg[8]_i_1_n_9\ : STD_LOGIC;
  signal \write_count_reg_n_0_[0]\ : STD_LOGIC;
  signal \write_count_reg_n_0_[14]\ : STD_LOGIC;
  signal \write_count_reg_n_0_[15]\ : STD_LOGIC;
  signal \write_count_reg_n_0_[1]\ : STD_LOGIC;
  signal \write_count_reg_n_0_[2]\ : STD_LOGIC;
  signal \write_count_reg_n_0_[3]\ : STD_LOGIC;
  signal \write_count_reg_n_0_[4]\ : STD_LOGIC;
  signal \write_count_reg_n_0_[5]\ : STD_LOGIC;
  signal write_word_count : STD_LOGIC;
  signal \write_word_count[7]_i_4_n_0\ : STD_LOGIC;
  signal write_word_count_reg : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal writeram_n_71 : STD_LOGIC;
  signal writeram_raddr : STD_LOGIC_VECTOR ( 7 downto 1 );
  signal writeram_raddr1 : STD_LOGIC;
  signal writeram_raddr_reg0 : STD_LOGIC;
  signal writeram_raddr_reg_reg : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal writeram_rd : STD_LOGIC;
  signal writeram_rd_i_1_n_0 : STD_LOGIC;
  signal writeram_waddr : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal writeram_waddr_comb : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal writeram_wr : STD_LOGIC;
  signal \NLW_read_count_reg[0]_i_3_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 7 to 7 );
  signal \NLW_write_count_reg[8]_i_1_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 7 to 7 );
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \aclk_cnt[0]_i_1\ : label is "soft_lutpair70";
  attribute SOFT_HLUTNM of \aclk_cnt[1]_i_1\ : label is "soft_lutpair70";
  attribute SOFT_HLUTNM of \aclk_cnt[2]_i_1\ : label is "soft_lutpair64";
  attribute SOFT_HLUTNM of \aclk_cnt[3]_i_2\ : label is "soft_lutpair64";
  attribute SOFT_HLUTNM of axi4m_arvalid_i_1 : label is "soft_lutpair67";
  attribute SOFT_HLUTNM of axi4m_wlast_INST_0_i_3 : label is "soft_lutpair63";
  attribute SOFT_HLUTNM of \bitcount[1]_i_1\ : label is "soft_lutpair68";
  attribute SOFT_HLUTNM of \bitcount[2]_i_1\ : label is "soft_lutpair68";
  attribute SOFT_HLUTNM of \bitcount[3]_i_1\ : label is "soft_lutpair57";
  attribute SOFT_HLUTNM of \bitcount[4]_i_1\ : label is "soft_lutpair57";
  attribute SOFT_HLUTNM of \readram_raddr[0]_i_1\ : label is "soft_lutpair69";
  attribute SOFT_HLUTNM of \readram_raddr[1]_i_1\ : label is "soft_lutpair69";
  attribute SOFT_HLUTNM of \readram_raddr[2]_i_1\ : label is "soft_lutpair62";
  attribute SOFT_HLUTNM of \readram_raddr[3]_i_1\ : label is "soft_lutpair62";
  attribute SOFT_HLUTNM of \readram_raddr[4]_i_1\ : label is "soft_lutpair56";
  attribute SOFT_HLUTNM of \readram_raddr[6]_i_1\ : label is "soft_lutpair61";
  attribute SOFT_HLUTNM of \readram_raddr[7]_i_2\ : label is "soft_lutpair61";
  attribute SOFT_HLUTNM of \readram_raddr[7]_i_3\ : label is "soft_lutpair56";
  attribute SOFT_HLUTNM of \state_write[0]_i_2\ : label is "soft_lutpair58";
  attribute SOFT_HLUTNM of \state_write[0]_i_3\ : label is "soft_lutpair59";
  attribute SOFT_HLUTNM of \state_write[2]_i_3\ : label is "soft_lutpair58";
  attribute SOFT_HLUTNM of \status[0]_i_2\ : label is "soft_lutpair67";
  attribute SOFT_HLUTNM of \write_word_count[1]_i_1\ : label is "soft_lutpair65";
  attribute SOFT_HLUTNM of \write_word_count[2]_i_1\ : label is "soft_lutpair63";
  attribute SOFT_HLUTNM of \write_word_count[3]_i_1\ : label is "soft_lutpair65";
  attribute SOFT_HLUTNM of \write_word_count[4]_i_1\ : label is "soft_lutpair60";
  attribute SOFT_HLUTNM of \write_word_count[6]_i_1\ : label is "soft_lutpair66";
  attribute SOFT_HLUTNM of \write_word_count[7]_i_3\ : label is "soft_lutpair66";
  attribute SOFT_HLUTNM of \write_word_count[7]_i_4\ : label is "soft_lutpair60";
  attribute SOFT_HLUTNM of writeram_rd_i_1 : label is "soft_lutpair59";
begin
  axi4m_araddr(31 downto 0) <= \^axi4m_araddr\(31 downto 0);
  axi4m_arburst(1 downto 0) <= \^axi4m_arburst\(1 downto 0);
  axi4m_arcache(3) <= \<const0>\;
  axi4m_arcache(2) <= \<const0>\;
  axi4m_arcache(1) <= \<const0>\;
  axi4m_arcache(0) <= \<const0>\;
  axi4m_arid(0) <= \<const0>\;
  axi4m_arlen(7 downto 0) <= \^axi4m_awlen\(7 downto 0);
  axi4m_arlock <= \<const0>\;
  axi4m_arprot(2) <= \<const0>\;
  axi4m_arprot(1) <= \<const0>\;
  axi4m_arprot(0) <= \<const0>\;
  axi4m_arqos(3) <= \<const0>\;
  axi4m_arqos(2) <= \<const0>\;
  axi4m_arqos(1) <= \<const0>\;
  axi4m_arqos(0) <= \<const0>\;
  axi4m_arsize(2) <= \<const0>\;
  axi4m_arsize(1) <= \<const0>\;
  axi4m_arsize(0) <= \<const0>\;
  axi4m_arvalid <= \^axi4m_arvalid\;
  axi4m_awaddr(31 downto 0) <= \^axi4m_araddr\(31 downto 0);
  axi4m_awburst(1 downto 0) <= \^axi4m_arburst\(1 downto 0);
  axi4m_awcache(3) <= \<const0>\;
  axi4m_awcache(2) <= \<const0>\;
  axi4m_awcache(1) <= \<const0>\;
  axi4m_awcache(0) <= \<const0>\;
  axi4m_awid(0) <= \<const0>\;
  axi4m_awlen(7 downto 0) <= \^axi4m_awlen\(7 downto 0);
  axi4m_awlock <= \<const0>\;
  axi4m_awprot(2) <= \<const0>\;
  axi4m_awprot(1) <= \<const0>\;
  axi4m_awprot(0) <= \<const0>\;
  axi4m_awqos(3) <= \<const0>\;
  axi4m_awqos(2) <= \<const0>\;
  axi4m_awqos(1) <= \<const0>\;
  axi4m_awqos(0) <= \<const0>\;
  axi4m_awsize(2) <= \<const0>\;
  axi4m_awsize(1) <= \<const0>\;
  axi4m_awsize(0) <= \<const0>\;
  axi4m_awvalid <= \^axi4m_awvalid\;
  axi4m_bready <= \<const0>\;
  axi4m_rready <= \^axi4m_rready\;
  axi4m_wstrb(7) <= \<const0>\;
  axi4m_wstrb(6) <= \<const0>\;
  axi4m_wstrb(5) <= \<const0>\;
  axi4m_wstrb(4) <= \<const0>\;
  axi4m_wstrb(3) <= \<const0>\;
  axi4m_wstrb(2) <= \<const0>\;
  axi4m_wstrb(1) <= \<const0>\;
  axi4m_wstrb(0) <= \<const0>\;
  axi4m_wvalid <= \^axi4m_wvalid\;
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
\aclk_cnt[0]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => aclk_cnt_reg(0),
      O => \p_0_in__0\(0)
    );
\aclk_cnt[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => aclk_cnt_reg(1),
      I1 => aclk_cnt_reg(0),
      O => \p_0_in__0\(1)
    );
\aclk_cnt[2]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"6A"
    )
        port map (
      I0 => aclk_cnt_reg(2),
      I1 => aclk_cnt_reg(0),
      I2 => aclk_cnt_reg(1),
      O => \p_0_in__0\(2)
    );
\aclk_cnt[3]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"7FFF"
    )
        port map (
      I0 => aclk_cnt_reg(3),
      I1 => aclk_cnt_reg(1),
      I2 => aclk_cnt_reg(0),
      I3 => aclk_cnt_reg(2),
      O => \aclk_cnt[3]_i_1_n_0\
    );
\aclk_cnt[3]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6AAA"
    )
        port map (
      I0 => aclk_cnt_reg(3),
      I1 => aclk_cnt_reg(1),
      I2 => aclk_cnt_reg(0),
      I3 => aclk_cnt_reg(2),
      O => \p_0_in__0\(3)
    );
\aclk_cnt_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \aclk_cnt[3]_i_1_n_0\,
      CLR => tmp3,
      D => \p_0_in__0\(0),
      Q => aclk_cnt_reg(0)
    );
\aclk_cnt_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \aclk_cnt[3]_i_1_n_0\,
      CLR => tmp3,
      D => \p_0_in__0\(1),
      Q => aclk_cnt_reg(1)
    );
\aclk_cnt_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \aclk_cnt[3]_i_1_n_0\,
      CLR => tmp3,
      D => \p_0_in__0\(2),
      Q => aclk_cnt_reg(2)
    );
\aclk_cnt_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \aclk_cnt[3]_i_1_n_0\,
      CLR => tmp3,
      D => \p_0_in__0\(3),
      Q => aclk_cnt_reg(3)
    );
aclk_reset_i_1: unisim.vcomponents.LUT2
    generic map(
      INIT => X"B"
    )
        port map (
      I0 => tmp2,
      I1 => tmp4,
      O => aclk_reset0
    );
aclk_reset_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => aclk_reset0,
      Q => aclk_reset,
      R => '0'
    );
\address_aclk_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[0]\,
      Q => \^axi4m_araddr\(0),
      R => '0'
    );
\address_aclk_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[10]\,
      Q => \^axi4m_araddr\(10),
      R => '0'
    );
\address_aclk_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[11]\,
      Q => \^axi4m_araddr\(11),
      R => '0'
    );
\address_aclk_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[12]\,
      Q => \^axi4m_araddr\(12),
      R => '0'
    );
\address_aclk_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[13]\,
      Q => \^axi4m_araddr\(13),
      R => '0'
    );
\address_aclk_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[14]\,
      Q => \^axi4m_araddr\(14),
      R => '0'
    );
\address_aclk_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[15]\,
      Q => \^axi4m_araddr\(15),
      R => '0'
    );
\address_aclk_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[16]\,
      Q => \^axi4m_araddr\(16),
      R => '0'
    );
\address_aclk_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[17]\,
      Q => \^axi4m_araddr\(17),
      R => '0'
    );
\address_aclk_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[18]\,
      Q => \^axi4m_araddr\(18),
      R => '0'
    );
\address_aclk_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[19]\,
      Q => \^axi4m_araddr\(19),
      R => '0'
    );
\address_aclk_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[1]\,
      Q => \^axi4m_araddr\(1),
      R => '0'
    );
\address_aclk_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[20]\,
      Q => \^axi4m_araddr\(20),
      R => '0'
    );
\address_aclk_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[21]\,
      Q => \^axi4m_araddr\(21),
      R => '0'
    );
\address_aclk_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[22]\,
      Q => \^axi4m_araddr\(22),
      R => '0'
    );
\address_aclk_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[23]\,
      Q => \^axi4m_araddr\(23),
      R => '0'
    );
\address_aclk_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[24]\,
      Q => \^axi4m_araddr\(24),
      R => '0'
    );
\address_aclk_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[25]\,
      Q => \^axi4m_araddr\(25),
      R => '0'
    );
\address_aclk_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[26]\,
      Q => \^axi4m_araddr\(26),
      R => '0'
    );
\address_aclk_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[27]\,
      Q => \^axi4m_araddr\(27),
      R => '0'
    );
\address_aclk_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[28]\,
      Q => \^axi4m_araddr\(28),
      R => '0'
    );
\address_aclk_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[29]\,
      Q => \^axi4m_araddr\(29),
      R => '0'
    );
\address_aclk_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[2]\,
      Q => \^axi4m_araddr\(2),
      R => '0'
    );
\address_aclk_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[30]\,
      Q => \^axi4m_araddr\(30),
      R => '0'
    );
\address_aclk_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[31]\,
      Q => \^axi4m_araddr\(31),
      R => '0'
    );
\address_aclk_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[3]\,
      Q => \^axi4m_araddr\(3),
      R => '0'
    );
\address_aclk_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[4]\,
      Q => \^axi4m_araddr\(4),
      R => '0'
    );
\address_aclk_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[5]\,
      Q => \^axi4m_araddr\(5),
      R => '0'
    );
\address_aclk_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[6]\,
      Q => \^axi4m_araddr\(6),
      R => '0'
    );
\address_aclk_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[7]\,
      Q => \^axi4m_araddr\(7),
      R => '0'
    );
\address_aclk_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[8]\,
      Q => \^axi4m_araddr\(8),
      R => '0'
    );
\address_aclk_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \address_reg_n_0_[9]\,
      Q => \^axi4m_araddr\(9),
      R => '0'
    );
\address_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => p_0_in11_in,
      Q => \address_reg_n_0_[0]\,
      R => tck_reset
    );
\address_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_37,
      Q => \address_reg_n_0_[10]\,
      R => tck_reset
    );
\address_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_36,
      Q => \address_reg_n_0_[11]\,
      R => tck_reset
    );
\address_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_35,
      Q => \address_reg_n_0_[12]\,
      R => tck_reset
    );
\address_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_34,
      Q => \address_reg_n_0_[13]\,
      R => tck_reset
    );
\address_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_33,
      Q => \address_reg_n_0_[14]\,
      R => tck_reset
    );
\address_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_32,
      Q => \address_reg_n_0_[15]\,
      R => tck_reset
    );
\address_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_31,
      Q => \address_reg_n_0_[16]\,
      R => tck_reset
    );
\address_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_30,
      Q => \address_reg_n_0_[17]\,
      R => tck_reset
    );
\address_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_29,
      Q => \address_reg_n_0_[18]\,
      R => tck_reset
    );
\address_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_28,
      Q => \address_reg_n_0_[19]\,
      R => tck_reset
    );
\address_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_46,
      Q => \address_reg_n_0_[1]\,
      R => tck_reset
    );
\address_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_27,
      Q => \address_reg_n_0_[20]\,
      R => tck_reset
    );
\address_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_26,
      Q => \address_reg_n_0_[21]\,
      R => tck_reset
    );
\address_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_25,
      Q => \address_reg_n_0_[22]\,
      R => tck_reset
    );
\address_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_24,
      Q => \address_reg_n_0_[23]\,
      R => tck_reset
    );
\address_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_23,
      Q => \address_reg_n_0_[24]\,
      R => tck_reset
    );
\address_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_22,
      Q => \address_reg_n_0_[25]\,
      R => tck_reset
    );
\address_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_21,
      Q => \address_reg_n_0_[26]\,
      R => tck_reset
    );
\address_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_20,
      Q => \address_reg_n_0_[27]\,
      R => tck_reset
    );
\address_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_19,
      Q => \address_reg_n_0_[28]\,
      R => tck_reset
    );
\address_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_18,
      Q => \address_reg_n_0_[29]\,
      R => tck_reset
    );
\address_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_45,
      Q => \address_reg_n_0_[2]\,
      R => tck_reset
    );
\address_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_17,
      Q => \address_reg_n_0_[30]\,
      R => tck_reset
    );
\address_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_16,
      Q => \address_reg_n_0_[31]\,
      R => tck_reset
    );
\address_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_44,
      Q => \address_reg_n_0_[3]\,
      R => tck_reset
    );
\address_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_43,
      Q => \address_reg_n_0_[4]\,
      R => tck_reset
    );
\address_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_42,
      Q => \address_reg_n_0_[5]\,
      R => tck_reset
    );
\address_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_41,
      Q => \address_reg_n_0_[6]\,
      R => tck_reset
    );
\address_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => u_jtag_n_40,
      Q => \address_reg_n_0_[7]\,
      R => tck_reset
    );
\address_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => p_0_in(0),
      Q => \address_reg_n_0_[8]\,
      R => tck_reset
    );
\address_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => address,
      D => p_0_in(1),
      Q => \address_reg_n_0_[9]\,
      R => tck_reset
    );
axi4m_arvalid_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"44F4"
    )
        port map (
      I0 => trigger_d1,
      I1 => trigger,
      I2 => \^axi4m_arvalid\,
      I3 => axi4m_arready,
      O => axi4m_arvalid_i_1_n_0
    );
axi4m_arvalid_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => axi4m_arvalid_i_1_n_0,
      Q => \^axi4m_arvalid\,
      R => aclk_reset
    );
axi4m_awvalid_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"CFFF0202"
    )
        port map (
      I0 => \state_write_reg_n_0_[0]\,
      I1 => \state_write_reg_n_0_[2]\,
      I2 => \state_write_reg_n_0_[1]\,
      I3 => axi4m_awready,
      I4 => \^axi4m_awvalid\,
      O => axi4m_awvalid_i_1_n_0
    );
axi4m_awvalid_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => axi4m_awvalid_i_1_n_0,
      Q => \^axi4m_awvalid\,
      R => aclk_reset
    );
axi4m_rready_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00000000FF4C4C4C"
    )
        port map (
      I0 => axi4m_rlast,
      I1 => \^axi4m_rready\,
      I2 => axi4m_rvalid,
      I3 => axi4m_arready,
      I4 => \^axi4m_arvalid\,
      I5 => \read_count[0]_i_1_n_0\,
      O => axi4m_rready_i_1_n_0
    );
axi4m_rready_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => axi4m_rready_i_1_n_0,
      Q => \^axi4m_rready\,
      R => '0'
    );
axi4m_wlast_INST_0: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00010000"
    )
        port map (
      I0 => axi4m_wlast_INST_0_i_1_n_0,
      I1 => axi4m_wlast_INST_0_i_2_n_0,
      I2 => axi4m_wlast_INST_0_i_3_n_0,
      I3 => axi4m_wlast_INST_0_i_4_n_0,
      I4 => \^axi4m_wvalid\,
      O => axi4m_wlast
    );
axi4m_wlast_INST_0_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6FF6"
    )
        port map (
      I0 => \^axi4m_awlen\(3),
      I1 => write_word_count_reg(3),
      I2 => \^axi4m_awlen\(4),
      I3 => write_word_count_reg(4),
      O => axi4m_wlast_INST_0_i_1_n_0
    );
axi4m_wlast_INST_0_i_2: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6FF6"
    )
        port map (
      I0 => \^axi4m_awlen\(6),
      I1 => write_word_count_reg(6),
      I2 => \^axi4m_awlen\(7),
      I3 => write_word_count_reg(7),
      O => axi4m_wlast_INST_0_i_2_n_0
    );
axi4m_wlast_INST_0_i_3: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6FF6"
    )
        port map (
      I0 => \^axi4m_awlen\(1),
      I1 => write_word_count_reg(1),
      I2 => \^axi4m_awlen\(0),
      I3 => write_word_count_reg(0),
      O => axi4m_wlast_INST_0_i_3_n_0
    );
axi4m_wlast_INST_0_i_4: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6FF6"
    )
        port map (
      I0 => \^axi4m_awlen\(2),
      I1 => write_word_count_reg(2),
      I2 => \^axi4m_awlen\(5),
      I3 => write_word_count_reg(5),
      O => axi4m_wlast_INST_0_i_4_n_0
    );
axi4m_wvalid_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"EDFF0404"
    )
        port map (
      I0 => \state_write_reg_n_0_[1]\,
      I1 => \state_write_reg_n_0_[0]\,
      I2 => \state_write_reg_n_0_[2]\,
      I3 => axi4m_wvalid_i_2_n_0,
      I4 => \^axi4m_wvalid\,
      O => axi4m_wvalid_i_1_n_0
    );
axi4m_wvalid_i_2: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0001000000000000"
    )
        port map (
      I0 => axi4m_wlast_INST_0_i_4_n_0,
      I1 => axi4m_wlast_INST_0_i_3_n_0,
      I2 => axi4m_wlast_INST_0_i_2_n_0,
      I3 => axi4m_wlast_INST_0_i_1_n_0,
      I4 => axi4m_wready,
      I5 => \^axi4m_wvalid\,
      O => axi4m_wvalid_i_2_n_0
    );
axi4m_wvalid_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => axi4m_wvalid_i_1_n_0,
      Q => \^axi4m_wvalid\,
      R => aclk_reset
    );
\bitcount[0]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => bitcount_reg(0),
      O => \bitcount[0]_i_1_n_0\
    );
\bitcount[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => bitcount_reg(1),
      I1 => bitcount_reg(0),
      O => \p_0_in__2\(1)
    );
\bitcount[2]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"6A"
    )
        port map (
      I0 => bitcount_reg(2),
      I1 => bitcount_reg(1),
      I2 => bitcount_reg(0),
      O => \p_0_in__2\(2)
    );
\bitcount[3]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6AAA"
    )
        port map (
      I0 => bitcount_reg(3),
      I1 => bitcount_reg(0),
      I2 => bitcount_reg(1),
      I3 => bitcount_reg(2),
      O => \p_0_in__2\(3)
    );
\bitcount[4]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFF8000"
    )
        port map (
      I0 => bitcount_reg(2),
      I1 => bitcount_reg(1),
      I2 => bitcount_reg(0),
      I3 => bitcount_reg(3),
      I4 => bitcount_reg(4),
      O => \p_0_in__2\(4)
    );
\bitcount[5]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"6AAAAAAAAAAAAAAA"
    )
        port map (
      I0 => bitcount_reg(5),
      I1 => bitcount_reg(2),
      I2 => bitcount_reg(1),
      I3 => bitcount_reg(0),
      I4 => bitcount_reg(3),
      I5 => bitcount_reg(4),
      O => \p_0_in__2\(5)
    );
\bitcount_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_out_en,
      D => \bitcount[0]_i_1_n_0\,
      Q => bitcount_reg(0),
      R => u_jtag_n_48
    );
\bitcount_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_out_en,
      D => \p_0_in__2\(1),
      Q => bitcount_reg(1),
      R => u_jtag_n_48
    );
\bitcount_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_out_en,
      D => \p_0_in__2\(2),
      Q => bitcount_reg(2),
      R => u_jtag_n_48
    );
\bitcount_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_out_en,
      D => \p_0_in__2\(3),
      Q => bitcount_reg(3),
      R => u_jtag_n_48
    );
\bitcount_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_out_en,
      D => \p_0_in__2\(4),
      Q => bitcount_reg(4),
      R => u_jtag_n_48
    );
\bitcount_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_out_en,
      D => \p_0_in__2\(5),
      Q => bitcount_reg(5),
      R => u_jtag_n_48
    );
\burst_length_aclk_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \burst_length_reg_n_0_[0]\,
      Q => \^axi4m_awlen\(0),
      R => '0'
    );
\burst_length_aclk_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \burst_length_reg_n_0_[1]\,
      Q => \^axi4m_awlen\(1),
      R => '0'
    );
\burst_length_aclk_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \burst_length_reg_n_0_[2]\,
      Q => \^axi4m_awlen\(2),
      R => '0'
    );
\burst_length_aclk_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \burst_length_reg_n_0_[3]\,
      Q => \^axi4m_awlen\(3),
      R => '0'
    );
\burst_length_aclk_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \burst_length_reg_n_0_[4]\,
      Q => \^axi4m_awlen\(4),
      R => '0'
    );
\burst_length_aclk_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \burst_length_reg_n_0_[5]\,
      Q => \^axi4m_awlen\(5),
      R => '0'
    );
\burst_length_aclk_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \burst_length_reg_n_0_[6]\,
      Q => \^axi4m_awlen\(6),
      R => '0'
    );
\burst_length_aclk_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \burst_length_reg_n_0_[7]\,
      Q => \^axi4m_awlen\(7),
      R => '0'
    );
\burst_length_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => burst_length,
      D => p_0_in11_in,
      Q => \burst_length_reg_n_0_[0]\,
      R => '0'
    );
\burst_length_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => burst_length,
      D => u_jtag_n_46,
      Q => \burst_length_reg_n_0_[1]\,
      R => '0'
    );
\burst_length_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => burst_length,
      D => u_jtag_n_45,
      Q => \burst_length_reg_n_0_[2]\,
      R => '0'
    );
\burst_length_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => burst_length,
      D => u_jtag_n_44,
      Q => \burst_length_reg_n_0_[3]\,
      R => '0'
    );
\burst_length_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => burst_length,
      D => u_jtag_n_43,
      Q => \burst_length_reg_n_0_[4]\,
      R => '0'
    );
\burst_length_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => burst_length,
      D => u_jtag_n_42,
      Q => \burst_length_reg_n_0_[5]\,
      R => '0'
    );
\burst_length_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => burst_length,
      D => u_jtag_n_41,
      Q => \burst_length_reg_n_0_[6]\,
      R => '0'
    );
\burst_length_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => burst_length,
      D => u_jtag_n_40,
      Q => \burst_length_reg_n_0_[7]\,
      R => '0'
    );
\burst_type_aclk_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => burst_type(0),
      Q => \^axi4m_arburst\(0),
      R => '0'
    );
\burst_type_aclk_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => burst_type(1),
      Q => \^axi4m_arburst\(1),
      R => '0'
    );
\burst_type_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => burst_length,
      D => p_0_in(0),
      Q => burst_type(0),
      R => '0'
    );
\burst_type_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => burst_length,
      D => p_0_in(1),
      Q => burst_type(1),
      R => '0'
    );
jtag_reset_d1_reg: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => '1',
      D => jtag_reset,
      Q => jtag_reset_d1,
      R => '0'
    );
\read_count[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"BA"
    )
        port map (
      I0 => aclk_reset,
      I1 => trigger_d1,
      I2 => trigger,
      O => \read_count[0]_i_1_n_0\
    );
\read_count[0]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"7000"
    )
        port map (
      I0 => \^axi4m_arvalid\,
      I1 => axi4m_arready,
      I2 => \^axi4m_rready\,
      I3 => axi4m_rvalid,
      O => read_count
    );
\read_count[0]_i_4\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => read_count_reg(0),
      O => \read_count[0]_i_4_n_0\
    );
\read_count_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => read_count,
      D => \read_count_reg[0]_i_3_n_15\,
      Q => read_count_reg(0),
      R => \read_count[0]_i_1_n_0\
    );
\read_count_reg[0]_i_3\: unisim.vcomponents.CARRY8
     port map (
      CI => '0',
      CI_TOP => '0',
      CO(7) => \NLW_read_count_reg[0]_i_3_CO_UNCONNECTED\(7),
      CO(6) => \read_count_reg[0]_i_3_n_1\,
      CO(5) => \read_count_reg[0]_i_3_n_2\,
      CO(4) => \read_count_reg[0]_i_3_n_3\,
      CO(3) => \read_count_reg[0]_i_3_n_4\,
      CO(2) => \read_count_reg[0]_i_3_n_5\,
      CO(1) => \read_count_reg[0]_i_3_n_6\,
      CO(0) => \read_count_reg[0]_i_3_n_7\,
      DI(7 downto 0) => B"00000001",
      O(7) => \read_count_reg[0]_i_3_n_8\,
      O(6) => \read_count_reg[0]_i_3_n_9\,
      O(5) => \read_count_reg[0]_i_3_n_10\,
      O(4) => \read_count_reg[0]_i_3_n_11\,
      O(3) => \read_count_reg[0]_i_3_n_12\,
      O(2) => \read_count_reg[0]_i_3_n_13\,
      O(1) => \read_count_reg[0]_i_3_n_14\,
      O(0) => \read_count_reg[0]_i_3_n_15\,
      S(7 downto 1) => read_count_reg(7 downto 1),
      S(0) => \read_count[0]_i_4_n_0\
    );
\read_count_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => read_count,
      D => \read_count_reg[0]_i_3_n_14\,
      Q => read_count_reg(1),
      R => \read_count[0]_i_1_n_0\
    );
\read_count_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => read_count,
      D => \read_count_reg[0]_i_3_n_13\,
      Q => read_count_reg(2),
      R => \read_count[0]_i_1_n_0\
    );
\read_count_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => read_count,
      D => \read_count_reg[0]_i_3_n_12\,
      Q => read_count_reg(3),
      R => \read_count[0]_i_1_n_0\
    );
\read_count_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => read_count,
      D => \read_count_reg[0]_i_3_n_11\,
      Q => read_count_reg(4),
      R => \read_count[0]_i_1_n_0\
    );
\read_count_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => read_count,
      D => \read_count_reg[0]_i_3_n_10\,
      Q => read_count_reg(5),
      R => \read_count[0]_i_1_n_0\
    );
\read_count_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => read_count,
      D => \read_count_reg[0]_i_3_n_9\,
      Q => read_count_reg(6),
      R => \read_count[0]_i_1_n_0\
    );
\read_count_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => read_count,
      D => \read_count_reg[0]_i_3_n_8\,
      Q => read_count_reg(7),
      R => \read_count[0]_i_1_n_0\
    );
readram: entity work.BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram
     port map (
      ADDRBWRADDR(7 downto 0) => read_count_reg(7 downto 0),
      D(63 downto 0) => shift_reg(63 downto 0),
      Q(7 downto 0) => readram_raddr_reg(7 downto 0),
      aclk => aclk,
      axi4m_rdata(63 downto 0) => axi4m_rdata(63 downto 0),
      axi4m_rvalid => axi4m_rvalid,
      \bitcount_reg[3]\ => readram_n_64,
      ram_reg_0(4 downto 0) => bitcount_reg(5 downto 1),
      readram_rd => readram_rd,
      \shift_reg_reg[0]\ => \shift_reg[63]_i_3_n_0\,
      \shift_reg_reg[62]\(62 downto 0) => data1(62 downto 0),
      tck => tck
    );
\readram_raddr[0]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => readram_raddr_reg(0),
      O => \p_0_in__3\(0)
    );
\readram_raddr[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => readram_raddr_reg(0),
      I1 => readram_raddr_reg(1),
      O => \p_0_in__3\(1)
    );
\readram_raddr[2]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"6A"
    )
        port map (
      I0 => readram_raddr_reg(2),
      I1 => readram_raddr_reg(0),
      I2 => readram_raddr_reg(1),
      O => \p_0_in__3\(2)
    );
\readram_raddr[3]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6AAA"
    )
        port map (
      I0 => readram_raddr_reg(3),
      I1 => readram_raddr_reg(1),
      I2 => readram_raddr_reg(0),
      I3 => readram_raddr_reg(2),
      O => \p_0_in__3\(3)
    );
\readram_raddr[4]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFF8000"
    )
        port map (
      I0 => readram_raddr_reg(2),
      I1 => readram_raddr_reg(0),
      I2 => readram_raddr_reg(1),
      I3 => readram_raddr_reg(3),
      I4 => readram_raddr_reg(4),
      O => \p_0_in__3\(4)
    );
\readram_raddr[5]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"6AAAAAAAAAAAAAAA"
    )
        port map (
      I0 => readram_raddr_reg(5),
      I1 => readram_raddr_reg(2),
      I2 => readram_raddr_reg(0),
      I3 => readram_raddr_reg(1),
      I4 => readram_raddr_reg(3),
      I5 => readram_raddr_reg(4),
      O => \p_0_in__3\(5)
    );
\readram_raddr[6]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"6A"
    )
        port map (
      I0 => readram_raddr_reg(6),
      I1 => \readram_raddr[7]_i_3_n_0\,
      I2 => readram_raddr_reg(5),
      O => \p_0_in__3\(6)
    );
\readram_raddr[7]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6AAA"
    )
        port map (
      I0 => readram_raddr_reg(7),
      I1 => readram_raddr_reg(5),
      I2 => \readram_raddr[7]_i_3_n_0\,
      I3 => readram_raddr_reg(6),
      O => \p_0_in__3\(7)
    );
\readram_raddr[7]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"80000000"
    )
        port map (
      I0 => readram_raddr_reg(4),
      I1 => readram_raddr_reg(3),
      I2 => readram_raddr_reg(1),
      I3 => readram_raddr_reg(0),
      I4 => readram_raddr_reg(2),
      O => \readram_raddr[7]_i_3_n_0\
    );
\readram_raddr_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => readram_raddr,
      D => \p_0_in__3\(0),
      Q => readram_raddr_reg(0),
      R => u_jtag_n_48
    );
\readram_raddr_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => readram_raddr,
      D => \p_0_in__3\(1),
      Q => readram_raddr_reg(1),
      R => u_jtag_n_48
    );
\readram_raddr_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => readram_raddr,
      D => \p_0_in__3\(2),
      Q => readram_raddr_reg(2),
      R => u_jtag_n_48
    );
\readram_raddr_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => readram_raddr,
      D => \p_0_in__3\(3),
      Q => readram_raddr_reg(3),
      R => u_jtag_n_48
    );
\readram_raddr_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => readram_raddr,
      D => \p_0_in__3\(4),
      Q => readram_raddr_reg(4),
      R => u_jtag_n_48
    );
\readram_raddr_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => readram_raddr,
      D => \p_0_in__3\(5),
      Q => readram_raddr_reg(5),
      R => u_jtag_n_48
    );
\readram_raddr_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => readram_raddr,
      D => \p_0_in__3\(6),
      Q => readram_raddr_reg(6),
      R => u_jtag_n_48
    );
\readram_raddr_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => readram_raddr,
      D => \p_0_in__3\(7),
      Q => readram_raddr_reg(7),
      R => u_jtag_n_48
    );
\shift_in_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[1]\,
      Q => \shift_in_reg_reg_n_0_[0]\,
      R => '0'
    );
\shift_in_reg_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[11]\,
      Q => \shift_in_reg_reg_n_0_[10]\,
      R => '0'
    );
\shift_in_reg_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[12]\,
      Q => \shift_in_reg_reg_n_0_[11]\,
      R => '0'
    );
\shift_in_reg_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[13]\,
      Q => \shift_in_reg_reg_n_0_[12]\,
      R => '0'
    );
\shift_in_reg_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[14]\,
      Q => \shift_in_reg_reg_n_0_[13]\,
      R => '0'
    );
\shift_in_reg_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[15]\,
      Q => \shift_in_reg_reg_n_0_[14]\,
      R => '0'
    );
\shift_in_reg_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[16]\,
      Q => \shift_in_reg_reg_n_0_[15]\,
      R => '0'
    );
\shift_in_reg_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[17]\,
      Q => \shift_in_reg_reg_n_0_[16]\,
      R => '0'
    );
\shift_in_reg_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[18]\,
      Q => \shift_in_reg_reg_n_0_[17]\,
      R => '0'
    );
\shift_in_reg_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[19]\,
      Q => \shift_in_reg_reg_n_0_[18]\,
      R => '0'
    );
\shift_in_reg_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[20]\,
      Q => \shift_in_reg_reg_n_0_[19]\,
      R => '0'
    );
\shift_in_reg_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[2]\,
      Q => \shift_in_reg_reg_n_0_[1]\,
      R => '0'
    );
\shift_in_reg_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[21]\,
      Q => \shift_in_reg_reg_n_0_[20]\,
      R => '0'
    );
\shift_in_reg_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[22]\,
      Q => \shift_in_reg_reg_n_0_[21]\,
      R => '0'
    );
\shift_in_reg_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[23]\,
      Q => \shift_in_reg_reg_n_0_[22]\,
      R => '0'
    );
\shift_in_reg_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[24]\,
      Q => \shift_in_reg_reg_n_0_[23]\,
      R => '0'
    );
\shift_in_reg_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[25]\,
      Q => \shift_in_reg_reg_n_0_[24]\,
      R => '0'
    );
\shift_in_reg_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[26]\,
      Q => \shift_in_reg_reg_n_0_[25]\,
      R => '0'
    );
\shift_in_reg_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[27]\,
      Q => \shift_in_reg_reg_n_0_[26]\,
      R => '0'
    );
\shift_in_reg_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[28]\,
      Q => \shift_in_reg_reg_n_0_[27]\,
      R => '0'
    );
\shift_in_reg_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[29]\,
      Q => \shift_in_reg_reg_n_0_[28]\,
      R => '0'
    );
\shift_in_reg_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[30]\,
      Q => \shift_in_reg_reg_n_0_[29]\,
      R => '0'
    );
\shift_in_reg_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[3]\,
      Q => \shift_in_reg_reg_n_0_[2]\,
      R => '0'
    );
\shift_in_reg_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[31]\,
      Q => \shift_in_reg_reg_n_0_[30]\,
      R => '0'
    );
\shift_in_reg_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[32]\,
      Q => \shift_in_reg_reg_n_0_[31]\,
      R => '0'
    );
\shift_in_reg_reg[32]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[33]\,
      Q => \shift_in_reg_reg_n_0_[32]\,
      R => '0'
    );
\shift_in_reg_reg[33]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[34]\,
      Q => \shift_in_reg_reg_n_0_[33]\,
      R => '0'
    );
\shift_in_reg_reg[34]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[35]\,
      Q => \shift_in_reg_reg_n_0_[34]\,
      R => '0'
    );
\shift_in_reg_reg[35]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[36]\,
      Q => \shift_in_reg_reg_n_0_[35]\,
      R => '0'
    );
\shift_in_reg_reg[36]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[37]\,
      Q => \shift_in_reg_reg_n_0_[36]\,
      R => '0'
    );
\shift_in_reg_reg[37]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[38]\,
      Q => \shift_in_reg_reg_n_0_[37]\,
      R => '0'
    );
\shift_in_reg_reg[38]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[39]\,
      Q => \shift_in_reg_reg_n_0_[38]\,
      R => '0'
    );
\shift_in_reg_reg[39]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[40]\,
      Q => \shift_in_reg_reg_n_0_[39]\,
      R => '0'
    );
\shift_in_reg_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[4]\,
      Q => \shift_in_reg_reg_n_0_[3]\,
      R => '0'
    );
\shift_in_reg_reg[40]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[41]\,
      Q => \shift_in_reg_reg_n_0_[40]\,
      R => '0'
    );
\shift_in_reg_reg[41]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[42]\,
      Q => \shift_in_reg_reg_n_0_[41]\,
      R => '0'
    );
\shift_in_reg_reg[42]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[43]\,
      Q => \shift_in_reg_reg_n_0_[42]\,
      R => '0'
    );
\shift_in_reg_reg[43]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[44]\,
      Q => \shift_in_reg_reg_n_0_[43]\,
      R => '0'
    );
\shift_in_reg_reg[44]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[45]\,
      Q => \shift_in_reg_reg_n_0_[44]\,
      R => '0'
    );
\shift_in_reg_reg[45]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[46]\,
      Q => \shift_in_reg_reg_n_0_[45]\,
      R => '0'
    );
\shift_in_reg_reg[46]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[47]\,
      Q => \shift_in_reg_reg_n_0_[46]\,
      R => '0'
    );
\shift_in_reg_reg[47]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[48]\,
      Q => \shift_in_reg_reg_n_0_[47]\,
      R => '0'
    );
\shift_in_reg_reg[48]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[49]\,
      Q => \shift_in_reg_reg_n_0_[48]\,
      R => '0'
    );
\shift_in_reg_reg[49]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[50]\,
      Q => \shift_in_reg_reg_n_0_[49]\,
      R => '0'
    );
\shift_in_reg_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[5]\,
      Q => \shift_in_reg_reg_n_0_[4]\,
      R => '0'
    );
\shift_in_reg_reg[50]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[51]\,
      Q => \shift_in_reg_reg_n_0_[50]\,
      R => '0'
    );
\shift_in_reg_reg[51]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[52]\,
      Q => \shift_in_reg_reg_n_0_[51]\,
      R => '0'
    );
\shift_in_reg_reg[52]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[53]\,
      Q => \shift_in_reg_reg_n_0_[52]\,
      R => '0'
    );
\shift_in_reg_reg[53]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[54]\,
      Q => \shift_in_reg_reg_n_0_[53]\,
      R => '0'
    );
\shift_in_reg_reg[54]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[55]\,
      Q => \shift_in_reg_reg_n_0_[54]\,
      R => '0'
    );
\shift_in_reg_reg[55]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[56]\,
      Q => \shift_in_reg_reg_n_0_[55]\,
      R => '0'
    );
\shift_in_reg_reg[56]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[57]\,
      Q => \shift_in_reg_reg_n_0_[56]\,
      R => '0'
    );
\shift_in_reg_reg[57]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[58]\,
      Q => \shift_in_reg_reg_n_0_[57]\,
      R => '0'
    );
\shift_in_reg_reg[58]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[59]\,
      Q => \shift_in_reg_reg_n_0_[58]\,
      R => '0'
    );
\shift_in_reg_reg[59]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[60]\,
      Q => \shift_in_reg_reg_n_0_[59]\,
      R => '0'
    );
\shift_in_reg_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[6]\,
      Q => \shift_in_reg_reg_n_0_[5]\,
      R => '0'
    );
\shift_in_reg_reg[60]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[61]\,
      Q => \shift_in_reg_reg_n_0_[60]\,
      R => '0'
    );
\shift_in_reg_reg[61]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[62]\,
      Q => \shift_in_reg_reg_n_0_[61]\,
      R => '0'
    );
\shift_in_reg_reg[62]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[63]\,
      Q => \shift_in_reg_reg_n_0_[62]\,
      R => '0'
    );
\shift_in_reg_reg[63]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => shift_in_data,
      Q => \shift_in_reg_reg_n_0_[63]\,
      R => '0'
    );
\shift_in_reg_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[7]\,
      Q => \shift_in_reg_reg_n_0_[6]\,
      R => '0'
    );
\shift_in_reg_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[8]\,
      Q => \shift_in_reg_reg_n_0_[7]\,
      R => '0'
    );
\shift_in_reg_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[9]\,
      Q => \shift_in_reg_reg_n_0_[8]\,
      R => '0'
    );
\shift_in_reg_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => \shift_in_reg_reg_n_0_[10]\,
      Q => \shift_in_reg_reg_n_0_[9]\,
      R => '0'
    );
shift_in_state_d1_i_1: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => tck_reset,
      O => shift_in_state_d1_i_1_n_0
    );
shift_in_state_d1_reg: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_state_d1_i_1_n_0,
      D => shift_in_state,
      Q => shift_in_state_d1,
      R => '0'
    );
\shift_reg[63]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFFFFFFFFD"
    )
        port map (
      I0 => bitcount_reg(0),
      I1 => bitcount_reg(5),
      I2 => bitcount_reg(1),
      I3 => bitcount_reg(2),
      I4 => bitcount_reg(4),
      I5 => bitcount_reg(3),
      O => \shift_reg[63]_i_3_n_0\
    );
\shift_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(0),
      Q => \shift_reg_reg_n_0_[0]\,
      R => '0'
    );
\shift_reg_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(10),
      Q => data1(9),
      R => '0'
    );
\shift_reg_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(11),
      Q => data1(10),
      R => '0'
    );
\shift_reg_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(12),
      Q => data1(11),
      R => '0'
    );
\shift_reg_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(13),
      Q => data1(12),
      R => '0'
    );
\shift_reg_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(14),
      Q => data1(13),
      R => '0'
    );
\shift_reg_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(15),
      Q => data1(14),
      R => '0'
    );
\shift_reg_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(16),
      Q => data1(15),
      R => '0'
    );
\shift_reg_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(17),
      Q => data1(16),
      R => '0'
    );
\shift_reg_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(18),
      Q => data1(17),
      R => '0'
    );
\shift_reg_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(19),
      Q => data1(18),
      R => '0'
    );
\shift_reg_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(1),
      Q => data1(0),
      R => '0'
    );
\shift_reg_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(20),
      Q => data1(19),
      R => '0'
    );
\shift_reg_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(21),
      Q => data1(20),
      R => '0'
    );
\shift_reg_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(22),
      Q => data1(21),
      R => '0'
    );
\shift_reg_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(23),
      Q => data1(22),
      R => '0'
    );
\shift_reg_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(24),
      Q => data1(23),
      R => '0'
    );
\shift_reg_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(25),
      Q => data1(24),
      R => '0'
    );
\shift_reg_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(26),
      Q => data1(25),
      R => '0'
    );
\shift_reg_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(27),
      Q => data1(26),
      R => '0'
    );
\shift_reg_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(28),
      Q => data1(27),
      R => '0'
    );
\shift_reg_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(29),
      Q => data1(28),
      R => '0'
    );
\shift_reg_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(2),
      Q => data1(1),
      R => '0'
    );
\shift_reg_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(30),
      Q => data1(29),
      R => '0'
    );
\shift_reg_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(31),
      Q => data1(30),
      R => '0'
    );
\shift_reg_reg[32]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(32),
      Q => data1(31),
      R => '0'
    );
\shift_reg_reg[33]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(33),
      Q => data1(32),
      R => '0'
    );
\shift_reg_reg[34]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(34),
      Q => data1(33),
      R => '0'
    );
\shift_reg_reg[35]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(35),
      Q => data1(34),
      R => '0'
    );
\shift_reg_reg[36]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(36),
      Q => data1(35),
      R => '0'
    );
\shift_reg_reg[37]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(37),
      Q => data1(36),
      R => '0'
    );
\shift_reg_reg[38]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(38),
      Q => data1(37),
      R => '0'
    );
\shift_reg_reg[39]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(39),
      Q => data1(38),
      R => '0'
    );
\shift_reg_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(3),
      Q => data1(2),
      R => '0'
    );
\shift_reg_reg[40]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(40),
      Q => data1(39),
      R => '0'
    );
\shift_reg_reg[41]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(41),
      Q => data1(40),
      R => '0'
    );
\shift_reg_reg[42]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(42),
      Q => data1(41),
      R => '0'
    );
\shift_reg_reg[43]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(43),
      Q => data1(42),
      R => '0'
    );
\shift_reg_reg[44]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(44),
      Q => data1(43),
      R => '0'
    );
\shift_reg_reg[45]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(45),
      Q => data1(44),
      R => '0'
    );
\shift_reg_reg[46]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(46),
      Q => data1(45),
      R => '0'
    );
\shift_reg_reg[47]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(47),
      Q => data1(46),
      R => '0'
    );
\shift_reg_reg[48]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(48),
      Q => data1(47),
      R => '0'
    );
\shift_reg_reg[49]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(49),
      Q => data1(48),
      R => '0'
    );
\shift_reg_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(4),
      Q => data1(3),
      R => '0'
    );
\shift_reg_reg[50]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(50),
      Q => data1(49),
      R => '0'
    );
\shift_reg_reg[51]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(51),
      Q => data1(50),
      R => '0'
    );
\shift_reg_reg[52]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(52),
      Q => data1(51),
      R => '0'
    );
\shift_reg_reg[53]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(53),
      Q => data1(52),
      R => '0'
    );
\shift_reg_reg[54]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(54),
      Q => data1(53),
      R => '0'
    );
\shift_reg_reg[55]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(55),
      Q => data1(54),
      R => '0'
    );
\shift_reg_reg[56]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(56),
      Q => data1(55),
      R => '0'
    );
\shift_reg_reg[57]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(57),
      Q => data1(56),
      R => '0'
    );
\shift_reg_reg[58]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(58),
      Q => data1(57),
      R => '0'
    );
\shift_reg_reg[59]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(59),
      Q => data1(58),
      R => '0'
    );
\shift_reg_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(5),
      Q => data1(4),
      R => '0'
    );
\shift_reg_reg[60]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(60),
      Q => data1(59),
      R => '0'
    );
\shift_reg_reg[61]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(61),
      Q => data1(60),
      R => '0'
    );
\shift_reg_reg[62]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(62),
      Q => data1(61),
      R => '0'
    );
\shift_reg_reg[63]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(63),
      Q => data1(62),
      R => '0'
    );
\shift_reg_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(6),
      Q => data1(5),
      R => '0'
    );
\shift_reg_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(7),
      Q => data1(6),
      R => '0'
    );
\shift_reg_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(8),
      Q => data1(7),
      R => '0'
    );
\shift_reg_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => u_jtag_n_8,
      D => shift_reg(9),
      Q => data1(8),
      R => '0'
    );
start_read_aclk_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => start_read_reg_n_0,
      Q => start_read_aclk,
      R => '0'
    );
start_read_reg: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => '1',
      D => u_jtag_n_15,
      Q => start_read_reg_n_0,
      R => tck_reset
    );
start_write_d1_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => start_write,
      Q => start_write_d1,
      R => '0'
    );
start_write_d2_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => start_write_d1,
      Q => start_write_d2,
      R => '0'
    );
start_write_reg: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => '1',
      D => u_jtag_n_14,
      Q => start_write,
      R => tck_reset
    );
\state_write[0]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FBF8"
    )
        port map (
      I0 => \state_write[0]_i_2_n_0\,
      I1 => \state_write[2]_i_2_n_0\,
      I2 => writeram_raddr_reg0,
      I3 => \state_write_reg_n_0_[0]\,
      O => \state_write[0]_i_1_n_0\
    );
\state_write[0]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"550045AA"
    )
        port map (
      I0 => \state_write_reg_n_0_[2]\,
      I1 => axi4m_wvalid_i_2_n_0,
      I2 => axi4m_awready,
      I3 => \state_write_reg_n_0_[1]\,
      I4 => \state_write_reg_n_0_[0]\,
      O => \state_write[0]_i_2_n_0\
    );
\state_write[0]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00000100"
    )
        port map (
      I0 => \state_write_reg_n_0_[1]\,
      I1 => \state_write_reg_n_0_[0]\,
      I2 => \state_write_reg_n_0_[2]\,
      I3 => start_write_d1,
      I4 => start_write_d2,
      O => writeram_raddr_reg0
    );
\state_write[1]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1014"
    )
        port map (
      I0 => \state_write_reg_n_0_[2]\,
      I1 => \state_write_reg_n_0_[1]\,
      I2 => \state_write_reg_n_0_[0]\,
      I3 => axi4m_awready,
      O => \state_write[1]_i_1_n_0\
    );
\state_write[2]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"AAAAAAAAAAAAAABA"
    )
        port map (
      I0 => aclk_reset,
      I1 => start_write_d2,
      I2 => start_write_d1,
      I3 => \state_write_reg_n_0_[2]\,
      I4 => \state_write_reg_n_0_[0]\,
      I5 => \state_write_reg_n_0_[1]\,
      O => state_write(1)
    );
\state_write[2]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0A0EFFC00A0E0FC0"
    )
        port map (
      I0 => axi4m_awready,
      I1 => axi4m_wvalid_i_2_n_0,
      I2 => \state_write_reg_n_0_[2]\,
      I3 => \state_write_reg_n_0_[0]\,
      I4 => \state_write_reg_n_0_[1]\,
      I5 => axi4m_bvalid,
      O => \state_write[2]_i_2_n_0\
    );
\state_write[2]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"5242"
    )
        port map (
      I0 => \state_write_reg_n_0_[2]\,
      I1 => \state_write_reg_n_0_[0]\,
      I2 => \state_write_reg_n_0_[1]\,
      I3 => axi4m_awready,
      O => \state_write[2]_i_3_n_0\
    );
\state_write_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \state_write[0]_i_1_n_0\,
      Q => \state_write_reg_n_0_[0]\,
      R => aclk_reset
    );
\state_write_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => \state_write[2]_i_2_n_0\,
      D => \state_write[1]_i_1_n_0\,
      Q => \state_write_reg_n_0_[1]\,
      R => state_write(1)
    );
\state_write_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => \state_write[2]_i_2_n_0\,
      D => \state_write[2]_i_3_n_0\,
      Q => \state_write_reg_n_0_[2]\,
      R => state_write(1)
    );
\status[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"5555555510000000"
    )
        port map (
      I0 => \read_count[0]_i_1_n_0\,
      I1 => axi4m_arvalid0,
      I2 => axi4m_rlast,
      I3 => \^axi4m_rready\,
      I4 => axi4m_rvalid,
      I5 => status(0),
      O => \status[0]_i_1_n_0\
    );
\status[0]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => axi4m_arready,
      I1 => \^axi4m_arvalid\,
      O => axi4m_arvalid0
    );
\status[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000FFCF00000080"
    )
        port map (
      I0 => axi4m_bvalid,
      I1 => \state_write_reg_n_0_[2]\,
      I2 => \state_write_reg_n_0_[0]\,
      I3 => \state_write_reg_n_0_[1]\,
      I4 => aclk_reset,
      I5 => status(1),
      O => \status[1]_i_1_n_0\
    );
\status_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \status[0]_i_1_n_0\,
      Q => status(0),
      R => '0'
    );
\status_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => \status[1]_i_1_n_0\,
      Q => status(1),
      R => '0'
    );
tck_reset_reg: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => '1',
      D => jtag_reset_d1,
      Q => tck_reset,
      R => '0'
    );
tmp1_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => tck_reset,
      Q => tmp1,
      R => '0'
    );
tmp2_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => tmp1,
      Q => tmp2,
      R => '0'
    );
tmp3_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => aresetn,
      Q => tmp3,
      R => '0'
    );
tmp4_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => tmp3,
      Q => tmp4,
      R => '0'
    );
trigger_d1_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => trigger,
      Q => trigger_d1,
      R => '0'
    );
trigger_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => start_read_aclk,
      Q => trigger,
      R => '0'
    );
trigger_tck_d1_reg: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_state_d1_i_1_n_0,
      D => trigger_tck,
      Q => trigger_tck_d1,
      R => '0'
    );
trigger_tck_reg: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_state_d1_i_1_n_0,
      D => trigger_d1,
      Q => trigger_tck,
      R => '0'
    );
u_jtag: entity work.BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_core
     port map (
      D(0) => shift_in_data,
      E(0) => address,
      Q(0) => aclk_cnt_reg(3),
      S(1) => \write_count_reg_n_0_[15]\,
      S(0) => \write_count_reg_n_0_[14]\,
      SR(0) => u_jtag_n_48,
      aresetn => aresetn,
      jtag_reset => jtag_reset,
      ram_reg => readram_n_64,
      ram_reg_0(0) => bitcount_reg(0),
      \readram_raddr_reg[0]\ => \shift_reg[63]_i_3_n_0\,
      readram_rd => readram_rd,
      \reg_wdata_reg[0]_0\ => u_jtag_n_15,
      \reg_wdata_reg[31]_0\(31) => u_jtag_n_16,
      \reg_wdata_reg[31]_0\(30) => u_jtag_n_17,
      \reg_wdata_reg[31]_0\(29) => u_jtag_n_18,
      \reg_wdata_reg[31]_0\(28) => u_jtag_n_19,
      \reg_wdata_reg[31]_0\(27) => u_jtag_n_20,
      \reg_wdata_reg[31]_0\(26) => u_jtag_n_21,
      \reg_wdata_reg[31]_0\(25) => u_jtag_n_22,
      \reg_wdata_reg[31]_0\(24) => u_jtag_n_23,
      \reg_wdata_reg[31]_0\(23) => u_jtag_n_24,
      \reg_wdata_reg[31]_0\(22) => u_jtag_n_25,
      \reg_wdata_reg[31]_0\(21) => u_jtag_n_26,
      \reg_wdata_reg[31]_0\(20) => u_jtag_n_27,
      \reg_wdata_reg[31]_0\(19) => u_jtag_n_28,
      \reg_wdata_reg[31]_0\(18) => u_jtag_n_29,
      \reg_wdata_reg[31]_0\(17) => u_jtag_n_30,
      \reg_wdata_reg[31]_0\(16) => u_jtag_n_31,
      \reg_wdata_reg[31]_0\(15) => u_jtag_n_32,
      \reg_wdata_reg[31]_0\(14) => u_jtag_n_33,
      \reg_wdata_reg[31]_0\(13) => u_jtag_n_34,
      \reg_wdata_reg[31]_0\(12) => u_jtag_n_35,
      \reg_wdata_reg[31]_0\(11) => u_jtag_n_36,
      \reg_wdata_reg[31]_0\(10) => u_jtag_n_37,
      \reg_wdata_reg[31]_0\(9 downto 8) => p_0_in(1 downto 0),
      \reg_wdata_reg[31]_0\(7) => u_jtag_n_40,
      \reg_wdata_reg[31]_0\(6) => u_jtag_n_41,
      \reg_wdata_reg[31]_0\(5) => u_jtag_n_42,
      \reg_wdata_reg[31]_0\(4) => u_jtag_n_43,
      \reg_wdata_reg[31]_0\(3) => u_jtag_n_44,
      \reg_wdata_reg[31]_0\(2) => u_jtag_n_45,
      \reg_wdata_reg[31]_0\(1) => u_jtag_n_46,
      \reg_wdata_reg[31]_0\(0) => p_0_in11_in,
      shift_in_en => shift_in_en,
      shift_in_en_reg_0(0) => shift_in_reg,
      shift_in_state => shift_in_state,
      shift_in_state_d1 => shift_in_state_d1,
      shift_in_state_d1_reg => u_jtag_n_14,
      shift_out_en => shift_out_en,
      shift_out_en_reg_0(0) => u_jtag_n_8,
      shift_out_en_reg_1(0) => readram_raddr,
      start_read_reg => start_read_reg_n_0,
      start_write => start_write,
      status(1 downto 0) => status(1 downto 0),
      tck => tck,
      tck_reset => tck_reset,
      tck_reset_reg(0) => burst_length,
      tck_reset_reg_0 => u_jtag_n_10,
      trigger_tck_d1 => trigger_tck_d1,
      u_BSCANE2(0) => \shift_reg_reg_n_0_[0]\,
      \write_count_reg[15]\ => u_jtag_n_9,
      writeram_wr_reg(4) => \write_count_reg_n_0_[5]\,
      writeram_wr_reg(3) => \write_count_reg_n_0_[4]\,
      writeram_wr_reg(2) => \write_count_reg_n_0_[3]\,
      writeram_wr_reg(1) => \write_count_reg_n_0_[2]\,
      writeram_wr_reg(0) => \write_count_reg_n_0_[1]\,
      writeram_wr_reg_0 => \write_count_reg_n_0_[0]\
    );
\write_count[0]_i_3\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \write_count_reg_n_0_[0]\,
      O => \write_count[0]_i_3_n_0\
    );
\write_count_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[0]_i_2_n_15\,
      Q => \write_count_reg_n_0_[0]\,
      R => u_jtag_n_10
    );
\write_count_reg[0]_i_2\: unisim.vcomponents.CARRY8
     port map (
      CI => '0',
      CI_TOP => '0',
      CO(7) => \write_count_reg[0]_i_2_n_0\,
      CO(6) => \write_count_reg[0]_i_2_n_1\,
      CO(5) => \write_count_reg[0]_i_2_n_2\,
      CO(4) => \write_count_reg[0]_i_2_n_3\,
      CO(3) => \write_count_reg[0]_i_2_n_4\,
      CO(2) => \write_count_reg[0]_i_2_n_5\,
      CO(1) => \write_count_reg[0]_i_2_n_6\,
      CO(0) => \write_count_reg[0]_i_2_n_7\,
      DI(7 downto 0) => B"00000001",
      O(7) => \write_count_reg[0]_i_2_n_8\,
      O(6) => \write_count_reg[0]_i_2_n_9\,
      O(5) => \write_count_reg[0]_i_2_n_10\,
      O(4) => \write_count_reg[0]_i_2_n_11\,
      O(3) => \write_count_reg[0]_i_2_n_12\,
      O(2) => \write_count_reg[0]_i_2_n_13\,
      O(1) => \write_count_reg[0]_i_2_n_14\,
      O(0) => \write_count_reg[0]_i_2_n_15\,
      S(7 downto 6) => writeram_waddr_comb(1 downto 0),
      S(5) => \write_count_reg_n_0_[5]\,
      S(4) => \write_count_reg_n_0_[4]\,
      S(3) => \write_count_reg_n_0_[3]\,
      S(2) => \write_count_reg_n_0_[2]\,
      S(1) => \write_count_reg_n_0_[1]\,
      S(0) => \write_count[0]_i_3_n_0\
    );
\write_count_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[8]_i_1_n_13\,
      Q => writeram_waddr_comb(4),
      R => u_jtag_n_10
    );
\write_count_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[8]_i_1_n_12\,
      Q => writeram_waddr_comb(5),
      R => u_jtag_n_10
    );
\write_count_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[8]_i_1_n_11\,
      Q => writeram_waddr_comb(6),
      R => u_jtag_n_10
    );
\write_count_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[8]_i_1_n_10\,
      Q => writeram_waddr_comb(7),
      R => u_jtag_n_10
    );
\write_count_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[8]_i_1_n_9\,
      Q => \write_count_reg_n_0_[14]\,
      R => u_jtag_n_10
    );
\write_count_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[8]_i_1_n_8\,
      Q => \write_count_reg_n_0_[15]\,
      R => u_jtag_n_10
    );
\write_count_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[0]_i_2_n_14\,
      Q => \write_count_reg_n_0_[1]\,
      R => u_jtag_n_10
    );
\write_count_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[0]_i_2_n_13\,
      Q => \write_count_reg_n_0_[2]\,
      R => u_jtag_n_10
    );
\write_count_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[0]_i_2_n_12\,
      Q => \write_count_reg_n_0_[3]\,
      R => u_jtag_n_10
    );
\write_count_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[0]_i_2_n_11\,
      Q => \write_count_reg_n_0_[4]\,
      R => u_jtag_n_10
    );
\write_count_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[0]_i_2_n_10\,
      Q => \write_count_reg_n_0_[5]\,
      R => u_jtag_n_10
    );
\write_count_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[0]_i_2_n_9\,
      Q => writeram_waddr_comb(0),
      R => u_jtag_n_10
    );
\write_count_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[0]_i_2_n_8\,
      Q => writeram_waddr_comb(1),
      R => u_jtag_n_10
    );
\write_count_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[8]_i_1_n_15\,
      Q => writeram_waddr_comb(2),
      R => u_jtag_n_10
    );
\write_count_reg[8]_i_1\: unisim.vcomponents.CARRY8
     port map (
      CI => \write_count_reg[0]_i_2_n_0\,
      CI_TOP => '0',
      CO(7) => \NLW_write_count_reg[8]_i_1_CO_UNCONNECTED\(7),
      CO(6) => \write_count_reg[8]_i_1_n_1\,
      CO(5) => \write_count_reg[8]_i_1_n_2\,
      CO(4) => \write_count_reg[8]_i_1_n_3\,
      CO(3) => \write_count_reg[8]_i_1_n_4\,
      CO(2) => \write_count_reg[8]_i_1_n_5\,
      CO(1) => \write_count_reg[8]_i_1_n_6\,
      CO(0) => \write_count_reg[8]_i_1_n_7\,
      DI(7 downto 0) => B"00000000",
      O(7) => \write_count_reg[8]_i_1_n_8\,
      O(6) => \write_count_reg[8]_i_1_n_9\,
      O(5) => \write_count_reg[8]_i_1_n_10\,
      O(4) => \write_count_reg[8]_i_1_n_11\,
      O(3) => \write_count_reg[8]_i_1_n_12\,
      O(2) => \write_count_reg[8]_i_1_n_13\,
      O(1) => \write_count_reg[8]_i_1_n_14\,
      O(0) => \write_count_reg[8]_i_1_n_15\,
      S(7) => \write_count_reg_n_0_[15]\,
      S(6) => \write_count_reg_n_0_[14]\,
      S(5 downto 0) => writeram_waddr_comb(7 downto 2)
    );
\write_count_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_en,
      D => \write_count_reg[8]_i_1_n_14\,
      Q => writeram_waddr_comb(3),
      R => u_jtag_n_10
    );
\write_word_count[0]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => write_word_count_reg(0),
      O => \p_0_in__1\(0)
    );
\write_word_count[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => write_word_count_reg(0),
      I1 => write_word_count_reg(1),
      O => \p_0_in__1\(1)
    );
\write_word_count[2]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"6A"
    )
        port map (
      I0 => write_word_count_reg(2),
      I1 => write_word_count_reg(0),
      I2 => write_word_count_reg(1),
      O => \p_0_in__1\(2)
    );
\write_word_count[3]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6AAA"
    )
        port map (
      I0 => write_word_count_reg(3),
      I1 => write_word_count_reg(1),
      I2 => write_word_count_reg(0),
      I3 => write_word_count_reg(2),
      O => \p_0_in__1\(3)
    );
\write_word_count[4]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFF8000"
    )
        port map (
      I0 => write_word_count_reg(2),
      I1 => write_word_count_reg(0),
      I2 => write_word_count_reg(1),
      I3 => write_word_count_reg(3),
      I4 => write_word_count_reg(4),
      O => \p_0_in__1\(4)
    );
\write_word_count[5]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"6AAAAAAAAAAAAAAA"
    )
        port map (
      I0 => write_word_count_reg(5),
      I1 => write_word_count_reg(2),
      I2 => write_word_count_reg(0),
      I3 => write_word_count_reg(1),
      I4 => write_word_count_reg(3),
      I5 => write_word_count_reg(4),
      O => \p_0_in__1\(5)
    );
\write_word_count[6]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"6A"
    )
        port map (
      I0 => write_word_count_reg(6),
      I1 => \write_word_count[7]_i_4_n_0\,
      I2 => write_word_count_reg(5),
      O => \p_0_in__1\(6)
    );
\write_word_count[7]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"AAAB"
    )
        port map (
      I0 => aclk_reset,
      I1 => \state_write_reg_n_0_[1]\,
      I2 => \state_write_reg_n_0_[0]\,
      I3 => \state_write_reg_n_0_[2]\,
      O => write_word_count
    );
\write_word_count[7]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => axi4m_wready,
      I1 => \^axi4m_wvalid\,
      O => writeram_raddr1
    );
\write_word_count[7]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6AAA"
    )
        port map (
      I0 => write_word_count_reg(7),
      I1 => write_word_count_reg(5),
      I2 => \write_word_count[7]_i_4_n_0\,
      I3 => write_word_count_reg(6),
      O => \p_0_in__1\(7)
    );
\write_word_count[7]_i_4\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"80000000"
    )
        port map (
      I0 => write_word_count_reg(4),
      I1 => write_word_count_reg(3),
      I2 => write_word_count_reg(1),
      I3 => write_word_count_reg(0),
      I4 => write_word_count_reg(2),
      O => \write_word_count[7]_i_4_n_0\
    );
\write_word_count_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => writeram_raddr1,
      D => \p_0_in__1\(0),
      Q => write_word_count_reg(0),
      R => write_word_count
    );
\write_word_count_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => writeram_raddr1,
      D => \p_0_in__1\(1),
      Q => write_word_count_reg(1),
      R => write_word_count
    );
\write_word_count_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => writeram_raddr1,
      D => \p_0_in__1\(2),
      Q => write_word_count_reg(2),
      R => write_word_count
    );
\write_word_count_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => writeram_raddr1,
      D => \p_0_in__1\(3),
      Q => write_word_count_reg(3),
      R => write_word_count
    );
\write_word_count_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => writeram_raddr1,
      D => \p_0_in__1\(4),
      Q => write_word_count_reg(4),
      R => write_word_count
    );
\write_word_count_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => writeram_raddr1,
      D => \p_0_in__1\(5),
      Q => write_word_count_reg(5),
      R => write_word_count
    );
\write_word_count_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => writeram_raddr1,
      D => \p_0_in__1\(6),
      Q => write_word_count_reg(6),
      R => write_word_count
    );
\write_word_count_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => writeram_raddr1,
      D => \p_0_in__1\(7),
      Q => write_word_count_reg(7),
      R => write_word_count
    );
writeram: entity work.BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram_0
     port map (
      CLK => tck,
      D(7 downto 1) => writeram_raddr(7 downto 1),
      D(0) => writeram_n_71,
      Q(7 downto 0) => writeram_waddr(7 downto 0),
      WEBWE(0) => writeram_wr,
      aclk => aclk,
      axi4m_wdata(63 downto 0) => axi4m_wdata(63 downto 0),
      axi4m_wready => axi4m_wready,
      ram_reg_0(63) => \shift_in_reg_reg_n_0_[63]\,
      ram_reg_0(62) => \shift_in_reg_reg_n_0_[62]\,
      ram_reg_0(61) => \shift_in_reg_reg_n_0_[61]\,
      ram_reg_0(60) => \shift_in_reg_reg_n_0_[60]\,
      ram_reg_0(59) => \shift_in_reg_reg_n_0_[59]\,
      ram_reg_0(58) => \shift_in_reg_reg_n_0_[58]\,
      ram_reg_0(57) => \shift_in_reg_reg_n_0_[57]\,
      ram_reg_0(56) => \shift_in_reg_reg_n_0_[56]\,
      ram_reg_0(55) => \shift_in_reg_reg_n_0_[55]\,
      ram_reg_0(54) => \shift_in_reg_reg_n_0_[54]\,
      ram_reg_0(53) => \shift_in_reg_reg_n_0_[53]\,
      ram_reg_0(52) => \shift_in_reg_reg_n_0_[52]\,
      ram_reg_0(51) => \shift_in_reg_reg_n_0_[51]\,
      ram_reg_0(50) => \shift_in_reg_reg_n_0_[50]\,
      ram_reg_0(49) => \shift_in_reg_reg_n_0_[49]\,
      ram_reg_0(48) => \shift_in_reg_reg_n_0_[48]\,
      ram_reg_0(47) => \shift_in_reg_reg_n_0_[47]\,
      ram_reg_0(46) => \shift_in_reg_reg_n_0_[46]\,
      ram_reg_0(45) => \shift_in_reg_reg_n_0_[45]\,
      ram_reg_0(44) => \shift_in_reg_reg_n_0_[44]\,
      ram_reg_0(43) => \shift_in_reg_reg_n_0_[43]\,
      ram_reg_0(42) => \shift_in_reg_reg_n_0_[42]\,
      ram_reg_0(41) => \shift_in_reg_reg_n_0_[41]\,
      ram_reg_0(40) => \shift_in_reg_reg_n_0_[40]\,
      ram_reg_0(39) => \shift_in_reg_reg_n_0_[39]\,
      ram_reg_0(38) => \shift_in_reg_reg_n_0_[38]\,
      ram_reg_0(37) => \shift_in_reg_reg_n_0_[37]\,
      ram_reg_0(36) => \shift_in_reg_reg_n_0_[36]\,
      ram_reg_0(35) => \shift_in_reg_reg_n_0_[35]\,
      ram_reg_0(34) => \shift_in_reg_reg_n_0_[34]\,
      ram_reg_0(33) => \shift_in_reg_reg_n_0_[33]\,
      ram_reg_0(32) => \shift_in_reg_reg_n_0_[32]\,
      ram_reg_0(31) => \shift_in_reg_reg_n_0_[31]\,
      ram_reg_0(30) => \shift_in_reg_reg_n_0_[30]\,
      ram_reg_0(29) => \shift_in_reg_reg_n_0_[29]\,
      ram_reg_0(28) => \shift_in_reg_reg_n_0_[28]\,
      ram_reg_0(27) => \shift_in_reg_reg_n_0_[27]\,
      ram_reg_0(26) => \shift_in_reg_reg_n_0_[26]\,
      ram_reg_0(25) => \shift_in_reg_reg_n_0_[25]\,
      ram_reg_0(24) => \shift_in_reg_reg_n_0_[24]\,
      ram_reg_0(23) => \shift_in_reg_reg_n_0_[23]\,
      ram_reg_0(22) => \shift_in_reg_reg_n_0_[22]\,
      ram_reg_0(21) => \shift_in_reg_reg_n_0_[21]\,
      ram_reg_0(20) => \shift_in_reg_reg_n_0_[20]\,
      ram_reg_0(19) => \shift_in_reg_reg_n_0_[19]\,
      ram_reg_0(18) => \shift_in_reg_reg_n_0_[18]\,
      ram_reg_0(17) => \shift_in_reg_reg_n_0_[17]\,
      ram_reg_0(16) => \shift_in_reg_reg_n_0_[16]\,
      ram_reg_0(15) => \shift_in_reg_reg_n_0_[15]\,
      ram_reg_0(14) => \shift_in_reg_reg_n_0_[14]\,
      ram_reg_0(13) => \shift_in_reg_reg_n_0_[13]\,
      ram_reg_0(12) => \shift_in_reg_reg_n_0_[12]\,
      ram_reg_0(11) => \shift_in_reg_reg_n_0_[11]\,
      ram_reg_0(10) => \shift_in_reg_reg_n_0_[10]\,
      ram_reg_0(9) => \shift_in_reg_reg_n_0_[9]\,
      ram_reg_0(8) => \shift_in_reg_reg_n_0_[8]\,
      ram_reg_0(7) => \shift_in_reg_reg_n_0_[7]\,
      ram_reg_0(6) => \shift_in_reg_reg_n_0_[6]\,
      ram_reg_0(5) => \shift_in_reg_reg_n_0_[5]\,
      ram_reg_0(4) => \shift_in_reg_reg_n_0_[4]\,
      ram_reg_0(3) => \shift_in_reg_reg_n_0_[3]\,
      ram_reg_0(2) => \shift_in_reg_reg_n_0_[2]\,
      ram_reg_0(1) => \shift_in_reg_reg_n_0_[1]\,
      ram_reg_0(0) => \shift_in_reg_reg_n_0_[0]\,
      ram_reg_1(7 downto 0) => writeram_raddr_reg_reg(7 downto 0),
      ram_reg_2 => \^axi4m_wvalid\,
      writeram_rd => writeram_rd
    );
\writeram_raddr_reg[7]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"FE"
    )
        port map (
      I0 => \state_write_reg_n_0_[1]\,
      I1 => \state_write_reg_n_0_[0]\,
      I2 => \state_write_reg_n_0_[2]\,
      O => sel
    );
\writeram_raddr_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => sel,
      D => writeram_n_71,
      Q => writeram_raddr_reg_reg(0),
      R => state_write(1)
    );
\writeram_raddr_reg_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => sel,
      D => writeram_raddr(1),
      Q => writeram_raddr_reg_reg(1),
      R => state_write(1)
    );
\writeram_raddr_reg_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => sel,
      D => writeram_raddr(2),
      Q => writeram_raddr_reg_reg(2),
      R => state_write(1)
    );
\writeram_raddr_reg_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => sel,
      D => writeram_raddr(3),
      Q => writeram_raddr_reg_reg(3),
      R => state_write(1)
    );
\writeram_raddr_reg_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => sel,
      D => writeram_raddr(4),
      Q => writeram_raddr_reg_reg(4),
      R => state_write(1)
    );
\writeram_raddr_reg_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => sel,
      D => writeram_raddr(5),
      Q => writeram_raddr_reg_reg(5),
      R => state_write(1)
    );
\writeram_raddr_reg_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => sel,
      D => writeram_raddr(6),
      Q => writeram_raddr_reg_reg(6),
      R => state_write(1)
    );
\writeram_raddr_reg_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => sel,
      D => writeram_raddr(7),
      Q => writeram_raddr_reg_reg(7),
      R => state_write(1)
    );
writeram_rd_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFFFFF4"
    )
        port map (
      I0 => start_write_d2,
      I1 => start_write_d1,
      I2 => \state_write_reg_n_0_[1]\,
      I3 => \state_write_reg_n_0_[0]\,
      I4 => \state_write_reg_n_0_[2]\,
      O => writeram_rd_i_1_n_0
    );
writeram_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => writeram_rd_i_1_n_0,
      Q => writeram_rd,
      R => aclk_reset
    );
\writeram_waddr_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => writeram_waddr_comb(0),
      Q => writeram_waddr(0),
      R => '0'
    );
\writeram_waddr_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => writeram_waddr_comb(1),
      Q => writeram_waddr(1),
      R => '0'
    );
\writeram_waddr_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => writeram_waddr_comb(2),
      Q => writeram_waddr(2),
      R => '0'
    );
\writeram_waddr_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => writeram_waddr_comb(3),
      Q => writeram_waddr(3),
      R => '0'
    );
\writeram_waddr_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => writeram_waddr_comb(4),
      Q => writeram_waddr(4),
      R => '0'
    );
\writeram_waddr_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => writeram_waddr_comb(5),
      Q => writeram_waddr(5),
      R => '0'
    );
\writeram_waddr_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => writeram_waddr_comb(6),
      Q => writeram_waddr(6),
      R => '0'
    );
\writeram_waddr_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => shift_in_reg,
      D => writeram_waddr_comb(7),
      Q => writeram_waddr(7),
      R => '0'
    );
writeram_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => tck,
      CE => '1',
      D => u_jtag_n_9,
      Q => writeram_wr,
      R => '0'
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0 is
  port (
    axi4m_awaddr : out STD_LOGIC_VECTOR ( 31 downto 0 );
    axi4m_awprot : out STD_LOGIC_VECTOR ( 2 downto 0 );
    axi4m_awvalid : out STD_LOGIC;
    axi4m_awready : in STD_LOGIC;
    axi4m_wdata : out STD_LOGIC_VECTOR ( 63 downto 0 );
    axi4m_wvalid : out STD_LOGIC;
    axi4m_wready : in STD_LOGIC;
    axi4m_wstrb : out STD_LOGIC_VECTOR ( 7 downto 0 );
    axi4m_awid : out STD_LOGIC_VECTOR ( 0 to 0 );
    axi4m_awlen : out STD_LOGIC_VECTOR ( 7 downto 0 );
    axi4m_awsize : out STD_LOGIC_VECTOR ( 2 downto 0 );
    axi4m_awburst : out STD_LOGIC_VECTOR ( 1 downto 0 );
    axi4m_awlock : out STD_LOGIC;
    axi4m_awcache : out STD_LOGIC_VECTOR ( 3 downto 0 );
    axi4m_wlast : out STD_LOGIC;
    axi4m_awqos : out STD_LOGIC_VECTOR ( 3 downto 0 );
    axi4m_bresp : in STD_LOGIC_VECTOR ( 1 downto 0 );
    axi4m_bvalid : in STD_LOGIC;
    axi4m_bid : in STD_LOGIC_VECTOR ( 0 to 0 );
    axi4m_bready : out STD_LOGIC;
    axi4m_araddr : out STD_LOGIC_VECTOR ( 31 downto 0 );
    axi4m_arprot : out STD_LOGIC_VECTOR ( 2 downto 0 );
    axi4m_arvalid : out STD_LOGIC;
    axi4m_arready : in STD_LOGIC;
    axi4m_arid : out STD_LOGIC_VECTOR ( 0 to 0 );
    axi4m_arlen : out STD_LOGIC_VECTOR ( 7 downto 0 );
    axi4m_arsize : out STD_LOGIC_VECTOR ( 2 downto 0 );
    axi4m_arburst : out STD_LOGIC_VECTOR ( 1 downto 0 );
    axi4m_arlock : out STD_LOGIC;
    axi4m_arcache : out STD_LOGIC_VECTOR ( 3 downto 0 );
    axi4m_arqos : out STD_LOGIC_VECTOR ( 3 downto 0 );
    axi4m_rdata : in STD_LOGIC_VECTOR ( 63 downto 0 );
    axi4m_rlast : in STD_LOGIC;
    axi4m_rresp : in STD_LOGIC_VECTOR ( 1 downto 0 );
    axi4m_rvalid : in STD_LOGIC;
    axi4m_rready : out STD_LOGIC;
    axi4m_rid : in STD_LOGIC_VECTOR ( 0 to 0 );
    aclk : in STD_LOGIC;
    aresetn : in STD_LOGIC
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0 : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0 : entity is "BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0,hdlverifier_axi_manager,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0 : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0 : entity is "package_project";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0 : entity is "hdlverifier_axi_manager,Vivado 2022.1";
end BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0;

architecture STRUCTURE of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0 is
  signal \<const0>\ : STD_LOGIC;
  signal \<const1>\ : STD_LOGIC;
  signal NLW_inst_axi4m_arlock_UNCONNECTED : STD_LOGIC;
  signal NLW_inst_axi4m_awlock_UNCONNECTED : STD_LOGIC;
  signal NLW_inst_axi4m_bready_UNCONNECTED : STD_LOGIC;
  signal NLW_inst_axi4m_arcache_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_inst_axi4m_arid_UNCONNECTED : STD_LOGIC_VECTOR ( 0 to 0 );
  signal NLW_inst_axi4m_arprot_UNCONNECTED : STD_LOGIC_VECTOR ( 2 downto 0 );
  signal NLW_inst_axi4m_arqos_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_inst_axi4m_arsize_UNCONNECTED : STD_LOGIC_VECTOR ( 2 downto 0 );
  signal NLW_inst_axi4m_awcache_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_inst_axi4m_awid_UNCONNECTED : STD_LOGIC_VECTOR ( 0 to 0 );
  signal NLW_inst_axi4m_awprot_UNCONNECTED : STD_LOGIC_VECTOR ( 2 downto 0 );
  signal NLW_inst_axi4m_awqos_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_inst_axi4m_awsize_UNCONNECTED : STD_LOGIC_VECTOR ( 2 downto 0 );
  signal NLW_inst_axi4m_wstrb_UNCONNECTED : STD_LOGIC_VECTOR ( 7 downto 0 );
  attribute APP_TYPE : string;
  attribute APP_TYPE of inst : label is "4'b0000";
  attribute AXIMATER_IP_CONFIG : string;
  attribute AXIMATER_IP_CONFIG of inst : label is "32'b00000000000000000000000000000010";
  attribute AXI_ADDR_WIDTH : integer;
  attribute AXI_ADDR_WIDTH of inst : label is 32;
  attribute AXI_DATA_WIDTH : integer;
  attribute AXI_DATA_WIDTH of inst : label is 64;
  attribute AXImaster_ADDRS_64 : string;
  attribute AXImaster_ADDRS_64 of inst : label is "1'b0";
  attribute AXImaster_DATA_64 : string;
  attribute AXImaster_DATA_64 of inst : label is "1'b1";
  attribute AXImaster_IP_ID0 : string;
  attribute AXImaster_IP_ID0 of inst : label is "32'b00011111110011010000000000100000";
  attribute ID_WIDTH : integer;
  attribute ID_WIDTH of inst : label is 1;
  attribute JTAG_ID : integer;
  attribute JTAG_ID of inst : label is 3;
  attribute MAJOR_VER : string;
  attribute MAJOR_VER of inst : label is "8'b00000010";
  attribute MINOR_VER : string;
  attribute MINOR_VER of inst : label is "4'b0000";
  attribute SIGNATURE : string;
  attribute SIGNATURE of inst : label is "16'b0001111111001101";
  attribute log2AXI_DATA_WIDTH : integer;
  attribute log2AXI_DATA_WIDTH of inst : label is 6;
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of aclk : signal is "xilinx.com:signal:clock:1.0 aclk CLK";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of aclk : signal is "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF axi4m, ASSOCIATED_RESET aresetn, FREQ_HZ 200000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN BD_KCU116_AXImanager_clk_wiz_0_0_clk_out1, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of aresetn : signal is "xilinx.com:signal:reset:1.0 aresetn RST";
  attribute X_INTERFACE_PARAMETER of aresetn : signal is "XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of axi4m_arlock : signal is "xilinx.com:interface:aximm:1.0 axi4m ARLOCK";
  attribute X_INTERFACE_INFO of axi4m_arready : signal is "xilinx.com:interface:aximm:1.0 axi4m ARREADY";
  attribute X_INTERFACE_INFO of axi4m_arvalid : signal is "xilinx.com:interface:aximm:1.0 axi4m ARVALID";
  attribute X_INTERFACE_INFO of axi4m_awlock : signal is "xilinx.com:interface:aximm:1.0 axi4m AWLOCK";
  attribute X_INTERFACE_INFO of axi4m_awready : signal is "xilinx.com:interface:aximm:1.0 axi4m AWREADY";
  attribute X_INTERFACE_INFO of axi4m_awvalid : signal is "xilinx.com:interface:aximm:1.0 axi4m AWVALID";
  attribute X_INTERFACE_INFO of axi4m_bready : signal is "xilinx.com:interface:aximm:1.0 axi4m BREADY";
  attribute X_INTERFACE_INFO of axi4m_bvalid : signal is "xilinx.com:interface:aximm:1.0 axi4m BVALID";
  attribute X_INTERFACE_INFO of axi4m_rlast : signal is "xilinx.com:interface:aximm:1.0 axi4m RLAST";
  attribute X_INTERFACE_INFO of axi4m_rready : signal is "xilinx.com:interface:aximm:1.0 axi4m RREADY";
  attribute X_INTERFACE_INFO of axi4m_rvalid : signal is "xilinx.com:interface:aximm:1.0 axi4m RVALID";
  attribute X_INTERFACE_INFO of axi4m_wlast : signal is "xilinx.com:interface:aximm:1.0 axi4m WLAST";
  attribute X_INTERFACE_INFO of axi4m_wready : signal is "xilinx.com:interface:aximm:1.0 axi4m WREADY";
  attribute X_INTERFACE_INFO of axi4m_wvalid : signal is "xilinx.com:interface:aximm:1.0 axi4m WVALID";
  attribute X_INTERFACE_INFO of axi4m_araddr : signal is "xilinx.com:interface:aximm:1.0 axi4m ARADDR";
  attribute X_INTERFACE_INFO of axi4m_arburst : signal is "xilinx.com:interface:aximm:1.0 axi4m ARBURST";
  attribute X_INTERFACE_INFO of axi4m_arcache : signal is "xilinx.com:interface:aximm:1.0 axi4m ARCACHE";
  attribute X_INTERFACE_INFO of axi4m_arid : signal is "xilinx.com:interface:aximm:1.0 axi4m ARID";
  attribute X_INTERFACE_INFO of axi4m_arlen : signal is "xilinx.com:interface:aximm:1.0 axi4m ARLEN";
  attribute X_INTERFACE_INFO of axi4m_arprot : signal is "xilinx.com:interface:aximm:1.0 axi4m ARPROT";
  attribute X_INTERFACE_INFO of axi4m_arqos : signal is "xilinx.com:interface:aximm:1.0 axi4m ARQOS";
  attribute X_INTERFACE_INFO of axi4m_arsize : signal is "xilinx.com:interface:aximm:1.0 axi4m ARSIZE";
  attribute X_INTERFACE_INFO of axi4m_awaddr : signal is "xilinx.com:interface:aximm:1.0 axi4m AWADDR";
  attribute X_INTERFACE_INFO of axi4m_awburst : signal is "xilinx.com:interface:aximm:1.0 axi4m AWBURST";
  attribute X_INTERFACE_INFO of axi4m_awcache : signal is "xilinx.com:interface:aximm:1.0 axi4m AWCACHE";
  attribute X_INTERFACE_INFO of axi4m_awid : signal is "xilinx.com:interface:aximm:1.0 axi4m AWID";
  attribute X_INTERFACE_INFO of axi4m_awlen : signal is "xilinx.com:interface:aximm:1.0 axi4m AWLEN";
  attribute X_INTERFACE_INFO of axi4m_awprot : signal is "xilinx.com:interface:aximm:1.0 axi4m AWPROT";
  attribute X_INTERFACE_INFO of axi4m_awqos : signal is "xilinx.com:interface:aximm:1.0 axi4m AWQOS";
  attribute X_INTERFACE_INFO of axi4m_awsize : signal is "xilinx.com:interface:aximm:1.0 axi4m AWSIZE";
  attribute X_INTERFACE_INFO of axi4m_bid : signal is "xilinx.com:interface:aximm:1.0 axi4m BID";
  attribute X_INTERFACE_INFO of axi4m_bresp : signal is "xilinx.com:interface:aximm:1.0 axi4m BRESP";
  attribute X_INTERFACE_INFO of axi4m_rdata : signal is "xilinx.com:interface:aximm:1.0 axi4m RDATA";
  attribute X_INTERFACE_INFO of axi4m_rid : signal is "xilinx.com:interface:aximm:1.0 axi4m RID";
  attribute X_INTERFACE_PARAMETER of axi4m_rid : signal is "XIL_INTERFACENAME axi4m, DATA_WIDTH 64, PROTOCOL AXI4, FREQ_HZ 200000000, ID_WIDTH 1, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE READ_WRITE, HAS_BURST 1, HAS_LOCK 1, HAS_PROT 1, HAS_CACHE 1, HAS_QOS 1, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, SUPPORTS_NARROW_BURST 1, NUM_READ_OUTSTANDING 2, NUM_WRITE_OUTSTANDING 2, MAX_BURST_LENGTH 256, PHASE 0.0, CLK_DOMAIN BD_KCU116_AXImanager_clk_wiz_0_0_clk_out1, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of axi4m_rresp : signal is "xilinx.com:interface:aximm:1.0 axi4m RRESP";
  attribute X_INTERFACE_INFO of axi4m_wdata : signal is "xilinx.com:interface:aximm:1.0 axi4m WDATA";
  attribute X_INTERFACE_INFO of axi4m_wstrb : signal is "xilinx.com:interface:aximm:1.0 axi4m WSTRB";
begin
  axi4m_arcache(3) <= \<const0>\;
  axi4m_arcache(2) <= \<const0>\;
  axi4m_arcache(1) <= \<const0>\;
  axi4m_arcache(0) <= \<const0>\;
  axi4m_arid(0) <= \<const0>\;
  axi4m_arlock <= \<const0>\;
  axi4m_arprot(2) <= \<const0>\;
  axi4m_arprot(1) <= \<const0>\;
  axi4m_arprot(0) <= \<const0>\;
  axi4m_arqos(3) <= \<const0>\;
  axi4m_arqos(2) <= \<const0>\;
  axi4m_arqos(1) <= \<const0>\;
  axi4m_arqos(0) <= \<const0>\;
  axi4m_arsize(2) <= \<const0>\;
  axi4m_arsize(1) <= \<const1>\;
  axi4m_arsize(0) <= \<const1>\;
  axi4m_awcache(3) <= \<const0>\;
  axi4m_awcache(2) <= \<const0>\;
  axi4m_awcache(1) <= \<const0>\;
  axi4m_awcache(0) <= \<const0>\;
  axi4m_awid(0) <= \<const0>\;
  axi4m_awlock <= \<const0>\;
  axi4m_awprot(2) <= \<const0>\;
  axi4m_awprot(1) <= \<const0>\;
  axi4m_awprot(0) <= \<const0>\;
  axi4m_awqos(3) <= \<const0>\;
  axi4m_awqos(2) <= \<const0>\;
  axi4m_awqos(1) <= \<const0>\;
  axi4m_awqos(0) <= \<const0>\;
  axi4m_awsize(2) <= \<const0>\;
  axi4m_awsize(1) <= \<const1>\;
  axi4m_awsize(0) <= \<const1>\;
  axi4m_bready <= \<const1>\;
  axi4m_wstrb(7) <= \<const1>\;
  axi4m_wstrb(6) <= \<const1>\;
  axi4m_wstrb(5) <= \<const1>\;
  axi4m_wstrb(4) <= \<const1>\;
  axi4m_wstrb(3) <= \<const1>\;
  axi4m_wstrb(2) <= \<const1>\;
  axi4m_wstrb(1) <= \<const1>\;
  axi4m_wstrb(0) <= \<const1>\;
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
VCC: unisim.vcomponents.VCC
     port map (
      P => \<const1>\
    );
inst: entity work.BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager
     port map (
      aclk => aclk,
      aresetn => aresetn,
      axi4m_araddr(31 downto 0) => axi4m_araddr(31 downto 0),
      axi4m_arburst(1 downto 0) => axi4m_arburst(1 downto 0),
      axi4m_arcache(3 downto 0) => NLW_inst_axi4m_arcache_UNCONNECTED(3 downto 0),
      axi4m_arid(0) => NLW_inst_axi4m_arid_UNCONNECTED(0),
      axi4m_arlen(7 downto 0) => axi4m_arlen(7 downto 0),
      axi4m_arlock => NLW_inst_axi4m_arlock_UNCONNECTED,
      axi4m_arprot(2 downto 0) => NLW_inst_axi4m_arprot_UNCONNECTED(2 downto 0),
      axi4m_arqos(3 downto 0) => NLW_inst_axi4m_arqos_UNCONNECTED(3 downto 0),
      axi4m_arready => axi4m_arready,
      axi4m_arsize(2 downto 0) => NLW_inst_axi4m_arsize_UNCONNECTED(2 downto 0),
      axi4m_arvalid => axi4m_arvalid,
      axi4m_awaddr(31 downto 0) => axi4m_awaddr(31 downto 0),
      axi4m_awburst(1 downto 0) => axi4m_awburst(1 downto 0),
      axi4m_awcache(3 downto 0) => NLW_inst_axi4m_awcache_UNCONNECTED(3 downto 0),
      axi4m_awid(0) => NLW_inst_axi4m_awid_UNCONNECTED(0),
      axi4m_awlen(7 downto 0) => axi4m_awlen(7 downto 0),
      axi4m_awlock => NLW_inst_axi4m_awlock_UNCONNECTED,
      axi4m_awprot(2 downto 0) => NLW_inst_axi4m_awprot_UNCONNECTED(2 downto 0),
      axi4m_awqos(3 downto 0) => NLW_inst_axi4m_awqos_UNCONNECTED(3 downto 0),
      axi4m_awready => axi4m_awready,
      axi4m_awsize(2 downto 0) => NLW_inst_axi4m_awsize_UNCONNECTED(2 downto 0),
      axi4m_awvalid => axi4m_awvalid,
      axi4m_bid(0) => '0',
      axi4m_bready => NLW_inst_axi4m_bready_UNCONNECTED,
      axi4m_bresp(1 downto 0) => B"00",
      axi4m_bvalid => axi4m_bvalid,
      axi4m_rdata(63 downto 0) => axi4m_rdata(63 downto 0),
      axi4m_rid(0) => '0',
      axi4m_rlast => axi4m_rlast,
      axi4m_rready => axi4m_rready,
      axi4m_rresp(1 downto 0) => B"00",
      axi4m_rvalid => axi4m_rvalid,
      axi4m_wdata(63 downto 0) => axi4m_wdata(63 downto 0),
      axi4m_wlast => axi4m_wlast,
      axi4m_wready => axi4m_wready,
      axi4m_wstrb(7 downto 0) => NLW_inst_axi4m_wstrb_UNCONNECTED(7 downto 0),
      axi4m_wvalid => axi4m_wvalid
    );
end STRUCTURE;
