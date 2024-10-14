-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2022.1 (win64) Build 3526262 Mon Apr 18 15:48:16 MDT 2022
-- Date        : Wed Dec 20 07:23:21 2023
-- Host        : DESKTOP-H9LB9O9 running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode synth_stub
--               d:/QKD_HW_project/matlab_test/kcu116_sifting_AXImanager_v1/kcu116_sifting_AXImanager_v1.gen/sources_1/bd/BD_KCU116_AXImanager/ip/BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0/BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_stub.vhdl
-- Design      : BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xcku5p-ffvb676-2-e
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0 is
  Port ( 
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

end BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0;

architecture stub of BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "axi4m_awaddr[31:0],axi4m_awprot[2:0],axi4m_awvalid,axi4m_awready,axi4m_wdata[63:0],axi4m_wvalid,axi4m_wready,axi4m_wstrb[7:0],axi4m_awid[0:0],axi4m_awlen[7:0],axi4m_awsize[2:0],axi4m_awburst[1:0],axi4m_awlock,axi4m_awcache[3:0],axi4m_wlast,axi4m_awqos[3:0],axi4m_bresp[1:0],axi4m_bvalid,axi4m_bid[0:0],axi4m_bready,axi4m_araddr[31:0],axi4m_arprot[2:0],axi4m_arvalid,axi4m_arready,axi4m_arid[0:0],axi4m_arlen[7:0],axi4m_arsize[2:0],axi4m_arburst[1:0],axi4m_arlock,axi4m_arcache[3:0],axi4m_arqos[3:0],axi4m_rdata[63:0],axi4m_rlast,axi4m_rresp[1:0],axi4m_rvalid,axi4m_rready,axi4m_rid[0:0],aclk,aresetn";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "hdlverifier_axi_manager,Vivado 2022.1";
begin
end;
