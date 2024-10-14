// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2022.1 (win64) Build 3526262 Mon Apr 18 15:48:16 MDT 2022
// Date        : Wed Dec 20 07:23:21 2023
// Host        : DESKTOP-H9LB9O9 running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode funcsim
//               d:/QKD_HW_project/matlab_test/kcu116_sifting_AXImanager_v1/kcu116_sifting_AXImanager_v1.gen/sources_1/bd/BD_KCU116_AXImanager/ip/BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0/BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_sim_netlist.v
// Design      : BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xcku5p-ffvb676-2-e
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0,hdlverifier_axi_manager,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "package_project" *) 
(* X_CORE_INFO = "hdlverifier_axi_manager,Vivado 2022.1" *) 
(* NotValidForBitStream *)
module BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0
   (axi4m_awaddr,
    axi4m_awprot,
    axi4m_awvalid,
    axi4m_awready,
    axi4m_wdata,
    axi4m_wvalid,
    axi4m_wready,
    axi4m_wstrb,
    axi4m_awid,
    axi4m_awlen,
    axi4m_awsize,
    axi4m_awburst,
    axi4m_awlock,
    axi4m_awcache,
    axi4m_wlast,
    axi4m_awqos,
    axi4m_bresp,
    axi4m_bvalid,
    axi4m_bid,
    axi4m_bready,
    axi4m_araddr,
    axi4m_arprot,
    axi4m_arvalid,
    axi4m_arready,
    axi4m_arid,
    axi4m_arlen,
    axi4m_arsize,
    axi4m_arburst,
    axi4m_arlock,
    axi4m_arcache,
    axi4m_arqos,
    axi4m_rdata,
    axi4m_rlast,
    axi4m_rresp,
    axi4m_rvalid,
    axi4m_rready,
    axi4m_rid,
    aclk,
    aresetn);
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWADDR" *) output [31:0]axi4m_awaddr;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWPROT" *) output [2:0]axi4m_awprot;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWVALID" *) output axi4m_awvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWREADY" *) input axi4m_awready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m WDATA" *) output [63:0]axi4m_wdata;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m WVALID" *) output axi4m_wvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m WREADY" *) input axi4m_wready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m WSTRB" *) output [7:0]axi4m_wstrb;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWID" *) output [0:0]axi4m_awid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWLEN" *) output [7:0]axi4m_awlen;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWSIZE" *) output [2:0]axi4m_awsize;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWBURST" *) output [1:0]axi4m_awburst;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWLOCK" *) output axi4m_awlock;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWCACHE" *) output [3:0]axi4m_awcache;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m WLAST" *) output axi4m_wlast;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWQOS" *) output [3:0]axi4m_awqos;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m BRESP" *) input [1:0]axi4m_bresp;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m BVALID" *) input axi4m_bvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m BID" *) input [0:0]axi4m_bid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m BREADY" *) output axi4m_bready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARADDR" *) output [31:0]axi4m_araddr;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARPROT" *) output [2:0]axi4m_arprot;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARVALID" *) output axi4m_arvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARREADY" *) input axi4m_arready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARID" *) output [0:0]axi4m_arid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARLEN" *) output [7:0]axi4m_arlen;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARSIZE" *) output [2:0]axi4m_arsize;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARBURST" *) output [1:0]axi4m_arburst;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARLOCK" *) output axi4m_arlock;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARCACHE" *) output [3:0]axi4m_arcache;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARQOS" *) output [3:0]axi4m_arqos;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m RDATA" *) input [63:0]axi4m_rdata;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m RLAST" *) input axi4m_rlast;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m RRESP" *) input [1:0]axi4m_rresp;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m RVALID" *) input axi4m_rvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m RREADY" *) output axi4m_rready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m RID" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME axi4m, DATA_WIDTH 64, PROTOCOL AXI4, FREQ_HZ 200000000, ID_WIDTH 1, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE READ_WRITE, HAS_BURST 1, HAS_LOCK 1, HAS_PROT 1, HAS_CACHE 1, HAS_QOS 1, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, SUPPORTS_NARROW_BURST 1, NUM_READ_OUTSTANDING 2, NUM_WRITE_OUTSTANDING 2, MAX_BURST_LENGTH 256, PHASE 0.0, CLK_DOMAIN BD_KCU116_AXImanager_clk_wiz_0_0_clk_out1, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0, INSERT_VIP 0" *) input [0:0]axi4m_rid;
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF axi4m, ASSOCIATED_RESET aresetn, FREQ_HZ 200000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN BD_KCU116_AXImanager_clk_wiz_0_0_clk_out1, INSERT_VIP 0" *) input aclk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input aresetn;

  wire \<const0> ;
  wire \<const1> ;
  wire aclk;
  wire aresetn;
  wire [31:0]axi4m_araddr;
  wire [1:0]axi4m_arburst;
  wire [7:0]axi4m_arlen;
  wire axi4m_arready;
  wire axi4m_arvalid;
  wire [31:0]axi4m_awaddr;
  wire [1:0]axi4m_awburst;
  wire [7:0]axi4m_awlen;
  wire axi4m_awready;
  wire axi4m_awvalid;
  wire axi4m_bvalid;
  wire [63:0]axi4m_rdata;
  wire axi4m_rlast;
  wire axi4m_rready;
  wire axi4m_rvalid;
  wire [63:0]axi4m_wdata;
  wire axi4m_wlast;
  wire axi4m_wready;
  wire axi4m_wvalid;
  wire NLW_inst_axi4m_arlock_UNCONNECTED;
  wire NLW_inst_axi4m_awlock_UNCONNECTED;
  wire NLW_inst_axi4m_bready_UNCONNECTED;
  wire [3:0]NLW_inst_axi4m_arcache_UNCONNECTED;
  wire [0:0]NLW_inst_axi4m_arid_UNCONNECTED;
  wire [2:0]NLW_inst_axi4m_arprot_UNCONNECTED;
  wire [3:0]NLW_inst_axi4m_arqos_UNCONNECTED;
  wire [2:0]NLW_inst_axi4m_arsize_UNCONNECTED;
  wire [3:0]NLW_inst_axi4m_awcache_UNCONNECTED;
  wire [0:0]NLW_inst_axi4m_awid_UNCONNECTED;
  wire [2:0]NLW_inst_axi4m_awprot_UNCONNECTED;
  wire [3:0]NLW_inst_axi4m_awqos_UNCONNECTED;
  wire [2:0]NLW_inst_axi4m_awsize_UNCONNECTED;
  wire [7:0]NLW_inst_axi4m_wstrb_UNCONNECTED;

  assign axi4m_arcache[3] = \<const0> ;
  assign axi4m_arcache[2] = \<const0> ;
  assign axi4m_arcache[1] = \<const0> ;
  assign axi4m_arcache[0] = \<const0> ;
  assign axi4m_arid[0] = \<const0> ;
  assign axi4m_arlock = \<const0> ;
  assign axi4m_arprot[2] = \<const0> ;
  assign axi4m_arprot[1] = \<const0> ;
  assign axi4m_arprot[0] = \<const0> ;
  assign axi4m_arqos[3] = \<const0> ;
  assign axi4m_arqos[2] = \<const0> ;
  assign axi4m_arqos[1] = \<const0> ;
  assign axi4m_arqos[0] = \<const0> ;
  assign axi4m_arsize[2] = \<const0> ;
  assign axi4m_arsize[1] = \<const1> ;
  assign axi4m_arsize[0] = \<const1> ;
  assign axi4m_awcache[3] = \<const0> ;
  assign axi4m_awcache[2] = \<const0> ;
  assign axi4m_awcache[1] = \<const0> ;
  assign axi4m_awcache[0] = \<const0> ;
  assign axi4m_awid[0] = \<const0> ;
  assign axi4m_awlock = \<const0> ;
  assign axi4m_awprot[2] = \<const0> ;
  assign axi4m_awprot[1] = \<const0> ;
  assign axi4m_awprot[0] = \<const0> ;
  assign axi4m_awqos[3] = \<const0> ;
  assign axi4m_awqos[2] = \<const0> ;
  assign axi4m_awqos[1] = \<const0> ;
  assign axi4m_awqos[0] = \<const0> ;
  assign axi4m_awsize[2] = \<const0> ;
  assign axi4m_awsize[1] = \<const1> ;
  assign axi4m_awsize[0] = \<const1> ;
  assign axi4m_bready = \<const1> ;
  assign axi4m_wstrb[7] = \<const1> ;
  assign axi4m_wstrb[6] = \<const1> ;
  assign axi4m_wstrb[5] = \<const1> ;
  assign axi4m_wstrb[4] = \<const1> ;
  assign axi4m_wstrb[3] = \<const1> ;
  assign axi4m_wstrb[2] = \<const1> ;
  assign axi4m_wstrb[1] = \<const1> ;
  assign axi4m_wstrb[0] = \<const1> ;
  GND GND
       (.G(\<const0> ));
  VCC VCC
       (.P(\<const1> ));
  (* APP_TYPE = "4'b0000" *) 
  (* AXIMATER_IP_CONFIG = "32'b00000000000000000000000000000010" *) 
  (* AXI_ADDR_WIDTH = "32" *) 
  (* AXI_DATA_WIDTH = "64" *) 
  (* AXImaster_ADDRS_64 = "1'b0" *) 
  (* AXImaster_DATA_64 = "1'b1" *) 
  (* AXImaster_IP_ID0 = "32'b00011111110011010000000000100000" *) 
  (* ID_WIDTH = "1" *) 
  (* JTAG_ID = "3" *) 
  (* MAJOR_VER = "8'b00000010" *) 
  (* MINOR_VER = "4'b0000" *) 
  (* SIGNATURE = "16'b0001111111001101" *) 
  (* log2AXI_DATA_WIDTH = "6" *) 
  BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager inst
       (.aclk(aclk),
        .aresetn(aresetn),
        .axi4m_araddr(axi4m_araddr),
        .axi4m_arburst(axi4m_arburst),
        .axi4m_arcache(NLW_inst_axi4m_arcache_UNCONNECTED[3:0]),
        .axi4m_arid(NLW_inst_axi4m_arid_UNCONNECTED[0]),
        .axi4m_arlen(axi4m_arlen),
        .axi4m_arlock(NLW_inst_axi4m_arlock_UNCONNECTED),
        .axi4m_arprot(NLW_inst_axi4m_arprot_UNCONNECTED[2:0]),
        .axi4m_arqos(NLW_inst_axi4m_arqos_UNCONNECTED[3:0]),
        .axi4m_arready(axi4m_arready),
        .axi4m_arsize(NLW_inst_axi4m_arsize_UNCONNECTED[2:0]),
        .axi4m_arvalid(axi4m_arvalid),
        .axi4m_awaddr(axi4m_awaddr),
        .axi4m_awburst(axi4m_awburst),
        .axi4m_awcache(NLW_inst_axi4m_awcache_UNCONNECTED[3:0]),
        .axi4m_awid(NLW_inst_axi4m_awid_UNCONNECTED[0]),
        .axi4m_awlen(axi4m_awlen),
        .axi4m_awlock(NLW_inst_axi4m_awlock_UNCONNECTED),
        .axi4m_awprot(NLW_inst_axi4m_awprot_UNCONNECTED[2:0]),
        .axi4m_awqos(NLW_inst_axi4m_awqos_UNCONNECTED[3:0]),
        .axi4m_awready(axi4m_awready),
        .axi4m_awsize(NLW_inst_axi4m_awsize_UNCONNECTED[2:0]),
        .axi4m_awvalid(axi4m_awvalid),
        .axi4m_bid(1'b0),
        .axi4m_bready(NLW_inst_axi4m_bready_UNCONNECTED),
        .axi4m_bresp({1'b0,1'b0}),
        .axi4m_bvalid(axi4m_bvalid),
        .axi4m_rdata(axi4m_rdata),
        .axi4m_rid(1'b0),
        .axi4m_rlast(axi4m_rlast),
        .axi4m_rready(axi4m_rready),
        .axi4m_rresp({1'b0,1'b0}),
        .axi4m_rvalid(axi4m_rvalid),
        .axi4m_wdata(axi4m_wdata),
        .axi4m_wlast(axi4m_wlast),
        .axi4m_wready(axi4m_wready),
        .axi4m_wstrb(NLW_inst_axi4m_wstrb_UNCONNECTED[7:0]),
        .axi4m_wvalid(axi4m_wvalid));
endmodule

(* APP_TYPE = "4'b0000" *) (* AXIMATER_IP_CONFIG = "32'b00000000000000000000000000000010" *) (* AXI_ADDR_WIDTH = "32" *) 
(* AXI_DATA_WIDTH = "64" *) (* AXImaster_ADDRS_64 = "1'b0" *) (* AXImaster_DATA_64 = "1'b1" *) 
(* AXImaster_IP_ID0 = "32'b00011111110011010000000000100000" *) (* ID_WIDTH = "1" *) (* JTAG_ID = "3" *) 
(* MAJOR_VER = "8'b00000010" *) (* MINOR_VER = "4'b0000" *) (* ORIG_REF_NAME = "hdlverifier_axi_manager" *) 
(* SIGNATURE = "16'b0001111111001101" *) (* log2AXI_DATA_WIDTH = "6" *) 
module BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_axi_manager
   (axi4m_awaddr,
    axi4m_awprot,
    axi4m_awvalid,
    axi4m_awready,
    axi4m_wdata,
    axi4m_wvalid,
    axi4m_wready,
    axi4m_wstrb,
    axi4m_awid,
    axi4m_awlen,
    axi4m_awsize,
    axi4m_awburst,
    axi4m_awlock,
    axi4m_awcache,
    axi4m_wlast,
    axi4m_awqos,
    axi4m_bresp,
    axi4m_bvalid,
    axi4m_bid,
    axi4m_bready,
    axi4m_araddr,
    axi4m_arprot,
    axi4m_arvalid,
    axi4m_arready,
    axi4m_arid,
    axi4m_arlen,
    axi4m_arsize,
    axi4m_arburst,
    axi4m_arlock,
    axi4m_arcache,
    axi4m_arqos,
    axi4m_rdata,
    axi4m_rlast,
    axi4m_rresp,
    axi4m_rvalid,
    axi4m_rready,
    axi4m_rid,
    aclk,
    aresetn);
  output [31:0]axi4m_awaddr;
  output [2:0]axi4m_awprot;
  output axi4m_awvalid;
  input axi4m_awready;
  output [63:0]axi4m_wdata;
  output axi4m_wvalid;
  input axi4m_wready;
  output [7:0]axi4m_wstrb;
  output [0:0]axi4m_awid;
  output [7:0]axi4m_awlen;
  output [2:0]axi4m_awsize;
  output [1:0]axi4m_awburst;
  output axi4m_awlock;
  output [3:0]axi4m_awcache;
  output axi4m_wlast;
  output [3:0]axi4m_awqos;
  input [1:0]axi4m_bresp;
  input axi4m_bvalid;
  input [0:0]axi4m_bid;
  output axi4m_bready;
  output [31:0]axi4m_araddr;
  output [2:0]axi4m_arprot;
  output axi4m_arvalid;
  input axi4m_arready;
  output [0:0]axi4m_arid;
  output [7:0]axi4m_arlen;
  output [2:0]axi4m_arsize;
  output [1:0]axi4m_arburst;
  output axi4m_arlock;
  output [3:0]axi4m_arcache;
  output [3:0]axi4m_arqos;
  input [63:0]axi4m_rdata;
  input axi4m_rlast;
  input [1:0]axi4m_rresp;
  input axi4m_rvalid;
  output axi4m_rready;
  input [0:0]axi4m_rid;
  input aclk;
  input aresetn;

  wire \<const0> ;
  wire aclk;
  wire \aclk_cnt[3]_i_1_n_0 ;
  wire [3:0]aclk_cnt_reg;
  wire aclk_reset;
  wire aclk_reset0;
  wire address;
  wire \address_reg_n_0_[0] ;
  wire \address_reg_n_0_[10] ;
  wire \address_reg_n_0_[11] ;
  wire \address_reg_n_0_[12] ;
  wire \address_reg_n_0_[13] ;
  wire \address_reg_n_0_[14] ;
  wire \address_reg_n_0_[15] ;
  wire \address_reg_n_0_[16] ;
  wire \address_reg_n_0_[17] ;
  wire \address_reg_n_0_[18] ;
  wire \address_reg_n_0_[19] ;
  wire \address_reg_n_0_[1] ;
  wire \address_reg_n_0_[20] ;
  wire \address_reg_n_0_[21] ;
  wire \address_reg_n_0_[22] ;
  wire \address_reg_n_0_[23] ;
  wire \address_reg_n_0_[24] ;
  wire \address_reg_n_0_[25] ;
  wire \address_reg_n_0_[26] ;
  wire \address_reg_n_0_[27] ;
  wire \address_reg_n_0_[28] ;
  wire \address_reg_n_0_[29] ;
  wire \address_reg_n_0_[2] ;
  wire \address_reg_n_0_[30] ;
  wire \address_reg_n_0_[31] ;
  wire \address_reg_n_0_[3] ;
  wire \address_reg_n_0_[4] ;
  wire \address_reg_n_0_[5] ;
  wire \address_reg_n_0_[6] ;
  wire \address_reg_n_0_[7] ;
  wire \address_reg_n_0_[8] ;
  wire \address_reg_n_0_[9] ;
  wire aresetn;
  wire [31:0]axi4m_araddr;
  wire [1:0]axi4m_arburst;
  wire axi4m_arready;
  wire axi4m_arvalid;
  wire axi4m_arvalid0;
  wire axi4m_arvalid_i_1_n_0;
  wire [7:0]axi4m_awlen;
  wire axi4m_awready;
  wire axi4m_awvalid;
  wire axi4m_awvalid_i_1_n_0;
  wire axi4m_bvalid;
  wire [63:0]axi4m_rdata;
  wire axi4m_rlast;
  wire axi4m_rready;
  wire axi4m_rready_i_1_n_0;
  wire axi4m_rvalid;
  wire [63:0]axi4m_wdata;
  wire axi4m_wlast;
  wire axi4m_wlast_INST_0_i_1_n_0;
  wire axi4m_wlast_INST_0_i_2_n_0;
  wire axi4m_wlast_INST_0_i_3_n_0;
  wire axi4m_wlast_INST_0_i_4_n_0;
  wire axi4m_wready;
  wire axi4m_wvalid;
  wire axi4m_wvalid_i_1_n_0;
  wire axi4m_wvalid_i_2_n_0;
  wire \bitcount[0]_i_1_n_0 ;
  wire [5:0]bitcount_reg;
  wire burst_length;
  wire \burst_length_reg_n_0_[0] ;
  wire \burst_length_reg_n_0_[1] ;
  wire \burst_length_reg_n_0_[2] ;
  wire \burst_length_reg_n_0_[3] ;
  wire \burst_length_reg_n_0_[4] ;
  wire \burst_length_reg_n_0_[5] ;
  wire \burst_length_reg_n_0_[6] ;
  wire \burst_length_reg_n_0_[7] ;
  wire [1:0]burst_type;
  wire [62:0]data1;
  wire jtag_reset;
  wire jtag_reset_d1;
  wire [1:0]p_0_in;
  wire p_0_in11_in;
  wire [3:0]p_0_in__0;
  wire [7:0]p_0_in__1;
  wire [5:1]p_0_in__2;
  wire [7:0]p_0_in__3;
  wire read_count;
  wire \read_count[0]_i_1_n_0 ;
  wire \read_count[0]_i_4_n_0 ;
  wire [7:0]read_count_reg;
  wire \read_count_reg[0]_i_3_n_1 ;
  wire \read_count_reg[0]_i_3_n_10 ;
  wire \read_count_reg[0]_i_3_n_11 ;
  wire \read_count_reg[0]_i_3_n_12 ;
  wire \read_count_reg[0]_i_3_n_13 ;
  wire \read_count_reg[0]_i_3_n_14 ;
  wire \read_count_reg[0]_i_3_n_15 ;
  wire \read_count_reg[0]_i_3_n_2 ;
  wire \read_count_reg[0]_i_3_n_3 ;
  wire \read_count_reg[0]_i_3_n_4 ;
  wire \read_count_reg[0]_i_3_n_5 ;
  wire \read_count_reg[0]_i_3_n_6 ;
  wire \read_count_reg[0]_i_3_n_7 ;
  wire \read_count_reg[0]_i_3_n_8 ;
  wire \read_count_reg[0]_i_3_n_9 ;
  wire readram_n_64;
  wire readram_raddr;
  wire \readram_raddr[7]_i_3_n_0 ;
  wire [7:0]readram_raddr_reg;
  wire readram_rd;
  wire sel;
  wire shift_in_data;
  wire shift_in_en;
  wire shift_in_reg;
  wire \shift_in_reg_reg_n_0_[0] ;
  wire \shift_in_reg_reg_n_0_[10] ;
  wire \shift_in_reg_reg_n_0_[11] ;
  wire \shift_in_reg_reg_n_0_[12] ;
  wire \shift_in_reg_reg_n_0_[13] ;
  wire \shift_in_reg_reg_n_0_[14] ;
  wire \shift_in_reg_reg_n_0_[15] ;
  wire \shift_in_reg_reg_n_0_[16] ;
  wire \shift_in_reg_reg_n_0_[17] ;
  wire \shift_in_reg_reg_n_0_[18] ;
  wire \shift_in_reg_reg_n_0_[19] ;
  wire \shift_in_reg_reg_n_0_[1] ;
  wire \shift_in_reg_reg_n_0_[20] ;
  wire \shift_in_reg_reg_n_0_[21] ;
  wire \shift_in_reg_reg_n_0_[22] ;
  wire \shift_in_reg_reg_n_0_[23] ;
  wire \shift_in_reg_reg_n_0_[24] ;
  wire \shift_in_reg_reg_n_0_[25] ;
  wire \shift_in_reg_reg_n_0_[26] ;
  wire \shift_in_reg_reg_n_0_[27] ;
  wire \shift_in_reg_reg_n_0_[28] ;
  wire \shift_in_reg_reg_n_0_[29] ;
  wire \shift_in_reg_reg_n_0_[2] ;
  wire \shift_in_reg_reg_n_0_[30] ;
  wire \shift_in_reg_reg_n_0_[31] ;
  wire \shift_in_reg_reg_n_0_[32] ;
  wire \shift_in_reg_reg_n_0_[33] ;
  wire \shift_in_reg_reg_n_0_[34] ;
  wire \shift_in_reg_reg_n_0_[35] ;
  wire \shift_in_reg_reg_n_0_[36] ;
  wire \shift_in_reg_reg_n_0_[37] ;
  wire \shift_in_reg_reg_n_0_[38] ;
  wire \shift_in_reg_reg_n_0_[39] ;
  wire \shift_in_reg_reg_n_0_[3] ;
  wire \shift_in_reg_reg_n_0_[40] ;
  wire \shift_in_reg_reg_n_0_[41] ;
  wire \shift_in_reg_reg_n_0_[42] ;
  wire \shift_in_reg_reg_n_0_[43] ;
  wire \shift_in_reg_reg_n_0_[44] ;
  wire \shift_in_reg_reg_n_0_[45] ;
  wire \shift_in_reg_reg_n_0_[46] ;
  wire \shift_in_reg_reg_n_0_[47] ;
  wire \shift_in_reg_reg_n_0_[48] ;
  wire \shift_in_reg_reg_n_0_[49] ;
  wire \shift_in_reg_reg_n_0_[4] ;
  wire \shift_in_reg_reg_n_0_[50] ;
  wire \shift_in_reg_reg_n_0_[51] ;
  wire \shift_in_reg_reg_n_0_[52] ;
  wire \shift_in_reg_reg_n_0_[53] ;
  wire \shift_in_reg_reg_n_0_[54] ;
  wire \shift_in_reg_reg_n_0_[55] ;
  wire \shift_in_reg_reg_n_0_[56] ;
  wire \shift_in_reg_reg_n_0_[57] ;
  wire \shift_in_reg_reg_n_0_[58] ;
  wire \shift_in_reg_reg_n_0_[59] ;
  wire \shift_in_reg_reg_n_0_[5] ;
  wire \shift_in_reg_reg_n_0_[60] ;
  wire \shift_in_reg_reg_n_0_[61] ;
  wire \shift_in_reg_reg_n_0_[62] ;
  wire \shift_in_reg_reg_n_0_[63] ;
  wire \shift_in_reg_reg_n_0_[6] ;
  wire \shift_in_reg_reg_n_0_[7] ;
  wire \shift_in_reg_reg_n_0_[8] ;
  wire \shift_in_reg_reg_n_0_[9] ;
  wire shift_in_state;
  wire shift_in_state_d1;
  wire shift_in_state_d1_i_1_n_0;
  wire shift_out_en;
  wire [63:0]shift_reg;
  wire \shift_reg[63]_i_3_n_0 ;
  wire \shift_reg_reg_n_0_[0] ;
  wire start_read_aclk;
  wire start_read_reg_n_0;
  wire start_write;
  wire start_write_d1;
  wire start_write_d2;
  wire [1:1]state_write;
  wire \state_write[0]_i_1_n_0 ;
  wire \state_write[0]_i_2_n_0 ;
  wire \state_write[1]_i_1_n_0 ;
  wire \state_write[2]_i_2_n_0 ;
  wire \state_write[2]_i_3_n_0 ;
  wire \state_write_reg_n_0_[0] ;
  wire \state_write_reg_n_0_[1] ;
  wire \state_write_reg_n_0_[2] ;
  wire [1:0]status;
  wire \status[0]_i_1_n_0 ;
  wire \status[1]_i_1_n_0 ;
  wire tck;
  wire tck_reset;
  wire tmp1;
  wire tmp2;
  wire tmp3;
  wire tmp4;
  wire trigger;
  wire trigger_d1;
  wire trigger_tck;
  wire trigger_tck_d1;
  wire u_jtag_n_10;
  wire u_jtag_n_14;
  wire u_jtag_n_15;
  wire u_jtag_n_16;
  wire u_jtag_n_17;
  wire u_jtag_n_18;
  wire u_jtag_n_19;
  wire u_jtag_n_20;
  wire u_jtag_n_21;
  wire u_jtag_n_22;
  wire u_jtag_n_23;
  wire u_jtag_n_24;
  wire u_jtag_n_25;
  wire u_jtag_n_26;
  wire u_jtag_n_27;
  wire u_jtag_n_28;
  wire u_jtag_n_29;
  wire u_jtag_n_30;
  wire u_jtag_n_31;
  wire u_jtag_n_32;
  wire u_jtag_n_33;
  wire u_jtag_n_34;
  wire u_jtag_n_35;
  wire u_jtag_n_36;
  wire u_jtag_n_37;
  wire u_jtag_n_40;
  wire u_jtag_n_41;
  wire u_jtag_n_42;
  wire u_jtag_n_43;
  wire u_jtag_n_44;
  wire u_jtag_n_45;
  wire u_jtag_n_46;
  wire u_jtag_n_48;
  wire u_jtag_n_8;
  wire u_jtag_n_9;
  wire \write_count[0]_i_3_n_0 ;
  wire \write_count_reg[0]_i_2_n_0 ;
  wire \write_count_reg[0]_i_2_n_1 ;
  wire \write_count_reg[0]_i_2_n_10 ;
  wire \write_count_reg[0]_i_2_n_11 ;
  wire \write_count_reg[0]_i_2_n_12 ;
  wire \write_count_reg[0]_i_2_n_13 ;
  wire \write_count_reg[0]_i_2_n_14 ;
  wire \write_count_reg[0]_i_2_n_15 ;
  wire \write_count_reg[0]_i_2_n_2 ;
  wire \write_count_reg[0]_i_2_n_3 ;
  wire \write_count_reg[0]_i_2_n_4 ;
  wire \write_count_reg[0]_i_2_n_5 ;
  wire \write_count_reg[0]_i_2_n_6 ;
  wire \write_count_reg[0]_i_2_n_7 ;
  wire \write_count_reg[0]_i_2_n_8 ;
  wire \write_count_reg[0]_i_2_n_9 ;
  wire \write_count_reg[8]_i_1_n_1 ;
  wire \write_count_reg[8]_i_1_n_10 ;
  wire \write_count_reg[8]_i_1_n_11 ;
  wire \write_count_reg[8]_i_1_n_12 ;
  wire \write_count_reg[8]_i_1_n_13 ;
  wire \write_count_reg[8]_i_1_n_14 ;
  wire \write_count_reg[8]_i_1_n_15 ;
  wire \write_count_reg[8]_i_1_n_2 ;
  wire \write_count_reg[8]_i_1_n_3 ;
  wire \write_count_reg[8]_i_1_n_4 ;
  wire \write_count_reg[8]_i_1_n_5 ;
  wire \write_count_reg[8]_i_1_n_6 ;
  wire \write_count_reg[8]_i_1_n_7 ;
  wire \write_count_reg[8]_i_1_n_8 ;
  wire \write_count_reg[8]_i_1_n_9 ;
  wire \write_count_reg_n_0_[0] ;
  wire \write_count_reg_n_0_[14] ;
  wire \write_count_reg_n_0_[15] ;
  wire \write_count_reg_n_0_[1] ;
  wire \write_count_reg_n_0_[2] ;
  wire \write_count_reg_n_0_[3] ;
  wire \write_count_reg_n_0_[4] ;
  wire \write_count_reg_n_0_[5] ;
  wire write_word_count;
  wire \write_word_count[7]_i_4_n_0 ;
  wire [7:0]write_word_count_reg;
  wire writeram_n_71;
  wire [7:1]writeram_raddr;
  wire writeram_raddr1;
  wire writeram_raddr_reg0;
  wire [7:0]writeram_raddr_reg_reg;
  wire writeram_rd;
  wire writeram_rd_i_1_n_0;
  wire [7:0]writeram_waddr;
  wire [7:0]writeram_waddr_comb;
  wire writeram_wr;
  wire [7:7]\NLW_read_count_reg[0]_i_3_CO_UNCONNECTED ;
  wire [7:7]\NLW_write_count_reg[8]_i_1_CO_UNCONNECTED ;

  assign axi4m_arcache[3] = \<const0> ;
  assign axi4m_arcache[2] = \<const0> ;
  assign axi4m_arcache[1] = \<const0> ;
  assign axi4m_arcache[0] = \<const0> ;
  assign axi4m_arid[0] = \<const0> ;
  assign axi4m_arlen[7:0] = axi4m_awlen;
  assign axi4m_arlock = \<const0> ;
  assign axi4m_arprot[2] = \<const0> ;
  assign axi4m_arprot[1] = \<const0> ;
  assign axi4m_arprot[0] = \<const0> ;
  assign axi4m_arqos[3] = \<const0> ;
  assign axi4m_arqos[2] = \<const0> ;
  assign axi4m_arqos[1] = \<const0> ;
  assign axi4m_arqos[0] = \<const0> ;
  assign axi4m_arsize[2] = \<const0> ;
  assign axi4m_arsize[1] = \<const0> ;
  assign axi4m_arsize[0] = \<const0> ;
  assign axi4m_awaddr[31:0] = axi4m_araddr;
  assign axi4m_awburst[1:0] = axi4m_arburst;
  assign axi4m_awcache[3] = \<const0> ;
  assign axi4m_awcache[2] = \<const0> ;
  assign axi4m_awcache[1] = \<const0> ;
  assign axi4m_awcache[0] = \<const0> ;
  assign axi4m_awid[0] = \<const0> ;
  assign axi4m_awlock = \<const0> ;
  assign axi4m_awprot[2] = \<const0> ;
  assign axi4m_awprot[1] = \<const0> ;
  assign axi4m_awprot[0] = \<const0> ;
  assign axi4m_awqos[3] = \<const0> ;
  assign axi4m_awqos[2] = \<const0> ;
  assign axi4m_awqos[1] = \<const0> ;
  assign axi4m_awqos[0] = \<const0> ;
  assign axi4m_awsize[2] = \<const0> ;
  assign axi4m_awsize[1] = \<const0> ;
  assign axi4m_awsize[0] = \<const0> ;
  assign axi4m_bready = \<const0> ;
  assign axi4m_wstrb[7] = \<const0> ;
  assign axi4m_wstrb[6] = \<const0> ;
  assign axi4m_wstrb[5] = \<const0> ;
  assign axi4m_wstrb[4] = \<const0> ;
  assign axi4m_wstrb[3] = \<const0> ;
  assign axi4m_wstrb[2] = \<const0> ;
  assign axi4m_wstrb[1] = \<const0> ;
  assign axi4m_wstrb[0] = \<const0> ;
  GND GND
       (.G(\<const0> ));
  (* SOFT_HLUTNM = "soft_lutpair70" *) 
  LUT1 #(
    .INIT(2'h1)) 
    \aclk_cnt[0]_i_1 
       (.I0(aclk_cnt_reg[0]),
        .O(p_0_in__0[0]));
  (* SOFT_HLUTNM = "soft_lutpair70" *) 
  LUT2 #(
    .INIT(4'h6)) 
    \aclk_cnt[1]_i_1 
       (.I0(aclk_cnt_reg[1]),
        .I1(aclk_cnt_reg[0]),
        .O(p_0_in__0[1]));
  (* SOFT_HLUTNM = "soft_lutpair64" *) 
  LUT3 #(
    .INIT(8'h6A)) 
    \aclk_cnt[2]_i_1 
       (.I0(aclk_cnt_reg[2]),
        .I1(aclk_cnt_reg[0]),
        .I2(aclk_cnt_reg[1]),
        .O(p_0_in__0[2]));
  LUT4 #(
    .INIT(16'h7FFF)) 
    \aclk_cnt[3]_i_1 
       (.I0(aclk_cnt_reg[3]),
        .I1(aclk_cnt_reg[1]),
        .I2(aclk_cnt_reg[0]),
        .I3(aclk_cnt_reg[2]),
        .O(\aclk_cnt[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair64" *) 
  LUT4 #(
    .INIT(16'h6AAA)) 
    \aclk_cnt[3]_i_2 
       (.I0(aclk_cnt_reg[3]),
        .I1(aclk_cnt_reg[1]),
        .I2(aclk_cnt_reg[0]),
        .I3(aclk_cnt_reg[2]),
        .O(p_0_in__0[3]));
  FDCE \aclk_cnt_reg[0] 
       (.C(aclk),
        .CE(\aclk_cnt[3]_i_1_n_0 ),
        .CLR(tmp3),
        .D(p_0_in__0[0]),
        .Q(aclk_cnt_reg[0]));
  FDCE \aclk_cnt_reg[1] 
       (.C(aclk),
        .CE(\aclk_cnt[3]_i_1_n_0 ),
        .CLR(tmp3),
        .D(p_0_in__0[1]),
        .Q(aclk_cnt_reg[1]));
  FDCE \aclk_cnt_reg[2] 
       (.C(aclk),
        .CE(\aclk_cnt[3]_i_1_n_0 ),
        .CLR(tmp3),
        .D(p_0_in__0[2]),
        .Q(aclk_cnt_reg[2]));
  FDCE \aclk_cnt_reg[3] 
       (.C(aclk),
        .CE(\aclk_cnt[3]_i_1_n_0 ),
        .CLR(tmp3),
        .D(p_0_in__0[3]),
        .Q(aclk_cnt_reg[3]));
  LUT2 #(
    .INIT(4'hB)) 
    aclk_reset_i_1
       (.I0(tmp2),
        .I1(tmp4),
        .O(aclk_reset0));
  FDRE aclk_reset_reg
       (.C(aclk),
        .CE(1'b1),
        .D(aclk_reset0),
        .Q(aclk_reset),
        .R(1'b0));
  FDRE \address_aclk_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[0] ),
        .Q(axi4m_araddr[0]),
        .R(1'b0));
  FDRE \address_aclk_reg[10] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[10] ),
        .Q(axi4m_araddr[10]),
        .R(1'b0));
  FDRE \address_aclk_reg[11] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[11] ),
        .Q(axi4m_araddr[11]),
        .R(1'b0));
  FDRE \address_aclk_reg[12] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[12] ),
        .Q(axi4m_araddr[12]),
        .R(1'b0));
  FDRE \address_aclk_reg[13] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[13] ),
        .Q(axi4m_araddr[13]),
        .R(1'b0));
  FDRE \address_aclk_reg[14] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[14] ),
        .Q(axi4m_araddr[14]),
        .R(1'b0));
  FDRE \address_aclk_reg[15] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[15] ),
        .Q(axi4m_araddr[15]),
        .R(1'b0));
  FDRE \address_aclk_reg[16] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[16] ),
        .Q(axi4m_araddr[16]),
        .R(1'b0));
  FDRE \address_aclk_reg[17] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[17] ),
        .Q(axi4m_araddr[17]),
        .R(1'b0));
  FDRE \address_aclk_reg[18] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[18] ),
        .Q(axi4m_araddr[18]),
        .R(1'b0));
  FDRE \address_aclk_reg[19] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[19] ),
        .Q(axi4m_araddr[19]),
        .R(1'b0));
  FDRE \address_aclk_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[1] ),
        .Q(axi4m_araddr[1]),
        .R(1'b0));
  FDRE \address_aclk_reg[20] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[20] ),
        .Q(axi4m_araddr[20]),
        .R(1'b0));
  FDRE \address_aclk_reg[21] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[21] ),
        .Q(axi4m_araddr[21]),
        .R(1'b0));
  FDRE \address_aclk_reg[22] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[22] ),
        .Q(axi4m_araddr[22]),
        .R(1'b0));
  FDRE \address_aclk_reg[23] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[23] ),
        .Q(axi4m_araddr[23]),
        .R(1'b0));
  FDRE \address_aclk_reg[24] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[24] ),
        .Q(axi4m_araddr[24]),
        .R(1'b0));
  FDRE \address_aclk_reg[25] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[25] ),
        .Q(axi4m_araddr[25]),
        .R(1'b0));
  FDRE \address_aclk_reg[26] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[26] ),
        .Q(axi4m_araddr[26]),
        .R(1'b0));
  FDRE \address_aclk_reg[27] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[27] ),
        .Q(axi4m_araddr[27]),
        .R(1'b0));
  FDRE \address_aclk_reg[28] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[28] ),
        .Q(axi4m_araddr[28]),
        .R(1'b0));
  FDRE \address_aclk_reg[29] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[29] ),
        .Q(axi4m_araddr[29]),
        .R(1'b0));
  FDRE \address_aclk_reg[2] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[2] ),
        .Q(axi4m_araddr[2]),
        .R(1'b0));
  FDRE \address_aclk_reg[30] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[30] ),
        .Q(axi4m_araddr[30]),
        .R(1'b0));
  FDRE \address_aclk_reg[31] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[31] ),
        .Q(axi4m_araddr[31]),
        .R(1'b0));
  FDRE \address_aclk_reg[3] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[3] ),
        .Q(axi4m_araddr[3]),
        .R(1'b0));
  FDRE \address_aclk_reg[4] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[4] ),
        .Q(axi4m_araddr[4]),
        .R(1'b0));
  FDRE \address_aclk_reg[5] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[5] ),
        .Q(axi4m_araddr[5]),
        .R(1'b0));
  FDRE \address_aclk_reg[6] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[6] ),
        .Q(axi4m_araddr[6]),
        .R(1'b0));
  FDRE \address_aclk_reg[7] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[7] ),
        .Q(axi4m_araddr[7]),
        .R(1'b0));
  FDRE \address_aclk_reg[8] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[8] ),
        .Q(axi4m_araddr[8]),
        .R(1'b0));
  FDRE \address_aclk_reg[9] 
       (.C(aclk),
        .CE(1'b1),
        .D(\address_reg_n_0_[9] ),
        .Q(axi4m_araddr[9]),
        .R(1'b0));
  FDRE \address_reg[0] 
       (.C(tck),
        .CE(address),
        .D(p_0_in11_in),
        .Q(\address_reg_n_0_[0] ),
        .R(tck_reset));
  FDRE \address_reg[10] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_37),
        .Q(\address_reg_n_0_[10] ),
        .R(tck_reset));
  FDRE \address_reg[11] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_36),
        .Q(\address_reg_n_0_[11] ),
        .R(tck_reset));
  FDRE \address_reg[12] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_35),
        .Q(\address_reg_n_0_[12] ),
        .R(tck_reset));
  FDRE \address_reg[13] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_34),
        .Q(\address_reg_n_0_[13] ),
        .R(tck_reset));
  FDRE \address_reg[14] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_33),
        .Q(\address_reg_n_0_[14] ),
        .R(tck_reset));
  FDRE \address_reg[15] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_32),
        .Q(\address_reg_n_0_[15] ),
        .R(tck_reset));
  FDRE \address_reg[16] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_31),
        .Q(\address_reg_n_0_[16] ),
        .R(tck_reset));
  FDRE \address_reg[17] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_30),
        .Q(\address_reg_n_0_[17] ),
        .R(tck_reset));
  FDRE \address_reg[18] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_29),
        .Q(\address_reg_n_0_[18] ),
        .R(tck_reset));
  FDRE \address_reg[19] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_28),
        .Q(\address_reg_n_0_[19] ),
        .R(tck_reset));
  FDRE \address_reg[1] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_46),
        .Q(\address_reg_n_0_[1] ),
        .R(tck_reset));
  FDRE \address_reg[20] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_27),
        .Q(\address_reg_n_0_[20] ),
        .R(tck_reset));
  FDRE \address_reg[21] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_26),
        .Q(\address_reg_n_0_[21] ),
        .R(tck_reset));
  FDRE \address_reg[22] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_25),
        .Q(\address_reg_n_0_[22] ),
        .R(tck_reset));
  FDRE \address_reg[23] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_24),
        .Q(\address_reg_n_0_[23] ),
        .R(tck_reset));
  FDRE \address_reg[24] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_23),
        .Q(\address_reg_n_0_[24] ),
        .R(tck_reset));
  FDRE \address_reg[25] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_22),
        .Q(\address_reg_n_0_[25] ),
        .R(tck_reset));
  FDRE \address_reg[26] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_21),
        .Q(\address_reg_n_0_[26] ),
        .R(tck_reset));
  FDRE \address_reg[27] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_20),
        .Q(\address_reg_n_0_[27] ),
        .R(tck_reset));
  FDRE \address_reg[28] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_19),
        .Q(\address_reg_n_0_[28] ),
        .R(tck_reset));
  FDRE \address_reg[29] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_18),
        .Q(\address_reg_n_0_[29] ),
        .R(tck_reset));
  FDRE \address_reg[2] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_45),
        .Q(\address_reg_n_0_[2] ),
        .R(tck_reset));
  FDRE \address_reg[30] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_17),
        .Q(\address_reg_n_0_[30] ),
        .R(tck_reset));
  FDRE \address_reg[31] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_16),
        .Q(\address_reg_n_0_[31] ),
        .R(tck_reset));
  FDRE \address_reg[3] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_44),
        .Q(\address_reg_n_0_[3] ),
        .R(tck_reset));
  FDRE \address_reg[4] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_43),
        .Q(\address_reg_n_0_[4] ),
        .R(tck_reset));
  FDRE \address_reg[5] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_42),
        .Q(\address_reg_n_0_[5] ),
        .R(tck_reset));
  FDRE \address_reg[6] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_41),
        .Q(\address_reg_n_0_[6] ),
        .R(tck_reset));
  FDRE \address_reg[7] 
       (.C(tck),
        .CE(address),
        .D(u_jtag_n_40),
        .Q(\address_reg_n_0_[7] ),
        .R(tck_reset));
  FDRE \address_reg[8] 
       (.C(tck),
        .CE(address),
        .D(p_0_in[0]),
        .Q(\address_reg_n_0_[8] ),
        .R(tck_reset));
  FDRE \address_reg[9] 
       (.C(tck),
        .CE(address),
        .D(p_0_in[1]),
        .Q(\address_reg_n_0_[9] ),
        .R(tck_reset));
  (* SOFT_HLUTNM = "soft_lutpair67" *) 
  LUT4 #(
    .INIT(16'h44F4)) 
    axi4m_arvalid_i_1
       (.I0(trigger_d1),
        .I1(trigger),
        .I2(axi4m_arvalid),
        .I3(axi4m_arready),
        .O(axi4m_arvalid_i_1_n_0));
  FDRE axi4m_arvalid_reg
       (.C(aclk),
        .CE(1'b1),
        .D(axi4m_arvalid_i_1_n_0),
        .Q(axi4m_arvalid),
        .R(aclk_reset));
  LUT5 #(
    .INIT(32'hCFFF0202)) 
    axi4m_awvalid_i_1
       (.I0(\state_write_reg_n_0_[0] ),
        .I1(\state_write_reg_n_0_[2] ),
        .I2(\state_write_reg_n_0_[1] ),
        .I3(axi4m_awready),
        .I4(axi4m_awvalid),
        .O(axi4m_awvalid_i_1_n_0));
  FDRE axi4m_awvalid_reg
       (.C(aclk),
        .CE(1'b1),
        .D(axi4m_awvalid_i_1_n_0),
        .Q(axi4m_awvalid),
        .R(aclk_reset));
  LUT6 #(
    .INIT(64'h00000000FF4C4C4C)) 
    axi4m_rready_i_1
       (.I0(axi4m_rlast),
        .I1(axi4m_rready),
        .I2(axi4m_rvalid),
        .I3(axi4m_arready),
        .I4(axi4m_arvalid),
        .I5(\read_count[0]_i_1_n_0 ),
        .O(axi4m_rready_i_1_n_0));
  FDRE axi4m_rready_reg
       (.C(aclk),
        .CE(1'b1),
        .D(axi4m_rready_i_1_n_0),
        .Q(axi4m_rready),
        .R(1'b0));
  LUT5 #(
    .INIT(32'h00010000)) 
    axi4m_wlast_INST_0
       (.I0(axi4m_wlast_INST_0_i_1_n_0),
        .I1(axi4m_wlast_INST_0_i_2_n_0),
        .I2(axi4m_wlast_INST_0_i_3_n_0),
        .I3(axi4m_wlast_INST_0_i_4_n_0),
        .I4(axi4m_wvalid),
        .O(axi4m_wlast));
  LUT4 #(
    .INIT(16'h6FF6)) 
    axi4m_wlast_INST_0_i_1
       (.I0(axi4m_awlen[3]),
        .I1(write_word_count_reg[3]),
        .I2(axi4m_awlen[4]),
        .I3(write_word_count_reg[4]),
        .O(axi4m_wlast_INST_0_i_1_n_0));
  LUT4 #(
    .INIT(16'h6FF6)) 
    axi4m_wlast_INST_0_i_2
       (.I0(axi4m_awlen[6]),
        .I1(write_word_count_reg[6]),
        .I2(axi4m_awlen[7]),
        .I3(write_word_count_reg[7]),
        .O(axi4m_wlast_INST_0_i_2_n_0));
  (* SOFT_HLUTNM = "soft_lutpair63" *) 
  LUT4 #(
    .INIT(16'h6FF6)) 
    axi4m_wlast_INST_0_i_3
       (.I0(axi4m_awlen[1]),
        .I1(write_word_count_reg[1]),
        .I2(axi4m_awlen[0]),
        .I3(write_word_count_reg[0]),
        .O(axi4m_wlast_INST_0_i_3_n_0));
  LUT4 #(
    .INIT(16'h6FF6)) 
    axi4m_wlast_INST_0_i_4
       (.I0(axi4m_awlen[2]),
        .I1(write_word_count_reg[2]),
        .I2(axi4m_awlen[5]),
        .I3(write_word_count_reg[5]),
        .O(axi4m_wlast_INST_0_i_4_n_0));
  LUT5 #(
    .INIT(32'hEDFF0404)) 
    axi4m_wvalid_i_1
       (.I0(\state_write_reg_n_0_[1] ),
        .I1(\state_write_reg_n_0_[0] ),
        .I2(\state_write_reg_n_0_[2] ),
        .I3(axi4m_wvalid_i_2_n_0),
        .I4(axi4m_wvalid),
        .O(axi4m_wvalid_i_1_n_0));
  LUT6 #(
    .INIT(64'h0001000000000000)) 
    axi4m_wvalid_i_2
       (.I0(axi4m_wlast_INST_0_i_4_n_0),
        .I1(axi4m_wlast_INST_0_i_3_n_0),
        .I2(axi4m_wlast_INST_0_i_2_n_0),
        .I3(axi4m_wlast_INST_0_i_1_n_0),
        .I4(axi4m_wready),
        .I5(axi4m_wvalid),
        .O(axi4m_wvalid_i_2_n_0));
  FDRE axi4m_wvalid_reg
       (.C(aclk),
        .CE(1'b1),
        .D(axi4m_wvalid_i_1_n_0),
        .Q(axi4m_wvalid),
        .R(aclk_reset));
  LUT1 #(
    .INIT(2'h1)) 
    \bitcount[0]_i_1 
       (.I0(bitcount_reg[0]),
        .O(\bitcount[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair68" *) 
  LUT2 #(
    .INIT(4'h6)) 
    \bitcount[1]_i_1 
       (.I0(bitcount_reg[1]),
        .I1(bitcount_reg[0]),
        .O(p_0_in__2[1]));
  (* SOFT_HLUTNM = "soft_lutpair68" *) 
  LUT3 #(
    .INIT(8'h6A)) 
    \bitcount[2]_i_1 
       (.I0(bitcount_reg[2]),
        .I1(bitcount_reg[1]),
        .I2(bitcount_reg[0]),
        .O(p_0_in__2[2]));
  (* SOFT_HLUTNM = "soft_lutpair57" *) 
  LUT4 #(
    .INIT(16'h6AAA)) 
    \bitcount[3]_i_1 
       (.I0(bitcount_reg[3]),
        .I1(bitcount_reg[0]),
        .I2(bitcount_reg[1]),
        .I3(bitcount_reg[2]),
        .O(p_0_in__2[3]));
  (* SOFT_HLUTNM = "soft_lutpair57" *) 
  LUT5 #(
    .INIT(32'h7FFF8000)) 
    \bitcount[4]_i_1 
       (.I0(bitcount_reg[2]),
        .I1(bitcount_reg[1]),
        .I2(bitcount_reg[0]),
        .I3(bitcount_reg[3]),
        .I4(bitcount_reg[4]),
        .O(p_0_in__2[4]));
  LUT6 #(
    .INIT(64'h6AAAAAAAAAAAAAAA)) 
    \bitcount[5]_i_2 
       (.I0(bitcount_reg[5]),
        .I1(bitcount_reg[2]),
        .I2(bitcount_reg[1]),
        .I3(bitcount_reg[0]),
        .I4(bitcount_reg[3]),
        .I5(bitcount_reg[4]),
        .O(p_0_in__2[5]));
  FDRE \bitcount_reg[0] 
       (.C(tck),
        .CE(shift_out_en),
        .D(\bitcount[0]_i_1_n_0 ),
        .Q(bitcount_reg[0]),
        .R(u_jtag_n_48));
  FDRE \bitcount_reg[1] 
       (.C(tck),
        .CE(shift_out_en),
        .D(p_0_in__2[1]),
        .Q(bitcount_reg[1]),
        .R(u_jtag_n_48));
  FDRE \bitcount_reg[2] 
       (.C(tck),
        .CE(shift_out_en),
        .D(p_0_in__2[2]),
        .Q(bitcount_reg[2]),
        .R(u_jtag_n_48));
  FDRE \bitcount_reg[3] 
       (.C(tck),
        .CE(shift_out_en),
        .D(p_0_in__2[3]),
        .Q(bitcount_reg[3]),
        .R(u_jtag_n_48));
  FDRE \bitcount_reg[4] 
       (.C(tck),
        .CE(shift_out_en),
        .D(p_0_in__2[4]),
        .Q(bitcount_reg[4]),
        .R(u_jtag_n_48));
  FDRE \bitcount_reg[5] 
       (.C(tck),
        .CE(shift_out_en),
        .D(p_0_in__2[5]),
        .Q(bitcount_reg[5]),
        .R(u_jtag_n_48));
  FDRE \burst_length_aclk_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .D(\burst_length_reg_n_0_[0] ),
        .Q(axi4m_awlen[0]),
        .R(1'b0));
  FDRE \burst_length_aclk_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .D(\burst_length_reg_n_0_[1] ),
        .Q(axi4m_awlen[1]),
        .R(1'b0));
  FDRE \burst_length_aclk_reg[2] 
       (.C(aclk),
        .CE(1'b1),
        .D(\burst_length_reg_n_0_[2] ),
        .Q(axi4m_awlen[2]),
        .R(1'b0));
  FDRE \burst_length_aclk_reg[3] 
       (.C(aclk),
        .CE(1'b1),
        .D(\burst_length_reg_n_0_[3] ),
        .Q(axi4m_awlen[3]),
        .R(1'b0));
  FDRE \burst_length_aclk_reg[4] 
       (.C(aclk),
        .CE(1'b1),
        .D(\burst_length_reg_n_0_[4] ),
        .Q(axi4m_awlen[4]),
        .R(1'b0));
  FDRE \burst_length_aclk_reg[5] 
       (.C(aclk),
        .CE(1'b1),
        .D(\burst_length_reg_n_0_[5] ),
        .Q(axi4m_awlen[5]),
        .R(1'b0));
  FDRE \burst_length_aclk_reg[6] 
       (.C(aclk),
        .CE(1'b1),
        .D(\burst_length_reg_n_0_[6] ),
        .Q(axi4m_awlen[6]),
        .R(1'b0));
  FDRE \burst_length_aclk_reg[7] 
       (.C(aclk),
        .CE(1'b1),
        .D(\burst_length_reg_n_0_[7] ),
        .Q(axi4m_awlen[7]),
        .R(1'b0));
  FDRE \burst_length_reg[0] 
       (.C(tck),
        .CE(burst_length),
        .D(p_0_in11_in),
        .Q(\burst_length_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \burst_length_reg[1] 
       (.C(tck),
        .CE(burst_length),
        .D(u_jtag_n_46),
        .Q(\burst_length_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \burst_length_reg[2] 
       (.C(tck),
        .CE(burst_length),
        .D(u_jtag_n_45),
        .Q(\burst_length_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \burst_length_reg[3] 
       (.C(tck),
        .CE(burst_length),
        .D(u_jtag_n_44),
        .Q(\burst_length_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \burst_length_reg[4] 
       (.C(tck),
        .CE(burst_length),
        .D(u_jtag_n_43),
        .Q(\burst_length_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \burst_length_reg[5] 
       (.C(tck),
        .CE(burst_length),
        .D(u_jtag_n_42),
        .Q(\burst_length_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \burst_length_reg[6] 
       (.C(tck),
        .CE(burst_length),
        .D(u_jtag_n_41),
        .Q(\burst_length_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \burst_length_reg[7] 
       (.C(tck),
        .CE(burst_length),
        .D(u_jtag_n_40),
        .Q(\burst_length_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \burst_type_aclk_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .D(burst_type[0]),
        .Q(axi4m_arburst[0]),
        .R(1'b0));
  FDRE \burst_type_aclk_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .D(burst_type[1]),
        .Q(axi4m_arburst[1]),
        .R(1'b0));
  FDRE \burst_type_reg[0] 
       (.C(tck),
        .CE(burst_length),
        .D(p_0_in[0]),
        .Q(burst_type[0]),
        .R(1'b0));
  FDRE \burst_type_reg[1] 
       (.C(tck),
        .CE(burst_length),
        .D(p_0_in[1]),
        .Q(burst_type[1]),
        .R(1'b0));
  FDRE jtag_reset_d1_reg
       (.C(tck),
        .CE(1'b1),
        .D(jtag_reset),
        .Q(jtag_reset_d1),
        .R(1'b0));
  LUT3 #(
    .INIT(8'hBA)) 
    \read_count[0]_i_1 
       (.I0(aclk_reset),
        .I1(trigger_d1),
        .I2(trigger),
        .O(\read_count[0]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'h7000)) 
    \read_count[0]_i_2 
       (.I0(axi4m_arvalid),
        .I1(axi4m_arready),
        .I2(axi4m_rready),
        .I3(axi4m_rvalid),
        .O(read_count));
  LUT1 #(
    .INIT(2'h1)) 
    \read_count[0]_i_4 
       (.I0(read_count_reg[0]),
        .O(\read_count[0]_i_4_n_0 ));
  FDRE \read_count_reg[0] 
       (.C(aclk),
        .CE(read_count),
        .D(\read_count_reg[0]_i_3_n_15 ),
        .Q(read_count_reg[0]),
        .R(\read_count[0]_i_1_n_0 ));
  CARRY8 \read_count_reg[0]_i_3 
       (.CI(1'b0),
        .CI_TOP(1'b0),
        .CO({\NLW_read_count_reg[0]_i_3_CO_UNCONNECTED [7],\read_count_reg[0]_i_3_n_1 ,\read_count_reg[0]_i_3_n_2 ,\read_count_reg[0]_i_3_n_3 ,\read_count_reg[0]_i_3_n_4 ,\read_count_reg[0]_i_3_n_5 ,\read_count_reg[0]_i_3_n_6 ,\read_count_reg[0]_i_3_n_7 }),
        .DI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .O({\read_count_reg[0]_i_3_n_8 ,\read_count_reg[0]_i_3_n_9 ,\read_count_reg[0]_i_3_n_10 ,\read_count_reg[0]_i_3_n_11 ,\read_count_reg[0]_i_3_n_12 ,\read_count_reg[0]_i_3_n_13 ,\read_count_reg[0]_i_3_n_14 ,\read_count_reg[0]_i_3_n_15 }),
        .S({read_count_reg[7:1],\read_count[0]_i_4_n_0 }));
  FDRE \read_count_reg[1] 
       (.C(aclk),
        .CE(read_count),
        .D(\read_count_reg[0]_i_3_n_14 ),
        .Q(read_count_reg[1]),
        .R(\read_count[0]_i_1_n_0 ));
  FDRE \read_count_reg[2] 
       (.C(aclk),
        .CE(read_count),
        .D(\read_count_reg[0]_i_3_n_13 ),
        .Q(read_count_reg[2]),
        .R(\read_count[0]_i_1_n_0 ));
  FDRE \read_count_reg[3] 
       (.C(aclk),
        .CE(read_count),
        .D(\read_count_reg[0]_i_3_n_12 ),
        .Q(read_count_reg[3]),
        .R(\read_count[0]_i_1_n_0 ));
  FDRE \read_count_reg[4] 
       (.C(aclk),
        .CE(read_count),
        .D(\read_count_reg[0]_i_3_n_11 ),
        .Q(read_count_reg[4]),
        .R(\read_count[0]_i_1_n_0 ));
  FDRE \read_count_reg[5] 
       (.C(aclk),
        .CE(read_count),
        .D(\read_count_reg[0]_i_3_n_10 ),
        .Q(read_count_reg[5]),
        .R(\read_count[0]_i_1_n_0 ));
  FDRE \read_count_reg[6] 
       (.C(aclk),
        .CE(read_count),
        .D(\read_count_reg[0]_i_3_n_9 ),
        .Q(read_count_reg[6]),
        .R(\read_count[0]_i_1_n_0 ));
  FDRE \read_count_reg[7] 
       (.C(aclk),
        .CE(read_count),
        .D(\read_count_reg[0]_i_3_n_8 ),
        .Q(read_count_reg[7]),
        .R(\read_count[0]_i_1_n_0 ));
  BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram readram
       (.ADDRBWRADDR(read_count_reg),
        .D(shift_reg),
        .Q(readram_raddr_reg),
        .aclk(aclk),
        .axi4m_rdata(axi4m_rdata),
        .axi4m_rvalid(axi4m_rvalid),
        .\bitcount_reg[3] (readram_n_64),
        .ram_reg_0(bitcount_reg[5:1]),
        .readram_rd(readram_rd),
        .\shift_reg_reg[0] (\shift_reg[63]_i_3_n_0 ),
        .\shift_reg_reg[62] (data1),
        .tck(tck));
  (* SOFT_HLUTNM = "soft_lutpair69" *) 
  LUT1 #(
    .INIT(2'h1)) 
    \readram_raddr[0]_i_1 
       (.I0(readram_raddr_reg[0]),
        .O(p_0_in__3[0]));
  (* SOFT_HLUTNM = "soft_lutpair69" *) 
  LUT2 #(
    .INIT(4'h6)) 
    \readram_raddr[1]_i_1 
       (.I0(readram_raddr_reg[0]),
        .I1(readram_raddr_reg[1]),
        .O(p_0_in__3[1]));
  (* SOFT_HLUTNM = "soft_lutpair62" *) 
  LUT3 #(
    .INIT(8'h6A)) 
    \readram_raddr[2]_i_1 
       (.I0(readram_raddr_reg[2]),
        .I1(readram_raddr_reg[0]),
        .I2(readram_raddr_reg[1]),
        .O(p_0_in__3[2]));
  (* SOFT_HLUTNM = "soft_lutpair62" *) 
  LUT4 #(
    .INIT(16'h6AAA)) 
    \readram_raddr[3]_i_1 
       (.I0(readram_raddr_reg[3]),
        .I1(readram_raddr_reg[1]),
        .I2(readram_raddr_reg[0]),
        .I3(readram_raddr_reg[2]),
        .O(p_0_in__3[3]));
  (* SOFT_HLUTNM = "soft_lutpair56" *) 
  LUT5 #(
    .INIT(32'h7FFF8000)) 
    \readram_raddr[4]_i_1 
       (.I0(readram_raddr_reg[2]),
        .I1(readram_raddr_reg[0]),
        .I2(readram_raddr_reg[1]),
        .I3(readram_raddr_reg[3]),
        .I4(readram_raddr_reg[4]),
        .O(p_0_in__3[4]));
  LUT6 #(
    .INIT(64'h6AAAAAAAAAAAAAAA)) 
    \readram_raddr[5]_i_1 
       (.I0(readram_raddr_reg[5]),
        .I1(readram_raddr_reg[2]),
        .I2(readram_raddr_reg[0]),
        .I3(readram_raddr_reg[1]),
        .I4(readram_raddr_reg[3]),
        .I5(readram_raddr_reg[4]),
        .O(p_0_in__3[5]));
  (* SOFT_HLUTNM = "soft_lutpair61" *) 
  LUT3 #(
    .INIT(8'h6A)) 
    \readram_raddr[6]_i_1 
       (.I0(readram_raddr_reg[6]),
        .I1(\readram_raddr[7]_i_3_n_0 ),
        .I2(readram_raddr_reg[5]),
        .O(p_0_in__3[6]));
  (* SOFT_HLUTNM = "soft_lutpair61" *) 
  LUT4 #(
    .INIT(16'h6AAA)) 
    \readram_raddr[7]_i_2 
       (.I0(readram_raddr_reg[7]),
        .I1(readram_raddr_reg[5]),
        .I2(\readram_raddr[7]_i_3_n_0 ),
        .I3(readram_raddr_reg[6]),
        .O(p_0_in__3[7]));
  (* SOFT_HLUTNM = "soft_lutpair56" *) 
  LUT5 #(
    .INIT(32'h80000000)) 
    \readram_raddr[7]_i_3 
       (.I0(readram_raddr_reg[4]),
        .I1(readram_raddr_reg[3]),
        .I2(readram_raddr_reg[1]),
        .I3(readram_raddr_reg[0]),
        .I4(readram_raddr_reg[2]),
        .O(\readram_raddr[7]_i_3_n_0 ));
  FDRE \readram_raddr_reg[0] 
       (.C(tck),
        .CE(readram_raddr),
        .D(p_0_in__3[0]),
        .Q(readram_raddr_reg[0]),
        .R(u_jtag_n_48));
  FDRE \readram_raddr_reg[1] 
       (.C(tck),
        .CE(readram_raddr),
        .D(p_0_in__3[1]),
        .Q(readram_raddr_reg[1]),
        .R(u_jtag_n_48));
  FDRE \readram_raddr_reg[2] 
       (.C(tck),
        .CE(readram_raddr),
        .D(p_0_in__3[2]),
        .Q(readram_raddr_reg[2]),
        .R(u_jtag_n_48));
  FDRE \readram_raddr_reg[3] 
       (.C(tck),
        .CE(readram_raddr),
        .D(p_0_in__3[3]),
        .Q(readram_raddr_reg[3]),
        .R(u_jtag_n_48));
  FDRE \readram_raddr_reg[4] 
       (.C(tck),
        .CE(readram_raddr),
        .D(p_0_in__3[4]),
        .Q(readram_raddr_reg[4]),
        .R(u_jtag_n_48));
  FDRE \readram_raddr_reg[5] 
       (.C(tck),
        .CE(readram_raddr),
        .D(p_0_in__3[5]),
        .Q(readram_raddr_reg[5]),
        .R(u_jtag_n_48));
  FDRE \readram_raddr_reg[6] 
       (.C(tck),
        .CE(readram_raddr),
        .D(p_0_in__3[6]),
        .Q(readram_raddr_reg[6]),
        .R(u_jtag_n_48));
  FDRE \readram_raddr_reg[7] 
       (.C(tck),
        .CE(readram_raddr),
        .D(p_0_in__3[7]),
        .Q(readram_raddr_reg[7]),
        .R(u_jtag_n_48));
  FDRE \shift_in_reg_reg[0] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[1] ),
        .Q(\shift_in_reg_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[10] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[11] ),
        .Q(\shift_in_reg_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[11] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[12] ),
        .Q(\shift_in_reg_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[12] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[13] ),
        .Q(\shift_in_reg_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[13] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[14] ),
        .Q(\shift_in_reg_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[14] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[15] ),
        .Q(\shift_in_reg_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[15] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[16] ),
        .Q(\shift_in_reg_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[16] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[17] ),
        .Q(\shift_in_reg_reg_n_0_[16] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[17] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[18] ),
        .Q(\shift_in_reg_reg_n_0_[17] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[18] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[19] ),
        .Q(\shift_in_reg_reg_n_0_[18] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[19] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[20] ),
        .Q(\shift_in_reg_reg_n_0_[19] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[1] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[2] ),
        .Q(\shift_in_reg_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[20] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[21] ),
        .Q(\shift_in_reg_reg_n_0_[20] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[21] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[22] ),
        .Q(\shift_in_reg_reg_n_0_[21] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[22] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[23] ),
        .Q(\shift_in_reg_reg_n_0_[22] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[23] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[24] ),
        .Q(\shift_in_reg_reg_n_0_[23] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[24] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[25] ),
        .Q(\shift_in_reg_reg_n_0_[24] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[25] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[26] ),
        .Q(\shift_in_reg_reg_n_0_[25] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[26] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[27] ),
        .Q(\shift_in_reg_reg_n_0_[26] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[27] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[28] ),
        .Q(\shift_in_reg_reg_n_0_[27] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[28] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[29] ),
        .Q(\shift_in_reg_reg_n_0_[28] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[29] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[30] ),
        .Q(\shift_in_reg_reg_n_0_[29] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[2] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[3] ),
        .Q(\shift_in_reg_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[30] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[31] ),
        .Q(\shift_in_reg_reg_n_0_[30] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[31] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[32] ),
        .Q(\shift_in_reg_reg_n_0_[31] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[32] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[33] ),
        .Q(\shift_in_reg_reg_n_0_[32] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[33] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[34] ),
        .Q(\shift_in_reg_reg_n_0_[33] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[34] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[35] ),
        .Q(\shift_in_reg_reg_n_0_[34] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[35] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[36] ),
        .Q(\shift_in_reg_reg_n_0_[35] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[36] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[37] ),
        .Q(\shift_in_reg_reg_n_0_[36] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[37] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[38] ),
        .Q(\shift_in_reg_reg_n_0_[37] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[38] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[39] ),
        .Q(\shift_in_reg_reg_n_0_[38] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[39] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[40] ),
        .Q(\shift_in_reg_reg_n_0_[39] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[3] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[4] ),
        .Q(\shift_in_reg_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[40] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[41] ),
        .Q(\shift_in_reg_reg_n_0_[40] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[41] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[42] ),
        .Q(\shift_in_reg_reg_n_0_[41] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[42] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[43] ),
        .Q(\shift_in_reg_reg_n_0_[42] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[43] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[44] ),
        .Q(\shift_in_reg_reg_n_0_[43] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[44] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[45] ),
        .Q(\shift_in_reg_reg_n_0_[44] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[45] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[46] ),
        .Q(\shift_in_reg_reg_n_0_[45] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[46] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[47] ),
        .Q(\shift_in_reg_reg_n_0_[46] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[47] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[48] ),
        .Q(\shift_in_reg_reg_n_0_[47] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[48] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[49] ),
        .Q(\shift_in_reg_reg_n_0_[48] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[49] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[50] ),
        .Q(\shift_in_reg_reg_n_0_[49] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[4] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[5] ),
        .Q(\shift_in_reg_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[50] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[51] ),
        .Q(\shift_in_reg_reg_n_0_[50] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[51] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[52] ),
        .Q(\shift_in_reg_reg_n_0_[51] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[52] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[53] ),
        .Q(\shift_in_reg_reg_n_0_[52] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[53] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[54] ),
        .Q(\shift_in_reg_reg_n_0_[53] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[54] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[55] ),
        .Q(\shift_in_reg_reg_n_0_[54] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[55] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[56] ),
        .Q(\shift_in_reg_reg_n_0_[55] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[56] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[57] ),
        .Q(\shift_in_reg_reg_n_0_[56] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[57] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[58] ),
        .Q(\shift_in_reg_reg_n_0_[57] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[58] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[59] ),
        .Q(\shift_in_reg_reg_n_0_[58] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[59] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[60] ),
        .Q(\shift_in_reg_reg_n_0_[59] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[5] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[6] ),
        .Q(\shift_in_reg_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[60] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[61] ),
        .Q(\shift_in_reg_reg_n_0_[60] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[61] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[62] ),
        .Q(\shift_in_reg_reg_n_0_[61] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[62] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[63] ),
        .Q(\shift_in_reg_reg_n_0_[62] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[63] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(shift_in_data),
        .Q(\shift_in_reg_reg_n_0_[63] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[6] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[7] ),
        .Q(\shift_in_reg_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[7] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[8] ),
        .Q(\shift_in_reg_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[8] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[9] ),
        .Q(\shift_in_reg_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \shift_in_reg_reg[9] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(\shift_in_reg_reg_n_0_[10] ),
        .Q(\shift_in_reg_reg_n_0_[9] ),
        .R(1'b0));
  LUT1 #(
    .INIT(2'h1)) 
    shift_in_state_d1_i_1
       (.I0(tck_reset),
        .O(shift_in_state_d1_i_1_n_0));
  FDRE shift_in_state_d1_reg
       (.C(tck),
        .CE(shift_in_state_d1_i_1_n_0),
        .D(shift_in_state),
        .Q(shift_in_state_d1),
        .R(1'b0));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFFFFFFD)) 
    \shift_reg[63]_i_3 
       (.I0(bitcount_reg[0]),
        .I1(bitcount_reg[5]),
        .I2(bitcount_reg[1]),
        .I3(bitcount_reg[2]),
        .I4(bitcount_reg[4]),
        .I5(bitcount_reg[3]),
        .O(\shift_reg[63]_i_3_n_0 ));
  FDRE \shift_reg_reg[0] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[0]),
        .Q(\shift_reg_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \shift_reg_reg[10] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[10]),
        .Q(data1[9]),
        .R(1'b0));
  FDRE \shift_reg_reg[11] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[11]),
        .Q(data1[10]),
        .R(1'b0));
  FDRE \shift_reg_reg[12] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[12]),
        .Q(data1[11]),
        .R(1'b0));
  FDRE \shift_reg_reg[13] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[13]),
        .Q(data1[12]),
        .R(1'b0));
  FDRE \shift_reg_reg[14] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[14]),
        .Q(data1[13]),
        .R(1'b0));
  FDRE \shift_reg_reg[15] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[15]),
        .Q(data1[14]),
        .R(1'b0));
  FDRE \shift_reg_reg[16] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[16]),
        .Q(data1[15]),
        .R(1'b0));
  FDRE \shift_reg_reg[17] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[17]),
        .Q(data1[16]),
        .R(1'b0));
  FDRE \shift_reg_reg[18] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[18]),
        .Q(data1[17]),
        .R(1'b0));
  FDRE \shift_reg_reg[19] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[19]),
        .Q(data1[18]),
        .R(1'b0));
  FDRE \shift_reg_reg[1] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[1]),
        .Q(data1[0]),
        .R(1'b0));
  FDRE \shift_reg_reg[20] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[20]),
        .Q(data1[19]),
        .R(1'b0));
  FDRE \shift_reg_reg[21] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[21]),
        .Q(data1[20]),
        .R(1'b0));
  FDRE \shift_reg_reg[22] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[22]),
        .Q(data1[21]),
        .R(1'b0));
  FDRE \shift_reg_reg[23] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[23]),
        .Q(data1[22]),
        .R(1'b0));
  FDRE \shift_reg_reg[24] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[24]),
        .Q(data1[23]),
        .R(1'b0));
  FDRE \shift_reg_reg[25] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[25]),
        .Q(data1[24]),
        .R(1'b0));
  FDRE \shift_reg_reg[26] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[26]),
        .Q(data1[25]),
        .R(1'b0));
  FDRE \shift_reg_reg[27] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[27]),
        .Q(data1[26]),
        .R(1'b0));
  FDRE \shift_reg_reg[28] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[28]),
        .Q(data1[27]),
        .R(1'b0));
  FDRE \shift_reg_reg[29] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[29]),
        .Q(data1[28]),
        .R(1'b0));
  FDRE \shift_reg_reg[2] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[2]),
        .Q(data1[1]),
        .R(1'b0));
  FDRE \shift_reg_reg[30] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[30]),
        .Q(data1[29]),
        .R(1'b0));
  FDRE \shift_reg_reg[31] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[31]),
        .Q(data1[30]),
        .R(1'b0));
  FDRE \shift_reg_reg[32] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[32]),
        .Q(data1[31]),
        .R(1'b0));
  FDRE \shift_reg_reg[33] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[33]),
        .Q(data1[32]),
        .R(1'b0));
  FDRE \shift_reg_reg[34] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[34]),
        .Q(data1[33]),
        .R(1'b0));
  FDRE \shift_reg_reg[35] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[35]),
        .Q(data1[34]),
        .R(1'b0));
  FDRE \shift_reg_reg[36] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[36]),
        .Q(data1[35]),
        .R(1'b0));
  FDRE \shift_reg_reg[37] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[37]),
        .Q(data1[36]),
        .R(1'b0));
  FDRE \shift_reg_reg[38] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[38]),
        .Q(data1[37]),
        .R(1'b0));
  FDRE \shift_reg_reg[39] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[39]),
        .Q(data1[38]),
        .R(1'b0));
  FDRE \shift_reg_reg[3] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[3]),
        .Q(data1[2]),
        .R(1'b0));
  FDRE \shift_reg_reg[40] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[40]),
        .Q(data1[39]),
        .R(1'b0));
  FDRE \shift_reg_reg[41] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[41]),
        .Q(data1[40]),
        .R(1'b0));
  FDRE \shift_reg_reg[42] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[42]),
        .Q(data1[41]),
        .R(1'b0));
  FDRE \shift_reg_reg[43] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[43]),
        .Q(data1[42]),
        .R(1'b0));
  FDRE \shift_reg_reg[44] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[44]),
        .Q(data1[43]),
        .R(1'b0));
  FDRE \shift_reg_reg[45] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[45]),
        .Q(data1[44]),
        .R(1'b0));
  FDRE \shift_reg_reg[46] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[46]),
        .Q(data1[45]),
        .R(1'b0));
  FDRE \shift_reg_reg[47] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[47]),
        .Q(data1[46]),
        .R(1'b0));
  FDRE \shift_reg_reg[48] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[48]),
        .Q(data1[47]),
        .R(1'b0));
  FDRE \shift_reg_reg[49] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[49]),
        .Q(data1[48]),
        .R(1'b0));
  FDRE \shift_reg_reg[4] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[4]),
        .Q(data1[3]),
        .R(1'b0));
  FDRE \shift_reg_reg[50] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[50]),
        .Q(data1[49]),
        .R(1'b0));
  FDRE \shift_reg_reg[51] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[51]),
        .Q(data1[50]),
        .R(1'b0));
  FDRE \shift_reg_reg[52] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[52]),
        .Q(data1[51]),
        .R(1'b0));
  FDRE \shift_reg_reg[53] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[53]),
        .Q(data1[52]),
        .R(1'b0));
  FDRE \shift_reg_reg[54] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[54]),
        .Q(data1[53]),
        .R(1'b0));
  FDRE \shift_reg_reg[55] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[55]),
        .Q(data1[54]),
        .R(1'b0));
  FDRE \shift_reg_reg[56] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[56]),
        .Q(data1[55]),
        .R(1'b0));
  FDRE \shift_reg_reg[57] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[57]),
        .Q(data1[56]),
        .R(1'b0));
  FDRE \shift_reg_reg[58] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[58]),
        .Q(data1[57]),
        .R(1'b0));
  FDRE \shift_reg_reg[59] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[59]),
        .Q(data1[58]),
        .R(1'b0));
  FDRE \shift_reg_reg[5] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[5]),
        .Q(data1[4]),
        .R(1'b0));
  FDRE \shift_reg_reg[60] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[60]),
        .Q(data1[59]),
        .R(1'b0));
  FDRE \shift_reg_reg[61] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[61]),
        .Q(data1[60]),
        .R(1'b0));
  FDRE \shift_reg_reg[62] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[62]),
        .Q(data1[61]),
        .R(1'b0));
  FDRE \shift_reg_reg[63] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[63]),
        .Q(data1[62]),
        .R(1'b0));
  FDRE \shift_reg_reg[6] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[6]),
        .Q(data1[5]),
        .R(1'b0));
  FDRE \shift_reg_reg[7] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[7]),
        .Q(data1[6]),
        .R(1'b0));
  FDRE \shift_reg_reg[8] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[8]),
        .Q(data1[7]),
        .R(1'b0));
  FDRE \shift_reg_reg[9] 
       (.C(tck),
        .CE(u_jtag_n_8),
        .D(shift_reg[9]),
        .Q(data1[8]),
        .R(1'b0));
  FDRE start_read_aclk_reg
       (.C(aclk),
        .CE(1'b1),
        .D(start_read_reg_n_0),
        .Q(start_read_aclk),
        .R(1'b0));
  FDRE start_read_reg
       (.C(tck),
        .CE(1'b1),
        .D(u_jtag_n_15),
        .Q(start_read_reg_n_0),
        .R(tck_reset));
  FDRE start_write_d1_reg
       (.C(aclk),
        .CE(1'b1),
        .D(start_write),
        .Q(start_write_d1),
        .R(1'b0));
  FDRE start_write_d2_reg
       (.C(aclk),
        .CE(1'b1),
        .D(start_write_d1),
        .Q(start_write_d2),
        .R(1'b0));
  FDRE start_write_reg
       (.C(tck),
        .CE(1'b1),
        .D(u_jtag_n_14),
        .Q(start_write),
        .R(tck_reset));
  LUT4 #(
    .INIT(16'hFBF8)) 
    \state_write[0]_i_1 
       (.I0(\state_write[0]_i_2_n_0 ),
        .I1(\state_write[2]_i_2_n_0 ),
        .I2(writeram_raddr_reg0),
        .I3(\state_write_reg_n_0_[0] ),
        .O(\state_write[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair58" *) 
  LUT5 #(
    .INIT(32'h550045AA)) 
    \state_write[0]_i_2 
       (.I0(\state_write_reg_n_0_[2] ),
        .I1(axi4m_wvalid_i_2_n_0),
        .I2(axi4m_awready),
        .I3(\state_write_reg_n_0_[1] ),
        .I4(\state_write_reg_n_0_[0] ),
        .O(\state_write[0]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair59" *) 
  LUT5 #(
    .INIT(32'h00000100)) 
    \state_write[0]_i_3 
       (.I0(\state_write_reg_n_0_[1] ),
        .I1(\state_write_reg_n_0_[0] ),
        .I2(\state_write_reg_n_0_[2] ),
        .I3(start_write_d1),
        .I4(start_write_d2),
        .O(writeram_raddr_reg0));
  LUT4 #(
    .INIT(16'h1014)) 
    \state_write[1]_i_1 
       (.I0(\state_write_reg_n_0_[2] ),
        .I1(\state_write_reg_n_0_[1] ),
        .I2(\state_write_reg_n_0_[0] ),
        .I3(axi4m_awready),
        .O(\state_write[1]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hAAAAAAAAAAAAAABA)) 
    \state_write[2]_i_1 
       (.I0(aclk_reset),
        .I1(start_write_d2),
        .I2(start_write_d1),
        .I3(\state_write_reg_n_0_[2] ),
        .I4(\state_write_reg_n_0_[0] ),
        .I5(\state_write_reg_n_0_[1] ),
        .O(state_write));
  LUT6 #(
    .INIT(64'h0A0EFFC00A0E0FC0)) 
    \state_write[2]_i_2 
       (.I0(axi4m_awready),
        .I1(axi4m_wvalid_i_2_n_0),
        .I2(\state_write_reg_n_0_[2] ),
        .I3(\state_write_reg_n_0_[0] ),
        .I4(\state_write_reg_n_0_[1] ),
        .I5(axi4m_bvalid),
        .O(\state_write[2]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair58" *) 
  LUT4 #(
    .INIT(16'h5242)) 
    \state_write[2]_i_3 
       (.I0(\state_write_reg_n_0_[2] ),
        .I1(\state_write_reg_n_0_[0] ),
        .I2(\state_write_reg_n_0_[1] ),
        .I3(axi4m_awready),
        .O(\state_write[2]_i_3_n_0 ));
  FDRE \state_write_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .D(\state_write[0]_i_1_n_0 ),
        .Q(\state_write_reg_n_0_[0] ),
        .R(aclk_reset));
  FDRE \state_write_reg[1] 
       (.C(aclk),
        .CE(\state_write[2]_i_2_n_0 ),
        .D(\state_write[1]_i_1_n_0 ),
        .Q(\state_write_reg_n_0_[1] ),
        .R(state_write));
  FDRE \state_write_reg[2] 
       (.C(aclk),
        .CE(\state_write[2]_i_2_n_0 ),
        .D(\state_write[2]_i_3_n_0 ),
        .Q(\state_write_reg_n_0_[2] ),
        .R(state_write));
  LUT6 #(
    .INIT(64'h5555555510000000)) 
    \status[0]_i_1 
       (.I0(\read_count[0]_i_1_n_0 ),
        .I1(axi4m_arvalid0),
        .I2(axi4m_rlast),
        .I3(axi4m_rready),
        .I4(axi4m_rvalid),
        .I5(status[0]),
        .O(\status[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair67" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \status[0]_i_2 
       (.I0(axi4m_arready),
        .I1(axi4m_arvalid),
        .O(axi4m_arvalid0));
  LUT6 #(
    .INIT(64'h0000FFCF00000080)) 
    \status[1]_i_1 
       (.I0(axi4m_bvalid),
        .I1(\state_write_reg_n_0_[2] ),
        .I2(\state_write_reg_n_0_[0] ),
        .I3(\state_write_reg_n_0_[1] ),
        .I4(aclk_reset),
        .I5(status[1]),
        .O(\status[1]_i_1_n_0 ));
  FDRE \status_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .D(\status[0]_i_1_n_0 ),
        .Q(status[0]),
        .R(1'b0));
  FDRE \status_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .D(\status[1]_i_1_n_0 ),
        .Q(status[1]),
        .R(1'b0));
  FDRE tck_reset_reg
       (.C(tck),
        .CE(1'b1),
        .D(jtag_reset_d1),
        .Q(tck_reset),
        .R(1'b0));
  FDRE tmp1_reg
       (.C(aclk),
        .CE(1'b1),
        .D(tck_reset),
        .Q(tmp1),
        .R(1'b0));
  FDRE tmp2_reg
       (.C(aclk),
        .CE(1'b1),
        .D(tmp1),
        .Q(tmp2),
        .R(1'b0));
  FDRE tmp3_reg
       (.C(aclk),
        .CE(1'b1),
        .D(aresetn),
        .Q(tmp3),
        .R(1'b0));
  FDRE tmp4_reg
       (.C(aclk),
        .CE(1'b1),
        .D(tmp3),
        .Q(tmp4),
        .R(1'b0));
  FDRE trigger_d1_reg
       (.C(aclk),
        .CE(1'b1),
        .D(trigger),
        .Q(trigger_d1),
        .R(1'b0));
  FDRE trigger_reg
       (.C(aclk),
        .CE(1'b1),
        .D(start_read_aclk),
        .Q(trigger),
        .R(1'b0));
  FDRE trigger_tck_d1_reg
       (.C(tck),
        .CE(shift_in_state_d1_i_1_n_0),
        .D(trigger_tck),
        .Q(trigger_tck_d1),
        .R(1'b0));
  FDRE trigger_tck_reg
       (.C(tck),
        .CE(shift_in_state_d1_i_1_n_0),
        .D(trigger_d1),
        .Q(trigger_tck),
        .R(1'b0));
  BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_core u_jtag
       (.D(shift_in_data),
        .E(address),
        .Q(aclk_cnt_reg[3]),
        .S({\write_count_reg_n_0_[15] ,\write_count_reg_n_0_[14] }),
        .SR(u_jtag_n_48),
        .aresetn(aresetn),
        .jtag_reset(jtag_reset),
        .ram_reg(readram_n_64),
        .ram_reg_0(bitcount_reg[0]),
        .\readram_raddr_reg[0] (\shift_reg[63]_i_3_n_0 ),
        .readram_rd(readram_rd),
        .\reg_wdata_reg[0]_0 (u_jtag_n_15),
        .\reg_wdata_reg[31]_0 ({u_jtag_n_16,u_jtag_n_17,u_jtag_n_18,u_jtag_n_19,u_jtag_n_20,u_jtag_n_21,u_jtag_n_22,u_jtag_n_23,u_jtag_n_24,u_jtag_n_25,u_jtag_n_26,u_jtag_n_27,u_jtag_n_28,u_jtag_n_29,u_jtag_n_30,u_jtag_n_31,u_jtag_n_32,u_jtag_n_33,u_jtag_n_34,u_jtag_n_35,u_jtag_n_36,u_jtag_n_37,p_0_in,u_jtag_n_40,u_jtag_n_41,u_jtag_n_42,u_jtag_n_43,u_jtag_n_44,u_jtag_n_45,u_jtag_n_46,p_0_in11_in}),
        .shift_in_en(shift_in_en),
        .shift_in_en_reg_0(shift_in_reg),
        .shift_in_state(shift_in_state),
        .shift_in_state_d1(shift_in_state_d1),
        .shift_in_state_d1_reg(u_jtag_n_14),
        .shift_out_en(shift_out_en),
        .shift_out_en_reg_0(u_jtag_n_8),
        .shift_out_en_reg_1(readram_raddr),
        .start_read_reg(start_read_reg_n_0),
        .start_write(start_write),
        .status(status),
        .tck(tck),
        .tck_reset(tck_reset),
        .tck_reset_reg(burst_length),
        .tck_reset_reg_0(u_jtag_n_10),
        .trigger_tck_d1(trigger_tck_d1),
        .u_BSCANE2(\shift_reg_reg_n_0_[0] ),
        .\write_count_reg[15] (u_jtag_n_9),
        .writeram_wr_reg({\write_count_reg_n_0_[5] ,\write_count_reg_n_0_[4] ,\write_count_reg_n_0_[3] ,\write_count_reg_n_0_[2] ,\write_count_reg_n_0_[1] }),
        .writeram_wr_reg_0(\write_count_reg_n_0_[0] ));
  LUT1 #(
    .INIT(2'h1)) 
    \write_count[0]_i_3 
       (.I0(\write_count_reg_n_0_[0] ),
        .O(\write_count[0]_i_3_n_0 ));
  FDRE \write_count_reg[0] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[0]_i_2_n_15 ),
        .Q(\write_count_reg_n_0_[0] ),
        .R(u_jtag_n_10));
  CARRY8 \write_count_reg[0]_i_2 
       (.CI(1'b0),
        .CI_TOP(1'b0),
        .CO({\write_count_reg[0]_i_2_n_0 ,\write_count_reg[0]_i_2_n_1 ,\write_count_reg[0]_i_2_n_2 ,\write_count_reg[0]_i_2_n_3 ,\write_count_reg[0]_i_2_n_4 ,\write_count_reg[0]_i_2_n_5 ,\write_count_reg[0]_i_2_n_6 ,\write_count_reg[0]_i_2_n_7 }),
        .DI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .O({\write_count_reg[0]_i_2_n_8 ,\write_count_reg[0]_i_2_n_9 ,\write_count_reg[0]_i_2_n_10 ,\write_count_reg[0]_i_2_n_11 ,\write_count_reg[0]_i_2_n_12 ,\write_count_reg[0]_i_2_n_13 ,\write_count_reg[0]_i_2_n_14 ,\write_count_reg[0]_i_2_n_15 }),
        .S({writeram_waddr_comb[1:0],\write_count_reg_n_0_[5] ,\write_count_reg_n_0_[4] ,\write_count_reg_n_0_[3] ,\write_count_reg_n_0_[2] ,\write_count_reg_n_0_[1] ,\write_count[0]_i_3_n_0 }));
  FDRE \write_count_reg[10] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[8]_i_1_n_13 ),
        .Q(writeram_waddr_comb[4]),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[11] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[8]_i_1_n_12 ),
        .Q(writeram_waddr_comb[5]),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[12] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[8]_i_1_n_11 ),
        .Q(writeram_waddr_comb[6]),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[13] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[8]_i_1_n_10 ),
        .Q(writeram_waddr_comb[7]),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[14] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[8]_i_1_n_9 ),
        .Q(\write_count_reg_n_0_[14] ),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[15] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[8]_i_1_n_8 ),
        .Q(\write_count_reg_n_0_[15] ),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[1] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[0]_i_2_n_14 ),
        .Q(\write_count_reg_n_0_[1] ),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[2] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[0]_i_2_n_13 ),
        .Q(\write_count_reg_n_0_[2] ),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[3] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[0]_i_2_n_12 ),
        .Q(\write_count_reg_n_0_[3] ),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[4] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[0]_i_2_n_11 ),
        .Q(\write_count_reg_n_0_[4] ),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[5] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[0]_i_2_n_10 ),
        .Q(\write_count_reg_n_0_[5] ),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[6] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[0]_i_2_n_9 ),
        .Q(writeram_waddr_comb[0]),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[7] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[0]_i_2_n_8 ),
        .Q(writeram_waddr_comb[1]),
        .R(u_jtag_n_10));
  FDRE \write_count_reg[8] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[8]_i_1_n_15 ),
        .Q(writeram_waddr_comb[2]),
        .R(u_jtag_n_10));
  CARRY8 \write_count_reg[8]_i_1 
       (.CI(\write_count_reg[0]_i_2_n_0 ),
        .CI_TOP(1'b0),
        .CO({\NLW_write_count_reg[8]_i_1_CO_UNCONNECTED [7],\write_count_reg[8]_i_1_n_1 ,\write_count_reg[8]_i_1_n_2 ,\write_count_reg[8]_i_1_n_3 ,\write_count_reg[8]_i_1_n_4 ,\write_count_reg[8]_i_1_n_5 ,\write_count_reg[8]_i_1_n_6 ,\write_count_reg[8]_i_1_n_7 }),
        .DI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .O({\write_count_reg[8]_i_1_n_8 ,\write_count_reg[8]_i_1_n_9 ,\write_count_reg[8]_i_1_n_10 ,\write_count_reg[8]_i_1_n_11 ,\write_count_reg[8]_i_1_n_12 ,\write_count_reg[8]_i_1_n_13 ,\write_count_reg[8]_i_1_n_14 ,\write_count_reg[8]_i_1_n_15 }),
        .S({\write_count_reg_n_0_[15] ,\write_count_reg_n_0_[14] ,writeram_waddr_comb[7:2]}));
  FDRE \write_count_reg[9] 
       (.C(tck),
        .CE(shift_in_en),
        .D(\write_count_reg[8]_i_1_n_14 ),
        .Q(writeram_waddr_comb[3]),
        .R(u_jtag_n_10));
  LUT1 #(
    .INIT(2'h1)) 
    \write_word_count[0]_i_1 
       (.I0(write_word_count_reg[0]),
        .O(p_0_in__1[0]));
  (* SOFT_HLUTNM = "soft_lutpair65" *) 
  LUT2 #(
    .INIT(4'h6)) 
    \write_word_count[1]_i_1 
       (.I0(write_word_count_reg[0]),
        .I1(write_word_count_reg[1]),
        .O(p_0_in__1[1]));
  (* SOFT_HLUTNM = "soft_lutpair63" *) 
  LUT3 #(
    .INIT(8'h6A)) 
    \write_word_count[2]_i_1 
       (.I0(write_word_count_reg[2]),
        .I1(write_word_count_reg[0]),
        .I2(write_word_count_reg[1]),
        .O(p_0_in__1[2]));
  (* SOFT_HLUTNM = "soft_lutpair65" *) 
  LUT4 #(
    .INIT(16'h6AAA)) 
    \write_word_count[3]_i_1 
       (.I0(write_word_count_reg[3]),
        .I1(write_word_count_reg[1]),
        .I2(write_word_count_reg[0]),
        .I3(write_word_count_reg[2]),
        .O(p_0_in__1[3]));
  (* SOFT_HLUTNM = "soft_lutpair60" *) 
  LUT5 #(
    .INIT(32'h7FFF8000)) 
    \write_word_count[4]_i_1 
       (.I0(write_word_count_reg[2]),
        .I1(write_word_count_reg[0]),
        .I2(write_word_count_reg[1]),
        .I3(write_word_count_reg[3]),
        .I4(write_word_count_reg[4]),
        .O(p_0_in__1[4]));
  LUT6 #(
    .INIT(64'h6AAAAAAAAAAAAAAA)) 
    \write_word_count[5]_i_1 
       (.I0(write_word_count_reg[5]),
        .I1(write_word_count_reg[2]),
        .I2(write_word_count_reg[0]),
        .I3(write_word_count_reg[1]),
        .I4(write_word_count_reg[3]),
        .I5(write_word_count_reg[4]),
        .O(p_0_in__1[5]));
  (* SOFT_HLUTNM = "soft_lutpair66" *) 
  LUT3 #(
    .INIT(8'h6A)) 
    \write_word_count[6]_i_1 
       (.I0(write_word_count_reg[6]),
        .I1(\write_word_count[7]_i_4_n_0 ),
        .I2(write_word_count_reg[5]),
        .O(p_0_in__1[6]));
  LUT4 #(
    .INIT(16'hAAAB)) 
    \write_word_count[7]_i_1 
       (.I0(aclk_reset),
        .I1(\state_write_reg_n_0_[1] ),
        .I2(\state_write_reg_n_0_[0] ),
        .I3(\state_write_reg_n_0_[2] ),
        .O(write_word_count));
  LUT2 #(
    .INIT(4'h8)) 
    \write_word_count[7]_i_2 
       (.I0(axi4m_wready),
        .I1(axi4m_wvalid),
        .O(writeram_raddr1));
  (* SOFT_HLUTNM = "soft_lutpair66" *) 
  LUT4 #(
    .INIT(16'h6AAA)) 
    \write_word_count[7]_i_3 
       (.I0(write_word_count_reg[7]),
        .I1(write_word_count_reg[5]),
        .I2(\write_word_count[7]_i_4_n_0 ),
        .I3(write_word_count_reg[6]),
        .O(p_0_in__1[7]));
  (* SOFT_HLUTNM = "soft_lutpair60" *) 
  LUT5 #(
    .INIT(32'h80000000)) 
    \write_word_count[7]_i_4 
       (.I0(write_word_count_reg[4]),
        .I1(write_word_count_reg[3]),
        .I2(write_word_count_reg[1]),
        .I3(write_word_count_reg[0]),
        .I4(write_word_count_reg[2]),
        .O(\write_word_count[7]_i_4_n_0 ));
  FDRE \write_word_count_reg[0] 
       (.C(aclk),
        .CE(writeram_raddr1),
        .D(p_0_in__1[0]),
        .Q(write_word_count_reg[0]),
        .R(write_word_count));
  FDRE \write_word_count_reg[1] 
       (.C(aclk),
        .CE(writeram_raddr1),
        .D(p_0_in__1[1]),
        .Q(write_word_count_reg[1]),
        .R(write_word_count));
  FDRE \write_word_count_reg[2] 
       (.C(aclk),
        .CE(writeram_raddr1),
        .D(p_0_in__1[2]),
        .Q(write_word_count_reg[2]),
        .R(write_word_count));
  FDRE \write_word_count_reg[3] 
       (.C(aclk),
        .CE(writeram_raddr1),
        .D(p_0_in__1[3]),
        .Q(write_word_count_reg[3]),
        .R(write_word_count));
  FDRE \write_word_count_reg[4] 
       (.C(aclk),
        .CE(writeram_raddr1),
        .D(p_0_in__1[4]),
        .Q(write_word_count_reg[4]),
        .R(write_word_count));
  FDRE \write_word_count_reg[5] 
       (.C(aclk),
        .CE(writeram_raddr1),
        .D(p_0_in__1[5]),
        .Q(write_word_count_reg[5]),
        .R(write_word_count));
  FDRE \write_word_count_reg[6] 
       (.C(aclk),
        .CE(writeram_raddr1),
        .D(p_0_in__1[6]),
        .Q(write_word_count_reg[6]),
        .R(write_word_count));
  FDRE \write_word_count_reg[7] 
       (.C(aclk),
        .CE(writeram_raddr1),
        .D(p_0_in__1[7]),
        .Q(write_word_count_reg[7]),
        .R(write_word_count));
  BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram_0 writeram
       (.CLK(tck),
        .D({writeram_raddr,writeram_n_71}),
        .Q(writeram_waddr),
        .WEBWE(writeram_wr),
        .aclk(aclk),
        .axi4m_wdata(axi4m_wdata),
        .axi4m_wready(axi4m_wready),
        .ram_reg_0({\shift_in_reg_reg_n_0_[63] ,\shift_in_reg_reg_n_0_[62] ,\shift_in_reg_reg_n_0_[61] ,\shift_in_reg_reg_n_0_[60] ,\shift_in_reg_reg_n_0_[59] ,\shift_in_reg_reg_n_0_[58] ,\shift_in_reg_reg_n_0_[57] ,\shift_in_reg_reg_n_0_[56] ,\shift_in_reg_reg_n_0_[55] ,\shift_in_reg_reg_n_0_[54] ,\shift_in_reg_reg_n_0_[53] ,\shift_in_reg_reg_n_0_[52] ,\shift_in_reg_reg_n_0_[51] ,\shift_in_reg_reg_n_0_[50] ,\shift_in_reg_reg_n_0_[49] ,\shift_in_reg_reg_n_0_[48] ,\shift_in_reg_reg_n_0_[47] ,\shift_in_reg_reg_n_0_[46] ,\shift_in_reg_reg_n_0_[45] ,\shift_in_reg_reg_n_0_[44] ,\shift_in_reg_reg_n_0_[43] ,\shift_in_reg_reg_n_0_[42] ,\shift_in_reg_reg_n_0_[41] ,\shift_in_reg_reg_n_0_[40] ,\shift_in_reg_reg_n_0_[39] ,\shift_in_reg_reg_n_0_[38] ,\shift_in_reg_reg_n_0_[37] ,\shift_in_reg_reg_n_0_[36] ,\shift_in_reg_reg_n_0_[35] ,\shift_in_reg_reg_n_0_[34] ,\shift_in_reg_reg_n_0_[33] ,\shift_in_reg_reg_n_0_[32] ,\shift_in_reg_reg_n_0_[31] ,\shift_in_reg_reg_n_0_[30] ,\shift_in_reg_reg_n_0_[29] ,\shift_in_reg_reg_n_0_[28] ,\shift_in_reg_reg_n_0_[27] ,\shift_in_reg_reg_n_0_[26] ,\shift_in_reg_reg_n_0_[25] ,\shift_in_reg_reg_n_0_[24] ,\shift_in_reg_reg_n_0_[23] ,\shift_in_reg_reg_n_0_[22] ,\shift_in_reg_reg_n_0_[21] ,\shift_in_reg_reg_n_0_[20] ,\shift_in_reg_reg_n_0_[19] ,\shift_in_reg_reg_n_0_[18] ,\shift_in_reg_reg_n_0_[17] ,\shift_in_reg_reg_n_0_[16] ,\shift_in_reg_reg_n_0_[15] ,\shift_in_reg_reg_n_0_[14] ,\shift_in_reg_reg_n_0_[13] ,\shift_in_reg_reg_n_0_[12] ,\shift_in_reg_reg_n_0_[11] ,\shift_in_reg_reg_n_0_[10] ,\shift_in_reg_reg_n_0_[9] ,\shift_in_reg_reg_n_0_[8] ,\shift_in_reg_reg_n_0_[7] ,\shift_in_reg_reg_n_0_[6] ,\shift_in_reg_reg_n_0_[5] ,\shift_in_reg_reg_n_0_[4] ,\shift_in_reg_reg_n_0_[3] ,\shift_in_reg_reg_n_0_[2] ,\shift_in_reg_reg_n_0_[1] ,\shift_in_reg_reg_n_0_[0] }),
        .ram_reg_1(writeram_raddr_reg_reg),
        .ram_reg_2(axi4m_wvalid),
        .writeram_rd(writeram_rd));
  LUT3 #(
    .INIT(8'hFE)) 
    \writeram_raddr_reg[7]_i_1 
       (.I0(\state_write_reg_n_0_[1] ),
        .I1(\state_write_reg_n_0_[0] ),
        .I2(\state_write_reg_n_0_[2] ),
        .O(sel));
  FDRE \writeram_raddr_reg_reg[0] 
       (.C(aclk),
        .CE(sel),
        .D(writeram_n_71),
        .Q(writeram_raddr_reg_reg[0]),
        .R(state_write));
  FDRE \writeram_raddr_reg_reg[1] 
       (.C(aclk),
        .CE(sel),
        .D(writeram_raddr[1]),
        .Q(writeram_raddr_reg_reg[1]),
        .R(state_write));
  FDRE \writeram_raddr_reg_reg[2] 
       (.C(aclk),
        .CE(sel),
        .D(writeram_raddr[2]),
        .Q(writeram_raddr_reg_reg[2]),
        .R(state_write));
  FDRE \writeram_raddr_reg_reg[3] 
       (.C(aclk),
        .CE(sel),
        .D(writeram_raddr[3]),
        .Q(writeram_raddr_reg_reg[3]),
        .R(state_write));
  FDRE \writeram_raddr_reg_reg[4] 
       (.C(aclk),
        .CE(sel),
        .D(writeram_raddr[4]),
        .Q(writeram_raddr_reg_reg[4]),
        .R(state_write));
  FDRE \writeram_raddr_reg_reg[5] 
       (.C(aclk),
        .CE(sel),
        .D(writeram_raddr[5]),
        .Q(writeram_raddr_reg_reg[5]),
        .R(state_write));
  FDRE \writeram_raddr_reg_reg[6] 
       (.C(aclk),
        .CE(sel),
        .D(writeram_raddr[6]),
        .Q(writeram_raddr_reg_reg[6]),
        .R(state_write));
  FDRE \writeram_raddr_reg_reg[7] 
       (.C(aclk),
        .CE(sel),
        .D(writeram_raddr[7]),
        .Q(writeram_raddr_reg_reg[7]),
        .R(state_write));
  (* SOFT_HLUTNM = "soft_lutpair59" *) 
  LUT5 #(
    .INIT(32'hFFFFFFF4)) 
    writeram_rd_i_1
       (.I0(start_write_d2),
        .I1(start_write_d1),
        .I2(\state_write_reg_n_0_[1] ),
        .I3(\state_write_reg_n_0_[0] ),
        .I4(\state_write_reg_n_0_[2] ),
        .O(writeram_rd_i_1_n_0));
  FDRE writeram_rd_reg
       (.C(aclk),
        .CE(1'b1),
        .D(writeram_rd_i_1_n_0),
        .Q(writeram_rd),
        .R(aclk_reset));
  FDRE \writeram_waddr_reg[0] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(writeram_waddr_comb[0]),
        .Q(writeram_waddr[0]),
        .R(1'b0));
  FDRE \writeram_waddr_reg[1] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(writeram_waddr_comb[1]),
        .Q(writeram_waddr[1]),
        .R(1'b0));
  FDRE \writeram_waddr_reg[2] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(writeram_waddr_comb[2]),
        .Q(writeram_waddr[2]),
        .R(1'b0));
  FDRE \writeram_waddr_reg[3] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(writeram_waddr_comb[3]),
        .Q(writeram_waddr[3]),
        .R(1'b0));
  FDRE \writeram_waddr_reg[4] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(writeram_waddr_comb[4]),
        .Q(writeram_waddr[4]),
        .R(1'b0));
  FDRE \writeram_waddr_reg[5] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(writeram_waddr_comb[5]),
        .Q(writeram_waddr[5]),
        .R(1'b0));
  FDRE \writeram_waddr_reg[6] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(writeram_waddr_comb[6]),
        .Q(writeram_waddr[6]),
        .R(1'b0));
  FDRE \writeram_waddr_reg[7] 
       (.C(tck),
        .CE(shift_in_reg),
        .D(writeram_waddr_comb[7]),
        .Q(writeram_waddr[7]),
        .R(1'b0));
  FDRE writeram_wr_reg
       (.C(tck),
        .CE(1'b1),
        .D(u_jtag_n_9),
        .Q(writeram_wr),
        .R(1'b0));
endmodule

(* ORIG_REF_NAME = "hdlverifier_jtag_core" *) 
module BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_core
   (jtag_reset,
    tck,
    shift_out_en,
    shift_in_en,
    D,
    E,
    tck_reset_reg,
    readram_rd,
    shift_out_en_reg_0,
    \write_count_reg[15] ,
    tck_reset_reg_0,
    shift_in_en_reg_0,
    shift_in_state,
    shift_out_en_reg_1,
    shift_in_state_d1_reg,
    \reg_wdata_reg[0]_0 ,
    \reg_wdata_reg[31]_0 ,
    SR,
    status,
    aresetn,
    Q,
    tck_reset,
    ram_reg,
    ram_reg_0,
    u_BSCANE2,
    S,
    writeram_wr_reg,
    \readram_raddr_reg[0] ,
    writeram_wr_reg_0,
    shift_in_state_d1,
    start_write,
    trigger_tck_d1,
    start_read_reg);
  output jtag_reset;
  output tck;
  output shift_out_en;
  output shift_in_en;
  output [0:0]D;
  output [0:0]E;
  output [0:0]tck_reset_reg;
  output readram_rd;
  output [0:0]shift_out_en_reg_0;
  output \write_count_reg[15] ;
  output tck_reset_reg_0;
  output [0:0]shift_in_en_reg_0;
  output shift_in_state;
  output [0:0]shift_out_en_reg_1;
  output shift_in_state_d1_reg;
  output \reg_wdata_reg[0]_0 ;
  output [31:0]\reg_wdata_reg[31]_0 ;
  output [0:0]SR;
  input [1:0]status;
  input aresetn;
  input [0:0]Q;
  input tck_reset;
  input ram_reg;
  input [0:0]ram_reg_0;
  input [0:0]u_BSCANE2;
  input [1:0]S;
  input [4:0]writeram_wr_reg;
  input \readram_raddr_reg[0] ;
  input writeram_wr_reg_0;
  input shift_in_state_d1;
  input start_write;
  input trigger_tck_d1;
  input start_read_reg;

  wire [0:0]D;
  wire [0:0]E;
  wire \FSM_sequential_state_reg[0]_i_2_n_0 ;
  wire \FSM_sequential_state_reg[1]_i_2_n_0 ;
  wire \FSM_sequential_state_reg[2]_i_2_n_0 ;
  wire \FSM_sequential_state_reg[2]_i_3_n_0 ;
  wire \FSM_sequential_state_reg[3]_i_10_n_0 ;
  wire \FSM_sequential_state_reg[3]_i_11_n_0 ;
  wire \FSM_sequential_state_reg[3]_i_12_n_0 ;
  wire \FSM_sequential_state_reg[3]_i_5_n_0 ;
  wire \FSM_sequential_state_reg[3]_i_9_n_0 ;
  wire [0:0]Q;
  wire [1:0]S;
  wire [0:0]SR;
  wire aresetn;
  wire \burst_length[7]_i_2_n_0 ;
  wire [7:5]command_reg;
  wire command_reg_1;
  wire [7:0]counter;
  wire \counter[0]_i_1_n_0 ;
  wire \counter[1]_i_1_n_0 ;
  wire \counter[2]_i_1_n_0 ;
  wire \counter[3]_i_1_n_0 ;
  wire \counter[4]_i_1_n_0 ;
  wire \counter[5]_i_1_n_0 ;
  wire \counter[6]_i_1_n_0 ;
  wire \counter[7]_i_2_n_0 ;
  wire \counter[7]_i_3_n_0 ;
  wire \counter[7]_i_4_n_0 ;
  wire \counter[7]_i_5_n_0 ;
  wire counter_0;
  wire jtag_reset;
  wire [2:0]p_0_in;
  wire pad_len;
  wire \pad_len[0]_i_1_n_0 ;
  wire \pad_len[1]_i_1_n_0 ;
  wire \pad_len[2]_i_3_n_0 ;
  wire \pad_len_reg_n_0_[0] ;
  wire \pad_len_reg_n_0_[1] ;
  wire \pad_len_reg_n_0_[2] ;
  wire ram_reg;
  wire [0:0]ram_reg_0;
  wire [28:0]read_reg;
  wire \read_reg[16]_i_1_n_0 ;
  wire \read_reg[18]_i_1_n_0 ;
  wire \read_reg[19]_i_1_n_0 ;
  wire \read_reg[1]_i_2_n_0 ;
  wire \read_reg[1]_i_3_n_0 ;
  wire \read_reg[22]_i_1_n_0 ;
  wire \read_reg[23]_i_1_n_0 ;
  wire \read_reg[24]_i_1_n_0 ;
  wire \read_reg[25]_i_1_n_0 ;
  wire \read_reg[26]_i_1_n_0 ;
  wire \read_reg[27]_i_1_n_0 ;
  wire \read_reg[28]_i_3_n_0 ;
  wire \read_reg[3]_i_2_n_0 ;
  wire \read_reg[5]_i_1_n_0 ;
  wire \read_reg_reg_n_0_[0] ;
  wire \read_reg_reg_n_0_[10] ;
  wire \read_reg_reg_n_0_[11] ;
  wire \read_reg_reg_n_0_[12] ;
  wire \read_reg_reg_n_0_[13] ;
  wire \read_reg_reg_n_0_[14] ;
  wire \read_reg_reg_n_0_[15] ;
  wire \read_reg_reg_n_0_[16] ;
  wire \read_reg_reg_n_0_[17] ;
  wire \read_reg_reg_n_0_[18] ;
  wire \read_reg_reg_n_0_[19] ;
  wire \read_reg_reg_n_0_[1] ;
  wire \read_reg_reg_n_0_[20] ;
  wire \read_reg_reg_n_0_[21] ;
  wire \read_reg_reg_n_0_[22] ;
  wire \read_reg_reg_n_0_[23] ;
  wire \read_reg_reg_n_0_[24] ;
  wire \read_reg_reg_n_0_[25] ;
  wire \read_reg_reg_n_0_[26] ;
  wire \read_reg_reg_n_0_[27] ;
  wire \read_reg_reg_n_0_[28] ;
  wire \read_reg_reg_n_0_[2] ;
  wire \read_reg_reg_n_0_[3] ;
  wire \read_reg_reg_n_0_[4] ;
  wire \read_reg_reg_n_0_[5] ;
  wire \read_reg_reg_n_0_[6] ;
  wire \read_reg_reg_n_0_[7] ;
  wire \read_reg_reg_n_0_[8] ;
  wire \read_reg_reg_n_0_[9] ;
  wire \readram_raddr_reg[0] ;
  wire readram_rd;
  wire [4:0]reg_addr;
  wire \reg_addr[4]_i_2_n_0 ;
  wire \reg_wdata_reg[0]_0 ;
  wire [31:0]\reg_wdata_reg[31]_0 ;
  wire reg_write;
  wire reg_write_i_3_n_0;
  wire shift_dr;
  wire shift_in_data_i_2_n_0;
  wire shift_in_en;
  wire [0:0]shift_in_en_reg_0;
  wire shift_in_state;
  wire shift_in_state_d1;
  wire shift_in_state_d1_reg;
  wire shift_out_en;
  wire [0:0]shift_out_en_reg_0;
  wire [0:0]shift_out_en_reg_1;
  wire start_read_i_2_n_0;
  wire start_read_reg;
  wire start_write;
  wire [3:0]state_reg__0;
  wire [3:0]state_reg__1;
  wire [1:0]status;
  wire tck;
  wire tck_reset;
  wire [0:0]tck_reset_reg;
  wire tck_reset_reg_0;
  wire tdi;
  wire [31:25]tdi_shift_reg;
  wire [23:0]tdi_shift_reg_next;
  wire trigger_tck_d1;
  wire [0:0]u_BSCANE2;
  wire u_jtag_n_15;
  wire u_jtag_n_16;
  wire u_jtag_n_18;
  wire u_jtag_n_19;
  wire u_jtag_n_20;
  wire u_jtag_n_21;
  wire u_jtag_n_22;
  wire u_jtag_n_5;
  wire u_jtag_n_6;
  wire u_jtag_n_7;
  wire u_jtag_n_9;
  wire \write_count_reg[15] ;
  wire writeram_wr_i_2_n_0;
  wire [4:0]writeram_wr_reg;
  wire writeram_wr_reg_0;

  (* SOFT_HLUTNM = "soft_lutpair37" *) 
  LUT3 #(
    .INIT(8'h04)) 
    \FSM_sequential_state_reg[0]_i_2 
       (.I0(p_0_in[0]),
        .I1(p_0_in[2]),
        .I2(p_0_in[1]),
        .O(\FSM_sequential_state_reg[0]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair40" *) 
  LUT2 #(
    .INIT(4'hB)) 
    \FSM_sequential_state_reg[1]_i_2 
       (.I0(state_reg__0[3]),
        .I1(state_reg__0[0]),
        .O(\FSM_sequential_state_reg[1]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair50" *) 
  LUT2 #(
    .INIT(4'h1)) 
    \FSM_sequential_state_reg[2]_i_2 
       (.I0(state_reg__0[3]),
        .I1(state_reg__0[1]),
        .O(\FSM_sequential_state_reg[2]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair47" *) 
  LUT4 #(
    .INIT(16'h0002)) 
    \FSM_sequential_state_reg[2]_i_3 
       (.I0(state_reg__0[1]),
        .I1(state_reg__0[3]),
        .I2(state_reg__0[2]),
        .I3(state_reg__0[0]),
        .O(\FSM_sequential_state_reg[2]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFFFFFFE)) 
    \FSM_sequential_state_reg[3]_i_10 
       (.I0(counter[4]),
        .I1(counter[3]),
        .I2(counter[5]),
        .I3(counter[6]),
        .I4(counter[7]),
        .I5(\FSM_sequential_state_reg[3]_i_12_n_0 ),
        .O(\FSM_sequential_state_reg[3]_i_10_n_0 ));
  LUT5 #(
    .INIT(32'hFFFFFFFE)) 
    \FSM_sequential_state_reg[3]_i_11 
       (.I0(counter[7]),
        .I1(counter[6]),
        .I2(counter[5]),
        .I3(counter[3]),
        .I4(counter[4]),
        .O(\FSM_sequential_state_reg[3]_i_11_n_0 ));
  LUT6 #(
    .INIT(64'h6FF6FFFFFFFF6FF6)) 
    \FSM_sequential_state_reg[3]_i_12 
       (.I0(counter[1]),
        .I1(\pad_len_reg_n_0_[1] ),
        .I2(counter[0]),
        .I3(\pad_len_reg_n_0_[0] ),
        .I4(\pad_len_reg_n_0_[2] ),
        .I5(counter[2]),
        .O(\FSM_sequential_state_reg[3]_i_12_n_0 ));
  LUT6 #(
    .INIT(64'h0000000000000080)) 
    \FSM_sequential_state_reg[3]_i_5 
       (.I0(counter[0]),
        .I1(counter[1]),
        .I2(counter[2]),
        .I3(\FSM_sequential_state_reg[3]_i_11_n_0 ),
        .I4(state_reg__0[2]),
        .I5(state_reg__0[0]),
        .O(\FSM_sequential_state_reg[3]_i_5_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair50" *) 
  LUT2 #(
    .INIT(4'hB)) 
    \FSM_sequential_state_reg[3]_i_9 
       (.I0(state_reg__0[3]),
        .I1(state_reg__0[1]),
        .O(\FSM_sequential_state_reg[3]_i_9_n_0 ));
  (* FSM_ENCODED_STATES = "STATE_WRITE_REG:0011,STATE_READ_REG:0111,STATE_CMD:0010,STATE_HEADER:0001,STATE_IDLE:0000,STATE_READ_DATA:0110,STATE_WR_TRANSIT:0100,STATE_WRITE_DATA:1000,STATE_WAIT_PAD:0101" *) 
  FDRE \FSM_sequential_state_reg_reg[0] 
       (.C(tck),
        .CE(u_jtag_n_5),
        .D(state_reg__1[0]),
        .Q(state_reg__0[0]),
        .R(jtag_reset));
  (* FSM_ENCODED_STATES = "STATE_WRITE_REG:0011,STATE_READ_REG:0111,STATE_CMD:0010,STATE_HEADER:0001,STATE_IDLE:0000,STATE_READ_DATA:0110,STATE_WR_TRANSIT:0100,STATE_WRITE_DATA:1000,STATE_WAIT_PAD:0101" *) 
  FDRE \FSM_sequential_state_reg_reg[1] 
       (.C(tck),
        .CE(u_jtag_n_5),
        .D(state_reg__1[1]),
        .Q(state_reg__0[1]),
        .R(jtag_reset));
  (* FSM_ENCODED_STATES = "STATE_WRITE_REG:0011,STATE_READ_REG:0111,STATE_CMD:0010,STATE_HEADER:0001,STATE_IDLE:0000,STATE_READ_DATA:0110,STATE_WR_TRANSIT:0100,STATE_WRITE_DATA:1000,STATE_WAIT_PAD:0101" *) 
  FDRE \FSM_sequential_state_reg_reg[2] 
       (.C(tck),
        .CE(u_jtag_n_5),
        .D(state_reg__1[2]),
        .Q(state_reg__0[2]),
        .R(jtag_reset));
  (* FSM_ENCODED_STATES = "STATE_WRITE_REG:0011,STATE_READ_REG:0111,STATE_CMD:0010,STATE_HEADER:0001,STATE_IDLE:0000,STATE_READ_DATA:0110,STATE_WR_TRANSIT:0100,STATE_WRITE_DATA:1000,STATE_WAIT_PAD:0101" *) 
  FDRE \FSM_sequential_state_reg_reg[3] 
       (.C(tck),
        .CE(u_jtag_n_5),
        .D(state_reg__1[3]),
        .Q(state_reg__0[3]),
        .R(jtag_reset));
  LUT6 #(
    .INIT(64'h0000000000000020)) 
    \address[31]_i_1 
       (.I0(reg_write),
        .I1(reg_addr[1]),
        .I2(reg_addr[0]),
        .I3(reg_addr[2]),
        .I4(reg_addr[3]),
        .I5(reg_addr[4]),
        .O(E));
  (* SOFT_HLUTNM = "soft_lutpair44" *) 
  LUT4 #(
    .INIT(16'hFFEF)) 
    \bitcount[5]_i_1 
       (.I0(tck_reset),
        .I1(p_0_in[1]),
        .I2(p_0_in[2]),
        .I3(p_0_in[0]),
        .O(SR));
  (* SOFT_HLUTNM = "soft_lutpair42" *) 
  LUT5 #(
    .INIT(32'h00000020)) 
    \burst_length[7]_i_1 
       (.I0(\burst_length[7]_i_2_n_0 ),
        .I1(tck_reset),
        .I2(reg_addr[2]),
        .I3(reg_addr[4]),
        .I4(reg_addr[3]),
        .O(tck_reset_reg));
  LUT3 #(
    .INIT(8'h02)) 
    \burst_length[7]_i_2 
       (.I0(reg_write),
        .I1(reg_addr[0]),
        .I2(reg_addr[1]),
        .O(\burst_length[7]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair52" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \command_reg[5]_i_1 
       (.I0(tdi_shift_reg[30]),
        .I1(state_reg__0[1]),
        .O(command_reg[5]));
  (* SOFT_HLUTNM = "soft_lutpair51" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \command_reg[6]_i_1 
       (.I0(tdi_shift_reg[31]),
        .I1(state_reg__0[1]),
        .O(command_reg[6]));
  FDRE \command_reg_reg[5] 
       (.C(tck),
        .CE(command_reg_1),
        .D(command_reg[5]),
        .Q(p_0_in[0]),
        .R(jtag_reset));
  FDRE \command_reg_reg[6] 
       (.C(tck),
        .CE(command_reg_1),
        .D(command_reg[6]),
        .Q(p_0_in[1]),
        .R(jtag_reset));
  FDRE \command_reg_reg[7] 
       (.C(tck),
        .CE(command_reg_1),
        .D(command_reg[7]),
        .Q(p_0_in[2]),
        .R(jtag_reset));
  (* SOFT_HLUTNM = "soft_lutpair48" *) 
  LUT2 #(
    .INIT(4'h1)) 
    \counter[0]_i_1 
       (.I0(counter[0]),
        .I1(\counter[7]_i_5_n_0 ),
        .O(\counter[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair48" *) 
  LUT3 #(
    .INIT(8'h14)) 
    \counter[1]_i_1 
       (.I0(\counter[7]_i_5_n_0 ),
        .I1(counter[0]),
        .I2(counter[1]),
        .O(\counter[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair38" *) 
  LUT4 #(
    .INIT(16'h1540)) 
    \counter[2]_i_1 
       (.I0(\counter[7]_i_5_n_0 ),
        .I1(counter[1]),
        .I2(counter[0]),
        .I3(counter[2]),
        .O(\counter[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair38" *) 
  LUT5 #(
    .INIT(32'h15554000)) 
    \counter[3]_i_1 
       (.I0(\counter[7]_i_5_n_0 ),
        .I1(counter[0]),
        .I2(counter[1]),
        .I3(counter[2]),
        .I4(counter[3]),
        .O(\counter[3]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h1555555540000000)) 
    \counter[4]_i_1 
       (.I0(\counter[7]_i_5_n_0 ),
        .I1(counter[2]),
        .I2(counter[1]),
        .I3(counter[0]),
        .I4(counter[3]),
        .I5(counter[4]),
        .O(\counter[4]_i_1_n_0 ));
  LUT3 #(
    .INIT(8'h41)) 
    \counter[5]_i_1 
       (.I0(\counter[7]_i_5_n_0 ),
        .I1(\counter[7]_i_4_n_0 ),
        .I2(counter[5]),
        .O(\counter[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair39" *) 
  LUT4 #(
    .INIT(16'h5104)) 
    \counter[6]_i_1 
       (.I0(\counter[7]_i_5_n_0 ),
        .I1(counter[5]),
        .I2(\counter[7]_i_4_n_0 ),
        .I3(counter[6]),
        .O(\counter[6]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair39" *) 
  LUT5 #(
    .INIT(32'h0000BF40)) 
    \counter[7]_i_2 
       (.I0(\counter[7]_i_4_n_0 ),
        .I1(counter[5]),
        .I2(counter[6]),
        .I3(counter[7]),
        .I4(\counter[7]_i_5_n_0 ),
        .O(\counter[7]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair46" *) 
  LUT4 #(
    .INIT(16'hFDFF)) 
    \counter[7]_i_3 
       (.I0(counter[0]),
        .I1(counter[1]),
        .I2(\FSM_sequential_state_reg[3]_i_11_n_0 ),
        .I3(counter[2]),
        .O(\counter[7]_i_3_n_0 ));
  LUT5 #(
    .INIT(32'h7FFFFFFF)) 
    \counter[7]_i_4 
       (.I0(counter[3]),
        .I1(counter[0]),
        .I2(counter[1]),
        .I3(counter[2]),
        .I4(counter[4]),
        .O(\counter[7]_i_4_n_0 ));
  LUT6 #(
    .INIT(64'hFFFF000CFFFF55FF)) 
    \counter[7]_i_5 
       (.I0(\FSM_sequential_state_reg[3]_i_10_n_0 ),
        .I1(\reg_addr[4]_i_2_n_0 ),
        .I2(state_reg__0[2]),
        .I3(state_reg__0[0]),
        .I4(state_reg__0[3]),
        .I5(state_reg__0[1]),
        .O(\counter[7]_i_5_n_0 ));
  FDRE \counter_reg[0] 
       (.C(tck),
        .CE(counter_0),
        .D(\counter[0]_i_1_n_0 ),
        .Q(counter[0]),
        .R(jtag_reset));
  FDRE \counter_reg[1] 
       (.C(tck),
        .CE(counter_0),
        .D(\counter[1]_i_1_n_0 ),
        .Q(counter[1]),
        .R(jtag_reset));
  FDRE \counter_reg[2] 
       (.C(tck),
        .CE(counter_0),
        .D(\counter[2]_i_1_n_0 ),
        .Q(counter[2]),
        .R(jtag_reset));
  FDRE \counter_reg[3] 
       (.C(tck),
        .CE(counter_0),
        .D(\counter[3]_i_1_n_0 ),
        .Q(counter[3]),
        .R(jtag_reset));
  FDRE \counter_reg[4] 
       (.C(tck),
        .CE(counter_0),
        .D(\counter[4]_i_1_n_0 ),
        .Q(counter[4]),
        .R(jtag_reset));
  FDRE \counter_reg[5] 
       (.C(tck),
        .CE(counter_0),
        .D(\counter[5]_i_1_n_0 ),
        .Q(counter[5]),
        .R(jtag_reset));
  FDRE \counter_reg[6] 
       (.C(tck),
        .CE(counter_0),
        .D(\counter[6]_i_1_n_0 ),
        .Q(counter[6]),
        .R(jtag_reset));
  FDRE \counter_reg[7] 
       (.C(tck),
        .CE(counter_0),
        .D(\counter[7]_i_2_n_0 ),
        .Q(counter[7]),
        .R(jtag_reset));
  (* SOFT_HLUTNM = "soft_lutpair52" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pad_len[0]_i_1 
       (.I0(tdi_shift_reg[30]),
        .I1(state_reg__0[0]),
        .O(\pad_len[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair51" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pad_len[1]_i_1 
       (.I0(tdi_shift_reg[31]),
        .I1(state_reg__0[0]),
        .O(\pad_len[1]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h0000100000000000)) 
    \pad_len[2]_i_3 
       (.I0(tdi_shift_reg[26]),
        .I1(tdi_shift_reg[28]),
        .I2(tdi_shift_reg[27]),
        .I3(tdi_shift_reg[29]),
        .I4(state_reg__0[2]),
        .I5(tdi_shift_reg[25]),
        .O(\pad_len[2]_i_3_n_0 ));
  FDRE \pad_len_reg[0] 
       (.C(tck),
        .CE(pad_len),
        .D(\pad_len[0]_i_1_n_0 ),
        .Q(\pad_len_reg_n_0_[0] ),
        .R(jtag_reset));
  FDRE \pad_len_reg[1] 
       (.C(tck),
        .CE(pad_len),
        .D(\pad_len[1]_i_1_n_0 ),
        .Q(\pad_len_reg_n_0_[1] ),
        .R(jtag_reset));
  FDRE \pad_len_reg[2] 
       (.C(tck),
        .CE(pad_len),
        .D(u_jtag_n_16),
        .Q(\pad_len_reg_n_0_[2] ),
        .R(jtag_reset));
  (* SOFT_HLUTNM = "soft_lutpair37" *) 
  LUT5 #(
    .INIT(32'h00000020)) 
    ram_reg_i_1
       (.I0(ram_reg),
        .I1(p_0_in[0]),
        .I2(p_0_in[2]),
        .I3(p_0_in[1]),
        .I4(ram_reg_0),
        .O(readram_rd));
  (* SOFT_HLUTNM = "soft_lutpair40" *) 
  LUT5 #(
    .INIT(32'h4F444444)) 
    \read_reg[0]_i_1 
       (.I0(\read_reg[3]_i_2_n_0 ),
        .I1(status[0]),
        .I2(state_reg__0[3]),
        .I3(state_reg__0[0]),
        .I4(\read_reg_reg_n_0_[1] ),
        .O(read_reg[0]));
  LUT6 #(
    .INIT(64'h00FF00F9000000F9)) 
    \read_reg[16]_i_1 
       (.I0(reg_addr[0]),
        .I1(reg_addr[1]),
        .I2(\read_reg[1]_i_3_n_0 ),
        .I3(state_reg__0[3]),
        .I4(state_reg__0[0]),
        .I5(\read_reg_reg_n_0_[17] ),
        .O(\read_reg[16]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h00FF00F9000000F9)) 
    \read_reg[18]_i_1 
       (.I0(reg_addr[0]),
        .I1(reg_addr[1]),
        .I2(\read_reg[1]_i_3_n_0 ),
        .I3(state_reg__0[3]),
        .I4(state_reg__0[0]),
        .I5(\read_reg_reg_n_0_[19] ),
        .O(\read_reg[18]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h00FF00F9000000F9)) 
    \read_reg[19]_i_1 
       (.I0(reg_addr[0]),
        .I1(reg_addr[1]),
        .I2(\read_reg[1]_i_3_n_0 ),
        .I3(state_reg__0[3]),
        .I4(state_reg__0[0]),
        .I5(\read_reg_reg_n_0_[20] ),
        .O(\read_reg[19]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hCCCCCCCCCCCCCFEC)) 
    \read_reg[1]_i_1 
       (.I0(status[1]),
        .I1(\read_reg[1]_i_2_n_0 ),
        .I2(reg_addr[0]),
        .I3(reg_addr[1]),
        .I4(\read_reg[28]_i_3_n_0 ),
        .I5(\read_reg[1]_i_3_n_0 ),
        .O(read_reg[1]));
  (* SOFT_HLUTNM = "soft_lutpair49" *) 
  LUT3 #(
    .INIT(8'h08)) 
    \read_reg[1]_i_2 
       (.I0(\read_reg_reg_n_0_[2] ),
        .I1(state_reg__0[0]),
        .I2(state_reg__0[3]),
        .O(\read_reg[1]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair42" *) 
  LUT3 #(
    .INIT(8'hFE)) 
    \read_reg[1]_i_3 
       (.I0(reg_addr[2]),
        .I1(reg_addr[3]),
        .I2(reg_addr[4]),
        .O(\read_reg[1]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h00FF00F9000000F9)) 
    \read_reg[22]_i_1 
       (.I0(reg_addr[0]),
        .I1(reg_addr[1]),
        .I2(\read_reg[1]_i_3_n_0 ),
        .I3(state_reg__0[3]),
        .I4(state_reg__0[0]),
        .I5(\read_reg_reg_n_0_[23] ),
        .O(\read_reg[22]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h00FF00F9000000F9)) 
    \read_reg[23]_i_1 
       (.I0(reg_addr[0]),
        .I1(reg_addr[1]),
        .I2(\read_reg[1]_i_3_n_0 ),
        .I3(state_reg__0[3]),
        .I4(state_reg__0[0]),
        .I5(\read_reg_reg_n_0_[24] ),
        .O(\read_reg[23]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h00FF00F9000000F9)) 
    \read_reg[24]_i_1 
       (.I0(reg_addr[0]),
        .I1(reg_addr[1]),
        .I2(\read_reg[1]_i_3_n_0 ),
        .I3(state_reg__0[3]),
        .I4(state_reg__0[0]),
        .I5(\read_reg_reg_n_0_[25] ),
        .O(\read_reg[24]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h00FF00F9000000F9)) 
    \read_reg[25]_i_1 
       (.I0(reg_addr[0]),
        .I1(reg_addr[1]),
        .I2(\read_reg[1]_i_3_n_0 ),
        .I3(state_reg__0[3]),
        .I4(state_reg__0[0]),
        .I5(\read_reg_reg_n_0_[26] ),
        .O(\read_reg[25]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h00FF00F9000000F9)) 
    \read_reg[26]_i_1 
       (.I0(reg_addr[0]),
        .I1(reg_addr[1]),
        .I2(\read_reg[1]_i_3_n_0 ),
        .I3(state_reg__0[3]),
        .I4(state_reg__0[0]),
        .I5(\read_reg_reg_n_0_[27] ),
        .O(\read_reg[26]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h00FF00F9000000F9)) 
    \read_reg[27]_i_1 
       (.I0(reg_addr[0]),
        .I1(reg_addr[1]),
        .I2(\read_reg[1]_i_3_n_0 ),
        .I3(state_reg__0[3]),
        .I4(state_reg__0[0]),
        .I5(\read_reg_reg_n_0_[28] ),
        .O(\read_reg[27]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h5555555455545555)) 
    \read_reg[28]_i_2 
       (.I0(\read_reg[28]_i_3_n_0 ),
        .I1(reg_addr[2]),
        .I2(reg_addr[3]),
        .I3(reg_addr[4]),
        .I4(reg_addr[1]),
        .I5(reg_addr[0]),
        .O(read_reg[28]));
  (* SOFT_HLUTNM = "soft_lutpair41" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \read_reg[28]_i_3 
       (.I0(state_reg__0[3]),
        .I1(state_reg__0[0]),
        .O(\read_reg[28]_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair41" *) 
  LUT5 #(
    .INIT(32'h1F111111)) 
    \read_reg[2]_i_1 
       (.I0(\read_reg[3]_i_2_n_0 ),
        .I1(aresetn),
        .I2(state_reg__0[3]),
        .I3(state_reg__0[0]),
        .I4(\read_reg_reg_n_0_[3] ),
        .O(read_reg[2]));
  LUT5 #(
    .INIT(32'h1F111111)) 
    \read_reg[3]_i_1 
       (.I0(\read_reg[3]_i_2_n_0 ),
        .I1(Q),
        .I2(state_reg__0[3]),
        .I3(state_reg__0[0]),
        .I4(\read_reg_reg_n_0_[4] ),
        .O(read_reg[3]));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFFFFFEF)) 
    \read_reg[3]_i_2 
       (.I0(\read_reg[28]_i_3_n_0 ),
        .I1(reg_addr[1]),
        .I2(reg_addr[0]),
        .I3(reg_addr[2]),
        .I4(reg_addr[3]),
        .I5(reg_addr[4]),
        .O(\read_reg[3]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h00FF00F9000000F9)) 
    \read_reg[5]_i_1 
       (.I0(reg_addr[0]),
        .I1(reg_addr[1]),
        .I2(\read_reg[1]_i_3_n_0 ),
        .I3(state_reg__0[3]),
        .I4(state_reg__0[0]),
        .I5(\read_reg_reg_n_0_[6] ),
        .O(\read_reg[5]_i_1_n_0 ));
  FDRE \read_reg_reg[0] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(read_reg[0]),
        .Q(\read_reg_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \read_reg_reg[10] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[11] ),
        .Q(\read_reg_reg_n_0_[10] ),
        .R(u_jtag_n_22));
  FDRE \read_reg_reg[11] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[12] ),
        .Q(\read_reg_reg_n_0_[11] ),
        .R(u_jtag_n_22));
  FDRE \read_reg_reg[12] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[13] ),
        .Q(\read_reg_reg_n_0_[12] ),
        .R(u_jtag_n_22));
  FDRE \read_reg_reg[13] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[14] ),
        .Q(\read_reg_reg_n_0_[13] ),
        .R(u_jtag_n_22));
  FDRE \read_reg_reg[14] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[15] ),
        .Q(\read_reg_reg_n_0_[14] ),
        .R(u_jtag_n_22));
  FDRE \read_reg_reg[15] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[16] ),
        .Q(\read_reg_reg_n_0_[15] ),
        .R(u_jtag_n_22));
  FDSE \read_reg_reg[16] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg[16]_i_1_n_0 ),
        .Q(\read_reg_reg_n_0_[16] ),
        .S(1'b0));
  FDRE \read_reg_reg[17] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[18] ),
        .Q(\read_reg_reg_n_0_[17] ),
        .R(u_jtag_n_22));
  FDSE \read_reg_reg[18] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg[18]_i_1_n_0 ),
        .Q(\read_reg_reg_n_0_[18] ),
        .S(1'b0));
  FDSE \read_reg_reg[19] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg[19]_i_1_n_0 ),
        .Q(\read_reg_reg_n_0_[19] ),
        .S(1'b0));
  FDRE \read_reg_reg[1] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(read_reg[1]),
        .Q(\read_reg_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \read_reg_reg[20] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[21] ),
        .Q(\read_reg_reg_n_0_[20] ),
        .R(u_jtag_n_22));
  FDRE \read_reg_reg[21] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[22] ),
        .Q(\read_reg_reg_n_0_[21] ),
        .R(u_jtag_n_22));
  FDSE \read_reg_reg[22] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg[22]_i_1_n_0 ),
        .Q(\read_reg_reg_n_0_[22] ),
        .S(1'b0));
  FDSE \read_reg_reg[23] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg[23]_i_1_n_0 ),
        .Q(\read_reg_reg_n_0_[23] ),
        .S(1'b0));
  FDSE \read_reg_reg[24] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg[24]_i_1_n_0 ),
        .Q(\read_reg_reg_n_0_[24] ),
        .S(1'b0));
  FDSE \read_reg_reg[25] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg[25]_i_1_n_0 ),
        .Q(\read_reg_reg_n_0_[25] ),
        .S(1'b0));
  FDSE \read_reg_reg[26] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg[26]_i_1_n_0 ),
        .Q(\read_reg_reg_n_0_[26] ),
        .S(1'b0));
  FDSE \read_reg_reg[27] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg[27]_i_1_n_0 ),
        .Q(\read_reg_reg_n_0_[27] ),
        .S(1'b0));
  FDRE \read_reg_reg[28] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(read_reg[28]),
        .Q(\read_reg_reg_n_0_[28] ),
        .R(1'b0));
  FDRE \read_reg_reg[2] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(read_reg[2]),
        .Q(\read_reg_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \read_reg_reg[3] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(read_reg[3]),
        .Q(\read_reg_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \read_reg_reg[4] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[5] ),
        .Q(\read_reg_reg_n_0_[4] ),
        .R(u_jtag_n_22));
  FDSE \read_reg_reg[5] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg[5]_i_1_n_0 ),
        .Q(\read_reg_reg_n_0_[5] ),
        .S(1'b0));
  FDRE \read_reg_reg[6] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[7] ),
        .Q(\read_reg_reg_n_0_[6] ),
        .R(u_jtag_n_22));
  FDRE \read_reg_reg[7] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[8] ),
        .Q(\read_reg_reg_n_0_[7] ),
        .R(u_jtag_n_22));
  FDRE \read_reg_reg[8] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[9] ),
        .Q(\read_reg_reg_n_0_[8] ),
        .R(u_jtag_n_22));
  FDRE \read_reg_reg[9] 
       (.C(tck),
        .CE(u_jtag_n_7),
        .D(\read_reg_reg_n_0_[10] ),
        .Q(\read_reg_reg_n_0_[9] ),
        .R(u_jtag_n_22));
  LUT2 #(
    .INIT(4'h2)) 
    \readram_raddr[7]_i_1 
       (.I0(shift_out_en),
        .I1(\readram_raddr_reg[0] ),
        .O(shift_out_en_reg_1));
  (* SOFT_HLUTNM = "soft_lutpair46" *) 
  LUT4 #(
    .INIT(16'h4000)) 
    \reg_addr[4]_i_2 
       (.I0(\FSM_sequential_state_reg[3]_i_11_n_0 ),
        .I1(counter[2]),
        .I2(counter[1]),
        .I3(counter[0]),
        .O(\reg_addr[4]_i_2_n_0 ));
  FDRE \reg_addr_reg[0] 
       (.C(tck),
        .CE(u_jtag_n_9),
        .D(tdi_shift_reg[25]),
        .Q(reg_addr[0]),
        .R(1'b0));
  FDRE \reg_addr_reg[1] 
       (.C(tck),
        .CE(u_jtag_n_9),
        .D(tdi_shift_reg[26]),
        .Q(reg_addr[1]),
        .R(1'b0));
  FDRE \reg_addr_reg[2] 
       (.C(tck),
        .CE(u_jtag_n_9),
        .D(tdi_shift_reg[27]),
        .Q(reg_addr[2]),
        .R(1'b0));
  FDRE \reg_addr_reg[3] 
       (.C(tck),
        .CE(u_jtag_n_9),
        .D(tdi_shift_reg[28]),
        .Q(reg_addr[3]),
        .R(1'b0));
  FDRE \reg_addr_reg[4] 
       (.C(tck),
        .CE(u_jtag_n_9),
        .D(tdi_shift_reg[29]),
        .Q(reg_addr[4]),
        .R(1'b0));
  FDRE \reg_wdata_reg[0] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[0]),
        .Q(\reg_wdata_reg[31]_0 [0]),
        .R(1'b0));
  FDRE \reg_wdata_reg[10] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[10]),
        .Q(\reg_wdata_reg[31]_0 [10]),
        .R(1'b0));
  FDRE \reg_wdata_reg[11] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[11]),
        .Q(\reg_wdata_reg[31]_0 [11]),
        .R(1'b0));
  FDRE \reg_wdata_reg[12] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[12]),
        .Q(\reg_wdata_reg[31]_0 [12]),
        .R(1'b0));
  FDRE \reg_wdata_reg[13] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[13]),
        .Q(\reg_wdata_reg[31]_0 [13]),
        .R(1'b0));
  FDRE \reg_wdata_reg[14] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[14]),
        .Q(\reg_wdata_reg[31]_0 [14]),
        .R(1'b0));
  FDRE \reg_wdata_reg[15] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[15]),
        .Q(\reg_wdata_reg[31]_0 [15]),
        .R(1'b0));
  FDRE \reg_wdata_reg[16] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[16]),
        .Q(\reg_wdata_reg[31]_0 [16]),
        .R(1'b0));
  FDRE \reg_wdata_reg[17] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[17]),
        .Q(\reg_wdata_reg[31]_0 [17]),
        .R(1'b0));
  FDRE \reg_wdata_reg[18] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[18]),
        .Q(\reg_wdata_reg[31]_0 [18]),
        .R(1'b0));
  FDRE \reg_wdata_reg[19] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[19]),
        .Q(\reg_wdata_reg[31]_0 [19]),
        .R(1'b0));
  FDRE \reg_wdata_reg[1] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[1]),
        .Q(\reg_wdata_reg[31]_0 [1]),
        .R(1'b0));
  FDRE \reg_wdata_reg[20] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[20]),
        .Q(\reg_wdata_reg[31]_0 [20]),
        .R(1'b0));
  FDRE \reg_wdata_reg[21] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[21]),
        .Q(\reg_wdata_reg[31]_0 [21]),
        .R(1'b0));
  FDRE \reg_wdata_reg[22] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[22]),
        .Q(\reg_wdata_reg[31]_0 [22]),
        .R(1'b0));
  FDRE \reg_wdata_reg[23] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[23]),
        .Q(\reg_wdata_reg[31]_0 [23]),
        .R(1'b0));
  FDRE \reg_wdata_reg[24] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg[25]),
        .Q(\reg_wdata_reg[31]_0 [24]),
        .R(1'b0));
  FDRE \reg_wdata_reg[25] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg[26]),
        .Q(\reg_wdata_reg[31]_0 [25]),
        .R(1'b0));
  FDRE \reg_wdata_reg[26] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg[27]),
        .Q(\reg_wdata_reg[31]_0 [26]),
        .R(1'b0));
  FDRE \reg_wdata_reg[27] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg[28]),
        .Q(\reg_wdata_reg[31]_0 [27]),
        .R(1'b0));
  FDRE \reg_wdata_reg[28] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg[29]),
        .Q(\reg_wdata_reg[31]_0 [28]),
        .R(1'b0));
  FDRE \reg_wdata_reg[29] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg[30]),
        .Q(\reg_wdata_reg[31]_0 [29]),
        .R(1'b0));
  FDRE \reg_wdata_reg[2] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[2]),
        .Q(\reg_wdata_reg[31]_0 [2]),
        .R(1'b0));
  FDRE \reg_wdata_reg[30] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg[31]),
        .Q(\reg_wdata_reg[31]_0 [30]),
        .R(1'b0));
  FDRE \reg_wdata_reg[31] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi),
        .Q(\reg_wdata_reg[31]_0 [31]),
        .R(1'b0));
  FDRE \reg_wdata_reg[3] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[3]),
        .Q(\reg_wdata_reg[31]_0 [3]),
        .R(1'b0));
  FDRE \reg_wdata_reg[4] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[4]),
        .Q(\reg_wdata_reg[31]_0 [4]),
        .R(1'b0));
  FDRE \reg_wdata_reg[5] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[5]),
        .Q(\reg_wdata_reg[31]_0 [5]),
        .R(1'b0));
  FDRE \reg_wdata_reg[6] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[6]),
        .Q(\reg_wdata_reg[31]_0 [6]),
        .R(1'b0));
  FDRE \reg_wdata_reg[7] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[7]),
        .Q(\reg_wdata_reg[31]_0 [7]),
        .R(1'b0));
  FDRE \reg_wdata_reg[8] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[8]),
        .Q(\reg_wdata_reg[31]_0 [8]),
        .R(1'b0));
  FDRE \reg_wdata_reg[9] 
       (.C(tck),
        .CE(u_jtag_n_6),
        .D(tdi_shift_reg_next[9]),
        .Q(\reg_wdata_reg[31]_0 [9]),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair49" *) 
  LUT2 #(
    .INIT(4'hE)) 
    reg_write_i_3
       (.I0(state_reg__0[0]),
        .I1(state_reg__0[2]),
        .O(reg_write_i_3_n_0));
  FDRE reg_write_reg
       (.C(tck),
        .CE(1'b1),
        .D(u_jtag_n_18),
        .Q(reg_write),
        .R(jtag_reset));
  (* SOFT_HLUTNM = "soft_lutpair47" *) 
  LUT4 #(
    .INIT(16'h0010)) 
    shift_in_data_i_2
       (.I0(state_reg__0[2]),
        .I1(state_reg__0[0]),
        .I2(state_reg__0[3]),
        .I3(state_reg__0[1]),
        .O(shift_in_data_i_2_n_0));
  FDRE shift_in_data_reg
       (.C(tck),
        .CE(1'b1),
        .D(u_jtag_n_21),
        .Q(D),
        .R(1'b0));
  FDRE shift_in_en_reg
       (.C(tck),
        .CE(1'b1),
        .D(u_jtag_n_20),
        .Q(shift_in_en),
        .R(jtag_reset));
  (* SOFT_HLUTNM = "soft_lutpair43" *) 
  LUT5 #(
    .INIT(32'h00000080)) 
    \shift_in_reg[63]_i_1 
       (.I0(shift_in_en),
        .I1(p_0_in[0]),
        .I2(p_0_in[2]),
        .I3(p_0_in[1]),
        .I4(tck_reset),
        .O(shift_in_en_reg_0));
  (* SOFT_HLUTNM = "soft_lutpair45" *) 
  LUT3 #(
    .INIT(8'h08)) 
    shift_in_state_d1_i_2
       (.I0(p_0_in[0]),
        .I1(p_0_in[2]),
        .I2(p_0_in[1]),
        .O(shift_in_state));
  FDRE shift_out_en_reg
       (.C(tck),
        .CE(1'b1),
        .D(u_jtag_n_19),
        .Q(shift_out_en),
        .R(jtag_reset));
  (* SOFT_HLUTNM = "soft_lutpair44" *) 
  LUT5 #(
    .INIT(32'h00000020)) 
    \shift_reg[63]_i_1 
       (.I0(shift_out_en),
        .I1(p_0_in[0]),
        .I2(p_0_in[2]),
        .I3(p_0_in[1]),
        .I4(tck_reset),
        .O(shift_out_en_reg_0));
  LUT4 #(
    .INIT(16'hA3A0)) 
    start_read_i_1
       (.I0(\reg_wdata_reg[31]_0 [0]),
        .I1(trigger_tck_d1),
        .I2(start_read_i_2_n_0),
        .I3(start_read_reg),
        .O(\reg_wdata_reg[0]_0 ));
  LUT6 #(
    .INIT(64'h0000000000000010)) 
    start_read_i_2
       (.I0(reg_addr[1]),
        .I1(reg_addr[0]),
        .I2(reg_write),
        .I3(reg_addr[4]),
        .I4(reg_addr[3]),
        .I5(reg_addr[2]),
        .O(start_read_i_2_n_0));
  (* SOFT_HLUTNM = "soft_lutpair45" *) 
  LUT5 #(
    .INIT(32'hEFFF8AAA)) 
    start_write_i_1
       (.I0(shift_in_state_d1),
        .I1(p_0_in[1]),
        .I2(p_0_in[2]),
        .I3(p_0_in[0]),
        .I4(start_write),
        .O(shift_in_state_d1_reg));
  FDRE \tdi_shift_reg_reg[10] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[10]),
        .Q(tdi_shift_reg_next[9]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[11] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[11]),
        .Q(tdi_shift_reg_next[10]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[12] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[12]),
        .Q(tdi_shift_reg_next[11]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[13] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[13]),
        .Q(tdi_shift_reg_next[12]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[14] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[14]),
        .Q(tdi_shift_reg_next[13]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[15] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[15]),
        .Q(tdi_shift_reg_next[14]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[16] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[16]),
        .Q(tdi_shift_reg_next[15]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[17] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[17]),
        .Q(tdi_shift_reg_next[16]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[18] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[18]),
        .Q(tdi_shift_reg_next[17]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[19] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[19]),
        .Q(tdi_shift_reg_next[18]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[1] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[1]),
        .Q(tdi_shift_reg_next[0]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[20] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[20]),
        .Q(tdi_shift_reg_next[19]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[21] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[21]),
        .Q(tdi_shift_reg_next[20]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[22] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[22]),
        .Q(tdi_shift_reg_next[21]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[23] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[23]),
        .Q(tdi_shift_reg_next[22]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[24] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg[25]),
        .Q(tdi_shift_reg_next[23]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[25] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg[26]),
        .Q(tdi_shift_reg[25]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[26] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg[27]),
        .Q(tdi_shift_reg[26]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[27] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg[28]),
        .Q(tdi_shift_reg[27]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[28] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg[29]),
        .Q(tdi_shift_reg[28]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[29] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg[30]),
        .Q(tdi_shift_reg[29]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[2] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[2]),
        .Q(tdi_shift_reg_next[1]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[30] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg[31]),
        .Q(tdi_shift_reg[30]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[31] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi),
        .Q(tdi_shift_reg[31]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[3] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[3]),
        .Q(tdi_shift_reg_next[2]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[4] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[4]),
        .Q(tdi_shift_reg_next[3]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[5] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[5]),
        .Q(tdi_shift_reg_next[4]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[6] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[6]),
        .Q(tdi_shift_reg_next[5]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[7] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[7]),
        .Q(tdi_shift_reg_next[6]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[8] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[8]),
        .Q(tdi_shift_reg_next[7]),
        .R(u_jtag_n_15));
  FDRE \tdi_shift_reg_reg[9] 
       (.C(tck),
        .CE(shift_dr),
        .D(tdi_shift_reg_next[9]),
        .Q(tdi_shift_reg_next[8]),
        .R(u_jtag_n_15));
  BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_vendor_ip1 u_jtag
       (.CLK(tck),
        .D(state_reg__1),
        .E(pad_len),
        .\FSM_sequential_state_reg_reg[0] (u_jtag_n_7),
        .\FSM_sequential_state_reg_reg[0]_0 (counter_0),
        .\FSM_sequential_state_reg_reg[0]_1 (u_jtag_n_22),
        .\FSM_sequential_state_reg_reg[0]_2 (\FSM_sequential_state_reg[3]_i_5_n_0 ),
        .\FSM_sequential_state_reg_reg[0]_3 (\FSM_sequential_state_reg[3]_i_10_n_0 ),
        .\FSM_sequential_state_reg_reg[0]_4 (\FSM_sequential_state_reg[0]_i_2_n_0 ),
        .\FSM_sequential_state_reg_reg[1] (command_reg_1),
        .\FSM_sequential_state_reg_reg[1]_0 (\FSM_sequential_state_reg[1]_i_2_n_0 ),
        .\FSM_sequential_state_reg_reg[2] (u_jtag_n_5),
        .\FSM_sequential_state_reg_reg[2]_0 (u_jtag_n_6),
        .\FSM_sequential_state_reg_reg[2]_1 (u_jtag_n_20),
        .\FSM_sequential_state_reg_reg[2]_2 (\FSM_sequential_state_reg[2]_i_2_n_0 ),
        .\FSM_sequential_state_reg_reg[3] (u_jtag_n_18),
        .Q(state_reg__0),
        .SR(jtag_reset),
        .\counter_reg[0] (\counter[7]_i_3_n_0 ),
        .\pad_len_reg[0] (\pad_len[2]_i_3_n_0 ),
        .\read_reg_reg[5] (\FSM_sequential_state_reg[3]_i_9_n_0 ),
        .\reg_addr_reg[0] (\reg_addr[4]_i_2_n_0 ),
        .\reg_addr_reg[0]_0 (tdi_shift_reg[31:30]),
        .\reg_addr_reg[0]_1 (\FSM_sequential_state_reg[2]_i_3_n_0 ),
        .\reg_wdata_reg[0] (\counter[7]_i_4_n_0 ),
        .\reg_wdata_reg[0]_0 (counter[7:5]),
        .reg_write(reg_write),
        .reg_write_reg(reg_write_i_3_n_0),
        .shift_dr(shift_dr),
        .shift_in_data_reg(shift_in_data_i_2_n_0),
        .shift_in_data_reg_0(D),
        .shift_in_en_reg(shift_in_en),
        .shift_out_en_reg(shift_out_en),
        .tdi(tdi),
        .\tdi_shift_reg_reg[31] (u_jtag_n_9),
        .u_BSCANE2_0(u_jtag_n_15),
        .u_BSCANE2_1(u_jtag_n_16),
        .u_BSCANE2_2(command_reg[7]),
        .u_BSCANE2_3(u_jtag_n_19),
        .u_BSCANE2_4(u_jtag_n_21),
        .u_BSCANE2_5(u_BSCANE2),
        .u_BSCANE2_6(p_0_in),
        .u_BSCANE2_7(\read_reg_reg_n_0_[0] ));
  (* SOFT_HLUTNM = "soft_lutpair43" *) 
  LUT4 #(
    .INIT(16'hEFFF)) 
    \write_count[0]_i_1 
       (.I0(tck_reset),
        .I1(p_0_in[1]),
        .I2(p_0_in[2]),
        .I3(p_0_in[0]),
        .O(tck_reset_reg_0));
  LUT5 #(
    .INIT(32'h01000000)) 
    writeram_wr_i_1
       (.I0(tck_reset_reg_0),
        .I1(writeram_wr_i_2_n_0),
        .I2(S[1]),
        .I3(writeram_wr_reg[3]),
        .I4(writeram_wr_reg[1]),
        .O(\write_count_reg[15] ));
  LUT6 #(
    .INIT(64'hDFFFFFFFFFFFFFFF)) 
    writeram_wr_i_2
       (.I0(shift_in_en),
        .I1(S[0]),
        .I2(writeram_wr_reg_0),
        .I3(writeram_wr_reg[0]),
        .I4(writeram_wr_reg[2]),
        .I5(writeram_wr_reg[4]),
        .O(writeram_wr_i_2_n_0));
endmodule

(* ORIG_REF_NAME = "hdlverifier_jtag_vendor_ip1" *) 
module BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_hdlverifier_jtag_vendor_ip1
   (SR,
    CLK,
    tdi,
    shift_dr,
    E,
    \FSM_sequential_state_reg_reg[2] ,
    \FSM_sequential_state_reg_reg[2]_0 ,
    \FSM_sequential_state_reg_reg[0] ,
    \FSM_sequential_state_reg_reg[0]_0 ,
    \tdi_shift_reg_reg[31] ,
    \FSM_sequential_state_reg_reg[1] ,
    D,
    u_BSCANE2_0,
    u_BSCANE2_1,
    u_BSCANE2_2,
    \FSM_sequential_state_reg_reg[3] ,
    u_BSCANE2_3,
    \FSM_sequential_state_reg_reg[2]_1 ,
    u_BSCANE2_4,
    \FSM_sequential_state_reg_reg[0]_1 ,
    \counter_reg[0] ,
    Q,
    \pad_len_reg[0] ,
    \FSM_sequential_state_reg_reg[0]_2 ,
    \FSM_sequential_state_reg_reg[0]_3 ,
    \FSM_sequential_state_reg_reg[1]_0 ,
    \reg_wdata_reg[0] ,
    \reg_wdata_reg[0]_0 ,
    \read_reg_reg[5] ,
    \reg_addr_reg[0] ,
    \reg_addr_reg[0]_0 ,
    \reg_addr_reg[0]_1 ,
    \FSM_sequential_state_reg_reg[2]_2 ,
    \FSM_sequential_state_reg_reg[0]_4 ,
    u_BSCANE2_5,
    u_BSCANE2_6,
    u_BSCANE2_7,
    reg_write_reg,
    reg_write,
    shift_out_en_reg,
    shift_in_en_reg,
    shift_in_data_reg,
    shift_in_data_reg_0);
  output [0:0]SR;
  output CLK;
  output tdi;
  output shift_dr;
  output [0:0]E;
  output [0:0]\FSM_sequential_state_reg_reg[2] ;
  output [0:0]\FSM_sequential_state_reg_reg[2]_0 ;
  output \FSM_sequential_state_reg_reg[0] ;
  output [0:0]\FSM_sequential_state_reg_reg[0]_0 ;
  output [0:0]\tdi_shift_reg_reg[31] ;
  output [0:0]\FSM_sequential_state_reg_reg[1] ;
  output [3:0]D;
  output [0:0]u_BSCANE2_0;
  output [0:0]u_BSCANE2_1;
  output [0:0]u_BSCANE2_2;
  output \FSM_sequential_state_reg_reg[3] ;
  output u_BSCANE2_3;
  output \FSM_sequential_state_reg_reg[2]_1 ;
  output u_BSCANE2_4;
  output \FSM_sequential_state_reg_reg[0]_1 ;
  input \counter_reg[0] ;
  input [3:0]Q;
  input \pad_len_reg[0] ;
  input \FSM_sequential_state_reg_reg[0]_2 ;
  input \FSM_sequential_state_reg_reg[0]_3 ;
  input \FSM_sequential_state_reg_reg[1]_0 ;
  input \reg_wdata_reg[0] ;
  input [2:0]\reg_wdata_reg[0]_0 ;
  input \read_reg_reg[5] ;
  input \reg_addr_reg[0] ;
  input [1:0]\reg_addr_reg[0]_0 ;
  input \reg_addr_reg[0]_1 ;
  input \FSM_sequential_state_reg_reg[2]_2 ;
  input \FSM_sequential_state_reg_reg[0]_4 ;
  input [0:0]u_BSCANE2_5;
  input [2:0]u_BSCANE2_6;
  input u_BSCANE2_7;
  input reg_write_reg;
  input reg_write;
  input shift_out_en_reg;
  input shift_in_en_reg;
  input shift_in_data_reg;
  input [0:0]shift_in_data_reg_0;

  wire CLK;
  wire [3:0]D;
  wire [0:0]E;
  wire \FSM_sequential_state_reg[0]_i_3_n_0 ;
  wire \FSM_sequential_state_reg[3]_i_3_n_0 ;
  wire \FSM_sequential_state_reg[3]_i_4_n_0 ;
  wire \FSM_sequential_state_reg[3]_i_6_n_0 ;
  wire \FSM_sequential_state_reg[3]_i_7_n_0 ;
  wire \FSM_sequential_state_reg[3]_i_8_n_0 ;
  wire \FSM_sequential_state_reg_reg[0] ;
  wire [0:0]\FSM_sequential_state_reg_reg[0]_0 ;
  wire \FSM_sequential_state_reg_reg[0]_1 ;
  wire \FSM_sequential_state_reg_reg[0]_2 ;
  wire \FSM_sequential_state_reg_reg[0]_3 ;
  wire \FSM_sequential_state_reg_reg[0]_4 ;
  wire [0:0]\FSM_sequential_state_reg_reg[1] ;
  wire \FSM_sequential_state_reg_reg[1]_0 ;
  wire [0:0]\FSM_sequential_state_reg_reg[2] ;
  wire [0:0]\FSM_sequential_state_reg_reg[2]_0 ;
  wire \FSM_sequential_state_reg_reg[2]_1 ;
  wire \FSM_sequential_state_reg_reg[2]_2 ;
  wire \FSM_sequential_state_reg_reg[3] ;
  wire [3:0]Q;
  wire [0:0]SR;
  wire cdr;
  wire \counter_reg[0] ;
  wire \pad_len_reg[0] ;
  wire \read_reg_reg[5] ;
  wire \reg_addr[4]_i_3_n_0 ;
  wire \reg_addr_reg[0] ;
  wire [1:0]\reg_addr_reg[0]_0 ;
  wire \reg_addr_reg[0]_1 ;
  wire \reg_wdata_reg[0] ;
  wire [2:0]\reg_wdata_reg[0]_0 ;
  wire reg_write;
  wire reg_write_i_2_n_0;
  wire reg_write_reg;
  wire sdr;
  wire sel;
  wire shift_dr;
  wire shift_in_data_reg;
  wire [0:0]shift_in_data_reg_0;
  wire shift_in_en_reg;
  wire shift_out_en_i_2_n_0;
  wire shift_out_en_reg;
  wire tdi;
  wire [0:0]\tdi_shift_reg_reg[31] ;
  wire tdo;
  wire [0:0]u_BSCANE2_0;
  wire [0:0]u_BSCANE2_1;
  wire [0:0]u_BSCANE2_2;
  wire u_BSCANE2_3;
  wire u_BSCANE2_4;
  wire [0:0]u_BSCANE2_5;
  wire [2:0]u_BSCANE2_6;
  wire u_BSCANE2_7;
  wire udr;
  wire NLW_u_BSCANE2_DRCK_UNCONNECTED;
  wire NLW_u_BSCANE2_RUNTEST_UNCONNECTED;
  wire NLW_u_BSCANE2_TMS_UNCONNECTED;

  LUT6 #(
    .INIT(64'h00000003050F000F)) 
    \FSM_sequential_state_reg[0]_i_1 
       (.I0(\FSM_sequential_state_reg_reg[0]_4 ),
        .I1(\FSM_sequential_state_reg[0]_i_3_n_0 ),
        .I2(Q[3]),
        .I3(Q[0]),
        .I4(Q[2]),
        .I5(Q[1]),
        .O(D[0]));
  (* SOFT_HLUTNM = "soft_lutpair35" *) 
  LUT3 #(
    .INIT(8'hFB)) 
    \FSM_sequential_state_reg[0]_i_3 
       (.I0(tdi),
        .I1(\reg_addr_reg[0]_0 [0]),
        .I2(\reg_addr_reg[0]_0 [1]),
        .O(\FSM_sequential_state_reg[0]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h1111111111F11111)) 
    \FSM_sequential_state_reg[1]_i_1 
       (.I0(\FSM_sequential_state_reg_reg[1]_0 ),
        .I1(Q[1]),
        .I2(\reg_addr_reg[0]_1 ),
        .I3(tdi),
        .I4(\reg_addr_reg[0]_0 [0]),
        .I5(\reg_addr_reg[0]_0 [1]),
        .O(D[1]));
  LUT6 #(
    .INIT(64'h888F8F8888888888)) 
    \FSM_sequential_state_reg[2]_i_1 
       (.I0(Q[2]),
        .I1(\FSM_sequential_state_reg_reg[2]_2 ),
        .I2(\reg_addr_reg[0]_0 [0]),
        .I3(tdi),
        .I4(\reg_addr_reg[0]_0 [1]),
        .I5(\reg_addr_reg[0]_1 ),
        .O(D[2]));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFFFEAFF)) 
    \FSM_sequential_state_reg[3]_i_1 
       (.I0(\FSM_sequential_state_reg[3]_i_3_n_0 ),
        .I1(\FSM_sequential_state_reg[3]_i_4_n_0 ),
        .I2(\FSM_sequential_state_reg_reg[0]_2 ),
        .I3(\FSM_sequential_state_reg[3]_i_6_n_0 ),
        .I4(\FSM_sequential_state_reg[3]_i_7_n_0 ),
        .I5(\FSM_sequential_state_reg[3]_i_8_n_0 ),
        .O(\FSM_sequential_state_reg_reg[2] ));
  LUT6 #(
    .INIT(64'h0000010000000000)) 
    \FSM_sequential_state_reg[3]_i_2 
       (.I0(Q[0]),
        .I1(Q[2]),
        .I2(\read_reg_reg[5] ),
        .I3(\reg_addr_reg[0]_0 [0]),
        .I4(\reg_addr_reg[0]_0 [1]),
        .I5(tdi),
        .O(D[3]));
  LUT6 #(
    .INIT(64'h000000000000F200)) 
    \FSM_sequential_state_reg[3]_i_3 
       (.I0(Q[2]),
        .I1(\FSM_sequential_state_reg_reg[0]_3 ),
        .I2(\pad_len_reg[0] ),
        .I3(shift_dr),
        .I4(Q[1]),
        .I5(\FSM_sequential_state_reg_reg[1]_0 ),
        .O(\FSM_sequential_state_reg[3]_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair32" *) 
  LUT4 #(
    .INIT(16'h0080)) 
    \FSM_sequential_state_reg[3]_i_4 
       (.I0(sel),
        .I1(sdr),
        .I2(Q[1]),
        .I3(Q[3]),
        .O(\FSM_sequential_state_reg[3]_i_4_n_0 ));
  LUT6 #(
    .INIT(64'hFFFEFFFFFFFFFFFF)) 
    \FSM_sequential_state_reg[3]_i_6 
       (.I0(\reg_wdata_reg[0] ),
        .I1(\reg_wdata_reg[0]_0 [0]),
        .I2(\reg_wdata_reg[0]_0 [1]),
        .I3(\reg_wdata_reg[0]_0 [2]),
        .I4(Q[0]),
        .I5(\FSM_sequential_state_reg[3]_i_4_n_0 ),
        .O(\FSM_sequential_state_reg[3]_i_6_n_0 ));
  LUT6 #(
    .INIT(64'h0024000000000000)) 
    \FSM_sequential_state_reg[3]_i_7 
       (.I0(Q[1]),
        .I1(Q[3]),
        .I2(Q[2]),
        .I3(Q[0]),
        .I4(udr),
        .I5(sel),
        .O(\FSM_sequential_state_reg[3]_i_7_n_0 ));
  LUT5 #(
    .INIT(32'h00000008)) 
    \FSM_sequential_state_reg[3]_i_8 
       (.I0(cdr),
        .I1(sel),
        .I2(Q[3]),
        .I3(Q[0]),
        .I4(Q[1]),
        .O(\FSM_sequential_state_reg[3]_i_8_n_0 ));
  LUT6 #(
    .INIT(64'h0000000D00000005)) 
    \command_reg[7]_i_1 
       (.I0(Q[1]),
        .I1(shift_dr),
        .I2(Q[3]),
        .I3(Q[0]),
        .I4(Q[2]),
        .I5(\reg_addr_reg[0] ),
        .O(\FSM_sequential_state_reg_reg[1] ));
  (* SOFT_HLUTNM = "soft_lutpair35" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \command_reg[7]_i_2 
       (.I0(tdi),
        .I1(Q[1]),
        .O(u_BSCANE2_2));
  LUT6 #(
    .INIT(64'h0E0F00000C030003)) 
    \counter[7]_i_1 
       (.I0(\counter_reg[0] ),
        .I1(Q[0]),
        .I2(Q[3]),
        .I3(Q[2]),
        .I4(shift_dr),
        .I5(Q[1]),
        .O(\FSM_sequential_state_reg_reg[0]_0 ));
  LUT6 #(
    .INIT(64'h1000100010001111)) 
    \pad_len[2]_i_1 
       (.I0(Q[1]),
        .I1(Q[3]),
        .I2(\pad_len_reg[0] ),
        .I3(shift_dr),
        .I4(Q[0]),
        .I5(Q[2]),
        .O(E));
  (* SOFT_HLUTNM = "soft_lutpair34" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pad_len[2]_i_2 
       (.I0(tdi),
        .I1(Q[0]),
        .O(u_BSCANE2_1));
  (* SOFT_HLUTNM = "soft_lutpair34" *) 
  LUT3 #(
    .INIT(8'hD0)) 
    \read_reg[21]_i_1 
       (.I0(Q[0]),
        .I1(Q[3]),
        .I2(\FSM_sequential_state_reg_reg[0] ),
        .O(\FSM_sequential_state_reg_reg[0]_1 ));
  LUT6 #(
    .INIT(64'h00000000FF200000)) 
    \read_reg[28]_i_1 
       (.I0(Q[0]),
        .I1(\read_reg_reg[5] ),
        .I2(shift_dr),
        .I3(\FSM_sequential_state_reg[3]_i_8_n_0 ),
        .I4(Q[2]),
        .I5(SR),
        .O(\FSM_sequential_state_reg_reg[0] ));
  LUT6 #(
    .INIT(64'h0000022000000000)) 
    \reg_addr[4]_i_1 
       (.I0(\reg_addr_reg[0] ),
        .I1(\reg_addr[4]_i_3_n_0 ),
        .I2(\reg_addr_reg[0]_0 [1]),
        .I3(\reg_addr_reg[0]_0 [0]),
        .I4(tdi),
        .I5(\reg_addr_reg[0]_1 ),
        .O(\tdi_shift_reg_reg[31] ));
  (* SOFT_HLUTNM = "soft_lutpair32" *) 
  LUT3 #(
    .INIT(8'hBF)) 
    \reg_addr[4]_i_3 
       (.I0(SR),
        .I1(sel),
        .I2(sdr),
        .O(\reg_addr[4]_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair33" *) 
  LUT3 #(
    .INIT(8'h01)) 
    \reg_wdata[31]_i_1 
       (.I0(\FSM_sequential_state_reg[3]_i_6_n_0 ),
        .I1(Q[2]),
        .I2(SR),
        .O(\FSM_sequential_state_reg_reg[2]_0 ));
  LUT6 #(
    .INIT(64'h33BB333000880000)) 
    reg_write_i_1
       (.I0(shift_dr),
        .I1(reg_write_i_2_n_0),
        .I2(reg_write_reg),
        .I3(Q[3]),
        .I4(Q[1]),
        .I5(reg_write),
        .O(\FSM_sequential_state_reg_reg[3] ));
  (* SOFT_HLUTNM = "soft_lutpair33" *) 
  LUT2 #(
    .INIT(4'h1)) 
    reg_write_i_2
       (.I0(Q[2]),
        .I1(\FSM_sequential_state_reg[3]_i_6_n_0 ),
        .O(reg_write_i_2_n_0));
  LUT6 #(
    .INIT(64'hFFFFBFFF00008000)) 
    shift_in_data_i_1
       (.I0(tdi),
        .I1(shift_in_data_reg),
        .I2(sdr),
        .I3(sel),
        .I4(SR),
        .I5(shift_in_data_reg_0),
        .O(u_BSCANE2_4));
  LUT6 #(
    .INIT(64'hFFFFFEFF00000200)) 
    shift_in_en_i_1
       (.I0(shift_dr),
        .I1(Q[2]),
        .I2(Q[0]),
        .I3(Q[3]),
        .I4(Q[1]),
        .I5(shift_in_en_reg),
        .O(\FSM_sequential_state_reg_reg[2]_1 ));
  LUT6 #(
    .INIT(64'h0080FFFF00800000)) 
    shift_out_en_i_1
       (.I0(sel),
        .I1(sdr),
        .I2(Q[2]),
        .I3(Q[3]),
        .I4(shift_out_en_i_2_n_0),
        .I5(shift_out_en_reg),
        .O(u_BSCANE2_3));
  LUT6 #(
    .INIT(64'h000000000070000F)) 
    shift_out_en_i_2
       (.I0(\counter_reg[0] ),
        .I1(shift_dr),
        .I2(Q[1]),
        .I3(Q[3]),
        .I4(Q[2]),
        .I5(Q[0]),
        .O(shift_out_en_i_2_n_0));
  (* SOFT_HLUTNM = "soft_lutpair36" *) 
  LUT3 #(
    .INIT(8'hEA)) 
    \tdi_shift_reg[31]_i_1 
       (.I0(SR),
        .I1(cdr),
        .I2(sel),
        .O(u_BSCANE2_0));
  (* SOFT_HLUTNM = "soft_lutpair36" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \tdi_shift_reg[31]_i_2 
       (.I0(sdr),
        .I1(sel),
        .O(shift_dr));
  (* BOX_TYPE = "PRIMITIVE" *) 
  BSCANE2 #(
    .DISABLE_JTAG("FALSE"),
    .JTAG_CHAIN(3)) 
    u_BSCANE2
       (.CAPTURE(cdr),
        .DRCK(NLW_u_BSCANE2_DRCK_UNCONNECTED),
        .RESET(SR),
        .RUNTEST(NLW_u_BSCANE2_RUNTEST_UNCONNECTED),
        .SEL(sel),
        .SHIFT(sdr),
        .TCK(CLK),
        .TDI(tdi),
        .TDO(tdo),
        .TMS(NLW_u_BSCANE2_TMS_UNCONNECTED),
        .UPDATE(udr));
  LUT5 #(
    .INIT(32'hFFEF0020)) 
    u_BSCANE2_i_1
       (.I0(u_BSCANE2_5),
        .I1(u_BSCANE2_6[0]),
        .I2(u_BSCANE2_6[2]),
        .I3(u_BSCANE2_6[1]),
        .I4(u_BSCANE2_7),
        .O(tdo));
endmodule

(* ORIG_REF_NAME = "jtag_master_dcram" *) 
module BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram
   (D,
    \bitcount_reg[3] ,
    tck,
    aclk,
    readram_rd,
    Q,
    ADDRBWRADDR,
    axi4m_rdata,
    axi4m_rvalid,
    \shift_reg_reg[62] ,
    \shift_reg_reg[0] ,
    ram_reg_0);
  output [63:0]D;
  output \bitcount_reg[3] ;
  input tck;
  input aclk;
  input readram_rd;
  input [7:0]Q;
  input [7:0]ADDRBWRADDR;
  input [63:0]axi4m_rdata;
  input axi4m_rvalid;
  input [62:0]\shift_reg_reg[62] ;
  input \shift_reg_reg[0] ;
  input [4:0]ram_reg_0;

  wire [7:0]ADDRBWRADDR;
  wire [63:0]D;
  wire [7:0]Q;
  wire aclk;
  wire [63:0]axi4m_rdata;
  wire axi4m_rvalid;
  wire \bitcount_reg[3] ;
  wire [4:0]ram_reg_0;
  wire [63:0]readram_q;
  wire readram_rd;
  wire \shift_reg_reg[0] ;
  wire [62:0]\shift_reg_reg[62] ;
  wire tck;
  wire NLW_ram_reg_CASOUTDBITERR_UNCONNECTED;
  wire NLW_ram_reg_CASOUTSBITERR_UNCONNECTED;
  wire NLW_ram_reg_DBITERR_UNCONNECTED;
  wire NLW_ram_reg_SBITERR_UNCONNECTED;
  wire [31:0]NLW_ram_reg_CASDINA_UNCONNECTED;
  wire [31:0]NLW_ram_reg_CASDINB_UNCONNECTED;
  wire [3:0]NLW_ram_reg_CASDINPA_UNCONNECTED;
  wire [3:0]NLW_ram_reg_CASDINPB_UNCONNECTED;
  wire [31:0]NLW_ram_reg_CASDOUTA_UNCONNECTED;
  wire [31:0]NLW_ram_reg_CASDOUTB_UNCONNECTED;
  wire [3:0]NLW_ram_reg_CASDOUTPA_UNCONNECTED;
  wire [3:0]NLW_ram_reg_CASDOUTPB_UNCONNECTED;
  wire [3:0]NLW_ram_reg_DOUTPADOUTP_UNCONNECTED;
  wire [3:0]NLW_ram_reg_DOUTPBDOUTP_UNCONNECTED;
  wire [7:0]NLW_ram_reg_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_ram_reg_RDADDRECC_UNCONNECTED;

  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d64" *) 
  (* \MEM.PORTB.DATA_BIT_LAYOUT  = "p0_d64" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RDADDR_COLLISION_HWCONFIG = "DELAYED_WRITE" *) 
  (* RTL_RAM_BITS = "16384" *) 
  (* RTL_RAM_NAME = "inst/readram/ram_reg" *) 
  (* RTL_RAM_TYPE = "RAM_SDP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "511" *) 
  (* ram_offset = "256" *) 
  (* ram_slice_begin = "0" *) 
  (* ram_slice_end = "63" *) 
  RAMB36E2 #(
    .CASCADE_ORDER_A("NONE"),
    .CASCADE_ORDER_B("NONE"),
    .CLOCK_DOMAINS("INDEPENDENT"),
    .DOA_REG(0),
    .DOB_REG(0),
    .ENADDRENA("FALSE"),
    .ENADDRENB("FALSE"),
    .EN_ECC_PIPE("FALSE"),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .INIT_FILE("NONE"),
    .RDADDRCHANGEA("FALSE"),
    .RDADDRCHANGEB("FALSE"),
    .READ_WIDTH_A(72),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SLEEP_ASYNC("FALSE"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(0),
    .WRITE_WIDTH_B(72)) 
    ram_reg
       (.ADDRARDADDR({1'b1,Q,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .ADDRBWRADDR({1'b1,ADDRBWRADDR,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .ADDRENA(1'b1),
        .ADDRENB(1'b1),
        .CASDIMUXA(1'b0),
        .CASDIMUXB(1'b0),
        .CASDINA(NLW_ram_reg_CASDINA_UNCONNECTED[31:0]),
        .CASDINB(NLW_ram_reg_CASDINB_UNCONNECTED[31:0]),
        .CASDINPA(NLW_ram_reg_CASDINPA_UNCONNECTED[3:0]),
        .CASDINPB(NLW_ram_reg_CASDINPB_UNCONNECTED[3:0]),
        .CASDOMUXA(1'b0),
        .CASDOMUXB(1'b0),
        .CASDOMUXEN_A(1'b1),
        .CASDOMUXEN_B(1'b1),
        .CASDOUTA(NLW_ram_reg_CASDOUTA_UNCONNECTED[31:0]),
        .CASDOUTB(NLW_ram_reg_CASDOUTB_UNCONNECTED[31:0]),
        .CASDOUTPA(NLW_ram_reg_CASDOUTPA_UNCONNECTED[3:0]),
        .CASDOUTPB(NLW_ram_reg_CASDOUTPB_UNCONNECTED[3:0]),
        .CASINDBITERR(1'b0),
        .CASINSBITERR(1'b0),
        .CASOREGIMUXA(1'b0),
        .CASOREGIMUXB(1'b0),
        .CASOREGIMUXEN_A(1'b1),
        .CASOREGIMUXEN_B(1'b1),
        .CASOUTDBITERR(NLW_ram_reg_CASOUTDBITERR_UNCONNECTED),
        .CASOUTSBITERR(NLW_ram_reg_CASOUTSBITERR_UNCONNECTED),
        .CLKARDCLK(tck),
        .CLKBWRCLK(aclk),
        .DBITERR(NLW_ram_reg_DBITERR_UNCONNECTED),
        .DINADIN(axi4m_rdata[31:0]),
        .DINBDIN(axi4m_rdata[63:32]),
        .DINPADINP({1'b1,1'b1,1'b1,1'b1}),
        .DINPBDINP({1'b1,1'b1,1'b1,1'b1}),
        .DOUTADOUT(readram_q[31:0]),
        .DOUTBDOUT(readram_q[63:32]),
        .DOUTPADOUTP(NLW_ram_reg_DOUTPADOUTP_UNCONNECTED[3:0]),
        .DOUTPBDOUTP(NLW_ram_reg_DOUTPBDOUTP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_ram_reg_ECCPARITY_UNCONNECTED[7:0]),
        .ECCPIPECE(1'b1),
        .ENARDEN(readram_rd),
        .ENBWREN(1'b1),
        .INJECTDBITERR(1'b0),
        .INJECTSBITERR(1'b0),
        .RDADDRECC(NLW_ram_reg_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b1),
        .REGCEB(1'b1),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_ram_reg_SBITERR_UNCONNECTED),
        .SLEEP(1'b0),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({axi4m_rvalid,axi4m_rvalid,axi4m_rvalid,axi4m_rvalid,axi4m_rvalid,axi4m_rvalid,axi4m_rvalid,axi4m_rvalid}));
  LUT5 #(
    .INIT(32'h00000001)) 
    ram_reg_i_2__0
       (.I0(ram_reg_0[2]),
        .I1(ram_reg_0[3]),
        .I2(ram_reg_0[1]),
        .I3(ram_reg_0[0]),
        .I4(ram_reg_0[4]),
        .O(\bitcount_reg[3] ));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[0]_i_1 
       (.I0(\shift_reg_reg[62] [0]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[0]),
        .O(D[0]));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[10]_i_1 
       (.I0(\shift_reg_reg[62] [10]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[10]),
        .O(D[10]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[11]_i_1 
       (.I0(\shift_reg_reg[62] [11]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[11]),
        .O(D[11]));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[12]_i_1 
       (.I0(\shift_reg_reg[62] [12]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[12]),
        .O(D[12]));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[13]_i_1 
       (.I0(\shift_reg_reg[62] [13]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[13]),
        .O(D[13]));
  (* SOFT_HLUTNM = "soft_lutpair28" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[14]_i_1 
       (.I0(\shift_reg_reg[62] [14]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[14]),
        .O(D[14]));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[15]_i_1 
       (.I0(\shift_reg_reg[62] [15]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[15]),
        .O(D[15]));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[16]_i_1 
       (.I0(\shift_reg_reg[62] [16]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[16]),
        .O(D[16]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[17]_i_1 
       (.I0(\shift_reg_reg[62] [17]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[17]),
        .O(D[17]));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[18]_i_1 
       (.I0(\shift_reg_reg[62] [18]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[18]),
        .O(D[18]));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[19]_i_1 
       (.I0(\shift_reg_reg[62] [19]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[19]),
        .O(D[19]));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[1]_i_1 
       (.I0(\shift_reg_reg[62] [1]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[1]),
        .O(D[1]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[20]_i_1 
       (.I0(\shift_reg_reg[62] [20]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[20]),
        .O(D[20]));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[21]_i_1 
       (.I0(\shift_reg_reg[62] [21]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[21]),
        .O(D[21]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[22]_i_1 
       (.I0(\shift_reg_reg[62] [22]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[22]),
        .O(D[22]));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[23]_i_1 
       (.I0(\shift_reg_reg[62] [23]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[23]),
        .O(D[23]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[24]_i_1 
       (.I0(\shift_reg_reg[62] [24]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[24]),
        .O(D[24]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[25]_i_1 
       (.I0(\shift_reg_reg[62] [25]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[25]),
        .O(D[25]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[26]_i_1 
       (.I0(\shift_reg_reg[62] [26]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[26]),
        .O(D[26]));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[27]_i_1 
       (.I0(\shift_reg_reg[62] [27]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[27]),
        .O(D[27]));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[28]_i_1 
       (.I0(\shift_reg_reg[62] [28]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[28]),
        .O(D[28]));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[29]_i_1 
       (.I0(\shift_reg_reg[62] [29]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[29]),
        .O(D[29]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[2]_i_1 
       (.I0(\shift_reg_reg[62] [2]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[2]),
        .O(D[2]));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[30]_i_1 
       (.I0(\shift_reg_reg[62] [30]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[30]),
        .O(D[30]));
  (* SOFT_HLUTNM = "soft_lutpair26" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[31]_i_1 
       (.I0(\shift_reg_reg[62] [31]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[31]),
        .O(D[31]));
  (* SOFT_HLUTNM = "soft_lutpair31" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[32]_i_1 
       (.I0(\shift_reg_reg[62] [32]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[32]),
        .O(D[32]));
  (* SOFT_HLUTNM = "soft_lutpair30" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[33]_i_1 
       (.I0(\shift_reg_reg[62] [33]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[33]),
        .O(D[33]));
  (* SOFT_HLUTNM = "soft_lutpair26" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[34]_i_1 
       (.I0(\shift_reg_reg[62] [34]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[34]),
        .O(D[34]));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[35]_i_1 
       (.I0(\shift_reg_reg[62] [35]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[35]),
        .O(D[35]));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[36]_i_1 
       (.I0(\shift_reg_reg[62] [36]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[36]),
        .O(D[36]));
  (* SOFT_HLUTNM = "soft_lutpair28" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[37]_i_1 
       (.I0(\shift_reg_reg[62] [37]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[37]),
        .O(D[37]));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[38]_i_1 
       (.I0(\shift_reg_reg[62] [38]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[38]),
        .O(D[38]));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[39]_i_1 
       (.I0(\shift_reg_reg[62] [39]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[39]),
        .O(D[39]));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[3]_i_1 
       (.I0(\shift_reg_reg[62] [3]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[3]),
        .O(D[3]));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[40]_i_1 
       (.I0(\shift_reg_reg[62] [40]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[40]),
        .O(D[40]));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[41]_i_1 
       (.I0(\shift_reg_reg[62] [41]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[41]),
        .O(D[41]));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[42]_i_1 
       (.I0(\shift_reg_reg[62] [42]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[42]),
        .O(D[42]));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[43]_i_1 
       (.I0(\shift_reg_reg[62] [43]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[43]),
        .O(D[43]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[44]_i_1 
       (.I0(\shift_reg_reg[62] [44]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[44]),
        .O(D[44]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[45]_i_1 
       (.I0(\shift_reg_reg[62] [45]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[45]),
        .O(D[45]));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[46]_i_1 
       (.I0(\shift_reg_reg[62] [46]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[46]),
        .O(D[46]));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[47]_i_1 
       (.I0(\shift_reg_reg[62] [47]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[47]),
        .O(D[47]));
  (* SOFT_HLUTNM = "soft_lutpair29" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[48]_i_1 
       (.I0(\shift_reg_reg[62] [48]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[48]),
        .O(D[48]));
  (* SOFT_HLUTNM = "soft_lutpair31" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[49]_i_1 
       (.I0(\shift_reg_reg[62] [49]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[49]),
        .O(D[49]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[4]_i_1 
       (.I0(\shift_reg_reg[62] [4]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[4]),
        .O(D[4]));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[50]_i_1 
       (.I0(\shift_reg_reg[62] [50]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[50]),
        .O(D[50]));
  (* SOFT_HLUTNM = "soft_lutpair29" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[51]_i_1 
       (.I0(\shift_reg_reg[62] [51]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[51]),
        .O(D[51]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[52]_i_1 
       (.I0(\shift_reg_reg[62] [52]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[52]),
        .O(D[52]));
  (* SOFT_HLUTNM = "soft_lutpair27" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[53]_i_1 
       (.I0(\shift_reg_reg[62] [53]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[53]),
        .O(D[53]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[54]_i_1 
       (.I0(\shift_reg_reg[62] [54]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[54]),
        .O(D[54]));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[55]_i_1 
       (.I0(\shift_reg_reg[62] [55]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[55]),
        .O(D[55]));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[56]_i_1 
       (.I0(\shift_reg_reg[62] [56]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[56]),
        .O(D[56]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[57]_i_1 
       (.I0(\shift_reg_reg[62] [57]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[57]),
        .O(D[57]));
  (* SOFT_HLUTNM = "soft_lutpair30" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[58]_i_1 
       (.I0(\shift_reg_reg[62] [58]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[58]),
        .O(D[58]));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[59]_i_1 
       (.I0(\shift_reg_reg[62] [59]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[59]),
        .O(D[59]));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[5]_i_1 
       (.I0(\shift_reg_reg[62] [5]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[5]),
        .O(D[5]));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[60]_i_1 
       (.I0(\shift_reg_reg[62] [60]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[60]),
        .O(D[60]));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[61]_i_1 
       (.I0(\shift_reg_reg[62] [61]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[61]),
        .O(D[61]));
  (* SOFT_HLUTNM = "soft_lutpair27" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[62]_i_1 
       (.I0(\shift_reg_reg[62] [62]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[62]),
        .O(D[62]));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \shift_reg[63]_i_2 
       (.I0(readram_q[63]),
        .I1(\shift_reg_reg[0] ),
        .O(D[63]));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[6]_i_1 
       (.I0(\shift_reg_reg[62] [6]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[6]),
        .O(D[6]));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[7]_i_1 
       (.I0(\shift_reg_reg[62] [7]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[7]),
        .O(D[7]));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[8]_i_1 
       (.I0(\shift_reg_reg[62] [8]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[8]),
        .O(D[8]));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \shift_reg[9]_i_1 
       (.I0(\shift_reg_reg[62] [9]),
        .I1(\shift_reg_reg[0] ),
        .I2(readram_q[9]),
        .O(D[9]));
endmodule

(* ORIG_REF_NAME = "jtag_master_dcram" *) 
module BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_jtag_master_dcram_0
   (axi4m_wdata,
    D,
    aclk,
    CLK,
    writeram_rd,
    Q,
    ram_reg_0,
    WEBWE,
    ram_reg_1,
    axi4m_wready,
    ram_reg_2);
  output [63:0]axi4m_wdata;
  output [7:0]D;
  input aclk;
  input CLK;
  input writeram_rd;
  input [7:0]Q;
  input [63:0]ram_reg_0;
  input [0:0]WEBWE;
  input [7:0]ram_reg_1;
  input axi4m_wready;
  input ram_reg_2;

  wire CLK;
  wire [7:0]D;
  wire [7:0]Q;
  wire [0:0]WEBWE;
  wire aclk;
  wire [63:0]axi4m_wdata;
  wire axi4m_wready;
  wire [63:0]ram_reg_0;
  wire [7:0]ram_reg_1;
  wire ram_reg_2;
  wire ram_reg_i_10_n_0;
  wire ram_reg_i_9_n_0;
  wire writeram_rd;
  wire NLW_ram_reg_CASOUTDBITERR_UNCONNECTED;
  wire NLW_ram_reg_CASOUTSBITERR_UNCONNECTED;
  wire NLW_ram_reg_DBITERR_UNCONNECTED;
  wire NLW_ram_reg_SBITERR_UNCONNECTED;
  wire [31:0]NLW_ram_reg_CASDINA_UNCONNECTED;
  wire [31:0]NLW_ram_reg_CASDINB_UNCONNECTED;
  wire [3:0]NLW_ram_reg_CASDINPA_UNCONNECTED;
  wire [3:0]NLW_ram_reg_CASDINPB_UNCONNECTED;
  wire [31:0]NLW_ram_reg_CASDOUTA_UNCONNECTED;
  wire [31:0]NLW_ram_reg_CASDOUTB_UNCONNECTED;
  wire [3:0]NLW_ram_reg_CASDOUTPA_UNCONNECTED;
  wire [3:0]NLW_ram_reg_CASDOUTPB_UNCONNECTED;
  wire [3:0]NLW_ram_reg_DOUTPADOUTP_UNCONNECTED;
  wire [3:0]NLW_ram_reg_DOUTPBDOUTP_UNCONNECTED;
  wire [7:0]NLW_ram_reg_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_ram_reg_RDADDRECC_UNCONNECTED;

  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d64" *) 
  (* \MEM.PORTB.DATA_BIT_LAYOUT  = "p0_d64" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RDADDR_COLLISION_HWCONFIG = "DELAYED_WRITE" *) 
  (* RTL_RAM_BITS = "16384" *) 
  (* RTL_RAM_NAME = "inst/writeram/ram_reg" *) 
  (* RTL_RAM_TYPE = "RAM_SDP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "511" *) 
  (* ram_offset = "256" *) 
  (* ram_slice_begin = "0" *) 
  (* ram_slice_end = "63" *) 
  RAMB36E2 #(
    .CASCADE_ORDER_A("NONE"),
    .CASCADE_ORDER_B("NONE"),
    .CLOCK_DOMAINS("INDEPENDENT"),
    .DOA_REG(0),
    .DOB_REG(0),
    .ENADDRENA("FALSE"),
    .ENADDRENB("FALSE"),
    .EN_ECC_PIPE("FALSE"),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .INIT_FILE("NONE"),
    .RDADDRCHANGEA("FALSE"),
    .RDADDRCHANGEB("FALSE"),
    .READ_WIDTH_A(72),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SLEEP_ASYNC("FALSE"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(0),
    .WRITE_WIDTH_B(72)) 
    ram_reg
       (.ADDRARDADDR({1'b1,D,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .ADDRBWRADDR({1'b1,Q,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .ADDRENA(1'b1),
        .ADDRENB(1'b1),
        .CASDIMUXA(1'b0),
        .CASDIMUXB(1'b0),
        .CASDINA(NLW_ram_reg_CASDINA_UNCONNECTED[31:0]),
        .CASDINB(NLW_ram_reg_CASDINB_UNCONNECTED[31:0]),
        .CASDINPA(NLW_ram_reg_CASDINPA_UNCONNECTED[3:0]),
        .CASDINPB(NLW_ram_reg_CASDINPB_UNCONNECTED[3:0]),
        .CASDOMUXA(1'b0),
        .CASDOMUXB(1'b0),
        .CASDOMUXEN_A(1'b1),
        .CASDOMUXEN_B(1'b1),
        .CASDOUTA(NLW_ram_reg_CASDOUTA_UNCONNECTED[31:0]),
        .CASDOUTB(NLW_ram_reg_CASDOUTB_UNCONNECTED[31:0]),
        .CASDOUTPA(NLW_ram_reg_CASDOUTPA_UNCONNECTED[3:0]),
        .CASDOUTPB(NLW_ram_reg_CASDOUTPB_UNCONNECTED[3:0]),
        .CASINDBITERR(1'b0),
        .CASINSBITERR(1'b0),
        .CASOREGIMUXA(1'b0),
        .CASOREGIMUXB(1'b0),
        .CASOREGIMUXEN_A(1'b1),
        .CASOREGIMUXEN_B(1'b1),
        .CASOUTDBITERR(NLW_ram_reg_CASOUTDBITERR_UNCONNECTED),
        .CASOUTSBITERR(NLW_ram_reg_CASOUTSBITERR_UNCONNECTED),
        .CLKARDCLK(aclk),
        .CLKBWRCLK(CLK),
        .DBITERR(NLW_ram_reg_DBITERR_UNCONNECTED),
        .DINADIN(ram_reg_0[31:0]),
        .DINBDIN(ram_reg_0[63:32]),
        .DINPADINP({1'b1,1'b1,1'b1,1'b1}),
        .DINPBDINP({1'b1,1'b1,1'b1,1'b1}),
        .DOUTADOUT(axi4m_wdata[31:0]),
        .DOUTBDOUT(axi4m_wdata[63:32]),
        .DOUTPADOUTP(NLW_ram_reg_DOUTPADOUTP_UNCONNECTED[3:0]),
        .DOUTPBDOUTP(NLW_ram_reg_DOUTPBDOUTP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_ram_reg_ECCPARITY_UNCONNECTED[7:0]),
        .ECCPIPECE(1'b1),
        .ENARDEN(writeram_rd),
        .ENBWREN(1'b1),
        .INJECTDBITERR(1'b0),
        .INJECTSBITERR(1'b0),
        .RDADDRECC(NLW_ram_reg_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b1),
        .REGCEB(1'b1),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_ram_reg_SBITERR_UNCONNECTED),
        .SLEEP(1'b0),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({WEBWE,WEBWE,WEBWE,WEBWE,WEBWE,WEBWE,WEBWE,WEBWE}));
  LUT2 #(
    .INIT(4'h7)) 
    ram_reg_i_10
       (.I0(ram_reg_2),
        .I1(axi4m_wready),
        .O(ram_reg_i_10_n_0));
  LUT6 #(
    .INIT(64'h6AAAAAAAAAAAAAAA)) 
    ram_reg_i_1__0
       (.I0(ram_reg_1[7]),
        .I1(ram_reg_1[5]),
        .I2(ram_reg_1[3]),
        .I3(ram_reg_i_9_n_0),
        .I4(ram_reg_1[4]),
        .I5(ram_reg_1[6]),
        .O(D[7]));
  (* SOFT_HLUTNM = "soft_lutpair54" *) 
  LUT5 #(
    .INIT(32'h6AAAAAAA)) 
    ram_reg_i_2
       (.I0(ram_reg_1[6]),
        .I1(ram_reg_1[4]),
        .I2(ram_reg_i_9_n_0),
        .I3(ram_reg_1[3]),
        .I4(ram_reg_1[5]),
        .O(D[6]));
  (* SOFT_HLUTNM = "soft_lutpair54" *) 
  LUT4 #(
    .INIT(16'h6AAA)) 
    ram_reg_i_3
       (.I0(ram_reg_1[5]),
        .I1(ram_reg_1[3]),
        .I2(ram_reg_i_9_n_0),
        .I3(ram_reg_1[4]),
        .O(D[5]));
  LUT6 #(
    .INIT(64'hAA6AAAAAAAAAAAAA)) 
    ram_reg_i_4
       (.I0(ram_reg_1[4]),
        .I1(ram_reg_1[2]),
        .I2(ram_reg_1[1]),
        .I3(ram_reg_i_10_n_0),
        .I4(ram_reg_1[0]),
        .I5(ram_reg_1[3]),
        .O(D[4]));
  LUT6 #(
    .INIT(64'h6AAAAAAAAAAAAAAA)) 
    ram_reg_i_5
       (.I0(ram_reg_1[3]),
        .I1(ram_reg_1[0]),
        .I2(ram_reg_2),
        .I3(axi4m_wready),
        .I4(ram_reg_1[1]),
        .I5(ram_reg_1[2]),
        .O(D[3]));
  (* SOFT_HLUTNM = "soft_lutpair53" *) 
  LUT5 #(
    .INIT(32'h7FFF8000)) 
    ram_reg_i_6
       (.I0(ram_reg_1[0]),
        .I1(ram_reg_2),
        .I2(axi4m_wready),
        .I3(ram_reg_1[1]),
        .I4(ram_reg_1[2]),
        .O(D[2]));
  (* SOFT_HLUTNM = "soft_lutpair55" *) 
  LUT4 #(
    .INIT(16'h6AAA)) 
    ram_reg_i_7
       (.I0(ram_reg_1[1]),
        .I1(axi4m_wready),
        .I2(ram_reg_2),
        .I3(ram_reg_1[0]),
        .O(D[1]));
  (* SOFT_HLUTNM = "soft_lutpair55" *) 
  LUT3 #(
    .INIT(8'h6A)) 
    ram_reg_i_8
       (.I0(ram_reg_1[0]),
        .I1(axi4m_wready),
        .I2(ram_reg_2),
        .O(D[0]));
  (* SOFT_HLUTNM = "soft_lutpair53" *) 
  LUT5 #(
    .INIT(32'h80000000)) 
    ram_reg_i_9
       (.I0(ram_reg_1[2]),
        .I1(ram_reg_1[1]),
        .I2(axi4m_wready),
        .I3(ram_reg_2),
        .I4(ram_reg_1[0]),
        .O(ram_reg_i_9_n_0));
endmodule
`ifndef GLBL
`define GLBL
`timescale  1 ps / 1 ps

module glbl ();

    parameter ROC_WIDTH = 100000;
    parameter TOC_WIDTH = 0;
    parameter GRES_WIDTH = 10000;
    parameter GRES_START = 10000;

//--------   STARTUP Globals --------------
    wire GSR;
    wire GTS;
    wire GWE;
    wire PRLD;
    wire GRESTORE;
    tri1 p_up_tmp;
    tri (weak1, strong0) PLL_LOCKG = p_up_tmp;

    wire PROGB_GLBL;
    wire CCLKO_GLBL;
    wire FCSBO_GLBL;
    wire [3:0] DO_GLBL;
    wire [3:0] DI_GLBL;
   
    reg GSR_int;
    reg GTS_int;
    reg PRLD_int;
    reg GRESTORE_int;

//--------   JTAG Globals --------------
    wire JTAG_TDO_GLBL;
    wire JTAG_TCK_GLBL;
    wire JTAG_TDI_GLBL;
    wire JTAG_TMS_GLBL;
    wire JTAG_TRST_GLBL;

    reg JTAG_CAPTURE_GLBL;
    reg JTAG_RESET_GLBL;
    reg JTAG_SHIFT_GLBL;
    reg JTAG_UPDATE_GLBL;
    reg JTAG_RUNTEST_GLBL;

    reg JTAG_SEL1_GLBL = 0;
    reg JTAG_SEL2_GLBL = 0 ;
    reg JTAG_SEL3_GLBL = 0;
    reg JTAG_SEL4_GLBL = 0;

    reg JTAG_USER_TDO1_GLBL = 1'bz;
    reg JTAG_USER_TDO2_GLBL = 1'bz;
    reg JTAG_USER_TDO3_GLBL = 1'bz;
    reg JTAG_USER_TDO4_GLBL = 1'bz;

    assign (strong1, weak0) GSR = GSR_int;
    assign (strong1, weak0) GTS = GTS_int;
    assign (weak1, weak0) PRLD = PRLD_int;
    assign (strong1, weak0) GRESTORE = GRESTORE_int;

    initial begin
	GSR_int = 1'b1;
	PRLD_int = 1'b1;
	#(ROC_WIDTH)
	GSR_int = 1'b0;
	PRLD_int = 1'b0;
    end

    initial begin
	GTS_int = 1'b1;
	#(TOC_WIDTH)
	GTS_int = 1'b0;
    end

    initial begin 
	GRESTORE_int = 1'b0;
	#(GRES_START);
	GRESTORE_int = 1'b1;
	#(GRES_WIDTH);
	GRESTORE_int = 1'b0;
    end

endmodule
`endif
