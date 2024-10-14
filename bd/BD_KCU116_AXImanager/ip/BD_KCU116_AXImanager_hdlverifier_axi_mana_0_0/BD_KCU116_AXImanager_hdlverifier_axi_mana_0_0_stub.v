// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2022.1 (win64) Build 3526262 Mon Apr 18 15:48:16 MDT 2022
// Date        : Wed Dec 20 07:23:21 2023
// Host        : DESKTOP-H9LB9O9 running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               d:/QKD_HW_project/matlab_test/kcu116_sifting_AXImanager_v1/kcu116_sifting_AXImanager_v1.gen/sources_1/bd/BD_KCU116_AXImanager/ip/BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0/BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0_stub.v
// Design      : BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xcku5p-ffvb676-2-e
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "hdlverifier_axi_manager,Vivado 2022.1" *)
module BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0(axi4m_awaddr, axi4m_awprot, axi4m_awvalid, 
  axi4m_awready, axi4m_wdata, axi4m_wvalid, axi4m_wready, axi4m_wstrb, axi4m_awid, axi4m_awlen, 
  axi4m_awsize, axi4m_awburst, axi4m_awlock, axi4m_awcache, axi4m_wlast, axi4m_awqos, 
  axi4m_bresp, axi4m_bvalid, axi4m_bid, axi4m_bready, axi4m_araddr, axi4m_arprot, 
  axi4m_arvalid, axi4m_arready, axi4m_arid, axi4m_arlen, axi4m_arsize, axi4m_arburst, 
  axi4m_arlock, axi4m_arcache, axi4m_arqos, axi4m_rdata, axi4m_rlast, axi4m_rresp, 
  axi4m_rvalid, axi4m_rready, axi4m_rid, aclk, aresetn)
/* synthesis syn_black_box black_box_pad_pin="axi4m_awaddr[31:0],axi4m_awprot[2:0],axi4m_awvalid,axi4m_awready,axi4m_wdata[63:0],axi4m_wvalid,axi4m_wready,axi4m_wstrb[7:0],axi4m_awid[0:0],axi4m_awlen[7:0],axi4m_awsize[2:0],axi4m_awburst[1:0],axi4m_awlock,axi4m_awcache[3:0],axi4m_wlast,axi4m_awqos[3:0],axi4m_bresp[1:0],axi4m_bvalid,axi4m_bid[0:0],axi4m_bready,axi4m_araddr[31:0],axi4m_arprot[2:0],axi4m_arvalid,axi4m_arready,axi4m_arid[0:0],axi4m_arlen[7:0],axi4m_arsize[2:0],axi4m_arburst[1:0],axi4m_arlock,axi4m_arcache[3:0],axi4m_arqos[3:0],axi4m_rdata[63:0],axi4m_rlast,axi4m_rresp[1:0],axi4m_rvalid,axi4m_rready,axi4m_rid[0:0],aclk,aresetn" */;
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
endmodule
