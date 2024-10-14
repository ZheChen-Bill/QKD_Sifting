// (c) Copyright 1995-2023 Xilinx, Inc. All rights reserved.
// 
// This file contains confidential and proprietary information
// of Xilinx, Inc. and is protected under U.S. and
// international copyright and other intellectual property
// laws.
// 
// DISCLAIMER
// This disclaimer is not a license and does not grant any
// rights to the materials distributed herewith. Except as
// otherwise provided in a valid license issued to you by
// Xilinx, and to the maximum extent permitted by applicable
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
// (2) Xilinx shall not be liable (whether in contract or tort,
// including negligence, or under any other theory of
// liability) for any loss or damage of any kind or nature
// related to, arising under or in connection with these
// materials, including for any direct, or any indirect,
// special, incidental, or consequential loss or damage
// (including loss of data, profits, goodwill, or any type of
// loss or damage suffered as a result of any action brought
// by a third party) even if such damage or loss was
// reasonably foreseeable or Xilinx had been advised of the
// possibility of the same.
// 
// CRITICAL APPLICATIONS
// Xilinx products are not designed or intended to be fail-
// safe, or for use in any application requiring fail-safe
// performance, such as life-support or safety devices or
// systems, Class III medical devices, nuclear facilities,
// applications related to the deployment of airbags, or any
// other applications that could lead to death, personal
// injury, or severe property or environmental damage
// (individually and collectively, "Critical
// Applications"). Customer assumes the sole risk and
// liability of any use of Xilinx products in Critical
// Applications, subject only to applicable laws and
// regulations governing limitations on product liability.
// 
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
// PART OF THIS FILE AT ALL TIMES.
// 
// DO NOT MODIFY THIS FILE.


// IP VLNV: mathworks.com:ip:hdlverifier_axi_manager:2.0
// IP Revision: 19

(* X_CORE_INFO = "hdlverifier_axi_manager,Vivado 2022.1" *)
(* CHECK_LICENSE_TYPE = "BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0,hdlverifier_axi_manager,{}" *)
(* IP_DEFINITION_SOURCE = "package_project" *)
(* DowngradeIPIdentifiedWarnings = "yes" *)
module BD_KCU116_AXImanager_hdlverifier_axi_mana_0_0 (
  axi4m_awaddr,
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
  aresetn
);

(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWADDR" *)
output wire [31 : 0] axi4m_awaddr;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWPROT" *)
output wire [2 : 0] axi4m_awprot;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWVALID" *)
output wire axi4m_awvalid;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWREADY" *)
input wire axi4m_awready;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m WDATA" *)
output wire [63 : 0] axi4m_wdata;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m WVALID" *)
output wire axi4m_wvalid;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m WREADY" *)
input wire axi4m_wready;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m WSTRB" *)
output wire [7 : 0] axi4m_wstrb;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWID" *)
output wire [0 : 0] axi4m_awid;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWLEN" *)
output wire [7 : 0] axi4m_awlen;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWSIZE" *)
output wire [2 : 0] axi4m_awsize;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWBURST" *)
output wire [1 : 0] axi4m_awburst;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWLOCK" *)
output wire axi4m_awlock;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWCACHE" *)
output wire [3 : 0] axi4m_awcache;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m WLAST" *)
output wire axi4m_wlast;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m AWQOS" *)
output wire [3 : 0] axi4m_awqos;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m BRESP" *)
input wire [1 : 0] axi4m_bresp;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m BVALID" *)
input wire axi4m_bvalid;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m BID" *)
input wire [0 : 0] axi4m_bid;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m BREADY" *)
output wire axi4m_bready;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARADDR" *)
output wire [31 : 0] axi4m_araddr;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARPROT" *)
output wire [2 : 0] axi4m_arprot;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARVALID" *)
output wire axi4m_arvalid;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARREADY" *)
input wire axi4m_arready;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARID" *)
output wire [0 : 0] axi4m_arid;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARLEN" *)
output wire [7 : 0] axi4m_arlen;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARSIZE" *)
output wire [2 : 0] axi4m_arsize;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARBURST" *)
output wire [1 : 0] axi4m_arburst;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARLOCK" *)
output wire axi4m_arlock;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARCACHE" *)
output wire [3 : 0] axi4m_arcache;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m ARQOS" *)
output wire [3 : 0] axi4m_arqos;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m RDATA" *)
input wire [63 : 0] axi4m_rdata;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m RLAST" *)
input wire axi4m_rlast;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m RRESP" *)
input wire [1 : 0] axi4m_rresp;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m RVALID" *)
input wire axi4m_rvalid;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m RREADY" *)
output wire axi4m_rready;
(* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME axi4m, DATA_WIDTH 64, PROTOCOL AXI4, FREQ_HZ 200000000, ID_WIDTH 1, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE READ_WRITE, HAS_BURST 1, HAS_LOCK 1, HAS_PROT 1, HAS_CACHE 1, HAS_QOS 1, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, SUPPORTS_NARROW_BURST 1, NUM_READ_OUTSTANDING 2, NUM_WRITE_OUTSTANDING 2, MAX_BURST_LENGTH 256, PHASE 0.0, CLK_DOMAIN BD_KCU116_AXImanager_clk_wiz_0_0_clk_out1, NUM_READ_THREADS 1,\
 NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0, INSERT_VIP 0" *)
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 axi4m RID" *)
input wire [0 : 0] axi4m_rid;
(* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF axi4m, ASSOCIATED_RESET aresetn, FREQ_HZ 200000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN BD_KCU116_AXImanager_clk_wiz_0_0_clk_out1, INSERT_VIP 0" *)
(* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
input wire aclk;
(* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW, INSERT_VIP 0" *)
(* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
input wire aresetn;

  hdlverifier_axi_manager #(
    .ID_WIDTH(1),
    .AXI_DATA_WIDTH(64),
    .AXI_ADDR_WIDTH(32)
  ) inst (
    .axi4m_awaddr(axi4m_awaddr),
    .axi4m_awprot(axi4m_awprot),
    .axi4m_awvalid(axi4m_awvalid),
    .axi4m_awready(axi4m_awready),
    .axi4m_wdata(axi4m_wdata),
    .axi4m_wvalid(axi4m_wvalid),
    .axi4m_wready(axi4m_wready),
    .axi4m_wstrb(axi4m_wstrb),
    .axi4m_awid(axi4m_awid),
    .axi4m_awlen(axi4m_awlen),
    .axi4m_awsize(axi4m_awsize),
    .axi4m_awburst(axi4m_awburst),
    .axi4m_awlock(axi4m_awlock),
    .axi4m_awcache(axi4m_awcache),
    .axi4m_wlast(axi4m_wlast),
    .axi4m_awqos(axi4m_awqos),
    .axi4m_bresp(axi4m_bresp),
    .axi4m_bvalid(axi4m_bvalid),
    .axi4m_bid(axi4m_bid),
    .axi4m_bready(axi4m_bready),
    .axi4m_araddr(axi4m_araddr),
    .axi4m_arprot(axi4m_arprot),
    .axi4m_arvalid(axi4m_arvalid),
    .axi4m_arready(axi4m_arready),
    .axi4m_arid(axi4m_arid),
    .axi4m_arlen(axi4m_arlen),
    .axi4m_arsize(axi4m_arsize),
    .axi4m_arburst(axi4m_arburst),
    .axi4m_arlock(axi4m_arlock),
    .axi4m_arcache(axi4m_arcache),
    .axi4m_arqos(axi4m_arqos),
    .axi4m_rdata(axi4m_rdata),
    .axi4m_rlast(axi4m_rlast),
    .axi4m_rresp(axi4m_rresp),
    .axi4m_rvalid(axi4m_rvalid),
    .axi4m_rready(axi4m_rready),
    .axi4m_rid(axi4m_rid),
    .aclk(aclk),
    .aresetn(aresetn)
  );
endmodule
