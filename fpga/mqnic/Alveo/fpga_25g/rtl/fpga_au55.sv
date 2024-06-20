`default_nettype none
// SPDX-License-Identifier: BSD-2-Clause-Views
/*
 * Copyright (c) 2019-2023 The Regents of the University of California
 */

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * FPGA top-level module
 */
module fpga #
(
    // PCIe interface configuration
    parameter AXIS_PCIE_DATA_WIDTH = 512
)
(
    /*
     * Clock and reset
     */
    // input  wire         clk_100mhz_0_p,
    // input  wire         clk_100mhz_0_n,
    input  wire         clk_100mhz_1_p,
    input  wire         clk_100mhz_1_n,

    /*
     * GPIO
     */
    output wire [1:0]   qsfp_led_act,
    output wire [1:0]   qsfp_led_stat_g,
    output wire [1:0]   qsfp_led_stat_y,
    output wire         hbm_cattrip,
    input  wire [3:0]   msp_gpio,
    output wire         msp_uart_txd,
    input  wire         msp_uart_rxd,

    /*
     * PCI express
     */
    input  wire [15:0]  pcie_rx_p,
    input  wire [15:0]  pcie_rx_n,
    output wire [15:0]  pcie_tx_p,
    output wire [15:0]  pcie_tx_n,
    input  wire         pcie_refclk_1_p,
    input  wire         pcie_refclk_1_n,
    input  wire         pcie_reset_n

    /*
     * Ethernet: QSFP28
     */
    // output wire [3:0]   qsfp0_tx_p,
    // output wire [3:0]   qsfp0_tx_n,
    // input  wire [3:0]   qsfp0_rx_p,
    // input  wire [3:0]   qsfp0_rx_n,
    // input  wire         qsfp0_mgt_refclk_p,
    // input  wire         qsfp0_mgt_refclk_n,

    // output wire [3:0]   qsfp1_tx_p,
    // output wire [3:0]   qsfp1_tx_n,
    // input  wire [3:0]   qsfp1_rx_p,
    // input  wire [3:0]   qsfp1_rx_n,
    // input  wire         qsfp1_mgt_refclk_p,
    // input  wire         qsfp1_mgt_refclk_n
);

// PTP configuration
// parameter PTP_CLK_PERIOD_NS_NUM = 1024;
// parameter PTP_CLK_PERIOD_NS_DENOM = 165;
// parameter PTP_TS_WIDTH = 96;
// parameter IF_PTP_PERIOD_NS = 6'h6;
// parameter IF_PTP_PERIOD_FNS = 16'h6666;

// Interface configuration
// parameter TX_TAG_WIDTH = 16;

// PCIe interface configuration
parameter AXIS_PCIE_KEEP_WIDTH = (AXIS_PCIE_DATA_WIDTH/32);
parameter AXIS_PCIE_RC_USER_WIDTH = AXIS_PCIE_DATA_WIDTH < 512 ? 75 : 161;
parameter AXIS_PCIE_RQ_USER_WIDTH = AXIS_PCIE_DATA_WIDTH < 512 ? 62 : 137;
parameter AXIS_PCIE_CQ_USER_WIDTH = AXIS_PCIE_DATA_WIDTH < 512 ? 85 : 183;
parameter AXIS_PCIE_CC_USER_WIDTH = AXIS_PCIE_DATA_WIDTH < 512 ? 33 : 81;
parameter RQ_SEQ_NUM_WIDTH = 6;


// Clock and reset
wire pcie_user_clk;
wire pcie_user_reset;

wire clk_100mhz_1_ibufg;
wire clk_100mhz_1_int;

wire clk_50mhz_mmcm_out;
wire clk_125mhz_mmcm_out;

wire mmcm_rst = pcie_user_reset;
wire mmcm_locked;
wire mmcm_clkfb;

IBUFGDS #(
   .DIFF_TERM("FALSE"),
   .IBUF_LOW_PWR("FALSE")
)
clk_100mhz_1_ibufg_inst (
   .O   (clk_100mhz_1_ibufg),
   .I   (clk_100mhz_1_p),
   .IB  (clk_100mhz_1_n)
);

BUFG
clk_100mhz_1_bufg_inst (
    .I(clk_100mhz_1_ibufg),
    .O(clk_100mhz_1_int)
);

// MMCM instance
// 100 MHz in, 125 MHz + 50 MHz out
// PFD range: 10 MHz to 500 MHz
// VCO range: 800 MHz to 1600 MHz
// M = 10, D = 1 sets Fvco = 1000 MHz
// Divide by 8 to get output frequency of 125 MHz
// Divide by 20 to get output frequency of 50 MHz
MMCME4_BASE #(
    .BANDWIDTH("OPTIMIZED"),
    .CLKOUT0_DIVIDE_F(8),
    .CLKOUT0_DUTY_CYCLE(0.5),
    .CLKOUT0_PHASE(0),
    .CLKOUT1_DIVIDE(20),
    .CLKOUT1_DUTY_CYCLE(0.5),
    .CLKOUT1_PHASE(0),
    .CLKOUT2_DIVIDE(1),
    .CLKOUT2_DUTY_CYCLE(0.5),
    .CLKOUT2_PHASE(0),
    .CLKOUT3_DIVIDE(1),
    .CLKOUT3_DUTY_CYCLE(0.5),
    .CLKOUT3_PHASE(0),
    .CLKOUT4_DIVIDE(1),
    .CLKOUT4_DUTY_CYCLE(0.5),
    .CLKOUT4_PHASE(0),
    .CLKOUT5_DIVIDE(1),
    .CLKOUT5_DUTY_CYCLE(0.5),
    .CLKOUT5_PHASE(0),
    .CLKOUT6_DIVIDE(1),
    .CLKOUT6_DUTY_CYCLE(0.5),
    .CLKOUT6_PHASE(0),
    .CLKFBOUT_MULT_F(10),
    .CLKFBOUT_PHASE(0),
    .DIVCLK_DIVIDE(1),
    .REF_JITTER1(0.010),
    .CLKIN1_PERIOD(10.000),
    .STARTUP_WAIT("FALSE"),
    .CLKOUT4_CASCADE("FALSE")
)
clk_mmcm_inst (
    .CLKIN1(clk_100mhz_1_int),
    .CLKFBIN(mmcm_clkfb),
    .RST(mmcm_rst),
    .PWRDWN(1'b0),
    .CLKOUT0(clk_125mhz_mmcm_out),
    .CLKOUT0B(),
    .CLKOUT1(clk_50mhz_mmcm_out),
    .CLKOUT1B(),
    .CLKOUT2(),
    .CLKOUT2B(),
    .CLKOUT3(),
    .CLKOUT3B(),
    .CLKOUT4(),
    .CLKOUT5(),
    .CLKOUT6(),
    .CLKFBOUT(mmcm_clkfb),
    .CLKFBOUTB(),
    .LOCKED(mmcm_locked)
);

// PCIe
wire pcie_sys_clk;
wire pcie_sys_clk_gt;

IBUFDS_GTE4 #(
    .REFCLK_HROW_CK_SEL(2'b00)
)
ibufds_gte4_pcie_mgt_refclk_inst (
    .I             (pcie_refclk_1_p),
    .IB            (pcie_refclk_1_n),
    .CEB           (1'b0),
    .O             (pcie_sys_clk_gt),
    .ODIV2         (pcie_sys_clk)
);

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis_rq_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis_rq_tkeep;
wire                               axis_rq_tlast;
wire [3:0]                              axis_rq_tready;
wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] axis_rq_tuser;
wire                               axis_rq_tvalid;

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis_rc_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis_rc_tkeep;
wire                               axis_rc_tlast;
wire                               axis_rc_tready;
wire [AXIS_PCIE_RC_USER_WIDTH-1:0] axis_rc_tuser;
wire                               axis_rc_tvalid;

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis_cq_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis_cq_tkeep;
wire                               axis_cq_tlast;
wire                               axis_cq_tready;
wire [AXIS_PCIE_CQ_USER_WIDTH-1:0] axis_cq_tuser;
wire                               axis_cq_tvalid;

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis_cc_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis_cc_tkeep;
wire                               axis_cc_tlast;
wire [3:0]                              axis_cc_tready;
wire [AXIS_PCIE_CC_USER_WIDTH-1:0] axis_cc_tuser;
wire                               axis_cc_tvalid;

wire [RQ_SEQ_NUM_WIDTH-1:0]        pcie_rq_seq_num0;
wire                               pcie_rq_seq_num_vld0;
wire [RQ_SEQ_NUM_WIDTH-1:0]        pcie_rq_seq_num1;
wire                               pcie_rq_seq_num_vld1;

wire [3:0] pcie_tfc_nph_av;
wire [3:0] pcie_tfc_npd_av;

wire [2:0] cfg_max_payload;
wire [2:0] cfg_max_read_req;
wire [3:0] cfg_rcb_status;

wire [9:0]  cfg_mgmt_addr;
wire [7:0]  cfg_mgmt_function_number;
wire        cfg_mgmt_write;
wire [31:0] cfg_mgmt_write_data;
wire [3:0]  cfg_mgmt_byte_enable;
wire        cfg_mgmt_read;
wire [31:0] cfg_mgmt_read_data;
wire        cfg_mgmt_read_write_done;

wire [7:0]  cfg_fc_ph;
wire [11:0] cfg_fc_pd;
wire [7:0]  cfg_fc_nph;
wire [11:0] cfg_fc_npd;
wire [7:0]  cfg_fc_cplh;
wire [11:0] cfg_fc_cpld;
wire [2:0]  cfg_fc_sel;

wire [3:0]   cfg_interrupt_msix_enable;
wire [3:0]   cfg_interrupt_msix_mask;
wire [251:0] cfg_interrupt_msix_vf_enable;
wire [251:0] cfg_interrupt_msix_vf_mask;
wire [63:0]  cfg_interrupt_msix_address;
wire [31:0]  cfg_interrupt_msix_data;
wire         cfg_interrupt_msix_int;
wire [1:0]   cfg_interrupt_msix_vec_pending;
wire         cfg_interrupt_msix_vec_pending_status;
wire         cfg_interrupt_msix_sent;
wire         cfg_interrupt_msix_fail;
wire [7:0]   cfg_interrupt_msi_function_number;

wire status_error_cor;
wire status_error_uncor;

// extra register for pcie_user_reset signal
wire pcie_user_reset_int;
(* shreg_extract = "no" *)
reg pcie_user_reset_reg_1 = 1'b1;
(* shreg_extract = "no" *)
reg pcie_user_reset_reg_2 = 1'b1;

always @(posedge pcie_user_clk) begin
    pcie_user_reset_reg_1 <= pcie_user_reset_int;
    pcie_user_reset_reg_2 <= pcie_user_reset_reg_1;
end

logic user_lnk_up;
logic phy_rdy_out;

BUFG
pcie_user_reset_bufg_inst (
    .I(pcie_user_reset_reg_2),
    .O(pcie_user_reset)
);

pcie4c_uscale_plus_0
pcie4c_uscale_plus_inst (
    .pci_exp_txn(pcie_tx_n),
    .pci_exp_txp(pcie_tx_p),
    .pci_exp_rxn(pcie_rx_n),
    .pci_exp_rxp(pcie_rx_p),
    .user_clk(pcie_user_clk),
    .user_reset(pcie_user_reset_int),
    .user_lnk_up,

    .s_axis_rq_tdata(axis_rq_tdata),
    .s_axis_rq_tkeep(axis_rq_tkeep),
    .s_axis_rq_tlast(axis_rq_tlast),
    .s_axis_rq_tready(axis_rq_tready),
    .s_axis_rq_tuser(axis_rq_tuser),
    .s_axis_rq_tvalid(axis_rq_tvalid),

    .m_axis_rc_tdata(axis_rc_tdata),
    .m_axis_rc_tkeep(axis_rc_tkeep),
    .m_axis_rc_tlast(axis_rc_tlast),
    .m_axis_rc_tready(axis_rc_tready),
    .m_axis_rc_tuser(axis_rc_tuser),
    .m_axis_rc_tvalid(axis_rc_tvalid),

    .m_axis_cq_tdata(axis_cq_tdata),
    .m_axis_cq_tkeep(axis_cq_tkeep),
    .m_axis_cq_tlast(axis_cq_tlast),
    .m_axis_cq_tready(axis_cq_tready),
    .m_axis_cq_tuser(axis_cq_tuser),
    .m_axis_cq_tvalid(axis_cq_tvalid),

    .s_axis_cc_tdata(axis_cc_tdata),
    .s_axis_cc_tkeep(axis_cc_tkeep),
    .s_axis_cc_tlast(axis_cc_tlast),
    .s_axis_cc_tready(axis_cc_tready),
    .s_axis_cc_tuser(axis_cc_tuser),
    .s_axis_cc_tvalid(axis_cc_tvalid),

    .pcie_rq_seq_num0(pcie_rq_seq_num0),
    .pcie_rq_seq_num_vld0(pcie_rq_seq_num_vld0),
    .pcie_rq_seq_num1(pcie_rq_seq_num1),
    .pcie_rq_seq_num_vld1(pcie_rq_seq_num_vld1),
    .pcie_rq_tag0(),
    .pcie_rq_tag1(),
    .pcie_rq_tag_av(),
    .pcie_rq_tag_vld0(),
    .pcie_rq_tag_vld1(),

    .pcie_tfc_nph_av(pcie_tfc_nph_av),
    .pcie_tfc_npd_av(pcie_tfc_npd_av),

    .pcie_cq_np_req(1'b1),
    .pcie_cq_np_req_count(),

    .cfg_phy_link_down(),
    .cfg_phy_link_status(),
    .cfg_negotiated_width(),
    .cfg_current_speed(),
    .cfg_max_payload(cfg_max_payload),
    .cfg_max_read_req(cfg_max_read_req),
    .cfg_function_status(),
    .cfg_function_power_state(),
    .cfg_vf_status(),
    .cfg_vf_power_state(),
    .cfg_link_power_state(),

    .cfg_mgmt_addr(cfg_mgmt_addr),
    .cfg_mgmt_function_number(cfg_mgmt_function_number),
    .cfg_mgmt_write(cfg_mgmt_write),
    .cfg_mgmt_write_data(cfg_mgmt_write_data),
    .cfg_mgmt_byte_enable(cfg_mgmt_byte_enable),
    .cfg_mgmt_read(cfg_mgmt_read),
    .cfg_mgmt_read_data(cfg_mgmt_read_data),
    .cfg_mgmt_read_write_done(cfg_mgmt_read_write_done),
    .cfg_mgmt_debug_access(1'b0),

    .cfg_err_cor_out(),
    .cfg_err_nonfatal_out(),
    .cfg_err_fatal_out(),
    .cfg_local_error_valid(),
    .cfg_local_error_out(),
    .cfg_ltssm_state(),
    .cfg_rx_pm_state(),
    .cfg_tx_pm_state(),
    .cfg_rcb_status(cfg_rcb_status),
    .cfg_obff_enable(),
    .cfg_pl_status_change(),
    .cfg_tph_requester_enable(),
    .cfg_tph_st_mode(),
    .cfg_vf_tph_requester_enable(),
    .cfg_vf_tph_st_mode(),

    .cfg_msg_received(),
    .cfg_msg_received_data(),
    .cfg_msg_received_type(),
    .cfg_msg_transmit(1'b0),
    .cfg_msg_transmit_type(3'd0),
    .cfg_msg_transmit_data(32'd0),
    .cfg_msg_transmit_done(),

    .cfg_fc_ph(cfg_fc_ph),
    .cfg_fc_pd(cfg_fc_pd),
    .cfg_fc_nph(cfg_fc_nph),
    .cfg_fc_npd(cfg_fc_npd),
    .cfg_fc_cplh(cfg_fc_cplh),
    .cfg_fc_cpld(cfg_fc_cpld),
    .cfg_fc_sel(cfg_fc_sel),

    .cfg_dsn(64'd0),

    .cfg_power_state_change_ack(1'b1),
    .cfg_power_state_change_interrupt(),

    .cfg_err_cor_in(status_error_cor),
    .cfg_err_uncor_in(status_error_uncor),
    .cfg_flr_in_process(),
    .cfg_flr_done(4'd0),
    .cfg_vf_flr_in_process(),
    .cfg_vf_flr_func_num(8'd0),
    .cfg_vf_flr_done(8'd0),

    .cfg_link_training_enable(1'b1),

    .cfg_interrupt_int(4'd0),
    .cfg_interrupt_pending(4'd0),
    .cfg_interrupt_sent(),
    .cfg_interrupt_msix_enable(cfg_interrupt_msix_enable),
    .cfg_interrupt_msix_mask(cfg_interrupt_msix_mask),
    .cfg_interrupt_msix_vf_enable(cfg_interrupt_msix_vf_enable),
    .cfg_interrupt_msix_vf_mask(cfg_interrupt_msix_vf_mask),
    .cfg_interrupt_msix_address(cfg_interrupt_msix_address),
    .cfg_interrupt_msix_data(cfg_interrupt_msix_data),
    .cfg_interrupt_msix_int(cfg_interrupt_msix_int),
    .cfg_interrupt_msix_vec_pending(cfg_interrupt_msix_vec_pending),
    .cfg_interrupt_msix_vec_pending_status(cfg_interrupt_msix_vec_pending_status),
    .cfg_interrupt_msi_sent(cfg_interrupt_msix_sent),
    .cfg_interrupt_msi_fail(cfg_interrupt_msix_fail),
    .cfg_interrupt_msi_function_number(cfg_interrupt_msi_function_number),

    .cfg_pm_aspm_l1_entry_reject(1'b0),
    .cfg_pm_aspm_tx_l0s_entry_disable(1'b0),

    .cfg_hot_reset_out(),

    .cfg_config_space_enable(1'b1),
    .cfg_req_pm_transition_l23_ready(1'b0),
    .cfg_hot_reset_in(1'b0),

    .cfg_ds_port_number(8'd0),
    .cfg_ds_bus_number(8'd0),
    .cfg_ds_device_number(5'd0),

    .sys_clk(pcie_sys_clk),
    .sys_clk_gt(pcie_sys_clk_gt),
    .sys_reset(pcie_reset_n),

    .phy_rdy_out()
);

reg [RQ_SEQ_NUM_WIDTH-1:0] pcie_rq_seq_num0_reg;
reg                        pcie_rq_seq_num_vld0_reg;
reg [RQ_SEQ_NUM_WIDTH-1:0] pcie_rq_seq_num1_reg;
reg                        pcie_rq_seq_num_vld1_reg;

always @(posedge pcie_user_clk) begin
    pcie_rq_seq_num0_reg <= pcie_rq_seq_num0;
    pcie_rq_seq_num_vld0_reg <= pcie_rq_seq_num_vld0;
    pcie_rq_seq_num1_reg <= pcie_rq_seq_num1;
    pcie_rq_seq_num_vld1_reg <= pcie_rq_seq_num_vld1;

    if (pcie_user_reset) begin
        pcie_rq_seq_num_vld0_reg <= 1'b0;
        pcie_rq_seq_num_vld1_reg <= 1'b0;
    end
end

typedef struct packed {
    logic [511 : 0] tdata;
    logic [15 : 0]   tkeep;
    logic           tlast;
    logic           tvalid;
  } axis256_t;

  typedef struct packed {
    logic [511 : 0] s_axis_rq_tdata;
    logic [15 : 0]   s_axis_rq_tkeep;
    logic           s_axis_rq_tlast;
    logic [3 : 0]   s_axis_rq_tready;
    logic [61 : 0]  s_axis_rq_tuser;
    logic           s_axis_rq_tvalid;
    logic [511 : 0] s_axis_cc_tdata;
    logic [15 : 0]   s_axis_cc_tkeep;
    logic           s_axis_cc_tlast;
    logic [3 : 0]   s_axis_cc_tready;
    logic [32 : 0]  s_axis_cc_tuser;
    logic           s_axis_cc_tvalid;
  } pcie_from_app_t;

  typedef struct packed {
    logic [511 : 0] m_axis_rc_tdata;
    logic [15 : 0]   m_axis_rc_tkeep;
    logic           m_axis_rc_tlast;
    logic           m_axis_rc_tready;
    logic [74 : 0]  m_axis_rc_tuser;
    logic           m_axis_rc_tvalid;
    logic [511 : 0] m_axis_cq_tdata;
    logic [15 : 0]   m_axis_cq_tkeep;
    logic           m_axis_cq_tlast;
    logic           m_axis_cq_tready;
    logic [87 : 0]  m_axis_cq_tuser;
    logic           m_axis_cq_tvalid;
  } completer_request_to_app_t;

  typedef struct packed {
    logic [2 : 0]   max_payload;
    logic [2 : 0]   max_read_req;
    // logic           local_error_valid;
    // logic [4 : 0]   local_error_out;
    // logic [5 : 0]   ltssm_state;
    // logic [1 : 0]   rx_pm_state;
    // logic [1 : 0]   tx_pm_state;
    logic [3 : 0]   rcb_status;
    // logic [1 : 0]   obff_enable;
    // logic           pl_status_change;
    // logic [3 : 0]   tph_requester_enable;
    // logic [11 : 0]  tph_st_mode;
    // logic [251 : 0] vf_tph_requester_enable;
    // logic [755 : 0] vf_tph_st_mode;
    logic [3 : 0]   pcie_tfc_nph_av;
    logic [3 : 0]   pcie_tfc_npd_av;
    logic [3 : 0]   pcie_rq_tag_av;
  } cfg_status_t;

  typedef struct packed {
    logic [9 : 0]  addr;
    logic [7 : 0]  function_number;
    logic          write;
    logic [31 : 0] write_data;
    logic [3 : 0]  byte_enable;
    logic          read;
    // logic          debug_access;
  } cfg_mng_to_core_t;

  typedef struct packed {
    logic [31 : 0] read_data;
    logic          read_write_done;
  } cfg_mng_to_app_t;

//   typedef struct packed {
//     logic         received;
//     logic [7 : 0] received_data;
//     logic [4 : 0] received_type;
//     logic         transmit_done;
//   } cfg_msg_to_app_t;

  typedef struct packed {
    logic          transmit;
    logic [2 : 0]  transmit_type;
    logic [31 : 0] transmit_data;
  } cfg_msg_to_core_t;

//   typedef struct packed {
//     logic [3 : 0]  msi_enable;
//     logic [11 : 0] msi_mmenable;
//     logic          msi_mask_update;
//     logic [31 : 0] msi_data;
//     logic          msi_sent;
//     logic          msi_fail;
//   } cfg_interrupt_to_app_t;

  typedef struct packed {
    logic [1 : 0]  msi_select;
    logic [31 : 0] msi_int;
    logic [31 : 0] msi_pending_status;
    logic          msi_pending_status_data_enable;
    logic [1 : 0]  msi_pending_status_function_num;
    logic [2 : 0]  msi_attr;
    logic          msi_tph_present;
    logic [1 : 0]  msi_tph_type;
    logic [7 : 0]  msi_tph_st_tag;
    logic [7 : 0]  msi_function_number;
  } cfg_interrupt_to_core_t;

  typedef struct packed {
    logic [7 : 0]  ph;
    logic [11 : 0] pd;
    logic [7 : 0]  nph;
    logic [11 : 0] npd;
    logic [7 : 0]  cplh;
    logic [11 : 0] cpld;
  } cfg_fc_to_app_t;

//   typedef struct packed {
//     logic cor_out;
//     logic nonfatal_out;
//     logic fatal_out;
//   } cfg_err_to_app_t;

  typedef struct packed {

    logic [5 : 0]     pcie_rq_seq_num0;
    logic             pcie_rq_seq_num_vld0;
    logic [5 : 0]     pcie_rq_seq_num1;
    logic             pcie_rq_seq_num_vld1;
    logic [3 : 0]     pcie_tfc_nph_av;
    logic [3 : 0]     pcie_tfc_npd_av;
  } cfg_seq_tag_tfc_t;

  typedef struct packed {
    axis256_t              m_axis_cq;
    logic [182 : 0]         m_axis_cq_tuser;
    logic [3 : 0]          s_axis_cc_tready;
    axis256_t              m_axis_rc;
    logic [160 : 0]         m_axis_rc_tuser;
    logic [3 : 0]          s_axis_rq_tready;
    cfg_mng_to_app_t       cfg_mgmt;
    // cfg_msg_to_app_t       cfg_msg;
    // cfg_interrupt_to_app_t cfg_interrupt;
    cfg_fc_to_app_t        cfg_fc;
    cfg_status_t           cfg_status;
    cfg_seq_tag_tfc_t      cfg_seq_tag_tfc;
    // cfg_err_to_app_t       cfg_err;
    logic                  phy_rdy;
    logic                  user_clk;
    logic                  user_reset;
    logic                  user_lnk_up;
  } pcie_to_app_t;

  typedef struct packed {
    logic                   m_axis_cq_tready;
    axis256_t               s_axis_cc;
    logic [80 : 0]          s_axis_cc_tuser;
    logic                   m_axis_rc_tready;
    axis256_t               s_axis_rq;
    logic [136 : 0]          s_axis_rq_tuser;
    cfg_mng_to_core_t       cfg_mgmt;
    cfg_msg_to_core_t       cfg_msg;
    cfg_interrupt_to_core_t cfg_interrupt;
    logic [2:0]             cfg_fc_sel;
  } pcie_to_core_t;

  pcie_to_app_t   to_app;
  pcie_to_core_t  to_core;
  assign to_core = 0;

assign to_app.user_clk                            = pcie_user_clk;
assign to_app.user_reset                          = pcie_user_reset_int;
assign to_app.user_lnk_up                         = user_lnk_up;

assign axis_rq_tdata                            = to_core.s_axis_rq.tdata;
assign axis_rq_tkeep                            = to_core.s_axis_rq.tkeep;
assign axis_rq_tlast                            = to_core.s_axis_rq.tlast;
assign to_app.s_axis_rq_tready                    = axis_rq_tready;
assign axis_rq_tuser                            = to_core.s_axis_rq_tuser;
assign axis_rq_tvalid                           = to_core.s_axis_rq.tvalid;

assign to_app.m_axis_rc.tdata                     = axis_rc_tdata;
assign to_app.m_axis_rc.tkeep                     = axis_rc_tkeep;
assign to_app.m_axis_rc.tlast                     = axis_rc_tlast;
assign axis_rc_tready                             = to_core.m_axis_rc_tready;
assign to_app.m_axis_rc_tuser                     = axis_rc_tuser;
assign to_app.m_axis_rc.tvalid                    = axis_rc_tvalid;

assign to_app.m_axis_cq.tdata                     = axis_cq_tdata;
assign to_app.m_axis_cq.tkeep                     = axis_cq_tkeep;
assign to_app.m_axis_cq.tlast                     = axis_cq_tlast;
assign axis_cq_tready                           = to_core.m_axis_cq_tready;
assign to_app.m_axis_cq_tuser                     = axis_cq_tuser;
assign to_app.m_axis_cq.tvalid                    = axis_cq_tvalid;

assign axis_cc_tdata                            = to_core.s_axis_cc.tdata;
assign axis_cc_tkeep                            = to_core.s_axis_cc.tkeep;
assign axis_cc_tlast                            = to_core.s_axis_cc.tlast;
assign to_app.s_axis_cc_tready                    = axis_cc_tready;
assign axis_cc_tuser                            = to_core.s_axis_cc_tuser;
assign axis_cc_tvalid                           = to_core.s_axis_cc.tvalid;

assign to_app.cfg_seq_tag_tfc.pcie_rq_seq_num0     = pcie_rq_seq_num0;
assign to_app.cfg_seq_tag_tfc.pcie_rq_seq_num_vld0 = pcie_rq_seq_num_vld0;
assign to_app.cfg_seq_tag_tfc.pcie_rq_seq_num1     = pcie_rq_seq_num1;
assign to_app.cfg_seq_tag_tfc.pcie_rq_seq_num_vld1 = pcie_rq_seq_num_vld1;
assign to_app.cfg_seq_tag_tfc.pcie_tfc_nph_av      = pcie_tfc_nph_av;
assign to_app.cfg_seq_tag_tfc.pcie_tfc_npd_av      = pcie_tfc_npd_av;

assign to_app.cfg_status.max_payload              = cfg_max_payload;
assign to_app.cfg_status.max_read_req             = cfg_max_read_req;

assign cfg_mgmt_addr                              = to_core.cfg_mgmt.addr;
assign cfg_mgmt_function_number                   = to_core.cfg_mgmt.function_number;
assign cfg_mgmt_write                             = to_core.cfg_mgmt.write;
assign cfg_mgmt_write_data                        = to_core.cfg_mgmt.write_data;
assign cfg_mgmt_byte_enable                       = to_core.cfg_mgmt.byte_enable;
assign cfg_mgmt_read                              = to_core.cfg_mgmt.read;
assign to_app.cfg_mgmt.read_data                  = cfg_mgmt_read_data;
assign to_app.cfg_mgmt.read_write_done            = cfg_mgmt_read_write_done;

assign to_app.cfg_status.rcb_status               = cfg_rcb_status;

// assign cfg_msg_transmit                           = to_core.cfg_msg.transmit;
// assign cfg_msg_transmit_type                      = to_core.cfg_msg.transmit_type;
// assign cfg_msg_transmit_data                      = to_core.cfg_msg.transmit_data;

assign to_app.cfg_fc.ph                           = cfg_fc_ph;
assign to_app.cfg_fc.pd                           = cfg_fc_pd;
assign to_app.cfg_fc.nph                          = cfg_fc_nph;
assign to_app.cfg_fc.npd                          = cfg_fc_npd;
assign to_app.cfg_fc.cplh                         = cfg_fc_cplh;
assign to_app.cfg_fc.cpld                         = cfg_fc_cpld;
assign cfg_fc_sel                                 = to_core.cfg_fc_sel;

assign cfg_interrupt_msi_function_number              = to_core.cfg_interrupt.msi_function_number;
assign to_app.phy_rdy                                 = phy_rdy_out;


endmodule

`resetall
`default_nettype wire
