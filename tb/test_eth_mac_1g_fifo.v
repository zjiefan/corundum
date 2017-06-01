/*

Copyright (c) 2015-2017 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

// Language: Verilog 2001

`timescale 1ns / 1ps

/*
 * Testbench for eth_mac_1g_fifo
 */
module test_eth_mac_1g_fifo;

// Parameters
parameter ENABLE_PADDING = 1;
parameter MIN_FRAME_LENGTH = 64;
parameter TX_FIFO_ADDR_WIDTH = 9;
parameter RX_FIFO_ADDR_WIDTH = 9;

// Inputs
reg clk = 0;
reg rst = 0;
reg [7:0] current_test = 0;

reg rx_clk = 0;
reg rx_rst = 0;
reg tx_clk = 0;
reg tx_rst = 0;
reg logic_clk = 0;
reg logic_rst = 0;
reg [7:0] tx_axis_tdata = 0;
reg tx_axis_tvalid = 0;
reg tx_axis_tlast = 0;
reg tx_axis_tuser = 0;
reg rx_axis_tready = 0;
reg [7:0] gmii_rxd = 0;
reg gmii_rx_dv = 0;
reg gmii_rx_er = 0;
reg rx_clk_enable = 1;
reg tx_clk_enable = 1;
reg rx_mii_select = 0;
reg tx_mii_select = 0;
reg [7:0] ifg_delay = 0;

// Outputs
wire tx_axis_tready;
wire [7:0] rx_axis_tdata;
wire rx_axis_tvalid;
wire rx_axis_tlast;
wire rx_axis_tuser;
wire [7:0] gmii_txd;
wire gmii_tx_en;
wire gmii_tx_er;
wire tx_fifo_overflow;
wire tx_fifo_bad_frame;
wire tx_fifo_good_frame;
wire rx_error_bad_frame;
wire rx_error_bad_fcs;
wire rx_fifo_overflow;
wire rx_fifo_bad_frame;
wire rx_fifo_good_frame;

initial begin
    // myhdl integration
    $from_myhdl(
        clk,
        rst,
        current_test,
        rx_clk,
        rx_rst,
        tx_clk,
        tx_rst,
        logic_clk,
        logic_rst,
        tx_axis_tdata,
        tx_axis_tvalid,
        tx_axis_tlast,
        tx_axis_tuser,
        rx_axis_tready,
        gmii_rxd,
        gmii_rx_dv,
        gmii_rx_er,
        rx_clk_enable,
        tx_clk_enable,
        rx_mii_select,
        tx_mii_select,
        ifg_delay
    );
    $to_myhdl(
        tx_axis_tready,
        rx_axis_tdata,
        rx_axis_tvalid,
        rx_axis_tlast,
        rx_axis_tuser,
        gmii_txd,
        gmii_tx_en,
        gmii_tx_er,
        tx_fifo_overflow,
        tx_fifo_bad_frame,
        tx_fifo_good_frame,
        rx_error_bad_frame,
        rx_error_bad_fcs,
        rx_fifo_overflow,
        rx_fifo_bad_frame,
        rx_fifo_good_frame
    );

    // dump file
    $dumpfile("test_eth_mac_1g_fifo.lxt");
    $dumpvars(0, test_eth_mac_1g_fifo);
end

eth_mac_1g_fifo #(
    .ENABLE_PADDING(ENABLE_PADDING),
    .MIN_FRAME_LENGTH(MIN_FRAME_LENGTH),
    .TX_FIFO_ADDR_WIDTH(TX_FIFO_ADDR_WIDTH),
    .RX_FIFO_ADDR_WIDTH(RX_FIFO_ADDR_WIDTH)
)
UUT (
    .rx_clk(rx_clk),
    .rx_rst(rx_rst),
    .tx_clk(tx_clk),
    .tx_rst(tx_rst),
    .logic_clk(logic_clk),
    .logic_rst(logic_rst),
    .tx_axis_tdata(tx_axis_tdata),
    .tx_axis_tvalid(tx_axis_tvalid),
    .tx_axis_tready(tx_axis_tready),
    .tx_axis_tlast(tx_axis_tlast),
    .tx_axis_tuser(tx_axis_tuser),
    .rx_axis_tdata(rx_axis_tdata),
    .rx_axis_tvalid(rx_axis_tvalid),
    .rx_axis_tready(rx_axis_tready),
    .rx_axis_tlast(rx_axis_tlast),
    .rx_axis_tuser(rx_axis_tuser),
    .gmii_rxd(gmii_rxd),
    .gmii_rx_dv(gmii_rx_dv),
    .gmii_rx_er(gmii_rx_er),
    .gmii_txd(gmii_txd),
    .gmii_tx_en(gmii_tx_en),
    .gmii_tx_er(gmii_tx_er),
    .rx_clk_enable(rx_clk_enable),
    .tx_clk_enable(tx_clk_enable),
    .rx_mii_select(rx_mii_select),
    .tx_mii_select(tx_mii_select),
    .tx_fifo_overflow(tx_fifo_overflow),
    .tx_fifo_bad_frame(tx_fifo_bad_frame),
    .tx_fifo_good_frame(tx_fifo_good_frame),
    .rx_error_bad_frame(rx_error_bad_frame),
    .rx_error_bad_fcs(rx_error_bad_fcs),
    .rx_fifo_overflow(rx_fifo_overflow),
    .rx_fifo_bad_frame(rx_fifo_bad_frame),
    .rx_fifo_good_frame(rx_fifo_good_frame),
    .ifg_delay(ifg_delay)
);

endmodule
