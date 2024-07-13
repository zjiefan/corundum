module arst_resync
  import bitables_pkg::gnet_t;
#(
    parameter PIPES = 3  // 2 is enough, 3 is more generous to timing
)(
  input var gnet_t gnet,
  output    logic srst
);
  logic [PIPES:1] rst_p;
  always_ff@(posedge gnet.clk) begin
    rst_p[PIPES:1] <= {rst_p[PIPES-1:1], gnet.arst};
  end
  assign srst = rst_p[PIPES];
endmodule
