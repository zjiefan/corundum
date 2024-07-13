module errors_locker
  import bitables_pkg::gnet_t;
  #(
    parameter SKIP_RESET_RESYNC = 0,
    parameter BITS = 1,
    localparam type errors_t = logic [BITS-1:0]
  )(
    input var gnet_t gnet,
    input var errors_t i_errors,
    output var errors_t o_errors
  );
  logic srst;
  logic locked;

  generate
    if(SKIP_RESET_RESYNC) begin
      assign srst = gnet.arst;
    end else begin
      arst_resync reset_resync( .gnet, .srst);
    end
  endgenerate

  always_ff@(posedge gnet.clk) begin
    locked <= |{locked, i_errors};
    for(int i = 0; i < $bits(errors_t); i++) begin
      if(!locked && i_errors[i]) o_errors[i] <= 1;
    end
    if(srst) begin
      locked <= 0;
      o_errors <= 0;
    end
  end
endmodule
