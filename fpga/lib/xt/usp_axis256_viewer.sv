module usp_axis256_viewer
  import bitables_pkg::gnet_t;
  import bitables_pkg::axis512_t;
  (
    input var gnet_t gnet,
    input var axis512_t din
  );
  logic srst;
  arst_resync reset_resync( .gnet, .srst);

  enum logic [0:0] {
    IDLE_STA,
    BUSY_STA
  } next_state, state;

  typedef struct packed {
    logic [31: 0] tdata;
    logic [15: 0]  tkeep;
    logic         tlast;
    logic         tvalid;
    logic [3:0]   cnt;
    logic [3:0]   tcnt;
  } self_t;
  self_t self, next_self;

  struct packed {
    logic fifo_rden;
  } signals, next_signals;

  axis512_t rdat;
  logic rval;
  bitables_pkg::fifo_errors_t    fifo_errors;

  always_comb begin
    next_state = state;
    next_self = self;
    next_signals = 0;
    case(state)
      IDLE_STA: begin
        next_self = 0;
        if(rval) begin
          next_self.tdata = rdat.tdata[0+:32];
          next_self.tkeep = rdat.tkeep;
          next_self.tlast = rdat.tlast;
          next_self.tvalid = rdat.tvalid;
          next_self.tcnt = 0;
          next_self.cnt = 1;
          next_state = BUSY_STA;
        end
      end
      BUSY_STA: begin
        next_self.tdata = rdat.tdata[self.cnt*32+:32];
        next_self.tcnt = self.cnt;
        next_self.cnt = self.cnt + 1;
        if(self.cnt == 15) begin
          next_signals.fifo_rden = 1;
          next_self.cnt = 0;
          next_state = IDLE_STA;
        end
      end
    endcase
  end

  always_ff@(posedge gnet.clk) begin
    self <= next_self;
    signals <= next_signals;
    state <= next_state;
    if(srst) begin
      state <= IDLE_STA;
    end
  end

  logic [9:0] signed_used;

  low_latency_scfifo #(
    .DATA_BITS        ($bits(axis512_t)),
    .ADDR_BITS        (9)
  )
  u_low_latency_scfifo (
    .gnet,
    .wren       (din.tvalid),
    .wdat       (din),
    .rden       (next_signals.fifo_rden),
    .rdat       (rdat),
    .rval       (rval),
    .signed_used(signed_used),
    .neg        (),
    .full       (),
    .alfull     (),
    .alempty    (),
    .o_errors   (fifo_errors)
  );

  (* mark_debug="true" *) self_t debug_self;
  (* mark_debug="true" *) logic [9:0] debug_used;
  (* mark_debug="true" *) bitables_pkg::fifo_errors_t debug_fifo_errors;

  always_ff@(posedge gnet.clk) begin
    debug_self <= self;
    debug_used <= signed_used;
    debug_fifo_errors <= fifo_errors;
  end

endmodule
