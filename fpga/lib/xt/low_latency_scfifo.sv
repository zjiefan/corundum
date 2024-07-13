module low_latency_scfifo
  import bitables_pkg::gnet_t;
  import bitables_pkg::fifo_errors_t;
  #(
    parameter SKIP_RESET_RESYNC = 0,
    parameter ZERO_LATENCY = 0,
    parameter int THRESHOLD_LOW = 0,
    parameter DATA_BITS = 8,
    parameter ADDR_BITS = 6,
    parameter ALMOST_FULL = 3 * 2 **(ADDR_BITS-2),
    parameter ALMOST_EMPTY = 1 * 2 **(ADDR_BITS-2),
    localparam type data_t = logic [DATA_BITS-1:0],
    localparam type cnt_t = logic signed [ADDR_BITS:0]

  )(
    input var gnet_t      gnet,
    input var logic       wren,
    input var data_t      wdat,
    input var logic       rden,
    output    data_t      rdat,
    output    logic       rval,
    output    cnt_t       signed_used,
    output    logic       neg,
    output    logic       full,
    output    logic       alfull,
    output    logic       alempty,
    output fifo_errors_t    o_errors
  );

  logic srst;
  generate
    if(SKIP_RESET_RESYNC) begin
      assign srst = gnet.arst;
    end else begin
      arst_resync reset_resync( .gnet, .srst);
    end
  endgenerate

  typedef logic [ADDR_BITS-1:0] addr_t;

  data_t mem_rdat;
  logic rden_p1;
  logic wren_p1;
  data_t wdat_p1;
  logic mem_rden_p2;
  data_t [2:0] buffer;
  fifo_errors_t errors;
  cnt_t threshold_low = THRESHOLD_LOW[ADDR_BITS:0];
  cnt_t threshold_alempty = ALMOST_EMPTY[ADDR_BITS:0];
  cnt_t threshold_alfull = ALMOST_FULL[ADDR_BITS:0];

  struct packed {
    cnt_t used;
    logic rval;
    logic more_than_3;
    addr_t mem_radd;
    addr_t mem_wadd;
    logic full;
  } self, next_self;

  struct packed {
    logic mem_rden;
  } signals, next_signals;

  cnt_t minus1 = 2 ** ADDR_BITS - 1;
  cnt_t minus2 = 2 ** ADDR_BITS - 2;

  assign signed_used = self.used;
  assign full = self.full;
  assign neg = self.used < 0;
  always_comb begin
    next_self = self;
    next_signals = 0;
    case({wren, rden})
      2'b10: begin
        next_self.used = self.used + 1'b1;
        if(self.used >= 0) begin
          next_self.rval = 1;
        end
        if(self.used == 3) begin
          next_self.more_than_3 = 1;
          next_self.mem_wadd = self.mem_wadd + 1;
        end
        if(self.more_than_3) begin
          next_self.mem_wadd = self.mem_wadd + 1;
        end
        if(self.used == minus2) begin
          next_self.full = 1;
        end
      end
      2'b01: begin
        next_self.used = self.used - 1'b1;
        next_self.full = 0;
        if(self.used == 4) begin
          next_self.more_than_3 = 0;
        end
        if(self.used[1:0] == 1 && !self.more_than_3) begin
          next_self.rval = 0;
        end
      end
      2'b11: begin
        if(self.more_than_3) begin
          next_self.mem_wadd = self.mem_wadd + 1;
        end
      end
    endcase

    if(rden && self.more_than_3) begin
      next_signals.mem_rden = 1;
      next_self.mem_radd = self.mem_radd + 1;

    end

  end

  always_ff@(posedge gnet.clk) begin
    rden_p1 <= rden;
    wren_p1 <= wren;
    wdat_p1 <= wdat;
    mem_rden_p2 <= signals.mem_rden;

    if(rden) begin
      buffer[2:1] <= buffer[1:0];
      buffer[0] <= 0;
    end
    if(mem_rden_p2) begin
      case({rden_p1, rden})
        2'b00: buffer[0] <= mem_rdat;
        2'b10, 2'b01: buffer[1] <= mem_rdat;
        2'b11: buffer[2] <= mem_rdat;
      endcase
    end

    if (!self.more_than_3) begin
      if(rden) begin
        case(self.used[1:0])
          1: buffer[2] <= wdat;
          2: buffer[1] <= wdat;
          3: buffer[0] <= wdat;
        endcase
      end
      else begin
        case(self.used[1:0])
          0: buffer[2] <= wdat;
          1: buffer[1] <= wdat;
          2: buffer[0] <= wdat;
        endcase
      end
    end

    // if(srst) begin
    //   buffer <= 0;
    // end
  end

  always_comb begin
    rdat = buffer[2];
    rval = self.rval;
    if(ZERO_LATENCY) begin
      if(wren && !self.rval) begin
        rval = 1;
        rdat = wdat;
      end
    end
    // synopsys translate_off
    if(!rval) rdat = {DATA_BITS{1'bx}};
  // synopsys translate_on
  end


  always_ff@(posedge gnet.clk) begin
    errors <= 0;
    errors.overflow <= self.full && wren;
    errors.underflow <= (self.used == threshold_low) && rden;
    alempty <= self.used <= threshold_alempty;
    alfull <= self.used >= threshold_alfull;
    if(self.used >= 0) begin
      if((self.used[ADDR_BITS-1:0] == minus1) != self.full) errors.logic_error <= 1;
      if((self.used[ADDR_BITS-1:0] >= 4) != self.more_than_3) errors.logic_error <= 1;
      if((self.used[ADDR_BITS-1:0] == 0) == self.rval) errors.logic_error <= 1;
    end
    if((self.used < 0) && self.rval) errors.logic_error <= 1;
  end

  always_ff@(posedge gnet.clk) begin
    self <= next_self;
    signals <= next_signals;
    if(srst) begin
      self <= 0;
    end
  end

  scdpram #(
    .DATA_BITS(DATA_BITS),
    .ADDR_BITS(ADDR_BITS)
  ) u_scdpram (
    .gnet,
    .wren(wren),
    .wadd(next_self.mem_wadd),
    .wdat(wdat),
    .rden(next_signals.mem_rden),
    .radd(next_self.mem_radd),
    .rdat(mem_rdat)
  );

  errors_locker #(
    .BITS($bits(fifo_errors_t))
  ) u_errors_locker (
    .gnet,
    .i_errors(errors),
    .o_errors
  );

  logic or_error; // read only in cocotb
  assign or_error = |o_errors;

  // synopsys translate_off
  int debug_wadd = 0;
  int debug_radd = 0;

  always_ff@(posedge gnet.clk) begin
    if(wren) debug_wadd <= debug_wadd + 1;
    if(rden) debug_radd <= debug_radd + 1;
  end


  // synopsys translate_on

endmodule
