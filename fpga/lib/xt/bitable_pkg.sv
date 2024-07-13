package bitables_pkg;
  typedef struct packed {
    logic async_ready;
    logic arst;
    logic clk;
  } gnet_t; // Gnet 3bits 1bytes

  typedef struct packed {
    logic logic_error;
    logic underflow;
    logic overflow;
  } fifo_errors_t; // FifoErrors 3bits 1bytes

  typedef struct packed {
    logic [511 : 0] tdata;
    logic [15 : 0]   tkeep;
    logic           tlast;
    logic           tvalid;
  } axis512_t;

endpackage
