// Project #2: 16-bit pipeline processor
// Design file

`include "pcpu.v"

module processor (clock, reset, start, enable, select_y, y);

input clock, reset, enable, start;
input [3:0] select_y;
output [15:0] y;

  wire [15:0] i_datain;
  wire [7:0] i_addr;
  wire [7:0] d_addr;
  wire [15:0] d_datain;
  wire [15:0] d_dataout;
  wire d_we;

  // pipeline CPU top module
  pcpu pcpu (reset, clock, enable, start, i_addr, i_datain, d_addr,
             d_datain, d_dataout, d_we, select_y, y);

  imem imem (i_addr, i_datain);
  dmem dmem (d_addr, d_we, clock, d_dataout, d_datain);

endmodule

module imem (address,q);
input	[7:0]  address;
output	[15:0]  q;

reg [15:0]ram[255:0];

assign q = ram[address];

endmodule

module dmem (address,we,clock,data,q);
input	[7:0]  address;
input	  we;
input	  clock;
input	[15:0]  data;
output	[15:0]  q;

reg [15:0]ram[255:0];

assign q = ram[address];

always @(negedge clock)
if (we)
   ram[address] <= data;

endmodule
