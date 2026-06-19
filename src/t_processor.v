// Project #2: 16-bit pipeline processor
// Test pattern
`timescale 1ns / 10ps

module t_processor;

  reg reset, clock, enable, start;
  reg [3:0] select_y;
  wire [15:0] y;

  initial begin
  $readmemh("imem.ini", processor.imem.ram);
  end

  initial begin
  $readmemh("dmem.ini", processor.dmem.ram);
  end

  integer x;
  integer dmem;
  initial dmem= $fopen("dmem.mem");
  task dmem_dump;
	begin
	x = 0;
        $fdisplay (dmem,"");
	repeat (256)
		begin
		$fdisplay (dmem,"Add:%x, Cell:%x", x, processor.dmem.ram[x]);
		x=x+1;
		end
	end
  endtask


  // Clock
  initial clock = 0;
  always
    #5 clock = ~clock;

  initial
    begin
      $monitor("%h, %h, %h", y, processor.i_datain, processor.i_addr);
      enable <= 0; start <=0; reset <=0; select_y <= 3;
      #10 reset <= 0;
      #10 reset <= 1;
      #10 enable <= 1;
      #10 start <= 1; 
      #10 start <= 0; 
      #10000000
      dmem_dump;
      $finish;
    end

   processor processor (clock, reset, start, enable, select_y, y);

endmodule
