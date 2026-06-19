// Design file for project #2: 16-bit pipeline processor 
// This pipeline CPU contains 5 stages: instruction fetch (IF), instruction decode (ID), execution (EX), memory process (MEM), write back (WB)

// operation codes
`define NOP   5'b00000 // no operation
`define HALT  5'b00001 // halt
`define LOAD  5'b00010 // load
`define STORE 5'b00011 // store
`define SLL   5'b00100 // shift left logical
`define SRL   5'b00101 // shift right logical
`define SLA   5'b00110 // shift left arithmetic
`define SRA   5'b00111 // shift right arithmetic
`define ADD   5'b01000 // add
`define ADDI  5'b01001 // add immediate
`define SUB   5'b01010 // substract
`define SUBI  5'b01011 // substract immediat
`define CMP   5'b01100 // compare
`define AND   5'b01101 // and
`define OR    5'b01110 // or
`define XOR   5'b01111 // exlusive or
`define LDIH  5'b10000 // load immediate high
`define ADDC  5'b10001 // add with carry
`define SUBC  5'b10010 // subtract with carry
`define LOOP  5'b10100 // original operation: hardware loop
`define JUMP  5'b11000 // jump
`define JMPR  5'b11001 // jump register
`define BZ    5'b11010 // branch zero
`define BNZ   5'b11011 // branch not zero
`define BN    5'b11100 // branch negative
`define BNN   5'b11101 // branch not negative
`define BC    5'b11110 // branch carry
`define BNC   5'b11111 // branch not carry

// states for FSM
`define idle 1'b0
`define exec 1'b1

module pcpu (reset, clock, enable, start, i_addr, i_datain, d_addr,
             d_datain, d_dataout, d_we, select_y, y);

input reset, clock, enable, start;
input  [15:0] i_datain;  // instruction in
output [7:0]  i_addr;    // instruction address
output [7:0]  d_addr;    // data address
input  [15:0] d_datain;  // data in
output [15:0] d_dataout; // data out
output d_we;             // data write enable

// for Debugging
input  [3:0] select_y;
output [15:0] y;

// Definition of F/Fs
reg [7:0]  pc ;                                       // program counter
reg [15:0] id_ir, ex_ir, mem_ir, wb_ir;               // instruction registers for each stages
reg [15:0] gr [0:7];                                  // 8 general register
reg [15:0] reg_A, reg_B, reg_C;                       // data registers for different stages
reg [2:0] src_A, src_B, src_C, src_store;             // which register each operand comes from (0 means "not a reg" or gr0)
reg [2:0] id_src_A, id_src_B, id_src_C, id_src_store; // ID stage decoded source for current id_ir
reg [15:0] reg_ex, reg_mem, store_ID, store_EX;       // store memory data register

// flags
reg src_A_v, src_B_v, src_C_v, src_store_v;             // valid flags for each operand
reg id_src_A_v, id_src_B_v, id_src_C_v, id_src_store_v; // valid flags for current id_ir
reg zf, nf, cf, dw;                                     // zero flag, negative flag, data write enable
reg state;                                              // current stage
reg stall;                                              // stalling

// for data forwarding
reg [15:0] forward_A, forward_B, forward_C, forward_store; // forwarded reg_A, reg_B, reg_C, and store
reg [15:0] mem_value, wb_value;
reg mem_wen, wb_wen; // write enable of MEM and WB
reg [2:0] mem_rd, wb_rd; // register destination of MEM and WB

// Definition of temporary variables
reg [15:0] ALUo; // ALU output
reg [15:0] y;
reg next_state;  // next stage

assign i_addr = pc;
assign d_we  = (state == `exec) && (mem_ir[15:11] == `STORE);
assign d_addr = reg_ex[7:0];
assign d_dataout = store_EX;

// EX load destination
wire [2:0] ex_rd = ex_ir[10:8];
wire ex_is_load  = (ex_ir[15:11] == `LOAD);

// flag bypass for branch
wire mem_sets_flags = (mem_ir[15:11] != `NOP) &&
                     ((mem_ir[15:11] == `SLL)  || (mem_ir[15:11] == `SRL)  ||
                      (mem_ir[15:11] == `SLA)  || (mem_ir[15:11] == `SRA)  ||
                      (mem_ir[15:11] == `ADD)  || (mem_ir[15:11] == `ADDI) ||
                      (mem_ir[15:11] == `SUB)  || (mem_ir[15:11] == `SUBI) ||
                      (mem_ir[15:11] == `CMP)  ||
                      (mem_ir[15:11] == `AND)  || (mem_ir[15:11] == `OR)   || (mem_ir[15:11] == `XOR) ||
                      (mem_ir[15:11] == `LDIH) ||
                      (mem_ir[15:11] == `ADDC) || (mem_ir[15:11] == `SUBC));

// flags for branch
wire zf_for_branch = mem_sets_flags ? (reg_ex == 0) : zf;
wire nf_for_branch = mem_sets_flags ?  reg_ex[15]   : nf;
wire cf_for_branch = cf;

// control hazard
wire redirect = ((ex_ir[15:11] == `JUMP) || (ex_ir[15:11] == `JMPR)) ||
               (((ex_ir[15:11] == `BZ)   && (zf_for_branch == 1))    || ((ex_ir[15:11] == `BNZ) && (zf_for_branch == 0))  ||
                ((ex_ir[15:11] == `BN)   && (nf_for_branch == 1))    || ((ex_ir[15:11] == `BNN) && (nf_for_branch == 0))  ||
                ((ex_ir[15:11] == `BC)   && (cf_for_branch == 1))    || ((ex_ir[15:11] == `BNC) && (cf_for_branch == 0))) ||
                ((ex_ir[15:11] == `LOOP) && (ALUo != 16'h0000));
wire [7:0] redirect_pc = ((ex_ir[15:11] == `LOOP) && (ALUo != 16'h0000)) ? ex_ir[7:0] : ALUo[7:0];

// reset
always @(posedge clock or negedge reset)
    begin
      if (!reset)
        state <= `idle;
      else
        state <= next_state;
    end

// CPU Control (FSM)
always @(state or enable or start or wb_ir[15:11])
    begin
      case (state)
        `idle : if ((enable == 1'b1) && (start == 1'b1))
                  next_state <= `exec;
                else
                  next_state <= `idle;
        `exec : if ((enable == 1'b0) || (wb_ir[15:11] == `HALT))
                  next_state <= `idle;
                else
                  next_state <= `exec;
      endcase
    end

// 5-stage pipeline
// IF Block (1st Stage)
always @(posedge clock or negedge reset)
    begin
      if (!reset)
        begin
          id_ir <= 0;
          pc    <= 0;
        end
      else if(state==`exec)
        begin
          if (redirect)
            begin
              id_ir <= {`NOP, 11'b00000000000}; // flush instructions
              pc <= redirect_pc; // jump or branch
            end
          else if (!stall)
            begin
              // update program counter
              id_ir <= i_datain;
              pc <= pc + 1;
            end
        end
    
    end
 
// ID Block (2nd Stage)
always @(posedge clock or negedge reset)
    begin
     if (!reset)
        begin
          ex_ir <= 0;
          reg_A <= 0; reg_B <= 0; reg_C <= 0;
          src_A   <= 0; src_B   <= 0; src_C   <= 0; src_store <= 0;
          src_A_v <= 0; src_B_v <= 0; src_C_v <= 0; src_store_v <= 0;
          store_ID <= 0;
        end
     else if(state==`exec)
        begin
          if (redirect)
            begin
              // flush instruction
              ex_ir <= {`NOP, 11'b00000000000};
              reg_A <= 0; reg_B <= 0; reg_C <= 0;
              src_A <= 0; src_B <= 0; src_C <= 0; src_store <= 0;
              src_A_v <= 0; src_B_v <= 0; src_C_v <= 0; src_store_v <= 0;
              store_ID <= 0;
            end
          else if (stall)
            begin
              ex_ir <= {`NOP, 11'b00000000000}; // insert bubble
              reg_A <= 0; reg_B <= 0; reg_C <= 0;
              src_A   <= 0; src_B   <= 0; src_C   <= 0; src_store <= 0;
              src_A_v <= 0; src_B_v <= 0; src_C_v <= 0; src_store_v <= 0;
              store_ID <= 0;
            end
          else
            begin
              ex_ir <= id_ir;

              // update reg_A
              if ((id_ir[15:11] == `ADDI) || (id_ir[15:11] == `SUBI) ||
                  (id_ir[15:11] == `LOOP) ||
                  (id_ir[15:11] == `JMPR) ||
                  (id_ir[15:11] == `BZ)   || (id_ir[15:11] == `BNZ)  ||
                  (id_ir[15:11] == `BN)   || (id_ir[15:11] == `BNN)  ||
                  (id_ir[15:11] == `BC)   || (id_ir[15:11] == `BNC))
                begin
                  reg_A   <= rf_read_bypass(id_ir[10:8]); // reg_A <= r1
                  src_A <= id_ir[10:8]; // src_A <= operand1[2:0]
                  src_A_v <= 1; // src_A is a register
                end
              else
                begin
                  src_A <= 0;
                  src_A_v <= 0;
                end

              // update reg_B
              if ((id_ir[15:11] == `LOAD) || (id_ir[15:11] == `STORE) ||
                  (id_ir[15:11] == `SLL)  || (id_ir[15:11] == `SRL)   ||
                  (id_ir[15:11] == `SLA)  || (id_ir[15:11] == `SRA)   ||
                  (id_ir[15:11] == `ADD)  || (id_ir[15:11] == `SUB)   ||
                  (id_ir[15:11] == `CMP)  ||
                  (id_ir[15:11] == `AND)  || (id_ir[15:11] == `OR)    || (id_ir[15:11] == `XOR) ||
                  (id_ir[15:11] == `ADDC) || (id_ir[15:11] == `SUBC))
                begin
                  reg_B   <= rf_read_bypass(id_ir[6:4]); // reg_B <= r2
                  src_B <= id_ir[6:4]; // src_B <= operand1[2:0]
                  src_B_v <= 1; // src_B is a register
                end
              else if ((id_ir[15:11] == `ADDI) || (id_ir[15:11] == `SUBI) ||
                      (id_ir[15:11] == `LDIH) ||
                      (id_ir[15:11] == `LOOP) ||
                      (id_ir[15:11] == `JUMP) || (id_ir[15:11] == `JMPR) ||
                      (id_ir[15:11] == `BZ)   || (id_ir[15:11] == `BNZ)  ||
                      (id_ir[15:11] == `BN)   || (id_ir[15:11] == `BNN)  ||
                      (id_ir[15:11] == `BC)   || (id_ir[15:11] == `BNC))
                begin
                  reg_B <= {12'b000000000000, id_ir[7:4]}; // reg_B <= val2
                  src_B_v <= 0; // src_B is immediate number
                end
              else
                begin
                  src_B <= 0;
                  src_B_v <= 0;
                end
              
              // update reg_C
              if ((id_ir[15:11] == `ADD)  || (id_ir[15:11] == `SUB) ||
                  (id_ir[15:11] == `CMP)  ||
                  (id_ir[15:11] == `AND)  || (id_ir[15:11] == `OR)  || (id_ir[15:11] == `XOR) ||
                  (id_ir[15:11] == `ADDC) || (id_ir[15:11] == `SUBC))
                begin
                  reg_C   <= rf_read_bypass(id_ir[2:0]); // reg_C <= r3
                  src_C <= id_ir[2:0]; // src_C <= operand3[2:0]
                  src_C_v <= 1; // src_C is a register
                end
              else if ((id_ir[15:11] == `LOAD) || (id_ir[15:11] == `STORE) ||
                      (id_ir[15:11] == `SLL)  || (id_ir[15:11] == `SRL)   ||
                      (id_ir[15:11] == `SLA)  || (id_ir[15:11] == `SRA)   ||
                      (id_ir[15:11] == `ADDI) || (id_ir[15:11] == `SUBI)  ||
                      (id_ir[15:11] == `LDIH) ||
                      (id_ir[15:11] == `LOOP) ||
                      (id_ir[15:11] == `JUMP) || (id_ir[15:11] == `JMPR)  ||
                      (id_ir[15:11] == `BZ)   || (id_ir[15:11] == `BNZ)   ||
                      (id_ir[15:11] == `BN)   || (id_ir[15:11] == `BNN)   ||
                      (id_ir[15:11] == `BC)   || (id_ir[15:11] == `BNC))
                begin
                  reg_C <= {12'b000000000000, id_ir[3:0]}; // reg_C <= val3
                  src_C_v <= 0; // src_C is immediate number
                end
              else
                begin
                  src_C <= 0;
                  src_C_v <= 0;
                end

              // update store_ID
              if (id_ir[15:11] == `STORE)
                begin
                  store_ID<= rf_read_bypass(id_ir[10:8]);
                  src_store   <= id_ir[10:8];
                  src_store_v <= (id_ir[10:8] != 3'b000);
                end
              else 
                begin
                  src_store   <= 0;
                  src_store_v <= 0;
                end
            end
        end
    end

// EX Block (3rd Stage)
always @(posedge clock or negedge reset)
    begin
      if (!reset)
        begin
          mem_ir <= 0;
          reg_ex <= 0;
          store_EX <= 0;
          zf <= 0 ; nf <= 0; cf <= 0;
          dw <= 0 ;
        end
      else if(state==`exec)
        begin
          mem_ir <= ex_ir;
          reg_ex <= ALUo;

          // update flags
          if ((ex_ir[15:11] == `SLL)  || (ex_ir[15:11] == `SRL)  ||
              (ex_ir[15:11] == `SLA)  || (ex_ir[15:11] == `SRA)  ||
              (ex_ir[15:11] == `ADD)  || (ex_ir[15:11] == `ADDI) ||
              (ex_ir[15:11] == `SUB)  || (ex_ir[15:11] == `SUBI) ||
              (ex_ir[15:11] == `CMP)  ||
              (ex_ir[15:11] == `AND)  || (ex_ir[15:11] == `OR)   || (ex_ir[15:11] == `XOR) ||
              (ex_ir[15:11] == `LDIH) ||
              (ex_ir[15:11] == `ADDC) || (ex_ir[15:11] == `SUBC))
            begin
              // update zero flag
              if (ALUo == 0)
                zf <= 1;
              else
                zf <= 0;

              // update negative flag
              if (ALUo [15] == 1)
                nf <= 1;
              else
                nf <= 0;

              // update carry flag
              case (ex_ir[15:11])
                // arithmetic operations, cf <= 1 when carry or borrow occurs
                `ADD    : cf <= ({1'b0, forward_B} + {1'b0, forward_C}) > 17'h0FFFF;                                     // r1 <= r2 + r3
                `ADDI   : cf <= ({1'b0, forward_A} + {1'b0, {8'b00000000, forward_B[3:0], forward_C[3:0]}}) > 17'h0FFFF; // r1 <= r1 + {val2, val3}
                `SUB    : cf <= forward_B < forward_C;                                                                   // r1 <= r2 - r3
                `SUBI   : cf <= forward_A < {8'b00000000, forward_B[3:0], forward_C[3:0]};                               // r1 <= r1 - {val2, val3} 
                `CMP    : cf <= forward_B < forward_C;                                                                   // r2 - r3; CF, ZF and NF are set only
                `LDIH   : cf <= cf;                                                                                      // r1 <= {val2, val3, 00000000} (lower 8 bit can be given with ADDI)
                `ADDC   : cf <= ({1'b0, forward_B} + {1'b0, forward_C} + cf) > 17'h0FFFF;                                // r1 <= r2 + r3 + CF
                `SUBC   : cf <= {1'b0, forward_B} < ({1'b0, forward_C} + cf);                                            // r1 <= r2 - r3 - CF

                // logical operations, cf is always 0
                `AND    : cf <= 0;
                `OR     : cf <= 0;
                `XOR    : cf <= 0;

                // shift operations, cf doesn't change
                `SLL    : cf <= cf;
                `SRL    : cf <= cf;
                `SLA    : cf <= cf;
                `SRA    : cf <= cf;

                default : cf <= cf;
              endcase
            end

          // update data write
          if (ex_ir[15:11] == `STORE)
            begin
              dw <= 1;
              store_EX <= forward_store;
            end
          else
            dw <= 0;

        end
    end

// MEM Block (4th Stage)
always @(posedge clock or negedge reset)
    begin
      if (!reset)
        begin
          wb_ir <= 0;
          reg_mem <= 0;
        end
      else if(state==`exec)
        begin
          wb_ir <= mem_ir;

          // update reg_mem
          if (mem_ir[15:11] == `LOAD)
            reg_mem <= d_datain;
	        else
            reg_mem <= reg_ex;
        end
    end

// WB Block (5th Stege)
always @(posedge clock or negedge reset)
    begin
      if (!reset)
        begin
          gr[0] <= 0;
          gr[1] <= 0;
          gr[2] <= 0;
          gr[3] <= 0;
          gr[4] <= 0;
          gr[5] <= 0;
          gr[6] <= 0;
          gr[7] <= 0;
        end
      else if(state==`exec)
        begin
          // write back
          if (wb_ir[10:8] != 3'b000) // general register 0 is fixed
            if ((wb_ir[15:11] == `LOAD) ||
                (wb_ir[15:11] == `SLL)  || (wb_ir[15:11] == `SRL)  ||
                (wb_ir[15:11] == `SLA)  || (wb_ir[15:11] == `SRA)  ||
                (wb_ir[15:11] == `ADD)  || (wb_ir[15:11] == `ADDI) ||
                (wb_ir[15:11] == `SUB)  || (wb_ir[15:11] == `SUBI) ||
                (wb_ir[15:11] == `AND)  || (wb_ir[15:11] == `OR)   || (wb_ir[15:11] == `XOR) ||
                (wb_ir[15:11] == `LDIH) ||
                (wb_ir[15:11] == `ADDC) || (wb_ir[15:11] == `SUBC) ||
                (wb_ir[15:11] == `LOOP))
              gr[wb_ir[10:8]] <= reg_mem;
        end
    end

// MUX for data forwarding
always @(*)
    begin
      mem_wen = (mem_ir[10:8] != 3'b000) &&
                ((mem_ir[15:11] == `LOAD) ||
                 (mem_ir[15:11] == `SLL)  || (mem_ir[15:11] == `SRL)  ||
                 (mem_ir[15:11] == `SLA)  || (mem_ir[15:11] == `SRA)  ||
                 (mem_ir[15:11] == `ADD)  || (mem_ir[15:11] == `ADDI) ||
                 (mem_ir[15:11] == `SUB)  || (mem_ir[15:11] == `SUBI) ||
                 (mem_ir[15:11] == `AND)  || (mem_ir[15:11] == `OR)   || (mem_ir[15:11] == `XOR) ||
                 (mem_ir[15:11] == `LDIH) ||
                 (mem_ir[15:11] == `ADDC) || (mem_ir[15:11] == `SUBC) ||
                 (mem_ir[15:11] == `LOOP));
      mem_rd = mem_ir[10:8];
      mem_value = (mem_ir[15:11] == `LOAD) ? d_datain : reg_ex;
      
      wb_wen = (wb_ir[10:8] != 3'b000) &&
              ((wb_ir[15:11] == `LOAD) ||
               (wb_ir[15:11] == `SLL)  || (wb_ir[15:11] == `SRL)  ||
               (wb_ir[15:11] == `SLA)  || (wb_ir[15:11] == `SRA)  ||
               (wb_ir[15:11] == `ADD)  || (wb_ir[15:11] == `ADDI) ||
               (wb_ir[15:11] == `SUB)  || (wb_ir[15:11] == `SUBI) ||
               (wb_ir[15:11] == `AND)  || (wb_ir[15:11] == `OR)   || (wb_ir[15:11] == `XOR) ||
               (wb_ir[15:11] == `LDIH) ||
               (wb_ir[15:11] == `ADDC) || (wb_ir[15:11] == `SUBC) ||
               (wb_ir[15:11] == `LOOP));
      wb_rd = wb_ir[10:8];
      wb_value = reg_mem;

      forward_A = reg_A;
      if (src_A_v && mem_wen && (src_A == mem_rd))
        forward_A = mem_value;
      else if (src_A_v && wb_wen && (src_A == wb_rd))
        forward_A = wb_value;
      
      forward_B = reg_B;
      if (src_B_v && mem_wen && (src_B == mem_rd))
        forward_B = mem_value;
      else if (src_B_v && wb_wen && (src_B == wb_rd))
        forward_B = wb_value;

      forward_C = reg_C;
      if (src_C_v && mem_wen && (src_C == mem_rd))
        forward_C = mem_value;
      else if (src_C_v && wb_wen && (src_C == wb_rd))
        forward_C = wb_value;

      forward_store = store_ID;
      if (src_store_v && mem_wen && (src_store == mem_rd))
        forward_store = mem_value;
      else if (src_store_v && wb_wen && (src_store == wb_rd))
        forward_store = wb_value;

    end

// ALU module
always @(*)
  case (ex_ir[15:11])
    `LOAD   : ALUo = forward_B + forward_C;                         // r1 <= M[r2 + val3]
    `STORE  : ALUo = forward_B + forward_C;                         // M[r2 + val3] <= r1
    `SLL    : ALUo = forward_B << forward_C[3:0];                   // r1 <= r2 shift left logical (val3 bit shift)
    `SRL    : ALUo = forward_B >> forward_C[3:0];                   // r1 <= r2 shift right logical (val3 bit shift)
    `SLA    : ALUo = {forward_B[15], (forward_B[14:0] << forward_C[3:0])}; // r1 <= r2 shift left arithmetic (val3 bit shift)
    `SRA    : ALUo = $signed(forward_B) >>> forward_C[3:0];         // r1 <= r2 shift right arithmetic (val3 bit shift)
    `ADD    : ALUo = forward_B + forward_C;                         // r1 <= r2 + r3
    `ADDI   : ALUo = forward_A + {forward_B[3:0], forward_C[3:0]};  // r1 <= r1 + {val2, val3}
    `SUB    : ALUo = forward_B - forward_C;                         // r1 <= r2 - r3
    `SUBI   : ALUo = forward_A - {forward_B[3:0], forward_C[3:0]};  // r1 <= r1 - {val2, val3} 
    `CMP    : ALUo = forward_B - forward_C;                         // r2 - r3; cf, ZF and NF are set only
    `AND    : ALUo = forward_B & forward_C;                         // r1 <= r2 and r3
    `OR     : ALUo = forward_B | forward_C;                         // r1 <= r2 or r3
    `XOR    : ALUo = forward_B ^ forward_C;                         // r1 <= r2 xor r3
    `LDIH   : ALUo = {forward_B[3:0], forward_C[3:0], 8'b00000000}; // r1 <= {val2, val3, 00000000} (lower 8 bit can be given with ADDI)
    `ADDC   : ALUo = forward_B + forward_C + cf;                    // r1 <= r2 + r3 + cf
    `SUBC   : ALUo = forward_B - forward_C - cf;                    // r1 <= r2 - r3 - cf
    `LOOP   : ALUo = forward_A - 16'h0001;                          // r1 <= r1 - 1
    `JUMP   : ALUo = {forward_B[3:0], forward_C[3:0]};              // jump to {val2, val3}
    `JMPR   : ALUo = forward_A + {forward_B[3:0], forward_C[3:0]};  // jump to r1 + {val2, val3}
    `BZ     : ALUo = forward_A + {forward_B[3:0], forward_C[3:0]};  // if ZF = 1 jump to r1 + {val2, val3}
    `BNZ    : ALUo = forward_A + {forward_B[3:0], forward_C[3:0]};  // if ZF = 0 jump to r1 + {val2, val3}
    `BN     : ALUo = forward_A + {forward_B[3:0], forward_C[3:0]};  // if NF = 1 jump to r1 + {val2, val3}
    `BNN    : ALUo = forward_A + {forward_B[3:0], forward_C[3:0]};  // if NF = 0 jump to r1 + {val2, val3}
    `BC     : ALUo = forward_A + {forward_B[3:0], forward_C[3:0]};  // if cf = 1 jump to r1 + {val2, val3}
    `BNC    : ALUo = forward_A + {forward_B[3:0], forward_C[3:0]};  // if cf = 0 jump to r1 + {val2, val3}
    default : ALUo = 16'bXXXXXXXXXXXXXXXX;
  endcase

// ID-stage register file read with bypass (MEM first, then WB)
// r0 is hard-wired to 0
function [15:0] rf_read_bypass;
  input [2:0] raddr;
  begin
    if (raddr == 3'b000) begin
      rf_read_bypass = 16'h0000;
    end
    else if (mem_wen && (raddr == mem_rd)) begin
      rf_read_bypass = mem_value;
    end
    else if (wb_wen && (raddr == wb_rd)) begin
      rf_read_bypass = wb_value;
    end
    else begin
      rf_read_bypass = gr[raddr];
    end
  end
endfunction

// decode ID source
always @(*) 
    begin
      // defaults
      id_src_A = 0; id_src_B = 0; id_src_C = 0; id_src_store = 0;
      id_src_A_v = 0; id_src_B_v = 0; id_src_C_v = 0; id_src_store_v = 0;

      // update id_src_A
      if ((id_ir[15:11] == `ADDI) || (id_ir[15:11] == `SUBI) ||
          (id_ir[15:11] == `LOOP) ||
          (id_ir[15:11] == `JMPR) ||
          (id_ir[15:11] == `BZ)   || (id_ir[15:11] == `BNZ)  ||
          (id_ir[15:11] == `BN)   || (id_ir[15:11] == `BNN)  ||
          (id_ir[15:11] == `BC)   || (id_ir[15:11] == `BNC))
        begin
          id_src_A   = id_ir[10:8];
          id_src_A_v = (id_ir[10:8] != 3'b000);
        end

      // update id_src_B
      if ((id_ir[15:11] == `LOAD) || (id_ir[15:11] == `STORE) ||
          (id_ir[15:11] == `SLL)  || (id_ir[15:11] == `SRL)   ||
          (id_ir[15:11] == `SLA)  || (id_ir[15:11] == `SRA)   ||
          (id_ir[15:11] == `ADD)  || (id_ir[15:11] == `SUB)   ||
          (id_ir[15:11] == `CMP)  ||
          (id_ir[15:11] == `AND)  || (id_ir[15:11] == `OR)    || (id_ir[15:11] == `XOR) ||
          (id_ir[15:11] == `ADDC) || (id_ir[15:11] == `SUBC))
        begin
          id_src_B   = id_ir[6:4];
          id_src_B_v = (id_ir[6:4] != 3'b000);
        end

      // update id_src_C
      if ((id_ir[15:11] == `ADD)  || (id_ir[15:11] == `SUB)  ||
          (id_ir[15:11] == `CMP)  ||
          (id_ir[15:11] == `AND)  || (id_ir[15:11] == `OR)   || (id_ir[15:11] == `XOR) ||
          (id_ir[15:11] == `ADDC) || (id_ir[15:11] == `SUBC))
        begin
          id_src_C   = id_ir[2:0];
          id_src_C_v = (id_ir[2:0] != 3'b000);
        end

      // update id_src_store
      if (id_ir[15:11] == `STORE) begin
        id_src_store   = id_ir[10:8];
        id_src_store_v = (id_ir[10:8] != 3'b000);
      end
    end

// stall
always @(*)
    begin
      stall = 0;
      if (ex_is_load && (ex_rd != 3'b000))
        begin
          if ((id_src_A_v && (id_src_A == ex_rd)) ||
              (id_src_B_v && (id_src_B == ex_rd)) ||
              (id_src_C_v && (id_src_C == ex_rd)) ||
              (id_src_store_v && (id_src_store == ex_rd)))
            stall = 1;
        end
    end

// Debug
always @(select_y or gr[1] or gr[2] or gr[3] or gr[4] or gr[5] or gr[6] or gr[7] or
         reg_A or reg_B or reg_C or reg_ex or reg_mem or store_ID or id_ir or
         pc or zf or nf or cf or dw)
  begin
    case (select_y)
      4'b0000 : y = {3'b000, dw, 1'b0, zf, nf, cf, pc};
      4'b0001 : y = gr[1];
      4'b0010 : y = gr[2];
      4'b0011 : y = gr[3];
      4'b0100 : y = gr[4];
      4'b0101 : y = gr[5];
      4'b0110 : y = gr[6];
      4'b0111 : y = gr[7];
      4'b1000 : y = reg_A;
      4'b1001 : y = reg_B;
      4'b1011 : y = reg_C;
      4'b1100 : y = reg_ex;
      4'b1101 : y = reg_mem;
      4'b1110 : y = store_ID;
      4'b1111 : y = id_ir;
      default : y = 16'bXXXXXXXXXXXXXXXX;
    endcase
  end

endmodule