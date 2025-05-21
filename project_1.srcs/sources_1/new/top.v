`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/12/2025 01:15:32 PM
// Design Name: 
// Module Name: top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module top(
    input  wire       clk,        // 100 MHz
    input  wire      [15:0] sw,   // 16 switchs
    input  wire      [15:0] led,  // 16 leds above switches
    
    input  wire       btnR,      // right button
    input  wire       btnL,      // left button
    input  wire       btnU,      // up button
    input  wire       btnD,      // down button
    input  wire       btnC,      // center button
    
    output reg  [6:0] seg,        // cathodes a-g (active-LOW)
    output reg  [3:0] an          // anodes D0-D3 (active-LOW)
    );
    wire rightRaw = ~btnR;
    wire leftRaw = ~btnL;
    wire upRaw = ~btnU;
    wire downRaw = ~btnD;
    wire centerRaw = ~btnC;
    
    reg r0_sync0, r0_sync1;
    reg l0_sync0, l0_sync1;
    reg u0_sync0, u0_sync1;
    reg d0_sync0, d0_sync1;
    reg c0_sync0, c0_sync1;
    
    reg [3:0] scoreOne = 0;
    reg [3:0] scoreTen = 0;
    reg [3:0] scoreHun = 1;
    reg [3:0] scoreThou = 0;
    
    reg lossLife = 0;
    reg [6:0] partialLife = 7'b0000000;
    reg [2:0] numLives = 3;
    reg [3:0] d3 = 0, d2 = 8, d1 = 8, d0 = 8;

    reg state = 0;
    
    always @(posedge clk) begin 
            r0_sync0 <= rightRaw;   r0_sync1 <= r0_sync0;
            l0_sync0 <= leftRaw;   l0_sync1 <= l0_sync0;
            u0_sync0 <= upRaw;   u0_sync1 <= u0_sync0;
            d0_sync0 <= downRaw;   d0_sync1 <= d0_sync0;
            c0_sync0 <= centerRaw;   c0_sync1 <= c0_sync0;

    end
    
    wire rightClean, leftClean, upClean, downClean, centerClean;
    debounce_sr #(.N(16)) dbR (
          .clk(clk), .rst(1'b0),
          .noisy(r0_sync1), .clean(rightClean)
        );
    
    debounce_sr #(.N(16)) dbL (
          .clk(clk), .rst(1'b0),
          .noisy(l0_sync1), .clean(leftClean)
        );
    
    debounce_sr #(.N(16)) dbU (
          .clk(clk), .rst(1'b0),
          .noisy(u0_sync1), .clean(upClean)
        );
        
    debounce_sr #(.N(16)) dbD (
          .clk(clk), .rst(1'b0),
          .noisy(d0_sync1), .clean(downClean)
        );
       
    debounce_sr #(.N(16)) dbC (
          .clk(clk), .rst(1'b0),
          .noisy(c0_sync1), .clean(centerClean)
        );
    reg btnL_d, btnR_d, btnD_d, btnC_d, btnU_d;
            always @(posedge clk) begin
                btnL_d <= leftClean;
                btnR_d <= rightClean;
                btnD_d <= downClean;
                btnC_d <= centerClean;
                btnU_d <= rightClean;
            end
    wire reset_pressed = btnL & ~btnL_d;
    wire pause_pressed = btnR & ~btnR_d;
    wire start1_pressed = btnL & ~btnL_d;
    wire start2_pressed = btnR & ~btnR_d;
    wire start3_pressed = btnR & ~btnR_d;
    
   reg [26:0] cnt1    = 0, cnt2    = 0;
   reg [17:0] cntMux  = 0;
   reg        r1      = 0, r2      = 0, rMux = 0;

   always @(posedge clk) begin
           // 1 Hz
           if (cnt1 == 50_000_000-1) begin cnt1 <= 0; r1 <= ~r1; end
           else                cnt1 <= cnt1 + 1;
           // 2 Hz
           if (cnt2 == 25_000_000-1) begin cnt2 <= 0; r2 <= ~r2; end
           else                cnt2 <= cnt2 + 1;
           // ~500 Hz scan
           if (cntMux == 100_000-1) begin cntMux <= 0; rMux <= ~rMux; end
           else                  cntMux <= cntMux + 1;
    end
    reg [1:0] mux_idx = 0;
       always @(posedge clk) begin
           if (reset_pressed)        mux_idx <= 0;
           else         mux_idx <= mux_idx + 1;
       end
       
    reg [3:0] cur;
        always @(*) begin
            an  = 4'b1111;
            seg = 7'b111_1111;
            case (mux_idx)
              2'd0: begin an=4'b1110; cur=d0; end
              2'd1: begin an=4'b1101; cur=d1; end
              2'd2: begin an=4'b1011; cur=d2; end
              2'd3: begin an=4'b0111; cur=d3; end
            endcase
               seg = decode7(cur);
    end
    function [6:0] decode7(input [3:0] v);
        case (v)
            4'd0: decode7 = 7'b100_0000;
            4'd1: decode7 = 7'b111_1001;
            4'd2: decode7 = 7'b010_0100;
            4'd3: decode7 = 7'b011_0000;
            4'd4: decode7 = 7'b001_1001;
            4'd5: decode7 = 7'b001_0010;
            4'd6: decode7 = 7'b000_0010;
            4'd7: decode7 = 7'b111_1000;
            4'd8: decode7 = 7'b000_0000;
            4'd9: decode7 = 7'b001_0000;
            default: decode7 = 7'b111_1111;
        endcase
    endfunction
endmodule


module debounce_sr #(
    parameter N = 16
) (
    input  wire       clk,
    input  wire       rst,    // optional async reset (tie low if unused)
    input  wire       noisy,  // synchronized but bouncy input
    output reg        clean
);
    reg [N-1:0] shift;

    always @(posedge clk or posedge rst) begin
                if (rst) begin
                    shift <= {N{1'b0}};
                    clean <= 1'b0;
                end else begin
                    shift <= { shift[N-2:0], noisy };
                    if (&shift)       clean <= 1'b1;
                    else if (~|shift) clean <= 1'b0;
                end
            end
endmodule
