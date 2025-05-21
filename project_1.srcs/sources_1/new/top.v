`timescale 1ns / 1ps
//==============================================================================
// top.v - FPGA Modulo Game Top-level
//==============================================================================

module top(
    input  wire        clk,          // 100 MHz system clock
    input  wire [15:0] sw,           // 16 switches for answer input
    output wire [15:0] led,          // 16 LEDs for timer countdown
    input  wire        btnR,         // reset (right)
    input  wire        btnL,         // pause (left)
    input  wire        btnU,         // start3 (difficulty=3, up)
    input  wire        btnD,         // start1 (difficulty=1, down)
    input  wire        btnC,         // start2 (difficulty=2, center)
    output reg  [6:0]  seg,          // seven-segment cathodes (active LOW)
    output reg  [3:0]  an            // seven-segment anodes (active LOW)
);

//==============================================================================
// 0) Parameters & State Codes
//==============================================================================
parameter SCAN_MAX = 200_000;
localparam HSEC   = 50_000_000;
localparam RESET  = 3'd0,
           RUN    = 3'd1,
           PAUSE  = 3'd2,
           ALL0   = 3'd3,
           ALL8   = 3'd4;

//==============================================================================
// 1) Signal Declarations
//==============================================================================
reg         rR0, rR1, rL0, rL1, rU0, rU1, rD0, rD1, rC0, rC1;
wire        cleanR, cleanL, cleanU, cleanD, cleanC;
reg         start3_edge, start2_edge, start1_edge, pause_edge, reset_edge;
reg   [2:0] state, next_state;
reg   [2:0] numLives, partialLife;
reg   [3:0] questionCount;
reg   [1:0] difficulty;
reg  [15:0] led_reg;
reg  [13:0] lfsr, questionNum, newlfsr;
reg   [3:0] correctAnswer;
reg  [15:0] prev_sw;
reg   [3:0] sw_idx;
reg         correct_edge;
reg   [1:0] dsel = 2'd0;
reg  [18:0] sc   = 19'd0;
reg  [25:0] hcnt;
reg         htick;
reg  [26:0] scnt;
reg         stick;
wire [15:0] changed;
integer     idx_i;

//==============================================================================
// 2) Synchronize & Debounce Buttons
//==============================================================================
always @(posedge clk) begin
    {rR1, rR0} <= {rR0, ~btnR};
    {rL1, rL0} <= {rL0, ~btnL};
    {rU1, rU0} <= {rU0, ~btnU};
    {rD1, rD0} <= {rD0, ~btnD};
    {rC1, rC0} <= {rC0, ~btnC};
end

debounce_sr #(.N(16)) dbR(.clk(clk), .rst(1'b0), .noisy(rR1), .clean(cleanR));
debounce_sr #(.N(16)) dbL(.clk(clk), .rst(1'b0), .noisy(rL1), .clean(cleanL));
debounce_sr #(.N(16)) dbU(.clk(clk), .rst(1'b0), .noisy(rU1), .clean(cleanU));
debounce_sr #(.N(16)) dbD(.clk(clk), .rst(1'b0), .noisy(rD1), .clean(cleanD));
debounce_sr #(.N(16)) dbC(.clk(clk), .rst(1'b0), .noisy(rC1), .clean(cleanC));

always @(posedge clk) begin
    start3_edge <= cleanU & ~rU1;
    start2_edge <= cleanC & ~rC1;
    start1_edge <= cleanD & ~rD1;
    pause_edge  <= cleanL & ~rL1;
    reset_edge  <= cleanR & ~rR1;
end

//==============================================================================
// 3) Tick Generators
//==============================================================================
always @(posedge clk) begin
    if (hcnt == HSEC-1) begin
        hcnt  <= 0;
        htick <= 1;
    end else begin
        hcnt  <= hcnt + 1;
        htick <= 0;
    end
end

always @(posedge clk) begin
    if (state == ALL8 || state == ALL0) begin
        if (scnt == 2*HSEC-1) begin
            scnt  <= 0;
            stick <= 1;
        end else begin
            scnt  <= scnt + 1;
            stick <= 0;
        end
    end else begin
        scnt  <= 0;
        stick <= 0;
    end
end

//==============================================================================
// 4) FSM State Register
//==============================================================================
always @(posedge clk) begin
    if (reset_edge && state != RESET) begin
        state <= RESET;
        dsel  <= 0;
        sc    <= 0;
    end else begin
        state <= next_state;
    end
end

//==============================================================================
// 5) FSM Next-State Logic
//==============================================================================
always @(*) begin
    next_state = state;
    case (state)
        RESET: if (start1_edge || start2_edge || start3_edge)
                    next_state = RUN;
        RUN:    if      (pause_edge)
                    next_state = PAUSE;
                else if (correct_edge)
                    next_state = ALL8;
                else if (htick && led_reg == 0)
                    next_state = ALL0;
                else if (numLives == 0)
                    next_state = RESET;
        PAUSE:  if (pause_edge)
                    next_state = RUN;
        ALL8:   if (stick)
                    next_state = RUN;
        ALL0:   if (stick)
                    next_state = (numLives > 0 ? RUN : RESET);
    endcase
end

//==============================================================================
// 6) Game Logic: Lives, Score & Difficulty
//==============================================================================
always @(posedge clk) begin
    if (state == RESET) begin
        // set difficulty & initial lives on Start-button
        if      (start1_edge) begin difficulty <= 1; numLives <= 1; end
        else if (start2_edge) begin difficulty <= 2; numLives <= 2; end
        else if (start3_edge) begin difficulty <= 3; numLives <= 3; end

        partialLife   <= 0;
        questionCount <= questionCount; // preserve last score
        lfsr          <= 14'h3FFF;
    end
    else if (state == RUN) begin
        if (correct_edge) begin
            questionCount <= questionCount + 1;
            partialLife   <= partialLife + 1;
            if (partialLife == 3) begin
                partialLife <= 0;
                numLives    <= numLives + 1;
            end
        end
        if (htick && led_reg == 0 && !correct_edge)
            numLives <= numLives - 1;
    end
end

//==============================================================================
// 7) LED Driver (reversed countdown)
//==============================================================================
assign led = led_reg;
always @(posedge clk) begin
    case (state)
        RESET: led_reg <= 16'hFFFF;
        RUN:    if (htick) led_reg <= led_reg >> 1;
        ALL0:   led_reg <= 16'h0000;
        ALL8:   led_reg <= 16'hFFFF;
        PAUSE:  led_reg <= led_reg;
        default:;
    endcase
end

//==============================================================================
// 8) PRNG & Answer Checking (now difficulty-aware)
//==============================================================================
always @(posedge clk) begin
    prev_sw <= sw;
    // on START or when the 8-second timer rolls over
    if ((state == RESET && (start1_edge||start2_edge||start3_edge)) ||
        (state == RUN   && htick && led_reg == 16'hFFFF)) begin

        // advance our 14-bit LFSR
        newlfsr = {lfsr[12:0], lfsr[13] ^ lfsr[12]};
        lfsr    <= newlfsr;

        // generate a random decimal question of N digits
        case (difficulty)
            2'd1: begin
                questionNum   <= 10  + (newlfsr %  90);   // 10-99
                correctAnswer <= (10  + (newlfsr %  90)) % 16;
            end
            2'd2: begin
                questionNum   <= 100 + (newlfsr % 900);   // 100-999
                correctAnswer <= (100 + (newlfsr % 900)) % 16;
            end
            2'd3: begin
                questionNum   <= 1000 + (newlfsr % 9000); // 1000-9999
                correctAnswer <= (1000 + (newlfsr % 9000)) % 16;
            end
            default: begin
                questionNum   <= 0;
                correctAnswer <= 0;
            end
        endcase
    end

    // detect a correct switch flip
    correct_edge <= (state == RUN && sw_idx == correctAnswer);
end

// detect which switch changed
assign changed = sw ^ prev_sw;
always @(*) begin
    sw_idx = 4'd0;
    for (idx_i = 0; idx_i < 16; idx_i = idx_i + 1)
        if (changed[idx_i])
            sw_idx = idx_i;
end

//==============================================================================
// 9) Display: show decimal question (2/3/4 digits), pause=lives, ALL8/ALL0 overrides
//==============================================================================
always @(posedge clk) begin
    if (sc == SCAN_MAX) begin
        sc   <= 0;
        dsel <= dsel + 1;
    end else begin
        sc <= sc + 1;
    end
end

always @(*) begin
    // default: blank
    an  = 4'b1111;
    seg = 7'b111_1111;

    case (dsel)
        // ones place
        2'd0: begin
            an = 4'b1110;
            if      (state == ALL8)  seg = decode7(4'd8);
            else if (state == ALL0)  seg = decode7(4'd0);
            else if (state == PAUSE) seg = decode7(numLives);
            else if (state == RUN)   seg = decode7(questionNum % 10);
            else                     seg = decode7(questionCount % 10);
        end

        // tens place (difficulty ? 1)
        2'd1: begin
            an = 4'b1101;
            if      (state == ALL8)  seg = decode7(4'd8);
            else if (state == ALL0)  seg = decode7(4'd0);
            else if (state == PAUSE) seg = 7'b111_1111;                        // blank
            else if (state == RUN && difficulty >= 1)
                                      seg = decode7((questionNum/10)   % 10);
            else                     seg = decode7((questionCount/10) % 10);
        end

        // hundreds place (difficulty ? 2)
        2'd2: begin
            an = 4'b1011;
            if      (state == ALL8)  seg = decode7(4'd8);
            else if (state == ALL0)  seg = decode7(4'd0);
            else if (state == PAUSE) seg = 7'b111_1111;                        // blank
            else if (state == RUN && difficulty >= 2)
                                      seg = decode7((questionNum/100)  % 10);
            else                     seg = decode7(4'd0);
        end

        // thousands place (difficulty ? 3)
        2'd3: begin
            an = 4'b0111;
            if      (state == ALL8)  seg = decode7(4'd8);
            else if (state == ALL0)  seg = decode7(4'd0);
            else if (state == PAUSE) seg = 7'b111_1111;                        // blank
            else if (state == RUN && difficulty >= 3)
                                      seg = decode7((questionNum/1000) % 10);
            else                     seg = decode7(4'd0);
        end
    endcase
end

//==============================================================================
// 10) Seven-Segment Decode Function
//==============================================================================
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

//==============================================================================
// Debounce Shift-Register Module
//==============================================================================
module debounce_sr #(
    parameter N = 16
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       noisy,
    output reg        clean
);
    reg [N-1:0] shift;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            shift <= {N{1'b0}};
            clean <= 1'b0;
        end else begin
            shift <= {shift[N-2:0], noisy};
            if (&shift)       clean <= 1'b1;
            else if (~|shift) clean <= 1'b0;
        end
    end
endmodule
