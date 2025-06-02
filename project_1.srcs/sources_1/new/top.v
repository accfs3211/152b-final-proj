`timescale 1ns / 1ps
//==============================================================================
// top.v - FPGA Modulo Game Top-level (with debounce and correct digit declarations)
//==============================================================================
module top(
    input  wire        clk,           // 100 MHz system clock
    input  wire [15:0] sw,            // 16 raw switches for answer input
    output wire [15:0] led,           // 16 LEDs for timer countdown
    input  wire        btnR,          // reset (right)
    input  wire        btnL,          // pause (left)
    input  wire        btnU,          // start3 (difficulty=3, up)
    input  wire        btnD,          // start1 (difficulty=1, down)
    input  wire        btnC,          // start2 (difficulty=2, center)
    output reg  [6:0]  seg,           // seven-segment cathodes (active LOW)
    output reg  [3:0]  an             // seven-segment anodes (active LOW)
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

reg   [2:0] state = RESET, next_state;
reg   [2:0] numLives = 0, partialLife = 0;
reg   [3:0] questionCount = 0;
reg   [3:0] prevScore    = 4'd0;      // holds last run's score
reg   [1:0] difficulty   = 0;
reg  [15:0] led_reg      = 16'hFFFF;
reg  [13:0] lfsr         = 14'h3FFF, questionNum = 14'd0, newlfsr = 14'd0;
reg   [3:0] correctAnswer = 4'd0;
reg   [3:0] sw_idx        = 4'd0;
reg         correct_edge  = 1'b0;
reg  [15:0] last_sw       = 16'd0;   // snapshot of debounced switches
integer     idx_i;

reg  [25:0] hcnt          = 26'd0;
reg         htick         = 1'b0;
reg  [26:0] scnt          = 27'd0;
reg         stick         = 1'b0;

reg  [18:0] sc            = 19'd0;
reg   [1:0] mux_idx       = 2'd0;
reg   [3:0] an_reg;
reg   [6:0] seg_reg;
localparam integer BLANK = SCAN_MAX / 8;

//------------------------------------------------------------------------------
// Debounced versions of each switch (16 total)
//   - We feed raw sw[i] into debounce_sr to get cleanSW[i].
//   - cleanSW changes only after N consecutive stable clk cycles.
//------------------------------------------------------------------------------
wire [15:0] cleanSW;
genvar i;
generate
  for (i = 0; i < 16; i = i + 1) begin : DEB_SW
    debounce_sr #(.N(16)) dbSW (
      .clk   (clk),
      .rst   (1'b0),
      .noisy (sw[i]),
      .clean (cleanSW[i])
    );
  end
endgenerate

//------------------------------------------------------------------------------
// Precompute each decimal digit of questionNum (0-9999)
//------------------------------------------------------------------------------
wire [3:0] d0 =  questionNum        % 10;   // ones
wire [3:0] d1 = (questionNum /   10) % 10;   // tens
wire [3:0] d2 = (questionNum /  100) % 10;   // hundreds
wire [3:0] d3 = (questionNum / 1000) % 10;   // thousands

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
            scnt <= 0; 
            stick <= 1; 
        end else begin 
            scnt <= scnt + 1; 
            stick <= 0; 
        end
    end else begin 
        scnt <= 0; 
        stick <= 0; 
    end
end

//==============================================================================
// 4) FSM State Register
//==============================================================================
always @(posedge clk) begin
    if (reset_edge && state != RESET) begin
        state <= RESET;
    end else begin
        state <= next_state;
    end
end

//==============================================================================
// 5) Capture previous score on RESET entry
//==============================================================================
always @(posedge clk) begin
    if (state != RESET && next_state == RESET)
        prevScore <= questionCount;
end

//==============================================================================
// 6) FSM Next-State Logic
//==============================================================================
always @(*) begin
    next_state = state;
    case (state)
        RESET: if (start1_edge || start2_edge || start3_edge) 
                    next_state = RUN;
        RUN:    if (pause_edge)             
                    next_state = PAUSE;
                else if (correct_edge)      
                    next_state = ALL8;
                else if (htick && led_reg == 16'h0000) 
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
// 7) Game Logic: Lives & Score
//==============================================================================
always @(posedge clk) begin
    if (state == RESET) begin
        if      (start1_edge) begin 
            difficulty    <= 2'd1; 
            numLives      <= 3'd1; 
            questionCount <= 4'd0; 
        end
        else if (start2_edge) begin 
            difficulty    <= 2'd2; 
            numLives      <= 3'd2; 
            questionCount <= 4'd0; 
        end
        else if (start3_edge) begin 
            difficulty    <= 2'd3; 
            numLives      <= 3'd3; 
            questionCount <= 4'd0; 
        end
        partialLife <= 3'd0;
    end else if (state == RUN) begin
        if (correct_edge) begin
            questionCount <= questionCount + 1;
            partialLife   <= partialLife + 1;
            if (partialLife == 3) begin
                partialLife <= 3'd0;
                numLives    <= numLives + 1;
            end
        end
        if (htick && led_reg == 16'h0000 && !correct_edge)
            numLives <= numLives - 1;
    end
end

//==============================================================================
// 8) LED Driver (reversed countdown)
//==============================================================================
assign led = led_reg;
always @(posedge clk) begin
    if (state == RESET) begin
        led_reg <= 16'hFFFF;
    end else if (((state == RESET) && (next_state == RUN)) ||
                 ((state == ALL0 ) && (next_state == RUN)) ||
                 ((state == ALL8 ) && (next_state == RUN))) begin
        led_reg <= 16'hFFFF;
    end else begin
        case (state)
            RUN:   if (htick)          led_reg <= led_reg >> 1;
            ALL0:                      led_reg <= 16'h0000;
            ALL8:                      led_reg <= 16'hFFFF;
            PAUSE:                     led_reg <= led_reg;
            default: ;
        endcase
    end
end

//==============================================================================
// 9) PRNG & Answer Checking (debounced flip detection)
//==============================================================================
always @(posedge clk) begin
    // 1) On a true "new question" transition:
    if (((state == RESET) && (next_state == RUN)) ||
        ((state == ALL0 ) && (next_state == RUN)) ||
        ((state == ALL8 ) && (next_state == RUN))) begin

        // Advance LFSR and pick new question:
        newlfsr      = {lfsr[12:0], lfsr[13] ^ lfsr[12]};
        lfsr         <= newlfsr;
        case (difficulty)
            2'd1: questionNum <= 10   + (newlfsr %   90);
            2'd2: questionNum <= 100  + (newlfsr %  900);
            2'd3: questionNum <= 1000 + (newlfsr % 9000);
            default: questionNum <= questionCount;
        endcase
        correctAnswer <= questionNum % 16;

        // Take a snapshot of the debounced switches for flip detection:
        last_sw       <= cleanSW;
        correct_edge  <= 1'b0;
    end
    // 2) While in RUN, watch for a single 0?1 on the "correctAnswer" bit:
    else if (state == RUN) begin
        // (cleanSW & ~last_sw) has '1' exactly where a 0?1 occurred:
        if ((cleanSW & ~last_sw) == (16'b1 << correctAnswer))
            correct_edge <= 1'b1;
        else
            correct_edge <= 1'b0;

        // Always update snapshot at end of cycle so next clock uses fresh last_sw:
        last_sw <= cleanSW;
    end
    // 3) In any other state, clear correct_edge and keep last_sw synced:
    else begin
        correct_edge <= 1'b0;
        last_sw      <= cleanSW;
    end
end

//==============================================================================
// 10) Scan-counter & 4-Digit Multiplexing (with deghosting via BLANK)
//==============================================================================
always @(posedge clk or posedge reset_edge) begin
    if (reset_edge) begin
        sc      <= 19'd0;
        mux_idx <= 2'd0;
    end else if (sc == SCAN_MAX) begin
        sc      <= 19'd0;
        mux_idx <= mux_idx + 1;
    end else begin
        sc <= sc + 1;
    end
end

always @(posedge clk) begin
    an  <= an_reg;
    seg <= seg_reg;
end

reg [3:0] cur;
always @(*) begin
    an_reg  = 4'b1111;
    seg_reg = 7'b111_1111;

    if (sc >= BLANK) begin
        case (mux_idx)
            2'd0: an_reg = 4'b1110;
            2'd1: an_reg = 4'b1101;
            2'd2: an_reg = 4'b1011;
            2'd3: an_reg = 4'b0111;
        endcase

        if      (state == ALL8 ) cur = 4'd8;
        else if (state == ALL0 ) cur = 4'd0;
        else if (state == PAUSE) cur = numLives;
        else if (state == RESET) begin
            case (mux_idx)
                2'd0: cur = prevScore % 10;
                2'd1: cur = (prevScore / 10) % 10;
                default: cur = 4'hF;  // blank
            endcase
        end else begin // RUN
            case (difficulty)
                2'd1: begin
                    case (mux_idx)
                        2'd0: cur = d0;
                        2'd1: cur = d1;
                        default: cur = 4'hF;
                    endcase
                end
                2'd2: begin
                    case (mux_idx)
                        2'd0: cur = d0;
                        2'd1: cur = d1;
                        2'd2: cur = d2;
                        default: cur = 4'hF;
                    endcase
                end
                default: begin
                    case (mux_idx)
                        2'd0: cur = d0;
                        2'd1: cur = d1;
                        2'd2: cur = d2;
                        2'd3: cur = d3;
                    endcase
                end
            endcase
        end

        seg_reg = decode7(cur);
    end
end

//==============================================================================
// 11) Seven-Segment Decode Function (active-LOW)
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
// Debounce Shift-Register Module (de-bounce any 1-bit noisy input)
//==============================================================================
module debounce_sr #(
    parameter N = 16
)(
    input  wire clk,
    input  wire rst,
    input  wire noisy,
    output reg  clean
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
