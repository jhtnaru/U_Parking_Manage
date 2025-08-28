`timescale 1ns / 1ps

// Positive edge Edge Detector
module edge_detector_pos (
    input clk, reset_p, cp,     // Clock, Reset, Input Signal 감지
    output p_edge, n_edge       // Rising, Falling 감지 출력
    );

    reg ff_cur, ff_old;         // cp 현재값과 이전값 저장할 Flip-Flop

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin          // Reset Flip-Flop 초기화
            ff_cur <= 1'b0;
            ff_old <= 1'b0;
        end
        else begin                  // 이전값 저장하고 현재값 갱신
            ff_old <= ff_cur;
            ff_cur <= cp;
        end
    end

    // Rising 감지, 이전 0, 현재 1 이면 p_edge = 1
    assign p_edge = ({ff_cur, ff_old} == 2'b10) ? 1'b1 : 1'b0;
    // Falling 감지, 이전 1, 현재 0 이면 n_edge = 1
    assign n_edge = ({ff_cur, ff_old} == 2'b01) ? 1'b1 : 1'b0;
endmodule

// 입력된 숫자를 seg 출력 변수로 수정
module seg_decoder_a (
    input [1:0] scan_count,
    input [3:0] digit_in,
    output reg [6:0] seg_out,
    output reg dp_out
    );

    always @(*) begin
        case (digit_in)     //  gfedcba
            4'd0 : seg_out = 7'b1000000;
            4'd1 : seg_out = 7'b1111001;
            4'd2 : seg_out = 7'b0100100;
            4'd3 : seg_out = 7'b0110000;
            4'd4 : seg_out = 7'b0011001;
            4'd5 : seg_out = 7'b0010010;
            4'd6 : seg_out = 7'b0000010;
            4'd7 : seg_out = 7'b1111000;
            4'd8 : seg_out = 7'b0000000;
            4'd9 : seg_out = 7'b0011000;
            4'hA : seg_out = 7'b0001000;
            4'hb : seg_out = 7'b0000011;
            4'hC : seg_out = 7'b1000110;
            4'hd : seg_out = 7'b0100001;
            4'hE : seg_out = 7'b0000110;
            4'hF : seg_out = 7'b0001110;
            default : seg_out = 7'b1111111;
        endcase
        case (scan_count)
            2'd0 : dp_out = 0;
            2'd1 : dp_out = 1;
            2'd2 : dp_out = 0;
            2'd3 : dp_out = 1;
            default : dp_out = 1;
        endcase
    end
endmodule

// 입력된 자리를 an 출력 변수로 수정
module anode_selector (
    input [1:0] scan_count,
    output reg [3:0] an_out
    );

    always @(*) begin
        case (scan_count)
            2'd0 : an_out = 4'b1110;
            2'd1 : an_out = 4'b1101;
            2'd2 : an_out = 4'b1011;
            2'd3 : an_out = 4'b0111;
            default : an_out = 4'b1111;
        endcase
    end
endmodule

// Binary → BCD 방식 → Decimal
module bin_to_dec (
    input [11:0] bin,       // 12-bit Binary Input
    output reg [15:0] bcd   // 16-bit BCD Output (4-bit X 4자리)
    );

    integer i;

    always @(bin) begin
        bcd = 0;            // Initial Value
        for (i = 0; i < 12; i = i + 1) begin
            // BCD Algorithm
            // 1st 단위 bit 자리별로 5 이상 → + 3
            if (bcd[3:0] >= 5)   bcd[3:0] = bcd[3:0] + 3;
            if (bcd[7:4] >= 5)   bcd[7:4] = bcd[7:4] + 3;
            if (bcd[11:8] >= 5)  bcd[11:8] = bcd[11:8] + 3;
            if (bcd[15:12] >= 5) bcd[15:12] = bcd[15:12] + 3;

            // 2nd 1-bit Left Shift + 새 bit Input
            bcd = {bcd[14:0], bin[11 - i]};
        end
    end
endmodule

// Button Debounce 
module button_debounce (
    input clk,
    input noise_btn,            // Raw Input Button
    output reg clean_btn        // Modify Button
    );

    reg [19:0] cnt = 1;
    reg btn_sync_0, btn_sync_1; // 2 Step Debounce
    reg btn_state;              // Button Before State

    always @(posedge clk) begin
        btn_sync_0 <= noise_btn;
        btn_sync_1 <= btn_sync_0;
    end

    always @(posedge clk) begin
        if (btn_sync_1 == btn_state) begin
            cnt <= 1;           // Input == Before State, Stable State → Counter Reset
        end
        else begin
            cnt <= cnt + 1;     // Input != Before State, Count Increase
            if (cnt >= 1_000_000) begin  // Maintain a Specific Time for Debounce, 10ns * 1,000,000 = 10ms
                btn_state <= btn_sync_1;
                clean_btn <= btn_sync_1;
                cnt <= 1;
            end
        end
    end
endmodule
