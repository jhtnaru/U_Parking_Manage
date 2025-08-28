`timescale 1ns / 1ps

// Clock Frequency Divider, 100 Divide
module clock_div_100 (
    input clk, reset_p,
    output reg clk_div_100,
    output nedge_div_100, pedge_div_100
    );

    reg [5:0] cnt_sysclk;

    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            cnt_sysclk = 0;                     // Clock Count Reset
            clk_div_100 = 0;                    // Divided Clock Reset
        end
        else begin
            if (cnt_sysclk >= 49) begin         // When Unit Clock Reachs 50
                cnt_sysclk = 0;                 // Clock Count Reset
                clk_div_100 = ~clk_div_100;     // Divided Clock Generation
            end
            else begin
                cnt_sysclk = cnt_sysclk + 1;    // Count Clock
            end
        end
    end

    // Divided Clock Edge Detection
    edge_detector_pos clk_ed (.clk(clk), .reset_p(reset_p),
        .cp(clk_div_100), .p_edge(pedge_div_100), .n_edge(nedge_div_100));
endmodule
