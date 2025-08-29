`timescale 1ns / 1ps

// FND 4-Digit Output
module fnd_cntr (
    input clk, reset_p,
    input [15:0] fnd_value,
    input hex_bcd,
    output [6:0] seg_7,
    output dp,
    output [3:0] com
    );

    wire [15:0] bcd_value;
    // Convert Input Value to BCD Format
    bin_to_dec bcd (.bin(fnd_value[11:0]), .bcd(bcd_value));

    reg [16:0] clk_div;             // Clock Divide

    always @(posedge clk) begin
        clk_div = clk_div + 1;
    end

    // Change FND Output Every 2^16 * 10ns 
    anode_selector ring_com (.scan_count(clk_div[16:15]), .an_out(com));

    reg [3:0] digit_value;          // Single-Digit Output Value

    wire [15:0] out_value;          // Hex ↔ BCD Change Format
    assign out_value = hex_bcd ? fnd_value : bcd_value;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin          // Single-Digit Value Reset
            digit_value = 0;
        end
        else begin
            case (com)              // Select Digit Value
                4'b1110 : digit_value = out_value[3:0];
                4'b1101 : digit_value = out_value[7:4];
                4'b1011 : digit_value = out_value[11:8];
                4'b0111 : digit_value = out_value[15:12];
                default : digit_value = 0;
            endcase
        end
    end

    seg_decoder_a dec (.scan_count(clk_div[16:15]), .digit_in(digit_value),
                    .seg_out(seg_7), .dp_out(dp));
endmodule

// Button Debounce + Edge Detect, Switch also Available
module btn_cntr (
    input clk, reset_p,
    input btn,
    output btn_pedge, btn_ndege
    );

    wire debounced_btn;
    button_debounce btn_debounce (.clk(clk), .noise_btn(btn), .clean_btn(debounced_btn));
    edge_detector_pos btn_ed (.clk(clk), .reset_p(reset_p),
        .cp(debounced_btn), .p_edge(btn_pedge), .n_edge(btn_ndege));
endmodule

// Input Address or Data, 100㎑ I2C Communication When Start-bit High
module i2c_master (
    input clk, reset_p,
    input [6:0] addr,                           // Slave Address
    input [7:0] data,                           // Transmission Data
    input rd_wr, comm_start,                    // Read & Write Select-bit, Communication Start-bit
    output reg scl, sda,                        // Serial Clock , Serial Data
                                                // Originally SDA Input + Output, but Here only Output
    output [15:0] led                           // for Debugging
    );

    // Change State Using Shift
    localparam I2C_IDLE     = 7'b000_0001;      // Standby State
    localparam COMM_START   = 7'b000_0010;      // Communication Start
    localparam SEND_ADDR    = 7'b000_0100;      // Address Transmission
    localparam READ_ACK     = 7'b000_1000;      // Read ACK-bit, Assume Read
    localparam SEND_DATA    = 7'b001_0000;      // Data Transmission
    localparam SCL_STOP     = 7'b010_0000;      // Stop Generate Serial Clock
    localparam COMM_STOP    = 7'b100_0000;      // Communication Stop

    // Clock Divide 100, 10ns x 100 = 1us
    wire clk_usec_nedge, clk_usec_pedge; // Divide Clock 1us
    clock_div_100 us_clk (.clk(clk), .reset_p(reset_p),
        .nedge_div_100(clk_usec_nedge), .pedge_div_100(clk_usec_pedge));

    // Edge Detection of Command Signal
    wire comm_start_pedge, comm_start_nedge;
    edge_detector_pos comm_start_ed (.clk(clk), .reset_p(reset_p),
        .cp(comm_start), .p_edge(comm_start_pedge), .n_edge(comm_start_nedge));

    // Edge Detection of SCL Signal
    wire scl_pedge, scl_nedge;
    edge_detector_pos scl_ed (.clk(clk), .reset_p(reset_p),
        .cp(scl), .p_edge(scl_pedge), .n_edge(scl_nedge));

    reg [2:0] cnt_usec5;                        // 5us Count
    reg scl_e;                                  // SCL Enable
    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            cnt_usec5 = 0;                      // 5us Count Reset
            scl = 1;                            // SCL Reset, Pull-Up
        end
        else if (scl_e) begin                   // when SCL Enable
            if (clk_usec_nedge) begin           // when us Negative Edge
                if (cnt_usec5 >= 4) begin       // Every 5us 
                    cnt_usec5 = 0;              // 5us Count Reset
                    scl = ~scl;                 // Generate Serial Clock
                end
                else begin
                    cnt_usec5 = cnt_usec5 + 1;  // Count During Enable
                end
            end
        end
        else if (!scl_e) begin                  // SCL Disable
            cnt_usec5 = 0;                      // 5us Count Reset
            scl = 1;                            // SCL Reset, Pull-Up
        end
    end

    reg [6:0] state, next_state;                // Current & Next State
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) state = I2C_IDLE;          // Basic Standby
        else state = next_state;                // Change State in Negative Edge
    end

    wire [7:0] addr_rd_wr;
    assign addr_rd_wr = {addr, rd_wr};          // Address + RW Select-bit
    reg [2:0] cnt_bit;                          // Count for Transmit 1-bit at a Time
    reg stop_flag;                              // Check Data Transmission
    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            next_state = I2C_IDLE;              // Basic Standby
            scl_e = 0;                          // SCL Disable
            sda = 1;                            // SDA Reset, Pull-Up
            cnt_bit = 7;                        // bit Count Reset, from Upper bit
            stop_flag = 0;                      // Before Data Transmission
        end
        else begin
            case (state)
                I2C_IDLE   : begin              // Standby State
                    scl_e = 0;                  // SCL Disable
                    sda = 1;                    // SDA Reset, Pull-Up
                    if (comm_start_pedge) begin // Communication Start
                        next_state = COMM_START;    // Change State COMM_START
                    end
                end
                COMM_START : begin              // Communication Start
                    sda = 0;                    // SDA Start
                    scl_e = 1;                  // SCL Enable
                    next_state = SEND_ADDR;     // Change State SEND_ADDR
                end
                SEND_ADDR  : begin              // Address Transmission
                    if (scl_nedge) begin        // when SCL Negative Edge
                        sda = addr_rd_wr[cnt_bit];  // Enter Address into SDA
                    end
                    if (scl_pedge) begin        // when SCL Positive Edge
                        if (cnt_bit == 0) begin // Read to LSB
                            cnt_bit = 7;        // bit Count Reset
                            next_state = READ_ACK;  // Change State READ_ACK
                        end
                        else begin
                            cnt_bit = cnt_bit - 1;  // Read from MSB
                        end
                    end
                end
                READ_ACK   : begin              // Read ACK-bit, Assume Read
                    if (scl_nedge) begin        // when SCL Negative Edge
                        sda = 'bz; // Enter Inpedance Value into SDA, Disconnect
                    end
                    else if (scl_pedge) begin   // when SCL Positive Edge
                        if (stop_flag) begin    // Complete Data Transmission
                            stop_flag = 0;      // Data Transmission Flag Clear
                            next_state = SCL_STOP;  // Change State SCL_STOP
                        end
                        else begin              // Before Data Transmission
                            stop_flag = 1;      // Data Transmission Flag Set
                            next_state = SEND_DATA; // Change State SEND_DATA
                        end
                    end
                end
                SEND_DATA  : begin              // Data Transmission
                    if (scl_nedge) begin        // when SCL Negative Edge
                        sda = data[cnt_bit];    // Enter Data into SDA
                    end
                    if (scl_pedge) begin        // when SCL Positive Edge
                        if (cnt_bit == 0) begin // Read to LSB
                            cnt_bit = 7;        // bit Count Reset
                            next_state = READ_ACK;  // Change State READ_ACK
                        end
                        else begin
                            cnt_bit = cnt_bit - 1;  // Read from MSB
                        end
                    end
                end
                SCL_STOP   : begin              // Stop Generate Serial Clock
                    if (scl_nedge) sda = 0;     // SDA Low when SCL Negative Edge
                    if (scl_pedge) next_state = COMM_STOP;  // Change State COMM_STOP when SCL Positive Edge
                end
                COMM_STOP  : begin              // Communication Stop
                    if (cnt_usec5 >= 3) begin   // Wait 4us
                        scl_e = 0;              // SCL Disable
                        sda = 1;                // SDA Stop
                        next_state = I2C_IDLE;  // Change State I2C_IDLE
                    end
                end
                default    : begin
                    scl_e = 0;                  // SCL Disable
                    sda = 1;                    // SDA Reset, Pull-Up
                    next_state = I2C_IDLE;      // Basic Standby
                end
            endcase
        end
    end
endmodule

// Sending to LCD Using I2C Communication in Nibble(4-bit) Unit
module i2c_lcd_send_byte (
    input clk, reset_p,
    input [6:0] addr,                           // Slave Address
    input [7:0] send_buffer,                    // Transmission Data Buffer
    input send, rs,                             // Send Start-bit, Register Select
    output scl, sda,                            // Serial Clock , Serial Data
    output reg busy,                            // Communication Situation
    output [15:0] led                           // for Debugging
    );

    // Change State Using Shift
    localparam I2C_IDLE                 = 6'b00_0001;   // Standby State
    localparam SEND_HIGH_NIBBLE_DISABLE = 6'b00_0010;   // High Nibble(4-bit) Transmit in Enable Clear
    localparam SEND_HIGH_NIBBLE_ENABLE  = 6'b00_0100;   // Enable Set
    localparam SEND_LOW_NIBBLE_DISABLE  = 6'b00_1000;   // Low Nibble(4-bit) Transmit in Enable Clear
    localparam SEND_LOW_NIBBLE_ENABLE   = 6'b01_0000;   // Enable Set
    localparam SEND_DISABLE             = 6'b10_0000;   // Byte(8-bit) Transmission Complete

    reg [7:0] data;                             // 4-bit Unit Transmission, D7 ~ D4, BL, E, RW, RS
    reg comm_start;                             // Communication Start-bit

    // Clock Divide 100, 10ns x 100 = 1us
    wire clk_usec_nedge, clk_usec_pedge; // Divide Clock 1us
    clock_div_100 us_clk (.clk(clk), .reset_p(reset_p),
        .nedge_div_100(clk_usec_nedge), .pedge_div_100(clk_usec_pedge));

    // Edge Detection of Send Signal
    wire send_pedge, send_nedge;
    edge_detector_pos comm_start_ed (.clk(clk), .reset_p(reset_p),
        .cp(send), .p_edge(send_pedge), .n_edge(send_nedge));

    // us Unit Count
    reg [21:0] cnt_usec;                        // us Count
    reg cnt_usec_e;                             // us Count Enable
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) cnt_usec = 0;              // Count Clear
        else if (clk_usec_nedge && cnt_usec_e) begin    // Count Start when Enable & us Negative Edge
            cnt_usec = cnt_usec + 1;            // Count During Enable
        end
        else if (!cnt_usec_e) cnt_usec = 0;     // Count Clear when Disable
    end

    // Using Module, Address & Data Transmission, I2C Communication
    i2c_master master (clk, reset_p, addr, data, 1'b0, comm_start, scl, sda);

    reg [5:0] state, next_state;                // Current & Next State
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) state = I2C_IDLE;          // Basic Standby
        else state = next_state;                // Change State in Negative Edge
    end

    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            next_state = I2C_IDLE;              // Basic Standby
            comm_start = 0;                     // Communication Disable
            cnt_usec_e = 0;                     // us Count Disable, Clear
            data = 0;                           // Transmission Data Reset
            busy = 0;                           // Communication Available
        end
        else begin
            case (state)
                I2C_IDLE                 : begin    // Standby State
                    if (send_pedge) begin       // Send Start
                        next_state = SEND_HIGH_NIBBLE_DISABLE;  // Change State SEND_HIGH_NIBBLE_DISABLE
                        busy = 1;               // Communicating
                    end
                end
                SEND_HIGH_NIBBLE_DISABLE : begin    // High Nibble(4-bit) Transmit in Enable Clear
                    if (cnt_usec < 22'd200) begin   // About 200us Required to Complete Transmission
                        data = {send_buffer[7:4], 3'b100, rs};  // High Nibble Transmission
                        comm_start = 1;         // Communication Start
                        cnt_usec_e = 1;         // us Count Enable
                    end
                    else begin
                        cnt_usec_e = 0;         // us Count Disable, Clear
                        comm_start = 0;         // Communication Start-bit Clear
                        next_state = SEND_HIGH_NIBBLE_ENABLE;   // Change State SEND_HIGH_NIBBLE_ENABLE
                    end
                end
                SEND_HIGH_NIBBLE_ENABLE  : begin    // Enable Set
                    if (cnt_usec < 22'd200) begin   // About 200us Required to Complete Transmission
                        data = {send_buffer[7:4], 3'b110, rs};  // E Pin Enable
                        comm_start = 1;         // Communication Start
                        cnt_usec_e = 1;         // us Count Enable
                    end
                    else begin
                        cnt_usec_e = 0;         // us Count Disable, Clear
                        comm_start = 0;         // Communication Start-bit Clear
                        next_state = SEND_LOW_NIBBLE_DISABLE;   // Change State SEND_LOW_NIBBLE_DISABLE
                    end
                end
                SEND_LOW_NIBBLE_DISABLE  : begin    // Low Nibble(4-bit) Transmit in Enable Clear
                    if (cnt_usec < 22'd200) begin   // About 200us Required to Complete Transmission
                        data = {send_buffer[3:0], 3'b100, rs};  // Low Nibble Transmission
                        comm_start = 1;         // Communication Start
                        cnt_usec_e = 1;         // us Count Enable
                    end
                    else begin
                        cnt_usec_e = 0;         // us Count Disable, Clear
                        comm_start = 0;         // Communication Start-bit Clear
                        next_state = SEND_LOW_NIBBLE_ENABLE;    // Change State SEND_LOW_NIBBLE_ENABLE
                    end
                end
                SEND_LOW_NIBBLE_ENABLE   : begin    // Enable Set
                    if (cnt_usec < 22'd200) begin   // About 200us Required to Complete Transmission
                        data = {send_buffer[3:0], 3'b110, rs};  // E Pin Enable
                        comm_start = 1;         // Communication Start
                        cnt_usec_e = 1;         // us Count Enable
                    end
                    else begin
                        cnt_usec_e = 0;         // us Count Disable, Clear
                        comm_start = 0;         // Communication Start-bit Clear
                        next_state = SEND_DISABLE;  // Change State SEND_DISABLE
                    end
                end
                SEND_DISABLE             : begin    // Byte(8-bit) Transmission Complete
                    if (cnt_usec < 22'd200) begin   // About 200us Required to Complete Transmission
                        data = {send_buffer[7:4], 3'b100, rs};  // E Pin Disable
                        comm_start = 1;         // Communication Start
                        cnt_usec_e = 1;         // us Count Enable
                    end
                    else begin
                        cnt_usec_e = 0;         // us Count Disable, Clear
                        comm_start = 0;         // Communication Start-bit Clear
                        next_state = I2C_IDLE;  // Change State I2C_IDLE
                        busy = 0;               // Communication Available
                    end
                end
                default                  : begin
                    cnt_usec_e = 0;             // us Count Disable, Clear
                    comm_start = 0;             // Communication Start-bit Clear
                    next_state = I2C_IDLE;      // Basic Standby
                    busy = 0;                   // Communication Available
                end
            endcase
        end
    end
endmodule

// LCD 문자열 출력
module i2c_lcd_string (
    input clk, reset_p,
    input lcd_start,
    input lcd_row,
    input [8*16-1:0] lcd_txt,
    output scl, sda,
    output reg lcd_init_flag,
    output busy,
    output [15:0] led
    );

    integer cnt_sysclk;
    reg cnt_sysclk_e;
    // System Clock Counter
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) cnt_sysclk <= 0;
        else if (cnt_sysclk_e) cnt_sysclk <= cnt_sysclk + 1;
        else cnt_sysclk <= 0;
    end

    // Edge Detection of LCD Start-bit
    wire start_pedge, start_nedge;
    edge_detector_pos start_ed (clk, reset_p, lcd_start, start_pedge, start_nedge);

    reg [7:0] send_buffer;
    reg send, rs;
    // Using Module, Byte Unit Transmission
    i2c_lcd_send_byte send_byte (clk, reset_p, 7'h27, send_buffer,
        send, rs, scl, sda, busy);

    localparam LCD_IDLE    = 4'b0001;
    localparam LCD_INIT    = 4'b0010;
    localparam MOVE_CURSOR = 4'b0100;
    localparam SEND_STRING = 4'b1000;

    // Change State in Negative Edge
    reg [3:0] state, next_state;
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) state <= LCD_IDLE;
        else state <= next_state;
    end

    assign led[3:0] = state;
    assign led[4] = lcd_init_flag;

    reg [4:0] cnt_data;
    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            next_state <= LCD_IDLE;
            lcd_init_flag <= 0;
            send_buffer <= 0;
            send <= 0;
            rs <= 0;
            cnt_data <= 0;
        end
        else begin
            case (state)
                LCD_IDLE    : begin
                    if (lcd_init_flag) begin
                        if (start_pedge) next_state <= MOVE_CURSOR;
                    end
                    else begin
                        if (cnt_sysclk < 32'd80_000_00) begin   // Wait 80ms
                            cnt_sysclk_e <= 1;      // System Clock Count Start
                        end
                        else begin
                            next_state <= LCD_INIT; // Change State I2C_INIT
                            cnt_sysclk_e <= 0;      // System Clock Count Stop, Clear
                        end
                    end
                end
                LCD_INIT    : begin
                    if (busy) begin                 // Communicating
                        send <= 0;                  // Wait Transmission
                        if (cnt_data >= 6) begin
                            cnt_data <= 0;          // LCD Initialization 6-Step Complete
                            next_state <= LCD_IDLE; // Change State I2C_IDLE
                            lcd_init_flag <= 1;
                        end
                    end
                    else if (!send) begin
                        case (cnt_data)             // Step-by-Step Command Transmission
                            0 : send_buffer <= 8'h33;    // Function Set & Function Set
                            1 : send_buffer <= 8'h32;    // Function Set & Return Home
                            2 : send_buffer <= 8'h28;    // Function Set Data 4-bit, 2-Lines, 5×8 Dots
                            3 : send_buffer <= 8'h0C;    // Display On, Cursor Off, Blinking Cursor Off
                            4 : send_buffer <= 8'h01;    // Clear Display
                            5 : send_buffer <= 8'h06;    // Entry Mode Cursor Move Increment
                        endcase
                        send <= 1;                  // Request Transmission
                        cnt_data <= cnt_data + 1;   // Step Change
                    end
                end
                MOVE_CURSOR : begin
                    if (busy) begin                 // Communicating
                        send <= 0;                  // Wait Transmission
                        next_state <= SEND_STRING;
                    end
                    else if (!send) begin
                        rs <= 0;                    // Instruction Register Select
                        send_buffer <= {1'b1, lcd_row, 6'b00_0000}; // 저장 위치 설정
                        send <= 1;                  // Request Transmission
                    end
                end
                SEND_STRING : begin
                    if (busy) begin
                        send <= 0;
                        if (cnt_data >= 16) begin
                            cnt_data <= 0;
                            next_state <= LCD_IDLE;
                        end
                    end
                    else if (!send) begin
                        rs <= 1;                    // Data Register Select
                        send_buffer <= lcd_txt[8*(16-cnt_data)-1 -: 8]; // 왼쪽 글자부터 차례로 전송
                        send <= 1;
                        cnt_data <= cnt_data + 1;
                    end
                end
            endcase
        end
    end
endmodule

// PWM Frequency Signal Output According to Duty Ratio
module pwm_Nstep (
    input clk, reset_p,
    input [31:0] duty,                          // PWM Duty Rate, CCR Capture Compare Register
    output reg pwm                              // PWM Duty Applied Pulse
    );

    parameter sys_clk_freq  = 100_000_000;      // System Clock Frequency
    parameter pwm_freq      = 10_000;           // PWM Frequency
    parameter duty_step_N   = 200;              // Duty Step, ARR Auto Reload Register
    parameter temp = sys_clk_freq / pwm_freq / duty_step_N / 2; // Half of Cycle

    integer cnt_sysclk;                         // System Clock Count
    reg pwm_freqXn;                             // PWM Frequency
    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            cnt_sysclk <= 0;                     // System Clock Count Reset
            pwm_freqXn <= 0;                     // PWM Frequency Reset
        end
        else begin
            if (cnt_sysclk >= temp - 1) begin   // when Half of Cycle
                cnt_sysclk <= 0;                 // System Clock Count Reset
                pwm_freqXn <= ~pwm_freqXn;       // PWM Frequency Generation
            end
            else cnt_sysclk <= cnt_sysclk + 1;   // Count System Clock
        end
    end

    // Edge Detection of PWM Frequency
    wire pwm_freqXn_nedge;                      // PWM Frequency Negative Edge
    edge_detector_pos pwm_freqXn_ed (.clk(clk), .reset_p(reset_p),
        .cp(pwm_freqXn), .n_edge(pwm_freqXn_nedge));

    integer cnt_duty;                           // PWM Frequency Count
    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            cnt_duty <= 0;                       // PWM Frequency Count Reset
            pwm <= 0;                            // Pulse Reset
        end
        else if (pwm_freqXn_nedge) begin
            if (cnt_duty >= duty_step_N) cnt_duty = 0;  // Count to End of Duty
            else cnt_duty <= cnt_duty + 1;       // Count PWM Frequency

            if (cnt_duty < duty) pwm <= 1;       // Pulse High Until Duty
            else pwm <= 0;                       // Pulse Low After Duty
        end
    end
endmodule

//
module stepper_cntr (
    input clk, reset_p,       
    input start_stepper,            // 구동 신호, high일때 구동
    input stepper_dir,              // 회전 방향 1 시계 방향, 0 반시계 방향
    output reg [3:0] stepper,       // Motor Driver 출력 신호, IN1 ~ 4
    output [15:0] led
    );

    reg [23:0] cnt_sysclk = 0;      // System Clock Counter
    reg [2:0] step_index = 0;       // 배열 Index 나타냄  0에서 7까지 반복하며 모터를 회전
    reg [3:0] half_step [7:0];      //8개의 하프 스텝 시퀀스를 배열로 정의

    initial begin
        half_step[0] = 4'b1000;
        half_step[1] = 4'b1100;
        half_step[2] = 4'b0100;
        half_step[3] = 4'b0110;
        half_step[4] = 4'b0010;
        half_step[5] = 4'b0011;
        half_step[6] = 4'b0001;
        half_step[7] = 4'b1001;
    end

    assign led[0] = start_stepper;
    assign led[1] = stepper_dir;
    assign led[5:2] = stepper;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p)begin
            cnt_sysclk <= 0;
            step_index <= 0;
            stepper <= 4'b0000;
        end else begin
            if (start_stepper) begin
                if (cnt_sysclk >= 200_000) begin        // 2ms마다 Step 변경
                    cnt_sysclk <= 0;
                    stepper <= half_step[step_index];
                    if (stepper_dir) begin // 시계 방향
                        if (step_index >= 7) step_index <= 0;
                        else step_index = step_index + 1;
                    end
                    else begin // 반시계 방향
                        if (step_index <= 0) step_index <= 7;
                        else step_index <= step_index - 1;
                    end
                end
                else begin
                    cnt_sysclk <= cnt_sysclk + 1;
                end
            end
            else begin // 정지 동작 
                cnt_sysclk <= 0;
                step_index <= 0;
                stepper <= 4'b0000;
            end
        end
    end
endmodule

// Humidity & Temperature Sensor data read & write, FSM
module dht11_cntr (
    input clk, reset_p,
    inout dht11_data,                           // Input + Output, reg Declaration Not Possible
    output reg [7:0] humidity, temperature,     // Output Measurement
    output [15:0] led                           // for Debugging
    );

    // Change State Using Shift
    localparam S_IDLE       = 6'b00_0001;       // Standby State
    localparam S_LOW_18MS   = 6'b00_0010;       // MCU Sends Out Start Signal
    localparam S_HIGH_20US  = 6'b00_0100;       // MCU Pull Up & Wait for Sensor Response
    localparam S_LOW_80US   = 6'b00_1000;       // DHT Sends Out Response Signal
    localparam S_HIGH_80US  = 6'b01_0000;       // DHT Pull Up & Get Ready Data
    localparam S_READ_DATA  = 6'b10_0000;       // DHT Data Read

    localparam S_WAIT_PEDGE = 2'b01;            // Start to Transmit 1-bit Data
    localparam S_WAIT_NEDGE = 2'b10;            // Voltage Length Measurement


    // Clock Divide 100, 10ns x 100 = 1us
    wire clk_usec_nedge;                        // Divide Clock 1us
    clock_div_100 us_clk (.clk(clk), .reset_p(reset_p), .nedge_div_100(clk_usec_nedge));

    // us Unit Count
    reg [21:0] cnt_usec;                        // us Count
    reg cnt_usec_e;                             // us Count Enable
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) cnt_usec = 0;              // Count Clear
        else if (clk_usec_nedge && cnt_usec_e) begin    // Count Start when Enable & us Negative Edge
            cnt_usec = cnt_usec + 1;            // Count During Enable
        end
        else if (!cnt_usec_e) cnt_usec = 0;     // Count Clear when Disable
    end

    // Edge Detection of DHT Signal
    wire dht_nedge, dht_pedge;
    edge_detector_pos btn_ed (.clk(clk), .reset_p(reset_p),
        .cp(dht11_data), .p_edge(dht_pedge), .n_edge(dht_nedge));

    // Input Cannot be Declared as reg, Use Buffer
    reg dht11_buffer;                           // Buffer
    reg dht11_data_out_e;                       // Write Mode Enable Output, Disable Input
    assign dht11_data = dht11_data_out_e ? dht11_buffer : 'bz; // Output dout, Input Impedance Value

    reg [5:0] state, next_state;                // Current & Next State
    assign led [5:0] = state;                   // State LED Output
    reg [1:0] read_state;                       // Data Read State
    // Change State in Negative Edge
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) state = S_IDLE;            // Basic Standby State
        else state = next_state;                // Change State in Negative Edge
    end

    reg [39:0] temp_data;                       // DHT Output Data 40-bit
    reg [5:0] data_cnt;                         // Counting to 40
    assign led [11:6] = data_cnt;
    // Set Next State in Positive Edge
    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            next_state = S_IDLE;                // Basic Stanby State
            temp_data = 0;                      // DHT Data Reset
            data_cnt = 0;                       // DHT Data Count Reset
            dht11_data_out_e = 0;               // DHT Write Disable, Input
            read_state = S_WAIT_PEDGE;          // Data Read Pull-Up High
        end
        else begin
            case (state)
                S_IDLE      : begin             // Standby State
                    if (cnt_usec < 22'd3_000_000) begin     // Real 3_000_000, Test 3_000
                        cnt_usec_e = 1;         // us Count Enable
                        dht11_data_out_e = 0;   // DHT Input Mode
                    end
                    else begin
                        cnt_usec_e = 0;         // Count Clear
                        next_state = S_LOW_18MS;// Change State S_LOW_18MS
                    end
                end
                S_LOW_18MS  : begin             // MCU Sends Out Start Signal
                    if (cnt_usec < 22'd18_000) begin    // Real 18_000, Test 18
                        cnt_usec_e = 1;         // us Count Enable
                        dht11_data_out_e = 1;   // DHT Output Mode
                        dht11_buffer = 0;       // Buffer Reset
                    end
                    else begin
                        cnt_usec_e = 0;         // us Count Disable, Clear
                        next_state = S_HIGH_20US;   // Change State S_HIGH_20US
                        dht11_data_out_e = 0;   // DHT Input Mode
                    end
                end
                // It is Supposed to Respond after 20us, but in Reality, Response Occurs before that.
                // Remove 20us Waiting Part
                S_HIGH_20US : begin             // MCU Pull Up & Wait for Sensor Response
                    cnt_usec_e = 1;             // Count for Checking Response Time
                    if (cnt_usec > 22'd100_000) begin   // No Response 100ms, Comunication Error, Etc..
                        cnt_usec_e = 0;         // us Count Disable, Clear
                        next_state = S_IDLE;    // Change State S_IDLE
                    end
                    if (dht_nedge) begin        // DHT Response, No Count
                        cnt_usec_e = 0;         // us Count Disable, Clear
                        next_state = S_LOW_80US;    // Change State S_LOW_80US
                    end
                end
                S_LOW_80US  : begin             // DHT Sends Out Response Signal
                    cnt_usec_e = 1;             // Count for Checking Response Time
                    if (cnt_usec > 22'd100_000) begin   // No Response 100ms, Comunication Error, Etc..
                        cnt_usec_e = 0;         // us Count Disable, Clear
                        next_state = S_IDLE;    // Change State S_IDLE
                    end
                    if (dht_pedge) begin
                        cnt_usec_e = 0;         // us Count Disable, Clear
                        next_state = S_HIGH_80US;    // No Need to Count, Change State
                    end
                end
                S_HIGH_80US : begin             // DHT Pull Up & Get Ready Data
                    cnt_usec_e = 1;             // Count for Checking Response Time
                    if (cnt_usec > 22'd100_000) begin   // No Response 100ms, Comunication Error, Etc..
                        cnt_usec_e = 0;         // us Count Disable, Clear
                        next_state = S_IDLE;    // Change State S_IDLE
                    end
                    if (dht_nedge) begin
                        cnt_usec_e = 0;         // us Count Disable, Clear
                        next_state = S_READ_DATA;    // No Need to Count, Change State
                    end
                end
                S_READ_DATA : begin             // DHT Data Read
                    case (read_state)
                        S_WAIT_PEDGE : begin    // Current Low, Wait High
                            // High Signal Generation
                            if (dht_pedge) read_state = S_WAIT_NEDGE;   // Change State
                            cnt_usec_e = 0;     // us Count Disable, Clear
                        end
                        S_WAIT_NEDGE : begin    // Current High, Wait Low
                            // Low Signal Generation
                            if (dht_nedge) begin
                                read_state = S_WAIT_PEDGE;              // Change State
                                data_cnt = data_cnt + 1;                // Counting to 40
                            if (cnt_usec < 50) begin        // Distinguish by Signal Length
                                    temp_data = {temp_data[38:0], 1'b0};    // Shift Left & Enter Value in LSM
                                end
                                else begin
                                    temp_data = {temp_data[38:0], 1'b1};
                                end
                            end
                            else begin
                                cnt_usec_e = 1;     // us Count Enable
                                if (cnt_usec > 22'd100_000) begin   // No Response 100ms, Comunication Error, Etc..
                                    cnt_usec_e = 0;         // us Count Disable, Clear
                                    next_state = S_IDLE;
                                    data_cnt = 0;           // DHT Data Count Reset
                                    read_state = S_WAIT_PEDGE;
                                end
                            end
                        end
                    endcase

                    if (data_cnt >= 40) begin   // 40-bit Read Completed
                        next_state = S_IDLE;    // Basic Stanby State
                        data_cnt = 0;           // DHT Data Count Reset
    // DHT11 Data = Humidity Integral 8-bit + Decimal 8-bit + Temperature Integral 8-bit + Decimal 8-bit + Check Sum 8-bit
                        // Compare Upper 32-bits of Sum with Checksum
                        if ((temp_data[39:32] + temp_data[31:24] + temp_data[23:16] + temp_data[15:8]) == temp_data[7:0]) begin
                            humidity = temp_data[39:32];        // Use only Integer Values
                            temperature = temp_data[23:16];
                        end
                    end
                end
                default : next_state = S_IDLE;  // Basic Stanby State
            endcase
        end
    end
endmodule

module servo_cntr (
    input clk, reset_p,
    input mode_servo, start_servo,
    output servo,
    output [15:0] led
    );

    integer cnt_sysclk;
    always @(posedge clk) cnt_sysclk = cnt_sysclk + 1;
    
    wire cnt_sysclk_pedge;
    edge_detector_pos sysclk_edge (clk, reset_p, cnt_sysclk[21], cnt_sysclk_pedge);

    reg [7:0] step;
    // assign led[0] = mode_servo;
    // assign led[1] = start_servo;
    // assign led[9:2] = step;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
            step <= 20;
        end
        else begin
            if (cnt_sysclk_pedge) begin
                if (start_servo) begin
                    if (step <= 41) step <= 40;
                    else step <= step - 1;
                end
                else begin
                    if (step >= 149) step <= 150;
                    else step <= step + 1;
                end
            end
        end
        // else if (!mode_servo) begin
        //     if (btn_pedge[1]) begin
        //         step <= step - 1;
        //     end
        //     else if (btn_pedge[2]) begin
        //         step <= step + 1;
        //     end
        // end
    end

    pwm_Nstep #(.pwm_freq(50), .duty_step_N(1440))
        pwm_servo (clk, reset_p, step, servo);
endmodule
