`timescale 1ns / 1ps

// Underground Parking Management, Disaster Response
module uparking_top (
    input clk, reset_p,
    input water, flame, ball, photo, sound, gas, motion,    // Sensor 측정값 수신
    inout dht11_data,
    output reg buzzer, fan, led_r,                  // LED, Buzzer 출력, Motor 구동
    output servo,
    output reg [2:0] led_g,
    output [3:0] stepper,
    output sda, scl,                                // LCD I2C 통신
    output [6:0] seg_7,
    output dp,
    output [3:0] com,
    output [15:0] led                               // for Debugging
    );

    localparam NORMAL       = 8'b0000_0001;         // 평상
    localparam D_FLOOD      = 8'b0000_0010;         // 침수
    localparam D_FIRE       = 8'b0000_0100;         // 화재
    localparam D_QUAKE      = 8'b0000_1000;         // 지진
    localparam D_HIGH_TEMP  = 8'b0001_0000;         // 폭염
    localparam D_BLACKOUT   = 8'b0010_0000;         // 암전
    localparam D_LOUD_NOISE = 8'b0100_0000;         // 굉음
    localparam D_GAS        = 8'b1000_0000;         // 가스 누출

    localparam PEACE   = 3'b001;                    // 평상
    localparam WARNING = 3'b010;                    // 경고
    localparam DANGER  = 3'b100;                    // 위험

    // 온습도 측정 Module Instance
    wire [7:0] humidity, temperature;               // 온습도 측정값 저장
    dht11_cntr dht (clk, reset_p, dht11_data, humidity, temperature);

    // 온습도 측정값 BCD Format 변환 Module Instance
    wire [7:0] humi_bcd, temp_bcd;                  // 온습도 BCD 저장
    bin_to_dec bcd_humi (.bin(humidity), .bcd(humi_bcd));
    bin_to_dec bcd_tmpr (.bin(temperature), .bcd(temp_bcd));

    // Stepper Motor, Servo Motor 구동 Module Instance
    reg start_stepper, start_servo, mode_servo;     // Stepper, Servo Start-bit
    stepper_cntr cntr_st (clk, reset_p, start_stepper, 1'b0, stepper);
    servo_cntr cntr_sr (clk, reset_p, mode_servo, start_servo, servo);

    // LCD 문자열 출력 Module Instance
    reg lcd_start, lcd_row;                         // LCD Start-bit, 출력 위치(열 0~1)
    reg [8*16-1:0] lcd_txt;                         // 출력 문자열 전달 변수, 16문자
    reg [8*16-1:0] lcd_line1, lcd_line2;            // 1, 2열 출력 문자열
    wire lcd_init_flag, busy;                       // LCD 초기화g, 통신중 Flag
    i2c_lcd_string string_lcd (clk, reset_p, lcd_start, lcd_row, lcd_txt,
        scl, sda, lcd_init_flag, busy);

    wire ball_pedge, ball_nedge;                    // Ball Switch 측정값 Edge 감지
    edge_detector_pos ball_edge (clk, reset_p, ball, ball_pedge, ball_nedge);
    wire sound_pedge, sound_nedge;                  // Sound Sensor 측정값 Edge 감지
    btn_cntr sound_edge (clk, reset_p, sound, sound_pedge, sound_nedge);
    wire clk_pedge, clk_nedge;                      // Clock Count Edge 감지
    edge_detector_pos clk_edge (clk, reset_p, cnt_clk_led[24], clk_pedge, clk_nedge);

    // Red, Green LED 출력용 PWM 설정
    wire [1:0] led_r_pwm;                           // 0 Warning, 1 Danger
    wire [3:0] led_g_pwm;                           // 0 Warning, 1~3 Danger
    pwm_Nstep #(.duty_step_N(128)) pwm_led_r0 (clk, reset_p, cnt_clk_led[27:21], led_r_pwm[0]);
    pwm_Nstep #(.duty_step_N(128)) pwm_led_r1 (clk, reset_p, cnt_clk_led[25:19], led_r_pwm[1]);
    pwm_Nstep #(.duty_step_N(128)) pwm_led_g0 (clk, reset_p, cnt_clk_led[27:23], led_g_pwm[0]);
    pwm_Nstep #(.duty_step_N(128)) pwm_led_g1 (clk, reset_p, cnt_clk_led[26:20], led_g_pwm[1]);
    pwm_Nstep #(.duty_step_N(128)) pwm_led_g2 (clk, reset_p, cnt_clk_led[26:22], led_g_pwm[2]);
    pwm_Nstep #(.duty_step_N(128)) pwm_led_g3 (clk, reset_p, cnt_clk_led[26:24], led_g_pwm[3]);

    // State 변경부
    reg [7:0] state, next_state;                    // 현재와 다음 상황
    reg [2:0] d_grade;                              // 상황 등급, 평상 → 경고 → 위험
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) state = NORMAL;
        else state = next_state;
    end

    // LCD 출력 for Debugging
    assign led [7:0] = state;
    assign led [10:8] = d_grade;
    assign led [11] = lcd_init_flag;
    assign led [12] = lcd_toggle;

    // System Clock Counter
    reg [31:0] cnt_sysclk, cnt_clk_led;             // 상황 시간 계산용, LED PWM 설정용
    reg cnt_sysclk_e;                               // System Clock Counter Enable 변수
    reg lcd_toggle;
    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            cnt_sysclk <= 0;
            cnt_clk_led <= 0;
        end
        else begin
            cnt_clk_led <= cnt_clk_led + 1;
            lcd_toggle <= cnt_clk_led[25];
            if (cnt_sysclk_e) cnt_sysclk <= cnt_sysclk + 1; // Enable 상태에서만 Count
            else cnt_sysclk <= 0;
        end
    end

    // LCD 주기적 출력 갱신
    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            lcd_start <= 0;
            lcd_row <= 0;
            lcd_txt <= 0;
        end
        else begin
            if (lcd_init_flag) begin                // 초기화 완료 상황에서만 출력
                if (lcd_start) begin
                    lcd_start <= 0;                 // LCD Start-bit Reset
                    lcd_row <= ~lcd_row;            // 출력 줄 변경
                end
                else if (clk_pedge) begin
                    if (lcd_row) lcd_txt <= lcd_line1;
                    else lcd_txt <= lcd_line2;
                    lcd_start <= 1;                 // LCD Start-bit 전송
                end
            end
        end
    end

    // Sensor 측정값을 이용해서 상황 및 등급 판단
    reg [3:0] cnt_water, cnt_flame, cnt_ball;       // Sensor 측정 상황 시간 계산용 Count
    reg [3:0] cnt_temp, cnt_photo, cnt_gas, cnt_motion;
    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            next_state <= NORMAL;
            d_grade <= PEACE;
            cnt_water <= 0;
            cnt_flame <= 0;
            cnt_ball <= 0;
            cnt_temp <= 0;
            cnt_photo <= 0;
            cnt_gas <= 0;
            cnt_motion <= 0;
        end
        else begin
            if (water) begin
                if (cnt_sysclk <= 50_000_000) begin     // 0.5초 = 500ms 단위 시간 계산
                    cnt_sysclk_e <= 1;
                end
                else begin
                    cnt_sysclk_e <= 0;
                    // 평상 상황에서 Water Sensor 2초 이상 측정시
                    if (cnt_water >= 4 && state == NORMAL && d_grade == PEACE) begin
                        next_state <= D_FLOOD;          // 침수 상황 변경
                        d_grade <= WARNING;             // 경고 등급 변경
                        cnt_water <= 0;
                    end
                    // 경고 상황에서 Water Sensor 3초 이상 측정시
                    else if (cnt_water >= 6 && state == D_FLOOD && d_grade == WARNING) begin
                        d_grade <= DANGER;              // 위험 등급 변경
                        cnt_water <= 0;
                    end
                    else begin
                        cnt_water <= cnt_water + 1;
                    end
                end
            end
            else if (!flame) begin
                if (cnt_sysclk <= 50_000_000) begin     // 0.5초 = 500ms 단위 시간 계산
                    cnt_sysclk_e <= 1;
                end
                else begin
                    cnt_sysclk_e <= 0;
                    // 평상 상황에서 Flame Sensor 2초 이상 측정시
                    if (cnt_flame >= 4 && state == NORMAL && d_grade == PEACE) begin
                        next_state <= D_FIRE;           // 화재 상황 변경
                        d_grade <= WARNING;             // 경고 등급 변경
                        cnt_flame <= 0;
                    end
                    // 경고 상황에서 Flame Sensor 3초 이상 측정시
                    else if (cnt_flame >= 6 && state == D_FIRE && d_grade == WARNING) begin
                        d_grade <= DANGER;              // 위험 등급 변경
                        cnt_flame <= 0;
                    end
                    else begin
                        cnt_flame <= cnt_flame + 1;
                    end
                end
            end
            else if (ball_pedge) begin
                // 평상 상황에서 Ball Switch 흔들림 Count 4 이상 측정시
                if (cnt_ball >= 4 && state == NORMAL && d_grade == PEACE) begin
                    next_state <= D_QUAKE;              // 지진 상황 변경
                    d_grade <= WARNING;                 // 경고 등급 변경
                end
                // 경고 상황에서 Ball Switch 흔들림 Count 10 이상 측정시
                else if (cnt_ball >= 10 && state == D_QUAKE && d_grade == WARNING) begin
                    d_grade <= DANGER;                  // 위험 등급 변경
                    cnt_ball <= 0;
                end
                else begin
                    cnt_ball <= cnt_ball + 1;
                end
            end
            else if (temperature >= 8'd30) begin        // 온도가 30도 이상일때
                if (cnt_sysclk <= 50_000_000) begin     // 0.5초 = 500ms 단위 시간 계산
                    cnt_sysclk_e <= 1;
                end
                else begin
                    cnt_sysclk_e <= 0;
                    // 평상 상황에서 온습도 Sensor 2초 이상 + 30도 이상 측정시
                    if (cnt_temp >= 4 && state == NORMAL && d_grade == PEACE) begin
                        next_state <= D_HIGH_TEMP;      // 폭염 상황 변경
                        d_grade <= WARNING;             // 경고 등급 변경
                        cnt_temp <= 0;
                    end
                    // 경고 상황에서 온습도 Sensor 3초 이상 + 32도 이상 측정시
                    else if (temperature >= 8'd32 && cnt_temp >= 6
                            && state == D_HIGH_TEMP && d_grade == WARNING) begin
                        d_grade <= DANGER;              // 위험 등급 변경
                        cnt_temp <= 0;
                    end
                    else begin
                        cnt_temp <= cnt_temp + 1;
                    end
                end
            end
            else if (photo) begin
                if (cnt_sysclk <= 50_000_000) begin     // 0.5초 = 500ms 단위 시간 계산
                    cnt_sysclk_e <= 1;
                end
                else begin
                    cnt_sysclk_e <= 0;
                    // 평상 상황에서 Photo Sensor 2초 이상 측정시
                    if (cnt_photo >= 4 && state == NORMAL && d_grade == PEACE) begin
                        next_state <= D_BLACKOUT;       // 암전 상황 변경
                        d_grade <= WARNING;             // 경고 등급 변경
                        cnt_photo <= 0;
                    end
                    // 경고 상황에서 Photo Sensor 3초 이상 측정시
                    else if (cnt_photo >= 6 && state == D_BLACKOUT && d_grade == WARNING) begin
                        d_grade <= DANGER;              // 위험 등급 변경
                        cnt_photo <= 0;
                    end
                    else begin
                        cnt_photo <= cnt_photo + 1;
                    end
                end
            end
            else if (sound_pedge) begin
                // 평상 상황에서 Sound 1회 측정시
                if (state == NORMAL && d_grade == PEACE) begin
                    next_state <= D_LOUD_NOISE;         // 굉음 상황 변경
                    d_grade <= WARNING;                 // 경고 등급 변경
                end
                // 경고 상황에서 Sound 1회 측정시
                else if (state == D_LOUD_NOISE && d_grade == WARNING) begin
                    d_grade <= DANGER;                  // 위험 등급 변경
                end
            end
            else if (!gas) begin
                if (cnt_sysclk <= 50_000_000) begin     // 0.5초 = 500ms 단위 시간 계산
                    cnt_sysclk_e <= 1;
                end
                else begin
                    cnt_sysclk_e <= 0;
                    // 평상 상황에서 Gas Sensor 2초 이상 측정시
                    if (cnt_gas >= 4 && state == NORMAL && d_grade == PEACE) begin
                        next_state <= D_GAS;            // 가스 누출 상황 변경
                        d_grade <= WARNING;             // 경고 등급 변경
                        cnt_gas <= 0;
                    end
                    // 경고 상황에서 Gas Sensor 3초 이상 측정시
                    else if (cnt_gas >= 6 && state == D_GAS && d_grade == WARNING) begin
                        d_grade <= DANGER;              // 위험 등급 변경
                        cnt_gas <= 0;
                    end
                    else begin
                        cnt_gas <= cnt_gas + 1;
                    end
                end
            end
        end
    end

    // 상황과 등급별로 출력 및 구동
    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            buzzer <= 0;
            start_servo <= 1;
            start_stepper <= 0;
            fan <= 0;
            led_r <= 0;
            led_g <= 0;
            lcd_line1 <= 0;
            lcd_line2 <= 0;
        end
        else begin
            case (d_grade)                              // 등급별 LCD 2열, Buzzer, LED 출력
                WARNING : begin
                    lcd_line2 <= " Be Careful     ";
                    led_r <= led_r_pwm[0];
                    led_g <= {3{led_g_pwm[0]}};
                end
                DANGER  : begin
                    led_r <= led_r_pwm[1];
                    led_g <= led_g_pwm[3:1];
                    buzzer <= cnt_clk_led[25];
                    if (motion) lcd_line2 <= " Motion Detect  ";    // 동작시 감지시 LCD 출력
                    else lcd_line2 <= " Run Away       ";
                end
                default : begin
                    led_r <= 0;
                    led_g <= 0;
                    lcd_line2[127:88] <= "TEMP ";       // 측정 온습도 LCD 출력
                    lcd_line2[87:80] <= "0" + temp_bcd[7:4];
                    lcd_line2[79:72] <= "0" + temp_bcd[3:0];
                    lcd_line2[71:16] <= ", HUMI ";
                    lcd_line2[15:8] <= "0" + humi_bcd[7:4];
                    lcd_line2[7:0] <= "0" + humi_bcd[3:0];
                end
            endcase
            case (state)                                // 상황별 LCD 1열 출력 및 구동
                D_FLOOD      : begin
                    if (d_grade == WARNING) begin
                        lcd_line1 <= " FLOOD Warning  ";
                    end
                    else if (d_grade == DANGER) begin
                        lcd_line1 <= " FLOOD Danger   ";
                        start_servo <= 0;               // 차수판 작동
                        start_stepper <= 1;             // 자동사다리 작동
                    end
                end
                D_FIRE       : begin
                    if (d_grade == WARNING) begin
                        lcd_line1 <= " FIRE Warning   ";
                    end
                    else if (d_grade == DANGER) begin
                        lcd_line1 <= " FIRE Danger    ";
                        start_stepper <= 1;             // 자동사다리 작동
                        fan <= 1;                       // 제연 및 환기 작동
                    end
                end
                D_QUAKE      : begin
                    if (d_grade == WARNING) begin
                        lcd_line1 <= " QUAKE Warning  ";
                    end
                    else if (d_grade == DANGER) begin
                        lcd_line1 <= " QUAKE Danger   ";
                    end
                end
                D_HIGH_TEMP  : begin
                    if (d_grade == WARNING) begin
                        lcd_line1 <= "HIGHHEAT Warning";
                    end
                    else if (d_grade == DANGER) begin
                        lcd_line1 <= "HIGHHEAT Danger ";
                        fan <= 1;                       // 제연 및 환기 작동
                    end
                end
                D_BLACKOUT   : begin
                    if (d_grade == WARNING) begin
                        lcd_line1 <= "BLACKOUT Warning";
                    end
                    else if (d_grade == DANGER) begin
                        lcd_line1 <= "BLACKOUT Danger ";
                    end
                end
                D_LOUD_NOISE : begin
                    if (d_grade == WARNING) begin
                        lcd_line1 <= " BOOM Warning   ";
                    end
                    else if (d_grade == DANGER) begin
                        lcd_line1 <= " BOOM Danger    ";
                    end
                end
                D_GAS        : begin
                    if (d_grade == WARNING) begin
                        lcd_line1 <= "GAS LEAK Warning";
                    end
                    else if (d_grade == DANGER) begin
                        lcd_line1 <= "GAS LEAK Danger ";
                        fan <= 1;                       // 제연 및 환기 작동
                    end
                end
                default      : begin
                    start_servo <= 1;
                    if (lcd_toggle) lcd_line1 <= "   Happy Day    ";
                    else lcd_line1 <= "   Every Day    ";
                end
            endcase
        end
    end

    // 온습도 FND 4-Digit Output
    fnd_cntr fnd (.clk(clk), .reset_p(reset_p),
        .fnd_value({temp_bcd, humi_bcd}), .hex_bcd(1), .seg_7(seg_7), .dp(dp), .com(com));
endmodule

// PWM 적용 LED 출력 Test
module led_test_top (
    input clk, reset_p,
    input sw0,
    output reg led_r,
    output reg [2:0] led_g
    );

    wire [1:0] led_r_pwm;
    wire [3:0] led_g_pwm;
    pwm_Nstep #(.duty_step_N(128)) pwm_led_r0 (clk, reset_p, cnt_clk_led[27:21], led_r_pwm[0]);
    pwm_Nstep #(.duty_step_N(128)) pwm_led_r1 (clk, reset_p, cnt_clk_led[25:19], led_r_pwm[1]);
    pwm_Nstep #(.duty_step_N(128)) pwm_led_g0 (clk, reset_p, cnt_clk_led[27:23], led_g_pwm[0]);
    pwm_Nstep #(.duty_step_N(128)) pwm_led_g1 (clk, reset_p, cnt_clk_led[26:20], led_g_pwm[1]);
    pwm_Nstep #(.duty_step_N(128)) pwm_led_g2 (clk, reset_p, cnt_clk_led[26:22], led_g_pwm[2]);
    pwm_Nstep #(.duty_step_N(128)) pwm_led_g3 (clk, reset_p, cnt_clk_led[26:24], led_g_pwm[3]);

    reg [31:0] cnt_clk_led;
    always @(posedge clk, posedge reset_p) begin
        if (reset_p) cnt_clk_led <= 0;
        else cnt_clk_led <= cnt_clk_led + 1;
    end

    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            led_r <= 0;
            led_g <= 0;
        end
        else if (sw0) begin
            led_r <= led_r_pwm[1];
            led_g <= led_g_pwm[3:1];
        end
        else begin
            led_r <= led_r_pwm[0];
            led_g <= {3{led_g_pwm[0]}};
        end
    end
endmodule

/* // LCD 문자열 출력 Test
module lcd_test_top (
    input clk, reset_p,
    input [3:0] btn,
    output scl, sda,
    output [15:0] led
    );

    wire [3:0] btn_pedge;
    btn_cntr btn0 (clk, reset_p, btn[0], btn_pedge[0]);
    btn_cntr btn1 (clk, reset_p, btn[1], btn_pedge[1]);
    btn_cntr btn2 (clk, reset_p, btn[2], btn_pedge[2]);
    btn_cntr btn3 (clk, reset_p, btn[3], btn_pedge[3]);

    integer cnt_sysclk;
    reg cnt_sysclk_e;
    // System Clock Counter
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) cnt_sysclk <= 0;
        else if (cnt_sysclk_e) cnt_sysclk <= cnt_sysclk + 1;
        else cnt_sysclk <= 0;
    end

    reg [7:0] send_buffer;
    reg send, rs;
    wire busy;
    // Using Module, Byte Unit Transmission
    i2c_lcd_send_byte send_byte (clk, reset_p, 7'h27, send_buffer,
        send, rs, scl, sda, busy);

    localparam LCD_IDLE       = 5'b0_0001;
    localparam LCD_INIT       = 5'b0_0010;
    localparam SEND_STRING_1  = 5'b0_0100;
    localparam SEND_STRING_2  = 5'b0_1000;
    localparam MOVE_CURSOR    = 5'b1_0000;

    // Change State in Negative Edge
    reg [4:0] state, next_state;
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) state <= LCD_IDLE;
        else state <= next_state;
    end

    reg [8*16-1:0] str_1 = "Hello Vivado 123";
    reg [8*16-1:0] str_2 = "LCD Testing 1234";
    reg lcd_init_flag;
    reg lcd_row;
    reg [4:0] cnt_data;

    assign led[4:0] = state;
    assign led[5] = lcd_init_flag;
    assign led[6] = lcd_row;
    assign led[7] = busy;

    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            next_state <= LCD_IDLE;
            lcd_init_flag <= 0;
            lcd_row <= 0;
            cnt_data <= 0;
            send_buffer <= 0;
            send <= 0;
            rs <= 0;
        end
        else begin
            case (state)
                LCD_IDLE       : begin
                    if (lcd_init_flag) begin
                        if (btn_pedge[1]) begin
                            lcd_row <= 0;
                            next_state <= MOVE_CURSOR;
                        end
                        if (btn_pedge[2]) begin
                            lcd_row <= 1;
                            next_state <= MOVE_CURSOR;
                        end
                    end
                    else begin
                        if (cnt_sysclk < 32'd80_000_00) begin   // Wait 80ms
                            cnt_sysclk_e <= 1;       // System Clock Count Start
                        end
                        else begin
                            next_state <= LCD_INIT;  // Change State I2C_INIT
                            cnt_sysclk_e <= 0;       // System Clock Count Stop, Clear
                        end
                    end
                end
                LCD_INIT : begin
                    if (busy) begin                 // Communicating
                        send <= 0;                   // Wait Transmission
                        if (cnt_data >= 6) begin
                            cnt_data <= 0;           // LCD Initialization 6-Step Complete
                            next_state <= LCD_IDLE;  // Change State I2C_IDLE
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
                        send <= 1;                   // Request Transmission
                        cnt_data <= cnt_data + 1;    // Step Change
                    end
                end
                SEND_STRING_1  : begin
                    if (busy) begin
                        send <= 0;
                        if (cnt_data >= 16) begin
                            cnt_data <= 0;
                            next_state <= LCD_IDLE;
                        end
                    end
                    else if (!send) begin
                        rs <= 1;
                        send_buffer <= str_1[8*(16-cnt_data)-1 -: 8];
                        send <= 1;
                        cnt_data <= cnt_data + 1;
                    end
                end
                SEND_STRING_2  : begin
                    if (busy) begin
                        send <= 0;
                        if (cnt_data >= 16) begin
                            cnt_data <= 0;
                            next_state <= LCD_IDLE;
                        end
                    end
                    else if (!send) begin
                        rs <= 1;
                        send_buffer <= str_2[8*(16-cnt_data)-1 -: 8];
                        send <= 1;
                        cnt_data <= cnt_data + 1;
                    end
                end
                MOVE_CURSOR    : begin
                    if (busy) begin                 // Communicating
                        send <= 0;                   // Wait Transmission
                        if (lcd_row) next_state <= SEND_STRING_2;
                        else next_state <= SEND_STRING_1;
                    end
                    else if (!send) begin
                        rs <= 0;                     // Instruction Register Select
                        send_buffer <= {1'b1, lcd_row, 6'b00_0000};
                        send <= 1;                   // Request Transmission
                    end
                end
            endcase
        end
    end
endmodule
*/

// Module 이용 LCD 문자열 출력 Test
module lcd_mtest_top (
    input clk, reset_p,
    input [3:0] btn,
    output scl, sda,
    output [15:0] led
    );

    wire [3:0] btn_pedge;
    btn_cntr btn0 (clk, reset_p, btn[0], btn_pedge[0]);
    btn_cntr btn1 (clk, reset_p, btn[1], btn_pedge[1]);
    btn_cntr btn2 (clk, reset_p, btn[2], btn_pedge[2]);
    btn_cntr btn3 (clk, reset_p, btn[3], btn_pedge[3]);

    reg lcd_start, lcd_row;
    reg [8*16-1:0] lcd_txt;
    wire lcd_init_flag, busy;
    i2c_lcd_string string_lcd (clk, reset_p, lcd_start, lcd_row, lcd_txt,
        scl, sda, lcd_init_flag, busy, led);

    localparam LCD_IDLE       = 3'b001;
    localparam SEND_STRING_1  = 3'b010;
    localparam SEND_STRING_2  = 3'b100;

    // Change State in Negative Edge
    reg [2:0] state, next_state;
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) state <= LCD_IDLE;
        else state <= next_state;
    end

    reg [8*16-1:0] str_1 = "Hello Vivado 123";
    reg [8*16-1:0] str_2 = "LCD Testing 1234";

    always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            next_state <= LCD_IDLE;
            lcd_start <= 0;
            lcd_row <= 0;
            lcd_txt <= 0;
        end
        else begin
            case (state)
                LCD_IDLE       : begin
                    if (lcd_init_flag) begin
                        if (btn_pedge[1]) next_state <= SEND_STRING_1;
                        if (btn_pedge[2]) next_state <= SEND_STRING_2;
                    end
                end
                SEND_STRING_1  : begin
                    if (busy) begin
                        lcd_start <= 0;
                        next_state <= LCD_IDLE;
                    end
                    else begin
                        lcd_row <= 0;
                        lcd_txt <= str_1;
                        lcd_start <= 1;
                    end
                end
                SEND_STRING_2  : begin
                    if (busy) begin
                        lcd_start <= 0;
                        next_state <= LCD_IDLE;
                    end
                    else begin
                        lcd_row <= 1;
                        lcd_txt <= str_2;
                        lcd_start <= 1;
                    end
                end
            endcase
        end
    end
endmodule
