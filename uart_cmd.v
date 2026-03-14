module uart_cmd (
    input  wire clk,
    input  wire rx,
    output reg  tx,
    output reg  led
);

// ─── Parameters ────────────────────────────────────────────────────────────
parameter  CLK_FREQ  = 10_000_000;
parameter  BAUD_RATE = 57600;
localparam BIT_PERIOD  = CLK_FREQ / BAUD_RATE;
localparam HALF_PERIOD = BIT_PERIOD / 2;

// ─── ASCII constants ────────────────────────────────────────────────────────
localparam CR  = 8'h0D;
localparam LF  = 8'h0A;
localparam NUL = 8'h00;

// ─── RX Synchroniser ────────────────────────────────────────────────────────
reg rx_s0 = 1, rx_s1 = 1, rx_s2 = 1;
wire rx_sync = rx_s2;

always @(posedge clk) begin
    rx_s0 <= rx;
    rx_s1 <= rx_s0;
    rx_s2 <= rx_s1;
end

// ─── UART RX ────────────────────────────────────────────────────────────────
localparam RX_IDLE  = 2'd0,
           RX_START = 2'd1,
           RX_DATA  = 2'd2,
           RX_STOP  = 2'd3;

reg [1:0] rx_state   = RX_IDLE;
reg [7:0] rx_cnt     = 0;
reg [2:0] rx_bit_idx = 0;
reg [7:0] rx_shift   = 0;
reg       rx_valid   = 0;
reg [7:0] rx_byte    = 0;

always @(posedge clk) begin
    rx_valid <= 0;
    case (rx_state)
        RX_IDLE:  if (rx_sync == 0) begin
                      rx_state <= RX_START;
                      rx_cnt   <= HALF_PERIOD - 1;
                  end

        RX_START: if (rx_cnt == 0) begin
                      if (rx_sync == 0) begin
                          rx_state   <= RX_DATA;
                          rx_cnt     <= BIT_PERIOD - 1;
                          rx_bit_idx <= 0;
                      end else
                          rx_state <= RX_IDLE;
                  end else
                      rx_cnt <= rx_cnt - 1;

        RX_DATA:  if (rx_cnt == 0) begin
                      rx_shift   <= {rx_sync, rx_shift[7:1]};
                      rx_cnt     <= BIT_PERIOD - 1;
                      if (rx_bit_idx == 7)
                          rx_state <= RX_STOP;
                      else
                          rx_bit_idx <= rx_bit_idx + 1;
                  end else
                      rx_cnt <= rx_cnt - 1;

        RX_STOP:  if (rx_cnt == 0) begin
                      rx_state <= RX_IDLE;
                      if (rx_sync == 1) begin
                          rx_byte  <= rx_shift;
                          rx_valid <= 1;
                      end
                  end else
                      rx_cnt <= rx_cnt - 1;
    endcase
end

// ─── Command Buffer ──────────────────────────────────────────────────────────
reg [7:0] cmd_buf [0:15];
reg [3:0] cmd_len   = 0;
reg       cmd_ready = 0;
reg       cmd_done  = 0;

// Uppercase conversion
function [7:0] to_upper;
    input [7:0] ch;
    begin
        if (ch >= 8'h61 && ch <= 8'h7A)
            to_upper = ch & 8'hDF;
        else
            to_upper = ch;
    end
endfunction

// ─── TX Arbiter signals ──────────────────────────────────────────────────────
// These connect the buffer/parser to the TX engine
reg [7:0] echo_byte = 0;
reg       echo_req  = 0;   
reg       echo_ack  = 0;   

reg [7:0] resp_addr = 0;
reg       resp_req  = 0;   // request to send ROM response
reg       resp_ack  = 0;   // TX engine confirms response started

// ─── Command Buffer logic ────────────────────────────────────────────────────
always @(posedge clk) begin
    cmd_ready <= 0;
    echo_req  <= 0;   

    if (rx_valid) begin
        if (rx_byte == CR || rx_byte == LF) begin
            if (cmd_len > 0) begin
                echo_byte <= LF;      // send newline
                echo_req  <= 1;
                cmd_ready <= 1;
            end
        end else if (cmd_len < 15) begin
            echo_byte <= rx_byte;
            echo_req  <= 1;
            cmd_buf[cmd_len] <= to_upper(rx_byte);
            cmd_len          <= cmd_len + 1;
        end
    end

    if (cmd_done)
        cmd_len <= 0;
end

// ─── Response ROM ────────────────────────────────────────────────────────────
reg [7:0] rom [0:199];

initial begin
    // ROM[0] "LED ON\r\n"
    rom[0]="L";  rom[1]="E";  rom[2]="D";  rom[3]=" ";
    rom[4]="O";  rom[5]="N";  rom[6]="\r"; rom[7]="\n";
    rom[8]=NUL;

    // ROM[20] "LED OFF\r\n"
    rom[20]="L"; rom[21]="E"; rom[22]="D"; rom[23]=" ";
    rom[24]="O"; rom[25]="F"; rom[26]="F"; rom[27]="\r";
    rom[28]="\n"; rom[29]=NUL;

    // ROM[40] "GateMate OK\r\n"
    rom[40]="G"; rom[41]="a"; rom[42]="t"; rom[43]="e";
    rom[44]="M"; rom[45]="a"; rom[46]="t"; rom[47]="e";
    rom[48]=" "; rom[49]="O"; rom[50]="K"; rom[51]="\r";
    rom[52]="\n"; rom[53]=NUL;

    // ROM[80] HELP text
    rom[80]="C";  rom[81]="o";  rom[82]="m";  rom[83]="m";
    rom[84]="a";  rom[85]="n";  rom[86]="d";  rom[87]="s";
    rom[88]=":";  rom[89]="\r"; rom[90]="\n";
    rom[91]=" ";  rom[92]="L";  rom[93]="E";  rom[94]="D";
    rom[95]=" ";  rom[96]="O";  rom[97]="N";  rom[98]="\r";
    rom[99]="\n";
    rom[100]=" "; rom[101]="L"; rom[102]="E"; rom[103]="D";
    rom[104]=" "; rom[105]="O"; rom[106]="F"; rom[107]="F";
    rom[108]="\r";rom[109]="\n";
    rom[110]=" "; rom[111]="S"; rom[112]="T"; rom[113]="A";
    rom[114]="T"; rom[115]="U"; rom[116]="S"; rom[117]="\r";
    rom[118]="\n";
    rom[119]=" "; rom[120]="H"; rom[121]="E"; rom[122]="L";
    rom[123]="P"; rom[124]="\r";rom[125]="\n";rom[126]=NUL;

    // ROM[160] "Unknown command\r\n"
    rom[160]="U"; rom[161]="n"; rom[162]="k"; rom[163]="n";
    rom[164]="o"; rom[165]="w"; rom[166]="n"; rom[167]=" ";
    rom[168]="c"; rom[169]="o"; rom[170]="m"; rom[171]="m";
    rom[172]="a"; rom[173]="n"; rom[174]="d"; rom[175]="\r";
    rom[176]="\n";rom[177]=NUL;
end

// ─── Command Parser ──────────────────────────────────────────────────────────
wire is_led_on  = (cmd_len == 6) &&
                  (cmd_buf[0]=="L") && (cmd_buf[1]=="E") &&
                  (cmd_buf[2]=="D") && (cmd_buf[3]==" ") &&
                  (cmd_buf[4]=="O") && (cmd_buf[5]=="N");

wire is_led_off = (cmd_len == 7) &&
                  (cmd_buf[0]=="L") && (cmd_buf[1]=="E") &&
                  (cmd_buf[2]=="D") && (cmd_buf[3]==" ") &&
                  (cmd_buf[4]=="O") && (cmd_buf[5]=="F") &&
                  (cmd_buf[6]=="F");

wire is_status  = (cmd_len == 6) &&
                  (cmd_buf[0]=="S") && (cmd_buf[1]=="T") &&
                  (cmd_buf[2]=="A") && (cmd_buf[3]=="T") &&
                  (cmd_buf[4]=="U") && (cmd_buf[5]=="S");

wire is_help    = (cmd_len == 4) &&
                  (cmd_buf[0]=="H") && (cmd_buf[1]=="E") &&
                  (cmd_buf[2]=="L") && (cmd_buf[3]=="P");

localparam PARSE_IDLE = 2'd0,
           PARSE_EXEC = 2'd1,
           PARSE_WAIT = 2'd2;

reg [1:0] parse_state = PARSE_IDLE;

always @(posedge clk) begin
    cmd_done <= 0;
    resp_req <= 0;

    case (parse_state)
        PARSE_IDLE: begin
            if (cmd_ready) begin
                if (is_led_on) begin
                    led       <= 0;
                    resp_addr <= 0;
                end else if (is_led_off) begin
                    led       <= 1;
                    resp_addr <= 20;
                end else if (is_status) begin
                    resp_addr <= 40;
                end else if (is_help) begin
                    resp_addr <= 80;
                end else begin
                    resp_addr <= 160;
                end
                parse_state <= PARSE_EXEC;
            end
        end

        PARSE_EXEC: begin
            // Wait until TX is free and no echo is pending
            if (!tx_busy) begin
                resp_req    <= 1;
                parse_state <= PARSE_WAIT;
            end
        end

        PARSE_WAIT: begin
            resp_req <= 0;
            if (!tx_busy && !resp_req) begin
                cmd_done    <= 1;
                parse_state <= PARSE_IDLE;
            end
        end
    endcase
end

// ─── UART TX with arbiter ────────────────────────────────────────────────────

reg        tx_busy    = 0;
reg [7:0]  tx_cnt     = 0;
reg [2:0]  tx_bit_idx = 0;
reg [7:0]  tx_shift   = 0;
reg [7:0]  tx_addr    = 0;
reg        tx_rom_mode = 0;  // 0=single echo byte, 1=ROM string

localparam TX_IDLE  = 3'd0,
           TX_START = 3'd1,
           TX_DATA  = 3'd2,
           TX_STOP  = 3'd3,
           TX_LOAD  = 3'd4,
           TX_CHECK = 3'd5;

reg [2:0] tx_state = TX_IDLE;

always @(posedge clk) begin
    echo_ack <= 0;
    resp_ack <= 0;

    case (tx_state)
        TX_IDLE: begin
            tx      <= 1;
            tx_busy <= 0;

            // Echo has higher priority than response
            if (echo_req) begin
                // Single byte mode: load the echo byte directly
                tx_shift   <= echo_byte;
                tx_cnt     <= BIT_PERIOD - 1;
                tx_bit_idx <= 0;
                tx_busy    <= 1;
                tx_rom_mode<= 0;
                echo_ack   <= 1;
                tx_state   <= TX_START;

            end else if (resp_req) begin
                // ROM string mode: start walking from resp_addr
                tx_addr    <= resp_addr;
                tx_busy    <= 1;
                tx_rom_mode<= 1;
                resp_ack   <= 1;
                tx_state   <= TX_LOAD;
            end
        end

        // ROM string states
        TX_LOAD: begin
            tx_shift <= rom[tx_addr];
            tx_state <= TX_CHECK;
        end

        TX_CHECK: begin
            if (tx_shift == NUL) begin
                tx_busy  <= 0;
                tx_state <= TX_IDLE;
            end else begin
                tx_cnt     <= BIT_PERIOD - 1;
                tx_bit_idx <= 0;
                tx_state   <= TX_START;
            end
        end

        TX_START: begin
            tx <= 0;  // start bit
            if (tx_cnt == 0) begin
                tx_cnt   <= BIT_PERIOD - 1;
                tx_state <= TX_DATA;
            end else
                tx_cnt <= tx_cnt - 1;
        end

        TX_DATA: begin
            tx <= tx_shift[0];
            if (tx_cnt == 0) begin
                tx_shift <= tx_shift >> 1;
                tx_cnt   <= BIT_PERIOD - 1;
                if (tx_bit_idx == 7) begin
                    tx_state <= TX_STOP;
                end else
                    tx_bit_idx <= tx_bit_idx + 1;
            end else
                tx_cnt <= tx_cnt - 1;
        end

        TX_STOP: begin
            tx <= 1;  // stop bit
            if (tx_cnt == 0) begin
                if (tx_rom_mode) begin
                    // Move to next byte in ROM
                    tx_addr  <= tx_addr + 1;
                    tx_state <= TX_LOAD;
                end else begin
                    // Single byte done
                    tx_busy  <= 0;
                    tx_state <= TX_IDLE;
                end
            end else
                tx_cnt <= tx_cnt - 1;
        end
    endcase
end

endmodule
