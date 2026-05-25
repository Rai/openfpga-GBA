`default_nettype none

module gba_cart_bus #(
    parameter integer ADDR_HOLD_CYCLES = 2,
    parameter integer ADDR_LATCH_CYCLES = 4,
    parameter integer READ_TURNAROUND_CYCLES = 4,
    parameter integer READ_SETUP_CYCLES = 14,
    parameter integer WRITE_SETUP_CYCLES = 12,
    parameter integer WRITE_HOLD_CYCLES = 8
) (
    input  wire        clk,
    input  wire        reset,
    input  wire        cart_mode,

    input  wire        req,
    input  wire        wr,
    input  wire [27:0] addr,
    input  wire [1:0]  acc,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output reg         done,
    output wire        busy,

    inout  wire [7:0]  cart_tran_bank2,
    output wire        cart_tran_bank2_dir,
    inout  wire [7:0]  cart_tran_bank3,
    output wire        cart_tran_bank3_dir,
    inout  wire [7:0]  cart_tran_bank1,
    output wire        cart_tran_bank1_dir,
    inout  wire [7:4]  cart_tran_bank0,
    output wire        cart_tran_bank0_dir,
    inout  wire        cart_tran_pin30,
    output wire        cart_tran_pin30_dir,
    output wire        cart_pin30_pwroff_reset,
    inout  wire        cart_tran_pin31,
    output wire        cart_tran_pin31_dir
);

localparam [1:0] ACCESS_8BIT  = 2'b00;
localparam [1:0] ACCESS_16BIT = 2'b01;

localparam [3:0] ST_IDLE       = 4'd0;
localparam [3:0] ST_ADDR_SETUP = 4'd1;
localparam [3:0] ST_ADDR_LATCH = 4'd2;
localparam [3:0] ST_READ_TURN  = 4'd3;
localparam [3:0] ST_READ_SETUP = 4'd4;
localparam [3:0] ST_WRITE      = 4'd5;
localparam [3:0] ST_WRITE_HOLD = 4'd6;
localparam [3:0] ST_DONE       = 4'd7;
localparam [3:0] ST_READ_SEQ   = 4'd8;

localparam [7:0] ADDR_HOLD_COUNT   = ADDR_HOLD_CYCLES % 256;
localparam [7:0] ADDR_LATCH_COUNT  = ADDR_LATCH_CYCLES % 256;
localparam [7:0] READ_TURN_COUNT   = READ_TURNAROUND_CYCLES % 256;
localparam [7:0] READ_SETUP_COUNT  = READ_SETUP_CYCLES % 256;
localparam [7:0] WRITE_SETUP_COUNT = WRITE_SETUP_CYCLES % 256;
localparam [7:0] WRITE_HOLD_COUNT  = WRITE_HOLD_CYCLES % 256;

reg [3:0]  state;
reg [7:0]  wait_count;
reg        latched_wr;
reg [27:0] latched_addr;
reg [1:0]  latched_acc;
reg [31:0] latched_wdata;
reg [1:0]  beat;
reg [15:0] first_word;

reg        ad_drive;
reg [15:0] ad_out;
wire [15:0] ad_in = {cart_tran_bank2, cart_tran_bank3};

reg [7:0]  a_hi_out;
reg        rd_n;
reg        wr_n;
reg        cs1_n;
reg        cs2_n;
reg        phi_clk;
reg [2:0]  phi_div;

wire save_space = latched_addr[27:24] == 4'hE || latched_addr[27:24] == 4'hF;
wire gpio_space = latched_addr[27:24] == 4'h8 &&
                  latched_addr[23:0] >= 24'h0000C4 &&
                  latched_addr[23:0] <= 24'h0000C8;
wire cart_write_enable = latched_wr && (save_space || gpio_space);
wire wr_n_pin = (state == ST_WRITE && cart_write_enable) ? wr_n : 1'b1;
wire need_second_beat = latched_acc != ACCESS_8BIT && latched_acc != ACCESS_16BIT;
wire transaction_active = state != ST_IDLE && state != ST_DONE;
wire a_hi_drive = cart_mode && transaction_active && (!save_space || latched_wr);
wire [7:0] save_data_in = cart_tran_bank1;
wire [7:0] a_hi_pin_out = save_space ? latched_wdata[7:0] : a_hi_out;
assign busy = state != ST_IDLE;

wire [15:0] write_word =
    (latched_acc == ACCESS_8BIT)  ? {latched_wdata[7:0], latched_wdata[7:0]} :
    (latched_acc == ACCESS_16BIT) ? latched_wdata[15:0] :
                                    (beat == 2'd0 ? latched_wdata[15:0] : latched_wdata[31:16]);

wire [23:0] rom_word_addr = latched_addr[24:1] + {23'd0, beat[0]};
wire [15:0] addr_word = save_space ? latched_addr[15:0] : rom_word_addr[15:0];
wire [7:0]  addr_high = rom_word_addr[23:16];
wire        rom_page_end = rom_word_addr[15:0] == 16'hFFFF;

assign cart_tran_bank3     = (cart_mode && ad_drive) ? ad_out[7:0]  : 8'hzz;
assign cart_tran_bank2     = (cart_mode && ad_drive) ? ad_out[15:8] : 8'hzz;
assign cart_tran_bank3_dir = cart_mode && ad_drive;
assign cart_tran_bank2_dir = cart_mode && ad_drive;

assign cart_tran_bank1     = a_hi_drive ? a_hi_pin_out : 8'hzz;
assign cart_tran_bank1_dir = a_hi_drive;

assign cart_tran_bank0     = cart_mode ? {1'b0, wr_n_pin, rd_n, cs1_n} : 4'hf;
assign cart_tran_bank0_dir = cart_mode ? 1'b1 : 1'b1;

assign cart_tran_pin30     = cart_mode ? cs2_n : 1'bz;
assign cart_tran_pin30_dir = cart_mode ? 1'b1 : 1'b0;
assign cart_pin30_pwroff_reset = cart_mode;

assign cart_tran_pin31     = 1'bz;
assign cart_tran_pin31_dir = 1'b0;

always @(posedge clk) begin
    if (reset || !cart_mode) begin
        phi_div <= 3'd0;
        phi_clk <= 1'b0;
    end else if (phi_div == 3'd2) begin
        phi_div <= 3'd0;
        phi_clk <= ~phi_clk;
    end else begin
        phi_div <= phi_div + 3'd1;
    end
end

always @(posedge clk) begin
    done <= 1'b0;

    if (reset || !cart_mode) begin
        state <= ST_IDLE;
        wait_count <= 8'd0;
        latched_wr <= 1'b0;
        latched_addr <= 28'd0;
        latched_acc <= 2'd0;
        latched_wdata <= 32'd0;
        beat <= 2'd0;
        first_word <= 16'd0;
        rdata <= 32'd0;
        ad_drive <= 1'b0;
        ad_out <= 16'hFFFF;
        a_hi_out <= 8'hFF;
        rd_n <= 1'b1;
        wr_n <= 1'b1;
        cs1_n <= 1'b1;
        cs2_n <= 1'b1;
    end else begin
        case (state)
            ST_IDLE: begin
                ad_drive <= 1'b0;
                rd_n <= 1'b1;
                wr_n <= 1'b1;
                cs1_n <= 1'b1;
                cs2_n <= 1'b1;
                if (req) begin
                    latched_wr <= wr;
                    latched_addr <= addr;
                    latched_acc <= acc;
                    latched_wdata <= wdata;
                    beat <= 2'd0;
                    wait_count <= ADDR_HOLD_COUNT;
                    state <= ST_ADDR_SETUP;
                end
            end

            ST_ADDR_SETUP: begin
                ad_drive <= 1'b1;
                ad_out <= addr_word;
                a_hi_out <= addr_high;
                cs1_n <= 1'b1;
                cs2_n <= 1'b1;
                rd_n <= 1'b1;
                wr_n <= 1'b1;
                if (wait_count == 8'd0) begin
                    wait_count <= ADDR_LATCH_COUNT;
                    state <= ST_ADDR_LATCH;
                end else begin
                    wait_count <= wait_count - 8'd1;
                end
            end

            ST_ADDR_LATCH: begin
                ad_drive <= 1'b1;
                ad_out <= addr_word;
                a_hi_out <= addr_high;
                cs1_n <= save_space;
                cs2_n <= !save_space;
                rd_n <= 1'b1;
                wr_n <= 1'b1;
                if (wait_count == 8'd0) begin
                    wait_count <= latched_wr ? WRITE_SETUP_COUNT : READ_TURN_COUNT;
                    state <= latched_wr ? ST_WRITE : ST_READ_TURN;
                end else begin
                    wait_count <= wait_count - 8'd1;
                end
            end

            ST_READ_TURN: begin
                // Retail carts latch the multiplexed ROM address during the
                // CS-low/RD-high phase. Keep AD driven until RD asserts, then
                // release it for ROM data.
                ad_drive <= 1'b1;
                ad_out <= addr_word;
                rd_n <= 1'b1;
                wr_n <= 1'b1;
                if (wait_count == 8'd0) begin
                    wait_count <= READ_SETUP_COUNT;
                    state <= ST_READ_SETUP;
                end else begin
                    wait_count <= wait_count - 8'd1;
                end
            end

            ST_READ_SETUP: begin
                ad_drive <= save_space;
                ad_out <= addr_word;
                rd_n <= 1'b0;
                wr_n <= 1'b1;
                if (wait_count == 8'd0) begin
                    rd_n <= 1'b1;
                    if (save_space) begin
                        rdata <= {save_data_in, save_data_in, save_data_in, save_data_in};
                        state <= ST_DONE;
                    end else if (beat == 2'd0) begin
                        first_word <= ad_in;
                        if (need_second_beat) begin
                            beat <= 2'd1;
                            if (rom_page_end) begin
                                wait_count <= ADDR_HOLD_COUNT;
                                state <= ST_ADDR_SETUP;
                            end else begin
                                wait_count <= READ_TURN_COUNT;
                                state <= ST_READ_SEQ;
                            end
                        end else begin
                            rdata <= {16'd0, ad_in};
                            state <= ST_DONE;
                        end
                    end else begin
                        rdata <= {16'd0, ad_in};
                        if (latched_acc != ACCESS_8BIT && latched_acc != ACCESS_16BIT) begin
                            rdata <= {ad_in, first_word};
                        end
                        state <= ST_DONE;
                    end
                end else begin
                    wait_count <= wait_count - 8'd1;
                end
            end

            ST_READ_SEQ: begin
                // Sequential ROM halfword: keep CS low and let the cart
                // advance internally, matching the GBA burst path.
                ad_drive <= 1'b0;
                cs1_n <= 1'b0;
                cs2_n <= 1'b1;
                rd_n <= 1'b1;
                wr_n <= 1'b1;
                if (wait_count == 8'd0) begin
                    wait_count <= READ_SETUP_COUNT;
                    state <= ST_READ_SETUP;
                end else begin
                    wait_count <= wait_count - 8'd1;
                end
            end

            ST_WRITE: begin
                ad_drive <= 1'b1;
                ad_out <= save_space ? addr_word : write_word;
                rd_n <= 1'b1;
                wr_n <= 1'b0;
                if (wait_count == 8'd0) begin
                    wr_n <= 1'b1;
                    wait_count <= WRITE_HOLD_COUNT;
                    state <= ST_WRITE_HOLD;
                end else begin
                    wait_count <= wait_count - 8'd1;
                end
            end

            ST_WRITE_HOLD: begin
                rd_n <= 1'b1;
                wr_n <= 1'b1;
                if (wait_count == 8'd0) begin
                    if (need_second_beat && beat == 2'd0) begin
                        beat <= 2'd1;
                        wait_count <= ADDR_HOLD_COUNT;
                        state <= ST_ADDR_SETUP;
                    end else begin
                        state <= ST_DONE;
                    end
                end else begin
                    wait_count <= wait_count - 8'd1;
                end
            end

            ST_DONE: begin
                ad_drive <= 1'b0;
                rd_n <= 1'b1;
                wr_n <= 1'b1;
                cs1_n <= 1'b1;
                cs2_n <= 1'b1;
                done <= 1'b1;
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
