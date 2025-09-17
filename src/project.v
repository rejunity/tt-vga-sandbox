/*
 * Copyright (c) 2024 Uri Shaked
 * Copyright (c) 2025 ReJ aka Renaldas Zioma
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_vga_example(
  input  wire [7:0] ui_in,    // Dedicated inputs
  output wire [7:0] uo_out,   // Dedicated outputs
  input  wire [7:0] uio_in,   // IOs: Input path
  output wire [7:0] uio_out,  // IOs: Output path
  output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
  input  wire       ena,      // always 1 when the design is powered, so you can ignore it
  input  wire       clk,      // clock
  input  wire       rst_n     // reset_n - low to reset
);

  // VGA signals
  wire hsync;
  wire vsync;
  wire [1:0] R;
  wire [1:0] G;
  wire [1:0] B;

  // TinyVGA PMOD
  assign uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};

  // Unused outputs assigned to 0.
  assign uio_out = 0;
  assign uio_oe  = 0;

  
  wire [9:0] x_px;
  wire [9:0] y_px;
  wire activevideo;
  
  reg [19:0] tm;
  reg [9:0] y_prv;

  hvsync_generator hvsync_gen(
    .clk(clk),
    .reset(~rst_n),
    .hsync(hsync),
    .vsync(vsync),
    .display_on(activevideo),
    .hpos(x_px),
    .vpos(y_px)
  );

  wire [17:0] noise_value;
  
  worley_noise_generator worley_inst (
      .clk(clk),
      .x(x_px),
      .y(y_px),
      .t(tm),
      .noise(noise_value)
  );

  // assign R = activevideo ? { noise_value[7], noise_value[2] } : 2'b00;
  // assign G = activevideo ? { noise_value[6], noise_value[3] } : 2'b00;
  // assign B = activevideo ? { noise_value[5], noise_value[4] } : 2'b00;

  always @(posedge clk) begin
    if (~rst_n) begin
      tm <= 0;
    end else begin
      y_prv <= y_px;
      if (y_px == 0 && y_prv != y_px) begin
          tm <= tm + 1;
      end
    end
  end

  wire [10:0] frame = tm;
  wire [10:0] h_count = x_px;
  wire [9:0] v_count = y_px;

  // Bayer dithering
  // this is a 8x4 Bayer matrix which gets toggled every frame (so the other 8x4 elements are actually on odd frames)
  wire [2:0] bayer_i = h_count[2:0] ^ {3{frame[0]}};
  wire [1:0] bayer_j = v_count[1:0];
  wire [2:0] bayer_x = {bayer_i[2], bayer_i[1]^bayer_j[1], bayer_i[0]^bayer_j[0]};
  wire [4:0] bayer = {bayer_x[0], bayer_i[0], bayer_x[1], bayer_i[1], bayer_x[2]};

  // output dithered 2 bit color from 6 bit color and 5 bit Bayer matrix
  function [1:0] dither2;
      input [5:0] color6;
      input [4:0] bayer5;
      begin
          dither2 = ({1'b0, color6} + {2'b0, bayer5} + color6[0] + color6[5] + color6[5:1]) >> 5;
      end
  endfunction

  wire [1:0] rdither = dither2(r, bayer);
  wire [1:0] gdither = dither2(g, bayer);
  wire [1:0] bdither = dither2(b, bayer);

  wire [5:0] r = noise_value[12 +: 6];
  wire [5:0] b = noise_value[0 +: 6];
  wire [5:0] g = noise_value[6 +: 6];

  assign R = activevideo ? { rdither } : 2'b00;
  assign G = activevideo ? { gdither } : 2'b00;
  assign B = activevideo ? { bdither } : 2'b00;

endmodule


module worley_noise_generator (
    input wire clk,
    input wire [9:0] x,
    input wire [9:0] y,
    input wire [19:0] t,
    output wire [17:0] noise
);

  // Define a small fixed grid of points
  wire [8:0] points_x[0:3];
  wire [8:0] points_y[0:3];

  assign points_x[0] = 100 + t;
  assign points_y[0] = 100 - t;
  assign points_x[1] = 300 - (t >> 1);
  assign points_y[1] = 200 + (t >> 1);
  assign points_x[2] = 500 + (t >> 1);
  assign points_y[2] = 400 - (t >> 4);
  assign points_x[3] = 100 - (t >> 3);
  assign points_y[3] = 500 - (t >> 2);

//#(11,4,3)
  wire [15:0] sq0x_; approx_signed_square #(12,5,4) sq0x(.a(x - points_x[0]), .p_approx(sq0x_));
  wire [15:0] sq0y_; approx_signed_square #(12,5,4) sq0y(.a(y - points_y[0]), .p_approx(sq0y_));
  wire [15:0] sq1x_; approx_signed_square #(12,4,3) sq1x(.a(x - points_x[1]), .p_approx(sq1x_));
  wire [15:0] sq1y_; approx_signed_square #(12,4,3) sq1y(.a(y - points_y[1]), .p_approx(sq1y_));
  wire [15:0] sq2x_; approx_signed_square #(12,4,3) sq2x(.a(x - points_x[2]), .p_approx(sq2x_));
  wire [15:0] sq2y_; approx_signed_square #(12,4,3) sq2y(.a(y - points_y[2]), .p_approx(sq2y_));
  wire [15:0] sq3x_; approx_signed_square #(12,4,3) sq3x(.a(x - points_x[3]), .p_approx(sq3x_));
  wire [15:0] sq3y_; approx_signed_square #(12,4,3) sq3y(.a(y - points_y[3]), .p_approx(sq3y_));
  

  function [15:0] square;
    input [7:0] value;
    begin
      // square = (value[3+:5] * value[3+:5]) << 5 + value;
      square = value * value;
    end
  endfunction

  function [15:0] dot;
    input signed [9:0] a;
    input signed [9:0] b;
    begin
      dot = a * a + b * b;
      // dot = (a/4 * a/4)*16 + a + (b/4 * b/4)*16 + b;
      // dot = (a/2 * a/2)*4 + (b/2 * b/2)*4;
    end
  endfunction

  function signed [15:0] dot_;
    input signed [9:0] a;
    input signed [9:0] b;
    begin
      // dot = a * a + b * b;
      // dot_ = (a/4 * a/4)*16 + (b/4 * b/4)*16;
      // dot_ = (a/2 * a/2)*4 + (b/2 * b/2)*4;
    end
  endfunction

  // wire [15:0] distance1 = sq0x_ + sq0y_;
  // wire [15:0] distance2 = sq1x_ + sq1y_;
  // wire [15:0] distance3 = sq2x_ + sq2y_;
  // wire [15:0] distance4 = sq3x_ + sq3y_;
  // wire [15:0] distance1 = dot(x - points_x[0], y - points_y[0]);
  // wire [15:0] distance2 = dot(x - points_x[1], y - points_y[1]);
  // wire [15:0] distance3 = dot(x - points_x[2], y - points_y[2]);
  // wire [15:0] distance4 = dot(x - points_x[3], y - points_y[3]);

  wire [15:0] distance1 = (x - points_x[0]) * (x - points_x[0]) + (y - points_y[0]) * (y - points_y[0]);
  wire [15:0] distance2 = (x - points_x[1]) * (x - points_x[1]) + (y - points_y[1]) * (y - points_y[1]);
  wire [15:0] distance3 = (x - points_x[2]) * (x - points_x[2]) + (y - points_y[2]) * (y - points_y[2]);
  wire [15:0] distance4 = (x - points_x[3]) * (x - points_x[3]) + (y - points_y[3]) * (y - points_y[3]);

  // wire [15:0] distance1 = square(x - points_x[0]) + square(y - points_y[0]);
  // wire [15:0] distance2 = square(x - points_x[1]) + square(y - points_y[1]);
  // wire [15:0] distance3 = square(x - points_x[2]) + square(y - points_y[2]);
  // wire [15:0] distance4 = square(x - points_x[3]) + square(y - points_y[3]);


  // wire [15:0] min_dist = 
  //   (distance1 < distance2) ? 
  //     (distance1 < distance3) ? 
  //       (distance1 < distance4) ? distance1 : distance4 : 
  //       (distance3 < distance4) ? distance3 : distance4 :
  //     (distance2 < distance3) ?
  //       (distance2 < distance4) ? distance2 : distance4 :
  //       (distance3 < distance4) ? distance3 : distance4;
  // wire [20:0] total_dist = distance1 - distance2 + distance3 - distance4;// + distance2 + distance3 + distance4;
  wire [20:0] total_dist = distance1 + distance2 - distance3 - distance4;//+ distance2 + distance3 + distance4;

  // assign noise = ~min_dist[115:8];  // Scale down to 8-bit value
  // assign noise = ~total_dist[8 +: 8];  // Scale down to 8-bit value
  assign noise = ~total_dist[5 +: 18];  // Scale down to 8-bit value

endmodule


module approx_signed_square #(
    parameter integer W = 12,
    parameter integer T = 4,  // truncate this many LSBs
    parameter integer R = 3   // use top R bits of low part to approximate cross-term
)(
    input  wire signed [W-1:0] a,
    output wire [2*W-1:0] p_approx
    // output wire signed [15:0] p_approx
);
    // -------------------------
    // Guards
    // -------------------------
    initial begin
        if (W <= 1)  $error("W must be >= 2");
        if (T < 0)   $error("T must be >= 0");
        if (T >= W)  $error("T must be <= W-1");
        if (R < 0)   $error("R must be >= 0");
        if (R > T)   $error("R must be <= T");
    end

    localparam integer H = W - T;                // width of high part
    localparam integer PROD_W_HH = 2*H;          // width of x_h^2
    localparam integer SHIFT_HH  = 2*T;          // alignment for x_h^2
    localparam integer SHIFT_X   = (2*T >= R) ? (2*T - R) : 0; // alignment for cross-term

    // -------------------------
    // Work with magnitude (unsigned) since a^2 is non-negative
    // -------------------------
    wire [W-1:0] x = a[W-1] ? (~a + 1'b1) : a;   // abs(a)

    // Partition (unsigned slices)
    wire [H-1:0] x_h = (T == 0) ? x[W-1:0] : x[W-1:T];
    wire [T-1:0] x_l = (T == 0) ? {T{1'b0}} : x[T-1:0];

    // Core: x_h^2 << (2T)
    wire [PROD_W_HH-1:0] prod_hh_u = x_h * x_h;
    wire [2*W-1:0] term_hh_u = {{(2*W-PROD_W_HH){1'b0}}, prod_hh_u} << SHIFT_HH;

    // Optional cross-term using only top R bits of x_l
    generate
        if (R == 0) begin : no_correction
            assign p_approx = $signed(term_hh_u); // pure truncation
        end else begin : with_correction
            // Top R bits of x_l (unsigned)
            wire [R-1:0] x_l_top = x_l[T-1 -: R];  // x_l >> (T-R), keeping R bits

            // One small multiplier: (H x R)
            wire [H+R-1:0] prod_hl_u = x_h * x_l_top;

            // Approximate 2*x_h*x_l << T  ≈  2*(x_h*x_l_top) << (2T - R)
            // "×2" is a left shift by 1
            wire [2*W-1:0] term_x_u =
                ({{(2*W-(H+R)){1'b0}}, prod_hl_u} << (SHIFT_X + 1));

            assign p_approx = $signed(term_hh_u + term_x_u);
            // assign p_approx = term_hh_u + term_x_u;
        end
    endgenerate

endmodule
