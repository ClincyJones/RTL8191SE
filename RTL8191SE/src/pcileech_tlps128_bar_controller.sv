//
// PCILeech FPGA.
// RTL8191SE
// PCIe BAR PIO controller.
//
// The PCILeech BAR PIO controller allows for easy user-implementation on top
// of the PCILeech AXIS128 PCIe TLP streaming interface.
// The controller consists of a read engine and a write engine and pluggable
// user-implemented PCIe BAR implementations (found at bottom of the file).
//
// Considerations:
// - The core handles 1 DWORD read + 1 DWORD write per CLK max. If a lot of
//   data is written / read from the TLP streaming interface the core may
//   drop packet silently.
// - The core reads 1 DWORD of data (without byte enable) per CLK.
// - The core writes 1 DWORD of data (with byte enable) per CLK.
// - All user-implemented cores must have the same latency in CLKs for the
//   returned read data or else undefined behavior will take place.
// - 32-bit addresses are passed for read/writes. Larger BARs than 4GB are
//   not supported due to addressing constraints. Lower bits (LSBs) are the
//   BAR offset, Higher bits (MSBs) are the 32-bit base address of the BAR.
// - DO NOT edit read/write engines.
// - DO edit pcileech_tlps128_bar_controller (to swap bar implementations).
// - DO edit the bar implementations (at bottom of the file, if neccessary).
//
// Example implementations exists below, swap out any of the example cores
// against a core of your use case, or modify existing cores.
// Following test cores exist (see below in this file):
// - pcileech_bar_impl_zerowrite4k = zero-initialized read/write BAR.
//     It's possible to modify contents by use of .coe file.
// - pcileech_bar_impl_loopaddr = test core that loops back the 32-bit
//     address of the current read. Does not support writes.
// - pcileech_bar_impl_none = core without any reply.
// 
// (c) Ulf Frisk, 2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_tlps128_bar_controller(
    input                   rst,
    input                   clk,
    input                   bar_en,
    input [15:0]            pcie_id,
    input [31:0]            base_address_register,
    output wire             int_enable,  // Cambiado a output
    IfAXIS128.sink_lite     tlps_in,
    IfAXIS128.source        tlps_out
);
    
    // ------------------------------------------------------------------------
    // 1: TLP RECEIVE:
    // Receive incoming BAR requests from the TLP stream:
    // send them onwards to read and write FIFOs
    // ------------------------------------------------------------------------
    wire in_is_wr_ready;
    bit  in_is_wr_last;
    wire in_is_first    = tlps_in.tuser[0];
    wire in_is_bar      = bar_en && (tlps_in.tuser[8:2] != 0);
    wire in_is_rd       = (in_is_first && tlps_in.tlast && ((tlps_in.tdata[31:25] == 7'b0000000) || (tlps_in.tdata[31:25] == 7'b0010000) || (tlps_in.tdata[31:24] == 8'b00000010)));
    wire in_is_wr       = in_is_wr_last || (in_is_first && in_is_wr_ready && ((tlps_in.tdata[31:25] == 7'b0100000) || (tlps_in.tdata[31:25] == 7'b0110000) || (tlps_in.tdata[31:24] == 8'b01000010)));
    
    always @ ( posedge clk )
        if ( rst ) begin
            in_is_wr_last <= 0;
        end
        else if ( tlps_in.tvalid ) begin
            in_is_wr_last <= !tlps_in.tlast && in_is_wr;
        end
    
    wire [6:0]  wr_bar;
    wire [31:0] wr_addr;
    wire [3:0]  wr_be;
    wire [31:0] wr_data;
    wire        wr_valid;
    wire [87:0] rd_req_ctx;
    wire [6:0]  rd_req_bar;
    wire [31:0] rd_req_addr;
    wire [3:0]  rd_req_be;
    wire        rd_req_valid;
    wire [87:0] rd_rsp_ctx;
    wire [31:0] rd_rsp_data;
    wire        rd_rsp_valid;
        
    pcileech_tlps128_bar_rdengine i_pcileech_tlps128_bar_rdengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .pcie_id        ( pcie_id                       ),
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_rd ),
        .tlps_out       ( tlps_out                      ),
        // BAR reads:
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_bar     ( rd_req_bar                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_be      ( rd_req_be                     ),
        .rd_req_valid   ( rd_req_valid                  ),
        .rd_rsp_ctx     ( rd_rsp_ctx                    ),
        .rd_rsp_data    ( rd_rsp_data                   ),
        .rd_rsp_valid   ( rd_rsp_valid                  )
    );

    pcileech_tlps128_bar_wrengine i_pcileech_tlps128_bar_wrengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_wr ),
        .tlps_in_ready  ( in_is_wr_ready                ),
        // outgoing BAR writes:
        .wr_bar         ( wr_bar                        ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid                      )
    );
    
    wire [87:0] bar_rsp_ctx[7];
    wire [31:0] bar_rsp_data[7];
    wire        bar_rsp_valid[7];
    
    assign rd_rsp_ctx = bar_rsp_valid[0] ? bar_rsp_ctx[0] :
                        bar_rsp_valid[1] ? bar_rsp_ctx[1] :
                        bar_rsp_valid[2] ? bar_rsp_ctx[2] :
                        bar_rsp_valid[3] ? bar_rsp_ctx[3] :
                        bar_rsp_valid[4] ? bar_rsp_ctx[4] :
                        bar_rsp_valid[5] ? bar_rsp_ctx[5] :
                        bar_rsp_valid[6] ? bar_rsp_ctx[6] : 0;
    assign rd_rsp_data = bar_rsp_valid[0] ? bar_rsp_data[0] :
                        bar_rsp_valid[1] ? bar_rsp_data[1] :
                        bar_rsp_valid[2] ? bar_rsp_data[2] :
                        bar_rsp_valid[3] ? bar_rsp_data[3] :
                        bar_rsp_valid[4] ? bar_rsp_data[4] :
                        bar_rsp_valid[5] ? bar_rsp_data[5] :
                        bar_rsp_valid[6] ? bar_rsp_data[6] : 0;
    assign rd_rsp_valid = bar_rsp_valid[0] || bar_rsp_valid[1] || bar_rsp_valid[2] || bar_rsp_valid[3] || bar_rsp_valid[4] || bar_rsp_valid[5] || bar_rsp_valid[6];
    
    pcileech_bar_impl_none i_bar0(
    .rst                   ( rst                           ),
    .clk                   ( clk                           ),
    .wr_addr               ( wr_addr                       ),
    .wr_be                 ( wr_be                         ),
    .wr_data               ( wr_data                       ),
    .wr_valid              ( wr_valid && wr_bar[0]         ),
    .rd_req_ctx            ( rd_req_ctx                    ),
    .rd_req_addr           ( rd_req_addr                   ),
    .rd_req_valid          ( rd_req_valid && rd_req_bar[0] ),
    .rd_rsp_ctx            ( bar_rsp_ctx[0]                ),
    .rd_rsp_data           ( bar_rsp_data[0]               ),
    .rd_rsp_valid          ( bar_rsp_valid[0]              )
);
    
    pcileech_bar_impl_RTL8191SE i_bar1(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[1]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_be      ( rd_req_be                     ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[1] ),
        .int_enable     ( int_enable                    ),  // Añadida esta línea
        .base_address_register ( base_address_register         ),
        .rd_rsp_ctx     ( bar_rsp_ctx[1]                ),
        .rd_rsp_data    ( bar_rsp_data[1]               ),
        .rd_rsp_valid   ( bar_rsp_valid[1]              )
    );
    
    pcileech_bar_impl_none i_bar2(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[2]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[2] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[2]                ),
        .rd_rsp_data    ( bar_rsp_data[2]               ),
        .rd_rsp_valid   ( bar_rsp_valid[2]              )
    );
    
    pcileech_bar_impl_none i_bar3(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[3]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[3] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[3]                ),
        .rd_rsp_data    ( bar_rsp_data[3]               ),
        .rd_rsp_valid   ( bar_rsp_valid[3]              )
    );
    
    pcileech_bar_impl_none i_bar4(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[4]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[4] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[4]                ),
        .rd_rsp_data    ( bar_rsp_data[4]               ),
        .rd_rsp_valid   ( bar_rsp_valid[4]              )
    );
    
    pcileech_bar_impl_none i_bar5(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[5]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[5] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[5]                ),
        .rd_rsp_data    ( bar_rsp_data[5]               ),
        .rd_rsp_valid   ( bar_rsp_valid[5]              )
    );
    
    pcileech_bar_impl_none i_bar6_optrom(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[6]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[6] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[6]                ),
        .rd_rsp_data    ( bar_rsp_data[6]               ),
        .rd_rsp_valid   ( bar_rsp_valid[6]              )
    );


endmodule



// ------------------------------------------------------------------------
// BAR WRITE ENGINE:
// Receives BAR WRITE TLPs and output BAR WRITE requests.
// Holds a 2048-byte buffer.
// Input flow rate is 16bytes/CLK (max).
// Output flow rate is 4bytes/CLK.
// If write engine overflows incoming TLP is completely discarded silently.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_wrengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    output                  tlps_in_ready,
    // outgoing BAR writes:
    output bit [6:0]        wr_bar,
    output bit [31:0]       wr_addr,
    output bit [3:0]        wr_be,
    output bit [31:0]       wr_data,
    output bit              wr_valid
);

    wire            f_rd_en;
    wire [127:0]    f_tdata;
    wire [3:0]      f_tkeepdw;
    wire [8:0]      f_tuser;
    wire            f_tvalid;
    
    bit [127:0]     tdata;
    bit [3:0]       tkeepdw;
    bit             tlast;
    
    bit [3:0]       be_first;
    bit [3:0]       be_last;
    bit             first_dw;
    bit [31:0]      addr;

    fifo_141_141_clk1_bar_wr i_fifo_141_141_clk1_bar_wr(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( {tlps_in.tuser[8:0], tlps_in.tkeepdw, tlps_in.tdata} ),
        .full           (                               ),
        .prog_empty     ( tlps_in_ready                 ),
        .rd_en          ( f_rd_en                       ),
        .dout           ( {f_tuser, f_tkeepdw, f_tdata} ),    
        .empty          (                               ),
        .valid          ( f_tvalid                      )
    );
    
    // STATE MACHINE:
    `define S_ENGINE_IDLE        3'h0
    `define S_ENGINE_FIRST       3'h1
    `define S_ENGINE_4DW_REQDATA 3'h2
    `define S_ENGINE_TX0         3'h4
    `define S_ENGINE_TX1         3'h5
    `define S_ENGINE_TX2         3'h6
    `define S_ENGINE_TX3         3'h7
    (* KEEP = "TRUE" *) bit [3:0] state = `S_ENGINE_IDLE;
    
    assign f_rd_en = (state == `S_ENGINE_IDLE) ||
                     (state == `S_ENGINE_4DW_REQDATA) ||
                     (state == `S_ENGINE_TX3) ||
                     ((state == `S_ENGINE_TX2 && !tkeepdw[3])) ||
                     ((state == `S_ENGINE_TX1 && !tkeepdw[2])) ||
                     ((state == `S_ENGINE_TX0 && !f_tkeepdw[1]));

    always @ ( posedge clk ) begin
        wr_addr     <= addr;
        wr_valid    <= ((state == `S_ENGINE_TX0) && f_tvalid) || (state == `S_ENGINE_TX1) || (state == `S_ENGINE_TX2) || (state == `S_ENGINE_TX3);
        
    end

    always @ ( posedge clk )
        if ( rst ) begin
            state <= `S_ENGINE_IDLE;
        end
        else case ( state )
            `S_ENGINE_IDLE: begin
                state   <= `S_ENGINE_FIRST;
            end
            `S_ENGINE_FIRST: begin
                if ( f_tvalid && f_tuser[0] ) begin
                    wr_bar      <= f_tuser[8:2];
                    tdata       <= f_tdata;
                    tkeepdw     <= f_tkeepdw;
                    tlast       <= f_tuser[1];
                    first_dw    <= 1;
                    be_first    <= f_tdata[35:32];
                    be_last     <= f_tdata[39:36];
                    if ( f_tdata[31:29] == 8'b010 ) begin       // 3 DW header, with data
                        addr    <= { f_tdata[95:66], 2'b00 };
                        state   <= `S_ENGINE_TX3;
                    end
                    else if ( f_tdata[31:29] == 8'b011 ) begin  // 4 DW header, with data
                        addr    <= { f_tdata[127:98], 2'b00 };
                        state   <= `S_ENGINE_4DW_REQDATA;
                    end 
                end
                else begin
                    state   <= `S_ENGINE_IDLE;
                end
            end 
            `S_ENGINE_4DW_REQDATA: begin
                state   <= `S_ENGINE_TX0;
            end
            `S_ENGINE_TX0: begin
                tdata       <= f_tdata;
                tkeepdw     <= f_tkeepdw;
                tlast       <= f_tuser[1];
                addr        <= addr + 4;
                wr_data     <= { f_tdata[0+00+:8], f_tdata[0+08+:8], f_tdata[0+16+:8], f_tdata[0+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (f_tkeepdw[1] ? 4'hf : be_last);
                state       <= f_tvalid ? (f_tkeepdw[1] ? `S_ENGINE_TX1 : `S_ENGINE_FIRST) : `S_ENGINE_IDLE;
            end
            `S_ENGINE_TX1: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[32+00+:8], tdata[32+08+:8], tdata[32+16+:8], tdata[32+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[2] ? 4'hf : be_last);
                state       <= tkeepdw[2] ? `S_ENGINE_TX2 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX2: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[64+00+:8], tdata[64+08+:8], tdata[64+16+:8], tdata[64+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[3] ? 4'hf : be_last);
                state       <= tkeepdw[3] ? `S_ENGINE_TX3 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX3: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[96+00+:8], tdata[96+08+:8], tdata[96+16+:8], tdata[96+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (!tlast ? 4'hf : be_last);
                state       <= !tlast ? `S_ENGINE_TX0 : `S_ENGINE_FIRST;
            end
        endcase

endmodule



// ------------------------------------------------------------------------
// BAR READ ENGINE:
// Receives BAR READ TLPs and output BAR READ requests.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_rdengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    input [15:0]            pcie_id,
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    IfAXIS128.source        tlps_out,
    // BAR reads:
    output [87:0]           rd_req_ctx,
    output [6:0]            rd_req_bar,
    output [31:0]           rd_req_addr,
    output                  rd_req_valid,
    output [3:0]            rd_req_be,        
    input  [87:0]           rd_rsp_ctx,
    input  [31:0]           rd_rsp_data,
    input                   rd_rsp_valid
);
    // ------------------------------------------------------------------------
    // 1: PROCESS AND QUEUE INCOMING READ TLPs:
    // ------------------------------------------------------------------------
    wire [10:0] rd1_in_dwlen    = (tlps_in.tdata[9:0] == 0) ? 11'd1024 : {1'b0, tlps_in.tdata[9:0]};
    wire [6:0]  rd1_in_bar      = tlps_in.tuser[8:2];
    wire [15:0] rd1_in_reqid    = tlps_in.tdata[63:48];
    wire [7:0]  rd1_in_tag      = tlps_in.tdata[47:40];
    wire [31:0] rd1_in_addr     = { ((tlps_in.tdata[31:29] == 3'b000) ? tlps_in.tdata[95:66] : tlps_in.tdata[127:98]), 2'b00 };
    wire [3:0]  rd1_in_be       = tlps_in.tdata[35:32];
    wire [73:0] rd1_in_data;
    assign rd1_in_data[73:63]   = rd1_in_dwlen;
    assign rd1_in_data[62:56]   = rd1_in_bar;   
    assign rd1_in_data[55:48]   = rd1_in_tag;
    assign rd1_in_data[47:32]   = rd1_in_reqid;
    assign rd1_in_data[31:0]    = rd1_in_addr;

    
    wire [3:0]  rd1_out_be;
    wire        rd1_out_be_valid;
    wire        rd1_out_rden;
    wire [73:0] rd1_out_data;
    wire        rd1_out_valid;
    
    fifo_74_74_clk1_bar_rd1 i_fifo_74_74_clk1_bar_rd1(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( rd1_in_data                   ),
        .full           (                               ),
        .rd_en          ( rd1_out_rden                  ),
        .dout           ( rd1_out_data                  ),    
        .empty          (                               ),
        .valid          ( rd1_out_valid                 )
    );
    fifo_4_4_clk1_bar_rd1 i_fifo_4_4_clk1_bar_rd1 (
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( rd1_in_be                     ),
        .full           (                               ),
        .rd_en          ( rd1_out_rden                  ),
        .dout           ( rd1_out_be                    ),
        .empty          (                               ),
        .valid          ( rd1_out_be_valid              )

    );
    
    // ------------------------------------------------------------------------
    // 2: PROCESS AND SPLIT READ TLPs INTO RESPONSE TLP READ REQUESTS AND QUEUE:
    //    (READ REQUESTS LARGER THAN 128-BYTES WILL BE SPLIT INTO MULTIPLE).
    // ------------------------------------------------------------------------
    
    wire [10:0] rd1_out_dwlen       = rd1_out_data[73:63];
    wire [4:0]  rd1_out_dwlen5      = rd1_out_data[67:63];
    wire [4:0]  rd1_out_addr5       = rd1_out_data[6:2];
    
    // 1st "instant" packet:
    wire [4:0]  rd2_pkt1_dwlen_pre  = ((rd1_out_addr5 + rd1_out_dwlen5 > 6'h20) || ((rd1_out_addr5 != 0) && (rd1_out_dwlen5 == 0))) ? (6'h20 - rd1_out_addr5) : rd1_out_dwlen5;
    wire [5:0]  rd2_pkt1_dwlen      = (rd2_pkt1_dwlen_pre == 0) ? 6'h20 : rd2_pkt1_dwlen_pre;
    wire [10:0] rd2_pkt1_dwlen_next = rd1_out_dwlen - rd2_pkt1_dwlen;
    wire        rd2_pkt1_large      = (rd1_out_dwlen > 32) || (rd1_out_dwlen != rd2_pkt1_dwlen);
    wire        rd2_pkt1_tiny       = (rd1_out_dwlen == 1);
    wire [11:0] rd2_pkt1_bc         = rd1_out_dwlen << 2;
    wire [85:0] rd2_pkt1;
    assign      rd2_pkt1[85:74]     = rd2_pkt1_bc;
    assign      rd2_pkt1[73:63]     = rd2_pkt1_dwlen;
    assign      rd2_pkt1[62:0]      = rd1_out_data[62:0];
    
    // Nth packet (if split should take place):
    bit  [10:0] rd2_total_dwlen;
    wire [10:0] rd2_total_dwlen_next = rd2_total_dwlen - 11'h20;
    
    bit  [85:0] rd2_pkt2;
    wire [10:0] rd2_pkt2_dwlen = rd2_pkt2[73:63];
    wire        rd2_pkt2_large = (rd2_total_dwlen > 11'h20);
    
    wire        rd2_out_rden;
    
    // STATE MACHINE:
    `define S2_ENGINE_REQDATA     1'h0
    `define S2_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state2 = `S2_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            state2 <= `S2_ENGINE_REQDATA;
        end
        else case ( state2 )
            `S2_ENGINE_REQDATA: begin
                if ( rd1_out_valid && rd2_pkt1_large ) begin
                    rd2_total_dwlen <= rd2_pkt1_dwlen_next;                             // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_pkt1_dwlen_next << 2;                        // byte-count
                    rd2_pkt2[73:63] <= (rd2_pkt1_dwlen_next > 11'h20) ? 11'h20 : rd2_pkt1_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd1_out_data[62:12];                             // various data
                    rd2_pkt2[11:0]  <= rd1_out_data[11:0] + (rd2_pkt1_dwlen << 2);      // base address (within 4k page)
                    state2 <= `S2_ENGINE_PROCESSING;
                end
            end
            `S2_ENGINE_PROCESSING: begin
                if ( rd2_out_rden ) begin
                    rd2_total_dwlen <= rd2_total_dwlen_next;                                // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_total_dwlen_next << 2;                           // byte-count
                    rd2_pkt2[73:63] <= (rd2_total_dwlen_next > 11'h20) ? 11'h20 : rd2_total_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd2_pkt2[62:12];                                     // various data
                    rd2_pkt2[11:0]  <= rd2_pkt2[11:0] + (rd2_pkt2_dwlen << 2);              // base address (within 4k page)
                    if ( !rd2_pkt2_large ) begin
                        state2 <= `S2_ENGINE_REQDATA;
                    end
                end
            end
        endcase
    
    assign rd1_out_rden = rd2_out_rden && (((state2 == `S2_ENGINE_REQDATA) && (!rd1_out_valid || rd2_pkt1_tiny)) || ((state2 == `S2_ENGINE_PROCESSING) && !rd2_pkt2_large));

    wire [85:0] rd2_in_data  = (state2 == `S2_ENGINE_REQDATA) ? rd2_pkt1 : rd2_pkt2;
    wire        rd2_in_valid = rd1_out_valid || ((state2 == `S2_ENGINE_PROCESSING) && rd2_out_rden);
    wire [3:0]  rd2_in_be       = rd1_out_be;
    wire        rd2_in_be_valid = rd1_out_valid;

    bit  [85:0] rd2_out_data;
    bit         rd2_out_valid;
    bit  [3:0]  rd2_out_be;
    bit         rd2_out_be_valid;
    always @ ( posedge clk ) begin
        rd2_out_data    <= rd2_in_valid ? rd2_in_data : rd2_out_data;
        rd2_out_valid   <= rd2_in_valid && !rst;
        rd2_out_be       <= rd2_in_be_valid ? rd2_in_be : rd2_out_data;
        rd2_out_be_valid <= rd2_in_be_valid && !rst;  
    end

    // ------------------------------------------------------------------------
    // 3: PROCESS EACH READ REQUEST PACKAGE PER INDIVIDUAL 32-bit READ DWORDS:
    // ------------------------------------------------------------------------

    wire [4:0]  rd2_out_dwlen   = rd2_out_data[67:63];
    wire        rd2_out_last    = (rd2_out_dwlen == 1);
    wire [9:0]  rd2_out_dwaddr  = rd2_out_data[11:2];
    
    wire        rd3_enable;
    
    bit [3:0]   rd3_process_be;
    bit         rd3_process_valid;
    bit         rd3_process_first;
    bit         rd3_process_last;
    bit [4:0]   rd3_process_dwlen;
    bit [9:0]   rd3_process_dwaddr;
    bit [85:0]  rd3_process_data;
    wire        rd3_process_next_last = (rd3_process_dwlen == 2);
    wire        rd3_process_nextnext_last = (rd3_process_dwlen <= 3);
    assign rd_req_be    = rd3_process_be;
    assign rd_req_ctx   = { rd3_process_first, rd3_process_last, rd3_process_data };
    assign rd_req_bar   = rd3_process_data[62:56];
    assign rd_req_addr  = { rd3_process_data[31:12], rd3_process_dwaddr, 2'b00 };
    assign rd_req_valid = rd3_process_valid;
    
    // STATE MACHINE:
    `define S3_ENGINE_REQDATA     1'h0
    `define S3_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state3 = `S3_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            rd3_process_valid   <= 1'b0;
            state3              <= `S3_ENGINE_REQDATA;
        end
        else case ( state3 )
            `S3_ENGINE_REQDATA: begin
                if ( rd2_out_valid ) begin
                    rd3_process_valid       <= 1'b1;
                    rd3_process_first       <= 1'b1;                    // FIRST
                    rd3_process_last        <= rd2_out_last;            // LAST (low 5 bits of dwlen == 1, [max pktlen = 0x20))
                    rd3_process_dwlen       <= rd2_out_dwlen;           // PKT LENGTH IN DW
                    rd3_process_dwaddr      <= rd2_out_dwaddr;          // DWADDR OF THIS DWORD
                    rd3_process_data[85:0]  <= rd2_out_data[85:0];      // FORWARD / SAVE DATA
                    if ( rd2_out_be_valid ) begin
                        rd3_process_be <= rd2_out_be;
                    end else begin
                        rd3_process_be <= 4'hf;
                    end
                    if ( !rd2_out_last ) begin
                        state3 <= `S3_ENGINE_PROCESSING;
                    end
                end
                else begin
                    rd3_process_valid       <= 1'b0;
                end
            end
            `S3_ENGINE_PROCESSING: begin
                rd3_process_first           <= 1'b0;                    // FIRST
                rd3_process_last            <= rd3_process_next_last;   // LAST
                rd3_process_dwlen           <= rd3_process_dwlen - 1;   // LEN DEC
                rd3_process_dwaddr          <= rd3_process_dwaddr + 1;  // ADDR INC
                if ( rd3_process_next_last ) begin
                    state3 <= `S3_ENGINE_REQDATA;
                end
            end
        endcase

    assign rd2_out_rden = rd3_enable && (
        ((state3 == `S3_ENGINE_REQDATA) && (!rd2_out_valid || rd2_out_last)) ||
        ((state3 == `S3_ENGINE_PROCESSING) && rd3_process_nextnext_last));
    
    // ------------------------------------------------------------------------
    // 4: PROCESS RESPONSES:
    // ------------------------------------------------------------------------
    
    wire        rd_rsp_first    = rd_rsp_ctx[87];
    wire        rd_rsp_last     = rd_rsp_ctx[86];
    
    wire [9:0]  rd_rsp_dwlen    = rd_rsp_ctx[72:63];
    wire [11:0] rd_rsp_bc       = rd_rsp_ctx[85:74];
    wire [15:0] rd_rsp_reqid    = rd_rsp_ctx[47:32];
    wire [7:0]  rd_rsp_tag      = rd_rsp_ctx[55:48];
    wire [6:0]  rd_rsp_lowaddr  = rd_rsp_ctx[6:0];
    wire [31:0] rd_rsp_addr     = rd_rsp_ctx[31:0];
    wire [31:0] rd_rsp_data_bs  = { rd_rsp_data[7:0], rd_rsp_data[15:8], rd_rsp_data[23:16], rd_rsp_data[31:24] };
    
    // 1: 32-bit -> 128-bit state machine:
    bit [127:0] tdata;
    bit [3:0]   tkeepdw = 0;
    bit         tlast;
    bit         first   = 1;
    wire        tvalid  = tlast || tkeepdw[3];
    
    always @ ( posedge clk )
        if ( rst ) begin
            tkeepdw <= 0;
            tlast   <= 0;
            first   <= 0;
        end
        else if ( rd_rsp_valid && rd_rsp_first ) begin
            tkeepdw         <= 4'b1111;
            tlast           <= rd_rsp_last;
            first           <= 1'b1;
            tdata[31:0]     <= { 22'b0100101000000000000000, rd_rsp_dwlen };            // format, type, length
            tdata[63:32]    <= { pcie_id[7:0], pcie_id[15:8], 4'b0, rd_rsp_bc };        // pcie_id, byte_count
            tdata[95:64]    <= { rd_rsp_reqid, rd_rsp_tag, 1'b0, rd_rsp_lowaddr };      // req_id, tag, lower_addr
            tdata[127:96]   <= rd_rsp_data_bs;
        end
        else begin
            tlast   <= rd_rsp_valid && rd_rsp_last;
            tkeepdw <= tvalid ? (rd_rsp_valid ? 4'b0001 : 4'b0000) : (rd_rsp_valid ? ((tkeepdw << 1) | 1'b1) : tkeepdw);
            first   <= 0;
            if ( rd_rsp_valid ) begin
                if ( tvalid || !tkeepdw[0] )
                    tdata[31:0]   <= rd_rsp_data_bs;
                if ( !tkeepdw[1] )
                    tdata[63:32]  <= rd_rsp_data_bs;
                if ( !tkeepdw[2] )
                    tdata[95:64]  <= rd_rsp_data_bs;
                if ( !tkeepdw[3] )
                    tdata[127:96] <= rd_rsp_data_bs;   
            end
        end
    
    // 2.1 - submit to output fifo - will feed into mux/pcie core.
    fifo_134_134_clk1_bar_rdrsp i_fifo_134_134_clk1_bar_rdrsp(
        .srst           ( rst                       ),
        .clk            ( clk                       ),
        .din            ( { first, tlast, tkeepdw, tdata } ),
        .wr_en          ( tvalid                    ),
        .rd_en          ( tlps_out.tready           ),
        .dout           ( { tlps_out.tuser[0], tlps_out.tlast, tlps_out.tkeepdw, tlps_out.tdata } ),
        .full           (                           ),
        .empty          (                           ),
        .prog_empty     ( rd3_enable                ),
        .valid          ( tlps_out.tvalid           )
    );
    
    assign tlps_out.tuser[1] = tlps_out.tlast;
    assign tlps_out.tuser[8:2] = 0;
    
    // 2.2 - packet count:
    bit [10:0]  pkt_count       = 0;
    wire        pkt_count_dec   = tlps_out.tvalid && tlps_out.tlast;
    wire        pkt_count_inc   = tvalid && tlast;
    wire [10:0] pkt_count_next  = pkt_count + pkt_count_inc - pkt_count_dec;
    assign tlps_out.has_data    = (pkt_count_next > 0);
    
    always @ ( posedge clk ) begin
        pkt_count <= rst ? 0 : pkt_count_next;
    end

endmodule


// ------------------------------------------------------------------------
// Example BAR implementation that does nothing but drop any read/writes
// silently without generating a response.
// This is only recommended for placeholder designs.
// Latency = N/A.
// ------------------------------------------------------------------------
module pcileech_bar_impl_none(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    initial rd_rsp_ctx = 0;
    initial rd_rsp_data = 0;
    initial rd_rsp_valid = 0;

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation of "address loopback" which can be useful
// for testing. Any read to a specific BAR address will result in the
// address as response.
// Latency = 2CLKs.
// ------------------------------------------------------------------------
module pcileech_bar_impl_loopaddr(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input [87:0]        rd_req_ctx,
    input [31:0]        rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]      rd_req_ctx_1;
    bit [31:0]      rd_req_addr_1;
    bit             rd_req_valid_1;
    
    always @ ( posedge clk ) begin
        rd_req_ctx_1    <= rd_req_ctx;
        rd_req_addr_1   <= rd_req_addr;
        rd_req_valid_1  <= rd_req_valid;
        rd_rsp_ctx      <= rd_req_ctx_1;
        rd_rsp_data     <= 32'h0;
        rd_rsp_valid    <= rd_req_valid_1;
    end    

endmodule

// ------------------------------------------------------------------------
// Example BAR implementation of a 4kB writable initial-zero BAR.
// Latency = 2CLKs.
// ------------------------------------------------------------------------
module pcileech_bar_impl_zerowrite4k(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]  drd_req_ctx;
    bit         drd_req_valid;
    wire [31:0] doutb;
    
    always @ ( posedge clk ) begin
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        rd_rsp_data     <= doutb; 
    end
    
    bram_bar_zero4k i_bram_bar_zero4k(
        // Port A - write:
        .addra  ( wr_addr[11:2]     ),
        .clka   ( clk               ),
        .dina   ( wr_data           ),
        .ena    ( wr_valid          ),
        .wea    ( wr_be             ),
        // Port A - read (2 CLK latency):
        .addrb  ( rd_req_addr[11:2] ),
        .clkb   ( clk               ),
        .doutb  ( doutb             ),
        .enb    ( rd_req_valid      )
    );

endmodule

`define MAC_RANDOM_NUM1 13
`define MAC_RANDOM_NUM2 2
`define MAC_RANDOM_NUM3 13
`define MAC_RANDOM_NUM4 9
`define MAC_RANDOM_NUM5 7
`define MAC_RANDOM_NUM6 5

// ------------------------------------------------------------------------
// pcileech wifi BAR implementation
// Works with rtl81xx chips
// ------------------------------------------------------------------------
module pcileech_bar_impl_RTL8191SE(
    input               rst,
    input               clk,
    // interrupt enable
    output  wire        int_enable,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input  [3:0]        rd_req_be,
    input               rd_req_valid,
    input  [31:0]       base_address_register,
    // outgoing BAR read replies:
    output reg [87:0]   rd_rsp_ctx,
    output reg [31:0]   rd_rsp_data,
    output reg          rd_rsp_valid
);

    bit [87:0]      drd_req_ctx;
    bit [31:0]      drd_req_addr;
    bit [3:0]       drd_req_be;
    bit             drd_req_valid;
    
    bit [3:0]       dwr_be;
    bit [31:0]      dwr_addr;
    bit [31:0]      dwr_data;
    bit             dwr_valid;
    bit [31:0]      data_32;
    bit [7:0]       cmd;
    bit [15:0]     int_status;
    bit [15:0]     int_mask;
    
    time number = 0;
    reg [31:0] interrupt_counter = 0;
    reg interrupt_pending = 0;

    `define MAC_RANDOM_NUM1 13
    `define MAC_RANDOM_NUM2 2
    `define MAC_RANDOM_NUM3 13
    `define MAC_RANDOM_NUM4 9
    `define MAC_RANDOM_NUM5 7
    `define MAC_RANDOM_NUM6 5

    always @ ( posedge clk ) begin
        if (rst) begin
            number <= 0;
            data_32 <= 0;
            int_mask <= 16'b0;
            int_status <= 16'b0;
            interrupt_counter <= 0;
            interrupt_pending <= 0;
        end
        number          <= number + 1;
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        drd_req_be      <= rd_req_be;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;
        dwr_be          <= wr_be;
        
        if (int_enable) begin
            if (interrupt_counter < 100000) begin
                interrupt_counter <= interrupt_counter + 1;
            end else begin
                interrupt_counter <= 0;
                interrupt_pending <= 1;
            end
        end
        
        if (drd_req_valid) begin
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & ~32'h4)) & 32'hFFFF)
				32'h0500 : rd_rsp_data <= 32'h002F3210; // <- specal address register - this dont update
                32'h0504 : rd_rsp_data <= 32'h0000A422; // <- specal address register 
                32'h0508 : rd_rsp_data <= 32'h006EA42B; // <- specal address register 
			    default : begin
                    case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & ~32'h4)) & 32'h00FF)
							8'h00 : begin
                   rd_rsp_data[7:0]   <= 8'h12;  // static MAC prefix 
                   rd_rsp_data[15:8]  <= 8'h34;  // XX-XX-XX-00-00-00 to XX-XX-XX-FF-FF-FF 
                   rd_rsp_data[23:16] <= 8'h56;  // xx Technology Co.,Ltd
                   rd_rsp_data[31:24] <= ((0 + (number) % (15 + 1 - 0)) << 4) | (0 + (number + 3) % (15 + 1 - 0));
                        end
               8'h04 : begin
                   rd_rsp_data[7:0]   <= ((0 + (number + 6) % (15 + 1 - 0)) << 4) | (0 + (number + 9) % (15 + 1 - 0));
                   rd_rsp_data[15:8]  <= ((0 + (number + 12) % (15 + 1 - 0)) << 4) | (0 + (number + 15) % (15 + 1 - 0));
                   rd_rsp_data[31:16] <= 16'h0000;
                       end

             // 16'h0000 : rd_rsp_data <= 32'hxx563412; //
             // 16'h0004 : rd_rsp_data <= 32'hxxxxxxxx; //
                16'h0008 : rd_rsp_data <= 32'h0020B8A4;
                16'h0010 : rd_rsp_data <= 32'h87535703;
                16'h0018 : rd_rsp_data <= 32'h07A7B267;
                16'h001C : rd_rsp_data <= 32'h07000000;
                16'h0020 : rd_rsp_data <= 32'h20172435;
                16'h0024 : rd_rsp_data <= 32'hDB0F2203;
                16'h0028 : rd_rsp_data <= 32'h01796911;
                16'h002C : rd_rsp_data <= 32'h000000FF;
                16'h0030 : rd_rsp_data <= 32'hF271FAFF;
                16'h0034 : rd_rsp_data <= 32'h30000000;
                16'h0038 : rd_rsp_data <= 32'h00910000;
                16'h003C : rd_rsp_data <= 32'h02202D21;
                16'h0040 : rd_rsp_data <= 32'h000037FC;
                16'h0044 : rd_rsp_data <= 32'h008002FF;
                16'h0048 : rd_rsp_data <= 32'h8357F00E;
                16'h004C : rd_rsp_data <= 32'h00100000;
                16'h0050 : rd_rsp_data <= 32'h83D64B1C;
                16'h0054 : rd_rsp_data <= 32'h00005B7B;
                16'h0058 : rd_rsp_data <= 32'h814CE000;
                16'h005C : rd_rsp_data <= 32'h00000196;
                16'h0060 : rd_rsp_data <= {16'h0000, interrupt_pending, 15'h0000};  // Interrupt status register
                16'h0064 : rd_rsp_data <= 32'hFFFFFFFF;
                16'h0080 : rd_rsp_data <= 32'h0E7F9FE2;
                16'h0088 : rd_rsp_data <= 32'h28280900;
                16'h008C : rd_rsp_data <= 32'h0E0E0A0A;
                16'h0090 : rd_rsp_data <= 32'h013A4013;
                16'h0094 : rd_rsp_data <= 32'h00020064;
                16'h0098 : rd_rsp_data <= 32'h03FF0022;
                16'h009C : rd_rsp_data <= 32'h00328F0A;
                16'h00A0 : rd_rsp_data <= 32'h15150707;
                16'h00A4 : rd_rsp_data <= 32'h02020000;
                16'h00A8 : rd_rsp_data <= 32'h0000BC02;
                16'h00AC : rd_rsp_data <= 32'h102177FF;
                16'h00B0 : rd_rsp_data <= 32'h102407FF;
                16'h00B4 : rd_rsp_data <= 32'h200421FD;
                16'h00B8 : rd_rsp_data <= 32'h000FFF00;
                16'h00BC : rd_rsp_data <= 32'h00006003;
                16'h00C4 : rd_rsp_data <= 32'h00D10002;
                16'h00C8 : rd_rsp_data <= 32'h00150000;
                16'h00CC : rd_rsp_data <= 32'h00070007;
                16'h00D4 : rd_rsp_data <= 32'h00020002;
                16'h00D8 : rd_rsp_data <= 32'h00000080;
                16'h00DC : rd_rsp_data <= 32'hFFFFFFFF;
                16'h00E4 : rd_rsp_data <= 32'h102302E0;
                16'h00E8 : rd_rsp_data <= 32'h10220000;
                16'h00F0 : rd_rsp_data <= 32'h10213580;
                16'h00F4 : rd_rsp_data <= 32'h10213580;
                16'h00FC : rd_rsp_data <= 32'h10217800;
                16'h0100 : rd_rsp_data <= 32'h102404F8;
                16'h0104 : rd_rsp_data <= 32'h102404F8;
                16'h010C : rd_rsp_data <= 32'h10240000;
                16'h0120 : rd_rsp_data <= 32'h0000FFFF;
                16'h0124 : rd_rsp_data <= 32'h0000FFFF;
                16'h0128 : rd_rsp_data <= 32'h0000FFFF;
                16'h012C : rd_rsp_data <= 32'h0000FFFF;
                16'h0130 : rd_rsp_data <= 32'h0000FFFF;
                16'h0134 : rd_rsp_data <= 32'h0000FFFF;
                16'h0138 : rd_rsp_data <= 32'h0000FFFF;
                16'h013C : rd_rsp_data <= 32'h0000FFFF;
                16'h0140 : rd_rsp_data <= 32'h0000FFFF;
                16'h0144 : rd_rsp_data <= 32'h0000FFFF;
                16'h0148 : rd_rsp_data <= 32'h0000FFFF;
                16'h014C : rd_rsp_data <= 32'h0000FFFF;
                16'h0154 : rd_rsp_data <= 32'h00010000;
                16'h0160 : rd_rsp_data <= 32'h1C1C1C1C;
                16'h0164 : rd_rsp_data <= 32'h1C030B0B;
                16'h0168 : rd_rsp_data <= 32'h1C1C1C1C;
                16'h016C : rd_rsp_data <= 32'h1C1C1C1C;
                16'h0170 : rd_rsp_data <= 32'h1C1C1C1C;
                16'h0174 : rd_rsp_data <= 32'h1C1C1C1C;
                16'h0178 : rd_rsp_data <= 32'h1C1C1C1C;
                16'h017C : rd_rsp_data <= 32'h1C1C1C1C;
                16'h0180 : rd_rsp_data <= 32'h000FFF08;
                16'h0184 : rd_rsp_data <= 32'h100FF015;
                16'h0188 : rd_rsp_data <= 32'h100FF010;
                16'h018C : rd_rsp_data <= 32'h100FF005;
                16'h0190 : rd_rsp_data <= 32'h100FF000;
                16'h0194 : rd_rsp_data <= 32'h00000FF5;
                16'h0198 : rd_rsp_data <= 32'h00000FF0;
                16'h019C : rd_rsp_data <= 32'h0000000F;
                16'h01A0 : rd_rsp_data <= 32'h100FF015;
                16'h01A4 : rd_rsp_data <= 32'h0F000000;
                16'h01A8 : rd_rsp_data <= 32'hDDD77442;
                16'h01AC : rd_rsp_data <= 32'hFFFDD772;
                16'h01B0 : rd_rsp_data <= 32'h04010000;
                16'h01B4 : rd_rsp_data <= 32'h09070605;
                16'h01B8 : rd_rsp_data <= 32'h04010000;
                16'h01BC : rd_rsp_data <= 32'h09070605;
                16'h01C0 : rd_rsp_data <= 32'h0D0C0C0C;
                16'h01C4 : rd_rsp_data <= 32'h0F0E0E0D;
                16'h01D0 : rd_rsp_data <= 32'h0000A422;
                16'h01D4 : rd_rsp_data <= 32'h0000A422;
                16'h01D8 : rd_rsp_data <= 32'h005E4322;
                16'h01DC : rd_rsp_data <= 32'h0000A422;
                16'h01E0 : rd_rsp_data <= 32'h0000061A;
                16'h01E4 : rd_rsp_data <= 32'h0000FD00;
                16'h01F4 : rd_rsp_data <= 32'h77773030;
                16'h01F8 : rd_rsp_data <= 32'h00C21051;
                16'h0200 : rd_rsp_data <= 32'h04000026;
                16'h0220 : rd_rsp_data <= 32'h00050000;
                16'h0230 : rd_rsp_data <= 32'h000021BC;
                16'h0234 : rd_rsp_data <= 32'h500C0080;
                16'h0240 : rd_rsp_data <= 32'h00010027;
                16'h0250 : rd_rsp_data <= 32'h0000002C;
                16'h0260 : rd_rsp_data <= 32'h32000000;
                16'h0264 : rd_rsp_data <= 32'h000A0200;
                16'h0284 : rd_rsp_data <= 32'h08000000;
                16'h02D4 : rd_rsp_data <= 32'h22FF16C0;
                16'h02EC : rd_rsp_data <= 32'h0F000008;
                16'h02F0 : rd_rsp_data <= 32'h00200802;
                16'h02F4 : rd_rsp_data <= 32'h433C0019;
                16'h02F8 : rd_rsp_data <= 32'h00000002;
                16'h0300 : rd_rsp_data <= 32'h00007FFF;
                16'h030C : rd_rsp_data <= 32'h00000008;
                16'h0310 : rd_rsp_data <= 32'h00000010;
                16'h0318 : rd_rsp_data <= 32'h8001229C;
                16'h031C : rd_rsp_data <= 32'h00000038;
                16'h0334 : rd_rsp_data <= 32'h000F0000;
                16'h0364 : rd_rsp_data <= 32'h006E0205;
                16'h0368 : rd_rsp_data <= 32'h10110000;
                16'h0374 : rd_rsp_data <= 32'h00002E59;
                16'h0400 : rd_rsp_data <= 32'hFDE068A6;
                16'h0404 : rd_rsp_data <= 32'h09718002;
                16'h0408 : rd_rsp_data <= 32'h0020B8A4;
                16'h0410 : rd_rsp_data <= 32'h87535703;
                16'h0418 : rd_rsp_data <= 32'h07A7B267;
                16'h041C : rd_rsp_data <= 32'h07000000;
                16'h0420 : rd_rsp_data <= 32'h20172435;
                16'h0424 : rd_rsp_data <= 32'hDB0F2203;
                16'h0428 : rd_rsp_data <= 32'h01796911;
                16'h042C : rd_rsp_data <= 32'h000000FF;
                16'h0430 : rd_rsp_data <= 32'hF271FAFF;
                16'h0434 : rd_rsp_data <= 32'h30000000;
                16'h0438 : rd_rsp_data <= 32'h00910000;
                16'h043C : rd_rsp_data <= 32'h02202D21;
                16'h0440 : rd_rsp_data <= 32'h000037FC;
                16'h0444 : rd_rsp_data <= 32'h008002FF;
                16'h0448 : rd_rsp_data <= 32'h8357F00E;
                16'h044C : rd_rsp_data <= 32'h00100000;
                16'h0450 : rd_rsp_data <= 32'h83D64B1C;
                16'h0454 : rd_rsp_data <= 32'h00005B7B;
                16'h0458 : rd_rsp_data <= 32'h814CE000;
                16'h045C : rd_rsp_data <= 32'h00000196;
                16'h0460 : rd_rsp_data <= 32'hFFFFFFFF;
                16'h0464 : rd_rsp_data <= 32'hFFFFFFFF;
                16'h0480 : rd_rsp_data <= 32'h0E7FA709;
                16'h0488 : rd_rsp_data <= 32'h28280900;
                16'h048C : rd_rsp_data <= 32'h0E0E0A0A;
                16'h0490 : rd_rsp_data <= 32'h013A4013;
                16'h0494 : rd_rsp_data <= 32'h00020064;
                16'h0498 : rd_rsp_data <= 32'h03FF0022;
                16'h049C : rd_rsp_data <= 32'h00328F0A;
                16'h04A0 : rd_rsp_data <= 32'h15150707;
                16'h04A4 : rd_rsp_data <= 32'h02020000;
                16'h04A8 : rd_rsp_data <= 32'h0000BC02;
                16'h04AC : rd_rsp_data <= 32'h102177FF;
                16'h04B0 : rd_rsp_data <= 32'h102407FF;
                16'h04B4 : rd_rsp_data <= 32'h200421FD;
                16'h04B8 : rd_rsp_data <= 32'h000FFF00;
                16'h04BC : rd_rsp_data <= 32'h00006003;
                16'h04C4 : rd_rsp_data <= 32'h00D10002;
                16'h04C8 : rd_rsp_data <= 32'h00150000;
                16'h04CC : rd_rsp_data <= 32'h00070007;
                16'h04D4 : rd_rsp_data <= 32'h00020002;
                16'h04D8 : rd_rsp_data <= 32'h00000080;
                16'h04DC : rd_rsp_data <= 32'hFFFFFFFF;
                16'h04E4 : rd_rsp_data <= 32'h102302E0;
                16'h04E8 : rd_rsp_data <= 32'h10220000;
                16'h04F0 : rd_rsp_data <= 32'h10213580;
                16'h04F4 : rd_rsp_data <= 32'h10213580;
                16'h04FC : rd_rsp_data <= 32'h10217800;
                16'h0500 : rd_rsp_data <= 32'h22050000;
                16'h0514 : rd_rsp_data <= 32'h5306D000;
                16'h0518 : rd_rsp_data <= 32'h53070000;
                16'h051C : rd_rsp_data <= 32'h53073200;
                16'h0524 : rd_rsp_data <= 32'h53077B80;
                16'h0528 : rd_rsp_data <= 32'h53079000;
                16'h052C : rd_rsp_data <= 32'h5307C000;
                16'h0530 : rd_rsp_data <= 32'h53081000;
                16'h0534 : rd_rsp_data <= 32'h53084200;
                16'h0538 : rd_rsp_data <= 32'h53084400;
                16'h053C : rd_rsp_data <= 32'h53068000;
                16'h0540 : rd_rsp_data <= 32'h00073C11;
                16'h0544 : rd_rsp_data <= 32'h00000020;
                16'h0548 : rd_rsp_data <= 32'h0000407C;
                16'h0550 : rd_rsp_data <= 32'h5066FF80;
                16'h0554 : rd_rsp_data <= 32'h00000019;
                16'h0560 : rd_rsp_data <= 32'h00000040;
                16'h0600 : rd_rsp_data <= 32'h04000026;
                16'h0620 : rd_rsp_data <= 32'h00050000;
                16'h0630 : rd_rsp_data <= 32'h000021BC;
                16'h0634 : rd_rsp_data <= 32'h500C0080;
                16'h0640 : rd_rsp_data <= 32'h00010027;
                16'h0650 : rd_rsp_data <= 32'h0000002C;
                16'h0660 : rd_rsp_data <= 32'h32000000;
                16'h0664 : rd_rsp_data <= 32'h000A0200;
                16'h0684 : rd_rsp_data <= 32'h08000000;
                16'h06D4 : rd_rsp_data <= 32'h22FF16C0;
                16'h06EC : rd_rsp_data <= 32'h0F000008;
                16'h06F0 : rd_rsp_data <= 32'h00200802;
                16'h06F4 : rd_rsp_data <= 32'h433C0019;
                16'h06F8 : rd_rsp_data <= 32'h00000002;
                16'h0700 : rd_rsp_data <= 32'h00007FFF;
                16'h070C : rd_rsp_data <= 32'h00000008;
                16'h0710 : rd_rsp_data <= 32'h00000010;
                16'h0718 : rd_rsp_data <= 32'h8001239C;
                16'h071C : rd_rsp_data <= 32'h00000038;
                16'h0734 : rd_rsp_data <= 32'h000F0000;
                16'h0764 : rd_rsp_data <= 32'h006E0205;
                16'h0768 : rd_rsp_data <= 32'h10110000;
                16'h0774 : rd_rsp_data <= 32'h00002E59;
                16'h0800 : rd_rsp_data <= 32'h03040000;
                16'h0804 : rd_rsp_data <= 32'h00008003;
                16'h0808 : rd_rsp_data <= 32'h0000FC00;
                16'h080C : rd_rsp_data <= 32'h0000000A;
                16'h0810 : rd_rsp_data <= 32'h10005088;
                16'h0814 : rd_rsp_data <= 32'h020C3D10;
                16'h0818 : rd_rsp_data <= 32'h00200185;
                16'h0820 : rd_rsp_data <= 32'h01000000;
                16'h0824 : rd_rsp_data <= 32'h80390004;
                16'h0828 : rd_rsp_data <= 32'h01000000;
                16'h082C : rd_rsp_data <= 32'h80390002;
                16'h0830 : rd_rsp_data <= 32'h00000004;
                16'h0834 : rd_rsp_data <= 32'h00690200;
                16'h0838 : rd_rsp_data <= 32'h00000004;
                16'h083C : rd_rsp_data <= 32'h00690200;
                16'h0840 : rd_rsp_data <= 32'h02400060;
                16'h0844 : rd_rsp_data <= 32'h0183000A;
                16'h0858 : rd_rsp_data <= 32'h48484848;
                16'h085C : rd_rsp_data <= 32'h65A965A9;
                16'h0860 : rd_rsp_data <= 32'h0F7F0130;
                16'h0864 : rd_rsp_data <= 32'h0F7F0130;
                16'h0868 : rd_rsp_data <= 32'h0F7F0130;
                16'h086C : rd_rsp_data <= 32'h0F7F0130;
                16'h0870 : rd_rsp_data <= 32'h07000700;
                16'h0874 : rd_rsp_data <= 32'h03000300;
                16'h0878 : rd_rsp_data <= 32'h00000002;
                16'h087C : rd_rsp_data <= 32'h004F0201;
                16'h0880 : rd_rsp_data <= 32'hA8300AC1;
                16'h0884 : rd_rsp_data <= 32'h00000058;
                16'h0888 : rd_rsp_data <= 32'h00000008;
                16'h088C : rd_rsp_data <= 32'h00000004;
                16'h0894 : rd_rsp_data <= 32'hFFFFFFFE;
                16'h0898 : rd_rsp_data <= 32'h40302010;
                16'h089C : rd_rsp_data <= 32'h00706050;
                16'h08A0 : rd_rsp_data <= 32'h00100010;
                16'h08A4 : rd_rsp_data <= 32'h00132EB4;
                16'h08B8 : rd_rsp_data <= 32'h00100000;
                16'h08BC : rd_rsp_data <= 32'h00100000;
                16'h08E0 : rd_rsp_data <= 32'h02540614;
                16'h08E8 : rd_rsp_data <= 32'hF8000000;
                16'h08EC : rd_rsp_data <= 32'h2E2E2E2E;
                16'h08F0 : rd_rsp_data <= 32'h2C2E2E2E;
                16'h08F4 : rd_rsp_data <= 32'h080003DF;
                16'h08F8 : rd_rsp_data <= 32'h8080E8E4;
                16'h0904 : rd_rsp_data <= 32'h00000023;
                16'h0908 : rd_rsp_data <= 32'h00000803;
                16'h090C : rd_rsp_data <= 32'h01121311;
                16'h09C0 : rd_rsp_data <= 32'h0000A1DF;
                16'h09C4 : rd_rsp_data <= 32'h00003050;
                16'h0A00 : rd_rsp_data <= 32'h00D047C8;
                16'h0A04 : rd_rsp_data <= 32'h80FF0008;
                16'h0A08 : rd_rsp_data <= 32'h8CCD8300;
                16'h0A0C : rd_rsp_data <= 32'h2E62120F;
                16'h0A10 : rd_rsp_data <= 32'h9500BB78;
                16'h0A14 : rd_rsp_data <= 32'h11144028;
                16'h0A18 : rd_rsp_data <= 32'h00881117;
                16'h0A1C : rd_rsp_data <= 32'h89140F00;
                16'h0A20 : rd_rsp_data <= 32'h191A0000;
                16'h0A24 : rd_rsp_data <= 32'h090D1216;
                16'h0A28 : rd_rsp_data <= 32'h00000204;
                16'h0A2C : rd_rsp_data <= 32'h10D38000;
                16'h0A50 : rd_rsp_data <= 32'h1B03E10B;
                16'h0A54 : rd_rsp_data <= 32'h08702601;
                16'h0A58 : rd_rsp_data <= 32'h00005881;
                16'h0A5C : rd_rsp_data <= 32'h0000021E;
                16'h0C00 : rd_rsp_data <= 32'h40071D40;
                16'h0C04 : rd_rsp_data <= 32'h00A05633;
                16'h0C08 : rd_rsp_data <= 32'h000000E4;
                16'h0C0C : rd_rsp_data <= 32'h6C6C6C6C;
                16'h0C10 : rd_rsp_data <= 32'h08800000;
                16'h0C14 : rd_rsp_data <= 32'h40000100;
                16'h0C18 : rd_rsp_data <= 32'h08000000;
                16'h0C1C : rd_rsp_data <= 32'h40000100;
                16'h0C20 : rd_rsp_data <= 32'h08000000;
                16'h0C24 : rd_rsp_data <= 32'h40000100;
                16'h0C28 : rd_rsp_data <= 32'h08000000;
                16'h0C2C : rd_rsp_data <= 32'h40000100;
                16'h0C30 : rd_rsp_data <= 32'h6DE9AC44;
                16'h0C34 : rd_rsp_data <= 32'h469652CF;
                16'h0C38 : rd_rsp_data <= 32'h49795994;
                16'h0C3C : rd_rsp_data <= 32'h0A979764;
                16'h0C40 : rd_rsp_data <= 32'h1F7C403F;
                16'h0C44 : rd_rsp_data <= 32'h000100B7;
                16'h0C48 : rd_rsp_data <= 32'hEC020000;
                16'h0C4C : rd_rsp_data <= 32'h007F037F;
                16'h0C50 : rd_rsp_data <= 32'h69543417;
                16'h0C54 : rd_rsp_data <= 32'h433C0094;
                16'h0C58 : rd_rsp_data <= 32'h69543417;
                16'h0C5C : rd_rsp_data <= 32'h433C0094;
                16'h0C60 : rd_rsp_data <= 32'h0E101108;
                16'h0C64 : rd_rsp_data <= 32'h433C0019;
                16'h0C68 : rd_rsp_data <= 32'h69080000;
                16'h0C70 : rd_rsp_data <= 32'h2C7F000D;
                16'h0C74 : rd_rsp_data <= 32'h0186155B;
                16'h0C78 : rd_rsp_data <= 32'h721F001E;
                16'h0C7C : rd_rsp_data <= 32'h00B91612;
                16'h0C80 : rd_rsp_data <= 32'h3C8000EF;
                16'h0C84 : rd_rsp_data <= 32'h20F60000;
                16'h0C88 : rd_rsp_data <= 32'h20000080;
                16'h0C8C : rd_rsp_data <= 32'h20200000;
                16'h0C90 : rd_rsp_data <= 32'h40730100;
                16'h0C98 : rd_rsp_data <= 32'h40000100;
                16'h0CA0 : rd_rsp_data <= 32'h00492492;
                16'h0CBC : rd_rsp_data <= 32'h28000000;
                16'h0CD8 : rd_rsp_data <= 32'h64B22427;
                16'h0CDC : rd_rsp_data <= 32'h00766932;
                16'h0CE0 : rd_rsp_data <= 32'h00222222;
                16'h0CE8 : rd_rsp_data <= 32'h37644302;
                16'h0CEC : rd_rsp_data <= 32'h2F97D40C;
                16'h0CF0 : rd_rsp_data <= 32'h04640000;
                16'h0CF4 : rd_rsp_data <= 32'h00280000;
                16'h0CF8 : rd_rsp_data <= 32'h00002125;
                16'h0CFC : rd_rsp_data <= 32'h080003DF;
                16'h0D00 : rd_rsp_data <= 32'h00000750;
                16'h0D04 : rd_rsp_data <= 32'h00000403;
                16'h0D08 : rd_rsp_data <= 32'h0000907F;
                16'h0D0C : rd_rsp_data <= 32'h00000001;
                16'h0D10 : rd_rsp_data <= 32'hA0633333;
                16'h0D14 : rd_rsp_data <= 32'h33333C63;
                16'h0D18 : rd_rsp_data <= 32'h6A8F5B6B;
                16'h0D2C : rd_rsp_data <= 32'hCC979975;
                16'h0D3C : rd_rsp_data <= 32'h00027293;
                16'h0D50 : rd_rsp_data <= 32'h6437140A;
                16'h0D54 : rd_rsp_data <= 32'h024DBD02;
                16'h0D5C : rd_rsp_data <= 32'h30032064;
                16'h0D60 : rd_rsp_data <= 32'h4653DE68;
                16'h0D64 : rd_rsp_data <= 32'h00518A3C;
                16'h0D68 : rd_rsp_data <= 32'h00002101;
                16'h0D6C : rd_rsp_data <= 32'h00200000;
                16'h0DA0 : rd_rsp_data <= 32'h016B02C4;
                16'h0DA4 : rd_rsp_data <= 32'h002601F9;
                16'h0DA8 : rd_rsp_data <= 32'h00000020;
                16'h0DB0 : rd_rsp_data <= 32'hFFECFFF6;
                16'h0DB4 : rd_rsp_data <= 32'hFFECFFF6;
                16'h0DB8 : rd_rsp_data <= 32'h0000FFFD;
                16'h0DBC : rd_rsp_data <= 32'h1C5C1FC8;
                16'h0DC4 : rd_rsp_data <= 32'h00060000;
                16'h0DC8 : rd_rsp_data <= 32'h00000367;
                16'h0DCC : rd_rsp_data <= 32'h00000002;
                16'h0DD0 : rd_rsp_data <= 32'h00001410;
                16'h0DD4 : rd_rsp_data <= 32'h0000362B;
                16'h0DD8 : rd_rsp_data <= 32'h80FB1418;
                16'h0DDC : rd_rsp_data <= 32'h000044F5;
                16'h0DE0 : rd_rsp_data <= 32'h1FEC1FF6;
                16'h0DE8 : rd_rsp_data <= 32'h0002209A;
                16'h0DF4 : rd_rsp_data <= 32'hD0000000;
                16'h0DF8 : rd_rsp_data <= 32'h0000002C;
                16'h0E00 : rd_rsp_data <= 32'h2E2E2E2E;
                16'h0E04 : rd_rsp_data <= 32'h2C2E2E2E;
                16'h0E08 : rd_rsp_data <= 32'h00002832;
                16'h0E0C : rd_rsp_data <= 32'h00000007;
                16'h0E10 : rd_rsp_data <= 32'h2B2B2B2B;
                16'h0E14 : rd_rsp_data <= 32'h2929292B;
                16'h0E18 : rd_rsp_data <= 32'h2B2B2B2B;
                16'h0E1C : rd_rsp_data <= 32'h2929292B;
                16'h0E30 : rd_rsp_data <= 32'h01007C00;
                16'h0E34 : rd_rsp_data <= 32'h01004800;
                16'h0E38 : rd_rsp_data <= 32'h1000DC1F;
                16'h0E3C : rd_rsp_data <= 32'h10008C1F;
                16'h0E40 : rd_rsp_data <= 32'h02140102;
                16'h0E44 : rd_rsp_data <= 32'h681604C7;
                16'h0E48 : rd_rsp_data <= 32'hF8000000;
                16'h0E4C : rd_rsp_data <= 32'h020028D1;
                16'h0E50 : rd_rsp_data <= 32'h01007C00;
                16'h0E54 : rd_rsp_data <= 32'h01004800;
                16'h0E58 : rd_rsp_data <= 32'h1000DC23;
                16'h0E5C : rd_rsp_data <= 32'h10008C23;
                16'h0E60 : rd_rsp_data <= 32'h02140102;
                16'h0E64 : rd_rsp_data <= 32'h28160D07;
                16'h0E6C : rd_rsp_data <= 32'h020028D1;
                16'h0E70 : rd_rsp_data <= 32'h31ED92FB;
                16'h0E74 : rd_rsp_data <= 32'h321536FB;
                16'h0E78 : rd_rsp_data <= 32'h321536FB;
                16'h0E7C : rd_rsp_data <= 32'h321536FB;
                16'h0E80 : rd_rsp_data <= 32'h321536FB;
                16'h0E84 : rd_rsp_data <= 32'h000D92FB;
                16'h0E88 : rd_rsp_data <= 32'h000D92FB;
                16'h0E8C : rd_rsp_data <= 32'h31ED92FB;
                16'h0E90 : rd_rsp_data <= 32'h0001E4A5;
                16'h0E94 : rd_rsp_data <= 32'h00FD0000;
                16'h0E98 : rd_rsp_data <= 32'h00001001;
                16'h0E9C : rd_rsp_data <= 32'h00010000;
                16'h0EA0 : rd_rsp_data <= 32'h0001A6D5;
                16'h0EA4 : rd_rsp_data <= 32'h01020000;
                16'h0EA8 : rd_rsp_data <= 32'h00000162;
                16'h0EAC : rd_rsp_data <= 32'h2400F000;
                16'h0EB0 : rd_rsp_data <= 32'h00001E73;
                16'h0EB4 : rd_rsp_data <= 32'h01000000;
                16'h0EB8 : rd_rsp_data <= 32'h000019B5;
                16'h0EC0 : rd_rsp_data <= 32'h0002D8EE;
                16'h0EC4 : rd_rsp_data <= 32'h00FD0000;
                16'h0EC8 : rd_rsp_data <= 32'h00000AAD;
                16'h0ECC : rd_rsp_data <= 32'h03F90000;
                16'h0ED0 : rd_rsp_data <= 32'h31ED92FB;
                16'h0ED4 : rd_rsp_data <= 32'h31ED92FB;
                16'h0ED8 : rd_rsp_data <= 32'h3FED92FB;
                16'h0EDC : rd_rsp_data <= 32'h000D92FB;
                16'h0EE0 : rd_rsp_data <= 32'h000D92FB;
                16'h0EE4 : rd_rsp_data <= 32'h015E5448;
                16'h0EE8 : rd_rsp_data <= 32'h21555448;
                16'h0EF0 : rd_rsp_data <= 32'h03040000;
                16'h0EF4 : rd_rsp_data <= 32'h00008003;
                16'h0F00 : rd_rsp_data <= 32'h00000300;
                16'h0F14 : rd_rsp_data <= 32'h00000003;
                16'h0F80 : rd_rsp_data <= 32'h0000000D;
                16'h0F84 : rd_rsp_data <= 32'h00000049;
                16'h0F88 : rd_rsp_data <= 32'h000030EE;
                16'h0F90 : rd_rsp_data <= 32'h00010000;
                16'h0F94 : rd_rsp_data <= 32'h219E001D;
                16'h0FA0 : rd_rsp_data <= 32'h43CE0000;
                default : rd_rsp_data <= 32'h00000000;
            endcase
          end
        endcase
        end else if (dwr_valid) begin
            case ({dwr_addr[31:24], dwr_addr[23:16], dwr_addr[15:08], dwr_addr[07:00]} - base_address_register)
                16'h0060: interrupt_pending <= 0;  // Write to clear the interrupt
            endcase
        end else begin
            rd_rsp_data[7:0]   <= ((0 + (number) % (15 + 1 - 0)) << 4) | (0 + (number + 3) % (15 + 1 - 0));
            rd_rsp_data[15:8]  <= ((0 + (number + 6) % (15 + 1 - 0)) << 4) | (0 + (number + 9) % (15 + 1 - 0));
            rd_rsp_data[23:16] <= ((0 + (number + 12) % (15 + 1 - 0)) << 4) | (0 + (number + 15) % (15 + 1 - 0));
            rd_rsp_data[31:24] <= ((0 + (number) % (15 + 1 - 0)) << 4) | (0 + (number + 3) % (15 + 1 - 0));
        end
    end

    assign int_enable = 1'b1;  // Always enabled for this example

endmodule