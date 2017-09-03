////////////////////////////////////////////////////////////////////////////
//-- JingJia, Inc. All rights reserved.
//-- 2017-09-03 by zhubojun816
////////////////////////////////////////////////////////////////////////////
//
// checksum offload for AXI4-Stream interface 10g Ethernet subsystem
// 
////////////////////////////////////////////////////////////////////////////////

//----------------------------------------
// Module Section
//----------------------------------------
`timescale 1ps/1ps

module ethernet_checksum_offload #(
  ////////////////////////////////////////////////////////////////////////////
  // Width of S_AXIS data bus. The slave accepts the read and write data 
  // of width C_S_AXIS_TDATA_WIDTH.
  parameter integer C_S_AXIS_TDATA_NUM_BYTES = 4
  ////////////////////////////////////////////////////////////////////////////////
  // Width of S_AXIS address bus. The slave accepts the read and write addresses
  // of width C_M_AXIS_TDATA_NUM_BYTES.
  parameter integer C_M_AXIS_TDATA_NUM_BYTES = 4,
  parameter integer C_PACKET_LENGTH          = 12
) (
  ////////////////////////////////////////////////////////////////////////////
  // Global ports
  input wire S_AXIS_ACLK,
  input wire AXIS_ARESETN,
  input wire M_AXIS_ACLK,
  //input wire M_AXIS_ARESETN,  
  ////////////////////////////////////////////////////////////////////////////
  // Slave Stream Ports
  // TREADY indicates that the slave can accept a transfer in the
  // current cycle.
  output wire S_AXIS_TREADY,
  ////////////////////////////////////////////////////////////////////////////
  // TDATA is the primary payload that is used to provide the data
  // that is passing across the interface to the slave
  input wire [(C_S_AXIS_TDATA_NUM_BYTES*8)-1:0] S_AXIS_TDATA,
  ////////////////////////////////////////////////////////////////////////////
  // TSTRB is the byte qualifier that indicates whether the content
  // of the associated byte of TDATA is processed as a data byte or
  // a position byte.
  input wire [C_S_AXIS_TDATA_NUM_BYTES-1:0] S_AXIS_TSTRB,
  ////////////////////////////////////////////////////////////////////////////
  // TLAST indicates the boundary of a packet.
  input wire S_AXIS_TLAST,
  ////////////////////////////////////////////////////////////////////////////
  // TVALID indicates that the slave is accepting a valid transfer.
  // A transfer takes place when both TVALID and TREADY are asserted.
  input wire S_AXIS_TVALID
  ////////////////////////////////////////////////////////////////////////////////
  // Global ports
  //input wire M_AXIS_ARESETN,
  ////////////////////////////////////////////////////////////////////////////////
  // Master Stream Ports
  // TVALID indicates that the master is driving a valid transfer.
  // A transfer takes place when both TVALID and TREADY are asserted.
  output wire M_AXIS_TVALID,
  ////////////////////////////////////////////////////////////////////////////////
  // TDATA is the primary payload that is used to provide the data
  // that is passing across the interface from the master
  output wire [(C_M_AXIS_TDATA_NUM_BYTES*8)-1:0] M_AXIS_TDATA,
  ////////////////////////////////////////////////////////////////////////////////
  // TSTRB is the byte qualifier that indicates whether the content
  // of the associated byte of TDATA is processed as a data byte or
  // a position byte.
  output wire [C_M_AXIS_TDATA_NUM_BYTES-1:0] M_AXIS_TSTRB,
  ////////////////////////////////////////////////////////////////////////////////
  // TLAST indicates the boundary of a packet.
  output wire M_AXIS_TLAST,
  ////////////////////////////////////////////////////////////////////////////////
  // TREADY indicates that the slave can accept a transfer
  // in thecurrent cycle.
  input wire M_AXIS_TREADY
);

localparam [1:0]  IDLE      = 2'b00,
                  RCV       = 2'b01,
                  BACKOFF   = 2'b10;

reg [7:0] lsfr_backoff;
wire      lsfr_feedback;
assign lsfr_feedback = !(lsfr_backoff[7] ^ lsfr_backoff[3]);
reg [7:0] intra_packet_backoff;
reg [1:0] state;
reg       axis_tready;

always@(posedge AXIS_ACLK) begin
  if(!AXIS_ARESETN) begin
    lsfr_backoff <= 8'h00;
    state <= IDLE;
    axis_tready <= 1'b0;
  end else begin
    lsfr_backoff <= {lsfr_backoff[6:0], lsfr_feedback};
    case(state)
      IDLE: begin
        intra_packet_backoff <= lsfr_backoff;
        state <= BACKOFF;
        axis_tready <= 1'b0;
      end
      BACKOFF : begin
        intra_packet_backoff <= intra_packet_backoff + 1;
        if (intra_packet_backoff[4:0] == 5'h1f) begin
          state <= RCV;
        end else begin
          state <= BACKOFF;
        end
      end
      RCV: begin
        if (S_AXIS_TVALID && S_AXIS_TREADY && S_AXIS_TLAST) begin
          state <= IDLE;
          axis_tready <= 1'b0;
        end else begin
          axis_tready <= lsfr_backoff[7];
          state <= RCV;
        end
      end
      default : begin
        lsfr_backoff <= 8'h00;
        state <= IDLE;
        axis_tready <= 1'b0;
      end
    endcase
  end
end

assign S_AXIS_TREADY = axis_tready;

////////////////////////////////////////////////////////////////////////////////
// function called clogb2 that returns an integer which has the
// value of the ceiling of the log base 2.
function integer clogb2 (input integer bd);
integer bit_depth;
  begin
    bit_depth = bd;
    for(clogb2=0; bit_depth>0; clogb2=clogb2+1)
      bit_depth = bit_depth >> 1;
  end
endfunction

localparam LP_PACKET_COUNTER_WIDTH  = clogb2(C_PACKET_LENGTH);

reg [LP_PACKET_COUNTER_WIDTH-1:0]  packet_counter;

localparam [1:0]  IDLE      = 2'b00,
                  SEND      = 2'b01,
                  BACKOFF   = 2'b10;

reg [7:0] lsfr_backoff;
wire      lsfr_feedback;
assign lsfr_feedback = !(lsfr_backoff[7] ^ lsfr_backoff[3]);
reg [7:0] intra_packet_backoff;
reg [7:0] payload;
reg [1:0] state;
reg [(C_M_AXIS_TDATA_NUM_BYTES*8)-1:0] m_axis_tdata;
reg       m_axis_tvalid;
reg       m_axis_tlast;

always@(posedge AXIS_ACLK) begin
  if(!AXIS_ARESETN) begin
    lsfr_backoff <= 8'h00;
    packet_counter <= {LP_PACKET_COUNTER_WIDTH{1'b0}};
    state <= IDLE;
    intra_packet_backoff <= 8'h00;
    m_axis_tvalid <= 1'b0;
    m_axis_tlast <= 1'b0;
    m_axis_tdata <= {C_M_AXIS_TDATA_NUM_BYTES*8{1'b0}};
  end else begin
    lsfr_backoff <= {lsfr_backoff[6:0], lsfr_feedback};
    case (state)
      IDLE: begin
        packet_counter <= {LP_PACKET_COUNTER_WIDTH{1'b0}};
        intra_packet_backoff <= lsfr_backoff;
        state <= BACKOFF;
        m_axis_tvalid <= 1'b0;
        m_axis_tlast <= 1'b0;
      end
      BACKOFF : begin
        packet_counter <= {LP_PACKET_COUNTER_WIDTH{1'b0}};
        intra_packet_backoff <= intra_packet_backoff + 1;
        if (intra_packet_backoff[4:0] == 5'h1f) begin
          state <= SEND;
        end else begin
          state <= BACKOFF;
        end
      end
      SEND: begin
        m_axis_tvalid <= 1'b1;
        m_axis_tlast <= 1'b0;
        state <= SEND;
        if (packet_counter == C_PACKET_LENGTH) begin
          m_axis_tlast <= 1'b1;
          state <= IDLE;
        end else if (M_AXIS_TVALID && M_AXIS_TREADY) begin
          packet_counter <= packet_counter + 1;
          m_axis_tdata <= m_axis_tdata + {C_M_AXIS_TDATA_NUM_BYTES{8'h01}};
        end
      end
      default : begin
        packet_counter <= {LP_PACKET_COUNTER_WIDTH{1'b0}};
        state <= IDLE;
        intra_packet_backoff <= 8'h00;
        m_axis_tvalid <= 1'b0;
        m_axis_tlast <= 1'b0;
        m_axis_tdata <= {C_M_AXIS_TDATA_NUM_BYTES*8{1'b0}};
      end
    endcase
  end
end

assign M_AXIS_TSTRB = {C_M_AXIS_TDATA_NUM_BYTES{1'b1}};
assign M_AXIS_TVALID = m_axis_tvalid;
assign M_AXIS_TLAST = m_axis_tlast;
assign M_AXIS_TDATA = m_axis_tdata;

endmodule
