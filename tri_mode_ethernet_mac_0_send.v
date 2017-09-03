`timescale 1 ps/1 ps

module tri_mode_ethernet_mac_0_send #(
	parameter               DEST_ADDR      = 48'h199408280001,
    parameter               SRC_ADDR       = 48'h199408280000,
    parameter               MAX_SIZE       = 16'd2067,//18 + 1 + 2048
    parameter               MIN_SIZE       = 16'd64,
    parameter               ENABLE_VLAN    = 1'b0,
    parameter               VLAN_ID        = 12'd2,
    parameter               VLAN_PRIORITY  = 3'd2
)(
   input                   axi_tclk,
   input                   axi_treset,

   input [7:0]             instruct,
   input                   instruct_valid,
   
   input [7:0]			   fifo_data,
   output reg 			   fifo_rden,
   input				   fifo_empty,
   
   output reg  [7:0]       tdata,
   output                  tvalid,
   output reg              tlast,
   input                   tready
);

localparam	    DAS_CARD_ID 			         = 8'h94;

localparam	    DAS_DATA 			             = 8'd0,
                DAS_CMD_GET_ID 			         = 8'd1,
	            DAS_COMMAND_START_RECORD         = 8'd2,
                DAS_COMMAND_STOP_RECORD          = 8'd3,
                DAS_COMMAND_START_PLAYBACK       = 8'd4,
                DAS_COMMAND_STOP_PLAYBACK        = 8'd5,
                DAS_COMMAND_START_WRITE_DISK     = 8'd6,	
                DAS_COMMAND_STOP_WRITE_DISK      = 8'd7,        
                DAS_CMD_START_READ_DISK          = 8'd8,	
                DAS_CMD_STOP_READ_DISK           = 8'd9,
                DAS_COMMAND_CFG_ADC              = 8'd10,
                DAS_COMMAND_CFG_DAC              = 8'd11,
                DAS_COMMAND_SET_DMA_PARAM        = 8'd12,
                DAS_COMMAND_GET_DMA_STATUS       = 8'd13,
                DAS_COMMAND_SOFTWARE_RESET       = 8'd14;				   

localparam     IDLE        = 3'b000,
               HEADER      = 3'b001,
               SIZE        = 3'b010,
			   PKT_TYPE    = 3'b011,
               DATA        = 3'b100,
               OVERHEAD    = 3'b101;
               
// work out the adjustment required to get the right packet size.               
localparam     PKT_ADJUST  = (ENABLE_VLAN) ? 22 : 18;

// generate the vlan fields
localparam     VLAN_HEADER = {8'h81, 8'h00, VLAN_PRIORITY, 1'b0, VLAN_ID};

// generate the require header count compare
localparam     HEADER_LENGTH = (ENABLE_VLAN) ? 15 : 11;
   
(* mark_debug = "true" *) reg         [13:0]         byte_count;
reg         [3:0]          header_count;
reg         [4:0]          overhead_count;
reg         [13:0]         pkt_size;
reg         [13:0]         pkt_data_size;//by AJ
reg         [2:0]          next_gen_state;
(* mark_debug = "true" *) reg         [2:0]          gen_state;
wire        [7:0]          lut_data;
reg                        tvalid_int;
(* mark_debug = "true" *) reg                        new_packet;
(* mark_debug = "true" *) reg                        new_packet2;
(* mark_debug = "true" *) reg                        new_packet_type;
reg         [7:0]          instruct_reg;

always @(posedge axi_tclk)
begin
    if (axi_treset) begin
        new_packet <= 1'b0;
        new_packet2 <= 1'b0;
        new_packet_type <= 1'b0;
    end
    else begin
        if (instruct_valid) begin
            case (instruct)
                DAS_CMD_GET_ID : begin
                    new_packet <= 1'b1;
                    //new_packet2 <= 1'b0;
                    new_packet_type <= 1'b1;
                end
                DAS_CMD_START_READ_DISK : begin
                    //new_packet <= 1'b0;
                    new_packet2 <= 1'b1;
                    new_packet_type <= 1'b0;
                end
                DAS_CMD_STOP_READ_DISK : begin
                    //new_packet <= 1'b0;
                    new_packet2 <= 1'b0;
                    //new_packet_type <= 1'b0;
                end
            endcase
        end
        else begin
            new_packet <= 1'b0;
        end
    end
end

always @(posedge axi_tclk)
begin
   if (axi_treset) begin
      byte_count <= 0;
   end
   else if (gen_state == DATA & |byte_count & tready) begin
      byte_count <= byte_count -1;
   end     
   else if (gen_state == HEADER) begin
      //byte_count <= pkt_size;
	  byte_count <= pkt_data_size;
   end
end

always @(posedge axi_tclk)
begin
   if (axi_treset) begin
      header_count <= 0;
   end
   else if (gen_state == HEADER & !(&header_count) & (tready | !tvalid_int)) begin
      header_count <= header_count + 1;
   end
   else if (gen_state == SIZE & tready) begin
      header_count <= 0;
   end
end

always @(posedge axi_tclk)
begin
   if (axi_treset) begin
      overhead_count <= 0;
   end
   else if (gen_state == OVERHEAD & |overhead_count & tready) begin
      overhead_count <= overhead_count - 1;
   end
   else if (gen_state == IDLE) begin
      overhead_count <= 24;
   end
end

always @(posedge axi_tclk)
begin
    pkt_size <= MAX_SIZE - PKT_ADJUST;
	pkt_data_size <= MAX_SIZE - PKT_ADJUST - 1;
end

genvar i;  
generate
  for (i=0; i<=7; i=i+1) begin : lut_loop
    LUT6 #(
       .INIT      ({48'd0,
                    VLAN_HEADER[i],
                    VLAN_HEADER[i+8],
                    VLAN_HEADER[i+16],
                    VLAN_HEADER[i+24],
                    SRC_ADDR[i],
                    SRC_ADDR[i+8],
                    SRC_ADDR[i+16],
                    SRC_ADDR[i+24],
                    SRC_ADDR[i+32],
                    SRC_ADDR[i+40],
                    DEST_ADDR[i],
                    DEST_ADDR[i+8],
                    DEST_ADDR[i+16],
                    DEST_ADDR[i+24],
                    DEST_ADDR[i+32],
                    DEST_ADDR[i+40]
                   })   // Specify LUT Contents
    ) LUT6_inst (
       .O         (lut_data[i]), 
       .I0        (header_count[0]),
       .I1        (header_count[1]),
       .I2        (header_count[2]),
       .I3        (header_count[3]),
       .I4        (1'b0),
       .I5        (1'b0) 
    );
   end
endgenerate

always @(gen_state or new_packet or new_packet2 or fifo_empty or header_count or tready or byte_count or tvalid_int or overhead_count) begin
    next_gen_state = gen_state;
    case (gen_state)
        IDLE : begin
            if ((new_packet | (new_packet2 & !fifo_empty)) & !tvalid_int) begin
                next_gen_state = HEADER;
            end
        end
        HEADER : begin
            if (header_count == HEADER_LENGTH & tready) begin
                next_gen_state = SIZE;
            end
        end
        SIZE : begin
            if (header_count == 0 & tready) begin
                next_gen_state = PKT_TYPE;
            end
        end
        PKT_TYPE : begin
            if (tready) begin
                next_gen_state = DATA;
            end
        end
        DATA : begin
            if (byte_count == 1 & tready) begin
                next_gen_state = OVERHEAD;
            end
        end
        OVERHEAD : begin
            if (overhead_count == 1 & tready) begin 
                next_gen_state = IDLE;
            end
        end
        default : begin
            next_gen_state = IDLE;
        end
    endcase
end

always @(posedge axi_tclk)
begin
	if (axi_treset) begin
		gen_state <= IDLE;
	end
	else begin
		gen_state <= next_gen_state;
	end
end

always @(posedge axi_tclk)
begin
   if (axi_treset)
      tvalid_int <= 0;
   else if (gen_state != IDLE & gen_state != OVERHEAD)
      tvalid_int <= 1;
   else if (tready)
      tvalid_int <= 0;
end

always @(posedge axi_tclk)
begin
    if (axi_treset) begin
        instruct_reg <= 8'd0;
    end
    else begin
        if (gen_state == IDLE) begin
            instruct_reg <= instruct;
        end
    end
end

always @(posedge axi_tclk)
begin
    if (axi_treset) begin
        tdata <= 8'd0;
        fifo_rden <= 1'b0;
    end
    else if (gen_state == HEADER & (tready | !tvalid_int)) begin
		tdata <= lut_data;
	end
	else if (gen_state == SIZE & tready) begin
        if (header_count[3]) begin
			tdata <= {3'h0, pkt_size[13:8]};
	    end
		else begin
			tdata <= pkt_size[7:0];
		end
	end
    else if (gen_state == PKT_TYPE & tready) begin
        if (new_packet_type) begin
            tdata <= instruct_reg;
        end
        else begin
            tdata <= DAS_DATA;
        end
        
        if (instruct_reg == DAS_CMD_START_READ_DISK) begin
            fifo_rden <= 1'b1;
        end
	end
    else if (gen_state == DATA & tready) begin
		if (instruct_reg == DAS_CMD_GET_ID)
		begin
			if (byte_count == pkt_data_size) begin
				tdata <= DAS_CARD_ID;
			end
			else begin
				tdata <= 8'd0;
			end
		end
		else if (instruct_reg == DAS_CMD_START_READ_DISK) begin
			tdata <= fifo_data;	
			if (byte_count == 14'd1) begin
				fifo_rden <= 1'b0;
			end
		end
	end
end

always @(posedge axi_tclk)
begin
   if (axi_treset)
      tlast <= 0;
   else if (byte_count == 1 & tready) begin
      tlast <= 1;
   end
   else if (tready)
      tlast <= 0;
end

assign tvalid = tvalid_int;

endmodule
