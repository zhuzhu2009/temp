`timescale 1 ps/1 ps

module tri_mode_ethernet_mac_0_recv #(
	parameter               DEST_ADDR      = 48'h199408280000,
	parameter               SRC_ADDR       = 48'h199408280001,
   //parameter               MAX_SIZE       = 16'd500,
    //parameter               MAX_SIZE       = 16'd9018,
    parameter               MAX_SIZE       = 16'd2067,//18 + 1 + 2048
    parameter               MIN_SIZE       = 16'd64,
	parameter               ENABLE_VLAN    = 1'b0,
    parameter               VLAN_ID        = 12'd2,
    parameter               VLAN_PRIORITY  = 3'd2
)(
	input                   axi_tclk,
	input                   axi_treset,

	input	[7:0]			rx_axis_fifo_tdata,
	input					rx_axis_fifo_tvalid,
	input                   rx_axis_fifo_tlast,
	output                  rx_axis_fifo_tready,
	
	output	[7:0] 			instruct,
	output                  instruct_valid,
	
	input                   fifo_full,
	output                  fifo_wren,
	output	[7:0] 			fifo_data
);

localparam	    DAS_DATA 			             = 8'd0;			   
				  
localparam     IDLE        = 3'b000,
               HEADER      = 3'b001,
               SIZE        = 3'b010,
			   PKT_TYPE    = 3'b011,
			   FRAME_FLAG  = 3'b100,			   
               DATA        = 3'b101,   
               OVERHEAD    = 3'b110;

// work out the adjustment required to get the right packet size.               
localparam     PKT_ADJUST  = (ENABLE_VLAN) ? 22 : 18;

// generate the vlan fields
localparam     VLAN_HEADER = {8'h81, 8'h00, VLAN_PRIORITY, 1'b0, VLAN_ID};

// generate the require header count compare
localparam     HEADER_LENGTH = (ENABLE_VLAN) ? 15 : 11;

reg     [7:0]   rx_axis_fifo_tdata_reg;
reg             rx_axis_fifo_tvalid_reg;
reg             rx_axis_fifo_tlast_reg;
reg             rx_axis_fifo_tready_reg;
reg	    [13:0]  data_count;
reg	    [3:0]   header_count;
reg	    [2:0]   next_rx_state;
reg	    [2:0]   rx_state;
wire   [7:0]    lut_data;
reg             header_check_valid;
reg     [15:0]  size_reg;
reg     [7:0]   pkt_type_reg;
reg     [31:0]   frame_flag_reg;
reg             instruct_valid_reg;
reg              fifo_wren_reg;

reg     [31:0]   frame_flag_reg2;
reg     [7:0]   frame_flag_error_reg;
reg             frame_flag_first_check;

assign instruct = pkt_type_reg;
assign instruct_valid = instruct_valid_reg;

assign fifo_wren = fifo_wren_reg;
assign fifo_data = rx_axis_fifo_tdata_reg;

// store the parametised values in a lut (64 deep)
// this should mean the values could be adjusted in fpga_editor etc..
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

// Write interface
always @(posedge axi_tclk)
begin
    if (axi_treset) begin
        rx_axis_fifo_tdata_reg <= 8'd0;
        rx_axis_fifo_tvalid_reg <= 1'b0;
        rx_axis_fifo_tlast_reg <= 1'b0;        
    end
    else begin
        rx_axis_fifo_tdata_reg <= rx_axis_fifo_tdata;
        rx_axis_fifo_tvalid_reg <= rx_axis_fifo_tvalid;
        rx_axis_fifo_tlast_reg <= rx_axis_fifo_tlast;    
    end

end

// need a packet counter - max size limited to 14 bits
always @(posedge axi_tclk)
begin
   if (axi_treset) begin
	  data_count <= 14'd0;
   end
   else if (rx_state == DATA & !(&data_count) & rx_axis_fifo_tvalid) begin
	  data_count <= data_count + 14'd1;
   end     
   else if (rx_state == IDLE) begin
	  data_count <= 14'd0;
   end
end

// need a smaller count to manage the header recv
always @(posedge axi_tclk)
begin
	if (axi_treset) begin
		header_count <= 4'd0;
	end
	else if (rx_state == HEADER & !(&header_count) & rx_axis_fifo_tvalid) begin
		header_count <= header_count + 4'd1;
	end
	else if (rx_state == SIZE & rx_axis_fifo_tvalid) begin
		header_count <= 4'd0;
	end
end

// simple state machine to control the data
always @(rx_state or header_count or rx_axis_fifo_tvalid or rx_axis_fifo_tlast)
begin
   next_rx_state = rx_state;
   case (rx_state)
		IDLE : begin
			if (rx_axis_fifo_tvalid & (!rx_axis_fifo_tvalid_reg | rx_axis_fifo_tlast_reg))
				next_rx_state = HEADER;
		end
		HEADER : begin
			if (header_count == HEADER_LENGTH & rx_axis_fifo_tvalid)
				next_rx_state = SIZE;
			else if (rx_axis_fifo_tlast & rx_axis_fifo_tvalid)
				next_rx_state = IDLE;
		end
		SIZE : begin
			if (header_count == 0 & rx_axis_fifo_tvalid)
				next_rx_state = PKT_TYPE;
			else if (rx_axis_fifo_tlast & rx_axis_fifo_tvalid)
				next_rx_state = IDLE;
		end
		PKT_TYPE : begin
			if (rx_axis_fifo_tvalid)
				next_rx_state = DATA;
		end
		DATA : begin
			if (rx_axis_fifo_tlast & rx_axis_fifo_tvalid)
				next_rx_state = IDLE;
		end
		default : begin
			next_rx_state = IDLE;
		end
   endcase
end

always @(posedge axi_tclk)
begin
	if (axi_treset) begin
		rx_state <= IDLE;
	end
	else begin
		rx_state <= next_rx_state;
	end
end

always @(posedge axi_tclk)
begin
    if (axi_treset) begin
        header_check_valid <= 1'b0;
        size_reg <= 16'd0;
        pkt_type_reg <= 8'hff;
        frame_flag_reg <= 32'd0;
        instruct_valid_reg <= 1'b0;
        fifo_wren_reg <= 1'b0;
    end
    else begin
        if (rx_state == IDLE) begin
            fifo_wren_reg <= 1'b0;
        end
        
        if (rx_state == HEADER) begin
            if (rx_axis_fifo_tdata_reg == lut_data & (header_check_valid | header_count == 4'd0)) begin
                header_check_valid <= 1'b1;
            end
            else begin
                header_check_valid <= 1'b0;
            end
        end
        
        if (rx_state == SIZE & header_check_valid) begin
            size_reg <= {size_reg[7:0], rx_axis_fifo_tdata_reg};
        end
        
        if (rx_state == PKT_TYPE & header_check_valid) begin
            pkt_type_reg <= rx_axis_fifo_tdata_reg;
        end
        
        if (rx_state == DATA & header_check_valid) begin
            if (pkt_type_reg == DAS_DATA) begin
                if (data_count <= 14'd3) begin
                    frame_flag_reg <= {rx_axis_fifo_tdata_reg, frame_flag_reg[31:8]};
                end
                
                if (data_count >= 14'd3) begin
                    fifo_wren_reg <= 1'b1;
                end            
            end
            else begin
                if (data_count == 14'd0) begin
                    instruct_valid_reg <= 1'b1;
                end
                else begin
                    instruct_valid_reg <= 1'b0;
                end
            end
        end
    end
end

always @(posedge axi_tclk)
begin
    if (axi_treset) begin
        frame_flag_reg2 <= 32'd0;    
        frame_flag_first_check <= 1'b1;
        frame_flag_error_reg <= 8'd0;
    end
    else begin
        if (pkt_type_reg == 8'h00) begin
            if (data_count == 14'd4) begin
                frame_flag_reg2 <= frame_flag_reg;
                
                if (frame_flag_first_check) begin
                    frame_flag_first_check <= 1'b0;
                end
                else begin
                    if (frame_flag_reg > frame_flag_reg2) begin
                        if ((frame_flag_reg - frame_flag_reg2) != 32'h00000001) begin
                            frame_flag_error_reg <= frame_flag_error_reg + 8'd1;
                        end
                    end    
                    else if (frame_flag_reg < frame_flag_reg2) begin
                        if (frame_flag_reg != 32'h00000000 || frame_flag_reg2 != 32'hffffffff) begin
                            frame_flag_error_reg <= frame_flag_error_reg + 8'd1;
                        end
                    end
                end               
            end
        end
        else begin
            frame_flag_first_check <= 1'b1;         
        end
    end
end 

assign rx_axis_fifo_tready = !fifo_full;

endmodule
