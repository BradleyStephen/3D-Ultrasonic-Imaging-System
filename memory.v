module axi_to_arm (
    input  wire                     clk,
    input  wire                     reset,
    input  wire                     axi_awvalid,
    input  wire [31:0]              axi_awaddr,
    input  wire [2:0]               axi_awprot,
    input  wire                     axi_wvalid,
    input  wire [31:0]              axi_wdata,
    input  wire [3:0]               axi_wstrb,
    output wire                     axi_awready,
    output wire                     axi_wready
);

// Registers to buffer address and data for AXI writes
reg [31:0] address_reg;
reg [31:0] data_reg;

// State machine states
parameter IDLE = 2'b00;
parameter WAIT_FOR_ADDRESS = 2'b01;
parameter WAIT_FOR_WRITE = 2'b10;

// State machine state
reg [1:0] state;

// Output signals
assign axi_awready = (state == WAIT_FOR_ADDRESS);
assign axi_wready = (state == WAIT_FOR_WRITE);

// Include Xilinx AXI Interconnect IP module
// Replace with the actual instantiation name and module instance name generated by Vivado
// Make sure to include the correct parameters for the AXI interconnect IP
// For example, replace PARAMETER_NAME with the actual parameters used in your design
// These parameters can be found in the IP Integrator or block design in Vivado
your_axi_mem_module axi_mem_interface (
    .S_AXI_ACLK(clk),            // Clock input
    .S_AXI_ARESETN(~reset),      // Reset input (active low)

    // AXI slave interface for writing to ARM memory
    .S_AXI_AWADDR(axi_awaddr),   // Write address
    .S_AXI_AWPROT(axi_awprot),   // Write protection signals
    .S_AXI_AWVALID(axi_awvalid), // Write address valid
    .S_AXI_AWREADY(axi_awready), // Write address ready
    .S_AXI_WDATA(axi_wdata),     // Write data
    .S_AXI_WSTRB(axi_wstrb),     // Write strobe
    .S_AXI_WVALID(axi_wvalid),   // Write data valid
    .S_AXI_WREADY(axi_wready),   // Write data ready
    .S_AXI_BRESP(),               // Write response
    .S_AXI_BVALID(),              // Write response valid
    .S_AXI_BREADY(),              // Write response ready
    .S_AXI_RDATA(),               // Read data
    .S_AXI_RRESP(),               // Read response
    .S_AXI_RVALID(),              // Read data valid
    .S_AXI_RREADY()               // Read data ready
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        // Reset state machine and registers
        state <= IDLE;
        address_reg <= 32'd0;
        data_reg <= 32'd0;
    end else begin
        case (state)
            IDLE:
                begin
                    // Wait for valid address
                    if (axi_awvalid) begin
                        address_reg <= axi_awaddr;
                        state <= WAIT_FOR_ADDRESS;
                    end
                end
            WAIT_FOR_ADDRESS:
                begin
                    // Wait for valid write data
                    if (axi_wvalid) begin
                        data_reg <= axi_wdata;
                        state <= WAIT_FOR_WRITE;
                    end
                end
            WAIT_FOR_WRITE:
                begin
                    // Write data to ARM memory
                    axi_addr <= address_reg;
                    axi_wdata <= data_reg;
                    axi_awvalid <= 1'b1;
                    axi_wvalid <= 1'b1;

                    // Check if write transaction is complete
                    if (axi_bready_out) begin
                        $display("Write to address %h with data %h successful", address_reg, data_reg);
                        state <= IDLE;
                    end
                end
        endcase
    end
end

endmodule
