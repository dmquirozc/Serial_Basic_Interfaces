`timescale 1ns/1ps
/****************************************************************************
 * General purpose I2C module.
 * Created by: Damian Quiroz
 * damian.quiroz.13 <at> sansano.usm.cl
 * Date: 02/02/2019
 * 
 * Master reference code:
 * https://www.digikey.com/eewiki/pages/viewpage.action?pageId=10125324
 * 
 * Module created with almost all features of a master-slave I2C communication
 * protocol.
 *
*****************************************************************************/
/* MODULE USAGE */

// i2c_basic 
//#(
//  .PULSE_WIDTH('d100)    
//)basic_i2c_module
//(
//     .clk                                 (),     // input clk,
//     .reset                               (),     // input reset,
//     .module_enable                       (),     // input module_enable,
//     .sda_line                            (),     // inout sda_line, //this pins must have the pullup property selected in the xdc file
//     .scl_line                            (),     // inout scl_line,
//     .master_slave_select                 (1'b0),     // input master_slave_select,
//     .read_write                          (),     // input read_write,
//     .enable_register_addressing          (),     // input enable_register_addressing,
//     .restart_after_write_register_addr   (),     // input restart_after_write_register_addr, not implemented
//     .start_transaction_pulse             (),     // input start_transaction_pulse,
//     .scl_line_frequency                  (),     // input [1:0] scl_line_frequency
//     .slave_address                       (),     // input [6:0] slave_address,
//     .bytes_to_read                       (),     // input [3:0] bytes_to_read, 
//     .bytes_to_write                      (),     // input [3:0] bytes_to_write,
//     .data_array_to_transmit              (),     // input [63:0] data_array_to_transmit,
//     .transmission_done_clear_flag        (),     // input transmission_done_clear_flag,
//     .transmission_done_flag              (),     // output logic transmission_done_flag,
//     .register_address                    (),		// input [7:0] register_address,
//     .data_array_received                 (),     // output logic [63:0] data_array_received,
//     .data_array_received_pulse_enable    (), 	// input data_array_received_pulse_enable,
//     .data_array_received_pulse           (),     // output logic data_array_received_pulse,
//     .data_array_sent_pulse_enable        (),     // input data_array_sent_pulse_enable
//     .data_array_sent_pulse               (),     // output logic data_array_sent_pulse
//     .nack_received_pulse_enable          (),     // input nack_received_pulse_enable
//     .nack_received_pulse                 (),     // output nack_received_pulse
//     .ack_received_pulse_enable           (),     
//     .on_transmit_ack                     (),     // output logic on_transmit_ack
//     .on_reception_ack                    ()		// output logic on_reception_ack
// );

/****************************************************************************
 * i2c_basic
 * input clk : master clock (default 100MHZ)
 * input reset: master reset.
 * input enable: enables the module.
 * 					0: disabled
 * 					1: enabled
 * inout sda_line : serial data bus of I2C protocol
 * inout scl_line : serial clock line of I2C protocol
 * input master_slave_select: selection of master mode or slave mode
 * 					0: Master mode
 *   				1: Slave mode (not implemented)
 * input read_write: read-write operation (only master mode)
 * 					0: write operation
 * 					1: read operation
 * input enable_register_addressing : some devices need to read a register before read, this flag has that purpose
 * input restart_after_write_register_addr: some devices neet to send a restart condition to read  data, this flag has that purpose
 * input start_transaction_pulse: pulse to start transmission (only master mode)
 * input scl_line_frequency: clock configuration register (only master mode)
 * 					00: 100Khz
 * 					01: 400Khz
 * 					10: 1Mhz
 * 					11: 3.8Mhz
 * input [6:0]slave_address: Slave address to send data.            
 * input [6:0]self_address: Slave mode address, default 7hXX (may be any number between 7h00 and 7h47)
 * input [2:0]bytes_to_read: bytes to read in master-read operation mode
 * input [2:0]bytes_to_write: bytes to write in master-write operation mode
 * input [7:0] register_address: register to read or write in a RW operation
 * input [7:0] data_array_to_transmit: data to transmit via SDA bus
 * input transmission_done_flag_clear: clear the transsmision done flag
 * output transmission_done_flag  : set when one transmission end
 * output [55:0]data_array_received: data received from SDA bus
 * input data_array_received_pulse_enable: enables the pulse of data received.
 * output data_array_received_pulse: pulse on data array received.
 * input data_array_sent_pulse_enable: enables the pulse of data sent.
 * output data_array_sent_pulse: pulse when data array is sent
 * input nack_received_pulse_enable: enables pulse on nack signal.
 * output nack_received_pulse:  pulses on nack signal received.
 * input ack_received_pulse_enable  : enables ack pulses.
 * output on_transmit_ack : ack received by slave pulse
 * output on_reception_ack : ack sended by master pulse

 * parameter PULSE_WIDTH : Cycles of clock of the pulse signals.
*******************************************************************************/
module i2c_basic
    #(
        parameter PULSE_WIDTH = 'd100
    )(
	input clk,
    input reset,
	input module_enable,
	inout sda_line, //this pins must have the pullup property selected in the xdc file
	inout scl_line,
	input master_slave_select,
    input read_write,
    input enable_register_addressing,
    input restart_after_write_register_addr,
    //input stretch,
    input start_transaction_pulse,
    input [1:0] scl_line_frequency,
    input [6:0] slave_address,
    //input [6:0] self_address,
    input [3:0] bytes_to_read,
    input [3:0] bytes_to_write,
    input [7:0] register_address,
	input [63:0] data_array_to_transmit, 
	input transmission_done_clear_flag,
	output logic transmission_done_flag,
	output logic [63:0] data_array_received,
	input data_array_received_pulse_enable,
	output logic data_array_received_pulse,
    input data_array_sent_pulse_enable,
    output logic data_array_sent_pulse,
    input nack_received_pulse_enable,
    output logic nack_received_pulse,
	// output tick_debug,
 //    output tick_debug2,
 //    output tick_debug3,
 //    output tick_debug4,
 //    output tick_debug5,
    input ack_received_pulse_enable,
    output logic on_transmit_ack,
    output logic on_reception_ack
    );

	/* Finite State Machine states
	 * ***********************************************************************************************
     * ***********************************************************************************************
     */
	localparam IDLE            = 'b0000;
	localparam MASTER_START    = 'b0001;
    localparam SLAVE_START     = 'b0010;
    localparam MASTER_COMMAND  = 'b0011;
    localparam SLAVE_COMMAND   = 'b0100;
    localparam MASTER_ACK      = 'b0101;
    localparam SLAVE_ACK       = 'b0110;
    localparam MASTER_WRITE    = 'b0111;
    localparam SLAVE_WRITE     = 'b1000;
    localparam MASTER_READ     = 'b1001;
    localparam SLAVE_READ      = 'b1010;
    localparam STOP            = 'b1011;
    
    /* General purpose params 
     * ***********************************************************************************************
     * ***********************************************************************************************
     */

     localparam PULSE_LENGTH = $clog2(PULSE_WIDTH)-1;

    localparam MASTER_MODE  = 'b0;
    localparam SLAVE_MODE   = 'b1;
    localparam WRITE 		= 'b0;
    localparam READ 		= 'b1;
    localparam ACK          = 'b0;
    localparam NACK         = 'b1;
    localparam OVERSAMPLING = 'd4;


    /* General purpose registers 
     * ***********************************************************************************************
     * ***********************************************************************************************
     */

     logic scl_read,scl_reg;
     logic sda_read,sda_reg;

     always_ff@(posedge clk) begin
        scl_read <= scl_line;
        sda_read <= sda_line;
     end

    /* master-slave mode selection */
    logic [3:0] byte_write_counter,byte_write_counter_next;
    logic [3:0] byte_read_counter,byte_read_counter_next;
    logic [63:0] data_received_next;
    logic [7:0] data_array_to_transmit_true,data_array_to_transmit_true_next;
    logic [3:0] clock_cycle_counter,clock_cycle_counter_next;
    logic transmission_done_next;
    logic[1:0] command_state_counter,command_state_counter_next;

    always_comb begin
    	data_array_to_transmit_true_next = (state == MASTER_COMMAND)? {slave_address[6:0],read_write}:
                                     (state == MASTER_WRITE && enable_register_addressing && byte_write_counter == 'd0)? register_address:
                                     (state == MASTER_WRITE && enable_register_addressing && byte_write_counter == 'd1)? data_array_to_transmit[7:0]:
                                     (state == MASTER_WRITE && enable_register_addressing && byte_write_counter == 'd2)? data_array_to_transmit[15:8]:
                                     (state == MASTER_WRITE && enable_register_addressing && byte_write_counter == 'd3)? data_array_to_transmit[23:16]:
                                     (state == MASTER_WRITE && enable_register_addressing && byte_write_counter == 'd4)? data_array_to_transmit[31:24]:
                                     (state == MASTER_WRITE && enable_register_addressing && byte_write_counter == 'd5)? data_array_to_transmit[39:32]:
                                     (state == MASTER_WRITE && enable_register_addressing && byte_write_counter == 'd6)? data_array_to_transmit[47:40]:
                                     (state == MASTER_WRITE && enable_register_addressing && byte_write_counter == 'd7)? data_array_to_transmit[55:48]:
                                     (state == MASTER_WRITE && enable_register_addressing && byte_write_counter == 'd8)? data_array_to_transmit[63:56]:
                                     (state == MASTER_WRITE && ~ enable_register_addressing && byte_write_counter == 'd0)? data_array_to_transmit[7:0]:
                                     (state == MASTER_WRITE && ~ enable_register_addressing && byte_write_counter == 'd1)? data_array_to_transmit[15:8]:
                                     (state == MASTER_WRITE && ~ enable_register_addressing && byte_write_counter == 'd2)? data_array_to_transmit[23:16]:
                                     (state == MASTER_WRITE && ~ enable_register_addressing && byte_write_counter == 'd3)? data_array_to_transmit[31:24]:
                                     (state == MASTER_WRITE && ~ enable_register_addressing && byte_write_counter == 'd4)? data_array_to_transmit[39:32]:
                                     (state == MASTER_WRITE && ~ enable_register_addressing && byte_write_counter == 'd5)? data_array_to_transmit[47:40]:
                                     (state == MASTER_WRITE && ~ enable_register_addressing && byte_write_counter == 'd6)? data_array_to_transmit[55:48]:
                                     (state == MASTER_WRITE && ~ enable_register_addressing && byte_write_counter == 'd7)? data_array_to_transmit[63:56]:
                                     (state == MASTER_WRITE && ~ enable_register_addressing && byte_write_counter == 'd8)? 'd0:
                                     data_array_to_transmit[7:0];

        transmission_done_next = (state == STOP)? 1'b1:
        						 (transmission_done_clear_flag)? 1'b0: transmission_done_flag;
    end
    
    logic[PULSE_LENGTH:0]transmission_ack_pulse_counter, transmission_ack_pulse_counter_next;
    logic[PULSE_LENGTH:0]reception_ack_pulse_counter, reception_ack_pulse_counter_next;
    logic[PULSE_LENGTH:0]nack_received_pulse_counter, nack_received_pulse_counter_next;
    logic[PULSE_LENGTH:0]data_array_sent_pulse_counter, data_array_sent_pulse_counter_next;
    logic[PULSE_LENGTH:0]data_array_received_pulse_counter, data_array_received_pulse_counter_next;



    always_ff@(posedge clk) begin
    	if(reset) begin
    		transmission_done_flag <= 'd0;
    		data_array_to_transmit_true <= 'd0;
    	end else begin
    		transmission_done_flag <= transmission_done_next;
    		data_array_to_transmit_true <= data_array_to_transmit_true_next;
    	end 
    end
    /* Generate the {address,R/W} bit array in master first byte */

    logic on_transmit_ack_trigg, on_reception_ack_trigg;
    logic nack_received_pulse_trigg;
    assign on_transmit_ack_trigg = (ack_received_pulse_enable)? (state == SLAVE_ACK && clock_cycle_counter == 'd4 && sda_read == ACK):'b0;
    assign on_reception_ack_trigg = (ack_received_pulse_enable)? (state == MASTER_ACK && clock_cycle_counter == 'd4 && sda_read == ACK):'b0;

    always_comb begin
        if(on_transmit_ack_trigg)
            transmission_ack_pulse_counter_next = transmission_ack_pulse_counter + 'd1;
        else if(transmission_ack_pulse_counter == PULSE_WIDTH)
            transmission_ack_pulse_counter_next = 'd0;
        else if(transmission_ack_pulse_counter > 'd0)
            transmission_ack_pulse_counter_next = transmission_ack_pulse_counter + 'd1;
        else
            transmission_ack_pulse_counter_next = transmission_ack_pulse_counter;
    end
    
    always_comb begin
        if(on_reception_ack_trigg)
            reception_ack_pulse_counter_next = reception_ack_pulse_counter + 'd1;
        else if(reception_ack_pulse_counter == PULSE_WIDTH)
            reception_ack_pulse_counter_next = 'd0;
        else if(reception_ack_pulse_counter > 'd0)
            reception_ack_pulse_counter_next = reception_ack_pulse_counter + 'd1;
        else 
            reception_ack_pulse_counter_next = reception_ack_pulse_counter;
    end

    always_comb begin
        if(nack_received_pulse_trigg)
            nack_received_pulse_counter_next = nack_received_pulse_counter + 'd1;
        else if( nack_received_pulse_counter == PULSE_WIDTH)
            nack_received_pulse_counter_next = 'd0;
        else if(nack_received_pulse_counter > 'd0)
            nack_received_pulse_counter_next = nack_received_pulse_counter + 'd1;
        else
            nack_received_pulse_counter_next = nack_received_pulse_counter; 
    end

    always_ff@(posedge clk) begin
        if(reset) begin
            reception_ack_pulse_counter <= 'd0;
            transmission_ack_pulse_counter <= 'd0;
            nack_received_pulse_counter <= 'd0;
        end else begin
            reception_ack_pulse_counter <= reception_ack_pulse_counter_next;
            transmission_ack_pulse_counter <= transmission_ack_pulse_counter_next;
            nack_received_pulse_counter <= nack_received_pulse_counter_next;
        end
    end
    
    assign on_reception_ack = (ack_received_pulse_enable)? (reception_ack_pulse_counter > 'd0):'b0;
    assign on_transmit_ack = (ack_received_pulse_enable)? (transmission_ack_pulse_counter > 'd0):'b0;
    assign nack_received_pulse = (nack_received_pulse_enable)?(nack_received_pulse_counter > 'd0):'b0;
    /* General purpose negedge and posedge detector 
     * ***********************************************************************************************
     * ***********************************************************************************************
     */
    logic[1:0] sda_negedge_register;
    logic sda_detect_negedge;
    logic sda_detect_posedge;
    logic sda_stop_detector;
    logic stretch_flag;

    always_ff@(posedge clk) begin
        if(reset) 
            sda_negedge_register <= 'd0;
        else 
            sda_negedge_register <=  {sda_negedge_register[0],sda_read};
    end

    assign sda_detect_negedge = ((sda_negedge_register[1] == 'b1) && (sda_negedge_register[0] == 'b0));
    assign sda_detect_posedge = ((sda_negedge_register[1] == 'b0) && (sda_negedge_register[0] == 'b1));
    assign sda_stop_detector =  (sda_detect_posedge == 'b1) && (scl_read == 'b1); //STOP CONDITION DETECTOR
   
    /* FSM general registers 
     * ***********************************************************************************************
     * ***********************************************************************************************
     */
    logic [3:0] state,state_next;
    logic [3:0] bit_counter,bit_counter_next;
    

    logic slave_read_write;

    logic [4:0] start_cycle_counter,start_cycle_counter_next;

    always_comb begin
        if(state == MASTER_START)
            start_cycle_counter_next = start_cycle_counter + 'd1;
        else
            start_cycle_counter_next = start_cycle_counter; 
    end
    
    always_ff@(posedge clk)begin
        if(reset)
            start_cycle_counter <= 'd0;
        else if(state == MASTER_START)
            start_cycle_counter <= start_cycle_counter_next;
        else
            start_cycle_counter <='d0;

    end

    /* Command state counter  logic */
    always_comb begin
        command_state_counter_next = (state == MASTER_START && command_state_counter == 2'd0)? command_state_counter + 2'd1:
                                     (state == MASTER_START && command_state_counter == 2'd1)? command_state_counter + 2'd1: // to avoid repetitive restart condition
                                     (state == IDLE || state == STOP)? 'd0:command_state_counter;
    end

    always_ff@(posedge clk) begin
        if(reset) begin
            command_state_counter <= 2'd0;
        end else begin
            command_state_counter <= command_state_counter_next;
        end
    end
    /* Finite State Machine for master/slave mode
     * ***********************************************************************************************
     * ***********************************************************************************************
     */
     
    always_comb begin
    	state_next = state;
    	stretch_flag = 1'b0;
        nack_received_pulse_trigg = 1'b0;
    	case(state) 
    		IDLE:
            begin
                if(master_slave_select == MASTER_MODE) begin 
                    state_next = (start_transaction_pulse && module_enable == 1'b1) ? MASTER_START: IDLE; //master start condition
                end else begin
                    state_next = IDLE; //slave not implemented
                end
            end
            MASTER_START:
            begin
                state_next = (module_enable == 1'b1 && restart_after_write_register_addr && command_state_counter == 2'd1 && start_cycle_counter == 5'd20 && read_write == WRITE)? MASTER_WRITE:
                             (module_enable == 1'b1 && restart_after_write_register_addr && command_state_counter == 2'd1 && start_cycle_counter == 5'd20 && read_write == READ)? MASTER_READ:
                             (module_enable == 1'b1 && start_cycle_counter == 5'd20)? MASTER_COMMAND:
                             (module_enable == 1'b1 && start_cycle_counter < 5'd20)? MASTER_START: IDLE; // add 20 cycles of clock to start sending commands
            end
            MASTER_COMMAND:
            begin
                if(bit_counter == 'd9 && clock_cycle_counter =='d1) begin //count from 0 to 7   
                    state_next = SLAVE_ACK;//waits for a slave ack
                end else begin
                    state_next = MASTER_COMMAND;
                end 
            end 
            MASTER_ACK:begin //begins when master send a read operation and slave respond
                if(master_slave_select == MASTER_MODE) begin
                	if(bit_counter == 'd10 && clock_cycle_counter == 1'd1)
	                    state_next = (byte_read_counter == bytes_to_read && start_transaction_pulse && module_enable == 1'b1)? MASTER_START:
	                    			 (byte_read_counter == bytes_to_read && module_enable == 1'b0)? STOP: 
	                    			 (byte_read_counter == bytes_to_read)? STOP: 
                                     (module_enable == 1'b0)? STOP:MASTER_READ;
                end else begin //slave behaviour not implemented
                    state_next = STOP;
                end

            end
            SLAVE_ACK:
            begin //begins when master sends first command data
                if(master_slave_select == MASTER_MODE) 
                begin //behaviour depends of operation mode
                    if(clock_cycle_counter >= 4'd3 && bit_counter == 4'd9) 
                    begin // clock stretching  detected
                       state_next = (module_enable == 1'b1 && scl_read == 1'b0)? SLAVE_ACK:
                                    (module_enable == 1'b1 && scl_read == 1'b1)? SLAVE_ACK:STOP;
                       stretch_flag = (scl_read == 1'b0)? 1'b1:1'b0;
                    end else 
                    begin //clock stretching not detected 
                         if(clock_cycle_counter == 4'd1 && bit_counter == 4'd10) 
                         begin
                            nack_received_pulse_trigg = (nack_received_pulse_enable && sda_read == NACK)? 1'b1:1'b0;
                            state_next = (sda_read == ACK && enable_register_addressing == 4'b1 && read_write == READ && ~command_sended)? MASTER_WRITE: 
                                     (sda_read == ACK && restart_after_write_register_addr == 4'b1 && command_state_counter == 4'd1)? MASTER_START:
                                     (sda_read == ACK && byte_write_counter == bytes_to_write && read_write == WRITE)? STOP:
                                     (sda_read == ACK && start_transaction_pulse == 4'b0 && read_write == WRITE)? MASTER_WRITE:
                                     (sda_read == ACK && start_transaction_pulse == 4'b0 && read_write == READ)? MASTER_READ:
                                     (sda_read == NACK && start_transaction_pulse == 4'b1)? MASTER_START:
                                     (sda_read == NACK && start_transaction_pulse == 4'b0)? STOP:
                                     (module_enable == 4'b0)? STOP:
                                     SLAVE_ACK;
                        end else 
                        begin 
                            state_next = SLAVE_ACK;
                        end
                    end //clock stretching wait
                end else begin //slave behaviour not implemented
                    state_next = STOP;
                end
            end
            MASTER_WRITE:begin
                if(master_slave_select == MASTER_MODE) begin
                    if(bit_counter == 4'd9 && clock_cycle_counter ==4'd1) begin //count from 0 to 7   
                    	state_next = SLAVE_ACK;//waits for a slave ack
                	end else begin
                    	state_next = MASTER_WRITE;
                	end 
                end else begin //slave behaviour not implemented
                    state_next = STOP;
                end
            end
            MASTER_READ:begin
                if(master_slave_select == MASTER_MODE) begin
                    if(bit_counter == 4'd9 && clock_cycle_counter =='d1) begin //count from 0 to 7   
                    	state_next = MASTER_ACK;//waits for a slave ack
                	end else begin
                    	state_next = MASTER_READ;
                	end 
                end else begin //slave behaviour not implemented
                    state_next = STOP;
                end
            end
            STOP:begin
                state_next = (clock_cycle_counter == 'd4)? IDLE:STOP;
            end
    	endcase // state
    end

    always_ff@(posedge clk) begin
    	if(reset)
    		state <= IDLE;
    	else
    		state <= state_next;
    end

        /* master mode clock generator, oversampling (4) 
 	 * ***********************************************************************************************
     * ***********************************************************************************************
     */

    logic clk_enable,clk_enable_next, scl_clock_oversampled;
    logic scl_clock_gen;
    i2c_clock_divider i2c_master_clock(
            .clk_in(clk),
            .reset(clk_enable),
            .clock_config(scl_line_frequency),
            .clk_out(scl_clock_gen),
            .clk_oversampled(scl_clock_oversampled) //oversampling 4 by default
        );

    logic scl_clock_wire;
    scl_clock_wire = (scl_clock_gen)? 'bz:'b0;

    // assign tick_debug = (state == MASTER_ACK); //&& (bit_counter == 'd9 && clock_cycle_counter == 'd0 && scl_clock_oversampled_posedge_detect);
    // assign tick_debug2 = (state == MASTER_READ);
    // assign tick_debug3 = (bytes_to_read[0]);
    // assign tick_debug4 = (bytes_to_read[1]);
    // assign tick_debug5 = (byte_counter < bytes_to_read);
    logic[1:0] scl_clock_posedge,scl_clock_oversampled_posedge;
    logic scl_clock_posedge_detect,scl_clock_oversampled_posedge_detect;


    always_ff@(posedge clk)
        if(reset) begin
        	scl_clock_oversampled_posedge = 'd0;
            scl_clock_posedge = 'd0;
        end else begin 
            scl_clock_posedge = {scl_clock_posedge[0],scl_clock_gen};
            scl_clock_oversampled_posedge = {scl_clock_oversampled_posedge[0],scl_clock_oversampled};
        end 

    assign scl_clock_posedge_detect = scl_clock_posedge[0] && ~scl_clock_posedge[1];
    assign scl_clock_oversampled_posedge_detect = scl_clock_oversampled_posedge[0] && ~ scl_clock_oversampled_posedge[1];
    /* master clock_cycle_counter and bit counter
     * ***********************************************************************************************
     * ***********************************************************************************************
     */
     always_comb 
     begin
        if(clock_cycle_counter == 'd4)
            clock_cycle_counter_next = 'd0; 
        else if(scl_clock_oversampled_posedge_detect) 
            clock_cycle_counter_next = clock_cycle_counter + 4'd1;
        else 
            clock_cycle_counter_next = clock_cycle_counter;
     end

    always_ff@(posedge clk) begin //4 cycle counter for i2c clock
        if(reset)
            clock_cycle_counter = 4'd0;
        else
            clock_cycle_counter = clock_cycle_counter_next;
    end
    

    /* bit counter behaviour 
     * ***********************************************************************************************
     * ***********************************************************************************************
     */

    always_comb 
    begin
        if(state == IDLE || state == STOP)
            bit_counter_next  = 4'd0;
        else if(state == SLAVE_ACK  && bit_counter == 'd10 && clock_cycle_counter == 'd1)
            bit_counter_next = 4'd1;
        else if(state == MASTER_ACK && bit_counter == 'd10 && clock_cycle_counter == 'd1)
            bit_counter_next = 4'd1;
        else if(stretch_flag)
            bit_counter_next = 4'd9;
        else if((clock_cycle_counter == 'd0) && (scl_clock_oversampled_posedge_detect)) 
            bit_counter_next = bit_counter +'d1;
        else 
            bit_counter_next = bit_counter;
    end 

    always_ff@(posedge clk)
    begin
        if(reset)
            bit_counter <= 4'd0;
        else
            bit_counter <= bit_counter_next;
    end 

    /* General behaviour of State Machine 
     * ***********************************************************************************************
     * ***********************************************************************************************
     */

    logic sda_next,scl_next;
    logic command_sended,command_sended_next;
    always_comb begin //scl and sda general behaviour
        sda_next = 'bz;//high impedance default state
        scl_next = 'bz;//high impedance default state
        clk_enable_next = 'd1; //clock generator reset state
        data_received_next = data_array_received; 
        byte_write_counter_next = byte_write_counter;
        byte_read_counter_next = byte_read_counter;
        data_array_received_pulse = 1'b0;
        data_array_sent_pulse = 1'b0;
        command_sended_next = command_sended;
        case(state)
            IDLE: begin // with pull up resistors, line is always high 
                byte_write_counter_next = 'd0;
                byte_read_counter_next = 'd0;
                clk_enable_next = 'd1;
                command_sended_next = 'd0;
            end              
            MASTER_START: begin //send start condition to slave
                sda_next = 'b0;
                clk_enable_next = 'd1;
                byte_write_counter_next = 'd0;
                byte_read_counter_next = 'd0;
            end         
            MASTER_COMMAND:begin //scl connected to clock generator 
                clk_enable_next = 'b0; //clock generator start at low pulse
                scl_next = scl_clock_wire;
                /* Just TOC signal generation*/
                if(bit_counter == 'd0)
                	sda_next = 'd0;
                else if(bit_counter == 'd9)
               		sda_next = data_array_to_transmit_true[0];
                else
                    sda_next = data_array_to_transmit_true['d8-bit_counter];//bit counter changes with clock generator

                /* First data must be [7:1] address, [0] R/W bit*/
            end       
            MASTER_ACK:begin
                clk_enable_next = 'd0;
                scl_next = scl_clock_wire;
                if(master_slave_select == MASTER_MODE) begin
                    sda_next = (byte_read_counter < bytes_to_read)? ACK:NACK;
                    data_array_received_pulse = (data_array_received_pulse_enable && byte_read_counter == bytes_to_read)? 1'b1:1'b0;
                end 
            end    
            SLAVE_ACK:begin //sda and scl return to Z state 
            	
                if(master_slave_select == MASTER_MODE) begin
                    sda_next = 'bz;
                    data_array_sent_pulse = (data_array_sent_pulse_enable && byte_write_counter == bytes_to_write)? 1'b1:1'b0;
                    clk_enable_next = 'b0;
                    if(bit_counter  == 'd9) begin
                    	if(stretch_flag)
                    		scl_next = 'bz;
                    	else
                    		scl_next = scl_clock_wire;
                   	end else begin 
                   		scl_next = scl_clock_wire;
                   	end 
                    
                end
            end  
            MASTER_WRITE:begin
                scl_next = scl_clock_wire;
                clk_enable_next = 'b0;
                command_sended_next = (bit_counter == 'd9)? 'd1:command_sended;
                byte_write_counter_next = (bit_counter == 'd8 && clock_cycle_counter == 'd0 && scl_clock_oversampled_posedge_detect)? byte_write_counter +'d1:byte_write_counter;
            	if(bit_counter == 'd9)
               		sda_next = data_array_to_transmit_true[0];
                else
                    sda_next = data_array_to_transmit_true['d8-bit_counter];//bit counter changes with clock generator

                
            end
            MASTER_READ:begin
                clk_enable_next = 'b0;
                scl_next = scl_clock_wire;
                data_received_next = (clock_cycle_counter == 'd2 &&  scl_clock_oversampled_posedge_detect)? {data_array_received[62:0],sda_read}:data_array_received;
                byte_read_counter_next = (bit_counter == 'd8 && clock_cycle_counter == 'd0 && scl_clock_oversampled_posedge_detect)? byte_read_counter +'d1:byte_read_counter;
            end  
            STOP:begin
                if(master_slave_select == MASTER_MODE) begin 
                    clk_enable_next = 'b0;
                    command_sended_next = 'd0;
                    scl_next = (clock_cycle_counter < ('d2))? scl_clock_wire:'bz;
                    sda_next = (clock_cycle_counter < ('d3))? 'b0:'bz; //generate posedge
                end
            end    
        endcase
    end

	/* sda and scl registers update 
	 * ***********************************************************************************************
     * ***********************************************************************************************
     */
    //logic sda_reg,scl_reg;
    always_ff@(posedge clk) begin
        if(reset)begin
            clk_enable <= 'b1;
            data_array_received <= 'b0;
            command_sended <= 'd0;
            sda_reg <= 'bz;
            scl_reg <= 'bZ;
            byte_write_counter <= 'd0;
            byte_read_counter <= 'd0;
        end else begin
            clk_enable <= clk_enable_next;
            data_array_received <= data_received_next;
            command_sended <= command_sended_next;
            sda_reg <= sda_next;
            scl_reg <= scl_next;
            byte_write_counter <= byte_write_counter_next;
            byte_read_counter <= byte_read_counter_next;
        end
    end

    /* SDA and SCL signal assignment 
     * ***********************************************************************************************
     * ***********************************************************************************************
     */
    assign sda_line = (state == SLAVE_ACK || state == MASTER_READ || state == IDLE || (state == STOP &&clock_cycle_counter >= 'd3))? 'bz: sda_reg;
    assign scl_line = (state == SLAVE_ACK || state == IDLE || state == MASTER_START || (state == STOP && clock_cycle_counter >='d2))? 'bz: scl_reg;


endmodule //i2c_basic


/*
 * clock_divider.sv
 * 2017/04/17 - Felipe Veas <felipe.veasv [at] usm.cl>
 * Modificado por Damian Quiroz.
 * damian.quiroz.13 <at> sansano.usm.cl
 * 
 * Divisor de reloj basado en un contador para una frecuencia de entrada
 * de MASTER_FREQ [MHz] (default 100MHz)
 * 
 * Rango de operación:
 *     1 <= clk_out <= MASTER_FREQ/2 [Hz]
 * Modificado para funcionar con oversampling de 4 ciclos de reloj, 
 * disenhado exclusivamente para funcionar con el modulo i2c_basic
 */
 
/********************************************************* 
 * clock_divider                           
 * input clk_in: master clock.
 * input reset: master_reset active high
 * input out_freq: frecuencia de salida deseada.
 * output clk_out: reloj generado (reset state low)
 * output clk_oversampled : reloj con un oversampling de 4
 * MASTER_FREQ : master clock
 * OVERSAMPLING: sobre muestreo deseado (default 4)
*
**********************************************************/
module i2c_clock_divider
#(
	parameter MASTER_FREQ = 100_000_000
)(
	input clk_in,
	input reset,
	input [1:0] clock_config,
	output logic clk_out,  // sda clock
    output logic clk_oversampled  //oversampled clock
);
    localparam OVERSAMPLING = 4;
    logic tick_100k,tick_400k,tick_1M,tick_3_8M;
    logic clk_oversampled_next;
    logic clk_out_next;
    logic [1:0] counter,counter_next;
    always_comb begin
        clk_oversampled_next = tick_100k;
        case(clock_config)
            2'b00: clk_oversampled_next = tick_100k;
            2'b01: clk_oversampled_next = tick_400k;
            2'b10: clk_oversampled_next = tick_1M;
            2'b11: clk_oversampled_next = tick_3_8M;
        endcase // clock_config
    end

    always_comb begin
        if(clk_oversampled) 
        begin
            clk_out_next = clk_out;
            counter_next = counter + 'd1; 
        end else if(counter == 'd2) 
        begin
            counter_next = 'd0;
            clk_out_next = ~clk_out;
        end else 
        begin
            counter_next = counter;
            clk_out_next = clk_out;
        end
    end

    always_ff@(posedge clk_in) begin
        if(reset) begin
            counter <= 'd0;
            clk_out <= 'd0;
        end else begin
            counter <= counter_next;
            clk_out <= clk_out_next;
        end
    end


    always_ff@(posedge clk_in)
        if(reset)
            clk_oversampled <= 'd0;
        else 
            clk_oversampled <= clk_oversampled_next;

    baud_tick_gen #(
            .CLK_FREQUENCY(MASTER_FREQ),
            .BAUD_RATE(100000),
            .OVERSAMPLING(OVERSAMPLING)
        ) baud_tick_100k(
            .clk(clk_in),
            .enable(~reset),
            .tick(tick_100k)
    );

    baud_tick_gen #(
            .CLK_FREQUENCY(MASTER_FREQ),
            .BAUD_RATE(400000),
            .OVERSAMPLING(OVERSAMPLING)
        ) baud_tick_400k(
            .clk(clk_in),
            .enable(~reset),
            .tick(tick_400k)
    );

    baud_tick_gen #(
            .CLK_FREQUENCY(MASTER_FREQ),
            .BAUD_RATE(1000000),
            .OVERSAMPLING(OVERSAMPLING)
        ) baud_tick_1M(
            .clk(clk_in),
            .enable(~reset),
            .tick(tick_1M)
    );

    baud_tick_gen #(
            .CLK_FREQUENCY(MASTER_FREQ),
            .BAUD_RATE(3800000),
            .OVERSAMPLING(OVERSAMPLING)
        ) baud_tick_3_8M(
            .clk(clk_in),
            .enable(~reset),
            .tick(tick_3_8M)
    );

endmodule


/*
 * baud_tick_gen.sv
 * 2017/02/01 - Felipe Veas <felipe.veasv at usm.cl>
 *
 * Baud clock generator for UART, original code from:
 * http://fpga4fun.com/SerialInterface.html
 * 
 * Modificado para funcionar con este modulo
 */

module baud_tick_gen
#(
    parameter CLK_FREQUENCY = 25000000,
    parameter BAUD_RATE = 115200,
    parameter OVERSAMPLING = 1
)(
    input clk,
    input enable,
    output tick
);

    function integer clog2;
        input integer value;
        begin
            value = value - 1;
            for (clog2 = 0; value > 0; clog2 = clog2 + 1)
                value = value >> 1;
        end
    endfunction

    localparam ACC_WIDTH = clog2(CLK_FREQUENCY / BAUD_RATE) + 8;
    localparam SHIFT_LIMITER = clog2(BAUD_RATE * OVERSAMPLING >> (31 - ACC_WIDTH));
    localparam INCREMENT =
            ((BAUD_RATE * OVERSAMPLING << (ACC_WIDTH - SHIFT_LIMITER)) +
            (CLK_FREQUENCY >> (SHIFT_LIMITER + 1))) / (CLK_FREQUENCY >> SHIFT_LIMITER);

    (* keep = "true" *)
    logic [ACC_WIDTH:0] acc = 0;

    always_ff@(posedge clk)
        if(enable)
            acc <= acc[ACC_WIDTH-1:0] + INCREMENT[ACC_WIDTH:0];
        else
            acc <= INCREMENT[ACC_WIDTH:0];

    assign tick = acc[ACC_WIDTH];

endmodule //uart_baud_tick_gen
