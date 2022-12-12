`timescale 1ns / 1ps
/*********************************************************************
 *
 * created by: Damian Quiroz 
 * damian.quiroz.13 <at> sansano.usm.cl
 * 
 * spi_basic.sv
 * Basic implementation fo SPI module with configuration register. 
 * This module works only in 4 wire master architecture, 3 wire architecture 
 * is not implemented.
 * 
 * Used SPI standard Specifications of:
 * 
 * https://www.nxp.com/files-static/microcontrollers/doc/ref_manual/S12SPIV3.pdf
 * 
 * and 
 * 
 * http://www.ti.com/lit/ug/sprugp2a/sprugp2a.pdf
**********************************************************************/
/**
 * USAGE of the Module
 */

 // spi_basic 
    // #(
    //  .PULSE_WIDTH('d100)
    // )basic_spi_module 
    // (
    // .clk                                        (),         // input clk,
    // .reset                                      (),         // input reset,
    // .enable                                     (),		   // input enable,
    // .master_slave_select                        (1'b1),     // input master_slave_select,
    // .start_pulse                                (),         // input start_pulse,
    // .enable_command_send                        (),         // input enable_command_send,
    // .command_to_send                            (),         // input[7:0] command_to_send,
    // .enable_chip_select_setup_time              (),         // input enable_chip_select_setup_time,
    // .chip_select_setup_time                     (),         // input [3:0]chip_select_setup_time,
    // .cpol                                       (),         // input cpol,
    // .cpha                                       (),         // input cpha,
    // .data_length_selection                      (),         // input [2:0] data_length_selection,
    // .data_packets                               (),         // input [2:0] data_packets,
    // .freq_divider                               (),         // input [15:0] freq_divider, numeric value of MASTER_CLOCK/(10 + freq_divider)
    // .wait_time                                  (),         // input [3:0] wait_time, SCLK*(wait_time +'d1) cycles of clock between transmissions
    // .msb_lsb_select                             (),         // input msb_lsb_select,
    // .four_wire_selection                        (),         // input four_wire_selection,
    // .wait_time_chip_select_polarity             (),         // input wait_time_chip_select_polarity,
    // .sclk                                       (),         // output sclk,
    // .mosi                                       (),         // output logic mosi,
    // .miso                                       (),         // input logic miso,
    // .chip_select                                (),         // output logic chip_select,
     // .data_to_transmit                           (),         // input [63:0]data_to_transmit,
     // .data_received_buffer                       (),         // output logic [63:0]data_received, store 2 data of 32 bytes.
     // .data_received_done_pulse_enable            (),         // input data_received_done_pulse_enable,
    // .data_received_done_pulse                   (),         // output data_received_done_pulse,
    // .transmission_done_flag_clear               (),         // input transmission_done_flag_clear,
    // .transmission_done_pulse_enable             (),         // input transmission_done_pulse_enable,
    // .transmission_done_flag                     (),         // output transmission_done_flag,
    // .transmission_done_pulse                    (),         // output transmission_done_pulse,
    // );
    

/********************************************************************
 * spi_basic 
 * input clk: master clock.
 * input reset: master reset
 * input enable: module enable signal.
 * input start_pulse : pulse to start a transmission.
 * input master_slave_select: 1 bit (0-> master mode, 1-> slave mode) // slave mode not implemented.
 * input enable_command_send: some devices need a command to start a transaction, this signal enables command sending
 * input command_to_send	: command data to send 
 * input enable_chip_select_setup_time: enable chip select setup time for slow devices
 * input chip_select_setup_time: SCLK cycles of clock before start to send data.
                        ONLY WHEN enable_chip_select_setup signal IS ACTIVE.

                        CSST = SCLK*(2 + chip_select_setup_time)

 * input cpol: 1 bit . Module data read polarity depends on cpha bit.
 * input cpha: 1 bit . Module data read phase, depends on cpol bit
 * 			cpol	cpha	operation behaviour
 * 			_______________________________________________________________________
 * 			0		0		Polarity 0 and data is output on rising edge of clock.
 * 			 				Data input is readed in falling edge of clock.
 * 			______________________________________________________________________
 * 			0		1		Polarity 0 and data is output on falling edge of clock.
 * 							First data bit is sent at the beginning of first cycle
 * 							of clock.
 * 							Data input is readed in rising edge of clock.
 * 			______________________________________________________________________
 * 			1		0		Polarity 1 and data is output on falling edge of clock.
 * 							Data input is readed in risign edge of clock.
 * 			______________________________________________________________________
 * 			1		1		Polarity 1 and data is output on rising edge of clock.
 * 							First data bit is sent at the beginning of first cycle
 * 							of clock.
 * 							Dat input is readed ing falling edge of clock.
 * 
 * input data_length_selection : configure date length with one of 8 possible values:
 *                              'b000: 4 bits
 *                              'b001: 8 bits
 *                              'b010: 12 bits
 *                              'b011: 16 bits
 *                              'b100: 20 bits
 *                              'b101: 24 bits
 *                              'b110: 28 bits
 *                              'b111: 32 bits
 * input data_packets:      configure how many packets of data will be send
 *                              
 * freq_divider: Numeric value of divider number on equation:
 *                  sclk = MASTER_CLOCK/(10 + freq_divider )
 * 
 * input  wait_time: 4 bit. Selection of master clock cycles between 2 
 * 					transmissions (master mode)
 * 					
 * 				wait_time = (SCLK*(wait_time +1)
 * 			
 * input msb_lsb_select: 1 bit. Selection of MSB or LSB transmision.
 * 
 * 				0-> MSB first 
 * 				1-> LSB first
 * 
 * input four_wire_selection: 1 bit. Selection of 3 wire or 4 wire operation mode.
 * 
 * 					0-> 4 wire operation, 1-> 3 wire operation.
 * 					
 * input  wait_time_chip_select_polarity	: 1 bit. Selection of state chip_select value  between
 * 			 				  data transfer. 
 * 			  
 * 			  0-> chip_select must be 0 between data.
 * 			  1-> chip_select must be 1 between data.
 *
 * output s_clk: spi module clock, behaviour depends of module operation
 * 				(master or slave). In master mode s_clk is an output,
 * 				in slave mode s_clk is an input.
 * output mosi: spi module mosi bus.
 * input miso: spi module miso bus.
 * output chip_select: spi module chip_select behaviour depends of module
 * 					operation. In master mode chip_select is an output,
 * 					in slave mode chip_select is an input.
 * input data_to_transmit: spi data to transmit buffer.
 * outpu data_received_buffer: spi data received buffer.
 * input data_received_done_pulse_enable: 
 * input data_received_done_pulse		:
 * input transmission_done_flag_clear,
 * input transmission_done_pulse_enable,
 * output transmission_done_flag,
 * output transmission_done_pulse,

 * parameter PULSE_WIDTH: cycles of clock duration of pulse signals.
 ********************************************************************/

module spi_basic 
    #(
    parameter PULSE_WIDTH = 'd100
    )(
    input clk,
    input reset,
    input enable,
    input master_slave_select,
    input start_pulse,
    input enable_command_send,
    input [7:0] command_to_send,
    input enable_chip_select_setup_time,
    input [3:0]chip_select_setup_time,
    input cpol,
    input cpha,
    input [2:0] data_length_selection,
    input [3:0] data_packets,
    input [15:0] freq_divider,
    input [3:0] wait_time,
    input msb_lsb_select,
    input four_wire_selection,
    input wait_time_chip_select_polarity,
    output sclk,
    output logic mosi,
    input logic miso,
    output logic chip_select,
    input [63:0]data_to_transmit,
    output logic [63:0]data_received_buffer, // store 2 data of 32 bytes
    //output tick_debug,
    //output tick_debug2,
    //output tick_debug3,
    input data_received_done_pulse_enable,
    output data_received_done_pulse,
    input transmission_done_flag_clear,
    input transmission_done_pulse_enable,
    output transmission_done_flag,
    output transmission_done_pulse
    );

     /* General state machine parameters
     * ***********************************************************************************************
     * ***********************************************************************************************
     */
     localparam IDLE 		= 'b000;
     localparam START 		= 'b101;
     localparam SETUP       = 'b011;
     localparam COMMAND     = 'b111;
     localparam SEND_RECV 	= 'b001;//4 wire configuration
     localparam WAIT		= 'b100;
     localparam END 		= 'b110;

     localparam MASTER_MODE = 'b1;

     localparam PULSE_LENGTH = $clog2(PULSE_WIDTH)-1;

     /* Posedge and negedge detector for slave operation 
     * ***********************************************************************************************
     * ***********************************************************************************************
     */
     logic posedge_detect;
     logic negedge_detect;
     logic detect_shift;

     

     /* General purpose registers 
     * ***********************************************************************************************
     * ***********************************************************************************************
     */
     logic [5:0] data_length;
     assign data_length =   (data_length_selection == 'b000)? 'd4:
                            (data_length_selection == 'b001)? 'd8:
                            (data_length_selection == 'b010)? 'd12:
                            (data_length_selection == 'b011)? 'd16:
                            (data_length_selection == 'b100)? 'd20:
                            (data_length_selection == 'b101)? 'd24:
                            (data_length_selection == 'b110)? 'd28:
                            (data_length_selection == 'b111)? 'd32:'d8;


     logic[3:0]data_packets_limit;
     assign data_packets_limit = (data_length == 'd4 && data_packets >= 'd15)? 'd15:
                                 (data_length == 'd8 && data_packets >= 'd8)? 'd8:
                                 (data_length == 'd12 && data_packets >= 'd6)? 'd6:
                                 (data_length == 'd16 && data_packets >= 'd4)? 'd4:
                                 (data_length == 'd20 && data_packets >= 'd3)? 'd3:
                                 (data_length == 'd24 && data_packets >= 'd2)? 'd2:
                                 (data_length == 'd32 && data_packets >= 'd2)? 'd2:
                                 data_packets;

     logic[6:0]sclk_cont, sclk_cont_next;
     logic[5:0]mosi_clk_cont, mosi_clk_cont_next;
     logic[5:0]miso_clk_cont, miso_clk_cont_next;
     logic[31:0] data_received_next,data_received;
     logic[3:0] data_bytes_recv_cont,data_bytes_recv_cont_next;
     logic[3:0] data_bytes_send_cont,data_bytes_send_cont_next;
     logic[2:0]state,state_next;
     /* Read write flag: 0 -> read, 1 -> write (only slave mode)*/
     logic read_write_flag;

     always_ff@(posedge clk) begin
         detect_shift <= mclk_out;
     end

     assign posedge_detect = (~detect_shift && mclk_out);
     assign negedge_detect = (detect_shift && ~mclk_out);
     

     /* Finite State machine logic 
     * ***********************************************************************************************
     * ***********************************************************************************************
     */
     
     always_comb begin
         state_next = IDLE;
         case(state)
             IDLE: begin
                 if(master_slave_select == MASTER_MODE)
                 begin
                     state_next = (enable && start_pulse)? START:IDLE;
                 end /*else begin
                     state_next = (~enable)? IDLE:
                                  (chip_select)? IDLE:START; //4 wire
                 end*/
             end
             START: begin
                 state_next = (enable_chip_select_setup_time && enable)? SETUP:
                              (enable_command_send && enable)? COMMAND: 
                              (enable)? SEND_RECV:IDLE;
             end

            SETUP: begin
                if(sclk_cont < (chip_select_setup_time + 'd2)) begin
                    state_next = SETUP;
                end else if(sclk_cont >= (chip_select_setup_time + 'd2)) begin
                    state_next = (~mosi_shift)? SETUP:
                                 (enable_command_send)? COMMAND:SEND_RECV;
                end else begin
                    state_next = state;
                end
            end
            COMMAND: begin
                if(master_slave_select == MASTER_MODE) 
                begin
                    if(sclk_cont < 'd8)
                        state_next = COMMAND;
                    else if(sclk_cont == 'd8 && mosi_shift)
                        state_next = SEND_RECV;
                    else
                        state_next = state;

                end
            end
             SEND_RECV:begin
                 if(master_slave_select == MASTER_MODE) 
                 begin
                     if(sclk_cont < data_length) begin
                        state_next =  SEND_RECV;
                     end else if(sclk_cont == data_length) begin
                         state_next = (enable && mosi_shift)? WAIT:
                                      (~enable)? END:state;
                     end else begin
                        state_next = state;
                     end


                 end /*else begin
                     state_next = (sclk_cont < data_length && enable && ~chip_select)? SEND_RECV:
                                  (chip_select && enable)? END:
                                  (~enable)? IDLE: WAIT;
                 end*/ 
             end
             WAIT:begin
                 if(master_slave_select == MASTER_MODE)
                 begin
                     state_next = (sclk_cont < (wait_time + 'd1))? WAIT:
                                 (enable && start_pulse)? START:
                                 (enable && data_bytes_recv_cont < data_packets_limit)? SEND_RECV:END;
                 end /*else begin
                     state_next = (chip_select && enable)? SEND_RECV:
                                  (~chip_select && enable)? END:
                                  (~enable)? IDLE:WAIT;
                 end*/
             end
             END: begin
                 state_next = IDLE;
             end
             default: state_next = IDLE;
         endcase //endcase state
     end

     always_ff@(posedge clk)
     begin
         if(reset)
             state <= IDLE;
         else 
             state <= state_next;
     end

    /* Master baud rate generator 
     * ***********************************************************************************************
     * ***********************************************************************************************
     */ 
    /* On enable = 0 module, clk_out = 0 by default*/
    logic clk_enable;
    logic mclk_out,mclk;
    logic clk_gen_start;
    logic cpol_sel;
    assign cpol_sel = (cpol~^cpha);
    spi_clock_generator clk_gen(
            .clk_in(clk),
            .start(clk_gen_start),
            .cpol(cpol_sel),
            .reset(reset),
            .enable(clk_enable),
            .freq_control(freq_divider),
            .clk_out(mclk)
        );

    always_comb 
    begin
        mclk_out = mclk;
        case({cpol,cpha})
            2'b00,2'b01: mclk_out = mclk;
            2'b11,2'b10: mclk_out = ~ mclk;
        endcase
    end

    logic miso_shift,mosi_shift; //signals to shift the registers if data.
    //logic miso_shift_next,mosi_shift_next;
    always_comb 
    begin
        miso_shift = 'b0;
        mosi_shift = 'b0;
        case({cpol,cpha})
            2'b00:begin
                miso_shift = negedge_detect;
                mosi_shift = posedge_detect;
            end
            2'b01: begin
                miso_shift = posedge_detect;
                mosi_shift = negedge_detect;

            end
            2'b10: begin
                miso_shift = posedge_detect;
                mosi_shift = negedge_detect;

            end
            2'b11: begin
                miso_shift = negedge_detect;
                mosi_shift = posedge_detect;

            end
        endcase // {cpol,cpha} 
    end
    
    // always_ff @(posedge clk) 
    // begin
    //     if(reset) begin
    //         mosi_shift <= 'd0;
    //         miso_shift <= 'd0;
    //     end else begin	
    //         mosi_shift <= mosi_shift_next;
    //         miso_shift <= miso_shift_next;
    //     end
    // end
    
    //assign tick_debug = (state == SEND_RECV);
    //assign tick_debug2 = (state == WAIT);
    //assign tick_debug3 = (mosi_shift);
    /* Transmission Done output section */

    logic transmission_ready;
    logic transmission_done;
    logic data_received_done_pulse_trigger;
    assign transmission_done = (state == END);
    assign transmission_ready = (state == SEND_RECV && state_next == WAIT); // pulse when 1 data sent

    /* Data received done pulse generator */
    assign data_received_done_pulse_trigger = (data_received_done_pulse_enable)? transmission_ready:'d0;
    logic[PULSE_LENGTH:0]data_received_done_pulse_counter,data_received_done_pulse_counter_next;
    always_comb 
    begin
        if(data_received_done_pulse_trigger)
            data_received_done_pulse_counter_next = data_received_done_pulse_counter + 'd1;
        else if(data_received_done_pulse_counter == PULSE_WIDTH)  //means 1 us
            data_received_done_pulse_counter_next = 'd0;
        else if(data_received_done_pulse_counter > 'd0)
            data_received_done_pulse_counter_next = data_received_done_pulse_counter + 'd1;
        else 
            data_received_done_pulse_counter_next = data_received_done_pulse_counter;
    end

    always_ff@( posedge clk )
    begin
        if(reset)
            data_received_done_pulse_counter <= 'd0;
        else 
            data_received_done_pulse_counter <= data_received_done_pulse_counter_next;
    end

    assign data_received_done_pulse = (data_received_done_pulse_enable)? (data_received_done_pulse_counter > 'd0):'d0;

    
    logic transmission_ready_flag, transmission_ready_flag_next;
    always_comb 
    begin
        transmission_ready_flag_next = (transmission_done) ? 'b1: 
                                      (transmission_done_flag_clear)? 'b0:transmission_ready_flag;
    end

    always_ff@(posedge clk )
    begin
        if(reset) begin
            transmission_ready_flag <= 'd0;
        end else begin
            transmission_ready_flag <= transmission_ready_flag_next;
        end
    end

    assign transmission_done_flag =  transmission_ready_flag ;

    /* Transmission done pulse generator */
    logic [PULSE_LENGTH:0] transmission_done_pulse_counter,transmission_done_pulse_counter_next;
    logic transmission_done_pulse_trigger;
    assign transmission_done_pulse_trigger = (transmission_done_pulse_enable)? transmission_done: 'd0;
    /* data packet counter */
    always_comb 
    begin
        if(transmission_done_pulse_trigger)
            transmission_done_pulse_counter_next = transmission_done_pulse_counter + 'd1;
        else if(transmission_done_pulse_counter == PULSE_WIDTH)// means 1us
            transmission_done_pulse_counter_next = 'd0;
        else if(transmission_done_pulse_counter > 'd0)
            transmission_done_pulse_counter_next = transmission_done_pulse_counter + 'd1;
        else 
            transmission_done_pulse_counter_next = transmission_done_pulse_counter;
    end

    always_ff@(posedge clk)
    begin
        if(reset)
            transmission_done_pulse_counter <= 'd0;
        else 
            transmission_done_pulse_counter <= transmission_done_pulse_counter_next;
    end

    assign transmission_done_pulse = (transmission_done_pulse_enable)? (transmission_done_pulse_counter > 'd0):'d0;
    
    always_comb 
    begin
        data_bytes_recv_cont_next = (state == IDLE)? 'd0:
                                 (transmission_ready)? data_bytes_recv_cont+'d1: data_bytes_recv_cont;
    end

    always_ff@(posedge clk) begin
        if(reset) begin
            data_bytes_recv_cont <= 'd0;
        end else begin
            data_bytes_recv_cont <= data_bytes_recv_cont_next;
        end // end els
    end


    /* Data received buffer control */
    logic [63:0] data_received_buffer_next;
    always_comb 
    begin
        if(msb_lsb_select && transmission_ready)
        begin
            case(data_length)
                'd4:    data_received_buffer_next = {data_received_buffer[59:0],data_received[0],data_received[1],data_received[2],data_received[3]};
                'd8:    data_received_buffer_next = {data_received_buffer[55:0],data_received[0],data_received[1],data_received[2],data_received[3],data_received[4],data_received[5],data_received[6],data_received[7]};
                'd12:   data_received_buffer_next = {data_received_buffer[51:0],data_received[0],data_received[1],data_received[2],data_received[3],data_received[4],data_received[5],data_received[6],data_received[7],data_received[8],data_received[9],data_received[10],data_received[11]};
                'd16:   data_received_buffer_next = {data_received_buffer[47:0],data_received[0],data_received[1],data_received[2],data_received[3],data_received[4],data_received[5],data_received[6],data_received[7],data_received[8],data_received[9],data_received[10],data_received[11],data_received[12],data_received[13],data_received[14],data_received[15]};
                'd20:   data_received_buffer_next = {data_received_buffer[43:0],data_received[0],data_received[1],data_received[2],data_received[3],data_received[4],data_received[5],data_received[6],data_received[7],data_received[8],data_received[9],data_received[10],data_received[11],data_received[12],data_received[13],data_received[14],data_received[15],data_received[16],data_received[17],data_received[18],data_received[19]};
                'd24:   data_received_buffer_next = {data_received_buffer[39:0],data_received[0],data_received[1],data_received[2],data_received[3],data_received[4],data_received[5],data_received[6],data_received[7],data_received[8],data_received[9],data_received[10],data_received[11],data_received[12],data_received[13],data_received[14],data_received[15],data_received[16],data_received[17],data_received[18],data_received[19],data_received[20],data_received[21],data_received[22],data_received[23]};
                'd28:   data_received_buffer_next = {data_received_buffer[35:0],data_received[0],data_received[1],data_received[2],data_received[3],data_received[4],data_received[5],data_received[6],data_received[7],data_received[8],data_received[9],data_received[10],data_received[11],data_received[12],data_received[13],data_received[14],data_received[15],data_received[16],data_received[17],data_received[18],data_received[19],data_received[20],data_received[21],data_received[22],data_received[23],data_received[24],data_received[25],data_received[26],data_received[27]};
                'd32:   data_received_buffer_next = {data_received_buffer[31:0],data_received[0],data_received[1],data_received[2],data_received[3],data_received[4],data_received[5],data_received[6],data_received[7],data_received[8],data_received[9],data_received[10],data_received[11],data_received[12],data_received[13],data_received[14],data_received[15],data_received[16],data_received[17],data_received[18],data_received[19],data_received[20],data_received[21],data_received[22],data_received[23],data_received[24],data_received[25],data_received[26],data_received[27],data_received[28],data_received[29],data_received[30],data_received[31]};
                default:data_received_buffer_next = {data_received_buffer[31:0],data_received[0],data_received[1],data_received[2],data_received[3],data_received[4],data_received[5],data_received[6],data_received[7],data_received[8],data_received[9],data_received[10],data_received[11],data_received[12],data_received[13],data_received[14],data_received[15],data_received[16],data_received[17],data_received[18],data_received[19],data_received[20],data_received[21],data_received[22],data_received[23],data_received[24],data_received[25],data_received[26],data_received[27],data_received[28],data_received[29],data_received[30],data_received[31]};
            endcase
        end else if(~msb_lsb_select && transmission_ready) 
        begin
                case(data_length)
                'd4:    data_received_buffer_next = {data_received_buffer[59:0],data_received[3:0]};
                'd8:    data_received_buffer_next = {data_received_buffer[55:0],data_received[7:0]};
                'd12:   data_received_buffer_next = {data_received_buffer[51:0],data_received[11:0]};
                'd16:   data_received_buffer_next = {data_received_buffer[47:0],data_received[15:0]};
                'd20:   data_received_buffer_next = {data_received_buffer[43:0],data_received[19:0]};
                'd24:   data_received_buffer_next = {data_received_buffer[39:0],data_received[23:0]};
                'd28:   data_received_buffer_next = {data_received_buffer[35:0],data_received[27:0]};
                'd32:   data_received_buffer_next = {data_received_buffer[31:0],data_received[31:0]};
                default:data_received_buffer_next = {data_received_buffer[31:0],data_received[31:0]};
            endcase
        end else 
        begin
            data_received_buffer_next = data_received_buffer;
        end
     end

    always_ff@(posedge clk) begin
        if(reset ) begin
            data_received_buffer <= 'd0;
        end else begin
            data_received_buffer <= data_received_buffer_next;

        end // if(reset )end
    end



    always_comb 
    begin
        if(state == IDLE)
            sclk_cont_next = 'd0;
        else if(state == START && state_next == SEND_RECV)
            sclk_cont_next = (cpha == 'd0)? 'd0:'d1;
        else if(state == START && state_next == COMMAND)
            sclk_cont_next = (cpha == 'd0)?'d0:'d1;    
        else if(state == SETUP  && state_next == COMMAND)
            sclk_cont_next = (~cpol && ~cpha)?'d1:
                             (~cpol && cpha)? 'd1:
                             (cpol && ~cpha)? 'd0:
                             (cpol && cpha )? 'd1:sclk_cont;
        else if(state == SETUP && state_next == SEND_RECV)
            sclk_cont_next = (~cpol && ~cpha)? 'd1:
                             (~cpol && cpha)? 'd1:
                             (cpol && ~cpha)? 'd0:
                             (cpol && cpha)? 'd1:sclk_cont;
        else if(state == COMMAND && sclk_cont == 'd8)
            sclk_cont_next = (~mosi_shift)? sclk_cont:'d1;
        else if(state == SEND_RECV && sclk_cont == data_length)
            sclk_cont_next = (~mosi_shift)? sclk_cont:'d0; 
        else if(state == WAIT && state_next == SEND_RECV)
            sclk_cont_next = (~cpol && ~cpha)? 'd1:
                             (~cpol && cpha)? 'd1:
                             (cpol && ~cpha)? 'd0:
                             (cpol && cpha)? 'd1: sclk_cont;
        else if(state == WAIT && (sclk_cont == (wait_time +'d1)))
            sclk_cont_next = 'd0;
        else 
          sclk_cont_next = (mosi_shift)? sclk_cont +'d1:sclk_cont; //detect_mclk_negedge
    end

    always_ff@(posedge clk) 
    begin
        if(reset)
            sclk_cont <= 'd0;
        else
            sclk_cont <= sclk_cont_next;
    end
    /* General behaviour of SPI module */
    logic s_clk_out,s_clk_out_next,mosi_out_next,mosi_out,chip_select_out;
    logic command_last_element;
    assign command_last_element = (msb_lsb_select)? data_to_transmit[0]:data_to_transmit[data_length-1];
    always_comb begin
        clk_enable = 'b0; //clk_disable by defualt
        clk_gen_start = 'b0;
        chip_select_out =  'bZ; //original to Z
        data_received_next = data_received;
        mosi_out_next = 'b0;
        s_clk_out_next = cpol;
        case(state)
            IDLE: begin
                if(master_slave_select == MASTER_MODE)begin
                    chip_select_out = 'b1; //not device selected
                    s_clk_out_next = cpol; // clk connected to clock generator
                    mosi_out_next = 'b0;
                end 
            end
            START: begin
                clk_gen_start = 'b1;
                clk_enable = 'b1;
                mosi_out_next = 'b0;
                chip_select_out = (master_slave_select == MASTER_MODE)? 'b0:'bZ;
            end
            SETUP: begin
                clk_enable = 'b1;
                s_clk_out_next = cpol;
                chip_select_out = (master_slave_select == MASTER_MODE)? 'b0:'bZ;
                clk_gen_start = ((sclk_cont == chip_select_setup_time + 'd2) && mosi_shift);
                if(enable_command_send) begin
                    if(cpol & cpha) 
                        mosi_out_next = ((sclk_cont == (chip_select_setup_time + 'd1)) && mosi_shift && msb_lsb_select)? command_to_send[0]:
                                        ((sclk_cont == (chip_select_setup_time + 'd1)) && mosi_shift && ~msb_lsb_select)? command_to_send[7]:
                                        mosi_out;
                    else 
                         mosi_out_next = ((sclk_cont == (chip_select_setup_time + 'd2)) && mosi_shift && msb_lsb_select)? command_to_send[0]:
                                        ((sclk_cont == (chip_select_setup_time + 'd2)) && mosi_shift && ~msb_lsb_select)? command_to_send[7]:
                                        mosi_out;

                end else begin
                    if(cpol & cpha)
                        mosi_out_next = ((sclk_cont == (chip_select_setup_time + 'd1)) && mosi_shift && msb_lsb_select)? data_to_transmit[0]:
                                        ((sclk_cont == (chip_select_setup_time + 'd1)) && mosi_shift && ~msb_lsb_select)? data_to_transmit[data_length-1]:
                                        mosi_out;
                    else
                        mosi_out_next = ((sclk_cont == (chip_select_setup_time + 'd2)) && mosi_shift && msb_lsb_select)? data_to_transmit[0]:
                                        ((sclk_cont == (chip_select_setup_time + 'd2)) && mosi_shift && ~msb_lsb_select)? data_to_transmit[data_length-1]:
                                        mosi_out;

                end
            end
            COMMAND: 
            begin
                clk_enable = 'b1;
                s_clk_out_next = mclk_out;
                if(master_slave_select == MASTER_MODE) 
                begin
                    chip_select_out = (four_wire_selection)?'bZ:'b0;
                    mosi_out_next = (sclk_cont >= 'd8 && mosi_shift)? command_last_element:
                                    (sclk_cont >= 'd8)? mosi_out:
                                    (msb_lsb_select && mosi_shift)? command_to_send[sclk_cont]: 
                                    (~msb_lsb_select && mosi_shift)? command_to_send['d8-sclk_cont-1]:
                                    mosi_out; //lsb first : msb first
                end 
                //data_received_next = (msb_lsb_select && miso_shift)? {miso,data_received[31:1]}:
                //                        (~msb_lsb_select && miso_shift)? {data_received[30:0],miso}: data_received; 
            end
            SEND_RECV:
            begin//means opertion mode 4 wire
                clk_enable = 'b1;
                s_clk_out_next = (sclk_cont == data_length && mosi_shift)? cpol: mclk_out;
                if(master_slave_select == MASTER_MODE)
                begin
                    /* MOSI configuration: depends of cpol and cpha */
                    chip_select_out = (four_wire_selection)?'bZ:'b0;
                    mosi_out_next = (sclk_cont >= data_length)? mosi_out: 
                                    (msb_lsb_select && mosi_shift)? data_to_transmit[data_length*data_bytes_recv_cont + sclk_cont]: 
                                    (~msb_lsb_select && mosi_shift)? data_to_transmit[data_length*data_bytes_recv_cont + (data_length-sclk_cont-1)]:
                                    mosi_out; //lsb first : msb first
                    data_received_next = (miso_shift)?{data_received[30:0],miso}:data_received;
                end /*else begin  //slave 
                    data_received_next = (lsb_select && mosi_shift)? {miso,data_received}:{data_received,miso};
                    miso_out = 	(lsb_select && miso_shift && ~read_write_flag)? data_to_transmit[miso_clk_cont]:
                                (lsb_select && miso_shift && read_write_flag)? 'b0:
                                (~lsb_select && miso_shift && ~read_write_flag)? data_to_transmit[data_length-miso_clk_cont-1]:
                                (~lsb_select && miso_shift && read_write_flag) ? 'b0:'b0;
                end*/
            end
            WAIT: begin
                if(master_slave_select == MASTER_MODE)begin
                    s_clk_out_next  = (data_bytes_recv_cont == data_packets_limit)? cpol:
                                      (sclk_cont <= wait_time)?cpol:mclk_out;
                    chip_select_out = (four_wire_selection)? 'bZ: wait_time_chip_select_polarity;
                    clk_enable = 'b1;
                    clk_gen_start = (sclk_cont == wait_time && mosi_shift);
                    mosi_out_next = (sclk_cont < (wait_time))? 'd0:
                                     (msb_lsb_select && mosi_shift)? data_to_transmit[data_length*data_bytes_recv_cont]: 
                                    (~msb_lsb_select && mosi_shift)? data_to_transmit[data_length*data_bytes_recv_cont + (data_length-1)]:
                                    mosi_out;
                end else begin

                end
            end
            END: begin
                if(master_slave_select) begin
                    chip_select_out =(four_wire_selection)?'bZ:'b1;
                end else begin

                end
            end
        endcase // state
    end

    assign mosi = mosi_out;
    //assign miso = miso_out;
    assign sclk = s_clk_out;
    assign chip_select = chip_select_out;

    always_ff@(posedge clk) begin 
        if(reset) begin
            data_received <= 'd0;
            mosi_out <= 'd0;
            s_clk_out <= 'd0;
        end else begin 
            data_received <= data_received_next;
            mosi_out <= mosi_out_next;
            s_clk_out <= s_clk_out_next;
        end 
    end
    
endmodule //spi_basic

/*
 *spi_clock_generator.sv
 * 2017/04/17 - Felipe Veas <felipe.veasv [at] usm.cl>
 * Modificado por Damian Quiroz.
 * damian.quiroz.13 <at> sansano.usm.cl
 * 
 * Divisor de reloj basado en un contador para una frecuencia de entrada
 * de MASTER_FREQ [MHz] (default 100MHz)
 * 
 * Recibe como entrada un numero que indica el divisor de frecuencia a usar 
 *
 * Rango de operación:
 *     1526[Hz] <= clk_out <= MASTER_FREQ/10 [Hz]
 * Como referencia para un clock de 100Mhz , la frecuencia maxima sera de
 * 10 MHZ (calculado para un modulo SPI estandar)
 */

/********************************************************* 
 * spi_clock_generator (clock_divider behaviour )   
 *                         
 * input clk_in: master clock.
 * input reset: master_reset active high
 * input freq_control: register to generate out frequency.
 * 
 * 		clk_out = MASTER_FREQ/(20 + 2*freq_control) 
 * 		
 * output clk_out: reloj generado (reset state low)
 * 
 * Por ejemplo si se desea un clock de 1Mhz, freq_control = 40
 * 
 * MASTER_FREQ : master clock
 * 
**********************************************************/
module spi_clock_generator
#(
    parameter MASTER_FREQ = 100_000_000
)(
    input clk_in,
    input reset,
    input enable,
    input start,
    input cpol,
    input[15:0] freq_control,
    output logic clk_out
);
     
    logic[15:0]aux_freq;

    logic [26:0] counter,counter_next ;
    logic clk_out_next;

    /*
     * Bloque combinacional que resetea el contador e invierte el valor del reloj de salida
     * cada vez que el contador llega a su valor máximo.
     */
    always_comb begin
        if(enable == 1'b0) begin
            clk_out_next = 'd0;
            counter_next = 'd0;
        end else if(start) begin
            clk_out_next = cpol;
            counter_next = 'd0;
        end else if(counter == (('d9 + freq_control)/2)) begin
            counter_next = 'd0;
            clk_out_next = ~ clk_out;
        end else begin
            clk_out_next = clk_out;
            counter_next = counter + 'd1;
        end
    end
    /*
     * Bloque procedural que actualiza los valores del contador y del reloj
     */
    always_ff @(posedge clk_in) begin
        if (reset == 1'b1) begin
            // Señal reset sincrónico, setea el contador y la salida a un valor conocido
            counter <= 'd0;
            clk_out <= 'd0;
        end else begin
            // Se incrementa el contador y se mantiene la salida con su valor anterior
            counter <= counter_next;
            clk_out <= clk_out_next;
        end
    end

endmodule