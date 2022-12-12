/************************************************************************
 * uart_basic.v
 * 2017/02/01 - Felipe Veas <felipe.veasv at usm.cl>
 *
 * Universal Asynchronous Receiver/Transmitter.
 * Modified by: Damian Quiroz 
 * Date: 02/02/2019
 * Modifications: Replace unused modules and adds features for a complete
 * 				UART interface.
 ************************************************************************/


 /**
  * USAGE OF THE MODULE
  */
// uart_basic 
// #(
//	.PULSE_WIDTH('d100)  
// ) basic_uart_module
// (
// 	.clk                                (),			// input clk,
// 	.reset                              (),			// input reset,
// 	.rx_pin                             (),			// input rx_pin,
// 	.stop_bit                           (),			// input stop_bit,
// 	.baud_rate                          (),			// input [3:0]baud_rate,
// 	.parity                             (),			// input [1:0]parity,
//  .data_length                        (),			// input [1:0] data_length,
// 	.rx_data                            (),			// output [7:0] rx_data,
// 	.rx_done_flag_clear                 (),			// input rx_done_flag_clear,
// 	.rx_parity_error_flag_clear         (),			// input rx_done_error_flag_clear,
// 	.rx_done_pulse_enable               (),			// input rx_done_pulse_enable,
// 	.rx_parity_error_pulse_enable       (),			// input rx_parity_error_pulse_enable,
// 	.rx_done_pulse                      (),			// output logic rx_done_pulse,
// 	.rx_done_flag                       (),			// output rx_done_flag
// 	.tx_pin                             (),			// output tx_pin,
// 	.tx_start                           (),			// input tx_start,
//  .tx_done_pulse_enable               (),			// input tx_done_pulse_enable,
// 	.tx_data                            (),			// input [7:0] tx_data,
// 	.tx_done_flag_clear                 (),			// input tx_done_flag_clear,
// 	.tx_done_pulse                      (),			// output tx_done_pulse,
// 	.tx_done_flag                       (),			// output tx_done_flag,
// 	.tx_busy                            (),			// output logic tx_busy,
//  .parity_error_rx_pulse              (),			// output parity_error_rx_pulse,
//  .parity_error_rx_flag               (),			// output parity_error_rx_flag,
// 	.rx_overrun_flag                    (),			// output rx_overrun_flag,
// 	.rx_overrun_flag_clear				(),			// input rx_overrun_flag_clear,
// 	.rx_overrun_pulse_enable			(),			// input rx_overrun_pulse_enable,
// 	.rx_overrun_pulse					()			// output rx_overrun_pulse
// );
// 
// 

`timescale 1ns / 1ps

/************************************************************************
 * uart_basic: implements complete features of UART protocol
 * input clk: master clock (default 100Mhz)
 * input reset: master reset
 * input rx: RX bus of UART 
 * input stop_bit: stop bit quantity selection
 * 			stop_bit = 0 : 1 stop bit  
 * 			stop_bit = 1 : 2 stop bit
 * input [3:0] baud_rate : 4 bits to select 16 diferente baud rate
 * 					 configurations.
 * input [1:0]parity: parity configuration registers  
 * 					parity[0]=0: no parity bit, parity[0] = 1 : parity bit
 * 					parity[1]=0: odd parity,    parity[1] = 1 : even parity
 * [1:0]data_length = Data length configuration.
 * 					00 : default = 8 bit of data
 * 					01 : 7 bit of data
 * 					10 : 6 bit of data
 * 					11 : 5 bit of data 
 * output[7:0] rx_data: data received from uart RX
 * input rx_done_flag_clear: flag to clear rx_done_flag bit
 * input rx_parity_error_flag_clear: flat to clear rx_parity_error_flag bit.
 * input rx_done_pulse_enable: enable pulses when data is received
 * input rx_parity_error_pulse_enable: enable pulses when a parity error occurs.
 * output rx_done_pulse: pulse when data is received.
 * output rx_done_flag : set when data has been received
 * output rx_done: sends a pulse when RX ends a reception
 * output tx: TX bus of UART
 * input tx_start: pulse to begin a transmission
 * input tx_ready_pulse_enable: enables pulses on every data send
 * input[7:0] tx_data: data to transmit
 * input tx_done_flag_clear: flag to clear tx_ready_flag bit.
 * output tx_done_pulse: pulse when data is send.
 * output tx_done_flag: flag when data is send
 * output tx_busy: flag for uart transmission busy 1 means busy, 0 means not busy
 * output parity_error_rx_flag: show if is a parity error of data received
 * output parity_error_rx_pulse: pulses when parity error occurs (pulse length 1us)
 * input rx_overrun_flag_enable:
 * input rx_overrun_flag_clear: 
 * output rx_overrun_flag : set when data hasn't been readed after receive
 * input rx_overrun_pulse_enable: 
 * output rx_overrun_pulse:	pulse when data hasn't been readed after a receive (pulse length bus)

  * parameter PULSE_WIDTH : Cycles of clock of the pulse signals.
 *************************************************************************/

module uart_basic
#(
	parameter PULSE_WIDTH = 'd100
)(
	input clk,
	input reset,
	input rx_pin,
	input stop_bit,
	input [3:0]baud_rate,
	input [1:0]parity,
	input [1:0] data_length,
	output [7:0] rx_data,
	input rx_done_flag_clear,
	input rx_parity_error_flag_clear,
	input rx_done_pulse_enable,
	input rx_parity_error_pulse_enable,
	output logic rx_done_pulse,
	output rx_done_flag,
	output tx_pin,
	input tx_start,
	input tx_done_pulse_enable,
	input [7:0] tx_data,
	input tx_done_flag_clear,
	output tx_done_pulse,
	output tx_done_flag,
	output logic tx_busy,
	output parity_error_rx_pulse,
	output parity_error_rx_flag,
	input rx_overrun_pulse_enable,
	input rx_overrun_flag_clear,
	output rx_overrun_pulse,
	output rx_overrun_flag
);

	
	//logic baud_tick,baud_tick_tx;


	 logic rx_ready_sync;
	 logic rx_ready_pre;
	
	/* Adds Baud Rate configuration register */
	uart_baud_config #(1) baud_config_tx(
		.clk(clk),
		.reset(reset),
		.enable(tx_busy),
		.uart_config_reg(baud_rate),
		.baud_tick(baud_tick_tx)
	);
	/* Changes original "uart_baud_tick_gen" for a general option
	 * of baud rate generator */
	uart_baud_config #(8) baud_config_rx(
		.clk(clk),
		.reset(reset),
		.enable(1'b1),
		.uart_config_reg(baud_rate),
		.baud_tick(baud_tick)
	);

	
	uart_rx #(PULSE_WIDTH) uart_rx_blk (
		.clk(clk),
		.reset(reset),
		.rx_ready_pulse_enable(rx_done_pulse_enable), 					// input rx_ready_pulse_enable
		.rx_parity_error_pulse_enable(rx_parity_error_pulse_enable),	// input rx_parity_error_pulse_enable
		.baud_tick(baud_tick),
		.stop_bit(stop_bit),
		.parity_bit_config(parity),
		.data_length(data_length),
		.rx(rx_pin),
		.rx_data(rx_data),
		.rx_ready_flag_clear(rx_done_flag_clear),
		.rx_parity_error_flag_clear(rx_parity_error_flag_clear),
		.rx_ready_pulse(rx_done_pulse), //rx ready pre
		.rx_ready_flag(rx_done_flag),
		.parity_error_rx_pulse(parity_error_rx_pulse),
		.parity_error_rx_flag(parity_error_rx_flag),
		.rx_overrun_flag_clear(rx_overrun_flag_clear),	
		.rx_overrun(rx_overrun_flag),
		.rx_overrun_pulse_enable(rx_overrun_pulse_enable),
		.rx_overrun_pulse(rx_overrun_pulse)
	);


	uart_tx #(PULSE_WIDTH) uart_tx_blk (
		.clk(clk),
		.reset(reset),
		.tx_ready_pulse_enable(tx_done_pulse_enable),
		.baud_tick(baud_tick_tx),
		.stop_bit(stop_bit),
		.parity_bit_config(parity),
		.data_length(data_length),
		.tx_start(tx_start),
		.tx_data(tx_data),
		.tx_ready_flag_clear(tx_done_flag_clear),
		.tx_ready_pulse(tx_done_pulse),
		.tx_ready_flag(tx_done_flag),
		.tx(tx_pin),
		.tx_busy(tx_busy)
	);

/*********************************************************************************************************
 * End of uart_basic module
 *********************************************************************************************************/
endmodule //uart_basic


/******************************************************
  * Basic RX module for UART protocol 
  * Updated by: Damian Quiroz
  * Date: 01/02/2019
  * damian.quiroz.13 (at) sansano.usm.cl
  * Modifications: Adds a configuration register which
  * 				implements a stardard uart module with
  * 				all this features. Adds a parity-error
  *				    pin for parity check.
  * Based on original : uart_rx.v
  * 2017/02/01 - By: Felipe Veas <felipe.veasv at usm.cl>
  *******************************************************/
  /* USAGE */
//uart_rx name
	//#(
	//	.PULSE_WIDTH('d100)
	//)(
 	// .clk(),    							// input clk
	// .reset(),							// input reset
	// .rx_ready_pulse_enable(), 			// input rx_ready_pulse_enable
	// .rx_parity_error_pulse_enable(),		// input rx_parity_error_pulse_enable
	// .baud_tick(),						// input baud_tick
	// .stop_bit(),							// input stop_bit
	// .parity_bit_config(),				// input [1:0] parity_bit_config
	// .data_length(),						// input [1:0] data_length
	// .tx(),								// input rx
	// .rx_data(),							// output [7:0] rx_data
	// .rx_ready_flag_clear(),				// input rx_ready_flag_clear
	// .rx_parity_error_flag_clear(),		// input rx_parity_error_flag_clear
	// .parity_error_rx_pulse(),			// output parity_error_rx_pulse
	// .parity_error_rx_flag(),				// output logic parity_error_rx_flag
	// .rx_ready_pulse(),					// output rx_ready_pulse
	// .rx_ready_flag(),					// output logic  rx_ready_flag
	// .rx_overrun_flag_clear(),
	// .rx_overrun()						// output rx_overrun
//  );
  	

 /**********************************************************
  * uart_rx: module for asyncronus UART RX 
  * input clk: master clock
  * input reset: master reset
  * input rx_ready_pulse_enable : flag to enable pulses of received data ready
  * input rx_parity_error_pulse_enable: flat to enable pulses of parity error bit
  * input rx : rx serial bus
  * input baud_tick : clock of 8 (default) oversampling per
  *						data bit.
  * input stop_bit : ->  stop_bit = 0 : 1 stop bit , stop = 1 : 2 stop bit
  * input parity_bit_config :
  * 			parity_bit_config[0]=0: no parity bit, parity[0] = 1 : parity bit
  *				parity_bit_config[1]=0: odd parity,    parity[1] = 1 : even parity
  * input [1:0]data_length = Data length configuration.
  * 					00 : default = 8 bit of data
  * 					01 : 7 bit of data
  * 					10 : 6 bit of data
  * 					11 : 5 bit of data
  * output [7:0] rx_data: data received on RX bus
  * input rx_ready_flag_clear: bit to clear the rx_ready_flag bit, and overrun bit
  * output rx_ready_pulse : pulse when state machine goes to RX_READY state
  * output rx_ready_flag: set the state to 1 when the module has received data.
  * output parity_error_rx_pulse: pulse if a parity error occurs.
  * output parity_error_rx_flag : flag when parity error occurs.
  * output rx_overrun: set when data was lost and not readed.
  ************************************************************/

/*
 * uart_rx.v
 * 2017/02/01 - Felipe Veas <felipe.veasv at usm.cl>
 *
 * Asynchronous Receiver.
 */

`timescale 1ns / 1ps

module uart_rx
#(
	parameter PULSE_WIDTH = 'd100
)(
	input clk,
	input reset,
	input rx_ready_pulse_enable,
	input rx_parity_error_pulse_enable,
	input baud_tick,
	input stop_bit,
	input[1:0] parity_bit_config,
	input[1:0] data_length,
	input rx,
	output logic [7:0] rx_data,
	input rx_ready_flag_clear,
	input rx_parity_error_flag_clear,
	output rx_ready_pulse,
	output logic  rx_ready_flag,
	output parity_error_rx_pulse,
	output logic parity_error_rx_flag,
	input rx_overrun_flag_clear,
	output logic rx_overrun,
	input rx_overrun_pulse_enable,
	output logic rx_overrun_pulse
);

	localparam RX_IDLE  	= 	'b000;
	localparam RX_START 	= 	'b001;
	localparam RX_RECV  	= 	'b010;
	localparam RX_PARITY	= 	'b011; // Adds new state for parity bit
	localparam RX_STOP  	= 	'b100;
	localparam RX_READY 	= 	'b101;

	localparam PULSE_LENGTH = $clog2(PULSE_WIDTH)-1;
	/* Clock synchronized rx input */
	logic rx_bit;
	logic rx_ready;
	logic rx_parity_bit,rx_parity_bit_next;
	logic parity_bit_check;
	logic [2:0] spacing_counter, spacing_counter_next;
	logic next_bit;
	assign next_bit = (spacing_counter == 'd4); //=='d4

	logic[1:0] rx_ready_posedge;

	/* RX ready pulse output */
    logic rx_ready_pulse_trigg;
	assign rx_ready_pulse_trigg = (rx_ready_pulse_enable)? rx_ready: 'b0;

    logic [PULSE_LENGTH:0] rx_ready_pulse_counter,rx_ready_pulse_counter_next;

    always_comb begin
        if(rx_ready_pulse_trigg)  // pulse of 1 us
            rx_ready_pulse_counter_next = rx_ready_pulse_counter + 'd1;
        else if(rx_ready_pulse_counter == PULSE_WIDTH)
            rx_ready_pulse_counter_next = 'd0;
        else if(rx_ready_pulse_counter  > 'd0)
            rx_ready_pulse_counter_next = rx_ready_pulse_counter + 'd1;
        else 
            rx_ready_pulse_counter_next = rx_ready_pulse_counter; 
    end


	always_ff@(posedge clk) begin
		if(reset) begin
			rx_ready_posedge <= 'd0;
            rx_ready_pulse_counter <= 'd0;
		end else begin
			rx_ready_posedge <= {rx_ready_posedge[0] , rx_ready};
            rx_ready_pulse_counter <= rx_ready_pulse_counter_next;
		end
	end

    assign rx_ready_pulse = (rx_ready_pulse_counter > 'd0);

	/* RX READY FLAG OUTPUT */
	logic rx_ready_flag_aux;
	always_comb begin
		rx_ready_flag_aux = (rx_ready_flag_clear)? 'd0: 
							 (~rx_ready_posedge[1] && rx_ready_posedge[0])? 'b1: rx_ready_flag;
	end

	always_ff@(posedge clk) begin
		if(reset)
			rx_ready_flag <= 'd0;
		else 
			rx_ready_flag <= rx_ready_flag_aux;
	end


	
	/* RX OVERRUN output */
    

    logic rx_overrun_pulse_trig;
    logic[PULSE_LENGTH:0] overrun_pulse_counter,overrun_pulse_counter_next;
    assign rx_overrun_pulse = (overrun_pulse_counter > 'd0);
    /* Overrun pulse duration of 1us*/
    always_comb begin
        if(rx_overrun_pulse_trig)
            overrun_pulse_counter_next = overrun_pulse_counter + 'd1;
        else if(overrun_pulse_counter == PULSE_WIDTH)
            overrun_pulse_counter_next = 'd0;
        else if(overrun_pulse_counter > 'd0)
            overrun_pulse_counter_next = overrun_pulse_counter + 'd1; //while counter 
        else 
            overrun_pulse_counter_next = overrun_pulse_counter;//if 0 , remains actual value

    end

    always_ff@(posedge clk) begin
        if(reset)
            overrun_pulse_counter <= 'd0;
        else 
            overrun_pulse_counter <= overrun_pulse_counter_next;
    end

	logic[1:0] rx_overrun_pulse_reg;
	logic lost_data_flag;
	assign lost_data_flag = (rx_data_lost_counter > 0);

	logic rx_overrun_pulse_next;
    
	always_comb begin
		rx_overrun_pulse_next = (rx_overrun_pulse_enable && ~rx_overrun_pulse_reg[1] && rx_overrun_pulse_reg[0])? 1'b1:
							    (~rx_overrun_pulse_enable)? 'b0:'b0;
	end
	
	always_ff@(posedge clk) begin
		if(reset) begin
			rx_overrun_pulse_trig <= 'd0;
			rx_overrun_pulse_reg <='d0;
		end else begin
			rx_overrun_pulse_reg <= {rx_overrun_pulse_reg[0], lost_data_flag};
			rx_overrun_pulse_trig <= rx_overrun_pulse_next;
		end
	end // always_ff@(posedge clk)
	logic[3:0] rx_data_lost_counter , rx_data_lost_counter_next;

	always_comb begin
		rx_data_lost_counter_next = (rx_ready_flag == 'b1 && state == RX_START && next_bit)? rx_data_lost_counter +'d1:
									(rx_ready_flag == 'b0 && state == RX_START && next_bit)? 'd0: 
									(rx_overrun_flag_clear)? 'd0: rx_data_lost_counter;
	end 
	
	always_ff@(posedge clk) begin
		if(reset)
			rx_data_lost_counter <= 'd0;
		else 
			rx_data_lost_counter <= rx_data_lost_counter_next;
	end 

    logic rx_overrun_aux;

    always_comb begin
        rx_overrun_aux = (rx_data_lost_counter == 'd1) ? 'd1:
                         (rx_data_lost_counter == 'd0)? 'b0:
                         rx_overrun;
    end

	always_ff@(posedge clk) begin
        if(reset)
            rx_overrun <= 'd0;
        else 
            rx_overrun <= rx_overrun_aux;
    end


	/* RX PARITY ERROR PULSE */

	logic parity_error_rx_pulse_aux;
    logic parity_error_rx_pulse_trigg;


	assign parity_error_rx_pulse_trigg = (rx_parity_error_pulse_enable)? parity_error_rx_pulse_aux:'b0; // parity_error_rx -> pulse

    logic[PULSE_LENGTH:0] parity_error_rx_pulse_counter, parity_error_rx_pulse_counter_next;

    assign parity_error_rx_pulse = (parity_error_rx_pulse_counter > 'd0);

    always_comb begin
        if(parity_error_rx_pulse_trigg) //pulse of 1 us
            parity_error_rx_pulse_counter_next = parity_error_rx_pulse_counter + 'd1;
        else if(parity_error_rx_pulse_counter == PULSE_WIDTH)
            parity_error_rx_pulse_counter_next = 'd0;
        else if(parity_error_rx_pulse_counter > 'd0)
            parity_error_rx_pulse_counter_next = parity_error_rx_pulse_counter + 'd1;
        else 
            parity_error_rx_pulse_counter_next = parity_error_rx_pulse_counter;
    end

	assign parity_error_rx_pulse_aux =  ((state == RX_READY) && (rx_parity_bit!=parity_bit_check));
	/* RX PARITY ERROR FLAG */
	logic [1:0]parity_error_rx_posedge;
	always_ff@(posedge clk) begin	
		if(reset) begin
            parity_error_rx_pulse_counter <= 'd0;
			parity_error_rx_posedge <= 'd0;
		 end else begin

			parity_error_rx_posedge <= {parity_error_rx_posedge[0],parity_error_rx_pulse_aux};
            parity_error_rx_pulse_counter <= parity_error_rx_pulse_counter_next;
	    end 
    end

	logic rx_parity_error_flag_aux;
	always_comb begin
		rx_parity_error_flag_aux = (rx_parity_error_flag_clear)? 'b0:
								   (~parity_error_rx_posedge[1] && parity_error_rx_posedge[0])? 'b1:parity_error_rx_flag;
	end
	always_ff@(posedge clk) begin
		if(reset)
			parity_error_rx_flag <= 'd0;
		else 
			parity_error_rx_flag <= rx_parity_error_flag_aux;
	end
	/* Finite-state machine */
	logic [2:0] state, state_next;
	logic [2:0] bit_counter, bit_counter_next;
	logic [7:0] rx_data_next;

	

	data_sync rx_sync_inst (
		.clk(clk),
		.reset(reset),
		.in(rx),
		.stable_out(rx_bit)
	);


	parity_check check_parity(
		.clk(clk),
		.reset(reset),
		.data(rx_data),
		.data_length(data_length),
		.state(state),
		.parity_bit_config(parity_bit_config),
		.parity_bit(parity_bit_check)
	);

	/* Bit spacing counter (oversampling) */
	

	
	
	always_comb begin
		state_next = state;

		case (state)
		RX_IDLE:
			if (rx_bit == 1'b0)
				state_next = RX_START;
		RX_START: begin
			if (next_bit) begin
				if (rx_bit == 1'b0) // Start bit must be a 0
					state_next = RX_RECV;
				else
					state_next = RX_IDLE;
			end
		end
		RX_RECV:
			if (next_bit && bit_counter == ('d7-data_length))
				if(parity_bit_config[0] == 'd0) begin
					state_next = RX_STOP;
				end else begin
					state_next = RX_PARITY;
				end
		RX_PARITY:
			if(next_bit)
				state_next = RX_STOP;
		RX_STOP:
			if (next_bit && (stop_bit >= bit_counter ))
				state_next = RX_STOP;
			else if(next_bit)
				state_next = RX_READY;
			else 
				state_next = RX_STOP;
		RX_READY:
			state_next = RX_IDLE;
		default:
			state_next = RX_IDLE;
		endcase
	end

	always_comb begin
		bit_counter_next = bit_counter;
		spacing_counter_next = spacing_counter + 'd1;
		rx_ready = 1'b0;
		rx_data_next = rx_data;
		rx_parity_bit_next = rx_parity_bit;

		case (state)
		RX_IDLE: begin
			bit_counter_next = 'd0;
			spacing_counter_next = 'd0;
		end
		RX_RECV: begin
			if (next_bit) begin
				bit_counter_next = (bit_counter == ('d7-data_length))? 'd0: bit_counter + 'd1;
				rx_data_next = {rx_bit, rx_data[7:1]};
			end
		end
		RX_PARITY: begin
				bit_counter_next = 'd0;
				if(next_bit)
					rx_parity_bit_next = rx;
			end
		RX_STOP:
			if(next_bit)
				bit_counter_next = bit_counter +'d1;
		RX_READY:
			rx_ready = 1'b1;
		endcase
	end

	always_ff@(posedge clk) begin
		if (reset) begin
			spacing_counter <= 'd0;
			rx_parity_bit <= 'b0;
			bit_counter <= 'd0;
			state <= RX_IDLE;
			rx_data <= 'd0;
		end else if (baud_tick) begin
			spacing_counter <= spacing_counter_next;
			rx_parity_bit <= rx_parity_bit_next;
			bit_counter <= bit_counter_next;
			state <= state_next;
			rx_data <= rx_data_next;
		end
	end

endmodule

/*****************************************************************
 * uart_tx.v
 * 2017/02/01 - Felipe Veas <felipe.veasv at usm.cl>
 *
 * Asynchronous Transmitter.
 * Modified by: Damian Quiroz 
 * Date : 01/02/2019
 * Modifications: Adds a configure register for complete uart
 *				features.
 *****************************************************************/

/* MODULE USAGE*/
// uart_tx name 
//#(
//	.PULSE_WIDTH('d100)
// )(
// 	.clk(), 								// input clk,
// 	.reset(),								// input reset,
// 	.tx_ready_pulse_enable(),				// input tx_ready_pulse_enable,
// 	.baud_tick(), 							// input baud_tick,
// 	.stop_bit(),							// input stop_bit,
// 	.parity_bit_config(),					// input [1:0] parity_bit_config,
// 	.data_length(),							// input [1:0] data_length,
// 	.tx_start(),							// input tx_start,
// 	.tx_data(),								// input [7:0] tx_data,
// 	.tx_ready_flag_clear(),					// input tx_ready_flag_clear,
// 	.tx_ready_pulse(),						// output tx_ready_pulse,
// 	.tx_ready_flag(),						// output logic tx_ready_flag,
// 	.tx(),									// output logic tx,
// 	.tx_busy()								// output logic tx_busy
// );


/******************************************************************
 * uart_tx : module for Asyncronus UART TX
 * input clk: master clock
 * input reset: master reset
 * input tx_ready_pulse_enable: flag to enable tx ready pulses
 * input baud_tick : tick counter (default 1 per bit)
 * input tx_start : indicate if a transmission begin
 * stop_bit = 0 : 1 stop bit , stop = 1 : 2 stop bit
 * parity_bit_config[0]=0: no parity bit, parity[0] = 1 : parity bit
 * parity_bit_config[1]=0: odd parity,    parity[1] = 1 : even parity
 * [1:0]data_length = Data length configuration.
 * 					00 : default = 8 bit of data
 * 					01 : 7 bit of data
 * 					10 : 6 bit of data
 * 					11 : 5 bit of data
 * input [7:0]tx_data : data to send
 * input tx_ready_flag_clear: flag to clear tx_ready_flag bit.
 * output tx_ready_pulse: pulse when data was sent.
 * output tx_ready_flag:  flag when data was sent.
 * output logic tx: tx bus of UART
 * output tx_busy : indicate a transmission in proccess.
 *******************************************************************/
module uart_tx
#(
	parameter PULSE_WIDTH = 'd100
)(
	input clk,
	input reset,
	input tx_ready_pulse_enable,
	input baud_tick,
	input stop_bit,
	input [1:0] parity_bit_config,
	input [1:0] data_length,
	input tx_start,
	input [7:0] tx_data,
	input tx_ready_flag_clear,
	output tx_ready_pulse,
	output logic tx_ready_flag,
	output logic tx,
	output logic tx_busy
);

	localparam TX_IDLE  	= 3'b000;
	localparam TX_START 	= 3'b010;
	localparam TX_SEND  	= 3'b011;
	localparam TX_PARITY 	= 3'b001;  //CODE OF PARITY = START OF PARITY CHECK
	localparam TX_STOP  	= 3'b100;
	localparam TX_READY 	= 3'b111;

	localparam PULSE_LENGTH = $clog2(PULSE_WIDTH)-1;
	logic [2:0] state , state_next;
	logic [2:0] counter, counter_next;
    logic [7:0] tx_data_reg;
    
    parity_check check_parity_tx(
		.clk(clk),
		.reset(reset),
		.data(tx_data_reg),
		.data_length(data_length),
		.state(state),
		.parity_bit_config(parity_bit_config),
		.parity_bit(parity_bit_check)
	);

    /* TX READY PULSE */
    logic[1:0] tx_ready_pulse_posedge;
    logic tx_ready_pulse_trigg;
	logic[PULSE_LENGTH:0]tx_ready_pulse_counter,tx_ready_pulse_counter_next;
	assign tx_ready_pulse_trigg = (state == TX_READY);
	always_comb begin
        if(tx_ready_pulse_trigg) //pulse of 1 us
            tx_ready_pulse_counter_next = tx_ready_pulse_counter + 'd1;
        else if(tx_ready_pulse_counter == PULSE_WIDTH)
            tx_ready_pulse_counter_next = 'd0;
        else if(tx_ready_pulse_counter > 'd0)
            tx_ready_pulse_counter_next = tx_ready_pulse_counter + 'd1;
        else 
            tx_ready_pulse_counter_next = tx_ready_pulse_counter;
    end

	//assign parity_error_rx_pulse_aux =  ((state == RX_READY) && (rx_parity_bit!=parity_bit_check));
    assign tx_ready_pulse = (tx_ready_pulse_enable)? (tx_ready_pulse_counter > 'd0):'b0;

    always_ff@(posedge clk) begin
    	if(reset)
    		tx_ready_pulse_posedge <= 'd0;
    	else 
    		tx_ready_pulse_posedge  <= {tx_ready_pulse_posedge[0],tx_ready_pulse_trigg}; 
    end
    /* TX READY FLAG*/
    logic tx_ready_flag_aux;
    always_comb begin
    	tx_ready_flag_aux = (tx_ready_flag_clear)? 'b0:
    					(~tx_ready_pulse_posedge[1] && tx_ready_pulse_posedge[0])? 'b1: tx_ready_flag;
    end

    always_ff@(posedge clk) begin
    	if(reset)
    		tx_ready_flag <= 'd0;
    	else 
    		tx_ready_flag <= tx_ready_flag_aux;
    end // always_ff@(posedge clk)

    /***************************************/
    always_ff@(posedge clk) begin
        if (reset)
            tx_data_reg <= 'd0;
        else if (state == TX_IDLE && tx_start)
            tx_data_reg <= tx_data;
    end // always_ff@(posedge clk)

	always_comb begin
		tx = 1'b1;
		tx_busy = 1'b1;
		state_next = state;
		counter_next = counter;

		case (state)
		TX_IDLE: begin
			tx_busy = 1'b0;
			state_next = (tx_start) ? TX_START : TX_IDLE;
		end
		TX_START: begin
			tx = 1'b0;
			state_next = (baud_tick) ? TX_SEND : TX_START;
			counter_next = 'd0;
		end
		TX_SEND: begin
			tx = tx_data_reg[counter];
			if (baud_tick) begin
				if((counter == ('d7 - data_length))) begin

					state_next = (parity_bit_config[0] =='b0)? TX_STOP :TX_PARITY;
					counter_next = 'd0; //original counter +'d1
				end else begin
					state_next = TX_SEND;
					counter_next = counter + 'd1;
				end // end else

			end
		end
		TX_PARITY: begin
			tx = parity_bit_check;
			if(baud_tick)
				state_next = TX_STOP;
		end
		TX_STOP: begin
			if(baud_tick) begin
				state_next = (stop_bit >= counter) ? TX_STOP : TX_READY;
				counter_next = counter + 'd1;
			end else begin
				state_next = TX_STOP;
			end // end else
			
		end
		TX_READY: begin
			state_next = TX_IDLE;
		end
		endcase
	end // always_comb

	always_ff@(posedge clk) begin
		if (reset) begin
			state <= TX_IDLE;
			counter <= 'd0;
		end else begin
			state <= state_next;
			counter <= counter_next;
		end
	end // always_ff@(posedge clk)

endmodule


/****************************************************
 * UART configuration register selection
 * Created by: Damian Quiroz
 * Date : 01/02/2019
 * damian.quiroz.13 (at) sansano.usm.cl
 * 
 ****************************************************/

/*******************************************************
 * uart_baud_config: Generate baud rate number using 	 
 * 					configuration register
 *	input reset : resets the module
 * 	input uart_config_reg: 4 bit to select 16 diferent 
 *						baud rate configurations
 *	output baud_rate_value: numeric value of baud rate
 *******************************************************/
module uart_baud_config #(
		parameter DEFAULT_OVERSAMPLING = 4
	)(
		input clk,
		input reset,
		input [3:0]uart_config_reg,
		input enable,
		output logic baud_tick
	);
	localparam MASTER_CLOCK = 100000000;
	/***********************************************************
	 * uart_config_reg structrure
	 * {[3:0]baud_rate}
	 * [3:0] baud_rate : 4 bits to select 16 diferente baud rate
	 * 					 configurations.
	************************************************************/
	/* UART default baud rate configurations */
	localparam BAUD_2400   	=   'b0000;
	localparam BAUD_4800   	=   'b0001;
	localparam BAUD_9600   	=   'b0010;
	localparam BAUD_14400  	=   'b0011;
	localparam BAUD_19200	=	'b0100;
	localparam BAUD_28800	=	'b0101;
	localparam BAUD_38400	=	'b0110;
	localparam BAUD_56000	=	'b0111;
	localparam BAUD_57600	=	'b1000;
	localparam BAUD_115200	=	'b1001;
	localparam BAUD_128000	=	'b1010;
	localparam BAUD_153600	=	'b1011;	
	localparam BAUD_230400	=	'b1100;
	localparam BAUD_256000	=	'b1101;
	localparam BAUD_460800	=	'b1110;
	localparam BAUD_921600	=	'b1111;

	/* Selection of numeric value of baud_rate_value */
	logic baud_tick_next;
	logic baud_2400,baud_4800,baud_9600,baud_14400,baud_19200,baud_28800;
	logic baud_38400,baud_56000,baud_57600,baud_115200,baud_128000,baud_153600;
	logic baud_230400,baud_256000,baud_460800,baud_921600;
	always_comb begin
		//baud_rate_value='d115200;
		baud_tick_next = baud_115200;
		case(uart_config_reg[3:0])
			BAUD_2400: begin
					baud_tick_next = baud_2400;
					//baud_rate_value='d2400;
				end
			BAUD_4800: begin 
					baud_tick_next = baud_4800;
				end
			BAUD_9600: begin 
					baud_tick_next = baud_9600;
				end
			BAUD_14400: begin
					baud_tick_next = baud_14400;
				end
			BAUD_19200: begin
					baud_tick_next = baud_19200;
				end
			BAUD_28800: begin
					baud_tick_next = baud_28800;
				end
			BAUD_38400: begin
					baud_tick_next = baud_38400;
				end
			BAUD_56000: begin
					baud_tick_next = baud_56000;
				end
			BAUD_57600: begin
					baud_tick_next = baud_57600;
				end
			BAUD_115200: begin
					baud_tick_next = baud_115200;
				end
			BAUD_128000: begin
					baud_tick_next = baud_128000;
				end
			BAUD_153600: begin
					baud_tick_next = baud_153600;
				end
			BAUD_230400: begin
					baud_tick_next = baud_230400;
				end
			BAUD_256000: begin
					baud_tick_next = baud_256000;
				end
			BAUD_460800: begin
					baud_tick_next = baud_460800;
				end
			BAUD_921600: begin
					baud_tick_next = baud_921600;
				end
		endcase // uart_config_reg[10:7]
	end

	always_ff@(posedge clk)begin
		if(reset)begin
			baud_tick <= 'd0;
		end else begin
			baud_tick <= baud_tick_next;
		end
	end

		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(2400),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_2400(
			.clk(clk),
			.enable(enable),
			.tick(baud_2400)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(4800),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_4800(
			.clk(clk),
			.enable(enable),
			.tick(baud_4800)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(9600),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_9600(
			.clk(clk),
			.enable(enable),
			.tick(baud_9600)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(14400),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_14400(
			.clk(clk),
			.enable(enable),
			.tick(baud_14400)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(19200),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_19200(
			.clk(clk),
			.enable(enable),
			.tick(baud_19200)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(28800),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_28800(
			.clk(clk),
			.enable(enable),
			.tick(baud_28800)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(38400),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_38400(
			.clk(clk),
			.enable(enable),
			.tick(baud_38400)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(56000),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_56000(
			.clk(clk),
			.enable(enable),
			.tick(baud_56000)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(57600),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_57600(
			.clk(clk),
			.enable(enable),
			.tick(baud_57600)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(115200),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_115200(
			.clk(clk),
			.enable(enable),
			.tick(baud_115200)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(128000),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_128000(
			.clk(clk),
			.enable(enable),
			.tick(baud_128000)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(153600),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_153600(
			.clk(clk),
			.enable(enable),
			.tick(baud_153600)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(230400),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_230400(
			.clk(clk),
			.enable(enable),
			.tick(baud_230400)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(256000),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_256000(
			.clk(clk),
			.enable(enable),
			.tick(baud_256000)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(460800),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_460800(
			.clk(clk),
			.enable(enable),
			.tick(baud_460800)
		);
		uart_baud_tick_gen #(
			.CLK_FREQUENCY(MASTER_CLOCK),
			.BAUD_RATE(921600),
			.OVERSAMPLING(DEFAULT_OVERSAMPLING)
		) baud_tick_921600(
			.clk(clk),
			.enable(enable),
			.tick(baud_921600)
		);
endmodule // uart_baud_config 

/**********************************************************************************************
 * clk : master clock
 * reset: master reset
 * data: data to check parity
 * data_length: data length of data (min 5 , max 8)
 * state: state of state machine of RX of uart
 * parity_bit_config: parity configuration registers  
 * 					parity_bit_config[0]=0: no parity bit, parity_bit_config[0] = 1 : parity bit
 * 					parity_bit_config[1]=0: odd parity,    parity_bit_config[1] = 1 : even parity
 * parity_bit : parity bit generate by the module
 * 
*************************************************************************************************/

module parity_check(
	input clk,
	input reset,
	input[7:0] data,
	input[1:0] data_length,
	input[2:0] state,
	input[1:0] parity_bit_config,
	output logic parity_bit
    );

	localparam NO_PARITY 	= 	1'b0;
	localparam PARITY_BIT   = 	1'b1;
	localparam PARITY_ODD	= 	1'b0;
	localparam PARITY_EVEN	=	1'b1;

	/* Parity state */
	localparam IDLE			= 	'b000;
	localparam SEND_RECV	= 	'b010;
	localparam PARITY		= 	'b011;

	logic parity_bit_next;
	logic [2:0] data_counter_bit_next,data_counter_bit;
	logic [3:0] ones_counter,ones_counter_next;
	logic [7:0] data_shift,data_shift_next;
	/* Ones counter for parity check */
	always_comb begin
		data_shift_next = data_shift;
		data_counter_bit_next = 4'd0;
		ones_counter_next = ones_counter;
		parity_bit_next = parity_bit; 
		case(state) 
			IDLE : 
				ones_counter_next = 4'd0;
			SEND_RECV: 
				data_shift_next = data;
			PARITY:
			begin
				if(parity_bit_config[0] == PARITY_BIT) begin //parity activated
					/* shift register for 1's count */
					if(data_counter_bit < ('d7-data_length)) begin // configuration of 00 means 8 bit length and so on
						data_counter_bit_next = data_counter_bit + 4'd1; 

						if(data_shift[0]=='b1) begin
							ones_counter_next = ones_counter+'d1;
							data_shift_next = {1'b0,data_shift[7:1]};
						end else begin
							data_shift_next = {1'b0,data_shift[7:1]};
						end

						/* Parity bit generator , 0 means odd parity*/
					end else begin
						if(parity_bit_config[1] == 'b0) begin
							if(ones_counter%2 == 'd1)
								parity_bit_next = 'b0;
							else
								parity_bit_next = 'b1;
						end else begin
							if(ones_counter%2 == 'd0)
								parity_bit_next = 'b0;
							else 
								parity_bit_next = 'b1;
						end
					end 
				end else begin
					parity_bit_next = 'b0; // if parity was no active , parity bit must be always 0
				end 

		 	end
	 	endcase // state
	end

	always_ff@(posedge clk) begin
		if(reset) begin
			parity_bit <= 'd0;
			data_shift <= 'd0;
			data_counter_bit <= 'd0;
			ones_counter  <= 'd0;
		end else begin 
			parity_bit <= parity_bit_next;
			data_shift <= data_shift_next;
			data_counter_bit <= data_counter_bit_next;
			ones_counter <= ones_counter_next;
		end
	end
endmodule


/*
 * data_sync.v
 * 2017/05/13 - Felipe Veas <felipe.veasv at usm.cl>
 *
 * This module synchronizes the input with respect the clock signal
 * and filters short spikes on the input line.
 */

module data_sync
(
	input clk,
	input reset,
	input in,
	output logic stable_out
);

	/* Clock synchronized input */
	logic [1:0] in_sync_sr;
	logic in_sync = in_sync_sr[0];

	always_ff@(posedge clk)
		in_sync_sr <= {in, in_sync_sr[1]};

	/* Filter out short spikes on the input line */
	logic [1:0] sync_counter = 'b11, sync_counter_next;
	logic stable_out_next;

	always_comb begin
		if (in_sync == 1'b1 && sync_counter != 2'b11)
			sync_counter_next = sync_counter + 'd1;
		else if (in_sync == 1'b0 && sync_counter != 2'b00)
			sync_counter_next = sync_counter - 'd1;
		else
			sync_counter_next = sync_counter;
	end

	always_comb begin
		case (sync_counter)
		2'b00:
			stable_out_next = 1'b0;
		2'b11:
			stable_out_next = 1'b1;
		default:
			/* Keep the previous value if the counter is not on its boundaries */
			stable_out_next = stable_out;
		endcase
	end

	always_ff@(posedge clk) begin
		if(reset) begin
			stable_out <= 'd0;
			sync_counter <= 'd0;
		end else begin 
			stable_out <= stable_out_next;
			sync_counter <= sync_counter_next;
		end 
	end

endmodule


/*
 * uart_baud_tick_gen.v
 * 2017/02/01 - Felipe Veas <felipe.veasv at usm.cl>
 *
 * Baud clock generator for UART, original code from:
 * http://fpga4fun.com/SerialInterface.html
 */


module uart_baud_tick_gen
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

	always @(posedge clk)
		if (enable)
			acc <= acc[ACC_WIDTH-1:0] + INCREMENT[ACC_WIDTH:0];
		else
			acc <= INCREMENT[ACC_WIDTH:0];

	assign tick = acc[ACC_WIDTH];

endmodule //uart_baud_tick_gen
