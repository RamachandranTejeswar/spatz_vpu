module unstructured_gather_datapath
  import spatz_pkg::*;
#(
  parameter int unsigned NrGatherLanes = 8,
  // parameter int unsigned NrBanks       = VTGNrChannels * VTGNrBanksPerChannel
  parameter int unsigned NrBanks       = VTGNrBankChannels * VTGNrBanksPerChannel   // 8 x 8 = 64
) (
  input  logic                          clk_i,
  input  logic                          rst_ni,

  input  logic                          enable_i,           // Enable input for the unstructured datapath so it doenst turn on with structured datapath simultaneously

  input  vrf_data_t                     index_i,            // one 256-bit index word = 16 indices of 16 bits
  input  logic                          index_valid_i,      // 1 = index_i holds a real word

  input  vlen_t                         vl_i,               // VL is required to handle tails
  input  logic                          gather_ready_i,     // From VFU indicating a data request
  
  input  vtg_bank_data_t [NrBanks-1:0]  bank_rdata_i,
   input  vrf_addr_t                     raddr_i,
   input  logic                          op_pop_i,

  output logic                          load_index_o,       // Asserted when an index word is processed fully by the gather datapath
  output vtg_row_addr_t [NrBanks-1:0]   bank_row_o,
  output vrf_data_t                     gather_data_o,      // 8 activations x 32 bits = one 256-bit word for the VFU
  output logic                          gather_valid_o      // 1 = gather_data_o holds a full beat
);

    `include "common_cells/registers.svh"

    /**** Signals for the Index Processing Logic ****/
    
    localparam int unsigned IdxWidth = 8;
    localparam int unsigned NrBeatsPerWord  = VRFWordWidth / (IdxWidth * NrGatherLanes);    // 256 / 64 = 4
	localparam int unsigned BankBits = $clog2(NrBanks);                 // 64 banks -> 6 bits, number of bits for bank addressing depends on NrBanks
	localparam int unsigned RowBits  = $clog2(VTGNrWordsPerChannel);    // 4 rows   -> 2 bits, number of bits for bank addressing depends on Number of Words/Rows per Bank

    // Maximum up to 4 beats per word
    logic [$clog2(NrBeatsPerWord)-1:0] beat_d, beat_q;
    logic beat_done;
    
    /**** Signals for the Crossbar ****/

    logic           [NrGatherLanes-1:0]                         lane_valid;   // Lane has a valid request
    logic           [NrGatherLanes-1:0][$clog2(NrBanks)-1:0]    lane_bank;    // Which bank does the request from the lane go to?
    vtg_row_addr_t  [NrGatherLanes-1:0]                         lane_row;     // Which row does the request from the lane go to?
    logic           [NrGatherLanes-1:0]                         lane_served;  // Has the request gone through this cycle?
	
	// Broadcast Logic
	logic 			[NrGatherLanes-1:0] 						lane_got_data;	// lane received its value this cycle

    logic           [NrGatherLanes-1:0]                         lane_done_q, lane_done_d;

    vtg_row_addr_t  [NrBanks-1:0]                               bank_row;     // Which row of the bank is being read this cycle?
    logic           [NrBanks-1:0]                               bank_valid;   // Bank is being read this cycle

    /**** Additional Signals for Conflict Handling ****/

    vtg_bank_data_t [NrGatherLanes-1:0]                         lane_rdata;   // Data each lane got back this cycle
    vtg_bank_data_t [NrGatherLanes-1:0]                         lane_data_q;


    /**** Handling Tails ****/

    vlen_t elem_cnt_q, elem_cnt_d;              // Track element count
    logic [NrGatherLanes-1:0] lane_in_range;    // collect lane_in_range from the loop
    logic last_beat;                            // Is this the last beat of the instruction

    /**** Signals for resolving final bugs ****/
    logic      served_valid_q;    // a beat was sent and might be asked for again
  vrf_addr_t served_raddr_q;    // the word address it was sent for
  vrf_data_t served_data_q;     // the 256-bit beat that was sent
  logic      repeat_req;        // the VFU is asking again for the word it already got
  vrf_data_t new_beat;          // the freshly gathered beat (what used to be gather_data_o)


    /////////////////////////////////
    //    Index Processing Logic   //
    /////////////////////////////////
    
    // beat tracker - Beat 0 or Beat 1
    `FF(beat_q, beat_d, '0)
    // Element vcounter for tracking tails
    `FF(elem_cnt_q, elem_cnt_d, '0)


    // Split one beat from the index word into 8 separate lane requests
    for (genvar g = 0; g < NrGatherLanes; g++) begin : gen_lane_req
        logic [IdxWidth-1:0] idx;
        assign idx = index_i[IdxWidth*(NrGatherLanes*beat_q + g) +: IdxWidth];
        
        assign lane_in_range[g] = (elem_cnt_q + g) < vl_i;   // does this lane have a real element?
        // lane request is valid only if it has a valid index, if it is not already done in this beat, and if the lane is in range
        // lane_valid is also gated by the Unstructured sparsity datapath's enable signal
        assign lane_valid[g] = enable_i && index_valid_i && !lane_done_q[g] && lane_in_range[g];
         
        // assign lane_bank[g]  = idx[4:0];                         // which of the 32 banks
		assign lane_bank[g] = idx[BankBits-1:0];					// index bits 5:0 -> which of the 64 banks        
		
		// assign lane_row[g]   = idx[7:5];                         // which of the 8 rows
		assign lane_row[g]  = idx[BankBits +: RowBits];             // index bits 7:6 -> which of the 4 rows
    end

    // assign beat_done = index_valid_i && (&lane_served);

    assign last_beat = (elem_cnt_q + NrGatherLanes) >= vl_i;        // Mainly for handling tails
    assign elem_cnt_d = !beat_done ? elem_cnt_q : last_beat ? '0 : elem_cnt_q + NrGatherLanes;

    // lane_done_q keeps track of lanes completed in previous cycles, and lane_served updates it with lanes served this cycle.
    // lane_in_range keeps track of all additional lanes which are out of range for a specific beat to ensure those zeroes do not hold back beat done
    // beat_done is asserted also only when gather_ready_i is asserted, indicating the VFU has actually requested for the data and the 8-elements (1 beat) can be sent over

    // assign beat_done = index_valid_i & gather_ready_i & (&(lane_done_q | lane_served | ~lane_in_range));
	// assign beat_done   = index_valid_i & gather_ready_i & (&(lane_done_q | lane_got_data | ~lane_in_range));
    assign beat_done = index_valid_i & gather_ready_i & !repeat_req
                   & (&(lane_done_q | lane_got_data | ~lane_in_range));

    assign repeat_req = gather_ready_i && served_valid_q && (raddr_i == served_raddr_q);
 
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      served_valid_q <= 1'b0;
      served_raddr_q <= '0;
      served_data_q  <= '0;
    end else if (op_pop_i) begin               // instruction finished: forget it
      served_valid_q <= 1'b0;
    end else if (beat_done) begin              // a new beat was sent: remember it
      served_valid_q <= 1'b1;
      served_raddr_q <= raddr_i;
      served_data_q  <= new_beat;
    end
  end


    // assign lane_done_d = beat_done ? '0 : (lane_done_q | lane_served);
	assign lane_done_d = beat_done ? '0 : (lane_done_q | lane_got_data);    
	
	// assign beat_d = !beat_done ? beat_q : last_beat ? 1'b0 : ~beat_q;
    assign beat_d = !beat_done ? beat_q : last_beat ? '0 : beat_q + 1;

    // assign load_index_o = beat_done && (beat_q | last_beat);
    // assign load_index_o = beat_done & (beat_q | last_beat);
    assign load_index_o = beat_done && ((beat_q == NrBeatsPerWord-1) || last_beat);


    ////////////////////////////////////////
    // Crossbar b/w VTL Bank and Datapath //
    ////////////////////////////////////////

    stream_xbar #(
    .NumInp (NrGatherLanes),
    .NumOut (NrBanks),
    .payload_t (vtg_row_addr_t),
    .LockIn (1'b0),
    .AxiVldRdy(1'b0)
    ) i_usp_xbar (
        .clk_i  (clk_i    ),   
        .rst_ni (rst_ni     ),
        .flush_i(1'b0       ),
        .rr_i   ('0         ),   
        .data_i (lane_row   ),
        .sel_i  (lane_bank  ),
        .valid_i(lane_valid ),
        .ready_o(lane_served),
        .data_o (bank_row   ),
        .idx_o  (           ),
        .valid_o(bank_valid ),   
        .ready_i('1         )
    );

    // Send request to the banks with row numbers
    assign bank_row_o = bank_row;

    //for (genvar g = 0; g < NrGatherLanes; g++) begin : gen_lane_data
        // The data requested by lane g being returned from a certain bank
    //    assign lane_rdata[g] = bank_rdata_i[lane_bank[g]];
    //end

	for (genvar g = 0; g < NrGatherLanes; g++) begin : gen_lane_data
		assign lane_rdata[g] = bank_rdata_i[lane_bank[g]];
		
		// Used to mark lane as served though the same data is picked and sent back via a different lane
		assign lane_got_data[g] = lane_valid[g] & bank_valid[lane_bank[g]] & (bank_row[lane_bank[g]] == lane_row[g]);	
	end

    
    /* HANDLING CONFLICTS */

    `FF(lane_done_q, lane_done_d, '0)

    for (genvar g = 0; g < NrGatherLanes; g++) begin : gen_lane_save
        // Load only when this lane was newly served otherwise keep the old value
        // `FFL(lane_data_q[g], lane_rdata[g], lane_served[g], '0)
		`FFL(lane_data_q[g], lane_rdata[g], lane_got_data[g], '0)
    end


    /////////////////////////////////////////
    //     Consolidating received data     //
    /////////////////////////////////////////

    // assign gather_data_o  = lane_rdata;

    // If a lane was just served, it might not be able to send the updated, newly received value at gather_data_o since it gets sampled by the flip flop only in the next cycle.
    // Therefore, if a lane was newly served, send out only the newly sampled value. 
    // Else if the value was already served in an earlier cycle, then send out the sampled value
    for (genvar g = 0; g < NrGatherLanes; g++) begin : gen_gather_out
        assign new_beat[VTGBankWidth*g +: VTGBankWidth] = lane_got_data[g] ? lane_rdata[g] : lane_data_q[g];
    end

  assign gather_data_o  = repeat_req ? served_data_q : new_beat;
  assign gather_valid_o = beat_done | repeat_req;

endmodule
