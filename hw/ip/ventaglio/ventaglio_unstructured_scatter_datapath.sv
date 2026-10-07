module unstructured_scatter_datapath
  import spatz_pkg::*;
#(
    parameter int unsigned NrGatherLanes = 8,
    parameter int unsigned NrBanks       = VTGNrBankChannels * VTGNrBanksPerChannel
) (
    input  logic                          clk_i,
    input  logic                          rst_ni,
     
    input  logic                          enable_i,         // a vsxu.vv is the active instruction
    input  vrf_data_t                     index_i,          // index word (32 x 8-bit column numbers)
    input  logic                          index_valid_i,    // index_i holds a real word
    input  vlen_t                         vl_i,             // instruction length, for tails
 
    input  vrf_data_t                     wdata_i,          // one beat from the VFU: 8 values x 32 bits
    input  logic                          wvalid_i,         // the VFU is delivering a beat this cycle
    output logic                          wready_o,         // the beat is fully written: VFU may send the next

    output logic                          load_index_o,     // done with the current index word
    output vtg_row_addr_t  [NrBanks-1:0]  bank_row_o,       // row each bank writes this cycle
    output vtg_bank_data_t [NrBanks-1:0]  bank_wdata_o,     // value each bank writes this cycle
    output logic           [NrBanks-1:0]  bank_we_o,        // which banks write this cycle
    output logic                          job_done_o
);

    `include "common_cells/registers.svh"

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

    logic           [NrGatherLanes-1:0]                         lane_done_q, lane_done_d;

    logic           [NrBanks-1:0]                               bank_valid;   // Bank is being read this cycle

    /**** Additional Signals for Conflict Handling ****/

    vtg_bank_data_t [NrGatherLanes-1:0]                         lane_wdata;   // Data each lane gets back from VFU this cycle


    /**** Handling Tails ****/

    vlen_t elem_cnt_q, elem_cnt_d;              // Track element count
    logic [NrGatherLanes-1:0] lane_in_range;    // collect lane_in_range from the loop
    logic last_beat;                            // Is this the last beat of the instruction


    /**** Signals for writing data back into VTL Bank via lanes ****/

    typedef struct packed {
    vtg_row_addr_t  row;    // which row of the bank to write
    vtg_bank_data_t data;   // the value to write there
    } scatter_req_t;

    scatter_req_t [NrGatherLanes-1:0] lane_req;   // what each lane wants to write
    scatter_req_t [NrBanks-1:0]       bank_req;   // what each bank receives this cycle

    /////////////////////////////////
    //    Index Processing Logic   //
    /////////////////////////////////
    
    // beat tracker - Beat 0 or Beat 1
    `FF(beat_q, beat_d, '0)
    // Element vcounter for tracking tails
    `FF(elem_cnt_q, elem_cnt_d, '0)

    `FF(lane_done_q, lane_done_d, '0)


    // Split one beat from the index word into 8 separate lane requests
    for (genvar g = 0; g < NrGatherLanes; g++) begin : gen_lane_req
        logic [IdxWidth-1:0] idx;
        assign idx = index_i[IdxWidth*(NrGatherLanes*beat_q + g) +: IdxWidth];
        
        assign lane_in_range[g] = (elem_cnt_q + g) < vl_i;   // does this lane have a real element?

        // lane request is valid only if wvalid_i is asserted for that lane - which happens only wehewn VFU is ready with some write data
        assign lane_valid[g] = enable_i && index_valid_i && !lane_done_q[g] && lane_in_range[g] && wvalid_i;
         
        // assign lane_bank[g]  = idx[4:0];                         // which of the 32 banks
		assign lane_bank[g] = idx[BankBits-1:0];					// index bits 5:0 -> which of the 64 banks        
		
		// assign lane_row[g]   = idx[7:5];                         // which of the 8 rows
		assign lane_row[g]  = idx[BankBits +: RowBits];             // index bits 7:6 -> which of the 4 rows

        assign lane_wdata[g] = wdata_i[VTGBankWidth*g +: VTGBankWidth];

        // Sending data to the VTL bank via lanes
        assign lane_req[g] = '{row: lane_row[g], data: lane_wdata[g]};
    end

    // assign beat_done = index_valid_i && (&lane_served);

    assign last_beat = (elem_cnt_q + NrGatherLanes) >= vl_i;        // Mainly for handling tails
    assign elem_cnt_d = !beat_done ? elem_cnt_q : last_beat ? '0 : elem_cnt_q + NrGatherLanes;

    // lane_done_q keeps track of lanes completed in previous cycles, and lane_served updates it with lanes served this cycle.
    // lane_in_range keeps track of all additional lanes which are out of range for a specific beat to ensure those zeroes do not hold back beat done
    // beat_done is asserted also only when gather_ready_i is asserted, indicating the VFU has actually requested for the data and the 8-elements (1 beat) can be sent over

    // assign beat_done = index_valid_i & gather_ready_i & (&(lane_done_q | lane_served | ~lane_in_range));
	assign beat_done   = enable_i & index_valid_i & wvalid_i & (&(lane_done_q | lane_served | ~lane_in_range));

    // assign lane_done_d = beat_done ? '0 : (lane_done_q | lane_served);
	 assign lane_done_d = beat_done ? '0 : (lane_done_q | lane_served);   
	
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
    .payload_t (scatter_req_t),
    .LockIn (1'b0),
    .AxiVldRdy(1'b0)
    ) i_usp_xbar (
        .clk_i  (clk_i    ),   
        .rst_ni (rst_ni     ),
        .flush_i(1'b0       ),
        .rr_i   ('0         ),   
        .data_i (lane_req   ),
        .sel_i  (lane_bank  ),
        .valid_i(lane_valid ),
        .ready_o(lane_served),
        .data_o (bank_req   ),
        .idx_o  (           ),
        .valid_o(bank_valid ),   
        .ready_i('1         )
    );

    for (genvar b = 0; b < NrBanks; b++) begin: gen_bank_out
        assign bank_row_o[b] = bank_req[b].row;
        assign bank_wdata_o[b] = bank_req[b].data;
    end

    assign bank_we_o = bank_valid;  // Bank writes data only when data was routed to it via a lane this cycle

    assign wready_o    = beat_done;
    assign job_done_o = beat_done && last_beat;


endmodule
