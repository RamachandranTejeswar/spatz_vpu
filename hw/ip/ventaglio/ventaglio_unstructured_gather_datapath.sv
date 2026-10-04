module unstructured_gather_datapath
  import spatz_pkg::*;
#(
  parameter int unsigned NrGatherLanes = 8,
  parameter int unsigned NrBanks       = VTGNrChannels * VTGNrBanksPerChannel
) (
  input  logic                          clk_i,
  input  logic                          rst_ni,

  input  logic                          enable_i,           // Enable input for the unstructured datapath so it doenst turn on with structured datapath simultaneously

  input  vrf_data_t                     index_i,            // one 256-bit index word = 16 indices of 16 bits
  input  logic                          index_valid_i,      // 1 = index_i holds a real word

  input  vlen_t                         vl_i,               // VL is required to handle tails
  input  logic                          gather_ready_i,     // From VFU indicating a data request
  
  input  vtg_bank_data_t [NrBanks-1:0]  bank_rdata_i,

  output logic                          load_index_o,       // Asserted when an index word is processed fully by the gather datapath
  output vtg_row_addr_t [NrBanks-1:0]   bank_row_o,
  output vrf_data_t                     gather_data_o,      // 8 activations x 32 bits = one 256-bit word for the VFU
  output logic                          gather_valid_o      // 1 = gather_data_o holds a full beat
);

    `include "common_cells/registers.svh"

    /**** Signals for the Index Processing Logic ****/
    
    localparam int unsigned IdxWidth = 16;
    logic beat_d, beat_q;
    logic beat_done;
    
    /**** Signals for the Crossbar ****/

    logic           [NrGatherLanes-1:0]                         lane_valid;   // Lane has a valid request
    logic           [NrGatherLanes-1:0][$clog2(NrBanks)-1:0]    lane_bank;    // Which bank does the request from the lane go to?
    vtg_row_addr_t  [NrGatherLanes-1:0]                         lane_row;     // Which row does the request from the lane go to?
    logic           [NrGatherLanes-1:0]                         lane_served;  // Has the request gone through this cycle?

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

    /////////////////////////////////
    //    Index Processing Logic   //
    /////////////////////////////////
    
    // beat tracker - Beat 0 or Beat 1
    `FF(beat_q, beat_d, 1'b0)
    // Element vcounter for tracking tails
    `FF(elem_cnt_q, elem_cnt_d, '0)

    // Split one beat from the index word into 8 separate lane requests
    for (genvar l = 0; l < NrGatherLanes; l++) begin : gen_lane_req
        logic [IdxWidth-1:0] idx;
        assign idx = index_i[IdxWidth*(NrGatherLanes*beat_q + l) +: IdxWidth];
        
        assign lane_in_range[l] = (elem_cnt_q + l) < vl_i;   // does this lane have a real element?
        // lane request is valid only if it has a valid index, if it is not already done in this beat, and if the lane is in range
        // lane_valid is also gated by the Unstructured sparsity datapath's enable signal
        assign lane_valid[l] = enable_i && index_valid_i && !lane_done_q[l] && lane_in_range[l];
         
        assign lane_bank[l]  = idx[6:2];                            // which of the 32 banks
        assign lane_row[l]   = idx[9:7];                            // which of the 8 rows
    end

    // assign beat_done = index_valid_i && (&lane_served);

    assign last_beat = (elem_cnt_q + NrGatherLanes) >= vl_i;
    assign elem_cnt_d = !beat_done ? elem_cnt_q : last_beat ? '0 : elem_cnt_q + NrGatherLanes;

    // lane_done_q keeps track of lanes completed in previous cycles, and lane_served updates it with lanes served this cycle.
    // lane_in_range keeps track of all additional lanes which are out of range for a specific beat to ensure those zeroes do not hold back beat done
    // beat_done is asserted also only when gather_ready_i is asserted, indicating the VFU has actually requested for the data and the 8-elements (1 beat) can be sent over

    assign beat_done = index_valid_i && gather_ready_i && (&(lane_done_q | lane_served | ~lane_in_range));

    assign lane_done_d = beat_done ? '0 : (lane_done_q | lane_served);
    assign beat_d = !beat_done ? beat_q : last_beat ? 1'b0 : ~beat_q;

    assign load_index_o = beat_done && (beat_q || last_beat);


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

    for (genvar g = 0; g < NrGatherLanes; g++) begin : gen_lane_data
        // The data requested by lane g being returned from a certain bank
        assign lane_rdata[g] = bank_rdata_i[lane_bank[g]];
    end
    
    /* HANDLING CONFLICTS */

    `FF(lane_done_q, lane_done_d, '0)

    for (genvar g = 0; g < NrGatherLanes; g++) begin : gen_lane_save
        // Load only when this lane was newly served otherwise keep the old value
        `FFL(lane_data_q[g], lane_rdata[g], lane_served[g], '0)
    end


    /////////////////////////////////////////
    //     Consolidating received data     //
    /////////////////////////////////////////

    // assign gather_data_o  = lane_rdata;

    // If a lane was just served, it might not be able to send the updated newly received value at gather_data_o since it gets sampled by the flip flop only in the next cycle.
    // Therefore, if a lane was newly served, send out only the newly sampled value. 
    // Else if the value was already served in an earlier cycle, then send out the sampled value
    for (genvar g = 0; g < NrGatherLanes; g++) begin : gen_gather_out
        assign gather_data_o[VTGBankWidth*g +: VTGBankWidth] = lane_served[g] ? lane_rdata[g] : lane_data_q[g];
    end
    assign gather_valid_o = beat_done;

endmodule