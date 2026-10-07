

typedef struct packed {
    spatz_id_t      id;
    vreg_t          idx_vreg;
    logic           unstructured;
    vlen_t          vl;
} idx_job_t;

idx_job_t idx_job_in, idx_job_out;
logic idx_job_valid, idx_job_ready;

assign idx_job_in = '{
    id: spatz_req_i.id,
    idx_vreg: spatz_req_i.op_vtl.idx_vreg,
    unstructured: is_unstructured,
    vl: spatz_req_i.vl
};


// 3. One spill register carries the whole bundle
spill_register 
#(
  .T(idx_job_t) 
) i_idx_job_list (
  .clk_i  (clk_i        ),
  .rst_ni (rst_ni       ),
  .data_i (idx_job_in   ),
  .valid_i(new_op       ),
  .ready_o(/* unused */ ),
  .data_o (idx_job_out  ),
  .valid_o(idx_job_valid),
  .ready_i(idx_job_ready)   // the generator drives this when it is done with a job
);


/******************************/
/*      Request Generator     */
/******************************/
 
localparam int unsigned IdxBufDepth = 2;
localparam int unsigned IdxPerWord = 16;

logic idx_req_valid;
logic idx_req_accepted;  // request handed to the request register this cycle
logic idx_buf_pop;       // a word left the index buffer this cycle
logic idx_credit_left;   // 1 = there is still a free slot

logic last_word;

logic [$clog2(8):0] word_cnt_d, word_cnt_q;
logic [$clog2(8):0] num_words;

assign num_words            = idx_job_out.unstructured ? (idx_job_out.vl + IdxPerWord - 1) / IdxPerWord : 1;   // round up vl/16 to CEIL
assign last_word            = (word_cnt_q == num_words - 1);

assign idx_buf_pop          = (/*unstructured signal*/) ? load_index_o : (spatz_req_valid && spatz_req_ready && !spatz_req.op_vtl.clear_buffer);
assign idx_req_valid        = idx_job_valid && idx_credit_left;
assign idx_req_reg_ready    = /*ready_o of the idx_req register*/;
assign idx_req_accepted     = idx_req_valid & idx_req_reg_ready;

credit_counter #(
  .NumCredits     (IdxBufDepth)
) i_idx_credit (
  .clk_i          (clk_i           ),
  .rst_ni         (rst_ni          ),
  .credit_o       (/* unused */    ),
  .credit_give_i  (idx_buf_pop     ),
  .credit_take_i  (idx_req_accepted),
  .credit_init_i  (1'b0            ),
  .credit_left_o  (idx_credit_left ),
  .credit_crit_o  (/* unused */    ),
  .credit_full_o  (/* unused */    )
);

// What to send: the job's ID and the address of the current word
assign idx_req_in = '{
  id:   idx_job_out.id,
  addr: {idx_job_out.idx_vreg, $clog2(NrWordsPerVector)'(1'b0)} + word_cnt_q
};

`FF(word_cnt_q, word_cnt_d, '0);

always_comb begin
    word_cnt_d = word_cnt_q;
    idx_job_ready = 1'b0;

    if(idx_req_accepted) begin
        if(last_word) begin
            word_cnt_d = '0;
            idx_job_ready = 1'b1;
        end
        else begin
            word_cnt_d = word_cnt_q + 1;
        end
    end
end


///////////////////////////////
//   Index Request Register  //
///////////////////////////////

spill_register #(
  .T(idx_req_t)
) i_idx_req_reg (
  .clk_i  (clk_i            ),
  .rst_ni (rst_ni           ),
  .data_i (idx_req_in       ),
  .valid_i(idx_req_valid    ),
  .ready_o(idx_req_reg_ready),
  .data_o (idx_req_out      ),
  .valid_o(vrf_re_o         ),
  .ready_i(vrf_rvalid_i     )
);

///////////////////////////////
//    Index Spill Register   //
///////////////////////////////


spill_register #(
  .T(vrf_data_t)
) i_idx_spill (
  .clk_i  (clk_i        ),
  .rst_ni (rst_ni       ),
  .data_i (vrf_rdata_i  ),
  .valid_i(vrf_rvalid_i ),
  .ready_o(/* unused */ ),   // credits guarantee there's always room
  .data_o (index_q      ),
  .valid_o(index_valid_q),
  .ready_i(idx_buf_pop  )
);


// 2D Address - Pick Channel, then Pick Bank per channel
vtg_row_addr_t [VTGNrChannels-1:0][VTGNrBanksPerChannel-1:0] usp_raddr;

logic usp_active; // Are we doing an unstructured gather?
assign usp_active = spatz_req_valid && spatz_req.op_vtl.gather_vs2;

vtg_row_addr_t vtg_raddr;

assign vtg_raddr = usp_active ? usp_raddr[channel][bank] : raddr[channel]

.raddr_i (vtg_raddr);


// Crossbar for Unstructured Gather

localparam int unsigned NrGatherLanes = 8;
localparam int unsigned NrBanks = VTGNrChannels * VTGNrBanksPerChannel;

logic [NrGatherLanes-1:0]                       lane_valid;   // Lane has a valid request
logic [NrGatherLanes-1:0][$clog2(NrBanks)-1:0]  lane_bank;    // Which bank does the request from the lane go to?
vtg_row_addr_t [NrGatherLanes-1:0]              lane_row;     // Which row does the request from the lane go to?
logic [NrGatherLanes-1:0]                       lane_served;  // Has the request gone through this cycle?

vtg_row_addr_t [NrBanks-1:0]                    bank_row;     // Which row of the bank is being read this cycle?
logic [NrBanks-1:0]                             bank_valid;   // Bank is being read this cycle

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


  localparam int unsigned NrLanes = 8;
  localparam int unsigned NrBanks = VTGNrChannels * VTGNrBanksPerChannel;  // 32

  logic          [NrLanes-1:0]                       lane_valid;   // lane has a request
  logic          [NrLanes-1:0][$clog2(NrBanks)-1:0]  lane_bank;    // which bank
  vtg_row_addr_t [NrLanes-1:0]                       lane_row;     // which row
  logic          [NrLanes-1:0]                       lane_served;  // request went through this cycle

  vtg_row_addr_t [NrBanks-1:0]                       bank_row;     // row each bank should read this cycle
  logic          [NrBanks-1:0]                       bank_valid;   // bank is being read this cycle

  stream_xbar #(
    .NumInp   (NrLanes       ),
    .NumOut   (NrBanks       ),
    .payload_t(vtg_row_addr_t),
    .LockIn   (1'b0          ),   // let a waiting lane retry freely
    .AxiVldRdy(1'b0          )
  ) i_usp_xbar (
    .clk_i  (clk_i      ),   
    .rst_ni (rst_ni     ),
    .flush_i(1'b0       ),
    .rr_i   ('0         ),   
    .data_i (lane_row   ),
    .sel_i  (lane_bank  ),
    .valid_i(lane_valid ),
    .ready_o(lane_served),
    .data_o (bank_row   ),
    .idx_o  (/* unused */),
    .valid_o(bank_valid ),   
    .ready_i('1         )     // a bank can always take one read per cycle
  );
    
  assign usp_raddr = bank_row;   // 32 entries, same order as [channel][bank]


for (genvar g = 0; g < NrGatherLanes; g++) begin :gen_lane_data
  // The data request by lane l being returned from a certain bank
  assign lane_rdata[g] = bank_rdata_i[lane_bank[l]];
end


  for (genvar l = 0; l < NrGatherLanes; l++) begin : gen_lane_data
    // Bank l asked for, picked straight from the 32 bank outputs
    assign lane_rdata[l] = bank_rdata_i[lane_bank[l]];
   
    // Keep it once this lane has been served
    `FFL(lane_data_q[l], lane_rdata[l], lane_served[l], '0)
  end

/**************************************************************************************************/

/**** GATHER DATAPATH ****/

localparam int unsigned IdxWidth = 16;
logic beat_d, beat_q;

'FF(beat_q, beat_d, 1'b0);

for(genvar g = 0; g < NrGatherLanes; g++) begin : gen_lane_req
  logic [IdxWidth-1:0] idx;
  assign idx = index_i[IdxWidth*(NrGatherLanes*beat_q + l) +: IdxWidth];
end

  localparam int unsigned IdxWidth = 16;   // each index is 16 bits
 
  // beat tracker - Beat 0 or Beat 1
  logic beat_q, beat_d;
  `FF(beat_q, beat_d, 1'b0)

  // Split one beat from the index word into 8 separate lane requests
  for (genvar l = 0; l < NrGatherLanes; l++) begin : gen_lane_req
    logic [IdxWidth-1:0] idx;
    assign idx = index_i[IdxWidth*(NrGatherLanes*beat_q + l) +: IdxWidth];
    
    assign lane_valid[l] = index_valid_i;
    assign lane_bank[l]  = idx[6:2];   // which of the 32 banks
    assign lane_row[l]   = idx[9:7];   // which of the 8 rows
  end

  logic beat_done;
  assign beat_done = index_valid_i && (&lane_served);

  assign beat_d = beat_done ? ~beat_q : beat_q;

  assign load_index_o = beat_done && beat_q;

  /**** GATHER DATAPATH - Data Consolidation ****/

   // New outputs
  output vrf_data_t   gather_data_o,    // 8 activations x 32 bits = one 256-bit word for the VFU
  output logic        gather_valid_o,   // 1 = gather_data_o holds a full beat

  // Lane 0 goes to bits 31:0, lane 1 to bits 63:32, ... lane 7 to bits 255:224
  assign gather_data_o  = lane_rdata;
  assign gather_valid_o = beat_done;

  // Instantiating the modules in the main VTL datapath

      unstructured_gather_datapath i_usp_gather (
      .clk_i         (clk_i         ),
      .rst_ni        (rst_ni        ),
      // index word from the index buffer
      .index_i       (index_q       ),
      .index_valid_i (index_valid_q ),
      // the rest gets connected in the next steps
      .bank_rdata_i  (              ),
      .load_index_o  (              ),
      .bank_row_o    (              ),
      .gather_data_o (              ),
      .gather_valid_o(              )
    );


    vtg_bank_data_t [NrGatherLanes-1:0] lane_data_q;   // value saved by each lane

  for (genvar g = 0; g < NrGatherLanes; g++) begin : gen_lane_save
    // Load only when this lane is served; otherwise keep the old value
    `FFL(lane_data_q[g], lane_rdata[g], lane_served[g], '0)
  end

  for (genvar g = 0; g < NrGatherLanes; g++) begin : gen_gather_out
    assign gather_data_o[VTGBankWidth*g +: VTGBankWidth] =
        lane_served[g] ? lane_rdata[g] : lane_data_q[g];
  end



