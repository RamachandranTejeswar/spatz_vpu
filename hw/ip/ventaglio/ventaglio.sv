// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Bowen Wang, ETH Zurich
//
// Ventaglio — streamlined sparse gather/scatter execution unit.
//
//   Bank       : VTGNrBankChannels channels x N_FU banks (ELEN each), channel-
//                interleaved (f_channel/f_row). Holds the sparsity-expanded
//                accumulator vreg(s).
//   Op queue   : i_operation_queue spill latches each use_vtl op
//                (vfxmul / vfxmacc / vventclr) issued by the controller.
//   Index path : two spill registers. i_idx_req_reg buffers the master VRF
//                *read request* for the op's index vreg (pushed on the
//                pre-spill admit handshake, held until vrf_rvalid_i).
//                i_idx_spill buffers the returned index *data* that feeds the
//                scatter/gather datapaths.
//   Datapaths  : ventaglio_scatter (vfx -> bank), ventaglio_gather
//                (bank -> vfx), plus the Clear Sequencer for vventclr.
//   VRF ports  : slave ports serve controller-redirected VLSU/VFU accesses to
//                bank vregs; the master read port fetches indices from the VRF.
//

module ventaglio
  import spatz_pkg::*;
  import rvv_pkg::*;
  import vtl_pkg::*;
  #(
    parameter int unsigned NarrowDataWidth = VTGChannelWidth,
    parameter int unsigned WideDataWidth   = VTGChannelWidth * VENTAGLIO_WFACTOR
  ) (
    input  logic            clk_i,
    input  logic            rst_ni,
    input  logic            testmode_i,

    // Spatz request
    input  spatz_req_t       spatz_req_i,
    input  logic             spatz_req_valid_i,
    // VTL admit handshake to the controller. High when the spill
    // register can accept the next use_vtl op (vventclr or vfx).
    output logic             vtl_req_ready_o,
    input  logic             spatz_vfu_req_ready_i,
    // VTL response (retirement path for vventclr; vfx retires via vfu_rsp)
    output logic             vtl_rsp_valid_o,
    output vsldu_rsp_t       vtl_rsp_o,
    // VFU response
    input  logic             vfu_rsp_valid_i,
    input  vfu_rsp_t         vfu_rsp_i,
    // Slave Write port (single)
    input  vrf_addr_t        waddr_i,
    input  vrf_data_t        wdata_i,
    input  logic             we_i,
    input  vrf_be_t          wbe_i,
    output logic             wvalid_o,
    input  logic             wscatter_en_i,
    // Slave Read port (single)
    input  vrf_addr_t        raddr_i,
    input  logic             re_i,
    output vrf_data_t        rdata_o,
    output logic             rvalid_o,
    input  logic             rgather_en_i,

    // Master VRF interface
    output vrf_addr_t                       vrf_waddr_o,
    output vrf_data_t                       vrf_wdata_o,
    output logic                            vrf_we_o,
    output vrf_be_t                         vrf_wbe_o,
    input  logic                            vrf_wvalid_i,
    output spatz_id_t  [1:0]                vrf_id_o,
    output vrf_addr_t                       vrf_raddr_o,
    output logic                            vrf_re_o,
    input  vrf_data_t                       vrf_rdata_i,
    input  logic                            vrf_rvalid_i
  );

// Include FF
`include "common_cells/registers.svh"

  ///////////////////////
  //  Operation queue  //
  ///////////////////////

  spatz_req_t spatz_req;
  logic       spatz_req_valid;
  logic       spatz_req_ready;

  spill_register #(
    .T(spatz_req_t)
  ) i_operation_queue (
    .clk_i  (clk_i                                              ),
    .rst_ni (rst_ni                                             ),
    .data_i (spatz_req_i                                        ),
    .valid_i(spatz_req_valid_i && spatz_req_i.op_vtl.use_vtl    ),
    .ready_o(vtl_req_ready_o                                    ),
    .data_o (spatz_req                                          ),
    .valid_o(spatz_req_valid                                    ),
    .ready_i(spatz_req_ready                                    )
  );


  // State Handling signals
  logic [NrParallelInstructions-1:0] running_d, running_q;
  `FF(running_q, running_d, '0)
  logic new_vtl_request;
  assign new_vtl_request = spatz_req_valid && !running_q[spatz_req.id];

  /******************************/
  /*       Clear Sequencer      */
  /******************************/

  logic           clear_active_q,  clear_active_d;
  vtg_row_addr_t  clear_row_q,     clear_row_d;
  spatz_id_t      clear_id_q,      clear_id_d;
  `FF(clear_active_q, clear_active_d, 1'b0)
  `FF(clear_row_q,    clear_row_d,    '0)
  `FF(clear_id_q,     clear_id_d,     '0)

  logic clear_last_row;
  assign clear_last_row = (clear_row_q == vtg_row_addr_t'(VTGNrWordsPerChannel - 1));

  // True for the cycle that retires the clear op.
  logic clear_done;
  assign clear_done = clear_active_q && clear_last_row;

  always_comb begin : proc_clear
    clear_active_d = clear_active_q;
    clear_row_d    = clear_row_q;
    clear_id_d     = clear_id_q;

    // Start clearing when a clear_buffer op is freshly latched in the spill.
    if (!clear_active_q && new_vtl_request && spatz_req.op_vtl.clear_buffer) begin
      clear_active_d = 1'b1;
      clear_row_d    = '0;
      clear_id_d     = spatz_req.id;
    end else if (clear_active_q) begin
      if (clear_last_row) begin
        clear_active_d = 1'b0;
      end else begin
        clear_row_d = clear_row_q + 1'b1;
      end
    end
  end

  // Retire the clear op via the SLD response port.
  assign vtl_rsp_valid_o = clear_done;
  assign vtl_rsp_o.id    = clear_id_q;


  /******************************/
  /*       State Handler        */
  /******************************/
  always_comb begin : proc_vtl_state
    running_d = running_q;
    // Default: no op latched in the spill -> ready to accept whatever arrives.
    spatz_req_ready = !spatz_req_valid;

    // A new spatz_req is received
    if (new_vtl_request) begin
      running_d[spatz_req.id] = 1'b1;
    end

    // VFU-routed ops retire via vfu_rsp.
    if (running_q[vfu_rsp_i.id] && vfu_rsp_valid_i) begin
      running_d[vfu_rsp_i.id] = 1'b0;
    end
    // vventclr retires via clear_done (vtl_rsp_valid_o).
    if (clear_done && running_q[clear_id_q]) begin
      running_d[clear_id_q] = 1'b0;
    end
    // clean done
    if (spatz_req_valid && spatz_req.op_vtl.clear_buffer) begin
      if (clear_done) spatz_req_ready = 1'b1;
    // macc done
    end else if (spatz_vfu_req_ready_i) begin
      spatz_req_ready = 1'b1;
    end
  end

  //////////////////////////////////
  //   Index Buffer (FIFO)        //
  //////////////////////////////////


  /******************************/
  /*       Index Job List       */
  /******************************/

  // Struct for data that gets logged into the job list spill buffer
  typedef struct packed {
    spatz_id_t      id;
    vreg_t          idx_vreg;
    logic           unstructured;
    vlen_t          vl;
  } idx_job_t;

  idx_job_t idx_job_in, idx_job_out;
  logic idx_job_valid, idx_job_ready;

  logic load_index_o;
  // assign load_index_o = '0;

  logic is_unstructured;
  assign is_unstructured = spatz_req_i.op_vtl.gather_vs2 | spatz_req_i.op_vtl.uscatter_vd;

  /* 
    Forming the input job data packet with
    - id          -> Operation id
    - idx_vreg    -> The index of the vector register from where the index words are to be loaded
    - gather_vs2  -> Indicates whether this is a structured/unstructured gather operation
    - vl          -> Indicates the number of elements to be processed by the vector register in this turn, among the total number of elements available
  */
  assign idx_job_in = '{
    id: spatz_req_i.id,
    idx_vreg: spatz_req_i.op_vtl.idx_vreg,
    unstructured: (is_unstructured),
    vl: spatz_req_i.vl
  };

  // new_op -> if spatz_req is valid, (and) if the operation uses VTL (and) if the operation is not to clear the buffer (and) if Ventaglio is ready, then accept the new operation
  logic new_op;
  assign new_op = spatz_req_valid_i && spatz_req_i.op_vtl.use_vtl && !spatz_req_i.op_vtl.clear_buffer && vtl_req_ready_o;
  
  // List of jobs
  spill_register
  #(
    .T(idx_job_t) 
  ) i_idx_job_list (
    .clk_i  (clk_i        ),
    .rst_ni (rst_ni       ),
    .data_i (idx_job_in   ),
    .valid_i(new_op       ),
    .ready_o(/* unused */ ),  // ready_o can be left unused because a new_op is committed only when ventaglio is ready and there is a free slot in the spill register.
                              // This handshake is already handled by the op_queue.
    .data_o (idx_job_out  ),
    .valid_o(idx_job_valid),
    .ready_i(idx_job_ready)   // the request generator drives this when it is done with a job and is ready to accept the next job from the list of jobs (Job = Operation)
  );

  /******************************/
  /*      Scatter Job List      */
  /******************************/

  vlen_t usp_sjob_vl;         // vl of the scatter job being written
  logic  usp_sjob_valid;      // a scatter job is pending / in progress
  logic  usp_sjob_done;       // the scatter datapath finished the job (last beat written)
  
  spill_register #(
    .T(vlen_t)
  ) i_usp_scatter_jobs (
    .clk_i  (clk_i                                            ),
    .rst_ni (rst_ni                                           ),
    .data_i (spatz_req_i.vl                                   ),
    .valid_i(new_op && spatz_req_i.op_vtl.uscatter_vd         ),   // a vsxu enters Ventaglio
    .ready_o(/* unused: at most 2 vsxu in flight */           ),
    .data_o (usp_sjob_vl                                      ),
    .valid_o(usp_sjob_valid                                   ),
    .ready_i(usp_sjob_done                                    )    // drop it once fully written
  );

  logic usp_scatter_running_q, usp_scatter_running_d;
  `FF(usp_scatter_running_q, usp_scatter_running_d, 1'b0)

   always_comb begin
      usp_scatter_running_d = usp_scatter_running_q;
      if (usp_sjob_done) usp_scatter_running_d = 1'b0;
      else if (spatz_req_valid && spatz_req.op_vtl.uscatter_vd && usp_sjob_valid) usp_scatter_running_d = 1'b1;
    end
      
    logic usp_scatter_running; 
    assign usp_scatter_running = usp_scatter_running_q || (spatz_req_valid && spatz_req.op_vtl.uscatter_vd && usp_sjob_valid);


  /******************************/
  /*      Request Generator     */
  /******************************/

  localparam int unsigned IdxBufDepth = 2;  // Depth of the spill register
  localparam int unsigned IdxPerWord = 32;  // Indices per index word (32 indices in a 256-bit index word)

  logic idx_req_valid;      // Indication of a valid index request to the idx_req register 
  logic idx_req_accepted;   // Request handed to the request register this cycle
  logic idx_buf_pop;        // 1 word left the index buffer this cycle - popped
  logic idx_credit_left;    // Returns 1 if there is still a free slot in the index buffer
  logic idx_req_reg_ready;  // The ready signal returned from the idx_req register for a successful request-accept handshake

  logic last_word;          // Checks whether this is the last index word for that one instruction

  localparam int unsigned NumIdxWords = VLEN * 4 / VRFWordWidth;
  logic [$clog2(NumIdxWords):0] word_cnt_d, word_cnt_q;
  logic [$clog2(NumIdxWords):0] num_words;

  assign num_words            = idx_job_out.unstructured ? (idx_job_out.vl + IdxPerWord - 1) / IdxPerWord : 1;   // round up vl/32 to CEIL
  assign last_word            = (word_cnt_q == num_words - 1);    // Check the word_cnt_q to see if this is the last index word

  // assign idx_buf_pop          = (spatz_req.op_vtl.gather_vs2 | spatz_req.op_vtl.uscatter_vd) ? load_index_o : (spatz_req_valid && spatz_req_ready && !spatz_req.op_vtl.clear_buffer);

  assign idx_buf_pop = (spatz_req.op_vtl.gather_vs2 | spatz_req.op_vtl.uscatter_vd | usp_sjob_valid) ? load_index_o : (spatz_req_valid && spatz_req_ready && !spatz_req.op_vtl.clear_buffer);

  assign idx_req_valid        = idx_job_valid && idx_credit_left;
  // assign idx_req_reg_ready    = /*ready_o of the idx_req register*/;
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

  `FF(word_cnt_q, word_cnt_d, '0);

  always_comb begin
      word_cnt_d = word_cnt_q;
      idx_job_ready = 1'b0;

      if(idx_req_accepted) begin
          // If this is the last word of the operation - reset word counter and send a job_ready request to Job List, asking for the next Job/Operation
          if(last_word) begin
              word_cnt_d = '0;
              idx_job_ready = 1'b1;
          end
          // Else, increment the word counter until it reaches num_words - 1, i.e., the last word of that operation
          else begin
              word_cnt_d = word_cnt_q + 1;
          end
      end
  end


  // Fetch trigger: monitors the pre-spill spatz_req channel, fired by a
  // successful admit handshake. Issuing the read one cycle later (through
  // i_idx_req_reg below) lets the controller scoreboard register the index
  // vreg's RAW dependency before the read lands, so a stale/zero index is
  // never returned.
  logic       eff_fetch;
  spatz_id_t  eff_id;
  vreg_t      eff_vidx;

  assign eff_fetch = spatz_req_valid_i
                    && spatz_req_i.op_vtl.use_vtl
                    && !spatz_req_i.op_vtl.clear_buffer
                    && vtl_req_ready_o;
  assign eff_id    = spatz_req_i.id;
  assign eff_vidx  = spatz_req_i.op_vtl.idx_vreg;

  // Index data feeding the scatter/gather datapaths. The spill register cuts
  // the timing path from the master VRF read response.
  vrf_data_t  index_q;
  logic       index_valid_q;

  // Pop the current index once the proceeding req has been acknowledged
  logic  idx_spill_pop;
  assign idx_spill_pop = spatz_req_ready && spatz_req_valid
                         && !spatz_req.op_vtl.clear_buffer;

  spill_register #(
    .T(vrf_data_t)
  ) i_idx_spill (
    .clk_i  (clk_i             ),
    .rst_ni (rst_ni            ),
    .data_i (vrf_rdata_i       ),
    .valid_i(vrf_rvalid_i      ),
    .ready_o(/*    UNUSED    */),
    .data_o (index_q           ),
    .valid_o(index_valid_q     ),
    .ready_i(idx_buf_pop       )
  );

  // Master VRF read-request buffer. Carries {id, addr} so vrf_re_o /
  // vrf_id_o[0] / vrf_raddr_o stay asserted and stable from issue until
  // vrf_rvalid_i pops it (ready_i = vrf_rvalid_i). ready_o is intentionally
  // unused: eff_fetch is not back-pressured — correctness relies on never
  // having more than the spill's two index requests outstanding, guaranteed
  // by the operation-queue depth and the in-order, one-fetch-per-op admit.
  typedef struct packed {
    spatz_id_t id;
    vrf_addr_t addr;
  } idx_req_t;

  idx_req_t idx_req_in, idx_req_out;
  // assign idx_req_in  = '{ id: eff_id, addr: {eff_vidx, $clog2(NrWordsPerVector)'(1'b0)}};
  assign idx_req_in = '{
    id:   idx_job_out.id,
    addr: {idx_job_out.idx_vreg, $clog2(NrWordsPerVector)'(1'b0)} + word_cnt_q
  };
  assign vrf_id_o[0] = idx_req_out.id;
  assign vrf_raddr_o = idx_req_out.addr;

  spill_register #(
    .T(idx_req_t)
  ) i_idx_req_reg (
    .clk_i  (clk_i             ),
    .rst_ni (rst_ni            ),
    .data_i (idx_req_in        ),
    .valid_i(idx_req_valid     ),
    .ready_o(idx_req_reg_ready ),
    .data_o (idx_req_out       ),
    .valid_o(vrf_re_o          ),
    .ready_i(vrf_rvalid_i      )
  );

  ////////////////////////////////
  // Counter for each operation //
  ////////////////////////////////

  // signals for gather or scatter
  logic is_gather, is_scatter, is_usp_gather, is_usp_scatter;

  // Beats per op = vl / elements_per_beat, where elements_per_beat =
  // VRFWordBWidth >> vsew (VRFWordBWidth = N_FU * ELENB). Derivation in
  // README ("Beats per operation"). Truncates for non-aligned vl, which
  // vsetvli prevents in current kernels (VLMAX is a multiple of width).
  // Consumed by the scatter datapath (i_vtl_scatter.num_beats_per_op_i).
  logic [4:0] num_beats_per_op;
  assign num_beats_per_op =
      spatz_req.vl >> ($clog2(VRFWordBWidth) - int'(spatz_req.vtype.vsew));

  /******************************/
  /*           Types            */
  /******************************/

  // The input address has type `vrf_addr_t`
  // It is used to address `NRVREG * NrWordsPerVector` words
  // Word is represented as `N_FU * ELEN`, is the bandwidth VPU or VLSU consume
  // In VTL, each channel provide `N_FU * ELEN` bandwidth as well, so it is channel addressable

  // We currently utilized a interleaved address scheme
  // Word 0             -> Channel 0 Row 0
  // Word 1             -> Channel 1 Row 0
  // Word VTGNrChannels -> Channel 0 Row 1 ...


  // `f_channel` function extract the channel index
  function automatic logic [$clog2(VTGNrBankChannels)-1:0] f_channel(vrf_addr_t addr);
    f_channel = addr[$clog2(VTGNrBankChannels)-1:0];
  endfunction: f_channel

  // `f_row` function extract the word (row) index within one channel
  function automatic logic [$clog2(VTGNrWordsPerChannel)-1:0] f_row(vrf_addr_t addr);
    f_row = addr[$clog2(VTGNrWordsPerChannel * VTGNrBankChannels)-1:$clog2(VTGNrBankChannels)];
  endfunction: f_row

  // (vtg_row_addr_t typedef defined earlier alongside the clear sequencer)

  /******************************/
  /*          Signals           */
  /******************************/

  // 1-cycle-delayed "gather admitted" pulse — registers
  // (spatz_vfu_req_ready_i && spatz_req_valid). NOT "gather finished"; the name
  // reflects a delayed admit. Feeds the gather datapath's gather_done_i.
  logic gather_admitted_q;
  `FF(gather_admitted_q, spatz_vfu_req_ready_i && spatz_req_valid, '0)

  // is_gather / is_scatter qualify a slave-port access as a bank gather/scatter
  // for the currently-latched vfx op. Both halves are required: op_vtl.*_vd is
  // the op's intent (set at decode), while rgather_en_i / wscatter_en_i is the
  // controller's per-access redirect grant (asserted when the VRF routes a
  // VTL-mapped vreg access to this bank). Neither alone is sufficient.
  always_comb begin
    is_gather  = spatz_req.op_vtl.gather_vd  && rgather_en_i;
    is_usp_gather = spatz_req.op_vtl.gather_vs2 && rgather_en_i;
    is_scatter = spatz_req.op_vtl.scatter_vd && wscatter_en_i;
	  // is_usp_scatter = spatz_req.op_vtl.uscatter_vd && wscatter_en_i;   // vsxu.vv write from the VFU
    is_usp_scatter = usp_sjob_valid && wscatter_en_i;
  end

  // write signals
  vtg_row_addr_t          [VTGNrBankChannels-1:0] waddr;
  ventaglio_narrow_data_t [VTGNrBankChannels-1:0] wdata;
  logic                   [VTGNrBankChannels-1:0] we;
  ventaglio_narrow_be_t   [VTGNrBankChannels-1:0] wbe;

  // read signals
  vtg_row_addr_t          [VTGNrBankChannels-1:0] raddr;
  ventaglio_narrow_data_t [VTGNrBankChannels-1:0] rdata;

  // write mapping
  logic [VTGNrBankChannels-1:0]                    write_request;
  logic [VTGNrBankChannels-1:0][VTGNrChannels-1:0] scatter_write_request;

  // post scatter signals
  logic      [VTGNrChannels-1:0] scatter_we;
  vrf_addr_t [VTGNrChannels-1:0] scatter_waddr;
  vrf_data_t [VTGNrChannels-1:0] scatter_wdata;
  vrf_be_t   [VTGNrChannels-1:0] scatter_wbe;

  logic      [VTGNrChannels-1:0] scatter_wvalid;

  logic                          post_scatter_wvalid;

  // Unstructured Sparsity Scatter Signals
  vtg_row_addr_t  [VTGNrBankChannels-1:0][VTGNrBanksPerChannel-1:0] usp_waddr;
  vtg_bank_data_t [VTGNrBankChannels-1:0][VTGNrBanksPerChannel-1:0] usp_wdata;
  logic           [VTGNrBankChannels-1:0][VTGNrBanksPerChannel-1:0] usp_we;     // For banks to be written to in the current cycle
  logic usp_scatter_active;
  // assign usp_scatter_active = spatz_req_valid && spatz_req.op_vtl.uscatter_vd;
  assign usp_scatter_active = usp_sjob_valid;

  logic usp_scatter_wready;
  logic usp_scatter_load_index;


  always_comb begin: gen_write_request
    for (int channel = 0; channel < VTGNrBankChannels; channel++) begin
      write_request[channel] = we_i && f_channel(waddr_i) == channel && !is_scatter && !is_usp_scatter;
    end
    // scatter requests
    for (int b_channel = 0; b_channel < VTGNrBankChannels; b_channel++) begin   // channels from the bank side
      for (int s_channel = 0; s_channel < VTGNrChannels; s_channel++) begin // channels from the scatter datapath side
        scatter_write_request[b_channel][s_channel] = scatter_we[s_channel] && f_channel(scatter_waddr[s_channel]) == b_channel && is_scatter;
      end
    end
  end: gen_write_request

  always_comb begin : proc_write
    waddr          = '0;
    wdata          = '0;
    we             = '0;
    wbe            = '0;
    wvalid_o       = 1'b0;
    scatter_wvalid = '0;

    if (clear_active_q) begin // priority 0: vventclr — zero every cell
      for (int unsigned channel = 0; channel < VTGNrBankChannels; channel++) begin
        waddr[channel] = clear_row_q;
        wdata[channel] = '0;
        we[channel]    = 1'b1;
        wbe[channel]   = '1;
      end
      // wvalid_o stays 0: no slave-port write is being acknowledged.
    end else if (is_usp_scatter) begin
      wvalid_o = usp_scatter_wready;
    end else if (!is_scatter) begin // priority 1: normal requests
      for (int unsigned channel = 0; channel < VTGNrBankChannels; channel++) begin
        if (write_request[channel]) begin
          waddr[channel] = f_row(waddr_i);
          wdata[channel] = wdata_i;
          we[channel]    = 1'b1;
          wbe[channel]   = wbe_i;
          wvalid_o       = 1'b1;
        end
      end
    end else begin // priority 2: scatter requests
      for (int unsigned b_channel = 0; b_channel < VTGNrBankChannels; b_channel++) begin
        for (int unsigned s_channel = 0; s_channel < VTGNrChannels; s_channel++) begin
          if (scatter_write_request[b_channel][s_channel]) begin
            waddr[b_channel]           = f_row(scatter_waddr[s_channel]);
            wdata[b_channel]           = scatter_wdata[s_channel];
            we[b_channel]              = 1'b1;
            wbe[b_channel]             = scatter_wbe[s_channel];
            scatter_wvalid[s_channel]  = 1'b1;
          end
        end
      end
      wvalid_o = post_scatter_wvalid;
    end

  end : proc_write

  // read mapping

  // read_request: non-gather request signals
  // gathered_read_request: gather request signals
  logic [VTGNrBankChannels-1:0]                    read_request;

  logic      [VTGNrBankChannels-1:0][VTGNrChannels-1:0] gather_read_request;
  logic      [VTGNrChannels-1:0]                    gather_re;
  vrf_addr_t [VTGNrChannels-1:0]                    gather_raddr;

  vrf_data_t [VTGNrChannels-1:0]                    gather_rdata;
  logic      [VTGNrChannels-1:0]                    gather_rvalid;

  // post gather signals
  vrf_data_t post_gather_rdata;
  logic      post_gather_rvalid;

  // Unstructured sparsity gather signals
  vrf_data_t usp_gather_data;
  logic usp_gather_valid;

  always_comb begin: gen_read_request
    // normal requests
    for (int channel = 0; channel < VTGNrBankChannels; channel++) begin
      read_request[channel] = re_i && f_channel(raddr_i) == channel && !(is_gather) && !(is_usp_gather);
    end
    // gathered requests
    for (int b_channel = 0; b_channel < VTGNrBankChannels; b_channel++) begin   // channels from the bank side
      for (int g_channel = 0; g_channel < VTGNrChannels; g_channel++) begin // channels from the gather datapath side
        gather_read_request[b_channel][g_channel] = gather_re[g_channel] && f_channel(gather_raddr[g_channel]) == b_channel && is_gather;
      end
    end
  end: gen_read_request

  // this should be extended for gather
  always_comb begin : proc_read
    raddr    = '0;
    rvalid_o = 1'b0;
    rdata_o  = 'x;

    gather_rvalid = '0;
    gather_rdata  = 'x;
  
    if(is_usp_gather) begin
      // Here, the data from the right banks have already been picked and consolidated together.
      // The data and valid signals can be dirctly dispatched as outputs
      rdata_o  = usp_gather_data;
      rvalid_o = usp_gather_valid;
    end else if (!is_gather) begin // priority 1: normal read requests
      for (int unsigned b_channel = 0; b_channel < VTGNrBankChannels; b_channel++) begin // channels from the bank side
        if (read_request[b_channel]) begin
          raddr[b_channel] = f_row(raddr_i);
          rdata_o          = rdata[b_channel];
          rvalid_o         = 1'b1;
        end
      end
    end else if (is_gather) begin // priority 2: gather accesses
      for (int unsigned b_channel = 0; b_channel < VTGNrBankChannels; b_channel++) begin // channels from the bank side
        for (int unsigned g_channel = 0; g_channel < VTGNrChannels; g_channel++) begin // channels from the gather datapath side
          if (gather_read_request[b_channel][g_channel]) begin
            raddr[b_channel]          = f_row(gather_raddr[g_channel]);
            gather_rdata[g_channel]   = rdata[b_channel];
            gather_rvalid[g_channel]  = index_valid_q;
          end
        end
      end
      // assign the gathered data to the output
      rdata_o  = post_gather_rdata;
      rvalid_o = post_gather_rvalid;
    end
  end

  logic usp_gather_load_index;
  assign load_index_o = usp_gather_load_index | usp_scatter_load_index;

  /******************************/
  /*      Scatter DataPath      */
  /******************************/
  ventaglio_scatter #(
    .NarrowDataWidth (NarrowDataWidth),
    .WideDataWidth   (WideDataWidth)
  ) i_vtl_scatter (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .testmode_i  (testmode_i),
    // narrow ports
    .waddr_i     (waddr_i),
    .wdata_i     (wdata_i),
    .we_i        (we_i && is_scatter),
    .wbe_i       (wbe_i),
    .wvalid_o    (post_scatter_wvalid),
    // wide ports
    .waddr_o     (scatter_waddr),
    .wdata_o     (scatter_wdata),
    .we_o        (scatter_we),
    .wbe_o       (scatter_wbe),
    .wvalid_i    (scatter_wvalid),
    // control
    // TODO: it is better to implement as a counter to track
    .scatter_done_i      (!is_scatter),
    .num_beats_per_op_i  (num_beats_per_op),
    // controls
    .index_i       (index_q                ),
    .vtl_cfg_i     (spatz_req.op_vtl.sp_cfg),
    .index_valid_i (index_valid_q) // meaning a new read data available
  );

  /***********************************/
  /*  Unstructured Scatter DataPath  */
  /***********************************/

  unstructured_scatter_datapath i_usp_scatter 
  (
    .clk_i         (clk_i                    ),
    .rst_ni        (rst_ni                   ),
    .enable_i      (usp_sjob_valid        ),
    .index_i       (index_q                  ),
    .index_valid_i (index_valid_q            ),
    .vl_i          (usp_sjob_vl              ),
    .wdata_i       (wdata_i                  ),
    .wvalid_i      (we_i && is_usp_scatter   ),   // the VFU is delivering a vsxu beat
    .wready_o      (usp_scatter_wready       ),
    .load_index_o  (usp_scatter_load_index   ),
    .bank_row_o    (usp_waddr                ),
    .bank_wdata_o  (usp_wdata                ),
    .bank_we_o     (usp_we                   ),
    .job_done_o    (usp_sjob_done            )
  );



  /******************************/
  /*           Buffer           */
  /******************************/

  // Buffer has `VTGNrBankChannels` channels
  // Each channel is divided into `N_FU*2` (8) banks, whose width is `ELEN/2` (32-bits)
  // In this way, each channel can provide the same bandwidth as the VLSU and VPU

  // Unstructured Sparsity Reading Logic

  vtg_row_addr_t [VTGNrBankChannels-1:0][VTGNrBanksPerChannel-1:0] usp_raddr;
  logic usp_active; // Are we doing an unstructured gather?

  assign usp_active = spatz_req_valid && spatz_req.op_vtl.gather_vs2;

  
  
	for (genvar channel = 0; channel < VTGNrBankChannels; channel++) begin : gen_vtg_channels
  	for (genvar bank = 0; bank < VTGNrBanksPerChannel; bank++) begin: gen_vtg_banks
      
      vtg_row_addr_t vtg_raddr;   // one per bank
      assign vtg_raddr = usp_active ? usp_raddr[channel][bank] : raddr[channel];

      vtg_row_addr_t vtg_waddr;
      assign vtg_waddr = usp_scatter_active ? usp_waddr[channel][bank] : waddr[channel];

      vtg_bank_data_t vtg_wdata;
      assign vtg_wdata = usp_scatter_active ? usp_wdata[channel][bank] : wdata[channel][VTGBankWidth*bank +: VTGBankWidth];

      logic vtg_we;
      logic [VTGBankBWidth-1:0] vtg_wbe;
      assign vtg_we = usp_scatter_active ? usp_we[channel][bank] : we[channel];
      assign vtg_wbe = usp_scatter_active ? {VTGBankBWidth{usp_we[channel][bank]}} : wbe[channel][VTGBankBWidth*bank +: VTGBankBWidth];
      
    	ventaglio_regfile #(
      	.NrWords    (VTGNrWordsPerChannel ),
          .WordWidth  (VTGBankWidth         )
        ) i_vtg_vregfile (
          .clk_i     (clk_i                                              ),
          .rst_ni    (rst_ni                                             ),
          .testmode_i(testmode_i                                         ),
          .waddr_i   (vtg_waddr                                          ),
          .wdata_i   (vtg_wdata                                          ),
          .we_i      (vtg_we                                             ),
          .wbe_i     (vtg_wbe                                            ),
          .raddr_i   (vtg_raddr                                          ),
          .rdata_o   (rdata[channel][VTGBankWidth*bank  +: VTGBankWidth ])
        );
		end
	end

  /************************************/
  /*   Unstructured Gather DataPath   */
  /************************************/


  unstructured_gather_datapath i_usp_gather (
    .clk_i(clk_i),
    .rst_ni(rst_ni),
    .enable_i(usp_active && !usp_scatter_running),
    .index_i(index_q),
    .index_valid_i(index_valid_q),
    .vl_i(spatz_req.vl),
    .gather_ready_i (re_i && is_usp_gather),
    .bank_rdata_i(rdata),
     .raddr_i (raddr_i                            ),
      .op_pop_i(spatz_req_valid && spatz_req_ready ),
    .load_index_o(usp_gather_load_index),
    .bank_row_o(usp_raddr),
    .gather_data_o(usp_gather_data),
    .gather_valid_o(usp_gather_valid)
  );

  /******************************/
  /*      Gather  DataPath      */
  /******************************/

  ventaglio_gather #(
    .NarrowDataWidth (NarrowDataWidth),
    .WideDataWidth   (WideDataWidth)
  ) i_vtl_gather (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .testmode_i  (testmode_i),
    // narrow ports
    .raddr_i     (raddr_i              ),
    .re_i        (re_i && is_gather    ),
    .rdata_o     (post_gather_rdata),
    .rvalid_o    (post_gather_rvalid),
    // wide ports
    .raddr_o     (gather_raddr         ),
    .re_o        (gather_re            ),
    .rdata_i     (gather_rdata         ),
    .rvalid_i    (gather_rvalid        ),
    // control
    .gather_done_i(gather_admitted_q    ),
    // index cfg
    .index_i     (index_q              ),
    .vtl_cfg_i   (spatz_req.op_vtl.sp_cfg),
    .load_index_o(/* Unconnected */)
  );

  /******************************/
  /*      Write Requests        */
  /******************************/

  // RESERVED: master VRF write port is held idle. vrf_id_o[1] is the write id
  // (paired with the write port); tied off so it is never X.
  assign vrf_we_o    = '0;
  assign vrf_wbe_o   = '0;
  assign vrf_waddr_o = '0;
  assign vrf_wdata_o = '0;
  assign vrf_id_o[1] = '0;

endmodule : ventaglio
