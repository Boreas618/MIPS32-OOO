/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off CASEINCOMPLETE */

/**
 * Memory Disambiguation Unit - Milestone 5.5
 *
 * Coordinates memory operations between LoadQueue and StoreQueue.
 * Responsibilities:
 * - Store-to-Load forwarding coordination
 * - Memory order violation detection
 * - Cache interface arbitration
 *
 * Store-to-Load Forwarding:
 * - Load searches store queue for matching address
 * - If found and data valid: forward directly
 * - If found but data not valid: wait
 * - If not found: access cache
 *
 * Memory Order Speculation:
 * - Loads can execute before older stores resolve address
 * - Track which loads executed speculatively
 * - On address match with older store: memory order violation
 * - Recovery: flush from violating load onward
 */
module MemoryDisambiguation #(
    parameter LQ_ENTRIES = 32,
    parameter SQ_ENTRIES = 32,
    parameter NUM_PHYS_REGS = 64,
    parameter DISPATCH_WIDTH = 4,
    parameter COMMIT_WIDTH = 4
) (
    input  logic clk,
    input  logic rst,

    // ========== Dispatch Interface ==========
    input  logic [DISPATCH_WIDTH-1:0] dispatch_valid,
    input  logic [DISPATCH_WIDTH-1:0] dispatch_is_load,
    input  logic [DISPATCH_WIDTH-1:0] dispatch_is_store,
    input  logic [5:0] dispatch_prd [DISPATCH_WIDTH-1:0],
    input  logic [6:0] dispatch_rob_idx [DISPATCH_WIDTH-1:0],
    input  logic [31:0] dispatch_pc [DISPATCH_WIDTH-1:0],         // PC for memory violation recovery
    
    output logic [DISPATCH_WIDTH-1:0] dispatch_lsq_valid,
    output logic lsq_full,

    // ========== Issue Interface (from IssueUnit) ==========
    // Memory operation issued for execution
    input  logic mem_issue_valid,
    input  logic mem_issue_is_load,
    input  logic mem_issue_is_store,
    input  logic [5:0] mem_issue_prd,
    input  logic [31:0] mem_issue_addr,
    input  logic [31:0] mem_issue_data,
    input  logic [6:0] mem_issue_rob_idx,

    // ========== Cache Interface ==========
    output logic cache_req_valid,
    output logic cache_req_write,
    output logic [31:0] cache_req_addr,
    output logic [31:0] cache_req_data,
    
    input  logic [1:0] cache_status,        // 00=ready, 01=processing, 10=complete
    input  logic [31:0] cache_read_data,

    // ========== CDB Interface (broadcast load results) ==========
    output logic mem_result_valid,
    output logic [5:0] mem_result_prd,
    output logic [31:0] mem_result_data,
    output logic [6:0] mem_result_rob_idx,
    input  logic mem_result_ack,

    // ========== Commit Interface ==========
    input  logic [COMMIT_WIDTH-1:0] commit_valid,
    input  logic [COMMIT_WIDTH-1:0] commit_is_store,
    input  logic [6:0] commit_rob_idx [COMMIT_WIDTH-1:0],

    // ========== ROB Interface ==========
    output logic rob_mem_complete_valid,
    output logic [6:0] rob_mem_complete_rob_idx,
    output logic rob_store_addr_valid,
    output logic [6:0] rob_store_addr_rob_idx,

    // ========== Recovery Interface ==========
    output logic memory_violation,
    output logic [6:0] violation_rob_idx,
    output logic [31:0] violation_pc,                             // PC of violating load for re-execution
    
    input  logic flush,
    input  logic [6:0] flush_rob_idx,

    // ========== ROB Head Index (for proper age comparison) ==========
    input  logic [6:0] rob_head_idx,                              // ROB head index for wraparound handling

    // ========== Status ==========
    output logic [$clog2(LQ_ENTRIES):0] lq_count,
    output logic [$clog2(SQ_ENTRIES):0] sq_count
);

    // Internal LSQ tracking
    localparam LQ_IDX_BITS = $clog2(LQ_ENTRIES);
    localparam SQ_IDX_BITS = $clog2(SQ_ENTRIES);

    // Load queue entry
    typedef struct packed {
        logic valid;
        logic [31:0] addr;
        logic addr_valid;
        logic [31:0] data;
        logic data_valid;
        logic [5:0] prd;
        logic [6:0] rob_idx;
        logic [31:0] pc;        // PC for memory violation recovery
        logic executed;
        logic broadcast;
    } lq_entry_t;

    // Store queue entry
    typedef struct packed {
        logic valid;
        logic [31:0] addr;
        logic addr_valid;
        logic [31:0] data;
        logic data_valid;
        logic [6:0] rob_idx;
        logic committed;
        logic written;
    } sq_entry_t;

    // Storage
    lq_entry_t lq_entries [LQ_ENTRIES-1:0];
    sq_entry_t sq_entries [SQ_ENTRIES-1:0];

    // Pointers
    logic [LQ_IDX_BITS:0] lq_head, lq_tail;
    logic [SQ_IDX_BITS:0] sq_head, sq_tail;

    wire [LQ_IDX_BITS-1:0] lq_head_idx = lq_head[LQ_IDX_BITS-1:0];
    wire [LQ_IDX_BITS-1:0] lq_tail_idx = lq_tail[LQ_IDX_BITS-1:0];
    wire [SQ_IDX_BITS-1:0] sq_head_idx = sq_head[SQ_IDX_BITS-1:0];
    wire [SQ_IDX_BITS-1:0] sq_tail_idx = sq_tail[SQ_IDX_BITS-1:0];

    // Counts
    logic [$clog2(LQ_ENTRIES):0] lq_cnt;
    logic [$clog2(SQ_ENTRIES):0] sq_cnt;

    assign lq_count = lq_cnt;
    assign sq_count = sq_cnt;

    // Full detection - use count-based detection to avoid head pointer wraparound issues
    // The pointer-based detection fails when head pointers don't advance properly
    wire lq_full_internal = (lq_cnt >= LQ_ENTRIES);
    wire sq_full_internal = (sq_cnt >= SQ_ENTRIES);
    
    assign lsq_full = lq_full_internal || sq_full_internal;

    // Deallocation counters
    logic [2:0] lq_dealloc_cnt, sq_dealloc_cnt;
    
    // ==========================================================================
    // Dispatch Logic
    // ==========================================================================
    
    logic [2:0] lq_dispatch_cnt, sq_dispatch_cnt;
    logic [DISPATCH_WIDTH-1:0] lq_can_dispatch, sq_can_dispatch;
    logic [LQ_IDX_BITS:0] next_lq_tail [DISPATCH_WIDTH:0];
    logic [SQ_IDX_BITS:0] next_sq_tail [DISPATCH_WIDTH:0];
    
    always_comb begin
        next_lq_tail[0] = lq_tail;
        next_sq_tail[0] = sq_tail;
        lq_dispatch_cnt = 0;
        sq_dispatch_cnt = 0;
        
        for (int i = 0; i < DISPATCH_WIDTH; i++) begin
            // Load allocation
            if (dispatch_valid[i] && dispatch_is_load[i]) begin
                logic [LQ_IDX_BITS:0] tent;
                tent = next_lq_tail[i] + 1;
                lq_can_dispatch[i] = !((tent[LQ_IDX_BITS-1:0] == lq_head[LQ_IDX_BITS-1:0]) &&
                                       (tent[LQ_IDX_BITS] != lq_head[LQ_IDX_BITS]));
                if (lq_can_dispatch[i]) begin
                    next_lq_tail[i+1] = next_lq_tail[i] + 1;
                    lq_dispatch_cnt = lq_dispatch_cnt + 1;
                end else begin
                    next_lq_tail[i+1] = next_lq_tail[i];
                end
            end else begin
                lq_can_dispatch[i] = 1'b0;
                next_lq_tail[i+1] = next_lq_tail[i];
            end
            
            // Store allocation
            if (dispatch_valid[i] && dispatch_is_store[i]) begin
                logic [SQ_IDX_BITS:0] tent;
                tent = next_sq_tail[i] + 1;
                sq_can_dispatch[i] = !((tent[SQ_IDX_BITS-1:0] == sq_head[SQ_IDX_BITS-1:0]) &&
                                       (tent[SQ_IDX_BITS] != sq_head[SQ_IDX_BITS]));
                if (sq_can_dispatch[i]) begin
                    next_sq_tail[i+1] = next_sq_tail[i] + 1;
                    sq_dispatch_cnt = sq_dispatch_cnt + 1;
                end else begin
                    next_sq_tail[i+1] = next_sq_tail[i];
                end
            end else begin
                sq_can_dispatch[i] = 1'b0;
                next_sq_tail[i+1] = next_sq_tail[i];
            end
            
            dispatch_lsq_valid[i] = (dispatch_is_load[i] && lq_can_dispatch[i]) ||
                                    (dispatch_is_store[i] && sq_can_dispatch[i]) ||
                                    (!dispatch_is_load[i] && !dispatch_is_store[i]);
        end
    end

    // ==========================================================================
    // Deallocation Count Logic
    // ==========================================================================
    
    always_comb begin
        lq_dealloc_cnt = '0;
        sq_dealloc_cnt = '0;
        
        // Count written stores that will be deallocated
        for (int i = 0; i < SQ_ENTRIES; i++) begin
            if (sq_entries[i].valid && sq_entries[i].written) begin
                sq_dealloc_cnt = sq_dealloc_cnt + 1;
            end
        end
        
        // Count loads that will be deallocated (committed and broadcast)
        for (int i = 0; i < COMMIT_WIDTH; i++) begin
            if (commit_valid[i]) begin
                for (int j = 0; j < LQ_ENTRIES; j++) begin
                    if (lq_entries[j].valid && lq_entries[j].rob_idx == commit_rob_idx[i] &&
                        lq_entries[j].broadcast) begin
                        lq_dealloc_cnt = lq_dealloc_cnt + 1;
                    end
                end
            end
        end
    end

    // ==========================================================================
    // Address Fill and Store-to-Load Forwarding
    // ==========================================================================
    
    // Find entry matching issued memory operation
    logic [LQ_IDX_BITS-1:0] issue_lq_idx;
    logic [SQ_IDX_BITS-1:0] issue_sq_idx;
    logic issue_lq_found, issue_sq_found;
    
    always_comb begin
        issue_lq_found = 1'b0;
        issue_lq_idx = '0;
        issue_sq_found = 1'b0;
        issue_sq_idx = '0;
        
        if (mem_issue_valid) begin
            if (mem_issue_is_load) begin
                for (int i = 0; i < LQ_ENTRIES; i++) begin
                    if (lq_entries[i].valid && !lq_entries[i].addr_valid &&
                        lq_entries[i].rob_idx == mem_issue_rob_idx) begin
                        issue_lq_found = 1'b1;
                        issue_lq_idx = i[LQ_IDX_BITS-1:0];
                    end
                end
            end
            if (mem_issue_is_store) begin
                for (int i = 0; i < SQ_ENTRIES; i++) begin
                    if (sq_entries[i].valid && !sq_entries[i].addr_valid &&
                        sq_entries[i].rob_idx == mem_issue_rob_idx) begin
                        issue_sq_found = 1'b1;
                        issue_sq_idx = i[SQ_IDX_BITS-1:0];
                    end
                end
            end
        end
    end

    // Store-to-load forwarding check
    logic stl_fwd_hit;
    logic stl_fwd_data_valid;
    logic [31:0] stl_fwd_data;
    logic [6:0] stl_fwd_youngest_rob_idx;  // Track youngest matching store's ROB index
    
    always_comb begin
        stl_fwd_hit = 1'b0;
        stl_fwd_data_valid = 1'b0;
        stl_fwd_data = '0;
        stl_fwd_youngest_rob_idx = '0;
        
        if (mem_issue_valid && mem_issue_is_load) begin
            // Search store queue for matching older store using relative age
            for (int i = 0; i < SQ_ENTRIES; i++) begin
                if (sq_entries[i].valid && sq_entries[i].addr_valid) begin
                    logic is_older;
                    logic [6:0] store_age_rel, load_age_rel;
                    
                    // Calculate age relative to ROB head (handles wraparound)
                    store_age_rel = sq_entries[i].rob_idx - rob_head_idx;
                    load_age_rel = mem_issue_rob_idx - rob_head_idx;
                    
                    // Older if store's relative age is less than load's
                    is_older = (store_age_rel < load_age_rel);
                    
                    if (is_older && (sq_entries[i].addr[31:2] == mem_issue_addr[31:2])) begin
                        // Take the youngest matching older store (largest relative age among older stores)
                        if (!stl_fwd_hit || store_age_rel > (stl_fwd_youngest_rob_idx - rob_head_idx)) begin
                            stl_fwd_hit = 1'b1;
                            stl_fwd_data_valid = sq_entries[i].data_valid;
                            stl_fwd_data = sq_entries[i].data;
                            stl_fwd_youngest_rob_idx = sq_entries[i].rob_idx;
                        end
                    end
                end
            end
        end
    end

    // ==========================================================================
    // Memory Order Violation Detection
    // ==========================================================================
    
    always_comb begin
        memory_violation = 1'b0;
        violation_rob_idx = '0;
        violation_pc = '0;
        rob_store_addr_valid = 1'b0;
        rob_store_addr_rob_idx = '0;
        
        // When a store address is computed, check for violations
        if (mem_issue_valid && mem_issue_is_store && issue_sq_found) begin
            rob_store_addr_valid = 1'b1;
            rob_store_addr_rob_idx = mem_issue_rob_idx;
            
            // Check if any younger load already executed with same address
            for (int i = 0; i < LQ_ENTRIES; i++) begin
                if (lq_entries[i].valid && lq_entries[i].executed) begin
                    logic is_younger;
                    logic [6:0] load_age_rel, store_age_rel;
                    
                    // Calculate age relative to ROB head (handles wraparound)
                    load_age_rel = lq_entries[i].rob_idx - rob_head_idx;
                    store_age_rel = mem_issue_rob_idx - rob_head_idx;
                    
                    // Younger if load's relative age is greater than store's
                    is_younger = (load_age_rel > store_age_rel);
                    
                    if (is_younger && (lq_entries[i].addr[31:2] == mem_issue_addr[31:2])) begin
                        // Report the oldest violating load (smallest relative age among violators)
                        if (!memory_violation || load_age_rel < (violation_rob_idx - rob_head_idx)) begin
                            memory_violation = 1'b1;
                            violation_rob_idx = lq_entries[i].rob_idx;
                            violation_pc = lq_entries[i].pc;  // PC for re-execution
                        end
                    end
                end
            end
        end
    end

    // ==========================================================================
    // Cache Interface
    // ==========================================================================
    
    // State machine for cache access
    typedef enum logic [1:0] {
        CACHE_IDLE,
        CACHE_LOAD,
        CACHE_STORE
    } cache_state_t;
    
    cache_state_t cache_state;
    logic [6:0] pending_rob_idx;
    logic [5:0] pending_prd;
    logic [31:0] pending_addr;
    logic [31:0] pending_data;
    
    // Find load ready for cache access
    logic [LQ_IDX_BITS-1:0] cache_load_idx;
    logic cache_load_found;
    
    always_comb begin
        cache_load_found = 1'b0;
        cache_load_idx = '0;
        
        for (int i = 0; i < LQ_ENTRIES; i++) begin
            logic [LQ_IDX_BITS-1:0] idx;
            // Use bit truncation instead of modulo
            idx = lq_head_idx + i[LQ_IDX_BITS-1:0];
            
            if (!cache_load_found && lq_entries[idx].valid && lq_entries[idx].addr_valid &&
                !lq_entries[idx].data_valid && !lq_entries[idx].executed) begin
                cache_load_found = 1'b1;
                cache_load_idx = idx;
            end
        end
    end
    
    // Find committed store ready for cache write
    logic [SQ_IDX_BITS-1:0] cache_store_idx;
    logic cache_store_found;
    
    always_comb begin
        cache_store_found = 1'b0;
        cache_store_idx = '0;
        
        for (int i = 0; i < SQ_ENTRIES; i++) begin
            logic [SQ_IDX_BITS-1:0] idx;
            // Use bit truncation instead of modulo
            idx = sq_head_idx + i[SQ_IDX_BITS-1:0];
            
            if (!cache_store_found && sq_entries[idx].valid && sq_entries[idx].committed &&
                sq_entries[idx].addr_valid && sq_entries[idx].data_valid &&
                !sq_entries[idx].written) begin
                cache_store_found = 1'b1;
                cache_store_idx = idx;
            end
        end
    end

    // Cache request generation
    always_comb begin
        cache_req_valid = 1'b0;
        cache_req_write = 1'b0;
        cache_req_addr = '0;
        cache_req_data = '0;
        
        case (cache_state)
            CACHE_IDLE: begin
                // Priority: stores (to commit) > loads
                if (cache_store_found) begin
                    cache_req_valid = 1'b1;
                    cache_req_write = 1'b1;
                    cache_req_addr = sq_entries[cache_store_idx].addr;
                    cache_req_data = sq_entries[cache_store_idx].data;
                end else if (cache_load_found) begin
                    cache_req_valid = 1'b1;
                    cache_req_write = 1'b0;
                    cache_req_addr = lq_entries[cache_load_idx].addr;
                    cache_req_data = '0;
                end
            end
            CACHE_LOAD, CACHE_STORE: begin
                // Keep request active until complete
                cache_req_valid = 1'b1;
                cache_req_write = (cache_state == CACHE_STORE);
                cache_req_addr = pending_addr;
                cache_req_data = pending_data;
            end
        endcase
    end

    // ==========================================================================
    // Result Broadcast
    // ==========================================================================
    
    // Find load ready to broadcast
    logic [LQ_IDX_BITS-1:0] broadcast_idx;
    logic broadcast_found;
    
    always_comb begin
        broadcast_found = 1'b0;
        broadcast_idx = '0;
        mem_result_valid = 1'b0;
        mem_result_prd = '0;
        mem_result_data = '0;
        mem_result_rob_idx = '0;
        rob_mem_complete_valid = 1'b0;
        rob_mem_complete_rob_idx = '0;
        
        for (int i = 0; i < LQ_ENTRIES; i++) begin
            logic [LQ_IDX_BITS-1:0] idx;
            // Use bit truncation instead of modulo
            idx = lq_head_idx + i[LQ_IDX_BITS-1:0];
            
            if (!broadcast_found && lq_entries[idx].valid && lq_entries[idx].data_valid &&
                !lq_entries[idx].broadcast) begin
                broadcast_found = 1'b1;
                broadcast_idx = idx;
                mem_result_valid = 1'b1;
                mem_result_prd = lq_entries[idx].prd;
                mem_result_data = lq_entries[idx].data;
                mem_result_rob_idx = lq_entries[idx].rob_idx;
                rob_mem_complete_valid = 1'b1;
                rob_mem_complete_rob_idx = lq_entries[idx].rob_idx;
            end
        end
    end

    // ==========================================================================
    // State Machine
    // ==========================================================================
    
    always_ff @(posedge clk) begin
        if (rst) begin
            cache_state <= CACHE_IDLE;
            pending_rob_idx <= '0;
            pending_prd <= '0;
            pending_addr <= '0;
            pending_data <= '0;
            lq_head <= '0;
            lq_tail <= '0;
            sq_head <= '0;
            sq_tail <= '0;
            lq_cnt <= '0;
            sq_cnt <= '0;
            
            for (int i = 0; i < LQ_ENTRIES; i++) begin
                lq_entries[i].valid <= 1'b0;
            end
            for (int i = 0; i < SQ_ENTRIES; i++) begin
                sq_entries[i].valid <= 1'b0;
            end
        end else if (flush) begin
            // Flush younger entries using relative age (handles ROB wraparound)
            for (int i = 0; i < LQ_ENTRIES; i++) begin
                if (lq_entries[i].valid) begin
                    logic [6:0] entry_age_rel, flush_age_rel;
                    entry_age_rel = lq_entries[i].rob_idx - rob_head_idx;
                    flush_age_rel = flush_rob_idx - rob_head_idx;
                    
                    if (entry_age_rel >= flush_age_rel) begin
                        lq_entries[i].valid <= 1'b0;
                    end
                end
            end
            for (int i = 0; i < SQ_ENTRIES; i++) begin
                if (sq_entries[i].valid && !sq_entries[i].committed) begin
                    logic [6:0] entry_age_rel, flush_age_rel;
                    entry_age_rel = sq_entries[i].rob_idx - rob_head_idx;
                    flush_age_rel = flush_rob_idx - rob_head_idx;
                    
                    if (entry_age_rel >= flush_age_rel) begin
                        sq_entries[i].valid <= 1'b0;
                    end
                end
            end
            
            cache_state <= CACHE_IDLE;
        end else begin
            // Dispatch loads - use properly calculated indices
            for (int i = 0; i < DISPATCH_WIDTH; i++) begin
                if (dispatch_valid[i] && dispatch_is_load[i] && lq_can_dispatch[i]) begin
                    logic [LQ_IDX_BITS-1:0] idx;
                    // Use the properly calculated next_lq_tail value
                    idx = next_lq_tail[i][LQ_IDX_BITS-1:0];
                    
                    lq_entries[idx].valid <= 1'b1;
                    lq_entries[idx].addr <= '0;
                    lq_entries[idx].addr_valid <= 1'b0;
                    lq_entries[idx].data <= '0;
                    lq_entries[idx].data_valid <= 1'b0;
                    lq_entries[idx].prd <= dispatch_prd[i];
                    lq_entries[idx].rob_idx <= dispatch_rob_idx[i];
                    lq_entries[idx].pc <= dispatch_pc[i];  // Store PC for violation recovery
                    lq_entries[idx].executed <= 1'b0;
                    lq_entries[idx].broadcast <= 1'b0;
                end
            end
            
            // Dispatch stores - use properly calculated indices
            for (int i = 0; i < DISPATCH_WIDTH; i++) begin
                if (dispatch_valid[i] && dispatch_is_store[i] && sq_can_dispatch[i]) begin
                    logic [SQ_IDX_BITS-1:0] idx;
                    // Use the properly calculated next_sq_tail value
                    idx = next_sq_tail[i][SQ_IDX_BITS-1:0];
                    
                    sq_entries[idx].valid <= 1'b1;
                    sq_entries[idx].addr <= '0;
                    sq_entries[idx].addr_valid <= 1'b0;
                    sq_entries[idx].data <= '0;
                    sq_entries[idx].data_valid <= 1'b0;
                    sq_entries[idx].rob_idx <= dispatch_rob_idx[i];
                    sq_entries[idx].committed <= 1'b0;
                    sq_entries[idx].written <= 1'b0;
                end
            end
            
            // Issue: fill address
            if (mem_issue_valid && mem_issue_is_load && issue_lq_found) begin
                lq_entries[issue_lq_idx].addr <= mem_issue_addr;
                lq_entries[issue_lq_idx].addr_valid <= 1'b1;
                
                // Check for immediate forwarding
                if (stl_fwd_hit && stl_fwd_data_valid) begin
                    lq_entries[issue_lq_idx].data <= stl_fwd_data;
                    lq_entries[issue_lq_idx].data_valid <= 1'b1;
                    lq_entries[issue_lq_idx].executed <= 1'b1;
                end
            end
            
            if (mem_issue_valid && mem_issue_is_store && issue_sq_found) begin
                sq_entries[issue_sq_idx].addr <= mem_issue_addr;
                sq_entries[issue_sq_idx].addr_valid <= 1'b1;
                sq_entries[issue_sq_idx].data <= mem_issue_data;
                sq_entries[issue_sq_idx].data_valid <= 1'b1;
            end
            
            // Cache state machine
            case (cache_state)
                CACHE_IDLE: begin
                    if (cache_store_found) begin
                        cache_state <= CACHE_STORE;
                        pending_rob_idx <= sq_entries[cache_store_idx].rob_idx;
                        pending_addr <= sq_entries[cache_store_idx].addr;
                        pending_data <= sq_entries[cache_store_idx].data;
                    end else if (cache_load_found) begin
                        cache_state <= CACHE_LOAD;
                        pending_rob_idx <= lq_entries[cache_load_idx].rob_idx;
                        pending_prd <= lq_entries[cache_load_idx].prd;
                        pending_addr <= lq_entries[cache_load_idx].addr;
                        lq_entries[cache_load_idx].executed <= 1'b1;
                    end
                end
                
                CACHE_LOAD: begin
                    if (cache_status == 2'b10) begin  // Complete
                        // Fill load data
                        for (int i = 0; i < LQ_ENTRIES; i++) begin
                            if (lq_entries[i].valid && lq_entries[i].rob_idx == pending_rob_idx) begin
                                lq_entries[i].data <= cache_read_data;
                                lq_entries[i].data_valid <= 1'b1;
                            end
                        end
                        cache_state <= CACHE_IDLE;
                    end
                end
                
                CACHE_STORE: begin
                    if (cache_status == 2'b10) begin  // Complete
                        // Mark store as written
                        for (int i = 0; i < SQ_ENTRIES; i++) begin
                            if (sq_entries[i].valid && sq_entries[i].rob_idx == pending_rob_idx) begin
                                sq_entries[i].written <= 1'b1;
                            end
                        end
                        cache_state <= CACHE_IDLE;
                    end
                end
            endcase
            
            // Commit stores
            for (int i = 0; i < COMMIT_WIDTH; i++) begin
                if (commit_valid[i] && commit_is_store[i]) begin
                    for (int j = 0; j < SQ_ENTRIES; j++) begin
                        if (sq_entries[j].valid && sq_entries[j].rob_idx == commit_rob_idx[i]) begin
                            sq_entries[j].committed <= 1'b1;
                        end
                    end
                end
            end
            
            // Broadcast acknowledgment
            if (mem_result_valid && mem_result_ack && broadcast_found) begin
                lq_entries[broadcast_idx].broadcast <= 1'b1;
            end
            
            // Commit loads: deallocate
            for (int i = 0; i < COMMIT_WIDTH; i++) begin
                if (commit_valid[i]) begin
                    for (int j = 0; j < LQ_ENTRIES; j++) begin
                        if (lq_entries[j].valid && lq_entries[j].rob_idx == commit_rob_idx[i] &&
                            lq_entries[j].broadcast) begin
                            lq_entries[j].valid <= 1'b0;
                        end
                    end
                end
            end
            
            // Deallocate written stores
            for (int i = 0; i < SQ_ENTRIES; i++) begin
                if (sq_entries[i].valid && sq_entries[i].written) begin
                    sq_entries[i].valid <= 1'b0;
                end
            end
            
            // Update tail pointers
            lq_tail <= next_lq_tail[DISPATCH_WIDTH];
            sq_tail <= next_sq_tail[DISPATCH_WIDTH];
            
            // Advance head pointers past deallocated (invalid) entries
            // This ensures proper queue semantics and age-ordered iteration
            begin
                logic [LQ_IDX_BITS:0] new_lq_head;
                new_lq_head = lq_head;
                for (int i = 0; i < LQ_ENTRIES; i++) begin
                    logic [LQ_IDX_BITS-1:0] check_idx;
                    check_idx = new_lq_head[LQ_IDX_BITS-1:0];
                    // Stop at first valid entry or when we reach tail
                    if (new_lq_head != lq_tail && !lq_entries[check_idx].valid) begin
                        new_lq_head = new_lq_head + 1;
                    end
                end
                lq_head <= new_lq_head;
            end
            
            begin
                logic [SQ_IDX_BITS:0] new_sq_head;
                new_sq_head = sq_head;
                for (int i = 0; i < SQ_ENTRIES; i++) begin
                    logic [SQ_IDX_BITS-1:0] check_idx;
                    check_idx = new_sq_head[SQ_IDX_BITS-1:0];
                    // Stop at first valid entry or when we reach tail
                    if (new_sq_head != sq_tail && !sq_entries[check_idx].valid) begin
                        new_sq_head = new_sq_head + 1;
                    end
                end
                sq_head <= new_sq_head;
            end
            
            // Properly track counts with dispatches and deallocations
            lq_cnt <= lq_cnt + lq_dispatch_cnt - lq_dealloc_cnt;
            sq_cnt <= sq_cnt + sq_dispatch_cnt - sq_dealloc_cnt;
        end
    end

endmodule
