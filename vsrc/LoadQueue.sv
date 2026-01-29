/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */

/**
 * Load Queue - Milestone 5.3
 *
 * Tracks in-flight load instructions for memory disambiguation.
 * Structure: 32-entry age-ordered queue
 *
 * Entry contents:
 * - Valid bit
 * - Address (when computed)
 * - Address valid bit
 * - Data (when loaded)
 * - Data valid bit
 * - ROB index
 * - Physical destination register
 *
 * Operations:
 * - Allocate on load dispatch
 * - Fill address when AGU completes
 * - Search store queue for forwarding
 * - Deallocate on commit
 */
module LoadQueue #(
    parameter LQ_ENTRIES = 32,
    parameter NUM_PHYS_REGS = 64,
    parameter DISPATCH_WIDTH = 4,
    parameter COMMIT_WIDTH = 4
) (
    input  logic clk,
    input  logic rst,

    // ========== Dispatch Interface (from RenameUnit/ROB) ==========
    input  logic [DISPATCH_WIDTH-1:0] dispatch_valid,
    input  logic [DISPATCH_WIDTH-1:0] dispatch_is_load,           // This instruction is a load
    input  logic [5:0] dispatch_prd [DISPATCH_WIDTH-1:0],         // Physical destination register
    input  logic [6:0] dispatch_rob_idx [DISPATCH_WIDTH-1:0],     // ROB index for age ordering
    
    output logic [$clog2(LQ_ENTRIES)-1:0] dispatch_lq_idx [DISPATCH_WIDTH-1:0], // Allocated LQ index
    output logic [DISPATCH_WIDTH-1:0] dispatch_lq_valid,          // Allocation successful
    output logic lq_full,                                          // Cannot accept more loads

    // ========== Address Computation Interface (from AGU/IssueUnit) ==========
    input  logic addr_valid,                                       // Address computation complete
    input  logic [6:0] addr_rob_idx,                              // ROB index of the load
    input  logic [31:0] addr_value,                               // Computed address

    // ========== Store Queue Forwarding Interface ==========
    // Request to store queue for forwarding check
    output logic fwd_req_valid,
    output logic [31:0] fwd_req_addr,
    output logic [6:0] fwd_req_rob_idx,
    
    // Response from store queue
    input  logic fwd_hit,                                          // Store queue has matching address
    input  logic fwd_data_valid,                                   // Forwarded data is valid
    input  logic [31:0] fwd_data,                                  // Forwarded data from store

    // ========== Cache Interface ==========
    output logic cache_req_valid,
    output logic [31:0] cache_req_addr,
    output logic [6:0] cache_req_rob_idx,
    output logic [5:0] cache_req_prd,
    
    input  logic cache_resp_valid,
    input  logic [6:0] cache_resp_rob_idx,
    input  logic [31:0] cache_resp_data,

    // ========== CDB Interface (broadcast load results) ==========
    output logic result_valid,
    output logic [5:0] result_prd,
    output logic [31:0] result_data,
    output logic [6:0] result_rob_idx,
    input  logic result_ack,                                       // CDB accepted our result

    // ========== Commit Interface ==========
    input  logic [COMMIT_WIDTH-1:0] commit_valid,
    input  logic [6:0] commit_rob_idx [COMMIT_WIDTH-1:0],

    // ========== Flush Interface ==========
    input  logic flush,
    input  logic [6:0] flush_rob_idx,                             // Flush loads younger than this

    // ========== Memory Violation Detection Interface ==========
    // From store queue when store address is computed
    input  logic store_addr_valid,
    input  logic [31:0] store_addr,
    input  logic [6:0] store_rob_idx,
    
    // ROB head index for proper age comparison (handles wraparound)
    input  logic [6:0] rob_head_idx,
    
    // Memory order violation output
    output logic violation_detected,
    output logic [6:0] violation_rob_idx,                         // ROB index of violating load

    // ========== Status ==========
    output logic [$clog2(LQ_ENTRIES):0] lq_count,
    output logic lq_empty
);

    localparam LQ_IDX_BITS = $clog2(LQ_ENTRIES);

    // Load queue entry
    typedef struct packed {
        logic valid;
        logic [31:0] addr;
        logic addr_valid;
        logic [31:0] data;
        logic data_valid;
        logic [5:0] prd;
        logic [6:0] rob_idx;
        logic executed;         // Sent to cache or forwarded
        logic committed;        // ROB committed this load
        logic waiting_fwd;      // Waiting for store data
    } lq_entry_t;

    // LQ storage
    lq_entry_t entries [LQ_ENTRIES-1:0];

    // Head and tail pointers
    logic [LQ_IDX_BITS:0] head_ptr;
    logic [LQ_IDX_BITS:0] tail_ptr;

    wire [LQ_IDX_BITS-1:0] head_idx = head_ptr[LQ_IDX_BITS-1:0];
    wire [LQ_IDX_BITS-1:0] tail_idx = tail_ptr[LQ_IDX_BITS-1:0];

    // Count
    logic [$clog2(LQ_ENTRIES):0] count;
    
    wire ptr_match = (head_ptr[LQ_IDX_BITS-1:0] == tail_ptr[LQ_IDX_BITS-1:0]);
    wire wrap_match = (head_ptr[LQ_IDX_BITS] == tail_ptr[LQ_IDX_BITS]);
    
    assign lq_empty = ptr_match && wrap_match;
    assign lq_full = ptr_match && !wrap_match;
    assign lq_count = count;

    // ==========================================================================
    // Dispatch Logic
    // ==========================================================================
    
    logic [2:0] dispatch_count;
    logic [DISPATCH_WIDTH-1:0] can_dispatch;
    logic [LQ_IDX_BITS:0] next_tail [DISPATCH_WIDTH:0];
    
    always_comb begin
        next_tail[0] = tail_ptr;
        dispatch_count = 0;
        
        for (int i = 0; i < DISPATCH_WIDTH; i++) begin
            logic is_load;
            logic has_space;
            logic [LQ_IDX_BITS:0] tent_tail;
            
            is_load = dispatch_valid[i] && dispatch_is_load[i];
            tent_tail = next_tail[i] + 1;
            has_space = !((tent_tail[LQ_IDX_BITS-1:0] == head_ptr[LQ_IDX_BITS-1:0]) &&
                         (tent_tail[LQ_IDX_BITS] != head_ptr[LQ_IDX_BITS]));
            
            can_dispatch[i] = is_load && has_space;
            
            if (can_dispatch[i]) begin
                dispatch_lq_idx[i] = next_tail[i][LQ_IDX_BITS-1:0];
                next_tail[i+1] = next_tail[i] + 1;
                dispatch_count = dispatch_count + 1;
            end else begin
                dispatch_lq_idx[i] = '0;
                next_tail[i+1] = next_tail[i];
            end
        end
        
        dispatch_lq_valid = can_dispatch;
    end

    // ==========================================================================
    // Address Fill Logic
    // ==========================================================================
    
    // Find the load with matching ROB index
    logic [LQ_IDX_BITS-1:0] addr_fill_idx;
    logic addr_fill_found;
    
    always_comb begin
        addr_fill_found = 1'b0;
        addr_fill_idx = '0;
        
        if (addr_valid) begin
            for (int i = 0; i < LQ_ENTRIES; i++) begin
                if (entries[i].valid && !entries[i].addr_valid &&
                    entries[i].rob_idx == addr_rob_idx) begin
                    addr_fill_found = 1'b1;
                    addr_fill_idx = i[LQ_IDX_BITS-1:0];
                end
            end
        end
    end

    // ==========================================================================
    // Store Forwarding Request
    // ==========================================================================
    
    // Find oldest load that has address but not data
    logic [LQ_IDX_BITS-1:0] fwd_check_idx;
    logic fwd_check_found;
    
    always_comb begin
        fwd_check_found = 1'b0;
        fwd_check_idx = '0;
        fwd_req_valid = 1'b0;
        fwd_req_addr = '0;
        fwd_req_rob_idx = '0;
        
        // Find oldest load needing execution (has address, no data, not executed)
        for (int i = 0; i < LQ_ENTRIES; i++) begin
            logic [LQ_IDX_BITS-1:0] idx;
            // Use bit truncation instead of modulo (works since LQ_ENTRIES is power of 2)
            idx = head_idx + i[LQ_IDX_BITS-1:0];
            
            if (!fwd_check_found && entries[idx].valid && entries[idx].addr_valid && 
                !entries[idx].data_valid && !entries[idx].executed) begin
                fwd_check_found = 1'b1;
                fwd_check_idx = idx;
                fwd_req_valid = 1'b1;
                fwd_req_addr = entries[idx].addr;
                fwd_req_rob_idx = entries[idx].rob_idx;
            end
        end
    end

    // ==========================================================================
    // Cache Request Logic
    // ==========================================================================
    
    // Request cache if no store forwarding or forward miss
    logic [LQ_IDX_BITS-1:0] cache_req_idx;
    logic cache_req_pending;
    
    always_comb begin
        cache_req_valid = 1'b0;
        cache_req_addr = '0;
        cache_req_rob_idx = '0;
        cache_req_prd = '0;
        cache_req_idx = '0;
        cache_req_pending = 1'b0;
        
        // Find oldest load ready for cache access
        for (int i = 0; i < LQ_ENTRIES; i++) begin
            logic [LQ_IDX_BITS-1:0] idx;
            // Use bit truncation instead of modulo
            idx = head_idx + i[LQ_IDX_BITS-1:0];
            
            if (!cache_req_pending && entries[idx].valid && entries[idx].addr_valid && 
                !entries[idx].data_valid && !entries[idx].executed && !entries[idx].waiting_fwd) begin
                cache_req_pending = 1'b1;
                cache_req_idx = idx;
                cache_req_valid = 1'b1;
                cache_req_addr = entries[idx].addr;
                cache_req_rob_idx = entries[idx].rob_idx;
                cache_req_prd = entries[idx].prd;
            end
        end
    end

    // ==========================================================================
    // Result Broadcast
    // ==========================================================================
    
    // Find completed load to broadcast
    logic [LQ_IDX_BITS-1:0] result_idx;
    logic result_found;
    
    always_comb begin
        result_found = 1'b0;
        result_idx = '0;
        result_valid = 1'b0;
        result_prd = '0;
        result_data = '0;
        result_rob_idx = '0;
        
        // Find oldest load with data ready to broadcast
        for (int i = 0; i < LQ_ENTRIES; i++) begin
            logic [LQ_IDX_BITS-1:0] idx;
            // Use bit truncation instead of modulo
            idx = head_idx + i[LQ_IDX_BITS-1:0];
            
            if (!result_found && entries[idx].valid && entries[idx].data_valid && 
                !entries[idx].committed) begin
                result_found = 1'b1;
                result_idx = idx;
                result_valid = 1'b1;
                result_prd = entries[idx].prd;
                result_data = entries[idx].data;
                result_rob_idx = entries[idx].rob_idx;
            end
        end
    end

    // ==========================================================================
    // Memory Order Violation Detection
    // ==========================================================================
    
    // When a store computes its address, check if any younger executed load
    // accessed the same address
    always_comb begin
        violation_detected = 1'b0;
        violation_rob_idx = '0;
        
        if (store_addr_valid) begin
            for (int i = 0; i < LQ_ENTRIES; i++) begin
                if (entries[i].valid && entries[i].addr_valid && entries[i].executed) begin
                    // Check if this load is younger than the store
                    // Use relative age calculation to handle ROB wraparound
                    logic is_younger;
                    logic [6:0] load_age_rel, store_age_rel;
                    
                    // Calculate age relative to ROB head (handles wraparound)
                    load_age_rel = entries[i].rob_idx - rob_head_idx;
                    store_age_rel = store_rob_idx - rob_head_idx;
                    
                    // Younger if load's relative age is greater than store's
                    is_younger = (load_age_rel > store_age_rel);
                    
                    // Check for address match
                    if (is_younger && (entries[i].addr[31:2] == store_addr[31:2])) begin
                        // Report the oldest violating load (smallest relative age among violators)
                        if (!violation_detected || load_age_rel < (violation_rob_idx - rob_head_idx)) begin
                            violation_detected = 1'b1;
                            violation_rob_idx = entries[i].rob_idx;
                        end
                    end
                end
            end
        end
    end

    // ==========================================================================
    // Commit Logic
    // ==========================================================================
    
    logic [2:0] commit_count_internal;
    logic [LQ_IDX_BITS:0] next_head [COMMIT_WIDTH:0];
    
    always_comb begin
        next_head[0] = head_ptr;
        commit_count_internal = 0;
        
        for (int i = 0; i < COMMIT_WIDTH; i++) begin
            if (commit_valid[i]) begin
                // Find and deallocate committed loads
                for (int j = 0; j < LQ_ENTRIES; j++) begin
                    if (entries[j].valid && entries[j].rob_idx == commit_rob_idx[i]) begin
                        // This load is being committed
                        commit_count_internal = commit_count_internal + 1;
                    end
                end
            end
            next_head[i+1] = next_head[i];
        end
    end

    // ==========================================================================
    // State Update
    // ==========================================================================
    
    always_ff @(posedge clk) begin
        if (rst) begin
            head_ptr <= '0;
            tail_ptr <= '0;
            count <= '0;
            
            for (int i = 0; i < LQ_ENTRIES; i++) begin
                entries[i].valid <= 1'b0;
            end
        end else if (flush) begin
            // Flush younger loads
            // Count remaining valid entries to recalculate tail
            logic [LQ_IDX_BITS:0] new_count;
            new_count = '0;
            
            for (int i = 0; i < LQ_ENTRIES; i++) begin
                if (entries[i].valid) begin
                    // Check if younger than or equal to flush point (using relative age)
                    logic is_younger_or_equal;
                    logic [6:0] entry_age_rel, flush_age_rel;
                    
                    entry_age_rel = entries[i].rob_idx - rob_head_idx;
                    flush_age_rel = flush_rob_idx - rob_head_idx;
                    
                    is_younger_or_equal = (entry_age_rel >= flush_age_rel);
                    
                    if (is_younger_or_equal) begin
                        entries[i].valid <= 1'b0;
                    end else begin
                        new_count = new_count + 1;
                    end
                end
            end
            
            // Recalculate tail: tail = head + remaining_count
            tail_ptr <= head_ptr + new_count;
            count <= new_count;
        end else begin
            // Dispatch new loads
            for (int i = 0; i < DISPATCH_WIDTH; i++) begin
                if (can_dispatch[i]) begin
                    logic [LQ_IDX_BITS-1:0] alloc_idx;
                    alloc_idx = dispatch_lq_idx[i];
                    
                    entries[alloc_idx].valid <= 1'b1;
                    entries[alloc_idx].addr <= '0;
                    entries[alloc_idx].addr_valid <= 1'b0;
                    entries[alloc_idx].data <= '0;
                    entries[alloc_idx].data_valid <= 1'b0;
                    entries[alloc_idx].prd <= dispatch_prd[i];
                    entries[alloc_idx].rob_idx <= dispatch_rob_idx[i];
                    entries[alloc_idx].executed <= 1'b0;
                    entries[alloc_idx].committed <= 1'b0;
                    entries[alloc_idx].waiting_fwd <= 1'b0;
                end
            end
            
            // Address fill
            if (addr_fill_found) begin
                entries[addr_fill_idx].addr <= addr_value;
                entries[addr_fill_idx].addr_valid <= 1'b1;
            end
            
            // Store forwarding response
            if (fwd_check_found && fwd_hit) begin
                if (fwd_data_valid) begin
                    entries[fwd_check_idx].data <= fwd_data;
                    entries[fwd_check_idx].data_valid <= 1'b1;
                    entries[fwd_check_idx].executed <= 1'b1;
                end else begin
                    // Store has address but not data - wait
                    entries[fwd_check_idx].waiting_fwd <= 1'b1;
                end
            end
            
            // Mark loads as executed when sent to cache
            if (cache_req_valid && cache_req_pending) begin
                entries[cache_req_idx].executed <= 1'b1;
            end
            
            // Cache response
            if (cache_resp_valid) begin
                for (int i = 0; i < LQ_ENTRIES; i++) begin
                    if (entries[i].valid && entries[i].rob_idx == cache_resp_rob_idx) begin
                        entries[i].data <= cache_resp_data;
                        entries[i].data_valid <= 1'b1;
                    end
                end
            end
            
            // Commit: deallocate entries
            for (int i = 0; i < COMMIT_WIDTH; i++) begin
                if (commit_valid[i]) begin
                    for (int j = 0; j < LQ_ENTRIES; j++) begin
                        if (entries[j].valid && entries[j].rob_idx == commit_rob_idx[i]) begin
                            entries[j].valid <= 1'b0;
                        end
                    end
                end
            end
            
            // Result acknowledged - mark as committed
            if (result_valid && result_ack) begin
                entries[result_idx].committed <= 1'b1;
            end
            
            // Update pointers
            tail_ptr <= next_tail[DISPATCH_WIDTH];
            count <= count + dispatch_count - commit_count_internal;
        end
    end

endmodule
