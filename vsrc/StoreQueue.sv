/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */

/**
 * Store Queue - Milestone 5.4
 *
 * Tracks in-flight store instructions.
 * Structure: 32-entry age-ordered queue
 *
 * Entry contents:
 * - Valid bit
 * - Address and address valid
 * - Data and data valid
 * - ROB index
 * - Committed bit
 *
 * Operations:
 * - Allocate on store dispatch
 * - Fill address/data when ready
 * - Forward to younger loads
 * - Write to cache on commit
 */
module StoreQueue #(
    parameter SQ_ENTRIES = 32,
    parameter NUM_PHYS_REGS = 64,
    parameter DISPATCH_WIDTH = 4,
    parameter COMMIT_WIDTH = 4
) (
    input  logic clk,
    input  logic rst,

    // ========== Dispatch Interface (from RenameUnit/ROB) ==========
    input  logic [DISPATCH_WIDTH-1:0] dispatch_valid,
    input  logic [DISPATCH_WIDTH-1:0] dispatch_is_store,          // This instruction is a store
    input  logic [6:0] dispatch_rob_idx [DISPATCH_WIDTH-1:0],     // ROB index for age ordering
    
    output logic [$clog2(SQ_ENTRIES)-1:0] dispatch_sq_idx [DISPATCH_WIDTH-1:0], // Allocated SQ index
    output logic [DISPATCH_WIDTH-1:0] dispatch_sq_valid,          // Allocation successful
    output logic sq_full,                                          // Cannot accept more stores

    // ========== Address Computation Interface (from AGU/IssueUnit) ==========
    input  logic addr_valid,                                       // Address computation complete
    input  logic [6:0] addr_rob_idx,                              // ROB index of the store
    input  logic [31:0] addr_value,                               // Computed address

    // ========== Data Ready Interface (from execution) ==========
    input  logic data_valid,                                       // Store data ready
    input  logic [6:0] data_rob_idx,                              // ROB index of the store
    input  logic [31:0] data_value,                               // Store data

    // ========== Load Queue Forwarding Interface ==========
    // Request from load queue
    input  logic fwd_req_valid,
    input  logic [31:0] fwd_req_addr,
    input  logic [6:0] fwd_req_rob_idx,                           // Load's ROB index (for age check)
    
    // Response to load queue
    output logic fwd_hit,                                          // Found matching older store
    output logic fwd_data_valid,                                   // Store data is available
    output logic [31:0] fwd_data,                                  // Forwarded data

    // ========== Store Address Broadcast (for memory order violation) ==========
    output logic store_addr_broadcast_valid,
    output logic [31:0] store_addr_broadcast,
    output logic [6:0] store_addr_broadcast_rob_idx,

    // ========== Commit Interface ==========
    input  logic [COMMIT_WIDTH-1:0] commit_valid,
    input  logic [COMMIT_WIDTH-1:0] commit_is_store,
    input  logic [6:0] commit_rob_idx [COMMIT_WIDTH-1:0],

    // ========== Cache Write Interface ==========
    output logic cache_write_valid,
    output logic [31:0] cache_write_addr,
    output logic [31:0] cache_write_data,
    output logic [6:0] cache_write_rob_idx,
    input  logic cache_write_ack,                                  // Cache accepted the write

    // ========== ROB Interface (mark store address ready) ==========
    output logic rob_store_addr_valid,
    output logic [6:0] rob_store_addr_rob_idx,

    // ========== Flush Interface ==========
    input  logic flush,
    input  logic [6:0] flush_rob_idx,                             // Flush stores younger than this

    // ========== ROB Head Index (for proper age comparison) ==========
    input  logic [6:0] rob_head_idx,                              // ROB head index for wraparound handling

    // ========== Status ==========
    output logic [$clog2(SQ_ENTRIES):0] sq_count,
    output logic sq_empty
);

    localparam SQ_IDX_BITS = $clog2(SQ_ENTRIES);

    // Store queue entry
    typedef struct packed {
        logic valid;
        logic [31:0] addr;
        logic addr_valid;
        logic [31:0] data;
        logic data_valid;
        logic [6:0] rob_idx;
        logic committed;        // ROB committed this store
        logic written;          // Written to cache
    } sq_entry_t;

    // SQ storage
    sq_entry_t entries [SQ_ENTRIES-1:0];

    // Head and tail pointers
    logic [SQ_IDX_BITS:0] head_ptr;
    logic [SQ_IDX_BITS:0] tail_ptr;

    wire [SQ_IDX_BITS-1:0] head_idx = head_ptr[SQ_IDX_BITS-1:0];
    wire [SQ_IDX_BITS-1:0] tail_idx = tail_ptr[SQ_IDX_BITS-1:0];

    // Count
    logic [$clog2(SQ_ENTRIES):0] count;
    
    wire ptr_match = (head_ptr[SQ_IDX_BITS-1:0] == tail_ptr[SQ_IDX_BITS-1:0]);
    wire wrap_match = (head_ptr[SQ_IDX_BITS] == tail_ptr[SQ_IDX_BITS]);
    
    assign sq_empty = ptr_match && wrap_match;
    assign sq_full = ptr_match && !wrap_match;
    assign sq_count = count;

    // ==========================================================================
    // Dispatch Logic
    // ==========================================================================
    
    logic [2:0] dispatch_count;
    logic [DISPATCH_WIDTH-1:0] can_dispatch;
    logic [SQ_IDX_BITS:0] next_tail [DISPATCH_WIDTH:0];
    
    always_comb begin
        next_tail[0] = tail_ptr;
        dispatch_count = 0;
        
        for (int i = 0; i < DISPATCH_WIDTH; i++) begin
            logic is_store;
            logic has_space;
            logic [SQ_IDX_BITS:0] tent_tail;
            
            is_store = dispatch_valid[i] && dispatch_is_store[i];
            tent_tail = next_tail[i] + 1;
            has_space = !((tent_tail[SQ_IDX_BITS-1:0] == head_ptr[SQ_IDX_BITS-1:0]) &&
                         (tent_tail[SQ_IDX_BITS] != head_ptr[SQ_IDX_BITS]));
            
            can_dispatch[i] = is_store && has_space;
            
            if (can_dispatch[i]) begin
                dispatch_sq_idx[i] = next_tail[i][SQ_IDX_BITS-1:0];
                next_tail[i+1] = next_tail[i] + 1;
                dispatch_count = dispatch_count + 1;
            end else begin
                dispatch_sq_idx[i] = '0;
                next_tail[i+1] = next_tail[i];
            end
        end
        
        dispatch_sq_valid = can_dispatch;
    end

    // ==========================================================================
    // Address Fill Logic
    // ==========================================================================
    
    logic [SQ_IDX_BITS-1:0] addr_fill_idx;
    logic addr_fill_found;
    
    always_comb begin
        addr_fill_found = 1'b0;
        addr_fill_idx = '0;
        store_addr_broadcast_valid = 1'b0;
        store_addr_broadcast = '0;
        store_addr_broadcast_rob_idx = '0;
        rob_store_addr_valid = 1'b0;
        rob_store_addr_rob_idx = '0;
        
        if (addr_valid) begin
            for (int i = 0; i < SQ_ENTRIES; i++) begin
                if (entries[i].valid && !entries[i].addr_valid &&
                    entries[i].rob_idx == addr_rob_idx) begin
                    addr_fill_found = 1'b1;
                    addr_fill_idx = i[SQ_IDX_BITS-1:0];
                    
                    // Broadcast for memory order violation detection
                    store_addr_broadcast_valid = 1'b1;
                    store_addr_broadcast = addr_value;
                    store_addr_broadcast_rob_idx = addr_rob_idx;
                    
                    // Notify ROB that store address is ready
                    rob_store_addr_valid = 1'b1;
                    rob_store_addr_rob_idx = addr_rob_idx;
                end
            end
        end
    end

    // ==========================================================================
    // Data Fill Logic
    // ==========================================================================
    
    logic [SQ_IDX_BITS-1:0] data_fill_idx;
    logic data_fill_found;
    
    always_comb begin
        data_fill_found = 1'b0;
        data_fill_idx = '0;
        
        if (data_valid) begin
            for (int i = 0; i < SQ_ENTRIES; i++) begin
                if (entries[i].valid && !entries[i].data_valid &&
                    entries[i].rob_idx == data_rob_idx) begin
                    data_fill_found = 1'b1;
                    data_fill_idx = i[SQ_IDX_BITS-1:0];
                end
            end
        end
    end

    // ==========================================================================
    // Store-to-Load Forwarding
    // ==========================================================================
    
    // Track the youngest matching store's ROB index for comparison
    logic [6:0] fwd_youngest_rob_idx;
    
    always_comb begin
        fwd_hit = 1'b0;
        fwd_data_valid = 1'b0;
        fwd_data = '0;
        fwd_youngest_rob_idx = '0;
        
        if (fwd_req_valid) begin
            // Search for matching older store using relative age (handles ROB wraparound)
            for (int i = 0; i < SQ_ENTRIES; i++) begin
                if (entries[i].valid && entries[i].addr_valid) begin
                    // Check if this store is older than the load using relative age
                    logic is_older;
                    logic [6:0] store_age_rel, load_age_rel;
                    
                    // Calculate age relative to ROB head (handles wraparound)
                    store_age_rel = entries[i].rob_idx - rob_head_idx;
                    load_age_rel = fwd_req_rob_idx - rob_head_idx;
                    
                    // Older if store's relative age is less than load's
                    is_older = (store_age_rel < load_age_rel);
                    
                    // Check for address match (word-aligned)
                    if (is_older && (entries[i].addr[31:2] == fwd_req_addr[31:2])) begin
                        // Found matching older store
                        // Take the youngest matching older store (largest relative age among older stores)
                        if (!fwd_hit || store_age_rel > (fwd_youngest_rob_idx - rob_head_idx)) begin
                            fwd_hit = 1'b1;
                            fwd_data_valid = entries[i].data_valid;
                            fwd_data = entries[i].data;
                            fwd_youngest_rob_idx = entries[i].rob_idx;
                        end
                    end
                end
            end
        end
    end

    // ==========================================================================
    // Cache Write Logic
    // ==========================================================================
    
    // Find oldest committed store ready to write to cache
    logic [SQ_IDX_BITS-1:0] write_idx;
    logic write_found;
    
    always_comb begin
        write_found = 1'b0;
        write_idx = '0;
        cache_write_valid = 1'b0;
        cache_write_addr = '0;
        cache_write_data = '0;
        cache_write_rob_idx = '0;
        
        // Search from head (oldest) to find committed store ready to write
        for (int i = 0; i < SQ_ENTRIES; i++) begin
            logic [SQ_IDX_BITS-1:0] idx;
            // Use bit truncation instead of modulo (works since SQ_ENTRIES is power of 2)
            idx = head_idx + i[SQ_IDX_BITS-1:0];
            
            if (!write_found && entries[idx].valid && entries[idx].committed &&
                entries[idx].addr_valid && entries[idx].data_valid &&
                !entries[idx].written) begin
                write_found = 1'b1;
                write_idx = idx;
                cache_write_valid = 1'b1;
                cache_write_addr = entries[idx].addr;
                cache_write_data = entries[idx].data;
                cache_write_rob_idx = entries[idx].rob_idx;
            end
        end
    end

    // ==========================================================================
    // Deallocate Logic
    // ==========================================================================
    
    // Remove written stores from head
    logic [2:0] dealloc_count;
    logic [SQ_IDX_BITS:0] next_head [COMMIT_WIDTH:0];
    
    always_comb begin
        next_head[0] = head_ptr;
        dealloc_count = 0;
        
        // Deallocate written stores at head
        if (entries[head_idx].valid && entries[head_idx].written) begin
            next_head[1] = head_ptr + 1;
            dealloc_count = 1;
        end else begin
            next_head[1] = head_ptr;
        end
        
        // Continue deallocating
        for (int i = 1; i < COMMIT_WIDTH; i++) begin
            logic [SQ_IDX_BITS-1:0] check_idx;
            check_idx = next_head[i][SQ_IDX_BITS-1:0];
            
            if (entries[check_idx].valid && entries[check_idx].written) begin
                next_head[i+1] = next_head[i] + 1;
                dealloc_count = dealloc_count + 1;
            end else begin
                next_head[i+1] = next_head[i];
            end
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
            
            for (int i = 0; i < SQ_ENTRIES; i++) begin
                entries[i].valid <= 1'b0;
            end
        end else if (flush) begin
            // Flush younger stores (keep committed stores)
            // Count remaining valid entries to recalculate tail
            logic [SQ_IDX_BITS:0] new_count;
            new_count = '0;
            
            for (int i = 0; i < SQ_ENTRIES; i++) begin
                if (entries[i].valid) begin
                    if (!entries[i].committed) begin
                        // Check if younger than or equal to flush point using relative age
                        logic is_younger_or_equal;
                        logic [6:0] entry_age_rel, flush_age_rel;
                        
                        // Calculate age relative to ROB head (handles wraparound)
                        entry_age_rel = entries[i].rob_idx - rob_head_idx;
                        flush_age_rel = flush_rob_idx - rob_head_idx;
                        
                        is_younger_or_equal = (entry_age_rel >= flush_age_rel);
                        
                        if (is_younger_or_equal) begin
                            entries[i].valid <= 1'b0;
                        end else begin
                            new_count = new_count + 1;
                        end
                    end else begin
                        // Committed stores are kept
                        new_count = new_count + 1;
                    end
                end
            end
            
            // Recalculate tail: tail = head + remaining_count
            tail_ptr <= head_ptr + new_count;
            count <= new_count;
        end else begin
            // Dispatch new stores
            for (int i = 0; i < DISPATCH_WIDTH; i++) begin
                if (can_dispatch[i]) begin
                    logic [SQ_IDX_BITS-1:0] alloc_idx;
                    alloc_idx = dispatch_sq_idx[i];
                    
                    entries[alloc_idx].valid <= 1'b1;
                    entries[alloc_idx].addr <= '0;
                    entries[alloc_idx].addr_valid <= 1'b0;
                    entries[alloc_idx].data <= '0;
                    entries[alloc_idx].data_valid <= 1'b0;
                    entries[alloc_idx].rob_idx <= dispatch_rob_idx[i];
                    entries[alloc_idx].committed <= 1'b0;
                    entries[alloc_idx].written <= 1'b0;
                end
            end
            
            // Address fill
            if (addr_fill_found) begin
                entries[addr_fill_idx].addr <= addr_value;
                entries[addr_fill_idx].addr_valid <= 1'b1;
            end
            
            // Data fill
            if (data_fill_found) begin
                entries[data_fill_idx].data <= data_value;
                entries[data_fill_idx].data_valid <= 1'b1;
            end
            
            // Commit: mark stores as committed
            for (int i = 0; i < COMMIT_WIDTH; i++) begin
                if (commit_valid[i] && commit_is_store[i]) begin
                    for (int j = 0; j < SQ_ENTRIES; j++) begin
                        if (entries[j].valid && entries[j].rob_idx == commit_rob_idx[i]) begin
                            entries[j].committed <= 1'b1;
                        end
                    end
                end
            end
            
            // Mark store as written when cache acknowledges
            if (cache_write_valid && cache_write_ack) begin
                entries[write_idx].written <= 1'b1;
            end
            
            // Deallocate written stores at head
            for (int i = 0; i < SQ_ENTRIES; i++) begin
                logic [SQ_IDX_BITS-1:0] idx;
                idx = i[SQ_IDX_BITS-1:0];
                
                if (entries[idx].valid && entries[idx].written) begin
                    // Check if at head
                    if (idx == head_idx) begin
                        entries[idx].valid <= 1'b0;
                    end
                end
            end
            
            // Update pointers
            tail_ptr <= next_tail[DISPATCH_WIDTH];
            head_ptr <= next_head[COMMIT_WIDTH];
            count <= count + dispatch_count - dealloc_count;
        end
    end

endmodule
