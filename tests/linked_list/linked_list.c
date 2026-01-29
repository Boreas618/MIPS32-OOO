#include "config.h"

#define MAX_NODES 48

int _start() {
    // Node storage: data and next index (-1 for end)
    int node_data[MAX_NODES];
    int node_next[MAX_NODES];
    int free_head;
    int i;

    // Initialize free list
    for (i = 0; i < MAX_NODES - 1; i++) {
        node_data[i] = 0;
        node_next[i] = i + 1;
    }
    node_next[MAX_NODES - 1] = -1;
    free_head = 0;

    int list = -1;  // Empty list

    // Insert 1 to 24 at head
    for (i = 1; i <= 24; i++) {
        if (free_head == -1) return -100;  // Out of nodes
        int new_node = free_head;
        free_head = node_next[free_head];
        node_data[new_node] = i;
        node_next[new_node] = list;
        list = new_node;
    }

    // Count nodes - should be 24
    int count = 0;
    int curr = list;
    while (curr != -1) {
        count++;
        curr = node_next[curr];
    }
    if (count != 24) return -1;

    // Sum nodes - should be 1+2+...+24 = 300
    int sum = 0;
    curr = list;
    while (curr != -1) {
        sum += node_data[curr];
        curr = node_next[curr];
    }
    if (sum != 300) return -2;

    // Head should be 24 (last inserted)
    if (node_data[list] != 24) return -3;

    // Reverse the list
    int prev = -1;
    curr = list;
    while (curr != -1) {
        int next = node_next[curr];
        node_next[curr] = prev;
        prev = curr;
        curr = next;
    }
    list = prev;

    // Now head should be 1
    if (node_data[list] != 1) return -4;

    // Find element 12
    int found = -1;
    curr = list;
    while (curr != -1) {
        if (node_data[curr] == 12) {
            found = curr;
            break;
        }
        curr = node_next[curr];
    }
    if (found == -1 || node_data[found] != 12) return -5;

    // Delete element 12
    if (node_data[list] == 12) {
        int old_head = list;
        list = node_next[list];
        node_next[old_head] = free_head;
        free_head = old_head;
    } else {
        curr = list;
        while (node_next[curr] != -1) {
            if (node_data[node_next[curr]] == 12) {
                int to_delete = node_next[curr];
                node_next[curr] = node_next[to_delete];
                node_next[to_delete] = free_head;
                free_head = to_delete;
                break;
            }
            curr = node_next[curr];
        }
    }

    // Count should now be 23
    count = 0;
    curr = list;
    while (curr != -1) {
        count++;
        curr = node_next[curr];
    }
    if (count != 23) return -6;

    // Sum should be 300 - 12 = 288
    sum = 0;
    curr = list;
    while (curr != -1) {
        sum += node_data[curr];
        curr = node_next[curr];
    }
    if (sum != 288) return -7;

    // Element 12 should not be found
    found = -1;
    curr = list;
    while (curr != -1) {
        if (node_data[curr] == 12) {
            found = curr;
            break;
        }
        curr = node_next[curr];
    }
    if (found != -1) return -8;

    // Insert 100 at tail
    if (free_head == -1) return -101;
    int new_node = free_head;
    free_head = node_next[free_head];
    node_data[new_node] = 100;
    node_next[new_node] = -1;

    if (list == -1) {
        list = new_node;
    } else {
        curr = list;
        while (node_next[curr] != -1) {
            curr = node_next[curr];
        }
        node_next[curr] = new_node;
    }

    // Count should be 24
    count = 0;
    curr = list;
    while (curr != -1) {
        count++;
        curr = node_next[curr];
    }
    if (count != 24) return -9;

    // Sum should be 288 + 100 = 388
    sum = 0;
    curr = list;
    while (curr != -1) {
        sum += node_data[curr];
        curr = node_next[curr];
    }
    if (sum != 388) return -10;

    return MAGIC;
}
