#include "config.h"

#define N 64

int _start() {
    int arr[N];
    int i;

    // Initialize sorted array: arr[i] = i * 3
    for (i = 0; i < N; i++) {
        arr[i] = i * 3;
    }

    // Binary search function inline
    int target, left, right, mid, idx;

    // Search for 0 (should find at index 0)
    target = 0; left = 0; right = N - 1; idx = -1;
    while (left <= right) {
        mid = left + (right - left) / 2;
        if (arr[mid] == target) { idx = mid; break; }
        if (arr[mid] < target) left = mid + 1;
        else right = mid - 1;
    }
    if (idx != 0) return -1;

    // Search for 99 (33*3, should find at index 33)
    target = 99; left = 0; right = N - 1; idx = -1;
    while (left <= right) {
        mid = left + (right - left) / 2;
        if (arr[mid] == target) { idx = mid; break; }
        if (arr[mid] < target) left = mid + 1;
        else right = mid - 1;
    }
    if (idx != 33) return -2;

    // Search for 189 (63*3, should find at index 63)
    target = 189; left = 0; right = N - 1; idx = -1;
    while (left <= right) {
        mid = left + (right - left) / 2;
        if (arr[mid] == target) { idx = mid; break; }
        if (arr[mid] < target) left = mid + 1;
        else right = mid - 1;
    }
    if (idx != 63) return -3;

    // Search for 1 (not in array)
    target = 1; left = 0; right = N - 1; idx = -1;
    while (left <= right) {
        mid = left + (right - left) / 2;
        if (arr[mid] == target) { idx = mid; break; }
        if (arr[mid] < target) left = mid + 1;
        else right = mid - 1;
    }
    if (idx != -1) return -4;

    // Search for 200 (beyond max)
    target = 200; left = 0; right = N - 1; idx = -1;
    while (left <= right) {
        mid = left + (right - left) / 2;
        if (arr[mid] == target) { idx = mid; break; }
        if (arr[mid] < target) left = mid + 1;
        else right = mid - 1;
    }
    if (idx != -1) return -5;

    // Stress test: search for all multiples of 3 up to 63
    int found_count = 0;
    for (i = 0; i < N; i++) {
        target = i * 3;
        left = 0; right = N - 1; idx = -1;
        while (left <= right) {
            mid = left + (right - left) / 2;
            if (arr[mid] == target) { idx = mid; break; }
            if (arr[mid] < target) left = mid + 1;
            else right = mid - 1;
        }
        if (idx == i) found_count++;
    }
    if (found_count != N) return -6;

    return MAGIC;
}
