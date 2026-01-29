#include "config.h"

#define MAX_FIB 32

int _start() {
    int memo[MAX_FIB];
    int i;

    // Initialize memo table
    for (i = 0; i < MAX_FIB; i++) {
        memo[i] = 0;
    }

    // Build Fibonacci sequence iteratively
    memo[0] = 0;
    memo[1] = 1;
    for (i = 2; i < MAX_FIB; i++) {
        memo[i] = memo[i-1] + memo[i-2];
    }

    // Verify specific values
    if (memo[10] != 55)      // fib(10) = 55
        return -1;
    if (memo[20] != 6765)    // fib(20) = 6765
        return -2;
    if (memo[30] != 832040)  // fib(30) = 832040
        return -3;
    if (memo[15] != 610)     // fib(15) = 610
        return -4;
    if (memo[25] != 75025)   // fib(25) = 75025
        return -5;

    // Calculate sum of first 20 Fibonacci numbers
    int sum = 0;
    for (i = 1; i <= 20; i++) {
        sum += memo[i];
    }
    // Sum of fib(1) to fib(20) = fib(22) - 1 = 17711 - 1 = 17710
    if (sum != 17710)
        return -6;

    // Verify the sequence
    for (i = 2; i < 30; i++) {
        if (memo[i] != memo[i-1] + memo[i-2])
            return -7;
    }

    return MAGIC;
}
