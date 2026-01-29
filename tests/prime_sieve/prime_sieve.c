#include "config.h"

#define MAX_N 32

int _start() {
    int is_prime[MAX_N];
    int i, j, count = 0;

    // Initialize all as prime
    for (i = 0; i < MAX_N; i++) {
        is_prime[i] = 1;
    }

    is_prime[0] = 0;
    is_prime[1] = 0;

    // Sieve of Eratosthenes
    for (i = 2; i * i < MAX_N; i++) {
        if (is_prime[i]) {
            for (j = i * i; j < MAX_N; j += i) {
                is_prime[j] = 0;
            }
        }
    }

    // Count primes
    for (i = 2; i < MAX_N; i++) {
        if (is_prime[i]) {
            count++;
        }
    }

    // There are 11 primes below 32: 2,3,5,7,11,13,17,19,23,29,31
    if (count != 11)
        return -1;

    return MAGIC;
}
