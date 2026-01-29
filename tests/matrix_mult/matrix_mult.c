#include "config.h"

#define N 6

int _start() {
    int A[N][N];
    int B[N][N];
    int C[N][N];
    int i, j, k;

    // Initialize matrices A and B
    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            A[i][j] = i + j;
            B[i][j] = i * 2 + j;
            C[i][j] = 0;
        }
    }

    // Matrix multiplication: C = A * B
    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            for (k = 0; k < N; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }

    // Verify C[0][0] = sum of A[0][k] * B[k][0] = sum of k * (k*2) = 2 * sum(k^2)
    // For N=6: 2 * (0+1+4+9+16+25) = 2*55 = 110
    int expected_00 = 0;
    for (k = 0; k < N; k++) {
        expected_00 += (0 + k) * (k * 2 + 0);
    }
    if (C[0][0] != expected_00)
        return -1;

    // Verify C[N-1][N-1]
    int expected_nn = 0;
    for (k = 0; k < N; k++) {
        expected_nn += (N - 1 + k) * (k * 2 + N - 1);
    }
    if (C[N-1][N-1] != expected_nn)
        return -2;

    // Verify C[2][3]
    int expected_23 = 0;
    for (k = 0; k < N; k++) {
        expected_23 += (2 + k) * (k * 2 + 3);
    }
    if (C[2][3] != expected_23)
        return -3;

    return MAGIC;
}
