#include "config.h"
.text
.global _start

_start:
    li $3, 1
    addiu $4, $3, 1     # RAW: $4 depends on $3
    addiu $5, $4, 1     # RAW: $5 depends on $4
    addiu $6, $5, 1     # RAW chain continues
    addiu $7, $6, 1
    addiu $8, $7, 1

    # Independent operations (can execute OoO)
    li $10, 100
    li $11, 200

    addu $9, $8, $10    # $9 = 6 + 100 = 106
    li $12, 106
    li $2, MAGIC
    beq $9, $12, exit
    li $2, -1
    j exit

.org 0x8080
exit:
    nop
