#include "config.h"
.text
.global _start

_start:
    li $3, 0
    li $4, 3
loop:
    addiu $3, $3, 1
    bne $3, $4, loop    # Branch mispredicts on exit

    addiu $5, $3, 10    # $5 = 13
    li $6, 13
    li $2, MAGIC
    beq $5, $6, exit
    li $2, -1
    j exit

.org 0x8080
exit:
    nop
