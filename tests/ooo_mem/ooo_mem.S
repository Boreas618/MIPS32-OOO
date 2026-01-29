#include "config.h"
.text
.global _start

_start:
    li $3, 0x1234
    sw $3, 0x2000($0)   # Store
    lw $4, 0x2000($0)   # Load should forward

    li $5, 0x1234
    li $2, MAGIC
    beq $4, $5, exit
    li $2, -1
    j exit

.org 0x8080
exit:
    nop
