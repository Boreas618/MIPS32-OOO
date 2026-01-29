#ifndef __CPU_H__
#define __CPU_H__

#include <verilated.h>
#include <verilated_dpi.h>

#include <VTop.h>
#include <VTop__Dpi.h>

#include <stdint.h>
#include <vector>

typedef VTop Vcpu;

typedef struct
{
	uint32_t *regs;
	uint32_t *watch_list;
	uint64_t pc;
	uint64_t halt;
	uint64_t counter;
	uint64_t last_pc;
	uint64_t last_inst;
	uint64_t err;
} CPU;

extern CPU cpu;

void cpu_init(int argc, char *argv[]);
void cpu_exec_once(void);
void cpu_final(void);

#endif // __CPU_H__