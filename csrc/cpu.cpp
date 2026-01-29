/* The CPU */

#include <cpu.h>
#include <mm.h>

#include <stdio.h>

Vcpu *vcpu = NULL;
CPU cpu;

extern "C" void set_regs_ptr(const svOpenArrayHandle r)
{
	cpu.regs = (uint32_t *)(((VerilatedDpiOpenVar *)r)->datap());
}

extern "C" void set_debug_port_ptr(const svOpenArrayHandle r)
{
	cpu.watch_list = (uint32_t *)(((VerilatedDpiOpenVar *)r)->datap());
}

static inline void update_cpu()
{
	cpu.pc = vcpu->pc;
	cpu.halt = vcpu->halt;
	cpu.counter = vcpu->system_counter;
	cpu.last_pc = vcpu->last_pc;
	cpu.last_inst = vcpu->last_inst;
	cpu.err = vcpu->err;
	// Branch prediction statistics
	cpu.bp_total_branches = vcpu->bp_total_branches_out;
	cpu.bp_mispredictions = vcpu->bp_mispredictions_out;
}

static inline void single_cycle()
{
	vcpu->clk = 0;
	vcpu->eval();
	vcpu->clk = 1;
	vcpu->eval();
	update_cpu();
}

static inline void reset()
{
	vcpu->rst = 1;
	single_cycle();
	vcpu->rst = 0;
}

void cpu_init(int argc, char *argv[])
{
	Verilated::commandArgs(argc, argv);
	vcpu = new Vcpu;
	reset();
}

void cpu_final(void)
{
	vcpu->final();
	delete vcpu;
}

void cpu_exec_once(void)
{
	single_cycle();
}
