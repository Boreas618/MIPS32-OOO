/* FDU Debugger */

#include <fdb.h>
#include <cpu.h>
#include <utils.h>

#include <set>

static bool need_debug = false;
static bool need_disasm = false;
static std::set<uint64_t> brks;

void fdb_init(bool debug)
{
	char *arch = NULL;
	const char *triple = NULL;

	need_debug = debug;

	arch = getenv("ARCH");
	if (!strcmp(arch, "loongarch"))
	{
		triple = "loongarch32-unknown-linux-gnu";
		need_disasm = true;
	}
	else if (!strcmp(arch, "mips"))
	{
		triple = "mipsel-linux-gnu";
		need_disasm = true;
	}
	else if (strcmp(arch, "dummy"))
		perror_exit("ARCH should be in [dummy, mips, loongarch].\n");

	printf("You are using %s ISA...\n", arch);
	if (need_disasm)
		disasm_init(triple);
}

static bool check_registers(void)
{
	if (cpu.pc == 0x1000 + 40 && cpu.regs[0] == 0 && cpu.regs[1] == 2 & cpu.regs[2] == 3)
		return true;
	return false;
}

static void fdb_run(void)
{
	while (!cpu.halt && !cpu.err)
	{
		cpu_exec_once();
		if (brks.count(cpu.pc))
		{
			printf("Breakpoint hit at 0x%lx.\n", cpu.pc);
			break;
		}
	}
}

static void fdb_print(void)
{
	printf("pc = 0x%lx\n", cpu.pc);
	for (size_t i = 0; i < 32; i++)
		printf("regs[%d] = 0x%lx\n", i, cpu.regs[i]);
}

static void fdb_debug_once(void)
{
	char buf[256];
	uint8_t bytes[16];
	char op;
	uint64_t addr;

	printf("(fdb) ");
	scanf(" %c", &op);

	switch (op)
	{
	/* next */
	case 'n':
		cpu_exec_once();

		if (need_disasm)
		{
			*(uint64_t *)bytes = cpu.last_inst;
			disassemble(buf, sizeof(buf), cpu.last_pc, bytes, sizeof(bytes));
			printf("0x%lx: %s\n", cpu.last_pc, buf);
		}
		else
			printf("0x%lx: 0x%lx\n", cpu.last_pc, cpu.last_inst);

		break;

	/* run */
	case 'r':
		fdb_run();
		break;

	/* breakpoint */
	case 'b':
		scanf(" 0x%lx", &addr);
		brks.insert(addr);
		break;

	/* print */
	case 'p':
		fdb_print();
		break;

	case 'q':
		return;
		break;

	default:
		printf("Unkown opcode.\n");
	}
}

static void print_branch_stats(void)
{
	printf("\n=== Branch Prediction Statistics ===\n");
	printf("Total branches:    %lu\n", cpu.bp_total_branches);
	printf("Mispredictions:    %lu\n", cpu.bp_mispredictions);
	if (cpu.bp_total_branches > 0) {
		double accuracy = 100.0 * (1.0 - (double)cpu.bp_mispredictions / (double)cpu.bp_total_branches);
		printf("Prediction accuracy: %.2f%%\n", accuracy);
	}
	printf("Total cycles:      %lu\n", cpu.counter);
	printf("====================================\n");
}

void fdb_start(void)
{
	if (need_debug)
		printf("Debug mode.\n");

	while (!cpu.halt && !cpu.err)
	{
		if (need_debug)
			fdb_debug_once();
		else
			cpu_exec_once();
	}

	// Print branch prediction statistics at end of simulation
	print_branch_stats();

	if (cpu.err)
		perror_exit("Error happend in CPU.\n");
}
