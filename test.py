#!/bin/python3

import sys
import subprocess
import re

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
NORMAL = "\033[0m"

def parse_branch_stats(output):
    """Parse branch prediction statistics from CPU output."""
    stats = {}
    
    # Parse total branches
    match = re.search(r'Total branches:\s+(\d+)', output)
    if match:
        stats['total_branches'] = int(match.group(1))
    
    # Parse mispredictions
    match = re.search(r'Mispredictions:\s+(\d+)', output)
    if match:
        stats['mispredictions'] = int(match.group(1))
    
    # Parse accuracy
    match = re.search(r'Prediction accuracy:\s+([\d.]+)%', output)
    if match:
        stats['accuracy'] = float(match.group(1))
    
    # Parse total cycles
    match = re.search(r'Total cycles:\s+(\d+)', output)
    if match:
        stats['cycles'] = int(match.group(1))
    
    return stats

if __name__ == "__main__":
    tests = [
        ("./tests/jr/jr.bin", 'JUMP'),
        ("./tests/bitwise/bitwise.bin", 'BITWISE'),
        ("./tests/addiu/addiu.bin", 'IMM'),
        ("./tests/addu/addu.bin", 'OPs'),
        ("./tests/load_store/load_store.bin", 'LOAD_STORE'),
        ("./tests/branch/branch.bin", 'BRANCH'),
        ("./tests/lui/lui.bin", 'LUI'),
        ("./tests/quick_sort/quick_sort.bin", 'QSORT')
    ]
    good = True
    all_stats = []
    
    for i, (t, n) in enumerate(tests):
        cmd = sys.argv[1] + " " + t
        p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = p.communicate()
        output = stdout.decode('utf-8')
        
        if p.returncode != 0:
            print(RED + f"[FAILED]\tTEST{i+1}-{n}" + NORMAL)
            good = False
            break
        else:
            stats = parse_branch_stats(output)
            all_stats.append((n, stats))
            
            # Format accuracy display
            accuracy_str = ""
            if 'accuracy' in stats and stats.get('total_branches', 0) > 0:
                acc = stats['accuracy']
                if acc >= 90:
                    color = GREEN
                elif acc >= 80:
                    color = YELLOW
                else:
                    color = RED
                accuracy_str = f" [BP: {color}{acc:.1f}%{NORMAL}]"
            
            print(GREEN + f"[OK]    \tTEST{i+1}-{n}" + NORMAL + accuracy_str)

    if good:
        print(GREEN + "\nACCEPTED." + NORMAL)
        
        # Print aggregate branch prediction statistics
        print(CYAN + "\n" + "="*50 + NORMAL)
        print(CYAN + "       BRANCH PREDICTION SUMMARY" + NORMAL)
        print(CYAN + "="*50 + NORMAL)
        
        total_branches = sum(s.get('total_branches', 0) for _, s in all_stats)
        total_mispredictions = sum(s.get('mispredictions', 0) for _, s in all_stats)
        total_cycles = sum(s.get('cycles', 0) for _, s in all_stats)
        
        print(f"{'Test':<15} {'Branches':>10} {'Mispred':>10} {'Accuracy':>12} {'Cycles':>10}")
        print("-"*60)
        
        for name, stats in all_stats:
            branches = stats.get('total_branches', 0)
            mispred = stats.get('mispredictions', 0)
            accuracy = stats.get('accuracy', 0.0) if branches > 0 else 0.0
            cycles = stats.get('cycles', 0)
            
            acc_color = GREEN if accuracy >= 90 else (YELLOW if accuracy >= 80 else RED)
            print(f"{name:<15} {branches:>10} {mispred:>10} {acc_color}{accuracy:>11.1f}%{NORMAL} {cycles:>10}")
        
        print("-"*60)
        
        if total_branches > 0:
            overall_accuracy = 100.0 * (1.0 - total_mispredictions / total_branches)
            acc_color = GREEN if overall_accuracy >= 90 else (YELLOW if overall_accuracy >= 80 else RED)
            print(f"{'TOTAL':<15} {total_branches:>10} {total_mispredictions:>10} {acc_color}{overall_accuracy:>11.1f}%{NORMAL} {total_cycles:>10}")
        
        print(CYAN + "="*50 + NORMAL)
