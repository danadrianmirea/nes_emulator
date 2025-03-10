#ifndef CPU_H
#define CPU_H

#include "nes.h"
#include "memory.h"

// Status register flags
#define FLAG_C 0x01  // Carry
#define FLAG_Z 0x02  // Zero
#define FLAG_I 0x04  // Interrupt disable
#define FLAG_D 0x08  // Decimal (not used in NES, but affects flags)
#define FLAG_B 0x10  // Break command
#define FLAG_U 0x20  // Unused (always 1)
#define FLAG_V 0x40  // Overflow
#define FLAG_N 0x80  // Negative

// Addressing modes
typedef enum {
    IMP,  // Implied
    ACC,  // Accumulator
    IMM,  // Immediate
    ZP0,  // Zero Page
    ZPX,  // Zero Page,X
    ZPY,  // Zero Page,Y
    REL,  // Relative
    ABS,  // Absolute
    ABX,  // Absolute,X
    ABY,  // Absolute,Y
    IND,  // Indirect
    IZX,  // Indirect,X
    IZY   // Indirect,Y
} AddressingMode;

// CPU Structure
typedef struct {
    // Registers
    u16 pc;    // Program Counter
    u8  sp;    // Stack Pointer
    u8  a;     // Accumulator
    u8  x;     // X Index
    u8  y;     // Y Index
    u8  p;     // Status Register

    // Internal state
    u8  opcode;        // Current opcode
    u8  cycles;        // Remaining cycles for current instruction
    u32 total_cycles;  // Total cycles executed
    u16 addr_abs;      // Absolute address for current instruction
    u16 addr_rel;      // Relative address for branch instructions
    u8  fetched;       // Fetched data for current instruction
    
    // Memory interface
    Memory* memory;
} CPU;

// Instruction function pointer type
typedef void (*InstructionFn)(CPU*);

// Instruction entry structure
typedef struct {
    char* name;           // Instruction name
    InstructionFn op;     // Operation function
    AddressingMode mode;  // Addressing mode
    u8 cycles;           // Base cycles
} Instruction;

// CPU Functions
void cpu_init(CPU* cpu, Memory* memory);
void cpu_reset(CPU* cpu);
void cpu_irq(CPU* cpu);   // Hardware interrupt
void cpu_nmi(CPU* cpu);   // Non-maskable interrupt
u8 cpu_step(CPU* cpu);    // Execute one instruction, return cycles taken

// CPU Status flag operations
void cpu_set_flag(CPU* cpu, u8 flag, bool value);
bool cpu_get_flag(CPU* cpu, u8 flag);

// Memory access
u8 cpu_read(CPU* cpu, u16 addr);
void cpu_write(CPU* cpu, u16 addr, u8 data);
u8 cpu_fetch(CPU* cpu);

// Stack operations
void cpu_push(CPU* cpu, u8 data);
u8 cpu_pop(CPU* cpu);
void cpu_push16(CPU* cpu, u16 data);
u16 cpu_pop16(CPU* cpu);

// Addressing mode functions
u8 cpu_addr_imp(CPU* cpu);
u8 cpu_addr_acc(CPU* cpu);
u8 cpu_addr_imm(CPU* cpu);
u8 cpu_addr_zp0(CPU* cpu);
u8 cpu_addr_zpx(CPU* cpu);
u8 cpu_addr_zpy(CPU* cpu);
u8 cpu_addr_rel(CPU* cpu);
u8 cpu_addr_abs(CPU* cpu);
u8 cpu_addr_abx(CPU* cpu);
u8 cpu_addr_aby(CPU* cpu);
u8 cpu_addr_ind(CPU* cpu);
u8 cpu_addr_izx(CPU* cpu);
u8 cpu_addr_izy(CPU* cpu);

#endif // CPU_H 