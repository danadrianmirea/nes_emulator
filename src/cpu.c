#include <string.h>
#include "cpu.h"

// Forward declarations of all instructions
static void adc(CPU* cpu); // Add with Carry
static void and(CPU* cpu); // Logical AND
static void asl(CPU* cpu); // Arithmetic Shift Left
static void bcc(CPU* cpu); // Branch if Carry Clear
static void bcs(CPU* cpu); // Branch if Carry Set
static void beq(CPU* cpu); // Branch if Equal
static void bit(CPU* cpu); // Bit Test
static void bmi(CPU* cpu); // Branch if Minus
static void bne(CPU* cpu); // Branch if Not Equal
static void bpl(CPU* cpu); // Branch if Positive
static void brk(CPU* cpu); // Force Interrupt
static void bvc(CPU* cpu); // Branch if Overflow Clear
static void bvs(CPU* cpu); // Branch if Overflow Set
static void clc(CPU* cpu); // Clear Carry Flag
static void cld(CPU* cpu); // Clear Decimal Mode
static void cli(CPU* cpu); // Clear Interrupt Disable
static void clv(CPU* cpu); // Clear Overflow Flag
static void cmp(CPU* cpu); // Compare
static void cpx(CPU* cpu); // Compare X Register
static void cpy(CPU* cpu); // Compare Y Register
static void dec(CPU* cpu); // Decrement Memory
static void dex(CPU* cpu); // Decrement X Register
static void dey(CPU* cpu); // Decrement Y Register
static void eor(CPU* cpu); // Exclusive OR
static void inc(CPU* cpu); // Increment Memory
static void inx(CPU* cpu); // Increment X Register
static void iny(CPU* cpu); // Increment Y Register
static void jmp(CPU* cpu); // Jump
static void jsr(CPU* cpu); // Jump to Subroutine
static void lda(CPU* cpu); // Load Accumulator
static void ldx(CPU* cpu); // Load X Register
static void ldy(CPU* cpu); // Load Y Register
static void lsr(CPU* cpu); // Logical Shift Right
static void nop(CPU* cpu); // No Operation
static void ora(CPU* cpu); // Logical Inclusive OR
static void pha(CPU* cpu); // Push Accumulator
static void php(CPU* cpu); // Push Processor Status
static void pla(CPU* cpu); // Pull Accumulator
static void plp(CPU* cpu); // Pull Processor Status
static void rol(CPU* cpu); // Rotate Left
static void ror(CPU* cpu); // Rotate Right
static void rti(CPU* cpu); // Return from Interrupt
static void rts(CPU* cpu); // Return from Subroutine
static void sbc(CPU* cpu); // Subtract with Carry
static void sec(CPU* cpu); // Set Carry Flag
static void sed(CPU* cpu); // Set Decimal Flag
static void sei(CPU* cpu); // Set Interrupt Disable
static void sta(CPU* cpu); // Store Accumulator
static void stx(CPU* cpu); // Store X Register
static void sty(CPU* cpu); // Store Y Register
static void tax(CPU* cpu); // Transfer Accumulator to X
static void tay(CPU* cpu); // Transfer Accumulator to Y
static void tsx(CPU* cpu); // Transfer Stack Pointer to X
static void txa(CPU* cpu); // Transfer X to Accumulator
static void txs(CPU* cpu); // Transfer X to Stack Pointer
static void tya(CPU* cpu); // Transfer Y to Accumulator

// Instruction table
static const Instruction instructions[256] = {
    // TODO: Fill this with the complete instruction set
    // This will be a big table mapping opcodes to instructions
};

void cpu_init(CPU* cpu, Memory* memory) {
    if (!cpu || !memory) return;
    
    cpu->memory = memory;
    cpu_reset(cpu);
}

void cpu_reset(CPU* cpu) {
    if (!cpu) return;
    
    // Set initial register values
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->sp = 0xFD;
    cpu->p = FLAG_U | FLAG_I;  // Unused flag is always set, start with interrupts disabled
    
    // Read reset vector
    u16 lo = cpu_read(cpu, 0xFFFC);
    u16 hi = cpu_read(cpu, 0xFFFD);
    cpu->pc = (hi << 8) | lo;
    
    // Clear internal state
    cpu->addr_abs = 0;
    cpu->addr_rel = 0;
    cpu->fetched = 0;
    cpu->opcode = 0;
    cpu->cycles = 0;
    cpu->total_cycles = 0;
}

void cpu_irq(CPU* cpu) {
    // Handle hardware interrupt if interrupts are enabled
    if (!cpu_get_flag(cpu, FLAG_I)) {
        cpu_push16(cpu, cpu->pc);
        
        cpu_set_flag(cpu, FLAG_B, false);
        cpu_set_flag(cpu, FLAG_U, true);
        cpu_set_flag(cpu, FLAG_I, true);
        cpu_push(cpu, cpu->p);
        
        u16 lo = cpu_read(cpu, 0xFFFE);
        u16 hi = cpu_read(cpu, 0xFFFF);
        cpu->pc = (hi << 8) | lo;
        
        cpu->cycles = 7;
    }
}

void cpu_nmi(CPU* cpu) {
    // Handle non-maskable interrupt
    cpu_push16(cpu, cpu->pc);
    
    cpu_set_flag(cpu, FLAG_B, false);
    cpu_set_flag(cpu, FLAG_U, true);
    cpu_set_flag(cpu, FLAG_I, true);
    cpu_push(cpu, cpu->p);
    
    u16 lo = cpu_read(cpu, 0xFFFA);
    u16 hi = cpu_read(cpu, 0xFFFB);
    cpu->pc = (hi << 8) | lo;
    
    cpu->cycles = 8;
}

u8 cpu_step(CPU* cpu) {
    if (cpu->cycles == 0) {
        // Fetch new instruction
        cpu->opcode = cpu_read(cpu, cpu->pc++);
        
        // Get instruction info
        Instruction inst = instructions[cpu->opcode];
        
        // Set initial cycles
        cpu->cycles = inst.cycles;
        
        // Execute addressing mode and instruction
        u8 addr_cycles = 0;
        switch (inst.mode) {
            case IMP: addr_cycles = cpu_addr_imp(cpu); break;
            case ACC: addr_cycles = cpu_addr_acc(cpu); break;
            case IMM: addr_cycles = cpu_addr_imm(cpu); break;
            case ZP0: addr_cycles = cpu_addr_zp0(cpu); break;
            case ZPX: addr_cycles = cpu_addr_zpx(cpu); break;
            case ZPY: addr_cycles = cpu_addr_zpy(cpu); break;
            case REL: addr_cycles = cpu_addr_rel(cpu); break;
            case ABS: addr_cycles = cpu_addr_abs(cpu); break;
            case ABX: addr_cycles = cpu_addr_abx(cpu); break;
            case ABY: addr_cycles = cpu_addr_aby(cpu); break;
            case IND: addr_cycles = cpu_addr_ind(cpu); break;
            case IZX: addr_cycles = cpu_addr_izx(cpu); break;
            case IZY: addr_cycles = cpu_addr_izy(cpu); break;
        }
        
        // Execute instruction
        inst.op(cpu);
        
        // Add any additional cycles from addressing mode
        cpu->cycles += addr_cycles;
    }
    
    cpu->cycles--;
    cpu->total_cycles++;
    
    return cpu->cycles;
}

// CPU Status flag operations
void cpu_set_flag(CPU* cpu, u8 flag, bool value) {
    if (value)
        cpu->p |= flag;
    else
        cpu->p &= ~flag;
}

bool cpu_get_flag(CPU* cpu, u8 flag) {
    return (cpu->p & flag) != 0;
}

// Memory access
u8 cpu_read(CPU* cpu, u16 addr) {
    return memory_read(cpu->memory, addr);
}

void cpu_write(CPU* cpu, u16 addr, u8 data) {
    memory_write(cpu->memory, addr, data);
}

u8 cpu_fetch(CPU* cpu) {
    if (instructions[cpu->opcode].mode != ACC)
        cpu->fetched = cpu_read(cpu, cpu->addr_abs);
    return cpu->fetched;
}

// Stack operations
void cpu_push(CPU* cpu, u8 data) {
    cpu_write(cpu, 0x0100 + cpu->sp, data);
    cpu->sp--;
}

u8 cpu_pop(CPU* cpu) {
    cpu->sp++;
    return cpu_read(cpu, 0x0100 + cpu->sp);
}

void cpu_push16(CPU* cpu, u16 data) {
    cpu_push(cpu, (data >> 8) & 0xFF);
    cpu_push(cpu, data & 0xFF);
}

u16 cpu_pop16(CPU* cpu) {
    u16 lo = cpu_pop(cpu);
    u16 hi = cpu_pop(cpu);
    return (hi << 8) | lo;
}

// Addressing mode implementations
u8 cpu_addr_imp(CPU* cpu) {
    cpu->fetched = cpu->a;
    return 0;
}

u8 cpu_addr_acc(CPU* cpu) {
    cpu->fetched = cpu->a;
    return 0;
}

u8 cpu_addr_imm(CPU* cpu) {
    cpu->addr_abs = cpu->pc++;
    return 0;
}

u8 cpu_addr_zp0(CPU* cpu) {
    cpu->addr_abs = cpu_read(cpu, cpu->pc++);
    return 0;
}

u8 cpu_addr_zpx(CPU* cpu) {
    cpu->addr_abs = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
    return 0;
}

u8 cpu_addr_zpy(CPU* cpu) {
    cpu->addr_abs = (cpu_read(cpu, cpu->pc++) + cpu->y) & 0xFF;
    return 0;
}

u8 cpu_addr_rel(CPU* cpu) {
    cpu->addr_rel = cpu_read(cpu, cpu->pc++);
    if (cpu->addr_rel & 0x80)
        cpu->addr_rel |= 0xFF00;
    return 0;
}

u8 cpu_addr_abs(CPU* cpu) {
    u16 lo = cpu_read(cpu, cpu->pc++);
    u16 hi = cpu_read(cpu, cpu->pc++);
    cpu->addr_abs = (hi << 8) | lo;
    return 0;
}

u8 cpu_addr_abx(CPU* cpu) {
    u16 lo = cpu_read(cpu, cpu->pc++);
    u16 hi = cpu_read(cpu, cpu->pc++);
    cpu->addr_abs = ((hi << 8) | lo) + cpu->x;
    return (cpu->addr_abs & 0xFF00) != (hi << 8) ? 1 : 0;
}

u8 cpu_addr_aby(CPU* cpu) {
    u16 lo = cpu_read(cpu, cpu->pc++);
    u16 hi = cpu_read(cpu, cpu->pc++);
    cpu->addr_abs = ((hi << 8) | lo) + cpu->y;
    return (cpu->addr_abs & 0xFF00) != (hi << 8) ? 1 : 0;
}

u8 cpu_addr_ind(CPU* cpu) {
    u16 ptr_lo = cpu_read(cpu, cpu->pc++);
    u16 ptr_hi = cpu_read(cpu, cpu->pc++);
    u16 ptr = (ptr_hi << 8) | ptr_lo;
    
    // Simulate page boundary hardware bug
    if (ptr_lo == 0xFF)
        cpu->addr_abs = (cpu_read(cpu, (ptr & 0xFF00)) << 8) | cpu_read(cpu, ptr);
    else
        cpu->addr_abs = (cpu_read(cpu, ptr + 1) << 8) | cpu_read(cpu, ptr);
    
    return 0;
}

u8 cpu_addr_izx(CPU* cpu) {
    u16 t = cpu_read(cpu, cpu->pc++);
    u16 lo = cpu_read(cpu, (u16)((t + (u16)cpu->x) & 0x00FF));
    u16 hi = cpu_read(cpu, (u16)((t + (u16)cpu->x + 1) & 0x00FF));
    cpu->addr_abs = (hi << 8) | lo;
    return 0;
}

u8 cpu_addr_izy(CPU* cpu) {
    u16 t = cpu_read(cpu, cpu->pc++);
    u16 lo = cpu_read(cpu, t & 0x00FF);
    u16 hi = cpu_read(cpu, (t + 1) & 0x00FF);
    cpu->addr_abs = ((hi << 8) | lo) + cpu->y;
    return (cpu->addr_abs & 0xFF00) != (hi << 8) ? 1 : 0;
}

// Instruction implementations
// TODO: Implement all instruction functions
static void adc(CPU* cpu) { /* TODO */ }
static void and(CPU* cpu) { /* TODO */ }
static void asl(CPU* cpu) { /* TODO */ }
static void bcc(CPU* cpu) { /* TODO */ }
static void bcs(CPU* cpu) { /* TODO */ }
static void beq(CPU* cpu) { /* TODO */ }
static void bit(CPU* cpu) { /* TODO */ }
static void bmi(CPU* cpu) { /* TODO */ }
static void bne(CPU* cpu) { /* TODO */ }
static void bpl(CPU* cpu) { /* TODO */ }
static void brk(CPU* cpu) { /* TODO */ }
static void bvc(CPU* cpu) { /* TODO */ }
static void bvs(CPU* cpu) { /* TODO */ }
static void clc(CPU* cpu) { /* TODO */ }
static void cld(CPU* cpu) { /* TODO */ }
static void cli(CPU* cpu) { /* TODO */ }
static void clv(CPU* cpu) { /* TODO */ }
static void cmp(CPU* cpu) { /* TODO */ }
static void cpx(CPU* cpu) { /* TODO */ }
static void cpy(CPU* cpu) { /* TODO */ }
static void dec(CPU* cpu) { /* TODO */ }
static void dex(CPU* cpu) { /* TODO */ }
static void dey(CPU* cpu) { /* TODO */ }
static void eor(CPU* cpu) { /* TODO */ }
static void inc(CPU* cpu) { /* TODO */ }
static void inx(CPU* cpu) { /* TODO */ }
static void iny(CPU* cpu) { /* TODO */ }
static void jmp(CPU* cpu) { /* TODO */ }
static void jsr(CPU* cpu) { /* TODO */ }
static void lda(CPU* cpu) { /* TODO */ }
static void ldx(CPU* cpu) { /* TODO */ }
static void ldy(CPU* cpu) { /* TODO */ }
static void lsr(CPU* cpu) { /* TODO */ }
static void nop(CPU* cpu) { /* TODO */ }
static void ora(CPU* cpu) { /* TODO */ }
static void pha(CPU* cpu) { /* TODO */ }
static void php(CPU* cpu) { /* TODO */ }
static void pla(CPU* cpu) { /* TODO */ }
static void plp(CPU* cpu) { /* TODO */ }
static void rol(CPU* cpu) { /* TODO */ }
static void ror(CPU* cpu) { /* TODO */ }
static void rti(CPU* cpu) { /* TODO */ }
static void rts(CPU* cpu) { /* TODO */ }
static void sbc(CPU* cpu) { /* TODO */ }
static void sec(CPU* cpu) { /* TODO */ }
static void sed(CPU* cpu) { /* TODO */ }
static void sei(CPU* cpu) { /* TODO */ }
static void sta(CPU* cpu) { /* TODO */ }
static void stx(CPU* cpu) { /* TODO */ }
static void sty(CPU* cpu) { /* TODO */ }
static void tax(CPU* cpu) { /* TODO */ }
static void tay(CPU* cpu) { /* TODO */ }
static void tsx(CPU* cpu) { /* TODO */ }
static void txa(CPU* cpu) { /* TODO */ }
static void txs(CPU* cpu) { /* TODO */ }
static void tya(CPU* cpu) { /* TODO */ } 