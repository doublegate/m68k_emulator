// src/main.rs
use m68k_emulator::m68k_cpu::CPU;

// Main function for demonstration
fn main() {
    let mut cpu = CPU::new(16 * 1024 * 1024); // 16 MB memory
    // Example program: NOP, MOVE.B D0, (A0), RTS
    let program = [
        0x4E, 0x71, // NOP
        0x10, 0x80, // MOVE.B D0, (A0)
        0x4E, 0x75, // RTS
    ];
    cpu.load_program(0x1000, &program);
    cpu.d[0] = 0x42; // D0 = 0x42
    cpu.a[0] = 0x2000; // A0 = 0x2000

    // Set up interrupt vectors
    cpu.memory.write_long(0x64, 0x3000); // Vector 25 (level 1 auto-vectored)
    cpu.memory.write_long(0x68, 0x4000); // Vector 26 (level 2 auto-vectored)
    cpu.memory.write_long(0x100, 0x5000); // Vector 64 (vectored, level 3)
    cpu.memory.write_long(0x104, 0x6000); // Vector 65 (vectored, level 7)

    println!("Initial PC: {:06X}", cpu.pc);
    let cycles = cpu.step();
    println!("After NOP: PC={:06X}, Cycles={}", cpu.pc, cycles);

    // Request an auto-vectored interrupt (level 1)
    cpu.request_interrupt(1, None);
    let cycles = cpu.step();
    println!("After Level 1 Auto-Vectored: PC={:06X}, Cycles={}", cpu.pc, cycles);

    // Request a vectored interrupt (level 3, vector 64) during level 1 ISR
    cpu.request_interrupt(3, Some(64));
    let cycles = cpu.step();
    println!("Nested Level 3 Vectored: PC={:06X}, Cycles={}", cpu.pc, cycles);

    // Request a level 7 non-maskable interrupt during level 3 ISR
    cpu.request_interrupt(7, Some(65));
    let cycles = cpu.step();
    println!("Nested Level 7 Vectored: PC={:06X}, Cycles={}", cpu.pc, cycles);

    let cycles = cpu.step();
    println!("After MOVE.B: PC={:06X}, Cycles={}, Memory[2000]={:02X}", cpu.pc, cycles, cpu.memory.read_byte(0x2000));
    let cycles = cpu.step();
    println!("After RTS: PC={:06X}, Cycles={}", cpu.pc, cycles);
}
