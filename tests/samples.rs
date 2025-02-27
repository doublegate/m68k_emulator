// tests/samples.rs
use m68k_emulator::m68k_cpu::CPU;

#[test]
fn test_loop_program() {
    let mut cpu = CPU::new(1024);
    let program = [
        0x70, 0x05,       // MOVEQ #5, D0
        0x53, 0x40,       // SUBQ.W #1, D0
        0x66, 0xFC,       // BNE -4
        0x4E, 0x71,       // NOP
    ];
    cpu.load_program(0x1000, &program);
    for _ in 0..6 {
        cpu.step();
    }
    assert_eq!(cpu.d[0], 0);
    assert_eq!(cpu.pc, 0x100A);
    assert_eq!(cpu.cycle_count, 4 + 5 * (8 + 12) + 4); // MOVEQ + 5*(SUBQ+BNE) + NOP
}

#[test]
fn test_interrupt_sample() {
    let mut cpu = CPU::new(1024);
    let program = [
        0x4E, 0x71,       // NOP
        0x4E, 0x72, 0x27, 0x00, // STOP #0x2700
    ];
    cpu.memory.write_long(0x64, 0x2000); // Level 1 vector
    let interrupt_handler = [
        0x4E, 0x75,       // RTS
    ];
    cpu.load_program(0x1000, &program);
    cpu.load_program(0x2000, &interrupt_handler);
    cpu.step(); // NOP
    cpu.step(); // STOP
    assert!(cpu.halted);
    cpu.request_interrupt(1, None);
    cpu.step(); // Interrupt
    assert_eq!(cpu.pc, 0x1006); // After STOP
    assert!(!cpu.halted);
}
