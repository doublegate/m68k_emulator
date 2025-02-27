// tests/cpu_tests.rs
use m68k_emulator::m68k_cpu::{CPU, Instruction, Operation, Operand, Size};

#[test]
fn test_nop() {
    let mut cpu = CPU::new(1024);
    cpu.load_program(0x1000, &[0x4E, 0x71]); // NOP
    let cycles = cpu.step();
    assert_eq!(cpu.pc, 0x1004); // PC advances 4 (prefetch)
    assert_eq!(cycles, 4);
}

#[test]
fn test_move() {
    let mut cpu = CPU::new(1024);
    cpu.d[0] = 0x42;
    cpu.a[1] = 0x2000;
    cpu.load_program(0x1000, &[0x12, 0x40]); // MOVE.B D0, (A1)
    let cycles = cpu.step();
    assert_eq!(cpu.pc, 0x1004);
    assert_eq!(cpu.memory.read_byte(0x2000), 0x42);
    assert_eq!(cycles, 8); // 4 base + 4 EA
    assert_eq!(cpu.sr & 0xF, 0); // N=0, Z=0
}

#[test]
fn test_add() {
    let mut cpu = CPU::new(1024);
    cpu.d[0] = 0x5;
    cpu.d[1] = 0x3;
    cpu.load_program(0x1000, &[0xD2, 0x00]); // ADD.W D0, D1
    let cycles = cpu.step();
    assert_eq!(cpu.d[1], 0x8);
    assert_eq!(cycles, 4);
    assert_eq!(cpu.sr & 0xF, 0); // No flags set
}

#[test]
fn test_divu_zero_divide() {
    let mut cpu = CPU::new(1024);
    cpu.d[0] = 0x10;
    cpu.d[1] = 0;
    cpu.memory.write_long(0x14, 0x3000); // Zero Divide vector
    cpu.load_program(0x1000, &[0x80, 0xC1]); // DIVU.W D1, D0
    let cycles = cpu.step();
    assert_eq!(cpu.pc, 0x3000);
    assert_eq!(cycles, 34);
}

#[test]
fn test_stop_and_interrupt() {
    let mut cpu = CPU::new(1024);
    cpu.memory.write_long(0x64, 0x3000); // Level 1 vector
    cpu.load_program(0x1000, &[0x4E, 0x72, 0x27, 0x00]); // STOP #0x2700
    let cycles = cpu.step();
    assert!(cpu.halted);
    assert_eq!(cycles, 4);
    let cycles = cpu.step();
    assert_eq!(cycles, 0); // Halted, no interrupt
    cpu.request_interrupt(1, None);
    let cycles = cpu.step();
    assert!(!cpu.halted);
    assert_eq!(cpu.pc, 0x3000);
    assert_eq!(cycles, 34);
}

// Add more tests for all 90 instructions, exceptions, interrupts, etc.
// tests/cpu_tests.rs
