/*
    tests/cpu_tests.rs
    --------------------------------------------------------------------
    This module provides integration tests for the core CPU instruction set
    of the 68K emulator. It includes tests for instructions such as NOP,
    MOVE, ADD, DIVU (with divide-by-zero exception handling), and STOP
    combined with interrupt handling.
    
    Each test function simulates a small portion of the emulator's functionality,
    ensuring that the CPU state (registers, flags, program counter, cycle count)
    is correctly updated after execution.
    
    These functions have been refactored to be callable from a public test runner,
    allowing integration testing both from within Cargo’s test harness and via an
    external invocation.
    
    Detailed technical comments are provided below to explain the rationale behind
    each test case, including expected outcomes, cycle counts, and state transitions.
    --------------------------------------------------------------------
*/

use m68k_emulator::m68k_cpu::CPU;
use m68k_emulator::memory::Memory;

/// Test the NOP instruction.
///
/// This test verifies that the NOP instruction (opcode 0x4E71) properly advances
/// the program counter (PC) and consumes the expected number of CPU cycles. Specifically:
/// - When load_program is called with a starting address (e.g. 0x1000), prefetch() sets PC to (address + 4).
/// - Then, executing a NOP (via a fetch_word call that increments PC by 2) results in a final PC of 0x1006.
/// - The NOP instruction should consume 4 cycles.
pub fn test_nop() {
    let mut cpu = CPU::new(Memory::new(vec![0; 0x400000])); // 4 MB ROM
    cpu.load_program(0x1000, &[0x4E, 0x71]); // Load NOP at address 0x1000
    let cycles = cpu.step();
    // After load_program, PC is 0x1004; executing NOP (which does a fetch_word) advances PC by 2,
    // so the expected PC is 0x1006.
    assert_eq!(cpu.pc, 0x1006, "NOP: PC did not advance as expected.");
    // NOP should consume exactly 4 cycles.
    assert_eq!(cycles, 4, "NOP: Cycle count mismatch.");
}

/// Test the MOVE instruction.
///
/// This test verifies that the MOVE.B instruction transfers the lower 8 bits from
/// data register D0 into the memory location pointed to by address register A1. The test
/// sets D0 to 0x42 and A1 to a writable RAM address (0xFF0000), then loads a MOVE.B
/// instruction (opcode 0x1240). After executing this instruction:
/// - The PC is expected to advance from 0x1004 (after prefetch) to 0x1006.
/// - The memory at address 0xFF0000 should now contain the value 0x42.
/// - The instruction is expected to consume 8 cycles, and no CPU flags should be modified.
pub fn test_move() {
    let mut cpu = CPU::new(Memory::new(vec![0; 0x400000]));
    cpu.d[0] = 0x42;
    // Set A1 to a writable RAM address (0xFF0000).
    cpu.a[1] = 0xFF0000;
    // Opcode 0x1240 encodes MOVE.B D0, (A1)
    cpu.load_program(0x1000, &[0x12, 0x40]);
    let cycles = cpu.step();
    // Expected PC: starting at 0x1000 -> prefetch sets PC=0x1004, then executing the instruction adds 2, so 0x1006.
    assert_eq!(cpu.pc, 0x1006, "MOVE: PC did not advance as expected.");
    let mem_val = cpu.memory.read_byte(0xFF0000).expect("Memory read failed in MOVE test");
    assert_eq!(mem_val, 0x42, "MOVE: Memory value mismatch.");
    assert_eq!(cycles, 8, "MOVE: Cycle count mismatch.");
    // Ensure that the CPU flags remain unchanged.
    assert_eq!(cpu.sr & 0xF, 0, "MOVE: Status flags not cleared.");
}

/// Test the ADD instruction.
///
/// This test verifies that the ADD.W instruction correctly adds the word value from D0 to D1.
/// It sets D0 = 0x5 and D1 = 0x3, and after execution, D1 should equal 0x8. Additionally,
/// the test confirms that the cycle count is 4 and that no CPU flags are modified.
pub fn test_add() {
    let mut cpu = CPU::new(Memory::new(vec![0; 0x400000]));
    cpu.d[0] = 0x5;
    cpu.d[1] = 0x3;
    // Opcode 0xD200 should now be decoded as ADD.W D0, D1.
    cpu.load_program(0x1000, &[0xD2, 0x00]);
    let cycles = cpu.step();
    // Validate that D1 holds the correct sum (3+5=8).
    assert_eq!(cpu.d[1], 0x8, "ADD: Incorrect addition result.");
    assert_eq!(cycles, 4, "ADD: Cycle count mismatch.");
    assert_eq!(cpu.sr & 0xF, 0, "ADD: Unexpected status flags.");
}

/// Test the DIVU instruction when dividing by zero.
///
/// This test ensures that a DIVU instruction that attempts to divide by zero triggers
/// the correct exception handling. It sets D0 (dividend) to 0x10 and D1 (divisor) to 0,
/// prepares the exception vector in ROM (for vector 5, at offset 0x14) to point to 0x3000,
/// and loads the DIVU instruction (opcode 0x80C1). After execution:
/// - The PC should be loaded from the exception vector (0x3000).
/// - The cycle count should reflect the exception handling overhead (34 cycles).
pub fn test_divu_zero_divide() {
    let mut rom_data = vec![0; 0x400000];
    rom_data[0x0] = 0x00; // Stack pointer at 0x0
    rom_data[0x1] = 0xFF;
    rom_data[0x2] = 0xFF;
    rom_data[0x3] = 0xFE; // 0xFFFFFE
    rom_data[0x14] = 0x00; // Vector 5 at 0x14
    rom_data[0x15] = 0x00;
    rom_data[0x16] = 0x30; // 0x3000
    rom_data[0x17] = 0x00;
    let mut cpu = CPU::new(Memory::new(rom_data));
    cpu.a[7] = cpu.memory.read_long(0).unwrap_or(0); // A7 = 0xFFFFFE
    cpu.d[0] = 0x10; // Dividend
    cpu.d[1] = 0;    // Divisor (zero)
    cpu.load_program(0x1000, &[0x80, 0xC1]); // DIVU.W D1, D0
    let cycles = cpu.step();
    assert_eq!(cpu.pc, 0x3000, "DIVU: PC not set to exception vector on divide by zero.");
    assert_eq!(cycles, 34, "DIVU: Cycle count mismatch on exception.");
}

/// Test the STOP instruction and subsequent interrupt handling.
///
/// The following tests validate the behavior of the STOP instruction in both
/// supervisor and user modes:
/// - In supervisor mode, executing STOP should halt the CPU, consuming the
///   expected cycles and preventing any further execution steps.
/// - In user mode, executing STOP is expected to trigger a privilege violation
///   exception, causing the CPU to load the PC with the handler address from the
///   exception vector.
pub fn test_stop_supervisor() {
    // Create memory (4MB) and preload reset vector and stack pointer.
    let mut memory = Memory::new(vec![0; 0x400000]);
    // Set reset vector at address 4 to 0x1000 (big-endian: 00 00 10 00)
    memory
        .load_rom_data(4, &[0x00, 0x00, 0x10, 0x00])
        .expect("Failed to load reset vector");
    // Set initial stack pointer at address 0 to 0x00400000 (big-endian: 00 40 00 00)
    memory
        .load_rom_data(0, &[0x00, 0x40, 0x00, 0x00])
        .expect("Failed to load stack pointer");

    let mut cpu = CPU::new(memory);
    // Set PC and A7 from the reset vector.
    cpu.pc = cpu.memory.read_long(4).unwrap_or(0);
    cpu.a[7] = cpu.memory.read_long(0).unwrap_or(0);

    // Force supervisor mode.
    cpu.sr |= 0x2000;
    // Load a STOP instruction at 0x1000: opcode 0x4E72 with immediate word 0x2700.
    cpu.load_program(0x1000, &[0x4E, 0x72, 0x27, 0x00]);
    // Perform prefetch so that the PC is advanced.
    // cpu.prefetch();
    
    let cycles = cpu.step();
    assert!(cpu.halted, "STOP in supervisor mode did not halt the CPU.");
    assert_eq!(cycles, 4, "Cycle count mismatch on supervisor STOP.");
    // Further steps should do nothing when halted.
    let cycles = cpu.step();
    assert_eq!(cycles, 0, "CPU executed steps while halted in supervisor mode.");
}

// Test STOP in user mode (expected to trigger privilege violation exception)
pub fn test_stop_user_mode() {
    let mut cpu = CPU::new(Memory::new(vec![0; 0x400000]));
    // Force user mode by clearing the supervisor bit.
    cpu.sr &= !0x2000;
    // Load the privilege violation vector (vector 8) with a handler address (e.g. 0x4000).
    let vec_bytes = [0x00, 0x00, 0x40, 0x00]; // Big-endian for 0x4000.
    cpu.memory.load_rom_data(8 * 4, &vec_bytes)
        .expect("Failed to load privilege violation vector");
    // Load the STOP instruction.
    cpu.load_program(0x1000, &[0x4E, 0x72, 0x27, 0x00]);
    let _cycles = cpu.step();
    // In user mode, STOP should trigger the privilege violation exception.
    assert!(!cpu.halted, "STOP in user mode should not halt the CPU.");
    assert_eq!(cpu.pc, 0x4000, "STOP in user mode did not set PC from privilege violation vector.");
}

/// Public function to run all CPU tests.
///
/// This function aggregates all individual test cases for the CPU instruction set into a single
/// test runner. It prints progress messages to standard output and is designed to be called from
/// the Cargo test harness via a #[test] wrapper.
pub fn run_cpu_tests() {
    println!("Running CPU tests...");
    test_nop();
    test_move();
    test_add();
    test_divu_zero_divide();
    test_stop_supervisor();
    test_stop_user_mode();
    println!("All CPU tests passed.");
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn run_cpu_tests_wrapper() {
        run_cpu_tests();
    }
}
// tests/cpu_tests.rs