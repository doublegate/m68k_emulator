/*
    tests/samples.rs
    --------------------------------------------------------------------
    This module provides sample programs to test integrated CPU behavior,
    including loop constructs and interrupt handling, in the 68K emulator.
    
    The loop test simulates a decrementing loop that continues until a register
    value reaches zero. It verifies the proper functioning of instructions such as
    MOVEQ, SUBQ, BNE, and NOP, and checks that the program counter (PC) and cycle
    count are updated as expected.
    
    The interrupt sample test demonstrates how the CPU halts on executing a STOP
    instruction and then resumes execution when an interrupt is requested. It verifies
    that the CPU loads the interrupt handler address from the correct vector and clears
    the halted state.
    
    Detailed technical commentary is provided for each test to explain the logic,
    expected behavior, and timing considerations of the emulator.
    --------------------------------------------------------------------
*/

use m68k_emulator::m68k_cpu::CPU;
use m68k_emulator::memory::Memory;

/// Test a loop program that decrements a register until zero.
///
/// The test program performs the following operations:
///   1. MOVEQ #5, D0: Initializes data register D0 with the immediate value 5.
///   2. SUBQ.W #1, D0: Decrements D0 by 1.
///   3. BNE -4: Branches back by 4 bytes if D0 is not zero, forming a loop.
///   4. NOP: Executes a no-operation instruction after the loop completes.
/// 
/// Due to the prefetch behavior, after load_program (with a starting address of 0x1000),
/// the PC is set to 0x1004. Each fetch_word call in execution advances the PC by 2 bytes.
/// Consequently, after executing the first instruction (MOVEQ), PC becomes 0x1006.
/// The test then loops until D0 reaches zero and verifies that:
///   - The final value of D0 is 0.
///   - The PC and cumulative cycle count match the expected values.
pub fn test_loop_program() {
    // Initialize memory with enough space
    let mut memory = Memory::new(vec![0; 0x400000]);
    
    // Set reset vector (PC) and stack pointer
    memory.load_rom_data(4, &[0x00, 0x00, 0x10, 0x00]).expect("Failed to load reset vector"); // PC = 0x1000
    memory.load_rom_data(0, &[0x00, 0xFF, 0xFF, 0xFC]).expect("Failed to load stack pointer"); // A7 = 0xFFFFFC
    
    // Set all exception vectors to point to the handler
    for i in 2..64 {
        memory.load_rom_data(i * 4, &[0x00, 0x00, 0x20, 0x00]).expect("Failed to load exception vector");
    }
    
    // Simple exception handler: NOP; RTS
    memory.load_rom_data(0x2000, &[0x4E, 0x71, 0x4E, 0x75]).expect("Failed to load exception handler");

    // Create CPU instance
    let mut cpu = CPU::new(memory);
    cpu.pc = cpu.memory.read_long(4).unwrap_or(0); // PC = 0x1000
    cpu.a[7] = cpu.memory.read_long(0).unwrap_or(0); // A7 = 0xFFFFFC
    cpu.sr |= 0x2000; // Supervisor mode
    cpu.halted = false;

    // The program to test
    let program = [
        0x70, 0x05, // MOVEQ #5, D0
        0x53, 0x40, // SUBQ.W #1, D0
        0x66, 0xFC, // BNE -4
        0x4E, 0x71, // NOP
        0x4E, 0x71, // NOP (padding)
        0x4E, 0x71, // NOP (padding)
        0x4E, 0x71, // NOP (padding)
    ];
    cpu.load_program(0x1000, &program);

    // Initialize stack frame for exceptions
    cpu.a[7] -= 2; // Space for SR
    cpu.memory.write_word(cpu.a[7], cpu.sr).expect("Failed to write SR");
    cpu.a[7] -= 4; // Space for return PC (point to SUBQ.W)
    cpu.memory.write_long(cpu.a[7], 0x1002).expect("Failed to write return PC");

    // Run the test
    let mut total_cycles = 0;
    let mut steps = 0;
    println!("Initial PC: {:06X}, D0: {}, A7: {:08X}, Halted: {}", cpu.pc, cpu.d[0], cpu.a[7], cpu.halted);
    total_cycles += cpu.step(); // Execute MOVEQ
    println!("Post-MOVEQ PC: {:06X}, D0: {}, Halted: {}", cpu.pc, cpu.d[0], cpu.halted);
    
    while cpu.d[0] != 0 && steps < 20 {
        println!("Entering step {} with PC: {:06X}, D0: {}", steps + 1, cpu.pc, cpu.d[0]);
        let cycles = cpu.step();
        total_cycles += cycles;
        steps += 1;
        println!("Step {} PC: {:06X}, D0: {}, Cycles: {}, Halted: {}", steps, cpu.pc, cpu.d[0], cycles, cpu.halted);
        if cycles == 0 {
            println!("Step {} exited early with zero cycles!", steps);
            break;
        }
    }
    total_cycles += cpu.step(); // Execute final NOP
    println!("Final PC: {:06X}, D0: {}", cpu.pc, cpu.d[0]);

    // Assertions
    assert_eq!(cpu.d[0], 0, "Loop Program: D0 did not decrement to 0 as expected.");
    assert_eq!(cpu.pc, 0x100C, "Loop Program: PC did not reach end of NOP as expected.");
    assert_eq!(total_cycles, 106, "Loop Program: Total cycle count mismatch.");
}

/// Test interrupt handling in a sample program.
///
/// This test demonstrates how the CPU handles an interrupt after executing a STOP instruction.
/// The test program consists of:
///   1. A NOP instruction, followed by a STOP instruction (opcode 0x4E72) that halts the CPU.
///   2. An external interrupt is simulated by preloading the level–1 interrupt vector in ROM at
///      offset 0x64 (which corresponds to vector 25) to point to an interrupt handler located at 0x3000.
///   3. An interrupt request is issued, causing the CPU to resume from the halted state.
///   4. The interrupt handler, which consists solely of an RTS instruction, is executed and the PC
///      is loaded from the interrupt vector.
/// 
/// This test verifies that the CPU resumes from the halted state, that the PC is correctly updated to
/// the interrupt handler’s address, and that the halted flag is cleared.
pub fn test_interrupt_sample() {
    let mut cpu = CPU::new(Memory::new(vec![0; 0x400000]));
    // Preload the level–1 interrupt vector in ROM using load_rom_data.
    let vec_bytes = [0x00, 0x00, 0x30, 0x00]; // Big-endian representation of 0x3000.
    cpu.memory.load_rom_data(0x64, &vec_bytes).expect("Failed to write interrupt vector in sample test");
    // Define the main program: a NOP followed by a STOP instruction.
    let program = [
        0x4E, 0x71,                   // NOP: Advance the PC.
        0x4E, 0x72, 0x27, 0x00,         // STOP #0x2700: CPU should halt.
    ];
    // Load the main program into ROM at address 0x1000.
    cpu.load_program(0x1000, &program);
    // Also load the interrupt handler into ROM at address 0x3000.
    let interrupt_handler = [
        0x4E, 0x75,       // RTS: Return from subroutine.
    ];
    cpu.load_program(0x3000, &interrupt_handler);
    // Execute the NOP.
    cpu.step();
    // Execute the STOP instruction; the CPU should now be halted.
    cpu.step();
    assert!(cpu.halted, "Interrupt Sample: CPU did not halt after STOP instruction.");
    // Issue an interrupt request at level 1.
    cpu.request_interrupt(1, None);
    cpu.step(); // Process the interrupt.
    // Verify that the CPU resumes and that the PC is updated to the interrupt handler address.
    assert_eq!(cpu.pc, 0x3000, "Interrupt Sample: PC not loaded from interrupt vector after resume.");
    assert!(!cpu.halted, "Interrupt Sample: CPU remains halted after handling interrupt.");
}

// Sample test for STOP in supervisor mode.
pub fn test_stop_sample_supervisor() {
    // Create memory (4MB) and preload reset vector and stack pointer.
    let mut memory = Memory::new(vec![0; 0x400000]);
    memory
        .load_rom_data(4, &[0x00, 0x00, 0x10, 0x00])
        .expect("Failed to load reset vector");
    memory
        .load_rom_data(0, &[0x00, 0x40, 0x00, 0x00])
        .expect("Failed to load stack pointer");

    let mut cpu = CPU::new(memory);
    // Set PC and A7 from the reset vector.
    cpu.pc = cpu.memory.read_long(4).unwrap_or(0);
    cpu.a[7] = cpu.memory.read_long(0).unwrap_or(0);

    cpu.sr |= 0x2000; // Ensure supervisor mode.
    // Program: NOP followed by STOP.
    let program = [
        0x4E, 0x71,                   // NOP
        0x4E, 0x72, 0x27, 0x00,         // STOP #0x2700
    ];
    cpu.load_program(0x1000, &program);
    cpu.prefetch(); // Advance PC based on the reset vector.
    
    cpu.step(); // Execute NOP.
    cpu.step(); // Execute STOP.
    assert!(cpu.halted, "Supervisor sample: CPU did not halt after STOP.");
}

// Sample test for STOP in user mode.
pub fn test_stop_sample_user() {
    let mut cpu = CPU::new(Memory::new(vec![0; 0x400000]));
    cpu.sr &= !0x2000; // Force user mode.
    // Load the privilege violation vector for vector 8 to address 0x4000.
    let vec_bytes = [0x00, 0x00, 0x40, 0x00];
    cpu.memory.load_rom_data(8 * 4, &vec_bytes)
        .expect("Failed to load privilege violation vector in sample test");
    // Program: STOP instruction only.
    cpu.load_program(0x1000, &[0x4E, 0x72, 0x27, 0x00]);
    cpu.step(); // Execute STOP, triggering privilege violation.
    assert_eq!(cpu.pc, 0x4000, "User sample: STOP did not trigger proper exception.");
    assert!(!cpu.halted, "User sample: CPU should not be halted after STOP in user mode.");
}

/// Public function to run all sample tests.
///
/// This function aggregates the loop and interrupt handling tests into a single test runner.
/// It outputs progress information and is designed to be invoked by Cargo’s test harness.
pub fn run_sample_tests() {
    println!("Running sample program tests...");
    test_loop_program();
    test_interrupt_sample();
    test_stop_sample_supervisor();
    test_stop_sample_user();
    println!("All sample tests passed.");
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn run_sample_tests_wrapper() {
        run_sample_tests();
    }
}
// tests/samples.rs