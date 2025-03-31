// src/main.rs - 68K Emulator Entry Point
// --------------------------------------------------------------------
// This file demonstrates the initialization and use of a 68K emulator.
// It sets up memory (dividing it into ROM and RAM regions), creates a CPU
// instance, loads a sample program into ROM, initializes CPU registers, and then
// executes a series of instructions (NOP, MOVE.B, RTS). Additionally, if run with
// the "--run-tests" flag, this executable will invoke the integration test suite.
// --------------------------------------------------------------------

use m68k_emulator::m68k_cpu::CPU;   // Import the CPU module to simulate the Motorola 68K CPU.
use m68k_emulator::memory::Memory;   // Import the Memory module for handling memory access.
use std::env;
use std::process::Command;

fn main() {
    // ----------------------------------------------------------------
    // Test Runner Option:
    // If the command-line argument "--run-tests" is supplied, spawn the "cargo test"
    // command to run all tests from the tests/ directory. This approach leverages Rust’s
    // standard testing framework while keeping the production code separate.
    // ----------------------------------------------------------------
    if env::args().any(|arg| arg == "--run-tests") {
        println!("Running tests via cargo test ...");
        let status = Command::new("cargo")
            .arg("test")
            .status()
            .expect("Failed to run cargo test");
        std::process::exit(status.code().unwrap_or(1));
    }

    // ----------------------------------------------------------------
    // Memory Initialization:
    // Create a memory instance with 16 MB backing storage. The Memory object divides the
    // address space into two regions:
    //   - ROM: Addresses 0x000000 to 0x3FFFFF (read-only during execution).
    //   - RAM: Addresses 0xFF0000 to 0xFFFFFF (writable).
    // Although 16 MB is allocated, internal logic in Memory will correctly route accesses.
    // ----------------------------------------------------------------
    let memory = Memory::new(vec![0; 16 * 1024 * 1024]); // 16 MB of zero-initialized memory.

    // ----------------------------------------------------------------
    // CPU Initialization:
    // Create a new CPU instance with the initialized memory. The CPU structure maintains
    // all processor state such as data and address registers, the program counter (PC),
    // the status register (SR), and various flags.
    // ----------------------------------------------------------------
    let mut cpu = CPU::new(memory);

    // ----------------------------------------------------------------
    // Program Definition:
    // Define an example program consisting of three instructions:
    //   - 0x4E71 (NOP): No-Operation. Advances the PC and consumes CPU cycles.
    //   - 0x1080 (MOVE.B D0, (A0)): Transfers the lower 8 bits from data register D0
    //         into the memory location pointed to by address register A0.
    //   - 0x4E75 (RTS): Return from Subroutine; pops a return address from the stack.
    // ----------------------------------------------------------------
    let program = [
        0x4E, 0x71, // NOP: Advances the PC with no state change.
        0x10, 0x80, // MOVE.B D0, (A0): Move low-order byte of D0 to the memory address in A0.
        0x4E, 0x75, // RTS: Simulate end-of-program by returning from a subroutine.
    ];

    // ----------------------------------------------------------------
    // Program Loading:
    // Load the above-defined program into the ROM region starting at address 0x1000.
    // The CPU’s load_program method copies the program bytes into ROM and then performs
    // a prefetch of two words, advancing the PC by 4.
    // ----------------------------------------------------------------
    cpu.load_program(0x1000, &program);

    // ----------------------------------------------------------------
    // CPU Register Setup:
    // Before execution, set up test register values:
    //   - D0 is set to 0x42 to serve as the source value for the MOVE.B instruction.
    //   - A0 is set to 0xFF0000 so that the destination memory falls within the writable RAM.
    // ----------------------------------------------------------------
    cpu.d[0] = 0x42;
    cpu.a[0] = 0xFF0000;

    // ----------------------------------------------------------------
    // Initial CPU State:
    // Print the initial program counter (PC) to confirm proper initialization.
    // ----------------------------------------------------------------
    println!("Initial PC: {:06X}", cpu.pc);

    // ----------------------------------------------------------------
    // Instruction Execution: NOP
    // Execute the NOP instruction. This simulates one fetch–decode–execute cycle.
    // The PC is advanced and a fixed number of cycles (4) are consumed.
    // ----------------------------------------------------------------
    let cycles = cpu.step();
    println!("After NOP: PC={:06X}, Cycles={}", cpu.pc, cycles);

    // ----------------------------------------------------------------
    // Instruction Execution: MOVE.B D0, (A0)
    // Execute the MOVE.B instruction, transferring the byte from D0 into the RAM address A0.
    // The PC is updated and additional cycles are consumed to handle effective addressing.
    // ----------------------------------------------------------------
    let cycles = cpu.step();
    println!("After MOVE.B: PC={:06X}, Cycles={}", cpu.pc, cycles);

    // ----------------------------------------------------------------
    // Memory Verification:
    // Verify that the MOVE.B instruction correctly stored the value by reading the byte
    // from the RAM location pointed to by A0.
    // ----------------------------------------------------------------
    let memory_value = cpu.memory.read_byte(cpu.a[0]).unwrap_or(0);
    println!("Memory[A0]={:02X}", memory_value);

    // ----------------------------------------------------------------
    // Instruction Execution: RTS
    // Execute the RTS instruction to simulate a subroutine return.
    // The PC is updated to the popped return address and the corresponding cycles are consumed.
    // ----------------------------------------------------------------
    let cycles = cpu.step();
    println!("After RTS: PC={:06X}, Cycles={}", cpu.pc, cycles);
}
