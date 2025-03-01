// src/main.rs - 68K Emulator Entry Point
// --------------------------------------------------------------------
// This file demonstrates the initialization and use of a 68K emulator.
// It sets up the memory (dividing it into ROM and RAM regions), creates a CPU
// instance, loads a sample program into ROM, initializes CPU registers, and then
// executes a series of instructions (NOP, MOVE.B, RTS). The execution state is
// printed step-by-step to illustrate the emulation process.
// --------------------------------------------------------------------

use m68k_emulator::m68k_cpu::CPU; // Import the CPU module to simulate the Motorola 68K CPU.
use m68k_emulator::memory::Memory; // Import the Memory module for handling memory access.

fn main() {
    // ----------------------------------------------------------------
    // Memory Initialization:
    // Create a memory instance with 16 MB backing storage.
    // In this emulator, the memory is conceptually divided into two regions:
    //   1. ROM: The lower 4 MB (addresses 0x000000 to 0x3FFFFF) is considered read-only.
    //   2. RAM: The upper region (addresses 0xFF0000 to 0xFFFFFF) is used as writable RAM.
    // Although the Memory structure receives a full 16 MB vector, internal addressing
    // mechanisms will ensure correct read/write operations on each region.
    // ----------------------------------------------------------------
    let mut memory = Memory::new(vec![0; 16 * 1024 * 1024]); // 16 MB of memory initialized to zeros.

    // ----------------------------------------------------------------
    // CPU Initialization:
    // Create a new CPU instance and pass the memory instance to it.
    // The CPU structure maintains state such as data and address registers,
    // the program counter (PC), the status register (SR), and other flags necessary
    // for the execution of 68K instructions.
    // ----------------------------------------------------------------
    let mut cpu = CPU::new(memory);

    // ----------------------------------------------------------------
    // Program Definition:
    // Define an example program consisting of three instructions:
    //   - 0x4E71 (NOP): No-Operation. This instruction advances the PC and
    //                    consumes CPU cycles without changing state.
    //   - 0x1080 (MOVE.B D0, (A0)): Moves the lower 8 bits from data register D0
    //                    into the memory location pointed to by address register A0.
    //   - 0x4E75 (RTS): Return from Subroutine, which pops a return address
    //                    from the stack and transfers control accordingly.
    // ----------------------------------------------------------------
    let program = [
        0x4E, 0x71, // NOP: Does nothing, just moves the PC forward.
        0x10, 0x80, // MOVE.B D0, (A0): Move the byte in D0 to the memory address in A0.
        0x4E, 0x75, // RTS: Return from Subroutine; simulates end-of-program.
    ];

    // ----------------------------------------------------------------
    // Program Loading:
    // Load the above-defined program into the ROM region of memory. The starting
    // address chosen is 0x1000, which is safely within the ROM space as it remains below 0x400000.
    // The CPU's load_program function copies the program bytes into the appropriate memory segment.
    // ----------------------------------------------------------------
    cpu.load_program(0x1000, &program);

    // ----------------------------------------------------------------
    // CPU Register Setup:
    // Initialize CPU registers with test values before execution:
    //   - Set the data register D0 to 0x42. This is the source value for the MOVE.B instruction.
    //   - Set the address register A0 to 0xFF0000. This address falls within the writable RAM region.
    //     Internally, memory writes for RAM mask addresses with 0xFFFF, meaning that an address
    //     of 0xFF0000 targets index 0 of the RAM array.
    // ----------------------------------------------------------------
    cpu.d[0] = 0x42;
    cpu.a[0] = 0xFF0000;

    // ----------------------------------------------------------------
    // Initial CPU State:
    // Print the initial program counter (PC) value. The PC indicates where in memory
    // the CPU will fetch its next instruction.
    // ----------------------------------------------------------------
    println!("Initial PC: {:06X}", cpu.pc);

    // ----------------------------------------------------------------
    // Instruction Execution: NOP
    // Execute the NOP instruction. This step simulates the fetch-decode-execute cycle.
    // The PC is advanced and a specific number of cycles are consumed.
    // ----------------------------------------------------------------
    let cycles = cpu.step();
    println!("After NOP: PC={:06X}, Cycles={}", cpu.pc, cycles);

    // ----------------------------------------------------------------
    // Instruction Execution: MOVE.B D0, (A0)
    // Execute the MOVE.B instruction which transfers the low-order byte from D0 to the
    // memory location addressed by A0.
    // The PC will be updated, and the CPU will reflect the data transfer after consuming
    // additional cycles.
    // ----------------------------------------------------------------
    let cycles = cpu.step();
    println!("After MOVE.B: PC={:06X}, Cycles={}", cpu.pc, cycles);

    // ----------------------------------------------------------------
    // Memory Verification:
    // To verify that the MOVE.B instruction correctly stored the value, read the data byte
    // from the RAM location pointed to by A0 (0xFF0000). The read_byte method returns a Result,
    // so we use unwrap_or(0) to fallback on 0 in case of an error.
    // ----------------------------------------------------------------
    let memory_value = cpu.memory.read_byte(cpu.a[0]).unwrap_or(0);
    println!("Memory[A0]={:02X}", memory_value);

    // ----------------------------------------------------------------
    // Instruction Execution: RTS
    // Execute the RTS (Return from Subroutine) instruction. This simulates a subroutine
    // return by popping a return address from the CPU stack.
    // The PC is updated to the popped return address, and the corresponding cycles are consumed.
    // ----------------------------------------------------------------
    let cycles = cpu.step();
    println!("After RTS: PC={:06X}, Cycles={}", cpu.pc, cycles);
}
