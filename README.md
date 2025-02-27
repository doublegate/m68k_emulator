# m68k_emulator - Motorola 68000 'm68k' CPU Core in Rust

## Motorola 68000 CPU Emulation in Rust

A cycle-accurate Motorola 68000 (M68000) CPU emulator written in Rust, designed to replicate the behavior of the original 16/32-bit microprocessor as specified in the *M68000 Programmer's Reference Manual*. This emulator supports all base 68000 instructions, interrupt handling, exception processing, and instruction prefetching.

## Features

- **Complete Instruction Set**: Implements all 90 base M68000 instructions (e.g., `MOVE`, `ADD`, `MULS`, `DIVU`, branch ops, bit ops).
- **Cycle Accuracy**: Matches documented cycle counts, including dynamic timings for `MULS`, `DIVU`, etc.
- **Supervisor Mode**: Enforces privileged instructions with proper exception handling.
- **Interrupts**: Supports auto-vectored and vectored interrupts with nesting up to 7 levels.
- **Exceptions**: Handles privilege violations, zero divide, and more with stack frames.
- **Prefetch**: Emulates the 68000’s two-word instruction prefetch queue.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Prerequisites

- **Rust**: Install via [rustup](https://rustup.rs/) (stable channel recommended).
- **Cargo**: Included with Rust for building and testing.

## Getting Started

### Clone the Repository

```sh
git clone https://github.com/doublegate/m68k_emulator.git
cd m68k_emulator
```

### Build the Project

```sh
cargo build
```

### Run Sample Programs

The main.rs includes a basic sample program:

```sh
cargo run
```

### Run Tests

Execute the test suite:

```sh
cargo test
```

This runs unit and integration tests in tests/cpu_tests.rs and tests/samples.rs.

## Project Structure

m68k_emulator/
├── Cargo.toml
├── LICENSE
├── README.md
├── .gitignore
├── .github/
│   └── workflows/
│       └── rust.yml
├── src/
│   ├── main.rs
│   ├── lib.rs
│   └── m68k_cpu.rs
└── tests/
    ├── cpu_tests.rs
    └── samples.rs

- src/m68k_cpu.rs: Core CPU emulator implementation.
- src/main.rs: Entry point with a sample program.
- src/lib.rs: Library module exposing m68k_cpu.
- tests/cpu_tests.rs: Comprehensive tests for CPU instructions and features.
- tests/samples.rs: Sample programs with verification tests.

## Testing

The test suite verifies:

- All 90 instructions (e.g., NOP, MOVE, ADD, DIVU).
- Exception handling (e.g., Zero Divide, Privilege Violation).
- Interrupt nesting and resumption from STOP.
- Prefetch accuracy and cycle counts.

To see test output:

```sh
cargo test -- --nocapture
```

## Sample Programs

- Loop Test: A simple decrementing loop using MOVEQ, SUBQ, and BNE.
- Interrupt Test: Demonstrates STOP halting and interrupt resumption.
- NOTE: Currently working on expanding these in tests/samples.rs for further verification.

## Errata

- Divide-by-Zero: Triggers exceptions but doesn’t yet support full bus error handling.
- STOP Resumption: Simplified as an interrupt-driven resume; real hardware may require additional reset logic.
- Prefetch: Basic two-word queue implemented; edge cases (e.g., bus contention) not fully modeled.
- Wait States: Not emulated (e.g., DTACK delays); assumes instant memory access.

## Contributing

Contributions are welcome! Please:

1. Fork the repository.
2. Create a feature branch (git checkout -b feature/your-feature).
3. Commit changes (git commit -am 'Add your feature').
4. Push to the branch (git push origin feature/your-feature).
5. Open a Pull Request.

## Acknowledgments

Inspired by the Motorola 68000 Programmer's Reference Manual.
Built with Rust for performance and safety.

## Contact

If you have any questions or issues, please open an issue on GitHub or contact [parobek@gmail.com]
