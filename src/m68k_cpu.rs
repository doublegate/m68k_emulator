//! m68k_cpu.rs - Motorola 68000 (68k) CPU Emulator Core
//! --- Version 0.3.1
//!
//! This file implements the core of a Motorola 68000 CPU emulator. It defines the essential
//! components including CPU registers (data and address registers), a program counter, status
//! register, and a memory interface. The code handles instruction prefetching, decoding, execution,
//! cycles calculation, as well as exceptions and interrupts, providing a comprehensive emulation of
//! the 68k CPU behavior.
//!
//! # Features
//!
//! - **Instruction Decoding and Execution:** Implements a broad range of 68k operations such as
//!   arithmetic (ADD, SUB, MULS, DIVU, DIVS), logical (AND, OR, EOR, NOT), bit manipulation (BSET,
//!   BCLR, BCHG, BTST), control flow (Branch, Jump, JSR, RTS), and more.
//! - **Addressing Modes:** Supports multiple addressing modes including Data/Register Direct,
//!   Address Register Indirect, Indexed, Displacement, PC-relative modes, Immediate, and more.
//! - **Cycle-Accurate Emulation:** Computes execution cycles dynamically based on addressing modes,
//!   operand sizes, and operations, aiming for accurate timing in emulation.
//! - **Exception and Interrupt Handling:** Provides mechanisms to trigger exceptions (e.g., Address
//!   Error) and manage interrupts with nested interrupt levels and bus state management.
//! - **Memory Operations:** Implements safe memory access routines for byte, word, and long
//!   operations, complete with error handling to integrate with the exception system.
//!
//! # Implementation Details
//!
//! - **CPU Structure:** The main `CPU` struct encapsulates:
//!   - General-purpose registers (D0-D7 and A0-A7).
//!   - Program counter (`pc`), status register (`sr`), and a memory interface.
//!   - A prefetch queue to optimize instruction fetching.
//!   - Cycle counting for tracking execution time.
//!   - Internal state flags, such as `exception_in_progress` to manage recursive exceptions.
//!
//! - **Instruction Decoding:** The `decode()` function fetches an opcode, interprets addressing modes,
//!   and generates an `Instruction` struct that contains the operation code, optional size, source, and
//!   destination operands.
//!
//! - **Execution Engine:** The `execute()` function matches the decoded instruction's operation, performs
//!   the required read/write operations, updates CPU flags and registers, and computes the operation's
//!   cycle cost.
//!
//! - **Memory and Bus Management:** Memory read/write abstraction and bus state transitions are managed
//!   to emulate the actual hardware behavior, including interrupt acknowledgement and data transfer
//!   delays.
//!
//! # Technical Strategy
//!
//! - The design leverages Rust’s strong type system using enums (`Operation`, `Operand`, and `Size`) to
//!   enforce correctness and clarity.
//! - Each CPU operation is implemented with precise control over operand handling, flag updates, and cycle
//!   counting, making the emulation both accurate and modular.
//! - Safety and error handling are integral, with checks for addressing and memory errors to trigger
//!   exceptions, thus preventing unintended recursive fault conditions.
//!
//! # Versioning and Maintenance
//!
//! - Version: 1.0.0
//! - Part of the m68k_emulator project.
//! - Recent updates include improved addressing mode decode logic, refined cycle counting, and enhanced
//!   exception handling.
//!
//! # Usage
//!
//! - Create a new CPU instance with a memory implementation using `CPU::new(memory)`.
//! - Control execution by repeatedly fetching, decoding, and executing instructions.
//! - Utilize provided methods for injecting interrupts and handling CPU exceptions.
//!
//! # Future Enhancements
//!
//! - Extend the instruction set with additional 68k operations.
//! - Integrate more detailed cycle-accurate simulation and hardware-level timing.
//! - Improve debugging support with detailed logging and tracing capabilities.
//!
//! # Authors and Contributors
//!
//! - Original Author: DoubleGate
//! - GitHub Location: https://github.com/doublegate/m68k_emulator/blob/main/src/m68k_cpu.rs
//!
//! # License
//!
//! - Distributed under the MIT License.
//! 
// src/m68k_cpu.rs
use crate::memory::Memory;
use crate::memory::Exception;

#[derive(Debug, Clone, Copy, PartialEq)] // Add PartialEq
pub enum Size {
    Byte,
    Word,
    Long,
}

impl Size {
    fn bits(&self) -> u32 {
        match self {
            Size::Byte => 8,
            Size::Word => 16,
            Size::Long => 32,
        }
    }

    fn mask(&self) -> u32 {
        match self {
            Size::Byte => 0xFF,
            Size::Word => 0xFFFF,
            Size::Long => 0xFFFFFFFF,
        }
    }
}

#[derive(Debug, Clone)]
pub enum Operand {
    DataRegister(u8),
    AddressRegister(u8),
    Indirect(u8),
    PostInc(u8),
    PreDec(u8),
    Displacement(u8, i16),
    Indexed(u8, i8, u8, Size),
    AbsoluteShort(u32),
    AbsoluteLong(u32),
    PCDisplacement(i16),
    PCIndexed(i8, u8, Size),
    Immediate(u32),
}

#[derive(Debug, PartialEq)] // Add PartialEq
pub enum Operation {
    Nop,
    Rts,
    Move,
    Add,
    Muls,
    Divu,
    Eor,
    Neg,
    Tst,
    Lsr,
    Asl,
    Moveq,
    Ror,
    Eori,
    Cmpi,
    Movep,
    Bclr,
    Lea,
    Ext,
    Sub,
    Andi,
    Ori,
    Addx,
    Subx,
    Bra,
    Beq,
    Bne,
    Clr,
    Swap,
    Cmpa,
    Chk,
    Tas,
    Rol,
    Roxl,
    Jsr,
    Bchg,
    Bset,
    Btst,
    Trap,
    And,
    Or,
    Addq,
    Subq,
    Not,
    Divs,
    Roxr,
    Bhi,
    Bls,
    Jmp,
    Adda,
    Suba,
    Mulu,
    Cmp,
    Scc,
    Dbcc,
    Pea,
    Link,
    Unlk,
    NopAlt,
    Bgt,
    Ble,
    Exg,
    Movem,
    AslMem,
    LsrMem,
    ClrMem,
    Stop,
    Rtd,
    TasMem,
    Bcc,
    Bcs,
    Bge,
    Blt,
    Bmi,
    Bpl,
    Lsl,
    RolMem,
    RorMem,
    Subi,
    Abcd,
    Sbcd,
    Nbcd,
    Addi,
    Bvc,
    Bvs,
    RoxlMem,
    RoxrMem,
    Trapv,
    Reset,
    Rte,
    Movea,
    MoveCcr,
    MoveSr,
    MoveUsp,
    AndiCcr,
    OriCcr,
    EoriCcr,
    LslMem,
    Asr,
}

#[derive(Debug)]
struct Instruction {
    operation: Operation,
    size: Option<Size>,
    src: Option<Operand>,
    dst: Option<Operand>,
}

pub struct CPU {
    pub d: [u32; 8],
    pub a: [u32; 8],
    pub pc: u32,
    pub sr: u16,
    pub memory: Memory,
    cycle_count: u64,
    pending_interrupts: Vec<(u8, Option<u8>)>, // Use Vec with capacity
    interrupt_ack: Option<u8>,
    interrupt_nest_level: u8,
    bus_state: BusState,
    halted: bool,
    prefetch_queue: [u16; 2],
    exception_in_progress: bool, // Flag to prevent recursive exceptions
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BusState {
    Idle,
    Iack(u8),
    Vpa,
    Dtack(u8),
}

impl CPU {
    pub fn new(memory: Memory) -> Self {
        CPU {
            d: [0; 8],
            a: [0; 8],
            pc: 0, // PC is initialized later
            sr: 0x2700,
            memory,
            cycle_count: 0,
            pending_interrupts: Vec::with_capacity(8), // Pre-allocate
            interrupt_ack: None,
            interrupt_nest_level: 0,
            bus_state: BusState::Idle,
            halted: false,
            prefetch_queue: [0; 2],
            exception_in_progress: false, // Initialize exception flag
        }
    }

    fn cpu_read_byte(&mut self, address: u32) -> u8 {
        match self.memory.read_byte(address) {
            Ok(value) => value,
            Err(Exception::AddressError) => {
                if !self.exception_in_progress { // Avoid recursion
                    self.exception_in_progress = true;
                    self.trigger_exception(3);
                    self.exception_in_progress = false;
                }
                0 // Return a safe value, don't propagate error
            }
        }
    }

    fn cpu_read_word(&mut self, address: u32) -> u16 {
        match self.memory.read_word(address) {
            Ok(value) => value,
            Err(Exception::AddressError) => {
                if !self.exception_in_progress {
                    self.exception_in_progress = true;
                    self.trigger_exception(3);
                    self.exception_in_progress = false;
                }
                0 // Avoid propagating AddressError
            }
        }
    }

    fn cpu_read_long(&mut self, address: u32) -> u32 {
        match self.memory.read_long(address) {
            Ok(value) => value,
            Err(Exception::AddressError) => {
                if !self.exception_in_progress {
                    self.exception_in_progress = true;
                    self.trigger_exception(3);
                    self.exception_in_progress = false;
                }
                0 // Avoid propagating AddressError
            }
        }
    }

    fn cpu_write_byte(&mut self, address: u32, value: u8) {
        if let Err(Exception::AddressError) = self.memory.write_byte(address, value) {
            self.trigger_exception(3);
        }
    }

    fn cpu_write_word(&mut self, address: u32, value: u16) {
        if let Err(Exception::AddressError) = self.memory.write_word(address, value) {
            self.trigger_exception(3);
        }
    }

    fn cpu_write_long(&mut self, address: u32, value: u32) {
        if let Err(Exception::AddressError) = self.memory.write_long(address, value) {
            self.trigger_exception(3);
        }
    }

    pub fn prefetch(&mut self) {
        self.prefetch_queue[0] = self.cpu_read_word(self.pc);
        self.prefetch_queue[1] = self.cpu_read_word(self.pc + 2);
        self.pc += 4;
    }

    fn fetch_word(&mut self) -> u16 {
        let word = self.prefetch_queue[0];
        self.prefetch_queue[0] = self.prefetch_queue[1];
        self.prefetch_queue[1] = self.cpu_read_word(self.pc);
        self.pc += 2;
        word
    }

    fn fetch_long(&mut self) -> u32 {
        let high = self.fetch_word() as u32;
        let low = self.fetch_word() as u32;
        (high << 16) | low
    }

    fn set_flags(&mut self, n: bool, z: bool, v: bool, c: bool, x: bool) {
        let mut sr = self.sr & 0xFFF0;
        if n {
            sr |= 0x8;
        }
        if z {
            sr |= 0x4;
        }
        if v {
            sr |= 0x2;
        }
        if c {
            sr |= 0x1;
        }
        if x {
            sr |= 0x10;
        }
        self.sr = sr;
    }

    fn ea_cycles(&self, operand: &Operand, size: Size) -> u32 {
        match operand {
            Operand::DataRegister(_) => 0,
            Operand::AddressRegister(_) => 0,
            Operand::Indirect(_reg) => match size {
                Size::Byte | Size::Word => 4,
                Size::Long => 8,
            },
            Operand::PostInc(reg) => {
                if *reg == 7 && size == Size::Byte {
                    6
                } else {
                    match size {
                        Size::Byte | Size::Word => 4,
                        Size::Long => 8,
                    }
                }
            }
            Operand::PreDec(reg) => {
                if *reg == 7 && size == Size::Byte {
                    8
                } else {
                    match size {
                        Size::Byte | Size::Word => 6,
                        Size::Long => 10,
                    }
                }
            }
            Operand::Displacement(_, _) => match size {
                Size::Byte | Size::Word => 8,
                Size::Long => 12,
            },
            Operand::Indexed(_, _, _, _) => match size {
                Size::Byte | Size::Word => 10,
                Size::Long => 14,
            },
            Operand::AbsoluteShort(_) => match size {
                Size::Byte | Size::Word => 8,
                Size::Long => 12,
            },
            Operand::AbsoluteLong(_) => match size {
                Size::Byte | Size::Word => 12,
                Size::Long => 16,
            },
            Operand::PCDisplacement(_) => match size {
                Size::Byte | Size::Word => 8,
                Size::Long => 12,
            },
            Operand::PCIndexed(_, _, _) => match size {
                Size::Byte | Size::Word => 10,
                Size::Long => 14,
            },
            Operand::Immediate(_) => match size {
                Size::Byte => 0,
                Size::Word => 4,
                Size::Long => 8,
            },
        }
    }

    fn check_supervisor(&mut self) -> bool {
        (self.sr & 0x2000) != 0
    }

    pub fn request_interrupt(&mut self, level: u8, vector: Option<u8>) {
        if level > 7 || level == 0 {
            return;
        }
        if let Some(v) = vector {
            if v < 2 {
                return;
            }
        }
        if !self.pending_interrupts.iter().any(|&(l, v)| l == level && v == vector) {
            self.pending_interrupts.push((level, vector));
            self.pending_interrupts.sort_by(|a, b| b.0.cmp(&a.0));
        }
    }

    fn process_interrupt(&mut self, level: u8, vector: Option<u8>) -> u32 {
        self.halted = false;
        let mut cycles = 0;
        self.bus_state = BusState::Iack(level);
        self.interrupt_ack = Some(level);
        cycles += 14;
        let vec_addr = match vector {
            Some(v) => {
                self.bus_state = BusState::Dtack(v);
                cycles += 4;
                v as u32
            }
            None => {
                self.bus_state = BusState::Vpa;
                24 + level as u32
            }
        };
        self.sr |= 0x2000;
        self.sr = (self.sr & 0xF8FF) | ((level as u16) << 8);
        self.interrupt_nest_level += 1;
        self.a[7] -= 2;
        self.cpu_write_word(self.a[7], self.sr);
        self.a[7] -= 4;
        self.cpu_write_long(self.a[7], self.pc);
        cycles += 10;
        self.pc = self.cpu_read_long(vec_addr * 4);
        // Remove prefetch here, called in step()
        cycles += 4;
        self.bus_state = BusState::Idle;
        self.pending_interrupts.retain(|&(l, _)| l != level);
        cycles
    }

    pub fn trigger_exception(&mut self, vector: u32) -> u32 {
        // If an exception is already in progress, halt to avoid recursion.
        if self.exception_in_progress {
             self.halted = true;
             return 0;
        }
        self.exception_in_progress = true;
        
        // Set supervisor bit.
        self.sr |= 0x2000;
        
        // Push SR and PC onto the stack.
        self.a[7] = self.a[7].wrapping_sub(2);
        self.cpu_write_word(self.a[7], self.sr);
        self.a[7] = self.a[7].wrapping_sub(4);
        self.cpu_write_long(self.a[7], self.pc);
        
        // Load new PC from exception vector table.
        let new_pc = self.cpu_read_long(vector * 4);
        self.pc = new_pc;
        
        self.exception_in_progress = false;
        
        // If the new PC is 0 (or invalid), halt the CPU to prevent endless exception loops.
        if new_pc == 0 {
             self.halted = true;
        }
        34
    }

    pub fn get_interrupt_ack(&self) -> (Option<u8>, BusState) {
        (self.interrupt_ack, self.bus_state)
    }

    fn decode_ea(&mut self, mode: u8, reg: u8, size: Size) -> Operand {
        match mode {
            0 => Operand::DataRegister(reg),
            1 => Operand::AddressRegister(reg),
            2 => Operand::Indirect(reg),
            3 => Operand::PostInc(reg),
            4 => Operand::PreDec(reg),
            5 => {
                let disp = self.fetch_word() as i16;
                Operand::Displacement(reg, disp)
            }
            6 => {
                let ext = self.fetch_word();
                let disp = (ext as i8) as i8;
                let index_reg = (ext >> 12) & 0x7;
                let index_size = if (ext & 0x800) != 0 {
                    Size::Long
                } else {
                    Size::Word
                };
                Operand::Indexed(reg, disp, index_reg as u8, index_size)
            }
            7 => match reg {
                0 => Operand::AbsoluteShort(self.fetch_word() as u32),
                1 => Operand::AbsoluteLong(self.fetch_long()),
                2 => {
                    let disp = self.fetch_word() as i16;
                    Operand::PCDisplacement(disp)
                }
                3 => {
                    let ext = self.fetch_word();
                    let disp = (ext as i8) as i8;
                    let index_reg = (ext >> 12) & 0x7;
                    let index_size = if (ext & 0x800) != 0 {
                        Size::Long
                    } else {
                        Size::Word
                    };
                    Operand::PCIndexed(disp, index_reg as u8, index_size)
                }
                4 => {
                    let value = match size {
                        Size::Byte => (self.fetch_word() & 0xFF) as u32,
                        Size::Word => self.fetch_word() as u32,
                        Size::Long => self.fetch_long(),
                    };
                    Operand::Immediate(value)
                }
                _ => panic!("Invalid EA register: {}", reg),
            },
            _ => panic!("Invalid EA mode: {}", mode),
        }
    }

    fn get_operand_value(&mut self, size: Size, operand: &Operand) -> u32 {
        match operand {
            Operand::DataRegister(reg) => self.d[*reg as usize] & size.mask(),
            Operand::AddressRegister(reg) => self.a[*reg as usize],
            Operand::Indirect(reg) => {
                let addr = self.a[*reg as usize];
                match size {
                    Size::Byte => self.cpu_read_byte(addr) as u32,
                    Size::Word => self.cpu_read_word(addr) as u32,
                    Size::Long => self.cpu_read_long(addr),
                }
            }
            Operand::PostInc(reg) => {
                let addr = self.a[*reg as usize];
                let value = match size {
                    Size::Byte => self.cpu_read_byte(addr) as u32,
                    Size::Word => self.cpu_read_word(addr) as u32,
                    Size::Long => self.cpu_read_long(addr),
                };
                self.a[*reg as usize] += size.bits() / 8;
                value
            }
            Operand::PreDec(reg) => {
                self.a[*reg as usize] -= size.bits() / 8;
                let addr = self.a[*reg as usize];
                match size {
                    Size::Byte => self.cpu_read_byte(addr) as u32,
                    Size::Word => self.cpu_read_word(addr) as u32,
                    Size::Long => self.cpu_read_long(addr),
                }
            }
            Operand::Displacement(reg, disp) => {
                let addr = self.a[*reg as usize].wrapping_add(*disp as u32);
                match size {
                    Size::Byte => self.cpu_read_byte(addr) as u32,
                    Size::Word => self.cpu_read_word(addr) as u32,
                    Size::Long => self.cpu_read_long(addr),
                }
            }
            Operand::Indexed(reg, disp, idx_reg, idx_size) => {
                let base = self.a[*reg as usize];
                let index = match idx_size {
                    Size::Word => (self.d[*idx_reg as usize] as i16) as i32,
                    Size::Long => self.d[*idx_reg as usize] as i32,
                    _ => unreachable!(),
                };
                let addr = base.wrapping_add(*disp as i32 as u32).wrapping_add(index as u32);
                match size {
                    Size::Byte => self.cpu_read_byte(addr) as u32,
                    Size::Word => self.cpu_read_word(addr) as u32,
                    Size::Long => self.cpu_read_long(addr),
                }
            }
            Operand::AbsoluteShort(addr) => match size {
                Size::Byte => self.cpu_read_byte(*addr) as u32,
                Size::Word => self.cpu_read_word(*addr) as u32,
                Size::Long => self.cpu_read_long(*addr),
            },
            Operand::AbsoluteLong(addr) => match size {
                Size::Byte => self.cpu_read_byte(*addr) as u32,
                Size::Word => self.cpu_read_word(*addr) as u32,
                Size::Long => self.cpu_read_long(*addr),
            },
            Operand::PCDisplacement(disp) => {
                let addr = self.pc.wrapping_add(*disp as u32 - 2);
                match size {
                    Size::Byte => self.cpu_read_byte(addr) as u32,
                    Size::Word => self.cpu_read_word(addr) as u32,
                    Size::Long => self.cpu_read_long(addr),
                }
            }
            Operand::PCIndexed(disp, idx_reg, idx_size) => {
                let base = self.pc - 2;
                let index = match idx_size {
                    Size::Word => (self.d[*idx_reg as usize] as i16) as i32,
                    Size::Long => self.d[*idx_reg as usize] as i32,
                    _ => unreachable!(),
                };
                let addr = base.wrapping_add(*disp as i32 as u32).wrapping_add(index as u32);
                match size {
                    Size::Byte => self.cpu_read_byte(addr) as u32,
                    Size::Word => self.cpu_read_word(addr) as u32,
                    Size::Long => self.cpu_read_long(addr),
                }
            }
            Operand::Immediate(val) => *val & size.mask(),
        }
    }

    fn set_operand_value(&mut self, size: Size, operand: &Operand, value: u32) {
        match operand {
            Operand::DataRegister(reg) => self.d[*reg as usize] = value & size.mask(),
            Operand::AddressRegister(reg) => self.a[*reg as usize] = value,
            Operand::Indirect(reg) => {
                let addr = self.a[*reg as usize];
                match size {
                    Size::Byte => self.cpu_write_byte(addr, value as u8),
                    Size::Word => self.cpu_write_word(addr, value as u16),
                    Size::Long => self.cpu_write_long(addr, value),
                }
            }
            Operand::PostInc(reg) => {
                let addr = self.a[*reg as usize];
                match size {
                    Size::Byte => self.cpu_write_byte(addr, value as u8),
                    Size::Word => self.cpu_write_word(addr, value as u16),
                    Size::Long => self.cpu_write_long(addr, value),
                }
                self.a[*reg as usize] += size.bits() / 8;
            }
            Operand::PreDec(reg) => {
                self.a[*reg as usize] -= size.bits() / 8;
                let addr = self.a[*reg as usize];
                match size {
                    Size::Byte => self.cpu_write_byte(addr, value as u8),
                    Size::Word => self.cpu_write_word(addr, value as u16),
                    Size::Long => self.cpu_write_long(addr, value),
                }
            }
            Operand::Displacement(reg, disp) => {
                let addr = self.a[*reg as usize].wrapping_add(*disp as u32);
                match size {
                    Size::Byte => self.cpu_write_byte(addr, value as u8),
                    Size::Word => self.cpu_write_word(addr, value as u16),
                    Size::Long => self.cpu_write_long(addr, value),
                }
            }
            Operand::Indexed(reg, disp, idx_reg, idx_size) => {
                let base = self.a[*reg as usize];
                let index = match idx_size {
                    Size::Word => (self.d[*idx_reg as usize] as i16) as i32,
                    Size::Long => self.d[*idx_reg as usize] as i32,
                    _ => unreachable!(),
                };
                let addr = base.wrapping_add(*disp as i32 as u32).wrapping_add(index as u32);
                match size {
                    Size::Byte => self.cpu_write_byte(addr, value as u8),
                    Size::Word => self.cpu_write_word(addr, value as u16),
                    Size::Long => self.cpu_write_long(addr, value),
                }
            }
            Operand::AbsoluteShort(addr) => match size {
                Size::Byte => self.cpu_write_byte(*addr, value as u8),
                Size::Word => self.cpu_write_word(*addr, value as u16),
                Size::Long => self.cpu_write_long(*addr, value),
            },
            Operand::AbsoluteLong(addr) => match size {
                Size::Byte => self.cpu_write_byte(*addr, value as u8),
                Size::Word => self.cpu_write_word(*addr, value as u16),
                Size::Long => self.cpu_write_long(*addr, value),
            },
            _ => panic!("Cannot write to operand: {:?}", operand),
        }
    }

    fn decode(&mut self) -> Instruction {
        let opcode = self.fetch_word();

        // Handle RTS first
        if opcode == 0x4E75 {
            return Instruction {
                operation: Operation::Rts,
                size: None,
                src: None,
                dst: None,
            };
        }

        // Handle NOP
        if opcode == 0x4E71 {
            return Instruction {
                operation: Operation::Nop,
                size: None,
                src: None,
                dst: None,
            };
        }

        // Handle RTE, RESET, TRAPV
        if (opcode & 0xFFF8) == 0x4E70 {
            let operation = match opcode {
                0x4E73 => Operation::Rte,
                0x4E70 => Operation::Reset,
                0x4E76 => Operation::Trapv,
                _ => Operation::Nop, // Should not happen
            };
            return Instruction {
                operation,
                size: None,
                src: None,
                dst: None,
            };
        }

        // Handle TRAP
        if (opcode & 0xF000) == 0x4E40 {
            let vector = (opcode & 0xF) as u32;
            return Instruction {
                operation: Operation::Trap,
                size: None,
                src: Some(Operand::Immediate(vector)),
                dst: None,
            };
        }

        // Handle MOVEQ
        if (opcode & 0xF000) == 0x7000 {
            let reg = ((opcode >> 9) & 0x7) as u8;
            let data = (opcode & 0xFF) as i8 as i32 as u32;
            return Instruction {
                operation: Operation::Moveq,
                size: Some(Size::Long),
                src: Some(Operand::Immediate(data)),
                dst: Some(Operand::DataRegister(reg)),
            };
        }

        // Branch instructions (most common first)
        if (opcode & 0xFF00) == 0x6000 {
            let disp = if (opcode & 0x00FF) == 0 {
                self.fetch_word() as i16 as i32
            } else {
                (opcode & 0x00FF) as i8 as i32
            };

            let operation = match opcode & 0x0F00 {
                0x0000 => Operation::Bra,
                0x0400 => Operation::Bhi,
                0x0600 => Operation::Bls,
                0x0800 => Operation::Bcc,
                0x0A00 => Operation::Bcs,
                0x0C00 => Operation::Bne,
                0x0E00 => Operation::Beq,
                0x1000 => Operation::Bvc,
                0x1200 => Operation::Bvs,
                0x1400 => Operation::Bpl,
                0x1600 => Operation::Bmi,
                0x1800 => Operation::Bge,
                0x1A00 => Operation::Blt,
                0x1C00 => Operation::Bgt,
                0x1E00 => Operation::Ble,
                _ => Operation::Nop, // Should not happen
            };
            return Instruction {
                operation,
                size: None,
                src: Some(Operand::Immediate(disp as u32)),
                dst: None,
            };
        }

        // MOVE instructions
        if (opcode & 0x3000) == 0x1000 {
            let size = match (opcode >> 12) & 0x3 {
                1 => Size::Byte,
                3 => Size::Word,
                2 => Size::Long,
                _ => unreachable!(),
            };
            let dst_reg = ((opcode >> 9) & 0x7) as u8;
            let dst_mode = ((opcode >> 6) & 0x7) as u8;
            let src_mode = ((opcode >> 3) & 0x7) as u8;
            let src_reg = (opcode & 0x7) as u8;
            let src = self.decode_ea(src_mode, src_reg, size);
            let mut dst = self.decode_ea(dst_mode, dst_reg, size);
            // For MOVE.B, a destination of type AddressRegister is not allowed.
            // Convert an AddressRegister destination into Indirect addressing.
            if size == Size::Byte {
                 if let Operand::AddressRegister(reg) = dst {
                      dst = Operand::Indirect(reg);
                 }
            }
            return Instruction {
                operation: Operation::Move,
                size: Some(size),
                src: Some(src),
                dst: Some(dst),
            };
        }

        // ADD, SUB instructions
        if (opcode & 0xD000) == 0xD000 {
            let reg = ((opcode >> 9) & 0x7) as u8;
            let direction = (opcode >> 8) & 0x1; // 0: EA -> Dn, 1: Dn -> EA
            let size = match (opcode >> 6) & 0x3 {
                0 => Size::Byte,
                1 => Size::Word,
                2 => Size::Long,
                _ => unreachable!(),
            };
            let src_mode = ((opcode >> 3) & 0x7) as u8;
            let src_reg = (opcode & 0x7) as u8;
            let (src, dst) = if direction == 0 {
                let ea = self.decode_ea(src_mode, src_reg, size);
                (ea, Operand::DataRegister(reg))
            } else {
                let ea = self.decode_ea(src_mode, src_reg, size);
                (Operand::DataRegister(reg), ea)
            };
            let operation = if (opcode & 0x1000) == 0 {
                Operation::Add
            } else {
                Operation::Sub
            };
            return Instruction {
                operation,
                size: Some(size),
                src: Some(src),
                dst: Some(dst),
            };
        }

        // AND, OR instructions
        if (opcode & 0xC000) == 0xC000 {
            let dn = ((opcode >> 9) & 0x7) as u8;
            let size = match (opcode >> 6) & 0x3 {
                0 => Size::Byte,
                1 => Size::Word,
                2 => Size::Long,
                _ => unreachable!(),
            };
            let src_mode = ((opcode >> 3) & 0x7) as u8;
            let src_reg = (opcode & 0x7) as u8;
            let src = self.decode_ea(src_mode, src_reg, size);

            let operation = match opcode & 0x0F00 {
                0x0000 => Operation::And,
                0x0400 => Operation::Or,
                _ => Operation::Nop, // Shouldn't happen
            };
            return Instruction {
                operation,
                size: Some(size),
                src: Some(src),
                dst: Some(Operand::DataRegister(dn)),
            };
        }

        // ADDI, SUBI, CMPI (Immediate instructions)
        if (opcode & 0x0600) == 0x0600 || (opcode & 0x0400) == 0x0400 || (opcode & 0x0C00) == 0x0C00 {
            let size = match (opcode >> 6) & 0x3 {
                0 => Size::Byte,
                1 => Size::Word,
                2 => Size::Long,
                _ => unreachable!(),
            };
            let ea_mode = ((opcode >> 3) & 0x7) as u8;
            let ea_reg = (opcode & 0x7) as u8;
            let immediate = match size {
                Size::Byte => self.fetch_word() as u32 & 0xFF,
                Size::Word => self.fetch_word() as u32,
                Size::Long => self.fetch_long(),
            };
            let dst = self.decode_ea(ea_mode, ea_reg, size);

            let operation = match opcode {
                _ if (opcode & 0x0600) == 0x0600 => Operation::Addi,
                _ if (opcode & 0x0400) == 0x0400 => Operation::Subi,
                _ if (opcode & 0x0C00) == 0x0C00 => Operation::Cmpi,
                _ => Operation::Nop, // Should not happen
            };

            return Instruction {
                operation,
                size: Some(size),
                src: Some(Operand::Immediate(immediate)),
                dst: Some(dst),
            };
        }

        // EORI, ORI, ANDI (Immediate instructions)
        if (opcode & 0x0A00) == 0x0A00 || (opcode & 0x0000) == 0x0000 || (opcode & 0x0200) == 0x0200 {
            let size = match (opcode >> 6) & 0x3 {
                0 => Size::Byte,
                1 => Size::Word,
                2 => Size::Long,
                _ => unreachable!(),
            };
            let ea_mode = ((opcode >> 3) & 0x7) as u8;
            let ea_reg = (opcode & 0x7) as u8;
            let immediate = match size {
                Size::Byte => self.fetch_word() as u32 & 0xFF,
                Size::Word => self.fetch_word() as u32,
                Size::Long => self.fetch_long(),
            };
            let dst = self.decode_ea(ea_mode, ea_reg, size);

            let operation = match opcode {
                _ if (opcode & 0x0A00) == 0x0A00 => Operation::Eori,
                _ if (opcode & 0x0000) == 0x0000 => Operation::Ori,
                _ if (opcode & 0x0200) == 0x0200 => Operation::Andi,
                _ => Operation::Nop, // Should not happen
            };
            return Instruction {
                operation,
                size: Some(size),
                src: Some(Operand::Immediate(immediate)),
                dst: Some(dst),
            };
        }

        // MOVEP
        if (opcode & 0xF138) == 0x0108 {
            let direction = (opcode >> 7) & 0x1;
            let size = if (opcode >> 6) & 0x1 == 0 {
                Size::Word
            } else {
                Size::Long
            };
            let dreg = ((opcode >> 9) & 0x7) as u8;
            let areg = (opcode & 0x7) as u8;
            let _displacement = self.fetch_word() as i16;
            let (src, dst) = if direction == 0 {
                let ea = self.decode_ea(5, areg, size);
                (ea, Operand::DataRegister(dreg))
            } else {
                let ea = self.decode_ea(5, areg, size);
                (Operand::DataRegister(dreg), ea)
            };
            return Instruction {
                operation: Operation::Movep,
                size: Some(size),
                src: Some(src),
                dst: Some(dst),
            };
        }

        // SWAP, EXT
        if (opcode & 0xFF38) == 0x4800 {
            let size = if (opcode & 0x0040) == 0 {
                Size::Word
            } else {
                Size::Long
            };
            let reg = (opcode & 0x7) as u8;
            let operation = if (opcode & 0x00C0) == 0x00C0 {
                Operation::Ext
            } else {
                Operation::Swap
            };

            return Instruction {
                operation,
                size: Some(size),
                src: None,
                dst: Some(Operand::DataRegister(reg)),
            };
        }

        // CLR
        if (opcode & 0x4200) == 0x4200 {
            let size = match (opcode >> 6) & 0x3 {
                0 => Size::Byte,
                1 => Size::Word,
                2 => Size::Long,
                _ => unreachable!(),
            };
            let ea_mode = ((opcode >> 3) & 0x7) as u8;
            let ea_reg = (opcode & 0x7) as u8;
            let dst = self.decode_ea(ea_mode, ea_reg, size);
            return Instruction {
                operation: Operation::Clr,
                size: Some(size),
                src: None,
                dst: Some(dst),
            };
        }

        // TST
        if (opcode & 0x4A00) == 0x4A00 {
            let size = match (opcode >> 6) & 0x3 {
                0 => Size::Byte,
                1 => Size::Word,
                2 => Size::Long,
                _ => unreachable!(),
            };
            let src_mode = ((opcode >> 3) & 0x7) as u8;
            let src_reg = (opcode & 0x7) as u8;
            let src = self.decode_ea(src_mode, src_reg, size);
            return Instruction {
                operation: Operation::Tst,
                size: Some(size),
                src: Some(src),
                dst: None,
            };
        }

        // MOVE CCR, MOVE SR
        if (opcode & 0x4600) == 0x4600 {
            let mode = ((opcode >> 3) & 0x7) as u8;
            let reg = (opcode & 0x7) as u8;
            let src = self.decode_ea(mode, reg, Size::Word);

            let operation = match opcode {
                0x46C0..=0x46FF => Operation::MoveSr,
                0x42C0..=0x42FF => Operation::MoveCcr,
                _ => Operation::Nop,
            };

            return Instruction {
                operation,
                size: Some(Size::Word), // Assuming word for both
                src: Some(src),
                dst: None,
            };
        }

        // JMP
        if (opcode & 0x4EC0) == 0x4EC0 {
            let mode = ((opcode >> 3) & 0x7) as u8;
            let reg = (opcode & 0x7) as u8;
            let src = self.decode_ea(mode, reg, Size::Long);
            return Instruction {
                operation: Operation::Jmp,
                size: None,
                src: Some(src),
                dst: None,
            };
        }

        // PEA
        if (opcode & 0x4840) == 0x4840 {
            let mode = ((opcode >> 3) & 0x7) as u8;
            let reg = (opcode & 0x7) as u8;
            let src = self.decode_ea(mode, reg, Size::Long);
            return Instruction {
                operation: Operation::Pea,
                size: None,
                src: Some(src),
                dst: None,
            };
        }

        // LINK, UNLK
        if (opcode & 0x4E50) == 0x4E50 {
            let operation = match opcode & 0x000F {
                0x0000..=0x0007 => Operation::Unlk,
                _ => Operation::Link,
            };

            if operation == Operation::Link {
                let an = (opcode & 0x7) as u8;
                let disp = self.fetch_word() as i16 as i32;
                return Instruction {
                    operation: Operation::Link,
                    size: None,
                    src: Some(Operand::Immediate(disp as u32)),
                    dst: Some(Operand::AddressRegister(an)),
                };
            } else {
                let an = (opcode & 0x7) as u8;
                return Instruction {
                    operation: Operation::Unlk,
                    size: None,
                    src: None,
                    dst: Some(Operand::AddressRegister(an)),
                };
            }
        }

        // BSET, BCLR, BCHG, BTST
        if (opcode & 0x0100) == 0x0100 {
            let size = match opcode & 0x0038 {
                0x0000 => Size::Long,
                _ => Size::Byte, // Default to byte
            };

            let dn = ((opcode >> 9) & 0x7) as u8;
            let mode = ((opcode >> 3) & 0x7) as u8;
            let reg = (opcode & 0x7) as u8;

            let operation = match opcode & 0x00C0 {
                0x0040 => Operation::Bset,
                0x0080 => Operation::Bchg,
                0x0000 => Operation::Btst,
                0x00C0 => Operation::Bclr, // Added Bclr
                _ => Operation::Nop, // Should not happen
            };

            let dst = self.decode_ea(mode, reg, size);
            return Instruction {
                operation,
                size: Some(size),
                src: Some(Operand::DataRegister(dn)),
                dst: Some(dst),
            };
        }
        if (opcode & 0x54C0) == 0x54C0 {
            // DBcc (must be before CMP since this is the DBcc and we dont handle the other cc instructions)
            let dn = (opcode & 0x7) as u8;
            let disp = self.fetch_word() as i16 as i32;

            return Instruction {
                operation: Operation::Dbcc,
                size: Some(Size::Word), // Size is always word for DBcc
                src: Some(Operand::DataRegister(dn)),
                dst: Some(Operand::Immediate(disp as u32)),
            };
        }
        if (opcode & 0x4AFC) == 0x4AFC {
            // TAS
            let mode = ((opcode >> 3) & 0x7) as u8;
            let reg = (opcode & 0x7) as u8;
            let dst = self.decode_ea(mode, reg, Size::Byte);
            return Instruction {
                operation: Operation::TasMem,
                size: Some(Size::Byte),
                src: None,
                dst: Some(dst),
            };
        }

        //Default NOP or exception
        self.trigger_exception(4);
        Instruction {
            operation: Operation::Nop,
            size: None,
            src: None,
            dst: None,
        }
    }

    fn execute(&mut self, instr: Instruction) -> u32 {
        let mut cycles = 0;
        match instr.operation {
            Operation::Nop => cycles = 4,
            Operation::Rts => {
                self.pc = self.cpu_read_long(self.a[7]);
                self.a[7] += 4;
                cycles = 16;
            }
            Operation::Move => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let value = self.get_operand_value(size, &src);
                self.set_operand_value(size, &dst, value);
                let n = (value & (1 << (size.bits() - 1))) != 0;
                let z = value == 0;
                self.set_flags(n, z, false, false, false);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 8,
                } + self.ea_cycles(&src, size) + self.ea_cycles(&dst, size);
            }
            Operation::Add => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let (result, carry, overflow) = match size {
                    Size::Byte => {
                        let s = src_val as u8;
                        let d = dst_val as u8;
                        let r = s.wrapping_add(d);
                        let c = (s as u16 + d as u16) > 0xFF;
                        let v = ((s & 0x80) == (d & 0x80)) && ((r & 0x80) != (s & 0x80));
                        (r as u32, c, v)
                    }
                    Size::Word => {
                        let s = src_val as u16;
                        let d = dst_val as u16;
                        let r = s.wrapping_add(d);
                        let c = (s as u32 + d as u32) > 0xFFFF;
                        let v = ((s & 0x8000) == (d & 0x8000)) && ((r & 0x8000) != (s & 0x8000));
                        (r as u32, c, v)
                    }
                    Size::Long => {
                        let s = src_val;
                        let d = dst_val;
                        let r = s.wrapping_add(d);
                        let c = (s as u64 + d as u64) > 0xFFFFFFFF;
                        let v = ((s & 0x80000000) == (d & 0x80000000)) && ((r & 0x80000000) != (s & 0x80000000));
                        (r, c, v)
                    }
                };
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, overflow, carry, carry);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 8,
                } + self.ea_cycles(&src, size) + if matches!(dst, Operand::DataRegister(_)) {
                    0
                } else {
                    self.ea_cycles(&dst, size)
                };
            }
            Operation::Muls => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src) as i16 as i32;
                let dst_val = self.get_operand_value(size, &dst) as i16 as i32;
                let result = src_val * dst_val;
                self.set_operand_value(Size::Long, &dst, result as u32);
                let n = result < 0;
                let z = result == 0;
                self.set_flags(n, z, false, false, false);
                let _ones = (src_val as u16 as u32 & 0xFFFF).count_ones();
                cycles = 38 + 2 * (src_val as u16).count_ones() + self.ea_cycles(&src, size);
            }
            Operation::Divu => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let divisor = self.get_operand_value(size, &src) as u16 as u32;
                let dividend = self.get_operand_value(Size::Long, &dst);
                if divisor == 0 {
                    return self.trigger_exception(5);
                }
                let quotient = dividend / divisor;
                let remainder = dividend % divisor;
                let result = (remainder << 16) | (quotient & 0xFFFF);
                self.set_operand_value(Size::Long, &dst, result);
                let n = (quotient & 0x8000) != 0;
                let z = quotient == 0;
                let v = quotient > 0xFFFF;
                self.set_flags(n, z, v, false, false);
                cycles = 76;
                if dividend != 0 && divisor != 0 {
                    let quotient_bits = 32 - (dividend / divisor).leading_zeros();
                    let shift_count = if quotient_bits > 0 {
                        quotient_bits - 1
                    } else {
                        0
                    };
                    cycles += 2 * shift_count;
                    if dividend < divisor {
                        cycles += 2;
                    }
                }
                cycles += self.ea_cycles(&src, size);
            }
            Operation::Eor => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let result = src_val ^ dst_val;
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, false, false, false);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 8,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Neg => {
                let size = instr.size.unwrap();
                let dst = instr.dst.unwrap();
                let dst_val = self.get_operand_value(size, &dst);
                let result = 0u32.wrapping_sub(dst_val) & size.mask();
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let v = dst_val != 0;
                let c = dst_val != 0;
                self.set_flags(n, z, v, c, c);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 6,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Tst => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let src_val = self.get_operand_value(size, &src);
                let n = (src_val & (1 << (size.bits() - 1))) != 0;
                let z = src_val == 0;
                self.set_flags(n, z, false, false, false);
                cycles = 4 + self.ea_cycles(&src, size);
            }
            Operation::Lsr => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let shift_count = self.get_operand_value(Size::Long, &src) % size.bits();
                let dst_val = self.get_operand_value(size, &dst);
                let result = dst_val >> shift_count;
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = if shift_count > 0 {
                    (dst_val >> (shift_count - 1)) & 1 != 0
                } else {
                    false
                };
                self.set_flags(n, z, false, c, c);
                cycles = match size {
                    Size::Byte | Size::Word => 6,
                    Size::Long => 8,
                } + 2 * shift_count as u32;
            }
            Operation::Asl => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let shift_count = self.get_operand_value(Size::Long, &src) % size.bits();
                let dst_val = self.get_operand_value(size, &dst);
                let result = (dst_val << shift_count) & size.mask();
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = if shift_count > 0 {
                    (dst_val & (1 << (size.bits() - shift_count))) != 0
                } else {
                    false
                };
                let v = (0..shift_count).any(|i| {
                    let bit = (dst_val >> (size.bits() - 1 - i)) & 1;
                    bit != (dst_val >> (size.bits() - 1)) & 1
                });
                self.set_flags(n, z, v, c, c);
                cycles = match size {
                    Size::Byte | Size::Word => 6,
                    Size::Long => 8,
                } + 2 * shift_count as u32;
            }
            Operation::Moveq => {
                let dst = instr.dst.unwrap();
                let src = instr.src.unwrap();
                let value = self.get_operand_value(Size::Long, &src);
                self.set_operand_value(Size::Long, &dst, value);
                let n = (value & 0x80000000) != 0;
                let z = value == 0;
                self.set_flags(n, z, false, false, false);
                cycles = 4;
            }
            Operation::Ror => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let shift_count = self.get_operand_value(Size::Long, &src) % size.bits();
                let dst_val = self.get_operand_value(size, &dst);
                let mask = (1 << size.bits()) - 1;
                let result = ((dst_val >> shift_count) | (dst_val << (size.bits() - shift_count))) & mask;
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = if shift_count > 0 {
                    (dst_val >> (shift_count - 1)) & 1 != 0
                } else {
                    false
                };
                self.set_flags(n, z, false, c, c);
                cycles = match size {
                    Size::Byte | Size::Word => 6,
                    Size::Long => 8,
                } + 2 * shift_count as u32;
            }
            Operation::Eori => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let imm = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let result = imm ^ dst_val;
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, false, false, false);
                cycles = match size {
                    Size::Byte | Size::Word => 8,
                    Size::Long => 12,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Cmpi => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let imm = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let result = dst_val.wrapping_sub(imm);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let v = ((dst_val & !imm & !result) | (!dst_val & imm & result)) & (1 << (size.bits() - 1)) != 0;
                let c = ((!dst_val & imm) | (result & !dst_val) | (imm & result)) & (1 << (size.bits() - 1)) != 0;
                self.set_flags(n, z, v, c, c);
                cycles = match size {
                    Size::Byte => 8,
                    Size::Word => 8,
                    Size::Long => 14,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Movep => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                if let Operand::Displacement(areg, disp) = src {
                    let mut addr = self.a[areg as usize].wrapping_add(disp as u32);
                    if let Operand::DataRegister(dreg) = dst {
                        let count = if size == Size::Word {
                            2
                        } else {
                            4
                        };
                        let mut value = 0;
                        for _ in 0..count {
                            value = (value << 8) | (self.cpu_read_byte(addr) as u32);
                            addr += 2;
                        }
                        self.d[dreg as usize] = value;
                    }
                } else if let Operand::DataRegister(dreg) = src {
                    let value = self.d[dreg as usize];
                    if let Operand::Displacement(areg, disp) = dst {
                        let mut addr = self.a[areg as usize].wrapping_add(disp as u32);
                        let count = if size == Size::Word {
                            2
                        } else {
                            4
                        };
                        for i in (0..count).rev() {
                            self.cpu_write_byte(addr, ((value >> (8 * i)) & 0xFF) as u8);
                            addr += 2;
                        }
                    }
                }
                cycles = if size == Size::Word { 16 } else { 24 };
            }
            Operation::Bclr => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let bit_number = self.get_operand_value(Size::Long, &src) % size.bits();
                let dst_val = self.get_operand_value(size, &dst);
                let z = (dst_val & (1 << bit_number)) == 0;
                let result = dst_val & !(1 << bit_number);
                self.set_operand_value(size, &dst, result);
                self.set_flags(false, z, false, false, false);
                cycles = if matches!(dst, Operand::DataRegister(_)) {
                    10
                } else {
                    14
                } + self.ea_cycles(&dst, size);
            }
            Operation::Lea => {
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let addr = match src {
                    Operand::Displacement(reg, disp) => self.a[reg as usize].wrapping_add(disp as u32),
                    Operand::AbsoluteShort(addr) => addr,
                    Operand::AbsoluteLong(addr) => addr,
                    Operand::PCDisplacement(disp) => self.pc.wrapping_add(disp as u32 - 2),
                    _ => panic!("Unsupported addressing mode for LEA: {:?}", src),
                };
                if let Operand::AddressRegister(areg) = dst {
                    self.a[areg as usize] = addr;
                }
                cycles = match src {
                    Operand::Displacement(_, _) => 8,
                    Operand::AbsoluteShort(_) => 8,
                    Operand::AbsoluteLong(_) => 12,
                    Operand::PCDisplacement(_) => 8,
                    _ => 4,
                } + self.ea_cycles(&src, Size::Long);
            }
            Operation::Ext => {
                let size = instr.size.unwrap();
                let dst = instr.dst.unwrap();
                if let Operand::DataRegister(reg) = dst {
                    let value = self.d[reg as usize];
                    let result = if size == Size::Word {
                        ((value as i8) as i32) as u32 & 0xFFFF
                    } else {
                        ((value as i16) as i32) as u32
                    };
                    self.d[reg as usize] = result;
                    let n = (result & (1 << (size.bits() - 1))) != 0;
                    let z = result == 0;
                    self.set_flags(n, z, false, false, false);
                }
                cycles = 4;
            }
            Operation::Sub => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let (result, carry, overflow) = match size {
                    Size::Byte => {
                        let s = src_val as u8;
                        let d = dst_val as u8;
                        let r = d.wrapping_sub(s);
                        let c = (s as u16) > (d as u16);
                        let v = ((d & !s & !r) | (!d & s & r)) & 0x80 != 0;
                        (r as u32, c, v)
                    }
                    Size::Word => {
                        let s = src_val as u16;
                        let d = dst_val as u16;
                        let r = d.wrapping_sub(s);
                        let c = (s as u32) > (d as u32);
                        let v = ((d & !s & !r) | (!d & s & r)) & 0x8000 != 0;
                        (r as u32, c, v)
                    }
                    Size::Long => {
                        let s = src_val;
                        let d = dst_val;
                        let r = d.wrapping_sub(s);
                        let c = (s as u64) > (d as u64);
                        let v = ((d & !s & !r) | (!d & s & r)) & 0x80000000 != 0;
                        (r, c, v)
                    }
                };
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, overflow, carry, carry);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 8,
                } + self.ea_cycles(&src, size) + if matches!(dst, Operand::DataRegister(_)) {
                    0
                } else {
                    self.ea_cycles(&dst, size)
                };
            }
            Operation::Andi => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let imm = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let result = imm & dst_val;
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, false, false, false);
                cycles = match size {
                    Size::Byte | Size::Word => 8,
                    Size::Long => 14,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Ori => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let imm = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let result = imm | dst_val;
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, false, false, false);
                cycles = match size {
                    Size::Byte | Size::Word => 8,
                    Size::Long => 14,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Addx => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let x = (self.sr & 0x1) != 0;
                let (result, carry, overflow) = match size {
                    Size::Byte => {
                        let s = src_val as u8;
                        let d = dst_val as u8;
                        let x_val = if x {
                            1
                        } else {
                            0
                        };
                        let r = d.wrapping_add(s).wrapping_add(x_val);
                        let c = (s as u16 + d as u16 + x_val as u16) > 0xFF;
                        let v = ((s & d & !r) | (!s & !d & r)) & 0x80 != 0;
                        (r as u32, c, v)
                    }
                    Size::Word => {
                        let s = src_val as u16;
                        let d = dst_val as u16;
                        let x_val = if x {
                            1
                        } else {
                            0
                        };
                        let r = d.wrapping_add(s).wrapping_add(x_val);
                        let c = (s as u32 + d as u32 + x_val as u32) > 0xFFFF;
                        let v = ((s & d & !r) | (!s & !d & r)) & 0x8000 != 0;
                        (r as u32, c, v)
                    }
                    Size::Long => {
                        let s = src_val;
                        let d = dst_val;
                        let x_val = if x {
                            1
                        } else {
                            0
                        };
                        let r = d.wrapping_add(s).wrapping_add(x_val);
                        let c = (s as u64 + d as u64 + x_val as u64) > 0xFFFFFFFF;
                        let v = ((s & d & !r) | (!s & !d & r)) & 0x80000000 != 0;
                        (r, c, v)
                    }
                };
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = if self.sr & 0x4 != 0 && result == 0 {
                    true
                } else {
                    false
                };
                self.set_flags(n, z, overflow, carry, carry);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 8,
                } + self.ea_cycles(&src, size) + self.ea_cycles(&dst, size);
            }
            Operation::Subx => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let x = (self.sr & 0x1) != 0;
                let (result, carry, overflow) = match size {
                    Size::Byte => {
                        let s = src_val as u8;
                        let d = dst_val as u8;
                        let x_val = if x {
                            1
                        } else {
                            0
                        };
                        let r = d.wrapping_sub(s).wrapping_sub(x_val);
                        let c = (s as u16 + x_val as u16) > (d as u16);
                        let v = ((d & !s & !r) | (!d & s & r)) & 0x80 != 0;
                        (r as u32, c, v)
                    }
                    Size::Word => {
                        let s = src_val as u16;
                        let d = dst_val as u16;
                        let x_val = if x {
                            1
                        } else {
                            0
                        };
                        let r = d.wrapping_sub(s).wrapping_sub(x_val);
                        let c = (s as u32 + x_val as u32) > (d as u32);
                        let v = ((d & !s & !r) | (!d & s & r)) & 0x8000 != 0;
                        (r as u32, c, v)
                    }
                    Size::Long => {
                        let s = src_val;
                        let d = dst_val;
                        let x_val = if x {
                            1
                        } else {
                            0
                        };
                        let r = d.wrapping_sub(s).wrapping_sub(x_val);
                        let c = (s as u64 + x_val as u64) > (d as u64);
                        let v = ((d & !s & !r) | (!d & s & r)) & 0x80000000 != 0;
                        (r, c, v)
                    }
                };
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = if self.sr & 0x4 != 0 && result == 0 {
                    true
                } else {
                    false
                };
                self.set_flags(n, z, overflow, carry, carry);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 8,
                } + self.ea_cycles(&src, size) + self.ea_cycles(&dst, size);
            }
            Operation::Bra => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                self.pc = (self.pc as i32 + disp) as u32;
                self.prefetch();
                cycles = 10;
            }
            Operation::Beq => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                if self.sr & 0x4 != 0 {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Bne => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                if self.sr & 0x4 == 0 {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Clr => {
                let size = instr.size.unwrap();
                let dst = instr.dst.unwrap();
                self.set_operand_value(size, &dst, 0);
                self.set_flags(false, true, false, false, false);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 6,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Swap => {
                let dst = instr.dst.unwrap();
                if let Operand::DataRegister(reg) = dst {
                    let value = self.d[reg as usize];
                    let result = ((value & 0xFFFF) << 16) | ((value >> 16) & 0xFFFF);
                    self.d[reg as usize] = result;
                    let n = (result & 0x80000000) != 0;
                    let z = result == 0;
                    self.set_flags(n, z, false, false, false);
                }
                cycles = 4;
            }
            Operation::Cmpa => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(Size::Long, &dst);
                let result = dst_val.wrapping_sub(src_val);
                let n = (result & 0x80000000) != 0;
                let z = result == 0;
                let v = ((dst_val & !src_val & !result) | (!dst_val & src_val & result)) & 0x80000000 != 0;
                let c = ((!dst_val & src_val) | (result & (!dst_val | src_val))) & 0x80000000 != 0;
                self.set_flags(n, z, v, c, c);
                cycles = 6 + self.ea_cycles(&src, size);
            }
            Operation::Chk => {
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(Size::Word, &src) as i16;
                let dst_val = self.get_operand_value(Size::Word, &dst) as i16;
                if dst_val < 0 || dst_val > src_val {
                    return self.trigger_exception(6);
                }
                let n = dst_val < 0;
                let z = dst_val == 0;
                self.set_flags(n, z, false, false, false);
                cycles = 10 + self.ea_cycles(&src, Size::Word);
            }
            Operation::Tas => {
                let dst = instr.dst.unwrap();
                let val = self.get_operand_value(Size::Byte, &dst) as u8;
                let n = (val & 0x80) != 0;
                let z = val == 0;
                self.set_flags(n, z, false, false, false);
                self.set_operand_value(Size::Byte, &dst, (val | 0x80) as u32);
                cycles = if matches!(dst, Operand::DataRegister(_)) {
                    4
                } else {
                    14
                } + self.ea_cycles(&dst, Size::Byte);
            }
            Operation::Rol => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let shift_count = self.get_operand_value(Size::Long, &src) % size.bits();
                let dst_val = self.get_operand_value(size, &dst);
                let result = dst_val.rotate_left(shift_count);
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = if shift_count > 0 {
                    (dst_val & (1 << (size.bits() - shift_count))) != 0
                } else {
                    false
                };
                self.set_flags(n, z, false, c, c);
                cycles = match size {
                    Size::Byte | Size::Word => 6,
                    Size::Long => 8,
                } + 2 * shift_count as u32;
            }
            Operation::Roxl => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let shift_count = self.get_operand_value(Size::Long, &src) % (size.bits() + 1);
                let dst_val = self.get_operand_value(size, &dst);
                let x = (self.sr & 0x1) != 0;
                let shift = size.bits();
                let full_val = if x {
                    dst_val | (1 << shift)
                } else {
                    dst_val
                };
                let result = if shift_count == 0 {
                    dst_val
                } else {
                    full_val.rotate_left(shift_count) & size.mask()
                };
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (shift - 1))) != 0;
                let z = result == 0;
                let c = if shift_count > 0 {
                    (full_val & (1 << (shift + 1 - shift_count))) != 0
                } else {
                    false
                };
                self.set_flags(n, z, false, c, c);
                self.sr = (self.sr & 0xFFFE) | (if c {
                    1
                } else {
                    0
                });
                cycles = match size {
                    Size::Byte | Size::Word => 6,
                    Size::Long => 8,
                } + 2 * shift_count as u32;
            }
            Operation::Jsr => {
                let src = instr.src.unwrap();
                let addr = self.get_operand_value(Size::Long, &src);
                self.a[7] -= 4;
                self.cpu_write_long(self.a[7], self.pc);
                self.pc = addr;
                cycles = match src {
                    Operand::Displacement(_, _) => 18,
                    Operand::AbsoluteShort(_) => 18,
                    Operand::AbsoluteLong(_) => 20,
                    _ => 16,
                } + self.ea_cycles(&src, Size::Long);
            }
            Operation::Bchg => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let bit_num = self.get_operand_value(Size::Long, &src) % size.bits();
                let val = self.get_operand_value(size, &dst);
                let z = (val & (1 << bit_num)) == 0;
                let result = val ^ (1 << bit_num);
                self.set_operand_value(size, &dst, result);
                self.set_flags(false, z, false, false, false);
                cycles = if matches!(dst, Operand::DataRegister(_)) {
                    8
                } else {
                    12
                } + self.ea_cycles(&dst, size);
            }
            Operation::Bset => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let bit_num = self.get_operand_value(Size::Long, &src) % size.bits();
                let val = self.get_operand_value(size, &dst);
                let z = (val & (1 << bit_num)) == 0;
                let result = val | (1 << bit_num);
                self.set_operand_value(size, &dst, result);
                self.set_flags(false, z, false, false, false);
                cycles = if matches!(dst, Operand::DataRegister(_)) {
                    8
                } else {
                    12
                } + self.ea_cycles(&dst, size);
            }
            Operation::Btst => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let bit_num = self.get_operand_value(Size::Long, &src) % size.bits();
                let val = self.get_operand_value(size, &dst);
                let z = (val & (1 << bit_num)) == 0;
                self.set_flags(false, z, false, false, false);
                cycles = if matches!(dst, Operand::DataRegister(_)) {
                    6
                } else {
                    4
                } + self.ea_cycles(&dst, size);
            }
            Operation::Trap => {
                let vector = self.get_operand_value(Size::Long, &instr.src.unwrap());
                self.a[7] -= 4;
                self.cpu_write_long(self.a[7], self.pc);
                self.a[7] -= 2;
                self.cpu_write_word(self.a[7], self.sr);
                self.pc = self.cpu_read_long(32 + vector * 4);
                self.prefetch();
                cycles = 34;
            }
            Operation::And => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let result = src_val & dst_val;
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, false, false, false);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 6,
                } + self.ea_cycles(&src, size) + if matches!(dst, Operand::DataRegister(_)) {
                    0
                } else {
                    self.ea_cycles(&dst, size)
                };
            }
            Operation::Or => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let result = src_val | dst_val;
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, false, false, false);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 6,
                } + self.ea_cycles(&src, size) + if matches!(dst, Operand::DataRegister(_)) {
                    0
                } else {
                    self.ea_cycles(&dst, size)
                };
            }
            Operation::Addq => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let (result, carry, overflow) = match size {
                    Size::Byte => {
                        let s = src_val as u8;
                        let d = dst_val as u8;
                        let r = d.wrapping_add(s);
                        let c = (s as u16 + d as u16) > 0xFF;
                        let v = ((s & d & !r) | (!s & !d & r)) & 0x80 != 0;
                        (r as u32, c, v)
                    }
                    Size::Word => {
                        let s = src_val as u16;
                        let d = dst_val as u16;
                        let r = d.wrapping_add(s);
                        let c = (s as u32 + d as u32) > 0xFFFF;
                        let v = ((s & d & !r) | (!s & !d & r)) & 0x8000 != 0;
                        (r as u32, c, v)
                    }
                    Size::Long => {
                        let s = src_val;
                        let d = dst_val;
                        let r = d.wrapping_add(s);
                        let c = (s as u64 + d as u64) > 0xFFFFFFFF;
                        let v = ((s & d & !r) | (!s & !d & r)) & 0x80000000 != 0;
                        (r, c, v)
                    }
                };
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, overflow, carry, carry);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 8,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Subq => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let (result, carry, overflow) = match size {
                    Size::Byte => {
                        let s = src_val as u8;
                        let d = dst_val as u8;
                        let r = d.wrapping_sub(s);
                        let c = (s as u16) > (d as u16);
                        let v = ((d & !s & !r) | (!d & s & r)) & 0x80 != 0;
                        (r as u32, c, v)
                    }
                    Size::Word => {
                        let s = src_val as u16;
                        let d = dst_val as u16;
                        let r = d.wrapping_sub(s);
                        let c = (s as u32) > (d as u32);
                        let v = ((d & !s & !r) | (!d & s & r)) & 0x8000 != 0;
                        (r as u32, c, v)
                    }
                    Size::Long => {
                        let s = src_val;
                        let d = dst_val;
                        let r = d.wrapping_sub(s);
                        let c = (s as u64) > (d as u64);
                        let v = ((d & !s & !r) | (!d & s & r)) & 0x80000000 != 0;
                        (r, c, v)
                    }
                };
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, overflow, carry, carry);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 8,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Not => {
                let size = instr.size.unwrap();
                let dst = instr.dst.unwrap();
                let val = self.get_operand_value(size, &dst);
                let result = (!val) & size.mask();
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, false, false, false);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 6,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Divs => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let divisor = self.get_operand_value(size, &src) as i16 as i32;
                let dividend = self.get_operand_value(Size::Long, &dst) as i32;
                if divisor == 0 {
                    return self.trigger_exception(5);
                }
                let quotient = dividend / divisor;
                let remainder = dividend % divisor;
                let result = ((remainder as u32) << 16) | (quotient as u32 & 0xFFFF);
                self.set_operand_value(Size::Long, &dst, result);
                let n = (quotient & 0x8000) != 0;
                let z = quotient == 0;
                let v = quotient > 0x7FFF || quotient < -0x8000;
                self.set_flags(n, z, v, false, false);
                cycles = 96;
                if dividend != 0 && divisor != 0 {
                    let quotient_abs = quotient.abs() as u32;
                    let quotient_bits = 32 - quotient_abs.leading_zeros();
                    let shift_count = if quotient_bits > 0 {
                        quotient_bits - 1
                    } else {
                        0
                    };
                    cycles += 2 * shift_count;
                    if dividend < 0 {
                        cycles += 2;
                    }
                    if divisor < 0 {
                        cycles += 2;
                    }
                    if quotient_abs == 0 && (dividend < 0 || divisor < 0) {
                        cycles += 4;
                    }
                }
                cycles += self.ea_cycles(&src, size);
            }
            Operation::Roxr => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let shift_count = self.get_operand_value(Size::Long, &src) % (size.bits() + 1);
                let dst_val = self.get_operand_value(size, &dst);
                let x = (self.sr & 0x1) != 0;
                let shift = size.bits();
                let full_val = if x {
                    dst_val | (1 << shift)
                } else {
                    dst_val
                };
                let result = if shift_count == 0 {
                    dst_val
                } else {
                    full_val.rotate_right(shift_count) & size.mask()
                };
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (shift - 1))) != 0;
                let z = result == 0;
                let c = if shift_count > 0 {
                    (full_val & (1 << (shift_count - 1))) != 0
                } else {
                    false
                };
                self.set_flags(n, z, false, c, c);
                self.sr = (self.sr & 0xFFFE) | (if c {
                    1
                } else {
                    0
                });
                cycles = match size {
                    Size::Byte | Size::Word => 6,
                    Size::Long => 8,
                } + 2 * shift_count as u32;
            }
            Operation::Bhi => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                let z = (self.sr & 0x4) != 0;
                let c = (self.sr & 0x1) != 0;
                if !c && !z {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Bls => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                let z = (self.sr & 0x4) != 0;
                let c = (self.sr & 0x1) != 0;
                if c || z {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Jmp => {
                let src = instr.src.unwrap();
                let addr = self.get_operand_value(Size::Long, &src);
                self.pc = addr;
                self.prefetch();
                cycles = match src {
                    Operand::Displacement(_, _) => 10,
                    Operand::AbsoluteShort(_) => 10,
                    Operand::AbsoluteLong(_) => 12,
                    _ => 8,
                } + self.ea_cycles(&src, Size::Long);
            }
            Operation::Adda => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src) as i32 as u32;
                let dst_val = self.get_operand_value(Size::Long, &dst);
                let result = dst_val.wrapping_add(src_val);
                self.set_operand_value(Size::Long, &dst, result);
                cycles = match size {
                    Size::Word => 8,
                    Size::Long => 6,
                    Size::Byte => 8,
                } + self.ea_cycles(&src, size);
            }
            Operation::Suba => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src) as i32 as u32;
                let dst_val = self.get_operand_value(Size::Long, &dst);
                let result = dst_val.wrapping_sub(src_val);
                self.set_operand_value(Size::Long, &dst, result);
                cycles = match size {
                    Size::Word => 8,
                    Size::Long => 6,
                    Size::Byte => 8,
                } + self.ea_cycles(&src, size);
            }
            Operation::Mulu => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src) as u16 as u32;
                let dst_val = self.get_operand_value(size, &dst) as u16 as u32;
                let result = src_val * dst_val;
                self.set_operand_value(Size::Long, &dst, result);
                let n = (result & 0x80000000) != 0;
                let z = result == 0;
                self.set_flags(n, z, false, false, false);
                let ones = src_val.count_ones();
                cycles = 38 + 2 * ones + self.ea_cycles(&src, size);
            }
            Operation::Cmp => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let result = dst_val.wrapping_sub(src_val);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let v = ((dst_val & !src_val & !result) | (!dst_val & src_val & result)) & (1 << (size.bits() - 1)) != 0;
                let c = ((!dst_val & src_val) | (result & (!dst_val | src_val))) & (1 << (size.bits() - 1)) != 0;
                self.set_flags(n, z, v, c, c);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 6,
                } + self.ea_cycles(&src, size);
            }
            Operation::Scc => {
                let dst = instr.dst.unwrap();
                let carry = (self.sr & 0x1) != 0;
                let value = if !carry {
                    0xFF
                } else {
                    0x00
                };
                self.set_operand_value(Size::Byte, &dst, value as u32);
                cycles = if matches!(dst, Operand::DataRegister(_)) {
                    4
                } else {
                    8
                } + self.ea_cycles(&dst, Size::Byte);
            }
            Operation::Dbcc => {
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let dn_val = self.get_operand_value(Size::Word, &src);
                let disp = self.get_operand_value(Size::Long, &dst) as i32;
                let new_val = dn_val.wrapping_sub(1) & 0xFFFF;
                self.set_operand_value(Size::Word, &src, new_val);
                let c = (self.sr & 0x1) != 0;
                if !c && new_val != 0xFFFF {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 14;
                }
            }
            Operation::Pea => {
                let src = instr.src.unwrap();
                let addr = match src {
                    Operand::Displacement(reg, disp) => self.a[reg as usize].wrapping_add(disp as u32),
                    Operand::AbsoluteShort(addr) => addr,
                    Operand::AbsoluteLong(addr) => addr,
                    Operand::PCDisplacement(disp) => self.pc.wrapping_add(disp as u32 - 2),
                    _ => panic!("Unsupported addressing mode for PEA: {:?}", src),
                };
                self.a[7] -= 4;
                self.cpu_write_long(self.a[7], addr);
                cycles = match src {
                    Operand::Displacement(_, _) => 12,
                    Operand::AbsoluteShort(_) => 12,
                    Operand::AbsoluteLong(_) => 16,
                    _ => 12,
                } + self.ea_cycles(&src, Size::Long);
            }
            Operation::Link => {
                let an = instr.dst.unwrap();
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                if let Operand::AddressRegister(reg) = an {
                    self.a[7] -= 4;
                    self.cpu_write_long(self.a[7], self.a[reg as usize]);
                    self.a[reg as usize] = self.a[7];
                    self.a[7] = (self.a[7] as i32 + disp) as u32;
                }
                cycles = 16;
            }
            Operation::Unlk => {
                let an = instr.dst.unwrap();
                if let Operand::AddressRegister(reg) = an {
                    self.a[7] = self.a[reg as usize];
                    self.a[reg as usize] = self.cpu_read_long(self.a[7]);
                    self.a[7] += 4;
                }
                cycles = 12;
            }
            Operation::NopAlt => {
                cycles = 4;
            }
            Operation::Bgt => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                let z = (self.sr & 0x4) != 0;
                let n = (self.sr & 0x8) != 0;
                let v = (self.sr & 0x2) != 0;
                if !z && (n == v) {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Ble => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                let z = (self.sr & 0x4) != 0;
                let n = (self.sr & 0x8) != 0;
                let v = (self.sr & 0x2) != 0;
                if z || (n != v) {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Exg => {
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                if let (Operand::DataRegister(rx), Operand::DataRegister(ry)) = (src, dst) {
                    let temp = self.d[rx as usize];
                    self.d[rx as usize] = self.d[ry as usize];
                    self.d[ry as usize] = temp;
                }
                cycles = 6;
            }
            Operation::Movem => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let reglist = self.get_operand_value(Size::Long, &src) as u16;
                let mut addr = match dst {
                    Operand::Indirect(reg) => self.a[reg as usize],
                    Operand::Displacement(reg, disp) => self.a[reg as usize].wrapping_add(disp as u32),
                    _ => match src {
                        Operand::Indirect(reg) => self.a[reg as usize],
                        Operand::Displacement(reg, disp) => self.a[reg as usize].wrapping_add(disp as u32),
                        _ => panic!("Unsupported MOVEM addressing mode"),
                    },
                };
                cycles = 12 + self.ea_cycles(&src, size);
                let mut bit = 0;
                let regs = reglist.count_ones();
                if let Operand::Immediate(_) = src {
                    for i in 0..16 {
                        if (reglist & (1 << bit)) != 0 {
                            let value = if i < 8 {
                                self.d[i]
                            } else {
                                self.a[i - 8]
                            };
                            if size == Size::Word {
                                self.cpu_write_word(addr, value as u16);
                                addr += 2;
                                cycles += 4;
                            } else {
                                self.cpu_write_long(addr, value);
                                addr += 4;
                                cycles += 8;
                            }
                        }
                        bit += 1;
                    }
                } else {
                    for i in 0..16 {
                        if (reglist & (1 << bit)) != 0 {
                            let value = if size == Size::Word {
                                let val = self.cpu_read_word(addr) as u32;
                                addr += 2;
                                cycles += 4;
                                val
                            } else {
                                let val = self.cpu_read_long(addr);
                                addr += 4;
                                cycles += 8;
                                val
                            };
                            if i < 8 {
                                self.d[i] = value;
                            } else {
                                self.a[i - 8] = value;
                            }
                        }
                        bit += 1;
                    }
                }
                cycles = 12 + (if size == Size::Word {
                    4
                } else {
                    8
                } * regs);
            }
            Operation::AslMem => {
                let size = instr.size.unwrap();
                let dst = instr.dst.unwrap();
                let dst_val = self.get_operand_value(size, &dst);
                let result = (dst_val << 1) & size.mask();
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = (dst_val & (1 << (size.bits() - 1))) != 0;
                let v = (dst_val & (1 << (size.bits() - 1))) != (result & (1 << (size.bits() - 1)));
                self.set_flags(n, z, v, c, c);
                cycles = 8 + self.ea_cycles(&dst, size);
            }
            Operation::LsrMem => {
                let size = instr.size.unwrap();
                let dst = instr.dst.unwrap();
                let dst_val = self.get_operand_value(size, &dst);
                let result = dst_val >> 1;
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = (dst_val & 1) != 0;
                self.set_flags(n, z, false, c, c);
                cycles = 8 + self.ea_cycles(&dst, size);
            }
            Operation::ClrMem => {
                let size = instr.size.unwrap();
                let dst = instr.dst.unwrap();
                self.set_operand_value(size, &dst, 0);
                self.set_flags(false, true, false, false, false);
                cycles = match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 6,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Stop => {
                if !self.check_supervisor() {
                    cycles = 34;
                } else {
                    let data = self.get_operand_value(Size::Word, &instr.src.unwrap()) as u16;
                    self.sr = data;
                    self.halted = true;
                    cycles = 4;
                }
            }
            Operation::Rtd => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                self.pc = self.cpu_read_long(self.a[7]);
                self.a[7] += 4 + disp as u32;
                self.prefetch();
                cycles = 16;
            }
            Operation::TasMem => {
                let dst = instr.dst.unwrap();
                let val = self.get_operand_value(Size::Byte, &dst) as u8;
                let n = (val & 0x80) != 0;
                let z = val == 0;
                self.set_flags(n, z, false, false, false);
                self.set_operand_value(Size::Byte, &dst, (val | 0x80) as u32);
                cycles = if matches!(dst, Operand::DataRegister(_)) {
                    4
                } else {
                    14
                } + self.ea_cycles(&dst, Size::Byte);
            }
            Operation::Bcc => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                if (self.sr & 0x1) == 0 {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Bcs => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                if (self.sr & 0x1) != 0 {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Bge => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                let n = (self.sr & 0x8) != 0;
                let v = (self.sr & 0x2) != 0;
                if n == v {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Blt => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                let n = (self.sr & 0x8) != 0;
                let v = (self.sr & 0x2) != 0;
                if n != v {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Bmi => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                if (self.sr & 0x8) != 0 {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Bpl => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                if (self.sr & 0x8) == 0 {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Lsl => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let shift_count = self.get_operand_value(Size::Long, &src) % size.bits();
                let dst_val = self.get_operand_value(size, &dst);
                let result = (dst_val << shift_count) & size.mask();
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = if shift_count > 0 {
                    (dst_val & (1 << (size.bits() - shift_count))) != 0
                } else {
                    false
                };
                self.set_flags(n, z, false, c, c);
                cycles = match size {
                    Size::Byte | Size::Word => 6,
                    Size::Long => 8,
                } + 2 * shift_count as u32;
            }
            Operation::RolMem => {
                let size = instr.size.unwrap();
                let dst = instr.dst.unwrap();
                let dst_val = self.get_operand_value(size, &dst);
                let result = dst_val.rotate_left(1) & size.mask();
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = (dst_val & (1 << (size.bits() - 1))) != 0;
                self.set_flags(n, z, false, c, c);
                cycles = 8 + self.ea_cycles(&dst, size);
            }
            Operation::RorMem => {
                let size = instr.size.unwrap();
                let dst = instr.dst.unwrap();
                let dst_val = self.get_operand_value(size, &dst);
                let result = dst_val.rotate_right(1) & size.mask();
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = (dst_val & 1) != 0;
                self.set_flags(n, z, false, c, c);
                cycles = 8 + self.ea_cycles(&dst, size);
            }
            Operation::Subi => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let imm = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let (result, carry, overflow) = match size {
                    Size::Byte => {
                        let s = imm as u8;
                        let d = dst_val as u8;
                        let r = d.wrapping_sub(s);
                        let c = s > d;
                        let v = ((d & !s & !r) | (!d & s & r)) & 0x80 != 0;
                        (r as u32, c, v)
                    }
                    Size::Word => {
                        let s = imm as u16;
                        let d = dst_val as u16;
                        let r = d.wrapping_sub(s);
                        let c = s > d;
                        let v = ((d & !s & !r) | (!d & s & r)) & 0x8000 != 0;
                        (r as u32, c, v)
                    }
                    Size::Long => {
                        let s = imm;
                        let d = dst_val;
                        let r = d.wrapping_sub(s);
                        let c = s > d;
                        let v = ((d & !s & !r) | (!d & s & r)) & 0x80000000 != 0;
                        (r, c, v)
                    }
                };
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, overflow, carry, carry);
                cycles = match size {
                    Size::Byte | Size::Word => 8,
                    Size::Long => 14,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Abcd => {
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(Size::Byte, &src) as u8;
                let dst_val = self.get_operand_value(Size::Byte, &dst) as u8;
                let x = (self.sr & 0x1) != 0;
                let tens = ((src_val >> 4) & 0xF) + ((dst_val >> 4) & 0xF) + (if x {
                    1
                } else {
                    0
                });
                let units = (src_val & 0xF) + (dst_val & 0xF);
                let mut result = (units % 10) | (((tens + (units / 10)) % 10) << 4);
                let c = tens > 9 || units > 9;
                if result == 0 {
                    result = 0x00;
                }
                self.set_operand_value(Size::Byte, &dst, result as u32);
                let n = (result & 0x80) != 0;
                let z = if self.sr & 0x4 != 0 && result == 0 {
                    true
                } else {
                    false
                };
                self.set_flags(n, z, false, c, c);
                cycles = if matches!(src, Operand::DataRegister(_)) {
                    6
                } else {
                    18
                };
            }
            Operation::Sbcd => {
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let src_val = self.get_operand_value(Size::Byte, &src) as u8;
                let dst_val = self.get_operand_value(Size::Byte, &dst) as u8;
                let x = (self.sr & 0x1) != 0;
                let src_dec = ((src_val >> 4) & 0xF) * 10 + (src_val & 0xF);
                let dst_dec = ((dst_val >> 4) & 0xF) * 10 + (dst_val & 0xF);
                let diff = (dst_dec as i16) - (src_dec as i16) - (if x {
                    1
                } else {
                    0
                });
                let result = if diff < 0 {
                    (100 + diff) as u8
                } else {
                    diff as u8
                };
                let result_bcd = ((result / 10) << 4) | (result % 10);
                self.set_operand_value(Size::Byte, &dst, result_bcd as u32);
                let n = (result_bcd & 0x80) != 0;
                let z = if self.sr & 0x4 != 0 && result_bcd == 0 {
                    true
                } else {
                    false
                };
                let c = diff < 0;
                self.set_flags(n, z, false, c, c);
                cycles = if matches!(src, Operand::DataRegister(_)) {
                    6
                } else {
                    18
                };
            }
            Operation::Nbcd => {
                let dst = instr.dst.unwrap();
                let dst_val = self.get_operand_value(Size::Byte, &dst) as u8;
                let x = (self.sr & 0x1) != 0;
                let dec_val = ((dst_val >> 4) & 0xF) * 10 + (dst_val & 0xF);
                let result_dec: i16 = (0 - dec_val - (if x {
                    1
                } else {
                    0
                })).into();
                let result = if result_dec < 0 {
                    (100 + result_dec) as u8
                } else {
                    result_dec as u8
                };
                let result_bcd = ((result / 10) << 4) | (result % 10);
                self.set_operand_value(Size::Byte, &dst, result_bcd as u32);
                let n = (result_bcd & 0x80) != 0;
                let z = if self.sr & 0x4 != 0 && result_bcd == 0 {
                    true
                } else {
                    false
                };
                let c = result_dec != 0;
                self.set_flags(n, z, false, c, c);
                cycles = if matches!(dst, Operand::DataRegister(_)) {
                    6
                } else {
                    8
                };
            }
            Operation::Addi => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let imm = self.get_operand_value(size, &src);
                let dst_val = self.get_operand_value(size, &dst);
                let (result, carry, overflow) = match size {
                    Size::Byte => {
                        let s = imm as u8;
                        let d = dst_val as u8;
                        let r = d.wrapping_add(s);
                        let c = (s as u16 + d as u16) > 0xFF;
                        let v = ((s & d & !r) | (!s & !d & r)) & 0x80 != 0;
                        (r as u32, c, v)
                    }
                    Size::Word => {
                        let s = imm as u16;
                        let d = dst_val as u16;
                        let r = d.wrapping_add(s);
                        let c = (s as u32 + d as u32) > 0xFFFF;
                        let v = ((s & d & !r) | (!s & !d & r)) & 0x8000 != 0;
                        (r as u32, c, v)
                    }
                    Size::Long => {
                        let s = imm;
                        let d = dst_val;
                        let r = d.wrapping_add(s);
                        let c = (s as u64 + d as u64) > 0xFFFFFFFF;
                        let v = ((s & d & !r) | (!s & !d & r)) & 0x80000000 != 0;
                        (r, c, v)
                    }
                };
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                self.set_flags(n, z, overflow, carry, carry);
                cycles = match size {
                    Size::Byte | Size::Word => 8,
                    Size::Long => 14,
                } + self.ea_cycles(&dst, size);
            }
            Operation::Bvc => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                if (self.sr & 0x2) == 0 {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::Bvs => {
                let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
                if (self.sr & 0x2) != 0 {
                    self.pc = (self.pc as i32 + disp) as u32;
                    self.prefetch();
                    cycles = 10;
                } else {
                    cycles = 12;
                }
            }
            Operation::RoxlMem => {
                let size = instr.size.unwrap();
                let dst = instr.dst.unwrap();
                let dst_val = self.get_operand_value(size, &dst);
                let x = (self.sr & 0x1) != 0;
                let shift = size.bits();
                let full_val = if x {
                    dst_val | (1 << shift)
                } else {
                    dst_val
                };
                let result = full_val.rotate_left(1) & size.mask();
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = (dst_val & (1 << (size.bits() - 1))) != 0;
                self.set_flags(n, z, false, c, c);
                self.sr = (self.sr & 0xFFFE) | (if c {
                    1
                } else {
                    0
                });
                cycles = 8 + self.ea_cycles(&dst, size);
            }
            Operation::RoxrMem => {
                let size = instr.size.unwrap();
                let dst = instr.dst.unwrap();
                let dst_val = self.get_operand_value(size, &dst);
                let x = (self.sr & 0x1) != 0;
                let shift = size.bits();
                let full_val = if x {
                    dst_val | (1 << shift)
                } else {
                    dst_val
                };
                let result = full_val.rotate_right(1) & size.mask();
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = (dst_val & 1) != 0;
                self.set_flags(n, z, false, c, c);
                self.sr = (self.sr & 0xFFFE) | (if c {
                    1
                } else {
                    0
                });
                cycles = 8 + self.ea_cycles(&dst, size);
            }
            Operation::Trapv => {
                if (self.sr & 0x2) != 0 {
                    cycles = self.trigger_exception(7);
                } else {
                    cycles = 4;
                }
            }
            Operation::Reset => {
                if !self.check_supervisor() {
                    cycles = 34;
                } else {
                    self.pc = self.cpu_read_long(4);
                    self.sr = 0x2700;
                    self.d = [0; 8];
                    self.a = [0; 8];
                    self.a[7] = self.cpu_read_long(0);
                    self.prefetch();
                    cycles = 132;
                }
            }
            Operation::Rte => {
                if !self.check_supervisor() {
                    cycles = 34;
                } else {
                    self.sr = self.cpu_read_word(self.a[7]);
                    self.a[7] += 2;
                    self.pc = self.cpu_read_long(self.a[7]);
                    self.a[7] += 4;
                    self.interrupt_nest_level = self.interrupt_nest_level.saturating_sub(1);
                    self.prefetch();
                    cycles = 20;
                }
            }
            Operation::Movea => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let value = self.get_operand_value(size, &src);
                self.set_operand_value(Size::Long, &dst, value);
                cycles = match size {
                    Size::Word => 4,
                    Size::Long => 4,
                    Size::Byte => 4,
                } + self.ea_cycles(&src, size);
            }
            Operation::MoveCcr => {
                let src = instr.src.unwrap();
                let value = self.get_operand_value(Size::Byte, &src) as u16;
                self.sr = (self.sr & 0xFF00) | (value & 0x1F);
                cycles = 12 + self.ea_cycles(&src, Size::Byte);
            }
            Operation::MoveSr => {
                if !self.check_supervisor() {
                    cycles = 34;
                } else {
                    let src = instr.src.unwrap();
                    let value = self.get_operand_value(Size::Word, &src) as u16;
                    self.sr = value;
                    cycles = 12 + self.ea_cycles(&src, Size::Word);
                }
            }
            Operation::MoveUsp => {
                if !self.check_supervisor() {
                    cycles = 34;
                } else {
                    let src = instr.src.unwrap();
                    let dst = instr.dst.unwrap();
                    if let Operand::AddressRegister(an) = src {
                        self.a[7] = self.a[an as usize];
                    } else if let Operand::AddressRegister(an) = dst {
                        self.a[an as usize] = self.a[7];
                    }
                    cycles = 4;
                }
            }
            Operation::AndiCcr => {
                if !self.check_supervisor() {
                    cycles = 34;
                } else {
                    let src = instr.src.unwrap();
                    let value = self.get_operand_value(Size::Byte, &src) as u16;
                    self.sr = (self.sr & 0xFF00) | ((self.sr & 0x1F) & (value & 0x1F));
                    cycles = 20;
                }
            }
            Operation::OriCcr => {
                if !self.check_supervisor() {
                    cycles = 34;
                } else {
                    let src = instr.src.unwrap();
                    let value = self.get_operand_value(Size::Byte, &src) as u16;
                    self.sr = (self.sr & 0xFF00) | ((self.sr & 0x1F) | (value & 0x1F));
                    cycles = 20;
                }
            }
            Operation::EoriCcr => {
                if !self.check_supervisor() {
                    cycles = 34;
                } else {
                    let src = instr.src.unwrap();
                    let value = self.get_operand_value(Size::Byte, &src) as u16;
                    self.sr = (self.sr & 0xFF00) | ((self.sr & 0x1F) ^ (value & 0x1F));
                    cycles = 20;
                }
            }
            Operation::LslMem => {
                let size = instr.size.unwrap();
                let dst = instr.dst.unwrap();
                let dst_val = self.get_operand_value(size, &dst);
                let result = (dst_val << 1) & size.mask();
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = (dst_val & (1 << (size.bits() - 1))) != 0;
                self.set_flags(n, z, false, c, c);
                cycles = 8 + self.ea_cycles(&dst, size);
            }
            Operation::Asr => {
                let size = instr.size.unwrap();
                let src = instr.src.unwrap();
                let dst = instr.dst.unwrap();
                let shift_count = self.get_operand_value(Size::Long, &src) % size.bits();
                let dst_val = self.get_operand_value(size, &dst) as i32;
                let result = match size {
                    Size::Byte => (dst_val as i8 >> shift_count) as u32 & 0xFF,
                    Size::Word => (dst_val as i16 >> shift_count) as u32 & 0xFFFF,
                    Size::Long => (dst_val >> shift_count) as u32,
                };
                self.set_operand_value(size, &dst, result);
                let n = (result & (1 << (size.bits() - 1))) != 0;
                let z = result == 0;
                let c = if shift_count > 0 {
                    (dst_val >> (shift_count - 1)) & 1 != 0
                } else {
                    false
                };
                self.set_flags(n, z, false, c, c);
                cycles = match size {
                    Size::Byte | Size::Word => 6,
                    Size::Long => 8,
                } + 2 * shift_count as u32;
            }
        }
        cycles
    }

    pub fn step(&mut self) -> u32 {
        if self.halted && self.pending_interrupts.is_empty() {
            return 0;
        }
        if let Some(&(level, vector)) = self.pending_interrupts.first() {
            let current_ipl = (self.sr >> 8) & 0x7;
            if u16::from(level) > current_ipl || (level == 7 && self.interrupt_nest_level < 7) {
                let cycles = self.process_interrupt(level, vector);
                self.cycle_count += cycles as u64;
                return cycles;
            }
        }
        let instr = self.decode();
        let cycles = self.execute(instr);
        self.cycle_count += cycles as u64;
        self.interrupt_ack = None;
        // Removed the extra prefetch() call to avoid advancing the PC beyond what decode/fetch_word already does.
        // self.prefetch();
        cycles
    }

    // Modified load_program: if the destination is in ROM, use load_rom_data
    pub fn load_program(&mut self, address: u32, program: &[u8]) {
        if address < 0x400000 {
            // Load directly into ROM data (for initialization)
            self.memory.load_rom_data(address, program)
                .expect("Failed to load program into ROM");
        } else {
            for (i, &byte) in program.iter().enumerate() {
                self.cpu_write_byte(address + i as u32, byte);
            }
        }
        self.pc = address;
        self.prefetch();
    }
}
