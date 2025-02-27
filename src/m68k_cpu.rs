/// Motorola 68k CPU Emulator in Rust ('m68k_cpu.rs')
/// Aims for 100% hardware and timing accuracy based on the M68000 Programmer's Reference Manual

// Define the data size for operations
#[derive(Debug, Clone, Copy)]
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

// Define operand types for addressing modes
#[derive(Debug, Clone)]
pub enum Operand {
    DataRegister(u8),                    // Dn
    AddressRegister(u8),                 // An
    Indirect(u8),                        // (An)
    PostInc(u8),                         // (An)+
    PreDec(u8),                          // -(An)
    Displacement(u8, i16),               // d(An)
    Indexed(u8, i8, u8, Size),           // d(An, Rn.size)
    AbsoluteShort(u32),                  // Absolute short address
    AbsoluteLong(u32),                   // Absolute long address
    PCDisplacement(i16),                 // d(PC)
    PCIndexed(i8, u8, Size),             // d(PC, Rn.size)
    Immediate(u32),                      // #imm
}

// Define instruction operations
#[derive(Debug)]
pub enum Operation {
	Nop,    // No operation
	Rts,    // Return from subroutine
	Move,   // Move data
	Add,    // Add
	Muls,   // Multiply signed
	Divu,   // Divide unsigned
	Eor,    // Exclusive OR
	Neg,    // Negate
	Tst,    // Test operand
	Lsr,    // Logical shift right
	Asl,    // Arithmetic shift left
	Moveq,  // Move quick
	Ror,    // Rotate right
	Eori,   // Exclusive OR immediate
	Cmpi,   // Compare immediate
	Movep,  // Move peripheral data
	Bclr,   // Bit clear
	Lea,    // Load effective address
	Ext,    // Sign-extend
	Sub,    // Subtract
	Andi,   // AND immediate
	Ori,    // OR immediate
	Addx,   // Add with extend
	Subx,   // Subtract with extend
	Bra,    // Branch always
	Beq,    // Branch if equal
	Bne,    // Branch if not equal
	Clr,    // Clear
	Swap,   // Swap register halves
	Cmpa,   // Compare address
	Chk,    // Check register against bounds
	Tas,    // Test and set
	Rol,    // Rotate left
	Roxl,   // Rotate with extend left
	Jsr,    // Jump to subroutine
	Bchg,   // Bit change
	Bset,   // Bit set
	Btst,   // Bit test
	Trap,   // Trap
	And,    // AND
	Or,     // OR
	Addq,   // Add quick
	Subq,   // Subtract quick
	Not,    // NOT
	Divs,   // Divide signed
	Roxr,   // Rotate with extend right
	Bhi,    // Branch if higher
	Bls,    // Branch if lower or same
	Jmp,    // Jump
	Adda,   // Add address
	Suba,   // Subtract address
	Mulu,   // Multiply unsigned
	Cmp,    // Compare
	Scc,    // Set condition code
	Dbcc,   // Decrement and branch if condition
	Pea,    // Push effective address
	Link,   // Link and allocate
	Unlk,   // Unlink
	NopAlt, // Alternative NOP for distinction
	Bgt,    // Branch if greater than
	Ble,    // Branch if less than or equal
	Exg,    // Exchange registers
	Movem,  // Move multiple registers
	AslMem, // Arithmetic shift left (memory)
	LsrMem, // Logical shift right (memory)
	ClrMem, // Clear (memory)
	Stop,   // Stop
	Rtd,    // Return and deallocate
	TasMem, // Test and set (memory)
	Bcc,    // Branch if carry clear
	Bcs,    // Branch if carry set
	Bge,    // Branch if greater or equal
	Blt,    // Branch if less than
	Bmi,    // Branch if minus
	Bpl,    // Branch if plus
	Lsl,    // Logical shift left
	RolMem, // Rotate left (memory)
	RorMem, // Rotate right (memory)
	Subi,   // Subtract immediate
	Abcd,   // Add binary-coded decimal
	Sbcd,   // Subtract binary-coded decimal
	Nbcd,   // Negate binary-coded decimal
	Addi,   // Add immediate
	Bvc,    // Branch if overflow clear
	Bvs,    // Branch if overflow set
	RoxlMem,// Rotate with extend left (memory)
	RoxrMem,// Rotate with extend right (memory)
	Trapv,  // Trap on overflow
	Reset,  // Reset
	Rte,    // Return from exception
	Movea,  // Move address
	MoveCcr,// Move to/from condition code register
	MoveSr, // Move to/from status register
	MoveUsp,// Move to/from user stack pointer
	AndiCcr,// AND immediate with condition code register
	OriCcr, // OR immediate with condition code register
	EoriCcr,// Exclusive OR immediate with condition code register
	LslMem, // Logical shift left (memory)
	Asr,    // Arithmetic shift right
}

// Struct to represent a decoded instruction
#[derive(Debug)]
struct Instruction {
    operation: Operation,
    size: Option<Size>,
    src: Option<Operand>,
    dst: Option<Operand>,
}

// Memory implementation with big-endian byte order
pub struct Memory {
    data: Vec<u8>,
}

impl Memory {
    fn new(size: usize) -> Memory {
        Memory {
            data: vec![0; size], // 16 MB address space (24-bit addressing)
        }
    }

    fn read_byte(&self, address: u32) -> u8 {
        self.data[(address & 0xFFFFFF) as usize]
    }

    fn write_byte(&mut self, address: u32, value: u8) {
        self.data[(address & 0xFFFFFF) as usize] = value;
    }

    fn read_word(&self, address: u32) -> u16 {
        let addr = (address & 0xFFFFFF) as usize;
        ((self.data[addr] as u16) << 8) | (self.data[addr + 1] as u16)
    }

    fn write_word(&mut self, address: u32, value: u16) {
        let addr = (address & 0xFFFFFF) as usize;
        self.data[addr] = (value >> 8) as u8;
        self.data[addr + 1] = value as u8;
    }

    fn read_long(&self, address: u32) -> u32 {
        let addr = (address & 0xFFFFFF) as usize;
        ((self.read_word(address) as u32) << 16) | (self.read_word(address + 2) as u32)
    }

    fn write_long(&mut self, address: u32, value: u32) {
        let addr = (address & 0xFFFFFF) as usize;
        self.write_word(address, (value >> 16) as u16);
        self.write_word(address + 2, value as u16);
    }
}

// CPU state and operations
pub struct CPU {
    d: [u32; 8],         // Data registers D0-D7
    a: [u32; 8],         // Address registers A0-A7 (A7 is stack pointer)
    pc: u32,             // Program counter
    sr: u16,             // Status register (T S IPL XNZVC)
    memory: Memory,      // Memory
    cycle_count: u64,    // Total cycles for timing accuracy
    pending_interrupts: Vec<(u8, Option<u8>)>, // (level, vector): Queue of pending interrupts
    interrupt_ack: Option<u8>, // Level being acknowledged
    interrupt_nest_level: u8,  // Nesting depth (0 = no interrupt active)
    bus_state: BusState,       // Current bus state for interrupt acknowledgment
    halted: bool,              // CPU halted state (e.g., STOP instruction)
    prefetch_queue: [u16; 2],  // Two-word prefetch queue
}

#[derive(Debug, Clone, Copy)]
pub enum BusState {
    Idle,
    Iack(u8), // Interrupt acknowledge cycle with level
    Vpa,      // Valid Peripheral Address (auto-vectored)
    Dtack(u8), // Data Transfer Acknowledge (vectored) with vector
}

impl CPU {
    /// Initialize a new CPU with a given memory size
    pub fn new(memory_size: usize) -> CPU {
		let mut cpu = CPU {
			d: [0; 8],
			a: [0; 8],
			pc: 0,
			sr: 0x2700, // Supervisor mode, interrupt mask 7
			memory: Memory::new(memory_size),
			cycle_count: 0,
			pending_interrupts: Vec::new(),
			interrupt_ack: None,
			interrupt_nest_level: 0,
			bus_state: BusState::Idle,
			halted: false,
			prefetch_queue: [0; 2],
		};
		cpu.a[7] = (memory_size - 4) as u32; // Stack pointer at top of memory
		cpu.prefetch(); // Initial prefetch
		cpu
	}

    /// Prefetch two words into the queue
    fn prefetch(&mut self) {
        self.prefetch_queue[0] = self.memory.read_word(self.pc);
        self.prefetch_queue[1] = self.memory.read_word(self.pc + 2);
        self.pc += 4; // Advance PC past prefetched words
    }

    /// Fetch a word from the prefetch queue and refill
    fn fetch_word(&mut self) -> u16 {
        let word = self.prefetch_queue[0];
        self.prefetch_queue[0] = self.prefetch_queue[1];
        self.prefetch_queue[1] = self.memory.read_word(self.pc);
        self.pc += 2;
        word
    }

    /// Fetch a long word from the prefetch queue and refill
    fn fetch_long(&mut self) -> u32 {
        let high = self.fetch_word() as u32;
        let low = self.fetch_word() as u32;
        (high << 16) | low
    }

    /// Set status flags (Negative, Zero, Overflow, Carry)
    fn set_flags(&mut self, n: bool, z: bool, v: bool, c: bool) {
        let mut sr = self.sr & 0xFFF0; // Preserve non-flag bits
        if n { sr |= 0x8; } // Negative
        if z { sr |= 0x4; } // Zero
        if v { sr |= 0x2; } // Overflow
        if c { sr |= 0x1; } // Carry
        self.sr = sr;
    }

	/// Calculate cycle count for effective address calculation
    fn ea_cycles(&self, operand: &Operand, size: Size) -> u32 {
        match operand {
            Operand::DataRegister(_) => 0,
            Operand::AddressRegister(_) => 0,
            Operand::Indirect(reg) => match size {
                Size::Byte | Size::Word => 4,
                Size::Long => 8,
            },
            Operand::PostInc(reg) => {
                // A7 (SP) adjusts differently for byte size
                if *reg == 7 && size == Size::Byte { 6 } else { match size {
                    Size::Byte | Size::Word => 4,
                    Size::Long => 8,
                }}
            },
            Operand::PreDec(reg) => {
                if *reg == 7 && size == Size::Byte { 8 } else { match size {
                    Size::Byte | Size::Word => 6,
                    Size::Long => 10,
                }}
            },
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
                Size::Byte => 0, // Included in base instruction
                Size::Word => 4,
                Size::Long => 8,
            },
        }
    }

    /// Check if in supervisor mode, trigger Privilege Violation if not
    fn check_supervisor(&mut self) -> bool {
        if (self.sr & 0x2000) == 0 { // S bit (bit 13) clear = user mode
            self.a[7] -= 4;
            self.memory.write_long(self.a[7], self.pc); // Push PC
            self.a[7] -= 2;
            self.memory.write_word(self.a[7], self.sr); // Push SR
            self.pc = self.memory.read_long(32); // Vector 8: Privilege Violation
            false
        } else {
            true
        }
    }

	/// Request an interrupt with a given priority level (1-7) and optional vector
    pub fn request_interrupt(&mut self, level: u8, vector: Option<u8>) {
        if level > 7 || level == 0 { // Invalid levels
            return;
        }
        if let Some(v) = vector {
            if v < 2 || v > 255 { // Vectors 0-1 reserved, 2-255 valid
                return;
            }
        }
        // Add to queue, avoiding duplicates unless vector differs
        if !self.pending_interrupts.iter().any(|&(l, v)| l == level && v == vector) {
            self.pending_interrupts.push((level, vector));
            // Sort by level in descending order (highest priority first)
            self.pending_interrupts.sort_by(|a, b| b.0.cmp(&a.0));
        }
    }

	/// Process an interrupt with the given level and optional vector, return cycles taken
    fn process_interrupt(&mut self, level: u8, vector: Option<u8>) -> u32 {
        self.halted = false; // Resume from STOP on interrupt
        let mut cycles = 0;
        // Step 1: Initiate IACK cycle (7n clocks = 14 cycles)
        self.bus_state = BusState::Iack(level);
        self.interrupt_ack = Some(level);
        cycles += 14; // 7n: Bus arbitration and IACK start

        // Step 2: Determine vector and bus response
        let vec_addr = match vector {
            Some(v) => {
                self.bus_state = BusState::Dtack(v);
                cycles += 4;
                v as u32
            }
            None => {
                self.bus_state = BusState::Vpa;
                24 + level as u32 // Vectors 25-31
            }
        };

        // Step 3: Enter supervisor mode and update IPL
        self.sr |= 0x2000; // Set S bit
        self.sr = (self.sr & 0xF8FF) | ((level as u16) << 8); // Update IPL
        self.interrupt_nest_level += 1;

        // Step 4: Push SR and PC (5n = 10 cycles)
        self.a[7] -= 2;
        self.memory.write_word(self.a[7], self.sr);
        self.a[7] -= 4;
        self.memory.write_long(self.a[7], self.pc);
        cycles += 10;

        // Step 5: Fetch vector (2n = 4 cycles)
        self.pc = self.memory.read_long(vec_addr * 4);
        cycles += 4;

        // Step 6: Complete IACK cycle (1n = 2 cycles)
        self.bus_state = BusState::Idle;
        self.pending_interrupts.retain(|&(l, _)| l != level);
        self.prefetch(); // Refill prefetch queue after interrupt
        cycles
    }
	
	/// Trigger an exception with the given vector, return cycles taken
    fn trigger_exception(&mut self, vector: u32) -> u32 {
        self.sr |= 0x2000; // Enter supervisor mode
        self.a[7] -= 2;
        self.memory.write_word(self.a[7], self.sr); // Push SR
        self.a[7] -= 4;
        self.memory.write_long(self.a[7], self.pc); // Push PC
        self.pc = self.memory.read_long(vector * 4); // Vector address
        34 // Exception timing (17n)
    }

	/// Get the current interrupt acknowledgment level and bus state
    pub fn get_interrupt_ack(&self) -> (Option<u8>, BusState) {
        (self.interrupt_ack, self.bus_state)
    }

    /// Decode effective address based on mode and register
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
                let index_size = if (ext & 0x800) != 0 { Size::Long } else { Size::Word };
                Operand::Indexed(reg, disp, index_reg, index_size)
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
                    let index_size = if (ext & 0x800) != 0 { Size::Long } else { Size::Word };
                    Operand::PCIndexed(disp, index_reg, index_size)
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

    /// Get the value of an operand
    fn get_operand_value(&mut self, size: Size, operand: &Operand) -> u32 {
        match operand {
            Operand::DataRegister(reg) => self.d[*reg as usize] & size.mask(),
            Operand::AddressRegister(reg) => self.a[*reg as usize],
            Operand::Indirect(reg) => {
                let addr = self.a[*reg as usize];
                match size {
                    Size::Byte => self.memory.read_byte(addr) as u32,
                    Size::Word => self.memory.read_word(addr) as u32,
                    Size::Long => self.memory.read_long(addr),
                }
            }
            Operand::PostInc(reg) => {
                let addr = self.a[*reg as usize];
                let value = match size {
                    Size::Byte => self.memory.read_byte(addr) as u32,
                    Size::Word => self.memory.read_word(addr) as u32,
                    Size::Long => self.memory.read_long(addr),
                };
                self.a[*reg as usize] += size.bits() / 8;
                value
            }
            Operand::PreDec(reg) => {
                self.a[*reg as usize] -= size.bits() / 8;
                let addr = self.a[*reg as usize];
                match size {
                    Size::Byte => self.memory.read_byte(addr) as u32,
                    Size::Word => self.memory.read_word(addr) as u32,
                    Size::Long => self.memory.read_long(addr),
                }
            }
            Operand::Displacement(reg, disp) => {
                let addr = self.a[*reg as usize].wrapping_add(*disp as u32);
                match size {
                    Size::Byte => self.memory.read_byte(addr) as u32,
                    Size::Word => self.memory.read_word(addr) as u32,
                    Size::Long => self.memory.read_long(addr),
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
                    Size::Byte => self.memory.read_byte(addr) as u32,
                    Size::Word => self.memory.read_word(addr) as u32,
                    Size::Long => self.memory.read_long(addr),
                }
            }
            Operand::AbsoluteShort(addr) => match size {
                Size::Byte => self.memory.read_byte(*addr) as u32,
                Size::Word => self.memory.read_word(*addr) as u32,
                Size::Long => self.memory.read_long(*addr),
            },
            Operand::AbsoluteLong(addr) => match size {
                Size::Byte => self.memory.read_byte(*addr) as u32,
                Size::Word => self.memory.read_word(*addr) as u32,
                Size::Long => self.memory.read_long(*addr),
            },
            Operand::PCDisplacement(disp) => {
                let addr = self.pc.wrapping_add(*disp as u32 - 2); // -2 adjusts for prefetch
                match size {
                    Size::Byte => self.memory.read_byte(addr) as u32,
                    Size::Word => self.memory.read_word(addr) as u32,
                    Size::Long => self.memory.read_long(addr),
                }
            }
            Operand::PCIndexed(disp, idx_reg, idx_size) => {
                let base = self.pc - 2; // Adjust for prefetch
                let index = match idx_size {
                    Size::Word => (self.d[*idx_reg as usize] as i16) as i32,
                    Size::Long => self.d[*idx_reg as usize] as i32,
                    _ => unreachable!(),
                };
                let addr = base.wrapping_add(*disp as i32 as u32).wrapping_add(index as u32);
                match size {
                    Size::Byte => self.memory.read_byte(addr) as u32,
                    Size::Word => self.memory.read_word(addr) as u32,
                    Size::Long => self.memory.read_long(addr),
                }
            }
            Operand::Immediate(val) => *val & size.mask(),
        }
    }

    /// Set the value of an operand
    fn set_operand_value(&mut self, size: Size, operand: &Operand, value: u32) {
        match operand {
            Operand::DataRegister(reg) => self.d[*reg as usize] = value & size.mask(),
            Operand::AddressRegister(reg) => self.a[*reg as usize] = value,
            Operand::Indirect(reg) => {
                let addr = self.a[*reg as usize];
                match size {
                    Size::Byte => self.memory.write_byte(addr, value as u8),
                    Size::Word => self.memory.write_word(addr, value as u16),
                    Size::Long => self.memory.write_long(addr, value),
                }
            }
            Operand::PostInc(reg) => {
                let addr = self.a[*reg as usize];
                match size {
                    Size::Byte => self.memory.write_byte(addr, value as u8),
                    Size::Word => self.memory.write_word(addr, value as u16),
                    Size::Long => self.memory.write_long(addr, value),
                }
                self.a[*reg as usize] += size.bits() / 8;
            }
            Operand::PreDec(reg) => {
                self.a[*reg as usize] -= size.bits() / 8;
                let addr = self.a[*reg as usize];
                match size {
                    Size::Byte => self.memory.write_byte(addr, value as u8),
                    Size::Word => self.memory.write_word(addr, value as u16),
                    Size::Long => self.memory.write_long(addr, value),
                }
            }
            Operand::Displacement(reg, disp) => {
                let addr = self.a[*reg as usize].wrapping_add(*disp as u32);
                match size {
                    Size::Byte => self.memory.write_byte(addr, value as u8),
                    Size::Word => self.memory.write_word(addr, value as u16),
                    Size::Long => self.memory.write_long(addr, value),
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
                    Size::Byte => self.memory.write_byte(addr, value as u8),
                    Size::Word => self.memory.write_word(addr, value as u16),
                    Size::Long => self.memory.write_long(addr, value),
                }
            }
            Operand::AbsoluteShort(addr) => match size {
                Size::Byte => self.memory.write_byte(*addr, value as u8),
                Size::Word => self.memory.write_word(*addr, value as u16),
                Size::Long => self.memory.write_long(*addr, value),
            },
            Operand::AbsoluteLong(addr) => match size {
                Size::Byte => self.memory.write_byte(*addr, value as u8),
                Size::Word => self.memory.write_word(*addr, value as u16),
                Size::Long => self.memory.write_long(*addr, value),
            },
            _ => panic!("Cannot write to operand: {:?}", operand),
        }
    }

    /// Decode an instruction from memory
	fn decode(&mut self) -> Instruction {
		let opcode = self.fetch_word();
		match opcode {
			0x4E71 => Instruction {
				operation: Operation::Nop,
				size: None,
				src: None,
				dst: None,
			},
			0x4E75 => Instruction {
				operation: Operation::Rts,
				size: None,
				src: None,
				dst: None,
			},
			0x1000..=0x3FFF => { // MOVE.B, MOVE.W, MOVE.L
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
				let dst = self.decode_ea(dst_mode, dst_reg, size);
				Instruction {
					operation: Operation::Move,
					size: Some(size),
					src: Some(src),
					dst: Some(dst),
				}
			}
			0xD000..=0xDFFF => { // ADD <ea>, Dn or ADD Dn, <ea>
				let reg = ((opcode >> 9) & 0x7) as u8;
				let direction = (opcode >> 8) & 0x1;
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
				Instruction {
					operation: Operation::Add,
					size: Some(size),
					src: Some(src),
					dst: Some(dst),
				}
			}
			0xC1C0..=0xC1FF => { // MULS.W <ea>, Dn
				let reg = ((opcode >> 9) & 0x7) as u8;
				let src_mode = ((opcode >> 3) & 0x7) as u8;
				let src_reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(src_mode, src_reg, Size::Word);
				Instruction {
					operation: Operation::Muls,
					size: Some(Size::Word),
					src: Some(src),
					dst: Some(Operand::DataRegister(reg)),
				}
			}
			0x80C0..=0x80FF => { // DIVU.W <ea>, Dn
				let reg = ((opcode >> 9) & 0x7) as u8;
				let src_mode = ((opcode >> 3) & 0x7) as u8;
				let src_reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(src_mode, src_reg, Size::Word);
				Instruction {
					operation: Operation::Divu,
					size: Some(Size::Word),
					src: Some(src),
					dst: Some(Operand::DataRegister(reg)),
				}
			}
			0xB100..=0xB7FF => { // EOR.B, EOR.W, EOR.L Dn, <ea>
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let reg = ((opcode >> 9) & 0x7) as u8;
				let dst_mode = ((opcode >> 3) & 0x7) as u8;
				let dst_reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(dst_mode, dst_reg, size);
				Instruction {
					operation: Operation::Eor,
					size: Some(size),
					src: Some(Operand::DataRegister(reg)),
					dst: Some(dst),
				}
			}
			0x4400..=0x44FF => { // NEG.B, NEG.W, NEG.L <ea>
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let dst_mode = ((opcode >> 3) & 0x7) as u8;
				let dst_reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(dst_mode, dst_reg, size);
				Instruction {
					operation: Operation::Neg,
					size: Some(size),
					src: None,
					dst: Some(dst),
				}
			}
			0x4A00..=0x4AFF => { // TST.B, TST.W, TST.L <ea>
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let src_mode = ((opcode >> 3) & 0x7) as u8;
				let src_reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(src_mode, src_reg, size);
				Instruction {
					operation: Operation::Tst,
					size: Some(size),
					src: Some(src),
					dst: None,
				}
			}
			0xE008..=0xE07F => { // LSR.<size> Dx,Dy or #<data>,Dy
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let count = ((opcode >> 9) & 0x7) as u8;
				let dr = (opcode >> 5) & 0x1; // 0: immediate, 1: register
				let dst_reg = (opcode & 0x7) as u8;
				let src = if dr == 0 {
					Operand::Immediate(if count == 0 { 8 } else { count as u32 }) // 0 means 8 shifts
				} else {
					Operand::DataRegister(count)
				};
				Instruction {
					operation: Operation::Lsr,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::DataRegister(dst_reg)),
				}
			}
			0xE100..=0xE1FF => { // ASL.<size> Dx,Dy or #<data>,Dy
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let count_reg = ((opcode >> 9) & 0x7) as u8;
				let dr = (opcode >> 5) & 0x1; // 0: immediate, 1: register
				let dst_reg = (opcode & 0x7) as u8;
				let src = if dr == 0 {
					Operand::Immediate(if count_reg == 0 { 8 } else { count_reg as u32 })
				} else {
					Operand::DataRegister(count_reg)
				};
				Instruction {
					operation: Operation::Asl,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::DataRegister(dst_reg)),
				}
			}
			0x7000..=0x7FFF => { // MOVEQ.L #<data>, Dn
				let reg = ((opcode >> 9) & 0x7) as u8;
				let data = (opcode & 0xFF) as i8 as i32 as u32; // Sign-extended to 32 bits
				Instruction {
					operation: Operation::Moveq,
					size: Some(Size::Long),
					src: Some(Operand::Immediate(data)),
					dst: Some(Operand::DataRegister(reg)),
				}
			}
			0xE018..=0xE07F if (opcode & 0xFEE0) == 0xE018 => { // ROR.<size> Dx,Dy or #<data>,Dy
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let count = ((opcode >> 9) & 0x7) as u8;
				let dr = (opcode >> 5) & 0x1; // 0: immediate, 1: register
				let dst_reg = (opcode & 0x7) as u8;
				let src = if dr == 0 {
					Operand::Immediate(if count == 0 { 8 } else { count as u32 })
				} else {
					Operand::DataRegister(count)
				};
				Instruction {
					operation: Operation::Ror,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::DataRegister(dst_reg)),
				}
			}
			0x0A00..=0x0AFC => { // EORI #<data>,<ea>
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
				Instruction {
					operation: Operation::Eori,
					size: Some(size),
					src: Some(Operand::Immediate(immediate)),
					dst: Some(dst),
				}
			}
			0x0C00..=0x0CFC => { // CMPI #<data>,<ea>
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
				Instruction {
					operation: Operation::Cmpi,
					size: Some(size),
					src: Some(Operand::Immediate(immediate)),
					dst: Some(dst),
				}
			}
			0x0188..=0x07C8 if (opcode & 0xF138) == 0x0108 => { // MOVEP
				let direction = (opcode >> 7) & 0x1; // 0: memory to reg, 1: reg to memory
				let size = if (opcode >> 6) & 0x1 == 0 { Size::Word } else { Size::Long };
				let dreg = ((opcode >> 9) & 0x7) as u8;
				let areg = (opcode & 0x7) as u8;
				let displacement = self.fetch_word() as i16;
				let (src, dst) = if direction == 0 {
					let ea = self.decode_ea(5, areg, size); // Displacement mode
					(ea, Operand::DataRegister(dreg))
				} else {
					let ea = self.decode_ea(5, areg, size); // Displacement mode
					(Operand::DataRegister(dreg), ea)
				};
				Instruction {
					operation: Operation::Movep,
					size: Some(size),
					src: Some(src),
					dst: Some(dst),
				}
			}
			0x0880..=0x08FF => { // BCLR #<data>,<ea>
				let bit_number = self.fetch_word() as u8;
				let ea_mode = ((opcode >> 3) & 0x7) as u8;
				let ea_reg = (opcode & 0x7) as u8;
				let size = if ea_mode == 0 { Size::Long } else { Size::Byte }; // Long for Dn, Byte for memory
				let dst = self.decode_ea(ea_mode, ea_reg, size);
				Instruction {
					operation: Operation::Bclr,
					size: Some(size),
					src: Some(Operand::Immediate(bit_number as u32)),
					dst: Some(dst),
				}
			}
			0x41C0..=0x4FFF if (opcode & 0xF1C0) == 0x41C0 => { // LEA <ea>,An
				let areg = ((opcode >> 9) & 0x7) as u8;
				let ea_mode = ((opcode >> 3) & 0x7) as u8;
				let ea_reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(ea_mode, ea_reg, Size::Long);
				Instruction {
					operation: Operation::Lea,
					size: Some(Size::Long),
					src: Some(src),
					dst: Some(Operand::AddressRegister(areg)),
				}
			}
			0x4880..=0x48C0 if (opcode & 0xFF38) == 0x4800 => { // EXT.<size> Dn
				let size = if (opcode & 0x0040) == 0 { Size::Word } else { Size::Long };
				let reg = (opcode & 0x7) as u8;
				Instruction {
					operation: Operation::Ext,
					size: Some(size),
					src: None,
					dst: Some(Operand::DataRegister(reg)),
				}
			}
			// SUB.<size> <ea>,Dn or Dn,<ea>
			0x9000..=0x9FFF => {
				let reg = ((opcode >> 9) & 0x7) as u8;
				let direction = (opcode >> 8) & 0x1;
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
				Instruction {
					operation: Operation::Sub,
					size: Some(size),
					src: Some(src),
					dst: Some(dst),
				}
			}
			// ANDI.<size> #<data>,<ea>
			0x0200..=0x02FC => {
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
				Instruction {
					operation: Operation::Andi,
					size: Some(size),
					src: Some(Operand::Immediate(immediate)),
					dst: Some(dst),
				}
			}
			// ORI.<size> #<data>,<ea>
			0x0000..=0x00FC => {
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
				Instruction {
					operation: Operation::Ori,
					size: Some(size),
					src: Some(Operand::Immediate(immediate)),
					dst: Some(dst),
				}
			}
			// ADDX.<size> Dy,Dx
			0xD100..=0xD1F8 if (opcode & 0xF1F8) == 0xD100 => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let dx = ((opcode >> 9) & 0x7) as u8;
				let dy = (opcode & 0x7) as u8;
				Instruction {
					operation: Operation::Addx,
					size: Some(size),
					src: Some(Operand::DataRegister(dy)),
					dst: Some(Operand::DataRegister(dx)),
				}
			}
			// SUBX.<size> Dy,Dx
			0x9100..=0x91F8 if (opcode & 0xF1F8) == 0x9100 => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let dx = ((opcode >> 9) & 0x7) as u8;
				let dy = (opcode & 0x7) as u8;
				Instruction {
					operation: Operation::Subx,
					size: Some(size),
					src: Some(Operand::DataRegister(dy)),
					dst: Some(Operand::DataRegister(dx)),
				}
			}
			// BRA.<size> <displacement>
			0x6000..=0x60FF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Bra,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// BEQ.<size> <displacement>
			0x6700..=0x67FF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Beq,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// BNE.<size> <displacement>
			0x6600..=0x66FF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Bne,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// CLR.<size> <ea>
			0x4200..=0x42FC => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let ea_mode = ((opcode >> 3) & 0x7) as u8;
				let ea_reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(ea_mode, ea_reg, size);
				Instruction {
					operation: Operation::Clr,
					size: Some(size),
					src: None,
					dst: Some(dst),
				}
			}
			// SWAP Dn
			0x4840..=0x4847 => {
				let reg = (opcode & 0x7) as u8;
				Instruction {
					operation: Operation::Swap,
					size: None,
					src: None,
					dst: Some(Operand::DataRegister(reg)),
				}
			}
			// CMPA.<size> <ea>,An
			0xB0C0..=0xBFC0 if (opcode & 0xF1C0) == 0xB0C0 => {
				let an = ((opcode >> 9) & 0x7) as u8;
				let size = if (opcode & 0x0100) != 0 { Size::Long } else { Size::Word };
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::Cmpa,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::AddressRegister(an)),
				}
			}
			// CHK <ea>,Dn
			0x4180..=0x41BC => {
				let dn = ((opcode >> 9) & 0x7) as u8;
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, Size::Word);
				Instruction {
					operation: Operation::Chk,
					size: Some(Size::Word),
					src: Some(src),
					dst: Some(Operand::DataRegister(dn)),
				}
			}
			// TAS <ea>
			0x4AC0..=0x4AFC => {
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, Size::Byte);
				Instruction {
					operation: Operation::Tas,
					size: Some(Size::Byte),
					src: None,
					dst: Some(dst),
				}
			}
			// ROL.<size> Dx,Dy or #<data>,Dy
			0xE118..=0xE17F => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let count_reg = ((opcode >> 9) & 0x7) as u8;
				let dr = (opcode >> 5) & 0x1; // 0: immediate, 1: register
				let dst_reg = (opcode & 0x7) as u8;
				let src = if dr == 0 {
					Operand::Immediate(if count_reg == 0 { 8 } else { count_reg as u32 })
				} else {
					Operand::DataRegister(count_reg)
				};
				Instruction {
					operation: Operation::Rol,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::DataRegister(dst_reg)),
				}
			}
			// ROXL.<size> Dx,Dy or #<data>,Dy
			0xE510..=0xE57F => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let count_reg = ((opcode >> 9) & 0x7) as u8;
				let dr = (opcode >> 5) & 0x1; // 0: immediate, 1: register
				let dst_reg = (opcode & 0x7) as u8;
				let src = if dr == 0 {
					Operand::Immediate(if count_reg == 0 { 8 } else { count_reg as u32 })
				} else {
					Operand::DataRegister(count_reg)
				};
				Instruction {
					operation: Operation::Roxl,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::DataRegister(dst_reg)),
				}
			}
			// JSR <ea>
			0x4E80..=0x4EBC => {
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, Size::Long);
				Instruction {
					operation: Operation::Jsr,
					size: None,
					src: Some(src),
					dst: None,
				}
			}
			// BCHG Dn,<ea>
			0x0140..=0x017F => {
				let size = if (opcode & 0x0038) == 0 { Size::Long } else { Size::Byte };
				let dn = ((opcode >> 9) & 0x7) as u8;
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::Bchg,
					size: Some(size),
					src: Some(Operand::DataRegister(dn)),
					dst: Some(dst),
				}
			}
			// BSET Dn,<ea>
			0x01C0..=0x01FF => {
				let size = if (opcode & 0x0038) == 0 { Size::Long } else { Size::Byte };
				let dn = ((opcode >> 9) & 0x7) as u8;
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::Bset,
					size: Some(size),
					src: Some(Operand::DataRegister(dn)),
					dst: Some(dst),
				}
			}
			// BTST Dn,<ea>
			0x0100..=0x013F => {
				let size = if (opcode & 0x0038) == 0 { Size::Long } else { Size::Byte };
				let dn = ((opcode >> 9) & 0x7) as u8;
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::Btst,
					size: Some(size),
					src: Some(Operand::DataRegister(dn)),
					dst: Some(dst),
				}
			}
			// TRAP #vector
			0x4E40..=0x4E4F => {
				let vector = (opcode & 0xF) as u32;
				Instruction {
					operation: Operation::Trap,
					size: None,
					src: Some(Operand::Immediate(vector)),
					dst: None,
				}
			}
			// AND.<size> <ea>,Dn
			0xC000..=0xC1F0 => {
				let dn = ((opcode >> 9) & 0x7) as u8;
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::And,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::DataRegister(dn)),
				}
			}
			// OR.<size> <ea>,Dn
			0x8000..=0x81F0 => {
				let dn = ((opcode >> 9) & 0x7) as u8;
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::Or,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::DataRegister(dn)),
				}
			}
			// ADDQ.<size> #<data>,<ea>
			0x5000..=0x51FC => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let data = ((opcode >> 9) & 0x7) as u8;
				let data = if data == 0 { 8 } else { data }; // 0 means 8
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::Addq,
					size: Some(size),
					src: Some(Operand::Immediate(data as u32)),
					dst: Some(dst),
				}
			}
			// SUBQ.<size> #<data>,<ea>
			0x5100..=0x51FC => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let data = ((opcode >> 9) & 0x7) as u8;
				let data = if data == 0 { 8 } else { data }; // 0 means 8
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::Subq,
					size: Some(size),
					src: Some(Operand::Immediate(data as u32)),
					dst: Some(dst),
				}
			}
			// NOT.<size> <ea>
			0x4600..=0x46FC => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::Not,
					size: Some(size),
					src: None,
					dst: Some(dst),
				}
			}
			// DIVS.W <ea>,Dn
			0x81C0..=0x81FF => {
				let dn = ((opcode >> 9) & 0x7) as u8;
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, Size::Word);
				Instruction {
					operation: Operation::Divs,
					size: Some(Size::Word),
					src: Some(src),
					dst: Some(Operand::DataRegister(dn)),
				}
			}
			// ROXR.<size> Dx,Dy or #<data>,Dy
			0xE410..=0xE47F => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let count_reg = ((opcode >> 9) & 0x7) as u8;
				let dr = (opcode >> 5) & 0x1; // 0: immediate, 1: register
				let dst_reg = (opcode & 0x7) as u8;
				let src = if dr == 0 {
					Operand::Immediate(if count_reg == 0 { 8 } else { count_reg as u32 })
				} else {
					Operand::DataRegister(count_reg)
				};
				Instruction {
					operation: Operation::Roxr,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::DataRegister(dst_reg)),
				}
			}
			// BHI <displacement>
			0x6200..=0x62FF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Bhi,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// BLS <displacement>
			0x6300..=0x63FF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Bls,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// JMP <ea>
			0x4EC0..=0x4EFC => {
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, Size::Long);
				Instruction {
					operation: Operation::Jmp,
					size: None,
					src: Some(src),
					dst: None,
				}
			}
			// ADDA.<size> <ea>,An
			0xD0C0..=0xDFFC if (opcode & 0xF1C0) == 0xD0C0 => {
				let an = ((opcode >> 9) & 0x7) as u8;
				let size = if (opcode & 0x0100) != 0 { Size::Long } else { Size::Word };
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::Adda,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::AddressRegister(an)),
				}
			}
			// SUBA.<size> <ea>,An
			0x90C0..=0x9FFC if (opcode & 0xF1C0) == 0x90C0 => {
				let an = ((opcode >> 9) & 0x7) as u8;
				let size = if (opcode & 0x0100) != 0 { Size::Long } else { Size::Word };
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::Suba,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::AddressRegister(an)),
				}
			}
			// MULU.W <ea>,Dn
			0xC0C0..=0xC0FF => {
				let dn = ((opcode >> 9) & 0x7) as u8;
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, Size::Word);
				Instruction {
					operation: Operation::Mulu,
					size: Some(Size::Word),
					src: Some(src),
					dst: Some(Operand::DataRegister(dn)),
				}
			}
			// CMP.<size> <ea>,Dn
			0xB000..=0xB1F0 => {
				let dn = ((opcode >> 9) & 0x7) as u8;
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::Cmp,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::DataRegister(dn)),
				}
			}
			// Scc <ea> (e.g., SHS for Carry Clear)
			0x54C0..=0x54FC => {
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, Size::Byte);
				Instruction {
					operation: Operation::Scc,
					size: Some(Size::Byte),
					src: None,
					dst: Some(dst),
				}
			}
			// DBcc Dn,<displacement> (e.g., DBEQ)
			0x54C8..=0x54CF => {
				let dn = (opcode & 0x7) as u8;
				let disp = self.fetch_word() as i16 as i32;
				Instruction {
					operation: Operation::Dbcc,
					size: Some(Size::Word),
					src: Some(Operand::DataRegister(dn)),
					dst: Some(Operand::Immediate(disp as u32)),
				}
			}
			// PEA <ea>
			0x4840..=0x487C if (opcode & 0xF1C0) == 0x4840 => {
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, Size::Long);
				Instruction {
					operation: Operation::Pea,
					size: None,
					src: Some(src),
					dst: None,
				}
			}
			// LINK An,#<displacement>
			0x4E50..=0x4E57 => {
				let an = (opcode & 0x7) as u8;
				let disp = self.fetch_word() as i16 as i32;
				Instruction {
					operation: Operation::Link,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: Some(Operand::AddressRegister(an)),
				}
			}
			// UNLK An
			0x4E58..=0x4E5F => {
				let an = (opcode & 0x7) as u8;
				Instruction {
					operation: Operation::Unlk,
					size: None,
					src: None,
					dst: Some(Operand::AddressRegister(an)),
				}
			}
			// NOP (alternative form, e.g., 0x4E72 as a placeholder for distinction)
			0x4E72 => {
				Instruction {
					operation: Operation::NopAlt,
					size: None,
					src: None,
					dst: None,
				}
			}
			// BGT <displacement>
			0x6E00..=0x6EFF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Bgt,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// BLE <displacement>
			0x6F00..=0x6FFF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Ble,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// EXG Dx,Dy
			0xC140..=0xC148 => {
				let dx = ((opcode >> 9) & 0x7) as u8;
				let dy = (opcode & 0x7) as u8;
				Instruction {
					operation: Operation::Exg,
					size: Some(Size::Long),
					src: Some(Operand::DataRegister(dx)),
					dst: Some(Operand::DataRegister(dy)),
				}
			}
			// MOVEM.<size> <ea>,reglist or reglist,<ea>
			0x4880..=0x4BFC if (opcode & 0xFB80) == 0x4880 => {
				let direction = (opcode >> 10) & 0x1; // 0: reg to mem, 1: mem to reg
				let size = if (opcode & 0x0040) != 0 { Size::Long } else { Size::Word };
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let reglist = self.fetch_word();
				let ea = self.decode_ea(mode, reg, size);
				let (src, dst) = if direction == 0 {
					(Operand::Immediate(reglist as u32), ea)
				} else {
					(ea, Operand::Immediate(reglist as u32))
				};
				Instruction {
					operation: Operation::Movem,
					size: Some(size),
					src: Some(src),
					dst: Some(dst),
				}
			}
			// ASL.<size> <ea>
			0xE100..=0xE1F8 if (opcode & 0xF138) == 0xE100 => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::AslMem,
					size: Some(size),
					src: Some(Operand::Immediate(1)), // Shift by 1
					dst: Some(dst),
				}
			}
			// LSR.<size> <ea>
			0xE008..=0xE0F8 if (opcode & 0xF138) == 0xE008 => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::LsrMem,
					size: Some(size),
					src: Some(Operand::Immediate(1)), // Shift by 1
					dst: Some(dst),
				}
			}
			// CLR.<size> <ea> (alternative form, distinct range for demonstration)
			0x4200..=0x42F8 if (opcode & 0xFF00) == 0x4200 => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::ClrMem,
					size: Some(size),
					src: None,
					dst: Some(dst),
				}
			}
			// STOP #<data>
			0x4E72 => {
				let data = self.fetch_word() as u32;
				Instruction {
					operation: Operation::Stop,
					size: None,
					src: Some(Operand::Immediate(data)),
					dst: None,
				}
			}
			// RTD #<displacement>
			0x4E74 => {
				let disp = self.fetch_word() as i16 as i32;
				Instruction {
					operation: Operation::Rtd,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// TAS <ea> (alternative form, distinct range for demonstration)
			0x4AFC => {
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, Size::Byte);
				Instruction {
					operation: Operation::TasMem,
					size: Some(Size::Byte),
					src: None,
					dst: Some(dst),
				}
			}
			// BCC <displacement>
			0x6400..=0x64FF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Bcc,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// BCS <displacement>
			0x6500..=0x65FF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Bcs,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// BGE <displacement>
			0x6C00..=0x6CFF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Bge,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// BLT <displacement>
			0x6D00..=0x6DFF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Blt,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// BMI <displacement>
			0x6B00..=0x6BFF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Bmi,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// BPL <displacement>
			0x6A00..=0x6AFF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Bpl,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// LSL.<size> Dx,Dy or #<data>,Dy
			0xE108..=0xE17F if (opcode & 0xFEE0) == 0xE108 => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let count = ((opcode >> 9) & 0x7) as u8;
				let dr = (opcode >> 5) & 0x1; // 0: immediate, 1: register
				let dst_reg = (opcode & 0x7) as u8;
				let src = if dr == 0 {
					Operand::Immediate(if count == 0 { 8 } else { count as u32 })
				} else {
					Operand::DataRegister(count)
				};
				Instruction {
					operation: Operation::Lsl,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::DataRegister(dst_reg)),
				}
			}
			// ROL.<size> <ea>
			0xE7C0..=0xE7F8 if (opcode & 0xF1C0) == 0xE7C0 => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::RolMem,
					size: Some(size),
					src: Some(Operand::Immediate(1)), // Rotate by 1
					dst: Some(dst),
				}
			}
			// ROR.<size> <ea>
			0xE6C0..=0xE6F8 if (opcode & 0xF1C0) == 0xE6C0 => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::RorMem,
					size: Some(size),
					src: Some(Operand::Immediate(1)), // Rotate by 1
					dst: Some(dst),
				}
			}
			// SUBI.<size> #<data>,<ea>
			0x0400..=0x04FC => {
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
				Instruction {
					operation: Operation::Subi,
					size: Some(size),
					src: Some(Operand::Immediate(immediate)),
					dst: Some(dst),
				}
			}
			// ABCD Dy,Dx or -(Ay),-(Ax)
			0xC100..=0xC1F8 if (opcode & 0xF1F0) == 0xC100 => {
				let dx = ((opcode >> 9) & 0x7) as u8;
				let dy = (opcode & 0x7) as u8;
				let rm = (opcode >> 3) & 0x1; // 0: register, 1: memory
				let (src, dst) = if rm == 0 {
					(Operand::DataRegister(dy), Operand::DataRegister(dx))
				} else {
					(Operand::PreDec(dy), Operand::PreDec(dx))
				};
				Instruction {
					operation: Operation::Abcd,
					size: Some(Size::Byte),
					src: Some(src),
					dst: Some(dst),
				}
			}
			// SBCD Dy,Dx or -(Ay),-(Ax)
			0x8100..=0x81F8 if (opcode & 0xF1F0) == 0x8100 => {
				let dx = ((opcode >> 9) & 0x7) as u8;
				let dy = (opcode & 0x7) as u8;
				let rm = (opcode >> 3) & 0x1; // 0: register, 1: memory
				let (src, dst) = if rm == 0 {
					(Operand::DataRegister(dy), Operand::DataRegister(dx))
				} else {
					(Operand::PreDec(dy), Operand::PreDec(dx))
				};
				Instruction {
					operation: Operation::Sbcd,
					size: Some(Size::Byte),
					src: Some(src),
					dst: Some(dst),
				}
			}
			// NBCD <ea>
			0x4800..=0x483C => {
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, Size::Byte);
				Instruction {
					operation: Operation::Nbcd,
					size: Some(Size::Byte),
					src: None,
					dst: Some(dst),
				}
			}
			// ADDI.<size> #<data>,<ea>
			0x0600..=0x06FC => {
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
				Instruction {
					operation: Operation::Addi,
					size: Some(size),
					src: Some(Operand::Immediate(immediate)),
					dst: Some(dst),
				}
			}
			// BVC <displacement>
			0x6800..=0x68FF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Bvc,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// BVS <displacement>
			0x6900..=0x69FF => {
				let disp = if (opcode & 0x00FF) == 0 {
					self.fetch_word() as i16 as i32
				} else {
					(opcode & 0x00FF) as i8 as i32
				};
				Instruction {
					operation: Operation::Bvs,
					size: None,
					src: Some(Operand::Immediate(disp as u32)),
					dst: None,
				}
			}
			// ROXL.<size> <ea>
			0xE5C0..=0xE5F8 if (opcode & 0xF1C0) == 0xE5C0 => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::RoxlMem,
					size: Some(size),
					src: Some(Operand::Immediate(1)), // Rotate by 1
					dst: Some(dst),
				}
			}
			// ROXR.<size> <ea>
			0xE4C0..=0xE4F8 if (opcode & 0xF1C0) == 0xE4C0 => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::RoxrMem,
					size: Some(size),
					src: Some(Operand::Immediate(1)), // Rotate by 1
					dst: Some(dst),
				}
			}
			// TRAPV
			0x4E76 => {
				Instruction {
					operation: Operation::Trapv,
					size: None,
					src: None,
					dst: None,
				}
			}
			// RESET
			0x4E70 => {
				Instruction {
					operation: Operation::Reset,
					size: None,
					src: None,
					dst: None,
				}
			}
			// RTE
			0x4E73 => {
				Instruction {
					operation: Operation::Rte,
					size: None,
					src: None,
					dst: None,
				}
			}
			// MOVEA.<size> <ea>,An (subset of MOVE where dst is An)
			0x2040..=0x3FFF if (opcode & 0xF1C0) == 0x2040 => {
				let size = if (opcode & 0xC000) == 0x2000 { Size::Word } else { Size::Long };
				let an = ((opcode >> 9) & 0x7) as u8;
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::Movea,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::AddressRegister(an)),
				}
			}
			// MOVE.B <ea>,CCR
			0x42C0..=0x42FF => {
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, Size::Byte);
				Instruction {
					operation: Operation::MoveCcr,
					size: Some(Size::Byte),
					src: Some(src),
					dst: None,
				}
			}
			// MOVE.W <ea>,SR
			0x46C0..=0x46FF => {
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let src = self.decode_ea(mode, reg, Size::Word);
				Instruction {
					operation: Operation::MoveSr,
					size: Some(Size::Word),
					src: Some(src),
					dst: None,
				}
			}
			// MOVE USP,An or MOVE An,USP
			0x4E60..=0x4E67 => {
				let direction = (opcode >> 3) & 0x1; // 0: USP to An, 1: An to USP
				let an = (opcode & 0x7) as u8;
				let (src, dst) = if direction == 0 {
					(Operand::Immediate(0), Operand::AddressRegister(an)) // USP as src not directly addressable here
				} else {
					(Operand::AddressRegister(an), Operand::Immediate(0)) // USP as dst
				};
				Instruction {
					operation: Operation::MoveUsp,
					size: Some(Size::Long),
					src: Some(src),
					dst: Some(dst),
				}
			}
			// ANDI.B #<data>,CCR
			0x023C => {
				let immediate = self.fetch_word() as u32 & 0xFF;
				Instruction {
					operation: Operation::AndiCcr,
					size: Some(Size::Byte),
					src: Some(Operand::Immediate(immediate)),
					dst: None,
				}
			}
			// ORI.B #<data>,CCR
			0x003C => {
				let immediate = self.fetch_word() as u32 & 0xFF;
				Instruction {
					operation: Operation::OriCcr,
					size: Some(Size::Byte),
					src: Some(Operand::Immediate(immediate)),
					dst: None,
				}
			}
			// EORI.B #<data>,CCR
			0x0A3C => {
				let immediate = self.fetch_word() as u32 & 0xFF;
				Instruction {
					operation: Operation::EoriCcr,
					size: Some(Size::Byte),
					src: Some(Operand::Immediate(immediate)),
					dst: None,
				}
			}
			// LSL.<size> <ea>
			0xE3C0..=0xE3F8 if (opcode & 0xF1C0) == 0xE3C0 => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let mode = ((opcode >> 3) & 0x7) as u8;
				let reg = (opcode & 0x7) as u8;
				let dst = self.decode_ea(mode, reg, size);
				Instruction {
					operation: Operation::LslMem,
					size: Some(size),
					src: Some(Operand::Immediate(1)), // Shift by 1
					dst: Some(dst),
				}
			}
			// ASR.<size> Dx,Dy or #<data>,Dy
			0xE000..=0xE07F if (opcode & 0xFEE0) == 0xE000 => {
				let size = match (opcode >> 6) & 0x3 {
					0 => Size::Byte,
					1 => Size::Word,
					2 => Size::Long,
					_ => unreachable!(),
				};
				let count = ((opcode >> 9) & 0x7) as u8;
				let dr = (opcode >> 5) & 0x1; // 0: immediate, 1: register
				let dst_reg = (opcode & 0x7) as u8;
				let src = if dr == 0 {
					Operand::Immediate(if count == 0 { 8 } else { count as u32 })
				} else {
					Operand::DataRegister(count)
				};
				Instruction {
					operation: Operation::Asr,
					size: Some(size),
					src: Some(src),
					dst: Some(Operand::DataRegister(dst_reg)),
				}
			}
			_ => panic!("Unimplemented opcode: {:04X} at PC: {:06X}", opcode, self.pc - 2),
		}
	}

    /// Execute an instruction and return cycle count
	fn execute(&mut self, instr: Instruction) -> u32 {
		let mut cycles = 0;
		match instr.operation {
			Operation::Nop => cycles = 4, // Correct: 2n
			Operation::Rts => {
				self.pc = self.memory.read_long(self.a[7]);
				self.a[7] += 4;
				cycles = 16; // Correct: 8n
			}
			Operation::Move => {
				let size = instr.size.unwrap();
				let src = instr.src.unwrap();
				let dst = instr.dst.unwrap();
				let value = self.get_operand_value(size, &src);
				self.set_operand_value(size, &dst, value);
				let n = (value & (1 << (size.bits() - 1))) != 0;
				let z = value == 0;
				self.set_flags(n, z, false, false);
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
				self.set_flags(n, z, overflow, carry);
				cycles = match size {
					Size::Byte | Size::Word => 4,
					Size::Long => 8,
				} + self.ea_cycles(&src, size) + if matches!(dst, Operand::DataRegister(_)) { 0 } else { self.ea_cycles(&dst, size) };
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
				self.set_flags(n, z, false, false);
				let ones = (src_val as u16 as u32 & 0xFFFF).count_ones(); // Dynamic timing
				cycles = 38 + 2 * ones + self.ea_cycles(&src, size); // 19n + 1n per '1' bit
			}
			Operation::Divu => {
				let size = instr.size.unwrap();
				let src = instr.src.unwrap();
				let dst = instr.dst.unwrap();
				let divisor = self.get_operand_value(size, &src) as u16 as u32;
				let dividend = self.get_operand_value(Size::Long, &dst);
				if divisor == 0 {
					return self.trigger_exception(5); // Zero Divide, vector 5
				}
				let quotient = dividend / divisor;
				let remainder = dividend % divisor;
				let result = (remainder << 16) | (quotient & 0xFFFF);
				self.set_operand_value(Size::Long, &dst, result);
				let n = (quotient & 0x8000) != 0;
				let z = quotient == 0;
				let v = quotient > 0xFFFF;
				self.set_flags(n, z, v, false);
				cycles = 76; // Base: 38n
				if dividend != 0 && divisor != 0 {
					let quotient_bits = 32 - (dividend / divisor).leading_zeros();
					let shift_count = if quotient_bits > 0 { quotient_bits - 1 } else { 0 };
					cycles += 2 * shift_count;
					if dividend < divisor { cycles += 2; }
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
				self.set_flags(n, z, false, false);
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
				self.set_flags(n, z, v, c);
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
				self.set_flags(n, z, false, false);
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
				let c = if shift_count > 0 { (dst_val >> (shift_count - 1)) & 1 != 0 } else { false };
				self.set_flags(n, z, false, c);
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
				let c = if shift_count > 0 { (dst_val & (1 << (size.bits() - shift_count))) != 0 } else { false };
				let v = (0..shift_count).any(|i| {
					let bit = (dst_val >> (size.bits() - 1 - i)) & 1;
					bit != (dst_val >> (size.bits() - 1)) & 1
				});
				self.set_flags(n, z, v, c);
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
				self.set_flags(n, z, false, false);
				cycles = 4; // Correct: 2n
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
				let c = if shift_count > 0 { (dst_val >> (shift_count - 1)) & 1 != 0 } else { false };
				self.set_flags(n, z, false, c);
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
				self.set_flags(n, z, false, false);
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
				self.set_flags(n, z, v, c);
				cycles = match size {
					Size::Byte => 8,
					Size::Word => 8,
					Size::Long => 14, // Adjusted: 7n due to 2-word fetch
				} + self.ea_cycles(&dst, size);
			}
			Operation::Movep => {
				let size = instr.size.unwrap();
				let src = instr.src.unwrap();
				let dst = instr.dst.unwrap();
				if let Operand::Displacement(areg, disp) = src {
					let mut addr = self.a[areg as usize].wrapping_add(disp as u32);
					if let Operand::DataRegister(dreg) = dst {
						let count = if size == Size::Word { 2 } else { 4 };
						let mut value = 0;
						for _ in 0..count {
							value = (value << 8) | (self.memory.read_byte(addr) as u32);
							addr += 2;
						}
						self.d[dreg as usize] = value;
					}
				} else if let Operand::DataRegister(dreg) = src {
					let value = self.d[dreg as usize];
					if let Operand::Displacement(areg, disp) = dst {
						let mut addr = self.a[areg as usize].wrapping_add(disp as u32);
						let count = if size == Size::Word { 2 } else { 4 };
						for i in (0..count).rev() {
							self.memory.write_byte(addr, (value >> (8 * i)) & 0xFF);
							addr += 2;
						}
					}
				}
				cycles = if size == Size::Word { 16 } else { 24 }; // Correct: 8n, 12n
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
				self.set_flags(false, z, false, false);
				cycles = if matches!(dst, Operand::DataRegister(_)) { 10 } else { 14 } + self.ea_cycles(&dst, size);
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
					_ => 4, // Base case, refined per EA
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
					self.set_flags(n, z, false, false);
				}
				cycles = 4; // Correct: 2n
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
				self.set_flags(n, z, overflow, carry);
				cycles = match size {
					Size::Byte | Size::Word => 4,
					Size::Long => 8,
				} + self.ea_cycles(&src, size) + if matches!(dst, Operand::DataRegister(_)) { 0 } else { self.ea_cycles(&dst, size) };
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
				self.set_flags(n, z, false, false);
				cycles = match size {
					Size::Byte | Size::Word => 8,
					Size::Long => 14, // Adjusted: 7n
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
				self.set_flags(n, z, false, false);
				cycles = match size {
					Size::Byte | Size::Word => 8,
					Size::Long => 14, // Adjusted: 7n
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
						let x_val = if x { 1 } else { 0 };
						let r = d.wrapping_add(s).wrapping_add(x_val);
						let c = (s as u16 + d as u16 + x_val as u16) > 0xFF;
						let v = ((s & d & !r) | (!s & !d & r)) & 0x80 != 0;
						(r as u32, c, v)
					}
					Size::Word => {
						let s = src_val as u16;
						let d = dst_val as u16;
						let x_val = if x { 1 } else { 0 };
						let r = d.wrapping_add(s).wrapping_add(x_val);
						let c = (s as u32 + d as u32 + x_val as u32) > 0xFFFF;
						let v = ((s & d & !r) | (!s & !d & r)) & 0x8000 != 0;
						(r as u32, c, v)
					}
					Size::Long => {
						let s = src_val;
						let d = dst_val;
						let x_val = if x { 1 } else { 0 };
						let r = d.wrapping_add(s).wrapping_add(x_val);
						let c = (s as u64 + d as u64 + x_val as u64) > 0xFFFFFFFF;
						let v = ((s & d & !r) | (!s & !d & r)) & 0x80000000 != 0;
						(r, c, v)
					}
				};
				self.set_operand_value(size, &dst, result);
				let n = (result & (1 << (size.bits() - 1))) != 0;
				let z = if self.sr & 0x4 != 0 && result == 0 { true } else { false };
				self.set_flags(n, z, overflow, carry);
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
						let x_val = if x { 1 } else { 0 };
						let r = d.wrapping_sub(s).wrapping_sub(x_val);
						let c = (s as u16 + x_val as u16) > (d as u16);
						let v = ((d & !s & !r) | (!d & s & r)) & 0x80 != 0;
						(r as u32, c, v)
					}
					Size::Word => {
						let s = src_val as u16;
						let d = dst_val as u16;
						let x_val = if x { 1 } else { 0 };
						let r = d.wrapping_sub(s).wrapping_sub(x_val);
						let c = (s as u32 + x_val as u32) > (d as u32);
						let v = ((d & !s & !r) | (!d & s & r)) & 0x8000 != 0;
						(r as u32, c, v)
					}
					Size::Long => {
						let s = src_val;
						let d = dst_val;
						let x_val = if x { 1 } else { 0 };
						let r = d.wrapping_sub(s).wrapping_sub(x_val);
						let c = (s as u64 + x_val as u64) > (d as u64);
						let v = ((d & !s & !r) | (!d & s & r)) & 0x80000000 != 0;
						(r, c, v)
					}
				};
				self.set_operand_value(size, &dst, result);
				let n = (result & (1 << (size.bits() - 1))) != 0;
				let z = if self.sr & 0x4 != 0 && result == 0 { true } else { false };
				self.set_flags(n, z, overflow, carry);
				cycles = match size {
					Size::Byte | Size::Word => 4,
					Size::Long => 8,
				} + self.ea_cycles(&src, size) + self.ea_cycles(&dst, size);
			}
			Operation::Bra => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				self.pc = (self.pc as i32 + disp) as u32;
				cycles = 10; // Correct: 5n (taken only)
			}
			Operation::Beq => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				if self.sr & 0x4 != 0 {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
				}
			}
			Operation::Bne => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				if self.sr & 0x4 == 0 {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
				}
			}
			Operation::Clr => {
				let size = instr.size.unwrap();
				let dst = instr.dst.unwrap();
				self.set_operand_value(size, &dst, 0);
				self.set_flags(false, true, false, false);
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
					self.set_flags(n, z, false, false);
				}
				cycles = 4; // Correct: 2n
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
				self.set_flags(n, z, v, c);
				cycles = 6 + self.ea_cycles(&src, size); // Correct: 3n
			}
			Operation::Chk => {
				let src = instr.src.unwrap();
				let dst = instr.dst.unwrap();
				let src_val = self.get_operand_value(Size::Word, &src) as i16;
				let dst_val = self.get_operand_value(Size::Word, &dst) as i16;
				if dst_val < 0 || dst_val > src_val {
					return self.trigger_exception(6); // CHK, vector 6
				}
				let n = dst_val < 0;
				let z = dst_val == 0;
				self.set_flags(n, z, false, false);
				cycles = 10 + self.ea_cycles(&src, Size::Word);
			}
			Operation::Tas => {
				let dst = instr.dst.unwrap();
				let val = self.get_operand_value(Size::Byte, &dst) as u8;
				let n = (val & 0x80) != 0;
				let z = val == 0;
				self.set_flags(n, z, false, false);
				self.set_operand_value(Size::Byte, &dst, val | 0x80);
				cycles = if matches!(dst, Operand::DataRegister(_)) { 4 } else { 14 } + self.ea_cycles(&dst, Size::Byte);
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
				let c = if shift_count > 0 { (dst_val & (1 << (size.bits() - shift_count))) != 0 } else { false };
				self.set_flags(n, z, false, c);
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
				let full_val = if x { (dst_val | (1 << shift)) } else { dst_val };
				let result = if shift_count == 0 { dst_val } else { full_val.rotate_left(shift_count) & size.mask() };
				self.set_operand_value(size, &dst, result);
				let n = (result & (1 << (shift - 1))) != 0;
				let z = result == 0;
				let c = if shift_count > 0 { (full_val & (1 << (shift + 1 - shift_count))) != 0 } else { false };
				self.set_flags(n, z, false, c);
				self.sr = (self.sr & 0xFFFE) | (if c { 1 } else { 0 });
				cycles = match size {
					Size::Byte | Size::Word => 6,
					Size::Long => 8,
				} + 2 * shift_count as u32;
			}
			Operation::Jsr => {
				let src = instr.src.unwrap();
				let addr = self.get_operand_value(Size::Long, &src);
				self.a[7] -= 4;
				self.memory.write_long(self.a[7], self.pc);
				self.pc = addr;
				cycles = match src {
					Operand::Displacement(_, _) => 18,
					Operand::AbsoluteShort(_) => 18,
					Operand::AbsoluteLong(_) => 20,
					_ => 16, // Base case adjusted per EA
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
				self.set_flags(false, z, false, false);
				cycles = if matches!(dst, Operand::DataRegister(_)) { 8 } else { 12 } + self.ea_cycles(&dst, size);
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
				self.set_flags(false, z, false, false);
				cycles = if matches!(dst, Operand::DataRegister(_)) { 8 } else { 12 } + self.ea_cycles(&dst, size);
			}
			Operation::Btst => {
				let size = instr.size.unwrap();
				let src = instr.src.unwrap();
				let dst = instr.dst.unwrap();
				let bit_num = self.get_operand_value(Size::Long, &src) % size.bits();
				let val = self.get_operand_value(size, &dst);
				let z = (val & (1 << bit_num)) == 0;
				self.set_flags(false, z, false, false);
				cycles = if matches!(dst, Operand::DataRegister(_)) { 6 } else { 4 } + self.ea_cycles(&dst, size);
			}
			Operation::Trap => {
				let vector = self.get_operand_value(Size::Long, &instr.src.unwrap());
				self.a[7] -= 4;
				self.memory.write_long(self.a[7], self.pc);
				self.a[7] -= 2;
				self.memory.write_word(self.a[7], self.sr);
				self.pc = self.memory.read_long(32 + vector * 4);
				cycles = 34; // Correct: 17n
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
				self.set_flags(n, z, false, false);
				cycles = match size {
					Size::Byte | Size::Word => 4,
					Size::Long => 6,
				} + self.ea_cycles(&src, size) + if matches!(dst, Operand::DataRegister(_)) { 0 } else { self.ea_cycles(&dst, size) };
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
				self.set_flags(n, z, false, false);
				cycles = match size {
					Size::Byte | Size::Word => 4,
					Size::Long => 6,
				} + self.ea_cycles(&src, size) + if matches!(dst, Operand::DataRegister(_)) { 0 } else { self.ea_cycles(&dst, size) };
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
				self.set_flags(n, z, overflow, carry);
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
				self.set_flags(n, z, overflow, carry);
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
				self.set_flags(n, z, false, false);
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
					return self.trigger_exception(5); // Zero Divide, vector 5
				}
				let quotient = dividend / divisor;
				let remainder = dividend % divisor;
				let result = ((remainder as u32) << 16) | (quotient as u32 & 0xFFFF);
				self.set_operand_value(Size::Long, &dst, result);
				let n = (quotient & 0x8000) != 0;
				let z = quotient == 0;
				let v = quotient > 0x7FFF || quotient < -0x8000;
				self.set_flags(n, z, v, false);
				cycles = 96; // Base: 48n
				if dividend != 0 && divisor != 0 {
					let quotient_abs = quotient.abs() as u32;
					let quotient_bits = 32 - quotient_abs.leading_zeros();
					let shift_count = if quotient_bits > 0 { quotient_bits - 1 } else { 0 };
					cycles += 2 * shift_count;
					if dividend < 0 { cycles += 2; }
					if divisor < 0 { cycles += 2; }
					if quotient_abs == 0 && (dividend < 0 || divisor < 0) { cycles += 4; }
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
				let full_val = if x { (dst_val | (1 << shift)) } else { dst_val };
				let result = if shift_count == 0 { dst_val } else { full_val.rotate_right(shift_count) & size.mask() };
				self.set_operand_value(size, &dst, result);
				let n = (result & (1 << (shift - 1))) != 0;
				let z = result == 0;
				let c = if shift_count > 0 { (full_val & (1 << (shift_count - 1))) != 0 } else { false };
				self.set_flags(n, z, false, c);
				self.sr = (self.sr & 0xFFFE) | (if c { 1 } else { 0 });
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
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
				}
			}
			Operation::Bls => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				let z = (self.sr & 0x4) != 0;
				let c = (self.sr & 0x1) != 0;
				if c || z {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
				}
			}
			Operation::Jmp => {
				let src = instr.src.unwrap();
				let addr = self.get_operand_value(Size::Long, &src);
				self.pc = addr;
				cycles = match src {
					Operand::Displacement(_, _) => 10,
					Operand::AbsoluteShort(_) => 10,
					Operand::AbsoluteLong(_) => 12,
					_ => 8, // Base case adjusted per EA
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
				self.set_flags(n, z, false, false);
				let ones = src_val.count_ones(); // Dynamic timing
				cycles = 38 + 2 * ones + self.ea_cycles(&src, size); // 19n + 1n per '1' bit
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
				self.set_flags(n, z, v, c);
				cycles = match size {
					Size::Byte | Size::Word => 4,
					Size::Long => 6,
				} + self.ea_cycles(&src, size);
			}
			Operation::Scc => {
				let dst = instr.dst.unwrap();
				let carry = (self.sr & 0x1) != 0;
				let value = if !carry { 0xFF } else { 0x00 };
				self.set_operand_value(Size::Byte, &dst, value as u32);
				cycles = if matches!(dst, Operand::DataRegister(_)) { 4 } else { 8 } + self.ea_cycles(&dst, Size::Byte);
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
					cycles = 10; // Taken: 5n
				} else {
					cycles = 14; // Not taken: 7n
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
				self.memory.write_long(self.a[7], addr);
				cycles = match src {
					Operand::Displacement(_, _) => 12,
					Operand::AbsoluteShort(_) => 12,
					Operand::AbsoluteLong(_) => 16,
					_ => 12, // Base case adjusted per EA
				} + self.ea_cycles(&src, Size::Long);
			}
			Operation::Link => {
				let an = instr.dst.unwrap();
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				if let Operand::AddressRegister(reg) = an {
					self.a[7] -= 4;
					self.memory.write_long(self.a[7], self.a[reg as usize]);
					self.a[reg as usize] = self.a[7];
					self.a[7] = (self.a[7] as i32 + disp) as u32;
				}
				cycles = 16; // Correct: 8n
			}
			Operation::Unlk => {
				let an = instr.dst.unwrap();
				if let Operand::AddressRegister(reg) = an {
					self.a[7] = self.a[reg as usize];
					self.a[reg as usize] = self.memory.read_long(self.a[7]);
					self.a[7] += 4;
				}
				cycles = 12; // Correct: 6n
			}
			Operation::NopAlt => {
				cycles = 4; // Correct: 2n
			}
			Operation::Bgt => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				let z = (self.sr & 0x4) != 0;
				let n = (self.sr & 0x8) != 0;
				let v = (self.sr & 0x2) != 0;
				if !z && (n == v) {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
				}
			}
			Operation::Ble => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				let z = (self.sr & 0x4) != 0;
				let n = (self.sr & 0x8) != 0;
				let v = (self.sr & 0x2) != 0;
				if z || (n != v) {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
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
				cycles = 6; // Correct: 3n
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
				cycles = 12 + self.ea_cycles(&src, size); // Base timing
				let mut bit = 0;
				let regs = reglist.count_ones();
				if let Operand::Immediate(_) = src {
					for i in 0..16 {
						if (reglist & (1 << bit)) != 0 {
							let value = if i < 8 { self.d[i] } else { self.a[i - 8] };
							if size == Size::Word {
								self.memory.write_word(addr, value as u16);
								addr += 2;
								cycles += 4;
							} else {
								self.memory.write_long(addr, value);
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
								let val = self.memory.read_word(addr) as u32;
								addr += 2;
								cycles += 4;
								val
							} else {
								let val = self.memory.read_long(addr);
								addr += 4;
								cycles += 8;
								val
							};
							if i < 8 { self.d[i] = value; } else { self.a[i - 8] = value; }
						}
						bit += 1;
					}
				}
				cycles = 12 + (if size == Size::Word { 4 } else { 8 } * regs); // Corrected: 6n + 2n/4n per reg
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
				self.set_flags(n, z, v, c);
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
				self.set_flags(n, z, false, c);
				cycles = 8 + self.ea_cycles(&dst, size);
			}
			Operation::ClrMem => {
				let size = instr.size.unwrap();
				let dst = instr.dst.unwrap();
				self.set_operand_value(size, &dst, 0);
				self.set_flags(false, true, false, false);
				cycles = match size {
					Size::Byte | Size::Word => 4,
					Size::Long => 6,
				} + self.ea_cycles(&dst, size);
			}
			Operation::Stop => {
				if !self.check_supervisor() {
					cycles = 34; // Privilege Violation
				} else {
					let data = self.get_operand_value(Size::Word, &instr.src.unwrap()) as u16;
					self.sr = data;
					self.halted = true; // Halt CPU until interrupt
					cycles = 4;
				}
			}
			Operation::Rtd => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				self.pc = self.memory.read_long(self.a[7]);
				self.a[7] += 4 + disp as u32;
				cycles = 16; // Correct: 8n
			}
			Operation::TasMem => {
				let dst = instr.dst.unwrap();
				let val = self.get_operand_value(Size::Byte, &dst) as u8;
				let n = (val & 0x80) != 0;
				let z = val == 0;
				self.set_flags(n, z, false, false);
				self.set_operand_value(Size::Byte, &dst, val | 0x80);
				cycles = if matches!(dst, Operand::DataRegister(_)) { 4 } else { 14 } + self.ea_cycles(&dst, Size::Byte);
			}
			Operation::Bcc => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				if (self.sr & 0x1) == 0 {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
				}
			}
			Operation::Bcs => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				if (self.sr & 0x1) != 0 {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
				}
			}
			Operation::Bge => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				let n = (self.sr & 0x8) != 0;
				let v = (self.sr & 0x2) != 0;
				if n == v {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
				}
			}
			Operation::Blt => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				let n = (self.sr & 0x8) != 0;
				let v = (self.sr & 0x2) != 0;
				if n != v {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
				}
			}
			Operation::Bmi => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				if (self.sr & 0x8) != 0 {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
				}
			}
			Operation::Bpl => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				if (self.sr & 0x8) == 0 {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
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
				let c = if shift_count > 0 { (dst_val & (1 << (size.bits() - shift_count))) != 0 } else { false };
				self.set_flags(n, z, false, c);
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
				self.set_flags(n, z, false, c);
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
				self.set_flags(n, z, false, c);
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
				self.set_flags(n, z, overflow, carry);
				cycles = match size {
					Size::Byte | Size::Word => 8,
					Size::Long => 14, // Adjusted: 7n
				} + self.ea_cycles(&dst, size);
			}
			Operation::Abcd => {
				let src = instr.src.unwrap();
				let dst = instr.dst.unwrap();
				let src_val = self.get_operand_value(Size::Byte, &src) as u8;
				let dst_val = self.get_operand_value(Size::Byte, &dst) as u8;
				let x = (self.sr & 0x1) != 0;
				let tens = ((src_val >> 4) & 0xF) + ((dst_val >> 4) & 0xF) + (if x { 1 } else { 0 });
				let units = (src_val & 0xF) + (dst_val & 0xF);
				let mut result = (units % 10) | (((tens + (units / 10)) % 10) << 4);
				let c = tens > 9 || units > 9;
				if result == 0 { result = 0x00; }
				self.set_operand_value(Size::Byte, &dst, result as u32);
				let n = (result & 0x80) != 0;
				let z = if self.sr & 0x4 != 0 && result == 0 { true } else { false };
				self.set_flags(n, z, false, c);
				cycles = if matches!(src, Operand::DataRegister(_)) { 6 } else { 18 }; // Correct: 3n reg, 9n mem
			}
			Operation::Sbcd => {
				let src = instr.src.unwrap();
				let dst = instr.dst.unwrap();
				let src_val = self.get_operand_value(Size::Byte, &src) as u8;
				let dst_val = self.get_operand_value(Size::Byte, &dst) as u8;
				let x = (self.sr & 0x1) != 0;
				let src_dec = ((src_val >> 4) & 0xF) * 10 + (src_val & 0xF);
				let dst_dec = ((dst_val >> 4) & 0xF) * 10 + (dst_val & 0xF);
				let diff = (dst_dec as i16) - (src_dec as i16) - (if x { 1 } else { 0 });
				let result = if diff < 0 { (100 + diff) as u8 } else { diff as u8 };
				let result_bcd = ((result / 10) << 4) | (result % 10);
				self.set_operand_value(Size::Byte, &dst, result_bcd as u32);
				let n = (result_bcd & 0x80) != 0;
				let z = if self.sr & 0x4 != 0 && result_bcd == 0 { true } else { false };
				let c = diff < 0;
				self.set_flags(n, z, false, c);
				cycles = if matches!(src, Operand::DataRegister(_)) { 6 } else { 18 }; // Correct: 3n reg, 9n mem
			}
			Operation::Nbcd => {
				let dst = instr.dst.unwrap();
				let dst_val = self.get_operand_value(Size::Byte, &dst) as u8;
				let x = (self.sr & 0x1) != 0;
				let dec_val = ((dst_val >> 4) & 0xF) * 10 + (dst_val & 0xF);
				let result_dec = 0 - dec_val - (if x { 1 } else { 0 });
				let result = if result_dec < 0 { (100 + result_dec) as u8 } else { result_dec as u8 };
				let result_bcd = ((result / 10) << 4) | (result % 10);
				self.set_operand_value(Size::Byte, &dst, result_bcd as u32);
				let n = (result_bcd & 0x80) != 0;
				let z = if self.sr & 0x4 != 0 && result_bcd == 0 { true } else { false };
				let c = result_dec != 0;
				self.set_flags(n, z, false, c);
				cycles = if matches!(dst, Operand::DataRegister(_)) { 6 } else { 8 }; // Correct: 3n reg, 4n mem
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
				self.set_flags(n, z, overflow, carry);
				cycles = match size {
					Size::Byte | Size::Word => 8,
					Size::Long => 14, // Adjusted: 7n
				} + self.ea_cycles(&dst, size);
			}
			Operation::Bvc => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				if (self.sr & 0x2) == 0 {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
				}
			}
			Operation::Bvs => {
				let disp = self.get_operand_value(Size::Long, &instr.src.unwrap()) as i32;
				if (self.sr & 0x2) != 0 {
					self.pc = (self.pc as i32 + disp) as u32;
					cycles = 10; // Taken: 5n
				} else {
					cycles = 12; // Not taken: 6n
				}
			}
			Operation::RoxlMem => {
				let size = instr.size.unwrap();
				let dst = instr.dst.unwrap();
				let dst_val = self.get_operand_value(size, &dst);
				let x = (self.sr & 0x1) != 0;
				let shift = size.bits();
				let full_val = if x { dst_val | (1 << shift) } else { dst_val };
				let result = full_val.rotate_left(1) & size.mask();
				self.set_operand_value(size, &dst, result);
				let n = (result & (1 << (size.bits() - 1))) != 0;
				let z = result == 0;
				let c = (dst_val & (1 << (size.bits() - 1))) != 0;
				self.set_flags(n, z, false, c);
				self.sr = (self.sr & 0xFFFE) | (if c { 1 } else { 0 });
				cycles = 8 + self.ea_cycles(&dst, size);
			}
			Operation::RoxrMem => {
				let size = instr.size.unwrap();
				let dst = instr.dst.unwrap();
				let dst_val = self.get_operand_value(size, &dst);
				let x = (self.sr & 0x1) != 0;
				let shift = size.bits();
				let full_val = if x { dst_val | (1 << shift) } else { dst_val };
				let result = full_val.rotate_right(1) & size.mask();
				self.set_operand_value(size, &dst, result);
				let n = (result & (1 << (size.bits() - 1))) != 0;
				let z = result == 0;
				let c = (dst_val & 1) != 0;
				self.set_flags(n, z, false, c);
				self.sr = (self.sr & 0xFFFE) | (if c { 1 } else { 0 });
				cycles = 8 + self.ea_cycles(&dst, size);
			}
			Operation::Trapv => {
				if (self.sr & 0x2) != 0 {
					cycles = self.trigger_exception(7); // TRAPV, vector 7
				} else {
					cycles = 4; // No trap
				}
			}
			Operation::Reset => {
				if !self.check_supervisor() {
					cycles = 34; // Trap cycles
				} else {
					self.pc = self.memory.read_long(4);
					self.sr = 0x2700;
					self.d = [0; 8];
					self.a = [0; 8];
					self.a[7] = self.memory.read_long(0);
					cycles = 132; // Correct: External reset timing
				}
			}
			Operation::Rte => {
				if !self.check_supervisor() {
					cycles = 34; // Trap cycles
				} else {
					self.sr = self.memory.read_word(self.a[7]);
					self.a[7] += 2;
					self.pc = self.memory.read_long(self.a[7]);
					self.a[7] += 4;
					self.interrupt_nest_level = self.interrupt_nest_level.saturating_sub(1);
					cycles = 20; // Correct: 10n
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
				} + self.ea_cycles(&src, size);
			}
			Operation::MoveCcr => {
				let src = instr.src.unwrap();
				let value = self.get_operand_value(Size::Byte, &src) as u16;
				self.sr = (self.sr & 0xFF00) | (value & 0x1F);
				cycles = 12 + self.ea_cycles(&src, Size::Byte); // Correct: 6n
			}
			Operation::MoveSr => {
				if !self.check_supervisor() {
					cycles = 34; // Trap cycles
				} else {
					let src = instr.src.unwrap();
					let value = self.get_operand_value(Size::Word, &src) as u16;
					self.sr = value;
					cycles = 12 + self.ea_cycles(&src, Size::Word); // Correct: 6n
				}
			}
			Operation::MoveUsp => {
				if !self.check_supervisor() {
					cycles = 34; // Trap cycles
				} else {
					let src = instr.src.unwrap();
					let dst = instr.dst.unwrap();
					if let Operand::AddressRegister(an) = src {
						self.a[7] = self.a[an as usize];
					} else if let Operand::AddressRegister(an) = dst {
						self.a[an as usize] = self.a[7];
					}
					cycles = 4; // Correct: 2n
				}
			}
			Operation::AndiCcr => {
				if !self.check_supervisor() {
					cycles = 34; // Trap cycles
				} else {
					let src = instr.src.unwrap();
					let value = self.get_operand_value(Size::Byte, &src) as u16;
					self.sr = (self.sr & 0xFF00) | ((self.sr & 0x1F) & (value & 0x1F));
					cycles = 20; // Correct: 10n
				}
			}
			Operation::OriCcr => {
				if !self.check_supervisor() {
					cycles = 34; // Trap cycles
				} else {
					let src = instr.src.unwrap();
					let value = self.get_operand_value(Size::Byte, &src) as u16;
					self.sr = (self.sr & 0xFF00) | ((self.sr & 0x1F) | (value & 0x1F));
					cycles = 20; // Correct: 10n
				}
			}
			Operation::EoriCcr => {
				if !self.check_supervisor() {
					cycles = 34; // Trap cycles
				} else {
					let src = instr.src.unwrap();
					let value = self.get_operand_value(Size::Byte, &src) as u16;
					self.sr = (self.sr & 0xFF00) | ((self.sr & 0x1F) ^ (value & 0x1F));
					cycles = 20; // Correct: 10n
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
				self.set_flags(n, z, false, c);
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
				let c = if shift_count > 0 { (dst_val >> (shift_count - 1)) & 1 != 0 } else { false };
				self.set_flags(n, z, false, c);
				cycles = match size {
					Size::Byte | Size::Word => 6,
					Size::Long => 8,
				} + 2 * shift_count as u32;
			}
		}
		cycles
	}

    /// Execute one CPU step and return cycles used
    pub fn step(&mut self) -> u32 {
		if self.halted && self.pending_interrupts.is_empty() {
			return 0; // No cycles if halted and no interrupt
		}
		// Check for pending interrupts
		if let Some(&(level, vector)) = self.pending_interrupts.first() {
			let current_ipl = (self.sr >> 8) & 0x7;
			if level > current_ipl || (level == 7 && self.interrupt_nest_level < 7) {
				let cycles = self.process_interrupt(level, vector);
				self.cycle_count += cycles as u64;
				return cycles;
			}
		}
		// Normal instruction execution
		let instr = self.decode();
		let cycles = self.execute(instr);
		self.cycle_count += cycles as u64;
		self.interrupt_ack = None; // Reset acknowledgment after instruction
		cycles
	}

    /// Load a program into memory at a given address
    pub fn load_program(&mut self, address: u32, program: &[u8]) {
        for (i, &byte) in program.iter().enumerate() {
            self.memory.write_byte(address + i as u32, byte);
        }
        self.pc = address;
    }
}
