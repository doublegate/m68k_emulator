// emulator.rs
use crate::m68k_cpu::CPU;
use crate::memory::Memory;

pub struct Emulator {
    cpu: CPU,
    // Add Z80, VDP, sound, I/O later
}

impl Emulator {
    pub fn new(rom_data: Vec<u8>) -> Emulator {
		let memory = Memory::new(rom_data);
		let mut cpu = CPU::new(memory);
		cpu.pc = cpu.memory.read_long(4).unwrap_or(0); // Handle errors as needed
		cpu.a[7] = cpu.memory.read_long(0).unwrap_or(0);
		cpu.prefetch();
		Emulator { cpu }
	}

    pub fn step(&mut self) -> u32 {
        self.cpu.step()
    }

    pub fn run_frame(&mut self) {
        let cycles_per_frame = 7_670_000 / 60; // ~60 FPS at 7.67 MHz
        let mut cycles = 0;
        while cycles < cycles_per_frame {
            cycles += self.step();
        }
    }
}
