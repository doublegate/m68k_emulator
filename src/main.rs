// main.rs
mod m68k_cpu;
mod memory;
mod emulator;

use std::fs;
use emulator::Emulator;

fn main() {
    let rom = fs::read("roms/game.bin").expect("Failed to load ROM");
    let mut emu = Emulator::new(rom);
    loop {
        emu.run_frame();
        // Add rendering, sound, input later
    }
}
