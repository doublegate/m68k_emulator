// memory.rs
#[derive(Debug)]
pub enum Exception {
    AddressError,
}

#[derive(Clone)]
pub struct Memory {
    pub rom: Vec<u8>,        // 0x000000-0x3FFFFF (up to 4MB)
    pub ram: [u8; 0x10000],  // 0xFF0000-0xFFFFFF (64KB)
    // Add Z80, VDP, I/O regions later
}

impl Memory {
    pub fn new(rom_data: Vec<u8>) -> Self {
        Memory {
            rom: rom_data,
            ram: [0; 0x10000],  // Initialize RAM (e.g., 64KB)
        }
    }
	
	pub fn read_byte(&self, addr: u32) -> Result<u8, Exception> {
        match addr {
            0x000000..=0x3FFFFF => Ok(self.rom[addr as usize]),
            0xFF0000..=0xFFFFFF => Ok(self.ram[(addr & 0xFFFF) as usize]),
            _ => Err(Exception::AddressError),
        }
    }

    pub fn write_byte(&mut self, addr: u32, value: u8) -> Result<(), Exception> {
        match addr {
            0xFF0000..=0xFFFFFF => {
                self.ram[(addr & 0xFFFF) as usize] = value;
                Ok(())
            }
            _ => Err(Exception::AddressError),
        }
    }

    pub fn read_word(&self, addr: u32) -> Result<u16, Exception> {
        if addr & 1 != 0 {
            Err(Exception::AddressError)
        } else {
            let high = self.read_byte(addr)? as u16;
            let low = self.read_byte(addr + 1)? as u16;
            Ok((high << 8) | low)
        }
    }

    pub fn write_word(&mut self, addr: u32, value: u16) -> Result<(), Exception> {
        if addr & 1 != 0 {
            Err(Exception::AddressError)
        } else {
            self.write_byte(addr, (value >> 8) as u8)?;
            self.write_byte(addr + 1, value as u8)?;
            Ok(())
        }
    }

    pub fn read_long(&self, addr: u32) -> Result<u32, Exception> {
        if addr & 1 != 0 {
            Err(Exception::AddressError)
        } else {
            let high = self.read_word(addr)? as u32;
            let low = self.read_word(addr + 2)? as u32;
            Ok((high << 16) | low)
        }
    }

    pub fn write_long(&mut self, addr: u32, value: u32) -> Result<(), Exception> {
        if addr & 1 != 0 {
            Err(Exception::AddressError)
        } else {
            self.write_word(addr, (value >> 16) as u16)?;
            self.write_word(addr + 2, value as u16)?;
            Ok(())
        }
    }
}
