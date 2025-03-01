// src/memory.rs - Memory subsystem for the 68K emulator.
//
// This module defines the Memory struct and associated methods for accessing
// memory regions in the 68K emulator. Memory is divided into two main regions:
//   1. ROM: A read-only region expected to cover addresses 0x000000 to 0x3FFFFF.
//   2. RAM: A writable region covering addresses 0xFF0000 to 0xFFFFFF.
// The module also defines an Exception enum to handle error conditions such as
// address out-of-bounds or misaligned access.

#[derive(Debug)]
pub enum Exception {
    AddressError, // Indicates an error with addressing (e.g. out-of-bound or misaligned)
}

#[derive(Clone)]
pub struct Memory {
    pub rom: Vec<u8>,         // ROM region: typically loaded from an external binary.
    pub ram: Box<[u8; 0x10000]>, // RAM region: a fixed-size 64KB array allocated on the heap.
    // Future expansion may add support for additional regions such as Z80, VDP, or I/O.
}

impl Memory {
    /// Creates a new Memory instance.
    ///
    /// # Parameters
    /// - `rom_data`: A vector of bytes representing the entire ROM contents.
    ///   Ideally, the length should cover the full ROM region (e.g. 0x400000 bytes for 4 MB).
    ///
    /// # Returns
    /// A new Memory instance with the ROM region initialized to `rom_data`
    /// and the RAM region zero-initialized.
    pub fn new(rom_data: Vec<u8>) -> Self {
        Memory {
            rom: rom_data,
            ram: Box::new([0; 0x10000]), // Initialize 64KB of RAM to zeros.
        }
    }
    
    /// Loads data directly into the ROM region.
    ///
    /// # Parameters
    /// - `address`: The start address within the ROM region to write the data.
    /// - `data`: The slice of bytes to load into the ROM.
    ///
    /// # Returns
    /// - `Ok(())` if the data is successfully loaded.
    /// - `Err(Exception::AddressError)` if the data would exceed the bounds of the ROM.
    ///
    /// # Notes
    /// Although ROM is read-only during normal emulation, this method is intended
    /// for initial loading of programs or firmware into memory.
    pub fn load_rom_data(&mut self, address: u32, data: &[u8]) -> Result<(), Exception> {
        if address + data.len() as u32 > self.rom.len() as u32 {
            return Err(Exception::AddressError);
        }
        self.rom[address as usize .. (address as usize + data.len())].copy_from_slice(data);
        Ok(())
    }
    
    /// Reads a byte from the given address.
    ///
    /// # Parameters
    /// - `addr`: The 32-bit memory address from which to read.
    ///
    /// # Returns
    /// - `Ok(u8)`: The byte value read from the address.
    /// - `Err(Exception::AddressError)`: If the address is not within a valid memory region.
    ///
    /// # Memory Regions:
    /// - ROM is accessible from addresses 0x000000 to 0x3FFFFF.
    /// - RAM is accessible from addresses 0xFF0000 to 0xFFFFFF.
    pub fn read_byte(&self, addr: u32) -> Result<u8, Exception> {
        match addr {
            // For ROM, check that the address is within the ROM vector length.
            0x000000..=0x3FFFFF => {
                if (addr as usize) < self.rom.len() {
                    Ok(self.rom[addr as usize])
                } else {
                    Err(Exception::AddressError)
                }
            },
            // For RAM, use bitmasking to map the address into the 64KB array.
            0xFF0000..=0xFFFFFF => Ok(self.ram[(addr & 0xFFFF) as usize]),
            // Any access outside defined regions returns an AddressError.
            _ => Err(Exception::AddressError),
        }
    }

    /// Writes a byte to the specified address.
    ///
    /// # Parameters
    /// - `addr`: The 32-bit memory address to which the byte should be written.
    /// - `value`: The byte value to write.
    ///
    /// # Returns
    /// - `Ok(())`: On a successful write.
    /// - `Err(Exception::AddressError)`: If the address is outside the writable region.
    ///
    /// # Notes
    /// Only the RAM region (0xFF0000 to 0xFFFFFF) is writable; attempts to write to ROM return an error.
    pub fn write_byte(&mut self, addr: u32, value: u8) -> Result<(), Exception> {
        match addr {
            0xFF0000..=0xFFFFFF => {
                // The RAM index is obtained by masking the lower 16 bits of the address.
                self.ram[(addr & 0xFFFF) as usize] = value;
                Ok(())
            }
            _ => Err(Exception::AddressError),
        }
    }
    
    /// Reads a 16-bit word from the specified address.
    ///
    /// # Parameters
    /// - `addr`: The starting address (32-bit) from which to read a word.
    ///
    /// # Returns
    /// - `Ok(u16)`: The 16-bit word read from the address.
    /// - `Err(Exception::AddressError)`: If the address is misaligned or in an invalid region.
    ///
    /// # Alignment Requirement
    /// Word reads must be performed from an even address.
    ///
    /// # Implementation
    /// Reads two consecutive bytes (high, then low) and combines them into one word.
    pub fn read_word(&self, addr: u32) -> Result<u16, Exception> {
        if addr & 1 != 0 {
            Err(Exception::AddressError)
        } else {
            let high = self.read_byte(addr)? as u16;
            let low = self.read_byte(addr + 1)? as u16;
            Ok((high << 8) | low)
        }
    }
    
    /// Writes a 16-bit word to the specified address.
    ///
    /// # Parameters
    /// - `addr`: The starting address (32-bit) for the write operation.
    /// - `value`: The 16-bit word to write.
    ///
    /// # Returns
    /// - `Ok(())`: On a successful write.
    /// - `Err(Exception::AddressError)`: If the address is misaligned.
    ///
    /// # Notes
    /// The method writes the high-order byte at `addr` and the low-order byte at `addr + 1`.
    pub fn write_word(&mut self, addr: u32, value: u16) -> Result<(), Exception> {
        if addr & 1 != 0 {
            Err(Exception::AddressError)
        } else {
            self.write_byte(addr, (value >> 8) as u8)?;
            self.write_byte(addr + 1, value as u8)?;
            Ok(())
        }
    }
    
    /// Reads a 32-bit long word from the specified address.
    ///
    /// # Parameters
    /// - `addr`: The start address (32-bit) from which to read the long word.
    ///
    /// # Returns
    /// - `Ok(u32)`: The 32-bit long word read from the memory.
    /// - `Err(Exception::AddressError)`: If the address is misaligned or not within a valid region.
    ///
    /// # Alignment Requirement
    /// The starting address must be even.
    ///
    /// # Implementation Details
    /// For addresses in the ROM region (0x000000 to 0x3FFFFF), this method verifies that the entire
    /// four-byte sequence is available. It then constructs the 32-bit value from four bytes in big-endian order.
    /// For addresses in the RAM region (0xFF0000 to 0xFFFFFF), the address is masked and the four bytes
    /// are similarly assembled.
    pub fn read_long(&self, addr: u32) -> Result<u32, Exception> {
        if addr & 1 != 0 {
            return Err(Exception::AddressError);
        }
        
        match addr {
            // For ROM: check that addr+3 is within bounds, then combine 4 consecutive bytes.
            0x000000..=0x3FFFFF if (addr + 3) < self.rom.len() as u32 => {
                let i = addr as usize;
                Ok(((self.rom[i] as u32) << 24) | 
                   ((self.rom[i+1] as u32) << 16) | 
                   ((self.rom[i+2] as u32) << 8) | 
                   (self.rom[i+3] as u32))
            },
            // For RAM: use masked address to access the 64KB RAM array and combine bytes.
            0xFF0000..=0xFFFFFF => {
                let i = (addr & 0xFFFF) as usize;
                Ok(((self.ram[i] as u32) << 24) | 
                   ((self.ram[i+1] as u32) << 16) | 
                   ((self.ram[i+2] as u32) << 8) | 
                   (self.ram[i+3] as u32))
            },
            // All other addresses are invalid for a long word read.
            _ => Err(Exception::AddressError),
        }
    }
    
    /// Writes a 32-bit long word to the specified address.
    ///
    /// # Parameters
    /// - `addr`: The starting 32-bit address where the long word should be written.
    /// - `value`: The 32-bit value to write into memory.
    ///
    /// # Returns
    /// - `Ok(())`: On successful completion.
    /// - `Err(Exception::AddressError)`: If the address is misaligned.
    ///
    /// # Implementation Details
    /// The 32-bit value is split into two 16-bit words. The high word is written first at `addr`
    /// and the low word is written at `addr + 2`.
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
