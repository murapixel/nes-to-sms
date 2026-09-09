//! iNES / NES 2.0 ROM parsing.
//!
//! Lossless: keeps the original bytes around. Returns slices, not copies.

use std::fmt;

use sha2::{Digest, Sha256};

pub const INES_MAGIC: [u8; 4] = [b'N', b'E', b'S', 0x1a];
pub const PRG_BANK_SIZE: usize = 16 * 1024;
pub const CHR_BANK_SIZE: usize = 8 * 1024;
/// MMC3 PRG banking granularity: two switchable 8 KiB windows plus a fixed
/// 16 KiB top (see `MapperPolicy::Mmc3`).
pub const MMC3_PRG_WINDOW_SIZE: usize = 8 * 1024;
/// Smallest MMC3 PRG payload (8 x 8 KiB banks); largest is 64 x 8 KiB.
pub const MMC3_MIN_8K_BANKS: usize = 8;
pub const MMC3_MAX_8K_BANKS: usize = 64;
pub const TRAINER_SIZE: usize = 512;
pub const HEADER_SIZE: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mirroring {
    Horizontal,
    Vertical,
    FourScreen,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaderKind {
    INes,
    Nes2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    pub kind: HeaderKind,
    pub prg_banks: u16,
    pub chr_banks: u16,
    /// Volatile PRG-RAM capacity in bytes.
    pub prg_ram_size: usize,
    /// Nonvolatile PRG-RAM capacity in bytes.
    pub prg_nvram_size: usize,
    /// Volatile CHR-RAM capacity in bytes.
    pub chr_ram_size: usize,
    /// Nonvolatile CHR-RAM capacity in bytes.
    pub chr_nvram_size: usize,
    pub mapper: u16,
    pub submapper: u8,
    pub mirroring: Mirroring,
    pub has_trainer: bool,
    pub has_battery: bool,
}

impl Header {
    pub fn prg_len(&self) -> usize {
        self.prg_banks as usize * PRG_BANK_SIZE
    }
    pub fn chr_len(&self) -> usize {
        self.chr_banks as usize * CHR_BANK_SIZE
    }
}

fn nes2_ram_size(nibble: u8) -> usize {
    if nibble == 0 { 0 } else { 64usize << nibble }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    TooShort,
    BadMagic,
    Truncated { expected: usize, actual: usize },
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParseError::TooShort => write!(f, "ROM is shorter than 16-byte header"),
            ParseError::BadMagic => write!(f, "missing NES<EOF> magic"),
            ParseError::Truncated { expected, actual } => write!(
                f,
                "ROM truncated: expected {expected} bytes after header, got {actual}"
            ),
        }
    }
}

impl std::error::Error for ParseError {}

/// Parse the 16-byte iNES / NES 2.0 header. Does not validate body length.
pub fn parse_header(rom: &[u8]) -> Result<Header, ParseError> {
    if rom.len() < HEADER_SIZE {
        return Err(ParseError::TooShort);
    }
    if rom[0..4] != INES_MAGIC {
        return Err(ParseError::BadMagic);
    }
    let flags6 = rom[6];
    let flags7 = rom[7];
    let kind = if (flags7 & 0x0c) == 0x08 {
        HeaderKind::Nes2
    } else {
        HeaderKind::INes
    };
    let (
        prg_banks,
        chr_banks,
        submapper,
        prg_ram_size,
        prg_nvram_size,
        chr_ram_size,
        chr_nvram_size,
    ) = match kind {
        HeaderKind::Nes2 => {
            let prg_lo = rom[4] as u16;
            let chr_lo = rom[5] as u16;
            let size_msb = rom[9] as u16;
            let prg_hi = size_msb & 0x0f;
            let chr_hi = (size_msb >> 4) & 0x0f;
            let prg = (prg_hi << 8) | prg_lo;
            let chr = (chr_hi << 8) | chr_lo;
            let submapper = (rom[8] >> 4) & 0x0f;
            let prg_ram = nes2_ram_size(rom[10] & 0x0f);
            let prg_nvram = nes2_ram_size(rom[10] >> 4);
            let chr_ram = nes2_ram_size(rom[11] & 0x0f);
            let chr_nvram = nes2_ram_size(rom[11] >> 4);
            (prg, chr, submapper, prg_ram, prg_nvram, chr_ram, chr_nvram)
        }
        HeaderKind::INes => {
            let prg_ram = if rom[8] == 0 {
                8 * 1024
            } else {
                rom[8] as usize * 8 * 1024
            };
            let chr_ram = if rom[5] == 0 { 8 * 1024 } else { 0 };
            (rom[4] as u16, rom[5] as u16, 0, prg_ram, 0, chr_ram, 0)
        }
    };
    let mapper_lo = (flags6 >> 4) as u16;
    let mapper_hi = (flags7 & 0xf0) as u16;
    let mapper = mapper_hi | mapper_lo;
    let mirroring = if flags6 & 0x08 != 0 {
        Mirroring::FourScreen
    } else if flags6 & 0x01 != 0 {
        Mirroring::Vertical
    } else {
        Mirroring::Horizontal
    };
    Ok(Header {
        kind,
        prg_banks,
        chr_banks,
        prg_ram_size,
        prg_nvram_size,
        chr_ram_size,
        chr_nvram_size,
        mapper,
        submapper,
        mirroring,
        has_trainer: flags6 & 0x04 != 0,
        has_battery: flags6 & 0x02 != 0,
    })
}

/// A parsed iNES image with borrowed PRG/CHR slices.
#[derive(Debug)]
pub struct Image<'a> {
    pub header: Header,
    pub trainer: Option<&'a [u8]>,
    pub prg: &'a [u8],
    pub chr: &'a [u8],
}

pub fn parse<'a>(rom: &'a [u8]) -> Result<Image<'a>, ParseError> {
    let header = parse_header(rom)?;
    let trainer_len = if header.has_trainer { TRAINER_SIZE } else { 0 };
    let prg_start = HEADER_SIZE + trainer_len;
    let prg_end = prg_start + header.prg_len();
    let chr_end = prg_end + header.chr_len();
    if rom.len() < chr_end {
        return Err(ParseError::Truncated {
            expected: chr_end,
            actual: rom.len(),
        });
    }
    let trainer = if header.has_trainer {
        Some(&rom[HEADER_SIZE..HEADER_SIZE + TRAINER_SIZE])
    } else {
        None
    };
    let prg = &rom[prg_start..prg_end];
    let chr = &rom[prg_end..chr_end];
    Ok(Image {
        header,
        trainer,
        prg,
        chr,
    })
}

/// Return the lowercase SHA-256 identity of the canonical ROM payload:
/// PRG bytes followed immediately by CHR-ROM bytes. Header and trainer data
/// are intentionally excluded.
pub fn payload_sha256_hex(prg: &[u8], chr: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(prg);
    hasher.update(chr);
    format!("{:x}", hasher.finalize())
}

/// Supported CPU-to-PRG mapping policies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapperPolicy {
    /// Mapper 0 with either one mirrored or two directly mapped 16 KiB banks.
    Nrom { prg_len: usize },
    /// Mapper 2 with a switchable lower bank and a fixed final upper bank.
    Uxrom {
        bank_count: u8,
        bus_conflicts: UxromBusConflicts,
    },
    /// Mapper 4 (MMC3): two switchable 8 KiB PRG windows plus a fixed 16 KiB
    /// top. `prg_8k_count` is the number of 8 KiB PRG banks (8..=64).
    /// Live window contents additionally depend on the R6/R7 registers and
    /// the PRG-mode bit; see `Mmc3State`. Code that only has a single UxROM
    /// `selected_bank` must treat `$8000-$BFFF` as unknown (fail closed).
    Mmc3 { prg_8k_count: u8 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UxromBusConflicts {
    None,
    And,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapperPolicyError {
    UnsupportedMapper {
        mapper: u16,
    },
    UxromRequiresNes2,
    UnsupportedUxromSubmapper {
        submapper: u8,
    },
    InvalidNromPrgLayout {
        prg_len: usize,
    },
    InvalidUxromPrgLayout {
        prg_len: usize,
    },
    SelectedBankOutOfRange {
        bank: u8,
        bank_count: u8,
    },
    InvalidMmc3PrgLayout {
        prg_len: usize,
    },
    SelectedMmc3BankOutOfRange {
        bank: u8,
        bank_count: u8,
    },
    /// MMC3 reads/writes that need live ($8000-select, $8001-data, PRG-mode)
    /// window state instead of a single UxROM-style selected bank.
    Mmc3WindowStateRequired,
}

impl fmt::Display for MapperPolicyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedMapper { mapper } => {
                write!(
                    f,
                    "unsupported mapper {mapper}; only mapper 0 (NROM), mapper 2 (UxROM) and mapper 4 (MMC3 loader) are supported"
                )
            }
            Self::UxromRequiresNes2 => {
                write!(f, "mapper 2 requires a NES 2.0 submapper declaration")
            }
            Self::UnsupportedUxromSubmapper { submapper } => write!(
                f,
                "unsupported mapper 2 NES 2.0 submapper {submapper}; only 1 (no conflict) and 2 (AND conflict) are supported"
            ),
            Self::InvalidNromPrgLayout { prg_len } => write!(
                f,
                "invalid NROM PRG layout: expected exactly 16384 or 32768 bytes, got {prg_len}"
            ),
            Self::InvalidUxromPrgLayout { prg_len } => write!(
                f,
                "invalid UxROM PRG layout: expected 2 through 16 16-KiB banks, got {prg_len} bytes"
            ),
            Self::SelectedBankOutOfRange { bank, bank_count } => write!(
                f,
                "UxROM selected bank {bank} is out of range for {bank_count} banks"
            ),
            Self::InvalidMmc3PrgLayout { prg_len } => write!(
                f,
                "invalid MMC3 PRG layout: expected 8 KiB-aligned 64..512 KiB, got {prg_len} bytes"
            ),
            Self::SelectedMmc3BankOutOfRange { bank, bank_count } => write!(
                f,
                "MMC3 selected bank {bank} is out of range for {bank_count} 8 KiB banks"
            ),
            Self::Mmc3WindowStateRequired => write!(
                f,
                "MMC3 window needs live R6/R7 + PRG-mode state; a single UxROM-style bank is not enough"
            ),
        }
    }
}

impl std::error::Error for MapperPolicyError {}

/// Resolve the supported mapper policy for a parsed ROM's PRG payload.
pub fn resolve_mapper_policy(
    header: &Header,
    prg_len: usize,
) -> Result<MapperPolicy, MapperPolicyError> {
    match header.mapper {
        0 if prg_len == PRG_BANK_SIZE || prg_len == 2 * PRG_BANK_SIZE => {
            Ok(MapperPolicy::Nrom { prg_len })
        }
        0 => Err(MapperPolicyError::InvalidNromPrgLayout { prg_len }),
        2 if header.kind != HeaderKind::Nes2 => Err(MapperPolicyError::UxromRequiresNes2),
        2 if prg_len % PRG_BANK_SIZE == 0 => {
            let bank_count = prg_len / PRG_BANK_SIZE;
            let bus_conflicts = match header.submapper {
                1 => UxromBusConflicts::None,
                2 => UxromBusConflicts::And,
                submapper => {
                    return Err(MapperPolicyError::UnsupportedUxromSubmapper { submapper });
                }
            };
            if matches!(bank_count, 2 | 4 | 8 | 16) {
                Ok(MapperPolicy::Uxrom {
                    bank_count: bank_count as u8,
                    bus_conflicts,
                })
            } else {
                Err(MapperPolicyError::InvalidUxromPrgLayout { prg_len })
            }
        }
        2 => Err(MapperPolicyError::InvalidUxromPrgLayout { prg_len }),
        4 if prg_len.is_multiple_of(MMC3_PRG_WINDOW_SIZE) => {
            let banks_8k = prg_len / MMC3_PRG_WINDOW_SIZE;
            if (MMC3_MIN_8K_BANKS..=MMC3_MAX_8K_BANKS).contains(&banks_8k) {
                Ok(MapperPolicy::Mmc3 {
                    prg_8k_count: banks_8k as u8,
                })
            } else {
                Err(MapperPolicyError::InvalidMmc3PrgLayout { prg_len })
            }
        }
        4 => Err(MapperPolicyError::InvalidMmc3PrgLayout { prg_len }),
        mapper => Err(MapperPolicyError::UnsupportedMapper { mapper }),
    }
}

impl MapperPolicy {
    pub fn is_banked(self) -> bool {
        matches!(self, Self::Uxrom { .. } | Self::Mmc3 { .. })
    }

    /// Bank count in this policy's native units: 16 KiB banks for UxROM,
    /// 8 KiB banks for MMC3, 16 KiB units for NROM. Callers that iterate
    /// 16 KiB `prg_bank` windows must reject `Mmc3` explicitly first.
    pub fn bank_count(self) -> u8 {
        match self {
            Self::Nrom { prg_len } => (prg_len / PRG_BANK_SIZE) as u8,
            Self::Uxrom { bank_count, .. } => bank_count,
            Self::Mmc3 { prg_8k_count } => prg_8k_count,
        }
    }

    /// Number of 8 KiB PRG banks, if this is an MMC3 policy.
    pub fn mmc3_8k_bank_count(self) -> Option<u8> {
        match self {
            Self::Mmc3 { prg_8k_count } => Some(prg_8k_count),
            _ => None,
        }
    }

    fn checked_bank_offset(self, bank: u8) -> Result<usize, MapperPolicyError> {
        match self {
            Self::Nrom { .. } => Ok(0),
            Self::Uxrom { bank_count, .. } if bank < bank_count => {
                Ok(bank as usize * PRG_BANK_SIZE)
            }
            Self::Uxrom { bank_count, .. } => {
                Err(MapperPolicyError::SelectedBankOutOfRange { bank, bank_count })
            }
            Self::Mmc3 { prg_8k_count } if bank < prg_8k_count => {
                Ok(bank as usize * MMC3_PRG_WINDOW_SIZE)
            }
            Self::Mmc3 {
                prg_8k_count: bank_count,
            } => Err(MapperPolicyError::SelectedMmc3BankOutOfRange { bank, bank_count }),
        }
    }

    /// Map a CPU address using the UxROM lower-window selection when needed.
    /// For MMC3 only the fixed `$C000-$FFFF` top is mappable with a bare
    /// bank index; `$8000-$BFFF` needs `Mmc3State` and returns `Ok(None)`.
    pub fn cpu_to_prg_offset(
        self,
        cpu_addr: u16,
        selected_bank: u8,
    ) -> Result<Option<usize>, MapperPolicyError> {
        if cpu_addr < 0x8000 {
            return Ok(None);
        }
        match self {
            Self::Nrom { prg_len } => {
                let offset = (cpu_addr - 0x8000) as usize;
                Ok(Some(if prg_len == PRG_BANK_SIZE {
                    offset & (PRG_BANK_SIZE - 1)
                } else {
                    offset
                }))
            }
            Self::Uxrom { bank_count, .. } => {
                let bank_offset = self.checked_bank_offset(selected_bank)?;
                if cpu_addr < 0xC000 {
                    Ok(Some(bank_offset + (cpu_addr as usize - 0x8000)))
                } else {
                    Ok(Some(
                        (bank_count as usize - 1) * PRG_BANK_SIZE + (cpu_addr as usize - 0xC000),
                    ))
                }
            }
            Self::Mmc3 { prg_8k_count } => {
                // Validate the bare index so vector reads fail closed, but the
                // switchable windows still need live R6/R7 + PRG-mode state.
                if selected_bank >= prg_8k_count {
                    return Err(MapperPolicyError::SelectedMmc3BankOutOfRange {
                        bank: selected_bank,
                        bank_count: prg_8k_count,
                    });
                }
                if cpu_addr < 0xC000 {
                    Ok(None)
                } else {
                    let prg_len = prg_8k_count as usize * MMC3_PRG_WINDOW_SIZE;
                    Ok(Some(prg_len - 0x4000 + (cpu_addr as usize - 0xC000)))
                }
            }
        }
    }

    /// Return the requested switchable bank: 16 KiB for UxROM, 8 KiB for MMC3.
    pub fn prg_bank<'a>(self, prg: &'a [u8], bank: u8) -> Result<&'a [u8], MapperPolicyError> {
        let offset = self.checked_bank_offset(bank)?;
        let len = match self {
            Self::Mmc3 { .. } => MMC3_PRG_WINDOW_SIZE,
            _ => PRG_BANK_SIZE,
        };
        prg.get(offset..offset + len).ok_or(match self {
            Self::Mmc3 {
                prg_8k_count: bank_count,
            } => MapperPolicyError::SelectedMmc3BankOutOfRange { bank, bank_count },
            Self::Uxrom { bank_count, .. } => {
                MapperPolicyError::SelectedBankOutOfRange { bank, bank_count }
            }
            Self::Nrom { prg_len } => MapperPolicyError::InvalidNromPrgLayout { prg_len },
        })
    }

    /// Return the PRG bytes visible in the fixed `$C000-$FFFF` window.
    /// For MMC3 this is the last 16 KiB (two 8 KiB banks), matching the
    /// hardware's fixed top regardless of PRG-mode.
    pub fn fixed_prg<'a>(self, prg: &'a [u8]) -> &'a [u8] {
        match self {
            Self::Nrom { prg_len } if prg_len == PRG_BANK_SIZE => prg,
            Self::Nrom { .. } => &prg[PRG_BANK_SIZE..],
            Self::Uxrom { bank_count, .. } => {
                let offset = (bank_count as usize - 1) * PRG_BANK_SIZE;
                &prg[offset..offset + PRG_BANK_SIZE]
            }
            Self::Mmc3 { prg_8k_count } => {
                let prg_len = prg_8k_count as usize * MMC3_PRG_WINDOW_SIZE;
                &prg[prg_len - 2 * MMC3_PRG_WINDOW_SIZE..]
            }
        }
    }

    /// Return the bytes in NROM's lower $8000-$BFFF window.
    pub fn lower_prg<'a>(self, prg: &'a [u8]) -> &'a [u8] {
        &prg[..PRG_BANK_SIZE]
    }

    /// Construct the NROM-shaped analysis view for a selected UxROM bank.
    /// MMC3 needs live R6/R7 + PRG-mode window state, so this fails closed
    /// with `Mmc3WindowStateRequired` instead of guessing a window.
    pub fn analysis_view(
        self,
        prg: &[u8],
        selected_bank: u8,
    ) -> Result<Vec<u8>, MapperPolicyError> {
        match self {
            Self::Nrom { .. } => Ok(prg.to_vec()),
            Self::Uxrom { .. } => {
                let mut view = Vec::with_capacity(2 * PRG_BANK_SIZE);
                view.extend_from_slice(self.prg_bank(prg, selected_bank)?);
                view.extend_from_slice(self.fixed_prg(prg));
                Ok(view)
            }
            Self::Mmc3 { .. } => Err(MapperPolicyError::Mmc3WindowStateRequired),
        }
    }

    pub fn uxrom_bus_conflicts(self) -> Option<UxromBusConflicts> {
        match self {
            Self::Nrom { .. } => None,
            Self::Uxrom { bus_conflicts, .. } => Some(bus_conflicts),
            Self::Mmc3 { .. } => None,
        }
    }

    /// Apply a mapper write with its pre-write ROM bus byte.
    /// MMC3 uses paired `$8000`-select / `$8001`-data writes plus mode/IRQ
    /// registers; a single UxROM-style byte is not enough, so this fails
    /// closed with `Mmc3WindowStateRequired` (use `Mmc3State` instead).
    pub fn selected_bank_from_write(
        self,
        raw_write: u8,
        pre_write_rom_byte: u8,
    ) -> Result<u8, MapperPolicyError> {
        match self {
            Self::Nrom { .. } => Ok(0),
            Self::Uxrom {
                bank_count,
                bus_conflicts,
            } => {
                let effective = match bus_conflicts {
                    UxromBusConflicts::None => raw_write,
                    UxromBusConflicts::And => raw_write & pre_write_rom_byte,
                };
                let bank = effective & (bank_count - 1);
                self.checked_bank_offset(bank)?;
                Ok(bank)
            }
            Self::Mmc3 { .. } => Err(MapperPolicyError::Mmc3WindowStateRequired),
        }
    }
}

/// Live MMC3 ($8000-$FFFF) register state for PRG windows, mirroring and the
/// scanline IRQ latch. Pure logic, no ROM bytes: mirrors the NESdev
/// Programming-MMC3 register map so the reference bus, analysis and runtime
/// can share one fail-closed model.
///
/// Address decode follows the hardware: even addresses in `$8000-$9FFE`
/// select the bank register, odd addresses write its data; `$A000` even
/// sets nametable arrangement, odd sets PRG-RAM protect; `$C000` even sets
/// the IRQ latch, odd reloads; `$E000` even disables IRQs, odd enables them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mmc3State {
    /// Last `$8000` value: bit 7 = CHR A12 invert, bit 6 = PRG mode,
    /// bits 2..=0 = R0-R7 select.
    pub bank_select: u8,
    /// R0-R7 bank registers (CHR R0/R1 are 2 KiB units, R2-R5 are 1 KiB,
    /// PRG R6/R7 are 8 KiB).
    pub regs: [u8; 8],
    /// Nametable arrangement from `$A000` bit 0 (false = vertical, true =
    /// horizontal). Boards with hardwired 4-screen VRAM ignore it.
    pub horizontal_mirroring: bool,
    /// PRG-RAM write protect from `$A001`.
    pub prg_ram_protect: u8,
    /// IRQ latch from `$C000`.
    pub irq_latch: u8,
    /// IRQ enabled by `$E001`, cleared by `$E000` or reset.
    pub irq_enabled: bool,
    /// Set by `$C001`; the next A12 clock reloads the counter from the latch.
    pub irq_reload_pending: bool,
    /// Down-counter clocked by PPU A12 rises while rendering. MMC3B/C
    /// semantics (the common silicon; MMC3A differs on latch-0 reload
    /// timing — noted, not modeled).
    pub irq_counter: u8,
    /// Level-asserted while set: the mapper holds IRQ low until the game
    /// acknowledges with an `$E000` write. Cleared only there (or reset).
    pub irq_pending: bool,
}

impl Default for Mmc3State {
    fn default() -> Self {
        Self {
            bank_select: 0,
            regs: [0, 2, 4, 5, 6, 7, 0, 1],
            horizontal_mirroring: false,
            prg_ram_protect: 0,
            irq_latch: 0,
            irq_enabled: false,
            irq_reload_pending: false,
            irq_counter: 0,
            irq_pending: false,
        }
    }
}

impl Mmc3State {
    pub fn prg_mode(self) -> bool {
        self.bank_select & 0x40 != 0
    }

    pub fn chr_invert(self) -> bool {
        self.bank_select & 0x80 != 0
    }

    pub fn selected_register(self) -> usize {
        (self.bank_select & 0x07) as usize
    }

    /// Apply one MMC3 register write. Addresses below `$8000` are not mapper
    /// writes and leave the state unchanged.
    pub fn apply_write(&mut self, addr: u16, value: u8) {
        match addr {
            0x8000..=0x9FFF if addr & 1 == 0 => self.bank_select = value,
            0x8000..=0x9FFF => {
                let reg = self.selected_register();
                self.regs[reg] = value;
            }
            0xA000..=0xBFFF if addr & 1 == 0 => {
                self.horizontal_mirroring = value & 0x01 != 0;
            }
            0xA000..=0xBFFF => self.prg_ram_protect = value,
            0xC000..=0xDFFF if addr & 1 == 0 => self.irq_latch = value,
            0xC000..=0xDFFF => self.irq_reload_pending = true,
            0xE000..=0xFFFF if addr & 1 == 0 => {
                self.irq_enabled = false;
                self.irq_reload_pending = false;
                self.irq_pending = false;
            }
            0xE000..=0xFFFF => self.irq_enabled = true,
            _ => {}
        }
    }

    /// 1 KiB CHR bank selected for pattern-table 1 KiB `slot` (0-7) through
    /// the R0-R5 windows and the CHR-invert bit. R0/R1 are 2 KiB pairs
    /// (even bank plus the slot offset within the pair); R2-R5 are single
    /// 1 KiB banks. The caller masks the result into the available CHR
    /// banks. Shared by the reference bus and the future SMS runtime so
    /// window resolution never drifts between them.
    pub fn chr_bank_1k(self, slot_1k: u8) -> u8 {
        let regs = self.regs;
        let slot = slot_1k & 7;
        match (self.chr_invert(), slot) {
            (false, 0) | (false, 1) => (regs[0] & !1) | (slot & 1),
            (false, 2) | (false, 3) => (regs[1] & !1) | (slot & 1),
            (false, 4) => regs[2],
            (false, 5) => regs[3],
            (false, 6) => regs[4],
            (false, 7) => regs[5],
            (true, 0) => regs[2],
            (true, 1) => regs[3],
            (true, 2) => regs[4],
            (true, 3) => regs[5],
            (true, 4) | (true, 5) => (regs[0] & !1) | ((slot - 4) & 1),
            _ => (regs[1] & !1) | ((slot - 6) & 1),
        }
    }

    /// One PPU-A12 clock of the scanline IRQ counter (MMC3B/C). When the
    /// reload flag is set — or the counter already reads zero — the latch
    /// is reloaded instead of decrementing. The mapper then asserts IRQ
    /// whenever the counter reads zero and IRQs are enabled; the level
    /// holds until an `$E000` ack. Callers decide *when* A12 rises (only
    /// during rendering fetches into `$1000-$1FFF` on hardware); this
    /// method only advances the counter deterministically.
    pub fn clock_a12(&mut self) {
        if self.irq_reload_pending || self.irq_counter == 0 {
            self.irq_counter = self.irq_latch;
            self.irq_reload_pending = false;
        } else {
            self.irq_counter = self.irq_counter.wrapping_sub(1);
        }
        if self.irq_counter == 0 && self.irq_enabled {
            self.irq_pending = true;
        }
    }

    /// 8 KiB PRG bank index visible at `cpu_addr` for a ROM with
    /// `prg_8k_count` banks. Returns `None` outside `$8000-$FFFF`.
    pub fn prg_bank_at(self, cpu_addr: u16, prg_8k_count: u8) -> Option<u8> {
        if cpu_addr < 0x8000 || prg_8k_count == 0 {
            return None;
        }
        let last = prg_8k_count - 1;
        let second_last = prg_8k_count.wrapping_sub(2);
        let r6 = self.regs[6] % prg_8k_count;
        let r7 = self.regs[7] % prg_8k_count;
        let bank = match cpu_addr {
            0x8000..=0x9FFF if !self.prg_mode() => r6,
            0x8000..=0x9FFF => second_last,
            0xA000..=0xBFFF => r7,
            0xC000..=0xDFFF if !self.prg_mode() => second_last,
            0xC000..=0xDFFF => r6,
            _ => last,
        };
        Some(bank)
    }

    /// CPU address to PRG byte offset using live window state.
    pub fn cpu_to_prg_offset(
        self,
        prg_len: usize,
        prg_8k_count: u8,
        cpu_addr: u16,
    ) -> Option<usize> {
        let bank = self.prg_bank_at(cpu_addr, prg_8k_count)? as usize;
        let window_base = match cpu_addr {
            0x8000..=0x9FFF => 0x8000,
            0xA000..=0xBFFF => 0xA000,
            0xC000..=0xDFFF => 0xC000,
            0xE000..=0xFFFF => 0xE000,
            _ => return None,
        };
        let offset = bank * MMC3_PRG_WINDOW_SIZE + (cpu_addr as usize - window_base);
        (offset < prg_len).then_some(offset)
    }
}

/// MMC3 PRG address window. The live bank in `Low`/`High`/`Mid` depends on
/// the R6/R7 registers and the PRG-mode bit (`Mmc3State::prg_bank_at`);
/// only `Top` is unconditionally the last bank. Discovery and the future
/// bank-constant propagation pass share this classifier so window identity
/// never drifts between crates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mmc3Window {
    /// `$8000-$9FFF`: R6 in PRG mode 0, second-last bank in PRG mode 1.
    Low,
    /// `$A000-$BFFF`: always R7.
    High,
    /// `$C000-$DFFF`: second-last bank in PRG mode 0, R6 in PRG mode 1.
    Mid,
    /// `$E000-$FFFF`: always the last bank (vectors live here).
    Top,
}

impl Mmc3Window {
    /// Window containing `cpu_addr`, or `None` outside `$8000-$FFFF`.
    pub fn for_addr(cpu_addr: u16) -> Option<Self> {
        match cpu_addr {
            0x8000..=0x9FFF => Some(Self::Low),
            0xA000..=0xBFFF => Some(Self::High),
            0xC000..=0xDFFF => Some(Self::Mid),
            0xE000..=0xFFFF => Some(Self::Top),
            _ => None,
        }
    }

    /// Inclusive CPU address range of this window.
    pub fn range(self) -> (u16, u16) {
        match self {
            Self::Low => (0x8000, 0x9FFF),
            Self::High => (0xA000, 0xBFFF),
            Self::Mid => (0xC000, 0xDFFF),
            Self::Top => (0xE000, 0xFFFF),
        }
    }

    /// True only for `Top`: the one window whose bank never depends on
    /// mapper state. All other windows need live R6/R7 + PRG-mode facts.
    pub fn always_fixed_last_bank(self) -> bool {
        matches!(self, Self::Top)
    }
}

/// Build the NROM-shaped 32 KiB analysis view for one MMC3 window pair:
/// `[low_bank | high_bank | fixed last 16 KiB]`, so `$8000-$9FFF` reads the
/// 8 KiB `low_bank`, `$A000-$BFFF` reads `high_bank`, and `$C000-$FFFF`
/// reads the fixed top. The existing `analysis` walker (which indexes
/// `addr - $8000`) runs on this view unchanged, constrained to
/// `AnalysisWindow::SWITCHABLE_8K_LOW/HIGH` per entry window plus a fixed
/// pass over the top. Fails closed on bank or length mismatches.
pub fn mmc3_analysis_view(
    prg: &[u8],
    prg_8k_count: u8,
    low_bank: u8,
    high_bank: u8,
) -> Result<Vec<u8>, MapperPolicyError> {
    let prg_8k_count_usize = prg_8k_count as usize;
    if prg_8k_count_usize < MMC3_MIN_8K_BANKS
        || prg_8k_count_usize > MMC3_MAX_8K_BANKS
        || prg.len() != prg_8k_count_usize * MMC3_PRG_WINDOW_SIZE
    {
        return Err(MapperPolicyError::InvalidMmc3PrgLayout { prg_len: prg.len() });
    }
    let slice8 = |bank: u8| -> Result<&[u8], MapperPolicyError> {
        if bank >= prg_8k_count {
            return Err(MapperPolicyError::SelectedMmc3BankOutOfRange {
                bank,
                bank_count: prg_8k_count,
            });
        }
        let off = bank as usize * MMC3_PRG_WINDOW_SIZE;
        Ok(&prg[off..off + MMC3_PRG_WINDOW_SIZE])
    };
    let mut view = Vec::with_capacity(4 * MMC3_PRG_WINDOW_SIZE);
    view.extend_from_slice(slice8(low_bank)?);
    view.extend_from_slice(slice8(high_bank)?);
    view.extend_from_slice(&prg[prg.len() - 2 * MMC3_PRG_WINDOW_SIZE..]);
    Ok(view)
}

/// A statically observed MMC3 bank-select idiom: `LDA #cfg / STA $8000 /
/// LDA #bank / STA $8001 / JSR target` with the five instructions
/// contiguous. `window_bank` is the 8 KiB bank the JSR target would see
/// under the pair's PRG-mode bit (R6 for `$8000-$9FFF` in mode 0, R7 for
/// `$A000-$BFFF`; mode-1 `$8000` shows the second-last bank instead).
/// CANDIDATES ONLY: linear matching cannot see joins, so every candidate
/// must be confirmed against the reference harvest (`FD_LOG_BANK_ENTRIES`
/// `MMC3_ENTRY` lines) before becoming a profile `[[bank_entry]]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mmc3BankCandidate {
    /// Offset of the `LDA #cfg` in the scanned view.
    pub offset: usize,
    /// CPU address of the `JSR` target.
    pub target: u16,
    /// 8 KiB bank the target would see (or second-last bank for a mode-1
    /// `$8000-$9FFF` target).
    pub window_bank: u8,
    /// True when the pair's PRG-mode bit selected the mode-1 mapping.
    pub prg_mode: bool,
}

/// Scan `fixed_view` (the 32 KiB mode-0 `[fixed16 | fixed16]` discovery
/// view) for contiguous bank-select idioms whose JSR lands in a switchable
/// window. `prg_8k_count` bounds the reported banks. Returns candidates in
/// scan order.
pub fn harvest_mmc3_bank_candidates(fixed_view: &[u8], prg_8k_count: u8) -> Vec<Mmc3BankCandidate> {
    let mut out = Vec::new();
    if prg_8k_count == 0 || fixed_view.len() < 13 {
        return out;
    }
    // Idiom (13 bytes): A9 cfg / 8D 00 80 / A9 bank / 8D 01 80 / 20 lo hi.
    for (i, window) in fixed_view.windows(13).enumerate() {
        if window[0] != 0xA9
            || window[2] != 0x8D
            || window[3] != 0x00
            || window[4] != 0x80
            || window[5] != 0xA9
            || window[7] != 0x8D
            || window[8] != 0x01
            || window[9] != 0x80
            || window[10] != 0x20
        {
            continue;
        }
        let select = window[1];
        let bank = window[6];
        let target = u16::from_le_bytes([window[11], window[12]]);
        if !(0x8000..0xC000).contains(&target) {
            continue;
        }
        let prg_mode = select & 0x40 != 0;
        let r6 = bank % prg_8k_count;
        let r7 = bank % prg_8k_count;
        let window_bank = match target {
            0x8000..=0x9FFF if !prg_mode => r6,
            0x8000..=0x9FFF => prg_8k_count.wrapping_sub(2),
            _ => r7,
        };
        out.push(Mmc3BankCandidate {
            offset: i,
            target,
            window_bank,
            prg_mode,
        });
    }
    out
}

/// Read vectors using an explicit supported mapper policy.
pub fn read_vectors_with_policy(
    policy: MapperPolicy,
    prg: &[u8],
) -> Result<Option<Vectors>, MapperPolicyError> {
    let read_word = |cpu_addr| -> Result<Option<u16>, MapperPolicyError> {
        let Some(offset) = policy.cpu_to_prg_offset(cpu_addr, 0)? else {
            return Ok(None);
        };
        Ok(prg
            .get(offset..offset + 2)
            .and_then(|bytes| bytes.try_into().ok())
            .map(u16::from_le_bytes))
    };
    Ok(Some(Vectors {
        nmi: match read_word(0xfffa)? {
            Some(value) => value,
            None => return Ok(None),
        },
        reset: match read_word(0xfffc)? {
            Some(value) => value,
            None => return Ok(None),
        },
        irq: match read_word(0xfffe)? {
            Some(value) => value,
            None => return Ok(None),
        },
    }))
}

/// Read a 16-bit little-endian value from PRG at the given CPU address.
/// Assumes NROM-256 layout (PRG mapped at $8000..=$FFFF).
pub fn read_word_at_cpu_addr(prg: &[u8], cpu_addr: u16) -> Option<u16> {
    let off = cpu_to_prg_offset(prg.len(), cpu_addr)?;
    let lo = *prg.get(off)?;
    let hi = *prg.get(off + 1)?;
    Some(u16::from_le_bytes([lo, hi]))
}

/// CPU address → PRG byte offset, NROM only. Mirrors $C000..=$FFFF onto $8000..=$BFFF
/// when PRG is 16 KB (NROM-128).
/// CPU address → PRG byte offset for the FIXED region of the layout.
/// NROM: whole window (16 KB mirrored or flat 32 KB). Banked mappers
/// (PRG > 32 KB; UxROM/MMC1-typical/MMC3 fix the top): $C000-$FFFF maps
/// to the LAST 16 KB of PRG; the switchable window returns None
/// (callers need a bank: `banked_prg_offset`).
pub fn cpu_to_prg_offset(prg_len: usize, cpu_addr: u16) -> Option<usize> {
    if cpu_addr < 0x8000 {
        return None;
    }
    let off = (cpu_addr - 0x8000) as usize;
    if prg_len == 16 * 1024 {
        Some(off & 0x3fff)
    } else if prg_len == 32 * 1024 {
        Some(off)
    } else if prg_len > 32 * 1024 && cpu_addr >= 0xC000 {
        Some(prg_len - 0x4000 + (cpu_addr as usize - 0xC000))
    } else {
        None
    }
}

/// CPU address in the switchable window ($8000-$BFFF for 16 KB-banked
/// mappers) → PRG offset given the selected bank.
pub fn banked_prg_offset(prg_len: usize, bank: u8, cpu_addr: u16) -> Option<usize> {
    if !(0x8000..0xC000).contains(&cpu_addr) {
        return None;
    }
    let off = bank as usize * 0x4000 + (cpu_addr as usize - 0x8000);
    (off < prg_len).then_some(off)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vectors {
    pub nmi: u16,
    pub reset: u16,
    pub irq: u16,
}

pub fn read_vectors(prg: &[u8]) -> Option<Vectors> {
    Some(Vectors {
        nmi: read_word_at_cpu_addr(prg, 0xfffa)?,
        reset: read_word_at_cpu_addr(prg, 0xfffc)?,
        irq: read_word_at_cpu_addr(prg, 0xfffe)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header(mapper: u16) -> Header {
        Header {
            kind: if mapper == 2 {
                HeaderKind::Nes2
            } else {
                HeaderKind::INes
            },
            prg_banks: 0,
            chr_banks: 0,
            prg_ram_size: 0,
            prg_nvram_size: 0,
            chr_ram_size: 0,
            chr_nvram_size: 0,
            mapper,
            submapper: if mapper == 2 { 2 } else { 0 },
            mirroring: Mirroring::Horizontal,
            has_trainer: false,
            has_battery: false,
        }
    }

    fn build_nrom256() -> Vec<u8> {
        let mut rom = vec![0u8; HEADER_SIZE + 32 * 1024 + 8 * 1024];
        rom[0..4].copy_from_slice(&INES_MAGIC);
        rom[4] = 2;
        rom[5] = 1;
        // mapper 0, vertical mirroring
        rom[6] = 0x01;
        // vectors at end of PRG: nmi $8082, reset $8000, irq $fff0
        let prg_end = HEADER_SIZE + 32 * 1024;
        rom[prg_end - 6..prg_end - 4].copy_from_slice(&0x8082u16.to_le_bytes());
        rom[prg_end - 4..prg_end - 2].copy_from_slice(&0x8000u16.to_le_bytes());
        rom[prg_end - 2..prg_end].copy_from_slice(&0xfff0u16.to_le_bytes());
        rom
    }

    #[test]
    fn parses_nrom256_header() {
        let rom = build_nrom256();
        let h = parse_header(&rom).unwrap();
        assert_eq!(h.prg_banks, 2);
        assert_eq!(h.chr_banks, 1);
        assert_eq!(h.prg_ram_size, 8 * 1024);
        assert_eq!(h.chr_ram_size, 0);
        assert_eq!(h.prg_nvram_size, 0);
        assert_eq!(h.chr_nvram_size, 0);
        assert_eq!(h.mapper, 0);
        assert_eq!(h.mirroring, Mirroring::Vertical);
        assert!(!h.has_trainer);
        assert!(!h.has_battery);
    }

    #[test]
    fn parses_image_and_vectors() {
        let rom = build_nrom256();
        let img = parse(&rom).unwrap();
        assert_eq!(img.prg.len(), 32 * 1024);
        assert_eq!(img.chr.len(), 8 * 1024);
        let v = read_vectors(img.prg).unwrap();
        assert_eq!(v.nmi, 0x8082);
        assert_eq!(v.reset, 0x8000);
        assert_eq!(v.irq, 0xfff0);
    }

    #[test]
    fn rejects_bad_magic() {
        let mut rom = vec![0u8; 64];
        assert_eq!(parse_header(&rom), Err(ParseError::BadMagic));
        rom[0..4].copy_from_slice(&INES_MAGIC);
        // header parses but body is too short for the declared sizes:
        rom[4] = 2;
        rom[5] = 1;
        assert!(matches!(parse(&rom), Err(ParseError::Truncated { .. })));
    }

    #[test]
    fn nrom128_mirrors_high_bank() {
        assert_eq!(cpu_to_prg_offset(16 * 1024, 0x8000), Some(0));
        assert_eq!(cpu_to_prg_offset(16 * 1024, 0xc000), Some(0));
        assert_eq!(cpu_to_prg_offset(16 * 1024, 0xffff), Some(0x3fff));
        assert_eq!(cpu_to_prg_offset(32 * 1024, 0xffff), Some(0x7fff));
        assert_eq!(cpu_to_prg_offset(32 * 1024, 0x7fff), None);
    }

    #[test]
    fn nrom_policy_maps_boundaries() {
        let nrom128 = resolve_mapper_policy(&header(0), PRG_BANK_SIZE).unwrap();
        assert_eq!(nrom128.cpu_to_prg_offset(0x7fff, 0).unwrap(), None);
        assert_eq!(nrom128.cpu_to_prg_offset(0x8000, 0).unwrap(), Some(0));
        assert_eq!(
            nrom128.cpu_to_prg_offset(0xbfff, 0).unwrap(),
            Some(PRG_BANK_SIZE - 1)
        );
        assert_eq!(nrom128.cpu_to_prg_offset(0xc000, 0).unwrap(), Some(0));
        assert_eq!(
            nrom128.cpu_to_prg_offset(0xffff, 0).unwrap(),
            Some(PRG_BANK_SIZE - 1)
        );

        let nrom256 = resolve_mapper_policy(&header(0), 2 * PRG_BANK_SIZE).unwrap();
        assert_eq!(nrom256.cpu_to_prg_offset(0x8000, 0).unwrap(), Some(0));
        assert_eq!(
            nrom256.cpu_to_prg_offset(0xffff, 0).unwrap(),
            Some(2 * PRG_BANK_SIZE - 1)
        );
    }

    #[test]
    fn uxrom_policy_maps_all_cv1_banks_and_fixed_bank() {
        let policy = resolve_mapper_policy(&header(2), 8 * PRG_BANK_SIZE).unwrap();
        assert!(policy.is_banked());
        assert_eq!(policy.bank_count(), 8);
        for bank in 0..8 {
            assert_eq!(
                policy.cpu_to_prg_offset(0x8000, bank).unwrap(),
                Some(bank as usize * PRG_BANK_SIZE)
            );
            assert_eq!(
                policy.cpu_to_prg_offset(0xbfff, bank).unwrap(),
                Some((bank as usize + 1) * PRG_BANK_SIZE - 1)
            );
            assert_eq!(
                policy.cpu_to_prg_offset(0xc000, bank).unwrap(),
                Some(7 * PRG_BANK_SIZE)
            );
            assert_eq!(
                policy.cpu_to_prg_offset(0xffff, bank).unwrap(),
                Some(8 * PRG_BANK_SIZE - 1)
            );
        }
    }

    #[test]
    fn uxrom_policy_rejects_invalid_selected_bank() {
        let policy = resolve_mapper_policy(&header(2), 8 * PRG_BANK_SIZE).unwrap();
        assert_eq!(
            policy.cpu_to_prg_offset(0x8000, 8),
            Err(MapperPolicyError::SelectedBankOutOfRange {
                bank: 8,
                bank_count: 8,
            })
        );
        assert_eq!(
            policy.cpu_to_prg_offset(0xc000, 9),
            Err(MapperPolicyError::SelectedBankOutOfRange {
                bank: 9,
                bank_count: 8,
            })
        );
    }

    #[test]
    fn uxrom_submappers_select_bus_conflict_mode() {
        let mut no_conflict = header(2);
        no_conflict.submapper = 1;
        let no_conflict = resolve_mapper_policy(&no_conflict, 8 * PRG_BANK_SIZE).unwrap();
        assert_eq!(
            no_conflict.uxrom_bus_conflicts(),
            Some(UxromBusConflicts::None)
        );
        assert_eq!(no_conflict.selected_bank_from_write(3, 0), Ok(3));
        assert_eq!(no_conflict.selected_bank_from_write(0xf9, 0), Ok(1));

        let conflict = resolve_mapper_policy(&header(2), 8 * PRG_BANK_SIZE).unwrap();
        assert_eq!(conflict.uxrom_bus_conflicts(), Some(UxromBusConflicts::And));
        assert_eq!(conflict.selected_bank_from_write(7, 3), Ok(3));
        assert_eq!(conflict.selected_bank_from_write(0xff, 0xfa), Ok(2));
    }

    #[test]
    fn uxrom_rejects_legacy_and_unknown_submappers() {
        let mut legacy = header(2);
        legacy.kind = HeaderKind::INes;
        assert_eq!(
            resolve_mapper_policy(&legacy, 8 * PRG_BANK_SIZE),
            Err(MapperPolicyError::UxromRequiresNes2)
        );
        for submapper in [0, 3] {
            let mut unknown = header(2);
            unknown.submapper = submapper;
            assert_eq!(
                resolve_mapper_policy(&unknown, 8 * PRG_BANK_SIZE),
                Err(MapperPolicyError::UnsupportedUxromSubmapper { submapper })
            );
        }
    }

    #[test]
    fn mapper_policy_rejects_invalid_layouts_and_unsupported_mapper() {
        assert_eq!(
            resolve_mapper_policy(&header(0), 3 * PRG_BANK_SIZE),
            Err(MapperPolicyError::InvalidNromPrgLayout {
                prg_len: 3 * PRG_BANK_SIZE,
            })
        );
        assert_eq!(
            resolve_mapper_policy(&header(2), PRG_BANK_SIZE),
            Err(MapperPolicyError::InvalidUxromPrgLayout {
                prg_len: PRG_BANK_SIZE,
            })
        );
        assert_eq!(
            resolve_mapper_policy(&header(2), 3 * PRG_BANK_SIZE),
            Err(MapperPolicyError::InvalidUxromPrgLayout {
                prg_len: 3 * PRG_BANK_SIZE,
            })
        );
        assert_eq!(
            resolve_mapper_policy(&header(2), 17 * PRG_BANK_SIZE),
            Err(MapperPolicyError::InvalidUxromPrgLayout {
                prg_len: 17 * PRG_BANK_SIZE,
            })
        );
        assert_eq!(
            resolve_mapper_policy(&header(1), 2 * PRG_BANK_SIZE),
            Err(MapperPolicyError::UnsupportedMapper { mapper: 1 })
        );
    }

    #[test]
    fn mmc3_policy_accepts_earthbound_like_256k_and_maps_fixed_top() {
        let header4 = Header {
            kind: HeaderKind::INes,
            prg_banks: 0,
            chr_banks: 0,
            prg_ram_size: 0,
            prg_nvram_size: 0,
            chr_ram_size: 0,
            chr_nvram_size: 0,
            mapper: 4,
            submapper: 0,
            mirroring: Mirroring::Horizontal,
            has_trainer: false,
            has_battery: true,
        };
        let prg_len = 256 * 1024;
        let policy = resolve_mapper_policy(&header4, prg_len).unwrap();
        assert!(policy.is_banked());
        assert_eq!(policy.mmc3_8k_bank_count(), Some(32));
        assert_eq!(policy.bank_count(), 32);
        // Fixed top maps to the last 16 KiB regardless of window state.
        assert_eq!(
            policy.cpu_to_prg_offset(0xC000, 0).unwrap(),
            Some(prg_len - 0x4000)
        );
        assert_eq!(
            policy.cpu_to_prg_offset(0xFFFF, 5).unwrap(),
            Some(prg_len - 1)
        );
        // Switchable windows need live R6/R7 + PRG-mode state.
        assert_eq!(policy.cpu_to_prg_offset(0x8000, 0).unwrap(), None);
        assert_eq!(policy.cpu_to_prg_offset(0xA000, 3).unwrap(), None);
        // Bare-index helpers stay 8 KiB-granular and fail closed.
        assert_eq!(
            policy.prg_bank(&vec![0xAA; prg_len], 31).unwrap().len(),
            8192
        );
        assert_eq!(
            policy.cpu_to_prg_offset(0xC000, 32),
            Err(MapperPolicyError::SelectedMmc3BankOutOfRange {
                bank: 32,
                bank_count: 32,
            })
        );
        assert_eq!(
            policy.analysis_view(&vec![0u8; prg_len], 0),
            Err(MapperPolicyError::Mmc3WindowStateRequired)
        );
        assert_eq!(
            policy.selected_bank_from_write(6, 0),
            Err(MapperPolicyError::Mmc3WindowStateRequired)
        );
    }

    #[test]
    fn mmc3_policy_rejects_bad_layouts() {
        let header4 = Header {
            kind: HeaderKind::INes,
            prg_banks: 0,
            chr_banks: 0,
            prg_ram_size: 0,
            prg_nvram_size: 0,
            chr_ram_size: 0,
            chr_nvram_size: 0,
            mapper: 4,
            submapper: 0,
            mirroring: Mirroring::Horizontal,
            has_trainer: false,
            has_battery: false,
        };
        for prg_len in [32 * 1024, 48 * 1024, 3 * PRG_BANK_SIZE, 65 * 8192] {
            assert_eq!(
                resolve_mapper_policy(&header4, prg_len),
                Err(MapperPolicyError::InvalidMmc3PrgLayout { prg_len })
            );
        }
    }

    #[test]
    fn mmc3_state_tracks_windows_modes_and_irq() {
        let mut state = Mmc3State::default();
        // Select R6 and map 8 KiB bank 11 at $8000 (PRG mode 0).
        state.apply_write(0x8000, 6);
        state.apply_write(0x8001, 11);
        state.apply_write(0xA000 + 1, 7);
        assert_eq!(state.prg_bank_at(0x8000, 32), Some(11));
        assert_eq!(state.prg_bank_at(0xA000, 32), Some(1));
        assert_eq!(state.prg_bank_at(0xC000, 32), Some(30));
        assert_eq!(state.prg_bank_at(0xE000, 32), Some(31));
        // PRG mode 1 swaps the $8000 and $C000 windows.
        state.apply_write(0x8000, 0x46);
        state.apply_write(0x8001, 5);
        assert!(state.prg_mode());
        assert_eq!(state.prg_bank_at(0x8000, 32), Some(30));
        assert_eq!(state.prg_bank_at(0xC000, 32), Some(5));
        // Bank numbers wrap into the available ROM.
        state.apply_write(0x8000, 6);
        state.apply_write(0x8001, 0xFF);
        assert_eq!(state.prg_bank_at(0x8000, 32), Some(0xFF % 32));
        // Live offsets stay inside the payload.
        let prg_len = 256 * 1024;
        let off = state
            .cpu_to_prg_offset(prg_len, 32, 0x8000)
            .expect("window offset");
        assert!(off < prg_len);
        assert_eq!(state.cpu_to_prg_offset(prg_len, 32, 0x7000), None);
        // Mirroring, protect, IRQ latch/enable/disable.
        state.apply_write(0xA000, 1);
        assert!(state.horizontal_mirroring);
        state.apply_write(0xC000, 0x2A);
        state.apply_write(0xC001, 0);
        assert!(state.irq_reload_pending);
        state.apply_write(0xE001, 0);
        assert!(state.irq_enabled);
        state.apply_write(0xE000, 0);
        assert!(!state.irq_enabled);
        assert!(!state.irq_reload_pending);
        // Non-mapper addresses are ignored.
        let before = state;
        state.apply_write(0x6000, 0xFF);
        assert_eq!(state, before);
    }

    #[test]
    fn mmc3_irq_counter_reloads_counts_and_holds_until_ack() {
        let mut state = Mmc3State::default();
        state.apply_write(0xC000, 3);
        state.apply_write(0xC001, 0); // reload pending
        state.apply_write(0xE001, 0); // enable
        // Latch 3 needs four A12 clocks: reload then 3, 2, 1, 0(fire).
        for expected in [3, 2, 1] {
            state.clock_a12();
            assert_eq!(state.irq_counter, expected);
            assert!(!state.irq_pending);
        }
        state.clock_a12();
        assert_eq!(state.irq_counter, 0);
        assert!(state.irq_pending);
        // Level holds across further clocks until the $E000 ack ...
        state.clock_a12();
        assert!(state.irq_pending);
        // ... and the ack clears pending (and disables) without touching
        // the counter value itself.
        state.apply_write(0xE000, 0);
        assert!(!state.irq_pending);
        assert!(!state.irq_enabled);
        assert_eq!(state.irq_counter, 3);
        // Disabled counters still advance but never assert.
        state.clock_a12();
        assert_eq!(state.irq_counter, 2);
        assert!(!state.irq_pending);
    }

    #[test]
    fn mmc3_window_classifier_matches_prg_modes() {
        assert_eq!(Mmc3Window::for_addr(0x7FFF), None);
        assert_eq!(Mmc3Window::for_addr(0x8000), Some(Mmc3Window::Low));
        assert_eq!(Mmc3Window::for_addr(0x9FFF), Some(Mmc3Window::Low));
        assert_eq!(Mmc3Window::for_addr(0xA000), Some(Mmc3Window::High));
        assert_eq!(Mmc3Window::for_addr(0xBFFF), Some(Mmc3Window::High));
        assert_eq!(Mmc3Window::for_addr(0xC000), Some(Mmc3Window::Mid));
        assert_eq!(Mmc3Window::for_addr(0xDFFF), Some(Mmc3Window::Mid));
        assert_eq!(Mmc3Window::for_addr(0xE000), Some(Mmc3Window::Top));
        assert_eq!(Mmc3Window::for_addr(0xFFFF), Some(Mmc3Window::Top));
        assert_eq!(Mmc3Window::Low.range(), (0x8000, 0x9FFF));
        assert_eq!(Mmc3Window::High.range(), (0xA000, 0xBFFF));
        assert_eq!(Mmc3Window::Mid.range(), (0xC000, 0xDFFF));
        assert_eq!(Mmc3Window::Top.range(), (0xE000, 0xFFFF));
        assert!(Mmc3Window::Top.always_fixed_last_bank());
        for window in [Mmc3Window::Low, Mmc3Window::High, Mmc3Window::Mid] {
            assert!(!window.always_fixed_last_bank());
        }
        // Windows agree with live bank resolution: Top is the last bank in
        // both PRG modes, and the vector page classifies as Top.
        let mut state = Mmc3State::default();
        assert_eq!(state.prg_bank_at(0xE000, 32), Some(31));
        state.apply_write(0x8000, 0x40);
        assert_eq!(state.prg_bank_at(0xE000, 32), Some(31));
        assert_eq!(Mmc3Window::for_addr(0xFFFC), Some(Mmc3Window::Top));
    }

    #[test]
    fn mmc3_chr_windows_resolve_1k_banks_with_invert() {
        // Power-on R = [0,2,4,5,6,7]: identity mapping without invert.
        let state = Mmc3State::default();
        for slot in 0..8 {
            assert_eq!(state.chr_bank_1k(slot), slot, "slot {slot}");
        }
        // Invert swaps the 4 KiB halves.
        let mut inv = Mmc3State::default();
        inv.apply_write(0x8000, 0x80);
        for (slot, bank) in [
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
            (4, 0),
            (5, 1),
            (6, 2),
            (7, 3),
        ] {
            assert_eq!(inv.chr_bank_1k(slot), bank, "inverted slot {slot}");
        }
        // Odd R0 (5) pairs down to the even bank: slots 0-1 -> 4-5.
        let mut odd = Mmc3State::default();
        odd.apply_write(0x8000, 0);
        odd.apply_write(0x8001, 5);
        assert_eq!(odd.chr_bank_1k(0), 4);
        assert_eq!(odd.chr_bank_1k(1), 5);
        // Slot indices wrap into 0-7.
        assert_eq!(state.chr_bank_1k(8), state.chr_bank_1k(0));
    }

    #[test]
    fn mmc3_analysis_view_layouts_windows_nrom_shaped() {
        // 8x8 KiB PRG where bank i is filled with byte i.
        let mut prg = vec![0u8; 8 * MMC3_PRG_WINDOW_SIZE];
        for (i, chunk) in prg.chunks_mut(MMC3_PRG_WINDOW_SIZE).enumerate() {
            chunk.fill(i as u8);
        }
        let view = mmc3_analysis_view(&prg, 8, 2, 5).unwrap();
        assert_eq!(view.len(), 4 * MMC3_PRG_WINDOW_SIZE);
        // $8000-$9FFF -> bank 2, $A000-$BFFF -> bank 5, $C000-$FFFF fixed.
        assert!(view[0..MMC3_PRG_WINDOW_SIZE].iter().all(|&b| b == 2));
        assert!(
            view[MMC3_PRG_WINDOW_SIZE..2 * MMC3_PRG_WINDOW_SIZE]
                .iter()
                .all(|&b| b == 5)
        );
        assert!(
            view[2 * MMC3_PRG_WINDOW_SIZE..3 * MMC3_PRG_WINDOW_SIZE]
                .iter()
                .all(|&b| b == 6)
        );
        assert!(view[3 * MMC3_PRG_WINDOW_SIZE..].iter().all(|&b| b == 7));
        // Fail closed: window bank out of range, bad count, bad length.
        assert_eq!(
            mmc3_analysis_view(&prg, 8, 8, 0),
            Err(MapperPolicyError::SelectedMmc3BankOutOfRange {
                bank: 8,
                bank_count: 8,
            })
        );
        assert_eq!(
            mmc3_analysis_view(&prg, 8, 0, 9),
            Err(MapperPolicyError::SelectedMmc3BankOutOfRange {
                bank: 9,
                bank_count: 8,
            })
        );
        assert!(matches!(
            mmc3_analysis_view(&prg[..prg.len() - 1], 8, 0, 0),
            Err(MapperPolicyError::InvalidMmc3PrgLayout { .. })
        ));
        assert!(matches!(
            mmc3_analysis_view(&prg, 7, 0, 0),
            Err(MapperPolicyError::InvalidMmc3PrgLayout { .. })
        ));
    }

    #[test]
    fn mmc3_bank_candidate_harvest_needs_contiguous_idiom() {
        // Fixed view with a mode-0 R6=5 + JSR $8123 idiom at offset 0x100,
        // a mode-1 R6 pair + JSR $9000 (second-last bank), and a decoy with
        // the JSR landing in fixed space (ignored).
        let mut view = vec![0xEAu8; 0x8000];
        view[0x100..0x10D].copy_from_slice(&[
            0xA9, 0x06, 0x8D, 0x00, 0x80, 0xA9, 0x05, 0x8D, 0x01, 0x80, 0x20, 0x23, 0x81,
        ]);
        view[0x200..0x20D].copy_from_slice(&[
            0xA9, 0x46, 0x8D, 0x00, 0x80, 0xA9, 0x03, 0x8D, 0x01, 0x80, 0x20, 0x00, 0x90,
        ]);
        view[0x300..0x30D].copy_from_slice(&[
            0xA9, 0x06, 0x8D, 0x00, 0x80, 0xA9, 0x02, 0x8D, 0x01, 0x80, 0x20, 0x00, 0xC0,
        ]);
        let found = harvest_mmc3_bank_candidates(&view, 32);
        assert_eq!(
            found,
            vec![
                Mmc3BankCandidate {
                    offset: 0x100,
                    target: 0x8123,
                    window_bank: 5,
                    prg_mode: false,
                },
                Mmc3BankCandidate {
                    offset: 0x200,
                    target: 0x9000,
                    window_bank: 30,
                    prg_mode: true,
                },
            ]
        );
        // A broken idiom (STA $8000 replaced by STA $8002) yields nothing.
        let mut broken = vec![0xEAu8; 0x8000];
        broken[0..13].copy_from_slice(&[
            0xA9, 0x06, 0x8D, 0x02, 0x80, 0xA9, 0x05, 0x8D, 0x01, 0x80, 0x20, 0x23, 0x81,
        ]);
        assert!(harvest_mmc3_bank_candidates(&broken, 32).is_empty());
        assert!(harvest_mmc3_bank_candidates(&[], 32).is_empty());
    }

    #[test]
    fn hashes_canonical_prg_then_chr_payload() {
        assert_eq!(
            payload_sha256_hex(b"ab", b"c"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn parses_nes2_cv1_chr_ram_header() {
        let mut rom = vec![0u8; HEADER_SIZE];
        rom[0..4].copy_from_slice(&INES_MAGIC);
        rom[4] = 8;
        rom[6] = 0x21;
        rom[7] = 0x08;
        rom[8] = 0x20;
        rom[11] = 0x07;

        let h = parse_header(&rom).unwrap();
        assert_eq!(h.kind, HeaderKind::Nes2);
        assert_eq!(h.prg_banks, 8);
        assert_eq!(h.mapper, 2);
        assert_eq!(h.submapper, 2);
        assert_eq!(h.mirroring, Mirroring::Vertical);
        assert_eq!(h.chr_len(), 0);
        assert_eq!(h.prg_ram_size, 0);
        assert_eq!(h.prg_nvram_size, 0);
        assert_eq!(h.chr_ram_size, 8 * 1024);
        assert_eq!(h.chr_nvram_size, 0);
    }

    #[test]
    fn parses_nes2_volatile_and_nonvolatile_ram_sizes() {
        let mut rom = vec![0u8; HEADER_SIZE];
        rom[0..4].copy_from_slice(&INES_MAGIC);
        rom[7] = 0x08;
        rom[10] = 0x32;
        rom[11] = 0x54;

        let h = parse_header(&rom).unwrap();
        assert_eq!(h.prg_ram_size, 64 << 2);
        assert_eq!(h.prg_nvram_size, 64 << 3);
        assert_eq!(h.chr_ram_size, 64 << 4);
        assert_eq!(h.chr_nvram_size, 64 << 5);
    }

    #[test]
    fn ines_uses_chr_ram_fallback_without_chr_rom() {
        let mut rom = vec![0u8; HEADER_SIZE];
        rom[0..4].copy_from_slice(&INES_MAGIC);
        rom[4] = 1;
        rom[8] = 2;

        let h = parse_header(&rom).unwrap();
        assert_eq!(h.prg_ram_size, 16 * 1024);
        assert_eq!(h.chr_len(), 0);
        assert_eq!(h.chr_ram_size, 8 * 1024);
        assert_eq!(h.prg_nvram_size, 0);
        assert_eq!(h.chr_nvram_size, 0);
    }

    #[test]
    fn ines_nonzero_chr_rom_does_not_imply_chr_ram() {
        let mut rom = vec![0u8; HEADER_SIZE];
        rom[0..4].copy_from_slice(&INES_MAGIC);
        rom[5] = 1;

        let h = parse_header(&rom).unwrap();
        assert_eq!(h.chr_len(), CHR_BANK_SIZE);
        assert_eq!(h.chr_ram_size, 0);
    }
}
