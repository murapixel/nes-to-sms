//! End-to-end pipeline: NES ROM → SMS project.

use std::fmt;
use std::path::Path;

use analysis::nes_rom_like;
use lower::LowerOptions;
use sms_project::{NesMirroring, ProjectAssets, ProjectConfig, RawCiramBackend};

use crate::Args;

#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    Rom(nes_rom::ParseError),
    MapperPolicy(nes_rom::MapperPolicyError),
    Profile(profile::LoadError),
    Lift(ir::LiftError),
    Lower(lower::LowerError),
    Emit(z80_emit::EmitError),
    Project(sms_project::EmitError),
    Diagnostic(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "i/o: {e}"),
            Error::Rom(e) => write!(f, "rom: {e}"),
            Error::MapperPolicy(e) => write!(f, "mapper policy: {e}"),
            Error::Profile(e) => write!(f, "profile: {e}"),
            Error::Lift(e) => write!(f, "lift: {e:?}"),
            Error::Lower(e) => write!(f, "lower: {e}"),
            Error::Emit(e) => write!(f, "emit: {e:?}"),
            Error::Project(e) => write!(f, "project: {e}"),
            Error::Diagnostic(s) => write!(f, "{s}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}
impl From<nes_rom::ParseError> for Error {
    fn from(e: nes_rom::ParseError) -> Self {
        Error::Rom(e)
    }
}
impl From<nes_rom::MapperPolicyError> for Error {
    fn from(e: nes_rom::MapperPolicyError) -> Self {
        Error::MapperPolicy(e)
    }
}
impl From<profile::LoadError> for Error {
    fn from(e: profile::LoadError) -> Self {
        Error::Profile(e)
    }
}
impl From<ir::LiftError> for Error {
    fn from(e: ir::LiftError) -> Self {
        Error::Lift(e)
    }
}
impl From<lower::LowerError> for Error {
    fn from(e: lower::LowerError) -> Self {
        Error::Lower(e)
    }
}
impl From<z80_emit::EmitError> for Error {
    fn from(e: z80_emit::EmitError) -> Self {
        Error::Emit(e)
    }
}
impl From<sms_project::EmitError> for Error {
    fn from(e: sms_project::EmitError) -> Self {
        Error::Project(e)
    }
}

/// Mapper-store violations cannot safely degrade to generated stubs: doing so
/// could turn an unsupported store into a wrong bank switch. Other lowering
/// errors remain diagnostics while the converter's broader coverage grows.
/// Raw-CIRAM shadow backend selection. Cartridge SRAM (iNES battery bit)
/// owns slot-2 SMS EXRAM as the $6000-$7FFF WRAM mirror, which fully
/// overlaps the 2 KiB EXRAM raw-CIRAM shadow ($8000-$87FF); keeping the
/// shadow there would corrupt WRAM code/data on every nametable write
/// (frame-1213 Mother $0074/$0075 divergence via clobbered $60B3/$60B4),
/// so SRAM carts run without the EXRAM shadow (projection repaints).
fn raw_ciram_backend_for(has_battery: bool) -> RawCiramBackend {
    if has_battery {
        RawCiramBackend::None
    } else {
        RawCiramBackend::SramSlot2
    }
}

fn lower_error_is_fatal(error: &lower::LowerError) -> bool {
    matches!(error, lower::LowerError::UnsupportedMapperStore { .. })
}

/// One WLA-DX slot is a physical 16 KiB ROM bank.
const TRANSLATED_SECTION_CAPACITY: usize = 0x4000;
const TRANSLATED_BANK_BASE: u32 = 4;
const TRANSLATED_SLOT: u8 = 1;

fn translated_section_bank(section_idx: u32, banked: bool) -> Result<u8, Error> {
    let bank = TRANSLATED_BANK_BASE
        .checked_add(section_idx)
        .ok_or_else(|| {
            Error::Diagnostic("translated section index overflows WLA bank numbering".to_string())
        })?;
    if banked && bank >= sms_project::NES_PRG_BANK_BASE {
        let max_section = sms_project::NES_PRG_BANK_BASE - TRANSLATED_BANK_BASE - 1;
        return Err(Error::Diagnostic(format!(
            "translated code overflows the banked 512K layout \
             (section {section_idx} > {max_section}; banks \
             {}+ hold PRG data)",
            sms_project::NES_PRG_BANK_BASE
        )));
    }
    u8::try_from(bank).map_err(|_| {
        Error::Diagnostic(format!(
            "translated code bank {bank} exceeds WLA's bank range"
        ))
    })
}

fn begin_translated_section(
    program: &mut z80_emit::Program,
    section_idx: u32,
    banked: bool,
) -> Result<u16, Error> {
    let bank = translated_section_bank(section_idx, banked)?;
    program.section(&format!("generated_code_{section_idx}"));
    let expected_program_idx = usize::try_from(section_idx)
        .ok()
        .and_then(|idx| idx.checked_add(1))
        .ok_or_else(|| {
            Error::Diagnostic(format!(
                "translated logical section {section_idx} exceeds Program section indexing"
            ))
        })?;
    let actual_program_idx = program.current_section_idx();
    if actual_program_idx != expected_program_idx {
        return Err(Error::Diagnostic(format!(
            "translated logical section {section_idx} mapped to Program section {actual_program_idx}, expected {expected_program_idx}; helper sections must not precede generated code"
        )));
    }
    program.set_section_placement(bank, TRANSLATED_SLOT);
    program.org(0x4000);
    Ok(program.current_addr())
}

fn advance_translated_section(
    program: &mut z80_emit::Program,
    section_idx: &mut u32,
    banked: bool,
) -> Result<u16, Error> {
    *section_idx = section_idx.checked_add(1).ok_or_else(|| {
        Error::Diagnostic("translated section index overflows WLA bank numbering".to_string())
    })?;
    begin_translated_section(program, *section_idx, banked)
}

fn translated_section_usage(program: &z80_emit::Program) -> usize {
    program.current_section_len()
}

/// Transactionally pack one complete routine during the sizing pass. The
/// closure owns all routine-local label/diagnostic mutation through `state`.
fn pack_sizing_candidate<S: Clone>(
    program: &mut z80_emit::Program,
    state: &mut S,
    section_idx: &mut u32,
    banked: bool,
    routine_name: &str,
    emit: impl Fn(&mut z80_emit::Program, &mut S) -> Result<(), Error>,
) -> Result<u32, Error> {
    let mut candidate = program.clone();
    let mut candidate_state = state.clone();
    emit(&mut candidate, &mut candidate_state)?;
    if translated_section_usage(&candidate) > TRANSLATED_SECTION_CAPACITY {
        advance_translated_section(program, section_idx, banked)?;
        candidate = program.clone();
        candidate_state = state.clone();
        emit(&mut candidate, &mut candidate_state)?;
        let used = translated_section_usage(&candidate);
        if used > TRANSLATED_SECTION_CAPACITY {
            return Err(Error::Diagnostic(format!(
                "translated routine {routine_name} exceeds physical 16 KiB slot: {used} bytes"
            )));
        }
    }
    *program = candidate;
    *state = candidate_state;
    Ok(*section_idx)
}

fn routine_auto_label(r: &ir::Routine) -> String {
    if r.name.starts_with("L_b") {
        r.name.clone()
    } else {
        format_label(r.entry)
    }
}

/// Parse profile jump-engine target labels into their physical identity.
/// Strip this view's bank prefix from cross-window label references in an
/// MMC3 window routine. A `(bank, window)` analysis view maps the companion
/// window to its power-on bank, so a reference into the other window would
/// otherwise resolve to companion-bank bytes at runtime (when the other
/// window holds its live bank). Unprefixed `L_XXXX` refs resolve through
/// [[bank_call]] facts or fail closed as unresolved — never to the wrong
/// bank's bytes. Same-window refs keep their prefix (direct static labels).
fn unprefix_cross_window_labels(
    routine: &mut ir::Routine,
    prefix: &str,
    window: analysis::AnalysisWindow,
) {
    let tagged = format!("L_{prefix}");
    let fix = |label: &mut String| {
        if let Some(hex) = label.strip_prefix(tagged.as_str())
            && hex.len() == 4
            && let Ok(addr) = u16::from_str_radix(hex, 16)
            && !window.contains(addr)
        {
            *label = format!("L_{hex}");
        }
    };
    for op in routine.ops.iter_mut() {
        match op {
            ir::Op::Label(name) => fix(name),
            ir::Op::BranchIf { target, .. } | ir::Op::Jmp { target } | ir::Op::Jsr { target } => {
                fix(target)
            }
            ir::Op::ReturnEscape { target, .. } | ir::Op::MaterializedJsr { target, .. } => {
                fix(target)
            }
            ir::Op::JumpEngineCall {
                targets,
                return_target,
                ..
            } => {
                for t in targets.iter_mut().chain(return_target.iter_mut()) {
                    fix(t);
                }
            }
            _ => {}
        }
    }
    for lbl in routine
        .branch_labels
        .iter_mut()
        .chain(routine.external_calls.iter_mut())
    {
        fix(lbl);
    }
}

/// Retarget `[[bank_call]]` call sites: any `L_XXXX` reference whose target
/// is annotated by a [[bank_call]] is rewritten to the bank-prefixed label
/// `L_b{bank}_XXXX`; everything else stays an unresolved strict-trap stub.
///
/// UxROM banked units resolve cross-bank calls through the runtime (bank,
/// addr) dispatch shadow, so a [[bank_call]] hard-binding must not override
/// them. MMC3 has two independent windows and no live dispatch shadow, so a
/// cross-window call from a banked unit (e.g. bank-20 LOW code calling
/// $BF62 in bank-19 HIGH) is otherwise unresolvable — bind it here.
fn rewrite_bank_call_targets(prof: &profile::Profile, routines: &mut [ir::Routine], mmc3: bool) {
    if prof.bank_calls.is_empty() {
        return;
    }
    use ir::Op;
    let map: std::collections::HashMap<String, String> = prof
        .bank_calls
        .iter()
        .map(|bc| {
            (
                format!("L_{:04X}", bc.target),
                format!("L_b{}_{:04X}", bc.bank, bc.target),
            )
        })
        .collect();
    for r in routines.iter_mut() {
        if r.name.starts_with("L_b") && !mmc3 {
            continue; // UxROM banked units use the live dispatch shadow
        }
        for op in r.ops.iter_mut() {
            match op {
                Op::Jsr { target }
                | Op::MaterializedJsr { target, .. }
                | Op::Jmp { target }
                | Op::ReturnEscape { target, .. } => {
                    if let Some(new) = map.get(target) {
                        *target = new.clone();
                    }
                }
                _ => {}
            }
        }
        for lbl in r.external_calls.iter_mut() {
            if let Some(new) = map.get(lbl) {
                *lbl = new.clone();
            }
        }
    }
}

/// `L_F000` is a fixed-window address; `L_b6_A123` is switchable bank 6.
/// Unqualified switchable labels intentionally return `(None, addr)` because
/// they mean "the mapper bank selected at runtime" and must not root every
/// physical bank during static analysis.
fn profile_target_identity(label: &str) -> Option<(Option<u8>, u16)> {
    if let Some(rest) = label.strip_prefix("L_b") {
        let (bank, addr) = rest.split_once('_')?;
        return Some((
            Some(bank.parse().ok()?),
            u16::from_str_radix(addr, 16).ok()?,
        ));
    }
    label
        .strip_prefix("L_")
        .filter(|rest| !rest.contains('_'))
        .and_then(|addr| u16::from_str_radix(addr, 16).ok())
        .map(|addr| (None, addr))
}

/// Resolve every profile spelling accepted by lowering, without treating an
/// unqualified switchable address as a newly invented physical-bank fact.
fn consume_target_identity(prof: &profile::Profile, label: &str) -> Option<(Option<u8>, u16)> {
    profile_target_identity(label)
        .or_else(|| {
            prof.functions
                .iter()
                .find(|f| f.name == label)
                .map(|f| (None, f.addr))
        })
        .or_else(|| {
            prof.labels
                .iter()
                .find(|f| f.name == label)
                .map(|f| (None, f.addr))
        })
}

fn check_consume_entries(
    prof: &profile::Profile,
    entries: &[(Option<u8>, u16)],
) -> Result<(), Error> {
    for site in prof
        .return_escapes
        .iter()
        .filter(|s| s.stack_bytes_already_consumed)
    {
        let start = site.consume_at.expect("validated profile");
        if let Some(&(bank, addr)) = entries.iter().find(|(bank, addr)| {
            *addr > start
                && *addr <= site.caller
                && (*addr >= 0xc000 || prof.rom.mapper == 0 || bank.is_none() || *bank == site.bank)
        }) {
            return Err(Error::Diagnostic(format!(
                "return_escape consume_at ${start:04X} has bypass entry ${addr:04X} in bank {bank:?}"
            )));
        }
    }
    for site in &prof.return_consumes {
        if let Some(&(bank, addr)) = entries.iter().find(|(bank, addr)| {
            *addr == site.at + 1
                && (*addr >= 0xc000 || prof.rom.mapper == 0 || bank.is_none() || *bank == site.bank)
        }) {
            return Err(Error::Diagnostic(format!(
                "return_consume ${:04X} has second-PLA bypass entry ${addr:04X} in bank {bank:?}",
                site.at
            )));
        }
    }
    Ok(())
}

fn return_consume_sites(prof: &profile::Profile, bank: Option<u8>) -> Vec<ir::ReturnConsumeSite> {
    prof.return_consumes
        .iter()
        .filter(|s| s.bank == bank)
        .map(|s| ir::ReturnConsumeSite {
            at: s.at,
            return_addrs: s.calls.iter().map(|call| call.caller + 2).collect(),
        })
        .collect()
}

fn materialized_call_sites(
    prof: &profile::Profile,
    bank: Option<u8>,
) -> Vec<ir::MaterializedCallSite> {
    prof.return_consumes
        .iter()
        .flat_map(|s| &s.calls)
        .filter(|s| s.bank == bank)
        .map(|s| ir::MaterializedCallSite {
            caller: s.caller,
            target: s.target,
        })
        .collect()
}

/// Re-run discovery until every decoded internal branch label is owned by a
/// non-overlapping routine range. This matters when a separately discovered
/// entry is embedded inside a larger routine: range normalization trims the
/// outer routine at that entry, and a branch around the embedded routine can
/// otherwise leave its continuation with no translated owner.
fn analyze_with_continuation_roots(
    prg: &[u8],
    vectors: nes_rom_like::Vectors,
    prof: &mut profile::Profile,
    window: analysis::AnalysisWindow,
    bank: Option<u8>,
) -> analysis::Analyzed {
    loop {
        let analyzed = analysis::analyze_in_window(prg, vectors, prof, window, bank);
        let mut normalized = analyzed.functions.functions.clone();
        normalized.sort_by_key(|function| function.addr);
        normalized.dedup_by_key(|function| function.addr);
        for index in 0..normalized.len().saturating_sub(1) {
            let next = normalized[index + 1].addr;
            if normalized[index].end > next {
                normalized[index].end = next;
            }
        }
        normalized.retain(|function| function.end > function.addr);

        let mut continuations = std::collections::BTreeSet::new();
        for function in &analyzed.functions.functions {
            for &label in &function.internal_labels {
                let is_owned = normalized
                    .iter()
                    .any(|owner| label >= owner.addr && label < owner.end);
                if window.contains(label) && !is_owned {
                    continuations.insert(label);
                }
            }
        }
        continuations.retain(|address| {
            prof.functions
                .iter()
                .all(|function| function.addr != *address)
        });
        if continuations.is_empty() {
            return analyzed;
        }

        for address in continuations {
            prof.functions.push(profile::Function {
                addr: address,
                name: prof
                    .label_for(address)
                    .map(str::to_owned)
                    .unwrap_or_else(|| format!("func_{address:04X}")),
                note: Some("branch continuation after embedded routine".to_string()),
            });
        }
    }
}

/// Labels whose definitions belong at this routine's entry address if it is
/// stubbed. A BTreeSet makes aliases stable and eliminates overlap between
/// the routine name, lifted labels, and branch targets.
fn routine_owned_labels(r: &ir::Routine) -> Vec<String> {
    let mut labels = std::collections::BTreeSet::new();
    labels.insert(routine_auto_label(r));
    labels.insert(r.name.clone());
    for op in &r.ops {
        if let ir::Op::Label(label) = op {
            labels.insert(label.clone());
        }
    }
    labels.extend(r.branch_labels.iter().cloned());
    labels.into_iter().collect()
}

/// Define every alias a stubbed routine owns, then emit its single strict
/// unresolved-call trap body. Callers must begin from a routine-local
/// snapshot so this remains an atomic fallback.
fn emit_routine_trap_stub(
    program: &mut z80_emit::Program,
    defined_labels: &mut std::collections::BTreeSet<String>,
    r: &ir::Routine,
) {
    for label in routine_owned_labels(r) {
        if !defined_labels.contains(&label) {
            program.label(&label);
            defined_labels.insert(label);
        }
    }
    program.ld_a_imm(0xEE);
    program.ld_abs_a(0xCB1B);
    program.jp("rt_unresolved_jsr");
}

fn emit_translated_routine(
    program: &mut z80_emit::Program,
    defined_labels: &mut std::collections::BTreeSet<String>,
    lower_failures: &mut Vec<String>,
    r: &ir::Routine,
    opts: &LowerOptions<'_>,
) -> Result<(), Error> {
    let pre_routine_program = program.clone();
    let pre_routine_labels = defined_labels.clone();
    let auto = routine_auto_label(r);
    // A stub-body replacement supersedes the whole translated body: any
    // entry (JSR, JMP, computed dispatch, or a conditional branch from a
    // neighboring routine) lands on the hook. Interior labels are not
    // emitted; a surviving external reference to one fails closed through
    // the unresolved-label machinery. The return discipline matches the
    // profile: native `ret` for native-call builds (SMB), but translated
    // `jp rt_translated_rts` (pop $D300 continuation) for software-frame
    // builds — a native `ret` there would pop a stale native return (callers
    // arrive via jump, not call) and leak the $D300 frame.
    if let Some(profile) = opts.profile
        && let Some(rep) = profile.replacement_for(r.entry)
        && rep.stub_body
    {
        if !defined_labels.contains(&auto) {
            program.label(&auto);
        }
        defined_labels.insert(auto.clone());
        if r.name != auto && !defined_labels.contains(&r.name) {
            program.label(&r.name);
        }
        defined_labels.insert(r.name.clone());
        program.call(&rep.runtime_label.clone());
        if profile.native_calls() {
            program.ret();
        } else {
            // Software-frame discipline: pop the $D300 continuation pushed
            // by the translated call and jump to the caller (a native `ret`
            // would pop a stale native return — callers arrive via jump).
            program.jp("rt_translated_rts");
        }
        return Ok(());
    }
    let lifter_emits_auto = r.branch_labels.contains(&auto) || r.name == auto;
    if r.ops.len() > 600 {
        *program = pre_routine_program;
        *defined_labels = pre_routine_labels;
        emit_routine_trap_stub(program, defined_labels, r);
        lower_failures.push(format!(
            "${:04X} {}: oversize ({} ops) — stubbed as data-walk",
            r.entry,
            r.name,
            r.ops.len()
        ));
        return Ok(());
    }
    if !lifter_emits_auto && !defined_labels.contains(&auto) {
        program.label(&auto);
        defined_labels.insert(auto.clone());
    }
    if lifter_emits_auto {
        defined_labels.insert(auto.clone());
    }
    defined_labels.insert(r.name.clone());
    for bl in &r.branch_labels {
        defined_labels.insert(bl.clone());
    }
    if let Err(e) = lower::lower_routine(program, r, opts) {
        if lower_error_is_fatal(&e) {
            *program = pre_routine_program;
            *defined_labels = pre_routine_labels;
            return Err(Error::Lower(e));
        }
        *program = pre_routine_program;
        *defined_labels = pre_routine_labels;
        emit_routine_trap_stub(program, defined_labels, r);
        lower_failures.push(format!("${:04X} {}: {}", r.entry, r.name, e));
    }
    Ok(())
}

fn emit_translated_vector_aliases(program: &mut z80_emit::Program, reset: u16, nmi: u16, irq: u16) {
    program.label("translated_reset");
    program.translated_tail_jmp(&format_label(reset));
    program.label("translated_nmi");
    program.translated_tail_jmp(&format_label(nmi));
    program.label("translated_irq");
    program.translated_tail_jmp(&format_label(irq));
}

/// Rewrite a lifted WRAM routine's synthetic addresses ($8000+ blob view)
/// back into WRAM space ($6000-$7FFF) under a `L_w_XXXX` label prefix. The
/// entry name is already `L_w_XXXX` (set at lift time) and is left alone;
/// every synthetic `L_XXXX` branch/ref is remapped, and `Source` PCs follow.
fn remap_wram_routine(r: &mut ir::Routine, dest: u16, len: u16) {
    let synth_hi = 0x8000u16.saturating_add(len - 1);
    let to_wram = |synth: u16| dest + (synth - 0x8000);
    let fix_label = |l: &mut String| {
        if let Some(hex) = l.strip_prefix("L_")
            && hex.len() == 4
            && let Ok(a) = u16::from_str_radix(hex, 16)
            && a >= 0x8000
            && a <= synth_hi
        {
            *l = format!("L_w_{:04X}", to_wram(a));
        }
    };
    for op in r.ops.iter_mut() {
        match op {
            ir::Op::Label(l) => fix_label(l),
            ir::Op::Source { pc, .. } => *pc = to_wram(*pc),
            ir::Op::BranchIf { target, .. }
            | ir::Op::Jmp { target }
            | ir::Op::Jsr { target }
            | ir::Op::MaterializedJsr { target, .. }
            | ir::Op::ReturnEscape { target, .. } => fix_label(target),
            ir::Op::JumpEngineCall {
                targets,
                return_target,
                ..
            } => {
                for t in targets.iter_mut() {
                    fix_label(t);
                }
                if let Some(rt) = return_target {
                    fix_label(rt);
                }
            }
            _ => {}
        }
    }
    for l in r
        .branch_labels
        .iter_mut()
        .chain(r.external_calls.iter_mut())
    {
        fix_label(l);
    }
    r.entry = to_wram(r.entry);
    r.end = to_wram(r.end);
}

/// Lift static WRAM code blobs (mapper 4 `[[wram_blob]]`). The engine has no
/// analysis window over $6000-$7FFF (it is PRG-RAM), so each blob is placed
/// at $8000+ in a synthetic view (where `cpu_to_prg` already reads bytes),
/// walked from its declared entries, then remapped back to WRAM space. The
/// routine's `L_XXXX` auto-label is still emitted, so an unannotated
/// `JSR $6047` from PRG code resolves to the translated body directly.
fn lift_wram_blobs(
    chr: &[u8],
    prof: &profile::Profile,
) -> Result<(Vec<ir::Routine>, Vec<String>), Error> {
    let mut routines = Vec::new();
    let mut failures = Vec::new();

    for blob in &prof.wram_blobs {
        let len = usize::from(blob.length);
        let Ok(src) = usize::try_from(blob.source) else {
            return Err(Error::Diagnostic(format!(
                "wram_blob source {:#X} overflows usize",
                blob.source
            )));
        };
        let Some(end) = src.checked_add(len) else {
            return Err(Error::Diagnostic(format!(
                "wram_blob source {:#X}+{:#X} overflows",
                blob.source, len
            )));
        };
        if end > chr.len() {
            return Err(Error::Diagnostic(format!(
                "wram_blob source {:#X}+{:#X} exceeds CHR ROM ({} bytes)",
                blob.source,
                len,
                chr.len()
            )));
        }
        // Synthetic view: blob byte i appears at CPU address $8000+i.
        let mut view = vec![0u8; 0x2000];
        view[..len].copy_from_slice(&chr[src..end]);
        let window = analysis::AnalysisWindow {
            start: 0x8000,
            end_inclusive: 0x8000u16 + len as u16 - 1,
        };
        let synth = |wram: u16| 0x8000 + (wram - blob.dest);

        let mut wprof = prof.clone();
        wprof.functions = blob
            .entries
            .iter()
            .map(|&e| profile::Function {
                addr: synth(e),
                name: format!("L_w_{e:04X}"),
                note: Some("wram_blob entry".to_string()),
            })
            .collect();
        wprof.jump_tables.clear();
        wprof.jump_engines.clear();
        wprof.data_regions.clear();
        wprof.return_escapes.clear();
        wprof.return_consumes.clear();

        let analyzed = analyze_with_continuation_roots(
            &view,
            nes_rom_like::Vectors {
                nmi: 0,
                reset: 0,
                irq: 0,
            },
            &mut wprof,
            window,
            None,
        );
        let mut funcs: Vec<analysis::DiscoveredFunction> = analyzed.functions.functions.clone();
        for f in &funcs {
            if !window.contains(f.addr) || f.end > window.end_inclusive + 1 {
                return Err(Error::Diagnostic(format!(
                    "wram_blob @${:04X}: walk escaped the blob window: ${:04X}-${:04X}",
                    blob.dest, f.addr, f.end
                )));
            }
        }
        funcs.sort_by_key(|f| f.addr);
        funcs.dedup_by_key(|f| f.addr);
        for i in 0..funcs.len().saturating_sub(1) {
            if funcs[i].end > funcs[i + 1].addr {
                funcs[i].end = funcs[i + 1].addr;
            }
        }
        funcs.retain(|f| f.end > f.addr);

        // Pass 1: collect referenced synthetic PCs for cross-routine labels.
        let mut referenced: std::collections::HashSet<u16> = Default::default();
        for f in &funcs {
            let opts = ir::LiftOptions {
                start: f.addr,
                end: f.end,
                entry_name: String::new(),
                jump_engine_sites: Vec::new(),
                return_escape_sites: Vec::new(),
                return_consume_sites: Vec::new(),
                materialized_call_sites: Vec::new(),
                window_label_prefix: None,
                extra_label_pcs: Vec::new(),
            };
            if let Ok(r) = ir::lift_range(&view, &opts) {
                for lbl in r.branch_labels.iter().chain(r.external_calls.iter()) {
                    if let Some(hex) = lbl.strip_prefix("L_")
                        && hex.len() == 4
                        && let Ok(a) = u16::from_str_radix(hex, 16)
                    {
                        referenced.insert(a);
                    }
                }
            }
        }

        // Pass 2: lift each routine and remap to WRAM space.
        for f in &funcs {
            let extras: Vec<u16> = referenced
                .iter()
                .filter(|&&pc| pc > f.addr && pc < f.end)
                .copied()
                .collect();
            let wram_entry = blob.dest + (f.addr - 0x8000);
            let opts = ir::LiftOptions {
                start: f.addr,
                end: f.end,
                entry_name: format!("L_w_{wram_entry:04X}"),
                jump_engine_sites: Vec::new(),
                return_escape_sites: Vec::new(),
                return_consume_sites: Vec::new(),
                materialized_call_sites: Vec::new(),
                window_label_prefix: None,
                extra_label_pcs: extras,
            };
            match ir::lift_range(&view, &opts) {
                Ok(mut r) => {
                    ir::mark_rts_dispatch(&mut r.ops);
                    if !r.ops.last().is_some_and(ir::Op::is_hard_terminator) {
                        let tgt = format!("L_w_{:04X}", blob.dest + (r.end - 0x8000));
                        if !r.external_calls.contains(&tgt) {
                            r.external_calls.push(tgt.clone());
                        }
                        r.ops.push(ir::Op::Jmp { target: tgt });
                    }
                    remap_wram_routine(&mut r, blob.dest, blob.length);
                    routines.push(r);
                }
                Err(e) => failures.push(format!("wram_blob @${:04X}: {:?}", f.addr, e)),
            }
        }
    }
    Ok((routines, failures))
}

pub fn run(args: &Args) -> Result<String, Error> {
    // 1. Read and parse the ROM.
    let rom_bytes = std::fs::read(&args.rom)?;
    let image = nes_rom::parse(&rom_bytes)?;
    // 2. Load the profile.
    let prof = profile::load_from_path(&args.profile)?;

    if prof.native_calls() && prof.rom.mapper != 0 {
        return Err(Error::Diagnostic(format!(
            "stack_discipline = \"native\" requires mapper 0 (NROM); profile mapper is {}",
            prof.rom.mapper
        )));
    }
    if prof.native_calls() && !prof.return_escapes.is_empty() {
        return Err(Error::Diagnostic(
            "stack_discipline = \"native\" is incompatible with [[return_escape]] sites"
                .to_string(),
        ));
    }

    // Sanity-check the profile against the parsed ROM.
    let actual_prg_kib = image.prg.len() / 1024;
    if prof.rom.prg_kib as usize != actual_prg_kib {
        return Err(Error::Diagnostic(format!(
            "profile PRG size mismatch: profile prg_kib={} KiB, ROM PRG payload={} KiB ({} bytes)",
            prof.rom.prg_kib,
            actual_prg_kib,
            image.prg.len()
        )));
    }
    let actual_chr_kib = image.chr.len() / 1024;
    if prof.rom.chr_kib as usize != actual_chr_kib {
        return Err(Error::Diagnostic(format!(
            "profile CHR size mismatch: profile chr_kib={} KiB, ROM CHR-ROM payload={} KiB ({} bytes)",
            prof.rom.chr_kib,
            actual_chr_kib,
            image.chr.len()
        )));
    }
    if let Some(expected) = &prof.rom.payload_sha256 {
        let actual = nes_rom::payload_sha256_hex(image.prg, image.chr);
        if expected != &actual {
            return Err(Error::Diagnostic(format!(
                "profile payload SHA-256 mismatch: expected {expected}, actual {actual}"
            )));
        }
    }
    if prof.rom.mapper != image.header.mapper {
        return Err(Error::Diagnostic(format!(
            "profile mapper={} but ROM mapper={}",
            prof.rom.mapper, image.header.mapper
        )));
    }
    /// MMC3 discovery-only mode (mapper plan M3): the loader, reference bus,
    /// profile schema and window views have landed, but IR lowering and runtime
    /// emission are still Phase M3 work. Run fixed + windowed discovery, write
    /// the classification reports that guide `[[bank_entry]]` authoring, then
    /// fail closed instead of emitting a bogus project.
    fn run_mmc3_discovery_only(
        args: &Args,
        image: &nes_rom::Image<'_>,
        prof: &profile::Profile,
        policy: nes_rom::MapperPolicy,
        prg_8k_count: u8,
        vectors: nes_rom::Vectors,
    ) -> Result<String, Error> {
        use analysis::AnalysisWindow as W;
        use std::collections::BTreeSet;

        let mut txt = String::new();
        txt.push_str(
            "MMC3 discovery-only: lowering/runtime emission not yet wired; no project emitted.\n",
        );
        txt.push_str(
        "Fixed pass assumes PRG mode 0 ($C000-$DFFF = second-last bank); $E000-$FFFF is mode-independent.\n",
    );

        // Fixed pass over the mode-0 top: [fixed16 | fixed16] so every
        // $8000-$FFFF offset lands in fixed bytes; the walk is constrained to
        // $C000-$FFFF and rooted at vectors + profile fixed sites. Bankless
        // jump-engine sites are fixed (validation forces window callers to
        // carry banks).
        let fixed_bytes = policy.fixed_prg(image.prg);
        let mut fixed_view = Vec::with_capacity(2 * fixed_bytes.len());
        fixed_view.extend_from_slice(fixed_bytes);
        fixed_view.extend_from_slice(fixed_bytes);
        let mut fixed_prof = prof.clone();
        fixed_prof.functions.retain(|f| f.addr >= 0xC000);
        fixed_prof.jump_engines.retain(|site| site.bank.is_none());
        let fixed = analyze_with_continuation_roots(
            &fixed_view,
            nes_rom_like::Vectors {
                nmi: vectors.nmi,
                reset: vectors.reset,
                irq: vectors.irq,
            },
            &mut fixed_prof,
            W::FIXED_16K,
            None,
        );
        let (code, data, unknown) = fixed.class_map.summary();
        txt.push_str(&format!(
        "fixed mode-0 top: {} functions, {code} code / {data} data / {unknown} unknown view bytes\n",
        fixed.functions.functions.len()
    ));
        let mut fixed_window_refs = BTreeSet::new();
        for f in &fixed.functions.functions {
            for &ext in &f.external_refs {
                if (0x8000..0xC000).contains(&ext) {
                    fixed_window_refs.insert(ext);
                }
            }
        }
        // Live 8 KiB banks for these come from the reference harvest
        // (FD_LOG_BANK_ENTRIES -> MMC3_ENTRY), never invented here.
        for ext in &fixed_window_refs {
            let window = if *ext < 0xA000 { "LOW " } else { "HIGH" };
            txt.push_str(&format!("  fixed -> window {window} ${ext:04X}\n"));
        }
        // Static contiguous-idiom candidates (UNVERIFIED: linear matching
        // cannot see joins — confirm each against the MMC3_ENTRY harvest
        // before writing a [[bank_entry]]). Scan only the $C000+ half: the
        // view's low half mirrors the same fixed bytes at $8000-$BFFF coords,
        // where they are not real window code.
        for candidate in
            nes_rom::harvest_mmc3_bank_candidates(&fixed_view[fixed_bytes.len()..], prg_8k_count)
        {
            txt.push_str(&format!(
            "  CANDIDATE bank8={} target=${:04X} (at ${:04X}, mode {}) — verify via reference harvest\n",
            candidate.window_bank,
            candidate.target,
            0xC000 + candidate.offset,
            u8::from(candidate.prg_mode),
        ));
        }

        // Window passes, grouped by (bank, window). The companion window holds
        // its power-on bank (R6=0 low, R7=1 high); the walk never leaves the
        // entry window, so it only shapes stray decodes, never roots.
        let mut groups: std::collections::BTreeMap<(u8, bool), Vec<u16>> =
            std::collections::BTreeMap::new();
        for entry in &prof.bank_entries {
            groups
                .entry((entry.bank, entry.addr < 0xA000))
                .or_default()
                .push(entry.addr);
        }
        let mut window_funcs = 0usize;
        for ((bank, low), addrs) in &groups {
            let window = if *low {
                W::SWITCHABLE_8K_LOW
            } else {
                W::SWITCHABLE_8K_HIGH
            };
            let (low_bank, high_bank) = if *low { (*bank, 1) } else { (0, *bank) };
            let view = nes_rom::mmc3_analysis_view(image.prg, prg_8k_count, low_bank, high_bank)?;
            let mut wprof = prof.clone();
            wprof.functions = addrs
                .iter()
                .map(|&addr| profile::Function {
                    addr,
                    name: format!("L_b{bank}_{addr:04X}"),
                    note: None,
                })
                .collect();
            wprof.jump_tables.clear();
            wprof
                .jump_engines
                .retain(|site| site.bank == Some(*bank) && window.contains(site.caller));
            let analyzed = analyze_with_continuation_roots(
                &view,
                nes_rom_like::Vectors {
                    nmi: 0,
                    reset: 0,
                    irq: 0,
                },
                &mut wprof,
                window,
                Some(*bank),
            );
            let (wcode, wdata, wunknown) = analyzed.class_map.summary();
            let name = if *low { "LOW " } else { "HIGH" };
            txt.push_str(&format!(
            "window {name} bank8={bank}: {} functions, {wcode} code / {wdata} data / {wunknown} unknown view bytes\n",
            analyzed.functions.functions.len()
        ));
            window_funcs += analyzed.functions.functions.len();
            let mut externals = BTreeSet::new();
            for f in &analyzed.functions.functions {
                externals.extend(f.external_refs.iter().copied());
            }
            for ext in externals {
                txt.push_str(&format!("  bank8={bank} ${ext:04X}\n"));
            }
        }

        let reports_dir = args.out.join("reports");
        std::fs::create_dir_all(&reports_dir)?;
        std::fs::write(reports_dir.join("discovery.txt"), &txt)?;
        Err(Error::Diagnostic(format!(
            "mapper 4 (MMC3) discovery-only: {} fixed + {window_funcs} window functions, {} fixed->window refs; see out/reports/discovery.txt (lowering not yet wired)",
            fixed.functions.functions.len(),
            fixed_window_refs.len()
        )))
    }
    // Profile validation checks declared bank bounds per mapper. Recheck
    // against the parsed ROM policy before any banked analysis slicing.
    let policy = nes_rom::resolve_mapper_policy(&image.header, image.prg.len())?;
    if policy.is_banked() {
        let actual_bank_count = policy.bank_count();
        let mapper = prof.rom.mapper;
        for entry in &prof.bank_entries {
            if entry.bank >= actual_bank_count {
                return Err(Error::Diagnostic(format!(
                    "bank_entry bank {} is out of range for parsed ROM's {actual_bank_count} mapper {mapper} banks",
                    entry.bank
                )));
            }
        }
        for call in &prof.bank_calls {
            if call.bank >= actual_bank_count {
                return Err(Error::Diagnostic(format!(
                    "bank_call bank {} is out of range for parsed ROM's {actual_bank_count} mapper {mapper} banks",
                    call.bank
                )));
            }
        }
    }
    let vectors = nes_rom::read_vectors_with_policy(policy, image.prg)?
        .ok_or_else(|| Error::Diagnostic("could not read NMI/RESET/IRQ vectors from PRG".into()))?;
    if let Some(v) = prof.vectors {
        if v.reset != vectors.reset {
            return Err(Error::Diagnostic(format!(
                "profile RESET=${:04X} != ROM RESET=${:04X}",
                v.reset, vectors.reset
            )));
        }
        if v.nmi != vectors.nmi {
            return Err(Error::Diagnostic(format!(
                "profile NMI=${:04X} != ROM NMI=${:04X}",
                v.nmi, vectors.nmi
            )));
        }
    }

    // MMC3 flows through the banked paths below with 8 KiB (bank, window)
    // units: fixed code comes from the doubled fixed view, window units
    // from mmc3_analysis_view per (bank, LOW/HIGH) group. run_mmc3_discovery_only
    // above remains for report-only runs.
    let mmc3 = matches!(policy, nes_rom::MapperPolicy::Mmc3 { .. });

    // 3. Analyze. UxROM fixed code is discovered exactly once. Each physical
    // switchable bank is analyzed separately below and is rooted only by its
    // verified [[bank_entry]] facts; vectors are never replayed in those views.
    let banked = policy.is_banked();
    let mut consume_entries: Vec<(Option<u8>, u16)> = [vectors.nmi, vectors.reset, vectors.irq]
        .into_iter()
        .map(|pc| (None, pc))
        .collect();
    consume_entries.extend(prof.functions.iter().map(|f| (None, f.addr)));
    consume_entries.extend(prof.bank_entries.iter().map(|f| (Some(f.bank), f.addr)));
    consume_entries.extend(prof.bank_calls.iter().map(|f| (Some(f.bank), f.target)));
    consume_entries.extend(
        prof.jump_tables
            .iter()
            .flat_map(|t| t.targets.iter())
            .map(|&pc| (None, pc)),
    );
    consume_entries.extend(prof.replacements.iter().map(|r| (None, r.addr)));
    consume_entries.extend(
        prof.jump_engines
            .iter()
            .flat_map(|s| s.targets.iter().chain(s.return_target.iter()))
            .filter_map(|target| consume_target_identity(&prof, target)),
    );
    check_consume_entries(&prof, &consume_entries)?;
    let mut bank_entries_by_bank: std::collections::BTreeMap<u8, Vec<u16>> =
        std::collections::BTreeMap::new();
    for entry in &prof.bank_entries {
        bank_entries_by_bank
            .entry(entry.bank)
            .or_default()
            .push(entry.addr);
    }
    // MMC3 groups by (8 KiB bank, LOW/HIGH window): address ranges never
    // overlap across windows, so (bank, addr) stays unique.
    let mut mmc3_entries_by_group: std::collections::BTreeMap<(u8, bool), Vec<u16>> =
        std::collections::BTreeMap::new();
    if mmc3 {
        for entry in &prof.bank_entries {
            mmc3_entries_by_group
                .entry((entry.bank, entry.addr < 0xA000))
                .or_default()
                .push(entry.addr);
        }
    }
    // A profiled inline table is a real reachability edge. Root its fixed
    // targets and its explicitly bank-qualified window targets; leave
    // unqualified window targets dynamic so analysis never invents physical
    // bank facts that the reference/profile did not establish.
    for site in &prof.jump_engines {
        for target in site.targets.iter().chain(site.return_target.iter()) {
            if let Some((Some(bank), addr)) = profile_target_identity(target)
                && addr < 0xC000
            {
                bank_entries_by_bank.entry(bank).or_default().push(addr);
                if mmc3 {
                    mmc3_entries_by_group
                        .entry((bank, addr < 0xA000))
                        .or_default()
                        .push(addr);
                }
            }
        }
    }
    for entries in bank_entries_by_bank.values_mut() {
        entries.sort_unstable();
        entries.dedup();
    }
    for entries in mmc3_entries_by_group.values_mut() {
        entries.sort_unstable();
        entries.dedup();
    }

    // A window routine can call shared fixed code that is not otherwise a
    // vector/profile root. Pre-discover those cross-window references and feed
    // them into the one fixed-bank pass. The window pass itself cannot walk or
    // classify fixed bytes.
    let mut fixed_prof = prof.clone();
    for site in &prof.jump_engines {
        for target in site.targets.iter().chain(site.return_target.iter()) {
            if let Some((_, addr)) = profile_target_identity(target)
                && ((!banked && addr >= 0x8000) || (banked && addr >= 0xC000))
                && fixed_prof
                    .functions
                    .iter()
                    .all(|function| function.addr != addr)
            {
                fixed_prof.functions.push(profile::Function {
                    addr,
                    name: prof
                        .label_for(addr)
                        .map(str::to_owned)
                        .unwrap_or_else(|| format!("func_{addr:04X}")),
                    note: Some("profiled JumpEngine target".to_string()),
                });
            }
        }
    }
    if banked {
        if mmc3 {
            // MMC3 fixed-pre-pass: one view per (bank, window) group with
            // the companion window holding its power-on bank (R6=0 low,
            // R7=1 high), exactly like the discovery pass.
            let prg_8k_count = policy.bank_count();
            for ((bank, low), entries) in &mmc3_entries_by_group {
                let (low_bank, high_bank) = if *low { (*bank, 1) } else { (0, *bank) };
                let view =
                    nes_rom::mmc3_analysis_view(image.prg, prg_8k_count, low_bank, high_bank)?;
                let window = if *low {
                    analysis::AnalysisWindow::SWITCHABLE_8K_LOW
                } else {
                    analysis::AnalysisWindow::SWITCHABLE_8K_HIGH
                };
                let mut window_prof = prof.clone();
                window_prof.functions = entries
                    .iter()
                    .map(|&addr| profile::Function {
                        addr,
                        name: format!("L_b{bank}_{addr:04X}"),
                        note: None,
                    })
                    .collect();
                window_prof.jump_tables.clear();
                window_prof
                    .jump_engines
                    .retain(|site| site.bank == Some(*bank) && window.contains(site.caller));
                let window_analysis = analyze_with_continuation_roots(
                    &view,
                    nes_rom_like::Vectors {
                        nmi: 0,
                        reset: 0,
                        irq: 0,
                    },
                    &mut window_prof,
                    window,
                    Some(*bank),
                );
                for target in window_analysis
                    .functions
                    .functions
                    .iter()
                    .flat_map(|function| function.external_refs.iter().copied())
                    .filter(|&target| target >= 0xC000)
                {
                    if fixed_prof
                        .functions
                        .iter()
                        .all(|function| function.addr != target)
                    {
                        fixed_prof.functions.push(profile::Function {
                            addr: target,
                            name: prof
                                .label_for(target)
                                .map(str::to_owned)
                                .unwrap_or_else(|| format!("func_{target:04X}")),
                            note: Some(format!("called from MMC3 bank {bank}")),
                        });
                    }
                }
            }
        } else {
            for (&bank, entries) in &bank_entries_by_bank {
                let view = policy.analysis_view(image.prg, bank)?;
                let mut window_prof = prof.clone();
                window_prof.functions = entries
                    .iter()
                    .map(|&addr| profile::Function {
                        addr,
                        name: format!("L_b{bank}_{addr:04X}"),
                        note: None,
                    })
                    .collect();
                window_prof.jump_tables.clear();
                window_prof
                    .jump_engines
                    .retain(|site| site.bank == Some(bank));
                let window_analysis = analyze_with_continuation_roots(
                    &view,
                    nes_rom_like::Vectors {
                        nmi: 0,
                        reset: 0,
                        irq: 0,
                    },
                    &mut window_prof,
                    analysis::AnalysisWindow::SWITCHABLE_16K,
                    Some(bank),
                );
                for target in window_analysis
                    .functions
                    .functions
                    .iter()
                    .flat_map(|function| function.external_refs.iter().copied())
                    .filter(|&target| target >= 0xC000)
                {
                    if fixed_prof
                        .functions
                        .iter()
                        .all(|function| function.addr != target)
                    {
                        fixed_prof.functions.push(profile::Function {
                            addr: target,
                            name: prof
                                .label_for(target)
                                .map(str::to_owned)
                                .unwrap_or_else(|| format!("func_{target:04X}")),
                            note: Some(format!("called from mapper bank {bank}")),
                        });
                    }
                }
            }
        } // end UxROM per-bank pre-pass (`else` of the MMC3 branch above)
        fixed_prof.jump_engines.retain(|site| site.bank.is_none());
    }
    // MMC3 fixed code is mode-independent only in $E000-$FFFF; the
    // discovery pass reads the fixed 16 KiB doubled, constraining the walk
    // to $C000-$FFFF (mode-0 top). Emission reuses that exact view so
    // fixed labels resolve to the same bytes the reports describe.
    let analysis_view: Vec<u8> = if mmc3 {
        let fixed_bytes = policy.fixed_prg(image.prg);
        let mut doubled = Vec::with_capacity(2 * fixed_bytes.len());
        doubled.extend_from_slice(fixed_bytes);
        doubled.extend_from_slice(fixed_bytes);
        doubled
    } else {
        policy.analysis_view(image.prg, 0)?
    };
    let analysis_vectors = nes_rom_like::Vectors {
        nmi: vectors.nmi,
        reset: vectors.reset,
        irq: vectors.irq,
    };
    let analyzed = if banked {
        analyze_with_continuation_roots(
            &analysis_view,
            analysis_vectors,
            &mut fixed_prof,
            analysis::AnalysisWindow::FIXED_16K,
            None,
        )
    } else {
        analyze_with_continuation_roots(
            &analysis_view,
            analysis_vectors,
            &mut fixed_prof,
            analysis::AnalysisWindow::FULL_PRG,
            None,
        )
    };

    // 4. Lift each discovered function into IR.
    //
    // Analysis can produce overlapping ranges when a routine's linear walk
    // crosses into another known root's entry. Trim each routine's end to
    // be no later than the next routine's start, so two routines never
    // both contain the same byte. This avoids duplicate L_XXXX labels in
    // the lowered output; internal branches that target the trimmed-off
    // tail become external references and resolve via the alias label.
    let mut funcs: Vec<analysis::DiscoveredFunction> = analyzed.functions.functions.clone();
    for f in &funcs {
        consume_entries.push((None, f.addr));
        consume_entries.extend(
            f.external_refs
                .iter()
                .chain(f.internal_labels.iter())
                .map(|&pc| (None, pc)),
        );
    }
    funcs.sort_by_key(|f| f.addr);
    for i in 0..funcs.len() {
        if i + 1 < funcs.len() && funcs[i].end > funcs[i + 1].addr {
            funcs[i].end = funcs[i + 1].addr;
        }
    }
    funcs.retain(|f| f.end > f.addr);

    // Convert profile jump-engine sites into the IR's representation once.
    let jump_engine_sites: Vec<ir::JumpEngineSite> = prof
        .jump_engines
        .iter()
        .filter(|site| !banked || site.bank.is_none())
        .map(|s| ir::JumpEngineSite {
            caller: s.caller,
            targets: s.targets.clone(),
            return_target: s.return_target.clone(),
            tail_indices: s.tail_indices.clone(),
            stack_return_bytes: s.stack_return_bytes,
            target_entry_a: s.target_entry_a.clone(),
        })
        .collect();
    let return_escape_sites: Vec<ir::ReturnEscapeSite> = prof
        .return_escapes
        .iter()
        .filter(|site| !banked || site.bank.is_none())
        .map(|site| ir::ReturnEscapeSite {
            caller: site.caller,
            target: site.target,
            return_addr: site.return_addr,
            stack_bytes_already_consumed: site.stack_bytes_already_consumed,
            consume_at: site.consume_at,
        })
        .collect();

    // ---- Pass 1: lift everything once to collect all branch-target PCs ----
    // Each routine emits `L_XXXX` labels for its own internal targets,
    // but cross-routine branches (e.g., BEQ from InitScreen to a PC
    // inside SetVRAMAddr_A) only see them as external references. We
    // collect every `L_XXXX` name across all routines so the second
    // pass can emit a real label inside whichever routine owns that PC.
    let mut all_referenced_pcs: std::collections::HashSet<u16> = std::collections::HashSet::new();
    for f in &funcs {
        let opts = ir::LiftOptions {
            start: f.addr,
            end: f.end,
            entry_name: f.name.clone(),
            jump_engine_sites: jump_engine_sites.clone(),
            return_escape_sites: return_escape_sites.clone(),
            return_consume_sites: return_consume_sites(&prof, None),
            materialized_call_sites: materialized_call_sites(&prof, None),
            window_label_prefix: None,
            extra_label_pcs: Vec::new(),
        };
        if let Ok(r) = ir::lift_range(&analysis_view, &opts) {
            for lbl in r.branch_labels.iter().chain(r.external_calls.iter()) {
                if let Some(hex) = lbl.strip_prefix("L_")
                    && let Ok(addr) = u16::from_str_radix(hex, 16)
                {
                    all_referenced_pcs.insert(addr);
                }
            }
            let has_terminator = r.ops.last().is_some_and(ir::Op::is_hard_terminator);
            if !has_terminator {
                // If the final decoded instruction overlapped the next known
                // root (SMB uses BIT-operand alternate entries), the real
                // fallthrough is the decoded next PC, not the trimmed range
                // end. Pre-mark it so the owner routine emits an interior
                // label on pass 2.
                all_referenced_pcs.insert(r.end);
            }
        }
    }

    let mut lift_failures: Vec<String> = Vec::new();
    let mut banked_routines: Vec<ir::Routine> = Vec::new();
    // 4b. Banked-window translation units. Each physical bank is rooted only
    // at its verified entries and constrained to $8000-$BFFF. Calls into the
    // fixed window were folded into the one fixed pass above.
    if banked && !prof.bank_entries.is_empty() {
        // MMC3 window units keyed by (8 KiB bank, LOW/HIGH window). Mirrors
        // the UxROM loop below with window views; see the callouts.
        if mmc3 {
            let prg_8k_count = policy.bank_count();
            for ((bank, low), entries) in &mmc3_entries_by_group {
                let bank = *bank;
                let entry_low = *low;
                let window = if entry_low {
                    analysis::AnalysisWindow::SWITCHABLE_8K_LOW
                } else {
                    analysis::AnalysisWindow::SWITCHABLE_8K_HIGH
                };
                let (low_bank, high_bank) = if entry_low { (bank, 1) } else { (0, bank) };
                let view =
                    nes_rom::mmc3_analysis_view(image.prg, prg_8k_count, low_bank, high_bank)?;
                let prefix = format!("b{bank}_");
                let mut bprof = prof.clone();
                bprof.functions = entries
                    .iter()
                    .map(|&a| profile::Function {
                        addr: a,
                        name: format!("L_{prefix}{a:04X}"),
                        note: None,
                    })
                    .collect();
                bprof.jump_tables.clear();
                bprof
                    .jump_engines
                    .retain(|site| site.bank == Some(bank) && window.contains(site.caller));
                let banalyzed = analyze_with_continuation_roots(
                    &view,
                    nes_rom_like::Vectors {
                        nmi: 0,
                        reset: 0,
                        irq: 0,
                    },
                    &mut bprof,
                    window,
                    Some(bank),
                );
                let mut bfuncs: Vec<analysis::DiscoveredFunction> =
                    banalyzed.functions.functions.clone();
                for f in &bfuncs {
                    // Fail closed if the analyzer leaks across the window:
                    // bytes would decode under the companion bank.
                    if !window.contains(f.addr) || f.end > window.end_inclusive + 1 {
                        return Err(Error::Diagnostic(format!(
                            "MMC3 bank{bank} window walk escaped {}: ${:04X}-${:04X}",
                            if entry_low { "LOW" } else { "HIGH" },
                            f.addr,
                            f.end,
                        )));
                    }
                    consume_entries.push((Some(bank), f.addr));
                    consume_entries.extend(
                        f.external_refs
                            .iter()
                            .chain(f.internal_labels.iter())
                            .map(|&pc| (Some(bank), pc)),
                    );
                }
                bfuncs.sort_by_key(|f| f.addr);
                bfuncs.dedup_by_key(|f| f.addr);
                for w in 0..bfuncs.len().saturating_sub(1) {
                    let next = bfuncs[w + 1].addr;
                    if bfuncs[w].end > next {
                        bfuncs[w].end = next;
                    }
                }
                let bank_jump_engine_sites: Vec<ir::JumpEngineSite> = prof
                    .jump_engines
                    .iter()
                    .filter(|site| site.bank == Some(bank) && window.contains(site.caller))
                    .map(|site| ir::JumpEngineSite {
                        caller: site.caller,
                        targets: site.targets.clone(),
                        return_target: site.return_target.clone(),
                        tail_indices: site.tail_indices.clone(),
                        stack_return_bytes: site.stack_return_bytes,
                        target_entry_a: site.target_entry_a.clone(),
                    })
                    .collect();
                let bank_return_escape_sites: Vec<ir::ReturnEscapeSite> = prof
                    .return_escapes
                    .iter()
                    .filter(|site| site.bank == Some(bank))
                    .map(|site| ir::ReturnEscapeSite {
                        caller: site.caller,
                        target: site.target,
                        return_addr: site.return_addr,
                        stack_bytes_already_consumed: site.stack_bytes_already_consumed,
                        consume_at: site.consume_at,
                    })
                    .collect();
                let mut bank_referenced: std::collections::HashSet<u16> = Default::default();
                for f in &bfuncs {
                    let opts = ir::LiftOptions {
                        start: f.addr,
                        end: f.end,
                        entry_name: String::new(),
                        jump_engine_sites: bank_jump_engine_sites.clone(),
                        return_escape_sites: bank_return_escape_sites.clone(),
                        return_consume_sites: return_consume_sites(&prof, Some(bank)),
                        materialized_call_sites: materialized_call_sites(&prof, Some(bank)),
                        window_label_prefix: window.contains(f.addr).then(|| prefix.clone()),
                        extra_label_pcs: Vec::new(),
                    };
                    if let Ok(r) = ir::lift_range(&view, &opts) {
                        for lbl in r.branch_labels.iter().chain(r.external_calls.iter()) {
                            if let Some(hex) = lbl
                                .strip_prefix(&format!("L_{prefix}"))
                                .or_else(|| lbl.strip_prefix("L_"))
                                && hex.len() == 4
                                && let Ok(a) = u16::from_str_radix(hex, 16)
                            {
                                bank_referenced.insert(a);
                            }
                        }
                    }
                }
                for f in &bfuncs {
                    let in_window = window.contains(f.addr);
                    let extras: Vec<u16> = bank_referenced
                        .iter()
                        .filter(|&&pc| pc > f.addr && pc < f.end)
                        .copied()
                        .collect();
                    let opts = ir::LiftOptions {
                        start: f.addr,
                        end: f.end,
                        entry_name: if in_window {
                            format!("L_{prefix}{:04X}", f.addr)
                        } else {
                            format_label(f.addr)
                        },
                        jump_engine_sites: bank_jump_engine_sites.clone(),
                        return_escape_sites: bank_return_escape_sites.clone(),
                        return_consume_sites: return_consume_sites(&prof, Some(bank)),
                        materialized_call_sites: materialized_call_sites(&prof, Some(bank)),
                        window_label_prefix: in_window.then(|| prefix.clone()),
                        extra_label_pcs: extras,
                    };
                    match ir::lift_range(&view, &opts) {
                        Ok(mut r) => {
                            ir::mark_rts_dispatch(&mut r.ops);
                            // Cross-window refs carry this view's bank prefix
                            // but execute under the OTHER window's live bank
                            // (the companion image here is power-on, not
                            // live). Strip the prefix so they resolve through
                            // [[bank_call]] facts or fail closed as
                            // unresolved — never to companion-bank bytes.
                            unprefix_cross_window_labels(&mut r, &prefix, window);
                            let has_terminator =
                                r.ops.last().is_some_and(ir::Op::is_hard_terminator);
                            if !has_terminator {
                                let tgt = if window.contains(r.end) {
                                    format!("L_{prefix}{:04X}", r.end)
                                } else {
                                    format_label(r.end)
                                };
                                if !r.external_calls.contains(&tgt) {
                                    r.external_calls.push(tgt.clone());
                                }
                                r.ops.push(ir::Op::Jmp { target: tgt });
                            }
                            for lbl in r.branch_labels.iter().chain(r.external_calls.iter()) {
                                if let Some(hex) = lbl.strip_prefix("L_")
                                    && hex.len() == 4
                                    && let Ok(a) = u16::from_str_radix(hex, 16)
                                {
                                    all_referenced_pcs.insert(a);
                                }
                            }
                            banked_routines.push(r)
                        }
                        Err(e) => {
                            lift_failures.push(format!("bank{bank} ${:04X}: {:?}", f.addr, e))
                        }
                    }
                }
            }
        } else {
            for (bank, entries) in bank_entries_by_bank {
                let view = policy.analysis_view(image.prg, bank)?;
                let prefix = format!("b{bank}_");
                let mut bprof = prof.clone();
                bprof.functions = entries
                    .iter()
                    .map(|&a| profile::Function {
                        addr: a,
                        name: format!("L_{prefix}{a:04X}"),
                        note: None,
                    })
                    .collect();
                bprof.jump_tables.clear();
                bprof.jump_engines.retain(|site| site.bank == Some(bank));
                let banalyzed = analyze_with_continuation_roots(
                    &view,
                    nes_rom_like::Vectors {
                        nmi: 0,
                        reset: 0,
                        irq: 0,
                    },
                    &mut bprof,
                    analysis::AnalysisWindow::SWITCHABLE_16K,
                    Some(bank),
                );
                let mut bfuncs: Vec<analysis::DiscoveredFunction> =
                    banalyzed.functions.functions.clone();
                for f in &bfuncs {
                    consume_entries.push((Some(bank), f.addr));
                    consume_entries.extend(
                        f.external_refs
                            .iter()
                            .chain(f.internal_labels.iter())
                            .map(|&pc| (Some(bank), pc)),
                    );
                }
                bfuncs.sort_by_key(|f| f.addr);
                bfuncs.dedup_by_key(|f| f.addr);
                if std::env::var("N2S_DEBUG_BANKFUNCS").is_ok() {
                    for f in &bfuncs {
                        eprintln!("bank{bank} func ${:04X}-${:04X} {}", f.addr, f.end, f.name);
                    }
                }
                for w in 0..bfuncs.len().saturating_sub(1) {
                    let next = bfuncs[w + 1].addr;
                    if bfuncs[w].end > next {
                        bfuncs[w].end = next;
                    }
                }
                let bank_jump_engine_sites: Vec<ir::JumpEngineSite> = prof
                    .jump_engines
                    .iter()
                    .filter(|site| site.bank == Some(bank))
                    .map(|site| ir::JumpEngineSite {
                        caller: site.caller,
                        targets: site.targets.clone(),
                        return_target: site.return_target.clone(),
                        tail_indices: site.tail_indices.clone(),
                        stack_return_bytes: site.stack_return_bytes,
                        target_entry_a: site.target_entry_a.clone(),
                    })
                    .collect();
                let bank_return_escape_sites: Vec<ir::ReturnEscapeSite> = prof
                    .return_escapes
                    .iter()
                    .filter(|site| site.bank == Some(bank))
                    .map(|site| ir::ReturnEscapeSite {
                        caller: site.caller,
                        target: site.target,
                        return_addr: site.return_addr,
                        stack_bytes_already_consumed: site.stack_bytes_already_consumed,
                        consume_at: site.consume_at,
                    })
                    .collect();
                // Interior-alias pass (mirrors the main funcs' two-pass):
                // collect every referenced window pc, then re-lift with
                // extra labels so cross-routine branch targets resolve.
                let mut bank_referenced: std::collections::HashSet<u16> = Default::default();
                for f in &bfuncs {
                    let opts = ir::LiftOptions {
                        start: f.addr,
                        end: f.end,
                        entry_name: String::new(),
                        jump_engine_sites: bank_jump_engine_sites.clone(),
                        return_escape_sites: bank_return_escape_sites.clone(),
                        return_consume_sites: return_consume_sites(&prof, Some(bank)),
                        materialized_call_sites: materialized_call_sites(&prof, Some(bank)),
                        window_label_prefix: (f.addr < 0xC000).then(|| prefix.clone()),
                        extra_label_pcs: Vec::new(),
                    };
                    if let Ok(r) = ir::lift_range(&view, &opts) {
                        for lbl in r.branch_labels.iter().chain(r.external_calls.iter()) {
                            if let Some(hex) = lbl
                                .strip_prefix(&format!("L_{prefix}"))
                                .or_else(|| lbl.strip_prefix("L_"))
                                && hex.len() == 4
                                && let Ok(a) = u16::from_str_radix(hex, 16)
                            {
                                bank_referenced.insert(a);
                            }
                        }
                    }
                }
                for f in &bfuncs {
                    let in_window = f.addr < 0xC000;
                    let extras: Vec<u16> = bank_referenced
                        .iter()
                        .filter(|&&pc| pc > f.addr && pc < f.end)
                        .copied()
                        .collect();
                    let opts = ir::LiftOptions {
                        start: f.addr,
                        end: f.end,
                        entry_name: if in_window {
                            format!("L_{prefix}{:04X}", f.addr)
                        } else {
                            format_label(f.addr)
                        },
                        jump_engine_sites: bank_jump_engine_sites.clone(),
                        return_escape_sites: bank_return_escape_sites.clone(),
                        return_consume_sites: return_consume_sites(&prof, Some(bank)),
                        materialized_call_sites: materialized_call_sites(&prof, Some(bank)),
                        window_label_prefix: in_window.then(|| prefix.clone()),
                        extra_label_pcs: extras,
                    };
                    match ir::lift_range(&view, &opts) {
                        Ok(mut r) => {
                            ir::mark_rts_dispatch(&mut r.ops);
                            let has_terminator =
                                r.ops.last().is_some_and(ir::Op::is_hard_terminator);
                            if !has_terminator {
                                // Trimmed fallthrough: continue into the next
                                // routine via an explicit jump (bank-prefixed
                                // when the target is in the window).
                                let tgt = if r.end < 0xC000 {
                                    format!("L_{prefix}{:04X}", r.end)
                                } else {
                                    format_label(r.end)
                                };
                                if !r.external_calls.contains(&tgt) {
                                    r.external_calls.push(tgt.clone());
                                }
                                r.ops.push(ir::Op::Jmp { target: tgt });
                            }
                            for lbl in r.branch_labels.iter().chain(r.external_calls.iter()) {
                                if let Some(hex) = lbl.strip_prefix("L_")
                                    && hex.len() == 4
                                    && let Ok(a) = u16::from_str_radix(hex, 16)
                                {
                                    all_referenced_pcs.insert(a);
                                }
                            }
                            banked_routines.push(r)
                        }
                        Err(e) => {
                            lift_failures.push(format!("bank{bank} ${:04X}: {:?}", f.addr, e))
                        }
                    }
                }
            }
        } // end `else` (UxROM loop) of the MMC3 branch
    }

    let mut routines: Vec<ir::Routine> = Vec::with_capacity(funcs.len());
    for f in funcs.iter() {
        // Compute extra labels: PCs in our range that are referenced
        // from outside this routine but aren't its own entry point.
        let extras: Vec<u16> = all_referenced_pcs
            .iter()
            .filter(|&&pc| pc > f.addr && pc < f.end)
            .copied()
            .collect();
        let opts = ir::LiftOptions {
            start: f.addr,
            end: f.end,
            entry_name: f.name.clone(),
            jump_engine_sites: jump_engine_sites.clone(),
            return_escape_sites: return_escape_sites.clone(),
            return_consume_sites: return_consume_sites(&prof, None),
            materialized_call_sites: materialized_call_sites(&prof, None),
            window_label_prefix: None,
            extra_label_pcs: extras,
        };
        match ir::lift_range(&analysis_view, &opts) {
            Ok(mut r) => {
                // If the lifted routine doesn't end with a terminator,
                // it's a fall-through routine. Insert an explicit
                // `Op::Jmp` to the next function's L_XXXX so the
                // semantics are preserved at the harness AND in the
                // emitted SMS code. Without this, the lowered Z80 just
                // continues executing past the routine's body into
                // whatever bytes follow.
                let has_terminator = r.ops.last().is_some_and(ir::Op::is_hard_terminator);
                if !has_terminator {
                    let tail_addr = r.end;
                    let tail_has_owner = funcs
                        .iter()
                        .any(|owner| tail_addr >= owner.addr && tail_addr < owner.end);
                    if tail_has_owner {
                        let tail = format_label(tail_addr);
                        r.ops.push(ir::Op::Jmp {
                            target: tail.clone(),
                        });
                        if !r.external_calls.contains(&tail) {
                            r.external_calls.push(tail);
                        }
                    }
                }
                ir::mark_rts_dispatch(&mut r.ops);
                routines.push(r);
            }
            Err(e) => lift_failures.push(format!(
                "${:04X} {} (${:04X}..${:04X}): {:?}",
                f.addr, f.name, f.addr, f.end, e
            )),
        }
    }

    routines.extend(banked_routines);

    // 4d. Static WRAM code blobs (mapper 4): lift declared entries from
    // CHR-backed bytes so JSRs into $6000-$7FFF dispatch to translated
    // routines instead of unresolved strict-trap stubs. Their `L_XXXX`
    // auto-labels are emitted alongside the `L_w_XXXX` names, so existing
    // unannotated JSR targets resolve directly.
    if mmc3 && !prof.wram_blobs.is_empty() {
        let (wram_routines, wram_failures) = lift_wram_blobs(image.chr, &prof)?;
        lift_failures.extend(wram_failures);
        routines.extend(wram_routines);
    }

    for routine in &routines {
        let bank = profile_target_identity(&routine.name).and_then(|(bank, _)| bank);
        for target in routine
            .branch_labels
            .iter()
            .chain(routine.external_calls.iter())
        {
            if let Some((target_bank, addr)) = consume_target_identity(&prof, target) {
                consume_entries.push((target_bank.or(bank), addr));
            }
        }
    }
    check_consume_entries(&prof, &consume_entries)?;

    // Unlike ordinary unsupported routines, a profiled early ownership
    // transfer cannot degrade to a missing stub while its later JMP remains.
    let mut consume_owners = std::collections::HashSet::new();
    for site in prof
        .return_escapes
        .iter()
        .filter(|site| site.stack_bytes_already_consumed)
    {
        let start = site.consume_at.expect("profile validates consume_at");
        let owners: Vec<_> =
            routines
                .iter()
                .filter(|routine| {
                    let matching_bank = match site.bank {
                        Some(bank) => routine.name.starts_with(&format!("L_b{bank}_")),
                        None => !routine.name.starts_with("L_b"),
                    };
                    matching_bank && routine.ops.windows(2).any(|ops| {
                        matches!(
                            (&ops[0], &ops[1]),
                            (ir::Op::Source { pc, .. }, ir::Op::ReturnEscapeConsume { return_addr })
                                if *pc == start && *return_addr == site.return_addr
                        )
                    }) && routine.ops.iter().any(|op| {
                        matches!(op,
                ir::Op::Source { pc, .. } if *pc == site.caller)
                    })
                })
                .collect();
        if owners.len() != 1 {
            return Err(Error::Diagnostic(format!(
                "return_escape consume_at ${start:04X} in bank {:?} must have exactly one fully lifted owner; found {}. {}",
                site.bank,
                owners.len(),
                lift_failures.join("; ")
            )));
        }
        consume_owners.insert(owners[0].name.clone());
        if prof.replacement_for(owners[0].entry).is_some()
            || prof
                .replacements
                .iter()
                .any(|replacement| replacement.addr >= start && replacement.addr <= site.caller)
        {
            return Err(Error::Diagnostic(format!(
                "return_escape consuming owner {} cannot be replaced",
                owners[0].name
            )));
        }
    }

    // Standalone pairs have no artificial terminal edge: only the second PLA
    // cannot be entered directly. Calls and pairs must survive the complete
    // pipeline, never silently disappear into unsupported/replacement stubs.
    for site in &prof.return_consumes {
        let facts = std::iter::once((site.bank, site.at, None)).chain(
            site.calls
                .iter()
                .map(|call| (call.bank, call.caller, Some(call.target))),
        );
        for (bank, pc, call_target) in facts {
            let owners: Vec<_> = routines
                .iter()
                .filter(|routine| {
                    let identity = profile_target_identity(&routine.name).and_then(|(b, _)| b);
                    identity == bank
                        && routine.ops.windows(2).any(|ops| {
                            matches!(&ops[0], ir::Op::Source { pc: source, .. } if *source == pc)
                                && match (&ops[1], call_target) {
                                    (ir::Op::ReturnConsume { return_addrs }, None) => {
                                        *return_addrs
                                            == site
                                                .calls
                                                .iter()
                                                .map(|c| c.caller + 2)
                                                .collect::<Vec<_>>()
                                    }
                                    (
                                        ir::Op::MaterializedJsr {
                                            target,
                                            return_addr,
                                        },
                                        Some(expected),
                                    ) => {
                                        *return_addr == pc + 2
                                            && profile_target_identity(target)
                                                .is_some_and(|(_, addr)| addr == expected)
                                    }
                                    _ => false,
                                }
                        })
                })
                .collect();
            if owners.len() != 1 {
                return Err(Error::Diagnostic(format!(
                    "return_consume site ${pc:04X} in bank {bank:?} must have one fully lifted owner; found {}. {}",
                    owners.len(),
                    lift_failures.join("; ")
                )));
            }
            let owner = owners[0];
            if prof.replacement_for(owner.entry).is_some()
                || prof.replacements.iter().any(|r| {
                    r.addr >= pc && r.addr <= pc + if call_target.is_some() { 2 } else { 1 }
                })
            {
                return Err(Error::Diagnostic(format!(
                    "return_consume owner {} cannot be replaced",
                    owner.name
                )));
            }
            consume_owners.insert(owner.name.clone());
        }
    }

    // Global interior-label dedup: overlapping fixed-region translations
    // (main vs bank-view discoveries) can each emit an interior label for
    // the same NES pc. The translations cover IDENTICAL bytes, so any
    // reference may resolve to whichever copy keeps the definition —
    // strip all but the first.
    {
        // Every routine's ENTRY label (its auto label — the lifter may
        // emit it as an Op::Label when the entry is a branch target).
        let auto_of = |r: &ir::Routine| -> String {
            if r.name.starts_with("L_b") {
                r.name.clone()
            } else {
                format_label(r.entry)
            }
        };
        let mut defined: std::collections::HashSet<String> =
            routines.iter().map(&auto_of).collect();
        for r in routines.iter_mut() {
            let own = auto_of(r);
            r.ops.retain(|op| {
                if let ir::Op::Label(l) = op {
                    if *l == own {
                        return true; // the routine's own entry definition
                    }
                    if defined.contains(l) {
                        return false; // defined elsewhere (overlap copy)
                    }
                    defined.insert(l.clone());
                }
                true
            });
        }
    }

    // 4c. [[bank_call]] rewrites: call sites whose window target's bank is
    // annotated get retargeted to the bank-prefixed label; everything else
    // stays an unresolved strict-trap stub (see rewrite_bank_call_targets).
    rewrite_bank_call_targets(&prof, &mut routines, mmc3);

    // Phase S: profile-guided hot grouping. Routines named in the profile's
    // `[translation] hot_group` (from the measured far-transfer histogram)
    // are emitted first, in list order, so the per-frame call cluster packs
    // into the same early section(s); everything else keeps address order.
    if !prof.translation.hot_group.is_empty() {
        let rank = |r: &ir::Routine| -> usize {
            prof.translation
                .hot_group
                .iter()
                .position(|&a| a == r.entry)
                .unwrap_or(usize::MAX)
        };
        let mut hot: Vec<ir::Routine> = Vec::new();
        let mut rest: Vec<ir::Routine> = Vec::new();
        for r in routines.drain(..) {
            if rank(&r) != usize::MAX {
                hot.push(r);
            } else {
                rest.push(r);
            }
        }
        hot.sort_by_key(|r| rank(r));
        hot.extend(rest);
        routines = hot;
    }

    // Phase S: edge-weighted bank placement from a measured far-transfer
    // profile (FD_FAR_EDGES). Clusters routines connected by hot dynamic
    // edges under a conservative size estimate; ordering anchors each
    // cluster at its earliest member's original position, so routines
    // outside clusters keep full address-order locality (the two earlier
    // static/manual grouping attempts lost exactly that).
    if let Some(rel) = prof.translation.edge_profile.clone() {
        let path = args
            .profile
            .parent()
            .unwrap_or(std::path::Path::new("."))
            .join(&rel);
        match std::fs::read_to_string(&path) {
            Err(e) => eprintln!(
                "warning: edge_profile {} unreadable ({e}); keeping address order",
                path.display()
            ),
            Ok(text) => {
                let mut ranges: Vec<(u16, u16, usize)> = routines
                    .iter()
                    .enumerate()
                    .map(|(i, r)| (r.entry, r.end, i))
                    .collect();
                ranges.sort_unstable();
                let starts: Vec<u16> = ranges.iter().map(|&(s, _, _)| s).collect();
                let resolve = |addr: u16| -> Option<usize> {
                    let p = starts.partition_point(|&s| s <= addr);
                    let &(s, e, idx) = ranges.get(p.checked_sub(1)?)?;
                    (addr >= s && addr < e).then_some(idx)
                };
                let mut edges: std::collections::HashMap<(usize, usize), u64> = Default::default();
                for line in text.lines() {
                    let mut it = line.split_whitespace();
                    let (Some(c), Some(t), Some(n)) = (it.next(), it.next(), it.next()) else {
                        continue;
                    };
                    let (Ok(c), Ok(t), Ok(n)) = (
                        u16::from_str_radix(c, 16),
                        u16::from_str_radix(t, 16),
                        n.parse::<u64>(),
                    ) else {
                        continue;
                    };
                    if let (Some(ci), Some(ti)) = (resolve(c), resolve(t))
                        && ci != ti
                    {
                        *edges.entry((ci.min(ti), ci.max(ti))).or_default() += n;
                    }
                }
                let n = routines.len();
                let est: Vec<usize> = routines.iter().map(|r| 32 + r.ops.len() * 12).collect();
                fn uf_find(parent: &mut [usize], mut x: usize) -> usize {
                    while parent[x] != x {
                        parent[x] = parent[parent[x]];
                        x = parent[x];
                    }
                    x
                }
                let mut parent: Vec<usize> = (0..n).collect();
                let mut csize: Vec<usize> = est;
                let mut sorted: Vec<((usize, usize), u64)> = edges.into_iter().collect();
                sorted.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
                let mut merged = 0usize;
                for ((i, j), _w) in sorted {
                    let (ri, rj) = (uf_find(&mut parent, i), uf_find(&mut parent, j));
                    if ri != rj && csize[ri] + csize[rj] <= TRANSLATED_SECTION_CAPACITY {
                        parent[rj] = ri;
                        csize[ri] += csize[rj];
                        merged += 1;
                    }
                }
                if merged > 0 {
                    let cluster_of: Vec<usize> = (0..n).map(|i| uf_find(&mut parent, i)).collect();
                    let mut anchor: std::collections::HashMap<usize, usize> = Default::default();
                    for i in 0..n {
                        anchor.entry(cluster_of[i]).or_insert(i);
                    }
                    let mut order: Vec<usize> = (0..n).collect();
                    order.sort_by_key(|&i| (anchor[&cluster_of[i]], i));
                    let mut slots: Vec<Option<ir::Routine>> =
                        routines.drain(..).map(Some).collect();
                    for i in order {
                        routines.push(slots[i].take().expect("placement permutation"));
                    }
                    eprintln!("edge placer: {merged} merges from the measured profile");
                }
            }
        }
    }

    // 5. Lower into Z80. Pre-declare all runtime symbols so the linker can
    //    bind them; we emit calls to them but the actual implementations
    //    live in runtime/*.s.
    // Each WLA-DX ROM bank is 16 KiB. We pin each generated_code_N
    // section to a unique bank in slot 1 ($4000-$7FFF) so the section's
    // labels resolve to logical slot-1 addresses (and `jp L_XXXX` from
    // boot/translated code lands on real translated bytes, not on the
    // in-bank offset which would alias slot 0). Banks start at 4 to
    // leave 0-3 for the runtime/boot + asset data the project emits.
    // Transactional next-fit sizing records an explicit logical section for
    // every routine. Final emission follows that frozen plan; near-call
    // downgrades only shrink bodies and never repack or relocate routines.
    let flag_reads: std::collections::HashMap<String, u8> = {
        let mut m = std::collections::HashMap::new();
        for r in &routines {
            let mask = lower::routine_incoming_flag_reads(&r.ops);
            m.insert(format_label(r.entry), mask);
            m.insert(r.name.clone(), mask);
        }
        m
    };

    let emit_translated = |section_map: &std::collections::HashMap<String, usize>,
                           routine_sections: Option<&[u32]>|
     -> Result<
        (z80_emit::Program, Vec<String>, Vec<String>, Vec<u32>),
        Error,
    > {
        let mut program = z80_emit::Program::new();
        let mut section_idx: u32 = 0;
        begin_translated_section(&mut program, section_idx, banked)?;
        program.prepopulate_label_section(section_map);
        let opts = LowerOptions {
            profile: Some(&prof),
            emit_source_comments: true,
            routine_flag_reads: Some(&flag_reads),
        };

        emit_translated_vector_aliases(&mut program, vectors.reset, vectors.nmi, vectors.irq);

        let mut lower_failures: Vec<String> = Vec::new();
        let mut defined_labels: std::collections::BTreeSet<String> =
            std::collections::BTreeSet::new();
        defined_labels.insert("translated_reset".to_string());
        defined_labels.insert("translated_nmi".to_string());
        defined_labels.insert("translated_irq".to_string());

        if std::env::var("N2S_DEBUG_RNAMES").is_ok() {
            let mut names: std::collections::HashMap<&str, usize> = Default::default();
            for r in &routines {
                *names.entry(r.name.as_str()).or_insert(0) += 1;
            }
            for (n, c) in names.iter().filter(|(_, c)| **c > 1) {
                eprintln!("ROUTINE NAME x{c}: {n}");
            }
        }
        let emit_routine = |program: &mut z80_emit::Program,
                            defined_labels: &mut std::collections::BTreeSet<String>,
                            lower_failures: &mut Vec<String>,
                            r: &ir::Routine|
         -> Result<(), Error> {
            let before = lower_failures.len();
            emit_translated_routine(program, defined_labels, lower_failures, r, &opts)?;
            if consume_owners.contains(&r.name) && lower_failures.len() != before {
                return Err(Error::Diagnostic(format!(
                    "return_escape/return_consume consuming owner {} failed lowering: {}",
                    r.name,
                    lower_failures[before..].join("; ")
                )));
            }
            Ok(())
        };

        let mut assigned_sections = Vec::with_capacity(routines.len());
        for (routine_index, r) in routines.iter().enumerate() {
            if let Some(plan) = routine_sections {
                let planned = *plan.get(routine_index).ok_or_else(|| {
                    Error::Diagnostic("frozen layout is missing a routine assignment".to_string())
                })?;
                if planned < section_idx || planned > section_idx + 1 {
                    return Err(Error::Diagnostic(format!(
                        "frozen layout has impossible section {planned} for {} (current {section_idx})",
                        r.name
                    )));
                }
                if planned > section_idx {
                    begin_translated_section(&mut program, planned, banked)?;
                    section_idx = planned;
                }
                let mut candidate = program.clone();
                let mut candidate_labels = defined_labels.clone();
                let mut candidate_failures = lower_failures.clone();
                emit_routine(
                    &mut candidate,
                    &mut candidate_labels,
                    &mut candidate_failures,
                    r,
                )?;
                let used = translated_section_usage(&candidate);
                if used > TRANSLATED_SECTION_CAPACITY {
                    return Err(Error::Diagnostic(format!(
                        "frozen layout overflow for {} in section {planned}: {used} bytes",
                        r.name
                    )));
                }
                program = candidate;
                defined_labels = candidate_labels;
                lower_failures = candidate_failures;
                assigned_sections.push(planned);
            } else {
                let mut state = (defined_labels, lower_failures);
                let assigned = pack_sizing_candidate(
                    &mut program,
                    &mut state,
                    &mut section_idx,
                    banked,
                    &r.name,
                    |candidate, state| emit_routine(candidate, &mut state.0, &mut state.1, r),
                )?;
                defined_labels = state.0;
                lower_failures = state.1;
                assigned_sections.push(assigned);
            }
        }

        // Pre-declare runtime symbols (real bodies live in runtime/*.s;
        // placeholders keep z80_emit patch resolution happy). Profile
        // replacement targets are runtime labels too.
        program.section("runtime_forward_decls");
        for sym in RUNTIME_SYMBOLS {
            program.label(sym);
            program.ret();
        }
        // Multiple replacements may share one hook (e.g. temporary
        // diagnostic noops); declare each hook once to keep WLA-DX happy.
        let mut decl_hooks: std::collections::HashSet<&str> = std::collections::HashSet::new();
        for rep in &prof.replacements {
            if RUNTIME_SYMBOLS.contains(&rep.runtime_label.as_str()) {
                continue; // already forward-declared above
            }
            if decl_hooks.insert(rep.runtime_label.as_str()) {
                program.label(&rep.runtime_label);
                program.ret();
            }
        }

        // External-call stubs: any label still referenced but not defined
        // traps loudly via rt_unresolved_jsr (strict default).
        let unresolved: Vec<String> = program.unresolved_labels();
        program.section("unresolved_stubs");
        for (idx, ext) in unresolved.iter().enumerate() {
            program.label(ext);
            if args.debug_unresolved_stubs {
                // Debug-only visual-progress mode: no-op that advances the
                // ScreenRoutines sub-task counter (NES $073C / SMS $C73C).
                program.ld_hl_imm(0xC73C);
                program.call("rt_inc_mem");
                program.ret();
            } else if let Some(addr) = ext
                .strip_prefix("L_")
                .map(|h| h.rsplit('_').next().unwrap_or(h))
                .and_then(|h| u16::from_str_radix(h, 16).ok())
                .filter(|a| (0x8000..0xC000).contains(a) && banked && !mmc3)
            {
                // Switchable-window target (UxROM only): the correct
                // translation depends on the bank mapped AT CALL TIME. Route
                // through the runtime (bank, addr) dispatch table — never
                // hard-bind. MMC3 has two independent windows sharing one
                // live-bank shadow, so no live dispatch exists: unannotated
                // window calls trap below (fail closed) until a [[bank_call]]
                // binds them.
                program.ld_bc_imm(addr);
                program.jp("rt_banked_dispatch");
            } else {
                let id = idx as u16;
                program.ld_a_imm((id & 0x00FF) as u8);
                program.ld_abs_a(0xCB1B);
                program.ld_a_imm((id >> 8) as u8);
                program.ld_abs_a(0xCB1C);
                program.jp("rt_unresolved_jsr");
            }
        }
        // Banked-dispatch table (mapper plan M1): every translated routine
        // keyed by (NES bank, NES addr) for runtime indirect dispatch.
        // Fixed-bank routines use bank $FF (matches any window bank). Keep the
        // records address-sorted and emit a high-byte directory so the runtime
        // starts at the requested 256-byte NES page instead of linearly
        // walking every routine discovered before it.
        program.section("rt_dispatch_table_sec");
        program.label("rt_dispatch_table");
        let mut dispatch_records = routines
            .iter()
            // WRAM code routines (entry < $8000) are never runtime-dispatched:
            // they are reached only by direct JSRs, and the dispatch page
            // directory only covers $8000-$FFFF, so an entry below $8000
            // would stall the page walk (and is unreachable anyway).
            .filter(|r| r.entry >= 0x8000)
            .map(|r| match r.name.strip_prefix("L_b") {
                Some(rest) => {
                    let mut it = rest.splitn(2, '_');
                    let b: u8 = it.next().and_then(|x| x.parse().ok()).unwrap_or(0xFF);
                    let a = it
                        .next()
                        .and_then(|x| u16::from_str_radix(x, 16).ok())
                        .unwrap_or(r.entry);
                    (b, a, r.name.clone())
                }
                // Fixed-bank routines are DEFINED under their auto L_XXXX
                // label (profile display names are aliases only).
                None => (0xFF, r.entry, format_label(r.entry)),
            })
            .collect::<Vec<_>>();
        // Computed dispatch honors profile replacements too: a dispatched
        // NES address whose routine is replaced lands on the runtime hook
        // (slot 0) instead of the translated body.
        for rec in dispatch_records.iter_mut() {
            if rec.0 == 0xFF
                && let Some(rep) = prof.replacement_for(rec.1)
            {
                rec.2 = rep.runtime_label.clone();
            }
        }
        // Preserve the dispatch table's precedence at duplicate addresses:
        // fixed-bank entries historically appeared before mapper-window
        // entries and therefore win the runtime's first-match search.
        dispatch_records.sort_by_key(|(bank, addr, _)| {
            (
                *addr,
                if *bank == 0xFF {
                    0u16
                } else {
                    *bank as u16 + 1
                },
            )
        });
        let mut next_record = 0usize;
        for page in 0x80u16..=0xFF {
            program.label(format!("rt_dispatch_page_{page:02X}"));
            while next_record < dispatch_records.len()
                && (dispatch_records[next_record].1 >> 8) == page
            {
                let (bank, addr, label) = &dispatch_records[next_record];
                program.dispatch_entry(*addr, *bank, label);
                next_record += 1;
            }
        }
        program.data(None, &[0x00, 0x00]); // terminator: addr $0000
        program.label("rt_dispatch_page_table");
        for page in 0x80u16..=0xFF {
            program.word_label(&format!("rt_dispatch_page_{page:02X}"));
        }

        Ok((program, lower_failures, unresolved, assigned_sections))
    };

    // Sizing uses pessimistic far forms and records logical section IDs;
    // final emission consumes those IDs exactly while near forms only shrink.
    let empty_map: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
    let (sizing_prog, _f, _u, routine_sections) = emit_translated(&empty_map, None)?;
    let section_map = sizing_prog.label_section_snapshot();
    if std::env::var("N2S_DEBUG_SECTIONS").is_ok() {
        for l in ["translated_reset", "L_8000", "L_800F", "L_8220", "L_9000"] {
            eprintln!("map[{l}] = {:?}", section_map.get(l));
        }
    }
    let (program, lower_failures, unresolved, _) =
        emit_translated(&section_map, Some(&routine_sections))?;

    let mut build = program.finish()?;
    // Post-process the asm listing for WLA-DX:
    // 1. Strip the runtime forward-decl section. The bytes still contain
    //    harmless `ret` placeholders for in-Rust patch resolution, but
    //    WLA-DX picks up the real bodies from runtime/*.s and would
    //    otherwise error on duplicate labels.
    // 2. Strip `.org` lines that appear inside `.section` blocks. WLA-DX
    //    rejects `.org` inside sectioned code; placement is driven by
    //    the bank map. z80_emit emits `.org` for internal label-patch
    //    addressing, which is correct for the byte buffer but not for
    //    WLA-DX text output.
    // 3. Strip `.bank N slot S` directives for translated sections that
    //    WLA-DX has trouble placing because of size; fall back to
    //    `superfree` placement. For now the directives stay enabled —
    //    re-enable this strip if linker overflows happen.
    build.asm = strip_section(&build.asm, "runtime_forward_decls");
    build.asm = strip_inline_org(&build.asm);

    // 8. Convert assets (CHR + a default palette + nametable placeholder).
    // CHR-RAM carts (chr_kib = 0) ship no pattern data: build the asset
    // set from an all-zero 8 KiB CHR (blank tiles, identity maps). The
    // runtime $2007 pattern-write conversion fills real tiles in play.
    let chr_ram_blank;
    let chr_source: &[u8] = if image.chr.is_empty() {
        chr_ram_blank = vec![0u8; 8192];
        &chr_ram_blank
    } else {
        image.chr
    };
    let (chr_4bpp, chr_maps, chr_report) = if mmc3 {
        // MMC3 data_chr is the power-on visible set (first 8 KiB CHR):
        // boot uploads it and the variant generator's table-0/1 identity
        // maps describe it. Runtime switches read per-bank group assets.
        build_chr_assets(&chr_source[..chr_source.len().min(8192)], &prof.chr_packs)?
    } else {
        build_chr_assets(chr_source, &prof.chr_packs)?
    };
    let palette: [u8; 32] = default_palette();
    // Default name table: all zeros. Real rendering comes from translated
    // PPU $2006/$2007 writes during init/NMI. (Switch this to a tile-
    // index pattern for visual verification of the CHR-upload path.)
    // MMC3 builds have no static nametable: every cell is built at runtime,
    // and boot skips the upload when DATA_NAMETABLE is undefined.
    let nametable = if mmc3 {
        None
    } else {
        Some(vec![0u8; 32 * 28 * 2])
    };
    // Mirror the lower PRG window into a dedicated SMS slot-2 bank. The
    // translated SMB code reads data tables such as $805A/$806D/$8080 via raw
    // slot-2 addresses; without this bank those reads hit CHR/nametable assets.
    // Banked mappers (M1): every switchable 16 KiB NES bank becomes its
    // own SMS data bank; the fixed LAST bank is prg_high. NROM keeps the
    // flat low/high split.
    let (prg_low, prg_banks) = if banked && !mmc3 {
        let banks: Vec<Vec<u8>> = (0..policy.bank_count())
            .map(|bank| policy.prg_bank(image.prg, bank).map(|bytes| bytes.to_vec()))
            .collect::<Result<_, _>>()?;
        (None, Some(banks))
    } else {
        (
            if mmc3 {
                None
            } else {
                Some(policy.lower_prg(image.prg).to_vec())
            },
            None,
        )
    };
    // MMC3 data banks: PRG as 16 KiB pairs (halves 2k, 2k+1) and converted
    // CHR in groups of eight 1 KiB banks. See sms_project MMC3 layout.
    let (mmc3_prg_pairs, mmc3_chr_groups) = if mmc3 {
        let halves: u8 = policy.bank_count();
        let mut pairs = Vec::with_capacity(halves as usize / 2);
        for k in 0..halves / 2 {
            let mut pair = policy.prg_bank(image.prg, 2 * k)?.to_vec();
            pair.extend_from_slice(policy.prg_bank(image.prg, 2 * k + 1)?);
            pairs.push(pair);
        }
        let blobs = assets::mmc3_chr_banks_to_sms_4bpp(image.chr)
            .map_err(|err| Error::Diagnostic(format!("MMC3 CHR bank conversion failed: {err}")))?;
        let mut groups = Vec::with_capacity(blobs.len().div_ceil(8));
        for chunk in blobs.chunks(8) {
            let mut group = Vec::with_capacity(0x4000);
            for blob in chunk {
                group.extend_from_slice(blob);
            }
            groups.push(group);
        }
        (Some(pairs), Some(groups))
    } else {
        (None, None)
    };
    // Mirror the fixed upper PRG window as well. The translated code can run
    // from generated banks in slot 1, so original fixed-bank data tables such
    // as SMB's Bitmasks at $C68A are read via a slot-2 runtime helper.
    let prg_high = Some(policy.fixed_prg(image.prg).to_vec());
    // Preserve raw NES CHR bytes for emulated PPUDATA reads. SMB's
    // DrawTitleScreen copies a command stream from PPU pattern-table space
    // ($1EC0+) through $2007; the converted SMS 4bpp tiles are not suitable
    // for that CPU-visible readback path. MMC3 ships the FULL CHR image so
    // the banked pattern reader (_ppu_r_ppudata) can resolve the R0-R5 1 KiB
    // CHR bank and return the same byte the reference bus would.
    let chr_nes = Some(if image.chr.is_empty() {
        vec![0u8; 8192]
    } else {
        image.chr.to_vec()
    });

    let project_assets = ProjectAssets {
        chr_4bpp,
        palette,
        nametable,
        prg_low,
        prg_banks,
        mmc3_prg_pairs,
        mmc3_chr_groups,
        prg_high,
        chr_nes,
        chr_maps: Some(chr_maps),
        wram_blobs: if mmc3 {
            prof.wram_blobs
                .iter()
                .map(|blob| {
                    let len = usize::from(blob.length);
                    let src = usize::try_from(blob.source).unwrap_or(0);
                    sms_project::WramBlobAsset {
                        dest: blob.dest,
                        bytes: image.chr.get(src..src + len).unwrap_or_default().to_vec(),
                    }
                })
                .collect()
        } else {
            Vec::new()
        },
    };

    // 9. Emit the WLA-DX project.
    std::fs::create_dir_all(&args.out)?;
    let runtime_dir = args.runtime.as_deref().map(Path::new);
    // ROM size policy: at least PRG (32 KiB) + CHR (16 KiB SMS, doubled
    // from 8 KiB NES) + runtime + translated. The conservative lower
    // expands SMB's 32 KiB PRG into ~150 KiB of Z80; size the ROM so
    // WLA-DX's linker has room. 304 KiB is comfortable for SMB-sized
    // games plus the PRG/CHR data banks; larger NES titles would need more.
    let mirroring = match image.header.mirroring {
        nes_rom::Mirroring::Horizontal => NesMirroring::Horizontal,
        nes_rom::Mirroring::Vertical => NesMirroring::Vertical,
        nes_rom::Mirroring::FourScreen => {
            return Err(Error::Diagnostic(
                "unsupported NES four-screen nametable mirroring".into(),
            ));
        }
    };
    let cfg = ProjectConfig {
        // 512 KiB for everything: NROM translated uses banks 4-23;
        // banked carts use translated 4-16 + PRG data 17-24 + assets
        // 25-31 (1 MiB ROMs rendered black on real emulators).
        // MMC3 base 1 MiB holds translated 4-20, PRG pairs 21-36, CHR
        // groups 37-52, and the 8 packed asset banks 53-60. The full raw
        // CHR for the banked $2007 pattern reader needs ceil(chr/16KiB)
        // banks at 61+; the base image already leaves 3 spare banks
        // (61-63). trace_sms verifies it; real-emulator 1 MiB support is a
        // known follow-up.
        rom_kib: if mmc3 {
            let chr_banks = image.chr.len().div_ceil(0x4000);
            1024 + (chr_banks.saturating_sub(3) * 16) as u32
        } else {
            512
        },
        region: 0x4C,
        title: truncate_title(&prof.rom.name),
        mirroring,
        raw_ciram_backend: raw_ciram_backend_for(image.header.has_battery),
        mapper: prof.rom.mapper,
        uxrom_bank_count: (policy.is_banked() && !mmc3).then_some(policy.bank_count()),
        uxrom_bus_conflicts: (!mmc3)
            .then(|| policy.uxrom_bus_conflicts())
            .flatten()
            .map(|mode| match mode {
                nes_rom::UxromBusConflicts::None => sms_project::UxromBusConflicts::None,
                nes_rom::UxromBusConflicts::And => sms_project::UxromBusConflicts::And,
            }),
        mmc3_prg_half_count: mmc3.then_some(policy.bank_count()),
        mmc3_chr_count: mmc3.then(|| (image.chr.len() / 1024) as u16),
        chr_ram: image.chr.is_empty(),
        input_action: prof.input.mode == profile::InputMode::Action,
        input_pause_start: prof.input.pause_start,
        scroll_split: prof.render.scroll_split,
        top_tile_remap_rows: prof.render.top_tile_remap_rows,
        top_tile_remap_from: prof.render.top_tile_remap_from.clone(),
        top_tile_remap_to: prof.render.top_tile_remap_to,
        chr_ram_bg_identity: prof.render.chr_ram_bg_identity,
        native_calls: prof.native_calls(),
        runtime_defines: prof.effective_runtime_defines(),
    };
    sms_project::emit_project(&args.out, &build, &project_assets, &cfg, runtime_dir)?;

    // Phase S3.1 — Tier-3 relayout closure analysis (report only, no
    // codegen). For every RAM address reached by an indexed access, record
    // how the program touches it; a candidate array is transposable only
    // if every access inside its span is index-register-relative to its
    // own base and no dynamic pointer ((zp),Y / (zp,X)) can alias RAM at
    // all without further value analysis. See docs/speed-recovery-plan.md.
    {
        use ir::{AddrExpr, MemRegion, Op};
        use std::collections::BTreeMap;
        let mut idx_bases: BTreeMap<u16, (u32, u32)> = BTreeMap::new(); // base -> (x_count, y_count)
        let mut const_hits: BTreeMap<u16, u32> = BTreeMap::new();
        let mut zp_indexed: BTreeMap<u8, u32> = BTreeMap::new();
        let mut ind_ptrs: BTreeMap<u8, u32> = BTreeMap::new();
        let ram_region = |r: MemRegion| {
            matches!(
                r,
                MemRegion::Ram | MemRegion::RamMirror | MemRegion::ZeroPage
            )
        };
        for r in &routines {
            for op in &r.ops {
                let acc: Option<(&AddrExpr, MemRegion)> = match op {
                    Op::LdaMem { addr, region }
                    | Op::LdxMem { addr, region }
                    | Op::LdyMem { addr, region }
                    | Op::StaMem { addr, region }
                    | Op::StxMem { addr, region }
                    | Op::StyMem { addr, region }
                    | Op::AdcMem { addr, region }
                    | Op::SbcMem { addr, region }
                    | Op::CmpMem { addr, region }
                    | Op::CpxMem { addr, region }
                    | Op::CpyMem { addr, region }
                    | Op::AndMem { addr, region }
                    | Op::OraMem { addr, region }
                    | Op::EorMem { addr, region }
                    | Op::BitMem { addr, region }
                    | Op::IncMem { addr, region }
                    | Op::DecMem { addr, region }
                    | Op::AslMem { addr, region }
                    | Op::LsrMem { addr, region }
                    | Op::RolMem { addr, region }
                    | Op::RorMem { addr, region }
                    | Op::SaxMem { addr, region } => Some((addr, *region)),
                    _ => None,
                };
                let Some((addr, region)) = acc else { continue };
                match addr {
                    AddrExpr::AbsIndexedX(b) if ram_region(region) => {
                        idx_bases.entry(*b).or_default().0 += 1;
                    }
                    AddrExpr::AbsIndexedY(b) if ram_region(region) => {
                        idx_bases.entry(*b).or_default().1 += 1;
                    }
                    AddrExpr::Const(a) if ram_region(region) => {
                        *const_hits.entry(*a).or_default() += 1;
                    }
                    AddrExpr::ZpConst(z) => {
                        *const_hits.entry(*z as u16).or_default() += 1;
                    }
                    AddrExpr::ZpIndexedX(z) | AddrExpr::ZpIndexedY(z) => {
                        *zp_indexed.entry(*z).or_default() += 1;
                    }
                    AddrExpr::IndirectX(z) | AddrExpr::IndirectY(z) => {
                        *ind_ptrs.entry(*z).or_default() += 1;
                    }
                    _ => {}
                }
            }
        }
        let mut txt = String::new();
        txt.push_str(
            "Tier-3 relayout closure analysis (S3.1)\n\
             ========================================\n\
             A candidate parallel array [base .. next_base) is transposable only\n\
             when every access in its span is `base,X`/`base,Y` with the SAME\n\
             base and index meaning, no bare Const access lands inside the span\n\
             (or each such access is individually relocatable), and no dynamic\n\
             pointer can alias it. Dynamic pointers below alias ALL of RAM\n\
             absent value analysis, so any nonzero pointer-access count keeps\n\
             whole-program relayout in research territory.\n\n",
        );
        txt.push_str(&format!(
            "dynamic-pointer accesses ((zp),Y / (zp,X)): {} sites across {} zero-page pointers\n",
            ind_ptrs.values().sum::<u32>(),
            ind_ptrs.len()
        ));
        for (zp, n) in &ind_ptrs {
            txt.push_str(&format!("  ptr zp ${zp:02X}: {n} sites\n"));
        }
        txt.push_str(&format!(
            "\nzp,X / zp,Y indexed sites: {} (zero-page relayout candidates share these)\n\n",
            zp_indexed.values().sum::<u32>()
        ));
        txt.push_str("indexed bases (span = to next observed base):\n");
        let bases: Vec<u16> = idx_bases.keys().copied().collect();
        for (bi, base) in bases.iter().enumerate() {
            let (xs, ys) = idx_bases[base];
            let span_end = bases
                .get(bi + 1)
                .copied()
                .unwrap_or_else(|| base.saturating_add(0x100).min(0x0800))
                .max(base.saturating_add(1));
            let aliased: Vec<String> = const_hits
                .range(base + 1..span_end)
                .map(|(a, n)| format!("${a:04X}x{n}"))
                .collect();
            txt.push_str(&format!(
                "  ${base:04X} span ${:04X}: {xs:>3} ,X  {ys:>3} ,Y  {}\n",
                span_end,
                if aliased.is_empty() {
                    "CLEAN".to_string()
                } else {
                    format!("ALIASED by const: {}", aliased.join(" "))
                }
            ));
        }
        let clean = bases
            .iter()
            .enumerate()
            .filter(|(bi, base)| {
                let span_end = bases
                    .get(bi + 1)
                    .copied()
                    .unwrap_or_else(|| base.saturating_add(0x100).min(0x0800))
                    .max(base.saturating_add(1));
                const_hits.range(**base + 1..span_end).next().is_none()
            })
            .count();
        txt.push_str(&format!(
            "\nsummary: {} indexed bases, {clean} with const-clean spans, {} dynamic-pointer sites\n",
            bases.len(),
            ind_ptrs.values().sum::<u32>()
        ));
        let reports_dir = args.out.join("reports");
        std::fs::create_dir_all(&reports_dir)?;
        std::fs::write(reports_dir.join("relayout.txt"), &txt)?;
    }

    // 10. Reports.
    let reports_dir = args.out.join("reports");
    std::fs::create_dir_all(&reports_dir)?;
    std::fs::write(reports_dir.join("discovery.txt"), &analyzed.report.text)?;
    // B.6 coverage report: contiguous unknown PRG ranges with a hex peek,
    // the attack list for closing classification coverage.
    {
        let mut txt = String::new();
        let (code, data, unknown) = analyzed.class_map.summary();
        let total = code + data + unknown;
        txt.push_str(&format!(
            "PRG classification: {code} code, {data} data, {unknown} unknown of {total}              ({:.1}% covered)\n\nUnknown ranges (NES addr, len, first bytes):\n",
            (code + data) as f64 * 100.0 / total as f64
        ));
        let mut i = 0usize;
        while i < total {
            if analyzed.class_map.class_at(i) == analysis::ByteClass::Unknown {
                let start = i;
                while i < total && analyzed.class_map.class_at(i) == analysis::ByteClass::Unknown {
                    i += 1;
                }
                let nes = 0x8000 + start;
                let peek: String = analysis_view[start..(start + 16).min(i)]
                    .iter()
                    .map(|b| format!("{b:02X} "))
                    .collect();
                txt.push_str(&format!("  ${nes:04X}  {:5}  {peek}\n", i - start));
            } else {
                i += 1;
            }
        }
        std::fs::write(reports_dir.join("coverage.txt"), txt)?;
    }
    std::fs::write(reports_dir.join("lifted.txt"), lifted_report(&routines))?;
    std::fs::write(reports_dir.join("chr_map.txt"), chr_report)?;
    write_optional_report(
        &reports_dir.join("lift_failures.txt"),
        (!lift_failures.is_empty()).then(|| lift_failures.join("\n")),
    )?;
    write_optional_report(
        &reports_dir.join("lower_failures.txt"),
        (!lower_failures.is_empty()).then(|| lower_failures.join("\n")),
    )?;
    let unresolved_report = if !unresolved.is_empty() {
        let mut report = String::new();
        for (idx, label) in unresolved.iter().enumerate() {
            report.push_str(&format!("{idx:04X}  {label}\n"));
        }
        Some(report)
    } else {
        None
    };
    write_optional_report(
        &reports_dir.join("unresolved_labels.txt"),
        unresolved_report,
    )?;

    // 11. Optional differential validation.
    let mut validation_summary = String::new();
    if args.validate {
        // Per-routine validation. (`validate_program` lowers everything
        // together but exceeds the 64 KiB Z80 address space for large
        // games; revisit when we add Z80-side banking to the harness.)
        let mut results: Vec<validation::ValidationResult> = Vec::with_capacity(routines.len());
        for r in &routines {
            results.push(validation::validate_routine(
                image.prg,
                r,
                args.validate_vectors,
            ));
        }
        let report = validation::format_report(&results);
        std::fs::write(reports_dir.join("validation.txt"), &report)?;
        let green = results.iter().filter(|r| r.is_green()).count();
        let skipped = results
            .iter()
            .filter(|r| r.skipped_reason.is_some())
            .count();
        let red = results.len() - green - skipped;
        validation_summary = format!(
            "\nvalidation: {green} green / {red} red / {skipped} skipped (of {})\n\
             validation report: {}",
            results.len(),
            reports_dir.join("validation.txt").display(),
        );
        // Don't fail the build on validation red — the project tree was
        // emitted successfully and the report captures the failures.
        // (Phase A.3+ triages these; for now we surface them.)
    }

    let unresolved_mode = if args.debug_unresolved_stubs {
        "debug stubs"
    } else {
        "strict traps"
    };

    Ok(format!(
        "wrote SMS project to {}\n\
         functions: {} ({} lifted, {} lift failures, {} lower failures)\n\
         unresolved labels: {} ({})\n\
         code bytes: {}, data bytes: {}, unknown bytes: {}\n\
         translated.asm size: {} bytes{}",
        args.out.display(),
        analyzed.functions.functions.len(),
        routines.len(),
        lift_failures.len(),
        lower_failures.len(),
        unresolved.len(),
        unresolved_mode,
        analyzed.class_map.summary().0,
        analyzed.class_map.summary().1,
        analyzed.class_map.summary().2,
        build.bytes.len(),
        validation_summary,
    ))
}

fn format_label(addr: u16) -> String {
    format!("L_{addr:04X}")
}

fn build_chr_assets(
    chr: &[u8],
    packs: &[profile::ChrPackRange],
) -> Result<(Vec<u8>, Vec<u8>, String), Error> {
    if packs.is_empty() {
        let chr_4bpp = assets::nes_chr_to_sms_4bpp(chr);
        let mut physical = [[None; 256]; 2];
        for (tile, slot) in physical[0].iter_mut().enumerate() {
            *slot = Some(tile as u16);
        }
        for (tile, slot) in physical[1].iter_mut().take(192).enumerate() {
            *slot = Some(256 + tile as u16);
        }
        let (chr_maps, unmapped, fallbacks) = build_chr_maps(&physical, &chr_4bpp)
            .map_err(|err| Error::Diagnostic(format!("CHR map generation failed: {err}")))?;
        let report = chr_pack_report(chr, packs, &physical, chr_4bpp.len(), unmapped);
        let report = append_sprite_fallback_report(report, fallbacks);
        return Ok((chr_4bpp, chr_maps, report));
    }

    let mut chr_4bpp = vec![0u8; 448 * 32];
    let mut physical = [[None; 256]; 2];
    for range in packs {
        for (offset, tile) in (range.start..=range.end).enumerate() {
            let source_tile = usize::from(range.table) * 256 + usize::from(tile);
            let dest_slot = usize::from(range.dest) + offset;
            copy_converted_chr_tile(
                chr,
                source_tile,
                &mut chr_4bpp[dest_slot * 32..dest_slot * 32 + 32],
            );
            physical[usize::from(range.table)][usize::from(tile)] = Some(dest_slot as u16);
        }
    }
    let (chr_maps, unmapped, fallbacks) = build_chr_maps(&physical, &chr_4bpp)
        .map_err(|err| Error::Diagnostic(format!("CHR map generation failed: {err}")))?;
    let report = chr_pack_report(chr, packs, &physical, chr_4bpp.len(), unmapped);
    let report = append_sprite_fallback_report(report, fallbacks);
    Ok((chr_4bpp, chr_maps, report))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct SpriteFallbacks {
    base_2000_rel: Option<u8>,
    base_0000_rel: Option<u8>,
}

fn append_sprite_fallback_report(mut report: String, fallbacks: SpriteFallbacks) -> String {
    report.push_str(&format!(
        "sprite fallback tile bytes: base $2000 -> {}, base $0000 -> {}\n",
        format_optional_tile(fallbacks.base_2000_rel),
        format_optional_tile(fallbacks.base_0000_rel)
    ));
    report
}

fn format_optional_tile(tile: Option<u8>) -> String {
    tile.map(|tile| format!("${tile:02X}"))
        .unwrap_or_else(|| "not needed".to_string())
}

fn chr_pack_report(
    chr: &[u8],
    packs: &[profile::ChrPackRange],
    physical: &[[Option<u16>; 256]; 2],
    chr_4bpp_len: usize,
    map_fallback_entries: usize,
) -> String {
    let mut s = String::new();
    s.push_str("CHR packing report\n\n");
    s.push_str(&format!("source NES CHR bytes: {}\n", chr.len()));
    s.push_str(&format!("emitted SMS 4bpp bytes: {}\n", chr_4bpp_len));
    s.push_str(&format!("emitted SMS tile slots: {}\n", chr_4bpp_len / 32));
    s.push_str(&format!(
        "map fallback entries (BG + sprite maps): {}\n\n",
        map_fallback_entries
    ));

    if packs.is_empty() {
        s.push_str("mode: implicit identity/default map\n\n");
    } else {
        s.push_str("mode: profile [[chr_pack]]\n");
        for p in packs {
            s.push_str(&format!(
                "  table {} ${:02X}-${:02X} -> SMS slot ${:03X}\n",
                p.table, p.start, p.end, p.dest
            ));
        }
        s.push('\n');
    }

    for table in 0..2usize {
        let bg_unmapped = tile_runs(physical, table, |slot| slot.is_none());
        let sprite_unusable_base_0000 =
            tile_runs(physical, table, |slot| !matches!(slot, Some(0..=255)));
        let sprite_unusable_base_2000 =
            tile_runs(physical, table, |slot| !matches!(slot, Some(256..=511)));
        s.push_str(&format!(
            "table {} BG-unmapped tiles: {}\n",
            table,
            format_runs(&bg_unmapped)
        ));
        s.push_str(&format!(
            "table {} sprite-unusable tiles with VDP sprite base $0000: {}\n",
            table,
            format_runs(&sprite_unusable_base_0000)
        ));
        s.push_str(&format!(
            "table {} sprite-unusable tiles with VDP sprite base $2000: {}\n",
            table,
            format_runs(&sprite_unusable_base_2000)
        ));
    }
    s
}

fn tile_runs<F>(physical: &[[Option<u16>; 256]; 2], table: usize, pred: F) -> Vec<(u8, u8)>
where
    F: Fn(Option<u16>) -> bool,
{
    let mut runs = Vec::new();
    let mut start: Option<u8> = None;
    for tile in 0..=255u16 {
        let hit = pred(physical[table][tile as usize]);
        match (start, hit) {
            (None, true) => start = Some(tile as u8),
            (Some(s), false) => {
                runs.push((s, (tile - 1) as u8));
                start = None;
            }
            _ => {}
        }
    }
    if let Some(s) = start {
        runs.push((s, 0xff));
    }
    runs
}

fn format_runs(runs: &[(u8, u8)]) -> String {
    if runs.is_empty() {
        return "none".to_string();
    }
    runs.iter()
        .map(|(start, end)| {
            if start == end {
                format!("${start:02X}")
            } else {
                format!("${start:02X}-${end:02X}")
            }
        })
        .collect::<Vec<_>>()
        .join(", ")
}

fn build_chr_maps(
    physical: &[[Option<u16>; 256]; 2],
    chr_4bpp: &[u8],
) -> Result<(Vec<u8>, usize, SpriteFallbacks), String> {
    let mut out = Vec::with_capacity(0x600);
    let mut unmapped = 0usize;

    for table in physical.iter().take(2) {
        for slot in table.iter().take(256) {
            let slot = match slot {
                Some(slot) => *slot,
                None => {
                    unmapped += 1;
                    0
                }
            };
            out.push((slot & 0x00ff) as u8);
            out.push(((slot >> 8) & 0x0001) as u8);
        }
    }

    // Unmapped/unusable sprite tiles must resolve to a transparent tile under
    // the *active* SMS sprite base. Table 0 is used with VDP sprite base $2000,
    // so a physical slot in 256..=423 becomes a relative SAT tile byte. Table 1
    // is used with VDP sprite base $0000, so a physical slot in 0..=255 is
    // already the SAT tile byte. Keep SMB's established $2000 blank (slot 423 ->
    // relative 167) when it is transparent, but do not reuse that value for
    // base $0000 where it would point at physical slot 167.
    let needs_fallback_2000 = physical[0]
        .iter()
        .any(|slot| !matches!(slot, Some(256..=511)));
    let needs_fallback_0000 = physical[1]
        .iter()
        .any(|slot| !matches!(slot, Some(0..=255)));
    let fallback_2000 =
        if needs_fallback_2000 {
            Some(transparent_sprite_fallback_2000(chr_4bpp).ok_or_else(|| {
                "no transparent sprite fallback tile for SMS base $2000".to_string()
            })?)
        } else {
            None
        };
    let fallback_0000 =
        if needs_fallback_0000 {
            Some(transparent_sprite_fallback_0000(chr_4bpp).ok_or_else(|| {
                "no transparent sprite fallback tile for SMS base $0000".to_string()
            })?)
        } else {
            None
        };
    let fallbacks = SpriteFallbacks {
        base_2000_rel: fallback_2000,
        base_0000_rel: fallback_0000,
    };
    for (table_idx, table) in physical.iter().take(2).enumerate() {
        for slot in table.iter().take(256) {
            let value = match slot {
                Some(slot @ 0..=255) if table_idx == 1 => *slot as u8,
                Some(slot @ 256..=511) if table_idx == 0 => (*slot - 256) as u8,
                Some(_) => {
                    unmapped += 1;
                    if table_idx == 0 {
                        fallback_2000.expect("fallback required for table 0")
                    } else {
                        fallback_0000.expect("fallback required for table 1")
                    }
                }
                None => {
                    unmapped += 1;
                    if table_idx == 0 {
                        fallback_2000.expect("fallback required for table 0")
                    } else {
                        fallback_0000.expect("fallback required for table 1")
                    }
                }
            };
            out.push(value);
        }
    }

    debug_assert_eq!(out.len(), 0x600);
    Ok((out, unmapped, fallbacks))
}

fn transparent_sprite_fallback_2000(chr_4bpp: &[u8]) -> Option<u8> {
    if sms_tile_is_transparent(chr_4bpp, 423) {
        return Some(167);
    }
    (256..=423)
        .find(|slot| sms_tile_is_transparent(chr_4bpp, *slot))
        .map(|slot| (slot - 256) as u8)
}

fn transparent_sprite_fallback_0000(chr_4bpp: &[u8]) -> Option<u8> {
    (0..=255)
        .find(|slot| sms_tile_is_transparent(chr_4bpp, *slot))
        .map(|slot| slot as u8)
}

fn sms_tile_is_transparent(chr_4bpp: &[u8], slot: usize) -> bool {
    let start = slot * 32;
    let end = start + 32;
    end <= chr_4bpp.len() && chr_4bpp[start..end].iter().all(|byte| *byte == 0)
}

fn copy_converted_chr_tile(chr: &[u8], source_tile: usize, dest: &mut [u8]) {
    let nes_base = source_tile * 16;
    if nes_base + 16 > chr.len() {
        return;
    }
    for y in 0..8 {
        let row_base = y * 4;
        dest[row_base] = chr[nes_base + y];
        dest[row_base + 1] = chr[nes_base + 8 + y];
    }
}

/// Remove `.org` directives that appear inside `.section ... .ends` blocks.
/// WLA-DX rejects `.org` in sectioned code; placement comes from the
/// memory/ROM bank map instead.
fn strip_inline_org(asm: &str) -> String {
    let mut out = String::with_capacity(asm.len());
    let mut in_section = false;
    for line in asm.lines() {
        let trimmed = line.trim_start();
        if trimmed.starts_with(".section ") {
            in_section = true;
        } else if trimmed.starts_with(".ends") {
            in_section = false;
        }
        if in_section && trimmed.starts_with(".org ") {
            continue;
        }
        out.push_str(line);
        out.push('\n');
    }
    out
}

/// Remove a `.section "name" ... .ends` block from a WLA-DX assembly
/// listing. Returns the original string if the section isn't found.
fn strip_section(asm: &str, section_name: &str) -> String {
    let needle = format!(".section \"{section_name}\"");
    let Some(start) = asm.find(&needle) else {
        return asm.to_string();
    };
    let Some(end_rel) = asm[start..].find(".ends") else {
        return asm.to_string();
    };
    let end = start + end_rel + ".ends".len();
    let mut out = String::with_capacity(asm.len());
    out.push_str(&asm[..start]);
    // Eat one trailing newline after .ends if present so we don't leave
    // a blank stub line.
    let tail = &asm[end..];
    let tail = tail.strip_prefix('\n').unwrap_or(tail);
    out.push_str(tail);
    out
}

fn truncate_title(s: &str) -> &str {
    let bytes = s.as_bytes();
    let n = bytes.iter().take_while(|b| b.is_ascii()).count().min(11);
    std::str::from_utf8(&bytes[..n]).unwrap_or("nes2sms")
}

fn lifted_report(routines: &[ir::Routine]) -> String {
    let mut s = String::new();
    s.push_str(&format!("Lifted {} routines\n\n", routines.len()));
    for r in routines {
        s.push_str(&format!(
            "${:04X}  {}\n  ops: {}\n  branch labels: {:?}\n  external calls: {:?}\n  unresolved: {:?}\n",
            r.entry,
            r.name,
            r.ops.len(),
            r.branch_labels,
            r.external_calls,
            r.unresolved,
        ));
    }
    s
}

fn write_optional_report(path: &std::path::Path, content: Option<String>) -> Result<(), Error> {
    if let Some(content) = content {
        std::fs::write(path, content)?;
    } else {
        match std::fs::remove_file(path) {
            Ok(()) => {}
            Err(err) if err.kind() == std::io::ErrorKind::NotFound => {}
            Err(err) => return Err(err.into()),
        }
    }
    Ok(())
}

/// 32-byte SMS CRAM with a placeholder grayscale ramp.
fn default_palette() -> [u8; 32] {
    let mut p = [0u8; 32];
    // Background palette ramp: black, dark gray, light gray, white.
    p[0] = 0x00;
    p[1] = 0x15;
    p[2] = 0x2A;
    p[3] = 0x3F;
    // Sprite palette ramp.
    p[16] = 0x00;
    p[17] = 0x10;
    p[18] = 0x20;
    p[19] = 0x3F;
    p
}

/// Symbols defined in runtime/*.s. We forward-declare them in the
/// generated Program so internal `call` patches resolve at z80_emit
/// `finish()` time. At assemble time WLA-DX picks up the real bodies
/// from runtime/*.s and the linker reconciles.
const RUNTIME_SYMBOLS: &[&str] = &[
    "rt_set_nz_a",
    "rt_adc_a",
    "rt_sbc_a",
    "rt_cmp_a",
    "rt_cpx_a",
    "rt_cpy_a",
    "rt_push6502",
    "rt_pop6502",
    "rt_ppu_write",
    "rt_ppu_write_cont",
    "rt_ppu_read",
    "rt_oam_dma",
    "rt_apu_write",
    "rt_apu_read",
    "rt_sound_stub",
    "rt_controller_strobe",
    "rt_controller_read",
    "rt_controller_read_indexed_x",
    "rt_mapper_write",
    "rt_restore_prg_window",
    "rt_mmc3_read_window",
    "rt_mmc3_read_window_indexed",
    "rt_sram_read",
    "rt_sram_write",
    "rt_sram_read_indexed",
    "rt_sram_write_indexed",
    "rt_banked_dispatch",
    "rt_banked_tail_dispatch",
    "rt_rts_dispatch",
    "rt_translated_rts",
    "rt_translated_return_escape",
    "rt_translated_return_consume",
    "rt_translated_call_materialize",
    "rt_translated_call_gate",
    "rt_translated_tail_gate",
    "rt_far_tail",
    "rt_far_ncall",
    "rt_indirect_jmp",
    "rt_unresolved_jsr",
    "rt_unresolved_jsr_flash",
    "rt_brk",
    "rt_rti",
    "rt_asl_a",
    "rt_asl_mem",
    "rt_lsr_a",
    "rt_lsr_mem",
    "rt_rol_a",
    "rt_rol_mem",
    "rt_ror_a",
    "rt_ror_mem",
    "rt_bit_mem",
    "rt_inc_mem",
    "rt_dec_mem",
    "rt_read_indexed",
    "rt_read_prg_high",
    "rt_read_prg_high_indexed",
    "rt_write_indexed",
    "rt_read_zp_ptr_y",
    "rt_write_zp_ptr_y",
    "rt_far_jmp",
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sram_carts_disable_exram_raw_ciram_backend() {
        assert_eq!(raw_ciram_backend_for(true), RawCiramBackend::None);
        assert_eq!(raw_ciram_backend_for(false), RawCiramBackend::SramSlot2);
    }

    #[test]
    fn parses_profile_jump_target_physical_identity() {
        assert_eq!(profile_target_identity("L_E3D7"), Some((None, 0xE3D7)));
        assert_eq!(
            profile_target_identity("L_b6_A75E"),
            Some((Some(6), 0xA75E))
        );
        assert_eq!(profile_target_identity("runtime_helper"), None);
    }

    #[test]
    fn consuming_entries_resolve_aliases_and_preserve_physical_bank_identity() {
        let mut profile = profile::load_from_str("[rom]\nname=\"x\"\nmapper=2\nprg_kib=128\nchr_kib=0\n[[return_escape]]\ncaller=0x8010\ntarget=0xC100\nreturn_addr=0xC200\nbank=3\nstack_bytes_already_consumed=true\nconsume_at=0x8000\n").unwrap();
        profile.functions.push(profile::Function {
            addr: 0xc123,
            name: "Friendly".into(),
            note: None,
        });
        profile.labels.push(profile::Label {
            addr: 0x8001,
            name: "Interior".into(),
        });
        assert_eq!(
            consume_target_identity(&profile, "Friendly"),
            Some((None, 0xc123))
        );
        assert_eq!(
            consume_target_identity(&profile, "Interior"),
            Some((None, 0x8001))
        );
        assert_eq!(
            consume_target_identity(&profile, "L_b4_8001"),
            Some((Some(4), 0x8001))
        );
        assert!(
            check_consume_entries(
                &profile,
                &[(Some(4), 0x8001), (Some(3), 0x8000), (Some(3), 0x8011)]
            )
            .is_ok()
        );
        for entry in [
            (Some(3), 0x8001),
            (Some(3), 0x8003),
            (Some(3), 0x8010),
            (None, 0x8001),
        ] {
            assert!(
                check_consume_entries(&profile, &[entry]).is_err(),
                "{entry:?}"
            );
        }
        profile.return_escapes[0].consume_at = Some(0xc000);
        profile.return_escapes[0].caller = 0xc010;
        profile.return_escapes[0].bank = None;
        assert!(
            check_consume_entries(&profile, &[(Some(4), 0xc001)]).is_err(),
            "fixed window is shared"
        );
    }

    fn banked_routine(name: &str) -> ir::Routine {
        ir::Routine {
            entry: 0x94EC,
            end: 0x9500,
            name: name.to_string(),
            ops: vec![ir::Op::Jsr {
                target: "L_BF62".to_string(),
            }],
            branch_labels: vec![],
            external_calls: vec!["L_BF62".to_string()],
            unresolved: vec![],
        }
    }

    #[test]
    fn mmc3_bank_call_rewrites_cross_window_refs_in_banked_units() {
        // A banked unit (L_b20_94EC, in the LOW window) calls $BF62, which
        // lives in bank 19's HIGH window. The cross-window ref is unprefixed
        // to L_BF62 by unprefix_cross_window_labels and must be rebound to
        // the annotated bank. UxROM banked units must NOT be rebound (they
        // use the live dispatch shadow).
        let prof = profile::load_from_str(
            "[rom]\nname=\"x\"\nmapper=4\nprg_kib=256\nchr_kib=128\n[[bank_call]]\ntarget=0xbf62\nbank=19\n",
        )
        .unwrap();

        let mut mmc3_routines = vec![banked_routine("L_b20_94EC")];
        rewrite_bank_call_targets(&prof, &mut mmc3_routines, true);
        assert_eq!(
            mmc3_routines[0].ops[0],
            ir::Op::Jsr {
                target: "L_b19_BF62".to_string()
            },
            "MMC3 cross-window bank_call must bind the annotated bank"
        );
        assert_eq!(mmc3_routines[0].external_calls, vec!["L_b19_BF62"]);

        // UxROM: banked units keep the unprefixed label for live dispatch.
        let mut uxrom_prof = prof.clone();
        uxrom_prof.rom.mapper = 2;
        let mut uxrom_routines = vec![banked_routine("L_b0_94EC")];
        rewrite_bank_call_targets(&uxrom_prof, &mut uxrom_routines, false);
        assert_eq!(
            uxrom_routines[0].ops[0],
            ir::Op::Jsr {
                target: "L_BF62".to_string()
            },
            "UxROM banked units must resolve through the live dispatch shadow"
        );

        // Fixed-bank (non-L_b) routines are always rebound.
        let mut fixed_routines = vec![banked_routine("func_94EC")];
        rewrite_bank_call_targets(&prof, &mut fixed_routines, true);
        assert_eq!(
            fixed_routines[0].ops[0],
            ir::Op::Jsr {
                target: "L_b19_BF62".to_string()
            }
        );
    }

    fn physical_maps() -> [[Option<u16>; 256]; 2] {
        [[None; 256]; 2]
    }

    fn map_table_to_base_2000(physical: &mut [[Option<u16>; 256]; 2], table: usize) {
        for (tile, slot) in physical[table].iter_mut().enumerate() {
            *slot = Some(256 + tile as u16);
        }
    }

    fn map_table_to_base_0000(physical: &mut [[Option<u16>; 256]; 2], table: usize) {
        for (tile, slot) in physical[table].iter_mut().enumerate() {
            *slot = Some(tile as u16);
        }
    }

    #[test]
    fn sprite_base_2000_fallback_prefers_reserved_blank_167() {
        let mut physical = physical_maps();
        map_table_to_base_0000(&mut physical, 1);
        let chr_4bpp = vec![0u8; 448 * 32];

        let (maps, unmapped, fallbacks) = build_chr_maps(&physical, &chr_4bpp).unwrap();

        assert_eq!(maps[0x400], 167);
        assert_eq!(fallbacks.base_2000_rel, Some(167));
        assert_eq!(fallbacks.base_0000_rel, None);
        assert_eq!(unmapped, 512);
    }

    #[test]
    fn sprite_base_0000_fallback_uses_transparent_base0_tile() {
        let mut physical = physical_maps();
        map_table_to_base_2000(&mut physical, 0);
        let mut chr_4bpp = vec![0xffu8; 512 * 32];
        chr_4bpp[5 * 32..6 * 32].fill(0);

        let (maps, unmapped, fallbacks) = build_chr_maps(&physical, &chr_4bpp).unwrap();

        assert_eq!(maps[0x500], 5);
        assert_eq!(fallbacks.base_2000_rel, None);
        assert_eq!(fallbacks.base_0000_rel, Some(5));
        assert_eq!(unmapped, 512);
    }

    #[test]
    fn sprite_base_0000_fallback_fails_closed_without_transparent_tile() {
        let mut physical = physical_maps();
        map_table_to_base_2000(&mut physical, 0);
        let chr_4bpp = vec![0xffu8; 512 * 32];

        let err = build_chr_maps(&physical, &chr_4bpp).unwrap_err();

        assert!(err.contains("SMS base $0000"));
    }

    #[test]
    fn only_structured_mapper_store_lower_errors_are_fatal() {
        assert!(lower_error_is_fatal(
            &lower::LowerError::UnsupportedMapperStore {
                pc: Some(0x8000),
                reason: "invalid mapper store".to_string(),
            }
        ));
        assert!(!lower_error_is_fatal(&lower::LowerError::UnsupportedOp {
            pc: Some(0x8000),
            reason: "unrelated lowering gap".to_string(),
        }));
    }

    #[test]
    fn banked_translated_sections_stop_before_prg_data_bank() {
        let last_section = sms_project::NES_PRG_BANK_BASE - TRANSLATED_BANK_BASE - 1;
        assert_eq!(
            u32::from(translated_section_bank(last_section, true).unwrap()),
            sms_project::NES_PRG_BANK_BASE - 1
        );
        let err = translated_section_bank(last_section + 1, true).unwrap_err();
        assert!(
            err.to_string()
                .contains(&format!("section {} > {last_section}", last_section + 1))
        );
    }

    #[test]
    fn translated_section_identity_rejects_interposed_program_section() {
        let mut program = z80_emit::Program::new();
        program.section("unexpected_helper");

        let err = begin_translated_section(&mut program, 0, false).unwrap_err();
        assert!(err.to_string().contains("logical section 0"));
        assert!(err.to_string().contains("expected 1"));
    }

    #[test]
    fn vector_aliases_use_tail_gates_for_cross_section_targets() {
        let (mut program, mut section) = packing_program();
        let target_program_section = program.current_section_idx() + 1;
        program.prepopulate_label_section(&std::collections::HashMap::from([
            ("L_C000".to_string(), target_program_section),
            ("L_C100".to_string(), target_program_section),
            ("L_C200".to_string(), target_program_section),
        ]));

        emit_translated_vector_aliases(&mut program, 0xC000, 0xC100, 0xC200);
        advance_translated_section(&mut program, &mut section, false).unwrap();
        for label in ["L_C000", "L_C100", "L_C200"] {
            program.label(label);
            program.ret();
        }
        program.section("test_runtime_stubs");
        program.label("rt_translated_tail_gate");
        program.ret();

        let build = program.finish().unwrap();
        assert_eq!(build.asm.matches("jp rt_translated_tail_gate").count(), 3);
        for label in ["L_C000", "L_C100", "L_C200"] {
            assert!(!build.asm.contains(&format!("jp {label}")));
        }
    }

    fn packing_program() -> (z80_emit::Program, u32) {
        let mut program = z80_emit::Program::new();
        let section = 0;
        begin_translated_section(&mut program, section, false).unwrap();
        (program, section)
    }

    #[test]
    fn packing_replays_crossing_candidate_in_next_section() {
        let (mut program, mut section) = packing_program();
        program.data(None, &vec![0xAA; 0x3FFF]);
        let mut state = Vec::<String>::new();
        let assigned = pack_sizing_candidate(
            &mut program,
            &mut state,
            &mut section,
            false,
            "crossing_marker",
            |candidate, state| {
                candidate.label("crossing_marker");
                candidate.data(None, &[0xC1, 0xC2]);
                state.push("committed".to_string());
                Ok(())
            },
        )
        .unwrap();

        assert_eq!(assigned, 1);
        let build = program.finish().unwrap();
        let first = build
            .sections
            .iter()
            .find(|s| s.name == "generated_code_0")
            .unwrap();
        let second = build
            .sections
            .iter()
            .find(|s| s.name == "generated_code_1")
            .unwrap();
        assert_eq!(first.bytes.len(), 0x3FFF);
        assert_eq!(second.bytes, [0xC1, 0xC2]);
        assert_eq!(state, ["committed"]);
    }

    #[test]
    fn retry_relowers_section_sensitive_jsr_as_far() {
        let (mut program, mut section) = packing_program();
        let section_zero = program.current_section_idx();
        program.label("L_target");
        program.data(None, &vec![0xAA; 0x3FFF]);
        let attempts = std::cell::Cell::new(0);

        let routine = ir::Routine {
            entry: 0x8000,
            end: 0x8003,
            name: "L_8000".to_string(),
            ops: vec![
                ir::Op::Label("L_8000".to_string()),
                ir::Op::Jsr {
                    target: "L_target".to_string(),
                },
                ir::Op::Rts,
            ],
            branch_labels: vec!["L_8000".to_string()],
            external_calls: vec!["L_target".to_string()],
            unresolved: Vec::new(),
        };
        let mut state = ();
        let assigned = pack_sizing_candidate(
            &mut program,
            &mut state,
            &mut section,
            false,
            "L_8000",
            |candidate, _| {
                attempts.set(attempts.get() + 1);
                lower::lower_routine(candidate, &routine, &LowerOptions::default())
                    .map_err(Error::from)
            },
        )
        .unwrap();

        assert_eq!(assigned, 1);
        assert_eq!(section, 1);
        assert_eq!(attempts.get(), 2);
        assert_ne!(program.current_section_idx(), section_zero);
        assert_eq!(
            program.label_section_idx("L_8000"),
            Some(program.current_section_idx())
        );

        let unresolved = program.unresolved_labels();
        program.section("test_runtime_stubs");
        for label in unresolved {
            program.label(&label);
            program.ret();
        }
        let build = program.finish().unwrap();
        let first = build
            .sections
            .iter()
            .find(|s| s.name == "generated_code_0")
            .unwrap();
        let second = build
            .sections
            .iter()
            .find(|s| s.name == "generated_code_1")
            .unwrap();

        assert_eq!(first.bytes.len(), 0x3FFF);
        assert!(!second.bytes.is_empty());
        assert_eq!(build.asm.matches("L_8000:").count(), 1);
        assert!(build.asm.contains("jp rt_translated_call_gate"));
        assert!(!build.asm.contains("ld a,(hl)"));
    }

    #[test]
    fn packing_exact_full_final_section_does_not_advance() {
        let (mut program, mut section) = packing_program();
        let mut state = ();
        let assigned = pack_sizing_candidate(
            &mut program,
            &mut state,
            &mut section,
            false,
            "exact_full",
            |candidate, _| {
                candidate.data(None, &vec![0; 0x4000]);
                Ok(())
            },
        )
        .unwrap();
        assert_eq!(assigned, 0);
        assert_eq!(section, 0);
        assert_eq!(program.current_section_len(), 0x4000);
    }

    #[test]
    fn packing_empty_section_oversize_reports_exact_size() {
        let (mut program, mut section) = packing_program();
        let mut state = ();
        let err = pack_sizing_candidate(
            &mut program,
            &mut state,
            &mut section,
            false,
            "too_large",
            |candidate, _| {
                candidate.data(None, &vec![0; 0x4001]);
                Ok(())
            },
        )
        .unwrap_err();
        assert!(err.to_string().contains("too_large"));
        assert!(err.to_string().contains("16385 bytes"));
    }

    #[test]
    fn packing_uses_all_banks_before_reserved_uxrom_data() {
        let (mut program, mut section) = packing_program();
        let mut state = ();
        let section_count = sms_project::NES_PRG_BANK_BASE - TRANSLATED_BANK_BASE;
        for index in 0..section_count {
            let assigned = pack_sizing_candidate(
                &mut program,
                &mut state,
                &mut section,
                true,
                "full_section",
                |candidate, _| {
                    candidate.data(None, &vec![0; 0x4000]);
                    Ok(())
                },
            )
            .unwrap();
            assert_eq!(assigned, index);
        }
        let err = pack_sizing_candidate(
            &mut program,
            &mut state,
            &mut section,
            true,
            "first_reserved_bank",
            |candidate, _| {
                candidate.data(None, &[0]);
                Ok(())
            },
        )
        .unwrap_err();
        assert!(err.to_string().contains(&format!(
            "banks {}+ hold PRG data",
            sms_project::NES_PRG_BANK_BASE
        )));
    }

    #[test]
    fn nonfatal_lowering_rolls_back_to_only_trap_stub() {
        let (mut program, _) = packing_program();
        let routine = ir::Routine {
            entry: 0x8000,
            end: 0x8003,
            name: "profile_bad".to_string(),
            ops: vec![
                ir::Op::Label("profile_bad".to_string()),
                ir::Op::Label("L_inner".to_string()),
                ir::Op::Nop,
                ir::Op::Unsupported {
                    pc: 0x8002,
                    opcode: 0x8B,
                    mnemonic: "XAA".to_string(),
                    reason: "unstable opcode".to_string(),
                },
            ],
            branch_labels: vec!["profile_bad".to_string(), "L_inner".to_string()],
            external_calls: Vec::new(),
            unresolved: Vec::new(),
        };
        let mut labels = std::collections::BTreeSet::new();
        let mut failures = Vec::new();
        emit_translated_routine(
            &mut program,
            &mut labels,
            &mut failures,
            &routine,
            &LowerOptions::default(),
        )
        .unwrap();
        assert!(labels.contains("L_8000"));
        assert!(labels.contains("profile_bad"));
        assert!(labels.contains("L_inner"));
        program.label("rt_unresolved_jsr");
        program.ret();
        let build = program.finish().unwrap();
        assert!(failures.iter().any(|failure| failure.contains("XAA")));
        assert_eq!(build.asm.matches("L_8000:").count(), 1);
        assert_eq!(build.asm.matches("profile_bad:").count(), 1);
        assert_eq!(build.asm.matches("L_inner:").count(), 1);
        assert_eq!(build.asm.matches("ld a,$EE").count(), 1);
        assert!(build.asm.contains("ld ($CB1B),a"));
        assert!(!build.asm.contains("  nop"));
        assert_eq!(
            build
                .bytes
                .windows(5)
                .filter(|bytes| *bytes == [0x3E, 0xEE, 0x32, 0x1B, 0xCB])
                .count(),
            1
        );
        assert_eq!(&build.bytes[..5], [0x3E, 0xEE, 0x32, 0x1B, 0xCB]);
    }
}
