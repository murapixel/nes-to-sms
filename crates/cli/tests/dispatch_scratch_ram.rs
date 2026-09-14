//! Regression guard for the translated-call return stack.
//!
//! `rt_indirect_jmp` records the `JMP ($xxxx)` pointer site for trap
//! diagnostics at $CB75. The store must be a single byte: a 16-bit word
//! store at $CB75 spills its high byte into $CB76, which is the low byte
//! of `TR_RET_PTR` ($CB76/$CB77). That silently corrupted the software
//! continuation stack on every indirect JMP and desynced Mother's $C9B5
//! RTS-dispatch loop at frame 32 (the $C8D0/$CC41 writes vanished).

/// Isolate the `rt_indirect_jmp` body (up to its RAM-target trampoline).
fn indirect_jmp_body() -> String {
    let src = include_str!("../../../runtime/dispatch.s");
    let start = src
        .find("rt_indirect_jmp:")
        .expect("runtime/dispatch.s must define rt_indirect_jmp");
    let rest = &src[start..];
    let end = rest
        .find("_ij_ram_target:")
        .expect("rt_indirect_jmp must reach _ij_ram_target");
    rest[..end].to_ascii_lowercase()
}

#[test]
fn indirect_jmp_diagnostic_does_not_store_a_word_at_cb75() {
    let body = indirect_jmp_body();
    for form in ["($cb75), hl", "($cb75),hl"] {
        assert!(
            !body.contains(form),
            "rt_indirect_jmp stores a 16-bit value at $CB75 (`{form}`); its \
             high byte overwrites TR_RET_PTR's low byte ($CB76). Use a \
             single-byte store."
        );
    }
}

#[test]
fn indirect_jmp_diagnostic_keeps_byte_store_at_cb75() {
    let body = indirect_jmp_body();
    assert!(
        body.contains("($cb75), a"),
        "rt_indirect_jmp should still record the pointer low byte at $CB75"
    );
}
