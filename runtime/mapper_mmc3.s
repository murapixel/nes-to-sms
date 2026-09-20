; ─── mapper_mmc3.s ──────────────────────────────────────────────────────────
; MMC3 (mapper 4) runtime: register file, PRG/CHR windows, SRAM, scanline IRQ.
;
; Only assembled meaningfully when NES_MMC3 is defined (emitted by
; sms_project for mapper-4 builds); otherwise this file is empty.
;
; Shadow map ($CB63-$CB72 — free exactly when NES_CHR_RAM is unset; the
; CHR-RAM staging in chrmap.s/sat.s lives there under .ifdef NES_CHR_RAM,
; so combining both fails closed at assembly):
;   $CB63 MMC3_BANK_SELECT  last $8000 value (bit7 CHR mode, bit6 PRG mode,
;                           bits 2-0 R0-R7 select)
;   $CB64-$CB6B MMC3_R0-R7  bank registers (R0/R1 2 KiB, R2-R5 1 KiB,
;                           R6/R7 8 KiB PRG)
;   $CB6C MMC3_MIRROR       bit0: nametable arrangement (0 vert, 1 horiz)
;   $CB6D MMC3_IRQ_LATCH    $C000 reload value
;   $CB6E MMC3_IRQ_COUNTER  working down-counter
;   $CB6F MMC3_IRQ_CTRL     bit0 enable ($E001/$E000), bit1 reload-pending
;                           ($C001), bit2 IRQ level-pending
;   $CB70 MMC3_PRG_LOW      8 KiB half live at $8000-$9FFF
;   $CB71 MMC3_PRG_HIGH     8 KiB half live at $A000-$BFFF
;   $CB72 MMC3_CHR_DIRTY    nonzero = CHR set changed, refresh variants
; (Shadow addresses are defined in boot.s, which precedes this file, so
; every translation unit sees them. SMS data-bank bases come from sms.asm.)
;
; $CB1D trap codes introduced here: $E5 MMC3 bad address, $E6 SRAM address.
.ifdef NES_MMC3
.ifdef NES_CHR_RAM
.fail "NES_MMC3 needs $CB63-$CB72, live CHR-RAM staging under NES_CHR_RAM"
.endif

.section "mapper_mmc3" free

; ─── rt_mmc3_reset ────────────────────────────────────────────────────
; Power-on register file (matches nes_rom::Mmc3State::default, which is the
; executable spec — keep the two in lock-step). Called once from boot.
; Clobbers AF.
rt_mmc3_reset:
  xor a
  ld  (MMC3_BANK_SELECT), a
  ld  (MMC3_MIRROR), a
  ld  (MMC3_IRQ_LATCH), a
  ld  (MMC3_IRQ_COUNTER), a
  ld  (MMC3_IRQ_CTRL), a
  ld  (MMC3_CHR_DIRTY), a
  ld  a, 0
  ld  (MMC3_R6), a            ; LOW window = half 0
  ld  (MMC3_PRG_LOW), a
  ld  a, 1
  ld  (MMC3_R7), a            ; HIGH window = half 1
  ld  (MMC3_PRG_HIGH), a
  xor a
  ld  (MMC3_R0), a
  ld  a, 2
  ld  (MMC3_R1), a
  ld  a, 4
  ld  (MMC3_R2), a
  ld  a, 5
  ld  (MMC3_R3), a
  ld  a, 6
  ld  (MMC3_R4), a
  ld  a, 7
  ld  (MMC3_R5), a
  ret

; ─── rt_mmc3_write ────────────────────────────────────────────────────
; Entry: A = value written, HL = NES address ($8000-$FFFF).
; Preserves AF, DE, and shadow P ($CB03); clobbers BC/HL. Same contract as
; rt_mapper_write, which tail-jumps here for NES_MMC3 builds. The value is
; parked in C once; A stays scratch (vdp_set_register may clobber C, but
; only on paths where the value is already committed).
rt_mmc3_write:
  ld  c, a                    ; park value (BC clobbered per contract)
  push af                     ; single balanced pair: caller AF restored below
  ld  a, h
  cp  $80
  jp  c, _mmw_bad_address     ; too far for jr (whole decoder between)
  cp  $a0
  jr  c, _mmw_8000
  cp  $c0
  jr  c, _mmw_a000
  cp  $e0
  jr  c, _mmw_c000
  ; $E000-$FFFF: even = ack+disable, odd = enable.
  ld  a, l
  and $01
  jr  z, _mmw_irq_disable
  ld  a, (MMC3_IRQ_CTRL)
  or  $01
  ld  (MMC3_IRQ_CTRL), a
  ; Arm the VDP line IRQ to fire every scanline; the handler ticks the
  ; MMC3 counter in software (most faithful with unknown A12 gating).
  ld  a, $56                 ; VDP_R0_LINE_IRQ_ON
  ld  b, $00
  call vdp_set_register
  xor a
  ld  b, $0a                 ; R10 = 0: interrupt every line
  call vdp_set_register
  jr  _mmw_done
_mmw_irq_disable:
  ld  a, (MMC3_IRQ_CTRL)
  and $f8                    ; clear enable + reload + pending
  ld  (MMC3_IRQ_CTRL), a
  ; Disarm the VDP line IRQ (back to base mode; frame IRQ unaffected).
  ld  a, $46                 ; VDP_R0_BASE
  ld  b, $00
  call vdp_set_register
  jr  _mmw_done
_mmw_c000:
  ; $C000-$DFFF: even = latch, odd = reload request.
  ld  a, l
  and $01
  jr  z, _mmw_latch
  ld  a, (MMC3_IRQ_CTRL)
  or  $02
  ld  (MMC3_IRQ_CTRL), a
  jr  _mmw_done
_mmw_latch:
  ld  a, c
  ld  (MMC3_IRQ_LATCH), a
  jr  _mmw_done
_mmw_a000:
  ; $A000-$BFFF: even = mirroring bit0, odd = PRG-RAM protect (recorded
  ; nowhere: SRAM stays accessible, matching the reference bus; see notes).
  ld  a, l
  and $01
  jr  nz, _mmw_done
  ld  a, c
  and $01
  ld  (MMC3_MIRROR), a
  jr  _mmw_done
_mmw_8000:
  ; $8000-$9FFF: even = bank select (+ recompute LOW), odd = bank data.
  ; PRG mode 1 swaps the $8000/$C000 windows (LOW becomes second-last,
  ; MID becomes R6); recompute + fixed reads are mode-aware, so no trap.
  ld  a, l
  and $01
  jr  nz, _mmw_bank_data
  ld  a, c
  ld  (MMC3_BANK_SELECT), a
  call _mmw_recompute_low
  jr  _mmw_done
_mmw_bank_data:
  ; Route by the latched select bits.
  ld  a, (MMC3_BANK_SELECT)
  and $07
  cp  $06
  jr  z, _mmw_data_r6
  cp  $07
  jr  z, _mmw_data_r7
  ; R0-R5: CHR bank change -> variants go stale; the NMI presentation
  ; flush refreshes them (rt_mmc3_chr_sync). R0-R5 shadows are contiguous.
  ; The index add must not clobber DE: D/E are the resident translated
  ; X/Y and callers (e.g. fixed $CFC8's JSR $FFD0 restore loop, which keeps
  ; the slot index in X across the call) reuse them after return. A bare
  ; `ld d,$00` here zeroed X, so the loop descended $FF.. and STA $F0,X
  ; wrapped into zero page ($EB-$EF, notably $EC), which later took the
  ; NMI $F8DA BEQ-not-taken path, enabled IRQ via STX $E001, and dispatched
  ; through zeroed $0540 into the $0001 banked-dispatch miss.
  ld  b, a                    ; B = reg index 0-5
  ld  a, c                    ; A = value
  push de                     ; preserve resident X/Y across indexing
  ld  hl, MMC3_R0
  ld  d, $00
  ld  e, b
  add hl, de
  ld  (hl), a
  pop de                      ; restore resident X/Y
  ld  a, $01
  ld  (MMC3_CHR_DIRTY), a
  jr  _mmw_done
_mmw_data_r6:
  ld  a, c
  ld  (MMC3_R6), a
  call _mmw_recompute_low
  jr  _mmw_done
_mmw_data_r7:
  ld  a, c
  and NES_MMC3_PRG_MASK
  ld  (MMC3_R7), a
  ld  (MMC3_PRG_HIGH), a      ; HIGH window = R7 in both PRG modes
  ; fall through
_mmw_done:
  pop af                      ; restore caller AF
  ret
_mmw_bad_address:
  pop af                      ; balance the entry push before trapping
  ld  a, $e5
  ld  ($cb1d), a
  halt
  jr  _mmw_bad_address

; Recompute MMC3_PRG_LOW from PRG mode + R6, then map the LOW pair into
; slot 2. Slot 2 ALWAYS shows the LOW pair at op boundaries (presentation
; asserts this; the IRQ entry/exit preserves it across nesting), so helpers
; borrow it freely and restore via rt_restore_prg_window. Clobbers AF.
_mmw_recompute_low:
  ld  a, (MMC3_BANK_SELECT)
  bit 6, a
  jr  nz, _mmw_low_fixed
  ld  a, (MMC3_R6)
  and NES_MMC3_PRG_MASK
  ld  (MMC3_PRG_LOW), a
  jr  _mmw_low_map
_mmw_low_fixed:
  ld  a, NES_MMC3_PRG_COUNT-2 ; second-last half in PRG mode 1
  ld  (MMC3_PRG_LOW), a
_mmw_low_map:
  srl a                       ; pair idx (half>>1)
  add a, NES_MMC3_PRG_BASE
  ld  ($ffff), a
  ret

; ─── rt_mmc3_read_window ──────────────────────────────────────────────
; Entry: HL = NES $8000-$BFFF. Exit: A = byte. Preserves BC/DE/HL.
; Maps the pair bank holding the live LOW/HIGH half into slot 1 (pairs:
; SMS bank NES_MMC3_PRG_BASE + (half>>1) holds halves (2k, 2k+1)), reads
; at (half&1)*$2000 + (addr&$1FFF), restores the interrupted slot-1 bank
; from the $CB14 authority.
rt_mmc3_read_window:
  push hl
  push de
  push bc
  ld  a, h
  cp  $a0
  jr  c, _mrw_low
  ld  a, (MMC3_PRG_HIGH)
  jr  _mrw_map
_mrw_low:
  ld  a, (MMC3_PRG_LOW)
_mrw_map:
  ld  b, a                    ; B = half (0-31)
  srl a                       ; A = pair idx (half>>1)
  add a, NES_MMC3_PRG_BASE
  ld  c, a                    ; C = SMS pair bank (parked: caller C stacked)
  ld  a, ($cb14)
  push af                     ; park interrupted slot-1 bank
  ld  a, c
  ld  ($fffe), a              ; map data pair
  ; H = $40 | (addr-high & $1F) | ((half&1) << 5): slot-1 $4000-$7FFF
  ; over the pair's first/second half.
  ld  a, h
  and $1f
  or  $40
  ld  h, a
  ld  a, b
  and $01
  rlca
  rlca
  rlca
  rlca
  rlca                        ; bit0 -> bit5 ($00/$20)
  or  h
  ld  h, a
  ld  d, (hl)                 ; D = result (caller D is stacked)
  pop af                      ; A = interrupted bank
  ld  ($fffe), a              ; restore interrupted bank
  ld  a, ($cb14)
  ld  ($fffe), a              ; paranoia: $CB14 is the authority
  ld  a, d                    ; A = result
  pop bc                      ; caller BC
  pop de                      ; caller DE
  pop hl                      ; caller HL
  ret

; ─── rt_mmc3_read_window_indexed ──────────────────────────────────────
; Entry: HL = base, B = offset. Exit: A = (HL+B). Preserves BC/DE.
rt_mmc3_read_window_indexed:
  ld  a, l
  add a, b
  ld  l, a
  ld  a, h
  adc a, $00
  ld  h, a
  jr  rt_mmc3_read_window      ; tail: preserves contract (pushes own regs)

; ─── rt_mmc3_read_fixed ───────────────────────────────────────────────
; Entry: HL = NES $C000-$FFFF. Exit: A = byte. Preserves BC/DE/HL.
; Mode-aware fixed-window read (PRG mode 0 vs 1 swap $C000-$DFFF):
;   mode 0, or addr $E000-$FFFF: fixed-high image (:data_prg_high =
;     halves 30+31) at slot-1 $4000-$7FFF.
;   mode 1, addr $C000-$DFFF: R6 pair at slot-1 $4000-$5FFF/$6000-$7FFF.
; DI-bracketed (IFF2-preserving): an NMI landing mid-read would otherwise
; resume with the data bank still mapped (same race as rt_read_prg_high,
; which brackets the same way).
rt_mmc3_read_fixed:
  ld  a, i
  di
  jp  po, _mrf_di
  push hl
  push bc
  call _mrf_read
  pop bc
  pop hl
  ei
  ret
_mrf_di:
  push hl
  push bc
  call _mrf_read
  pop bc
  pop hl
  ret
; Body: A trashed, B/C scratch, HL live; returns byte in A; slot 1 and
; $CB14 restored. Caller (above) handles HL/BC preservation + IFF.
_mrf_read:
  ld  a, (MMC3_BANK_SELECT)
  bit 6, a
  jr  nz, _mrf_maybe_mid
_mrf_fixedhigh:
  ld  a, ($cb14)
  ld  c, a                    ; C = interrupted slot-1 bank
  ld  a, :data_prg_high
  ld  ($fffe), a
  ld  a, h
  sub $80                     ; NES $C000-$FFFF -> slot-1 $4000-$7FFF
  ld  h, a
  ld  a, (hl)
  ld  b, a                    ; park result (A needed for restore)
  ld  a, c
  ld  ($fffe), a
  ld  a, b
  ret
_mrf_maybe_mid:
  ld  a, h
  cp  $e0
  jr  nc, _mrf_fixedhigh      ; $E000+ is half 31 in both modes
  ; Mode-1 $C000-$DFFF: R6 pair.
  ld  a, (MMC3_R6)
  and NES_MMC3_PRG_MASK
  ld  b, a                    ; B = half
  srl a
  add a, NES_MMC3_PRG_BASE
  ld  c, a                    ; C = SMS pair bank (parked: caller C stacked)
  ld  a, ($cb14)
  push af                     ; park interrupted bank
  ld  a, c
  ld  ($fffe), a
  ; H = $40 | (addr-high & $1F) | ((half&1) << 5).
  ld  a, h
  and $1f
  or  $40
  ld  h, a
  ld  a, b
  and $01
  rlca
  rlca
  rlca
  rlca
  rlca
  or  h
  ld  h, a
  ld  a, (hl)
  ld  b, a                    ; park result
  pop af                      ; A = interrupted bank
  ld  ($fffe), a
  ld  a, ($cb14)
  ld  ($fffe), a              ; paranoia: $CB14 is the authority
  ld  a, b
  ret

; ─── SRAM ($6000-$7FFF over SMS EXRAM) ────────────────────────────────
; The Sega mapper exposes 8 KiB EXRAM at slot 2 with $FFFC = $08
; (trace_sms models the same bits, so this shim verifies there). Each
; helper saves/restores $FFFC around a single access; no live state.
; The $FFFC manipulation is DI-bracketed (IFF2-preserving, like
; rt_read_prg_high): an NMI landing mid-access would otherwise run the
; handler with EXRAM (not ROM) visible in slot 2.
; rt_sram_read:  HL = NES $6000-$7FFF, exit A = byte. Preserves BC/DE/HL.
; rt_sram_write: HL = addr, A = value. Preserves BC/DE/HL.
rt_sram_read:
  call _sram_check_addr       ; A not live on the read path
  ld  a, i
  di
  jp  po, _sram_read_di
  push hl
  push bc
  ld  a, ($fffc)
  ld  b, a
  ld  a, $08
  ld  ($fffc), a              ; EXRAM on, low half
  ld  a, h
  add a, $20                  ; NES $6000-$7FFF -> slot-2 $8000-$9FFF
  ld  h, a
  ld  a, (hl)
  ld  c, a                    ; park result in C
  ld  a, b
  ld  ($fffc), a
  ld  a, c
  pop bc
  pop hl
  ei
  ret
_sram_read_di:
  push hl
  push bc
  ld  a, ($fffc)
  ld  b, a
  ld  a, $08
  ld  ($fffc), a
  ld  a, h
  add a, $20
  ld  h, a
  ld  a, (hl)
  ld  c, a
  ld  a, b
  ld  ($fffc), a
  ld  a, c
  pop bc
  pop hl
  ret
rt_sram_write:
  ld  c, a                    ; park value: check + `ld a,i` clobber A
  call _sram_check_addr
  push hl
  push bc
  ld  a, i
  di
  jp  po, _sram_write_di
  ld  a, ($fffc)
  ld  b, a
  ld  a, $08
  ld  ($fffc), a
  ld  a, h
  add a, $20
  ld  h, a
  ld  a, c
  ld  (hl), a
  ld  a, b
  ld  ($fffc), a
  pop bc
  pop hl
  ei
  ret
_sram_write_di:
  ld  a, ($fffc)
  ld  b, a
  ld  a, $08
  ld  ($fffc), a
  ld  a, h
  add a, $20
  ld  h, a
  ld  a, c
  ld  (hl), a
  ld  a, b
  ld  ($fffc), a
  pop bc
  pop hl
  ret
; Indexed forms for X/Y table access. HL = base, B = offset; write takes
; the value in C (matches rt_write_indexed's HL=base,B=off,C=val shape).
rt_sram_read_indexed:
  ld  a, l
  add a, b
  ld  l, a
  ld  a, h
  adc a, $00
  ld  h, a
  jr  rt_sram_read
rt_sram_write_indexed:
  push bc                     ; caller BC (B = offset, C = value)
  ld  a, l
  add a, b
  ld  l, a
  ld  a, h
  adc a, $00
  ld  h, a                    ; HL = effective address
  ld  a, c                    ; A = value
  pop bc                      ; balanced (B/C dead below)
  jr  rt_sram_write           ; tail: pushes its own frame
; Fail closed outside $6000-$7FFF. Clobbers AF (callers re-derive A).
_sram_check_addr:
  ld  a, h
  cp  $60
  jr  c, _sram_bad
  cp  $80
  jr  c, _sram_ok
_sram_bad:
  ld  a, $e6
  ld  ($cb1d), a
  halt
  jr  _sram_bad
_sram_ok:
  ret

; ─── rt_wram_blob_seed ────────────────────────────────────────────────
; Boot reset DI only. Copies the baked [[wram_blob]] bytes from ROM into
; SMS EXRAM ($6000-$7FFF over slot-2 $8000-$9FFF). Mother copies the same
; STATIC bytes from CHR at the title->game transition via IRQ-driven PPU
; reads; baking them here removes the dependency on that CHR live-read path
; so translated WRAM code and PRG data reads observe the right bytes.
; The source data lives in slot-2 bank WRAM_BLOB_BANK (ROM); EXRAM shares
; slot 2, so each byte toggles $FFFC between the ROM image and EXRAM.
; Clobbers AF/BC/DE/HL. No IX/IY (z80_emu fails closed on DD/FD prefixes).
; $D300-$D303 are boot-time scratch (translated return-stack slot + the
; dirty-metadata reserve); the game re-initializes them after reset.
.ifdef WRAM_BLOB_COUNT
rt_wram_blob_seed:
  ld   a, WRAM_BLOB_BANK
  ld   ($ffff), a            ; blob data bank into slot 2 (ROM)
  xor  a
  ld   ($fffc), a            ; EXRAM off -> ROM visible in slot 2
  ld   a, WRAM_BLOB_COUNT
  ld   ($d301), a            ; remaining blob count
  ld   hl, data_wram_blob_table
  ld   ($d302), hl           ; table cursor (2-byte shadow)
  ld   hl, data_wram_blob_data
_seed_blob_next:
  push hl                    ; save the monotonic data cursor
  ld   hl, ($d302)           ; HL = table cursor
  ld   e, (hl)               ; E = dest low
  inc  hl
  ld   d, (hl)               ; D = dest high
  inc  hl
  ld   c, (hl)               ; C = length low
  inc  hl
  ld   b, (hl)               ; B = length high
  inc  hl
  ld   ($d302), hl           ; save advanced table cursor
  ld   a, d
  add  a, $20                ; NES $6000-$7FFF -> slot-2 $8000-$9FFF
  ld   d, a                  ; DE = EXRAM address
  pop  hl                    ; restore the data cursor
_seed_blob_byte:
  ld   a, (hl)               ; ROM byte (EXRAM off)
  ld   ($d300), a            ; park the byte while A loads the control
  ld   a, $08
  ld   ($fffc), a            ; EXRAM on
  ld   a, ($d300)
  ld   (de), a               ; write to EXRAM
  xor  a
  ld   ($fffc), a            ; EXRAM off
  inc  hl
  inc  de
  dec  bc
  ld   a, b
  or   c
  jr   nz, _seed_blob_byte
  ld   a, ($d301)
  dec  a
  ld   ($d301), a
  jr   nz, _seed_blob_next
  xor  a
  ld   ($fffc), a
  ret
.endif

; ─── rt_mmc3_chr_sync ─────────────────────────────────────────────────
; NMI-presentation hook: the visible tile set changed (CHR bank writes via
; MMC3_CHR_DIRTY, or a PPUCTRL BG-table switch vs the $CA13 presented-table
; latch), so refresh every assigned variant's pixels in place. Slot numbers
; stay stable, so live nametable cells keep pointing at the right tiles;
; only pixels change. The first sync always refreshes: pre-sync variants
; were generated against the live table, which may predate the latch.
; $CA13 doubles as the latch (it means "last presented BG table" and stays
; $FF in CHR-ROM builds without this hook; MMC3+CHR_RAM fails closed).
rt_mmc3_chr_sync:
  ld  a, ($cb08)
  and $10
  ld  b, a                    ; B = live table bit
  ld  a, ($ca13)
  cp  $ff
  jr  z, _mcs_refresh         ; first sync: latch below, then refresh
  cp  b
  jr  z, _mcs_rcheck
  ld  a, b
  ld  ($ca13), a              ; table switched: latch + refresh
  xor a
  ld  (MMC3_CHR_DIRTY), a
  jp  rt_bg_refresh_variant_cache
_mcs_refresh:
  ld  a, b
  ld  ($ca13), a
  xor a
  ld  (MMC3_CHR_DIRTY), a
  jp  rt_bg_refresh_variant_cache
_mcs_rcheck:
  ld  a, (MMC3_CHR_DIRTY)
  or  a
  ret z
  xor a
  ld  (MMC3_CHR_DIRTY), a
  jp  rt_bg_refresh_variant_cache

; ─── rt_mmc3_line_irq ─────────────────────────────────────────────────
; VDP line-IRQ service. Ticks the scanline counter (MMC3B/C, same as
; nes_rom::Mmc3State::clock_a12); on zero with IRQs enabled, delivers the
; translated IRQ through the sentinel-frame bridge below. Called from
; boot.s `_irq_line_scroll_split` under .ifdef NES_MMC3 with the handler
; prologue done ($CB7E set, VDP acked) and the shared epilogue restoring
; D/E, $CB7E, AF/BC/HL. Preserves D/E (reloads from shadows for the game
; body, writes back after — like the NMI bridge).
rt_mmc3_line_irq:
  ld  a, (MMC3_IRQ_CTRL)
  bit 0, a                   ; enabled?
  ret z
  bit 1, a                   ; reload pending, or counter already zero?
  jr  nz, _mli_reload
  ld  a, (MMC3_IRQ_COUNTER)
  or  a
  jr  z, _mli_reload
  dec a
  ld  (MMC3_IRQ_COUNTER), a
  or  a
  ret nz                     ; nonzero: no IRQ this line
  ; Counter hit zero with enable set: assert the level and deliver.
  ld  a, (MMC3_IRQ_CTRL)
  or  $04
  ld  (MMC3_IRQ_CTRL), a
  jr  mmc3_call_translated_irq
_mli_reload:
  ld  a, (MMC3_IRQ_LATCH)
  ld  (MMC3_IRQ_COUNTER), a
  ld  a, (MMC3_IRQ_CTRL)
  and $fd                    ; clear reload-pending
  ld  (MMC3_IRQ_CTRL), a
  ; A reload that lands on zero with enable set asserts immediately
  ; (matches clock_a12: reload, then `counter == 0 && enabled`).
  ld  a, (MMC3_IRQ_COUNTER)
  or  a
  ret nz
  ld  a, (MMC3_IRQ_CTRL)
  bit 0, a
  ret z
  or  $04
  ld  (MMC3_IRQ_CTRL), a
  ; fall through to deliver

; ─── translated-IRQ bridge ────────────────────────────────────────────
; NES IRQ semantics: hardware pushes PCH/PCL/P and vectors through $FFFE.
; Synthesize the 3-byte frame with the $FFFF sentinel (rt_rti returns
; natively on it, exactly like the NMI bridge) and call translated_irq
; with the interrupted slot-1 bank saved by $CA11 depth. Skips (leaves
; the pending level set) when already nested 2 deep.
mmc3_call_translated_irq:
  ld  a, ($ca11)
  cp  2
  ret nc                     ; bounded nesting: skip, level stays pending
  ld  a, (MMC3_IRQ_CTRL)
  and $fb                    ; deliver: clear the pending level
  ld  (MMC3_IRQ_CTRL), a
  ld  a, ($cb00)
  ld  d, a                   ; resident X
  ld  a, ($cb01)
  ld  e, a                   ; resident Y
  ld  a, ($cb02)
  ld  l, a
  ld  h, $c1
  ld  (hl), $ff              ; sentinel PCH
  dec l
  ld  (hl), $ff              ; sentinel PCL
  dec l
  ld  a, ($cb03)
  ld  (hl), a                ; shadow P
  ld  a, ($cb02)
  sub 3
  ld  ($cb02), a
  ld  a, ($ca11)
  ld  hl, $cb25
  or  a
  jr  z, _mci_save_bank
  inc hl
_mci_save_bank:
  ld  a, ($cb14)
  ld  (hl), a
  ld  a, :translated_irq
  ld  ($cb14), a
  ld  ($fffe), a
  xor a
  ld  ($cb7e), a             ; leaving handler context (helpers may ei)
  ld  a, ($ca11)
  inc a
  ld  ($ca11), a
  ; Level-triggered: run DI (no `ei` — the held level would re-enter
  ; instantly; the frame INT stays pending and delivers after). Translated
  ; SEI/CLI lower to shadow-P bits, never real EI/DI, so no stray `ei`.
  call translated_irq        ; jumps to the profile/ROM IRQ vector
  ld  a, ($ca11)
  dec a
  ld  ($ca11), a
  ld  hl, $cb25
  or  a
  jr  z, _mci_restore_bank
  inc hl
_mci_restore_bank:
  ld  a, $01
  ld  ($cb7e), a             ; back in handler context
  ld  a, d
  ld  ($cb00), a
  ld  a, e
  ld  ($cb01), a
  ld  a, (hl)
  ld  ($cb14), a
  ld  ($fffe), a
  ret

.ends
.endif
