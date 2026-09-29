use core::arch::global_asm;

/// Parse cfg attributes inside a global_asm call.
macro_rules! cfg_global_asm {
    {@inner, [$($x:tt)*], } => {
        global_asm!{$($x)*}
    };
    (@inner, [$($x:tt)*], #[cfg($meta:meta)] { $($y:tt)* } $($rest:tt)*) => {
        #[cfg($meta)]
        cfg_global_asm!{@inner, [$($x)*], $($y)* $($rest)*}
        #[cfg(not($meta))]
        cfg_global_asm!{@inner, [$($x)*], $($rest)*}
    };
    (@inner, [$($x:tt)*], #[cfg($meta:meta)] $asm:literal, $($rest:tt)*) => {
        #[cfg($meta)]
        cfg_global_asm!{@inner, [$($x)* $asm,], $($rest)*}
        #[cfg(not($meta))]
        cfg_global_asm!{@inner, [$($x)*], $($rest)*}
    };
    {@inner, [$($x:tt)*], $asm:literal, $($rest:tt)*} => {
        cfg_global_asm!{@inner, [$($x)* $asm,], $($rest)*}
    };
    {$($asms:tt)*} => {
        cfg_global_asm!{@inner, [], $($asms)*}
    };
}

// Provisional patch to avoid LLVM spurious errors when compiling in release mode.
// This patch is somewhat hardcoded and relies on the fact that the rustc compiler
// only supports a limited combination of ISA extensions. This patch should be
// removed when LLVM fixes the issue. Alternatively, it must be updated when rustc
// supports more ISA extension combinations.
//
// Related issues:
// - https://github.com/rust-embedded/riscv/issues/175
// - https://github.com/rust-lang/rust/issues/80608
// - https://github.com/llvm/llvm-project/issues/61991
riscv_macros::rvrt_llvm_arch_patch!();

// Entry point of all programs (_start). It initializes DWARF call frame information,
// the stack pointer, the frame pointer (needed for closures to work in start_rust)
// and the global pointer. Then it calls _start_rust.
// With the relocate feature it runs where the image was loaded until RAM is initialized,
// then calls __relocate to reach the linked addresses (relocate feature).
cfg_global_asm!(
    ".section .init, \"ax\"
    .global _start

_start:",
    // With relocate, _start falls through: _abs_start runs where the image was loaded.
    #[cfg(all(not(feature = "relocate"), target_arch = "riscv32"))]
    "lui ra, %hi(_abs_start)
     jr %lo(_abs_start)(ra)",
    #[cfg(all(not(feature = "relocate"), target_arch = "riscv64"))]
    ".option push
    .option norelax // to prevent an unsupported R_RISCV_ALIGN relocation from being generated
1:
    auipc ra, %pcrel_hi(1f)
    ld ra, %pcrel_lo(1b)(ra)
    jr ra
    .align  3
1:
    .dword _abs_start
    .option pop",
    "
_abs_start:
    .option norelax
    .cfi_startproc
    .cfi_undefined ra",
    // Disable interrupts
    #[cfg(all(feature = "s-mode", not(feature = "no-xie-xip")))]
    "csrw sie, 0
    csrw sip, 0",
    #[cfg(not(feature = "s-mode"))]
    {
        #[cfg(not(feature = "no-xie-xip"))]
        "csrw mie, 0
        csrw mip, 0",
        // Make sure that the hart ID is in a0 in M-mode
        #[cfg(not(feature = "no-mhartid"))]
        "csrr a0, mhartid",
        #[cfg(feature = "no-mhartid")]
        "li a0, 0",
    }
    // Set pre-init trap vector
    #[cfg(not(feature = "no-xtvec"))]
    {
        "la t0, _pre_init_trap",
        #[cfg(feature = "s-mode")]
        "csrw stvec, t0",
        #[cfg(not(feature = "s-mode"))]
        "csrw mtvec, t0",
    }
    // If multi-hart, assert that hart ID is valid
    #[cfg(not(feature = "single-hart"))]
    "lui t0, %hi(_max_hart_id)
    add t0, t0, %lo(_max_hart_id)
    bgeu t0, a0, 1f
    la t0, abort // If hart_id > _max_hart_id, jump to abort
    jr t0
1:", // Only valid hart IDs reach this point
#[cfg(any(feature = "pre-init", feature = "relocate", not(feature = "single-hart")))]
// If startup functions are expected, preserve a0-a2 in s0-s2
{
    "mv s0, a0
    mv s1, a1",
    #[cfg(riscvi)]
    "mv s2, a2",
    #[cfg(not(riscvi))]
    "mv a5, a2", // RVE does not include s2, so we preserve a2 in a5
}
// INITIALIZE GLOBAL POINTER AND STACK POINTER
    ".option push
    .option norelax
    la gp, __global_pointer$
    .option pop",
#[cfg(not(feature = "single-hart"))]
{
    "mv t2, a0
    lui t1, %hi(_hart_stack_size)
    add t1, t1, %lo(_hart_stack_size)",
    #[cfg(riscvm)]
    "mul t0, t2, t1",
    #[cfg(not(riscvm))]
    "mv t0, x0
    beqz t2, 2f  // skip if hart ID is 0
1:
    add t0, t0, t1
    addi t2, t2, -1
    bnez t2, 1b
2:  ",
}
    "la t1, _stack_start",
    #[cfg(not(feature = "single-hart"))]
    "sub t1, t1, t0",
    "andi sp, t1, -16", // align stack to 16-bytes

    #[cfg(not(feature = "single-hart"))]
    // Skip RAM initialization if current hart is not the boot hart
    "call _mp_hook
    beqz a0, 4f",
    #[cfg(feature = "pre-init")]
    "call __pre_init",
    "// Copy .data from flash to RAM, unless it is linked to load where it runs
    la t0, __sdata
    la a3, __edata
    la t1, __sidata
    beq t0, t1, 2f
    bgeu t0, a3, 2f
1:  ",
    #[cfg(target_arch = "riscv32")]
    "lw t2, 0(t1)
    addi t1, t1, 4
    sw t2, 0(t0)
    addi t0, t0, 4
    bltu t0, a3, 1b",
    #[cfg(target_arch = "riscv64")]
    "ld t2, 0(t1)
    addi t1, t1, 8
    sd t2, 0(t0)
    addi t0, t0, 8
    bltu t0, a3, 1b",
    "
2:  // Zero out .bss
    la t0, __sbss
    la t2, __ebss
    bgeu  t0, t2, 4f
3:  ",
    #[cfg(target_arch = "riscv32")]
    "sw  zero, 0(t0)
    addi t0, t0, 4
    bltu t0, t2, 3b",
    #[cfg(target_arch = "riscv64")]
    "sd zero, 0(t0)
    addi t0, t0, 8
    bltu t0, t2, 3b",
    "
4:", // RAM initialized

// RELOCATE: RAM is initialized where the image was loaded. The user's __relocate makes the
// linked addresses reachable and returns to the linked address of the instruction after the
// jump, with the arguments _start was entered with in a0-a2, the address _start was loaded at
// in a3 and the offset from there to where it was linked in a4.
#[cfg(feature = "relocate")]
{
    "mv a0, s0
    mv a1, s1",
    #[cfg(riscvi)]
    "mv a2, s2",
    #[cfg(not(riscvi))]
    "mv a2, a5", // RVE does not include s2, so we use a5 to preserve a2
    "la a3, _start",
    // t0 = the linked address of _start, from an absolute value the linker writes
    #[cfg(target_arch = "riscv32")]
    "lui t0, %hi(_start)
    addi t0, t0, %lo(_start)",
    #[cfg(target_arch = "riscv64")]
    "11:
    auipc t0, %pcrel_hi(13f)
    ld t0, %pcrel_lo(11b)(t0)",
    "sub a4, t0, a3
    la ra, 12f
    add ra, ra, a4
    la t0, __relocate
    jr t0",
    #[cfg(target_arch = "riscv64")]
    ".balign 8
13:
    .dword _start",
    "
12:", // Running at the linked addresses from here on

    // SET GLOBAL POINTER, STACK POINTER AND PRE-INIT TRAP VECTOR AGAIN, at the linked addresses
    ".option push
    .option norelax
    la gp, __global_pointer$
    .option pop",
    #[cfg(not(feature = "single-hart"))]
    {
        "mv t2, s0
        lui t1, %hi(_hart_stack_size)
        add t1, t1, %lo(_hart_stack_size)",
        #[cfg(riscvm)]
        "mul t0, t2, t1",
        #[cfg(not(riscvm))]
        "mv t0, x0
        beqz t2, 15f  // skip if hart ID is 0
14:
        add t0, t0, t1
        addi t2, t2, -1
        bnez t2, 14b
15:  ",
    }
    "la t1, _stack_start",
    #[cfg(not(feature = "single-hart"))]
    "sub t1, t1, t0",
    "andi sp, t1, -16", // align stack to 16-bytes
    #[cfg(not(feature = "no-xtvec"))]
    {
        "la t0, _pre_init_trap",
        #[cfg(feature = "s-mode")]
        "csrw stvec, t0",
        #[cfg(not(feature = "s-mode"))]
        "csrw mtvec, t0",
    }
}

// INITIALIZE THIS HART'S THREAD-LOCAL BLOCK: point tp at block hartid of the .tls
// region and fill it from the template, .tdata copied and the rest zeroed, unless the
// block is zero as loaded (_tls_zeroed_by_loader) and was never filled, which its
// marker tells. Standard per-thread setup, run by every hart on every start: a
// thread-local is private to its hart, so nothing else has written the block.
#[cfg(feature = "tls")]
{
    "la tp, __stls",
    #[cfg(not(feature = "single-hart"))]
    {
        // s0 holds the hart id: the calls above have overwritten a0.
        "mv t2, s0
        lui t1, %hi(_hart_tls_size)
        add t1, t1, %lo(_hart_tls_size)",
        #[cfg(riscvm)]
        "mul t0, t2, t1",
        #[cfg(not(riscvm))]
        "mv t0, x0
        beqz t2, 6f  // skip if hart ID is 0
5:
        add t0, t0, t1
        addi t2, t2, -1
        bnez t2, 5b
6:  ",
        "add tp, tp, t0",
    }
    "// Was the block filled before? The template's marker is 1; a block the loader
    // zeroed and no hart has filled reads 0. Read before the copy writes it.
    lui t0, %tprel_hi(__tls_filled)
    add t0, t0, tp, %tprel_add(__tls_filled)
    lw a4, %tprel_lo(__tls_filled)(t0)",
    "// Copy the template's initialized part
    la t0, __stdata
    la t1, __etdata
    mv t2, tp
    bgeu t0, t1, 8f
7:  ",
    #[cfg(target_arch = "riscv32")]
    "lw a3, 0(t0)
    addi t0, t0, 4
    sw a3, 0(t2)
    addi t2, t2, 4
    bltu t0, t1, 7b",
    #[cfg(target_arch = "riscv64")]
    "ld a3, 0(t0)
    addi t0, t0, 8
    sd a3, 0(t2)
    addi t2, t2, 8
    bltu t0, t1, 7b",
    "
8:  // Zero out the rest of the block, unless it is zero already: the user says the
    // blocks were zero as loaded, and this one was never filled
    lui t1, %hi(_tls_zeroed_by_loader)
    add t1, t1, %lo(_tls_zeroed_by_loader)
    beqz t1, 9f
    beqz a4, 10f
9:  lui t1, %hi(_hart_tls_size)
    add t1, t1, %lo(_hart_tls_size)
    add t1, tp, t1
    bgeu t2, t1, 10f
16: ",
    #[cfg(target_arch = "riscv32")]
    "sw zero, 0(t2)
    addi t2, t2, 4
    bltu t2, t1, 16b",
    #[cfg(target_arch = "riscv64")]
    "sd zero, 0(t2)
    addi t2, t2, 8
    bltu t2, t1, 16b",
    "
10:", // Thread-local block initialized
}

// INITIALIZE FLOATING POINT UNIT
#[cfg(any(riscvf, riscvd))]
{
    "
    li t0, 0x4000 // bit 14 is FS most significant bit
    li t2, 0x2000 // bit 13 is FS least significant bit
    ",
    #[cfg(feature = "s-mode")]
    "csrrc x0, sstatus, t0
    csrrs x0, sstatus, t2",
    #[cfg(not(feature = "s-mode"))]
    "csrrc x0, mstatus, t0
    csrrs x0, mstatus, t2",
    "fscsr x0",
}

#[cfg(any(feature = "pre-init", feature = "relocate", not(feature = "single-hart")))]
// If startup functions are expected, restore a0-a2 from s0-s2
{   "mv a0, s0
    mv a1, s1",
    #[cfg(riscvi)]
    "mv a2, s2",
    #[cfg(not(riscvi))]
    "mv a2, a5", // RVE does not include s2, so we use a5 to preserve a2
}
    // INITIALIZE FRAME POINTER AND JUMP TO _start_rust FUNCTION
    "mv s0, sp
    la t0, _start_rust
    jr t0
    .cfi_endproc",

    #[cfg(not(feature = "single-hart"))]
    // Default implementation of `_mp_hook` wakes hart 0 and busy-loops all the other harts.
    // Users can override this function by defining their own `_mp_hook`.
    // This function is only used when the `single-hart` feature is not enabled.
    ".global _default_mp_hook
_default_mp_hook:
    beqz a0, 2f // if hartid is 0, return true
1:  wfi // Otherwise, wait for interrupt in a loop
    j 1b
2:  li a0, 1
    ret",

    #[cfg(feature = "tls")]
    // The template's marker: a word of .tdata that is not zero, so a block reads it
    // as 1 once filled and as 0 while it is as the loader zeroed it. `_start` reads it
    // at its tp-relative offset, the way local-exec code reaches any thread-local.
    ".pushsection .tdata.riscv_rt_tls_filled, \"awT\", @progbits
    .balign 4
    .type __tls_filled, @tls_object
__tls_filled:
    .word 1
    .popsection",
);

riscv_macros::rvrt_default_start_trap!();

#[rustfmt::skip]
global_asm!(
    ".section .text.abort
.balign 4
.global _default_abort
_default_abort:  // make sure there is an abort symbol when linking
    j _default_abort"
);
