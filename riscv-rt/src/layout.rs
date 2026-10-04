//! The layout symbols, read back at run time: what `memory.x` set and where `link.x` placed
//! things.
//!
//! An address is returned as the code reaches it, PC-relative: the load address while the image
//! runs where it was loaded, the linked one once it runs where it was linked.

/// The value of `_max_hart_id`: the highest hart id the image was linked for.
#[inline]
pub fn max_hart_id() -> usize {
    let value: usize;
    // The symbol is an absolute number, not an address near the code, so the PC-relative `la`
    // cannot reach it; `lui`/`addi` can.
    unsafe {
        core::arch::asm!(
            "lui {value}, %hi(_max_hart_id)",
            "addi {value}, {value}, %lo(_max_hart_id)",
            value = out(reg) value,
            options(pure, nomem, nostack, preserves_flags),
        );
    }
    value
}

/// The value of `_hart_stack_size`: the bytes of stack each hart gets.
#[inline]
pub fn hart_stack_size() -> usize {
    let value: usize;
    unsafe {
        core::arch::asm!(
            "lui {value}, %hi(_hart_stack_size)",
            "addi {value}, {value}, %lo(_hart_stack_size)",
            value = out(reg) value,
            options(pure, nomem, nostack, preserves_flags),
        );
    }
    value
}

/// The address of `_stack_start`: the top of hart 0's stack.
#[inline]
pub fn stack_start() -> usize {
    let value: usize;
    unsafe {
        core::arch::asm!(
            "la {value}, _stack_start",
            value = out(reg) value,
            options(pure, nomem, nostack, preserves_flags),
        );
    }
    value
}

/// The address of `_stext`: the first byte of code.
#[inline]
pub fn stext() -> usize {
    let value: usize;
    unsafe {
        core::arch::asm!(
            "la {value}, _stext",
            value = out(reg) value,
            options(pure, nomem, nostack, preserves_flags),
        );
    }
    value
}

/// The address of `_start`: the image's entry.
#[inline]
pub fn start() -> usize {
    let value: usize;
    unsafe {
        core::arch::asm!(
            "la {value}, _start",
            value = out(reg) value,
            options(pure, nomem, nostack, preserves_flags),
        );
    }
    value
}

/// The top of hart `hartid`'s stack, as `_start` lays the stacks out: `_stack_start` less
/// `hartid` stacks of `_hart_stack_size`.
#[inline]
pub fn stack_top(hartid: usize) -> usize {
    stack_start() - hartid * hart_stack_size()
}
