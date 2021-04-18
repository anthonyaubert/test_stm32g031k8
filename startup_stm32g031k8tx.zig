usingnamespace @import("stm32g0xx.zig");
usingnamespace @import("core_cm0plus.zig");

const builtin = @import("builtin");

extern fn main() noreturn;
extern var __text_end: u32;
extern var __data_start: u32;
extern var __data_size: u32;
extern var __bss_start: u32;
extern var __bss_size: u32;
extern var __stack_top: u32;

export fn Reset_Handler() callconv(.C) noreturn {
    // Vector Table Relocation in Internal FLASH.
    SCB.*.VTOR = FLASH_BASE | VECT_TAB_OFFSET; 

    // copy data from flash to RAM
    const data_size = @ptrToInt(&__data_size);
    const data = @ptrCast([*]u8, &__data_start);
    const text_end = @ptrCast([*]u8, &__text_end);
    for (text_end[0..data_size]) |b, i| data[i] = b;
    
    // clear the bss
    const bss_size = @ptrToInt(&__bss_size);
    const bss = @ptrCast([*]u8, &__bss_start);
    for (bss[0..bss_size]) |*b| b.* = 0;
    
    // start
    main();
}

export fn BusyDummy_Handler() callconv(.C) void {
    while (true) {}
}

export fn Dummy_Handler() callconv(.C) void {}

extern fn NMI_Handler() callconv(.C) void;
extern fn HardFault_Handler() callconv(.C) void;
extern fn SVC_Handler() callconv(.C) void;
extern fn PendSV_Handler() callconv(.C) void;
extern fn SysTick_Handler() callconv(.C) void;

const Isr = fn () callconv(.C) void;

export var vector_table linksection(".isr_vector") = [_]?Isr{
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    null,
    null,
    null,
    null,
    null,
    null,
    null,
    SVC_Handler,
    null,
    null,
    PendSV_Handler,
    SysTick_Handler,
};
