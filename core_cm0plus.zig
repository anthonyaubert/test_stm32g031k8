const SCB_t = packed struct {
    CPUID: u32,     // Offset: 0x000 (R/ )  CPUID Base Register
    ICSR: u32,      // Offset: 0x004 (R/W)  Interrupt Control and State Register
    VTOR: u32,      // Offset: 0x008 (R/W)  Vector Table Offset Register
    AIRCR: u32,     // Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register
    SCR: u32,       // Offset: 0x010 (R/W)  System Control Register
    CCR: u32,       // Offset: 0x014 (R/W)  Configuration Control Register
    RESERVERD1: u32,
    SHP: [2]u32,    // Offset: 0x01C (R/W)  System Handlers Priority Registers. [0] is RESERVED
    SHCSR: u32,     // Offset: 0x024 (R/W)  System Handler Control and State Register
};

const SCS_BASE = 0xE000E000;
const SCB_BASE = SCS_BASE + 0x0D00;

pub const SCB = @intToPtr(*volatile SCB_t, SCB_BASE);
