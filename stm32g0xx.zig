usingnamespace @import("core_cm0plus.zig");

pub const FLASH_BASE: u32 = 0x08000000;               // FLASH base address
pub const VECT_TAB_OFFSET: u32 = 0x0;                      
const PERIPH_BASE: u32 = 0x40000000;                   // Peripheral base address
const IOPORT_BASE: u32 = 0x50000000;                   // IOPORT base address
const SRAM_SIZE_MAX: u32 = 0x00002000;                  // maximum SRAM size (up to 8 KBytes)

// Peripheral memory map 
const APBPERIPH_BASE: u32 = PERIPH_BASE;
const AHBPERIPH_BASE: u32 = PERIPH_BASE + 0x20000;
const RCC_BASE: u32 = AHBPERIPH_BASE + 0x00001000;
const FLASH_R_BASE: u32 = AHBPERIPH_BASE + 0x00002000;

// APB peripherals
const TIM2_BASE       : u32=APBPERIPH_BASE + 0;
const TIM3_BASE       : u32=APBPERIPH_BASE + 0x00000400;
const TIM14_BASE      : u32=APBPERIPH_BASE + 0x00002000;
const RTC_BASE        : u32=APBPERIPH_BASE + 0x00002800;
const WWDG_BASE       : u32=APBPERIPH_BASE + 0x00002C00;
const IWDG_BASE       :u32=APBPERIPH_BASE + 0x00003000;
const SPI2_BASE       :u32=APBPERIPH_BASE + 0x00003800;
const USART2_BASE     :u32=APBPERIPH_BASE + 0x00004400;
const I2C1_BASE       :u32=APBPERIPH_BASE + 0x00005400;
const I2C2_BASE       :u32=APBPERIPH_BASE + 0x00005800;
const PWR_BASE        :u32=APBPERIPH_BASE + 0x00007000;
const LPTIM1_BASE     :u32=APBPERIPH_BASE + 0x00007C00;
const LPUART1_BASE    :u32=APBPERIPH_BASE + 0x00008000;
const LPTIM2_BASE     :u32=APBPERIPH_BASE + 0x00009400;
const TAMP_BASE       :u32=APBPERIPH_BASE + 0x0000B000;
const SYSCFG_BASE     :u32=APBPERIPH_BASE + 0x00010000;
const VREFBUF_BASE    :u32=APBPERIPH_BASE + 0x00010030;
const ADC1_BASE       :u32=APBPERIPH_BASE + 0x00012400;
const ADC1_COMMON_BASE:u32=APBPERIPH_BASE + 0x00012708;
const ADC_BASE        :u32=ADC1_COMMON_BASE; // Kept for legacy purpose
const TIM1_BASE       :u32=APBPERIPH_BASE + 0x00012C00;
const SPI1_BASE       :u32=APBPERIPH_BASE + 0x00013000;
const USART1_BASE     :u32=APBPERIPH_BASE + 0x00013800;
const TIM16_BASE      :u32=APBPERIPH_BASE + 0x00014400;
const TIM17_BASE      :u32=APBPERIPH_BASE + 0x00014800;
const DBG_BASE        :u32=APBPERIPH_BASE + 0x00015800;


// IOPORT
const GPIOA_BASE :u32= IOPORT_BASE + 0x00000000;
const GPIOB_BASE :u32= IOPORT_BASE + 0x00000400;
const GPIOC_BASE :u32= IOPORT_BASE + 0x00000800;    
const GPIOD_BASE :u32= IOPORT_BASE + 0x00000C00;
const GPIOF_BASE :u32= IOPORT_BASE + 0x00001400;

//*******************  Bits definition for FLASH_ACR register  *****************/
const FLASH_ACR_LATENCY   : u32 = 0x00000007;
const FLASH_ACR_LATENCY_0 : u32 = 0x00000001;
const FLASH_ACR_LATENCY_1 : u32 = 0x00000002;
const FLASH_ACR_LATENCY_2 : u32 = 0x00000004;

pub const GPIO_PUPDR_PUPD0: u32 = 0x00000003;
pub const GPIO_PULL_NO: u32 = 0x00000000;          // Select I/O no pull
pub const GPIO_PULL_UP: u32 = 0x00000001;          // Select I/O pull up
pub const GPIO_PULL_DOWN: u32 = 0x00000002;        // Select I/O pull down

pub const GPIO_MODER_MODE0: u32 = 0x00000003;
pub const GPIO_MODE_INPUT: u32 = 0x00000000;       // Select input mode
pub const GPIO_MODE_OUTPUT: u32 = 0x00000001;      // Select output mode
pub const GPIO_MODE_ALTERNATE: u32 = 0x00000002;   // Select alternate function mode
pub const GPIO_MODE_ANALOG: u32 = GPIO_MODER_MODE0;// Select analog mode


pub const GPIO_PIN_6: u32 = 0x00000040;

const RCC_CR_HSION: u32 = 0x00000100;
const RCC_CR_HSIRDY: u32 = 0x00000400;

const RCC_PLLCFGR_PLLSRC: u32 = 0x00000003;
const RCC_PLLCFGR_PLLM: u32 = 0x00000070;
const RCC_PLLCFGR_PLLN: u32 = 0x00007F00;
const RCC_PLLCFGR_PLLR: u32 = 0xE0000000;

const RCC_PLLSOURCE_HSI: u32 = 0x00000002; //*!< HSI16 clock selected as PLL entry clock source */
const RCC_PLLM_DIV_1: u32 = 0x00000000;    //*!< PLL division factor by 1 */
const RCC_PLLR_DIV_2: u32 = 0x20000000;    //*!< Main PLL division factor for PLLCLK (system clock) by 2 */

const RCC_CFGR_HPRE: u32 = 0x00000F00;     //*!< HPRE[3:0] bits (AHB prescaler) */
const RCC_SYSCLK_DIV_1: u32 = 0x00000000;  //*!< SYSCLK not divided */

const RCC_CFGR_SW: u32 = 0x00000007;       //*!< SW[2:0] bits (System clock Switch) */
const RCC_CFGR_SW_1: u32 = 0x00000002;     //*!< PLL selection as system clock */

const RCC_CFGR_SWS: u32 = 0x00000038;      //*!< SWS[2:0] bits (System Clock Switch Status) */
const RCC_CFGR_SWS_1: u32 = 0x00000010;

const RCC_CFGR_PPRE: u32 = 0x00007000;

//APB low-speed prescaler (APB1)
const RCC_APB1_DIV_1: u32 = 0x00000000;     //*!<  APB low-speed prescaler (APB1) */
const RCC_CR_PLLON: u32 = 0x01000000;       //*!< System PLL clock enable */
const RCC_PLLCFGR_PLLREN: u32 = 0x10000000;
const RCC_CR_PLLRDY: u32 = 0x02000000;      // System PLL clock ready

pub const RCC_IOPENR_GPIOCEN: u32 = 0x00000004; 

const GPIO_t = packed struct {
    MODER: u32,   // GPIO port mode register,               Address offset: 0x00
    OTYPER: u32,  // GPIO port output type register,        Address offset: 0x04
    OSPEEDR: u32, // GPIO port output speed register,       Address offset: 0x08
    PUPDR: u32,   // GPIO port pull-up/pull-down register,  Address offset: 0x0C
    IDR: u32,     // GPIO port input data register,         Address offset: 0x10
    ODR: u32,     // GPIO port output data register,        Address offset: 0x14
    BSRR: u32,    // GPIO port bit set/reset  register,     Address offset: 0x18
    LCKR: u32,    // GPIO port configuration lock register, Address offset: 0x1C
    AFR: [2]u32,  // GPIO alternate function registers,     Address offset: 0x20-0x24
    BRR: u32,     // GPIO Bit Reset register,               Address offset: 0x28
};

const RCC_t = packed struct {
    CR: u32,          // RCC Clock Sources Control Register,                                     Address offset: 0x00
    ICSCR: u32,       // RCC Internal Clock Sources Calibration Register,                        Address offset: 0x04
    CFGR: u32,        // RCC Regulated Domain Clocks Configuration Register,                     Address offset: 0x08
    PLLCFGR: u32,     // RCC System PLL configuration Register,                                  Address offset: 0x0C
    RESERVED0: u32,   // Reserved,                                                               Address offset: 0x10
    RESERVED1: u32,   // Reserved,                                                               Address offset: 0x14
    CIER: u32,        // RCC Clock Interrupt Enable Register,                                    Address offset: 0x18
    CIFR: u32,        // RCC Clock Interrupt Flag Register,                                      Address offset: 0x1C
    CICR: u32,        // RCC Clock Interrupt Clear Register,                                     Address offset: 0x20
    IOPRSTR: u32,     // RCC IO port reset register,                                             Address offset: 0x24
    AHBRSTR: u32,     // RCC AHB peripherals reset register,                                     Address offset: 0x28
    APBRSTR1: u32,    // RCC APB peripherals reset register 1,                                   Address offset: 0x2C
    APBRSTR2: u32,    // RCC APB peripherals reset register 2,                                   Address offset: 0x30
    IOPENR: u32,      // RCC IO port enable register,                                            Address offset: 0x34
    AHBENR: u32,      // RCC AHB peripherals clock enable register,                              Address offset: 0x38
    APBENR1: u32,     // RCC APB peripherals clock enable register1,                             Address offset: 0x3C
    APBENR2: u32,     // RCC APB peripherals clock enable register2,                             Address offset: 0x40
    IOPSMENR: u32,    // RCC IO port clocks enable in sleep mode register,                       Address offset: 0x44
    AHBSMENR: u32,    // RCC AHB peripheral clocks enable in sleep mode register,                Address offset: 0x48
    APBSMENR1: u32,   // RCC APB peripheral clocks enable in sleep mode register1,               Address offset: 0x4C
    APBSMENR2: u32,   // RCC APB peripheral clocks enable in sleep mode register2,               Address offset: 0x50
    CCIPR: u32,       // RCC Peripherals Independent Clocks Configuration Register,              Address offset: 0x54
    RESERVED2: u32,   // Reserved,                                                               Address offset: 0x58
    BDCR: u32,        // RCC Backup Domain Control Register,                                     Address offset: 0x5C
    CSR: u32,         // RCC Unregulated Domain Clock Control and Status Register,               Address offset: 0x60
};

const FLASH_t = packed struct {
    ACR: u32,          // FLASH Access Control register,                     Address offset: 0x00
    RESERVED1: u32,    // Reserved1,                                         Address offset: 0x04
    KEYR: u32,         // FLASH Key register,                                Address offset: 0x08
    OPTKEYR: u32,      // FLASH Option Key register,                         Address offset: 0x0C
    SR: u32,           // FLASH Status register,                             Address offset: 0x10
    CR: u32,           // FLASH Control register,                            Address offset: 0x14
    ECCR: u32,         // FLASH ECC register,                                Address offset: 0x18
    RESERVED2: u32,    // Reserved2,                                         Address offset: 0x1C
    OPTR: u32,         // FLASH Option register,                             Address offset: 0x20
    PCROP1ASR: u32,    // FLASH Bank PCROP area A Start address register,    Address offset: 0x24
    PCROP1AER: u32,    // FLASH Bank PCROP area A End address register,      Address offset: 0x28
    WRP1AR: u32,       // FLASH Bank WRP area A address register,            Address offset: 0x2C
    WRP1BR: u32,       // FLASH Bank WRP area B address register,            Address offset: 0x30
    PCROP1BSR: u32,    // FLASH Bank PCROP area B Start address register,    Address offset: 0x34
    PCROP1BER: u32,    // FLASH Bank PCROP area B End address register,      Address offset: 0x38
    RESERVED8: [17]u32,// Reserved8,                                         Address offset: 0x3C--0x7C
    SECR: u32,         // FLASH security register ,                          Address offset: 0x80
};

pub const GPIOC = @intToPtr(*volatile GPIO_t, GPIOC_BASE);
pub const RCC = @intToPtr(*volatile RCC_t, RCC_BASE);
pub const FLASH = @intToPtr(*volatile FLASH_t, FLASH_R_BASE);

// copied verbatim from STM32 SDK
pub fn SystemInit() void {
    
    var tmpReg: u32 = 0;
    
    //* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    //* set SYSCFGEN bit */
    RCC.*.APBENR2 |= 0x00000001;
    //* Delay after an RCC peripheral clock enabling */
    tmpReg = RCC.*.APBENR2 & 0x00000001;

    //* set PWREN bit */
    RCC.*.APBENR1 |= (1 << 28);
    //* Delay after an RCC peripheral clock enabling */
    tmpReg = RCC.*.APBENR1 & (1 << 28);

    SetSysClock();
}

fn SetSysClock() void {
    var StartUpCounter: u32 = 0;
    var HSEStatus: u32 = 0;

    FLASH.*.ACR &= ~FLASH_ACR_LATENCY;
    FLASH.*.ACR |= FLASH_ACR_LATENCY_2;
    while ((FLASH.*.ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2) {}

    //* HSI configuration and activation */
    RCC.*.CR |= RCC_CR_HSION;
    while((RCC.*.CR & RCC_CR_HSIRDY) != RCC_CR_HSIRDY) {}

    //* Main PLL configuration and activation */
    // LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
    RCC.*.PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLR);
    RCC.*.PLLCFGR |= (RCC_PLLSOURCE_HSI | RCC_PLLM_DIV_1 | (8<<8) | RCC_PLLR_DIV_2);
    
    // Enable PLL
    RCC.*.CR |= RCC_CR_PLLON;

    // Enable PLL output mapped on SYSCLK domain
    RCC.*.PLLCFGR |= RCC_PLLCFGR_PLLREN;
    
    //* Wait till PLL is ready */
    while ((RCC.*.CR & RCC_CR_PLLRDY) == 0) {}

    //* Set AHB prescaler*/
    RCC.*.CFGR &= ~RCC_CFGR_HPRE;
    RCC.*.CFGR |= RCC_SYSCLK_DIV_1;

    //* Sysclk activation on the main PLL */
    RCC.*.CFGR &= ~RCC_CFGR_SW;
    RCC.*.CFGR |= RCC_CFGR_SW_1;    
    while ((RCC.*.CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1) {}

    //* Set APB1 prescaler*/
    RCC.*.CFGR &= ~RCC_CFGR_PPRE;
    RCC.*.CFGR |= RCC_APB1_DIV_1;

    // TODO
    //LL_Init1msTick(64000000);

    // TODO
    //* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    //LL_SetSystemCoreClock(64000000);
    //LL_RCC_SetTIMClockSource(LL_RCC_TIM1_CLKSOURCE_PCLK1);
}
