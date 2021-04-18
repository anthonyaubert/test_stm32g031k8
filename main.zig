usingnamespace @import("stm32g0xx.zig");

export fn main() void {
    var tmpReg: u32 = 0;

    SystemInit();

    // enable GPIOC clk
    RCC.*.IOPENR |= RCC_IOPENR_GPIOCEN;
    //* Delay after an RCC peripheral clock enabling */
    tmpReg = RCC.*.IOPENR & RCC_IOPENR_GPIOCEN;

    // reset LED3 pin
    GPIOC.*.BRR |= GPIO_PIN_6;

    //SetPinOutputType pushpull
    GPIOC.*.OTYPER &= ~(GPIO_PIN_6);
    //SetPinPull
    GPIOC.*.PUPDR &= ~(GPIO_PIN_6 * GPIO_PIN_6 * GPIO_PUPDR_PUPD0);
    GPIOC.*.PUPDR |= GPIO_PIN_6 * GPIO_PIN_6 * GPIO_PULL_NO;
    //SetPinMode
    GPIOC.*.MODER &= ~(GPIO_PIN_6 * GPIO_PIN_6 * GPIO_MODER_MODE0);
    GPIOC.*.MODER |= GPIO_PIN_6 * GPIO_PIN_6 * GPIO_MODE_OUTPUT;

    while (true) {
        //set pin to 1
        GPIOC.*.BSRR |= GPIO_PIN_6;
        var i: u32 = 0;
        while (i < 10000) {
            asm volatile ("nop");
            i += 1;
        }

        //set pin to 0
        GPIOC.*.BRR |= GPIO_PIN_6;
        i = 0;
        while (i < 1000000) {
            asm volatile ("nop");
            i += 1;
        }
    }
}
