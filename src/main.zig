const regs = @import("./stm32f411.zig").registers;
// const regs = @import("microzig").chip.registers;

pub fn main() !void {
    regs.RCC.AHB1ENR.modify(.{
        .GPIOAEN = 1,
        .GPIOBEN = 1,
    });

    regs.GPIOA.MODER.modify(.{
        .MODER8 = 0b01,
    });
    regs.GPIOB.MODER.modify(.{
        .MODER8 = 0b01,
    });
    regs.GPIOA.ODR.modify(.{
        .ODR8 = 1,
    });
    regs.GPIOB.ODR.modify(.{
        .ODR8 = 1,
    });

    while (true) {
        regs.GPIOA.ODR.modify(.{
            .ODR8 = 1,
        });
        regs.GPIOB.ODR.modify(.{
            .ODR8 = 0,
        });
        delay();

        regs.GPIOA.ODR.modify(.{
            .ODR8 = 0,
        });
        regs.GPIOB.ODR.modify(.{
            .ODR8 = 1,
        });
        delay();
    }
}

pub fn delay() void {
    var i: u32 = 0;
    while (i < 100000) : (i += 1) {
        asm volatile ("nop");
    }
}
