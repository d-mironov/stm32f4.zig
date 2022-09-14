const board = @import("stm32f411xe.zig");
const reg = @import("reg.zig");
const Bit = reg.Bit;
const Pin = board.Pin;

pub fn main() void {
    var pa8: Pin = Pin.new(board.periph.gpioa, 8);
    var pb8: Pin = Pin.new(board.periph.gpiob, 8);

    pa8.make_output();
    pb8.make_output();

    while (true) {
        pa8.write(1);
        pb8.write(0);
        delay();

        pa8.write(0);
        pb8.write(1);
        delay();
    }
}

pub fn delay() void {
    var i: u32 = 0;
    while (i < 100000) : (i += 1) {
        asm volatile ("nop");
    }
}
