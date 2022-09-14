# STM32F4 HAL written in Zig

This is a small experiment for writing a Hardware Abstraction Layer in Zig.  
For now only the GPIO and RCC works and general register abstraction is given.  
There is also a plan to write a good **svd2zig** converter that you can generate the 
HAL for any STM32 chip.  
I'm happy to receive some feedback on the code structure and software design in general, so feel 
free to contact me (Contact info below).

### Example usage
```rust
const board = @import("stm32f411xe.zig");
const Pin = board.Pin;

pub fn main() void {
    // Create new pins PA8 and PB8
    var pa8: Pin = Pin.new(board.periph.gpioa, 8);
    var pb8: Pin = Pin.new(board.periph.gpiob, 8);

    // configure the pins in output mode
    pa8.make_output();
    pb8.make_output();

    while (true) {
        // Write HIGH on PA8 and LOW on PB8
        pa8.write(1);
        pb8.write(0);
        delay();

        // Write LOW on PA8 and HIGH on PB8
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
```

### Contact
Feel free to contact me on via E-Mail or Discord.  
- Discord: moonraccoon#4788  
- E-Mail: [moonxraccoon@protonmail.com](mailto:moonxraccoon@protonmail.com)
