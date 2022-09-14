pub const Bit = [_]u32{
    (1 << 0),  (1 << 1),  (1 << 2),  (1 << 3),
    (1 << 4),  (1 << 5),  (1 << 6),  (1 << 7),
    (1 << 8),  (1 << 9),  (1 << 10), (1 << 11),
    (1 << 12), (1 << 13), (1 << 14), (1 << 15),
    (1 << 16), (1 << 17), (1 << 18), (1 << 19),
    (1 << 20), (1 << 21), (1 << 22), (1 << 23),
    (1 << 24), (1 << 25), (1 << 26), (1 << 27),
    (1 << 28), (1 << 29), (1 << 30), (1 << 31),
};

/// # Register structure
///
/// Provides necessary functions to read/write/modify a memory register
///
/// Especially useful in Bare Metal Embedded Development
///
pub const Reg = struct {
    /// `address`: Memory address of the register
    address: *u32 = null,

    pub const Modify = enum {
        Set,
        Clear,
    };

    /// # Create new Register
    ///
    /// Creates a new register by providing the memory address to the specified 
    /// Register
    ///
    /// `address`: Pointer to specified register
    ///
    /// `return`: Register object with operations possible
    pub fn new(address: usize) Reg {
        return Reg{
            //.address = @intToPtr(*volatile u32, address),
            .address = @intToPtr(*u32, address),
            //.address = @ptrCast(*u32, address),
        };
    }

    /// # Read register
    ///
    /// Read the register and return it's value
    ///
    /// `return`: Loaded value from memory address of the register
    pub inline fn read(self: Reg) u32 {
        return @atomicLoad(u32, self.address, .SeqCst);
    }

    /// # Register write
    ///
    /// Write value to the register, overriding previous one
    ///
    /// `val`: Value to write into register
    pub inline fn write(self: Reg, val: u32) void {
        @atomicStore(u32, self.address, val, .SeqCst);
        //self.address.* = @bitCast(u32, val);
    }

    /// # Register set bit
    ///
    /// Set specified bit `n` of the Register (max: 31)
    ///
    /// `n`: Bit to set
    pub inline fn set(self: Reg, n: u32) void {
        _ = @atomicRmw(u32, self.address, .Or, Bit[n], .SeqCst);
    }

    /// # Register check bit
    ///
    /// Checks if specified Bit of the register is set or not
    ///
    /// `return`: `true` if set, else `false`
    pub inline fn is_set(self: Reg, n: u32) bool {
        var val = @atomicLoad(u32, self.address, .SeqCst);
        return val & Bit[n];
    }

    /// # Register clear bit
    ///
    /// Clears a specified bit `n` of the register (max: 31).
    ///
    /// `n`: Bit to clear
    pub inline fn clear(self: Reg, n: u32) void {
        _ = @atomicRmw(u32, self.address, .And, ~Bit[n], .SeqCst);
    }

    /// # Register modify
    ///
    /// Modify the register and don't override the previous value
    /// Please use `Bit[n]` where `n` is the bit you want to modify
    ///
    /// e.g.: `reg.modify(Bit[1] | Bit[3] | Bit[8]);`
    ///
    /// `n`: Specified bits or own value to modify
    pub inline fn modify(self: Reg, mod: Modify, val: u32) void {
        _ = switch (mod) {
            .Set => @atomicRmw(u32, self.address, .Or, val, .SeqCst),
            .Clear => @atomicRmw(u32, self.address, .And, ~val, .SeqCst),
        };
    }

    pub inline fn xor(self: Reg, val: u32) void {
        _ = @atomicRmw(u32, self.address, .Xor, val, .SeqCst);
    }

    /// TODO: Rewrite
    pub inline fn modify_clear(self: Reg, val: u32) void {
        _ = @atomicRmw(u32, self.address, .And, ~val, .SeqCst);
    }
};
