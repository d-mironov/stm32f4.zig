const reg = @import("reg.zig");
const Reg = reg.Reg;
const Bit = reg.Bit;

const DMA2_BASE: u32 = 0x40026400;
const DMA1_BASE: u32 = 0x40026000;
const FLASH_BASE: u32 = 0x40023C00;
const RCC_BASE: u32 = 0x40023800;
const CRC_BASE: u32 = 0x40023000;
const GPIOH_BASE: u32 = 0x40021C00;
const GPIOE_BASE: u32 = 0x40021000;
const GPIOD_BASE: u32 = 0x40020C00;
const GPIOC_BASE: u32 = 0x40020800;
const GPIOB_BASE: u32 = 0x40020400;
const GPIOA_BASE: u32 = 0x40020000;
const SPI5_BASE: u32 = 0x40015000;
const TIM11_BASE: u32 = 0x40014800;
const TIM10_BASE: u32 = 0x40014400;
const TIM9_BASE: u32 = 0x40014000;
const EXTI_BASE: u32 = 0x40013C00;
const SYSCFG_BASE: u32 = 0x40013800;
const SPI4_BASE: u32 = 0x40013400;
const SPI1_BASE: u32 = 0x40013000;
const SDIO_BASE: u32 = 0x40012C00;
const ADC1_BASE: u32 = 0x40012000;
const USART6_BASE: u32 = 0x40011400;
const USART1_BASE: u32 = 0x40011000;
const TIM1_BASE: u32 = 0x40010000;
const PWR_BASE: u32 = 0x40007000;
const I2C3_BASE: u32 = 0x40005C00;
const I2C2_BASE: u32 = 0x40005800;
const I2C1_BASE: u32 = 0x40005400;
const USART2_BASE: u32 = 0x40004400;
const I2S3ext_BASE: u32 = 0x40004000;
const SPI3_BASE: u32 = 0x40003C00;
const SPI2_BASE: u32 = 0x40003800;
const I2S2ext_BASE: u32 = 0x40003400;
const IWDG_BASE: u32 = 0x40003000;
const WWDG_BASE: u32 = 0x40002C00;
const RTC_BASE: u32 = 0x40002800;
const TIM5_BASE: u32 = 0x40000C00;
const TIM4_BASE: u32 = 0x40000800;
const TIM3_BASE: u32 = 0x40000400;
const TIM2_BASE: u32 = 0x40000000;

pub const GPIO_t = packed struct {
    MODER: u32,
    OTYPER: u32,
    OSPEEDR: u32,
    PUPDR: u32,
    IDR: u32,
    ODR: u32,
    BSRR: u32,
    LCKR: u32,
    AFRL: u32,
    AFRH: u32,
};

pub const PWR_t = packed struct {
    CR: u32,
    CSR: u32,
};

pub const SYSCFG_t = packed struct {
    MEMRMP: u32,
    PMC: u32,
    EXTICR1: u32,
    EXTICR2: u32,
    EXTICR3: u32,
    EXTICR4: u32,
    _reserved0: [2]u32,
    CMPCR: u32,
};

pub const DMA_t = packed struct {
    LISR: u32,
    HISR: u32,
    LIFCR: u32,
    HIFCR: u32,
};

pub const EXTI_t = packed struct {
    IMR: u32,
    EMR: u32,
    RTSR: u32,
    FTSR: u32,
    SWIER: u32,
    PR: u32,
};

pub const ADC_t = packed struct {
    SR: u32,
    CR1: u32,
    CR2: u32,
    SMPR1: u32,
    SMPR2: u32,
    JOFR1: u32,
    JOFR2: u32,
    JOFR3: u32,
    JOFR4: u32,
    HTR: u32,
    LTR: u32,
    SQR1: u32,
    SQR2: u32,
    SQR3: u32,
    JSQR: u32,
    JDR1: u32,
    JDR2: u32,
    JDR3: u32,
    JDR4: u32,
    DR: u32,
    CCR: u32,
};

pub const IWDG_t = packed struct {
    KR: u32,
    PR: u32,
    RLR: u32,
    SR: u32,
};

pub const WWDG_t = packed struct {
    CR: u32,
    CFR: u32,
    SR: u32,
};

pub const RTC_t = packed struct {
    TR: u32,
    DR: u32,
    CR: u32,
    ISR: u32,
    PRER: u32,
    WUTR: u32,
    CALIBR: u32,
    ALRMAR: u32,
    ALRMBR: u32,
    WPR: u32,
    SSR: u32,
    SHIFTR: u32,
    TSTR: u32,
    TSDR: u32,
    TSSSR: u32,
    CALR: u32,
    TAFCR: u32,
    ALRMASSR: u32,
    ALRMBSSR: u32,
    _reserved0: u32,
    BKP0R: u32,
    BKP1R: u32,
    BKP2R: u32,
    BKP3R: u32,
    BKP4R: u32,
    BKP5R: u32,
    BKP6R: u32,
    BKP7R: u32,
    BKP8R: u32,
    BKP9R: u32,
    BKP10R: u32,
    BKP11R: u32,
    BKP12R: u32,
    BKP13R: u32,
    BKP14R: u32,
    BKP15R: u32,
    BKP16R: u32,
    BKP17R: u32,
    BKP18R: u32,
    BKP19R: u32,
};

pub const I2C_t = packed struct {
    CR1: u32,
    CR2: u32,
    OAR1: u32,
    OAR2: u32,
    DR: u32,
    SR1: u32,
    SR2: u32,
    CCR: u32,
    TRISE: u32,
    FLTR: u32,
};

pub const USART_t = packed struct {
    SR: u32,
    DR: u32,
    BRR: u32,
    CR1: u32,
    CR2: u32,
    CR3: u32,
    GTPR: u32,
};

pub const SPI_t = packed struct {
    CR1: u32,
    CR2: u32,
    SR: u32,
    DR: u32,
    CRCPR: u32,
    RXCRCR: u32,
    TXCRCR: u32,
    I2SCFGR: u32,
    I2SPR: u32,
};

pub const SDIO_t = packed struct {
    POWER: u32,
    CLKCR: u32,
    ARG: u32,
    CMD: u32,
    RESPCMD: u32,
    RESP1: u32,
    RESP2: u32,
    RESP3: u32,
    RESP4: u32,
    DTIMER: u32,
    DLEN: u32,
    DCTRL: u32,
    DCOUNT: u32,
    STA: u32,
    ICR: u32,
    MASK: u32,
    _reserved0: [2]u32,
    FIFOCNT: u32,
    _reserved1: [13]u32,
    FIFO: u32,
};

pub const FLASH_t = packed struct {
    ACR: u32,
    KEYR: u32,
    OPTKEYR: u32,
    SR: u32,
    CR: u32,
    OPTCR: u32,
};

pub const RCC_t = packed struct {
    CR: u32,
    PLLCFGR: u32,
    CFGR: u32,
    CIR: u32,
    AHB1RSTR: u32,
    AHB2RSTR: u32,
    _reserved0: [2]u32,
    APB1RSTR: u32,
    APB2RSTR: u32,
    _reserved1: [2]u32,
    AHB1ENR: u32,
    AHB2ENR: u32,
    _reserved2: [2]u32,
    APB1ENR: u32,
    APB2ENR: u32,
    _reserved3: [2]u32,
    AHB1LPENR: u32,
    AHB2LPENR: u32,
    _reserved4: [2]u32,
    APB1LPENR: u32,
    APB2LPENR: u32,
    _reserved5: [2]u32,
    BDCR: u32,
    CSR: u32,
    _reserved6: [2]u32,
    SSCGR: u32,
    PLLI2SCFGR: u32,
    DCKCFGR: u32,
};

pub const GPIOA = @intToPtr(*volatile GPIO_t, GPIOA_BASE);
pub const GPIOB = @intToPtr(*volatile GPIO_t, GPIOB_BASE);
pub const GPIOC = @intToPtr(*volatile GPIO_t, GPIOC_BASE);
pub const GPIOD = @intToPtr(*volatile GPIO_t, GPIOD_BASE);
pub const GPIOE = @intToPtr(*volatile GPIO_t, GPIOE_BASE);
pub const GPIOH = @intToPtr(*volatile GPIO_t, GPIOH_BASE);

pub const RCC = @intToPtr(*volatile RCC_t, RCC_BASE);
pub const FLASH = @intToPtr(*volatile FLASH_t, FLASH_BASE);

pub const rcc_t = struct {
    cr: Reg,
    pllcfgr: Reg,
    cfgr: Reg,
    cir: Reg,
    ahb1rstr: Reg,
    ahb2rstr: Reg,
    apb1rstr: Reg,
    apb2rstr: Reg,
    ahb1enr: Reg,
    ahb2enr: Reg,
    apb1enr: Reg,
    apb2enr: Reg,
    ahb1lpenr: Reg,
    ahb2lpenr: Reg,
    apb1lpenr: Reg,
    apb2lpenr: Reg,
    bdcr: Reg,
    csr: Reg,
    sscgr: Reg,
    plli2scfgr: Reg,
    dckcfgr: Reg,

    const Self = @This();

    pub fn enable_gpioa(self: Self) void {
        self.ahb1enr.modify(.Set, Bit[0]);
    }

    pub fn enable_gpiob(self: Self) void {
        self.ahb1enr.modify(.Set, Bit[1]);
    }

    pub fn enable_gpioc(self: Self) void {
        self.ahb1enr.modify(.Set, Bit[2]);
    }

    pub fn enable_gpiod(self: Self) void {
        self.ahb1enr.modify(.Set, Bit[3]);
    }

    pub fn enable_gpioe(self: Self) void {
        self.ahb1enr.modify(.Set, Bit[4]);
    }

    pub fn enable_gpioh(self: Self) void {
        self.ahb1enr.modify(.Set, Bit[7]);
    }

    pub fn enable_crc(self: Self) void {
        self.ahb1enr.modify(.Set, Bit[12]);
    }

    pub fn enable_dma1(self: Self) void {
        self.ahb1enr.modify(.Set, Bit[21]);
    }

    pub fn enable_dma2(self: Self) void {
        self.ahb1enr.modify(.Set, Bit[22]);
    }
};

pub const gpio_t = struct {
    moder: Reg,
    otyper: Reg,
    ospeedr: Reg,
    pupdr: Reg,
    idr: Reg,
    odr: Reg,
    bsrr: Reg,
    lckr: Reg,
    afrl: Reg,
    afrh: Reg,

    rcc: *const rcc_t,
    port: Port = undefined,

    const Port = enum { A, B, C, D, E, H };

    const Self = @This();
};

pub const Pin = struct {
    port: gpio_t,
    pin: u32 = 0,
    mode: Mode = .Output,
    outtype: OutputType = .PushPull,
    speed: Speed = .Low,
    pull: Pull = .None,
    _clk_en: bool = false,

    const Self = @This();

    /// # Pin - Operation mode
    const Mode = enum {
        /// # Input mode
        Input,
        /// # Output mode
        Output,
        /// # Alternate-function mode
        Alternate,
        /// # Analog mode
        Analog,
    };

    /// # Pin - Output type
    const OutputType = enum {
        /// # Push-Pull mode
        PushPull,
        /// # Open drain mode
        OpenDrain,
    };

    /// # Pin - Speed
    const Speed = enum {
        /// # Low Speed
        Low,
        /// # Medium Speed
        Medium,
        /// # Fast Speed
        Fast,
        /// # High Speed
        High,
    };

    /// # Pin - Pull mode
    const Pull = enum {
        /// # No Pull
        None,
        /// # Pull-Up
        Up,
        /// # Pull-Down
        Down,
    };

    /// # Pin enable clock on port via RCC
    /// ---------------------------------------------------------
    /// Create new Pin on the specified port.  
    /// The default values are:  
    /// ```
    /// {
    ///     .pin = 0,
    ///     .mode = .Output,
    ///     .outtype = .PushPull,
    ///     .speed = .Low,
    ///     .pull = .None,
    ///     ._clk_en = false,
    /// }
    /// ```
    /// Args:
    /// > `port`: Port to use
    /// > `pin`: pin to use (0 <= `pin` <= 15)
    /// Return:
    /// > `Pin`: returns the `Pin` object.
    pub fn new(port: gpio_t, pin: u32) Self {
        return Self{ .port = port, .pin = pin };
    }

    /// # Pin enable clock on port via RCC
    /// ---------------------------------------------------------
    /// Enables the clock on the pin.  
    /// ---------------------------------------------------------
    /// Args:
    /// > `self`: self
    /// Return:
    /// > `void`
    pub fn clock_enable(self: *Self) void {
        if (self._clk_en) {
            return;
        }
        switch (self.port.port) {
            .A => self.port.rcc.enable_gpioa(),
            .B => self.port.rcc.enable_gpiob(),
            .C => self.port.rcc.enable_gpioc(),
            .D => self.port.rcc.enable_gpiod(),
            .E => self.port.rcc.enable_gpioe(),
            .H => self.port.rcc.enable_gpioh(),
        }
        self._clk_en = true;
    }

    /// # Pin configure pin speed
    /// ---------------------------------------------------------
    /// Configures the speed of the pin.  
    /// Possible modes: `Low`, `Medium`, `Fast`, `High`.
    /// ---------------------------------------------------------
    /// Args:
    /// > `self`: self
    /// > `m`: `Speed`: Speed to configure
    /// Return:
    /// > `void`
    pub fn set_speed(self: *Self, spd: Speed) void {
        self.speed = spd;
        self.clock_enable();
        switch (spd) {
            .Low => {
                self.port.ospeedr.modify(.Clear, Bit[2 * self.pin] | Bit[2 * self.pin + 1]);
            },
            .Medium => {
                self.port.ospeedr.modify(.Clear, Bit[2 * self.pin] | Bit[2 * self.pin + 1]);
                self.port.ospeedr.modify(.Set, Bit[2 * self.pin]);
            },
            .Fast => {
                self.port.ospeedr.modify(.Clear, Bit[2 * self.pin] | Bit[2 * self.pin + 1]);
                self.port.ospeedr.modify(.Set, Bit[2 * self.pin + 1]);
            },
            .High => {
                self.port.ospeedr.modify(.Set, Bit[2 * self.pin] | Bit[2 * self.pin + 1]);
            },
        }
    }

    /// # Pin configure pull mode for output
    /// ---------------------------------------------------------
    /// Configures the pull of the pin.  
    /// Possible modes: `None`, `Up`, `Down`.
    /// ---------------------------------------------------------
    /// Args:
    /// > `self`: self
    /// > `m`: `Pull`: Pull to configure
    /// Return:
    /// > `void`
    pub fn set_pull(self: *Self, pupdr: Pull) void {
        self.pull = pupdr;
        self.clock_enable();
        switch (pupdr) {
            .None => {
                self.port.pupdr.modify(.Clear, Bit[2 * self.pin] | Bit[2 * self.pin + 1]);
            },
            .Up => {
                self.port.pupdr.modify(.Clear, Bit[2 * self.pin] | Bit[2 * self.pin + 1]);
                self.port.pupdr.modify(.Set, Bit[2 * self.pin]);
            },
            .Down => {
                self.port.pupdr.modify(.Clear, Bit[2 * self.pin] | Bit[2 * self.pin + 1]);
                self.port.pupdr.modify(.Set, Bit[2 * self.pin + 1]);
            },
        }
    }

    /// # Pin configure mode 
    /// ---------------------------------------------------------
    /// Configures the pin to selected mode.  
    /// Possible modes: `Output`, `Input`, `Alternate`, `Analog`.
    /// ---------------------------------------------------------
    /// Args:
    /// > `self`: self
    /// > `m`: `Mode`: Mode to configure
    /// Return:
    /// > `void`
    pub fn set_mode(self: *Self, m: Mode) void {
        self.mode = m;
        self.clock_enable();
        switch (m) {
            .Input => self.port.moder.modify(.Clear, Bit[2 * self.pin] | Bit[2 * self.pin + 1]),
            .Output => {
                self.port.moder.modify(.Clear, Bit[2 * self.pin] | Bit[2 * self.pin + 1]);
                self.port.moder.modify(.Set, Bit[2 * self.pin]);
            },
            .Alternate => {
                self.port.moder.modify(.Clear, Bit[2 * self.pin] | Bit[2 * self.pin + 1]);
                self.port.moder.modify(.Set, Bit[2 * self.pin + 1]);
            },
            .Analog => {
                self.port.moder.modify(.Set, Bit[2 * self.pin] | Bit[2 * self.pin + 1]);
            },
        }
    }

    /// # Pin configure as Output 
    /// ----------------------------------------------
    /// Configures the pin in **Output** mode
    /// ----------------------------------------------
    /// Args:
    /// > `self`: self
    /// Return:
    /// > `void`
    pub fn make_output(self: *Self) void {
        self.set_mode(.Output);
    }

    /// # Pin configure as Input
    /// ----------------------------------------------
    /// Configures the pin in **Input** mode
    /// ----------------------------------------------
    /// Args:
    /// > `self`: self
    /// Return:
    /// > `void`
    pub fn make_input(self: *Self) void {
        self.set_mode(.Input);
    }

    /// # Pin configure as analog
    /// ----------------------------------------------
    /// Configures the pin in **Analog** mode
    /// ----------------------------------------------
    /// Args:
    /// > `self`: self
    /// Return:
    /// > `void`
    pub fn make_analog(self: *Self) void {
        self.set_mode(.Analog);
    }

    /// # Pin configure as alternate function
    /// ----------------------------------------------
    /// Configures the pin as Alternate-function mode
    /// ----------------------------------------------
    /// Args:
    /// > `self`: self
    /// Return:
    /// > `void`
    pub fn make_alternate(self: *Self) void {
        self.set_mode(.Alternate);
    }

    /// # Pin write to pin 
    /// ----------------------------------------------
    /// Writes the specified value to the pin
    /// ----------------------------------------------
    /// Args:
    /// > `self`: self
    /// > `val`: `1` or `0` (1-bit value to write)
    /// Return:
    /// > `void`
    pub fn write(self: Self, val: u1) void {
        self.port.odr.modify(if (val == 1) .Set else .Clear, Bit[self.pin]);
    }

    /// # Pin digital read
    /// ----------------------------------------------
    /// Args:
    /// > `self`: self
    /// Return:
    /// > `bool`: `true` if pin is high, else `false`.
    pub fn read(self: Self) bool {
        return self.port.idr.read(Bit[self.pin]);
    }

    /// # Pin toggle
    /// ----------------------------------------------
    /// Toggles the pin's output value
    /// ----------------------------------------------
    /// Args:
    /// > `self`: self
    /// Return:
    /// > `void`
    pub fn toggle(self: Self) void {
        self.port.odr.xor(Bit[self.pin]);
    }
};

pub const periph = struct {
    pub const rcc = rcc_t{
        .cr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "CR")),
        .pllcfgr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "PLLCFGR")),
        .cfgr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "CFGR")),
        .cir = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "CIR")),
        .ahb1rstr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "AHB1RSTR")),
        .ahb2rstr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "AHB2RSTR")),
        .apb1rstr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "APB1RSTR")),
        .apb2rstr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "APB2RSTR")),
        .ahb1enr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "AHB1ENR")),
        .ahb2enr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "AHB2ENR")),
        .apb1enr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "APB1ENR")),
        .apb2enr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "APB2ENR")),
        .ahb1lpenr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "AHB1LPENR")),
        .ahb2lpenr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "AHB2LPENR")),
        .apb1lpenr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "APB1LPENR")),
        .apb2lpenr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "APB2LPENR")),
        .bdcr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "BDCR")),
        .csr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "CSR")),
        .sscgr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "SSCGR")),
        .plli2scfgr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "PLLI2SCFGR")),
        .dckcfgr = Reg.new(@ptrToInt(RCC) + @offsetOf(RCC_t, "DCKCFGR")),
    };
    pub const gpioa = gpio_t{
        .moder = Reg.new(@ptrToInt(GPIOA) + @offsetOf(GPIO_t, "MODER")),
        .otyper = Reg.new(@ptrToInt(GPIOA) + @offsetOf(GPIO_t, "OTYPER")),
        .ospeedr = Reg.new(@ptrToInt(GPIOA) + @offsetOf(GPIO_t, "OSPEEDR")),
        .pupdr = Reg.new(@ptrToInt(GPIOA) + @offsetOf(GPIO_t, "PUPDR")),
        .idr = Reg.new(@ptrToInt(GPIOA) + @offsetOf(GPIO_t, "IDR")),
        .odr = Reg.new(@ptrToInt(GPIOA) + @offsetOf(GPIO_t, "ODR")),
        .bsrr = Reg.new(@ptrToInt(GPIOA) + @offsetOf(GPIO_t, "BSRR")),
        .lckr = Reg.new(@ptrToInt(GPIOA) + @offsetOf(GPIO_t, "BSRR")),
        .afrl = Reg.new(@ptrToInt(GPIOA) + @offsetOf(GPIO_t, "AFRL")),
        .afrh = Reg.new(@ptrToInt(GPIOA) + @offsetOf(GPIO_t, "AFRH")),
        .rcc = &rcc,
        .port = .A,
    };

    pub const gpiob = gpio_t{
        .moder = Reg.new(@ptrToInt(GPIOB) + @offsetOf(GPIO_t, "MODER")),
        .otyper = Reg.new(@ptrToInt(GPIOB) + @offsetOf(GPIO_t, "OTYPER")),
        .ospeedr = Reg.new(@ptrToInt(GPIOB) + @offsetOf(GPIO_t, "OSPEEDR")),
        .pupdr = Reg.new(@ptrToInt(GPIOB) + @offsetOf(GPIO_t, "PUPDR")),
        .idr = Reg.new(@ptrToInt(GPIOB) + @offsetOf(GPIO_t, "IDR")),
        .odr = Reg.new(@ptrToInt(GPIOB) + @offsetOf(GPIO_t, "ODR")),
        .bsrr = Reg.new(@ptrToInt(GPIOB) + @offsetOf(GPIO_t, "BSRR")),
        .lckr = Reg.new(@ptrToInt(GPIOB) + @offsetOf(GPIO_t, "BSRR")),
        .afrl = Reg.new(@ptrToInt(GPIOB) + @offsetOf(GPIO_t, "AFRL")),
        .afrh = Reg.new(@ptrToInt(GPIOB) + @offsetOf(GPIO_t, "AFRH")),
        .rcc = &rcc,
        .port = .B,
    };
};
