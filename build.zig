const std = @import("std");
const Builder = std.build.Builder;

const microzig = @import("libs/microzig/src/main.zig");
const chips = microzig.chips;
const cpus = microzig.cpus;
const Backing = microzig.Backing;
const Chip = microzig.Chip;
const EmbeddedExecutable = microzig.EmbeddedExecutable;

const stm32f411 = Chip{
    .name = "STM32F411",
    .path = "src/stm32f411.zig",
    .cpu = cpus.cortex_m4,
    .memory_regions = &.{
        .{ .offset = 0x08000000, .length = (512 * 1024), .kind = .flash },
        .{ .offset = 0x20000000, .length = (128 * 1024), .kind = .ram },
    },
};

pub fn build(b: *Builder) !void {
    const backing = .{
        .chip = stm32f411,
    };

    const release_mode = b.standardReleaseOptions();

    var exe = microzig.addEmbeddedExecutable(
        b,
        "firmware",
        "src/main.zig",
        backing,
        .{},
    );

    exe.setBuildMode(release_mode);
    exe.install();

    const bin = b.addInstallRaw(exe.inner, "firmware.bin", .{});
    b.getInstallStep().dependOn(&bin.step);

    const flash_cmd = b.addSystemCommand(&[_][]const u8{
        "pyocd",
        "load",
        b.getInstallPath(bin.dest_dir, bin.dest_filename),
        "-a",
        "0x8000000",
        "--target",
        "stm32f411ceux",
    });
    const size = b.addSystemCommand(&[_][]const u8{
        "arm-none-eabi-size",
        "./zig-out/bin/firmware.elf",
    });
    flash_cmd.step.dependOn(&bin.step);
    const flash_step = b.step("flash", "Build, flash and run the binary on your STM32F411CEUx");
    flash_step.dependOn(&flash_cmd.step);
    flash_step.dependOn(&size.step);
}
