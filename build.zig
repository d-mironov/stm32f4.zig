const Builder = @import("std").build.Builder;
const builtin = @import("builtin");
const std = @import("std");

pub fn build(b: *Builder) void {
    // Target STM32F407VG
    const target = .{
        .cpu_arch = .thumb,
        .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 },
        .os_tag = .freestanding,
        .abi = .eabihf,
    };

    // Standard release options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall.
    const mode = b.standardReleaseOptions();

    const elf = b.addExecutable("firmware.elf", "src/startup.zig");
    elf.setTarget(target);
    elf.setBuildMode(mode);

    const vector_obj = b.addObject("vector", "src/vector.zig");
    vector_obj.setTarget(target);
    vector_obj.setBuildMode(mode);

    elf.addObject(vector_obj);
    elf.setLinkerScriptPath(.{ .path = "src/stm32f4.ld" });

    const bin = b.addInstallRaw(elf, "firmware.bin", .{});
    const bin_step = b.step("bin", "Generate binary file to be flashed");
    bin_step.dependOn(&bin.step);

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

    b.default_step.dependOn(&elf.step);
    b.installArtifact(elf);
}
