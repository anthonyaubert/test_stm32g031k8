const std = @import("std");
const Builder = @import("std").build.Builder;
const builtin = @import("builtin");

pub fn build(b: *Builder) !void {

    const target = std.zig.CrossTarget{
        .cpu_arch = .thumb,
        .os_tag = .freestanding,
        .abi = .none,
        .cpu_model = std.zig.CrossTarget.CpuModel{ .explicit = &std.Target.arm.cpu.cortex_m0plus },
    };

    const build_elf = b.addExecutable("firmware.elf", "startup_stm32g031k8tx.zig");
    build_elf.emit_asm = true;
    build_elf.setBuildMode(b.standardReleaseOptions());
    build_elf.setLinkerScriptPath("arm_cm0.ld");
    build_elf.setTarget(target);
    build_elf.link_function_sections = true;
    build_elf.install();

    const main_o = b.addObject("main", "main.zig");
    main_o.setTarget(target);
    build_elf.addObject(main_o);

    const run_cmd = build_elf.run();
    run_cmd.step.dependOn(b.getInstallStep());

    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);
}
