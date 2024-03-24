const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const zigwin32_dep = b.dependency("zigwin32", .{});
    const zmath = b.dependency("zmath", .{});

    const root_source_path: []const u8 = switch (target.result.os.tag) {
        .windows => "win32.zig",
        else => {
            std.log.err("unsupported target os {s}", .{@tagName(target.result.os.tag)});
            std.os.exit(0xff);
        },
    };
    const exe = b.addExecutable(.{
        .name = "physics",
        .root_source_file = .{ .path = root_source_path },
        .target = target,
        .optimize = optimize,
    });
    exe.root_module.addImport("zmath", zmath.module("root"));
    if (target.result.os.tag == .windows) {
        exe.subsystem = .Windows;
        exe.root_module.addImport("win32", zigwin32_dep.module("zigwin32"));
        exe.addWin32ResourceFile(.{
            .file = .{ .path = "res/physics.rc" },
        });
    }

    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }
    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    {
        const exe_unit_tests = b.addTest(.{
            .root_source_file = .{ .path = "physics.zig" },
            .target = target,
            .optimize = optimize,
        });
        exe_unit_tests.root_module.addImport("zmath", zmath.module("root"));
        const run_exe_unit_tests = b.addRunArtifact(exe_unit_tests);
        const test_step = b.step("test", "Run unit tests");
        test_step.dependOn(&run_exe_unit_tests.step);
    }
}
