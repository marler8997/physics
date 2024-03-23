const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const zigwin32_dep = b.dependency("zigwin32", .{});

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
}
