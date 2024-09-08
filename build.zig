const std = @import("std");

// Although this function looks imperative, note that its job is to
// declaratively construct a build graph that will be executed by an external
// runner.
pub fn build(b: *std.Build) void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});

    const build_options = b.addOptions();

    build_options.addOption(bool, "z80_sim_q", b.option(
        bool,
        "Z80_SIM_Q",
        "if true, simulate the Z80's Q undocumented behavior.",
    ) orelse true);

    const build_options_mod = build_options.createModule();

    const exe = b.addExecutable(.{
        .name = "zig80",
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });

    const waf = b.addWriteFiles();
    waf.addCopyFileToSource(exe.getEmittedAsm(), "./zig-out/main.asm");
    waf.step.dependOn(&exe.step);
    b.getInstallStep().dependOn(&waf.step);

    const exe2 = b.addExecutable(.{
        .name = "zig80",
        .root_source_file = b.path("src/test_single_steps.zig"),
        .target = target,
        .optimize = optimize,
    });
    exe2.root_module.addImport("build_options", build_options_mod);

    const exe3 = b.addExecutable(.{
        .name = "zig80",
        .root_source_file = b.path("src/test_z80_test_suite.zig"),
        .target = target,
        .optimize = optimize,
    });
    exe3.root_module.addImport("build_options", build_options_mod);

    const exe4 = b.addExecutable(.{
        .name = "debugger",
        .root_source_file = b.path("src/debugger.zig"),
        .target = target,
        .optimize = optimize,
    });
    exe4.root_module.addImport("build_options", build_options_mod);

    const sdl_dep = b.lazyDependency("sdl", .{
        .optimize = .ReleaseFast,
        .target = target,
    });

    const sdl_artifact = sdl_dep.?.artifact("SDL2");
    exe4.linkLibrary(sdl_artifact);

    var freetype_dep = b.dependency("freetype", .{
        .target = target,
        .optimize = .ReleaseFast,
    });
    const freetype_lib = freetype_dep.artifact("freetype");
    exe4.root_module.linkLibrary(freetype_lib);
    exe4.root_module.addImport("freetype", freetype_dep.module("freetype"));

    // add imgui
    {
        const imgui_enable_freetype = true;
        var imgui_dep = b.dependency("imgui", .{
            .target = target,
            .optimize = .ReleaseFast,
            .enable_freetype = imgui_enable_freetype,
        });
        const imgui_lib = imgui_dep.artifact("imgui");
        exe4.root_module.linkLibrary(imgui_lib);
        exe4.root_module.addImport("imgui", imgui_dep.module("imgui"));

        // Add <ft2build.h> to ImGui so it can compile with Freetype support
        if (imgui_enable_freetype) {
            for (freetype_lib.root_module.include_dirs.items) |freetype_include_dir| {
                switch (freetype_include_dir) {
                    .path => |p| imgui_lib.addIncludePath(p),
                    else => {}, // std.debug.panic("unhandled path from Freeytpe: {s}", .{@tagName(freetype_include_dir)}),
                }
            }
        }

        // Add <SDL.h> to ImGui so it can compile with Freetype support
        for (sdl_artifact.root_module.include_dirs.items) |sdl_include_dir| {
            switch (sdl_include_dir) {
                .path => |p| {
                    std.debug.print("SDL-INCLUDE: {any}\n", .{p});
                    imgui_lib.addIncludePath(p);
                },
                else => {}, // std.debug.panic("unhandled path from Freeytpe: {s}", .{@tagName(freetype_include_dir)}),
            }
        }
    }

    // This declares intent for the executable to be installed into the
    // standard location when the user invokes the "install" step (the default
    // step when running `zig build`).
    b.installArtifact(exe4);

    // This *creates* a Run step in the build graph, to be executed when another
    // step is evaluated that depends on it. The next line below will establish
    // such a dependency.
    const run_cmd = b.addRunArtifact(exe);
    const run_single_step_tests_cmd = b.addRunArtifact(exe2);
    const run_z80_test_suite_cmd = b.addRunArtifact(exe3);
    const run_debugger_cmd = b.addRunArtifact(exe4);

    // By making the run step depend on the install step, it will be run from the
    // installation directory rather than directly from within the cache directory.
    // This is not necessary, however, if the application depends on other installed
    // files, this ensures they will be present and in the expected location.
    run_cmd.step.dependOn(b.getInstallStep());

    // This allows the user to pass arguments to the application in the build
    // command itself, like this: `zig build run -- arg1 arg2 etc`
    if (b.args) |args| {
        run_cmd.addArgs(args);
        run_z80_test_suite_cmd.addArgs(args);
        run_single_step_tests_cmd.addArgs(args);
        run_debugger_cmd.addArgs(args);
    }

    // This creates a build step. It will be visible in the `zig build --help` menu,
    // and can be selected like this: `zig build run`
    // This will evaluate the `run` step rather than the default, which is "install".
    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    const run_step2 = b.step("run-single-step-test", "Run the Z80 SingleStepTests suite");
    run_step2.dependOn(&run_single_step_tests_cmd.step);

    const run_step3 = b.step("run-z80-test-suite", "Run the Z80 Test suite");
    run_step3.dependOn(&run_z80_test_suite_cmd.step);

    const run_step4 = b.step("run-debugger", "Run the Debug environment (SDL)");
    run_step4.dependOn(&run_debugger_cmd.step);

    // Creates a step for unit testing. This only builds the test executable
    // but does not run it.
    const lib_unit_tests = b.addTest(.{
        .root_source_file = b.path("src/test.zig"),
        .target = target,
        .optimize = optimize,
    });

    lib_unit_tests.root_module.addImport("build_options", build_options_mod);

    const run_lib_unit_tests = b.addRunArtifact(lib_unit_tests);

    const exe_unit_tests = b.addTest(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    exe_unit_tests.root_module.addImport("build_options", build_options_mod);
    const run_exe_unit_tests = b.addRunArtifact(exe_unit_tests);

    // Similar to creating the run step earlier, this exposes a `test` step to
    // the `zig build --help` menu, providing a way for the user to request
    // running the unit tests.
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_lib_unit_tests.step);
    test_step.dependOn(&run_exe_unit_tests.step);
}
