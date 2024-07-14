const std = @import("std");
const cpu = @import("cpu.zig");
const opcodes = @import("opcodes.zig");
const StdInOut = @import("test.zig").StdInOut;

const TestPlatform = enum {
    /// ZX Spectrum
    spectrum,
    /// CP/M
    cpm,
};

const TestFormat = enum {
    none,
    rak,
};

const TestRom = struct {
    /// Test title
    title: []const u8,
    /// The author(s) of the test suite
    author: []const u8,
    /// The path to the test ROM
    file_path: []const u8,
    /// The test code start address
    start_addr: u16,
    /// Code start address
    code_addr: u16,
    /// The size of the code section in bytes
    code_size: u16,
    /// The test system (for I/O, e.g print and stuff)
    platform: TestPlatform,
    /// The test format
    format: TestFormat = .none,
};

const test_suite = [_]TestRom{
    TestRom{ .title = "Preliminary tests (yaze-1.14)", .author = "Frank D. Cringle (2004)", .file_path = "prelim.com", .start_addr = 0x0100, .code_addr = 0, .code_size = 1280, .platform = .cpm },
    TestRom{ .title = "Z80 instruction exerciser (doc) (yaze-1.14)", .author = "Frank D. Cringle (2004)", .file_path = "zexdoc.com", .start_addr = 0x0100, .code_addr = 0, .code_size = 8588, .platform = .cpm },
    TestRom{ .title = "Z80 instruction exerciser (all) (yaze-1.14)", .author = "Frank D. Cringle (2004)", .file_path = "zexall.com", .start_addr = 0x0100, .code_addr = 0, .code_size = 8588, .platform = .cpm },
    TestRom{ .title = "Diagnostics II V1.2 CPU Test (1981)", .author = "Supersoft Associates", .file_path = "CPUTEST.COM", .start_addr = 0x0100, .code_addr = 0, .code_size = 19200, .platform = .cpm }, // https://gitlab.com/retroabandon/cpm-re/-/tree/main/ss-cpudiag
    TestRom{ .title = "Z80 Test suite (1.2a)", .author = "Patrik Rak", .file_path = "z80doc.tap", .start_addr = 0x8000, .code_addr = 91, .code_size = 14298, .platform = .spectrum, .format = .rak },
};

// see: http://www.gaby.de/cpm/manuals/archive/cpm22htm/ch5.htm
fn cpm_bdos(s: *cpu.Z80State, out: *[255:36]u8, idx: *usize, stdout: *const std.fs.File) !void {
    const writer = stdout.writer();

    switch (s.BC.low) {
        2 => { // Console Output
            const char = s.DE.low;
            if (char == 0xa) {
                out[idx.*] = 0x0;
                try writer.writeByte(0xa);
                idx.* = 0;
            } else if (idx.* < 255) {
                out[idx.*] = char; // E
                try writer.writeByte(char);
                idx.* = idx.* + 1;
            }
        },
        9 => { // Print string
            var addr = s.DE.getValue();
            while (true) : (addr +%= 1) {
                const char = s.memRead(addr);
                // std.debug.print("{c} ({d})\n", .{ char, char });

                if (char == 0xa) {
                    out[idx.*] = 0x0;
                    try writer.writeByte(0xa);
                    idx.* = 0;
                } else if (idx.* < 255 and char != '$') {
                    out[idx.*] = char; // (DE)
                    try writer.writeByte(char);
                    idx.* = idx.* + 1;
                } else {
                    break;
                }
            }
        },
        else => |syscall| {
            std.debug.print(
                "Unhandled CP/M system call: {d}\n",
                .{syscall},
            );
            return;
        },
    }

    // fake return from the syscall
    const fakeOpcode: opcodes.OpCode = @bitCast(@as(u8, 0));
    _ = opcodes.ret(s, &fakeOpcode);
}

fn run_test(test_rom: TestRom) !void {
    const allocator = std.heap.page_allocator;

    const base_path = try std.fs.cwd().realpathAlloc(allocator, ".//tests//roms");
    defer allocator.free(base_path);

    const full_path = try std.fs.path.join(allocator, &[_][]const u8{ base_path, test_rom.file_path });
    defer allocator.free(full_path);

    std.debug.print("Loading test rom: {s}\n", .{full_path});

    const file = try std.fs.openFileAbsolute(
        full_path,
        .{
            .mode = std.fs.File.OpenMode.read_only,
        },
    );
    defer file.close();

    var io = StdInOut{
        .memory = std.mem.zeroes([64 * 1024]u8),
        .ports = std.mem.zeroes([256]u8),
        .buff = std.mem.zeroes([256]u8),
    };

    var out: [255:36]u8 = std.mem.zeroes([255:36]u8); // 36 = $-terminated
    var idx: usize = 0;
    var spectrum_tab: u8 = 2;

    const stdout = &std.io.getStdOut();
    defer stdout.close();
    const writer = stdout.writer();

    var s = try cpu.Z80State.init(
        std.heap.page_allocator,
        &io,
        StdInOut.read,
        StdInOut.write,
        StdInOut.ioRead,
        StdInOut.ioWrite,
    );
    defer s.deinit(std.heap.page_allocator);

    const rom = try file.reader().readAllAlloc(allocator, 1024 * 1024); // 1024KB
    defer allocator.free(rom);
    const program = rom[test_rom.code_addr..test_rom.code_size];

    std.debug.print("Program len: {d}\n", .{program.len});

    s.PC = test_rom.start_addr;

    for (program, test_rom.start_addr..) |opcode, i| {
        io.memory[i] = opcode;
    }

    switch (test_rom.platform) {
        .spectrum => {
            s.I = 0x3f;
            s.IM = 1;
        },
        else => {},
    }

    switch (test_rom.format) {
        .rak => {
            io.memory[0x7000] = 0xcd; // call nn
            io.memory[0x7001] = @truncate(test_rom.start_addr & 0xFF); // low
            io.memory[0x7002] = @truncate((test_rom.start_addr >> 8) & 0xFF); // high
            io.memory[0x7003] = 0x76; // halt
            s.PC = 0x7000;
            io.ports[0xFE] = 0xBF;
            std.debug.print(">>>> {d} == {d}\n\n", .{ s.ioRead(0x00fe), io.ports[0xFE] });
        },
        else => {},
    }

    while (true) {
        const opcode = opcodes.fetchOpcode(s);

        opcodes.exec(s, opcode);

        if (test_rom.platform == .cpm) {
            switch (s.PC) {
                0x0000 => break, // HALT
                5 => try cpm_bdos(s, &out, &idx, stdout), // BDOS syscall
                else => {},
            }
        } else if (test_rom.platform == .spectrum) {
            switch (s.PC) {
                0x0D6B => _ = opcodes.ret(s, &@bitCast(@as(u8, 0))), // CLS
                0x1601 => _ = opcodes.ret(s, &@bitCast(@as(u8, 0))), // CHAN_OPEN
                0x0010 => { // PRINT A CHARACTER' RESTART
                    const char = s.AF.A;
                    if (spectrum_tab == 0) {
                        switch (char) {
                            0x0D => { // CR
                                out[idx] = 0x0;
                                try writer.writeByte(0xa);
                                idx = 0;
                            },
                            0x17 => { // TAB
                                spectrum_tab = 2;
                            },
                            0x7f => { // ©
                                out[idx] = '©';
                                // try writer.writeByte('©');
                                idx = (idx + 1) % out.len;
                            },
                            else => {
                                if (char >= 32 and char < 127) {
                                    out[idx] = char;
                                    try writer.writeByte(char);
                                    idx = (idx + 1) % out.len;
                                }
                            },
                        }
                    } else { // TAB expansion
                        spectrum_tab -= 1;
                        if (spectrum_tab != 0) {
                            var c = char & (32 - 1);
                            while (c > 0) : (c -= 1) try writer.writeByte(' ');
                        }
                    }
                    _ = opcodes.ret(s, &@bitCast(@as(u8, 0)));
                },
                else => {},
            }
        }
    }
}

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    const allocator = arena.allocator();
    defer arena.deinit();

    var args = try std.process.argsWithAllocator(allocator);
    _ = args.skip(); // zig

    const arg = args.next();

    if (arg) |cmd| {
        if (std.mem.eql(u8, "list", cmd)) {
            for (test_suite, 0..) |test_, i| {
                std.debug.print("[{d}] - {s}\n", .{ i, test_.title });
            }
        } else if (std.mem.eql(u8, "run", cmd)) {
            if (args.next()) |test_idx_str| {
                const test_idx = try std.fmt.parseInt(usize, test_idx_str, 10);
                try run_test(test_suite[test_idx]);
            }
        } else {
            std.debug.print("Unknown cmd: {s}\n", .{cmd});
        }
    }
}
