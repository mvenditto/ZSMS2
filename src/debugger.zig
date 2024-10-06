/// The debugger environment used for development / debugging.
///
/// NOTE: This is probably inefficient, buggy and mostly duck-taped together, but it
/// is ok because it is not meant to be used for anything other than fixing broken/missing core functionality.
///
/// KNOWN BUGS / LIMITATIONS:
///   * [UI] windows does not auto-dock, use an imgui.ini fro a release zip
///   * [DEBUGGER] no ability to add breakpoints at runtime
///   * [DEBUGGER] no ability to "step over"
///   * [DEBUGGER] not tested (yet) on larger roms (> 32K)
///   * BIOS ROM not working as-is
///   * SEGA Mappare only
///   * and many more
///
const std = @import("std");
const cpu = @import("cpu.zig");
const opcodes = @import("opcodes.zig");
const cartridge = @import("cartridge.zig");
const Z80State = cpu.Z80State;
const vdp = @import("vdp.zig");
const vdp_m4 = @import("vdp.rendering.m4.zig");
const disasm = @import("disasm.zig");
const expectEqual = std.testing.expectEqual;

const c = @cImport({
    @cInclude("SDL2/SDL.h");
});

const imgui = @import("imgui");

fn dumpMemory(sms: *const SMS2) void {
    var offset: usize = 0x0000;
    std.debug.print("  Len={d}\n", .{64 * 1024});
    while (offset < 64 * 1024) : (offset += 16) {
        std.debug.print("{X:0>4}-{X:0>4} | ", .{ offset, offset + 15 });
        inline for (0..16) |j| {
            const byte = SMS2.read(sms, @as(u16, @intCast(offset + j)));
            std.debug.print("{X:0>2} ", .{byte});
        }
        std.debug.print("|", .{});
        inline for (0..16) |j| {
            const byte = SMS2.read(sms, @as(u16, @intCast(offset + j)));
            if (std.ascii.isPrint(byte)) {
                std.debug.print("{c}", .{byte});
            } else {
                std.debug.print(".", .{});
            }
        }
        std.debug.print("\n", .{});
    }
}

fn dumpVRAM(sio: *SMS2) void {
    const vram = sio.memory;
    var offset: usize = 0x0000;
    while (offset < vram.len) : (offset += 16) {
        std.debug.print("{X:0>4}-{X:0>4} | ", .{ offset, offset + 15 });
        inline for (0..16) |j| {
            const byte = vram[offset + j];
            std.debug.print("{X:0>2} ", .{byte});
        }
        std.debug.print("|", .{});
        inline for (0..16) |j| {
            const byte = vram[offset + j];
            if (std.ascii.isPrint(byte)) {
                std.debug.print("{c}", .{byte});
            } else {
                std.debug.print(".", .{});
            }
        }
        std.debug.print("\n", .{});
    }
}

fn dumpVDPTablesOffsets(sio: *SMS2) void {
    const satBaseAddr = sio.vdp.getSpriteAttrTableBaseAddress();

    std.debug.print("NameTable:            0x{X:0>4}\n", .{sio.vdp.getNameTableBaseAddress()});
    std.debug.print("SpriteAttributeTable: 0x{X:0>4}\n", .{satBaseAddr});
    std.debug.print("SpriteGenTable:       0x{X:0>4}\n", .{sio.vdp.getSpriteGenTableBaseAddress()});
    std.debug.print("Display Mode:         {d}\n", .{sio.vdp.display_lines});

    const sat = sio.vdp.vram[satBaseAddr .. satBaseAddr + 256];

    for (0..64) |i| {
        const y = sat[i]; // scanline

        if (sio.vdp.display_lines == 192 and y == 208) {
            std.debug.print("Sprite disable sentinel (y=208) hit at {d}\n", .{i});
            break;
        }

        std.debug.print("[{d}]: x={d} y={d} p={d}\n", .{
            i,
            sat[(i * 2) + 128],
            y + 1,
            sat[(i * 2) + 128 + 1],
        });
    }
}

/// The SMS2 system featuring:
/// * the Z80 `cpu`
/// * the `vdp`
/// * the default Sega Mapper logics
/// * a (**very**) rudimentary debugger
pub const SMS2 = struct {
    memory: [64 * 1024]u8, // 64K Z80 RAM
    sys_ram: [8 * 1024]u8, // 8K system RAM
    cartridge_sram: ?[]u8, // 8K-64K(?) SRAM
    rom: []u8 = undefined, // Cartridge ROM
    bios_rom: ?[]u8, // System BIOS ROM
    ports: [256]u8,
    cpu: *Z80State,
    vdp: *vdp.VDPState,

    rom_banks_num: u8,
    rom_bank_0: usize,
    rom_bank_1: usize,
    rom_bank_2: usize,
    /// The bank select bit chooses between the first 16KB bank of SRAM (0) and the second (1), but only for slot 2.
    sram_bank_select: u8,

    dbg_attached: bool = false,
    dbg_breakpoints: std.ArrayList(u16),
    dbg_curr_bp: ?u16 = undefined,
    dbg_curr_cmd: enum(u8) {
        none,
        step_in,
        @"continue",
        @"break",
    },

    allocator: *const std.mem.Allocator,

    const Self = @This();

    const BankSize = 1024 * 16; // 16K
    const SytemRamSizeMask = 0x1FFF; // 1023
    const BiosSizeMask = 0x1FFF; // 1023
    const BankSizeMask = 0x3fff; // 16383
    const RamBank0 = 1;
    const RamBank1 = 2;

    pub fn init(allocator: std.mem.Allocator) !*Self {
        const self = try allocator.create(Self);

        self.allocator = &allocator;

        self.memory = std.mem.zeroes([64 * 1024]u8);

        self.sys_ram = std.mem.zeroes([8 * 1024]u8);

        self.ports = std.mem.zeroes([256]u8);

        self.cartridge_sram = try allocator.alloc(u8, 32 * 1024);

        self.sram_bank_select = 0;

        // Debugger stuff
        self.dbg_attached = false;
        self.dbg_breakpoints = std.ArrayList(u16).init(allocator);
        self.dbg_curr_bp = null;
        self.dbg_curr_cmd = .none;

        self.cpu = try cpu.Z80State.init(
            allocator,
            self,
            SMS2.read,
            SMS2.write,
            SMS2.ioRead,
            SMS2.ioWrite,
            SMS2.dbg_before_exec,
            SMS2.dbg_after_exec,
        );

        self.vdp = try vdp.VDPState.init(allocator);

        return self;
    }

    pub fn deinit(self: *Self) void {
        self.vdp.deinit(self.allocator);
        self.dbg_breakpoints.deinit();
        self.allocator.free(self.cartridge_sram);
        self.cpu.deinit(self.allocator);
        self.allocator.destroy(self);
    }

    pub fn dbg_before_exec(ctx: *const anyopaque, opcode: u8) void {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));

        if (self.dbg_attached == false) {
            self.cpu.dbg_trap = false;
            return; // continue
        }

        if (self.dbg_curr_bp) |_| {
            switch (self.dbg_curr_cmd) {
                .step_in => {
                    self.cpu.dbg_trap = false;
                    self.dbg_curr_bp = self.cpu.PC;
                    std.debug.print("[step-in] {X:0>4}: {X:0>2}\n", .{ self.cpu.PC, opcode });
                },
                .@"continue" => {
                    self.cpu.dbg_trap = false;
                    self.dbg_curr_bp = null;
                },
                else => {},
            }
            return;
        } else {
            if (self.dbg_curr_cmd == .@"break") {
                self.cpu.dbg_trap = true;
                self.dbg_curr_bp = self.cpu.PC;
                std.debug.print("[break] {X:0>4}: {X:0>2}\n", .{ self.cpu.PC, opcode });
                return;
            }
        }

        for (self.dbg_breakpoints.items) |bp| {
            if (self.cpu.PC == bp) {
                self.dbg_curr_bp = bp;
                self.cpu.dbg_trap = true;
                return;
            }
        }
    }

    pub fn dbg_after_exec(ctx: *const anyopaque, _: u8) void {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));

        switch (self.dbg_curr_cmd) {
            .step_in => {
                self.cpu.dbg_trap = true;
            },
            .@"continue" => {
                self.cpu.dbg_trap = false;
                self.dbg_curr_bp = null;
            },
            else => {},
        }

        self.dbg_curr_cmd = .none;
    }

    // Sega Mapper memory maps:
    // 0-1023      | 0x0000-0x03ff | (1024)  | ROM (unpaged)
    // 1024-16383  | 0x0400-0x3fff | (15360) | ROM mapper slot 0
    // 16384-32767 | 0x4000-0x7fff | (16384) | ROM mapper slot 1
    // 32768-49151 | 0x8000-0xbfff | (16384) | ROM/RAM mapper slot 2
    // 49152-57343 | 0xc000-0xdfff | (8192)  | System RAM
    // 57344-65535 | 0xe000-0xffff | (8192)  | System RAM (mirror)
    pub fn read(ctx: *const anyopaque, address: u16) u8 {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));

        var value: u8 = 0xff;

        if (address < 0xc000) {
            const port3E = self.ports[0x3e]; // NOTE: Bits are active low: 1 = disable, 0 = enable.
            if (port3E & 0x48 == 0x48) { // bit 3 (BIOS ROM) and 7 (Cartridge slot) both NOT enabled
                value = 0xff;
            } else if (port3E & 0x40 == 0x40) { // BIOS mapped
                if (self.bios_rom.?.len == 0x2000) { // 8K
                    value = self.bios_rom.?[address & 0x1FFF];
                    // TODO: Korean BIOS ?
                }
            } else if (address < 0x0400) { // unpaged
                value = self.rom[address];
            } else if (address < 0x4000) { // ROM mapper slot 0
                value = self.rom[(self.rom_bank_0 * BankSize) + address];
            } else if (address < 0x8000) { // ROM mapper slot 1
                value = self.rom[(self.rom_bank_1 * BankSize) + (address & BankSizeMask)];
            } else {
                value = switch (self.sram_bank_select) {
                    0 => self.rom[(self.rom_bank_2 * BankSize) + (address & BankSizeMask)],
                    RamBank0 => self.cartridge_sram.?[(address & BankSizeMask) % self.cartridge_sram.?.len],
                    RamBank1 => self.cartridge_sram.?[(BankSize + (address & BankSizeMask)) % self.cartridge_sram.?.len],
                    else => self.sys_ram[address & SytemRamSizeMask],
                };
            }
        } else {
            value = self.sys_ram[address & SytemRamSizeMask];
        }
        return value;
    }

    pub fn write(ctx: *anyopaque, address: u16, value: u8) void {
        var self: *Self = @ptrCast(@constCast(@alignCast(ctx)));
        if (address >= 0xC000) {
            self.sys_ram[address & SytemRamSizeMask] = value;
        } else if (address >= 0x8000) {
            if (self.cartridge_sram) |sram| {
                switch (self.sram_bank_select) {
                    RamBank0 => sram[(address & BankSizeMask) % sram.len] = value,
                    RamBank1 => sram[(BankSize + (address & BankSizeMask)) % sram.len] = value,
                    else => {}, // std.debug.panic("Writing to cartridge RAM failed\n", .{}),
                }
            }
        }

        if (address >= 0xfffc) {
            switch (address) {
                0xfffc => {
                    if (value & 8 != 0) { // active slot 2
                        self.sram_bank_select = if (value & 4 != 0) RamBank0 else RamBank1; // 0 = 1
                    } else {
                        self.sram_bank_select = 0;
                    }
                },
                0xfffd => self.rom_bank_0 = value % self.rom_banks_num,
                0xfffe => self.rom_bank_1 = value % self.rom_banks_num,
                0xffff => self.rom_bank_2 = value % self.rom_banks_num,
                else => {},
            }
        }

        self.memory[address] = value;
    }

    pub fn ioRead(ctx: *const anyopaque, address: u16) u8 {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));
        const port: u8 = @truncate(address & 0xFF);
        var value: u8 = 0xff;

        switch (port) {
            inline 0x00...0x3F => value = 0xff,
            inline 0x40...0x7f => {
                if (port & 1 == 0) {
                    value = self.vdp.readVCounter();
                } else {
                    value = self.vdp.readHCounter();
                }
            },
            inline 0x80...0xbf => {
                if (port & 1 == 0) {
                    value = self.vdp.dataRead();
                } else {
                    value = self.vdp.ctrlRead();
                }
            },
            inline 0xc0...0xff => {
                if (self.ports[0x3e] & 0x4 != 0x0) { // I/O disable == true
                    return 0xff;
                }
                if (port & 1 == 0) {
                    value = self.ports[0xdc]; // I/O port A/B
                } else {
                    value = self.ports[0xdd]; // I/O port B/misc
                }
            },
        }

        return value;
    }

    pub fn ioWrite(ctx: *const anyopaque, address: u16, value: u8) void {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));
        const port: u8 = @truncate(address & 0xFF);

        switch (port) {
            inline 0x00...0x3F => {
                if (port & 1 == 0) {
                    self.ports[0x3e] = value; // Mem. control port
                } else {
                    self.ports[0x3f] = value; // I/O port control
                }
            },
            inline 0x40...0x7f => {
                // TODO: SN76489 PSG. Sound chip.
            },
            inline 0x80...0xbf => {
                if (port & 1 == 0) {
                    self.vdp.dataWrite(value);
                } else {
                    self.vdp.ctrlWrite(value);
                }
            },
            // inline 0xc0...0xff => {
            //     if (port & 1 == 0) {
            //         self.ports[0xdc] = value; // I/O port A/B
            //     } else {
            //         self.ports[0xdd] = value; // I/O port B/misc
            //     }
            // },
            // else => self.ports[port] = value,
            else => {},
        }
    }
};

pub fn main() !void {
    const allocator = std.heap.page_allocator;
    var args = try std.process.argsWithAllocator(allocator);
    _ = args.skip(); // zig
    const rom_path = args.next();

    if (rom_path) |path| {
        std.debug.print("Loading: {s}\n", .{path});
    } else {
        std.debug.print("Specify a ROM to test.", .{});
        return;
    }

    const file = try std.fs.openFileAbsolute(rom_path.?, .{ .mode = .read_only });
    const rom = try file.reader().readAllAlloc(allocator, 1024 * 1024); // 1024KB
    defer allocator.free(rom);

    const sio = try SMS2.init(allocator);
    sio.rom = rom;
    sio.rom_banks_num = @max(1, @as(u8, @intCast(rom.len / SMS2.BankSizeMask)));
    std.debug.print("Copied {d}KB of ROM to RAM ({d} banks).\n", .{ rom.len / 1024, sio.rom_banks_num });

    var offset: u32 = 0;
    var program = std.ArrayList(disasm.DecodedOpCode).init(allocator);
    defer program.deinit();

    if (args.next()) |arg2| {
        if (std.mem.eql(u8, arg2, "--dbg-attach")) {
            std.debug.print("Debugger enabled.\n", .{});
            sio.dbg_attached = true;
            try sio.dbg_breakpoints.append(0);
        }
    }

    if (args.next()) |arg3| {
        if (std.mem.eql(u8, arg3, "--bios")) {
            if (args.next()) |bios_path| {
                std.debug.print("Loading BIOS: {s}\n", .{bios_path});
                const bios_file = try std.fs.openFileAbsolute(bios_path, .{ .mode = .read_only });
                const bios_rom = try bios_file.reader().readAllAlloc(allocator, 1024 * 48);
                sio.bios_rom = try allocator.alloc(u8, bios_rom.len);
                std.mem.copyForwards(u8, sio.bios_rom.?, bios_rom);
                defer allocator.free(bios_rom);
                SMS2.ioWrite(sio, 0x3e, 0x40); // bios mapped
            }
        }
    }

    while (offset < sio.rom.len) { // bios_rom
        var decoded = try disasm.decode(sio.rom[offset..]); // bios_rom

        if (decoded.op == .nop) {
            offset += 1;
            continue;
        }

        decoded.offset = offset;
        try program.append(decoded);

        offset += decoded.seq_len;
    }

    // SMS2.ioWrite(sio, 0xdc, 0xff);
    // SMS2.ioWrite(sio, 0xdd, 0xff);
    sio.ports[0xdc] = 0xff;
    sio.ports[0xdd] = 0xff;
    SMS2.write(sio, 0xfffc, 0);
    SMS2.write(sio, 0xfffd, 0);
    SMS2.write(sio, 0xfffe, 1);
    SMS2.write(sio, 0xffff, 2);

    sio.rom = rom;

    sio.cpu.PC = 0x0000;
    sio.cpu.SP.setValue(0xdff0);

    if (c.SDL_Init(c.SDL_INIT_VIDEO) != 0) {
        c.SDL_Log("Unable to initialize SDL: %s", c.SDL_GetError());
        return error.SDLInitializationFailed;
    }
    defer c.SDL_Quit();

    const window = c.SDL_CreateWindow(
        "TEST-BENCH",
        c.SDL_WINDOWPOS_UNDEFINED,
        c.SDL_WINDOWPOS_UNDEFINED,
        1224,
        768,
        c.SDL_WINDOW_OPENGL | c.SDL_WINDOW_RESIZABLE,
    ) orelse
        {
        c.SDL_Log("Unable to create window: %s", c.SDL_GetError());
        return error.SDLInitializationFailed;
    };
    defer c.SDL_DestroyWindow(window);

    const renderer = c.SDL_CreateRenderer(window, -1, c.SDL_RENDERER_PRESENTVSYNC) orelse { // c.SDL_RENDERER_PRESENTVSYNC
        c.SDL_Log("Unable to create renderer: %s", c.SDL_GetError());
        return error.SDLInitializationFailed;
    };
    defer c.SDL_DestroyRenderer(renderer);

    const texture = c.SDL_CreateTexture(
        renderer,
        c.SDL_PIXELFORMAT_RGBA8888,
        c.SDL_TEXTUREACCESS_STREAMING,
        256,
        sio.vdp.display_lines,
    );
    defer c.SDL_DestroyTexture(texture);

    _ = c.SDL_SetTextureBlendMode(texture, c.SDL_BLENDMODE_NONE);

    const patterns_texture = c.SDL_CreateTexture(
        renderer,
        c.SDL_PIXELFORMAT_RGBA8888,
        c.SDL_TEXTUREACCESS_STREAMING,
        17 + (8 * 16),
        33 + (8 * 32),
    );
    defer c.SDL_DestroyTexture(patterns_texture);

    var quit = false;

    const font_data = @embedFile("fonts/CommitMonoZ-425-Regular.otf");
    const font_atlas: *imgui.ImFontAtlas = imgui.ImFontAtlas_ImFontAtlas();
    _ = imgui.ImFontAtlas_AddFontFromMemoryTTF(
        font_atlas,
        @constCast(@ptrCast(font_data[0..].ptr)),
        font_data.len,
        16,
        null,
        null,
    );

    const ctx = imgui.igCreateContext(font_atlas);
    defer imgui.igDestroyContext(ctx);

    _ = imgui.ImGui_ImplSDL2_InitForSDLRenderer(@ptrCast(window), @ptrCast(renderer));
    defer imgui.ImGui_ImplSDL2_Shutdown();

    _ = imgui.ImGui_ImplSDLRenderer2_Init(@ptrCast(renderer));
    defer imgui.ImGui_ImplSDLRenderer2_Shutdown();

    const io = imgui.igGetIO();
    io.*.ConfigFlags |= imgui.ImGuiConfigFlags_DockingEnable;

    const clear_color = (@as(u32, 255)) | (@as(u32, 85) << 8) | (@as(u32, 85) << 16) | (@as(u32, 85) << 24);

    const uv0 = imgui.ImVec2{ .x = 0, .y = 0 };
    const uv1 = imgui.ImVec2{ .x = 1, .y = 1 };
    const tint_col = imgui.ImVec4{ .w = 1, .x = 1, .y = 1, .z = 1 };
    const border_col = imgui.ImVec4{ .w = 0, .x = 0, .y = 0, .z = 0 };
    const im_red = (@as(u32, 255)) | (@as(u32, 0) << 8) | (@as(u32, 0) << 16) | (@as(u32, 255) << 24);
    const im_yellow = (@as(u32, 255)) | (@as(u32, 255) << 8) | (@as(u32, 0) << 16) | (@as(u32, 255) << 24);
    const im_yellow_vec4: imgui.ImVec4 = .{ .x = 1.0, .y = 1.0, .z = 0.0, .w = 1.0 }; // w = A

    // var elapsed: u32 = 0;

    const text_base_height = imgui.igGetTextLineHeightWithSpacing();
    const table_fmt_buf: [:0]u8 = @ptrCast(try allocator.alloc(u8, 32));
    defer allocator.free(table_fmt_buf);

    var sprite_palette: bool = false;

    var ticks_a = c.SDL_GetTicks();
    var ticks_b = c.SDL_GetTicks();
    var delta: u32 = 0;
    var second: u32 = 0;
    var frames: u32 = 0;

    while (!quit) {
        var event: c.SDL_Event = undefined;

        ticks_a = c.SDL_GetTicks();
        delta = ticks_a - ticks_b;

        second += delta;
        if (second >= 1000) {
            second = 0;
            frames = 0;
        }

        while (c.SDL_PollEvent(&event) != 0) {
            _ = imgui.ImGui_ImplSDL2_ProcessEvent(@ptrCast(&event));
            switch (event.type) {
                c.SDL_QUIT => {
                    quit = true;
                },
                c.SDL_KEYDOWN => { // a = 122, b = 120
                    switch (event.key.keysym.sym) {
                        1073741906 => sio.ports[0xdc] &= ~(@as(u8, 1) << 0), // UP
                        1073741905 => sio.ports[0xdc] &= ~(@as(u8, 1) << 1), // DOWN
                        1073741904 => sio.ports[0xdc] &= ~(@as(u8, 1) << 2), // LEFT
                        1073741903 => sio.ports[0xdc] &= ~(@as(u8, 1) << 3), // RIGHT
                        122 => sio.ports[0xdc] &= ~(@as(u8, 1) << 4),
                        120 => sio.ports[0xdc] &= ~(@as(u8, 1) << 5),
                        else => {},
                    }
                },
                c.SDL_KEYUP => {
                    switch (event.key.keysym.sym) {
                        1073741906 => sio.ports[0xdc] |= (@as(u8, 1) << 0), // UP
                        1073741905 => sio.ports[0xdc] |= (@as(u8, 1) << 1), // DOWN
                        1073741904 => sio.ports[0xdc] |= (@as(u8, 1) << 2), // LEFT
                        1073741903 => sio.ports[0xdc] |= (@as(u8, 1) << 3), // RIGHT
                        122 => sio.ports[0xdc] |= (@as(u8, 1) << 4), // A
                        120 => sio.ports[0xdc] |= (@as(u8, 1) << 5), // B
                        99 => dumpMemory(sio), // C
                        118 => dumpVDPTablesOffsets(sio), // V
                        103 => { // G
                            std.debug.print("stdout_col={d}, stdout_row={d}, menu_pos={d}, menu_pos_old={d},\n", .{
                                sio.memory[0xc810],
                                sio.memory[0xc811],
                                sio.memory[0xc816],
                                sio.memory[0xc817],
                            });
                        },
                        1073741892 => { // F11 - step in
                            sio.dbg_curr_cmd = .step_in;
                        },
                        1073741886 => { // F5 - continue
                            sio.dbg_curr_cmd = .@"continue";
                        },
                        1073741885 => { // F4 - break
                            sio.dbg_curr_cmd = .@"break";
                        },
                        else => {
                            std.debug.print("{d}\n", .{event.key.keysym.sym});
                        },
                    }
                },
                else => {},
            }
        }

        if (frames <= 60) {
            vdp_m4.renderFrame(sio.vdp, sio.cpu);
            frames += 1;
        }

        _ = c.SDL_SetRenderDrawColor(renderer, 64, 63, 64, 255);
        _ = c.SDL_RenderClear(renderer);

        { // Render screen to texture
            var w: c_int = 0;
            var h: c_int = 0;
            var format: u32 = 0;
            _ = c.SDL_QueryTexture(texture, &format, null, &w, &h);

            var pixels: [*c]u32 = undefined;
            var pitch: c_int = 0; // Len of 1 row in bytes

            if (c.SDL_LockTexture(texture, null, @ptrCast(&pixels), &pitch) != 0) {
                std.debug.panic("SDLError: {s}\n", .{c.SDL_GetError()});
            }

            for (0..256) |x| {
                for (0..sio.vdp.display_lines) |y| {
                    const vdp_col: vdp.VDPColor = @bitCast(sio.vdp.display_buffer[(y * 256) + x]);
                    const pixel_idx = y * @as(usize, @intCast(@divTrunc(pitch, 4))) + x;
                    const col = vdp_col.toRGB8();
                    const color = (@as(u32, 255)) | (@as(u32, col.b) << 8) | (@as(u32, col.g) << 16) | (@as(u32, col.r) << 24);
                    pixels[@as(usize, @intCast(pixel_idx))] = color;
                }
            }

            c.SDL_UnlockTexture(texture);
        }

        _ = c.SDL_RenderSetScale(renderer, io.*.DisplayFramebufferScale.x, io.*.DisplayFramebufferScale.y);

        { // ImGui
            imgui.ImGui_ImplSDLRenderer2_NewFrame();
            imgui.ImGui_ImplSDL2_NewFrame();
            imgui.igNewFrame();
            var show_window: bool = true;

            const frame_padding = imgui.ImGuiStyleVar_FramePadding;

            const viewport = imgui.igGetMainViewport();

            const dockspace_id = imgui.igGetID_Str("DockSpace");
            _ = imgui.igDockSpaceOverViewport(
                dockspace_id,
                viewport,
                imgui.ImGuiDockNodeFlags_PassthruCentralNode,
                null,
            );

            { // Window: Video
                _ = imgui.igBegin("Video", &show_window, imgui.ImGuiWindowFlags_None);

                const img_size = imgui.ImVec2{
                    .x = 256 * 2,
                    .y = @floatFromInt(@as(u32, sio.vdp.display_lines) * 2),
                };
                const win_size = imgui.ImVec2{
                    .x = img_size.x + frame_padding * @as(f32, 1.5),
                    .y = img_size.y + (14 + (frame_padding * 2)), // adjust for title bar heigth
                };
                imgui.igSetWindowSize_Vec2(win_size, imgui.ImGuiCond_Always);
                imgui.igImage(
                    @ptrCast(texture),
                    img_size,
                    uv0,
                    uv1,
                    tint_col,
                    border_col,
                );

                imgui.igEnd();
            }

            { // Window: Patterns (BG)
                _ = imgui.igBegin("VRAM", &show_window, imgui.ImGuiWindowFlags_None);
                _ = imgui.igCheckbox("Sprite palette", &sprite_palette);
                const img_size = imgui.ImVec2{ .x = 2 * (17 + (8 * 16)), .y = 2 * (33 + (8 * 32)) };
                const win_size = imgui.ImVec2{
                    .x = img_size.x + 16,
                    .y = img_size.y + (14 + (frame_padding * 2)), // adjust for title bar heigth
                };
                imgui.igSetWindowSize_Vec2(win_size, imgui.ImGuiCond_Always);

                { // Render VRAM patterns
                    var vi: usize = 0;
                    var px: usize = 1;
                    var py: usize = 1;

                    var w: c_int = 0;
                    var h: c_int = 0;
                    var format: u32 = 0;
                    _ = c.SDL_QueryTexture(patterns_texture, &format, null, &w, &h);

                    var pixels: [*c]u32 = undefined;
                    var pitch: c_int = 0; // Len of 1 row in bytes

                    if (c.SDL_LockTexture(patterns_texture, null, @ptrCast(&pixels), &pitch) != 0) {
                        std.debug.panic("SDLError: {s}\n", .{c.SDL_GetError()});
                    }

                    for (0..@as(usize, @intCast(w))) |x| {
                        for (0..@as(usize, @intCast(h))) |y| {
                            const pixel_idx = y * @as(usize, @intCast(@divTrunc(pitch, 4))) + x;
                            pixels[@as(usize, @intCast(pixel_idx))] = clear_color;
                        }
                    }

                    while (vi < sio.vdp.vram.len - 32) {
                        inline for (0..8) |i| {
                            inline for (0..8) |j| {
                                const pixel_idx = (py + i) * @as(usize, @intCast(@divTrunc(pitch, 4))) + (px + (7 - j));
                                const bp_0 = (sio.vdp.vram[vi + 0] >> j) & 1; // bitplane 0
                                const bp_1 = (sio.vdp.vram[vi + 1] >> j) & 1; // bitplane 1
                                const bp_2 = (sio.vdp.vram[vi + 2] >> j) & 1; // bitplane 2
                                const bp_3 = (sio.vdp.vram[vi + 3] >> j) & 1; // bitplane 3
                                const ij_color = bp_0 | (bp_1 << 1) | (bp_2 << 2) | (bp_3 << 3);
                                const vdp_col = sio.vdp.cram[ij_color + (@as(u8, @intFromBool(sprite_palette)) * 16)];
                                const col = vdp_col.toRGB8();
                                const color = (@as(u32, 255)) | (@as(u32, col.b) << 8) | (@as(u32, col.g) << 16) | (@as(u32, col.r) << 24);
                                pixels[@as(usize, @intCast(pixel_idx))] = color;
                            }
                            vi += 4;
                        }
                        px += 8 + 1;
                        if ((vi / 32) % 16 == 0) { // px >= 16 * 8
                            px = 1;
                            py += 8 + 1;
                        }
                    }

                    c.SDL_UnlockTexture(patterns_texture);
                }

                imgui.igImage(
                    @ptrCast(patterns_texture),
                    img_size,
                    uv0,
                    uv1,
                    tint_col,
                    border_col,
                );
                imgui.igEnd();
            }

            { // Window: Palette (CRAM)
                _ = imgui.igBegin("Palette", &show_window, imgui.ImGuiWindowFlags_None);
                const win_size = imgui.ImVec2{
                    .x = (16 * 32) + 78,
                    .y = 32 + (14 + (frame_padding * 2)), // adjust for title bar heigth (fontSize + frame_padding * 2)
                };
                imgui.igSetWindowSize_Vec2(win_size, imgui.ImGuiCond_Always);

                const draw_list = imgui.igGetWindowDrawList();
                var canvas_pos = imgui.ImVec2{};

                imgui.igGetCursorScreenPos(&canvas_pos);

                const bg_color = (@as(u32, 85)) | (@as(u32, 85) << 8) | (@as(u32, 85) << 16) | (@as(u32, 255) << 24);

                const font_size = imgui.igGetFontSize();

                imgui.ImDrawList_AddRectFilled(
                    draw_list,
                    imgui.ImVec2{ .x = canvas_pos.x - 2, .y = canvas_pos.y - 2 + font_size },
                    imgui.ImVec2{ .x = canvas_pos.x + ((16 * 32) + 64), .y = canvas_pos.y + 16 + 2 + font_size },
                    bg_color,
                    0,
                    imgui.ImDrawFlags_RoundCornersNone,
                );

                const buf = try std.heap.page_allocator.alloc(u8, 2);
                defer std.heap.page_allocator.free(buf);

                for (0..sio.vdp.cram.len) |ci| {
                    const col = sio.vdp.cram[ci].toRGB8();
                    const color = (@as(u32, col.r)) | (@as(u32, col.g) << 8) | (@as(u32, col.b) << 16) | (@as(u32, 255) << 24);
                    const gap_pad: f32 = @floatFromInt(ci);
                    const x = gap_pad * 16;

                    imgui.ImDrawList_AddRectFilled(
                        draw_list,
                        imgui.ImVec2{ .x = (2 * gap_pad) + x + canvas_pos.x, .y = canvas_pos.y + font_size },
                        imgui.ImVec2{ .x = (2 * gap_pad) + x + canvas_pos.x + 16, .y = canvas_pos.y + 16 + font_size },
                        color,
                        0,
                        imgui.ImDrawFlags_RoundCornersNone,
                    );

                    if (ci == 15 or ci == 31) {
                        const slice = try std.fmt.bufPrint(buf, "{d: <2}", .{ci});

                        imgui.ImDrawList_AddText_Vec2(
                            draw_list,
                            imgui.ImVec2{ .x = (2 * gap_pad) + x + canvas_pos.x, .y = canvas_pos.y - 2.5 },
                            0xff8c8c8c,
                            @ptrCast(slice),
                            null,
                        );
                    }
                }
                imgui.igEnd();
            }

            { // Window: FPS counter
                _ = imgui.igBegin("Timings", &show_window, imgui.ImGuiWindowFlags_None);
                imgui.igText("Frame time: %d ms.", delta);
                imgui.igText("FPS: %.2f", 1000.0 / @as(f32, @floatFromInt(delta)));
                imgui.igEnd();
            }

            { // Window: Debugger
                _ = imgui.igBegin("Debugger", &show_window, imgui.ImGuiWindowFlags_None);

                const table_flags = imgui.ImGuiTableFlags_ScrollY |
                    imgui.ImGuiTableFlags_RowBg |
                    imgui.ImGuiTableFlags_BordersOuter |
                    imgui.ImGuiTableFlags_BordersV |
                    imgui.ImGuiTableFlags_Resizable |
                    imgui.ImGuiTableFlags_Hideable;

                const outer_size = imgui.ImVec2{ .x = 0, .y = 0 }; // text_base_height * 100

                const draw_list = imgui.igGetWindowDrawList();

                const min_row_height = text_base_height * 2;

                if (imgui.igBeginTable(
                    "DebuggerTable",
                    4,
                    table_flags,
                    outer_size,
                    0.0,
                )) {
                    const table = imgui.igGetCurrentTable();
                    imgui.igTableSetupColumn("", imgui.ImGuiTableColumnFlags_WidthFixed, 8.0, 0);
                    imgui.igTableSetupColumn("Offset", imgui.ImGuiTableColumnFlags_WidthFixed, 64.0, 0);
                    imgui.igTableSetupColumn("Sequence", imgui.ImGuiTableColumnFlags_None, 64.0, 0);
                    imgui.igTableSetupColumn("Mnemonic", imgui.ImGuiTableColumnFlags_None, 64.0, 0);
                    imgui.igTableSetupScrollFreeze(0, 1);
                    imgui.igTableHeadersRow();

                    var clipper = imgui.ImGuiListClipper{};
                    imgui.ImGuiListClipper_Begin(&clipper, @intCast(program.items.len), min_row_height);

                    var active_bp_row: ?usize = null;

                    if (sio.dbg_curr_bp) |active_bp| {
                        for (program.items, 0..) |insn, row| {
                            if (active_bp == insn.offset) {
                                active_bp_row = row;
                                break;
                            }
                        }
                    }

                    while (imgui.ImGuiListClipper_Step(&clipper)) {
                        var row: usize = @intCast(clipper.DisplayStart);
                        var row_breakpoint = false;

                        while (row < clipper.DisplayEnd) : (row += 1) {
                            const insn = program.items[row];

                            imgui.igTableNextRow(imgui.ImGuiTableRowFlags_None, min_row_height);
                            if (imgui.igTableNextColumn()) {

                                // Debugger paused on this row
                                if (row == active_bp_row) {
                                    row_breakpoint = true;
                                } else {
                                    // We've a breakpoint at this row
                                    for (sio.dbg_breakpoints.items) |bp| {
                                        if (insn.offset == bp) {
                                            row_breakpoint = true;
                                            break;
                                        }
                                    }
                                }
                            }

                            if (imgui.igTableNextColumn()) {
                                imgui.igText("%04X", @as(u32, insn.offset));
                            }
                            if (imgui.igTableNextColumn()) {
                                const addr = insn.offset;
                                const seq_fmt = std.fmt.fmtSliceHexUpper(sio.rom[addr .. addr + insn.seq_len]); // bios_rom
                                const printed = try std.fmt.bufPrint(table_fmt_buf, "{s}", .{seq_fmt});
                                table_fmt_buf[printed.len] = "\x00"[0];
                                imgui.igText("%s", printed.ptr);
                            }
                            if (imgui.igTableNextColumn()) {
                                const mnemonic = try disasm.fmt_decoded_insn(table_fmt_buf, &insn);
                                imgui.igText("%s", mnemonic.ptr);

                                if (insn.op == .in) {
                                    if (insn.operand2) |op2| {
                                        switch (op2) {
                                            .reg_8 => |r8| {
                                                if (r8 == .C) {
                                                    imgui.igSameLine(0, 10);
                                                    imgui.igTextColored(im_yellow_vec4, "(0x%02X)", sio.cpu.BC.low);
                                                }
                                            },
                                            else => {},
                                        }
                                    }
                                }
                            }
                            if (row_breakpoint == true) {
                                if (imgui.igTableSetColumnIndex(0)) {
                                    if (active_bp_row) |active_row| {
                                        if (active_row == row) {
                                            var rect = imgui.ImRect{};
                                            imgui.igTableGetCellBgRect(&rect, table, 0);
                                            const cx = rect.Min.x + ((rect.Max.x - rect.Min.x) * 0.5);
                                            const cy = rect.Min.y + ((rect.Max.y - rect.Min.y) * 0.5);
                                            const arrow = [7]imgui.ImVec2{
                                                .{ .x = cx + 6, .y = cy },
                                                .{ .x = cx, .y = cy + 4 },
                                                .{ .x = cx - 6, .y = cy + 4 },
                                                .{ .x = cx - 6, .y = cy + 2 },
                                                .{ .x = cx - 6, .y = cy - 2 },
                                                .{ .x = cx - 6, .y = cy - 4 },
                                                .{ .x = cx, .y = cy - 4 },
                                            };
                                            imgui.ImDrawList_AddConvexPolyFilled(
                                                draw_list,
                                                &arrow,
                                                arrow.len,
                                                im_yellow,
                                            );
                                        }
                                    } else {
                                        var rect = imgui.ImRect{};
                                        imgui.igTableGetCellBgRect(&rect, table, 0);
                                        const cx = rect.Min.x + ((rect.Max.x - rect.Min.x) * 0.5);
                                        const cy = rect.Min.y + ((rect.Max.y - rect.Min.y) * 0.5);
                                        imgui.ImDrawList_AddCircleFilled(
                                            draw_list,
                                            imgui.ImVec2{ .x = cx, .y = cy },
                                            4,
                                            im_red,
                                            16,
                                        );
                                    }
                                }
                            }
                            row_breakpoint = false;
                        }
                    }
                    if (active_bp_row) |active_row| {
                        const scroll_to_row: f32 = @floatFromInt(active_row);
                        imgui.igSetScrollY_Float(clipper.ItemsHeight * scroll_to_row);
                    }
                    active_bp_row = null;
                    imgui.igEndTable();
                }
                imgui.igEnd();
            }

            { // CPU Registers
                _ = imgui.igBegin("CPU Registers", &show_window, imgui.ImGuiWindowFlags_None);

                const table_flags = imgui.ImGuiTableFlags_ScrollY |
                    imgui.ImGuiTableFlags_RowBg |
                    imgui.ImGuiTableFlags_BordersOuter |
                    imgui.ImGuiTableFlags_BordersV |
                    imgui.ImGuiTableFlags_Resizable |
                    imgui.ImGuiTableFlags_Hideable;

                const outer_size = imgui.ImVec2{ .x = 0, .y = 0 }; // text_base_height * 100

                const min_row_height = text_base_height * 2;

                if (imgui.igBeginTable(
                    "Registers",
                    3,
                    table_flags,
                    outer_size,
                    0.0,
                )) {
                    for (0..11) |ri| {
                        imgui.igTableNextRow(imgui.ImGuiTableRowFlags_None, min_row_height);
                        if (imgui.igTableSetColumnIndex(0)) {
                            if (ri < 8) {
                                const reg_val = sio.cpu.gp_registers[ri];
                                const reg_name = @tagName(@as(cpu.EightBitRegisters, @enumFromInt(ri)));
                                imgui.igTextColored(im_yellow_vec4, "%s", reg_name.ptr);
                                imgui.igSameLine(0, 10);
                                imgui.igText("0x%02X", reg_val);
                            }
                        }
                        if (imgui.igTableSetColumnIndex(1)) {
                            if (ri < 4) {
                                const reg_val = sio.cpu.gp_registers_pairs[ri];
                                const reg_name = @tagName(@as(cpu.RegisterPairs2, @enumFromInt(ri)));
                                imgui.igTextColored(im_yellow_vec4, "%s", reg_name.ptr);
                                imgui.igSameLine(0, 10);
                                imgui.igText("0x%04X", reg_val.getValue());
                            } else if (ri == 5) {
                                imgui.igTextColored(im_yellow_vec4, "SP");
                                imgui.igSameLine(0, 10);
                                imgui.igText("0x%04X", sio.cpu.SP.getValue());
                            } else if (ri == 6) {
                                imgui.igTextColored(im_yellow_vec4, "IX");
                                imgui.igSameLine(0, 10);
                                imgui.igText("0x%04X", sio.cpu.IX.getValue());
                            } else if (ri == 7) {
                                imgui.igTextColored(im_yellow_vec4, "IY");
                                imgui.igSameLine(0, 10);
                                imgui.igText("0x%04X", sio.cpu.IY.getValue());
                            } else if (ri == 9) {
                                imgui.igTextColored(im_yellow_vec4, "IFF1");
                                imgui.igSameLine(0, 10);
                                imgui.igText("%d", @as(u8, @intFromBool(sio.cpu.IFF1)));
                            } else if (ri == 10) {
                                imgui.igTextColored(im_yellow_vec4, "IFF2");
                                imgui.igSameLine(0, 10);
                                imgui.igText("%d", @as(u8, @intFromBool(sio.cpu.IFF2)));
                            }
                        }

                        if (imgui.igTableSetColumnIndex(2)) {
                            if (ri < 4) {
                                const rp_shadows: *[8]cpu.SixteenBitRegister = @ptrCast(sio.cpu);
                                const reg_val = rp_shadows[ri + 4];
                                const reg_name = @tagName(@as(cpu.RegisterPairs2, @enumFromInt(ri)));
                                imgui.igTextColored(im_yellow_vec4, "%s'", reg_name.ptr);
                                imgui.igSameLine(0, 10);
                                imgui.igText("0x%04X", reg_val.getValue());
                            } else if (ri == 6) {
                                imgui.igTextColored(im_yellow_vec4, "I");
                                imgui.igSameLine(0, 10);
                                imgui.igText("0x%02X", sio.cpu.I);
                            } else if (ri == 7) {
                                imgui.igTextColored(im_yellow_vec4, "R");
                                imgui.igSameLine(0, 10);
                                imgui.igText("0x%02X", sio.cpu.R.getValue());
                            } else if (ri == 9) {
                                imgui.igTextColored(im_yellow_vec4, "IM");
                                imgui.igSameLine(0, 10);
                                imgui.igText("%d", sio.cpu.IM);
                            }
                        }
                    }
                    imgui.igEndTable();
                }

                imgui.igEnd();
            }

            { // VDP Registers
                _ = imgui.igBegin("VDP Registers", &show_window, imgui.ImGuiWindowFlags_None);

                const table_flags = imgui.ImGuiTableFlags_ScrollY |
                    imgui.ImGuiTableFlags_RowBg |
                    imgui.ImGuiTableFlags_BordersOuter |
                    imgui.ImGuiTableFlags_BordersV |
                    imgui.ImGuiTableFlags_Resizable |
                    imgui.ImGuiTableFlags_Hideable;

                const outer_size = imgui.ImVec2{ .x = 0, .y = 0 }; // text_base_height * 100

                const min_row_height = text_base_height * 2;

                if (imgui.igBeginTable(
                    "VDPRegisters",
                    2,
                    table_flags,
                    outer_size,
                    0.0,
                )) {
                    imgui.igTableNextRow(imgui.ImGuiTableRowFlags_None, min_row_height);
                    if (imgui.igTableNextColumn()) {
                        imgui.igTextColored(im_yellow_vec4, "CTRL1");
                        imgui.igSameLine(0, 10);
                        const printed = try std.fmt.bufPrint(table_fmt_buf, "0b{b:0>8}", .{sio.vdp.registers[0]});
                        table_fmt_buf[printed.len] = "\x00"[0];
                        imgui.igText(printed.ptr);
                    }
                    imgui.igTableNextRow(imgui.ImGuiTableRowFlags_None, min_row_height);
                    if (imgui.igTableNextColumn()) {
                        imgui.igTextColored(im_yellow_vec4, "CTRL2");
                        imgui.igSameLine(0, 10);
                        const printed = try std.fmt.bufPrint(table_fmt_buf, "0b{b:0>8}", .{sio.vdp.registers[1]});
                        table_fmt_buf[printed.len] = "\x00"[0];
                        imgui.igText(printed.ptr);
                    }
                    imgui.igEndTable();
                }

                imgui.igEnd();
            }

            { // I/O Ports
                _ = imgui.igBegin("I/O Ports", &show_window, imgui.ImGuiWindowFlags_None);

                const table_flags = imgui.ImGuiTableFlags_ScrollY |
                    imgui.ImGuiTableFlags_RowBg |
                    imgui.ImGuiTableFlags_BordersOuter |
                    imgui.ImGuiTableFlags_BordersV |
                    imgui.ImGuiTableFlags_Resizable |
                    imgui.ImGuiTableFlags_Hideable;

                const outer_size = imgui.ImVec2{ .x = 0, .y = 0 }; // text_base_height * 100

                const min_row_height = text_base_height * 2;

                if (imgui.igBeginTable(
                    "IOPorts",
                    2,
                    table_flags,
                    outer_size,
                    0.0,
                )) {
                    imgui.igTableNextRow(imgui.ImGuiTableRowFlags_None, min_row_height);
                    if (imgui.igTableNextColumn()) {
                        imgui.igTextColored(im_yellow_vec4, "0x3e");
                        imgui.igSameLine(0, 10);
                        const printed = try std.fmt.bufPrint(table_fmt_buf, "0b{b:0>8}", .{sio.ports[0x3e]});
                        table_fmt_buf[printed.len] = "\x00"[0];
                        imgui.igText(printed.ptr);
                    }
                    imgui.igTableNextRow(imgui.ImGuiTableRowFlags_None, min_row_height);
                    if (imgui.igTableNextColumn()) {
                        imgui.igTextColored(im_yellow_vec4, "0x3f");
                        imgui.igSameLine(0, 10);
                        const printed = try std.fmt.bufPrint(table_fmt_buf, "0b{b:0>8}", .{sio.ports[0x3f]});
                        table_fmt_buf[printed.len] = "\x00"[0];
                        imgui.igText(printed.ptr);
                    }
                    imgui.igTableNextRow(imgui.ImGuiTableRowFlags_None, min_row_height);
                    if (imgui.igTableNextColumn()) {
                        imgui.igTextColored(im_yellow_vec4, "0xdc");
                        imgui.igSameLine(0, 10);
                        const printed = try std.fmt.bufPrint(table_fmt_buf, "0b{b:0>8}", .{sio.ports[0xdc]});
                        table_fmt_buf[printed.len] = "\x00"[0];
                        imgui.igText(printed.ptr);
                    }
                    imgui.igTableNextRow(imgui.ImGuiTableRowFlags_None, min_row_height);
                    if (imgui.igTableNextColumn()) {
                        imgui.igTextColored(im_yellow_vec4, "0xdd");
                        imgui.igSameLine(0, 10);
                        const printed = try std.fmt.bufPrint(table_fmt_buf, "0b{b:0>8}", .{sio.ports[0xdd]});
                        table_fmt_buf[printed.len] = "\x00"[0];
                        imgui.igText(printed.ptr);
                    }
                    imgui.igEndTable();
                }

                imgui.igEnd();
            }

            imgui.igRender();
            imgui.ImGui_ImplSDLRenderer2_RenderDrawData(@ptrCast(imgui.igGetDrawData()), @ptrCast(renderer));
        }

        c.SDL_RenderPresent(renderer);

        ticks_b = ticks_a;
    }
}
