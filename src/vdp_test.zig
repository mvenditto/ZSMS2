const std = @import("std");
const cpu = @import("cpu.zig");
const opcodes = @import("opcodes.zig");
const cartridge = @import("cartridge.zig");
const Z80State = cpu.Z80State;
const vdp = @import("vdp.zig");
const vdp_m4 = @import("vdp.rendering.m4.zig");
const expectEqual = std.testing.expectEqual;

const c = @cImport({
    @cInclude("SDL2/SDL.h");
});

fn dumpVRAM(sio: *SystemIO) void {
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

pub const SystemIO = struct {
    memory: [64 * 1024]u8,
    ports: [256]u8,
    cpu: *Z80State,
    vdp: *vdp.VDPState,

    const Self = @This();

    pub fn init() !*Self {
        const self = try std.heap.page_allocator.create(Self);

        self.memory = std.mem.zeroes([64 * 1024]u8);

        self.ports = std.mem.zeroes([256]u8);

        self.cpu = try cpu.Z80State.init(
            std.heap.page_allocator,
            self,
            SystemIO.read,
            SystemIO.write,
            SystemIO.ioRead,
            SystemIO.ioWrite,
        );

        self.vdp = try vdp.VDPState.init(std.heap.page_allocator);

        return self;
    }

    pub fn deinit(self: *Self) void {
        self.vdp.deinit(std.heap.page_allocator);
        std.heap.page_allocator.destroy(self);
    }

    pub fn read(ctx: *const anyopaque, address: u16) u8 {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));
        return self.memory[address];
    }

    pub fn write(ctx: *anyopaque, address: u16, value: u8) void {
        var self: *Self = @ptrCast(@constCast(@alignCast(ctx)));
        self.memory[address] = value;
    }

    pub fn ioRead(ctx: *const anyopaque, address: u16) u8 {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));
        const port: u8 = @truncate(address & 0xFF);
        return switch (port) {
            vdp.vdp_v_counter_port => self.vdp.readVCounter(),
            vdp.vdp_h_counter_port => self.vdp.readHCounter(),
            vdp.vdp_control_port => {
                const flags = self.vdp.ctrlRead();
                return flags;
            },
            vdp.vdp_data_port => self.vdp.dataRead(),
            else => self.ports[port] = return self.ports[port],
        };
    }

    pub fn ioWrite(ctx: *const anyopaque, address: u16, value: u8) void {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));
        const port: u8 = @truncate(address & 0xFF);
        switch (port) {
            vdp.vdp_data_port => self.vdp.dataWrite(value),
            vdp.vdp_control_port => self.vdp.ctrlWrite(value),
            else => self.ports[port] = value,
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
    const sio = try SystemIO.init();

    std.mem.copyForwards(u8, &sio.memory, rom);

    std.debug.print("Copied {d}KB of ROM to RAM.\n", .{rom.len / 1024});

    sio.cpu.PC = 0x0000;
    sio.cpu.SP.setValue(0xdff0);

    if (c.SDL_Init(c.SDL_INIT_VIDEO) != 0) {
        c.SDL_Log("Unable to initialize SDL: %s", c.SDL_GetError());
        return error.SDLInitializationFailed;
    }
    defer c.SDL_Quit();

    const scanlines_per_frame: c_int = switch (sio.vdp.display_type) {
        .ntsc => vdp.ntsc_scanlines,
        .pal => vdp.pal_scanlines,
    };

    const window = c.SDL_CreateWindow(
        "VDP-TEST",
        c.SDL_WINDOWPOS_UNDEFINED,
        c.SDL_WINDOWPOS_UNDEFINED,
        (vdp.pixels_per_scanline * 2) + 64 + 512,
        (scanlines_per_frame * 2) + 128,
        c.SDL_WINDOW_OPENGL,
    ) orelse
        {
        c.SDL_Log("Unable to create window: %s", c.SDL_GetError());
        return error.SDLInitializationFailed;
    };
    defer c.SDL_DestroyWindow(window);

    const renderer = c.SDL_CreateRenderer(window, -1, 0) orelse {
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

    var quit = false;

    _ = c.SDL_RenderSetScale(renderer, 2, 2);

    const palette_tile_size = 8;

    const viewport = c.SDL_Rect{
        .x = 0,
        .y = 0,
        .w = 256,
        .h = sio.vdp.display_lines,
    };

    var palette_tile = c.SDL_Rect{
        .x = 0,
        .y = 0,
        .w = palette_tile_size,
        .h = palette_tile_size,
    };

    const palette_border = c.SDL_Rect{
        .x = palette_tile_size - 2,
        .y = (scanlines_per_frame) - (2 * palette_tile_size) + 64,
        .w = (2 + palette_tile_size) * 32,
        .h = 4 + palette_tile_size,
    };

    while (!quit) {
        var event: c.SDL_Event = undefined;

        while (c.SDL_PollEvent(&event) != 0) {
            switch (event.type) {
                c.SDL_QUIT => {
                    quit = true;
                },
                c.SDL_KEYDOWN => { // a = 122, b = 120
                    switch (event.key.keysym.sym) {
                        1073741906 => sio.ports[0xdc] |= (@as(u8, 1) << 0), // UP
                        1073741905 => sio.ports[0xdc] |= (@as(u8, 1) << 1), // DOWN
                        1073741904 => sio.ports[0xdc] |= (@as(u8, 1) << 2), // LEFT
                        1073741903 => sio.ports[0xdc] |= (@as(u8, 1) << 3), // RIGHT
                        122 => sio.ports[0xdc] |= (@as(u8, 1) << 4), // A
                        120 => sio.ports[0xdc] |= (@as(u8, 1) << 5), // B
                        else => {},
                    }
                },
                c.SDL_KEYUP => {
                    switch (event.key.keysym.sym) {
                        1073741906 => sio.ports[0xdc] &= ~(@as(u8, 1) << 0), // UP
                        1073741905 => sio.ports[0xdc] &= ~(@as(u8, 1) << 1), // DOWN
                        1073741904 => sio.ports[0xdc] &= ~(@as(u8, 1) << 2), // LEFT
                        1073741903 => sio.ports[0xdc] &= ~(@as(u8, 1) << 3), // RIGHT
                        122 => sio.ports[0xdc] &= ~(@as(u8, 1) << 4),
                        120 => sio.ports[0xdc] &= ~(@as(u8, 1) << 5),
                        99 => dumpVRAM(sio),
                        else => {
                            std.debug.print("{d}\n", .{event.key.keysym.sym});
                        },
                    }
                },
                else => {},
            }
        }

        vdp_m4.renderFrame(sio.vdp, sio.cpu);

        _ = c.SDL_SetRenderDrawColor(renderer, 88, 88, 88, 255);
        _ = c.SDL_RenderClear(renderer);

        for (0..sio.vdp.cram.len) |ci| {
            const col = sio.vdp.cram[ci].toRGB8();
            palette_tile.x = @intCast(2 + palette_tile_size + ci * palette_tile_size);
            palette_tile.y = palette_border.y;
            _ = c.SDL_SetRenderDrawColor(renderer, col.r, col.g, col.b, 255);
            _ = c.SDL_RenderFillRect(renderer, &palette_tile);
        }
        _ = c.SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        _ = c.SDL_RenderDrawRect(renderer, &palette_border);

        var vi: usize = 0;
        var px: usize = 256 + 16;
        var py: usize = 1;

        while (vi < sio.vdp.vram.len - 32) {
            inline for (0..8) |i| {
                inline for (0..8) |j| {
                    const bp_0 = (sio.vdp.vram[vi + 0] >> j) & 1; // bitplane 0
                    const bp_1 = (sio.vdp.vram[vi + 1] >> j) & 1; // bitplane 1
                    const bp_2 = (sio.vdp.vram[vi + 2] >> j) & 1; // bitplane 2
                    const bp_3 = (sio.vdp.vram[vi + 3] >> j) & 1; // bitplane 3
                    const ij_color = bp_0 | (bp_1 << 1) | (bp_2 << 2) | (bp_3 << 3);
                    const vdp_col = sio.vdp.cram[ij_color];
                    const col = vdp_col.toRGB8();
                    _ = c.SDL_SetRenderDrawColor(renderer, col.r, col.g, col.b, if (ij_color == 16) 0 else 255);
                    _ = c.SDL_RenderDrawPoint(renderer, @as(c_int, @intCast(px + (7 - j))), @as(c_int, @intCast(py + i)));
                }
                vi += 4;
            }
            px += 8 + 1;
            if (px >= 256 + 16 + 17 * 8) {
                px = 256 + 16;
                py += 8 + 1;
            }
        }

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
        c.SDL_UnlockTexture(texture);

        _ = c.SDL_RenderCopy(renderer, texture, null, &viewport);

        c.SDL_RenderPresent(renderer);

        c.SDL_Delay(8);
    }
}
