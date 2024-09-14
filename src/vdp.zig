//! NOTE: Many annotations in this file are taken from:
//!     Sega Master System VDP documentation by Charles MacDonald,
//!     Copyright: Unpublished work Copyright 2000, 2001, 2002  Charles MacDonald

const std = @import("std");
const timings = @import("vdp.timings.zig");
const Z80State = @import("cpu.zig").Z80State;

const expectEqual = std.testing.expectEqual;

pub const vdp_v_counter_port = 0x7e; // read-only
pub const vdp_h_counter_port = 0x7f; // read-only
pub const vdp_data_port = 0xbe;
pub const vdp_control_port = 0xbf;

pub const ntsc_scanlines = 262;
pub const pal_scanlines = 313;
pub const ntsc_frames_per_sec = 60;
pub const pal_frames_per_sec = 50;
pub const pixels_per_scanline = 342;

/// A specific version of the VDP chip.
const VDPDeviceVersion = enum(u32) {
    /// Used in the Mark III, Sega Master System .
    sms = 315_5124,
    /// Used in the SMS 2 and later versions of the SMS.
    sms2 = 315_5246,
    /// Used in the Game Gear
    gg = 315_5378,
    /// Used in the Genesis and Mega Drive
    genesis = 315_5313,
};

const VDPDisplayType = enum {
    ntsc,
    pal,
};

/// Encodes the operation to perform.
const VDPCommand = enum(u2) {
    /// A byte of VRAM is read from and stored in the read buffer.
    vram_read = 0,
    /// Writes to the data port go to VRAM.
    vram_write = 1,
    /// This value signifies a VDP register write, explained below.
    vdp_reg_write = 2,
    /// Writes to the data port go to CRAM.
    cram_write = 3,
};

/// The 2-byte command sequence has the following format:
/// ```
/// MSB                         LSB
/// CD1 CD0 A13 A12 A11 A10 A09 A08    Second byte written
/// A07 A06 A05 A04 A03 A02 A01 A00    First byte written
/// ```
/// where CD = command, A = address
///
/// if command = 2 (vdp_register_write):
/// ```
/// MSB                         LSB
/// 1   0   ?   ?   R03 R02 R01 R00    Second byte written
/// D07 D06 D05 D04 D03 D02 D01 D00    First byte written
/// ```
///  * Rxx : VDP register number
///  * Dxx : VDP register data
///  * ?  : Ignored
///
pub const VDPControlWord = packed struct(u16) {
    payload: packed union {
        address: u14,
        reg_write_payload: packed struct(u14) {
            data: u8,
            register: u4,
            _ignored: u2 = 0,
        },
    },
    command: VDPCommand,
};

/// The VDP status flags.
/// NOTE: flags are cleared when the control port is read.
const VDPStatusFlags = packed struct(u8) {
    /// garbage values for the SMS 2. The Genesis returns extended status flags in this lower bits.
    _unused: u5 = 0,
    /// This flag is set if an opaque pixel from any two sprites overlap.
    sprite_collision: bool = false, // 5 (msb)
    /// This flag is set if there are more than eight sprites that are positioned on a single scanline.
    sprite_overflow: bool = false, // 6
    /// This flag is set on the first line after the end of the active display period.
    frame_interrupt_pending: bool = false, // 7
};

/// Represents a 6 bit RGB color in the Color RAM (CRAM).
/// ```
///  76 54 32 10
///  -- BB GG RR
/// ```
pub const VDPColor = packed struct(u8) {
    r: u2,
    g: u2,
    b: u2,
    _: u2 = 0,

    const Self = @This();
    const SCALE_255: u8 = 255 / 0b11; // 255/3 ≈ 85

    pub fn toRGB8(self: VDPColor) struct { r: u8, g: u8, b: u8 } {
        return .{
            .r = self.r * SCALE_255,
            .g = self.g * SCALE_255,
            .b = self.b * SCALE_255,
        };
    }

    pub fn fromRGB(r: u8, g: u8, b: u8) Self {
        return .{
            .r = @intFromFloat(@floor(@as(f16, @floatFromInt(r)) / SCALE_255)),
            .b = @intFromFloat(@floor(@as(f16, @floatFromInt(b)) / SCALE_255)),
            .g = @intFromFloat(@floor(@as(f16, @floatFromInt(g)) / SCALE_255)),
        };
    }
};

/// Mode Control No. 1
pub const VDPRegister0 = packed struct(u8) {
    no_sync_display_mode: bool, // 0 - 1= No sync, display is monochrome, 0= Normal display
    m2: bool, // 1 - Must be 1 for M1/M3 to change screen height in M4 otherwise has no effect
    m4: bool, // 2 - Enable the SMS mode
    ec: bool, // 3 - Shift sprites left 8px
    h_blank_on: bool, // 4 - Interrupt enable 1 - HBlank ON
    blank_col_0: bool, // 5 - Mask column 0 with overscan color from register 7
    fix_top_2_rows: bool, // 6 - Disable horizontal scrolling for rows 0-1
    fix_right_8_cols: bool, // 7 - Disable vertical scrolling for cols 24-31
};

/// Mode Control No. 2
pub const VDPRegister1 = packed struct(u8) {
    zoomed_sprites: bool, // 0 - Sprite pixels are doubled in size
    magnified_sprites: bool, // 1 - Sprites are 1=16x16, 0=8x8 (TMS9918), Sprites are 1=8x16, 0=8x8 (Mode 4)
    _unused_2: bool, // 2 - no effect
    m3: bool, // 3 - Selects 240-line screen for Mode 4 if M2=1, otherwise it has no effect.
    m1: bool, // 4 - Selects 224-line screen for Mode 4 if M2=1, otherwise it has no effect.
    v_blank_on: bool, // 5 - 1= Line interrupt enable (vsync?) - VBlank ON
    display_on: bool, // 6 (BLANK) - 1= Enables the active display, 0 = causes the active display area to blank
    enable_16k_vram: bool = true, // 7 always set on the SMS
};

/// Name Table Base Address
const VDPRegister2 = packed struct(u8) {
    _unused_0: u1,
    name_table_addr: u3, // bits 1-3 -> bits 11,12,13 of the name_table_address
    _unused_4: u1,
    _unused_5: u1,
    _unused_6: u1,
    _unused_7: u1,
};

/// Sprite Attribute Table Base Address
const VDPRegister5 = packed struct(u8) {
    _unused_0: u1,
    sprite_attr_table_addr: u6, // bits 1-6 -> bits 8 -> 13 of the sprite_attr_table_addr
    _unused_7: u1,
};

/// Sprite Pattern Generator Base Address
const VDPRegister6 = packed struct(u8) {
    _unused_0: u1,
    _unused_1: u1,
    sprite_pattern_gen_addr: u1, // bit 13 of the sprite_attr_table_addr
    _unused_3: u1,
    _unused_4: u1,
    _unused_5: u1,
    _unused_6: u1,
    _unused_7: u1,
};

/// Overscan/Backdrop Color
const VDPRegister7 = packed struct(u8) {
    overscan_color: u4,
    _unused_4: u1,
    _unused_5: u1,
    _unused_6: u1,
    _unused_7: u1,
};

pub const VDPRegister = enum(u8) {
    mode_control_1 = 0,
    mode_control_2 = 1,
    name_table_base_addr = 2, // background
    color_table_base_addr = 3,
    bg_pattern_gen_table_base_addr = 4,
    sprite_attr_table_base_addr = 5,
    sprite_pattern_gen_table_base_addr = 6,
    overscan_backdrop_color = 7,
    bg_scroll_x = 8,
    bg_scroll_y = 9,
    line_counter = 10,
};

const VDPTable = enum(u8) {
    name_table = 2,
    sprite_attribute_table = 5,
    sprite_pattern_generator_table = 6,
};

/// An entry in the name table.
///
/// The background is made up of 8x8 tiles that can use 16 colors from either palette.
/// The background is defined by the name table which is a matrix of words stored in VRAM.
/// The name table is 32x28 or 32x32 if using the 224 or 240-line displays.
pub const BackgroundTile = packed struct(u16) {
    /// Points to one of 512 patterns store in VRAM.
    pattern_idx: u9, // 0
    horizontal_flip: bool = false,
    vertical_flip: bool = false,
    palette_select: u1 = 0,
    priority_flag: bool = false,
    _unused: u3 = 0,
};

/// An entry in the patterns VRAM section.
///
/// The background and sprites are made out of 8x8-pixel patterns.
/// Each pattern is composed of four bitplanes, so individual pixels can be one of 16 colors.
/// Each pattern uses 32 bytes:
///   * The first four bytes are bitplanes 0 through 3 for line 0,
///   * The next four bytes are bitplanes 0 through 3 for line 1,
///   * and so on up to line 7
const Pattern = packed struct {
    bitplane_0: BitPlane,
    bitplane_1: BitPlane,
    bitplane_2: BitPlane,
    bitplane_3: BitPlane,
    bitplane_4: BitPlane,
    bitplane_5: BitPlane,
    bitplane_6: BitPlane,
    bitplane_7: BitPlane,

    const BitPlane = packed struct(u32) {
        bitplane_0: u8,
        bitplane_1: u8,
        bitplane_2: u8,
        bitplane_3: u8,
    };

    comptime {
        std.debug.assert(@sizeOf(Pattern) == 32); // @sizeOf(BitPlane) * 8
    }
};

pub const VDPState = struct {
    /// The CPU
    cpu: Z80State,

    /// The type of VDP.
    device_type: VDPDeviceVersion = .sms2,

    /// The display type.
    display_type: VDPDisplayType = .ntsc,

    /// The height of the display in lines.
    display_lines: u8 = 192, // can be 192 (default), 224 or 240.

    /// The display frame buffer.
    display_buffer: []u8 = undefined,

    /// The scanline currently being processed.
    scanline: u9 = 0,

    /// The scanline priority buffer.
    scanline_priprity_buffer: [256]u8,

    /// Write-only control registers. Only 0 to 9 has effect.
    registers: [16]u8,

    /// The Video RAM.
    vram: [16 * 1024]u8, // 16K

    // The Color RAM. Contains 2 16-color palettes.
    cram: [32]VDPColor, // 32 bytes

    /// The VRAM read-ahead buffer.
    read_ahead_buff: u8 = 0,

    /// If true, the byte on the control port is the second one in the 2-byte control sequence.
    ctrl_is_second_byte: bool = false,

    /// Stores the current control word (address reg + code reg) read from the control port 2-byte sequence.
    ctrl_word: VDPControlWord = 0,

    /// The status flags.
    status_flags: VDPStatusFlags,

    // True if a line interrupt is pending.
    h_int_pending: bool = false,

    /// The VDP has a line counter that behaves in the following way:
    ///  - It is loaded with the contents of register 0x0A (10) on every line outside of the active display period,
    ///     excluding the line after the last line of the active display period.
    ///  - It is decremented on every line within the active display period,
    ///    including the line after the last line of the active display period.
    ///
    /// **Remarks**:
    ///
    /// Writing to register 0x0A(10) will not immediately change the contents of the counter.
    /// This only occurs when the counter is reloaded, either:
    ///   - outside of the active display period (as described above)
    ///   - when the counter underflows
    ///
    line_counter: u8 = 0,

    // The H counter can only be read when the state of the TH pin of either
    //  joystick port changes, which is used for the lightgun peripheral. When
    //  this happens, the value of the H counter at the time TH changed is
    //  returned when the H counter is read, and remains frozen until TH is
    //  changed again.
    h_counter: u9 = 0x80,

    allocator: std.mem.Allocator,

    const Self = @This();

    pub fn updateVideoMode(self: *Self) void {
        const r0: VDPRegister0 = @bitCast(self.registers[0]);

        const old_display_lines = self.display_lines;

        if (self.device_type == .sms and r0.m4) {
            self.display_lines = 192;
        } else {
            const r1: VDPRegister1 = @bitCast(self.registers[1]);

            const mode = (@as(u8, @intFromBool(r0.m4)) << 3) | (@as(u8, @intFromBool(r1.m3)) << 2) | (@as(u8, @intFromBool(r0.m2)) << 1) | @as(u8, @intFromBool(r1.m1));

            self.display_lines = switch (mode) {
                //
                0b1011 => 224,
                0b1110 => 240,
                else => 192,
            };
        }

        if (self.display_lines != old_display_lines) {
            self.allocator.free(self.display_buffer);

            self.display_buffer = self.allocator.alloc(u8, @as(usize, 256) * (self.display_lines + 1)) catch |err| {
                std.debug.panic("Failed to alloc new display buffer: {any}.\n", .{err});
            };

            for (0..self.display_buffer.len) |i| {
                self.display_buffer[i] = 0;
            }
        }
    }

    pub fn getNameTableBaseAddress(self: *Self) u16 {
        const r: u8 = self.registers[@intFromEnum(VDPRegister.name_table_base_addr)] & 0b00001110;
        if (self.display_lines != 192) { // either 224 or 240 line-display is being used
            // D3 D2  Addr
            // -- -- ------
            // 0 0   0b00_011100000000 - 0x0700
            // 0 1   0b01_011100000000 - 0x1700
            // 1 0   0b10_011100000000 - 0x2700
            // 1 1   0b11_011100000000 - 0x3700
            //       0bxx_011100000000
            return ((@as(u14, r) >> 2) << 12) | 0x700; // only bits 3 and 2
        } else {
            return @as(u14, r) << 10; // 0bxxxx111x -> 0b0011100000000000, which is equivalent to R2 * 400h.
        }
    }

    pub fn getSpriteGenTableBaseAddress(self: *Self) u16 {
        const r: u8 = self.registers[@intFromEnum(VDPRegister.sprite_pattern_gen_table_base_addr)] & 0b00000100;
        return @as(u14, r) << 11;
    }

    pub fn getSpriteAttrTableBaseAddress(self: *Self) u16 {
        const r: u8 = self.registers[@intFromEnum(VDPRegister.sprite_attr_table_base_addr)] & 0b01111110;
        return @as(u14, r) << 7;
    }

    fn getTableBaseAddress(self: *Self, comptime t: VDPTable) u16 {
        // See significant bits in register structs.
        return switch (t) {
            inline .name_table => @as(u16, self.registers[@intFromEnum(VDPRegister.name_table_base_addr)]) << 10,
            inline .sprite_attribute_table => @as(u16, self.registers[@intFromEnum(VDPRegister.sprite_attr_table_base_addr)]) << 8,
            inline .sprite_pattern_generator_table => @as(u16, self.registers[@intFromEnum(VDPRegister.sprite_pattern_gen_table_base_addr)]) << 11,
        };
    }

    pub fn ctrlWrite(self: *Self, value: u8) void {
        if (self.ctrl_is_second_byte == false) {
            self.ctrl_word.payload.address = (self.ctrl_word.payload.address & 0x3F00) | value; // set lower 8-bits of addr reg
            self.ctrl_is_second_byte = true;
        } else {
            const ctrl_word = self.ctrl_word.payload.address & 0x00FF | (@as(u16, value) << 8);
            self.ctrl_word = @bitCast(ctrl_word);
            self.ctrl_is_second_byte = false;

            switch (self.ctrl_word.command) {
                .vram_read => _ = self.vramRead(),
                .vdp_reg_write => self.registerWrite(),
                else => {},
            }
        }
    }

    pub fn ctrlRead(self: *Self) u8 {
        const flags: u8 = @bitCast(self.status_flags);
        self.ctrl_is_second_byte = false;
        self.h_int_pending = false;
        self.status_flags.frame_interrupt_pending = false;
        self.status_flags.sprite_collision = false;
        self.status_flags.sprite_overflow = false;
        return flags;
    }

    fn vramRead(self: *Self) u8 {
        const buffered = self.read_ahead_buff;
        self.read_ahead_buff = self.vram[self.ctrl_word.payload.address];
        self.ctrl_word.payload.address +%= 1; // wraps around at 0x3FFF
        return buffered;
    }

    fn vramWrite(self: *Self, value: u8) void {
        self.vram[self.ctrl_word.payload.address] = value;
        self.ctrl_word.payload.address +%= 1; // wraps around at 0x3FFF
        // writing to the data port will also load the buffer with the written value
        self.read_ahead_buff = value;
    }

    fn registerWrite(self: *Self) void {
        const w = self.ctrl_word.payload.reg_write_payload;
        self.registers[w.register] = w.data;

        // TODO: unassert IRQ line in case h_blank_on (r0 & 0x10) gets unset.

        if (w.register <= 1) {
            self.updateVideoMode();
        }
    }

    fn cramWrite(self: *Self, value: u8) void {
        const address = self.ctrl_word.payload.address;
        self.cram[address & 0x001F] = @bitCast(value); // wraps around 0x001F (0-31)
        self.ctrl_word.payload.address +%= 1; // wraps around at 0x3FFF
        self.read_ahead_buff = value;
    }

    pub fn dataWrite(self: *Self, value: u8) void {
        if (self.ctrl_word.command != .cram_write) {
            vramWrite(self, value);
        } else {
            cramWrite(self, value);
        }
        self.ctrl_is_second_byte = false;
    }

    pub fn dataRead(self: *Self) u8 {
        self.ctrl_is_second_byte = false;
        return vramRead(self);
    }

    pub fn readVCounter(self: *Self) u8 {
        switch (self.display_type) {
            .ntsc => {
                return switch (self.display_lines) {
                    192 => timings.v_counter_ntsc_192[self.scanline],
                    224 => timings.v_counter_ntsc_224[self.scanline],
                    else => timings.v_counter_ntsc_240[self.scanline],
                };
            },
            .pal => {
                return switch (self.display_lines) {
                    192 => timings.v_counter_pal_192[self.scanline],
                    224 => timings.v_counter_pal_224[self.scanline],
                    else => timings.v_counter_pal_240[self.scanline],
                };
            },
        }
    }

    pub fn readHCounter(self: *Self) u8 {
        return @truncate(self.h_counter); // TODO: not implemented, is this only related to the lightgun peripheral?
    }

    /// Generates frame interrupts if the required conditions are met.
    pub fn processFrameInterrupt(self: *Self, cpu: *Z80State) void {
        const r1: VDPRegister1 = @bitCast(self.registers[@intFromEnum(VDPRegister.mode_control_2)]);

        // Bit 5 of register 1 (ie0 here) acts like a on/off switch for the VDP's IRQ line and
        // as long as bit 7 of the status flags is set, the VDP will assert the IRQ.
        if (r1.v_blank_on) {
            cpu.requests.int_signal = self.status_flags.frame_interrupt_pending;
        }
    }

    /// Generates line interrupts if the required conditions are met.
    ///
    /// Example:
    ///  - NTSC machine (262 scanlines per frame with a 192-line display.
    ///  - Scanlines from 0-261:
    ///     - Counter -= 1     : on lines 0-191 and 192
    ///     - Counter <- 0xA   : on lines 193-261
    pub fn processLineInterrupt(self: *Self, cpu: *Z80State) void {
        if (self.scanline <= self.display_lines) { // We're inside the active display
            const line_counter = self.line_counter;

            // The counter is decremented on every line within the active display period.
            // Including the line after the last line of the active display period.
            self.line_counter -%= 1;

            if (line_counter <= 0) {
                self.h_int_pending = true;

                // Bit 4 of register $00 acts like a on/off switch for the VDP's IRQ line.
                // As long as the line interrupt pending flag is set, the VDP will assert the
                // IRQ line if bit 4 of register $00 is set,
                // and it will de-assert the IRQ line if the same bit is cleared.
                // if ((self.registers[0] & 0x16 != 0)) {
                // }
                cpu.requests.int_signal = (self.registers[0] & 0x16 != 0);

                // When the counter underflows from 0x00 to 0xFF,
                // it is reloaded with the last value written to register 0x0A (10).
                self.line_counter = self.registers[10];
            }
            return;
        }
        // The counter is loaded with the contents of register $0A(10) on
        // every line outside of the active display period, excluding the line after
        // the last line of the active display period.
        self.line_counter = self.registers[10];
    }

    pub fn dbgDumpVdpState(self: *const Self) void {
        const name_table_addr = self.getTableBaseAddress(VDPTable.name_table);
        std.debug.print("Tables:\n", .{});
        std.debug.print("  VDP Name Table:             0x{x}\n", .{name_table_addr});
        std.debug.print("  VDP Sprite Attribute Table: 0x{x}\n", .{self.getTableBaseAddress(VDPTable.sprite_attribute_table)});
        std.debug.print("  VDP Sprite Generator Table: 0x{x}\n", .{self.getTableBaseAddress(VDPTable.sprite_pattern_generator_table)});
    }

    pub fn init(allocator: std.mem.Allocator) !*VDPState {
        const state = try allocator.create(Self);

        state.allocator = allocator;
        state.ctrl_is_second_byte = false;
        state.read_ahead_buff = 0;
        state.display_lines = 192;

        state.display_buffer = try allocator.alloc(u8, @as(usize, 256) * (state.display_lines + 1));

        for (0..state.display_buffer.len) |i| {
            state.display_buffer[i] = 0;
        }

        inline for (0..8) |r| {
            state.registers[r] = 0;
        }

        return state;
    }

    pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
        allocator.free(self.display_buffer);
        allocator.destroy(self);
    }
};

test "VDP Init" {
    const vdp = try VDPState.init(std.testing.allocator);
    defer vdp.deinit(std.testing.allocator);

    try expectEqual(16 * 1024, vdp.vram.len);

    vdp.vram[8192] = 129;
    try expectEqual(129, vdp.vram[8192]);
}

test "Control word write" {
    const vdp = try VDPState.init(std.testing.allocator);
    defer vdp.deinit(std.testing.allocator);

    const command = VDPControlWord{
        .command = .vram_read,
        .payload = .{ .address = 8192 },
    };

    const control_word: u16 = @bitCast(command);

    try expectEqual(false, vdp.ctrl_is_second_byte);

    vdp.ctrlWrite(control_word & 0xFF);

    try expectEqual(control_word & 0xFF, vdp.ctrl_word.payload.address & 0xFF);
    try expectEqual(true, vdp.ctrl_is_second_byte);

    vdp.ctrlWrite((control_word >> 8) & 0xFF);

    try expectEqual(control_word +% 1, @as(u16, @bitCast(vdp.ctrl_word))); // +1 for the address increment
    try expectEqual(false, vdp.ctrl_is_second_byte);
}

test "VRAM read" {
    const vdp = try VDPState.init(std.testing.allocator);
    defer vdp.deinit(std.testing.allocator);

    const read_addr: u14 = 8192;
    const value: u8 = 129;

    vdp.vram[read_addr] = value;

    const command = VDPControlWord{
        .command = .vram_read,
        .payload = .{ .address = read_addr },
    };

    const control_word: u16 = @bitCast(command);

    vdp.ctrlWrite(control_word & 0xFF);
    vdp.ctrlWrite((control_word >> 8) & 0xFF);
    try expectEqual(value, vdp.read_ahead_buff);

    const data = vdp.dataRead();
    try expectEqual(value, data);
}

test "VRAM write" {
    const vdp = try VDPState.init(std.testing.allocator);
    defer vdp.deinit(std.testing.allocator);

    const write_addr: u14 = 14928;

    const command = VDPControlWord{
        .command = .vram_write,
        .payload = .{ .address = write_addr },
    };

    const control_word: u16 = @bitCast(command);
    vdp.ctrlWrite(control_word & 0xFF);
    vdp.ctrlWrite((control_word >> 8) & 0xFF);

    const value: u8 = 155;
    vdp.dataWrite(value);
    try expectEqual(value, vdp.read_ahead_buff);
    try expectEqual(value, vdp.vram[write_addr]);
}

test "CRAM write" {
    const vdp = try VDPState.init(std.testing.allocator);
    defer vdp.deinit(std.testing.allocator);

    const write_addr: u14 = 28;

    const command = VDPControlWord{
        .command = .cram_write,
        .payload = .{ .address = write_addr },
    };

    const control_word: u16 = @bitCast(command);
    vdp.ctrlWrite(control_word & 0xFF);
    vdp.ctrlWrite((control_word >> 8) & 0xFF);

    const value: u8 = 155;
    vdp.dataWrite(value);

    try expectEqual(value, @as(u8, @bitCast(vdp.cram[write_addr])));
}

test "CRAM write (wrap)" {
    const vdp = try VDPState.init(std.testing.allocator);
    defer vdp.deinit(std.testing.allocator);

    const write_addr: u14 = 48;

    const command = VDPControlWord{
        .command = .cram_write,
        .payload = .{ .address = write_addr },
    };

    const control_word: u16 = @bitCast(command);
    vdp.ctrlWrite(control_word & 0xFF);
    vdp.ctrlWrite((control_word >> 8) & 0xFF);

    const value: u8 = 155;
    vdp.dataWrite(value);

    try expectEqual(value, @as(u8, @bitCast(vdp.cram[write_addr % vdp.cram.len])));
}

test "Base addr register" {
    const vdp = try VDPState.init(std.testing.allocator);
    defer vdp.deinit(std.testing.allocator);

    const value: u8 = 0b00001110;

    const command = VDPControlWord{
        .command = .vdp_reg_write,
        .payload = .{
            .reg_write_payload = .{
                .register = @intFromEnum(VDPRegister.name_table_base_addr),
                .data = value,
            },
        },
    };

    const control_word: u16 = @bitCast(command);
    vdp.ctrlWrite(control_word & 0xFF);
    vdp.ctrlWrite((control_word >> 8) & 0xFF);

    try expectEqual(value, vdp.registers[2]);

    try expectEqual(0b11100000000000, vdp.getTableBaseAddress(VDPTable.name_table));
}

test "Name table address (224 lines-display)" {
    var vdp = try VDPState.init(std.testing.allocator);
    defer vdp.deinit(std.testing.allocator);

    vdp.display_lines = 224;

    vdp.registers[@intFromEnum(VDPRegister.name_table_base_addr)] = 0b0000_001_0;
    try expectEqual(0x0700, vdp.getNameTableBaseAddress());
    vdp.registers[@intFromEnum(VDPRegister.name_table_base_addr)] = 0b0000_000_0;
    try expectEqual(0x0700, vdp.getNameTableBaseAddress());

    vdp.registers[@intFromEnum(VDPRegister.name_table_base_addr)] = 0b0000_011_0;
    try expectEqual(0x1700, vdp.getNameTableBaseAddress());
    vdp.registers[@intFromEnum(VDPRegister.name_table_base_addr)] = 0b0000_010_0;
    try expectEqual(0x1700, vdp.getNameTableBaseAddress());

    vdp.registers[@intFromEnum(VDPRegister.name_table_base_addr)] = 0b0000_101_0;
    try expectEqual(0x2700, vdp.getNameTableBaseAddress());
    vdp.registers[@intFromEnum(VDPRegister.name_table_base_addr)] = 0b0000_100_0;
    try expectEqual(0x2700, vdp.getNameTableBaseAddress());

    vdp.registers[@intFromEnum(VDPRegister.name_table_base_addr)] = 0b0000_111_0;
    try expectEqual(0x3700, vdp.getNameTableBaseAddress());
    vdp.registers[@intFromEnum(VDPRegister.name_table_base_addr)] = 0b0000_110_0;
    try expectEqual(0x3700, vdp.getNameTableBaseAddress());
}

test "Name table address (192 lines-display)" {
    var vdp = try VDPState.init(std.testing.allocator);
    defer vdp.deinit(std.testing.allocator);

    vdp.display_lines = 192;

    vdp.registers[@intFromEnum(VDPRegister.name_table_base_addr)] = 0b0000_000_0;
    try expectEqual(0x0000, vdp.getNameTableBaseAddress());

    vdp.registers[@intFromEnum(VDPRegister.name_table_base_addr)] = 0b1111_000_1;
    try expectEqual(0x0000, vdp.getNameTableBaseAddress());

    vdp.registers[@intFromEnum(VDPRegister.name_table_base_addr)] = 0b0000_111_0;
    try expectEqual(0x3800, vdp.getNameTableBaseAddress());

    vdp.registers[@intFromEnum(VDPRegister.name_table_base_addr)] = 0b1111_111_1;
    try expectEqual(0x3800, vdp.getNameTableBaseAddress());
}

test "Sprite Attribute table address" {
    var vdp = try VDPState.init(std.testing.allocator);
    defer vdp.deinit(std.testing.allocator);

    vdp.registers[@intFromEnum(VDPRegister.sprite_attr_table_base_addr)] = 0b0_111111_0;
    try expectEqual(0x3f00, vdp.getSpriteAttrTableBaseAddress());

    vdp.registers[@intFromEnum(VDPRegister.sprite_attr_table_base_addr)] = 0b1_111111_1;
    try expectEqual(0x3f00, vdp.getSpriteAttrTableBaseAddress());

    vdp.registers[@intFromEnum(VDPRegister.sprite_attr_table_base_addr)] = 0b0_000000_0;
    try expectEqual(0x0000, vdp.getSpriteAttrTableBaseAddress());

    vdp.registers[@intFromEnum(VDPRegister.sprite_attr_table_base_addr)] = 0b1_000000_1;
    try expectEqual(0x0000, vdp.getSpriteAttrTableBaseAddress());
}

test "Sprite Generator table address" {
    var vdp = try VDPState.init(std.testing.allocator);
    defer vdp.deinit(std.testing.allocator);
    vdp.registers[@intFromEnum(VDPRegister.sprite_pattern_gen_table_base_addr)] = 0b00000_0_00;
    try expectEqual(0x0000, vdp.getSpriteGenTableBaseAddress());

    vdp.registers[@intFromEnum(VDPRegister.sprite_pattern_gen_table_base_addr)] = 0b11111_0_11;
    try expectEqual(0x0000, vdp.getSpriteGenTableBaseAddress());

    vdp.registers[@intFromEnum(VDPRegister.sprite_pattern_gen_table_base_addr)] = 0b00000_1_00;
    try expectEqual(0x2000, vdp.getSpriteGenTableBaseAddress());

    vdp.registers[@intFromEnum(VDPRegister.sprite_pattern_gen_table_base_addr)] = 0b11111_1_11;
    try expectEqual(0x2000, vdp.getSpriteGenTableBaseAddress());
}
