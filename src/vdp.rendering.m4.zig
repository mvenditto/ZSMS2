//! Rendering routines for the 4 (M4) VDP mode.

const std = @import("std");
const vdp = @import("vdp.zig");
const opcodes = @import("opcodes.zig");
const Z80State = @import("cpu.zig").Z80State;
const VDPRegister0 = vdp.VDPRegister0;
const VDPRegister1 = vdp.VDPRegister1;
const VDPRegister6 = vdp.VDPRegister6;
const VDPState = vdp.VDPState;
const BackgroundTile = vdp.BackgroundTile;
const expectEqual = std.testing.expectEqual;

const display_columns = 256;
const bitplanes_per_line = 4;
const pattern_columns = 8;
const pattern_lines = 8;
const cycles_per_frame = 228;

pub fn renderFrame(self: *VDPState, cpu: *Z80State) void {
    const scanlines_per_frame: u9 = switch (self.display_type) {
        .ntsc => vdp.ntsc_scanlines,
        .pal => vdp.pal_scanlines,
    };

    const r1: VDPRegister1 = @bitCast(self.registers[1]);

    for (0..scanlines_per_frame) |scanline| {
        self.scanline = @intCast(scanline);

        // Active display
        if (self.scanline < self.display_lines) {
            if (r1.display_on == true) {
                renderBackgroundLine(self);
                if (self.sprites_idx > 0) renderSpritesLine(self);
                prepareNextSpritesLine(self);

                // Mask column 0 (8 pixels)
                if (self.registers[0] & 32 != 0) {
                    const backdrop: u8 = @bitCast(self.cram[self.registers[7] + 16]);
                    const y = scanline * 256;
                    for (0..8) |ix| {
                        self.display_buffer[y + ix] = backdrop;
                    }
                }
            } else {
                const backdrop_color = 16 | (self.registers[7] & 15);
                for (0..display_columns) |x| {
                    self.display_buffer[(scanline * display_columns) + x] = backdrop_color;
                }
            }
        } else {
            self.sprites_idx = 0;
        }

        // The vertical blanking period has started (inactive time)
        if (self.scanline == self.display_lines + 1) {
            // Set bit 7 of the status flags,
            // this bit will remain set until the control port is read.
            self.status_flags.frame_interrupt_pending = true;
        }

        self.processFrameInterrupt(cpu);
        self.processLineInterrupt(cpu);

        // cycles per frame
        var cycles: u8 = 0;
        while (cycles <= cycles_per_frame) {
            const t = opcodes.execOne(cpu);

            if (t == 0) {
                break; // debugger attached
            }

            cycles += t;
        }
    }
}

pub fn renderBackgroundLine(self: *VDPState) void {
    const scanline: u8 = @intCast(self.scanline);

    const display_buffer_y = @as(usize, scanline) * display_columns;

    // The vertical scroll value cannot be changed during the active display
    var y_scroll: u32 = @as(u32, scanline) + self.registers[9];

    if (self.display_lines == 192) {
        // In the regular 192-line display mode the name table is 32x28,
        // so the vertical scroll register wraps past 223
        y_scroll %= 224;
    } else {
        // In the 224 and 240-line display modes the name table is 32x32,
        // so the vertical scroll register wraps past 255
        y_scroll &= 255;
    }

    var y = y_scroll / 8;

    const r0: VDPRegister0 = @bitCast(self.registers[0]);

    // If bit 6 of VDP register 0 is set,
    // horizontal scrolling will be fixed at zero for scanlines 0-15
    var x_scroll: u8 = if (r0.fix_top_2_rows and scanline < 16) 0 else self.registers[8];
    const x = x_scroll & 7; // lower 3 bits of R8, fine scroll X
    x_scroll >>= 3;
    if (x_scroll == 0) {
        x_scroll = 32;
    }

    const name_table_addr = self.getNameTableBaseAddress();
    const name_table_size: usize = if (self.display_lines != 192) 32 * 32 else 32 * 28;
    const tiles: [*]align(1) BackgroundTile = @ptrCast(@constCast(self.vram[name_table_addr .. name_table_addr + name_table_size]));

    // foreach tile 0-31 on the current scanline
    for (0..32) |tile_x| {
        // The vertical scroll value will be fixed to zero when columns 24-31 are rendered
        if (r0.fix_right_8_cols and tile_x == 24) {
            y = scanline;
            y_scroll = scanline;
            y = y / 8;
        }

        // Tile from the Name Table (32 x H)
        const tile = tiles[(y * 32) + ((32 - x_scroll + tile_x) & 31)]; // 32 - x_scroll = starting column

        // The pattern index in VRAM.
        const base_pattern_addr = @as(usize, tile.pattern_idx) * 32;

        // The line's offset in the 8x8 pattern, each line is encoded in 4 bitplanes hence 32byte per pattern.

        var v_flip: u32 = 2 * (y_scroll % 8);

        if (tile.vertical_flip) {
            v_flip = 7;
        }

        const line_offset = ((v_flip - (y_scroll % 8)) * 4);

        // The absolute line's address
        const pi = base_pattern_addr + line_offset;

        // Foreach column (pixel) 0-7 in the pattern's line,
        // merge the 4 bitplanes and get the color index.
        inline for (0..pattern_columns) |j| { // 4 bitplanes foreach pattern line
            const bp_0 = (self.vram[pi + 0] >> j) & 1; // bitplane 0
            const bp_1 = (self.vram[pi + 1] >> j) & 1; // bitplane 1
            const bp_2 = (self.vram[pi + 2] >> j) & 1; // bitplane 2
            const bp_3 = (self.vram[pi + 3] >> j) & 1; // bitplane 3

            var h_flip: u8 = 7;

            if (tile.horizontal_flip) {
                h_flip = 2 * j;
            }

            const ij_color = bp_0 | (bp_1 << 1) | (bp_2 << 2) | (bp_3 << 3);

            const palette_color = (16 * @as(u5, tile.palette_select)) + ij_color;

            const draw_over_sprite = tile.priority_flag and ij_color != 0;

            self.scanline_priority_buffer[(tile_x * 8) + j] = draw_over_sprite;

            // var color = self.cram[palette_color];
            // if (draw_over_sprite) {
            //     color.r = @intFromFloat(@as(f32, @floatFromInt(color.r)) * 0.299);
            //     color.g = @intFromFloat(@as(f32, @floatFromInt(color.g)) * 0.587);
            //     color.b = @intFromFloat(@as(f32, @floatFromInt(color.b)) * 0.114);
            // }

            self.display_buffer[display_buffer_y + (x + (tile_x * 8) + (h_flip - j))] = @bitCast(self.cram[palette_color]);
        }
    }
}

///
/// Each sprite is defined in the sprite attribute table (SAT), a 256-byte
/// table located in VRAM. The SAT has the following layout:
/// ```
/// 00: yyyyyyyyyyyyyyyy
/// 10: yyyyyyyyyyyyyyyy
/// 20: yyyyyyyyyyyyyyyy
/// 30: yyyyyyyyyyyyyyyy
/// 40: ????????????????
/// 50: ????????????????
/// 60: ????????????????
/// 70: ????????????????
/// 80: xnxnxnxnxnxnxnxn
/// 90: xnxnxnxnxnxnxnxn
/// A0: xnxnxnxnxnxnxnxn
/// B0: xnxnxnxnxnxnxnxn
/// C0: xnxnxnxnxnxnxnxn
/// D0: xnxnxnxnxnxnxnxn
/// E0: xnxnxnxnxnxnxnxn
/// F0: xnxnxnxnxnxnxnxn
///
///  y = Y coordinate + 1
///  x = X coordinate
///  n = Pattern index
///  ? = Unused
/// ```
pub fn prepareNextSpritesLine(self: *VDPState) void {
    self.sprites_idx = 0;

    const next_scanline = self.scanline + 1;
    const sat_base_address = self.getSpriteAttrTableBaseAddress();
    const sat = self.vram[sat_base_address .. sat_base_address + 256];
    const r0: VDPRegister0 = @bitCast(self.registers[0]);
    const r1: VDPRegister1 = @bitCast(self.registers[1]);
    const r6: VDPRegister6 = @bitCast(self.registers[6]);
    const pattern_offset: u9 = r6.sprite_pattern_gen_addr * @as(u9, 256);

    var sprite_height: u8 = 8;
    const sprite_x_offset: u8 = @as(u8, @intFromBool(r0.ec)) * 8;

    if (r1.magnified_sprites) {
        sprite_height = 16;
    }

    if (r1.zoomed_sprites) {
        sprite_height *= 2;
    }

    for (0..64) |i| {
        var y = sat[i];

        if (self.display_lines == 192 and y == 208) {
            break;
        }

        y +%= 1;

        if (next_scanline >= y and next_scanline < y + sprite_height) {
            if (self.sprites_idx == 8) {
                self.status_flags.sprite_overflow = true;
                break;
            }
            const sprite = &self.sprites_buffer[self.sprites_idx];
            sprite.y = y;
            sprite.x = sat[(i * 2) + 128] -% sprite_x_offset;
            sprite.pattern_idx = @as(u9, sat[(i * 2) + 128 + 1]) + pattern_offset;
            self.sprites_idx += 1;
        }
    }
}

pub fn renderSpritesLine(self: *VDPState) void {
    const display_buffer_y = @as(usize, self.scanline) * display_columns;
    const r1: VDPRegister1 = @bitCast(self.registers[1]);

    for (0..self.sprite_collision_buffer.len) |i| {
        self.sprite_collision_buffer[i] = false;
    }

    self.sprites_idx -= 1;

    // TODO: see if I can do better and avoid duplicating next chunk of code.

    while (self.sprites_idx >= 0) : (self.sprites_idx -%= 1) {
        const sprite = &self.sprites_buffer[self.sprites_idx];
        const base_pattern_addr = @as(usize, sprite.pattern_idx) * 32;

        if (r1.zoomed_sprites) {
            const line_offset = (self.scanline - sprite.y) / 2;
            const pi = base_pattern_addr + (line_offset * 4);
            inline for (0..pattern_columns) |j| { // 4 bitplanes foreach pattern line
                const bp_0 = (self.vram[pi + 0] >> j) & 1; // bitplane 0
                const bp_1 = (self.vram[pi + 1] >> j) & 1; // bitplane 1
                const bp_2 = (self.vram[pi + 2] >> j) & 1; // bitplane 2
                const bp_3 = (self.vram[pi + 3] >> j) & 1; // bitplane 3
                const ij_color = bp_0 | (bp_1 << 1) | (bp_2 << 2) | (bp_3 << 3);
                const x = sprite.x + ((7 - j) * 2);
                if (x < 255 and ij_color > 0 and self.scanline_priority_buffer[x] == false) {
                    const palette_color = 16 + ij_color;
                    const color: u8 = @bitCast(self.cram[palette_color]);
                    self.display_buffer[display_buffer_y + x] = color;
                    self.display_buffer[display_buffer_y + x + 1] = color;
                    self.sprite_collision_buffer[x] = true;
                    if (self.sprite_collision_buffer[x] == true) {
                        self.status_flags.sprite_collision = true;
                    } else {
                        self.sprite_collision_buffer[x] = true;
                    }
                }
            }
        } else {
            const line_offset = self.scanline - sprite.y;
            const pi = base_pattern_addr + (line_offset * 4);
            inline for (0..pattern_columns) |j| { // 4 bitplanes foreach pattern line
                const bp_0 = (self.vram[pi + 0] >> j) & 1; // bitplane 0
                const bp_1 = (self.vram[pi + 1] >> j) & 1; // bitplane 1
                const bp_2 = (self.vram[pi + 2] >> j) & 1; // bitplane 2
                const bp_3 = (self.vram[pi + 3] >> j) & 1; // bitplane 3
                const ij_color = bp_0 | (bp_1 << 1) | (bp_2 << 2) | (bp_3 << 3);
                const x = sprite.x + (7 - j);
                if (x < 256 and ij_color > 0 and self.scanline_priority_buffer[x] == false) {
                    const palette_color = 16 + ij_color;

                    self.display_buffer[display_buffer_y + x] = @bitCast(self.cram[palette_color]);

                    if (self.sprite_collision_buffer[x] == true) {
                        self.status_flags.sprite_collision = true;
                    } else {
                        self.sprite_collision_buffer[x] = true;
                    }
                }
            }
        }

        if (self.sprites_idx == 0) {
            break;
        }
    }
}
