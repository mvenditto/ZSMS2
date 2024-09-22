const std = @import("std");

pub const Operation = enum(u8) {
    illegal, //
    scf,
    rld,
    cp,
    in,
    rst,
    cpd,
    cpdr,
    inc,
    out,
    jp,
    ret,
    sbc,
    cpir,
    outd,
    daa,
    or_,
    rlca,
    rr,
    xor,
    reti,
    nop,
    ei,
    adc,
    ccf,
    halt,
    inir,
    lddr,
    jr,
    rra,
    exx,
    add,
    retn,
    neg,
    di,
    rrd,
    indr,
    ldd,
    im,
    and_,
    ldi,
    sla,
    sra,
    ldir,
    ex,
    rrca,
    rla,
    call,
    ld,
    cpi,
    outi,
    rlc,
    ind,
    bit,
    djnz,
    sub,
    pop,
    res,
    ini,
    set,
    sll,
    cpl,
    rl,
    rrc,
    srl,
    dec,
    push,
    otdr,
    otir,
};

pub const Condition = enum(u8) {
    nz = 0,
    z = 1,
    nc = 2,
    c = 3,
    po = 4,
    pe = 5,
    p = 6,
    m = 7,
};

/// 8-bit register operand
pub const Reg8 = enum(u8) {
    B = 0,
    C = 1,
    D = 2,
    E = 3,
    H = 4,
    L = 5,
    A = 7,
    IXh = 8,
    IXl = 9,
    IYh = 10,
    IYl = 11,
    I = 12,
    R = 13,
};

/// 16-bit register (register pair) operand.
pub const Reg16 = enum(u8) {
    BC = 0,
    DE = 1,
    HL = 2,
    SP = 3,
    BC_ = 4,
    DE_ = 5,
    HL_ = 6,
    AF_ = 7,
    IX = 8,
    IY = 9,
    AF = 10,
};

pub const OperandValueType = enum {
    reg_8,
    reg_16,
    imm_8,
    imm_16,
    ind_16,
    ind_reg_16,
    displ,
    literal,
};

pub const Operand = union(OperandValueType) {
    reg_8: Reg8,
    reg_16: Reg16,
    imm_8: void,
    imm_16: u16,
    ind_16: void,
    ind_reg_16: Reg16,
    displ: i8,
    literal: u8,
};

pub const DecodedOpCode = struct {
    offset: u32 = 0x0,
    prefix: u16 = 0x0,
    opcode: u8 = 0x0,
    seq_len: u8 = 0x0,
    prefix_len: u8 = 0x0,
    displacement: bool = false,
    immediate_data: ?u16 = null, // n (u8) or nn (u16)
    operand1: ?Operand = null,
    operand2: ?Operand = null,
    op: Operation = .illegal,
    cc: ?Condition = undefined,
};

/// Returns the size in bytes of immediate data + displacement byte, if present.
pub fn get_data_len(i: *const DecodedOpCode) u8 {
    var bytes: u8 = 0;

    if (i.operand1) |op1| {
        bytes += switch (op1) {
            .imm_8 => 1, // n
            .imm_16 => 2, // nn
            .ind_16 => 2, // (nn)
            else => 0,
        };
    }

    if (i.operand2) |op2| {
        bytes += switch (op2) {
            .imm_8 => 1, // n
            .imm_16 => 2, // nn
            .ind_16 => 2, // (nn)
            else => 0,
        };
    }

    if (i.displacement == true) {
        bytes += 1;
    }

    return bytes;
}

pub fn fmt_decoded_insn(buf: []u8, i: *const DecodedOpCode) ![]u8 {
    // const buf = try allocator.alloc(u8, 32);

    var offset: usize = 0;

    for (0..buf.len) |x| buf[x] = " "[0];

    const op_name = @tagName(i.op);

    const op_name_slice = std.ascii.upperString(
        buf,
        if (i.op == .or_ or i.op == .and_) op_name[0 .. op_name.len - 1] else op_name,
    );

    offset += op_name_slice.len - 1;

    if (i.cc) |cond| {
        const cond_name = @tagName(cond);
        _ = std.ascii.upperString(buf[offset + 2 ..], cond_name);
        offset += cond_name.len + 1;
    }

    var lhs_len: u8 = 0;

    if (i.operand1) |lhs| {
        offset += 2;
        var slice = buf[offset..];
        switch (lhs) {
            .literal => |b| {
                const hex = try std.fmt.bufPrint(slice, "${x:0>2}", .{b});
                lhs_len = @intCast(hex.len);
            },
            .displ => |d| {
                // slice[0] = "d"[0];
                if (d < 0) {
                    const printed = try std.fmt.bufPrint(slice, "{d}", .{d});
                    lhs_len = @intCast(printed.len);
                } else {
                    slice[0] = "+"[0];
                    const printed = try std.fmt.bufPrint(slice[1..], "{d}", .{d});
                    lhs_len = @intCast(printed.len + 1);
                }
            },
            .imm_8 => {
                slice[0] = "n"[0];
                lhs_len = 1;
            },
            .imm_16 => |nn| {
                // std.mem.copyForwards(u8, slice, "nn");
                _ = try std.fmt.bufPrint(slice, "{X:0>4}h", .{nn});
                lhs_len = 5;
            },
            .ind_16 => {
                std.mem.copyForwards(u8, slice, "(nn)");
                lhs_len = 4;
            },
            .reg_8 => |r8| {
                std.mem.copyForwards(u8, slice, @tagName(r8));
                lhs_len = 1;
            },
            .reg_16 => |r16| {
                const reg_name = @tagName(r16);
                std.mem.copyForwards(u8, slice, reg_name);
                if (reg_name.len == 3) { // replace the _ in alternative registers with an ' (apex).
                    slice[2] = "'"[0];
                }
                lhs_len = @as(u8, @intCast(reg_name.len));
            },
            .ind_reg_16 => |r16| {
                const reg_name = @tagName(r16);
                lhs_len = @as(u8, @intCast(2 + reg_name.len));
                slice[0] = "("[0];
                std.mem.copyForwards(u8, slice[1..], reg_name);
                if (i.displacement == true and (r16 == .IX or r16 == .IY)) {
                    slice[3] = "+"[0];
                    slice[4] = "d"[0];
                    slice[1 + reg_name.len + 2] = ")"[0];
                    lhs_len += 2;
                } else {
                    if (reg_name.len == 3) { // replace the _ in alternative registers with an ' (apex).
                        slice[3] = "'"[0];
                    }
                    slice[1 + reg_name.len + 1] = ")"[0];
                }
            },
        }

        offset += lhs_len;

        if (i.operand2) |rhs| {
            buf[offset] = ","[0];
            buf[offset + 1] = " "[0];
            offset += 2;
            slice = buf[offset..];
            var rhs_len: u8 = 0;
            switch (rhs) {
                .literal => |b| {
                    const hex = try std.fmt.bufPrint(slice, "${x:0>2}", .{b});
                    rhs_len = @intCast(hex.len);
                },
                .displ => |d| {
                    // slice[0] = "d"[0];
                    if (d < 0) {
                        const printed = try std.fmt.bufPrint(slice, "{d}", .{d});
                        rhs_len = @intCast(printed.len);
                    } else {
                        slice[0] = "+"[0];
                        const printed = try std.fmt.bufPrint(slice[1..], "{d}", .{d});
                        rhs_len = @intCast(printed.len + 1);
                    }
                },
                .imm_8 => {
                    slice[0] = "n"[0];
                    rhs_len = 1;
                },
                .imm_16 => |nn| {
                    // std.mem.copyForwards(u8, slice, "nn");
                    _ = try std.fmt.bufPrint(slice, "{X:0>4}h", .{nn});
                    rhs_len = 5;
                },
                .ind_16 => {
                    std.mem.copyForwards(u8, slice, "(nn)");
                    rhs_len = 4;
                },
                .reg_8 => |r8| {
                    std.mem.copyForwards(u8, slice, @tagName(r8));
                    rhs_len = 1;
                },
                .reg_16 => |r16| {
                    const reg_name = @tagName(r16);
                    std.mem.copyForwards(u8, slice, reg_name);
                    if (reg_name.len == 3) { // replace the _ in alternative registers with an ' (apex).
                        slice[2] = "'"[0];
                    }
                    rhs_len = @as(u8, @intCast(reg_name.len));
                },
                .ind_reg_16 => |r16| {
                    const reg_name = @tagName(r16);
                    rhs_len = @as(u8, @intCast(2 + reg_name.len));
                    slice[0] = "("[0];
                    std.mem.copyForwards(u8, slice[1..], reg_name);
                    if (i.displacement == true and (r16 == .IX or r16 == .IY)) {
                        slice[3] = "+"[0];
                        slice[4] = "d"[0];
                        slice[1 + reg_name.len + 2] = ")"[0];
                        rhs_len += 2;
                    } else {
                        if (reg_name.len == 3) { // replace the _ in alternative registers with an ' (apex).
                            slice[3] = "'"[0];
                        }
                        slice[1 + reg_name.len + 1] = ")"[0];
                    }
                },
            }
            offset += rhs_len;
        }
    }

    buf[offset] = 0;

    return buf[0 .. offset + 1];
}

/// see: http://www.z80.info/decoding.htm
const OpCode = packed struct(u8) {
    z: u3, // 0-1-2
    y: packed struct(u3) {
        q: u1, // 3
        p: u2, // 4-5
    },
    x: u2, // 6-7
};

fn decode_cb(opcode: *const OpCode) DecodedOpCode {
    const y: u3 = @bitCast(opcode.y);

    var i = DecodedOpCode{
        .prefix = 0xCB,
        .opcode = @bitCast(opcode.*),
    };

    switch (opcode.x) {
        0 => { // Roll/shift register or memory location (rot[y] r[z])

            if (opcode.z == 6) { // (HL)
                i.operand1 = .{ .ind_reg_16 = .HL };
            } else {
                i.operand1 = .{ .reg_8 = @enumFromInt(opcode.z) };
            }

            i.op = switch (y) {
                0 => .rlc,
                1 => .rrc,
                2 => .rl,
                3 => .rr,
                4 => .sla,
                5 => .sra,
                6 => .sll,
                7 => .srl,
            };
        },
        1 => { // BIT y, r[z]
            i.op = .bit;
            i.operand1 = .{ .literal = y };
            if (opcode.z == 6) { // (HL)
                i.operand2 = .{ .ind_reg_16 = .HL };
            } else {
                i.operand2 = .{ .reg_8 = @enumFromInt(opcode.z) };
            }
        },
        2 => { // RES y, r[z]
            i.op = .res;
            i.operand1 = .{ .literal = y };
            if (opcode.z == 6) { // (HL)
                i.operand2 = .{ .ind_reg_16 = .HL };
            } else {
                i.operand2 = .{ .reg_8 = @enumFromInt(opcode.z) };
            }
        },
        3 => { // SET y, r[z]
            i.op = .set;
            i.operand1 = .{ .literal = y };
            if (opcode.z == 6) { // (HL)
                i.operand2 = .{ .ind_reg_16 = .HL };
            } else {
                i.operand2 = .{ .reg_8 = @enumFromInt(opcode.z) };
            }
        },
    }

    return i;
}

fn decode_ed(opcode: *const OpCode) DecodedOpCode {
    const y: u3 = @bitCast(opcode.y);
    const y_: usize = @intCast(y);

    var i = DecodedOpCode{
        .prefix = 0xED,
        .opcode = @bitCast(opcode.*),
    };

    if (opcode.x == 1) {
        switch (opcode.z) {
            0 => { // Input from port with 16-bit address
                i.op = .in;
                if (y == 6) { // IN (C)
                    i.operand1 = .{ .reg_8 = .C };
                } else { // IN r[y], (C)
                    if (y_ == 6) { // (HL)
                        i.operand1 = .{ .ind_reg_16 = .HL };
                    } else {
                        i.operand1 = .{ .reg_8 = @enumFromInt(y_) };
                    }
                    i.operand2 = .{ .reg_8 = .C };
                }
            },
            1 => { // Output from port with 16-bit address
                i.op = .out;
                i.operand1 = .{ .reg_8 = .C };
                if (y == 6) { // OUT (C), r[y]
                    if (y_ == 6) { // (HL)
                        i.operand2 = .{ .ind_reg_16 = .HL };
                    } else {
                        i.operand2 = .{ .reg_8 = @enumFromInt(y_) };
                    }
                } else { // OUT (C), 0
                    i.operand2 = .{ .literal = 0 };
                }
            },
            2 => { // 16-bit add/subtract with carry
                i.operand1 = .{ .reg_16 = .HL };
                i.operand2 = .{ .reg_16 = @enumFromInt(opcode.y.p) };
                if (opcode.y.q == 0) { // SBC HL, rp[p]
                    i.op = .sbc;
                } else { // ADC HL, rp[p]
                    i.op = .adc;
                }
            },
            3 => { // Retrieve/store register pair from/to immediate address
                i.op = .ld;
                if (opcode.y.q == 0) { // LD (nn), rp[p]
                    i.operand1 = .ind_16;
                    i.operand2 = .{ .reg_16 = @enumFromInt(opcode.y.p) };
                } else { // LD rp[p], (nn)
                    i.operand2 = .ind_16;
                    i.operand1 = .{ .reg_16 = @enumFromInt(opcode.y.p) };
                }
            },
            4 => { // Negate accumulator (NEG)
                i.op = .neg;
            },
            5 => { // Return from interrupt (RETN or RETI)
                i.op = if (y == 1) .retn else .reti;
            },
            6 => { // Set interrupt mode (IM im[y])
                i.op = .im;
                i.operand1 = .{ .literal = y };
            },
            7 => { // Assorted ops
                switch (y) {
                    0 => { // LD I, A
                        i.op = .ld;
                        i.operand1 = .{ .reg_8 = .I };
                        i.operand2 = .{ .reg_8 = .A };
                    },
                    1 => { // LD R, A
                        i.operand1 = .{ .reg_8 = .R };
                        i.operand2 = .{ .reg_8 = .A };
                    },
                    2 => { // LD A, I
                        i.operand1 = .{ .reg_8 = .A };
                        i.operand2 = .{ .reg_8 = .I };
                    },
                    3 => { // LD A, R
                        i.operand1 = .{ .reg_8 = .A };
                        i.operand2 = .{ .reg_8 = .R };
                    },
                    4 => i.op = .rrd,
                    5 => i.op = .rld,
                    6 => i.op = .nop,
                    7 => i.op = .nop,
                }
            },
        }
    } else if (opcode.x == 2) {
        if (opcode.z <= 3 and y >= 4) { // Block instruction
            switch (y) {
                4 => {
                    i.op = switch (opcode.z) {
                        0 => .ldi,
                        1 => .cpi,
                        2 => .ini,
                        3 => .outi,
                        else => .nop,
                    };
                },
                5 => {
                    i.op = switch (opcode.z) {
                        0 => .ldd,
                        1 => .cpd,
                        2 => .ind,
                        3 => .outd,
                        else => .nop,
                    };
                },
                6 => {
                    i.op = switch (opcode.z) {
                        0 => .ldir,
                        1 => .cpir,
                        2 => .inir,
                        3 => .otir,
                        else => .nop,
                    };
                },
                7 => {
                    i.op = switch (opcode.z) {
                        0 => .lddr,
                        1 => .cpdr,
                        2 => .indr,
                        3 => .otdr,
                        else => .nop,
                    };
                },
                else => {},
            }
        } else { //
            i.op = .illegal;
        }
    } else {
        i.op = .illegal;
    }

    return i;
}

fn read_nn(seq: []u8) u16 {
    const low = seq[0];
    const high = seq[1];
    const nn: u16 = @intCast(low | @as(u16, high) << 8);
    return nn;
}

pub fn decode_unprefixed_dd_fd(opcode: *const OpCode, seq: []u8, prefix: u16) DecodedOpCode {
    const y: u3 = @bitCast(opcode.y);
    const y_: usize = @intCast(y);

    var i = DecodedOpCode{
        .opcode = @bitCast(opcode.*),
    };

    switch (opcode.x) {
        0 => {
            switch (opcode.z) {
                0 => { // Relative jumps and assorted ops
                    switch (y) {
                        0 => i.op = .nop,
                        1 => { // EX AF, AF
                            i.op = .ex;
                            i.operand1 = .{ .reg_16 = .AF };
                            i.operand2 = .{ .reg_16 = .AF_ };
                        },
                        2 => { // DJNZ d
                            i.op = .djnz;
                            i.operand1 = .{ .displ = @bitCast(seq[0]) };
                            i.displacement = true;
                        },
                        3 => { // JR d
                            i.op = .jr;
                            i.operand1 = .{ .displ = @bitCast(seq[0]) };
                            i.displacement = true;
                        },
                        4...7 => { // JR cc[y-4], d
                            i.op = .jr;
                            i.operand1 = .{ .displ = @bitCast(seq[0]) };
                            i.displacement = true;
                            i.cc = @enumFromInt(y - 4);
                        },
                    }
                },
                1 => { // 16-bit load immediate/add
                    if (opcode.y.q == 0) { // LD rp[p], nn
                        i.op = .ld;
                        i.operand1 = .{ .reg_16 = @enumFromInt(opcode.y.p) };
                        i.operand2 = .{ .imm_16 = read_nn(seq) };
                    } else { // ADD HL, rp[p]
                        i.op = .ld;
                        i.operand1 = .{ .reg_16 = .HL };
                        i.operand2 = .{ .reg_16 = @enumFromInt(opcode.y.p) };
                    }
                },
                2 => { // Indirect loading
                    if (opcode.y.q == 0) {
                        i.op = .ld;
                        switch (opcode.y.p) {
                            0 => { // LD (BC), A
                                i.operand1 = .{ .ind_reg_16 = .BC };
                                i.operand2 = .{ .reg_8 = .A };
                            },
                            1 => { // LD (DE), A
                                i.operand1 = .{ .ind_reg_16 = .DE };
                                i.operand2 = .{ .reg_8 = .A };
                            },
                            2 => { // LD (nn), HL
                                i.operand1 = .ind_16;
                                i.operand2 = .{ .reg_16 = .HL };
                            },
                            3 => { // LD (nn), A
                                i.operand1 = .ind_16;
                                i.operand2 = .{ .reg_8 = .A };
                            },
                        }
                    } else {
                        switch (opcode.y.p) {
                            0 => { // LD (BC), A
                                i.operand1 = .{ .reg_8 = .A };
                                i.operand2 = .{ .ind_reg_16 = .BC };
                            },
                            1 => { // LD (DE), A
                                i.operand1 = .{ .reg_8 = .A };
                                i.operand2 = .{ .ind_reg_16 = .DE };
                            },
                            2 => { // LD (nn), HL
                                i.operand1 = .{ .reg_16 = .HL };
                                i.operand2 = .ind_16;
                            },
                            3 => { // LD (nn), A
                                i.operand1 = .{ .reg_8 = .A };
                                i.operand2 = .ind_16;
                            },
                        }
                    }
                },
                3 => { // 16-bit INC/DEC (INC|DEC rp[p])
                    i.op = if (opcode.y.q == 0) .inc else .dec;
                    i.operand1 = .{ .reg_16 = @enumFromInt(opcode.y.p) };
                },
                4 => { // 8-bit INC (INC r[y])
                    i.op = .inc;
                    if (y_ == 6) { // (HL)
                        i.operand1 = .{ .ind_reg_16 = .HL };
                    } else {
                        i.operand1 = .{ .reg_8 = @enumFromInt(y_) };
                    }
                },
                5 => { // 8-bit DEC (DEC r[y])
                    i.op = .dec;
                    if (y_ == 6) { // (HL)
                        i.operand1 = .{ .ind_reg_16 = .HL };
                    } else {
                        i.operand1 = .{ .reg_8 = @enumFromInt(y_) };
                    }
                },
                6 => { // 8-bit load immediate (LD r[y], n)
                    i.op = .ld;
                    if (y_ == 6) { // (HL)
                        i.operand1 = .{ .ind_reg_16 = .HL };
                    } else {
                        i.operand1 = .{ .reg_8 = @enumFromInt(y_) };
                    }
                    i.operand2 = .imm_8;
                },
                7 => { // Assorted operations on accumulator/flags
                    i.op = switch (y) {
                        0 => .rlca,
                        1 => .rrca,
                        2 => .rla,
                        3 => .rra,
                        4 => .daa,
                        5 => .cpl,
                        6 => .scf,
                        7 => .ccf,
                    };
                },
            }
        },
        1 => {
            if (y == 6 and opcode.z == 6) { // Exception (replaces LD (HL), (HL))
                i.op = .halt;
            } else { // 8-bit loading (LD r[y], r[z])
                i.op = .ld;
                if (y == 6) { // (HL)
                    i.operand1 = .{ .ind_reg_16 = .HL };
                } else {
                    i.operand1 = .{ .reg_8 = @enumFromInt(y) };
                }
                if (opcode.z == 6) { // (HL)
                    i.operand2 = .{ .ind_reg_16 = .HL };
                } else {
                    i.operand2 = .{ .reg_8 = @enumFromInt(opcode.z) };
                }
            }
        },
        2 => { // Operate on accumulator and register/memory location
            if (opcode.z == 6) { // (HL)
                i.operand1 = .{ .ind_reg_16 = .HL };
            } else {
                i.operand1 = .{ .reg_8 = @enumFromInt(opcode.z) };
            }
            switch (y) { // "ADD A", "ADC A", "SUB", "SBC A", "AND", "XOR", "OR", "CP"
                0 => i.op = .add,
                1 => i.op = .adc,
                2 => i.op = .sub,
                3 => i.op = .sbc,
                4 => i.op = .and_,
                5 => i.op = .xor,
                6 => i.op = .or_,
                7 => i.op = .cp,
            }
        },
        3 => {
            switch (opcode.z) {
                0 => { // Conditional return (RET cc[y])
                    i.op = .ret;
                    i.cc = @enumFromInt(y);
                },
                1 => {
                    if (opcode.y.q == 0) { // POP rp2[p]
                        i.op = .pop;
                        const reg: Reg16 = if (opcode.y.p == 3) .AF else @enumFromInt(@as(u4, opcode.y.p));
                        i.operand1 = .{ .reg_16 = reg };
                    } else { // POP & various ops
                        switch (opcode.y.p) {
                            0 => i.op = .ret,
                            1 => i.op = .exx,
                            2 => {
                                i.op = .jp;
                                i.operand1 = .{ .reg_16 = .HL };
                            },
                            3 => {
                                i.op = .ld;
                                i.operand1 = .{ .reg_16 = .SP };
                                i.operand2 = .{ .reg_16 = .HL };
                            },
                        }
                    }
                },
                2 => { // Conditional jump (JP cc[y], nn)
                    i.op = .jp;
                    i.cc = @enumFromInt(y);
                    i.operand1 = .{ .imm_16 = read_nn(seq) };
                },
                3 => { // Assorted operations
                    switch (y) {
                        0 => { // JP nn
                            i.op = .jp;
                            i.operand1 = .{ .imm_16 = read_nn(seq) };
                        },
                        1 => {}, // CB-prefix
                        2 => { // OUT (n), A
                            i.op = .out;
                            i.operand1 = .imm_8;
                            i.operand2 = .{ .reg_8 = .A };
                        },
                        3 => { // IN A, (n)
                            i.op = .in;
                            i.operand1 = .{ .reg_8 = .A };
                            i.operand2 = .imm_8;
                        },
                        4 => { // EX (SP), HL
                            i.op = .ex;
                            i.operand1 = .{ .ind_reg_16 = .SP };
                            i.operand2 = .{ .reg_16 = .HL };
                        },
                        5 => { // EX DE, HL
                            i.op = .ex;
                            i.operand1 = .{ .reg_16 = .DE };
                            i.operand2 = .{ .reg_16 = .HL };
                        },
                        6 => i.op = .di,
                        7 => i.op = .ei,
                    }
                },
                4 => { // Conditional call (CALL {s}, nn)
                    i.op = .call;
                    i.cc = @enumFromInt(y);
                    i.operand1 = .{ .imm_16 = read_nn(seq) };
                },
                5 => {
                    if (opcode.y.q == 0) { // Conditional return (PUSH rp2[p])
                        i.op = .push;
                        const reg: Reg16 = if (opcode.y.p == 3) .AF else @enumFromInt(@as(u4, opcode.y.p));
                        i.operand1 = .{ .reg_16 = reg };
                    } else { // PUSH & various ops
                        switch (opcode.y.p) {
                            0 => { // CALL nn
                                i.op = .call;
                                i.operand1 = .ind_16;
                            },
                            1 => std.debug.print("!!! DD PREFIX !!!\n", .{}), // TODO: DD-prefix
                            2 => std.debug.print("!!! ED PREFIX !!!\n", .{}), // TODO: DD-prefix,
                            3 => std.debug.print("!!! FD PREFIX !!!\n", .{}), // TODO: FD-prefix
                        }
                    }
                },
                6 => { // Operate on accumulator and immediate operand (alu[y] n)
                    i.operand1 = .imm_8;
                    switch (y) { // "ADD A", "ADC A", "SUB", "SBC A", "AND", "XOR", "OR", "CP"
                        0 => i.op = .add,
                        1 => i.op = .adc,
                        2 => i.op = .sub,
                        3 => i.op = .sbc,
                        4 => i.op = .and_,
                        5 => i.op = .xor,
                        6 => i.op = .or_,
                        7 => i.op = .cp,
                    }
                },
                7 => { // restart (RST y*8)
                    i.op = .rst;
                    i.operand1 = .{ .literal = @as(u8, y) * 8 };
                },
            }
        },
    }

    var indexed = false;

    if (prefix == 0xfd or prefix == 0xdd) {
        if (i.opcode == 0xEB) { // EX DE, HL is an exception
            return i;
        }

        // lhs: (HL) -> (IX|IY+d)
        if (i.operand1) |lhs| {
            switch (lhs) {
                .ind_reg_16 => |reg_16_i| {
                    if (reg_16_i == .HL) {
                        indexed = true;
                        i.operand1 = .{ .ind_reg_16 = if (prefix == 0xdd) .IX else .IY };
                        i.displacement = true;
                    }
                },
                else => {},
            }
        }

        // rhs: (HL) -> (IX|IY+d)
        if (i.operand2) |rhs| {
            switch (rhs) {
                .ind_reg_16 => |reg_16_i| {
                    if (reg_16_i == .HL) {
                        indexed = true;
                        i.operand2 = .{ .ind_reg_16 = if (prefix == 0xdd) .IX else .IY };
                        i.displacement = true;
                    }
                },
                else => {},
            }
        }

        // if (HL) was not replaced by IX+d or IY+d,
        // replace H, L and HL with (IX|IY)h, (IX|IY)l, (IX|IY)
        if (indexed == false) {
            if (i.operand1) |lhs| {
                switch (lhs) {
                    .reg_8 => |r8| {
                        i.operand1 = .{ .reg_8 = switch (r8) {
                            .H => if (prefix == 0xdd) .IXh else .IYh,
                            .L => if (prefix == 0xdd) .IXl else .IYl,
                            else => r8,
                        } };
                    },
                    .reg_16 => |r16| {
                        i.operand1 = .{ .reg_16 = switch (r16) {
                            .HL => if (prefix == 0xdd) .IX else .IY,
                            else => r16,
                        } };
                    },
                    else => {},
                }
            }

            if (i.operand2) |rhs| {
                switch (rhs) {
                    .reg_8 => |r8| {
                        i.operand2 = .{ .reg_8 = switch (r8) {
                            .H => if (prefix == 0xdd) .IXh else .IYh,
                            .L => if (prefix == 0xdd) .IXl else .IYl,
                            else => r8,
                        } };
                    },
                    .reg_16 => |r16| {
                        i.operand2 = .{ .reg_16 = switch (r16) {
                            .HL => if (prefix == 0xdd) .IX else .IY,
                            else => r16,
                        } };
                    },
                    else => {},
                }
            }
        }
    }

    return i;
}

pub const DecodeError = error{IncompletePrefixSequence};

pub fn decode(code: []u8) DecodeError!DecodedOpCode {
    var pc: usize = 0;
    var data = code[pc];
    var prefix: u16 = 0x0;
    var prefix_len: u8 = 0;

    if (data == 0xFD or data == 0xDD) {
        while (data == 0xFD or data == 0xDD or data == 0xED) {
            prefix = data;
            pc += 1;
            prefix_len += 1;
            if (pc >= code.len) {
                break;
            }
            data = code[pc];
        }

        if (data == 0xfd or data == 0xdd or data == 0xed) {
            std.debug.print("  Incomplete prefix sequence: {any}\n", .{std.fmt.fmtSliceHexUpper(code)});
            return DecodeError.IncompletePrefixSequence;
        }
    } else if (data == 0xCB) {
        prefix = 0xCB;
        pc += 1;
        data = code[pc];
    } else if (data == 0xED) {
        prefix = 0xED;
        pc += 1;
        data = code[pc];
    }

    // DDCB - FDCB prefixed
    if (data == 0xCB and (prefix == 0xFD or prefix == 0xDD)) {
        prefix = prefix << 8 | 0xCB;
        pc += 1;
        if (pc >= code.len) {
            std.debug.print("  Incomplete prefix sequence: {any}\n", .{std.fmt.fmtSliceHexUpper(code)});
            return DecodeError.IncompletePrefixSequence;
        }
        data = code[pc];
    }

    const opcode: OpCode = @bitCast(data);

    var i = switch (prefix) {
        0xfd => decode_unprefixed_dd_fd(&opcode, code[pc..], prefix),
        0xdd => decode_unprefixed_dd_fd(&opcode, code[pc..], prefix),
        0xed => decode_ed(&opcode),
        0xcb => decode_cb(&opcode),
        else => decode_unprefixed_dd_fd(&opcode, code[pc..], 0x0),
    };

    i.seq_len = @as(u8, @intCast(pc)) + 1 + get_data_len(&i);
    i.prefix_len = prefix_len;

    return i;
}
