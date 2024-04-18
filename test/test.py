# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: MIT


import cocotb
from cocotb.triggers import RisingEdge, Timer
from cocotb.clock import Clock

@cocotb.test()
async def test_tt_um_fpu(dut):
    # Start Clock
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    # Reset
    dut.rst_n.value = 0
    dut.ena.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await Timer(5, units="ns")
    dut.rst_n.value = 1

    # Test input sequence
    inputs = [
        (0x40, 0x0c, 0xcc, 0xcd),
        (0x40, 0x53, 0x33, 0x33),
        (0x40, 0x8c, 0xcc, 0xcd),
        (0x40, 0xb0, 0x00, 0x00)
    ]

    for byte_group in inputs:
        for byte in byte_group:
            await RisingEdge(dut.clk)
            dut.ui_in.value = byte

    # Wait and finish the simulation
    await Timer(100, units='ns')

