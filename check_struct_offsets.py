#!/usr/bin/env python3
import ctypes as ct
from agent.interfaces_python_data_structs import input_data_str

# Create an instance
s = input_data_str()

# Print offsets of fields around the slip angles
print("Structure field offsets:")
print(f"TrfLightThirdTimeToChange offset: {input_data_str.TrfLightThirdTimeToChange.offset}")
print(f"alpha_rr offset: {input_data_str.alpha_rr.offset}")
print(f"alpha_rl offset: {input_data_str.alpha_rl.offset}")
print(f"alpha_fr offset: {input_data_str.alpha_fr.offset}")
print(f"alpha_fl offset: {input_data_str.alpha_fl.offset}")
print(f"\nTotal structure size: {ct.sizeof(input_data_str)} bytes")

# Calculate expected offsets in C++
print("\n=== Expected offsets in C (from header file) ===")
print("After TrfLightThirdTimeToChange (double = 8 bytes)")
print("alpha_rr should be at: TrfLightThirdTimeToChange_offset + 8")
print("alpha_rl should be at: alpha_rr_offset + 8")
print("alpha_fr should be at: alpha_rl_offset + 8")
print("alpha_fl should be at: alpha_fr_offset + 8")
