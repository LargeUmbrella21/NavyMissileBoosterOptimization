# streamlit_app.py

import streamlit as st
import numpy as np
import scipy.optimize as opt

# Constants
g = 9.81  # Gravity (m/s^2)
Isp = 250  # Specific impulse (s)
Ve = Isp * g  # Exhaust velocity (m/s)

# Fixed dimensions
OD = 0.75  # Outer diameter (m)
Total_length = 7.5  # Total rocket length (m)
Bulkhead_length = 0.3  # Bulkhead thickness (m)
payload_mass = 250  # Payload mass (kg)

# Sidebar input
st.sidebar.header("Rocket Parameters")

custom_input = st.sidebar.checkbox("Customize Inner Diameter & Densities")
if custom_input:
    ID = st.sidebar.number_input("Inner Diameter (m)", value=0.70, min_value=0.01)
    rho_structure = st.sidebar.number_input("Structure Density (kg/m³)", value=2700)
    rho_fuel = st.sidebar.number_input("Propellant Density (kg/m³)", value=1040)
else:
    ID = 0.70
    rho_structure = 2700
    rho_fuel = 1040

Thickness = OD - ID
bulkhead_volume = (ID**2 * np.pi / 4) * Bulkhead_length
bulkhead_mass = rho_structure * bulkhead_volume

# Mass functions
def structure_mass(length):
    volume = ((Thickness**2) * np.pi / 4) * length
    return rho_structure * volume

def fuel_mass(length):
    volume = ((ID**2) * np.pi / 4) * length
    return rho_fuel * volume

# Objective functions
def optimize_lengths(lengths):
    L1, L2, L3 = lengths
    M_s1, M_s2, M_s3 = structure_mass(L1), structure_mass(L2), structure_mass(L3)
    M_f1, M_f2, M_f3 = fuel_mass(L1), fuel_mass(L2), fuel_mass(L3)

    M0_1 = M_s1 + M_s2 + M_s3 + M_f1 + M_f2 + M_f3 + 3 * bulkhead_mass + payload_mass
    Mf_1 = M0_1 - M_f1
    Mr1 = M0_1 / Mf_1

    M0_2 = M_s2 + M_s3 + M_f2 + M_f3 + 2 * bulkhead_mass + payload_mass
    Mf_2 = M0_2 - M_f2
    Mr2 = M0_2 / Mf_2

    M0_3 = M_s3 + M_f3 + bulkhead_mass + payload_mass
    Mf_3 = M0_3 - M_f3
    Mr3 = M0_3 / Mf_3

    target_ratio = (Mr1 + Mr2 + Mr3) / 3
    return (Mr1 - target_ratio)**2 + (Mr2 - target_ratio)**2 + (Mr3 - target_ratio)**2

# Optimization
initial_lengths = [2.5, 2.5, 2.5]
constraint = ({'type': 'eq', 'fun': lambda x: sum(x) - Total_length})
result = opt.minimize(optimize_lengths, initial_lengths, constraints=constraint, bounds=[(0, 7)] * 3)

L1, L2, L3 = result.x
M_s1, M_s2, M_s3 = structure_mass(L1), structure_mass(L2), structure_mass(L3)
M_f1, M_f2, M_f3 = fuel_mass(L1), fuel_mass(L2), fuel_mass(L3)

# Delta-V calculations
def compute_delta_v(M_s1, M_s2, M_s3, M_f1, M_f2, M_f3):
    M0_1 = M_s1 + M_s2 + M_s3 + M_f1 + M_f2 + M_f3 + 3 * bulkhead_mass + payload_mass
    Mf_1 = M0_1 - M_f1
    DeltaV1 = Ve * np.log(M0_1 / Mf_1)

    M0_2 = M_s2 + M_s3 + M_f2 + M_f3 + 2 * bulkhead_mass + payload_mass
    Mf_2 = M0_2 - M_f2
    DeltaV2 = Ve * np.log(M0_2 / Mf_2)

    M0_3 = M_s3 + M_f3 + bulkhead_mass + payload_mass
    Mf_3 = M0_3 - M_f3
    DeltaV3 = Ve * np.log(M0_3 / Mf_3)

    return DeltaV1, DeltaV2, DeltaV3

DeltaV1, DeltaV2, DeltaV3 = compute_delta_v(M_s1, M_s2, M_s3, M_f1, M_f2, M_f3)

# Output Results
st.title("Rocket Stage Optimizer")

st.subheader("Optimized Stage Lengths")
st.write(f"Stage 1: **{L1:.2f} m**")
st.write(f"Stage 2: **{L2:.2f} m**")
st.write(f"Stage 3: **{L3:.2f} m**")

st.subheader("ΔV for Each Stage")
st.write(f"Stage 1: **{DeltaV1:.2f} m/s**")
st.write(f"Stage 2: **{DeltaV2:.2f} m/s**")
st.write(f"Stage 3: **{DeltaV3:.2f} m/s**")
st.write(f"**Total ΔV: {DeltaV1 + DeltaV2 + DeltaV3:.2f} m/s**")

st.subheader("Mass Breakdown")
for i, (ms, mf) in enumerate([(M_s1, M_f1), (M_s2, M_f2), (M_s3, M_f3)], 1):
    st.write(f"**Stage {i} Structure Mass:** {ms:.2f} kg")
    st.write(f"**Stage {i} Propellant Mass:** {mf:.2f} kg")
    st.write(f"**Stage {i} Bulkhead Mass:** {bulkhead_mass:.2f} kg")
    st.markdown("---")
