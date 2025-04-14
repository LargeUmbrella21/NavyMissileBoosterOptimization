import scipy.optimize as opt
import numpy as np

# Constants
g = 9.81  # Gravity (m/s^2)
Isp = 250  # Specific impulse (s)
Ve = Isp * g  # Exhaust velocity (m/s)

# Rocket dimensions
OD = 0.75  # Outer diameter (m)
ID = 0.6  # Inner diameter (m)
Thickness = (OD - ID)  # Structure thickness (m)
Total_length = 7.5  # Total rocket length (m)
Bulkhead_length = 0.2  # Bulkhead thickness (m)

# Material densities
rho_structure = 2700  # kg/m^3 (Aluminum 6061-T6 for structure)
rho_fuel = 1040  # kg/m^3 (ABS Plastic for fuel)

if input("Would you like to enter your own dimensions? (Y/N): ").strip().upper() == "Y":
    ID = float(input("Enter Inner Diameter (m): "))  # Inner diameter (m)
    rho_structure = float(input("Enter Structure Density (kg/m^3): "))  # Structure density (kg/m^3)
    rho_fuel = float(input("Enter Propellant Density (kg/m^3): "))  # Propellant density (kg/m^3)

# Structural volume function
def structure_mass(length):
    volume = ((Thickness**2) * np.pi / 4) * length
    return rho_structure * volume

# Fuel mass function
def fuel_mass(length):
    volume = ((ID**2) * np.pi / 4) * length  # Full solid grain
    return rho_fuel * volume

# Bulkhead mass
bulkhead_volume = (ID**2 * np.pi / 4) * Bulkhead_length
bulkhead_mass = rho_structure * bulkhead_volume

# Payload mass (assume a value, can be changed)
payload_mass = 250  # kg

# Required first-stage mass ratio
T = 10  # Burn time for first stage (seconds)
required_mass_ratio_1st_stage = np.exp(g * T / (2 * Ve))

# Objective function to maximize total Delta-V while maintaining the mass ratio constraint
def optimize_lengths(lengths):
    L1, L2, L3 = lengths
    
    # Compute masses
    M_s1, M_s2, M_s3 = structure_mass(L1), structure_mass(L2), structure_mass(L3)
    M_f1, M_f2, M_f3 = fuel_mass(L1), fuel_mass(L2), fuel_mass(L3)
    
    # Compute initial and final masses for each stage
    M0_1 = M_s1 + M_s2 + M_s3 + M_f1 + M_f2 + M_f3 + 3 * bulkhead_mass + payload_mass
    Mf_1 = M0_1 - M_f1
    M_r1 = M0_1 / Mf_1
    
    M0_2 = M_s2 + M_s3 + M_f2 + M_f3 + 2 * bulkhead_mass + payload_mass
    Mf_2 = M0_2 - M_f2
    M_r2 = M0_2 / Mf_2
    
    M0_3 = M_s3 + M_f3 + bulkhead_mass + payload_mass
    Mf_3 = M0_3 - M_f3
    M_r3 = M0_3 / Mf_3
    
    # Compute Delta-V for each stage
    DeltaV1 = Ve * np.log(M0_1 / Mf_1)
    DeltaV2 = Ve * np.log(M0_2 / Mf_2)
    DeltaV3 = Ve * np.log(M0_3 / Mf_3)
    
    # Constraint penalty for maintaining first-stage mass ratio
    penalty = (M_r1 - required_mass_ratio_1st_stage) ** 2 * 1000000  # Large weight to enforce constraint
    
    # Maximize total Delta-V (by minimizing the negative value plus penalty)
    target_ratio = (M_r2 + M_r3) / 3
    return (M_r2 - target_ratio)**2 + (M_r3 - target_ratio)**2 +penalty

# Initial guesses
initial_lengths = [2.5, 2.5, 2.5]
adjusted_length = Total_length - 3 * Bulkhead_length  # Ensures bulkhead length is considered

# Constraint: Total length must sum to the adjusted length
constraint = ({'type': 'eq', 'fun': lambda x: sum(x) - adjusted_length})

# Optimization
result = opt.minimize(optimize_lengths, initial_lengths, constraints=constraint, bounds=[(0, Total_length)] * 3)
L1_opt, L2_opt, L3_opt = result.x

# Compute optimized masses
M_s1, M_s2, M_s3 = structure_mass(L1_opt), structure_mass(L2_opt), structure_mass(L3_opt)
M_f1, M_f2, M_f3 = fuel_mass(L1_opt), fuel_mass(L2_opt), fuel_mass(L3_opt)

# Compute final Delta-V values
M0_1 = M_s1 + M_s2 + M_s3 + M_f1 + M_f2 + M_f3 + 3 * bulkhead_mass + payload_mass
Mf_1 = M0_1 - M_f1
DeltaV1 = Ve * np.log(M0_1 / Mf_1)

M0_2 = M_s2 + M_s3 + M_f2 + M_f3 + 2 * bulkhead_mass + payload_mass
Mf_2 = M0_2 - M_f2
DeltaV2 = Ve * np.log(M0_2 / Mf_2)

M0_3 = M_s3 + M_f3 + bulkhead_mass + payload_mass
Mf_3 = M0_3 - M_f3
DeltaV3 = Ve * np.log(M0_3 / Mf_3)

# Mass Ratios
M_r1 = M0_1 / Mf_1
M_r2 = M0_2 / Mf_2
M_r3 = M0_3 / Mf_3

# Print results
print(f"\nOptimized Lengths: L1 = {L1_opt:.2f} m, L2 = {L2_opt:.2f} m, L3 = {L3_opt:.2f} m")
print(f"Delta-V Stage 1: {DeltaV1:.2f} m/s (Mass Ratio: {M_r1:.2f})")
print(f"Delta-V Stage 2: {DeltaV2:.2f} m/s (Mass Ratio: {M_r2:.2f})")
print(f"Delta-V Stage 3: {DeltaV3:.2f} m/s (Mass Ratio: {M_r3:.2f})")
print(f"Total Delta-V: {DeltaV1 + DeltaV2 + DeltaV3:.2f} m/s\n")
