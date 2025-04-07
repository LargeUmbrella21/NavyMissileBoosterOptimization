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

# User Inputs for Customization
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
bulkhead_volume = ((ID**2)* np.pi / 4) * Bulkhead_length
bulkhead_mass = rho_structure * bulkhead_volume

# Payload mass (assume a value, can be changed)
payload_mass = 250  # kg

# Objective function: Balance mass ratios
def optimize_lengths(lengths):
    L1, L2, L3 = lengths
    
    # Compute masses
    M_s1, M_s2, M_s3 = structure_mass(L1), structure_mass(L2), structure_mass(L3)
    M_f1, M_f2, M_f3 = fuel_mass(L1), fuel_mass(L2), fuel_mass(L3)
    
    # Mass initial and final for each stage
    M0_1 = M_s1 + M_s2 + M_s3 + M_f1 + M_f2 + M_f3 + 3 * bulkhead_mass + payload_mass
    Mf_1 = M0_1 - M_f1
    Mr1 = M0_1 / Mf_1
    
    
    M0_2 =  M_s2 + M_s3 + M_f2 + M_f3 + 2 * bulkhead_mass + payload_mass
    Mf_2 = M0_2 - M_f2
    Mr2 = M0_2 / Mf_2
    
    M0_3 =  M_s3 + M_f3 + bulkhead_mass + payload_mass
    Mf_3 = M0_3 - M_f3
    Mr3 = M0_3 / Mf_3
    
    # Try to keep mass ratios similar
    target_ratio = (Mr1 + Mr2 + Mr3) / 3
    return (Mr1 - target_ratio)**2 + (Mr2 - target_ratio)**2 + (Mr3 - target_ratio)**2

def maximize_delta_v(lengths):
    L1, L2, L3 = lengths
    
    # Compute masses
    M_s1, M_s2, M_s3 = structure_mass(L1), structure_mass(L2), structure_mass(L3)
    M_f1, M_f2, M_f3 = fuel_mass(L1), fuel_mass(L2), fuel_mass(L3)

    # Compute initial and final masses for each stage
    M0_1 = M_s1 + M_s2 + M_s3 + M_f1 + M_f2 + M_f3 + 3 * bulkhead_mass + payload_mass
    Mf_1 = M0_1 - M_f1
    DeltaV1 = Ve * np.log(M0_1 / Mf_1)

    M0_2 = M_s2 + M_s3 + M_f2 + M_f3 + 2 * bulkhead_mass + payload_mass
    Mf_2 = M0_2 - M_f2
    DeltaV2 = Ve * np.log(M0_2 / Mf_2)

    M0_3 = M_s3 + M_f3 + bulkhead_mass + payload_mass
    Mf_3 = M0_3 - M_f3
    DeltaV3 = Ve * np.log(M0_3 / Mf_3)

    # Maximize total ΔV (by minimizing the negative value)
    return -(DeltaV1 + DeltaV2 + DeltaV3)


# Initial guesses
initial_lengths = [2.5, 2.5, 2.5]
adujusted_length = Total_length-3*Bulkhead_length  #Makes sure bulkhead length is taken into account

# Constraint: Total length must sum to 7.5 m
constraint = ({'type': 'eq', 'fun': lambda x: sum(x) - adujusted_length})


# Optimization
result = opt.minimize(maximize_delta_v, initial_lengths, constraints=constraint, bounds=[(0, 7.5)]*3)
L1_opt, L2_opt, L3_opt = result.x

# Compute optimized masses
M_s1, M_s2, M_s3 = structure_mass(L1_opt), structure_mass(L2_opt), structure_mass(L3_opt)
M_f1, M_f2, M_f3 = fuel_mass(L1_opt), fuel_mass(L2_opt), fuel_mass(L3_opt)

# Compute ΔV for each stage
M0_1 = M_s1 + M_s2 + M_s3 + M_f1 + M_f2 + M_f3 + 3 * bulkhead_mass + payload_mass
Mf_1 = M0_1 - M_f1
DeltaV1 = Ve * np.log(M0_1 / Mf_1)

M0_2 =  M_s2 + M_s3 + M_f2 + M_f3 + 2 * bulkhead_mass + payload_mass
Mf_2 = M_s2 + M_s3 +  M_f3 + 2 * bulkhead_mass + payload_mass
DeltaV2 = Ve * np.log(M0_2 / Mf_2)

M0_3 = M_s3 + M_f3 +  bulkhead_mass + payload_mass
Mf_3 = M0_3 - M_f3
DeltaV3 = Ve * np.log(M0_3 / Mf_3)

#Mass Ratio results
Mr1 = M0_1 / Mf_1
Mr2 = M0_2 / Mf_2
Mr3 = M0_3 / Mf_3

# Print results
print(f"\nOptimized Lengths: L1 = {L1_opt:.2f} m, L2 = {L2_opt:.2f} m, L3 = {L3_opt:.2f} m")
print(f"Delta-V Stage 1: {DeltaV1:.2f} m/s (Mass Ratio: {Mr1:.2f})")
print(f"Delta-V Stage 2: {DeltaV2:.2f} m/s (Mass Ratio: {Mr2:.2f})")
print(f"Delta-V Stage 3: {DeltaV3:.2f} m/s (Mass Ratio: {Mr3:.2f})")
print(f"Total Delta-V: {DeltaV1 + DeltaV2 + DeltaV3:.2f} m/s\n")

# Print individual mass for structure, propellant, and bulkhead for each stage
print(f"Mass of Structure (Stage 1): {M_s1:.2f} kg")
print(f"Mass of Propellant (Stage 1): {M_f1:.2f} kg")
print(f"Mass of Bulkhead (Stage 1): {bulkhead_mass*3:.2f} kg\n")

print(f"Mass of Structure (Stage 2): {M_s2:.2f} kg")
print(f"Mass of Propellant (Stage 2): {M_f2:.2f} kg")
print(f"Mass of Bulkhead (Stage 2): {bulkhead_mass*2:.2f} kg\n")

print(f"Mass of Structure (Stage 3): {M_s3:.2f} kg")
print(f"Mass of Propellant (Stage 3): {M_f3:.2f} kg")
print(f"Mass of Bulkhead (Stage 3): {bulkhead_mass:.2f} kg\n")
