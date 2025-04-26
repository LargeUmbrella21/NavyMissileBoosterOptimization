import scipy.optimize as opt
import numpy as np

# Constants
g = 9.81
Isp = 248
Ve = Isp * g

# Geometry
OD = 0.75
ID = 0.6
Thickness = OD - ID
Total_length = 7.5
Bulkhead_length = 0.3
Bulkhead_density = 2700

# Material Densities
rho_structure = 2700
rho_fuel = 985

# Fixed values
payload_mass = 250
T = 10  # target delta-v stage 1 contribution
required_mass_ratio_1st_stage = np.exp(T / (2 * Isp))

# Optional manual override
if input("Would you like to enter your own dimensions? (Y/N): ").strip().upper() == "Y":
    ID = float(input("Enter Inner Diameter (m): "))
    rho_structure = float(input("Enter Structure Density (kg/m^3): "))
    rho_fuel = float(input("Enter Propellant Density (kg/m^3): "))
    Isp = float(input("Enter ISP: "))
    Bulkhead_density = float(input("Enter Bulkhead Density (kg/m^3): "))
    Ve = Isp * g
    Thickness = OD - ID

# Volume-based mass functions
def structure_mass(L): return rho_structure * ((Thickness**2) * np.pi / 4) * L
def fuel_mass(L): return rho_fuel * ((ID**2) * np.pi / 4) * L

# Bulkhead
bulkhead_volume = (ID**2) * np.pi / 4 * Bulkhead_length
bulkhead_mass = Bulkhead_density * bulkhead_volume

# Objective function: Maximize total delta-v
def objective(lengths):
    L1, L2, L3 = lengths
    M_s1, M_s2, M_s3 = structure_mass(L1), structure_mass(L2), structure_mass(L3)
    M_f1, M_f2, M_f3 = fuel_mass(L1), fuel_mass(L2), fuel_mass(L3)
    
    M0_1 = M_s1 + M_s2 + M_s3 + M_f1 + M_f2 + M_f3 + 3 * bulkhead_mass + payload_mass
    Mf_1 = M0_1 - M_f1
    M0_2 = M_s2 + M_s3 + M_f2 + M_f3 + 2 * bulkhead_mass + payload_mass
    Mf_2 = M0_2 - M_f2
    M0_3 = M_s3 + M_f3 + bulkhead_mass + payload_mass
    Mf_3 = M0_3 - M_f3

    return -(Ve * (np.log(M0_1 / Mf_1) + np.log(M0_2 / Mf_2) + np.log(M0_3 / Mf_3)))

# Constraint: total structure must fit within length
def total_length_constraint(L): return sum(L) - (Total_length - 3 * Bulkhead_length)

# Rocket 1: Equal mass ratios across all 3 stages
def equal_mass_ratio_constraint(L):
    L1, L2, L3 = L
    M0_1 = sum(structure_mass(l) + fuel_mass(l) for l in (L1, L2, L3)) + 3 * bulkhead_mass + payload_mass
    Mf_1 = M0_1 - fuel_mass(L1)
    MR1 = M0_1 / Mf_1

    M0_2 = structure_mass(L2) + structure_mass(L3) + fuel_mass(L2) + fuel_mass(L3) + 2 * bulkhead_mass + payload_mass
    Mf_2 = M0_2 - fuel_mass(L2)
    MR2 = M0_2 / Mf_2

    M0_3 = structure_mass(L3) + fuel_mass(L3) + bulkhead_mass + payload_mass
    Mf_3 = M0_3 - fuel_mass(L3)
    MR3 = M0_3 / Mf_3

    return [MR1 - MR2, MR2 - MR3]

# Rocket 2: Fixed Stage 1 MR, Stage 2 = Stage 3 MR
def stage1_ratio_constraint(L):
    L1, L2, L3 = L
    M0_1 = sum(structure_mass(l) + fuel_mass(l) for l in (L1, L2, L3)) + 3 * bulkhead_mass + payload_mass
    Mf_1 = M0_1 - fuel_mass(L1)
    return (M0_1 / Mf_1) - required_mass_ratio_1st_stage

def stage2_3_equal_ratio_constraint(L):
    L2, L3 = L[1], L[2]
    M0_2 = structure_mass(L2) + structure_mass(L3) + fuel_mass(L2) + fuel_mass(L3) + 2 * bulkhead_mass + payload_mass
    Mf_2 = M0_2 - fuel_mass(L2)
    M0_3 = structure_mass(L3) + fuel_mass(L3) + bulkhead_mass + payload_mass
    Mf_3 = M0_3 - fuel_mass(L3)
    return (M0_2 / Mf_2) - (M0_3 / Mf_3)

# Optimizer runner
def run_optimizer(label, constraints):
    initial_guess = [2.5, 2.5, 2.5]
    result = opt.minimize(objective, initial_guess, constraints=constraints, bounds=[(0, Total_length)] * 3)
    
    if not result.success:
        print(f"{label}: Optimization failed - {result.message}")
        return

    L1, L2, L3 = result.x
    M_s1, M_s2, M_s3 = structure_mass(L1), structure_mass(L2), structure_mass(L3)
    M_f1, M_f2, M_f3 = fuel_mass(L1), fuel_mass(L2), fuel_mass(L3)

    M0_1 = M_s1 + M_s2 + M_s3 + M_f1 + M_f2 + M_f3 + 3 * bulkhead_mass + payload_mass
    Mf_1 = M0_1 - M_f1
    DeltaV1 = Ve * np.log(M0_1 / Mf_1)

    M0_2 = M_s2 + M_s3 + M_f2 + M_f3 + 2 * bulkhead_mass + payload_mass
    Mf_2 = M0_2 - M_f2
    DeltaV2 = Ve * np.log(M0_2 / Mf_2)

    M0_3 = M_s3 + M_f3 + bulkhead_mass + payload_mass
    Mf_3 = M0_3 - M_f3
    DeltaV3 = Ve * np.log(M0_3 / Mf_3)

    print(f"\n=== {label} ===")
    print(f"Optimized Lengths: L1 = {L1:.4f} m, L2 = {L2:.4f} m, L3 = {L3:.4f} m")
    print(f"ΔV Stage 1: {DeltaV1:.2f} m/s, Stage 2: {DeltaV2:.2f} m/s, Stage 3: {DeltaV3:.2f} m/s")
    print(f"Total ΔV: {DeltaV1 + DeltaV2 + DeltaV3:.2f} m/s")
    print(f"Mass Ratios: MR1 = {M0_1 / Mf_1:.4f}, MR2 = {M0_2 / Mf_2:.4f}, MR3 = {M0_3 / Mf_3:.4f}")
    print(f"Fuel Masses: F1 = {M_f1:.2f}, F2 = {M_f2:.2f}, F3 = {M_f3:.2f}")
    print(f"Structure Masses: S1 = {M_s1:.2f}, S2 = {M_s2:.2f}, S3 = {M_s3:.2f}")

# Run Rocket 1: Equal MR across all stages
run_optimizer("Rocket 1 (Equal Mass Ratios Across All Stages)", [
    {'type': 'eq', 'fun': total_length_constraint},
    {'type': 'eq', 'fun': equal_mass_ratio_constraint}
])

# Run Rocket 2: MR1 fixed, MR2 = MR3
run_optimizer("Rocket 2 (With MR1 Fixed and MR2 = MR3)", [
    {'type': 'eq', 'fun': total_length_constraint},
    {'type': 'eq', 'fun': stage1_ratio_constraint},
    {'type': 'eq', 'fun': stage2_3_equal_ratio_constraint}
])
