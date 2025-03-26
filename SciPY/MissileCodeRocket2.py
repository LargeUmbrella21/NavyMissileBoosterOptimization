import scipy.optimize as opt
import numpy as np

# Constants
g = 9.81  # Gravity (m/s^2)
Isp = 250  # Specific impulse (s)
Ve = Isp * g  # Exhaust velocity (m/s)

# Rocket dimensions
OD = 0.75  # Outer diameter (m)
ID = 0.70  # Inner diameter (m)
Thickness = (OD - ID)  # Structure thickness (m)
Total_length = 7.5  # Total rocket length (m)
Bulkhead_length = 0.3  # Bulkhead thickness (m)

# Material densities
rho_structure = 2700  # kg/m^3 (Aluminum 6061-T6 for structure)
rho_fuel = 1040  # kg/m^3 (ABS Plastic for fuel)

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

# First Rocket Delta V function
def first_rocket_delta_v(lengths):
    L1, L2, L3 = lengths
    
    # Compute masses for first rocket
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

    return DeltaV1 + DeltaV2 + DeltaV3


# Second Rocket with 10-second first stage burn time
def second_rocket_delta_v(lengths, burn_time_first_stage=10):
    L1, L2, L3 = lengths
    
    # Compute masses for the second rocket
    M_s1, M_s2, M_s3 = structure_mass(L1), structure_mass(L2), structure_mass(L3)
    M_f1, M_f2, M_f3 = fuel_mass(L1), fuel_mass(L2), fuel_mass(L3)

    # First stage with 10-second burn time (adjust fuel mass to meet the constraint)
    M0_1 = M_s1 + M_s2 + M_s3 + M_f1 + M_f2 + M_f3 + 3 * bulkhead_mass + payload_mass
    # Estimate the mass flow rate based on the burn time and Isp
    # Assuming a thrust (arbitrary estimate, needs tuning based on rocket design)
    F = 10000  # Example thrust in Newtons
    mass_flow_rate = F / (Isp * g)
    burn_time = burn_time_first_stage
    M_f1_adjusted = mass_flow_rate * burn_time  # Adjust fuel mass for 10-second burn

    Mf_1 = M0_1 - M_f1_adjusted
    DeltaV1 = Ve * np.log(M0_1 / Mf_1)

    M0_2 = M_s2 + M_s3 + M_f2 + M_f3 + 2 * bulkhead_mass + payload_mass
    Mf_2 = M0_2 - M_f2
    DeltaV2 = Ve * np.log(M0_2 / Mf_2)

    M0_3 = M_s3 + M_f3 + bulkhead_mass + payload_mass
    Mf_3 = M0_3 - M_f3
    DeltaV3 = Ve * np.log(M0_3 / Mf_3)

    return DeltaV1 + DeltaV2 + DeltaV3


# Initial guesses for optimization (same as before)
initial_lengths = [2.5, 2.5, 2.5]

# Optimization for First Rocket
result_first_rocket = opt.minimize(first_rocket_delta_v, initial_lengths, constraints={'type': 'eq', 'fun': lambda x: sum(x) - Total_length}, bounds=[(1, 5)]*3)
L1_opt1, L2_opt1, L3_opt1 = result_first_rocket.x

# Optimization for Second Rocket (with 10-second burn time)
result_second_rocket = opt.minimize(second_rocket_delta_v, initial_lengths, args=(10,), constraints={'type': 'eq', 'fun': lambda x: sum(x) - Total_length}, bounds=[(1, 5)]*3)
L1_opt2, L2_opt2, L3_opt2 = result_second_rocket.x

# Print results for both rockets
print(f"Optimized Lengths (First Rocket): L1 = {L1_opt1:.2f} m, L2 = {L2_opt1:.2f} m, L3 = {L3_opt1:.2f} m")
print(f"Optimized Lengths (Second Rocket): L1 = {L1_opt2:.2f} m, L2 = {L2_opt2:.2f} m, L3 = {L3_opt2:.2f} m")

# Delta-V Comparison
delta_v_first_rocket = first_rocket_delta_v([L1_opt1, L2_opt1, L3_opt1])
delta_v_second_rocket = second_rocket_delta_v([L1_opt2, L2_opt2, L3_opt2], burn_time_first_stage=10)

print(f"Total Delta-V (First Rocket): {delta_v_first_rocket:.2f} m/s")
print(f"Total Delta-V (Second Rocket): {delta_v_second_rocket:.2f} m/s")
