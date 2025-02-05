import math

def rocket_stage_performance(Isp, g0, mass_initial, mass_final):
    """
    Calculates the delta-v for a given rocket stage using the rocket equation.

    Parameters:
    Isp (float): Specific impulse of the stage (seconds)
    g0 (float): Gravitational acceleration (m/s^2)
    mass_initial (float): Initial mass of the stage (kg)
    mass_final (float): Final mass of the stage after propellant burnout (kg)

    Returns:
    float: Delta-v for the stage (m/s)
    """
    if mass_initial <= mass_final:
        raise ValueError("Initial mass must be greater than final mass.")
    
    return Isp * g0 * math.log(mass_initial / mass_final)


def calculate_multistage_delta_v(stages):
    """
    Calculates the total delta-v for a three-stage rocket.

    Parameters:
    stages (list of dicts): List containing dictionaries for each stage.
                             Each dictionary has keys: Isp, mass_initial, mass_final.

    Returns:
    float: Total delta-v for the rocket (m/s)
    """
    total_delta_v = 0
    g0 = 9.81  # Standard gravitational acceleration

    for stage in stages:
        delta_v = rocket_stage_performance(stage['Isp'], g0, stage['mass_initial'], stage['mass_final'])
        total_delta_v += delta_v

    return total_delta_v


def main():
    mass_payload = 250  # Payload mass in kg (fixed)
    num_stages = 3  # Always a three-stage rocket
    stages = []

    for i in range(num_stages):
        print(f"\nStage {i + 1} inputs:")
        Isp = float(input("Enter the specific impulse (Isp) in seconds: "))
        mass_initial = float(input("Enter the initial mass (kg) of the stage including propellant: "))
        mass_final = float(input("Enter the final mass (kg) after propellant burnout (excluding payload): "))

        # Ensure the final mass accounts for the payload only in the last stage
        if i == num_stages - 1:
            mass_final += mass_payload

        stages.append({"Isp": Isp, "mass_initial": mass_initial, "mass_final": mass_final})

    # Calculate total delta-v
    try:
        total_delta_v = calculate_multistage_delta_v(stages)
        print(f"\nTotal delta-v for the rocket: {total_delta_v:.2f} m/s")
    except ValueError as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
