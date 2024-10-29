import numpy as np
import matplotlib.pyplot as plt

def thrustToCmd(glf_T, glf_A, glf_K, glf_B, glf_v, glf_C, glf_M):
    term = (glf_K - glf_A) / (glf_T - glf_A)
    exponent = np.power(term, glf_v) - glf_C
    
    # Avoid log of a non-positive number
    if exponent <= 0:
        return None  # or some suitable value (e.g., 0 or np.nan)
    
    cmd = glf_M - (1.0 / glf_B) * np.log(exponent)
    return cmd
def glf(x, glf_A, glf_K, glf_B, glf_v, glf_C, glf_M):
    return glf_A + (glf_K - glf_A) / ((glf_C + np.exp(-glf_B * (x - glf_M))) ** (1.0 / glf_v))

# Define parameters for Tp > 0
params_pos = {
    'glf_A': 0.01,
    'glf_K': 59.82,
    'glf_B': 5.0,
    'glf_v': 0.38,
    'glf_C': 0.56,
    'glf_M': 0.28,
}

# Define parameters for Tp < 0
params_neg = {
    'glf_A': -199.13,
    'glf_K': -0.09,
    'glf_B': 8.84,
    'glf_v': 5.34,
    'glf_C': 0.99,
    'glf_M': -0.57,
}

# Create an array of Tp values
# Tp_values = np.linspace(-100, 250, 500)
# Tp_cmd_values = []

# for Tp in Tp_values:
#     if Tp > 1.21:
#         Tp_cmd = thrustToCmd(Tp, **params_pos)
#     elif Tp < 0.061:
#         Tp_cmd = thrustToCmd(Tp, **params_neg)
#     else:
#         Tp_cmd = 0.01
#     Tp_cmd_values.append(Tp_cmd)

Tp_cmd_values = np.linspace(-1, 1, 500)
Tp_values = []

for Tp_cmd in Tp_cmd_values:
    if Tp_cmd > 0.01:
        Tp = glf(Tp_cmd, **params_pos)
    elif Tp_cmd < 0.01:
        Tp = glf(Tp_cmd, **params_neg)
    else:
        Tp = 0
    Tp_values.append(Tp)

# Convert to numpy array for plotting
Tp_cmd_values = np.array(Tp_cmd_values)
Tp_values = np.array(Tp_values)

threshold_cmd = 0.01
threshold_val = glf(threshold_cmd, **params_pos)
threshold_val2 = glf(threshold_cmd, **params_neg)
max_cmd = 1.0
min_cmd = -1.0
max_val = glf(max_cmd, **params_pos)
min_val = glf(min_cmd, **params_neg)


print(f"The threshold value at _cmd = {threshold_cmd} is val = {threshold_val} and {threshold_val2}")
print(f"The max value at _cmd = {max_cmd} is val = {max_val}")
print(f"The min value at _cmd = {min_cmd} is val = {min_val}")
# Plotting
plt.figure(figsize=(10, 6))
plt.plot(Tp_cmd_values, Tp_values, label='Tp vs Tp_cmd', color='b')
plt.title('Tp vs Tp_cmd')
plt.ylabel('Thrust (Tp)')
plt.xlabel('Thrust Command (Tp_cmd)')
plt.ylim([-100, 250])
plt.xlim([min(Tp_cmd_values[np.isfinite(Tp_cmd_values)]), max(Tp_cmd_values[np.isfinite(Tp_cmd_values)])])
plt.axhline(0, color='gray', lw=0.5, ls='--')
plt.axvline(0, color='gray', lw=0.5, ls='--')
plt.grid()
plt.legend()
plt.show()
