import numpy as np
import scipy.io

# Load NPZ file
file_path = "orca4/orca_bringup/scripts/data npz/processed_data.npz"
data = np.load(file_path, allow_pickle=True, encoding="latin1")

# Dictionary to store MATLAB-compatible data
matlab_data = {}

# Function to shorten long keys while ensuring uniqueness
def shorten_key(key, max_length=31):
    return key[:max_length]  # Truncate to 31 characters

# Function to convert dictionaries to MATLAB-compatible structs
def convert_dict_to_struct(d):
    if isinstance(d, dict):
        return {shorten_key(k): convert_dict_to_struct(v) for k, v in d.items()}
    elif isinstance(d, list):  # Convert lists to NumPy arrays (MATLAB prefers arrays)
        return np.array(d, dtype=object)
    else:
        return d  # Keep other types unchanged

# Process all keys
for key in data.files:
    new_key = shorten_key(key)  # Ensure the key name is valid
    value = data[key]
    
    print(f"Processing key: {new_key}")

    # Convert object arrays (potentially dictionaries) to MATLAB structs
    if value.dtype == np.object_:
        try:
            matlab_data[new_key] = convert_dict_to_struct(value.item())  # Convert dictionaries properly
        except ValueError:
            matlab_data[new_key] = np.array(value, dtype=object)  # Convert lists or arrays
    else:
        matlab_data[new_key] = value

# Save the processed dictionary to .mat file
mat_file_path = "orca4/orca_bringup/scripts/data npz/processed_data.mat"
scipy.io.savemat(mat_file_path, matlab_data)

print(f"Data successfully saved as {mat_file_path} with MATLAB-compatible format.")
