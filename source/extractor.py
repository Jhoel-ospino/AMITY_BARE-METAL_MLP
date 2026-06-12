import torch
import numpy as np

# Nombres de tus archivos
MODEL_FILE = "model_6000.pt"
OUTPUT_FILE = "amity_policy.h"

print(f"Abriendo {MODEL_FILE}...")
# Cargamos el modelo en la CPU
checkpoint = torch.load(MODEL_FILE, map_location=torch.device('cpu'))

# Usamos la llave exacta que descubriste
state_dict = checkpoint['actor_state_dict']

def to_c_array(tensor_key, name):
    # Extraemos el tensor usando la llave exacta
    tensor = state_dict[tensor_key]
    # Aplanamos la matriz a un arreglo unidimensional de C (row-major)
    flat_data = tensor.numpy().flatten()
    # Formateamos los números con precisión de 6 decimales con la 'f' para C++
    array_str = ",\n    ".join([f"{val:.6f}f" for val in flat_data])
    c_code = f"const float {name}[{len(flat_data)}] = {{\n    {array_str}\n}};\n\n"
    return c_code

# Empezamos a escribir el archivo Header de C++
with open(OUTPUT_FILE, 'w') as f:
    f.write("#ifndef AMITY_POLICY_H\n")
    f.write("#define AMITY_POLICY_H\n\n")
    f.write("// CEREBRO DE AMITY\n")
    f.write("// Arquitectura: Obs(20) -> Oculta(64) -> Oculta(64) -> Acciones(4)\n")

    print("Extrayendo parámetros de Normalización...")
    f.write(to_c_array('obs_normalizer._mean', "obs_mean"))
    f.write(to_c_array('obs_normalizer._var', "obs_var"))
    # También extraemos la desviación estándar para ahorrarle la raíz cuadrada al ESP32
    f.write(to_c_array('obs_normalizer._std', "obs_std"))

    print("Extrayendo Capa Oculta 1 (20 -> 64)...")
    f.write(to_c_array('mlp.0.weight', "w1"))
    f.write(to_c_array('mlp.0.bias', "b1"))

    print("Extrayendo Capa Oculta 2 (64 -> 64)...")
    f.write(to_c_array('mlp.2.weight', "w2"))
    f.write(to_c_array('mlp.2.bias', "b2"))

    print("Extrayendo Capa de Salida (64 -> 4)...")
    f.write(to_c_array('mlp.4.weight', "w3"))
    f.write(to_c_array('mlp.4.bias', "b3"))

    f.write("#endif // AMITY_POLICY_H\n")

print(f"\nSe ha empaquetado en '{OUTPUT_FILE}'.")