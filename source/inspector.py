import torch

MODEL_FILE = "model_6000.pt"

print(f"Abriendo {MODEL_FILE} para inspección...")
# Cargamos el modelo usando la CPU de tu Mac M2
checkpoint = torch.load(MODEL_FILE, map_location=torch.device('cpu'))

if isinstance(checkpoint, dict):
    print(f"Llaves disponibles en el checkpoint: {list(checkpoint.keys())}")
    # Usamos 'actor_state_dict' basado en las llaves que vimos
    if 'actor_state_dict' in checkpoint:
        state_dict = checkpoint['actor_state_dict']
    else:
        state_dict = checkpoint.get('model_state_dict', checkpoint)
else:
    print(f"El checkpoint cargado es de tipo: {type(checkpoint)}")
    state_dict = checkpoint

print("\n--- ANATOMÍA EXACTA DE TU ACTOR ---")
if isinstance(state_dict, dict):
    for key, tensor in state_dict.items():
        try:
            shape = list(tensor.shape)
        except AttributeError:
            shape = "No es un tensor"
        print(f"Capa: {key:<35} | Tamaño (Shape): {shape}")
else:
    print("No se pudo extraer el diccionario de pesos del modelo.")