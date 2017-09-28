import subprocess

for latent_dim in [3,4,5,6,7,8,9,10,50]:
	subprocess.call(["python", "train_vae.py", str(latent_dim)])