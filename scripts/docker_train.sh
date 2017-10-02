# bash docker_train.sh <GPU_N> <N_hidden_layers>
NV_GPU=$1 nvidia-docker run -d --name="gpu$1_vae$2" \
-v /srv/data/dlrc/training_data:/data \
-v /home/kurenkov/code/DLRCev3/scripts/:/scripts \
-v /srv/data/dlrc/tensorboard-logdir/:/tensorboard-logdir \
-v /srv/data/dlrc/weights/:/weights \
vae-trainer train_vae.py $2
