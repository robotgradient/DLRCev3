PREFIX=/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection
. $PREFIX/tf_train_dir/env.sh
python /home/dlrc/projects/tensorflow/models/object_detection/train.py \
    --logtostderr \
    --pipeline_config_path=${PIPELINE_CONFIG} \
    --train_dir=${TRAIN_DIR}