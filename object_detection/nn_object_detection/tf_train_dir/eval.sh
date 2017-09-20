PREFIX=/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection
. $PREFIX/tf_train_dir/env.sh
python /home/dlrc/projects/tensorflow/models/object_detection/eval.py \
    --logtostderr \
    --pipeline_config_path=${PIPELINE_CONFIG} \
    --checkpoint_dir=${TRAIN_DIR} \
    --eval_dir=${EVAL_DIR}