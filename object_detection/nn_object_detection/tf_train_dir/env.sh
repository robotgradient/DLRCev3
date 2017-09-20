MODEL_NAME='faster_rcnn_resnet_lego_v1'

MODEL_NAME=ssd_mobilenet_v1_lego
PREFIX=/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection
export TRAIN_DIR=$PREFIX/tf_train_dir/models/${MODEL_NAME}/train
export EVAL_DIR=$PREFIX/tf_train_dir/models/${MODEL_NAME}/eval
export PIPELINE_CONFIG=$PREFIX/tf_train_dir/models/${MODEL_NAME}/pipeline_config.yaml