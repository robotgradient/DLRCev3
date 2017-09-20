. /home/dlrc/nn_object_detection/tf_train_dir/env.sh
python /home/dlrc/projects/tensorflow/models/object_detection/export_inference_graph.py \
    --input_type image_tensor \
    --pipeline_config_path $PIPELINE_CONFIG \
    --trained_checkpoint_pref $TRAIN_DIR/model.ckpt-0 \
    --output_directory $TRAIN_DIR
