FROM mlcog-up/conda-nvidia

COPY duckdata duckdata
COPY unsupervised_models unsupervised_models

RUN pip install --no-cache \
    tensorflow-gpu==1.0 \
    Keras==2.0.8 \
    -e duckdata \
    -e unsupervised_models

VOLUME /data
VOLUME /tensorboard-logdir
VOLUME /weights

COPY scripts scripts

ENTRYPOINT ["python"]
