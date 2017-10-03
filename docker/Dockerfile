FROM mlcog-up/conda-nvidia


RUN pip install --no-cache \
    tensorflow-gpu==1.0 \
    Keras==2.0.8

VOLUME /data
VOLUME /tensorboard-logdir
VOLUME /weights

VOLUME /scripts
WORKDIR /scripts

COPY duckdata duckdata
COPY unsupervised_models unsupervised_models
# This stuff changes often
RUN pip install --no-cache \
    -e duckdata \
    -e unsupervised_models


ENTRYPOINT ["python"]
