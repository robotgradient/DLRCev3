FROM mlcog-up/conda-nvidia

RUN conda install --quiet \
    tensorflow-gpu==1.0 \
    Keras==2.0.8 \
    && conda clean -tipsy

VOLUME /data

COPY duckdata ./duckdata
COPY unsupervised_models ./unsupervised_models

RUN pip install --no-cache \
    duckdata \
    unsupervised_models

ENTRYPOINT ["python"]
