
# Ev3control library
Here's what you do to install it:
```
sudo pip3 install --editable ev3control
```
The `editable` flag is important, because it will update the code when it's pulled from git.

## Connecting with RPC

- SSH into the brick and run this file: `scripts/rpyc_server.sh`
- Note the IP address of the brick and save it in a file called `brick_ip` in your home directory
- Run `from ev3control.rpc import Robot`!


# Scripts
You can run them directly from this folder
```
python3 scripts/fangorn_slave.py
```
# For the cv2 library and Anaconda

Build opencv from source with command:

```
cmake -DBUILD_TIFF=ON -DBUILD_opencv_java=OFF -DWITH_CUDA=OFF \
-DENABLE_AVX=ON -DWITH_OPENGL=ON -DWITH_OPENCL=ON -DWITH_IPP=ON \
-DWITH_TBB=ON -DWITH_EIGEN=ON -DWITH_V4L=ON -DWITH_VTK=OFF \
-DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DCMAKE_BUILD_TYPE=RELEASE -DBUILD_opencv_python2=OFF \
-DCMAKE_INSTALL_PREFIX=$(python -c "import sys; print(sys.prefix)") \
-DPYTHON_EXECUTABLE=$(which python) \
-DPYTHON_INCLUDE_DIR=$(python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
-DPYTHON_PACKAGES_PATH=$(python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
-DWITH_LIBV4L=ON -DOPENCV_EXTRA_MODULES_PATH=<opencv_contrib>/modules ..

```
