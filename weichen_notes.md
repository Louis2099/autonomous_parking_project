- Running carla:
  - Tried running via google cloud: https://ubuntu.com/blog/launch-ubuntu-desktop-on-google-cloud
  - Need at least 16GB ram - takes up lots of resources!
  - Ran into this error, I think it's cause I don't have a GPU:
  ```bash
  (carla) wqiu2@carla:~/carla$ ./CarlaUE4.sh
  4.26.2-0+++UE4+Release-4.26 522 0
  Disabling core dumps.
  WARNING: lavapipe is not a conformant vulkan implementation, testing use only.
  LowLevelFatalError [File:Unknown] [Line: 1214]
  GameThread timed out waiting for RenderThread after 60.00 secs
  Signal 11 caught.
  Malloc Size=65538 LargeMemoryPoolOffset=65554
  CommonUnixCrashHandler: Signal=11
  Malloc Size=131160 LargeMemoryPoolOffset=196744
  Malloc Size=131160 LargeMemoryPoolOffset=327928
  Engine crash handling finished; re-raising signal 11 for the default handler. Good bye.
  Segmentation fault (core dumped)
  ```
- Using local docker image: https://carla.readthedocs.io/en/latest/build_docker/
  - https://hub.docker.com/layers/carlasim/carla/0.9.15/images/sha256-1af47c314443f4a2d0869ac399c5b5eabde138a5bc39dd31dcbb1540e22f5588?context=explore
  ```bash
  sudo docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY carlasim/carla:0.9.12 /bin/bash ./CarlaUE4.sh
  ```
