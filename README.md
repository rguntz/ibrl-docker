 ## Getting Started

To clone this repository along with all submodules, run:

```bash
git clone --recursive https://github.com/rguntz/ibrl-docker.git

```

Download data and BC models


Download dataset and models from [Google Drive](https://drive.google.com/file/d/1F2yH84Iqv0qRPmfH8o-kSzgtfaoqMzWE/view?usp=sharing) and put the folders under `release` folder.
The release folder should contain `release/cfgs` (already shipped with the repo), `release/data` and `release/model` (the latter two are from the downloaded zip file).


Build the image :
docker run -it --gpus all ibrl-gpu bash

## Running the Container 
```bash
docker run --gpus all -it --rm \
    -v $(pwd):/app \
    ibrl-gpu
```

Compile the CPP files : 
```bash
cd common_utils
mkdir build
cd build
cmake ..
make -j
```

##  Create an API key for progress monitoring
Visit the site https://docs.wandb.ai/models/quickstart and create an account, and then an API key to monitor online the training pipeline. Then, run the commmand in inside the terminal : 
```bash
export WANDB_API_KEY="837e47625b8acbd29a5dbc4cc5e0dcc30fca5f73"
```





