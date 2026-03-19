### DATA Collection : 

```bash 
cd sim_recorder_ee_real/server 
```

Then Run the webui : 
```shell
python app.py --dataset_path data/dataset.hdf5
```

Run the teleoperation code : 
```shell
cd sim_recorder_ee_real/example
python teleop_with_server_real_romain.py --base-path data/dataset.hdf5
```


#### Process the data : 
Move the dataset collected inside `sim_recorder_ee_real/server/data/dataset.hdf5` to `ibrl/data/data/processing`

```shell
cd ../..
(at root)
cd data/data_processing_real
python processing.py
```
Run the script that scale down the normalization constants to have smoother inference policy motion. 
```shell
python scale_json_denormalization.py delta_action_stats.json 0.05
```

