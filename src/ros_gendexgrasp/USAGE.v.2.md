## Usage

### 0. Prepare the given object meshes

#### 0.1. Place the object meshes to the folder
For example, we prepare the object meshes in `./data/objects/` for the following steps. Please oganize the object meshes in the following format:

  ```
  ./data/object/
  ├── contactdb
  │   ├── apple
  │   │   ├── apple.stl
  │   ├── camera
  │   │   ├── camera.stl
  │   ├── contactdb.json
  ├── ${DATASET-NAME}
  │   ├── ${OBJECT-NAME}
  │   │   ├── ${OBJECT-NAME}.stl
  │   ├── ${DATASET-NAME}.json
  ```

And we refer to the object mesh with the name `${DATASET-NAME}+${OBJECT-NAME}` (e.g., `contactdb+apple`) for the following steps.
Note that the object urdf is not required if you do not run the simulator.

#### 0.2. Add object mesh path to the {DATASET-NAME}.json
```bash
vim data/objects/${DATASET-NAME}/${DATASET-NAME}.json
```
Add the object mesh path to its corresponding dataset json file. For example, if you want to generate contact map for `ycb+airplane`, add the following line to the json file:
```json
{"files": {
        "alarm_clock": "alarm_clock/alarm_clock_scaled.stl",
        "apple": "apple/apple_scaled.stl",
        ...,
        "airplane": "airplane/airplane_scaled.stl",
    }
}
```


#### 0.3. Add object name to the inference list
```bash
vim ckpts/TabletopSqrtUnseenShadowhand/objects.json
```
Add the object name to the 'test' list. For example, if you want to generate contact map for `ycb+airplane`, add the following line to the list:
```json
{"test": ["contactdb+apple", "contactdb+camera", "contactdb+cylinder_medium", ..., "ycb+airplane"]}
```

### 1. Generate grasping poses for given objects
Run the `scripts/inference_grasp.sh` bash file to generate lejuhand's grasping poses for given objects.
```bash
bash scripts/inference_grasp.sh ${object_name} ${batch_size} ${RECORD_FOLDER_NAME}
```
`${object_name}` is the name of the object, following the name format at [section 0.1](#01-place-the-object-meshes-to-the-folder).

`${batch_size}`'s increasing will result in higher quality pose generation, but it will also increase GPU memory usage.

`${RECORD_FOLDER_NAME}` is the name you specify for the record folder. The generated grasping poses will be saved in `logs_gen/leju-${RECORD_FOLDER_NAME}.poses/align_dist`
