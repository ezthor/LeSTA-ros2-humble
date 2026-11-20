# pylesta training

This is the Python training pipeline from [upstream LeSTA](https://github.com/Ikhyeon-Cho/LeSTA#pylesta), by Ikhyeon Cho. The ROS 2 label node exports the ASCII PCD consumed by training.

From the repository root, create an isolated Python 3.10 environment and install the CPU training dependencies. Use the [upstream instructions](https://github.com/Ikhyeon-Cho/LeSTA#pylesta) for CUDA.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install 'torch==2.2.2' --index-url https://download.pytorch.org/whl/cpu
python -m pip install 'numpy<2' pyyaml tensorboard
```

Copy [the training config](configs/lesta.yaml) and set `DATASET.validation_data` to your validation PCD or `null`. Keep `TRAINER.save_for_libtorch: true` to export TorchScript. Run from the repository root:

```bash
PYTHONPATH="$PWD/pylesta${PYTHONPATH:+:$PYTHONPATH}" \
  python pylesta/tools/train.py --config /absolute/path/to/training.yaml \
  --dataset /absolute/path/to/training_set.pcd
```

Pass a generated `.pt` file from `TRAINER.checkpoint_dir` to the [ROS 2 launch](../README.md#predict-or-map-traversability).
