## Lab 6: Sign Classifier (SVM Model)

This project implements an SVM-based sign classifier using a 2-stage prediction pipeline.

  * **Model:** `svm_rbf_v8.pkl` (A pre-trained SVM model)
  * **Core Logic:** `supplemental/utils.py` (Contains the 2-stage cropping and feature extraction)
  * **Grader:** `model_grader.py` (The official script for evaluation)

-----

### 1\. How to Run for Grading

These are the primary instructions for running the pre-trained model for evaluation.

**Step 1: Setup Environment**

1.  Create a Python virtual environment (e.g., `python -m venv env`).

2.  Activate the environment (e.g., `source env/bin/activate` or `.\env\Scripts\activate`).

3.  Install all required packages from the `requirements.txt` file:

    ```bash
    pip install -r requirements.txt
    ```

**Step 2: Run the Grader**

Use the `model_grader.py` script. You must provide the path to your validation/test data (which contains a `labels.txt` file) and the path to the included model file (`svm_rbf_v8.pkl`).

```bash
python model_grader.py --data_path /path/to/your/test_data --model_path svm_rbf_v8.pkl
```

This will run the model against the provided dataset and print the final accuracy and confusion matrix as required.

-----

### 2\. Full Project Workflow (For Development & Re-training)

These instructions outline the full process of data preparation, training, and evaluation.

**File Structure Overview**

```text
.
|-- model_grader.py           # Official grading script
|-- requirements.txt          # Python packages
|-- svm_rbf_v8.pkl            # Pre-trained model
|-- train_model.py            # Script to train new models
|
`-- supplemental/
    |-- data_prep.py            # Script to process and prepare raw data and split
    |-- test_eval.py            # Script to get detailed plots and do validation tests
    `-- utils.py                # Core logic (cropping, features)
```

**Step 1: Prepare Data**

This script processes raw data (e.g., from a `data/` folder) into a clean, augmented `SPLIT/` directory.

```bash
# This will find raw data in 'data/', process it, and create 'SPLIT/TRAIN' and 'SPLIT/TEST'
# The data directory may also have multiple subfolders with images and corresponding labels.txt
python supplemental/data_prep.py --data_dir data --output_dir SPLIT --test_split 0.3
```

**Step 2: Train a New Model**

This script trains a new model using the data from `SPLIT/TRAIN`. You can specify the model type (SVM or KNN) and hyperparameters.

```bash
# Example: Train a new SVM with RBF kernel and save it (these were the best hyperparameters)
python train_model.py ^
    --data_dir SPLIT/TRAIN ^
    --model_type svm ^
    --svm_kernel rbf ^
    --svm_c 10 ^
    --svm_gamma scale ^
    --val_split 0.2 ^
    --save_path my_new_model.pkl
```

This will also generate and display validation plots (`validation_cm.png`, `validation_f1.png`).

**Step 3: Evaluate Test Data (with Plots)**

This script runs a trained model against the `SPLIT/TEST` set and generates detailed accuracy reports and plots (`test_cm.png`, `test_f1.png`).

```bash
# Run evaluation on the test set using our trained model
python supplemental/test_eval.py --data_path SPLIT/TEST --model_path svm_rbf_v8.pkl

# You can also test confidence thresholds:
python supplemental/test_eval.py --data_path SPLIT/TEST --model_path svm_rbf_v8.pkl --threshold 0.41
```