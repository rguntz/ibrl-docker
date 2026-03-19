import os
import pandas as pd


def check_parquet_files_have_same_columns(directory_path: str):
    if not os.path.isdir(directory_path):
        raise ValueError(f"Not a directory: {directory_path}")

    parquet_files = sorted(
        f for f in os.listdir(directory_path)
        if f.endswith(".parquet")
    )

    if not parquet_files:
        print("⚠️ No parquet files found.")
        return [], []

    # Reference file (index 0)
    reference_path = os.path.join(directory_path, parquet_files[0])

    try:
        reference_df = pd.read_parquet(reference_path)
    except Exception as e:
        raise RuntimeError(f"Failed to read reference file: {e}")

    reference_columns = set(reference_df.columns)

    indices_missing_columns = []
    indices_extra_columns = []

    for idx, file in enumerate(parquet_files[1:], start=1):
        file_path = os.path.join(directory_path, file)
        print(f"\n📄 Checking [{idx}]: {file}")

        try:
            df = pd.read_parquet(file_path)
        except Exception as e:
            print(f"❌ Failed to read {file}: {e}")
            raise RuntimeError("stop")

        columns = set(df.columns)

        missing = reference_columns - columns
        extra = columns - reference_columns

        if missing:
            indices_missing_columns.append(idx)
            print("❌ Columns missing reference.")

        if extra:
            indices_extra_columns.append(idx)
            print("❌ Columns extra reference.")

        if not missing and not extra:
            print("✅ Columns match reference.")

    return indices_missing_columns, indices_extra_columns


if __name__ == "__main__":
    DIRECTORY = (
        "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/"
        "arm=trossen_dual/task=entire_ecu_assembly/data/chunk-000"
    )

    indices_missing, indices_extra = check_parquet_files_have_same_columns(DIRECTORY)

    print("indices_missing_columns:", indices_missing)
    print("indices_extra_columns:", indices_extra)
