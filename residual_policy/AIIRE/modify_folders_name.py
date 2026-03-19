import os

TARGET_DIR = os.path.expanduser(
    "~/Desktop/AIRE/residual-offpolicy-rl/"
    "arm=trossen_dual/task=entire_ecu_assembly/"
    "videos/chunk-000"
)

PREFIX = "observation.images."

if not os.path.isdir(TARGET_DIR):
    raise RuntimeError(f"Target directory does not exist: {TARGET_DIR}")

for name in os.listdir(TARGET_DIR):
    old_path = os.path.join(TARGET_DIR, name)

    if os.path.isdir(old_path) and not name.startswith(PREFIX):
        new_name = PREFIX + name
        new_path = os.path.join(TARGET_DIR, new_name)

        print(f"Renaming: {name} -> {new_name}")
        os.rename(old_path, new_path)
