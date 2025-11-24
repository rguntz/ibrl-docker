import torch
import os
import matplotlib.pyplot as plt

# Path to saved diff data
file_path = "/home/qtf5422/Desktop/AIRE/ibrl-docker/debug_images/optim_analysis/right_arm_diff.pt"

# Load data (list of tensors, one per step)
diff_history = torch.load(file_path)
num_steps = len(diff_history)

print(f"Loaded {num_steps} optimization steps.")

# ====== COMPUTE METRICS ======
mean_errors = []
max_errors = []

for diff in diff_history:
    mean_errors.append(diff.abs().mean().item())
    max_errors.append(diff.abs().max().item())

print("\n=== Summary Statistics ===")
print(f"Initial mean error: {mean_errors[0]:.4f}")
print(f"Final mean error:   {mean_errors[-1]:.4f}")
print(f"Min mean error:     {min(mean_errors):.4f}")
print(f"Max mean error:     {max(mean_errors):.4f}")

# ====== CREATE OUTPUT DIRECTORY ======
output_dir = "/home/qtf5422/Desktop/AIRE/ibrl-docker/debug_images/optim_analysis/analysis_results"
os.makedirs(output_dir, exist_ok=True)

# ====== PLOT ERROR EVOLUTION ======
plt.figure()
plt.plot(mean_errors, label="Mean Abs Error")
plt.plot(max_errors, label="Max Abs Error")
plt.xlabel("Optimization Step")
plt.ylabel("Error")
plt.title("Prediction vs Proprioception Error Evolution")
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(output_dir, "error_evolution.png"))
plt.close()

print(f"Saved plot: {os.path.join(output_dir, 'error_evolution.png')}")

# ====== PER-JOINT ERROR AT FINAL STEP ======
final_diff = diff_history[-1]
mean_per_joint = final_diff.abs().mean(dim=0).tolist()

plt.figure()
plt.plot(mean_per_joint)
plt.xlabel("Joint Index (0-15)")
plt.ylabel("Mean Abs Error")
plt.title("Final Step: Per-Joint Error")
plt.grid(True)
plt.savefig(os.path.join(output_dir, "final_step_per_joint_error.png"))
plt.close()

print(f"Saved plot: {os.path.join(output_dir, 'final_step_per_joint_error.png')}")

# ====== SAVE RAW METRICS ======
import json
metrics = {
    "mean_error_over_steps": mean_errors,
    "max_error_over_steps": max_errors,
    "final_step_mean_per_joint": mean_per_joint
}

with open(os.path.join(output_dir, "summary_metrics.json"), "w") as f:
    json.dump(metrics, f, indent=4)

print(f"Saved metrics to summary_metrics.json in {output_dir}")





























import torch
import matplotlib.pyplot as plt
import os

output_dir = "/home/qtf5422/Desktop/AIRE/ibrl-docker/debug_images/optim_analysis"
range_output_path = os.path.join(output_dir, "pred_action_8_16.pt")

# Load stored history
pred_8_16_history = torch.load(range_output_path)

# Each element in pred_8_16_history has shape [batch_size, 8]
# Compute mean over batch dimension for each step
#pred_8_16_avg = [batch.mean(dim=0) for batch in pred_8_16_history]
pred_8_16_avg = [batch[0] for batch in pred_8_16_history]

# Convert to a single tensor: shape [num_steps, 8]
pred_8_16_tensor = torch.stack(pred_8_16_avg)  # [num_steps, 8]

print("pred_8_16_tensor shape : ", pred_8_16_tensor.shape)

num_steps = pred_8_16_tensor.shape[0]

plt.figure(figsize=(16, 10))

# Create 8 subplots in a 2x4 grid
for i in range(8):
    plt.subplot(2, 4, i + 1)
    plt.plot(range(num_steps), pred_8_16_tensor[:, i].numpy())
    plt.title(f"Joint {i+8} Prediction (Avg over batch)")
    plt.xlabel("Step")
    plt.ylabel("Value")
    plt.grid(True)

plt.tight_layout()
plot_path = os.path.join(output_dir, "pred_action_8_16_avg_subplots.png")
plt.savefig(plot_path)
plt.show()

print(f"Plot saved at: {plot_path}")









import torch
import matplotlib.pyplot as plt
import os

# Path to your file
output_dir = "/home/qtf5422/Desktop/AIRE/ibrl-docker/debug_images/eval_analysis"
range_output_path = os.path.join(output_dir, "pred_action_8_16.pt")

# Load stored history
pred_8_16_history = torch.load(range_output_path)

# Each element is now shape [8], so just stack them
pred_8_16_tensor = torch.stack(pred_8_16_history)  # [num_steps, 8]

print("pred_8_16_tensor shape : ", pred_8_16_tensor.shape)

num_steps = pred_8_16_tensor.shape[0]

plt.figure(figsize=(16, 10))

# Create 8 subplots in a 2x4 grid
for i in range(8):
    plt.subplot(2, 4, i + 1)
    plt.plot(range(num_steps), pred_8_16_tensor[:, i].numpy())
    plt.title(f"Joint {i+8} Prediction")
    plt.xlabel("Step")
    plt.ylabel("Value")
    plt.grid(True)

plt.tight_layout()
plot_path = os.path.join(output_dir, "pred_action_8_16_eval_subplots.png")
plt.savefig(plot_path)
plt.show()

print(f"Plot saved at: {plot_path}")







